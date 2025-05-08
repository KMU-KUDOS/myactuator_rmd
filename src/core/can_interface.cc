#include "myactuator_rmd/core/can_interface.h"

#include <errno.h>        // For errno
#include <fcntl.h>        // For fcntl, O_NONBLOCK
#include <linux/can/raw.h>// For SOL_CAN_RAW, CAN_RAW_FILTER, CAN_RAW_ERR_FILTER
#include <net/if.h>       // For struct ifreq, SIOCGIFINDEX
#include <poll.h>         // For poll, struct pollfd, POLLOUT
#include <string.h>       // For strerror, memset, strncpy
#include <sys/ioctl.h>    // For SIOCGIFINDEX
#include <sys/socket.h>   // For socket, bind, setsockopt, recv, send, struct sockaddr_can
#include <unistd.h>       // For close

#include <iostream> // For potential debug output, consider using ROS logger later

#include "absl/strings/str_format.h"
#include "myactuator_rmd/core/can_frame.h" // Include CanFrame definition

namespace myactuator_rmd::core
{

namespace
{
// Helper function to create a descriptive error status
absl::Status CreateErrorStatus(const std::string& context)
{
    // Using absl::StatusCode::kUnavailable for socket errors seems appropriate
    return absl::UnavailableError(absl::StrFormat("%s failed: %s (errno=%d)", context, strerror(errno), errno));
}
} // namespace

CanInterface::CanInterface(
    std::string interface_name,
    std::chrono::milliseconds default_receive_timeout,
    std::chrono::milliseconds default_send_timeout)
: interface_name_(std::move(interface_name)),
  default_receive_timeout_(default_receive_timeout),
  default_send_timeout_(default_send_timeout),
  socket_fd_(-1),
  is_open_(false)
{
}

CanInterface::~CanInterface()
{
    // Ensure the socket is closed when the object is destroyed.
    if (isOpen())
    {
        auto status = close();
        if (!status.ok())
        {
            // Log error, but can't do much more in destructor
            // Consider using a ROS logger here in the ROS context
            std::cerr << "Error closing CAN interface in destructor: " << status << std::endl;
        }
    }
}

absl::Status CanInterface::open()
{
    if (isOpen())
    {
        return absl::AlreadyExistsError(absl::StrFormat("CAN interface '%s' is already open.", interface_name_));
    }

    // Create socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        return CreateErrorStatus("socket(PF_CAN, SOCK_RAW, CAN_RAW)");
    }

    // Set socket to non-blocking mode
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags == -1)
    {
        auto status = CreateErrorStatus("fcntl(F_GETFL)");
        ::close(socket_fd_); // Clean up socket before returning
        socket_fd_ = -1;
        return status;
    }
    if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        auto status = CreateErrorStatus("fcntl(F_SETFL, O_NONBLOCK)");
        ::close(socket_fd_);
        socket_fd_ = -1;
        return status;
    }
    
    // Enable reception of CAN error frames
    const int enable_error_frames = 1;
    // Using CAN_RAW_ERR_FILTER requires kernel >= 2.6.28
    // Consider making this optional or checking kernel version if needed
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &enable_error_frames, sizeof(enable_error_frames)) < 0)
    {
        auto status = CreateErrorStatus("setsockopt(SOL_CAN_RAW, CAN_RAW_ERR_FILTER)");
        ::close(socket_fd_);
        socket_fd_ = -1;
        return status;
    }

    // Get interface index
    struct ifreq ifr;
    // Use memset for safety, although struct initialization would also work
    memset(&ifr, 0, sizeof(ifr)); 
    strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0'; // Ensure null termination
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        // Use NotFoundError for interface lookup failure
        auto status = absl::NotFoundError(absl::StrFormat("ioctl(SIOCGIFINDEX) failed for interface '%s': %s (errno=%d)", interface_name_, strerror(errno), errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return status;
    }

    // Bind socket to the CAN interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        // Use FailedPreconditionError if bind fails (e.g., address already in use)
        auto status = absl::FailedPreconditionError(absl::StrFormat("bind() to interface '%s' failed: %s (errno=%d)", interface_name_, strerror(errno), errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return status;
    }

    is_open_.store(true);
    return absl::OkStatus();
}

absl::Status CanInterface::close()
{
    if (!isOpen())
    {
        return absl::OkStatus(); // Already closed or never opened
    }

    int result = ::close(socket_fd_);
    const int saved_errno = errno; // Save errno immediately after close()
    socket_fd_ = -1;
    is_open_.store(false);

    if (result < 0)
    {
        // Use saved errno
        return absl::InternalError(absl::StrFormat("close() failed: %s (errno=%d)", strerror(saved_errno), saved_errno));
    }

    return absl::OkStatus();
}

bool CanInterface::isOpen() const noexcept
{
    // Check atomic flag and also if fd is valid (belt and suspenders)
    return is_open_.load() && (socket_fd_ >= 0);
}

// --- Implementation for sendFrame --- 
absl::Status CanInterface::sendFrame(const CanFrame& frame, std::chrono::milliseconds timeout)
{
    if (!isOpen())
    {
         return absl::FailedPreconditionError("Interface is not open.");
    }

    // Use default timeout if a negative value is provided
    const std::chrono::milliseconds final_timeout = (timeout.count() < 0) ? default_send_timeout_ : timeout;

    // Prepare for poll
    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLOUT; // Check for writability
    pfd.revents = 0;

    // Call poll() to wait for writability or timeout
    int poll_result = poll(&pfd, 1, static_cast<int>(final_timeout.count()));

    if (poll_result < 0)
    {
        // poll() error
        return CreateErrorStatus("poll() for sendFrame");
    }
    else if (poll_result == 0)
    {
        // poll() timed out
        return absl::DeadlineExceededError(absl::StrFormat("sendFrame timed out after %d ms", final_timeout.count()));
    }
    else
    { // poll_result > 0
        // Check if the event is POLLOUT
        if (!(pfd.revents & POLLOUT))
        {
            // Unexpected event or error on the socket
            // Could be POLLERR, POLLHUP, POLLNVAL - check socket error
            return checkSocketError(); 
        }

        // Socket is writable, attempt to send the frame
        struct can_frame linux_frame = frame.toLinuxCanFrame();
        ssize_t bytes_sent = send(socket_fd_, &linux_frame, sizeof(struct can_frame), MSG_DONTWAIT);

        if (bytes_sent < 0)
        {
            // Send error
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // This might happen if the buffer became full between poll() and send()
                // Treat as a timeout or temporary unavailability
                return absl::UnavailableError("sendFrame failed: Socket buffer full (EAGAIN/EWOULDBLOCK)");
            }
            else
            {
                return CreateErrorStatus("send()");
            }
        }
        else if (static_cast<size_t>(bytes_sent) != sizeof(struct can_frame))
        {
            // Partial send - should not happen with CAN_RAW sockets, indicates a problem
            return absl::InternalError(absl::StrFormat("send() sent incomplete frame: %d bytes instead of %d", bytes_sent, sizeof(struct can_frame)));
        }
        
        // Frame sent successfully
        return absl::OkStatus();
    }
}

// --- Placeholder implementation for receiveFrame --- 
absl::StatusOr<CanFrame> CanInterface::receiveFrame(std::chrono::milliseconds /*timeout*/)
{
    if (!isOpen())
    {
         return absl::FailedPreconditionError("Interface is not open.");
    }
    // Implementation deferred to Task 2.4
    return absl::UnimplementedError("receiveFrame not yet implemented");
}

// --- Implementation for checkSocketError helper --- 
absl::Status CanInterface::checkSocketError() const 
{
    int error = 0;
    socklen_t len = sizeof(error);
    if (getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &error, &len) == 0) 
    {
        if (error != 0) 
        {
             return absl::InternalError(absl::StrFormat("Socket error detected: %s (errno=%d)", strerror(error), error));
        }
    } else {
        // Failed to get socket error, report getsockopt error
        return CreateErrorStatus("getsockopt(SOL_SOCKET, SO_ERROR)");
    }
    // If no error was reported by getsockopt, but poll indicated an issue,
    // return a generic internal error.
    return absl::InternalError("Unknown socket error indicated by poll()");
}

} // namespace myactuator_rmd::core 