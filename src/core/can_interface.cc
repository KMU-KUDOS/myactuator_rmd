#include "myactuator_rmd/core/can_interface.h"

#include <errno.h>        // For errno
#include <fcntl.h>        // For fcntl, O_NONBLOCK
#include <linux/can/raw.h>// For SOL_CAN_RAW, CAN_RAW_FILTER, CAN_RAW_ERR_FILTER
#include <net/if.h>       // For struct ifreq, SIOCGIFINDEX
#include <string.h>       // For strerror, memset, strncpy
#include <sys/ioctl.h>    // For SIOCGIFINDEX
#include <sys/socket.h>   // For socket, bind, setsockopt, recv, send, struct sockaddr_can
#include <unistd.h>       // For close

#include <iostream> // For potential debug output, consider using ROS logger later

#include "absl/strings/str_format.h"

namespace myactuator_rmd::core
{

namespace
{
// Helper function to create a descriptive error status
absl::Status CreateErrorStatus(const std::string& context)
{
    return absl::InternalError(absl::StrFormat("%s failed: %s (errno=%d)", context, strerror(errno), errno));
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
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &enable_error_frames, sizeof(enable_error_frames)) < 0)
    {
        auto status = CreateErrorStatus("setsockopt(SOL_CAN_RAW, CAN_RAW_ERR_FILTER)");
        ::close(socket_fd_);
        socket_fd_ = -1;
        return status;
    }

    // Get interface index
    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0'; // Ensure null termination
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        auto status = CreateErrorStatus(absl::StrFormat("ioctl(SIOCGIFINDEX) for interface '%s'", interface_name_));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return status;
    }

    // Bind socket to the CAN interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        auto status = CreateErrorStatus(absl::StrFormat("bind() to interface '%s'", interface_name_));
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
    socket_fd_ = -1;
    is_open_.store(false);

    if (result < 0)
    {
        return CreateErrorStatus("close()");
    }

    return absl::OkStatus();
}

bool CanInterface::isOpen() const noexcept
{
    // Check atomic flag and also if fd is valid (belt and suspenders)
    return is_open_.load() && (socket_fd_ >= 0);
}

// --- Placeholder implementations for send/receive --- 

absl::Status CanInterface::sendFrame(const CanFrame& /*frame*/, std::chrono::milliseconds /*timeout*/)
{
    if (!isOpen())
    {
         return absl::FailedPreconditionError("Interface is not open.");
    }
    // Implementation deferred to Task 2.3
    return absl::UnimplementedError("sendFrame not yet implemented");
}

absl::StatusOr<CanFrame> CanInterface::receiveFrame(std::chrono::milliseconds /*timeout*/)
{
    if (!isOpen())
    {
         return absl::FailedPreconditionError("Interface is not open.");
    }
    // Implementation deferred to Task 2.4
    return absl::UnimplementedError("receiveFrame not yet implemented");
}

} // namespace myactuator_rmd::core 