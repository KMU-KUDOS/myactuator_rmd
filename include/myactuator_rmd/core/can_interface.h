#ifndef MYACTUATOR_RMD_CORE_CAN_INTERFACE_H_
#define MYACTUATOR_RMD_CORE_CAN_INTERFACE_H_

#include <string>
#include <atomic>

#include "myactuator_rmd/core/can_frame.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
namespace myactuator_rmd::core
{

/**
 * @brief Provides an interface for interacting with a Linux SocketCAN device.
 *
 * This class wraps the low-level SocketCAN API to provide methods for
 * opening, closing, sending, and receiving CAN frames in a non-blocking manner
 * suitable for real-time applications.
 */
class CanInterface
{
public:
    /**
     * @brief Constructs a CanInterface instance.
     *
     * @param interface_name The name of the CAN interface (e.g., "can0").
     * @param default_receive_timeout Default timeout for receive operations.
     * @param default_send_timeout Default timeout for send operations.
     */
    explicit CanInterface(
        std::string interface_name,
        absl::Duration default_receive_timeout = absl::Milliseconds(10),
        absl::Duration default_send_timeout = absl::Milliseconds(10));

    /**
     * @brief Destructor. Automatically closes the interface if it is open.
     */
    ~CanInterface();

    // Disable copy and move operations to prevent issues with resource management (socket fd).
    CanInterface(const CanInterface&) = delete;
    CanInterface& operator=(const CanInterface&) = delete;
    CanInterface(CanInterface&&) = delete;
    CanInterface& operator=(CanInterface&&) = delete;

    /**
     * @brief Opens and initializes the CAN interface.
     *
     * Creates a SocketCAN socket, sets it to non-blocking mode,
     * enables error frame reception, and binds it to the specified interface name.
     *
     * @return absl::OkStatus() on success, or an error status otherwise.
     */
    absl::Status open();

    /**
     * @brief Closes the CAN interface.
     *
     * If the interface is open, the underlying socket file descriptor is closed.
     *
     * @return absl::OkStatus() on success, or an error status if closing fails.
     */
    absl::Status close();

    /**
     * @brief Checks if the CAN interface is currently open.
     *
     * @return True if the interface is open and the socket is valid, false otherwise.
     */
    bool is_open() const noexcept;

    /**
     * @brief Sends a CAN frame.
     *
     * Attempts to send the given frame within the specified timeout.
     * If timeout is negative (or absl::InfiniteDuration), the default send timeout is used.
     * A zero timeout means non-blocking.
     *
     * @param frame The CanFrame to send.
     * @param timeout The maximum time to wait for the send operation to complete.
     *                Use absl::Milliseconds(-1) or similar for default timeout.
     * @return absl::OkStatus() on success, or an error status (e.g., timeout, socket error).
     */
    absl::Status send_frame(const CanFrame& frame,
                           absl::Duration timeout = absl::Milliseconds(-1));

    /**
     * @brief Receives a CAN frame.
     *
     * Attempts to receive a frame within the specified timeout.
     * If timeout is negative (or absl::InfiniteDuration), the default receive timeout is used.
     * A zero timeout means non-blocking.
     *
     * @param timeout The maximum time to wait for a frame to be received.
     *                Use absl::Milliseconds(-1) or similar for default timeout.
     * @return An absl::StatusOr containing the received CanFrame on success,
     *         or an error status (e.g., timeout, socket error, received error frame).
     */
    absl::StatusOr<CanFrame> receive_frame(
        absl::Duration timeout = absl::Milliseconds(-1));

private:
    int socket_fd_{-1};                 // Socket file descriptor, -1 if closed.
    const std::string interface_name_; // Name of the CAN interface (e.g., "can0").
    std::atomic<bool> is_open_{false}; // Atomic flag indicating if the interface is open.

    const absl::Duration default_receive_timeout_;
    const absl::Duration default_send_timeout_;

    // Potential helper methods for internal use (implementation in .cpp)
    absl::Status check_socket_error() const;
};

} // namespace myactuator_rmd::core

#endif // MYACTUATOR_RMD_CORE_CAN_INTERFACE_H_ 