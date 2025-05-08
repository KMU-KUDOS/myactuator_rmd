#ifndef MYACTUATOR_RMD_CORE_CAN_INTERFACE_H_
#define MYACTUATOR_RMD_CORE_CAN_INTERFACE_H_

#include <string>
#include <atomic>
#include <chrono>

#include "myactuator_rmd/core/can_frame.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"

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
        std::chrono::milliseconds default_receive_timeout = std::chrono::milliseconds(10),
        std::chrono::milliseconds default_send_timeout = std::chrono::milliseconds(10));

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
    bool isOpen() const noexcept;

    /**
     * @brief Sends a CAN frame.
     *
     * Attempts to send the given frame within the specified timeout.
     * If timeout is negative, the default send timeout is used.
     *
     * @param frame The CanFrame to send.
     * @param timeout The maximum time to wait for the send operation to complete.
     * @return absl::OkStatus() on success, or an error status (e.g., timeout, socket error).
     */
    absl::Status sendFrame(const CanFrame& frame,
                           std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));

    /**
     * @brief Receives a CAN frame.
     *
     * Attempts to receive a frame within the specified timeout.
     * If timeout is negative, the default receive timeout is used.
     *
     * @param timeout The maximum time to wait for a frame to be received.
     * @return An absl::StatusOr containing the received CanFrame on success,
     *         or an error status (e.g., timeout, socket error, received error frame).
     */
    absl::StatusOr<CanFrame> receiveFrame(
        std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));

private:
    int socket_fd_{-1};                 // Socket file descriptor, -1 if closed.
    const std::string interface_name_; // Name of the CAN interface (e.g., "can0").
    std::atomic<bool> is_open_{false}; // Atomic flag indicating if the interface is open.

    const std::chrono::milliseconds default_receive_timeout_;
    const std::chrono::milliseconds default_send_timeout_;

    // Potential helper methods for internal use (implementation in .cpp)
    absl::Status checkSocketError() const;
};

} // namespace myactuator_rmd::core

#endif // MYACTUATOR_RMD_CORE_CAN_INTERFACE_H_ 