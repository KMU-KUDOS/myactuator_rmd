#ifndef MYACTUATOR_RMD_CORE_CAN_FRAME_H_
#define MYACTUATOR_RMD_CORE_CAN_FRAME_H_

#include <cstdint>
#include <string> // For std::string in toString() (if implemented here)
#include <array>  // For std::array
#include <linux/can.h> // For struct can_frame and CAN_MAX_DLEN

namespace myactuator_rmd::core
{

// Constants for CAN ID flags, aligned with <linux/can.h>
// These are primarily for clarity if direct bitwise operations are preferred by users,
// but generally, direct use of CAN_EFF_FLAG etc. from <linux/can.h> is better.
constexpr ::canid_t CAN_ID_EFF_FLAG = CAN_EFF_FLAG; // 0x80000000U
constexpr ::canid_t CAN_ID_RTR_FLAG = CAN_RTR_FLAG; // 0x40000000U
constexpr ::canid_t CAN_ID_ERR_FLAG = CAN_ERR_FLAG; // 0x20000000U

// Masks for extracting the ID part
constexpr ::canid_t CAN_ID_SFF_MASK = CAN_SFF_MASK; // 0x000007FFU
constexpr ::canid_t CAN_ID_EFF_MASK = CAN_EFF_MASK; // 0x1FFFFFFFU
constexpr ::canid_t CAN_ID_ERR_MASK = CAN_ERR_MASK; // 0x1FFFFFFFU (error mask is also 29 bits in Linux)

/**
 * @brief Represents a single CAN (Controller Area Network) frame.
 *
 * This structure is an abstraction over the Linux SocketCAN `struct can_frame`,
 * providing a more C++ idiomatic way to handle CAN messages.
 */
struct CanFrame
{
    ::canid_t id {0};      // CAN ID (including EFF, RTR, ERR flags)
    uint8_t dlc {0};     // Data Length Code (0-8 bytes)
    std::array<uint8_t, CAN_MAX_DLEN> data {}; // Payload data, CAN_MAX_DLEN is usually 8

    /**
     * @brief Default constructor.
     */
    CanFrame() = default;

    /**
     * @brief Constructs a CanFrame with ID, data, and DLC.
     *
     * @param can_id The CAN identifier (including any flags like EFF, RTR).
     * @param payload The data payload. Only the first `data_len` bytes are used.
     * @param data_len The length of the data payload (0-8).
     */
    CanFrame(::canid_t can_id, const std::array<uint8_t, CAN_MAX_DLEN>& payload, uint8_t data_len);

    /**
     * @brief Constructs a CanFrame from a Linux SocketCAN frame.
     *
     * @param linux_frame The `struct can_frame` to convert from.
     */
    explicit CanFrame(const struct can_frame& linux_frame);

    /**
     * @brief Converts this CanFrame to a Linux SocketCAN `struct can_frame`.
     *
     * @return The equivalent `struct can_frame`.
     */
    struct can_frame toLinuxCanFrame() const;

    /**
     * @brief Checks if the frame is an Extended Frame Format (EFF) frame.
     * @return True if it's an extended frame, false otherwise.
     */
    bool isExtended() const noexcept;

    /**
     * @brief Checks if the frame is a Remote Transmission Request (RTR) frame.
     * @return True if it's an RTR frame, false otherwise.
     */
    bool isRemote() const noexcept;

    /**
     * @brief Checks if the frame is an error frame.
     * @return True if it's an error frame, false otherwise.
     */
    bool isError() const noexcept;

    /**
     * @brief Gets the actual CAN ID without any flags (EFF, RTR, ERR).
     * @return The CAN ID (11-bit for standard, 29-bit for extended).
     */
    ::canid_t getPlainId() const noexcept;

    /**
     * @brief Sets the plain CAN ID, preserving existing flags.
     * @param plain_id The CAN ID to set (11-bit or 29-bit).
     *                 If the frame is already marked extended, plain_id is masked with CAN_EFF_MASK.
     *                 Otherwise, it's masked with CAN_SFF_MASK.
     */
    void setPlainId(::canid_t plain_id) noexcept;

    /**
     * @brief Sets the frame as an Extended Frame Format (EFF) frame.
     * @param extended True to set as extended, false for standard.
     *                 If setting to standard, the ID is masked with CAN_SFF_MASK.
     *                 If setting to extended, the ID is masked with CAN_EFF_MASK.
     */
    void setExtended(bool extended = true) noexcept;
    
    /**
     * @brief Returns a string representation of the CAN frame for debugging.
     * @return A string describing the frame.
     */
    std::string toString() const;
};

// Free function converters (alternative to member/constructor converters if preferred)
// CanFrame toCanFrame(const struct can_frame& linux_frame);
// struct can_frame toLinuxCanFrame(const CanFrame& frame);

} // namespace myactuator_rmd::core

#endif // MYACTUATOR_RMD_CORE_CAN_FRAME_H_ 