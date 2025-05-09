#ifndef MYACTUATOR_RMD_PROTOCOL_MESSAGE_PACKER_H_
#define MYACTUATOR_RMD_PROTOCOL_MESSAGE_PACKER_H_

#include <cstdint>
#include <vector> // Potentially for multi-motor commands or dynamic data
#include <array>  // For CanFrame data array

#include "myactuator_rmd/core/can_frame.h"

namespace myactuator_rmd::protocol
{

/**
 * @brief Utility class for packing RMD motor control commands into CAN frames.
 *
 * This class provides static methods to construct CanFrame objects representing
 * various commands defined in the RMD V1.61 CAN bus protocol.
 */
class MessagePacker
{
public:
    // Delete constructor/destructor for a static utility class
    MessagePacker() = delete;
    ~MessagePacker() = delete;

    // Public static methods for packing specific RMD commands will be declared here
    // as part of subsequent subtasks. For example:
    //
    // static core::CanFrame pack_read_pid_parameters_command(uint8_t motor_id);
    // static core::CanFrame pack_torque_control_command(uint8_t motor_id, int16_t torque_current);
    // ... and so on for all commands listed in FR1.1 and FR1.2

private:
    // Helper method to create a basic command frame structure for most single-motor commands.
    // Most RMD commands share a common CAN ID structure (0x140 + motor_id) and DLC (8).
    // The first data byte is usually the command byte itself.
    static core::CanFrame create_base_command_frame(uint8_t motor_id, uint8_t command_byte);

    // Static helper methods for writing data types to a CAN frame's data buffer
    // in Little-Endian byte order.
    // These will be implemented in the .cc file.

    /**
     * @brief Writes a uint8_t value to the buffer at the specified offset.
     * @param buffer Pointer to the data buffer (uint8_t[8]).
     * @param offset Offset in bytes from the start of the buffer.
     * @param value The value to write.
     */
    static void write_u8(uint8_t* buffer, size_t offset, uint8_t value);

    /**
     * @brief Writes an int16_t value to the buffer at the specified offset in little-endian.
     * @param buffer Pointer to the data buffer.
     * @param offset Offset in bytes.
     * @param value The value to write.
     */
    static void write_i16_le(uint8_t* buffer, size_t offset, int16_t value);

    /**
     * @brief Writes a uint16_t value to the buffer at the specified offset in little-endian.
     * @param buffer Pointer to the data buffer.
     * @param offset Offset in bytes.
     * @param value The value to write.
     */
    static void write_u16_le(uint8_t* buffer, size_t offset, uint16_t value);

    /**
     * @brief Writes an int32_t value to the buffer at the specified offset in little-endian.
     * @param buffer Pointer to the data buffer.
     * @param offset Offset in bytes.
     * @param value The value to write.
     */
    static void write_i32_le(uint8_t* buffer, size_t offset, int32_t value);

    /**
     * @brief Writes a uint32_t value to the buffer at the specified offset in little-endian.
     * @param buffer Pointer to the data buffer.
     * @param offset Offset in bytes.
     * @param value The value to write.
     */
    static void write_u32_le(uint8_t* buffer, size_t offset, uint32_t value);

    // Note: The RMD protocol often uses scaled integers instead of direct floats.
    // If direct float-to-bytes conversion is needed (e.g., for a different protocol or utility),
    // a write_f32_le could be added, but for RMD, it's usually about packing specific integer types.
    // static void write_f32_le(uint8_t* buffer, size_t offset, float value);

    // Allow specific command packing methods to be friends if they need direct access
    // to these write helpers and create_base_command_frame, though typically they would just call them.
};

} // namespace myactuator_rmd::protocol

#endif // MYACTUATOR_RMD_PROTOCOL_MESSAGE_PACKER_H_ 