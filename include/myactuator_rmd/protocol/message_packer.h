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

    // Read Commands
    static core::CanFrame pack_read_pid_data_command(uint8_t motor_id);                     // 0x30
    static core::CanFrame pack_read_encoder_command(uint8_t motor_id);                      // 0x90
    static core::CanFrame pack_read_multi_turn_angle_command(uint8_t motor_id);             // 0x92
    static core::CanFrame pack_read_single_turn_angle_command(uint8_t motor_id);            // 0x94
    static core::CanFrame pack_read_motor_status_1_command(uint8_t motor_id);               // 0x9A
    static core::CanFrame pack_read_motor_status_2_command(uint8_t motor_id);               // 0x9C
    static core::CanFrame pack_read_motor_status_3_command(uint8_t motor_id);               // 0x9D
    static core::CanFrame pack_read_firmware_version_command(uint8_t motor_id);             // 0x12

    // Write/Control Commands will be added in subsequent subtasks.
    // static core::CanFrame pack_write_pid_to_ram_command(...);
    // static core::CanFrame pack_torque_control_command(...);
    // ...

private:
    // Helper method to create a basic command frame structure.
    static core::CanFrame create_command_frame(uint8_t motor_id, uint8_t command_byte);

    // Static helper methods for writing data types to a CAN frame's data buffer
    // in Little-Endian byte order.
    // Bounds checking is performed in the .cc implementation.

    /**
     * @brief Writes a uint8_t value to the buffer at the specified offset.
     */
    static bool write_u8(uint8_t* data_ptr, uint8_t offset, uint8_t value, uint8_t max_size = 8);

    /**
     * @brief Writes an int16_t value to the buffer at the specified offset in little-endian.
     */
    static bool write_i16_le(uint8_t* data_ptr, uint8_t offset, int16_t value, uint8_t max_size = 8);

    /**
     * @brief Writes a uint16_t value to the buffer at the specified offset in little-endian.
     */
    static bool write_u16_le(uint8_t* data_ptr, uint8_t offset, uint16_t value, uint8_t max_size = 8);

    /**
     * @brief Writes an int32_t value to the buffer at the specified offset in little-endian.
     */
    static bool write_i32_le(uint8_t* data_ptr, uint8_t offset, int32_t value, uint8_t max_size = 8);

    /**
     * @brief Writes a uint32_t value to the buffer at the specified offset in little-endian.
     */
    static bool write_u32_le(uint8_t* data_ptr, uint8_t offset, uint32_t value, uint8_t max_size = 8);
    
    /**
     * @brief Writes a float value (as int32_t) to the buffer at the specified offset in little-endian.
     */
    static bool write_f32_le(uint8_t* data_ptr, uint8_t offset, float value, uint8_t max_size = 8);

};

} // namespace myactuator_rmd::protocol

#endif // MYACTUATOR_RMD_PROTOCOL_MESSAGE_PACKER_H_ 