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

    // Write/Set Commands
    static core::CanFrame pack_write_pid_to_ram_command(uint8_t motor_id, uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t torque_kp, uint8_t torque_ki); // 0x31
    static core::CanFrame pack_write_pid_to_rom_command(uint8_t motor_id);                     // 0x32
    static core::CanFrame pack_write_encoder_offset_command(uint8_t motor_id, uint16_t encoder_offset); // 0x91
    static core::CanFrame pack_clear_motor_error_flags_command(uint8_t motor_id);            // 0x9B
    // Note: Set Motor ID (0x79) uses a fixed CAN ID of 0x79. current_motor_id might be for addressing before change.
    static core::CanFrame pack_set_motor_id_command(uint8_t new_motor_id_payload); // CAN ID fixed to 0x79, payload is new ID.
    static core::CanFrame pack_set_communication_baud_rate_command(uint8_t motor_id, uint8_t baud_rate_index); // 0xB4

    // Motion Control Commands
    static core::CanFrame pack_torque_control_command(uint8_t motor_id, int16_t torque_current_ma); // 0xA1 (Torque current is in mA per some docs, or 0.01A/LSB. Assuming direct value from user for now)
    static core::CanFrame pack_speed_control_command(uint8_t motor_id, int32_t speed_dps_scaled);    // 0xA2 (Speed is 0.01 dps/LSB)
    static core::CanFrame pack_position_control_1_command(uint8_t motor_id, int32_t position_deg_scaled); // 0xA3 (Position is 0.01 deg/LSB)
    static core::CanFrame pack_position_control_2_command(uint8_t motor_id, uint16_t speed_limit_dps_scaled, int32_t position_deg_scaled); // 0xA4
    static core::CanFrame pack_position_control_3_command(uint8_t motor_id, uint8_t spin_direction, int16_t position_deg_scaled_short); // 0xA5 (Position is 0.01 deg/LSB, short int16_t)
    static core::CanFrame pack_position_control_4_command(uint8_t motor_id, uint8_t spin_direction, uint16_t speed_limit_dps_scaled, int16_t position_deg_scaled_short); // 0xA6
    static core::CanFrame pack_motor_stop_command(uint8_t motor_id);                               // 0x81
    static core::CanFrame pack_motor_off_command(uint8_t motor_id);                                // 0x80 (Motor Off / Power Down)

    // Multi-Motor Commands
    static core::CanFrame pack_multi_motor_torque_control_command(
        int16_t torque_motor1_scaled, 
        int16_t torque_motor2_scaled, 
        int16_t torque_motor3_scaled, 
        int16_t torque_motor4_scaled);

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