#include "myactuator_rmd/protocol/message_packer.h"

// #include <cstring> // For memcpy, not strictly needed for these implementations

namespace myactuator_rmd::protocol
{

// Renamed from create_base_command_frame
core::CanFrame MessagePacker::create_command_frame(uint8_t motor_id, uint8_t command_byte)
{
    core::CanFrame frame{}; // Value-initialize (zeroes out members)
    if (motor_id == 0 || motor_id > 0x1F) { // RMD motor IDs are typically 1-31 (0x01-0x1F)
        // Handle invalid motor_id, perhaps log an error or return an invalid frame.
        // For now, proceed but this should be validated.
        // Or, throw an exception if exceptions are allowed by project style (they are not).
        // Returning an empty/default frame might be one way if status isn't returned.
    }
    frame.id = 0x140 + motor_id; // Standard CAN ID for RMD commands
    frame.is_extended_id = false;
    frame.is_rtr = false;
    frame.dlc = 8; // Most RMD commands use 8-byte DLC
    frame.data.fill(0U); // Initialize data to zeros explicitly
    frame.data[0] = command_byte;
    return frame;
}

bool MessagePacker::write_u8(uint8_t* data_ptr, uint8_t offset, uint8_t value, uint8_t max_size)
{
    if (!data_ptr || offset >= max_size)
    {
        return false; // Error: null pointer or out of bounds
    }
    data_ptr[offset] = value;
    return true;
}

bool MessagePacker::write_i16_le(uint8_t* data_ptr, uint8_t offset, int16_t value, uint8_t max_size)
{
    if (!data_ptr || (offset + sizeof(int16_t) > max_size)) // Need 2 bytes
    {
        return false; // Error: null pointer or not enough space
    }
    data_ptr[offset]     = static_cast<uint8_t>(value & 0xFF);
    data_ptr[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    return true;
}

bool MessagePacker::write_u16_le(uint8_t* data_ptr, uint8_t offset, uint16_t value, uint8_t max_size)
{
    if (!data_ptr || (offset + sizeof(uint16_t) > max_size)) // Need 2 bytes
    {
        return false; // Error: null pointer or not enough space
    }
    data_ptr[offset]     = static_cast<uint8_t>(value & 0xFF);
    data_ptr[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    return true;
}

bool MessagePacker::write_i32_le(uint8_t* data_ptr, uint8_t offset, int32_t value, uint8_t max_size)
{
    if (!data_ptr || (offset + sizeof(int32_t) > max_size)) // Need 4 bytes
    {
        return false; // Error: null pointer or not enough space
    }
    data_ptr[offset]     = static_cast<uint8_t>(value & 0xFF);
    data_ptr[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_ptr[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data_ptr[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
    return true;
}

bool MessagePacker::write_u32_le(uint8_t* data_ptr, uint8_t offset, uint32_t value, uint8_t max_size)
{
    if (!data_ptr || (offset + sizeof(uint32_t) > max_size)) // Need 4 bytes
    {
        return false; // Error: null pointer or not enough space
    }
    data_ptr[offset]     = static_cast<uint8_t>(value & 0xFF);
    data_ptr[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_ptr[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data_ptr[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
    return true;
}

bool MessagePacker::write_f32_le(uint8_t* data_ptr, uint8_t offset, float value, uint8_t max_size) {
    if (!data_ptr || (offset + sizeof(float) > max_size)) { // Need 4 bytes for a float
        return false; // Error: null pointer or not enough space
    }
    // Reinterpret float as int32_t/uint32_t for byte representation
    union {
        float f_val;
        uint32_t u_val;
    } converter;
    converter.f_val = value;
    return write_u32_le(data_ptr, offset, converter.u_val, max_size); // Use u32_le for bitwise copy
}


// Implementations for Read Commands

core::CanFrame MessagePacker::pack_read_pid_data_command(uint8_t motor_id)
{
    // Command: 0x30
    // Data: No specific data bytes needed for this read command beyond command byte itself.
    return create_command_frame(motor_id, 0x30);
}

core::CanFrame MessagePacker::pack_read_encoder_command(uint8_t motor_id)
{
    // Command: 0x90
    // Data: No specific data bytes needed for this read command.
    // Response contains: Encoder Position, Encoder Original Position, Encoder Offset
    return create_command_frame(motor_id, 0x90);
}

core::CanFrame MessagePacker::pack_read_multi_turn_angle_command(uint8_t motor_id)
{
    // Command: 0x92
    // Data: No specific data bytes needed.
    return create_command_frame(motor_id, 0x92);
}

core::CanFrame MessagePacker::pack_read_single_turn_angle_command(uint8_t motor_id)
{
    // Command: 0x94
    // Data: No specific data bytes needed.
    return create_command_frame(motor_id, 0x94);
}

core::CanFrame MessagePacker::pack_read_motor_status_1_command(uint8_t motor_id)
{
    // Command: 0x9A
    // Data: No specific data bytes needed.
    // Response contains: Temperature, Voltage, ErrorState
    return create_command_frame(motor_id, 0x9A);
}

core::CanFrame MessagePacker::pack_read_motor_status_2_command(uint8_t motor_id)
{
    // Command: 0x9C
    // Data: No specific data bytes needed.
    // Response contains: Temperature, Voltage, Speed, Encoder Position
    return create_command_frame(motor_id, 0x9C);
}

core::CanFrame MessagePacker::pack_read_motor_status_3_command(uint8_t motor_id)
{
    // Command: 0x9D
    // Data: No specific data bytes needed.
    // Response contains: Temperature, Phase A Current, Phase B Current, Phase C Current
    return create_command_frame(motor_id, 0x9D);
}

core::CanFrame MessagePacker::pack_read_firmware_version_command(uint8_t motor_id)
{
    // Command: 0x12 (RMD-L V3.x uses 0x12, older might use others)
    // Data: No specific data bytes needed.
    // Assuming 0x12 based on some RMD docs. V1.61 might differ, needs verification.
    // If V1.61 uses a different command for firmware, this should be updated.
    // For RMD-X series, 0xB2 is firmware version. Let's assume 0x12 is a generic one or specific to a sub-model.
    // The PRD does not specify this command, so 0x12 is an assumption.
    // If no general firmware version command for V1.61, this function might be removed or adapted.
    // Protocol V1.61 document says 0x12 for "Read Product Model" which includes firmware date.
    return create_command_frame(motor_id, 0x12);
}

// Placeholder for actual command packing methods to be added in later subtasks.

} // namespace myactuator_rmd::protocol 