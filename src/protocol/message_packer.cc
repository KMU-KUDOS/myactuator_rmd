#include "myactuator_rmd/protocol/message_packer.h"

#include <cstring> // For memcpy, though direct assignment is often used for these types

namespace myactuator_rmd::protocol
{

core::CanFrame MessagePacker::create_base_command_frame(uint8_t motor_id, uint8_t command_byte)
{
    core::CanFrame frame{}; // Value-initialize (zeroes out members)
    frame.id = 0x140 + motor_id;
    frame.is_extended_id = false;
    frame.is_rtr = false;
    frame.dlc = 8; // Most RMD commands use 8-byte DLC
    // frame.data is already zeroed by value initialization.
    frame.data[0] = command_byte;
    return frame;
}

void MessagePacker::write_u8(uint8_t* buffer, size_t offset, uint8_t value)
{
    if (buffer && (offset < core::CanFrame::kMaxDataLength))
    {
        buffer[offset] = value;
    }
    // Else: error or out of bounds, could log or assert in a debug build
}

void MessagePacker::write_i16_le(uint8_t* buffer, size_t offset, int16_t value)
{
    if (buffer && (offset + 1 < core::CanFrame::kMaxDataLength)) // Need 2 bytes
    {
        buffer[offset]     = static_cast<uint8_t>(value & 0xFF);
        buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    }
}

void MessagePacker::write_u16_le(uint8_t* buffer, size_t offset, uint16_t value)
{
    if (buffer && (offset + 1 < core::CanFrame::kMaxDataLength)) // Need 2 bytes
    {
        buffer[offset]     = static_cast<uint8_t>(value & 0xFF);
        buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    }
}

void MessagePacker::write_i32_le(uint8_t* buffer, size_t offset, int32_t value)
{
    if (buffer && (offset + 3 < core::CanFrame::kMaxDataLength)) // Need 4 bytes
    {
        buffer[offset]     = static_cast<uint8_t>(value & 0xFF);
        buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        buffer[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
        buffer[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
    }
}

void MessagePacker::write_u32_le(uint8_t* buffer, size_t offset, uint32_t value)
{
    if (buffer && (offset + 3 < core::CanFrame::kMaxDataLength)) // Need 4 bytes
    {
        buffer[offset]     = static_cast<uint8_t>(value & 0xFF);
        buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        buffer[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
        buffer[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
    }
}

// Placeholder for actual command packing methods to be added in later subtasks.

} // namespace myactuator_rmd::protocol 