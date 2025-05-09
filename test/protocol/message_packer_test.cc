#include "gtest/gtest.h"
#include "myactuator_rmd/protocol/message_packer.h"
#include "myactuator_rmd/core/can_frame.h" // For CanFrame definition and kMaxDataLength

#include <array> // For std::array

// Using namespace for convenience in test file
using namespace myactuator_rmd::protocol;
using namespace myactuator_rmd::core;

class MessagePackerTest : public ::testing::Test {
protected:
    std::array<uint8_t, CanFrame::kMaxDataLength> buffer_{}; // Test buffer

    void SetUp() override {
        // Initialize buffer to a known state before each test, e.g., all zeros or a pattern
        buffer_.fill(0xAA); // Fill with a pattern to ensure writes are happening
    }

    // void TearDown() override {} // No specific teardown needed for these tests
};

// --- Test Cases for Utility Functions ---

TEST_F(MessagePackerTest, WriteU8) {
    // Success case
    EXPECT_TRUE(MessagePacker::write_u8(buffer_.data(), 2, 0x12));
    EXPECT_EQ(buffer_[2], 0x12);

    // Boundary: Write at start
    EXPECT_TRUE(MessagePacker::write_u8(buffer_.data(), 0, 0x34));
    EXPECT_EQ(buffer_[0], 0x34);

    // Boundary: Write at end
    EXPECT_TRUE(MessagePacker::write_u8(buffer_.data(), CanFrame::kMaxDataLength - 1, 0x56));
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 1], 0x56);

    // Out of bounds: Offset too large
    EXPECT_FALSE(MessagePacker::write_u8(buffer_.data(), CanFrame::kMaxDataLength, 0x78));

    // Null buffer
    EXPECT_FALSE(MessagePacker::write_u8(nullptr, 0, 0x9A));
}

TEST_F(MessagePackerTest, WriteI16LE) {
    // Success case: Positive value
    EXPECT_TRUE(MessagePacker::write_i16_le(buffer_.data(), 1, 0x1234)); // 4660 decimal
    EXPECT_EQ(buffer_[1], 0x34); // Low byte
    EXPECT_EQ(buffer_[2], 0x12); // High byte

    // Success case: Negative value
    buffer_.fill(0xAA); // Reset buffer
    EXPECT_TRUE(MessagePacker::write_i16_le(buffer_.data(), 3, -2)); // 0xFFFE
    EXPECT_EQ(buffer_[3], 0xFE);
    EXPECT_EQ(buffer_[4], 0xFF);

    // Boundary: Write at end
    buffer_.fill(0xAA);
    EXPECT_TRUE(MessagePacker::write_i16_le(buffer_.data(), CanFrame::kMaxDataLength - 2, 0x5678));
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 2], 0x78);
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 1], 0x56);

    // Out of bounds: Offset too large for 2 bytes
    EXPECT_FALSE(MessagePacker::write_i16_le(buffer_.data(), CanFrame::kMaxDataLength - 1, 0xABCD));

    // Null buffer
    EXPECT_FALSE(MessagePacker::write_i16_le(nullptr, 0, 0xABCD));
}

TEST_F(MessagePackerTest, WriteU16LE) {
    // Success case
    EXPECT_TRUE(MessagePacker::write_u16_le(buffer_.data(), 1, 0xABCD));
    EXPECT_EQ(buffer_[1], 0xCD); // Low byte
    EXPECT_EQ(buffer_[2], 0xAB); // High byte

    // Boundary: Write at end
    buffer_.fill(0xAA);
    EXPECT_TRUE(MessagePacker::write_u16_le(buffer_.data(), CanFrame::kMaxDataLength - 2, 0x1234));
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 2], 0x34);
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 1], 0x12);
    
    // Out of bounds
    EXPECT_FALSE(MessagePacker::write_u16_le(buffer_.data(), CanFrame::kMaxDataLength - 1, 0xEF01));

    // Null buffer
    EXPECT_FALSE(MessagePacker::write_u16_le(nullptr, 0, 0xEF01));
}

TEST_F(MessagePackerTest, WriteI32LE) {
    // Success case: Positive value
    EXPECT_TRUE(MessagePacker::write_i32_le(buffer_.data(), 0, 0x12345678));
    EXPECT_EQ(buffer_[0], 0x78);
    EXPECT_EQ(buffer_[1], 0x56);
    EXPECT_EQ(buffer_[2], 0x34);
    EXPECT_EQ(buffer_[3], 0x12);

    // Success case: Negative value
    buffer_.fill(0xAA);
    EXPECT_TRUE(MessagePacker::write_i32_le(buffer_.data(), 2, -3)); // 0xFFFFFFFD
    EXPECT_EQ(buffer_[2], 0xFD);
    EXPECT_EQ(buffer_[3], 0xFF);
    EXPECT_EQ(buffer_[4], 0xFF);
    EXPECT_EQ(buffer_[5], 0xFF);

    // Boundary: Write at end
    buffer_.fill(0xAA);
    EXPECT_TRUE(MessagePacker::write_i32_le(buffer_.data(), CanFrame::kMaxDataLength - 4, 0xABCDEF01));
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 4], 0x01);
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 3], 0xEF);
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 2], 0xCD);
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 1], 0xAB);

    // Out of bounds
    EXPECT_FALSE(MessagePacker::write_i32_le(buffer_.data(), CanFrame::kMaxDataLength - 3, 0x12345678));
    
    // Null buffer
    EXPECT_FALSE(MessagePacker::write_i32_le(nullptr, 0, 0x12345678));
}

TEST_F(MessagePackerTest, WriteU32LE) {
    // Success case
    EXPECT_TRUE(MessagePacker::write_u32_le(buffer_.data(), 1, 0xAABBCCDD));
    EXPECT_EQ(buffer_[1], 0xDD);
    EXPECT_EQ(buffer_[2], 0xCC);
    EXPECT_EQ(buffer_[3], 0xBB);
    EXPECT_EQ(buffer_[4], 0xAA);

    // Boundary: Write at end
    buffer_.fill(0xAA);
    EXPECT_TRUE(MessagePacker::write_u32_le(buffer_.data(), CanFrame::kMaxDataLength - 4, 0x11223344));
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 4], 0x44);
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 3], 0x33);
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 2], 0x22);
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 1], 0x11);
    
    // Out of bounds
    EXPECT_FALSE(MessagePacker::write_u32_le(buffer_.data(), CanFrame::kMaxDataLength - 2, 0x55667788));

    // Null buffer
    EXPECT_FALSE(MessagePacker::write_u32_le(nullptr, 0, 0x55667788));
}

TEST_F(MessagePackerTest, WriteF32LE) {
    // Success case
    // IEEE 754 representation for 123.456f is approx 0x42F6E979
    float test_float = 123.456f;
    uint32_t expected_u32 = 0;
    std::memcpy(&expected_u32, &test_float, sizeof(float)); // Get bit representation

    EXPECT_TRUE(MessagePacker::write_f32_le(buffer_.data(), 0, test_float));
    EXPECT_EQ(buffer_[0], static_cast<uint8_t>(expected_u32 & 0xFF));
    EXPECT_EQ(buffer_[1], static_cast<uint8_t>((expected_u32 >> 8) & 0xFF));
    EXPECT_EQ(buffer_[2], static_cast<uint8_t>((expected_u32 >> 16) & 0xFF));
    EXPECT_EQ(buffer_[3], static_cast<uint8_t>((expected_u32 >> 24) & 0xFF));

    // Boundary: Write at end
    buffer_.fill(0xAA);
    // IEEE 754 representation for -98.765f is approx 0xC2C5851F
    float boundary_float = -98.765f; 
    std::memcpy(&expected_u32, &boundary_float, sizeof(float));
    EXPECT_TRUE(MessagePacker::write_f32_le(buffer_.data(), CanFrame::kMaxDataLength - 4, boundary_float));
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 4], static_cast<uint8_t>(expected_u32 & 0xFF));
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 3], static_cast<uint8_t>((expected_u32 >> 8) & 0xFF));
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 2], static_cast<uint8_t>((expected_u32 >> 16) & 0xFF));
    EXPECT_EQ(buffer_[CanFrame::kMaxDataLength - 1], static_cast<uint8_t>((expected_u32 >> 24) & 0xFF));

    // Out of bounds
    EXPECT_FALSE(MessagePacker::write_f32_le(buffer_.data(), CanFrame::kMaxDataLength - 3, 1.0f));
    
    // Null buffer
    EXPECT_FALSE(MessagePacker::write_f32_le(nullptr, 0, 1.0f));
}


// --- Test Cases for create_command_frame ---

TEST_F(MessagePackerTest, CreateCommandFrameValid) {
    uint8_t motor_id = 0x0A;
    uint8_t command_byte = 0x30; // Example: Read PID Data
    CanFrame frame = MessagePacker::create_command_frame(motor_id, command_byte);

    EXPECT_EQ(frame.id, 0x140 + motor_id);
    EXPECT_FALSE(frame.is_extended_id);
    EXPECT_FALSE(frame.is_rtr);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_EQ(frame.data[0], command_byte);
    for (size_t i = 1; i < frame.dlc; ++i) {
        EXPECT_EQ(frame.data[i], 0x00) << "Byte at index " << i << " was not zero.";
    }
}

TEST_F(MessagePackerTest, CreateCommandFrameMotorIdZero) {
    // Test behavior with motor_id = 0 (should ideally be caught or handled, but test current behavior)
    uint8_t motor_id = 0x00; 
    uint8_t command_byte = 0x9A;
    CanFrame frame = MessagePacker::create_command_frame(motor_id, command_byte);
    EXPECT_EQ(frame.id, 0x140 + motor_id); // ID will be 0x140
    EXPECT_EQ(frame.data[0], command_byte);
}

TEST_F(MessagePackerTest, CreateCommandFrameMotorIdOutOfRange) {
    // Test behavior with motor_id > 0x1F (e.g., 32)
    uint8_t motor_id = 32; // 0x20
    uint8_t command_byte = 0x9C;
    CanFrame frame = MessagePacker::create_command_frame(motor_id, command_byte);
    // ID will be 0x140 + 32 = 0x160. The function currently doesn't prevent this.
    EXPECT_EQ(frame.id, 0x140 + motor_id); 
    EXPECT_EQ(frame.data[0], command_byte);
}

// --- Test Cases for Read/Status Command Packing Methods ---

TEST_F(MessagePackerTest, PackReadPidDataCommand) {
    uint8_t motor_id = 0x05;
    CanFrame frame = MessagePacker::pack_read_pid_data_command(motor_id);
    EXPECT_EQ(frame.id, 0x140 + motor_id);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_EQ(frame.data[0], 0x30);
    for (size_t i = 1; i < frame.dlc; ++i) EXPECT_EQ(frame.data[i], 0);
}

TEST_F(MessagePackerTest, PackReadEncoderCommand) {
    uint8_t motor_id = 0x0B;
    CanFrame frame = MessagePacker::pack_read_encoder_command(motor_id);
    EXPECT_EQ(frame.id, 0x140 + motor_id);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_EQ(frame.data[0], 0x90);
    for (size_t i = 1; i < frame.dlc; ++i) EXPECT_EQ(frame.data[i], 0);
}

TEST_F(MessagePackerTest, PackReadMultiTurnAngleCommand) {
    uint8_t motor_id = 0x01;
    CanFrame frame = MessagePacker::pack_read_multi_turn_angle_command(motor_id);
    EXPECT_EQ(frame.id, 0x140 + motor_id);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_EQ(frame.data[0], 0x92);
    for (size_t i = 1; i < frame.dlc; ++i) EXPECT_EQ(frame.data[i], 0);
}

TEST_F(MessagePackerTest, PackReadSingleTurnAngleCommand) {
    uint8_t motor_id = 0x1F;
    CanFrame frame = MessagePacker::pack_read_single_turn_angle_command(motor_id);
    EXPECT_EQ(frame.id, 0x140 + motor_id);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_EQ(frame.data[0], 0x94);
    for (size_t i = 1; i < frame.dlc; ++i) EXPECT_EQ(frame.data[i], 0);
}

TEST_F(MessagePackerTest, PackReadMotorStatus1Command) {
    uint8_t motor_id = 0x03;
    CanFrame frame = MessagePacker::pack_read_motor_status_1_command(motor_id);
    EXPECT_EQ(frame.id, 0x140 + motor_id);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_EQ(frame.data[0], 0x9A);
    for (size_t i = 1; i < frame.dlc; ++i) EXPECT_EQ(frame.data[i], 0);
}

TEST_F(MessagePackerTest, PackReadMotorStatus2Command) {
    uint8_t motor_id = 0x07;
    CanFrame frame = MessagePacker::pack_read_motor_status_2_command(motor_id);
    EXPECT_EQ(frame.id, 0x140 + motor_id);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_EQ(frame.data[0], 0x9C);
    for (size_t i = 1; i < frame.dlc; ++i) EXPECT_EQ(frame.data[i], 0);
}

TEST_F(MessagePackerTest, PackReadMotorStatus3Command) {
    uint8_t motor_id = 0x0D;
    CanFrame frame = MessagePacker::pack_read_motor_status_3_command(motor_id);
    EXPECT_EQ(frame.id, 0x140 + motor_id);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_EQ(frame.data[0], 0x9D);
    for (size_t i = 1; i < frame.dlc; ++i) EXPECT_EQ(frame.data[i], 0);
}

TEST_F(MessagePackerTest, PackReadFirmwareVersionCommand) {
    uint8_t motor_id = 0x02;
    CanFrame frame = MessagePacker::pack_read_firmware_version_command(motor_id);
    EXPECT_EQ(frame.id, 0x140 + motor_id);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_EQ(frame.data[0], 0x12); // Assuming 0x12 is correct as per previous implementation
    for (size_t i = 1; i < frame.dlc; ++i) EXPECT_EQ(frame.data[i], 0);
} 