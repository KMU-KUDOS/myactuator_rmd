#include "gtest/gtest.h"
#include "myactuator_rmd/protocol/command_type.hpp"

namespace myactuator_rmd {
namespace test {

TEST(CommandTypeTest, VerifyCommandValuesForV161Protocol) {
  // PID and acceleration related commands (0x30-0x34)
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::READ_PID_PARAMETERS), 0x30);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::WRITE_PID_PARAMETERS_TO_RAM), 0x31);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::WRITE_PID_PARAMETERS_TO_ROM), 0x32);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::READ_ACCELERATION), 0x33);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::WRITE_ACCELERATION_TO_RAM), 0x34);
  
  // Zero position setting (0x19)
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::WRITE_CURRENT_POSITION_TO_ROM_AS_ZERO), 0x19);
  
  // Motor control commands (0x80-0x88)
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::MOTOR_OFF), 0x80);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::MOTOR_STOP), 0x81);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::MOTOR_RUNNING), 0x88);
  
  // Encoder and motor status commands (0x90-0x9D)
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::READ_ENCODER_DATA), 0x90);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::WRITE_ENCODER_OFFSET), 0x91);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::READ_MULTI_TURNS_ANGLE), 0x92);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::READ_SINGLE_CIRCLE_ANGLE), 0x94);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::READ_MOTOR_STATUS_1), 0x9A);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::CLEAR_MOTOR_ERROR_FLAG), 0x9B);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::READ_MOTOR_STATUS_2), 0x9C);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::READ_MOTOR_STATUS_3), 0x9D);
  
  // Closed-loop control commands (0xA1-0xA6)
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::TORQUE_CLOSED_LOOP_CONTROL), 0xA1);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::SPEED_CLOSED_LOOP_CONTROL), 0xA2);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::POSITION_CLOSED_LOOP_CONTROL_1), 0xA3);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::POSITION_CLOSED_LOOP_CONTROL_2), 0xA4);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::POSITION_CLOSED_LOOP_CONTROL_3), 0xA5);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::POSITION_CLOSED_LOOP_CONTROL_4), 0xA6);
}

TEST(CommandTypeTest, SymmetricComparisonOperators) {
  // Test symmetric equality operator
  EXPECT_TRUE(CommandType::READ_PID_PARAMETERS == 0x30);
  EXPECT_TRUE(0x30 == CommandType::READ_PID_PARAMETERS);
  
  // Test symmetric inequality operator
  EXPECT_FALSE(CommandType::READ_PID_PARAMETERS != 0x30);
  EXPECT_FALSE(0x30 != CommandType::READ_PID_PARAMETERS);
  EXPECT_TRUE(CommandType::READ_PID_PARAMETERS != 0x31);
  EXPECT_TRUE(0x31 != CommandType::READ_PID_PARAMETERS);
}

TEST(CommandTypeTest, V161ProtocolSpecificCommands) {
  // Test commands that are specific to V1.61 protocol
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::WRITE_CURRENT_POSITION_TO_ROM_AS_ZERO), 0x19);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::MOTOR_RUNNING), 0x88);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::WRITE_ENCODER_OFFSET), 0x91);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::CLEAR_MOTOR_ERROR_FLAG), 0x9B);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::POSITION_CLOSED_LOOP_CONTROL_1), 0xA3);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::POSITION_CLOSED_LOOP_CONTROL_3), 0xA5);
  EXPECT_EQ(static_cast<std::uint8_t>(CommandType::POSITION_CLOSED_LOOP_CONTROL_4), 0xA6);
}

}  // namespace test
}  // namespace myactuator_rmd
