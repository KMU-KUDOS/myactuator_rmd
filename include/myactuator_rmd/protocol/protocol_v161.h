#ifndef V161_MOTOR_CONTROL__PROTOCOL_V161_HPP
#define V161_MOTOR_CONTROL__PROTOCOL_V161_HPP

#include <cmath>
#include <cstdint>

namespace v161_motor_control::protocol {

// V1.61 Control Command List
constexpr uint8_t CMD_READ_PID = 0x30;
constexpr uint8_t CMD_WRITE_PID_RAM = 0x31;
constexpr uint8_t CMD_WRITE_PID_ROM = 0x32;
constexpr uint8_t CMD_READ_ACCEL = 0x33;
constexpr uint8_t CMD_WRITE_ACCEL_RAM = 0x34;
constexpr uint8_t CMD_READ_ENCODER = 0x90;
constexpr uint8_t CMD_WRITE_ENCODER_OFFSET = 0x91;
constexpr uint8_t CMD_WRITE_POS_AS_ZERO_ROM = 0x19;
constexpr uint8_t CMD_READ_MULTI_TURN_ANGLE = 0x92;
constexpr uint8_t CMD_READ_SINGLE_CIRCLE_ANGLE = 0x94;
constexpr uint8_t CMD_READ_STATUS_1 = 0x9A;
constexpr uint8_t CMD_CLEAR_ERROR = 0x9B;
constexpr uint8_t CMD_READ_STATUS_2 = 0x9C;
constexpr uint8_t CMD_READ_STATUS_3 = 0x9D;
constexpr uint8_t CMD_MOTOR_OFF = 0x80;
constexpr uint8_t CMD_MOTOR_STOP = 0x81;
constexpr uint8_t CMD_MOTOR_RUN = 0x88;
constexpr uint8_t CMD_TORQUE_CONTROL = 0xA1;
constexpr uint8_t CMD_SPEED_CONTROL = 0xA2;
constexpr uint8_t CMD_POSITION_CONTROL_1 = 0xA3;
constexpr uint8_t CMD_POSITION_CONTROL_2 = 0xA4;
constexpr uint8_t CMD_POSITION_CONTROL_3 = 0xA5;
constexpr uint8_t CMD_POSITION_CONTROL_4 = 0xA6;

inline uint32_t getV161RequestId(uint8_t motor_id) {
  // motor id is in the range 1 ~ 32
  if (motor_id < 1 || motor_id > 32) {
    return 0;
  }
  return 0x140 + static_cast<uint32_t>(motor_id);
}

inline uint32_t getV161ResponseId(uint8_t motor_id) {
  // motor id is in the range 1 ~ 32
  if (motor_id < 1 || motor_id > 32) {
    return 0;
  }
  return 0x140 + static_cast<uint32_t>(motor_id);
}

constexpr uint32_t MULTI_MOTOR_TORQUE_REQ_ID = 0x280;

}  // namespace v161_motor_control::protocol

#endif  // V161_MOTOR_CONTROL__PROTOCOL_V161_HPP
