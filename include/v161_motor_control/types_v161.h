#ifndef V161_MOTOR_CONTROL__TYPES_V161_HPP
#define V161_MOTOR_CONTROL__TYPES_V161_HPP

#include <cstdint>
#include <vector> // for ErrorState bits

namespace v161_motor_control::types {

// --- Read Command ---

// 0x30: Read PID data command
struct PidDataV161 {
  uint8_t anglePidKp = 0;
  uint8_t anglePidKi = 0;
  uint8_t speedPidKp = 0;
  uint8_t speedPidKi = 0;
  uint8_t iqPidKp = 0;
  uint8_t iqPidKi = 0;
};

// 0x33: Read acceleration data command
struct AccelDataV161 {
  int32_t acceleration = 0; // unit: 1dps/s
};

// 0x90: Read encoder data command
struct EncoderDataV161 {
  uint16_t position = 0;     // 0 ~ 16383, Current position (with offset)
  uint16_t raw_position = 0; // 0 ~ 16383, Raw location
  uint16_t offset = 0;       // 0 ~ 16383, Zero Offset
};

// 0x92: Read multi turns angle command
struct MultiTurnAngleV161 {
  int64_t angle = 0; // uint: 0.01 degree / LSB
};

// 0x94: Read single circle angle command
struct SingleCircleAngleV161 {
  uint16_t angle = 0; // unit: 0.01 degree / LSB (0 ~ 35999)
};

// 0x9A: Read motor status 1 & error flag command
struct Status1DataV161 {
  int8_t temperature = 0;      // unit: 1 degree C / LSB
  uint16_t voltage = 0;        // unit: 0.1 V / LSB
  uint8_t error_state_raw = 0; // Raw error state byte

  bool isVoltageLow() const { return (error_state_raw & 0x01); }
  bool isOverTemperature() const {
    return (error_state_raw & 0x08);
  } // Bit 3 for temperature
};

// 0x9C; Read motor status 2
struct Status2DataV161 {
  int8_t temperature = 0;        // unit: 1 degree C / LSB
  int16_t torque_current = 0;    // Range: -2048 ~ 2048 (maps to -33A ~ 33A)
  int16_t speed = 0;             // unit: 1 dps / LSB
  uint16_t encoder_position = 0; // 0 ~ 16383
};

// 0x9D: Read motor status 3
struct Status3DataV161 {
  int16_t current_A = 0; // unit: 1 A / 64 LSB
  int16_t current_B = 0; // unit: 1 A / 64 LSB
  int16_t current_C = 0; // unit: 1 A / 64 LSB

  float getCurrentA() const { return static_cast<float>(current_A) / 64.0f; }
  float getCurrentB() const { return static_cast<float>(current_B) / 64.0f; }
  float getCurrentC() const { return static_cast<float>(current_C) / 64.0f; }
};

// --- Write Command ---

// 0x91: Write encoder offset command
struct EncoderOffsetResponseDataV161 {
  uint16_t written_offset = 0; // 0 ~ 16383
};

// --- Control Command ---

enum class SpinDirection : uint8_t {
  CLOCKWISE = 0x00,
  COUNTER_CLOCKWISE = 0x01
};

} // namespace v161_motor_control::types

#endif // V161_MOTOR_CONTROL__TYPES_V161_HPP
