#ifndef V161_MOTOR_CONTROL__MOTOR_V161_HPP
#define V161_MOTOR_CONTROL__MOTOR_V161_HPP

#include <cstdint>
#include <memory> // for std::shared_ptr or reference

#include "can_interface.hpp"
#include "types_v161.hpp"

namespace v161_motor_control {

class MotorV161 {
public:
  /**
   * @brief : Constructor
   * @param can_interface : Shared CAN Interface Objects
   * @param motor_id : Motor ID to control (1-32)
   */
  MotorV161(std::shared_ptr<CanInterface> can_interface, uint8_t motor_id);

  // --- Read Method
  /**
   * @brief (0x30): Read to Parameter
   * @return
   */
  types::PidDataV161 readPid();

  /**
   * @brief (0x33): Read to Accel
   * @return
   */
  types::AccelDataV161 readAcceleration();

  /**
   * @brief (0x90): Read to Encoder data
   * @return
   */
  types::EncoderDataV161 readEncoder();

  /**
   * @brief (0x92): Read to MultiTurnAngle
   * @return
   */
  types::MultiTurnAngleV161 readMultiTurnAngle();

  /**
   * @brief (0x94): Read to SingleCircleAngle
   * @return
   */
  types::SingleCircleAngleV161 readSingleCircleAngle();

  /**
   * @brief (0x9A): Read to Motor Status_1
   * @return
   */
  types::Status1DataV161 readStatus1();

  /**
   * @brief (0x9C): Read to Motor Status_2
   * @return
   */
  types::Status2DataV161 readStatus2();

  /**
   * @brief (0x9D): Read to Motor Status_3
   * @return
   */
  types::Status3DataV161 readStatus3();

  // --- Write/Action Method ---
  /**
   * @brief (0x31): Write the PID parameters to RAM, which will be initialized
   * on power off
   * @param pid_data: PID values to set
   * @return true on success, false on failure
   */
  bool writePidToRam(const types::PidDataV161 &pid_data);

  /**
   * @brief (0x32): Write PID parameters to ROM, which are retained after power
   * off (Note: Chip lifetime impact)
   * @param pid_data: PID values to set
   * @return True on success, false on failure
   */
  bool writePidToRom(const types::PidDataV161 &pid_data);

  /**
   * @brief (0x34): Write the acceleration value to RAM, which is reset on power
   * off
   * @param accel_data: Acceleration value to set
   * @return True on success, False on failure
   */
  bool writeAccelerationToRam(const types::AccelDataV161 &accel_data);

  /**
   * @brief (0x91): Set the encoder zero offset value
   * @param offset: Offset value to set (0 ~ 16383)
   * @param written_offset_out: [Output] Offset value written to the actual
   * motor
   * @return True on success, False on failure
   */
  bool writeEncoderOffset(uint16_t offset, uint16_t &written_offset_out);

  /**
   * @brief (0x19): Write the current motor position to zero in ROM (Caution:
   * Chip lifetime impact, reboot required)
   * @param written_offset_out: [Output] Recorded zero offset value
   * @return True on success, False on failure
   */
  bool writePositionAsZero(uint16_t &written_offset_out);

  /**
   * @brief (0x9B): Reset motor error flags (possible after error condition is
   * cleared)
   * @param status_out [Output] Motor Status 1 Information after Error Clear
   * 2return True on success, False on failure
   */
  bool clearErrorFlag(types::Status1DataV161 &status_out);

  // --- Motor State Control Method ---
  /**
   * @brief (0x80): Turn off the motor. All control states are reset.
   * @return true on success, false on failure
   */
  bool motorOff();

  /**
   * @brief (0x81): Stop the motor. The current control state is maintained.
   */
  bool motorStop();

  /**
   * @brief (0x88): Resume motor control from the Stop state.
   * @return ture on success, false on failure
   */
  bool motorRun();

  // --- Closed-Loop Control Method ---
  /**
   * @brief (0xA1): Start torque closed loop control.
   * @param torque_setpoint: Target torque current value (-2000 to 2000)
   * @param feedback_out [Output]: Motor feedback (Status 2 format)
   * @return True if command succeeds and feedback is received, false if command
   * fails
   */
  bool setTorqueControl(int16_t torque_setpoint,
                        types::Status2DataV161 &feedback_out);

  /**
   * @brief (0xA2): Start speed closed loop control.
   * @param speed_setpoint: Target speed value (0.01 dps/LSB)
   * @param feedback_out [Output]: Motor feedback (Status 2 format)
   * @return True if command succeeds and feedback is received, false if command
   * fails
   */
  bool setSpeedControl(int32_t speed_setpoint,
                       types::Status2DataV161 &feedback_out);

  /**
   * @brief (0xA3): Start position closed loop control 1 (multi-turn)
   * @param angle_setpoint: Target angle value (0.01 degree/LSB)
   * @param feedback_out [Output]: Motor feedback (Status 2 format)
   * @return True if command succeeds and feedback is received, false if command
   * fails
   */
  bool setPositionControl1(int32_t angle_setpoint,
                           types::Status2DataV161 &feedback_out);

  /**
   * @brief (0xA4): Start position closed loop control 2 (multi-turn, speed
   * limit)
   * @param angle_setpoint: Target angle value (0.01 degree/LSB)
   * @param max_speed: Maximum speed limit (1 dps/LSB)
   * @param feedback_out [Output]: Motor feedback (Status 2 format)
   * @return True if command succeeds and feedback is received, false if command
   * fails
   */
  bool setPositionControl2(int32_t angle_setpoint, uint16_t max_speed,
                           types::Status2DataV161 &feedback_out);

  /**
   * @brief (0xA5): Start position closed loop control 3 (single rotation,
   * orientation).
   * @param angle_setpoint: target angle value (0-35999, 0.01 degree/LSB)
   * @param direction: Rotation direction
   * feedback_out [Output]: Motor feedback (Status 2 format)
   * @return True if command succeeds and feedback is received, false if command
   * fails
   */
  bool setPositionControl3(uint16_t angle_setpoint,
                           types::SpinDirection direction,
                           types::Status2DataV161 &feedback_out);

  /**
   * @brief (0xA6): Start position closed loop control 4 (single turn, heading,
   * speed limit).
   * @param angle_setpoint target angle value (0-35999, 0.01 degree/LSB)
   * @param direction: Rotation direction
   * @param max_speed: Maximum speed limit (1 dps/LSB)
   * @param feedback_out [Output]: Motor feedback (Status 2 format)
   * @return True if command succeeds and feedback is received, false if command
   * fails
   */
  bool setPositionControl4(uint16_t angle_setpoint,
                           types::SpinDirection direction, uint16_t max_speed,
                           types::Status2DataV161 &feedback_out);

private:
  std::shared_ptr<CanInterface> can_interface_;

  uint8_t motor_id_;
  uint32_t request_id_;
  uint32_t response_id_;

  /**
   * @brief
   * @param
   * @param
   * @param
   * @param
   * @return
   */
  bool sendCommandAndGetResponse(const std::array<uint8_t, 8> &command_data,
                                 uint8_t expected_response_cmd_code,
                                 std::array<uint8_t, 8> &response_data_out,
                                 int retry_count = 0);
};

} // namespace v161_motor_control

#endif // V161_MOTOR_CONTROL__MOTOR_V161_HPP
