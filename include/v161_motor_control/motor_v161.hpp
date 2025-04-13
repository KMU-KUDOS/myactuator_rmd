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
   * @brief : Read to Parameter. (0x30)
   * @return
   */
  types::PidDataV161 readPid();

  /**
   * @brief : Read to Accel (0x33)
   * @return
   */
  types::AccelDataV161 readAcceleration();

  /**
   * @brief : Read to Encoder data (0x90)
   * @return
   */
  types::EncoderDataV161 readEncoder();

  /**
   * @brief : Read to MultiTurnAngle (0x92)
   * @return
   */
  types::MultiTurnAngleV161 readMultiTurnAngle();

  /**
   * @brief : Read to SingleCircleAngle (0x94)
   * @return
   */
  types::SingleCircleAngleV161 readSingleCircleAngle();

  /**
   * @brief : Read to Motor Status_1 (0x9A)
   * @return
   */
  types::Status1DataV161 readStatus1();

  /**
   * @brief : Read to Motor Status_2 (0x9C)
   * @return
   */
  types::Status2DataV161 readStatus2();

  /**
   * @brief : Read to Motor Status_3 (0x9D)
   * @return
   */
  types::Status3DataV161 readStatus3();

  // --- Write/Action Method ---
  /**
   * @brief 0x31: Write the PID parameters to RAM, which will be initialized on
   * power off
   * @param pid_data: PID values to set
   * @return true on success, false on failure
   */
  bool writePidToRam(const types::PidDataV161 &pid_data);

  /**
   * @brief 0x32: Write PID parameters to ROM, which are retained after power
   * off (Note: Chip lifetime impact)
   * @param pid_data: PID values to set
   * @return True on success, false on failure
   */
  bool writePidToRom(const types::PidDataV161 &pid_data);

  /**
   * @brief 0x34: Write the acceleration value to RAM, which is reset on power
   * off
   * @param accel_data: Acceleration value to set
   * @return True on success, False on failure
   */
  bool writeAccelerationToRam(const types::AccelDataV161 &accel_data);

  /**
   * @brief 0x91: Set the encoder zero offset value
   * @param offset: Offset value to set (0 ~ 16383)
   * @param written_offset_out: [Output] Offset value written to the actual
   * motor
   * @return True on success, False on failure
   */
  bool writeEncoderOffset(uint16_t offset, uint16_t &written_offset_out);

  /**
   * @brief 0x19: Write the current motor position to zero in ROM (Caution: Chip
   * lifetime impact, reboot required)
   * @param written_offset_out: [Output] Recorded zero offset value
   * @return True on success, False on failure
   */
  bool writePositionAsZero(uint16_t &written_offset_out);

  /**
   * @brief 0x9B: Reset motor error flags (possible after error condition is
   * cleared)
   * @param status_out [Output] Motor Status 1 Information after Error Clear
   * 2return True on success, False on failure
   */
  bool clearErrorFlag(types::Status1DataV161 &status_out);

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
