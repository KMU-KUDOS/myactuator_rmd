#ifndef V161_MOTOR_CONTROL__MOTOR_V161_HPP
#define V161_MOTOR_CONTROL__MOTOR_V161_HPP

#include <memory>  // for std::shared_ptr or reference

#include <cstdint>

#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/motor_registry.h"
#include "myactuator_rmd/protocol/types_v161.h"
#include "myactuator_rmd/protocol/motor_configurator.h"
#include "myactuator_rmd/protocol/motor_actuator.h"
#include "myactuator_rmd/protocol/motor_status_querier.h"

namespace v161_motor_control {

class MotorV161 {
 public:
  /**
   * @brief : Constructor
   * @param can_interface : Shared CAN Interface Objects
   * @param motor_registry : Shared Motor Registry for ID management
   * @param motor_id : Motor ID to control (1-32)
   */
  MotorV161(std::shared_ptr<CanInterface> can_interface, 
            std::shared_ptr<MotorRegistry> motor_registry,
            uint8_t motor_id);

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
   * @brief (0x9B): Reset motor error flags (possible after error condition is
   * cleared)
   * @param status_out [Output] Motor Status 1 Information after Error Clear
   * 2return True on success, False on failure
   */
  bool clearErrorFlag(types::Status1DataV161& status_out);

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
                        types::Status2DataV161& feedback_out);

  /**
   * @brief (0xA2): Start speed closed loop control.
   * @param speed_setpoint: Target speed value (0.01 dps/LSB)
   * @param feedback_out [Output]: Motor feedback (Status 2 format)
   * @return True if command succeeds and feedback is received, false if command
   * fails
   */
  bool setSpeedControl(int32_t speed_setpoint,
                       types::Status2DataV161& feedback_out);

  /**
   * @brief (0xA3): Start position closed loop control 1 (multi-turn)
   * @param angle_setpoint: Target angle value (0.01 degree/LSB)
   * @param feedback_out [Output]: Motor feedback (Status 2 format)
   * @return True if command succeeds and feedback is received, false if command
   * fails
   */
  bool setPositionControl1(int32_t angle_setpoint,
                           types::Status2DataV161& feedback_out);

  /**
   * @brief (0xA4): Start position closed loop control 2 (multi-turn, speed
   * limit)
   * @param angle_setpoint: Target angle value (0.01 degree/LSB)
   * @param max_speed: Maximum speed limit (1 dps/LSB)
   * @param feedback_out [Output]: Motor feedback (Status 2 format)
   * @return True if command succeeds and feedback is received, false if command
   * fails
   */
  bool setPositionControl2(int32_t angle_setpoint,
                           uint16_t max_speed,
                           types::Status2DataV161& feedback_out);

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
                           types::Status2DataV161& feedback_out);

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
                           types::SpinDirection direction,
                           uint16_t max_speed,
                           types::Status2DataV161& feedback_out);

  /**
   * @brief 모터 구성 작업을 위한 MotorConfigurator 인스턴스 얻기
   * @return MotorConfigurator 인스턴스에 대한 참조
   */
  MotorConfigurator& getConfigurator() { return *configurator_; }

  /**
   * @brief 모터 액추에이터 작업을 위한 MotorActuator 인스턴스 얻기
   * @return MotorActuator 인스턴스에 대한 참조
   */
  MotorActuator& getActuator() { return *actuator_; }

  /**
   * @brief 모터 상태 조회 작업을 위한 MotorStatusQuerier 인스턴스 얻기
   * @return MotorStatusQuerier 인스턴스에 대한 참조
   */
  MotorStatusQuerier& getStatusQuerier() { return *status_querier_; }

 private:
  std::shared_ptr<CanInterface> can_interface_;
  std::shared_ptr<MotorRegistry> motor_registry_;
  std::shared_ptr<MotorConfigurator> configurator_;
  std::unique_ptr<MotorActuator> actuator_;
  std::unique_ptr<MotorStatusQuerier> status_querier_;

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
  bool sendCommandAndGetResponse(const std::array<uint8_t, 8>& command_data,
                                 uint8_t expected_response_cmd_code,
                                 std::array<uint8_t, 8>& response_data_out,
                                 int retry_count = 0);
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_V161_HPP
