#include "myactuator_rmd/protocol/motor_v161.h"

#include <chrono>    // for milliseconds
#include <iostream>  // for debugging/error messages
#include <thread>    // for sleep_for

#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "myactuator_rmd/protocol/packing_v161.h"
#include "myactuator_rmd/protocol/parsing_v161.h"
#include "myactuator_rmd/protocol/protocol_v161.h"

namespace v161_motor_control {

MotorV161::MotorV161(std::shared_ptr<CanInterface> can_interface,
                     std::shared_ptr<MotorRegistry> motor_registry,
                     uint8_t motor_id)
    : can_interface_(can_interface), 
      motor_registry_(motor_registry),
      motor_id_(motor_id) {
  if (!can_interface_) {
    throw std::invalid_argument("CAN interface pointer is null");
  }
  
  if (!motor_registry_) {
    throw std::invalid_argument("Motor registry pointer is null");
  }

  if (motor_id_ < 1 || motor_id_ > 32) {
    throw std::invalid_argument("Invalid motor ID");
  }

  // Precomputing CAN ID
  request_id_ = protocol::getV161RequestId(motor_id_);
  response_id_ = protocol::getV161ResponseId(motor_id_);

  // Register the motor ID with the registry
  auto add_status = motor_registry_->addMotorId(motor_id_);
  if (add_status.ok() || absl::IsAlreadyExists(add_status)) {
    // 정상 등록 또는 이미 등록된 경우 모두 필터 업데이트 수행
    // Update the filters on the CAN interface
    auto filter_ids = motor_registry_->getFilterIds();
    auto status = can_interface_->setReceiveFilters(filter_ids);
    if (!status.ok()) {
      std::cerr << "Warning: Failed to set receive filters: " << status.message() << '\n';
    }
  } else {
    std::cerr << "Failed to register motor ID " << static_cast<int>(motor_id_) 
              << " with the motor registry: " << add_status.message() << '\n';
  }
  
  // MotorConfigurator 인스턴스 생성
  configurator_ = std::make_shared<MotorConfigurator>(can_interface_, motor_id_);
  
  // MotorActuator 인스턴스 생성
  actuator_ = std::make_unique<MotorActuator>(can_interface_, motor_id_);
  
  // MotorStatusQuerier 인스턴스 생성
  status_querier_ = std::make_unique<MotorStatusQuerier>(can_interface_, motor_id_);
}

// --- Read Method Implementation ---
absl::StatusOr<types::PidDataV161> MotorV161::readPid() {
  return status_querier_->readPidData();
}

absl::StatusOr<types::AccelDataV161> MotorV161::readAcceleration() {
  return status_querier_->readAccelerationData();
}

absl::StatusOr<types::EncoderDataV161> MotorV161::readEncoder() {
  return status_querier_->readEncoderData();
}

absl::StatusOr<types::MultiTurnAngleV161> MotorV161::readMultiTurnAngle() {
  return status_querier_->readMultiTurnAngle();
}

absl::StatusOr<types::SingleCircleAngleV161> MotorV161::readSingleCircleAngle() {
  return status_querier_->readSingleCircleAngle();
}

absl::StatusOr<types::Status1DataV161> MotorV161::readStatus1() {
  return status_querier_->readMotorStatus1();
}

absl::StatusOr<types::Status2DataV161> MotorV161::readStatus2() {
  return status_querier_->readMotorStatus2();
}

absl::StatusOr<types::Status3DataV161> MotorV161::readStatus3() {
  return status_querier_->readMotorStatus3();
}

// --- Write/Action Method Implementation ---
absl::StatusOr<types::Status1DataV161> MotorV161::clearErrorFlag() {
  // 기존 status_out을 직접 반환하는 대신, StatusOr를 반환
  
  // 현재 상태 확인
  auto status_or = status_querier_->readMotorStatus1();
  if (!status_or.ok()) {
    return status_or;
  }
  
  if (status_or.value().error_state_raw != 0) {
    // 오류가 있는 경우 resetMotorError를 호출
    auto reset_status = actuator_->resetMotorError();
    if (!reset_status.ok()) {
      return reset_status;
      }
    
    // 오류 초기화 후 다시 상태 읽기
    return status_querier_->readMotorStatus1();
  }
  
  // 이미 오류가 없는 경우
  return status_or;
}

// --- Motor State Control Method Implementation ---
absl::Status MotorV161::motorOff() {
  return actuator_->powerOffMotor();
}

absl::Status MotorV161::motorStop() {
  return actuator_->stopMotor();
}

absl::Status MotorV161::motorRun() {
  return actuator_->runMotor();
}

// --- Closed-Loop Control Method Implementation ---
absl::StatusOr<types::Status2DataV161> MotorV161::setTorqueControl(int16_t torque_setpoint) {
  auto torque_data_or = actuator_->setTorque(torque_setpoint);
  if (!torque_data_or.ok()) {
    return torque_data_or.status();
  }
  
  // Convert TorqueResponseV161 to Status2DataV161
  types::Status2DataV161 feedback;
  feedback.torque_current = torque_data_or.value().torque_current;
  feedback.speed = torque_data_or.value().speed;
  feedback.encoder_position = torque_data_or.value().encoder_position;
  return feedback;
}

absl::StatusOr<types::Status2DataV161> MotorV161::setSpeedControl(int32_t speed_setpoint) {
  auto speed_data_or = actuator_->setSpeed(speed_setpoint);
  if (!speed_data_or.ok()) {
    return speed_data_or.status();
  }
  
  // Convert SpeedResponseV161 to Status2DataV161
  types::Status2DataV161 feedback;
  feedback.speed = speed_data_or.value().speed;
  feedback.torque_current = speed_data_or.value().torque_current;
  feedback.encoder_position = speed_data_or.value().encoder_position;
  return feedback;
}

absl::StatusOr<types::Status2DataV161> MotorV161::setPositionControl1(int32_t angle_setpoint) {
  // 모터가 이미 켜져 있으면 바로 제어할 수 있지만, 안전을 위해 먼저 run 상태로 만듭니다.
  auto status = motorRun();
  if (!status.ok()) {
    return status;
  }
  
  // 최대 속도를 0으로 설정하여 무제한으로 설정합니다
  uint16_t max_speed = 0;
  
  auto position_data_or = actuator_->setPositionAbsolute(angle_setpoint, max_speed);
  if (!position_data_or.ok()) {
    return position_data_or.status();
  }
  
  // Convert PositionResponseV161 to Status2DataV161
  types::Status2DataV161 feedback;
  feedback.encoder_position = position_data_or.value().encoder_position;
  feedback.speed = position_data_or.value().speed;
  feedback.torque_current = position_data_or.value().torque_current;
  return feedback;
}

absl::StatusOr<types::Status2DataV161> MotorV161::setPositionControl2(int32_t angle_setpoint,
                                        uint16_t max_speed) {
  // 모터가 이미 켜져 있으면 바로 제어할 수 있지만, 안전을 위해 먼저 run 상태로 만듭니다.
  auto status = motorRun();
  if (!status.ok()) {
    return status;
  }
  
  auto position_data_or = actuator_->setPositionAbsolute(angle_setpoint, max_speed);
  if (!position_data_or.ok()) {
    return position_data_or.status();
  }
  
  // Convert PositionResponseV161 to Status2DataV161
  types::Status2DataV161 feedback;
  feedback.encoder_position = position_data_or.value().encoder_position;
  feedback.speed = position_data_or.value().speed;
  feedback.torque_current = position_data_or.value().torque_current;
  return feedback;
}

absl::StatusOr<types::Status2DataV161> MotorV161::setPositionControl3(uint16_t angle_setpoint,
                                       types::SpinDirection direction) {
  // 모터가 이미 켜져 있으면 바로 제어할 수 있지만, 안전을 위해 먼저 run 상태로 만듭니다.
  auto status = motorRun();
  if (!status.ok()) {
    return status;
  }
  
  auto position_data_or = actuator_->setPositionControlWithDirection(angle_setpoint, direction);
  if (!position_data_or.ok()) {
    return position_data_or.status();
  }
  
  // Convert PositionResponseV161 to Status2DataV161
  types::Status2DataV161 feedback;
  feedback.encoder_position = position_data_or.value().encoder_position;
  feedback.speed = position_data_or.value().speed;
  feedback.torque_current = position_data_or.value().torque_current;
  return feedback;
}

absl::StatusOr<types::Status2DataV161> MotorV161::setPositionControl4(int32_t angle_setpoint,
                                       uint16_t max_time) {
  // 모터가 이미 켜져 있으면 바로 제어할 수 있지만, 안전을 위해 먼저 run 상태로 만듭니다.
  auto status = motorRun();
  if (!status.ok()) {
    return status;
  }
  
  // MotorActuator에는 setPositionAbsoluteWithTime 대신 setPositionControlWithTime이 있을 수 있습니다.
  // 현재 구현된 기능에 맞게 수정
  auto position_data_or = actuator_->setPositionAbsolute(angle_setpoint, 0);  // max_speed 0으로 설정, max_time 사용 안 함
  if (!position_data_or.ok()) {
    return position_data_or.status();
  }
  
  // Convert PositionResponseV161 to Status2DataV161
  types::Status2DataV161 feedback;
  feedback.encoder_position = position_data_or.value().encoder_position;
  feedback.speed = position_data_or.value().speed;
  feedback.torque_current = position_data_or.value().torque_current;
  return feedback;
}

}  // namespace v161_motor_control
