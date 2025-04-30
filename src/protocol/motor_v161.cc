#include "myactuator_rmd/protocol/motor_v161.h"

#include <chrono>    // for milliseconds
#include <iostream>  // for debugging/error messages
#include <thread>    // for sleep_for

#include <cstdint>

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
  if (motor_registry_->addMotorId(motor_id_)) {
    // Update the filters on the CAN interface
    auto filter_ids = motor_registry_->getFilterIds();
    can_interface_->setReceiveFilters(filter_ids);
  } else {
    std::cerr << "Failed to register motor ID " << static_cast<int>(motor_id_) 
              << " with the motor registry" << '\n';
  }
  
  // MotorConfigurator 인스턴스 생성
  configurator_ = std::make_shared<MotorConfigurator>(can_interface_, motor_id_);
  
  // MotorActuator 인스턴스 생성
  actuator_ = std::make_unique<MotorActuator>(can_interface_, motor_id_);
  
  // MotorStatusQuerier 인스턴스 생성
  status_querier_ = std::make_unique<MotorStatusQuerier>(can_interface_, motor_id_);
}

bool MotorV161::sendCommandAndGetResponse(
    const std::array<uint8_t, 8>& command_data,
    uint8_t expected_response_cmd_code,
    std::array<uint8_t, 8>& response_data_out,
    int retry_count) {
  if (!can_interface_)
    return false;

  for (int i = 0; i <= retry_count; ++i) {
    if (!can_interface_->sendFrame(request_id_, command_data)) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << ": Failed to send command 0x" << std::hex
                << static_cast<int>(command_data[0]) << std::dec << '\n';
      if (i == retry_count)
        return false;  // Last attempt failed
      std::this_thread::sleep_for(
          std::chrono::milliseconds(10));  // Retry after a short wait
      continue;
    }

    // Waiting for a response
    if (can_interface_->receiveFrame(response_id_, response_data_out)) {
      // Successfully received a frame with the expected ID
      // Now check the command code within the data
      if (response_data_out[0] == expected_response_cmd_code) {
        return true;  // Success: Correct ID and correct command code
      } else {
        // Received correct ID, but unexpected command code
        std::cerr << "Motor " << static_cast<int>(motor_id_)
                  << ": Received unexpected command code 0x" << std::hex
                  << static_cast<int>(response_data_out[0]) << " (expected 0x"
                  << static_cast<int>(expected_response_cmd_code) << ")"
                  << std::dec << '\n';
        // Decide if this constitutes a failure or if we should wait for another
        // frame For now, treat as failure for this attempt return false; // Or
        // continue the loop if retries are left? Let's treat as failure for
        // now.
      }
    } else {
      // receiveFrame returned false: Timeout occurred or received frame with
      // unexpected ID (handled inside receiveFrame)
      if (i < retry_count) {
        // Don't print timeout message here, it's potentially handled in
        // receiveFrame or could be misleading std::cerr << "Motor " <<
        // static_cast<int>(motor_id_)
        //           << ": Receive timeout/failure for cmd 0x" << std::hex
        //           << static_cast<int>(command_data[0]) << std::dec
        //           << ". Retrying (" << i + 1 << "/" << retry_count << ")..."
        //           << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(
            50));  // Wait a little longer before retrying
      } else {
        std::cerr << "Motor " << static_cast<int>(motor_id_)
                  << ": Receive timeout/failure for cmd 0x" << std::hex
                  << static_cast<int>(command_data[0]) << std::dec
                  << ". No more retries after " << retry_count << " attempts."
                  << '\n';
      }
    }
  }
  return false;  // All retries failed
}

// --- Read Method Implementation ---
types::PidDataV161 MotorV161::readPid() {
  types::PidDataV161 pid_data = {};
  if (status_querier_->readPidData(pid_data)) {
    return pid_data;
  }
  return {};  // Returning defaults on failure
}

types::AccelDataV161 MotorV161::readAcceleration() {
  types::AccelDataV161 accel_data = {};
  if (status_querier_->readAccelerationData(accel_data)) {
    return accel_data;
  }
  return {};
}

types::EncoderDataV161 MotorV161::readEncoder() {
  types::EncoderDataV161 encoder_data = {};
  if (status_querier_->readEncoderData(encoder_data)) {
    return encoder_data;
  }
  return {};
}

types::MultiTurnAngleV161 MotorV161::readMultiTurnAngle() {
  types::MultiTurnAngleV161 angle_data = {};
  if (status_querier_->readMultiTurnAngle(angle_data)) {
    return angle_data;
  }
  return {};
}

types::SingleCircleAngleV161 MotorV161::readSingleCircleAngle() {
  auto command_data = packing::createReadSingleCircleAngleFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data,
                                protocol::CMD_READ_SINGLE_CIRCLE_ANGLE,
                                response_data,
                                1)) {
    try {
      return parsing::parseReadSingleCircleAngleResponse(response_data);
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing SingleCircleAngle response: " << e.what()
                << '\n';
    }
  }
  return {};
}

types::Status1DataV161 MotorV161::readStatus1() {
  types::Status1DataV161 status_data = {};
  if (status_querier_->readMotorStatus1(status_data)) {
    return status_data;
  }
  return {};
}

types::Status2DataV161 MotorV161::readStatus2() {
  types::Status2DataV161 status_data = {};
  if (status_querier_->readMotorStatus2(status_data)) {
    return status_data;
  }
  return {};
}

types::Status3DataV161 MotorV161::readStatus3() {
  types::Status3DataV161 status_data = {};
  if (status_querier_->readMotorStatus3(status_data)) {
    return status_data;
  }
  return {};
}

// --- Write/Action Method Implementation ---
// 설정 관련 메서드는 MotorConfigurator로 이동됨

bool MotorV161::clearErrorFlag(types::Status1DataV161& status_out) {
  // 일단 status_out을 초기화
  status_out = {};
  
  // readMotorStatus1을 통해 직접 status_out을 채움
  bool result = status_querier_->readMotorStatus1(status_out);
  if (!result || status_out.error_state_raw != 0) {
    // 오류가 있는 경우 resetMotorError를 호출
    if (actuator_->resetMotorError()) {
      // 오류 초기화 후 다시 상태 읽기
      if (status_querier_->readMotorStatus1(status_out)) {
        return true;
      }
    }
    return false;
  }
  
  // 이미 오류가 없는 경우
  return true;
}

// --- Motor State Control Method Implementation ---
bool MotorV161::motorOff() {
  return actuator_->powerOffMotor();
}

bool MotorV161::motorStop() {
  return actuator_->stopMotor();
}

bool MotorV161::motorRun() {
  return actuator_->runMotor();
}

// --- Closed-Loop Control Method Implementation ---
bool MotorV161::setTorqueControl(int16_t torque_setpoint,
                                 types::Status2DataV161& feedback_out) {
  types::TorqueResponseV161 torque_data = {};
  if (actuator_->setTorque(torque_setpoint, torque_data)) {
    // Convert TorqueResponseV161 to Status2DataV161
    feedback_out.torque_current = torque_data.torque_current;
    feedback_out.speed = torque_data.speed;
    feedback_out.encoder_position = torque_data.encoder_position;
    return true;
  }
  return false;
}

bool MotorV161::setSpeedControl(int32_t speed_setpoint,
                                types::Status2DataV161& feedback_out) {
  types::SpeedResponseV161 speed_data = {};
  if (actuator_->setSpeed(speed_setpoint, speed_data)) {
    // Convert SpeedResponseV161 to Status2DataV161
    feedback_out.speed = speed_data.speed;
    feedback_out.torque_current = speed_data.torque_current;
    feedback_out.encoder_position = speed_data.encoder_position;
    return true;
  }
  return false;
}

bool MotorV161::setPositionControl1(int32_t angle_setpoint,
                                    types::Status2DataV161& feedback_out) {
  // 모터가 이미 켜져 있으면 바로 제어할 수 있지만, 안전을 위해 먼저 run 상태로 만듭니다.
  if (!motorRun()) {
    return false;
  }
  
  // 최대 속도를 0으로 설정하여 무제한으로 설정합니다
  uint16_t max_speed = 0;
  
  types::PositionResponseV161 position_data = {};
  if (actuator_->setPositionAbsolute(angle_setpoint, max_speed, position_data)) {
    // Convert PositionResponseV161 to Status2DataV161
    feedback_out.encoder_position = position_data.encoder_position;
    feedback_out.speed = position_data.speed;
    feedback_out.torque_current = position_data.torque_current;
    return true;
  }
  return false;
}

bool MotorV161::setPositionControl2(int32_t angle_setpoint,
                                    uint16_t max_speed,
                                    types::Status2DataV161& feedback_out) {
  // 모터가 이미 켜져 있으면 바로 제어할 수 있지만, 안전을 위해 먼저 run 상태로 만듭니다.
  if (!motorRun()) {
    return false;
  }
  
  types::PositionResponseV161 position_data = {};
  if (actuator_->setPositionAbsolute(angle_setpoint, max_speed, position_data)) {
    // Convert PositionResponseV161 to Status2DataV161
    feedback_out.encoder_position = position_data.encoder_position;
    feedback_out.speed = position_data.speed;
    feedback_out.torque_current = position_data.torque_current;
    return true;
  }
  return false;
}

bool MotorV161::setPositionControl3(uint16_t angle_setpoint,
                                    types::SpinDirection direction,
                                    types::Status2DataV161& feedback_out) {
  // 이 메서드는 0-360도 범위 내 절대 위치 제어를 위한 것입니다.
  // 현재 MotorActuator에는 방향(direction)을 직접 받는 메서드가 없습니다.
  // setAbsolutePosition은 내부적으로 방향 설정을 고정하여 사용합니다.
  
  // 모터가 이미 켜져 있으면 바로 제어할 수 있지만, 안전을 위해 먼저 run 상태로 만듭니다.
  if (!motorRun()) {
    return false;
  }
  
  // 최대 속도를 0으로 설정하여 무제한으로 설정합니다
  uint16_t max_speed = 0;
  
  // 방향을 고려한 제어 메서드는 없어서 기존 구현을 유지합니다
  if (angle_setpoint > 35999) {
    std::cerr
        << "Warning: angle_setpoint for PositionControl3 should be 0 ~ 35999"
        << '\n';
    // angle_setpoint = angle_setpoint % 36000; // Or handle as error
  }
  auto command_data =
      packing::createPositionControl3Frame(angle_setpoint, direction);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_POSITION_CONTROL_3, response_data, 0)) {
    try {
      feedback_out = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_POSITION_CONTROL_3);

      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PositionControl3 response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorV161::setPositionControl4(uint16_t angle_setpoint,
                                    types::SpinDirection direction,
                                    uint16_t max_speed,
                                    types::Status2DataV161& feedback_out) {
  // 이 메서드는 0-360도 범위 내 절대 위치 제어를 위한 것입니다.
  // 현재 MotorActuator에는 방향(direction)을 직접 받는 메서드가 없습니다.
  
  // 모터가 이미 켜져 있으면 바로 제어할 수 있지만, 안전을 위해 먼저 run 상태로 만듭니다.
  if (!motorRun()) {
    return false;
  }
  
  // 방향을 고려한 제어 메서드는 없어서 기존 구현을 유지합니다
  if (angle_setpoint > 35999) {
    std::cerr
        << "Warning: angle_setpoint for PositionControl4  should be 0 ~ 35999"
        << '\n';
  }
  auto command_data = packing::createPositionControl4Frame(
      angle_setpoint, direction, max_speed);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_POSITION_CONTROL_4, response_data, 0)) {
    try {
      feedback_out = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_POSITION_CONTROL_4);

      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PositionControl4 response: " << e.what()
                << '\n';
    }
  }
  return false;
}

}  // namespace v161_motor_control
