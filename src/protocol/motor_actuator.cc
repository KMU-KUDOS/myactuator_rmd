#include "myactuator_rmd/protocol/motor_actuator.h"
#include "myactuator_rmd/protocol/motor_v161.h"
#include "myactuator_rmd/protocol/packing_v161.h"
#include "myactuator_rmd/protocol/parsing_v161.h"
#include "myactuator_rmd/protocol/protocol_v161.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"

namespace v161_motor_control {

MotorActuator::MotorActuator(std::shared_ptr<CanInterface> can_interface, uint8_t motor_id)
    : can_interface_(can_interface), motor_id_(motor_id) {
  if (!can_interface_) {
    throw std::invalid_argument("CAN interface pointer is null");
  }

  if (motor_id_ < 1 || motor_id_ > 32) {
    throw std::invalid_argument("Invalid motor ID");
  }

  // Precomputing CAN ID
  request_id_ = protocol::getV161RequestId(motor_id_);
  response_id_ = protocol::getV161ResponseId(motor_id_);
}

MotorActuator::~MotorActuator() {
  // 소멸자 구현
}

absl::Status MotorActuator::sendCommandAndGetResponse(
    const std::array<uint8_t, 8>& command_data,
    uint8_t expected_response_cmd_code,
    std::array<uint8_t, 8>& response_data_out,
    int retry_count) {
  if (!can_interface_)
    return absl::InternalError("CAN interface is null");

  for (int i = 0; i <= retry_count; ++i) {
    auto send_status = can_interface_->sendFrame(request_id_, command_data);
    if (!send_status.ok()) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << ": Failed to send command 0x" << std::hex
                << static_cast<int>(command_data[0]) << std::dec 
                << ": " << send_status.message() << '\n';
      if (i == retry_count)
        return send_status;  // Last attempt failed, return the status
      std::this_thread::sleep_for(
          std::chrono::milliseconds(10));  // Retry after a short wait
      continue;
    }

    // Waiting for a response
    auto recv_status = can_interface_->receiveFrame(response_id_, response_data_out);
    if (recv_status.ok()) {
      // Successfully received a frame with the expected ID
      // Now check the command code within the data
      if (response_data_out[0] == expected_response_cmd_code) {
        return absl::OkStatus();  // Success: Correct ID and correct command code
      } else {
        // Received correct ID, but unexpected command code
        std::cerr << "Motor " << static_cast<int>(motor_id_)
                  << ": Received unexpected command code 0x" << std::hex
                  << static_cast<int>(response_data_out[0]) << " (expected 0x"
                  << static_cast<int>(expected_response_cmd_code) << ")"
                  << std::dec << '\n';
        // 현재 시도를 실패로 처리
      }
    } else {
      // receiveFrame returned an error: 타임아웃 발생 또는 예상치 않은 ID의 프레임 수신
      if (i < retry_count) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 재시도 전 대기
      } else {
        std::cerr << "Motor " << static_cast<int>(motor_id_)
                  << ": Receive timeout/failure for cmd 0x" << std::hex
                  << static_cast<int>(command_data[0]) << std::dec
                  << ". No more retries after " << retry_count << " attempts."
                  << " Error: " << recv_status.message()
                  << '\n';
        return recv_status;  // Return the receive error status
      }
    }
  }
  return absl::DeadlineExceededError(absl::StrCat(
      "Failed to get valid response after ", retry_count + 1, " attempts"));
}

absl::StatusOr<types::Status1DataV161> MotorActuator::resetMotorError() {
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createClearErrorFlagFrame();
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  std::array<uint8_t, 8> response_data;
  absl::Status status = sendCommandAndGetResponse(
          command_data_or.value(), protocol::CMD_CLEAR_ERROR, response_data, 1);
  
  if (!status.ok()) {
    return status;
  }
  
  absl::StatusOr<types::Status1DataV161> status_or = parsing::parseClearErrorFlagResponse(response_data);
  if (!status_or.ok()) {
    return status_or.status();
  }
  
  types::Status1DataV161 status_out = status_or.value();
  if (status_out.error_state_raw == 0) {
    std::cout << "Motor " << static_cast<int>(motor_id_)
              << ": Error flags cleared Successfully" << '\n';
  } else {
    std::cout << "Motor " << static_cast<int>(motor_id_)
              << ": Clear error command sent, but errors might still "
                 "persist (Raw Error: 0x"
              << std::hex << static_cast<int>(status_out.error_state_raw)
              << std::dec << "). Ensure error condition is resolved"
              << '\n';
  }
  return status_out;
}

absl::Status MotorActuator::powerOffMotor() {
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createMotorOffFrame();
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }

  std::array<uint8_t, 8> response_data;
  absl::Status status = sendCommandAndGetResponse(
          command_data_or.value(), protocol::CMD_MOTOR_OFF, response_data, 1);
  
  if (!status.ok()) {
    return status;
  }

  // Check if the response matches the command (echo verification)
  if (response_data != command_data_or.value()) {
    return absl::InternalError("Motor response does not match command (echo verification failed)");
  }
  
  return absl::OkStatus();
}

absl::Status MotorActuator::stopMotor() {
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createMotorStopFrame();
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }

  std::array<uint8_t, 8> response_data;
  absl::Status status = sendCommandAndGetResponse(
          command_data_or.value(), protocol::CMD_MOTOR_STOP, response_data, 1);
  
  if (!status.ok()) {
    return status;
  }

  // Check if the response matches the command (echo verification)
  if (response_data != command_data_or.value()) {
    return absl::InternalError("Motor response does not match command (echo verification failed)");
  }
  
  return absl::OkStatus();
}

absl::Status MotorActuator::runMotor() {
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createMotorRunFrame();
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }

  std::array<uint8_t, 8> response_data;
  absl::Status status = sendCommandAndGetResponse(
          command_data_or.value(), protocol::CMD_MOTOR_RUN, response_data, 1);
  
  if (!status.ok()) {
    return status;
  }

  // Check if the response matches the command (echo verification)
  if (response_data != command_data_or.value()) {
    return absl::InternalError("Motor response does not match command (echo verification failed)");
  }
  
  return absl::OkStatus();
}

absl::StatusOr<types::TorqueResponseV161> MotorActuator::setTorque(int16_t iqControl) {
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createTorqueControlFrame(iqControl);
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  std::array<uint8_t, 8> response_data;
  absl::Status status = sendCommandAndGetResponse(
      command_data_or.value(), protocol::CMD_TORQUE_CONTROL, response_data, 0);
      
  if (!status.ok()) {
    return status;
  }
  
  // 응답 데이터 파싱
  absl::StatusOr<types::Status2DataV161> status_data_or = 
      parsing::parseClosedLoopResponse(response_data, protocol::CMD_TORQUE_CONTROL);
  
  if (!status_data_or.ok()) {
    return status_data_or.status();
  }
  
  // Status2DataV161에서 TorqueResponseV161로 데이터 복사
  types::TorqueResponseV161 torque_data_out;
  torque_data_out.torque_current = status_data_or.value().torque_current;
  torque_data_out.speed = status_data_or.value().speed;
  torque_data_out.encoder_position = status_data_or.value().encoder_position;
  
  return torque_data_out;
}

absl::StatusOr<types::SpeedResponseV161> MotorActuator::setSpeed(int32_t speed) {
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createSpeedControlFrame(speed);
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  std::array<uint8_t, 8> response_data;
  absl::Status status = sendCommandAndGetResponse(
      command_data_or.value(), protocol::CMD_SPEED_CONTROL, response_data, 0);
      
  if (!status.ok()) {
    return status;
  }
  
  // 응답 데이터 파싱
  absl::StatusOr<types::Status2DataV161> status_data_or = 
      parsing::parseClosedLoopResponse(response_data, protocol::CMD_SPEED_CONTROL);
  
  if (!status_data_or.ok()) {
    return status_data_or.status();
  }
  
  // Status2DataV161에서 SpeedResponseV161로 데이터 복사
  types::SpeedResponseV161 speed_data_out;
  speed_data_out.speed = status_data_or.value().speed;
  speed_data_out.torque_current = status_data_or.value().torque_current;
  speed_data_out.encoder_position = status_data_or.value().encoder_position;
  
  return speed_data_out;
}

absl::StatusOr<types::PositionResponseV161> MotorActuator::setPositionAbsolute(
    int32_t position, uint16_t max_speed) {
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = 
      packing::createPositionControl2Frame(position, max_speed);
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  std::array<uint8_t, 8> response_data;
  absl::Status status = sendCommandAndGetResponse(
      command_data_or.value(), protocol::CMD_POSITION_CONTROL_2, response_data, 0);
      
  if (!status.ok()) {
    return status;
  }
  
  // 응답 데이터 파싱
  absl::StatusOr<types::Status2DataV161> status_data_or = 
      parsing::parseClosedLoopResponse(response_data, protocol::CMD_POSITION_CONTROL_2);
  
  if (!status_data_or.ok()) {
    return status_data_or.status();
  }
  
  // Status2DataV161에서 PositionResponseV161로 데이터 복사
  types::PositionResponseV161 position_data_out;
  position_data_out.encoder_position = status_data_or.value().encoder_position;
  position_data_out.speed = status_data_or.value().speed;
  position_data_out.torque_current = status_data_or.value().torque_current;
  
  return position_data_out;
}

absl::StatusOr<types::PositionResponseV161> MotorActuator::setAbsolutePosition(
    uint16_t angle, uint16_t max_speed) {
  if (angle > 35999) {
    return absl::InvalidArgumentError(
        absl::StrCat("Angle for PositionControl4 should be 0 ~ 35999, got ", angle));
  }
  
  // 기본적으로 시계 방향을 사용
  auto direction = types::SpinDirection::CLOCKWISE;
  
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = 
      packing::createPositionControl4Frame(angle, direction, max_speed);
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  std::array<uint8_t, 8> response_data;
  absl::Status status = sendCommandAndGetResponse(
      command_data_or.value(), protocol::CMD_POSITION_CONTROL_4, response_data, 0);
      
  if (!status.ok()) {
    return status;
  }
  
  // 응답 데이터 파싱
  absl::StatusOr<types::Status2DataV161> status_data_or = 
      parsing::parseClosedLoopResponse(response_data, protocol::CMD_POSITION_CONTROL_4);
  
  if (!status_data_or.ok()) {
    return status_data_or.status();
  }
  
  // Status2DataV161에서 PositionResponseV161로 데이터 복사
  types::PositionResponseV161 position_data_out;
  position_data_out.encoder_position = status_data_or.value().encoder_position;
  position_data_out.speed = status_data_or.value().speed;
  position_data_out.torque_current = status_data_or.value().torque_current;
  
  return position_data_out;
}

absl::StatusOr<types::PositionResponseV161> MotorActuator::setPositionRelative(
    int32_t position_increment, uint16_t max_speed) {
  auto direction = types::SpinDirection::CLOCKWISE; // 기본값으로 시계 방향 사용
  auto angle_setpoint = static_cast<uint16_t>(std::abs(position_increment) % 36000);
  
  if (position_increment < 0) {
    direction = types::SpinDirection::COUNTER_CLOCKWISE;
  }
  
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = 
      packing::createPositionControl3Frame(angle_setpoint, direction);
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  std::array<uint8_t, 8> response_data;
  absl::Status status = sendCommandAndGetResponse(
      command_data_or.value(), protocol::CMD_POSITION_CONTROL_3, response_data, 0);
      
  if (!status.ok()) {
    return status;
  }
  
  // 응답 데이터 파싱
  absl::StatusOr<types::Status2DataV161> status_data_or = 
      parsing::parseClosedLoopResponse(response_data, protocol::CMD_POSITION_CONTROL_3);
  
  if (!status_data_or.ok()) {
    return status_data_or.status();
  }
  
  // Status2DataV161에서 PositionResponseV161로 데이터 복사
  types::PositionResponseV161 position_data_out;
  position_data_out.encoder_position = status_data_or.value().encoder_position;
  position_data_out.speed = status_data_or.value().speed;
  position_data_out.torque_current = status_data_or.value().torque_current;
  
  return position_data_out;
}

}  // namespace v161_motor_control 