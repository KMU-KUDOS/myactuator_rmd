#include "myactuator_rmd/protocol/motor_actuator.h"
#include "myactuator_rmd/protocol/motor_v161.h"
#include "myactuator_rmd/protocol/packing_v161.h"
#include "myactuator_rmd/protocol/parsing_v161.h"
#include "myactuator_rmd/protocol/protocol_v161.h"

#include <chrono>
#include <iostream>
#include <thread>

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

bool MotorActuator::sendCommandAndGetResponse(
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
        // 현재 시도를 실패로 처리
      }
    } else {
      // receiveFrame returned false: 타임아웃 발생 또는 예상치 않은 ID의 프레임 수신
      if (i < retry_count) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 재시도 전 대기
      } else {
        std::cerr << "Motor " << static_cast<int>(motor_id_)
                  << ": Receive timeout/failure for cmd 0x" << std::hex
                  << static_cast<int>(command_data[0]) << std::dec
                  << ". No more retries after " << retry_count << " attempts."
                  << '\n';
      }
    }
  }
  return false;  // 모든 재시도 실패
}

bool MotorActuator::resetMotorError() {
  auto command_data = packing::createClearErrorFlagFrame();
  std::array<uint8_t, 8> response_data;
  types::Status1DataV161 status_out;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_CLEAR_ERROR, response_data, 1)) {
    try {
      status_out = parsing::parseClearErrorFlagResponse(response_data);

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
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing clearErrorFlag response: " << e.what() << '\n';
    }
  }
  return false;
}

bool MotorActuator::powerOffMotor() {
  auto command_data = packing::createMotorOffFrame();
  std::array<uint8_t, 8> response_data;

  // The response is sent to the Echo
  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_MOTOR_OFF, response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorActuator::stopMotor() {
  auto command_data = packing::createMotorStopFrame();
  std::array<uint8_t, 8> response_data;

  // The response is sent to the Echo
  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_MOTOR_STOP, response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorActuator::runMotor() {
  auto command_data = packing::createMotorRunFrame();
  std::array<uint8_t, 8> response_data;

  // The response is sent to the Echo
  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_MOTOR_RUN, response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorActuator::setTorque(int16_t iqControl, types::TorqueResponseV161& torque_data_out) {
  auto command_data = packing::createTorqueControlFrame(iqControl);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data,
                                protocol::CMD_TORQUE_CONTROL,
                                response_data,
                                0)) {  // Control commands run without retries
    try {
      // 임시로 Status2DataV161로 변환한 후 필요한 데이터만 추출
      auto status_data = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_TORQUE_CONTROL);
      
      // Status2DataV161에서 TorqueResponseV161로 데이터 복사
      torque_data_out.torque_current = status_data.torque_current;
      torque_data_out.speed = status_data.speed;
      torque_data_out.encoder_position = status_data.encoder_position;

      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing TorqueControl response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorActuator::setSpeed(int32_t speed, types::SpeedResponseV161& speed_data_out) {
  auto command_data = packing::createSpeedControlFrame(speed);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_SPEED_CONTROL, response_data, 0)) {
    try {
      // 임시로 Status2DataV161로 변환한 후 필요한 데이터만 추출
      auto status_data = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_SPEED_CONTROL);
      
      // Status2DataV161에서 SpeedResponseV161로 데이터 복사
      speed_data_out.speed = status_data.speed;
      speed_data_out.torque_current = status_data.torque_current;
      speed_data_out.encoder_position = status_data.encoder_position;

      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing SpeedControl response: " << e.what() << '\n';
    }
  }
  return false;
}

bool MotorActuator::setPositionAbsolute(int32_t position, uint16_t max_speed,
                                        types::PositionResponseV161& position_data_out) {
  auto command_data =
      packing::createPositionControl2Frame(position, max_speed);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_POSITION_CONTROL_2, response_data, 0)) {
    try {
      // 임시로 Status2DataV161로 변환한 후 필요한 데이터만 추출
      auto status_data = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_POSITION_CONTROL_2);
      
      // Status2DataV161에서 PositionResponseV161로 데이터 복사
      position_data_out.encoder_position = status_data.encoder_position;
      position_data_out.speed = status_data.speed;
      position_data_out.torque_current = status_data.torque_current;

      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PositionControl2 response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorActuator::setAbsolutePosition(uint16_t angle, uint16_t max_speed,
                                        types::PositionResponseV161& position_data_out) {
  if (angle > 35999) {
    std::cerr
        << "Warning: angle for PositionControl3 should be 0 ~ 35999"
        << '\n';
    // angle = angle % 36000; // 또는 오류로 처리
  }
  
  // 기본적으로 시계 방향을 사용
  auto direction = types::SpinDirection::CLOCKWISE;
  
  auto command_data =
      packing::createPositionControl4Frame(angle, direction, max_speed);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_POSITION_CONTROL_4, response_data, 0)) {
    try {
      // 임시로 Status2DataV161로 변환한 후 필요한 데이터만 추출
      auto status_data = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_POSITION_CONTROL_4);
      
      // Status2DataV161에서 PositionResponseV161로 데이터 복사
      position_data_out.encoder_position = status_data.encoder_position;
      position_data_out.speed = status_data.speed;
      position_data_out.torque_current = status_data.torque_current;

      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PositionControl4 response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorActuator::setPositionRelative(int32_t position_increment, uint16_t max_speed,
                                        types::PositionResponseV161& position_data_out) {
  auto direction = types::SpinDirection::CLOCKWISE; // 기본값으로 시계 방향 사용
  auto angle_setpoint = static_cast<uint16_t>(std::abs(position_increment) % 36000);
  
  if (position_increment < 0) {
    direction = types::SpinDirection::COUNTER_CLOCKWISE;
  }
  
  auto command_data = packing::createPositionControl3Frame(angle_setpoint, direction);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_POSITION_CONTROL_3, response_data, 0)) {
    try {
      // 임시로 Status2DataV161로 변환한 후 필요한 데이터만 추출
      auto status_data = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_POSITION_CONTROL_3);
      
      // Status2DataV161에서 PositionResponseV161로 데이터 복사
      position_data_out.encoder_position = status_data.encoder_position;
      position_data_out.speed = status_data.speed;
      position_data_out.torque_current = status_data.torque_current;

      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PositionControl3 response: " << e.what()
                << '\n';
    }
  }
  return false;
}

}  // namespace v161_motor_control 