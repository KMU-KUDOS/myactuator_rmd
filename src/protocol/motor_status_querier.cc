#include "myactuator_rmd/protocol/motor_status_querier.h"
#include "myactuator_rmd/protocol/motor_v161.h"
#include "myactuator_rmd/protocol/packing_v161.h"
#include "myactuator_rmd/protocol/parsing_v161.h"
#include "myactuator_rmd/protocol/protocol_v161.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

namespace v161_motor_control {

MotorStatusQuerier::MotorStatusQuerier(std::shared_ptr<CanInterface> can_interface, uint8_t motor_id)
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

MotorStatusQuerier::~MotorStatusQuerier() {
  // 소멸자 구현
}

bool MotorStatusQuerier::sendCommandAndGetResponse(
    const std::array<uint8_t, 8>& command_data,
    uint8_t expected_response_cmd_code,
    std::array<uint8_t, 8>& response_data_out,
    int retry_count) {
  if (!can_interface_)
    return false;

  for (int i = 0; i <= retry_count; ++i) {
    auto send_status = can_interface_->sendFrame(request_id_, command_data);
    if (!send_status.ok()) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << ": Failed to send command 0x" << std::hex
                << static_cast<int>(command_data[0]) << std::dec 
                << ": " << send_status.message() << '\n';
      if (i == retry_count)
        return false;  // Last attempt failed
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
      }
    }
  }
  return false;  // 모든 재시도 실패
}

bool MotorStatusQuerier::readPidData(types::PidDataV161& pid_data_out) {
  auto command_data = packing::createReadPidFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_READ_PID, response_data, 1)) {  // 1 retry
    try {
      pid_data_out = parsing::parseReadPidResponse(response_data);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PID response: " << e.what() << '\n';
    }
  }
  return false;  // 실패 시 false 반환
}

bool MotorStatusQuerier::readEncoderData(types::EncoderDataV161& encoder_data_out) {
  auto command_data = packing::createReadEncoderFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_READ_ENCODER, response_data, 1)) {
    try {
      encoder_data_out = parsing::parseReadEncoderResponse(response_data);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing Encoder response: " << e.what() << '\n';
    }
  }
  return false;  // 실패 시 false 반환
}

bool MotorStatusQuerier::readMultiTurnAngle(types::MultiTurnAngleV161& angle_data_out) {
  auto command_data = packing::createReadMultiTurnAngleFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data,
                                protocol::CMD_READ_MULTI_TURN_ANGLE,
                                response_data,
                                1)) {
    try {
      angle_data_out = parsing::parseReadMultiTurnAngleResponse(response_data);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing MultiTurnAngle response: " << e.what()
                << '\n';
    }
  }
  return false;  // 실패 시 false 반환
}

bool MotorStatusQuerier::readMotorStatus1(types::Status1DataV161& status_data_out) {
  auto command_data = packing::createReadStatus1Frame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_READ_STATUS_1, response_data, 1)) {
    try {
      status_data_out = parsing::parseReadStatus1Response(response_data);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing Status1 response: " << e.what() << '\n';
    }
  }
  return false;  // 실패 시 false 반환
}

bool MotorStatusQuerier::readMotorStatus2(types::Status2DataV161& status_data_out) {
  auto command_data = packing::createReadStatus2Frame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_READ_STATUS_2, response_data, 2)) {
    try {
      status_data_out = parsing::parseReadStatus2Response(response_data);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing Status2 response: " << e.what() << '\n';
    }
  }
  return false;  // 실패 시 false 반환
}

bool MotorStatusQuerier::readMotorStatus3(types::Status3DataV161& status_data_out) {
  auto command_data = packing::createReadStatus3Frame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_READ_STATUS_3, response_data, 3)) {
    try {
      status_data_out = parsing::parseReadStatus3Response(response_data);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing Status3 response: " << e.what() << '\n';
    }
  }
  return false;  // 실패 시 false 반환
}

bool MotorStatusQuerier::readAccelerationData(types::AccelDataV161& accel_data_out) {
  auto command_data = packing::createReadAccelFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_READ_ACCEL, response_data, 1)) {
    try {
      accel_data_out = parsing::parseReadAccelResponse(response_data);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing Accel response: " << e.what() << '\n';
    }
  }
  return false;  // 실패 시 false 반환
}

}  // namespace v161_motor_control 