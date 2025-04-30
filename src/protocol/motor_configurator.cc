#include "myactuator_rmd/protocol/motor_configurator.h"
#include "myactuator_rmd/protocol/packing_v161.h"
#include "myactuator_rmd/protocol/parsing_v161.h"
#include "myactuator_rmd/protocol/protocol_v161.h"

#include <chrono>
#include <iostream>
#include <thread>

namespace v161_motor_control {

MotorConfigurator::MotorConfigurator(std::shared_ptr<CanInterface> can_interface, uint8_t motor_id)
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

MotorConfigurator::~MotorConfigurator() {
  // 소멸자 구현
}

bool MotorConfigurator::sendCommandAndGetResponse(
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

bool MotorConfigurator::writePidToRam(const types::PidDataV161& pid_data) {
  auto command_data = packing::createWritePidRamFrame(pid_data);
  std::array<uint8_t, 8> response_data;

  // 쓰기 명령의 응답은 요청과 동일한 데이터(Echo)
  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_WRITE_PID_RAM, response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorConfigurator::writePidToRom(const types::PidDataV161& pid_data) {
  std::cout << "경고: ROM에 PID 쓰기 (0x32). 빈번한 쓰기는 칩 수명에 영향을 줄 수 있습니다."
            << '\n';
  auto command_data = packing::createWritePidRomFrame(pid_data);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_WRITE_PID_ROM, response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorConfigurator::writeAccelerationToRam(const types::AccelDataV161& accel_data) {
  auto command_data = packing::createWriteAccelRamFrame(accel_data);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_WRITE_ACCEL_RAM, response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorConfigurator::writeEncoderOffset(uint16_t offset, uint16_t& written_offset_out) {
  auto command_data = packing::createWriteEncoderOffsetFrame(offset);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_WRITE_ENCODER_OFFSET, response_data, 1)) {
    try {
      written_offset_out = parsing::parseWriteEncoderOffsetResponse(response_data);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing WriteEncoderOffset response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorConfigurator::writePositionAsZero(uint16_t& written_offset_out) {
  std::cout << "경고: 현재 위치를 ROM에 0으로 쓰기 (0x19). "
               "재시작이 필요합니다. 빈번한 쓰기는 칩 수명에 영향을 줄 수 있습니다."
            << '\n';
  auto command_data = packing::createWritePosAsZeroRomFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data,
                                protocol::CMD_WRITE_POS_AS_ZERO_ROM,
                                response_data,
                                1)) {
    try {
      written_offset_out = parsing::parseWritePosAsZeroRomResponse(response_data);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing WritePosAsZero response: " << e.what()
                << '\n';
    }
  }
  return false;
}

}  // namespace v161_motor_control 