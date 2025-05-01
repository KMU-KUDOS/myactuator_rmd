#include "myactuator_rmd/protocol/motor_configurator.h"
#include "myactuator_rmd/protocol/packing_v161.h"
#include "myactuator_rmd/protocol/parsing_v161.h"
#include "myactuator_rmd/protocol/protocol_v161.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

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

absl::StatusOr<std::array<uint8_t, 8>> MotorConfigurator::sendCommandAndGetResponse(
    const std::array<uint8_t, 8>& command_data,
    uint8_t expected_response_cmd_code,
    int retry_count) {
  if (!can_interface_) {
    return absl::InternalError("CAN interface is null");
  }

  for (int i = 0; i <= retry_count; ++i) {
    auto send_status = can_interface_->sendFrame(request_id_, command_data);
    if (!send_status.ok()) {
      // 마지막 시도에서 실패한 경우 상태를 반환하고, 그렇지 않으면 재시도
      if (i == retry_count) {
        return absl::Status(
            send_status.code(),
            absl::StrCat("Motor ", static_cast<int>(motor_id_),
                        ": Failed to send command 0x", absl::Hex(command_data[0]),
                        ": ", send_status.message()));
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds(10));  // Retry after a short wait
      continue;
    }

    // 응답 대기
    std::array<uint8_t, 8> response_data;
    auto recv_status = can_interface_->receiveFrame(response_id_, response_data);
    if (recv_status.ok()) {
      // 성공적으로 기대 ID의 프레임을 수신함
      // 이제 데이터 내의 명령 코드 확인
      if (response_data[0] == expected_response_cmd_code) {
        return response_data;  // 성공: 올바른 ID와 명령 코드
      } else {
        // 올바른 ID를 받았지만 예상치 못한 명령 코드
        // 현재 시도를 실패로 처리하고 다음 시도로 계속
        // 로그 대신 마지막 시도가 아니면 그냥 다음 반복으로 진행
      }
    } else {
      // receiveFrame이 오류 반환: 타임아웃 또는 예상치 못한 ID의 프레임 수신
      if (i < retry_count) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 재시도 전 대기
      } else {
        // 마지막 시도에서 실패
        return absl::Status(
            recv_status.code(),
            absl::StrCat("Motor ", static_cast<int>(motor_id_),
                        ": Receive timeout/failure for cmd 0x", absl::Hex(command_data[0]),
                        ". No more retries after ", retry_count, " attempts. Error: ",
                        recv_status.message()));
      }
    }
  }
  
  // 모든 시도 실패 시
  return absl::DeadlineExceededError(absl::StrCat(
      "Motor ", static_cast<int>(motor_id_),
      ": Failed to get valid response for command 0x", absl::Hex(command_data[0]),
      " after ", retry_count + 1, " attempts"));
}

absl::Status MotorConfigurator::writePidToRam(const types::PidDataV161& pid_data) {
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createWritePidRamFrame(pid_data);
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  auto response_or = sendCommandAndGetResponse(
      command_data_or.value(), protocol::CMD_WRITE_PID_RAM, 1);
  if (!response_or.ok()) {
    return response_or.status();
  }
  
  // 쓰기 명령의 응답은 요청과 동일한 데이터(Echo) 확인
  if (response_or.value() != command_data_or.value()) {
    return absl::InternalError("Response does not match command data (echo verification failed)");
  }
  
  return absl::OkStatus();
}

absl::Status MotorConfigurator::writePidToRom(const types::PidDataV161& pid_data) {
  std::cout << "경고: ROM에 PID 쓰기 (0x32). 빈번한 쓰기는 칩 수명에 영향을 줄 수 있습니다."
            << '\n';
  
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createWritePidRomFrame(pid_data);
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  auto response_or = sendCommandAndGetResponse(
      command_data_or.value(), protocol::CMD_WRITE_PID_ROM, 1);
  if (!response_or.ok()) {
    return response_or.status();
  }
  
  // 쓰기 명령의 응답은 요청과 동일한 데이터(Echo) 확인
  if (response_or.value() != command_data_or.value()) {
    return absl::InternalError("Response does not match command data (echo verification failed)");
  }
  
  return absl::OkStatus();
}

absl::Status MotorConfigurator::writeAccelerationToRam(const types::AccelDataV161& accel_data) {
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createWriteAccelRamFrame(accel_data);
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  auto response_or = sendCommandAndGetResponse(
      command_data_or.value(), protocol::CMD_WRITE_ACCEL_RAM, 1);
  if (!response_or.ok()) {
    return response_or.status();
  }
  
  // 쓰기 명령의 응답은 요청과 동일한 데이터(Echo) 확인
  if (response_or.value() != command_data_or.value()) {
    return absl::InternalError("Response does not match command data (echo verification failed)");
  }
  
  return absl::OkStatus();
}

absl::StatusOr<uint16_t> MotorConfigurator::writeEncoderOffset(uint16_t offset) {
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createWriteEncoderOffsetFrame(offset);
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  auto response_or = sendCommandAndGetResponse(
      command_data_or.value(), protocol::CMD_WRITE_ENCODER_OFFSET, 1);
  if (!response_or.ok()) {
    return response_or.status();
  }
  
  // 응답 데이터 파싱
  absl::StatusOr<uint16_t> written_offset = parsing::parseWriteEncoderOffsetResponse(response_or.value());
  if (!written_offset.ok()) {
    return written_offset.status();
    }
  
  return written_offset;
}

absl::StatusOr<uint16_t> MotorConfigurator::writePositionAsZero() {
  std::cout << "경고: 현재 위치를 ROM에 0으로 쓰기 (0x19). "
               "재시작이 필요합니다. 빈번한 쓰기는 칩 수명에 영향을 줄 수 있습니다."
            << '\n';
  
  absl::StatusOr<std::array<uint8_t, 8>> command_data_or = packing::createWritePosAsZeroRomFrame();
  if (!command_data_or.ok()) {
    return command_data_or.status();
  }
  
  auto response_or = sendCommandAndGetResponse(
      command_data_or.value(), protocol::CMD_WRITE_POS_AS_ZERO_ROM, 1);
  if (!response_or.ok()) {
    return response_or.status();
  }
  
  // 응답 데이터 파싱
  absl::StatusOr<uint16_t> written_offset = parsing::parseWritePosAsZeroRomResponse(response_or.value());
  if (!written_offset.ok()) {
    return written_offset.status();
    }
  
  return written_offset;
}

}  // namespace v161_motor_control 