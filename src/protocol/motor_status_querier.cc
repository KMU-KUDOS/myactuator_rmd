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
#include "myactuator_rmd/status_macros.h"

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

absl::StatusOr<std::array<uint8_t, 8>> MotorStatusQuerier::sendCommandAndGetResponse(
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

absl::StatusOr<types::PidDataV161> MotorStatusQuerier::readPidData() {
  auto command_data_or = packing::createReadPidFrame();
  if (!command_data_or.ok()) {
    return absl::Status(
        command_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                   ": Error creating PID command: ", 
                   command_data_or.status().message()));
  }
  
  // 명령 전송 및 응답 수신
  auto response_data_or = sendCommandAndGetResponse(command_data_or.value(), protocol::CMD_READ_PID, 1);
  if (!response_data_or.ok()) {
    return response_data_or.status();
  }
  
  // 응답 데이터 파싱
  auto pid_data_or = parsing::parseReadPidResponse(response_data_or.value());
  if (!pid_data_or.ok()) {
    return absl::Status(
        pid_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                    ": Error parsing PID response: ", 
                    pid_data_or.status().message()));
  }
  
  return pid_data_or;
}

absl::StatusOr<types::EncoderDataV161> MotorStatusQuerier::readEncoderData() {
  auto command_data_or = packing::createReadEncoderFrame();
  if (!command_data_or.ok()) {
    return absl::Status(
        command_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                   ": Error creating Encoder command: ", 
                   command_data_or.status().message()));
  }
  
  // 명령 전송 및 응답 수신
  auto response_data_or = sendCommandAndGetResponse(command_data_or.value(), protocol::CMD_READ_ENCODER, 1);
  if (!response_data_or.ok()) {
    return response_data_or.status();
  }
  
  // 응답 데이터 파싱
  auto encoder_data_or = parsing::parseReadEncoderResponse(response_data_or.value());
  if (!encoder_data_or.ok()) {
    return absl::Status(
        encoder_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                    ": Error parsing Encoder response: ", 
                    encoder_data_or.status().message()));
  }
  
  return encoder_data_or;
}

absl::StatusOr<types::MultiTurnAngleV161> MotorStatusQuerier::readMultiTurnAngle() {
  auto command_data_or = packing::createReadMultiTurnAngleFrame();
  if (!command_data_or.ok()) {
    return absl::Status(
        command_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                   ": Error creating MultiTurnAngle command: ", 
                   command_data_or.status().message()));
  }
  
  // 명령 전송 및 응답 수신
  auto response_data_or = sendCommandAndGetResponse(command_data_or.value(), protocol::CMD_READ_MULTI_TURN_ANGLE, 1);
  if (!response_data_or.ok()) {
    return response_data_or.status();
  }
  
  // 응답 데이터 파싱
  auto angle_data_or = parsing::parseReadMultiTurnAngleResponse(response_data_or.value());
  if (!angle_data_or.ok()) {
    return absl::Status(
        angle_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                    ": Error parsing MultiTurnAngle response: ", 
                    angle_data_or.status().message()));
  }
  
  return angle_data_or;
}

absl::StatusOr<types::Status1DataV161> MotorStatusQuerier::readMotorStatus1() {
  auto command_data_or = packing::createReadStatus1Frame();
  if (!command_data_or.ok()) {
    return absl::Status(
        command_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                   ": Error creating Status1 command: ", 
                   command_data_or.status().message()));
  }
  
  // 명령 전송 및 응답 수신
  auto response_data_or = sendCommandAndGetResponse(command_data_or.value(), protocol::CMD_READ_STATUS_1, 1);
  if (!response_data_or.ok()) {
    return response_data_or.status();
  }
  
  // 응답 데이터 파싱
  auto status_data_or = parsing::parseReadStatus1Response(response_data_or.value());
  if (!status_data_or.ok()) {
    return absl::Status(
        status_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                    ": Error parsing Status1 response: ", 
                    status_data_or.status().message()));
  }
  
  return status_data_or;
}

absl::StatusOr<types::Status2DataV161> MotorStatusQuerier::readMotorStatus2() {
  auto command_data_or = packing::createReadStatus2Frame();
  if (!command_data_or.ok()) {
    return absl::Status(
        command_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                   ": Error creating Status2 command: ", 
                   command_data_or.status().message()));
  }
  
  // 명령 전송 및 응답 수신
  auto response_data_or = sendCommandAndGetResponse(command_data_or.value(), protocol::CMD_READ_STATUS_2, 2);
  if (!response_data_or.ok()) {
    return response_data_or.status();
  }
  
  // 응답 데이터 파싱
  auto status_data_or = parsing::parseReadStatus2Response(response_data_or.value());
  if (!status_data_or.ok()) {
    return absl::Status(
        status_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                    ": Error parsing Status2 response: ", 
                    status_data_or.status().message()));
  }
  
  return status_data_or;
}

absl::StatusOr<types::Status3DataV161> MotorStatusQuerier::readMotorStatus3() {
  auto command_data_or = packing::createReadStatus3Frame();
  if (!command_data_or.ok()) {
    return absl::Status(
        command_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                   ": Error creating Status3 command: ", 
                   command_data_or.status().message()));
  }
  
  // 명령 전송 및 응답 수신
  auto response_data_or = sendCommandAndGetResponse(command_data_or.value(), protocol::CMD_READ_STATUS_3, 3);
  if (!response_data_or.ok()) {
    return response_data_or.status();
  }
  
  // 응답 데이터 파싱
  auto status_data_or = parsing::parseReadStatus3Response(response_data_or.value());
  if (!status_data_or.ok()) {
    return absl::Status(
        status_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                    ": Error parsing Status3 response: ", 
                    status_data_or.status().message()));
  }
  
  return status_data_or;
}

absl::StatusOr<types::AccelDataV161> MotorStatusQuerier::readAccelerationData() {
  auto command_data_or = packing::createReadAccelFrame();
  if (!command_data_or.ok()) {
    return absl::Status(
        command_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                   ": Error creating Accel command: ", 
                   command_data_or.status().message()));
  }
  
  // 명령 전송 및 응답 수신
  auto response_data_or = sendCommandAndGetResponse(command_data_or.value(), protocol::CMD_READ_ACCEL, 1);
  if (!response_data_or.ok()) {
    return response_data_or.status();
  }
  
  // 응답 데이터 파싱
  auto accel_data_or = parsing::parseReadAccelResponse(response_data_or.value());
  if (!accel_data_or.ok()) {
    return absl::Status(
        accel_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                    ": Error parsing Accel response: ", 
                    accel_data_or.status().message()));
  }
  
  return accel_data_or;
}

absl::StatusOr<types::SingleCircleAngleV161> MotorStatusQuerier::readSingleCircleAngle() {
  auto command_data_or = packing::createReadSingleCircleAngleFrame();
  if (!command_data_or.ok()) {
    return absl::Status(
        command_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                   ": Error creating SingleCircleAngle command: ", 
                   command_data_or.status().message()));
  }
  
  // 명령 전송 및 응답 수신
  auto response_data_or = sendCommandAndGetResponse(command_data_or.value(), protocol::CMD_READ_SINGLE_CIRCLE_ANGLE, 1);
  if (!response_data_or.ok()) {
    return response_data_or.status();
  }
  
  // 응답 데이터 파싱
  auto angle_data_or = parsing::parseReadSingleCircleAngleResponse(response_data_or.value());
  if (!angle_data_or.ok()) {
    return absl::Status(
        angle_data_or.status().code(),
        absl::StrCat("Motor ", static_cast<int>(motor_id_),
                    ": Error parsing SingleCircleAngle response: ", 
                    angle_data_or.status().message()));
  }
  
  return angle_data_or;
}

}  // namespace v161_motor_control 