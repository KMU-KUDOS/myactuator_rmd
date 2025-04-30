#ifndef V161_MOTOR_CONTROL__MOTOR_STATUS_QUERIER_H_
#define V161_MOTOR_CONTROL__MOTOR_STATUS_QUERIER_H_

#include <memory>  // for std::shared_ptr
#include <cstdint>
#include <array>

#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/protocol/types_v161.h"
#include "myactuator_rmd/protocol/protocol_v161.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace v161_motor_control {

/**
 * @brief 모터 상태 조회 작업을 담당하는 클래스
 * 
 * 엔코더 값, 각도 값, 온도, 전류, 전압 등 모터의 다양한 상태 정보를 조회하는 기능을 담당함
 */
class MotorStatusQuerier {
 public:
  /**
   * @brief 생성자
   * @param can_interface CAN 인터페이스 객체
   * @param motor_id 대상 모터 ID (1-32)
   */
  MotorStatusQuerier(std::shared_ptr<CanInterface> can_interface, uint8_t motor_id);
  
  /**
   * @brief 소멸자
   */
  ~MotorStatusQuerier();
  
  /**
   * @brief (0x90): 모터의 현재 엔코더 값 읽기
   * @return 성공 시 엔코더 데이터 정보를 담은 StatusOr 객체, 실패 시 오류 상태
   */
  absl::StatusOr<types::EncoderDataV161> readEncoderData();
  
  /**
   * @brief (0x92): 멀티턴 각도 정보 읽기
   * @return 성공 시 각도 정보를 담은 StatusOr 객체, 실패 시 오류 상태
   */
  absl::StatusOr<types::MultiTurnAngleV161> readMultiTurnAngle();
  
  /**
   * @brief (0x9A): 모터의 현재 상태 정보 읽기
   * @return 성공 시 상태 정보(온도, 전압, 오류 플래그 등)를 담은 StatusOr 객체, 실패 시 오류 상태
   */
  absl::StatusOr<types::Status1DataV161> readMotorStatus1();
  
  /**
   * @brief (0x9C): 모터의 현재 상태 정보 2 읽기
   * @return 성공 시 상태 정보(온도, 토크 전류, 속도, 엔코더 위치)를 담은 StatusOr 객체, 실패 시 오류 상태
   */
  absl::StatusOr<types::Status2DataV161> readMotorStatus2();
  
  /**
   * @brief (0x9D): 모터의 전류 값 읽기
   * @return 성공 시 전류 정보(A, B, C 상)를 담은 StatusOr 객체, 실패 시 오류 상태
   */
  absl::StatusOr<types::Status3DataV161> readMotorStatus3();
  
  /**
   * @brief (0x30): 모터의 현재 PID 파라미터 값 읽기
   * @return 성공 시 PID 파라미터 정보를 담은 StatusOr 객체, 실패 시 오류 상태
   */
  absl::StatusOr<types::PidDataV161> readPidData();
  
  /**
   * @brief (0x33): 모터의 현재 가속도 값 읽기
   * @return 성공 시 가속도 정보를 담은 StatusOr 객체, 실패 시 오류 상태
   */
  absl::StatusOr<types::AccelDataV161> readAccelerationData();
  
  /**
   * @brief (0x94): 단일 회전 각도 정보 읽기
   * @return 성공 시 단일 회전 각도 정보를 담은 StatusOr 객체, 실패 시 오류 상태
   */
  absl::StatusOr<types::SingleCircleAngleV161> readSingleCircleAngle();
  
 private:
  std::shared_ptr<CanInterface> can_interface_;
  uint8_t motor_id_;
  uint32_t request_id_;
  uint32_t response_id_;

  /**
   * @brief 명령을 전송하고 응답을 받는 헬퍼 메서드
   * @param command_data 전송할 명령 데이터
   * @param expected_response_cmd_code 예상되는 응답 명령 코드
   * @param retry_count 재시도 횟수
   * @return 성공 시 응답 데이터를 담은 StatusOr 객체, 실패 시 적절한 오류 상태를 담은 StatusOr 객체
   */
  absl::StatusOr<std::array<uint8_t, 8>> sendCommandAndGetResponse(
      const std::array<uint8_t, 8>& command_data,
      uint8_t expected_response_cmd_code,
      int retry_count = 0);
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_STATUS_QUERIER_H_ 