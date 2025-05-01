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
 * MyActuator RMD 시리즈 모터의 상태 정보를 읽고 모니터링하기 위한 클래스입니다.
 * 엔코더 값, 각도 값, 온도, 전류, 전압 등 모터의 다양한 상태 정보를 조회하는 기능을 제공합니다.
 * 
 * MotorV161 클래스의 getStatusQuerier() 메서드를 통해 인스턴스를 얻을 수 있습니다.
 */
class MotorStatusQuerier {
 public:
  /**
   * @brief MotorStatusQuerier 클래스의 생성자
   * 
   * @param can_interface CAN 통신을 위한 인터페이스 객체
   * @param motor_id 대상 모터 ID (유효 범위: 1-32)
   */
  MotorStatusQuerier(std::shared_ptr<CanInterface> can_interface, uint8_t motor_id);
  
  /**
   * @brief 소멸자
   */
  ~MotorStatusQuerier();
  
  /**
   * @brief (0x90): 모터의 현재 엔코더 값 읽기
   * 
   * 모터의 현재 엔코더 카운트, 원본 엔코더 값, 오프셋 설정 값 등의 정보를 조회합니다.
   * 이 정보는 모터의 정확한 위치와 회전 정보를 파악하는 데 사용됩니다.
   * 
   * @return 엔코더 데이터 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::EncoderDataV161> 성공 시 엔코더 위치, 원본 위치, 오프셋 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::EncoderDataV161> readEncoderData();
  
  /**
   * @brief (0x92): 멀티턴 각도 정보 읽기
   * 
   * 모터의 누적된 회전 각도 정보를 조회합니다. 이 값은 여러 회전을 포함하여
   * 모터가 초기 위치로부터 얼마나 회전했는지를 나타냅니다.
   * 
   * @return 멀티턴 각도 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::MultiTurnAngleV161> 성공 시 누적 회전 각도 정보 포함 (0.01도 단위)
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::MultiTurnAngleV161> readMultiTurnAngle();
  
  /**
   * @brief (0x9A): 모터의 현재 상태 정보 1 읽기
   * 
   * 모터의 온도, 전압 및 오류 플래그 등 기본적인 상태 정보를 조회합니다.
   * 모터의 건강 상태와 동작 환경을 모니터링하는 데 사용됩니다.
   * 
   * @return 상태 1 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status1DataV161> 성공 시 온도, 전압, 오류 플래그 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::Status1DataV161> readMotorStatus1();
  
  /**
   * @brief (0x9C): 모터의 현재 상태 정보 2 읽기
   * 
   * 모터의 온도, 토크 전류, 속도, 엔코더 위치 정보를 조회합니다.
   * 모터의 동작 상태와 성능을 모니터링하는 데 사용됩니다.
   * 
   * @return 상태 2 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status2DataV161> 성공 시 온도, 토크 전류, 속도, 엔코더 위치 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::Status2DataV161> readMotorStatus2();
  
  /**
   * @brief (0x9D): 모터의 3상 전류 값 읽기
   * 
   * 모터의 A, B, C 3상 전류 값을 조회합니다.
   * 모터의 전기적 상태와 부하 조건을 모니터링하는 데 사용됩니다.
   * 
   * @return 상태 3 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status3DataV161> 성공 시 A, B, C 각 상의 전류 값 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::Status3DataV161> readMotorStatus3();
  
  /**
   * @brief (0x30): 모터의 현재 PID 파라미터 값 읽기
   * 
   * 모터의 위치 및 속도 제어에 사용되는 PID 제어 파라미터 값을 조회합니다.
   * 모터 성능 튜닝과 제어 시스템 분석에 사용됩니다.
   * 
   * @return PID 파라미터 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::PidDataV161> 성공 시 위치 제어와 속도 제어 각각의 P, I, D 게인 값 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::PidDataV161> readPidData();
  
  /**
   * @brief (0x33): 모터의 현재 가속도 값 읽기
   * 
   * 모터의 가속 및 감속 설정 값을 조회합니다.
   * 모터의 동작 패턴과 반응 속도를 분석하는 데 사용됩니다.
   * 
   * @return 가속도 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::AccelDataV161> 성공 시 가속도 및 감속도 값 포함 (dps/s 단위)
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::AccelDataV161> readAccelerationData();
  
  /**
   * @brief (0x94): 단일 회전 각도 정보 읽기
   * 
   * 모터의 현재 단일 회전 위치 각도를 조회합니다.
   * 이 값은 0-360도 범위 내에서의 모터 위치를 나타냅니다.
   * 
   * @return 단일 회전 각도 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::SingleCircleAngleV161> 성공 시 현재 각도 정보 포함 (0.01도 단위, 0-35999)
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::SingleCircleAngleV161> readSingleCircleAngle();
  
 private:
  std::shared_ptr<CanInterface> can_interface_;
  uint8_t motor_id_;
  uint32_t request_id_;
  uint32_t response_id_;

  /**
   * @brief 명령을 전송하고 응답을 받는 헬퍼 메서드
   * 
   * CAN 인터페이스를 통해 모터에 명령을 전송하고 응답을 수신하는 내부 유틸리티 메서드입니다.
   * 명령 실패 시 지정된 횟수만큼 재시도할 수 있습니다.
   * 
   * @param command_data 전송할 명령 데이터 (8바이트 배열)
   * @param expected_response_cmd_code 예상되는 응답 명령 코드 (첫 번째 바이트)
   * @param retry_count 통신 실패 시 재시도 횟수 (기본값: 0, 재시도 없음)
   * @return 성공 시 8바이트 응답 데이터 배열, 실패 시 적절한 오류 상태
   * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 응답 데이터 포함
   * @retval absl::UnavailableError CAN 통신 오류 (전송 또는 수신 실패)
   * @retval absl::InternalError 내부 처리 오류 또는 예상 응답 코드와 불일치
   */
  absl::StatusOr<std::array<uint8_t, 8>> sendCommandAndGetResponse(
      const std::array<uint8_t, 8>& command_data,
      uint8_t expected_response_cmd_code,
      int retry_count = 0);
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_STATUS_QUERIER_H_ 