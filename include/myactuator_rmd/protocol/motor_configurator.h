#ifndef V161_MOTOR_CONTROL__MOTOR_CONFIGURATOR_H_
#define V161_MOTOR_CONTROL__MOTOR_CONFIGURATOR_H_

#include <memory>  // for std::shared_ptr
#include <cstdint>
#include <array>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/protocol/types_v161.h"

namespace v161_motor_control {

/**
 * @brief 모터 설정 작업을 담당하는 클래스
 * 
 * MyActuator RMD 시리즈 모터의 설정 관련 기능을 담당하는 클래스입니다.
 * PID 제어 파라미터 설정, 가속도 설정, 엔코더 오프셋 설정 등 모터의
 * 구성 및 제어 특성을 조정하는 기능을 제공합니다.
 * 
 * MotorV161 클래스의 getConfigurator() 메서드를 통해 인스턴스를 얻을 수 있습니다.
 */
class MotorConfigurator {
 public:
  /**
   * @brief MotorConfigurator 클래스의 생성자
   * 
   * @param can_interface CAN 통신을 위한 인터페이스 객체
   * @param motor_id 대상 모터 ID (유효 범위: 1-32)
   */
  MotorConfigurator(std::shared_ptr<CanInterface> can_interface, uint8_t motor_id);
  
  /**
   * @brief 소멸자
   */
  ~MotorConfigurator();
  
  /**
   * @brief PID 제어 파라미터를 RAM에 쓰기 (명령 코드: 0x31)
   * 
   * 모터의 PID 제어 파라미터를 RAM에 기록합니다. 이 설정은 모터 전원을 끄면 초기화됩니다.
   * 
   * @param pid_data 설정할 PID 값을 담은 구조체
   * 
   * @return 성공 또는 실패를 나타내는 Status
   * @retval absl::OkStatus() 성공적으로 설정됨
   * @retval absl::InvalidArgumentError 유효하지 않은 PID 데이터
   * @retval absl::UnavailableError CAN 통신 오류
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::Status writePidToRam(const types::PidDataV161& pid_data);

  /**
   * @brief PID 제어 파라미터를 ROM에 쓰기 (명령 코드: 0x32)
   * 
   * 모터의 PID 제어 파라미터를 ROM에 기록합니다. 이 설정은 모터 전원을 꺼도 유지됩니다.
   * 
   * @param pid_data 설정할 PID 값을 담은 구조체
   * 
   * @return 성공 또는 실패를 나타내는 Status
   * @retval absl::OkStatus() 성공적으로 설정됨
   * @retval absl::InvalidArgumentError 유효하지 않은 PID 데이터
   * @retval absl::UnavailableError CAN 통신 오류
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::Status writePidToRom(const types::PidDataV161& pid_data);

  /**
   * @brief 가속도 값을 RAM에 쓰기 (명령 코드: 0x34)
   * 
   * 모터의 가속도 설정을 RAM에 기록합니다. 이 설정은 모터 전원을 끄면 초기화됩니다.
   * 
   * @param accel_data 설정할 가속도 값을 담은 구조체 (단위: 1 dps/s)
   * 
   * @return 성공 또는 실패를 나타내는 Status
   * @retval absl::OkStatus() 성공적으로 설정됨
   * @retval absl::InvalidArgumentError 유효하지 않은 가속도 데이터
   * @retval absl::UnavailableError CAN 통신 오류
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::Status writeAccelerationToRam(const types::AccelDataV161& accel_data);

  /**
   * @brief 엔코더 제로 오프셋 값 설정 (명령 코드: 0x91)
   * 
   * 모터 엔코더의 영점 오프셋 값을 설정합니다. 이 설정은 모터 위치 인식의 기준점을 변경합니다.
   * 
   * @param offset 설정할 오프셋 값 (유효 범위: 0~16383)
   * 
   * @return 성공 시 설정된 오프셋 값을 포함하는 StatusOr 객체, 실패 시 오류 상태
   * @retval absl::StatusOr<uint16_t> 성공 시 설정된 오프셋 값 포함
   * @retval absl::InvalidArgumentError 유효하지 않은 오프셋 값 (0~16383 범위를 벗어남)
   * @retval absl::UnavailableError CAN 통신 오류
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<uint16_t> writeEncoderOffset(uint16_t offset);

  /**
   * @brief 현재 모터 위치를 0으로 ROM에 쓰기 (명령 코드: 0x19)
   * 
   * 현재 모터 위치를 영점(0)으로 설정하고 ROM에 기록합니다.
   * 
   * @warning 이 함수는 ROM에 쓰기 작업을 수행하므로 빈번한 호출은 ROM 수명에 영향을 줄 수 있습니다.
   * @warning 설정 완료 후 모터를 재부팅해야 정상적으로 적용됩니다.
   * 
   * @return 성공 시 기록된 제로 오프셋 값을 포함하는 StatusOr 객체, 실패 시 오류 상태
   * @retval absl::StatusOr<uint16_t> 성공 시 설정된 제로 오프셋 값 포함
   * @retval absl::UnavailableError CAN 통신 오류
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<uint16_t> writePositionAsZero();
  
 private:
  /** @brief CAN 통신을 위한 인터페이스 객체 */
  std::shared_ptr<CanInterface> can_interface_;
  
  /** @brief 대상 모터 ID */
  uint8_t motor_id_;
  
  /** @brief 모터에 명령을 전송할 때 사용하는 CAN ID */
  uint32_t request_id_;
  
  /** @brief 모터로부터 응답을 받을 때 사용하는 CAN ID */
  uint32_t response_id_;
  
  /**
   * @brief 명령을 전송하고 응답을 받는 내부 헬퍼 메서드
   * 
   * 이 메서드는 CAN 프레임을 전송하고 응답을 기다린 후 응답 데이터를 반환합니다.
   * 
   * @param command_data 전송할 명령 데이터 (8바이트 배열)
   * @param expected_response_cmd_code 예상되는 응답 명령 코드
   * @param retry_count 통신 실패 시 재시도 횟수
   * 
   * @return 성공 시 모터 응답 데이터를 포함하는 StatusOr 객체, 실패 시 오류 상태
   * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 응답 데이터 포함
   * @retval absl::UnavailableError CAN 통신 오류
   * @retval absl::InvalidArgumentError 예상하지 않은 응답 명령 코드
   * @retval absl::DeadlineExceededError 응답 타임아웃
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<std::array<uint8_t, 8>> sendCommandAndGetResponse(
      const std::array<uint8_t, 8>& command_data,
                                 uint8_t expected_response_cmd_code,
                                 int retry_count = 0);
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_CONFIGURATOR_H_ 