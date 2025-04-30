#ifndef V161_MOTOR_CONTROL__MOTOR_ACTUATOR_H_
#define V161_MOTOR_CONTROL__MOTOR_ACTUATOR_H_

#include <memory>  // for std::shared_ptr
#include <cstdint>
#include <array>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/protocol/types_v161.h"
#include "myactuator_rmd/protocol/protocol_v161.h"

namespace v161_motor_control {

/**
 * @brief 모터 제어 응답 데이터 타입 정의
 * 
 * 이 네임스페이스는 모터 제어 명령의 응답으로 반환되는 데이터 구조체들을 정의합니다.
 */
namespace types {
  /**
   * @brief 토크 제어 응답 데이터 구조체 (명령 코드: 0xA1)
   * 
   * 토크 제어 명령(setTorque)의 응답으로 수신되는 데이터를 담는 구조체입니다.
   */
  struct TorqueResponseV161 {
    int16_t torque_current = 0;     ///< 현재 토크 전류값 (범위: -2048~2048, -33A~33A에 매핑됨)
    int16_t speed = 0;              ///< 현재 모터 속도 (단위: 1dps/LSB, 초당 회전 각도)
    uint16_t encoder_position = 0;  ///< 현재 엔코더 위치 (범위: 0~16383)
  };

  /**
   * @brief 속도 제어 응답 데이터 구조체 (명령 코드: 0xA2)
   * 
   * 속도 제어 명령(setSpeed)의 응답으로 수신되는 데이터를 담는 구조체입니다.
   */
  struct SpeedResponseV161 {
    int16_t speed = 0;              ///< 현재 모터 속도 (단위: 1dps/LSB, 초당 회전 각도)
    int16_t torque_current = 0;     ///< 현재 토크 전류값 (범위: -2048~2048, -33A~33A에 매핑됨)
    uint16_t encoder_position = 0;  ///< 현재 엔코더 위치 (범위: 0~16383)
  };

  /**
   * @brief 위치 제어 응답 데이터 구조체 (명령 코드: 0xA3, 0xA4, 0xA5, 0xA6)
   * 
   * 위치 제어 명령(setAbsolutePosition, setPositionAbsolute, setPositionRelative)의
   * 응답으로 수신되는 데이터를 담는 구조체입니다.
   */
  struct PositionResponseV161 {
    uint16_t encoder_position = 0;  ///< 현재 엔코더 위치 (범위: 0~16383)
    int16_t speed = 0;              ///< 현재 모터 속도 (단위: 1dps/LSB, 초당 회전 각도)
    int16_t torque_current = 0;     ///< 현재 토크 전류값 (범위: -2048~2048, -33A~33A에 매핑됨)
  };
}

/**
 * @brief 모터 제어 작업을 담당하는 클래스
 * 
 * MyActuator RMD 시리즈 모터의 제어 관련 기능을 담당하는 클래스입니다.
 * 모터 전원 제어, 토크 제어, 속도 제어, 위치 제어 등 모터를 직접 
 * 움직이고 작동시키는 명령들을 제공합니다.
 * 
 * MotorV161 클래스의 getActuator() 메서드를 통해 인스턴스를 얻을 수 있습니다.
 */
class MotorActuator {
 public:
  /**
   * @brief MotorActuator 클래스의 생성자
   * 
   * @param can_interface CAN 통신을 위한 인터페이스 객체
   * @param motor_id 대상 모터 ID (유효 범위: 1-32)
   */
  MotorActuator(std::shared_ptr<CanInterface> can_interface, uint8_t motor_id);
  
  /**
   * @brief 소멸자
   */
  ~MotorActuator();
  
  /**
   * @brief 모터 정지 명령 실행 (명령 코드: 0x81)
   * 
   * 모터를 현재 위치에서 정지시킵니다. 제어 루프는 계속 활성화되어 있기 때문에
   * 토크는 유지됩니다.
   * 
   * @return 성공 또는 실패를 나타내는 Status
   * @retval absl::OkStatus() 성공적으로 모터가 정지됨
   * @retval absl::UnavailableError CAN 통신 오류
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::Status stopMotor();
  
  /**
   * @brief 모터 구동 명령 실행 (명령 코드: 0x88)
   * 
   * 모터를 구동 상태로 만듭니다. 이전에 설정된 제어 모드에 따라 동작합니다.
   * 모터가 정지 상태일 때 다시 구동을 시작하려면 이 명령을 사용합니다.
   * 
   * @return 성공 또는 실패를 나타내는 Status
   * @retval absl::OkStatus() 성공적으로 모터가 구동됨
   * @retval absl::UnavailableError CAN 통신 오류
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::Status runMotor();
  
  /**
   * @brief 모터 전원 끄기 명령 실행 (명령 코드: 0x80)
   * 
   * 모터의 토크를 끄고 자유 회전 상태로 만듭니다. 제어 루프도 비활성화됩니다.
   * 
   * @return 성공 또는 실패를 나타내는 Status
   * @retval absl::OkStatus() 성공적으로 모터 전원이 꺼짐
   * @retval absl::UnavailableError CAN 통신 오류
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::Status powerOffMotor();
  
  /**
   * @brief 모터 오류 플래그 초기화 명령 실행 (명령 코드: 0x9B)
   * 
   * 모터에 발생한 오류 플래그를 초기화합니다. 초기화 후에는 모터가 정상 동작할 수 있습니다.
   * 
   * @return 오류 초기화 후 모터 상태 1 데이터를 포함하는 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status1DataV161> 성공 시 모터 상태 1 데이터 포함
   * @retval absl::UnavailableError CAN 통신 오류
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::Status1DataV161> resetMotorError();
  
  /**
   * @brief (0xA1): 토크 제어 모드
   * @param iqControl 토크 제어 값 (-2000 ~ 2000)
   * @return 성공 시 TorqueResponseV161 데이터, 실패 시 에러 Status
   */
  absl::StatusOr<types::TorqueResponseV161> setTorque(int16_t iqControl);
  
  /**
   * @brief (0xA2): 속도 제어 모드
   * @param speed 목표 속도(rpm 단위, -1000 ~ 1000)
   * @return 성공 시 SpeedResponseV161 데이터, 실패 시 에러 Status
   */
  absl::StatusOr<types::SpeedResponseV161> setSpeed(int32_t speed);
  
  /**
   * @brief (0xA3): 절대 위치 제어 모드 (위치<0,360>)
   * @param angle 목표 각도(0.01도 단위, 0 ~ 36000)
   * @param max_speed 최대 속도(0.01rpm 단위, >=0)
   * @return 성공 시 PositionResponseV161 데이터, 실패 시 에러 Status
   */
  absl::StatusOr<types::PositionResponseV161> setAbsolutePosition(
      uint16_t angle, uint16_t max_speed);
  
  /**
   * @brief (0xA4): 다중턴 위치 제어 모드
   * @param position 목표 위치(0.01도 단위, 제한 없음)
   * @param max_speed 최대 속도(0.01rpm 단위, >=0)
   * @return 성공 시 PositionResponseV161 데이터, 실패 시 에러 Status
   */
  absl::StatusOr<types::PositionResponseV161> setPositionAbsolute(
      int32_t position, uint16_t max_speed);
  
  /**
   * @brief (0xA5): 증분 위치 제어 모드
   * @param position_increment 목표 위치 증분값(0.01도 단위)
   * @param max_speed 최대 속도(0.01rpm 단위, >=0)
   * @return 성공 시 PositionResponseV161 데이터, 실패 시 에러 Status
   */
  absl::StatusOr<types::PositionResponseV161> setPositionRelative(
      int32_t position_increment, uint16_t max_speed);
  
  /**
   * @brief (0xA5): 단일 회전 각도 제어 모드 (방향 지정)
   * @param angle_setpoint 목표 각도(0.01도 단위, 0-35999)
   * @param direction 회전 방향(SpinDirection::CLOCKWISE 또는 SpinDirection::COUNTERCLOCKWISE)
   * @return 성공 시 Status2DataV161 데이터, 실패 시 에러 Status
   */
  absl::StatusOr<types::Status2DataV161> setPositionControlWithDirection(
      uint16_t angle_setpoint, types::SpinDirection direction);
  
  /**
   * @brief (0xA6): 단일 회전 각도 제어 모드 (방향 및 최대 속도 지정)
   * @param angle_setpoint 목표 각도(0.01도 단위, 0-35999)
   * @param direction 회전 방향(SpinDirection::CLOCKWISE 또는 SpinDirection::COUNTERCLOCKWISE)
   * @param max_speed 최대 속도(0.01rpm 단위, >=0)
   * @return 성공 시 Status2DataV161 데이터, 실패 시 에러 Status
   */
  absl::StatusOr<types::Status2DataV161> setPositionControlWithDirectionAndSpeed(
      uint16_t angle_setpoint, types::SpinDirection direction, uint16_t max_speed);
  
 private:
  std::shared_ptr<CanInterface> can_interface_;
  uint8_t motor_id_;
  uint32_t request_id_;
  uint32_t response_id_;

  /**
   * @brief 명령을 전송하고 응답을 받는 헬퍼 메서드
   * @param command_data 전송할 명령 데이터
   * @param expected_response_cmd_code 예상되는 응답 명령 코드
   * @param response_data_out [출력] 받은 응답 데이터
   * @param retry_count 재시도 횟수
   * @return 성공 시 OkStatus, 실패 시 에러 Status
   */
  absl::Status sendCommandAndGetResponse(const std::array<uint8_t, 8>& command_data,
                                uint8_t expected_response_cmd_code,
                                std::array<uint8_t, 8>& response_data_out,
                                int retry_count = 0);
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_ACTUATOR_H_ 