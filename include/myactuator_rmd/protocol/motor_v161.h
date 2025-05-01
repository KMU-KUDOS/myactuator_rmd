#ifndef V161_MOTOR_CONTROL__MOTOR_V161_HPP
#define V161_MOTOR_CONTROL__MOTOR_V161_HPP

#include <memory>  // for std::shared_ptr or reference

#include <cstdint>

#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/motor_registry.h"
#include "myactuator_rmd/protocol/types_v161.h"
#include "myactuator_rmd/protocol/motor_configurator.h"
#include "myactuator_rmd/protocol/motor_actuator.h"
#include "myactuator_rmd/protocol/motor_status_querier.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace v161_motor_control {

/**
 * @brief MyActuator RMD-X 시리즈 V161 펌웨어 모터 제어 클래스
 * 
 * MyActuator RMD 모터의 V161 펌웨어를 위한 주요 제어 인터페이스를 제공합니다.
 * 이 클래스는 모터 구성, 제어, 상태 조회를 위한 세 가지 헬퍼 클래스를 통합하며,
 * CAN 통신을 통해 모터와 통신합니다.
 * 
 * 각 모터 인스턴스는 고유한 ID를 가지며, 여러 모터를 함께 제어하기 위해 
 * MotorRegistry를 통해 ID 관리가 이루어집니다.
 */
class MotorV161 {
 public:
  /**
   * @brief MotorV161 클래스 생성자
   * 
   * @param can_interface CAN 통신을 위한 인터페이스 객체
   * @param motor_registry 모터 ID 관리를 위한 레지스트리
   * @param motor_id 제어할 모터의 ID (유효 범위: 1-32)
   */
  MotorV161(std::shared_ptr<CanInterface> can_interface, 
            std::shared_ptr<MotorRegistry> motor_registry,
            uint8_t motor_id);

  // --- Read Method
  /**
   * @brief (0x30): 모터의 PID 파라미터 값 읽기
   * 
   * 모터의 위치 및 속도 제어에 사용되는 PID 제어 파라미터 값을 조회합니다.
   * 이 메서드는 내부적으로 status_querier_->readPidData()를 호출합니다.
   * 
   * @return PID 파라미터 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::PidDataV161> 성공 시 위치 제어와 속도 제어 각각의 P, I, D 게인 값 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::PidDataV161> readPid();

  /**
   * @brief (0x33): 모터의 가속도 값 읽기
   * 
   * 모터의 가속 및 감속 설정 값을 조회합니다.
   * 이 메서드는 내부적으로 status_querier_->readAccelerationData()를 호출합니다.
   * 
   * @return 가속도 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::AccelDataV161> 성공 시 가속도 및 감속도 값 포함 (dps/s 단위)
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::AccelDataV161> readAcceleration();

  /**
   * @brief (0x90): 모터의 엔코더 데이터 읽기
   * 
   * 모터의 현재 엔코더 카운트, 원본 엔코더 값, 오프셋 설정 값 등의 정보를 조회합니다.
   * 이 메서드는 내부적으로 status_querier_->readEncoderData()를 호출합니다.
   * 
   * @return 엔코더 데이터 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::EncoderDataV161> 성공 시 엔코더 위치, 원본 위치, 오프셋 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::EncoderDataV161> readEncoder();

  /**
   * @brief (0x92): 모터의 멀티턴 각도 정보 읽기
   * 
   * 모터의 누적된 회전 각도 정보를 조회합니다. 이 값은 여러 회전을 포함하여
   * 모터가 초기 위치로부터 얼마나 회전했는지를 나타냅니다.
   * 이 메서드는 내부적으로 status_querier_->readMultiTurnAngle()를 호출합니다.
   * 
   * @return 멀티턴 각도 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::MultiTurnAngleV161> 성공 시 누적 회전 각도 정보 포함 (0.01도 단위)
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::MultiTurnAngleV161> readMultiTurnAngle();

  /**
   * @brief (0x94): 모터의 단일 회전 각도 정보 읽기
   * 
   * 모터의 현재 단일 회전 위치 각도를 조회합니다.
   * 이 값은 0-360도 범위 내에서의 모터 위치를 나타냅니다.
   * 이 메서드는 내부적으로 status_querier_->readSingleCircleAngle()를 호출합니다.
   * 
   * @return 단일 회전 각도 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::SingleCircleAngleV161> 성공 시 현재 각도 정보 포함 (0.01도 단위, 0-35999)
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::SingleCircleAngleV161> readSingleCircleAngle();

  /**
   * @brief (0x9A): 모터의 상태 정보 1 읽기
   * 
   * 모터의 온도, 전압 및 오류 플래그 등 기본적인 상태 정보를 조회합니다.
   * 이 메서드는 내부적으로 status_querier_->readMotorStatus1()를 호출합니다.
   * 
   * @return 상태 1 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status1DataV161> 성공 시 온도, 전압, 오류 플래그 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::Status1DataV161> readStatus1();

  /**
   * @brief (0x9C): 모터의 상태 정보 2 읽기
   * 
   * 모터의 온도, 토크 전류, 속도, 엔코더 위치 정보를 조회합니다.
   * 이 메서드는 내부적으로 status_querier_->readMotorStatus2()를 호출합니다.
   * 
   * @return 상태 2 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status2DataV161> 성공 시 온도, 토크 전류, 속도, 엔코더 위치 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::Status2DataV161> readStatus2();

  /**
   * @brief (0x9D): 모터의 3상 전류 값 읽기
   * 
   * 모터의 A, B, C 3상 전류 값을 조회합니다.
   * 이 메서드는 내부적으로 status_querier_->readMotorStatus3()를 호출합니다.
   * 
   * @return 상태 3 정보를 담은 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status3DataV161> 성공 시 A, B, C 각 상의 전류 값 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::Status3DataV161> readStatus3();

  // --- Write/Action Method ---
  /**
   * @brief (0x9B): 모터 오류 플래그 초기화
   * 
   * 모터에 발생한 오류 플래그를 초기화합니다. 실제 오류 원인이 해결된 후에 
   * 이 메서드를 호출해야 모터가 정상 동작할 수 있습니다.
   * 이 메서드는 내부적으로 actuator_->resetMotorError()를 호출합니다.
   * 
   * @return 오류 초기화 후 모터 상태 1 데이터를 포함하는 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status1DataV161> 성공 시 모터 상태 1 데이터 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::StatusOr<types::Status1DataV161> clearErrorFlag();

  // --- Motor State Control Method ---
  /**
   * @brief (0x80): 모터 전원 끄기
   * 
   * 모터의 토크를 끄고 자유 회전 상태로 만듭니다. 제어 루프도 비활성화됩니다.
   * 모든 제어 상태가 초기화됩니다.
   * 이 메서드는 내부적으로 actuator_->powerOffMotor()를 호출합니다.
   * 
   * @return 성공 또는 실패를 나타내는 Status
   * @retval absl::OkStatus() 성공적으로 모터 전원이 꺼짐
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::Status motorOff();

  /**
   * @brief (0x81): 모터 정지
   * 
   * 모터를 현재 위치에서 정지시킵니다. 제어 루프는 계속 활성화되어 있기 때문에
   * 토크는 유지됩니다. 현재 제어 상태는 유지됩니다.
   * 이 메서드는 내부적으로 actuator_->stopMotor()를 호출합니다.
   * 
   * @return 성공 또는 실패를 나타내는 Status
   * @retval absl::OkStatus() 성공적으로 모터가 정지됨
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::Status motorStop();

  /**
   * @brief (0x88): 모터 구동 시작/재개
   * 
   * 정지 상태에서 모터 구동을 시작하거나 재개합니다. 이전에 설정된 제어 모드에 따라 동작합니다.
   * 이 메서드는 내부적으로 actuator_->runMotor()를 호출합니다.
   * 
   * @return 성공 또는 실패를 나타내는 Status
   * @retval absl::OkStatus() 성공적으로 모터가 구동됨
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   */
  absl::Status motorRun();

  // --- Closed-Loop Control Method ---
  /**
   * @brief (0xA1): 토크 제어 모드로 모터 구동
   * 
   * 모터의 토크를 직접 제어하는 명령을 전송합니다. 토크 제어 모드에서는
   * 모터가 지정된 토크 값을 유지하도록 작동합니다.
   * 이 메서드는 내부적으로 actuator_->setTorque()를 호출하고 결과를 Status2DataV161 형식으로 변환합니다.
   * 
   * @param torque_setpoint 토크 제어 값 (-2000 ~ 2000, -32A ~ 32A에 매핑됨)
   * @return 모터 상태 2 데이터를 포함하는 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status2DataV161> 성공 시 온도, 토크, 속도, 엔코더 위치 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   * @retval absl::InvalidArgumentError 유효하지 않은 토크 제어 값
   */
  absl::StatusOr<types::Status2DataV161> setTorqueControl(int16_t torque_setpoint);

  /**
   * @brief (0xA2): 속도 제어 모드로 모터 구동
   * 
   * 모터의 회전 속도를 지정된 값으로 제어합니다. 모터는 내부 PID 제어기를
   * 사용하여 지정된 속도를 유지하려고 시도합니다.
   * 이 메서드는 내부적으로 actuator_->setSpeed()를 호출하고 결과를 Status2DataV161 형식으로 변환합니다.
   * 
   * @param speed_setpoint 목표 속도(0.01 dps/LSB 단위, -32768 ~ 32767)
   * @return 모터 상태 2 데이터를 포함하는 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status2DataV161> 성공 시 온도, 토크, 속도, 엔코더 위치 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   * @retval absl::InvalidArgumentError 유효하지 않은 속도 값
   */
  absl::StatusOr<types::Status2DataV161> setSpeedControl(int32_t speed_setpoint);

  /**
   * @brief (0xA3): 위치 제어 모드 1 (다중턴)
   * 
   * 모터를 지정된 절대 위치로 이동시킵니다. 여러 회전이 가능하며,
   * 기본 속도 설정으로 이동합니다.
   * 이 메서드는 내부적으로 actuator_->setAbsolutePosition()을 호출하고 결과를 Status2DataV161 형식으로 변환합니다.
   * 
   * @param angle_setpoint 목표 각도(0.01도 단위), 양수나 음수 값으로 다양한 회전 가능
   * @return 모터 상태 2 데이터를 포함하는 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status2DataV161> 성공 시 온도, 토크, 속도, 엔코더 위치 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   * @retval absl::InvalidArgumentError 유효하지 않은 각도 값
   */
  absl::StatusOr<types::Status2DataV161> setPositionControl1(int32_t angle_setpoint);

  /**
   * @brief (0xA4): 위치 제어 모드 2 (다중턴, 속도 제한)
   * 
   * 모터를 지정된 절대 위치로 이동시키며, 최대 속도를 제한할 수 있습니다.
   * 이 메서드는 내부적으로 actuator_->setPositionAbsolute()를 호출하고 결과를 Status2DataV161 형식으로 변환합니다.
   * 
   * @param angle_setpoint 목표 각도(0.01도 단위), 양수나 음수 값으로 다양한 회전 가능
   * @param max_speed 최대 속도(0.01rpm 단위, >=0), 모터가 목표 위치로 이동할 때 사용할 최대 속도
   * @return 모터 상태 2 데이터를 포함하는 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status2DataV161> 성공 시 온도, 토크, 속도, 엔코더 위치 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   * @retval absl::InvalidArgumentError 유효하지 않은 각도 또는 속도 값
   */
  absl::StatusOr<types::Status2DataV161> setPositionControl2(int32_t angle_setpoint,
                           uint16_t max_speed);

  /**
   * @brief (0xA5): 위치 제어 모드 3 (단일턴, 방향 지정)
   * 
   * 모터를 0~360도 범위 내의 지정된 위치로 이동시키며, 회전 방향을 명시적으로 지정합니다.
   * 이 메서드는 내부적으로 actuator_->setPositionControlWithDirection()을 호출합니다.
   * 
   * @param angle_setpoint 목표 각도(0.01도 단위, 0-35999 = 0-359.99도)
   * @param direction 회전 방향(SpinDirection::CLOCKWISE: 시계 방향, SpinDirection::COUNTERCLOCKWISE: 시계 반대 방향)
   * @return 모터 상태 2 데이터를 포함하는 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status2DataV161> 성공 시 온도, 토크, 속도, 엔코더 위치 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   * @retval absl::InvalidArgumentError 유효하지 않은 각도 값 또는 방향 값
   */
  absl::StatusOr<types::Status2DataV161> setPositionControl3(uint16_t angle_setpoint,
                           types::SpinDirection direction);

  /**
   * @brief (0xA6): 위치 제어 모드 4 (단일턴, 방향 및 최대 속도 지정)
   * 
   * 모터를 0~360도 범위 내의 지정된 위치로 이동시키며, 회전 방향과 최대 속도를 지정합니다.
   * 이 메서드는 내부적으로 actuator_->setPositionControlWithDirectionAndSpeed()를 호출합니다.
   * 
   * @param angle_setpoint 목표 각도(0.01도 단위, 0-35999 = 0-359.99도)
   * @param max_time 최대 시간 또는 속도 제한(구체적인 단위와 사용법은 모터 모델에 따라 다름)
   * @return 모터 상태 2 데이터를 포함하는 StatusOr 객체 또는 오류 상태
   * @retval absl::StatusOr<types::Status2DataV161> 성공 시 온도, 토크, 속도, 엔코더 위치 정보 포함
   * @retval absl::UnavailableError CAN 통신 오류 발생
   * @retval absl::InternalError 내부 처리 오류
   * @retval absl::InvalidArgumentError 유효하지 않은 각도 값 또는 시간/속도 값
   */
  absl::StatusOr<types::Status2DataV161> setPositionControl4(int32_t angle_setpoint,
                           uint16_t max_time);

  /**
   * @brief 모터 구성 작업을 위한 MotorConfigurator 인스턴스 얻기
   * 
   * 모터의 PID 설정, 가속도 설정, 엔코더 오프셋 등 모터 구성 작업을 수행하기 위한
   * MotorConfigurator 인스턴스에 대한 접근을 제공합니다.
   * 
   * @return MotorConfigurator 인스턴스에 대한 참조
   */
  const std::shared_ptr<MotorConfigurator>& getConfigurator() const {
    return configurator_;
  }

  /**
   * @brief 모터 액추에이터 작업을 위한 MotorActuator 인스턴스 얻기
   * 
   * 모터의 토크 제어, 속도 제어, 위치 제어 등 모터를 직접 제어하기 위한
   * MotorActuator 인스턴스에 대한 접근을 제공합니다.
   * 
   * @return MotorActuator 인스턴스에 대한 참조
   */
  MotorActuator& getActuator() { return *actuator_; }

  /**
   * @brief 모터 상태 조회 작업을 위한 MotorStatusQuerier 인스턴스 얻기
   * 
   * 모터의 상태 정보, 엔코더 값, 각도 값 등을 조회하기 위한
   * MotorStatusQuerier 인스턴스에 대한 접근을 제공합니다.
   * 
   * @return MotorStatusQuerier 인스턴스에 대한 참조
   */
  MotorStatusQuerier& getStatusQuerier() { return *status_querier_; }

 private:
  std::shared_ptr<CanInterface> can_interface_;
  std::shared_ptr<MotorRegistry> motor_registry_;
  std::shared_ptr<MotorConfigurator> configurator_;
  std::unique_ptr<MotorActuator> actuator_;
  std::unique_ptr<MotorStatusQuerier> status_querier_;

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
   * @param response_data_out [출력] 받은 응답 데이터를 저장할 8바이트 배열
   * @param retry_count 통신 실패 시 재시도 횟수 (기본값: 0, 재시도 없음)
   * @return 성공 여부(true: 성공, false: 실패)
   */
  bool sendCommandAndGetResponse(const std::array<uint8_t, 8>& command_data,
                                 uint8_t expected_response_cmd_code,
                                 std::array<uint8_t, 8>& response_data_out,
                                 int retry_count = 0);
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_V161_HPP
