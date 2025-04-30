#ifndef V161_MOTOR_CONTROL__MOTOR_ACTUATOR_H_
#define V161_MOTOR_CONTROL__MOTOR_ACTUATOR_H_

#include <memory>  // for std::shared_ptr
#include <cstdint>
#include <array>

#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/protocol/types_v161.h"
#include "myactuator_rmd/protocol/protocol_v161.h"

namespace v161_motor_control {

// 타입 정의가 없는 경우 여기서 정의
namespace types {
  // 토크 제어 응답 데이터
  struct TorqueResponseV161 {
    int16_t torque_current = 0;    // 토크 전류값
    int16_t speed = 0;             // 현재 속도
    uint16_t encoder_position = 0;  // 현재 엔코더 위치
  };

  // 속도 제어 응답 데이터
  struct SpeedResponseV161 {
    int16_t speed = 0;             // 현재 속도
    int16_t torque_current = 0;    // 토크 전류값
    uint16_t encoder_position = 0;  // 현재 엔코더 위치
  };

  // 위치 제어 응답 데이터
  struct PositionResponseV161 {
    uint16_t encoder_position = 0;  // 현재 엔코더 위치
    int16_t speed = 0;             // 현재 속도
    int16_t torque_current = 0;    // 토크 전류값
  };
}

/**
 * @brief 모터 제어 작업을 담당하는 클래스
 * 
 * 모터 전원 제어, 토크 제어, 속도 제어, 위치 제어 등 모터 작동 관련 기능을 담당함
 */
class MotorActuator {
 public:
  /**
   * @brief 생성자
   * @param can_interface CAN 인터페이스 객체
   * @param motor_id 대상 모터 ID (1-32)
   */
  MotorActuator(std::shared_ptr<CanInterface> can_interface, uint8_t motor_id);
  
  /**
   * @brief 소멸자
   */
  ~MotorActuator();
  
  /**
   * @brief (0x88): 모터의 정지 명령 실행
   * @return 성공 시 true, 실패 시 false
   */
  bool stopMotor();
  
  /**
   * @brief (0x80): 모터 제어 시작 명령 실행
   * @return 성공 시 true, 실패 시 false
   */
  bool runMotor();
  
  /**
   * @brief (0x81): 모터 전원 끄기 명령 실행
   * @return 성공 시 true, 실패 시 false
   */
  bool powerOffMotor();
  
  /**
   * @brief (0x76): 모터 오류 플래그 초기화
   * @return 성공 시 true, 실패 시 false
   */
  bool resetMotorError();
  
  /**
   * @brief (0xA1): 토크 제어 모드
   * @param iqControl 토크 제어 값 (-2000 ~ 2000)
   * @param torque_data_out [출력] 토크 응답 데이터
   * @return 성공 시 true, 실패 시 false
   */
  bool setTorque(int16_t iqControl, types::TorqueResponseV161& torque_data_out);
  
  /**
   * @brief (0xA2): 속도 제어 모드
   * @param speed 목표 속도(rpm 단위, -1000 ~ 1000)
   * @param speed_data_out [출력] 속도 응답 데이터
   * @return 성공 시 true, 실패 시 false
   */
  bool setSpeed(int32_t speed, types::SpeedResponseV161& speed_data_out);
  
  /**
   * @brief (0xA3): 절대 위치 제어 모드 (위치<0,360>)
   * @param angle 목표 각도(0.01도 단위, 0 ~ 36000)
   * @param max_speed 최대 속도(0.01rpm 단위, >=0)
   * @param position_data_out [출력] 위치 응답 데이터
   * @return 성공 시 true, 실패 시 false
   */
  bool setAbsolutePosition(uint16_t angle, uint16_t max_speed, 
                          types::PositionResponseV161& position_data_out);
  
  /**
   * @brief (0xA4): 다중턴 위치 제어 모드
   * @param position 목표 위치(0.01도 단위, 제한 없음)
   * @param max_speed 최대 속도(0.01rpm 단위, >=0)
   * @param position_data_out [출력] 위치 응답 데이터
   * @return 성공 시 true, 실패 시 false
   */
  bool setPositionAbsolute(int32_t position, uint16_t max_speed,
                          types::PositionResponseV161& position_data_out);
  
  /**
   * @brief (0xA5): 증분 위치 제어 모드
   * @param position_increment 목표 위치 증분값(0.01도 단위)
   * @param max_speed 최대 속도(0.01rpm 단위, >=0)
   * @param position_data_out [출력] 위치 응답 데이터
   * @return 성공 시 true, 실패 시 false
   */
  bool setPositionRelative(int32_t position_increment, uint16_t max_speed,
                          types::PositionResponseV161& position_data_out);
  
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
   * @return 성공 시 true, 실패 시 false
   */
  bool sendCommandAndGetResponse(const std::array<uint8_t, 8>& command_data,
                                uint8_t expected_response_cmd_code,
                                std::array<uint8_t, 8>& response_data_out,
                                int retry_count = 0);
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_ACTUATOR_H_ 