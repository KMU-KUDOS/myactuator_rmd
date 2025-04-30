#ifndef V161_MOTOR_CONTROL__MOTOR_STATUS_QUERIER_H_
#define V161_MOTOR_CONTROL__MOTOR_STATUS_QUERIER_H_

#include <memory>  // for std::shared_ptr
#include <cstdint>
#include <array>

#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/protocol/types_v161.h"
#include "myactuator_rmd/protocol/protocol_v161.h"

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
   * @param encoder_data_out [출력] 엔코더 데이터 정보
   * @return 성공 시 true, 실패 시 false
   */
  bool readEncoderData(types::EncoderDataV161& encoder_data_out);
  
  /**
   * @brief (0x92): 멀티턴 각도 정보 읽기
   * @param angle_data_out [출력] 각도 정보
   * @return 성공 시 true, 실패 시 false
   */
  bool readMultiTurnAngle(types::MultiTurnAngleV161& angle_data_out);
  
  /**
   * @brief (0x9A): 모터의 현재 상태 정보 읽기
   * @param status_data_out [출력] 상태 정보 (온도, 전압, 오류 플래그 등)
   * @return 성공 시 true, 실패 시 false
   */
  bool readMotorStatus1(types::Status1DataV161& status_data_out);
  
  /**
   * @brief (0x9C): 모터의 현재 상태 정보 2 읽기
   * @param status_data_out [출력] 상태 정보 (온도, 토크 전류, 속도, 엔코더 위치)
   * @return 성공 시 true, 실패 시 false
   */
  bool readMotorStatus2(types::Status2DataV161& status_data_out);
  
  /**
   * @brief (0x9D): 모터의 전류 값 읽기
   * @param status_data_out [출력] 전류 정보 (A, B, C 상)
   * @return 성공 시 true, 실패 시 false
   */
  bool readMotorStatus3(types::Status3DataV161& status_data_out);
  
  /**
   * @brief (0x30): 모터의 현재 PID 파라미터 값 읽기
   * @param pid_data_out [출력] PID 파라미터 정보
   * @return 성공 시 true, 실패 시 false
   */
  bool readPidData(types::PidDataV161& pid_data_out);
  
  /**
   * @brief (0x33): 모터의 현재 가속도 값 읽기
   * @param accel_data_out [출력] 가속도 정보
   * @return 성공 시 true, 실패 시 false
   */
  bool readAccelerationData(types::AccelDataV161& accel_data_out);
  
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

#endif  // V161_MOTOR_CONTROL__MOTOR_STATUS_QUERIER_H_ 