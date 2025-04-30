#ifndef V161_MOTOR_CONTROL__MOTOR_STATUS_QUERIER_H_
#define V161_MOTOR_CONTROL__MOTOR_STATUS_QUERIER_H_

#include <memory>  // for std::shared_ptr
#include <cstdint>

#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/protocol/types_v161.h"

namespace v161_motor_control {

class MotorV161;  // Forward declaration

/**
 * @brief 모터 상태 조회 작업을 담당하는 클래스
 * 
 * 엔코더 값, 각도 값, 온도, 전류, 전압 등 모터의 다양한 상태 정보를 조회하는 기능을 담당함
 */
class MotorStatusQuerier {
 public:
  /**
   * @brief 생성자
   * @param motor_v161 MotorV161 클래스에 대한 참조 (통신 헬퍼 메서드 접근용)
   * @param motor_id 대상 모터 ID (1-32)
   */
  MotorStatusQuerier(MotorV161* motor_v161, uint8_t motor_id);
  
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
  MotorV161* motor_v161_;
  uint8_t motor_id_;
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_STATUS_QUERIER_H_ 