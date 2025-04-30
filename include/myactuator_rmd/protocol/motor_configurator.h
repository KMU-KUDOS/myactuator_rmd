#ifndef V161_MOTOR_CONTROL__MOTOR_CONFIGURATOR_H_
#define V161_MOTOR_CONTROL__MOTOR_CONFIGURATOR_H_

#include <memory>  // for std::shared_ptr
#include <cstdint>

#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/protocol/types_v161.h"

namespace v161_motor_control {

class MotorV161;  // Forward declaration

/**
 * @brief 모터 설정 작업을 담당하는 클래스
 * 
 * PID 설정, 가속도 설정, 엔코더 오프셋 설정 등 모터 구성 관련 쓰기 기능을 담당함
 */
class MotorConfigurator {
 public:
  /**
   * @brief 생성자
   * @param motor_v161 MotorV161 클래스에 대한 참조 (통신 헬퍼 메서드 접근용)
   * @param motor_id 대상 모터 ID (1-32)
   */
  MotorConfigurator(MotorV161* motor_v161, uint8_t motor_id);
  
  /**
   * @brief 소멸자
   */
  ~MotorConfigurator();
  
  /**
   * @brief (0x31): PID 파라미터를 RAM에 쓰기 (전원 끄면 초기화됨)
   * @param pid_data 설정할 PID 값
   * @return 성공 시 true, 실패 시 false
   */
  bool writePidToRam(const types::PidDataV161& pid_data);

  /**
   * @brief (0x32): PID 파라미터를 ROM에 쓰기 (전원 꺼도 유지됨)
   * @param pid_data 설정할 PID 값
   * @return 성공 시 true, 실패 시 false
   */
  bool writePidToRom(const types::PidDataV161& pid_data);

  /**
   * @brief (0x34): 가속도 값을 RAM에 쓰기 (전원 끄면 초기화됨)
   * @param accel_data 설정할 가속도 값
   * @return 성공 시 true, 실패 시 false
   */
  bool writeAccelerationToRam(const types::AccelDataV161& accel_data);

  /**
   * @brief (0x91): 엔코더 제로 오프셋 값 설정
   * @param offset 설정할 오프셋 값 (0 ~ 16383)
   * @param written_offset_out [출력] 실제 모터에 기록된 오프셋 값
   * @return 성공 시 true, 실패 시 false
   */
  bool writeEncoderOffset(uint16_t offset, uint16_t& written_offset_out);

  /**
   * @brief (0x19): 현재 모터 위치를 0으로 ROM에 쓰기 (주의: 칩 수명 영향, 재부팅 필요)
   * @param written_offset_out [출력] 기록된 제로 오프셋 값
   * @return 성공 시 true, 실패 시 false
   */
  bool writePositionAsZero(uint16_t& written_offset_out);
  
 private:
  MotorV161* motor_v161_;
  uint8_t motor_id_;
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_CONFIGURATOR_H_ 