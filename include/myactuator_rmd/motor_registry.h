#ifndef V161_MOTOR_CONTROL__MOTOR_REGISTRY_H_
#define V161_MOTOR_CONTROL__MOTOR_REGISTRY_H_

#include <vector>
#include <cstdint>

#include "absl/status/status.h"

namespace v161_motor_control {

/**
 * @brief 모터 ID와 해당 CAN 필터 ID를 관리하는 클래스
 * 
 * 이 클래스는 MyActuator RMD 시리즈 모터의 ID를 등록하고 추적하며,
 * 등록된 모터들에 대한 적절한 CAN 필터 ID를 생성합니다.
 * 
 * CAN 필터 ID는 모터 응답을 효율적으로 필터링하기 위해 사용되며,
 * 각 모터 ID에 대해 두 개의 필터 ID(읽기 명령 응답용, 쓰기 명령 응답용)가 생성됩니다.
 */
class MotorRegistry {
 public:
  /**
   * @brief 빈 모터 레지스트리로 초기화하는 생성자
   */
  MotorRegistry();
  
  /**
   * @brief 레지스트리에 모터 ID를 추가합니다
   * 
   * @param motor_id 추가할 모터 ID (유효 범위: 1-32)
   * 
   * @return 성공 또는 실패 이유를 나타내는 Status 객체
   * @retval absl::OkStatus() 성공적으로 추가됨
   * @retval absl::InvalidArgumentError 모터 ID가 유효하지 않음 (1-32 범위를 벗어남)
   * @retval absl::AlreadyExistsError 모터 ID가 이미 등록되어 있음
   * 
   * @note 모터 ID는 V1.61 프로토콜에서 1부터 32까지의 값만 유효합니다.
   */
  absl::Status addMotorId(uint8_t motor_id);
  
  /**
   * @brief 등록된 모든 모터 ID 목록을 반환합니다
   * 
   * @return 등록된 모터 ID의 벡터
   */
  std::vector<uint8_t> getRegisteredMotorIds() const;
  
  /**
   * @brief 등록된 모든 모터에 대한 CAN 필터 ID를 생성합니다
   * 
   * 각 모터 ID에 대해 두 개의 필터 ID가 생성됩니다:
   * 1. 읽기 명령 응답용 (0x140 + motor_id)
   * 2. 쓰기 명령 응답용 (0x240 + motor_id)
   * 
   * 이 필터 ID는 CanInterface::setReceiveFilters()에 제공되어
   * 관련 없는 CAN 메시지를 필터링하는 데 사용됩니다.
   * 
   * @return CAN 필터 ID의 벡터
   */
  std::vector<uint32_t> getFilterIds() const;
  
 private:
  std::vector<uint8_t> registered_motor_ids_;
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_REGISTRY_H_ 