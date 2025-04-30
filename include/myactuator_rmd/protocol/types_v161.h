#ifndef V161_MOTOR_CONTROL__TYPES_V161_HPP
#define V161_MOTOR_CONTROL__TYPES_V161_HPP

#include <vector>  // for ErrorState bits

#include <cstdint>

namespace v161_motor_control::types {

/**
 * @brief MyActuator RMD 시리즈 모터(V1.61 프로토콜)에서 사용되는 데이터 타입 모음
 * 
 * 이 네임스페이스는 모터 통신 프로토콜에서 사용되는 모든 데이터 구조체와 열거형을 정의합니다.
 * 각 구조체는 특정 명령에 대응하는 응답 데이터 형식을 나타냅니다.
 */

// --- Read Command ---

/**
 * @brief PID 제어 파라미터 데이터 (명령 코드: 0x30)
 * 
 * 모터의 각도, 속도, 전류에 대한 PID 제어 파라미터를 담고 있습니다.
 */
struct PidDataV161 {
  uint8_t anglePidKp = 0;  ///< 각도 PID 제어기의 비례 이득
  uint8_t anglePidKi = 0;  ///< 각도 PID 제어기의 적분 이득
  uint8_t speedPidKp = 0;  ///< 속도 PID 제어기의 비례 이득
  uint8_t speedPidKi = 0;  ///< 속도 PID 제어기의 적분 이득
  uint8_t iqPidKp = 0;     ///< 전류(토크) PID 제어기의 비례 이득
  uint8_t iqPidKi = 0;     ///< 전류(토크) PID 제어기의 적분 이득
};

/**
 * @brief 가속도 데이터 (명령 코드: 0x33)
 * 
 * 모터의 가속도 설정값을 담고 있습니다.
 */
struct AccelDataV161 {
  int32_t acceleration = 0;  ///< 가속도 값 (단위: 1 dps/s, 초당 각속도 변화량)
};

/**
 * @brief 엔코더 데이터 (명령 코드: 0x90)
 * 
 * 모터 엔코더의 현재 위치, 원시 위치, 오프셋 값을 담고 있습니다.
 */
struct EncoderDataV161 {
  uint16_t position = 0;      ///< 현재 위치 (오프셋 적용됨), 범위: 0~16383
  uint16_t raw_position = 0;  ///< 원시 위치 (오프셋 미적용), 범위: 0~16383
  uint16_t offset = 0;        ///< 영점 오프셋 값, 범위: 0~16383
};

/**
 * @brief 다중 회전 각도 데이터 (명령 코드: 0x92)
 * 
 * 모터의 다중 회전(multi-turn) 각도 정보를 담고 있습니다.
 * 여러 회전을 누적하여 각도 값을 제공합니다.
 */
struct MultiTurnAngleV161 {
  int64_t angle = 0;  ///< 다중 회전 각도 (단위: 0.01도/LSB, 즉 100으로 나누면 도 단위)
};

/**
 * @brief 단일 회전 각도 데이터 (명령 코드: 0x94)
 * 
 * 모터의 단일 회전(single-turn) 각도 정보를 담고 있습니다.
 * 한 바퀴(360도) 내에서의 위치만 제공합니다.
 */
struct SingleCircleAngleV161 {
  uint16_t angle = 0;  ///< 단일 회전 각도 (단위: 0.01도/LSB, 범위: 0~35999, 즉 0~359.99도)
};

/**
 * @brief 모터 상태 1 및 오류 플래그 데이터 (명령 코드: 0x9A)
 * 
 * 모터의 기본 상태 정보(온도, 전압)와 오류 플래그를 담고 있습니다.
 */
struct Status1DataV161 {
  int8_t temperature = 0;       ///< 모터 온도 (단위: 1℃/LSB)
  uint16_t voltage = 0;         ///< 입력 전압 (단위: 0.1V/LSB, 즉 10으로 나누면 V 단위)
  uint8_t error_state_raw = 0;  ///< 원시 오류 상태 바이트 (각 비트가 특정 오류 상태를 나타냄)

  /**
   * @brief 저전압 오류 상태 확인
   * @return 저전압 오류가 있으면 true, 그렇지 않으면 false
   */
  bool isVoltageLow() const {
    return (error_state_raw & 0x01);  // 비트 0은 저전압 오류를 나타냄
  }
  
  /**
   * @brief 과열 오류 상태 확인
   * @return 과열 오류가 있으면 true, 그렇지 않으면 false
   */
  bool isOverTemperature() const {
    return (error_state_raw & 0x08);  // 비트 3은 과열 오류를 나타냄
  }
};

/**
 * @brief 모터 상태 2 데이터 (명령 코드: 0x9C)
 * 
 * 모터의 상세 상태 정보(온도, 토크 전류, 속도, 엔코더 위치)를 담고 있습니다.
 */
struct Status2DataV161 {
  int8_t temperature = 0;         ///< 모터 온도 (단위: 1℃/LSB)
  int16_t torque_current = 0;     ///< 토크 전류 (범위: -2048~2048, -33A~33A에 매핑됨)
  int16_t speed = 0;              ///< 모터 속도 (단위: 1dps/LSB, 초당 회전 각도)
  uint16_t encoder_position = 0;  ///< 엔코더 위치 (범위: 0~16383)
};

/**
 * @brief 모터 상태 3 데이터 (명령 코드: 0x9D)
 * 
 * 모터의 3상 전류(A, B, C상) 정보를 담고 있습니다.
 */
struct Status3DataV161 {
  int16_t current_A = 0;  ///< A상 전류 (단위: 1A/64LSB)
  int16_t current_B = 0;  ///< B상 전류 (단위: 1A/64LSB)
  int16_t current_C = 0;  ///< C상 전류 (단위: 1A/64LSB)

  /**
   * @brief A상 전류를 암페어 단위로 반환
   * @return A상 전류 (단위: A)
   */
  float getCurrentA() const {
    return static_cast<float>(current_A) / 64.0f;
  }
  
  /**
   * @brief B상 전류를 암페어 단위로 반환
   * @return B상 전류 (단위: A)
   */
  float getCurrentB() const {
    return static_cast<float>(current_B) / 64.0f;
  }
  
  /**
   * @brief C상 전류를 암페어 단위로 반환
   * @return C상 전류 (단위: A)
   */
  float getCurrentC() const {
    return static_cast<float>(current_C) / 64.0f;
  }
};

// --- Write Command ---

/**
 * @brief 엔코더 오프셋 설정 응답 데이터 (명령 코드: 0x91)
 * 
 * 엔코더 오프셋 설정 명령에 대한 응답 데이터를 담고 있습니다.
 */
struct EncoderOffsetResponseDataV161 {
  uint16_t written_offset = 0;  ///< 기록된 오프셋 값 (범위: 0~16383)
};

// --- Control Command ---

/**
 * @brief 모터 회전 방향 열거형
 * 
 * 모터 제어 명령에서 회전 방향을 지정할 때 사용됩니다.
 */
enum class SpinDirection : uint8_t {
  CLOCKWISE = 0x00,            ///< 시계 방향 회전
  COUNTER_CLOCKWISE = 0x01     ///< 반시계 방향 회전
};

}  // namespace v161_motor_control::types

#endif  // V161_MOTOR_CONTROL__TYPES_V161_HPP
