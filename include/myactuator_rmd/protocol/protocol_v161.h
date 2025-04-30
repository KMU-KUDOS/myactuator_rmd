#ifndef V161_MOTOR_CONTROL__PROTOCOL_V161_HPP
#define V161_MOTOR_CONTROL__PROTOCOL_V161_HPP

#include <cmath>
#include <cstdint>

namespace v161_motor_control::protocol {

/**
 * @brief MyActuator RMD 시리즈 모터(V1.61 프로토콜)에서 사용하는 명령어 코드와 프로토콜 관련 상수/함수
 * 
 * 이 네임스페이스는 V1.61 프로토콜의 모든 명령어 코드와 CAN ID 계산 함수를 제공합니다.
 */

// V1.61 Control Command List
/**
 * @brief PID 제어 파라미터 읽기 명령 (0x30)
 * 
 * 모터의 현재 PID 제어 파라미터를 요청합니다.
 */
constexpr uint8_t CMD_READ_PID = 0x30;

/**
 * @brief PID 제어 파라미터를 RAM에 쓰기 명령 (0x31)
 * 
 * 모터의 PID 제어 파라미터를 RAM에 기록합니다. 전원 끄면 설정이 초기화됩니다.
 */
constexpr uint8_t CMD_WRITE_PID_RAM = 0x31;

/**
 * @brief PID 제어 파라미터를 ROM에 쓰기 명령 (0x32)
 * 
 * 모터의 PID 제어 파라미터를 ROM에 기록합니다. 전원을 꺼도 설정이 유지됩니다.
 */
constexpr uint8_t CMD_WRITE_PID_ROM = 0x32;

/**
 * @brief 가속도 읽기 명령 (0x33)
 * 
 * 모터의 현재 가속도 설정을 요청합니다.
 */
constexpr uint8_t CMD_READ_ACCEL = 0x33;

/**
 * @brief 가속도를 RAM에 쓰기 명령 (0x34)
 * 
 * 모터의 가속도 설정을 RAM에 기록합니다. 전원 끄면 설정이 초기화됩니다.
 */
constexpr uint8_t CMD_WRITE_ACCEL_RAM = 0x34;

/**
 * @brief 엔코더 데이터 읽기 명령 (0x90)
 * 
 * 모터 엔코더의 현재 위치, 원시 위치, 오프셋을 요청합니다.
 */
constexpr uint8_t CMD_READ_ENCODER = 0x90;

/**
 * @brief 엔코더 오프셋 쓰기 명령 (0x91)
 * 
 * 모터 엔코더의 영점 오프셋을 설정합니다.
 */
constexpr uint8_t CMD_WRITE_ENCODER_OFFSET = 0x91;

/**
 * @brief 현재 위치를 영점으로 설정하는 명령 (0x19)
 * 
 * 모터의 현재 위치를 영점(0)으로 설정하고 ROM에 저장합니다.
 */
constexpr uint8_t CMD_WRITE_POS_AS_ZERO_ROM = 0x19;

/**
 * @brief 다중 회전 각도 읽기 명령 (0x92)
 * 
 * 모터의 다중 회전(multi-turn) 각도를 요청합니다.
 */
constexpr uint8_t CMD_READ_MULTI_TURN_ANGLE = 0x92;

/**
 * @brief 단일 회전 각도 읽기 명령 (0x94)
 * 
 * 모터의 단일 회전(single-turn) 내 각도를 요청합니다.
 */
constexpr uint8_t CMD_READ_SINGLE_CIRCLE_ANGLE = 0x94;

/**
 * @brief 모터 상태 1 읽기 명령 (0x9A)
 * 
 * 모터의 온도, 전압, 오류 상태 정보를 요청합니다.
 */
constexpr uint8_t CMD_READ_STATUS_1 = 0x9A;

/**
 * @brief 오류 플래그 초기화 명령 (0x9B)
 * 
 * 모터의 오류 플래그를 초기화(클리어)합니다.
 */
constexpr uint8_t CMD_CLEAR_ERROR = 0x9B;

/**
 * @brief 모터 상태 2 읽기 명령 (0x9C)
 * 
 * 모터의 온도, 토크 전류, 속도, 엔코더 위치 정보를 요청합니다.
 */
constexpr uint8_t CMD_READ_STATUS_2 = 0x9C;

/**
 * @brief 모터 상태 3 읽기 명령 (0x9D)
 * 
 * 모터의 3상(A, B, C) 전류 정보를 요청합니다.
 */
constexpr uint8_t CMD_READ_STATUS_3 = 0x9D;

/**
 * @brief 모터 끄기 명령 (0x80)
 * 
 * 모터의 토크를 끄고 자유 회전 상태로 만듭니다.
 */
constexpr uint8_t CMD_MOTOR_OFF = 0x80;

/**
 * @brief 모터 정지 명령 (0x81)
 * 
 * 모터를 현재 위치에서 정지 상태로 만듭니다. 토크는 유지됩니다.
 */
constexpr uint8_t CMD_MOTOR_STOP = 0x81;

/**
 * @brief 모터 구동 명령 (0x88)
 * 
 * 모터를 구동 상태로 만듭니다. 이전에 설정된 제어 모드에 따라 동작합니다.
 */
constexpr uint8_t CMD_MOTOR_RUN = 0x88;

/**
 * @brief 토크 제어 명령 (0xA1)
 * 
 * 모터의 토크를 직접 제어합니다.
 */
constexpr uint8_t CMD_TORQUE_CONTROL = 0xA1;

/**
 * @brief 속도 제어 명령 (0xA2)
 * 
 * 모터의 회전 속도를 제어합니다.
 */
constexpr uint8_t CMD_SPEED_CONTROL = 0xA2;

/**
 * @brief 위치 제어 명령 1 (0xA3)
 * 
 * 모터의 위치를 제어합니다. (위치만 지정)
 */
constexpr uint8_t CMD_POSITION_CONTROL_1 = 0xA3;

/**
 * @brief 위치 제어 명령 2 (0xA4)
 * 
 * 모터의 위치를 제어합니다. (위치 + 속도 지정)
 */
constexpr uint8_t CMD_POSITION_CONTROL_2 = 0xA4;

/**
 * @brief 위치 제어 명령 3 (0xA5)
 * 
 * 모터의 위치를 제어합니다. (위치 + 속도 + 방향 지정)
 */
constexpr uint8_t CMD_POSITION_CONTROL_3 = 0xA5;

/**
 * @brief 위치 제어 명령 4 (0xA6)
 * 
 * 모터의 위치를 제어합니다. (위치 + 속도 + 방향 + 토크 제한 지정)
 */
constexpr uint8_t CMD_POSITION_CONTROL_4 = 0xA6;

/**
 * @brief 모터 ID로부터 V1.61 프로토콜 요청 CAN ID를 계산합니다.
 * 
 * @param motor_id 모터 ID (유효 범위: 1-32)
 * @return 계산된 요청 CAN ID, 유효하지 않은 모터 ID인 경우 0을 반환
 */
inline uint32_t getV161RequestId(uint8_t motor_id) {
  // motor id is in the range 1 ~ 32
  if (motor_id < 1 || motor_id > 32) {
    return 0;
  }
  return 0x140 + static_cast<uint32_t>(motor_id);
}

/**
 * @brief 모터 ID로부터 V1.61 프로토콜 응답 CAN ID를 계산합니다.
 * 
 * @param motor_id 모터 ID (유효 범위: 1-32)
 * @return 계산된 응답 CAN ID, 유효하지 않은 모터 ID인 경우 0을 반환
 */
inline uint32_t getV161ResponseId(uint8_t motor_id) {
  // motor id is in the range 1 ~ 32
  if (motor_id < 1 || motor_id > 32) {
    return 0;
  }
  return 0x140 + static_cast<uint32_t>(motor_id);
}

/**
 * @brief 다중 모터 토크 제어를 위한 요청 CAN ID
 * 
 * 브로드캐스트 방식으로 여러 모터의 토크를 동시에 제어할 때 사용합니다.
 */
constexpr uint32_t MULTI_MOTOR_TORQUE_REQ_ID = 0x280;

}  // namespace v161_motor_control::protocol

#endif  // V161_MOTOR_CONTROL__PROTOCOL_V161_HPP
