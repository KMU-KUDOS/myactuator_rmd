#ifndef V161_MOTOR_CONTROL__PACKING_V161_HPP
#define V161_MOTOR_CONTROL__PACKING_V161_HPP

#include <array>
#include <stdexcept>

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "myactuator_rmd/protocol/types_v161.h"

namespace v161_motor_control::packing {

/**
 * @brief V1.61 프로토콜 패킹 함수 모음
 * 
 * 이 네임스페이스는 MyActuator RMD 시리즈 모터 제어를 위한 V1.61 프로토콜의
 * 데이터 패킹(명령 프레임 생성) 함수들을 제공합니다.
 */

// --- Read Command Frame ---
/**
 * @brief PID 제어 파라미터 읽기 명령 프레임 생성 (0x30)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadPidFrame();

/**
 * @brief 가속도 읽기 명령 프레임 생성 (0x33)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadAccelFrame();

/**
 * @brief 엔코더 데이터 읽기 명령 프레임 생성 (0x90)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadEncoderFrame();

/**
 * @brief 다중 회전 각도 읽기 명령 프레임 생성 (0x92)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadMultiTurnAngleFrame();

/**
 * @brief 단일 회전 각도 읽기 명령 프레임 생성 (0x94)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadSingleCircleAngleFrame();

/**
 * @brief 모터 상태 1 읽기 명령 프레임 생성 (0x9A)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadStatus1Frame();

/**
 * @brief 모터 상태 2 읽기 명령 프레임 생성 (0x9C)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadStatus2Frame();

/**
 * @brief 모터 상태 3 읽기 명령 프레임 생성 (0x9D)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadStatus3Frame();

// --- Write/Action Command Frame ---
/**
 * @brief PID 제어 파라미터를 RAM에 쓰기 명령 프레임 생성 (0x31)
 * 
 * @param pid_data PID 제어 파라미터 정보를 담고 있는 구조체
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createWritePidRamFrame(
    const types::PidDataV161& pid_data);

/**
 * @brief PID 제어 파라미터를 ROM에 쓰기 명령 프레임 생성 (0x32)
 * 
 * @param pid_data PID 제어 파라미터 정보를 담고 있는 구조체
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createWritePidRomFrame(
    const types::PidDataV161& pid_data);

/**
 * @brief 가속도를 RAM에 쓰기 명령 프레임 생성 (0x34)
 * 
 * @param accel_data 가속도 정보를 담고 있는 구조체
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createWriteAccelRamFrame(
    const types::AccelDataV161& accel_data);

/**
 * @brief 엔코더 오프셋 쓰기 명령 프레임 생성 (0x91)
 * 
 * @param offset 설정할 엔코더 오프셋 값 (범위: 0~16383)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 오프셋 값 제공 시
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createWriteEncoderOffsetFrame(uint16_t offset);

/**
 * @brief 현재 위치를 영점으로 설정하는 명령 프레임 생성 (0x19)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createWritePosAsZeroRomFrame();

/**
 * @brief 오류 플래그 초기화 명령 프레임 생성 (0x9B)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createClearErrorFlagFrame();

// --- Control Command Frame ---
/**
 * @brief 모터 끄기 명령 프레임 생성 (0x80)
 * 
 * 모터의 토크를 끄고 자유 회전 상태로 전환합니다.
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createMotorOffFrame();

/**
 * @brief 모터 정지 명령 프레임 생성 (0x81)
 * 
 * 모터를 현재 위치에서 정지 상태로 만듭니다. 토크는 유지됩니다.
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createMotorStopFrame();

/**
 * @brief 모터 구동 명령 프레임 생성 (0x88)
 * 
 * 모터를 구동 상태로 만듭니다. 이전에 설정된 제어 모드에 따라 동작합니다.
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createMotorRunFrame();

/**
 * @brief 토크 제어 명령 프레임 생성 (0xA1)
 * 
 * @param torque_setpoint 토크 전류 설정값 (범위: -2000~2000, 실제 전류 -32A~32A에 해당)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 토크 설정값 제공 시
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createTorqueControlFrame(int16_t torque_setpoint);

/**
 * @brief 속도 제어 명령 프레임 생성 (0xA2)
 * 
 * @param speed_setpoint 속도 설정값 (단위: 0.01 dps/LSB, 100으로 나누면 초당 회전 각도)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createSpeedControlFrame(int32_t speed_setpoint);

/**
 * @brief 위치 제어 1 명령 프레임 생성 (0xA3, 다중 회전 위치 제어)
 * 
 * @param angle_setpoint 위치 설정값 (단위: 0.01도/LSB, 100으로 나누면 도 단위)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createPositionControl1Frame(int32_t angle_setpoint);

/**
 * @brief 위치 제어 2 명령 프레임 생성 (0xA4, 다중 회전 위치 제어 + 최대 속도 제한)
 * 
 * @param angle_setpoint 위치 설정값 (단위: 0.01도/LSB, 100으로 나누면 도 단위)
 * @param max_speed 최대 속도 제한 (단위: 1 dps/LSB, 초당 회전 각도)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 최대 속도 제공 시
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createPositionControl2Frame(int32_t angle_setpoint,
                                                   uint16_t max_speed);

/**
 * @brief 위치 제어 3 명령 프레임 생성 (0xA5, 단일 회전 위치 제어 + 회전 방향 지정)
 * 
 * @param angle_setpoint 위치 설정값 (범위: 0~35999, 단위: 0.01도/LSB, 0~359.99도)
 * @param direction 회전 방향 (CLOCKWISE: 시계 방향, COUNTER_CLOCKWISE: 반시계 방향)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 위치 설정값 제공 시 (0~35999 범위를 벗어남)
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createPositionControl3Frame(
    uint16_t angle_setpoint, types::SpinDirection direction);

/**
 * @brief 위치 제어 4 명령 프레임 생성 (0xA6, 단일 회전 위치 제어 + 회전 방향 + 최대 속도 제한)
 * 
 * @param angle_setpoint 위치 설정값 (범위: 0~35999, 단위: 0.01도/LSB, 0~359.99도)
 * @param direction 회전 방향 (CLOCKWISE: 시계 방향, COUNTER_CLOCKWISE: 반시계 방향)
 * @param max_speed 최대 속도 제한 (단위: 1 dps/LSB, 초당 회전 각도)
 * 
 * @return 8바이트 명령 프레임을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<std::array<uint8_t, 8>> 성공 시 8바이트 명령 프레임 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 위치 또는 속도 설정값 제공 시
 * @retval absl::InternalError 내부 처리 오류 발생 시
 */
absl::StatusOr<std::array<uint8_t, 8>> createPositionControl4Frame(
    uint16_t angle_setpoint,
    types::SpinDirection direction,
    uint16_t max_speed);

// --- Helper function for packing data (will be used more in Write command) ---
/**
 * @brief 값을 리틀 엔디안 형식으로 바이트 배열에 패킹하는 헬퍼 함수
 * 
 * @tparam T 패킹할 값의 타입 (일반적으로 정수 타입)
 * @param data 값을 패킹할 바이트 배열
 * @param index 배열에서 값을 배치할 시작 인덱스
 * @param value 패킹할 값
 * 
 * @return 성공 또는 실패 이유를 나타내는 Status
 * @retval absl::OkStatus() 성공적으로 패킹됨
 * @retval absl::InvalidArgumentError 인덱스가 유효하지 않거나 데이터가 배열 경계를 벗어나는 경우
 */
template <typename T>
absl::Status packLittleEndian(std::array<uint8_t, 8>& data, size_t index, T value);

}  // namespace v161_motor_control::packing

#endif  // V161_MOTOR_CONTROL__PACKING_V161_HPP
