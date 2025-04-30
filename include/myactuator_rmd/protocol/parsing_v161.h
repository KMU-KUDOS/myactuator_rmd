#ifndef V161_MOTOR_CONTROL__PARSING_V161_HPP
#define V161_MOTOR_CONTROL__PARSING_V161_HPP

#include <array>
#include <stdexcept>

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "myactuator_rmd/protocol/types_v161.h"

namespace v161_motor_control::parsing {

/**
 * @brief V1.61 프로토콜 파싱 함수 모음
 * 
 * 이 네임스페이스는 MyActuator RMD 시리즈 모터로부터 받은 응답 데이터를
 * 파싱(해석)하는 함수들을 제공합니다.
 */

// --- Helper function for unpacking data (Little Endian)
/**
 * @brief 바이트 배열에서 리틀 엔디안 형식의 값을 추출하는 헬퍼 함수
 * 
 * @tparam T 추출할 값의 타입 (일반적으로 정수 타입)
 * @param data 값을 추출할 바이트 배열
 * @param index 배열에서 값이 저장된 시작 인덱스
 * 
 * @return 추출된 값을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<T> 성공 시 추출된 값 포함
 * @retval absl::InvalidArgumentError 인덱스가 범위를 벗어나거나 데이터 크기가 불충분할 경우
 */
template <typename T>
absl::StatusOr<T> unpackLittleEndian(const std::array<uint8_t, 8>& data, size_t index);

// --- Read Command Response Parsing Function Declaration
/**
 * @brief PID 응답 데이터 파싱 (명령 코드: 0x30)
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 PID 데이터를 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<types::PidDataV161> 성공 시 파싱된 PID 데이터 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<types::PidDataV161> parseReadPidResponse(const std::array<uint8_t, 8>& data);

/**
 * @brief 가속도 응답 데이터 파싱 (명령 코드: 0x33)
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 가속도 데이터를 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<types::AccelDataV161> 성공 시 파싱된 가속도 데이터 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<types::AccelDataV161> parseReadAccelResponse(const std::array<uint8_t, 8>& data);

/**
 * @brief 엔코더 응답 데이터 파싱 (명령 코드: 0x90)
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 엔코더 데이터를 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<types::EncoderDataV161> 성공 시 파싱된 엔코더 데이터 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<types::EncoderDataV161> parseReadEncoderResponse(
    const std::array<uint8_t, 8>& data);

/**
 * @brief 다중 회전 각도 응답 데이터 파싱 (명령 코드: 0x92)
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 다중 회전 각도 데이터를 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<types::MultiTurnAngleV161> 성공 시 파싱된 다중 회전 각도 데이터 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<types::MultiTurnAngleV161> parseReadMultiTurnAngleResponse(
    const std::array<uint8_t, 8>& data);

/**
 * @brief 단일 회전 각도 응답 데이터 파싱 (명령 코드: 0x94)
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 단일 회전 각도 데이터를 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<types::SingleCircleAngleV161> 성공 시 파싱된 단일 회전 각도 데이터 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<types::SingleCircleAngleV161> parseReadSingleCircleAngleResponse(
    const std::array<uint8_t, 8>& data);

/**
 * @brief 모터 상태 1 응답 데이터 파싱 (명령 코드: 0x9A)
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 모터 상태 1 데이터를 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<types::Status1DataV161> 성공 시 파싱된 모터 상태 1 데이터 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<types::Status1DataV161> parseReadStatus1Response(
    const std::array<uint8_t, 8>& data);

/**
 * @brief 모터 상태 2 응답 데이터 파싱 (명령 코드: 0x9C)
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 모터 상태 2 데이터를 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<types::Status2DataV161> 성공 시 파싱된 모터 상태 2 데이터 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<types::Status2DataV161> parseReadStatus2Response(
    const std::array<uint8_t, 8>& data);

/**
 * @brief 모터 상태 3 응답 데이터 파싱 (명령 코드: 0x9D)
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 모터 상태 3 데이터를 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<types::Status3DataV161> 성공 시 파싱된 모터 상태 3 데이터 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<types::Status3DataV161> parseReadStatus3Response(
    const std::array<uint8_t, 8>& data);

// --- Write/Action Command Response Parsing Function Declaration ---
/**
 * @brief 엔코더 오프셋 설정 응답 데이터 파싱 (명령 코드: 0x91)
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 오프셋 값을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<uint16_t> 성공 시 파싱된 오프셋 값(0~16383) 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<uint16_t> parseWriteEncoderOffsetResponse(const std::array<uint8_t, 8>& data);

/**
 * @brief 현재 위치를 영점으로 설정 응답 데이터 파싱 (명령 코드: 0x19)
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 오프셋 값을 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<uint16_t> 성공 시 파싱된 오프셋 값(0~16383) 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<uint16_t> parseWritePosAsZeroRomResponse(const std::array<uint8_t, 8>& data);

/**
 * @brief 오류 플래그 초기화 응답 데이터 파싱 (명령 코드: 0x9B)
 * 
 * 응답 형식은 모터 상태 1(Status1)과 동일합니다.
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * 
 * @return 파싱된 모터 상태 1 데이터를 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<types::Status1DataV161> 성공 시 파싱된 모터 상태 1 데이터 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<types::Status1DataV161> parseClearErrorFlagResponse(
    const std::array<uint8_t, 8>& data);

// --- Control Command Response Parsing Function Declaration ---
/**
 * @brief 폐루프 제어 명령 응답 데이터 파싱 (명령 코드: 0xA1~0xA6)
 * 
 * 응답 형식은 모터 상태 2(Status2)와 동일합니다.
 * 이 함수는 토크 제어(0xA1), 속도 제어(0xA2), 위치 제어(0xA3~0xA6) 
 * 등 폐루프 제어 명령의 응답을 파싱하는 데 사용됩니다.
 * 
 * @param data 수신된 데이터 배열 (8바이트)
 * @param expected_cmd_code 예상되는 명령 코드 (0xA1~0xA6)
 * 
 * @return 파싱된 모터 상태 2 데이터를 포함하는 StatusOr 객체 또는 오류 상태
 * @retval absl::StatusOr<types::Status2DataV161> 성공 시 파싱된 모터 상태 2 데이터 포함
 * @retval absl::InvalidArgumentError 유효하지 않은 데이터 형식이거나 명령 코드가 일치하지 않을 경우
 * @retval absl::InternalError 내부 파싱 오류 발생 시
 */
absl::StatusOr<types::Status2DataV161> parseClosedLoopResponse(
    const std::array<uint8_t, 8>& data, uint8_t expected_cmd_code);

}  // namespace v161_motor_control::parsing

#endif  // V161_MOTOR_CONTROL__PARSING_V161_HPP
