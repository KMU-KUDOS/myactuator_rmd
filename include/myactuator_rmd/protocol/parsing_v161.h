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

// --- Helper function for unpacking data (Little Endian)
/**
 * @brief Unpack a value in little-endian format from a byte array
 * @tparam T Type of the value to unpack (typically an integer type)
 * @param data Byte array to unpack the value from
 * @param index Starting index in the array where the value is stored
 * @return StatusOr containing the unpacked value or an error status
 */
template <typename T>
absl::StatusOr<T> unpackLittleEndian(const std::array<uint8_t, 8>& data, size_t index);

// --- Read Command Response Parsing Function Declaration
types::PidDataV161 parseReadPidResponse(const std::array<uint8_t, 8>& data);
types::AccelDataV161 parseReadAccelResponse(const std::array<uint8_t, 8>& data);
types::EncoderDataV161 parseReadEncoderResponse(
    const std::array<uint8_t, 8>& data);
types::MultiTurnAngleV161 parseReadMultiTurnAngleResponse(
    const std::array<uint8_t, 8>& data);
types::SingleCircleAngleV161 parseReadSingleCircleAngleResponse(
    const std::array<uint8_t, 8>& data);
types::Status1DataV161 parseReadStatus1Response(
    const std::array<uint8_t, 8>& data);
types::Status2DataV161 parseReadStatus2Response(
    const std::array<uint8_t, 8>& data);
types::Status3DataV161 parseReadStatus3Response(
    const std::array<uint8_t, 8>& data);

// --- Write/Action Command Response Parsing Function Declaration ---
/**
 * @brief 0x91: Parse the encoder offset value set in the response
 * @param data: Array of received data
 * @return Parsed offset value (uint16_t)
 */
uint16_t parseWriteEncoderOffsetResponse(const std::array<uint8_t, 8>& data);

/**
 * @brief 0x19: Parse the encoder offset value set in the response
 * @param data: Array of received data
 * @return Parsed offset value (uint16_t)
 */
uint16_t parseWritePosAsZeroRomResponse(const std::array<uint8_t, 8>& data);

/**
 * @brief 0x9B: Parses the response (same format as Status 1).
 * @param data: Array of received data
 * @return Parsed Status1 data
 */
types::Status1DataV161 parseClearErrorFlagResponse(
    const std::array<uint8_t, 8>& data);

// --- Control Command Response Parsing Function Declaration ---
/**
 * @brief Parses responses from closed loop control instructions (0xA1 to 0xA6)
 * (same as Status 2)
 * @param data: Array of received data
 * @param expected_cmd_code: Command codes to expect in the response (0xA1
 * through 0xA6)
 * @return Parsed Status2 data
 */
types::Status2DataV161 parseClosedLoopResponse(
    const std::array<uint8_t, 8>& data, uint8_t expected_cmd_code);

}  // namespace v161_motor_control::parsing

#endif  // V161_MOTOR_CONTROL__PACKING_V161_HPP
