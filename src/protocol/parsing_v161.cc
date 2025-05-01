#include "myactuator_rmd/protocol/parsing_v161.h"

#include <stdexcept>  // for out_of_range

#include <cstdint>
#include <cstring>  // for memcpy

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "myactuator_rmd/protocol/protocol_v161.h"  // Command code

namespace v161_motor_control::parsing {

// --- Helper function implementation
template <typename T>
absl::StatusOr<T> unpackLittleEndian(const std::array<uint8_t, 8>& data, size_t index) {
  if (index + sizeof(T) > data.size()) {
    return absl::OutOfRangeError(
        absl::StrCat("Parsing index out of range. Index: ", index,
                    ", Size: ", sizeof(T), ", Array size: ", data.size()));
  }
  T value{};
  // Assumes the system executing this code is Little Endian
  memcpy(&value, &data[index], sizeof(T));
  return value;
}

// Explicit template instantiation (optional)
template absl::StatusOr<int8_t> unpackLittleEndian<int8_t>(const std::array<uint8_t, 8>& data,
                                           size_t index);
template absl::StatusOr<uint8_t> unpackLittleEndian<uint8_t>(const std::array<uint8_t, 8>& data,
                                             size_t index);
template absl::StatusOr<int16_t> unpackLittleEndian<int16_t>(const std::array<uint8_t, 8>& data,
                                             size_t index);
template absl::StatusOr<uint16_t> unpackLittleEndian<uint16_t>(
    const std::array<uint8_t, 8>& data, size_t index);
template absl::StatusOr<int32_t> unpackLittleEndian<int32_t>(const std::array<uint8_t, 8>& data,
                                             size_t index);
template absl::StatusOr<uint32_t> unpackLittleEndian<uint32_t>(
    const std::array<uint8_t, 8>& data, size_t index);
template absl::StatusOr<int64_t> unpackLittleEndian<int64_t>(const std::array<uint8_t, 8>& data,
                                             size_t index);
template absl::StatusOr<uint64_t> unpackLittleEndian<uint64_t>(
    const std::array<uint8_t, 8>& data, size_t index);

// --- Read Command Response Parsing Function Implementation ---
absl::StatusOr<types::PidDataV161> parseReadPidResponse(const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_READ_PID) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command for PID response. Expected: 0x", 
                   absl::Hex(protocol::CMD_READ_PID),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  types::PidDataV161 result;

  result.anglePidKp = data[2];
  result.anglePidKi = data[3];
  result.speedPidKp = data[4];
  result.speedPidKi = data[5];
  result.iqPidKp = data[6];
  result.iqPidKi = data[7];

  return result;
}

absl::StatusOr<types::AccelDataV161> parseReadAccelResponse(
    const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_READ_ACCEL) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command for Accel response. Expected: 0x", 
                   absl::Hex(protocol::CMD_READ_ACCEL),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  types::AccelDataV161 result;

  auto acceleration_or = unpackLittleEndian<int32_t>(data, 4);
  if (!acceleration_or.ok()) {
    return acceleration_or.status();
  }
  result.acceleration = *acceleration_or;

  return result;
}

absl::StatusOr<types::EncoderDataV161> parseReadEncoderResponse(
    const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_READ_ENCODER) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command code for Encoder response. Expected: 0x",
                   absl::Hex(protocol::CMD_READ_ENCODER),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  types::EncoderDataV161 result;

  auto position_or = unpackLittleEndian<uint16_t>(data, 2);
  if (!position_or.ok()) {
    return position_or.status();
  }
  result.position = *position_or;
  
  auto raw_position_or = unpackLittleEndian<uint16_t>(data, 4);
  if (!raw_position_or.ok()) {
    return raw_position_or.status();
  }
  result.raw_position = *raw_position_or;
  
  auto offset_or = unpackLittleEndian<uint16_t>(data, 6);
  if (!offset_or.ok()) {
    return offset_or.status();
  }
  result.offset = *offset_or;

  return result;
}

absl::StatusOr<types::MultiTurnAngleV161> parseReadMultiTurnAngleResponse(
    const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_READ_MULTI_TURN_ANGLE) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command code for MultiTurnAngle response. Expected: 0x",
                   absl::Hex(protocol::CMD_READ_MULTI_TURN_ANGLE),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  types::MultiTurnAngleV161 result;

  int64_t angle_raw = 0;
  memcpy(reinterpret_cast<uint8_t*>(&angle_raw), &data[1], 7);  // Copy 7 bytes
  // Need sign extension if the highest bit (bit 7 of data[7]) is 1
  if (data[7] & 0x80) {
    // Maunally sign extend by setting the highest byte to 0xFF
    reinterpret_cast<uint8_t*>(&angle_raw)[7] = 0xFF;
  }
  result.angle = angle_raw;
  // Alternative (if it's actually int32_t in DATA[4-7]):
  // auto angle_or = unpackLittleEndian<int32_t>(data, 4);
  // if (!angle_or.ok()) {
  //   return angle_or.status();
  // }
  // result.angle = static_cast<int64_t>(*angle_or);

  return result;
}

absl::StatusOr<types::SingleCircleAngleV161> parseReadSingleCircleAngleResponse(
    const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_READ_SINGLE_CIRCLE_ANGLE) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command for SingleCircleAngle response. Expected: 0x",
                   absl::Hex(protocol::CMD_READ_SINGLE_CIRCLE_ANGLE),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  types::SingleCircleAngleV161 result;

  auto angle_or = unpackLittleEndian<uint16_t>(data, 6);
  if (!angle_or.ok()) {
    return angle_or.status();
  }
  result.angle = *angle_or;

  return result;
}

absl::StatusOr<types::Status1DataV161> parseReadStatus1Response(
    const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_READ_STATUS_1) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command code for Status1 response. Expected: 0x",
                   absl::Hex(protocol::CMD_READ_STATUS_1),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  types::Status1DataV161 result;

  auto temperature_or = unpackLittleEndian<int8_t>(data, 1);
  if (!temperature_or.ok()) {
    return temperature_or.status();
  }
  result.temperature = *temperature_or;
  
  auto voltage_or = unpackLittleEndian<uint16_t>(data, 3);
  if (!voltage_or.ok()) {
    return voltage_or.status();
  }
  result.voltage = *voltage_or;
  
  result.error_state_raw = data[7];

  return result;
}

absl::StatusOr<types::Status2DataV161> parseReadStatus2Response(
    const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_READ_STATUS_2) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command code for Status2 response. Expected: 0x",
                   absl::Hex(protocol::CMD_READ_STATUS_2),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  types::Status2DataV161 result;

  auto temperature_or = unpackLittleEndian<int8_t>(data, 1);
  if (!temperature_or.ok()) {
    return temperature_or.status();
  }
  result.temperature = *temperature_or;
  
  auto torque_current_or = unpackLittleEndian<int16_t>(data, 2);
  if (!torque_current_or.ok()) {
    return torque_current_or.status();
  }
  result.torque_current = *torque_current_or;
  
  auto speed_or = unpackLittleEndian<int16_t>(data, 4);
  if (!speed_or.ok()) {
    return speed_or.status();
  }
  result.speed = *speed_or;
  
  auto encoder_position_or = unpackLittleEndian<uint16_t>(data, 6);
  if (!encoder_position_or.ok()) {
    return encoder_position_or.status();
  }
  result.encoder_position = *encoder_position_or;

  return result;
}

absl::StatusOr<types::Status3DataV161> parseReadStatus3Response(
    const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_READ_STATUS_3) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command code for Status3 response. Expected: 0x",
                   absl::Hex(protocol::CMD_READ_STATUS_3),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  types::Status3DataV161 result;

  auto current_a_or = unpackLittleEndian<int16_t>(data, 2);
  if (!current_a_or.ok()) {
    return current_a_or.status();
  }
  result.current_A = *current_a_or;
  
  auto current_b_or = unpackLittleEndian<int16_t>(data, 4);
  if (!current_b_or.ok()) {
    return current_b_or.status();
  }
  result.current_B = *current_b_or;
  
  auto current_c_or = unpackLittleEndian<int16_t>(data, 6);
  if (!current_c_or.ok()) {
    return current_c_or.status();
  }
  result.current_C = *current_c_or;

  return result;
}

// --- Write Command Response Parsing Implementation ---
absl::StatusOr<uint16_t> parseWriteEncoderOffsetResponse(const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_WRITE_ENCODER_OFFSET) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command code for WriteEncoderOffset response. Expected: 0x",
                   absl::Hex(protocol::CMD_WRITE_ENCODER_OFFSET),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  
  auto offset_or = unpackLittleEndian<uint16_t>(data, 6);
  if (!offset_or.ok()) {
    return offset_or.status();
  }
  return *offset_or;
}

absl::StatusOr<uint16_t> parseWritePosAsZeroRomResponse(const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_WRITE_POS_AS_ZERO_ROM) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command code for WritePosAsZeroRom response. Expected: 0x",
                   absl::Hex(protocol::CMD_WRITE_POS_AS_ZERO_ROM),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  
  auto offset_or = unpackLittleEndian<uint16_t>(data, 6);
  if (!offset_or.ok()) {
    return offset_or.status();
  }
  return *offset_or;
}

absl::StatusOr<types::Status1DataV161> parseClearErrorFlagResponse(
    const std::array<uint8_t, 8>& data) {
  if (data[0] != protocol::CMD_CLEAR_ERROR) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command code for ClearErrorFlag response. Expected: 0x",
                   absl::Hex(protocol::CMD_CLEAR_ERROR),
                   ", Got: 0x", absl::Hex(data[0])));
  }

  // 직접 Status1 데이터를 파싱합니다 (parseReadStatus1Response의 내부 구현과
  // 동일)
  types::Status1DataV161 result;
  
  auto temperature_or = unpackLittleEndian<int8_t>(data, 1);
  if (!temperature_or.ok()) {
    return temperature_or.status();
  }
  result.temperature = *temperature_or;
  
  auto voltage_or = unpackLittleEndian<uint16_t>(data, 3);
  if (!voltage_or.ok()) {
    return voltage_or.status();
  }
  result.voltage = *voltage_or;
  
  result.error_state_raw = data[7];

  return result;
}

// --- Control Command Response Parsing Implementation ---
absl::StatusOr<types::Status2DataV161> parseClosedLoopResponse(
    const std::array<uint8_t, 8>& data, uint8_t expected_cmd_code) {
  // First, verify that the response command code matches the code you expect
  if (data[0] != expected_cmd_code) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid command code for closed-loop response. Expected: 0x",
                   absl::Hex(expected_cmd_code),
                   ", Got: 0x", absl::Hex(data[0])));
  }
  types::Status2DataV161 result;

  auto temperature_or = unpackLittleEndian<int8_t>(data, 1);
  if (!temperature_or.ok()) {
    return temperature_or.status();
  }
  result.temperature = *temperature_or;
  
  auto torque_current_or = unpackLittleEndian<int16_t>(data, 2);
  if (!torque_current_or.ok()) {
    return torque_current_or.status();
  }
  result.torque_current = *torque_current_or;
  
  auto speed_or = unpackLittleEndian<int16_t>(data, 4);
  if (!speed_or.ok()) {
    return speed_or.status();
  }
  result.speed = *speed_or;
  
  auto encoder_position_or = unpackLittleEndian<uint32_t>(data, 6);
  if (!encoder_position_or.ok()) {
    return encoder_position_or.status();
  }
  result.encoder_position = *encoder_position_or;

  return result;
}

}  // namespace v161_motor_control::parsing
