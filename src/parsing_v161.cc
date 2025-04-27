#include "v161_motor_control/parsing_v161.h"
#include "v161_motor_control/protocol_v161.h" // Command code

#include <cstdint>
#include <cstring>   // for memcpy
#include <stdexcept> // for out_of_range

namespace v161_motor_control::parsing {

// --- Helper function implementation
template <typename T>
T unpackLittleEndian(const std::array<uint8_t, 8> &data, size_t index) {
  if (index + sizeof(T) > data.size()) {
    throw std::out_of_range(
        "Parsing index out of range. Index: " + std::to_string(index) +
        ", Size: " + std::to_string(sizeof(T)));
  }
  T value{};
  // Assumes the system executing this code is Little Endian
  memcpy(&value, &data[index], sizeof(T));
  return value;
}

// Explicit template instantiation (optional)
template int8_t unpackLittleEndian<int8_t>(const std::array<uint8_t, 8> &data,
                                           size_t index);
template uint8_t unpackLittleEndian<uint8_t>(const std::array<uint8_t, 8> &data,
                                             size_t index);
template int16_t unpackLittleEndian<int16_t>(const std::array<uint8_t, 8> &data,
                                             size_t index);
template uint16_t
unpackLittleEndian<uint16_t>(const std::array<uint8_t, 8> &data, size_t index);
template int32_t unpackLittleEndian<int32_t>(const std::array<uint8_t, 8> &data,
                                             size_t index);
template uint32_t
unpackLittleEndian<uint32_t>(const std::array<uint8_t, 8> &data, size_t index);
template int64_t unpackLittleEndian<int64_t>(const std::array<uint8_t, 8> &data,
                                             size_t index);
template uint64_t
unpackLittleEndian<uint64_t>(const std::array<uint8_t, 8> &data, size_t index);

// --- Read Command Response Parsing Function Implementation ---
types::PidDataV161 parseReadPidResponse(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_READ_PID) {
    throw std::runtime_error("Invalid command for PID response");
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

types::AccelDataV161
parseReadAccelResponse(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_READ_ACCEL) {
    throw std::runtime_error("Invalid command for Accel response");
  }
  types::AccelDataV161 result;

  result.acceleration = unpackLittleEndian<int32_t>(data, 4);

  return result;
}

types::EncoderDataV161
parseReadEncoderResponse(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_READ_ENCODER) {
    throw std::runtime_error("Invalid command code for Encoder response");
  }
  types::EncoderDataV161 result;

  result.position = unpackLittleEndian<uint16_t>(data, 2);
  result.raw_position = unpackLittleEndian<uint16_t>(data, 4);
  result.offset = unpackLittleEndian<uint16_t>(data, 6);

  return result;
}

types::MultiTurnAngleV161
parseReadMultiTurnAngleResponse(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_READ_MULTI_TURN_ANGLE) {
    throw std::runtime_error(
        "Invalid command code for MultiTurnAngle response");
  }
  types::MultiTurnAngleV161 result;

  int64_t angle_raw = 0;
  memcpy(reinterpret_cast<uint8_t *>(&angle_raw), &data[1], 7); // Copy 7 bytes
  // Need sign extension if the highest bit (bit 7 of data[7]) is 1
  if (data[7] & 0x80) {
    // Maunally sign extend by setting the highest byte to 0xFF
    reinterpret_cast<uint8_t *>(&angle_raw)[7] = 0xFF;
  }
  result.angle = angle_raw;
  // Alternative (if it's actually int32_t in DATA[4-7]):
  // result.angle = static_cast<int64_t>(unpackLittleEndian<int32_t>(data, 4));

  return result;
}

types::SingleCircleAngleV161
parseReadSingleCircleAngleResponse(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_READ_SINGLE_CIRCLE_ANGLE) {
    throw std::runtime_error("Invalid command for SingleCircleAngle response");
  }
  types::SingleCircleAngleV161 result;

  result.angle = unpackLittleEndian<uint16_t>(data, 6);

  return result;
}

types::Status1DataV161
parseReadStatus1Response(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_READ_STATUS_1) {
    throw std::runtime_error("Invalid command code for Status1 response");
  }
  types::Status1DataV161 result;

  result.temperature = unpackLittleEndian<int8_t>(data, 1);
  result.voltage = unpackLittleEndian<uint16_t>(data, 3);
  result.error_state_raw = data[7];

  return result;
}

types::Status2DataV161
parseReadStatus2Response(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_READ_STATUS_2) {
    throw std::runtime_error("Invalid command code for Status2 response");
  }
  types::Status2DataV161 result;

  result.temperature = unpackLittleEndian<int8_t>(data, 1);
  result.torque_current = unpackLittleEndian<int16_t>(data, 2);
  result.speed = unpackLittleEndian<int16_t>(data, 4);
  result.encoder_position = unpackLittleEndian<uint16_t>(data, 6);

  return result;
}

types::Status3DataV161
parseReadStatus3Response(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_READ_STATUS_3) {
    throw std::runtime_error("Invalid command code for Status3 response");
  }
  types::Status3DataV161 result;

  result.current_A = unpackLittleEndian<int16_t>(data, 2);
  result.current_B = unpackLittleEndian<int16_t>(data, 4);
  result.current_C = unpackLittleEndian<int16_t>(data, 6);

  return result;
}

// --- Write Command Response Parsing Implementation ---
uint16_t parseWriteEncoderOffsetResponse(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_WRITE_ENCODER_OFFSET) {
    throw std::runtime_error(
        "Invalid command code for WriteEncoderOffset response");
  }
  return unpackLittleEndian<uint16_t>(data, 6);
}

uint16_t parseWritePosAsZeroRomResponse(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_WRITE_POS_AS_ZERO_ROM) {
    throw std::runtime_error(
        "Invalid command code for WritePosAsZeroRom response");
  }
  return unpackLittleEndian<uint16_t>(data, 6);
}

types::Status1DataV161
parseClearErrorFlagResponse(const std::array<uint8_t, 8> &data) {
  if (data[0] != protocol::CMD_READ_STATUS_1) {
    throw std::runtime_error(
        "Invalid command code for CLearErrorFlag response");
  }
  return parseReadStatus1Response(data);
}

// --- Control Command Response Parsing Implementation ---
types::Status2DataV161
parseClosedLoopResponse(const std::array<uint8_t, 8> &data,
                        uint8_t expected_cmd_code) {
  // First, verify that the response command code matches the code you expect
  if (data[0] != expected_cmd_code) {
    throw std::runtime_error(
        "Invalid command code for closed-loop response Expected 0x" +
        std::to_string(expected_cmd_code) + " got 0x" +
        std::to_string(data[0]));
  }
  types::Status2DataV161 result;

  result.temperature = unpackLittleEndian<int8_t>(data, 1);
  result.torque_current = unpackLittleEndian<int16_t>(data, 2);
  result.speed = unpackLittleEndian<int16_t>(data, 4);
  result.encoder_position = unpackLittleEndian<uint32_t>(data, 6);

  return result;
}

} // namespace v161_motor_control::parsing
