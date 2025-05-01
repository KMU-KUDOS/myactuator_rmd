#include "myactuator_rmd/protocol/packing_v161.h"

#include <stdexcept>  // for out_of_range

#include <cstdint>
#include <cstring>  // for memcpy

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "myactuator_rmd/protocol/protocol_v161.h"  // Command code
#include "myactuator_rmd/status_macros.h"  // for RETURN_IF_ERROR

namespace v161_motor_control::packing {

// Template implementation for packing (needs to be visible, often put in header
// or inline)
template <typename T>
absl::Status packLittleEndian(std::array<uint8_t, 8>& data, size_t index, T value) {
  if (index + sizeof(T) > data.size()) {
    return absl::OutOfRangeError(
        absl::StrCat("Packing index out of range. Index: ", index,
                    ", Size: ", sizeof(T), ", Array size: ", data.size()));
  }
  // Assumes the system executing this code is also Little Endian
  // If system architecture differs from CAN bus endianness, byte swapping is
  // needed For PC(x86, x64) which are Little Endian, direct memcpy is usually
  // correct for Little Endian CAN data
  memcpy(&data[index], &value, sizeof(T));
  return absl::OkStatus();
}

// Explicit template instantiation (optional, helps with compile
// times/organization)
template absl::Status packLittleEndian<int8_t>(std::array<uint8_t, 8>& data,
                                       size_t index,
                                       int8_t value);
template absl::Status packLittleEndian<uint8_t>(std::array<uint8_t, 8>& data,
                                        size_t index,
                                        uint8_t value);
template absl::Status packLittleEndian<int16_t>(std::array<uint8_t, 8>& data,
                                        size_t index,
                                        int16_t value);
template absl::Status packLittleEndian<uint16_t>(std::array<uint8_t, 8>& data,
                                         size_t index,
                                         uint16_t value);
template absl::Status packLittleEndian<int32_t>(std::array<uint8_t, 8>& data,
                                        size_t index,
                                        int32_t value);
template absl::Status packLittleEndian<uint32_t>(std::array<uint8_t, 8>& data,
                                         size_t index,
                                         uint32_t value);
template absl::Status packLittleEndian<int64_t>(std::array<uint8_t, 8>& data,
                                        size_t index,
                                        int64_t value);
template absl::Status packLittleEndian<uint64_t>(std::array<uint8_t, 8>& data,
                                         size_t index,
                                         uint64_t value);

// --- Read Command Frame ---
absl::StatusOr<std::array<uint8_t, 8>> createReadPidFrame() {
  return std::array<uint8_t, 8>{protocol::CMD_READ_PID, 0, 0, 0, 0, 0, 0, 0};
}

absl::StatusOr<std::array<uint8_t, 8>> createReadAccelFrame() {
  return std::array<uint8_t, 8>{protocol::CMD_READ_ACCEL, 0, 0, 0, 0, 0, 0, 0};
}

absl::StatusOr<std::array<uint8_t, 8>> createReadEncoderFrame() {
  return std::array<uint8_t, 8>{protocol::CMD_READ_ENCODER, 0, 0, 0, 0, 0, 0, 0};
}

absl::StatusOr<std::array<uint8_t, 8>> createReadMultiTurnAngleFrame() {
  return std::array<uint8_t, 8>{protocol::CMD_READ_MULTI_TURN_ANGLE, 0, 0, 0, 0, 0, 0, 0};
}

absl::StatusOr<std::array<uint8_t, 8>> createReadSingleCircleAngleFrame() {
  return std::array<uint8_t, 8>{protocol::CMD_READ_SINGLE_CIRCLE_ANGLE, 0, 0, 0, 0, 0, 0, 0};
}

absl::StatusOr<std::array<uint8_t, 8>> createReadStatus1Frame() {
  return std::array<uint8_t, 8>{protocol::CMD_READ_STATUS_1, 0, 0, 0, 0, 0, 0, 0};
}

absl::StatusOr<std::array<uint8_t, 8>> createReadStatus2Frame() {
  return std::array<uint8_t, 8>{protocol::CMD_READ_STATUS_2, 0, 0, 0, 0, 0, 0, 0};
}

absl::StatusOr<std::array<uint8_t, 8>> createReadStatus3Frame() {
  return std::array<uint8_t, 8>{protocol::CMD_READ_STATUS_3, 0, 0, 0, 0, 0, 0, 0};
}

// --- Write/Action Command Frame ---
absl::StatusOr<std::array<uint8_t, 8>> createWritePidRamFrame(
    const types::PidDataV161& pid_data) {
  std::array<uint8_t, 8> data = {
      protocol::CMD_WRITE_PID_RAM, 0, 0, 0, 0, 0, 0, 0};

  data[2] = pid_data.anglePidKp;
  data[3] = pid_data.anglePidKi;
  data[4] = pid_data.speedPidKp;
  data[5] = pid_data.speedPidKi;
  data[6] = pid_data.iqPidKp;
  data[7] = pid_data.iqPidKi;

  return data;
}

absl::StatusOr<std::array<uint8_t, 8>> createWritePidRomFrame(
    const types::PidDataV161& pid_data) {
  std::array<uint8_t, 8> data = {
      protocol::CMD_WRITE_PID_ROM, 0, 0, 0, 0, 0, 0, 0};

  data[2] = pid_data.anglePidKp;
  data[3] = pid_data.anglePidKi;
  data[4] = pid_data.speedPidKp;
  data[5] = pid_data.speedPidKi;
  data[6] = pid_data.iqPidKp;
  data[7] = pid_data.iqPidKi;

  return data;
}

absl::StatusOr<std::array<uint8_t, 8>> createWriteAccelRamFrame(
    const types::AccelDataV161& accel_data) {
  std::array<uint8_t, 8> data = {
      protocol::CMD_WRITE_ACCEL_RAM, 0, 0, 0, 0, 0, 0, 0};

  RETURN_IF_ERROR(packLittleEndian<int32_t>(data, 4, accel_data.acceleration));
  return data;
}

absl::StatusOr<std::array<uint8_t, 8>> createWriteEncoderOffsetFrame(uint16_t offset) {
  std::array<uint8_t, 8> data = {
      protocol::CMD_WRITE_ENCODER_OFFSET, 0, 0, 0, 0, 0, 0, 0};
  // Assuming offset is packed at index 6 based on parsing function
  RETURN_IF_ERROR(packLittleEndian<uint16_t>(data, 6, offset));
  return data;
}

absl::StatusOr<std::array<uint8_t, 8>> createClearErrorFlagFrame() {
  return std::array<uint8_t, 8>{protocol::CMD_CLEAR_ERROR, 0, 0, 0, 0, 0, 0, 0};
}

// Added missing function definition
absl::StatusOr<std::array<uint8_t, 8>> createWritePosAsZeroRomFrame() {
  return std::array<uint8_t, 8>{protocol::CMD_WRITE_POS_AS_ZERO_ROM, 0, 0, 0, 0, 0, 0, 0};
}

// --- Control Command Frame ---
absl::StatusOr<std::array<uint8_t, 8>> createMotorOffFrame() {
  return std::array<uint8_t, 8>{protocol::CMD_MOTOR_OFF, 0, 0, 0, 0, 0, 0, 0};
}

absl::StatusOr<std::array<uint8_t, 8>> createMotorStopFrame() {
  return std::array<uint8_t, 8>{protocol::CMD_MOTOR_STOP, 0, 0, 0, 0, 0, 0, 0};
}

absl::StatusOr<std::array<uint8_t, 8>> createMotorRunFrame() {
  return std::array<uint8_t, 8>{protocol::CMD_MOTOR_RUN, 0, 0, 0, 0, 0, 0, 0};
}

absl::StatusOr<std::array<uint8_t, 8>> createTorqueControlFrame(int16_t torque_setpoint) {
  std::array<uint8_t, 8> data = {
      protocol::CMD_TORQUE_CONTROL, 0, 0, 0, 0, 0, 0, 0};
  // torque control value (int16_t) in bytes 4-5
  RETURN_IF_ERROR(packLittleEndian<int16_t>(data, 4, torque_setpoint));
  return data;
}

absl::StatusOr<std::array<uint8_t, 8>> createSpeedControlFrame(int32_t speed_setpoint) {
  std::array<uint8_t, 8> data = {
      protocol::CMD_SPEED_CONTROL, 0, 0, 0, 0, 0, 0, 0};
  // speed control value (int32_t) in bytes 4-7
  RETURN_IF_ERROR(packLittleEndian<int32_t>(data, 4, speed_setpoint));
  return data;
}

absl::StatusOr<std::array<uint8_t, 8>> createPositionControl1Frame(int32_t angle_setpoint) {
  std::array<uint8_t, 8> data = {
      protocol::CMD_POSITION_CONTROL_1, 0, 0, 0, 0, 0, 0, 0};
  // Position control (int32_t) in bytes 4-7
  // Let's follow the byte count: DATA[4] low byte ... DATA[7] high byte
  RETURN_IF_ERROR(packLittleEndian<int32_t>(data, 4, angle_setpoint));
  return data;
}

absl::StatusOr<std::array<uint8_t, 8>> createPositionControl2Frame(
    int32_t angle_setpoint,
    uint16_t max_speed) {  // Add max_speed parameter
  std::array<uint8_t, 8> data = {
      protocol::CMD_POSITION_CONTROL_2, 0, 0, 0, 0, 0, 0, 0};
  // speed limit (uint16_t) in bytes 2-3, position (int32_t) in bytes 4-7
  // Let's follow the description: speed bytes 2,3; position bytes 4,5,6,7
  RETURN_IF_ERROR(packLittleEndian<uint16_t>(data, 2, max_speed));
  RETURN_IF_ERROR(packLittleEndian<int32_t>(data, 4, angle_setpoint));
  return data;
}

absl::StatusOr<std::array<uint8_t, 8>> createPositionControl3Frame(
    uint16_t angle_setpoint, types::SpinDirection direction) {
  std::array<uint8_t, 8> data = {
      protocol::CMD_POSITION_CONTROL_3, 0, 0, 0, 0, 0, 0, 0};
  // Spin Direction (uint8_t) in byte 1, position (uint16_t) in bytes 4-5
  data[1] = static_cast<uint8_t>(direction);
  RETURN_IF_ERROR(packLittleEndian<uint16_t>(data, 4, angle_setpoint));
  return data;
}

absl::StatusOr<std::array<uint8_t, 8>> createPositionControl4Frame(
    uint16_t angle_setpoint,
    types::SpinDirection direction,
    uint16_t max_speed) {
  std::array<uint8_t, 8> data = {
      protocol::CMD_POSITION_CONTROL_4, 0, 0, 0, 0, 0, 0, 0};
  // Spin Direction (uint8_t) in byte 1, speed limit (uint16_t) in bytes 2-3,
  // position (uint16_t) in bytes 4-5
  data[1] = static_cast<uint8_t>(direction);
  
  RETURN_IF_ERROR(packLittleEndian<uint16_t>(data, 2, max_speed));
  RETURN_IF_ERROR(packLittleEndian<uint16_t>(data, 4, angle_setpoint));
  return data;
}

}  // namespace v161_motor_control::packing
