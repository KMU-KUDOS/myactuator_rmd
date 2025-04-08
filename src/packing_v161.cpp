#include "v161_motor_control/packing_v161.hpp"
#include "v161_motor_control/protocol_v161.hpp" // Command code

#include <cstdint>
#include <cstring>   // for memcpy
#include <stdexcept> // for out_of_range

namespace v161_motor_control::packing {

// Template implementation for packing (needs to be visible, often put in header
// or inline)
template <typename T>
void packLittleEndian(std::array<uint8_t, 8> &data, size_t index, T value) {
  if (index + sizeof(T) > data.size()) {
    throw std::out_of_range("Packing index out of range");
  }
  // Assumes the system executing this code is also Little Endian
  // If system architecture differs from CAN bus endianness, byte swapping is
  // needed For PC(x86, x64) which are Little Endian, direct memcpy is usually
  // correct for Little Endian CAN data
  memcpy(&data[index], &value, sizeof(T));
}

// Explicit template instantiation (optional, helps with compile
// times/organization)
template void packLittleEndian<int8_t>(std::array<uint8_t, 8> &data,
                                       size_t index, int8_t value);
template void packLittleEndian<uint8_t>(std::array<uint8_t, 8> &data,
                                        size_t index, uint8_t value);
template void packLittleEndian<int16_t>(std::array<uint8_t, 8> &data,
                                        size_t index, int16_t value);
template void packLittleEndian<uint16_t>(std::array<uint8_t, 8> &data,
                                         size_t index, uint16_t value);
template void packLittleEndian<int32_t>(std::array<uint8_t, 8> &data,
                                        size_t index, int32_t value);
template void packLittleEndian<uint32_t>(std::array<uint8_t, 8> &data,
                                         size_t index, uint32_t value);
template void packLittleEndian<int64_t>(std::array<uint8_t, 8> &data,
                                        size_t index, int64_t value);
template void packLittleEndian<uint64_t>(std::array<uint8_t, 8> &data,
                                         size_t index, uint64_t value);

// --- Read Command Frame ---
std::array<uint8_t, 8> createReadPidFrame() {
  return {protocol::CMD_READ_PID, 0, 0, 0, 0, 0, 0, 0};
}

std::array<uint8_t, 8> createReadAccelFrame() {
  return {protocol::CMD_READ_ACCEL, 0, 0, 0, 0, 0, 0, 0};
}

std::array<uint8_t, 8> createReadEncoderFrame() {
  return {protocol::CMD_READ_ENCODER, 0, 0, 0, 0, 0, 0, 0};
}

std::array<uint8_t, 8> createReadMultiTurnAngleFrame() {
  return {protocol::CMD_READ_MULTI_TURN_ANGLE, 0, 0, 0, 0, 0, 0, 0};
}

std::array<uint8_t, 8> createReadSingleCircleAngleFrame() {
  return {protocol::CMD_READ_SINGLE_CIRCLE_ANGLE, 0, 0, 0, 0, 0, 0, 0};
}

std::array<uint8_t, 8> createReadStatus1Frame() {
  return {protocol::CMD_READ_STATUS_1, 0, 0, 0, 0, 0, 0, 0};
}

std::array<uint8_t, 8> createReadStatus2Frame() {
  return {protocol::CMD_READ_STATUS_2, 0, 0, 0, 0, 0, 0, 0};
}

std::array<uint8_t, 8> createReadStatus3Frame() {
  return {protocol::CMD_READ_STATUS_3, 0, 0, 0, 0, 0, 0, 0};
}

} // namespace v161_motor_control::packing
