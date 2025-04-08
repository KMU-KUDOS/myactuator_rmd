#ifndef V161_MOTOR_CONTROL__PACKING_V161_HPP
#define V161_MOTOR_CONTROL__PACKING_V161_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include "types_v161.hpp"

namespace v161_motor_control::packing {

// --- Read Command Frame ---
std::array<uint8_t, 8> createReadPidFrame();
std::array<uint8_t, 8> createReadAccelFrame();
std::array<uint8_t, 8> createReadEncoderFrame();
std::array<uint8_t, 8> createReadMultiTurnAngleFrame();
std::array<uint8_t, 8> createReadSingleCircleAngleFrame();
std::array<uint8_t, 8> createReadStatus1Frame();
std::array<uint8_t, 8> createReadStatus2Frame();
std::array<uint8_t, 8> createReadStatus3Frame();

// --- Helper function for packing data (will be used more in Write command) ---
template <typename T>
void packLittleEndian(std::array<uint8_t, 8> &data, size_t index, T value);

} // namespace v161_motor_control::packing

#endif // V161_MOTOR_CONTROL__PACKING_V161_HPP
