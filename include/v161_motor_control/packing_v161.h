#ifndef V161_MOTOR_CONTROL__PACKING_V161_HPP
#define V161_MOTOR_CONTROL__PACKING_V161_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include "types_v161.h"

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

// --- Write/Action Command Frame ---
std::array<uint8_t, 8>
createWritePidRamFrame(const types::PidDataV161 &pid_data);
std::array<uint8_t, 8>
createWritePidRomFrame(const types::PidDataV161 &pid_data);
std::array<uint8_t, 8>
createWriteAccelRamFrame(const types::AccelDataV161 &accel_data);
std::array<uint8_t, 8> createWriteEncoderOffsetFrame(uint16_t offset);
std::array<uint8_t, 8> createWritePosAsZeroRomFrame();
std::array<uint8_t, 8> createClearErrorFlagFrame();

// --- Control Command Frame ---
std::array<uint8_t, 8> createMotorOffFrame();
std::array<uint8_t, 8> createMotorStopFrame();
std::array<uint8_t, 8> createMotorRunFrame();

/**
 * @brief Generate Torque Control Command Frame (0xA1)
 * @param torque_setpoint torque current setpoint (-2000 to 2000, corresponding
 * to an actual current of -32A to 32A)
 */
std::array<uint8_t, 8> createTorqueControlFrame(int16_t torque_setpoint);

/**
 * @brief Generate Velocity Control Command Frame (0xA2)
 * @param speed_setpoint Speed setpoint (int32_t, unit: 0.01 dps/LSB)
 */
std::array<uint8_t, 8> createSpeedControlFrame(int32_t speed_setpoint);

/**
 * @brief Generate position control command 1 frame (0xA3, multi-rotation angle)
 * @param angle_setpoint position setpoint (int32_t, unit: 0.01 degree/LSB)
 */
std::array<uint8_t, 8> createPositionControl1Frame(int32_t angle_setpoint);

/**
 * @brief Generate position control command 2 frames (0xA4, multi-rotation
 * angle, velocity limit)
 * @param angle_setpoint position setpoint (int32_t, unit: 0.01 degree/LSB)
 * @param max_speed Maximum speed limit (uint16_t, unit: 1 dps/LSB)
 */
std::array<uint8_t, 8> createPositionControl2Frame(int32_t angle_setpoint,
                                                   uint16_t max_speed);

/**
 * @brief Generate position control command 3 frames (0xA5, single rotation
 * angle, specify orientation)
 * @param angle_setpoint position setpoint (uint16_t, 0~35999, unit: 0.01
 * degree/LSB)
 * @param direction rotation direction (CLOCKWISE or COUNTER_CLOCKWISE)
 */
std::array<uint8_t, 8>
createPositionControl3Frame(uint16_t angle_setpoint,
                            types::SpinDirection direction);

/**
 * @brief Generate position control command 4 frames (0xA6, single rotation
 * angle, orientation, velocity limit)
 * @param angle_setpoint position setpoint (uint16_t, 0~35999, unit: 0.01
 * degree/LSB)
 * @param direction rotation direction (CLOCKWISE or COUNTER_CLOCKWISE)
 * @param max_speed Maximum speed limit (uint16_t, unit: 1 dps/LSB)
 */
std::array<uint8_t, 8>
createPositionControl4Frame(uint16_t angle_setpoint,
                            types::SpinDirection direction, uint16_t max_speed);

// --- Helper function for packing data (will be used more in Write command) ---
template <typename T>
void packLittleEndian(std::array<uint8_t, 8> &data, size_t index, T value);

} // namespace v161_motor_control::packing

#endif // V161_MOTOR_CONTROL__PACKING_V161_HPP
