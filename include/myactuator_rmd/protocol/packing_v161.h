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

// --- Read Command Frame ---
/**
 * @brief Generate Read PID Command Frame (0x30)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadPidFrame();

/**
 * @brief Generate Read Acceleration Command Frame (0x33)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadAccelFrame();

/**
 * @brief Generate Read Encoder Command Frame (0x90)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadEncoderFrame();

/**
 * @brief Generate Read Multi-Turn Angle Command Frame (0x92)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadMultiTurnAngleFrame();

/**
 * @brief Generate Read Single Circle Angle Command Frame (0x94)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadSingleCircleAngleFrame();

/**
 * @brief Generate Read Status 1 Command Frame (0x9A)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadStatus1Frame();

/**
 * @brief Generate Read Status 2 Command Frame (0x9C)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadStatus2Frame();

/**
 * @brief Generate Read Status 3 Command Frame (0x9D)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createReadStatus3Frame();

// --- Write/Action Command Frame ---
/**
 * @brief Generate Write PID to RAM Command Frame (0x31)
 * @param pid_data PID data structure containing PID parameters
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createWritePidRamFrame(
    const types::PidDataV161& pid_data);

/**
 * @brief Generate Write PID to ROM Command Frame (0x32)
 * @param pid_data PID data structure containing PID parameters
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createWritePidRomFrame(
    const types::PidDataV161& pid_data);

/**
 * @brief Generate Write Acceleration to RAM Command Frame (0x34)
 * @param accel_data Acceleration data structure
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createWriteAccelRamFrame(
    const types::AccelDataV161& accel_data);

/**
 * @brief Generate Write Encoder Offset Command Frame (0x91)
 * @param offset Encoder offset value
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createWriteEncoderOffsetFrame(uint16_t offset);

/**
 * @brief Generate Write Current Position as Zero to ROM Command Frame (0x19)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createWritePosAsZeroRomFrame();

/**
 * @brief Generate Clear Error Flag Command Frame (0x9B)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createClearErrorFlagFrame();

// --- Control Command Frame ---
/**
 * @brief Generate Motor Off Command Frame (0x80)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createMotorOffFrame();

/**
 * @brief Generate Motor Stop Command Frame (0x81)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createMotorStopFrame();

/**
 * @brief Generate Motor Run Command Frame (0x88)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createMotorRunFrame();

/**
 * @brief Generate Torque Control Command Frame (0xA1)
 * @param torque_setpoint torque current setpoint (-2000 to 2000, corresponding
 * to an actual current of -32A to 32A)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createTorqueControlFrame(int16_t torque_setpoint);

/**
 * @brief Generate Velocity Control Command Frame (0xA2)
 * @param speed_setpoint Speed setpoint (int32_t, unit: 0.01 dps/LSB)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createSpeedControlFrame(int32_t speed_setpoint);

/**
 * @brief Generate position control command 1 frame (0xA3, multi-rotation angle)
 * @param angle_setpoint position setpoint (int32_t, unit: 0.01 degree/LSB)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createPositionControl1Frame(int32_t angle_setpoint);

/**
 * @brief Generate position control command 2 frames (0xA4, multi-rotation
 * angle, velocity limit)
 * @param angle_setpoint position setpoint (int32_t, unit: 0.01 degree/LSB)
 * @param max_speed Maximum speed limit (uint16_t, unit: 1 dps/LSB)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createPositionControl2Frame(int32_t angle_setpoint,
                                                   uint16_t max_speed);

/**
 * @brief Generate position control command 3 frames (0xA5, single rotation
 * angle, specify orientation)
 * @param angle_setpoint position setpoint (uint16_t, 0~35999, unit: 0.01
 * degree/LSB)
 * @param direction rotation direction (CLOCKWISE or COUNTER_CLOCKWISE)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createPositionControl3Frame(
    uint16_t angle_setpoint, types::SpinDirection direction);

/**
 * @brief Generate position control command 4 frames (0xA6, single rotation
 * angle, orientation, velocity limit)
 * @param angle_setpoint position setpoint (uint16_t, 0~35999, unit: 0.01
 * degree/LSB)
 * @param direction rotation direction (CLOCKWISE or COUNTER_CLOCKWISE)
 * @param max_speed Maximum speed limit (uint16_t, unit: 1 dps/LSB)
 * @return StatusOr containing the 8-byte command frame or an error status
 */
absl::StatusOr<std::array<uint8_t, 8>> createPositionControl4Frame(
    uint16_t angle_setpoint,
    types::SpinDirection direction,
    uint16_t max_speed);

// --- Helper function for packing data (will be used more in Write command) ---
/**
 * @brief Pack a value in little-endian format into a byte array
 * @tparam T Type of the value to pack (typically an integer type)
 * @param data Byte array to pack the value into
 * @param index Starting index in the array where to place the value
 * @param value The value to pack
 * @return Status indicating success or the reason for failure
 */
template <typename T>
absl::Status packLittleEndian(std::array<uint8_t, 8>& data, size_t index, T value);

}  // namespace v161_motor_control::packing

#endif  // V161_MOTOR_CONTROL__PACKING_V161_HPP
