#include "v161_motor_control/motor_v161.hpp"
#include "v161_motor_control/packing_v161.hpp"
#include "v161_motor_control/parsing_v161.hpp"
#include "v161_motor_control/protocol_v161.hpp"

#include <chrono> // for milliseconds
#include <cstdint>
#include <iostream> // for debugging/error messages
#include <thread>   // for sleep_for

namespace v161_motor_control {

MotorV161::MotorV161(std::shared_ptr<CanInterface> can_interface,
                     uint8_t motor_id)
    : can_interface_(can_interface), motor_id_(motor_id) {
  if (!can_interface_) {
    throw std::invalid_argument("CAN interface pointer is null");
  }

  if (motor_id_ < 1 || motor_id_ > 32) {
    throw std::invalid_argument("Invalid motor ID");
  }

  // Precomputing CAN ID
  request_id_ = protocol::getV161RequestId(motor_id_);
  response_id_ = protocol::getV161ResponseId(motor_id_);

  // Request to register the motor ID with the interface (for filter settings)
  can_interface_->addMotorId(motor_id_);
}

bool MotorV161::sendCommandAndGetResponse(
    const std::array<uint8_t, 8> &command_data,
    uint8_t expected_response_cmd_code,
    std::array<uint8_t, 8> &response_data_out, int retry_count) {
  if (!can_interface_)
    return false;

  for (int i = 0; i <= retry_count; ++i) {
    if (!can_interface_->sendFrame(request_id_, command_data)) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << ": Failed to send command 0x" << std::hex
                << static_cast<int>(command_data[0]) << std::dec << '\n';
      if (i == retry_count)
        return false; // Last attempt failed
      std::this_thread::sleep_for(
          std::chrono::milliseconds(10)); // Retry after a short wait
      continue;
    }

    // Waiting for a response
    if (can_interface_->receiveFrame(response_id_, response_data_out)) {
      // Successfully received a frame with the expected ID
      // Now check the command code within the data
      if (response_data_out[0] == expected_response_cmd_code) {
        return true; // Success: Correct ID and correct command code
      } else {
        // Received correct ID, but unexpected command code
        std::cerr << "Motor " << static_cast<int>(motor_id_)
                  << ": Received unexpected command code 0x" << std::hex
                  << static_cast<int>(response_data_out[0]) << " (expected 0x"
                  << static_cast<int>(expected_response_cmd_code) << ")"
                  << std::dec << '\n';
        // Decide if this constitutes a failure or if we should wait for another
        // frame For now, treat as failure for this attempt return false; // Or
        // continue the loop if retries are left? Let's treat as failure for
        // now.
      }
    } else {
      // receiveFrame returned false: Timeout occurred or received frame with
      // unexpected ID (handled inside receiveFrame)
      if (i < retry_count) {
        // Don't print timeout message here, it's potentially handled in
        // receiveFrame or could be misleading std::cerr << "Motor " <<
        // static_cast<int>(motor_id_)
        //           << ": Receive timeout/failure for cmd 0x" << std::hex
        //           << static_cast<int>(command_data[0]) << std::dec
        //           << ". Retrying (" << i + 1 << "/" << retry_count << ")..."
        //           << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(
            50)); // Wait a little longer before retrying
      } else {
        std::cerr << "Motor " << static_cast<int>(motor_id_)
                  << ": Receive timeout/failure for cmd 0x" << std::hex
                  << static_cast<int>(command_data[0]) << std::dec
                  << ". No more retries after " << retry_count << " attempts."
                  << '\n';
      }
    }
  }
  return false; // All retries failed
}

// --- Read Method Implementation ---
types::PidDataV161 MotorV161::readPid() {
  auto command_data = packing::createReadPidFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_READ_PID,
                                response_data, 1)) { // 1 retry
    try {
      return parsing::parseReadPidResponse(response_data);
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PID response: " << e.what() << '\n';
    }
  }
  return {}; // Returning defaults on failure
}

types::AccelDataV161 MotorV161::readAcceleration() {
  auto command_data = packing::createReadAccelFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_READ_ACCEL,
                                response_data, 1)) {
    try {
      return parsing::parseReadAccelResponse(response_data);
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing Accel response: " << e.what() << '\n';
    }
  }
  return {};
}

types::EncoderDataV161 MotorV161::readEncoder() {
  auto command_data = packing::createReadEncoderFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_READ_ENCODER,
                                response_data, 1)) {
    try {
      return parsing::parseReadEncoderResponse(response_data);
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing Encoder response: " << e.what() << '\n';
    }
  }
  return {};
}

types::MultiTurnAngleV161 MotorV161::readMultiTurnAngle() {
  auto command_data = packing::createReadMultiTurnAngleFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data,
                                protocol::CMD_READ_MULTI_TURN_ANGLE,
                                response_data, 1)) {
    try {
      return parsing::parseReadMultiTurnAngleResponse(response_data);
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing MultiTurnAngle response: " << e.what()
                << '\n';
    }
  }
  return {};
}

types::SingleCircleAngleV161 MotorV161::readSingleCircleAngle() {
  auto command_data = packing::createReadSingleCircleAngleFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data,
                                protocol::CMD_READ_SINGLE_CIRCLE_ANGLE,
                                response_data, 1)) {
    try {
      return parsing::parseReadSingleCircleAngleResponse(response_data);
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing SingleCircleAngle response: " << e.what()
                << '\n';
    }
  }
  return {};
}

types::Status1DataV161 MotorV161::readStatus1() {
  auto command_data = packing::createReadStatus1Frame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_READ_STATUS_1,
                                response_data, 1)) {
    try {
      return parsing::parseReadStatus1Response(response_data);
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing Status1 response: " << e.what() << '\n';
    }
  }
  return {};
}

types::Status2DataV161 MotorV161::readStatus2() {
  auto command_data = packing::createReadStatus2Frame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_READ_STATUS_2,
                                response_data, 2)) {
    try {
      return parsing::parseReadStatus2Response(response_data);
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing Status2 response: " << e.what() << '\n';
    }
  }
  return {};
}

types::Status3DataV161 MotorV161::readStatus3() {
  auto command_data = packing::createReadStatus3Frame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_READ_STATUS_3,
                                response_data, 3)) {
    try {
      return parsing::parseReadStatus3Response(response_data);
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing Status3 response: " << e.what() << '\n';
    }
  }
  return {};
}

// --- Wriet/Action Method Implementation ---
bool MotorV161::writePidToRam(const types::PidDataV161 &pid_data) {
  auto command_data = packing::createWritePidRamFrame(pid_data);
  std::array<uint8_t, 8> response_data;

  // The response of a write command is the same data as the request (Echo)
  if (sendCommandAndGetResponse(command_data, protocol::CMD_WRITE_PID_RAM,
                                response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorV161::writePidToRom(const types::PidDataV161 &pid_data) {
  std::cout << "Warning: Writing PID to ROM (0x32). Frequent writes may affect "
               "chip life"
            << '\n';
  auto command_data = packing::createWritePidRomFrame(pid_data);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_WRITE_PID_ROM,
                                response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorV161::writeAccelerationToRam(const types::AccelDataV161 &accel_data) {
  auto command_data = packing::createWriteAccelRamFrame(accel_data);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_WRITE_ACCEL_RAM,
                                response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorV161::writeEncoderOffset(uint16_t offset,
                                   uint16_t &written_offset_out) {
  auto command_data = packing::createWriteEncoderOffsetFrame(offset);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(
          command_data, protocol::CMD_WRITE_ENCODER_OFFSET, response_data, 1)) {
    try {
      written_offset_out =
          parsing::parseWriteEncoderOffsetResponse(response_data);
      return true;
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing WriteEncoderOffset response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorV161::writePositionAsZero(uint16_t &written_offset_out) {
  std::cout << "Warning: Writing current position as zero to ROM (0x19). "
               "Requires restart. Frequent writes may effect chip life"
            << '\n';
  auto command_data = packing::createWritePosAsZeroRomFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data,
                                protocol::CMD_WRITE_POS_AS_ZERO_ROM,
                                response_data, 1)) {
    try {
      written_offset_out =
          parsing::parseWritePosAsZeroRomResponse(response_data);
      return true;
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing WritePosAsZero response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorV161::clearErrorFlag(types::Status1DataV161 &status_out) {
  auto command_data = packing::createClearErrorFlagFrame();
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_READ_STATUS_1,
                                response_data, 1)) {
    try {
      status_out = parsing::parseClearErrorFlagResponse(response_data);

      if (status_out.error_state_raw = 0) {
        std::cout << "Motor " << static_cast<int>(motor_id_)
                  << ": Error flags cleared Successfully" << '\n';
      } else {
        std::cout << "Motor " << static_cast<int>(motor_id_)
                  << ": Clear error command sent, but errors might still "
                     "persist (Raw Error: 0x"
                  << std::hex << static_cast<int>(status_out.error_state_raw)
                  << std::dec << "). Ensure error condition is resolved"
                  << '\n';
      }
      return true;
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing clearErrorFlag response: " << '\n';
    }
  }
  return false;
}

// --- Motor State Control Method Implementation ---
bool MotorV161::motorOff() {
  auto command_data = packing::createMotorOffFrame();
  std::array<uint8_t, 8> response_data;

  // The response is sent to the Echo
  if (sendCommandAndGetResponse(command_data, protocol::CMD_MOTOR_OFF,
                                response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorV161::motorStop() {
  auto command_data = packing::createMotorStopFrame();
  std::array<uint8_t, 8> response_data;

  // The response is sent to the Echo
  if (sendCommandAndGetResponse(command_data, protocol::CMD_MOTOR_STOP,
                                response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

bool MotorV161::motorRun() {
  auto command_data = packing::createMotorRunFrame();
  std::array<uint8_t, 8> response_data;

  // The response is sent to the Echo
  if (sendCommandAndGetResponse(command_data, protocol::CMD_MOTOR_RUN,
                                response_data, 1)) {
    return response_data == command_data;
  }
  return false;
}

// --- Closed-Loop Control Method Implementation ---
bool MotorV161::setTorqueControl(int16_t torque_setpoint,
                                 types::Status2DataV161 &feedback_out) {
  auto command_data = packing::createTorqueControlFrame(torque_setpoint);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_TORQUE_CONTROL,
                                response_data,
                                0)) { // Control commands run without retries
    try {
      feedback_out = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_TORQUE_CONTROL);

      return true;
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing TorqueControl response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorV161::setSpeedControl(int32_t speed_setpoint,
                                types::Status2DataV161 &feedback_out) {
  auto command_data = packing::createSpeedControlFrame(speed_setpoint);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_SPEED_CONTROL,
                                response_data, 0)) {
    try {
      feedback_out = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_SPEED_CONTROL);

      return true;
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing SpeedControl response: " << e.what() << '\n';
    }
  }
  return false;
}

bool MotorV161::setPositionControl1(int32_t angle_setpoint,
                                    types::Status2DataV161 &feedback_out) {
  auto command_data = packing::createPositionControl1Frame(angle_setpoint);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_POSITION_CONTROL_1,
                                response_data, 0)) {
    try {
      feedback_out = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_POSITION_CONTROL_1);

      return true;
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PositionControl1 response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorV161::setPositionControl2(int32_t angle_setpoint, uint16_t max_speed,
                                    types::Status2DataV161 &feedback_out) {
  auto command_data =
      packing::createPositionControl2Frame(angle_setpoint, max_speed);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_POSITION_CONTROL_2,
                                response_data, 0)) {
    try {
      feedback_out = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_POSITION_CONTROL_2);

      return true;
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PositionControl2 response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorV161::setPositionControl3(uint16_t angle_setpoint,
                                    types::SpinDirection direction,
                                    types::Status2DataV161 &feedback_out) {
  if (angle_setpoint > 35999) {
    std::cerr
        << "Warning: angle_setpoint for PositionControl3 should be 0 ~ 35999"
        << '\n';
    // angle_setpoint = angle_setpoint % 36000; // Or handle as error
  }
  auto command_data =
      packing::createPositionControl3Frame(angle_setpoint, direction);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_POSITION_CONTROL_3,
                                response_data, 0)) {
    try {
      feedback_out = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_POSITION_CONTROL_3);

      return true;
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PositionControl3 response: " << e.what()
                << '\n';
    }
  }
  return false;
}

bool MotorV161::setPositionControl4(uint16_t angle_setpoint,
                                    types::SpinDirection direction,
                                    uint16_t max_speed,
                                    types::Status2DataV161 &feedback_out) {
  if (angle_setpoint > 35999) {
    std::cerr
        << "Warning: angle_setpoint for PositionControl4  should be 0 ~ 35999"
        << '\n';
  }
  auto command_data = packing::createPositionControl4Frame(
      angle_setpoint, direction, max_speed);
  std::array<uint8_t, 8> response_data;

  if (sendCommandAndGetResponse(command_data, protocol::CMD_POSITION_CONTROL_4,
                                response_data, 0)) {
    try {
      feedback_out = parsing::parseClosedLoopResponse(
          response_data, protocol::CMD_POSITION_CONTROL_4);

      return true;
    } catch (const std::exception &e) {
      std::cerr << "Motor " << static_cast<int>(motor_id_)
                << " Error parsing PositionControl4 response: " << e.what()
                << '\n';
    }
  }
  return false;
}

} // namespace v161_motor_control
