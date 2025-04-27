#include "v161_motor_control/can_interface.h"
#include "v161_motor_control/motor_v161.h"

#include <chrono> // for std::chrono::milliseconds
#include <cstdio>
#include <iomanip> // for std::fixed, std::setprecision
#include <iostream>
#include <memory> // for std::make_shared
#include <thread> // for std::this_thread::sleep_for
#include <vector>

// Helper function to print Status 1 data
void printStatus1(const v161_motor_control::types::Status1DataV161 &status) {
  std::cout << "  Temp: " << static_cast<int>(status.temperature) << " C"
            << ", Voltage: " << std::fixed << std::setprecision(1)
            << static_cast<float>(status.voltage) * 0.1f << " V"
            << ", Raw Error: 0x" << std::hex
            << static_cast<int>(status.error_state_raw) << std::dec
            << " (LowVolt: " << status.isVoltageLow()
            << ", OverTemp: " << status.isOverTemperature() << ")" << '\n';
}

// Helper function to print Status 2 data
void printStatus2(const v161_motor_control::types::Status2DataV161 &status) {
  // Calculate torque current in Amps (approximate, depends on motor scaling)
  float torque_amps =
      static_cast<float>(status.torque_current) * (33.0f / 2048.0f);

  std::cout << "  Temp: " << static_cast<int>(status.temperature) << " C"
            << ", TorqueRaw: " << status.torque_current << " (~" << std::fixed
            << std::setprecision(2) << torque_amps << " A)"
            << ", Speed: " << status.speed << " dps"
            << ", Encoder: " << status.encoder_position << '\n';
}

// Helper function to print Status 3 data
void printStatus3(const v161_motor_control::types::Status3DataV161 &status) {
  std::cout << "  Phase A: " << std::fixed << std::setprecision(2)
            << status.getCurrentA() << " A"
            << ", Phase B: " << status.getCurrentB() << " A"
            << ", Phase C: " << status.getCurrentC() << " A" << '\n';
}

int main() {
  std::string can_interface_name = "can12";
  uint8_t motor_id_to_test = 1;

  try {
    // Create a CAN interface (managed by shared_ptr)
    auto can_interface =
        std::make_shared<v161_motor_control::CanInterface>(can_interface_name);
    v161_motor_control::MotorV161 motor(can_interface, motor_id_to_test);

    std::cout << "Successfully initialized motor control for ID "
              << static_cast<int>(motor_id_to_test) << '\n'
              << '\n';

    // Read Motor Command
    std::cout << "--- Reading Motor Data ---" << '\n';

    // Read PID
    auto pid_data = motor.readPid();
    std::cout << "[Read PID (0x30)]"
              << " Angle(Kp:" << static_cast<int>(pid_data.anglePidKp)
              << ", Ki:" << static_cast<int>(pid_data.anglePidKp) << ")"
              << " Speed(Kp:" << static_cast<int>(pid_data.speedPidKp)
              << ", Ki:" << static_cast<int>(pid_data.speedPidKi) << ")"
              << " Torque(Kp:" << static_cast<int>(pid_data.iqPidKp)
              << ", Ki:" << static_cast<int>(pid_data.iqPidKi) << ")" << '\n'
              << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Accel
    auto accel_data = motor.readAcceleration();
    std::cout << "[Read Accel (0x33)] Acceleration: " << accel_data.acceleration
              << " dps/s" << ")" << '\n'
              << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Encoder
    auto encoder_data = motor.readEncoder();
    std::cout << "[Read Encoder (0x90)] Pos: " << encoder_data.position
              << ", Raw: " << encoder_data.raw_position
              << ", Offset: " << encoder_data.offset << ")" << '\n'
              << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Multi-Turn Angle
    auto multi_turn_angle = motor.readMultiTurnAngle();
    double angle_deg_multi = static_cast<double>(multi_turn_angle.angle) * 0.01;
    std::cout << "[Read Multi-Turn Angle (0x92)] Raw: "
              << multi_turn_angle.angle << " (" << std::fixed
              << std::setprecision(2) << angle_deg_multi << " deg)" << '\n'
              << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Single-Turn Angle
    auto single_circle_angle = motor.readSingleCircleAngle();
    double angle_deg_single =
        static_cast<double>(single_circle_angle.angle) * 0.01;
    std::cout << "[Read Single Circle Angle (0x94)] Raw: "
              << single_circle_angle.angle << " (" << std::fixed
              << std::setprecision(2) << angle_deg_single << " deg)" << '\n'
              << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Status 1
    auto status1_data = motor.readStatus1();
    std::cout << "[Read Status 1 (0x9A)]" << '\n';
    printStatus1(status1_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Status 2
    auto status2_data = motor.readStatus2();
    std::cout << '\n' << "[Read Status 2 (0x9C)]" << '\n';
    printStatus2(status2_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Status 3
    auto status3_data = motor.readStatus3();
    std::cout << '\n' << "[Read Status 3 (0x9D)]" << '\n';
    printStatus3(status3_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto initial_status1 = motor.readStatus1();
    printStatus1(initial_status1);

    std::cout << "--- Finished reading ---" << '\n' << '\n';

    auto initial_encoder = motor.readEncoder();
    std::cout << "[Initial Encoder (0x90)] Pos: " << initial_encoder.position
              << ", Raw: " << initial_encoder.raw_position
              << ", Offset: " << initial_encoder.offset << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Write/Action Motor Command
    std::cout << '\n' << "--- Write/Action Command ---" << '\n';

    // Read the PID value and write it back to RAM (no value change)
    std::cout << '\n' << "Write PID to RAM (0x31)..." << '\n';
    auto current_pid = motor.readPid();

    if (motor.writePidToRam(current_pid)) {
      std::cout << " -> Successfully wrote PID to RAM (echo verified)" << '\n';
    } else {
      std::cerr << " -> Failed to write PID to RAM" << '\n';
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(200)); // Wait after writing

    // Read acceleration value and write it back to RAM (no value change)
    std::cout << "Write Acceleration to RAM (0x34)..." << '\n';
    auto current_accel = motor.readAcceleration();

    if (motor.writeAccelerationToRam(current_accel)) {
      std::cout << " -> Successfully wrote Acceleration to RAM (echo verified)"
                << '\n';
    } else {
      std::cerr << " -> Failed to write Acceleration to RAM" << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Attempting to change the encoder offset (e.g. current offset + 0), which
    // can be dangerous depending on the actual motor state! Caution: When
    // connecting real motors, run this test carefully as it can affect motor
    // behavior
    std::cout << '\n' << "Write Encoder Offset (0x91)..." << '\n';
    uint16_t new_offset = (initial_encoder.offset + 0) % 16384; // Stay in scope
    uint16_t written_offset = 0;
    std::cout << " -> Successfully sent write new offset: " << new_offset
              << '\n';

    if (motor.writeEncoderOffset(new_offset, written_offset)) {
      std::cout << " -> Successfully sent Write Encoder Offset command. Motor "
                   "reported written offset: "
                << written_offset << '\n';
    } else {
      std::cerr << " -> Failed to send Write Encoder Offset command" << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Attempt to clear error flag (0x9B)
    std::cout << '\n' << "Clear Error Flag (0x9B)..." << '\n';
    v161_motor_control::types::Status1DataV161 status_after_clear;

    if (motor.clearErrorFlag(status_after_clear)) {
      std::cout
          << " -> Successfully sent Clear Error Flag command. Current Status:"
          << '\n';
      printStatus1(status_after_clear);
    } else {
      std::cerr << " -> Failed to send Clear Error Flag command or get response"
                << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Save current location as zero (0x19) - Caution! Write ROM, reboot
    // required Be very careful when connecting the actual motor. The zero point
    // will change.
    std::cout << '\n'
              << "Testing Write Position As Zero to ROM (0x19)..." << '\n';
    uint16_t zero_offset_written = 0;

    if (motor.writePositionAsZero(zero_offset_written)) {
      std::cout << " -> Successfully sent Write Position As Zero command. "
                   "Reported new offset: "
                << zero_offset_written << '\n';
      std::cout
          << " -> !!! Motor requires restart for this change to effect !!!"
          << '\n';
    } else {
      std::cerr << " -> Failed to send Write Position As Zero command" << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << '\n' << "--- Finished Write/Action Command ---" << '\n';

    // !!! Caution !!!
    // The code below moves the actual motor
    // Make sure the motor is securely fastened and that there are no obstacles
    // around it Power off the motor before testing, and have an emergency stop
    // available during testing
    // !!! Caution !!!

    v161_motor_control::types::Status2DataV161 feedback; // For storing feedback

    // --- Check motor status and stop first for safety ---
    std::cout << '\n' << "--- Initializing Motor State ---" << '\n';
    std::cout << "Stopping motor (0x81)..." << '\n';

    if (!motor.motorStop()) {
      std::cerr << "Failed to stop motor initially" << '\n';
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "Turning motor off (0x80) for safety..." << '\n';

    if (!motor.motorOff()) {
      std::cerr << "Failed to turn off motor off" << '\n';
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(500)); // Wait long enough for it to turn off

    // Control Motor Command
    std::cout << '\n'
              << "--- Control Motor Command (Caution: Motor will move!)---"
              << '\n';

    // Speed control test (e.g. 0 dps)
    std::cout << '\n'
              << "Testing Speed Control (0xA2)... Target: 30.0 dps" << '\n';
    int32_t speed_sp_dps100 = 0 * 100; // 0 dps * 100 (0.01 dps/LSB)

    if (motor.setSpeedControl(speed_sp_dps100, feedback)) {
      std::cout << " -> Speed command sent. Feedback:" << '\n';
      printStatus2(feedback);
    } else {
      std::cerr << " -> Failed to send Speed command" << '\n';
    }
    std::cout << " -> Stopping motor (0x81)..." << '\n';
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << " -> Running at 0 dps for 2 seconds..." << '\n';
    motor.motorStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Position Control Test 2 (Multiple Turns, Speed Limit) - Example: 0 degree
    // movement
    std::cout
        << '\n'
        << "Testing Position Control 2 (0xA4)... Target: 0 deg. Max Speed 0 dps"
        << '\n';
    int32_t angle_sp_deg100 = 0 * 100; // 0 deg * 0 (0.01 deg/LSB)
    uint16_t max_speed_dps = 0;        // 0 dps

    if (motor.setPositionControl2(angle_sp_deg100, max_speed_dps, feedback)) {
      std::cout << " -> Position 2 command sent. Feedback:" << '\n';
      printStatus2(feedback);
    } else {
      std::cerr << " -> Failed to send Position Control 2 command" << '\n';
    }
    std::cout << " -> Moving to 0 deg for 3 seconds (adjust time as needed)..."
              << '\n';
    std::this_thread::sleep_for(std::chrono::seconds(
        3)); // Time to reach the target location (long enough)

    // Position Control Test 4 (Single Turn, Directional, Speed Limit) -
    // Example: 180 degree travel (CCW)
    std::cout << '\n'
              << "Testing Position Control 4 (0xA6)... Target: 0 deg (CCW). "
                 "Max Speed 0 dps"
              << '\n';
    uint16_t angle_sp_single_deg100 = 0 * 100;
    uint16_t max_speed_single_dps = 0;

    if (motor.setPositionControl4(
            angle_sp_single_deg100,
            v161_motor_control::types::SpinDirection::COUNTER_CLOCKWISE,
            max_speed_single_dps, feedback)) {
      std::cout << " -> Position 4 command sent. Feedback:" << '\n';
      printStatus2(feedback);
    } else {
      std::cerr << " -> Failed to send Position Control 4 command" << '\n';
    }
    std::cout
        << " -> Moving to 0 deg (CCW) for 4 seconds (adjust time as needed)..."
        << '\n';
    std::this_thread::sleep_for(std::chrono::seconds(4));

    // Torque Control Test - Caution: Can be dangerous depending on motor load
    // and model! Example: Small torque value (equivalent to 0.1 A) - value
    // needs to be calculated
    int16_t torque_sp =
        static_cast<int16_t>(0.1f * (2000.0f / 32.0f)); // 0.1A -> approx 62
    std::cout << '\n'
              << "Testing Torque Control (0xA1)... Target: " << torque_sp
              << " (~0.1A)" << '\n';

    if (motor.setTorqueControl(torque_sp, feedback)) {
      std::cout << " -> Torque command sent. Feedback:" << '\n';
      printStatus2(feedback);
    } else {
      std::cerr << " -> Failed to send Torque Control command" << '\n';
    }
    std::cout << " -> Applying torque for 1 second..." << '\n';
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << '\n' << "--- Finished Control Command ---" << '\n';

    // --- Stopping and turning off the motor ---
    std::cout << "Stopping motor (0x81)..." << '\n';
    motor.motorStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Turning motor off (0x80)..." << '\n';
    motor.motorOff();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

  } catch (const std::exception &e) {
    std::cerr << "An error occurred: " << e.what() << '\n';
    return 1;
  }

  std::cout << "Program finished" << '\n';
  return 0;
}
