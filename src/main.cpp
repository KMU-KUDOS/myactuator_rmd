#include "v161_motor_control/can_interface.hpp"
#include "v161_motor_control/motor_v161.hpp"

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

    // --- Read Command ---
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

    std::cout << "--- Finished reading ---" << '\n' << '\n';

    // --- Write/Action Command ---
    std::cout << '\n' << "--- Write/Action Motor ---" << '\n';

    // Read the PID value and write it back to RAM (no value change)
    std::cout << "Write PID to RAM (0x31)..." << '\n';
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
    auto current_accel = motor.readAcceleration(); // Fix variable redeclaration

    if (motor.writeAccelerationToRam(
            current_accel)) { // Use the correct variable name
      std::cout << " -> Successfully wrote Acceleration to RAM (echo verified)"
                << '\n';
    } else {
      std::cerr << " -> Failed to write Acceleration to RAM" << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Attempting to change the encoder offset (e.g. current offset + 0), which
    // can be dangerous depending on the actual motor state! Caution: When
    // connecting real motors, run this test carefully as it can affect the
    // behavior of the motors.
    std::cout << "Write Encoder Offset (0x91)..." << '\n';
    uint16_t new_offset = (encoder_data.offset + 0) %
                          16384; // Use the correct variable name 'encoder_data'
    uint16_t written_offset = 0;
    std::cout << " -> Attempting to write new offset: " << new_offset << '\n';

    if (motor.writeEncoderOffset(new_offset, written_offset)) {
      std::cout << " -> Successfully sent Write ENcoder Offset command. Motor "
                   "reported written offset: "
                << written_offset << '\n';

      // Value verification (is the value received in response the same as the
      // value sent)
      if (new_offset == written_offset) {
        std::cout << " -> Written offset matches requested offset" << '\n';
      } else {
        std::cout << " -> Warning: Written offset (" << written_offset
                  << ") does not match requested offset (" << new_offset << ")"
                  << '\n';
      }
    } else {
      std::cerr << " -> Failed to send Write Encoder Offset command" << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Attempt to clear error flag (0x9B)
    std::cout << "Clear Error Flag (0x9B)..." << '\n';
    v161_motor_control::types::Status1DataV161
        status_after_clear; // Declare variable of the correct type
    if (motor.clearErrorFlag(status_after_clear)) {
      std::cout
          << " -> Successfully sent Clear Error Flag command. Current status:"
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
    std::cout << "Write Position As Zero to ROM (0x19)..." << '\n';
    uint16_t zero_offset_written = 0;

    if (motor.writePositionAsZero(zero_offset_written)) {
      std::cout << " -> Successfully sent Write Position As Zero command. "
                   "Reported new offset: "
                << zero_offset_written << '\n';
      std::cout
          << " -> !!! Motor requires restart for this change to take effect !!!"
          << '\n';
    } else {
      std::cerr << " -> Failed to send Write Position As Zero command" << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "--- Finished Write/Action ---" << '\n';

  } catch (const std::exception &e) {
    std::cerr << "An error occurred: " << e.what() << '\n';
    return 1;
  }

  std::cout << "Program finished" << '\n';
  return 0;
}
