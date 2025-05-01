#include <chrono>   // for std::chrono::milliseconds
#include <iomanip>  // for std::fixed, std::setprecision
#include <iostream>
#include <memory>  // for std::make_shared
#include <thread>  // for std::this_thread::sleep_for
#include <vector>

#include <cstdio>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/motor_registry.h"
#include "myactuator_rmd/protocol/motor_v161.h"

// Helper function to print Status 1 data
void printStatus1(const v161_motor_control::types::Status1DataV161& status) {
  std::cout << "  Temp: " << static_cast<int>(status.temperature) << " C"
            << ", Voltage: " << std::fixed << std::setprecision(1)
            << static_cast<float>(status.voltage) * 0.1f << " V"
            << ", Raw Error: 0x" << std::hex
            << static_cast<int>(status.error_state_raw) << std::dec
            << " (LowVolt: " << status.isVoltageLow()
            << ", OverTemp: " << status.isOverTemperature() << ")" << '\n';
}

// Helper function to print Status 2 data
void printStatus2(const v161_motor_control::types::Status2DataV161& status) {
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
void printStatus3(const v161_motor_control::types::Status3DataV161& status) {
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
    
    // Create a Motor Registry (managed by shared_ptr)
    auto motor_registry = 
        std::make_shared<v161_motor_control::MotorRegistry>();
    
    // Add motor ID to registry
    auto status = motor_registry->addMotorId(motor_id_to_test);
    if (!status.ok()) {
      if (absl::IsAlreadyExists(status)) {
        // This is just informational, we can continue
        std::cout << "Information: " << status.message() << '\n';
      } else {
        // Real error we should stop for
        std::cerr << "Error adding motor ID: " << status.message() << '\n';
        return 1;
      }
    }
    
    // Create Motor with both CAN Interface and Motor Registry
    v161_motor_control::MotorV161 motor(can_interface, motor_registry, motor_id_to_test);

    std::cout << "Successfully initialized motor control for ID "
              << static_cast<int>(motor_id_to_test) << '\n'
              << '\n';

    // Read Motor Command
    std::cout << "--- Reading Motor Data ---" << '\n';

    // Read PID
    auto pid_data_or = motor.readPid();
    if (!pid_data_or.ok()) {
      std::cerr << "Failed to read PID data: " << pid_data_or.status().message() << '\n';
      return 1;
    }
    auto pid_data = pid_data_or.value();
    
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
    auto accel_data_or = motor.readAcceleration();
    if (!accel_data_or.ok()) {
      std::cerr << "Failed to read acceleration data: " << accel_data_or.status().message() << '\n';
      return 1;
    }
    auto accel_data = accel_data_or.value();
    
    std::cout << "[Read Accel (0x33)] Acceleration: " << accel_data.acceleration
              << " dps/s"
              << ")" << '\n'
              << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Encoder
    auto encoder_data_or = motor.readEncoder();
    if (!encoder_data_or.ok()) {
      std::cerr << "Failed to read encoder data: " << encoder_data_or.status().message() << '\n';
      return 1;
    }
    auto encoder_data = encoder_data_or.value();
    
    std::cout << "[Read Encoder (0x90)] Pos: " << encoder_data.position
              << ", Raw: " << encoder_data.raw_position
              << ", Offset: " << encoder_data.offset << ")" << '\n'
              << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Multi-Turn Angle
    auto multi_turn_angle_or = motor.readMultiTurnAngle();
    if (!multi_turn_angle_or.ok()) {
      std::cerr << "Failed to read multi-turn angle: " << multi_turn_angle_or.status().message() << '\n';
      return 1;
    }
    auto multi_turn_angle = multi_turn_angle_or.value();
    
    double angle_deg_multi = static_cast<double>(multi_turn_angle.angle) * 0.01;
    std::cout << "[Read Multi-Turn Angle (0x92)] Raw: "
              << multi_turn_angle.angle << " (" << std::fixed
              << std::setprecision(2) << angle_deg_multi << " deg)" << '\n'
              << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Single-Turn Angle
    auto single_circle_angle_or = motor.readSingleCircleAngle();
    if (!single_circle_angle_or.ok()) {
      std::cerr << "Failed to read single-circle angle: " << single_circle_angle_or.status().message() << '\n';
      return 1;
    }
    auto single_circle_angle = single_circle_angle_or.value();
    
    double angle_deg_single =
        static_cast<double>(single_circle_angle.angle) * 0.01;
    std::cout << "[Read Single Circle Angle (0x94)] Raw: "
              << single_circle_angle.angle << " (" << std::fixed
              << std::setprecision(2) << angle_deg_single << " deg)" << '\n'
              << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Status 1
    auto status1_data_or = motor.readStatus1();
    if (!status1_data_or.ok()) {
      std::cerr << "Failed to read status 1: " << status1_data_or.status().message() << '\n';
      return 1;
    }
    auto status1_data = status1_data_or.value();
    
    std::cout << "[Read Status 1 (0x9A)]" << '\n';
    printStatus1(status1_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Status 2
    auto status2_data_or = motor.readStatus2();
    if (!status2_data_or.ok()) {
      std::cerr << "Failed to read status 2: " << status2_data_or.status().message() << '\n';
      return 1;
    }
    auto status2_data = status2_data_or.value();
    
    std::cout << '\n' << "[Read Status 2 (0x9C)]" << '\n';
    printStatus2(status2_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read Status 3
    auto status3_data_or = motor.readStatus3();
    if (!status3_data_or.ok()) {
      std::cerr << "Failed to read status 3: " << status3_data_or.status().message() << '\n';
      return 1;
    }
    auto status3_data = status3_data_or.value();
    
    std::cout << '\n' << "[Read Status 3 (0x9D)]" << '\n';
    printStatus3(status3_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto initial_status1_or = motor.readStatus1();
    if (!initial_status1_or.ok()) {
      std::cerr << "Failed to read initial status 1: " << initial_status1_or.status().message() << '\n';
      return 1;
    }
    auto initial_status1 = initial_status1_or.value();
    
    printStatus1(initial_status1);

    std::cout << "--- Finished reading ---" << '\n' << '\n';

    auto initial_encoder_or = motor.readEncoder();
    if (!initial_encoder_or.ok()) {
      std::cerr << "Failed to read initial encoder: " << initial_encoder_or.status().message() << '\n';
      return 1;
    }
    auto initial_encoder = initial_encoder_or.value();
    
    std::cout << "[Initial Encoder (0x90)] Pos: " << initial_encoder.position
              << ", Raw: " << initial_encoder.raw_position
              << ", Offset: " << initial_encoder.offset << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Write/Action Motor Command
    std::cout << '\n' << "--- Write/Action Command ---" << '\n';

    // Read the PID value and write it back to RAM (no value change)
    std::cout << '\n' << "Write PID to RAM (0x31)..." << '\n';
    auto current_pid_or = motor.readPid();
    if (!current_pid_or.ok()) {
      std::cerr << "Failed to read current PID: " << current_pid_or.status().message() << '\n';
      return 1;
    }
    auto current_pid = current_pid_or.value();

    status = motor.getConfigurator()->writePidToRam(current_pid);
    if (status.ok()) {
      std::cout << " -> Successfully wrote PID to RAM (echo verified)" << '\n';
    } else {
      std::cerr << " -> Failed to write PID to RAM: " << status.message() << '\n';
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(200));  // Wait after writing

    // Read acceleration value and write it back to RAM (no value change)
    std::cout << "Write Acceleration to RAM (0x34)..." << '\n';
    auto current_accel_or = motor.readAcceleration();
    if (!current_accel_or.ok()) {
      std::cerr << "Failed to read current acceleration: " << current_accel_or.status().message() << '\n';
      return 1;
    }
    auto current_accel = current_accel_or.value();

    status = motor.getConfigurator()->writeAccelerationToRam(current_accel);
    if (status.ok()) {
      std::cout << " -> Successfully wrote Acceleration to RAM (echo verified)"
                << '\n';
    } else {
      std::cerr << " -> Failed to write Acceleration to RAM: " << status.message() << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Attempting to change the encoder offset (e.g. current offset + 0), which
    // can be dangerous depending on the actual motor state! Caution: When
    // connecting real motors, run this test carefully as it can affect motor
    // behavior
    std::cout << '\n' << "Write Encoder Offset (0x91)..." << '\n';
    uint16_t new_offset =
        (initial_encoder.offset + 0) % 16384;  // Stay in scope
    std::cout << " -> Successfully sent write new offset: " << new_offset
              << '\n';

    auto written_offset_or = motor.getConfigurator()->writeEncoderOffset(new_offset);
    if (written_offset_or.ok()) {
      std::cout << " -> Successfully sent Write Encoder Offset command. Motor "
                   "reported written offset: "
                << written_offset_or.value() << '\n';
    } else {
      std::cerr << " -> Failed to send Write Encoder Offset command: " 
                << written_offset_or.status().message() << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Attempt to clear error flag (0x9B)
    std::cout << '\n' << "Clear Error Flag (0x9B)..." << '\n';

    auto status_after_clear_or = motor.clearErrorFlag();
    if (status_after_clear_or.ok()) {
      std::cout
          << " -> Successfully sent Clear Error Flag command. Current Status:"
          << '\n';
      printStatus1(status_after_clear_or.value());
    } else {
      std::cerr << " -> Failed to send Clear Error Flag command or get response: "
                << status_after_clear_or.status().message() << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Save current location as zero (0x19) - Caution! Write ROM, reboot
    // required Be very careful when connecting the actual motor. The zero point
    // will change.
    std::cout << '\n'
              << "Testing Write Position As Zero to ROM (0x19)..." << '\n';

    auto zero_offset_written_or = motor.getConfigurator()->writePositionAsZero();
    if (zero_offset_written_or.ok()) {
      std::cout << " -> Successfully sent Write Position As Zero command. "
                   "Reported new offset: "
                << zero_offset_written_or.value() << '\n';
      std::cout
          << " -> !!! Motor requires restart for this change to effect !!!"
          << '\n';
    } else {
      std::cerr << " -> Failed to send Write Position As Zero command: "
                << zero_offset_written_or.status().message() << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << '\n' << "--- Finished Write/Action Command ---" << '\n';

    // !!! Caution !!!
    // The code below moves the actual motor
    // Make sure the motor is securely fastened and that there are no obstacles
    // around it Power off the motor before testing, and have an emergency stop
    // available during testing
    // !!! Caution !!!

    v161_motor_control::types::Status2DataV161
        feedback;  // For storing feedback

    // --- Check motor status and stop first for safety ---
    std::cout << '\n' << "--- Initializing Motor State ---" << '\n';
    std::cout << "Stopping motor (0x81)..." << '\n';

    status = motor.motorStop();
    if (!status.ok()) {
      std::cerr << "Failed to stop motor initially: " << status.message() << '\n';
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "Turning motor off (0x80) for safety..." << '\n';

    status = motor.motorOff();
    if (!status.ok()) {
      std::cerr << "Failed to turn off motor off: " << status.message() << '\n';
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(500));  // Wait long enough for it to turn off

    // Control Motor Command
    std::cout << '\n'
              << "--- Control Motor Command (Caution: Motor will move!)---"
              << '\n';

    // --- Motor State & Control Tests ---
    std::cout << '\n' << "--- Testing Basic Motor Controls ---" << '\n';
    std::cout << "Testing motor on-off cycle..." << '\n';

    // Ensure motor is on
    status = motor.motorRun();
    if (!status.ok()) {
      std::cerr << "Failed to turn on motor: " << status.message() << '\n';
      return 1;
    }

    // Speed Control
    std::cout << '\n' << "Testing Speed Control (0xA2)..." << '\n';
    int32_t speed_sp_dps100 = 100;  // Using a slow movement for safety (1.0 dps)
    std::cout << "Setting speed: " << speed_sp_dps100 << " (0.01 dps)" << '\n';

    auto feedback_or = motor.setSpeedControl(speed_sp_dps100);
    if (feedback_or.ok()) {
      auto feedback = feedback_or.value();
      std::cout << "  -> Speed command sent. Feedback data:" << '\n';
      printStatus2(feedback);
    } else {
      std::cerr << "  -> Failed to send speed command: " << feedback_or.status().message() << '\n';
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // Let it run for 1 sec
    status = motor.motorStop();
    if (!status.ok()) {
      std::cerr << "Failed to stop motor: " << status.message() << '\n';
    }

    // Position Control with speed limit
    std::cout << '\n' << "Testing Position Control with Speed Limit (0xA4)..." << '\n';
    int32_t angle_sp_deg100 = 10 * 100;  // 10 degrees in 0.01-degree units
    uint16_t max_speed_dps = 10;  // 10 dps maximum speed

    std::cout << "Setting position: " << angle_sp_deg100 / 100.0 << " deg with speed limit " 
              << max_speed_dps << " dps" << '\n';

    feedback_or = motor.setPositionControl2(angle_sp_deg100, max_speed_dps);
    if (feedback_or.ok()) {
      auto feedback = feedback_or.value();
      std::cout << "  -> Position command sent. Feedback data:" << '\n';
      printStatus2(feedback);
    } else {
      std::cerr << "  -> Failed to send position command: " << feedback_or.status().message() << '\n';
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));  // Wait for motion to complete

    // Position control with direction
    std::cout << '\n' << "Testing Position Control with Direction (0xA5)..." << '\n';
    uint16_t angle_sp_single_deg100 = 180 * 100;  // 180 degrees in single turn (0-360)
    uint16_t max_speed_single_dps = 30;  // 30 dps speed

    std::cout << "Setting position: " << angle_sp_single_deg100 / 100.0
              << " deg (single turn) with " 
              << "counter-clockwise direction" << '\n';

    feedback_or = motor.setPositionControl3(angle_sp_single_deg100, 
                                          v161_motor_control::types::SpinDirection::COUNTER_CLOCKWISE);
    if (feedback_or.ok()) {
      auto feedback = feedback_or.value();
      std::cout << "  -> Position + direction command sent. Feedback data:" << '\n';
      printStatus2(feedback);
    } else {
      std::cerr << "  -> Failed to send position + direction command: " << feedback_or.status().message() << '\n';
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));  // Wait for motion to complete

    // Torque Control
    std::cout << '\n' << "Testing Torque Control (0xA1)..." << '\n';
    int16_t torque_sp = 200;  // Small positive torque
    std::cout << "Setting torque: " << torque_sp << '\n';

    feedback_or = motor.setTorqueControl(torque_sp);
    if (feedback_or.ok()) {
      auto feedback = feedback_or.value();
      std::cout << "  -> Torque command sent. Feedback data:" << '\n';
      printStatus2(feedback);
    } else {
      std::cerr << "  -> Failed to send torque command: " << feedback_or.status().message() << '\n';
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));  // Apply torque for 2 sec
    status = motor.motorStop();
    if (!status.ok()) {
      std::cerr << "Failed to stop motor: " << status.message() << '\n';
    }

    std::cout << '\n' << "Turning motor off..." << '\n';
    status = motor.motorOff();
    if (!status.ok()) {
      std::cerr << "Failed to turn off motor: " << status.message() << '\n';
    }

    std::cout << '\n' << "Motor test complete!" << '\n';

  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << '\n';
    return 1;
  }

  std::cout << "Program finished" << '\n';
  return 0;
}
