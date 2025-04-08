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

  } catch (const std::exception &e) {
    std::cerr << "An error occurred: " << e.what() << '\n';
    return 1;
  }

  std::cout << "Program finished" << '\n';
  return 0;
}
