#include <chrono>   // for std::chrono::milliseconds
#include <iomanip>  // for std::fixed, std::setprecision
#include <iostream>
#include <memory>  // for std::make_shared
#include <thread>  // for std::this_thread::sleep_for
#include <vector>
#include <cstdio>
#include <functional>  // for std::function

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

// Helper function to check absl::Status
// Returns true if successful, false and prints error message if failed
bool checkStatus(const absl::Status& status, const std::string& operation) {
  if (!status.ok()) {
    std::cerr << "Error occurred (" << operation << "): " << status.message() << '\n';
    if (absl::IsInvalidArgument(status)) {
      std::cerr << "  -> Invalid argument passed" << '\n';
    } else if (absl::IsUnavailable(status)) {
      std::cerr << "  -> CAN communication error occurred. Check connection" << '\n';
    } else if (absl::IsInternal(status)) {
      std::cerr << "  -> Internal processing error occurred" << '\n';
    }
    return false;
  }
  return true;
}

// Helper function to check absl::StatusOr<T>
// Returns true if successful and stores value in out_value, false and prints error message if failed
template <typename T>
bool checkStatusOr(const absl::StatusOr<T>& status_or, T& out_value, 
                  const std::string& operation) {
  if (!status_or.ok()) {
    std::cerr << "Error occurred (" << operation << "): " << status_or.status().message() << '\n';
    if (absl::IsInvalidArgument(status_or.status())) {
      std::cerr << "  -> Invalid argument passed" << '\n';
    } else if (absl::IsUnavailable(status_or.status())) {
      std::cerr << "  -> CAN communication error occurred. Check connection" << '\n';
    } else if (absl::IsInternal(status_or.status())) {
      std::cerr << "  -> Internal processing error occurred" << '\n';
    }
    return false;
  }
  out_value = status_or.value();
  return true;
}

// Single motor control demo
bool runSingleMotorDemo(v161_motor_control::MotorV161& motor) {
  std::cout << "\n--- Single Motor Basic Function Demo ---\n";
  
  // Read motor data
  std::cout << "\n[1] Read Motor Status Information\n";
  {
    v161_motor_control::types::Status1DataV161 status1_data;
    if (!checkStatusOr(motor.readStatus1(), status1_data, "Read Status 1")) {
      return false;
    }
    
    std::cout << "Motor Status 1 Data:\n";
    printStatus1(status1_data);
    
    v161_motor_control::types::Status2DataV161 status2_data;
    if (!checkStatusOr(motor.readStatus2(), status2_data, "Read Status 2")) {
      return false;
    }
    
    std::cout << "Motor Status 2 Data:\n";
    printStatus2(status2_data);
    
    v161_motor_control::types::MultiTurnAngleV161 angle_data;
    if (!checkStatusOr(motor.readMultiTurnAngle(), angle_data, "Read Multi-Turn Angle")) {
      return false;
    }
    
    double angle_deg = static_cast<double>(angle_data.angle) * 0.01;
    std::cout << "Current Motor Angle: " << std::fixed << std::setprecision(2) 
              << angle_deg << " deg\n";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  // Basic motor control
  std::cout << "\n[2] Basic Motor Control\n";
  {
    // Stop motor (for safety)
    std::cout << "Stopping motor..." << '\n';
    if (!checkStatus(motor.motorStop(), "Stop Motor")) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Turn on motor
    std::cout << "Turning motor on..." << '\n';
    if (!checkStatus(motor.motorRun(), "Turn Motor On")) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Speed control test
    std::cout << "Speed Control Test (10 dps)...\n";
    v161_motor_control::types::Status2DataV161 feedback;
    if (!checkStatusOr(motor.setSpeedControl(1000), feedback, "Speed Control")) {
      return false;
    }
    std::cout << "Speed Control Feedback:\n";
    printStatus2(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Torque control test
    std::cout << "Torque Control Test (5% of max torque)...\n";
    if (!checkStatusOr(motor.setTorqueControl(100), feedback, "Torque Control")) {
      return false;
    }
    std::cout << "Torque Control Feedback:\n";
    printStatus2(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Stop motor
    std::cout << "Stopping motor..." << '\n';
    if (!checkStatus(motor.motorStop(), "Stop Motor")) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  
  // Position control test
  std::cout << "\n[3] Position Control Test\n";
  {
    // Absolute position movement (with speed limit)
    std::cout << "Multi-Turn Position Control Test (10 degrees, max speed: 20 dps)...\n";
    v161_motor_control::types::Status2DataV161 feedback;
    
    if (!checkStatusOr(motor.setPositionControl2(1000, 20), feedback, "Position Control")) {
      return false;
    }
    std::cout << "Position Control Feedback:\n";
    printStatus2(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    // Position control with direction
    std::cout << "Position Control with Direction Test (180 degrees, clockwise)...\n";
    if (!checkStatusOr(motor.setPositionControl3(18000, 
        v161_motor_control::types::SpinDirection::CLOCKWISE), feedback, "Position Control with Direction")) {
      return false;
    }
    std::cout << "Position Control with Direction Feedback:\n";
    printStatus2(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    // Stop motor
    std::cout << "Stopping motor..." << '\n';
    if (!checkStatus(motor.motorStop(), "Stop Motor")) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  
  // Motor configuration test
  std::cout << "\n[4] Motor Configuration Test\n";
  {
    // Read PID
    v161_motor_control::types::PidDataV161 pid_data;
    if (!checkStatusOr(motor.readPid(), pid_data, "Read PID")) {
      return false;
    }
    
    std::cout << "Current PID Configuration: "
              << " Angle(Kp:" << static_cast<int>(pid_data.anglePidKp)
              << ", Ki:" << static_cast<int>(pid_data.anglePidKi) << ")"
              << " Speed(Kp:" << static_cast<int>(pid_data.speedPidKp)
              << ", Ki:" << static_cast<int>(pid_data.speedPidKi) << ")"
              << " Torque(Kp:" << static_cast<int>(pid_data.iqPidKp)
              << ", Ki:" << static_cast<int>(pid_data.iqPidKi) << ")\n";
    
    // Write PID to RAM (no value change)
    std::cout << "Writing PID to RAM..." << '\n';
    if (!checkStatus(motor.getConfigurator()->writePidToRam(pid_data), "Write PID to RAM")) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  
  // Motor shutdown
  std::cout << "\n[5] Motor Shutdown\n";
  {
    std::cout << "Turning motor off..." << '\n';
    if (!checkStatus(motor.motorOff(), "Turn Motor Off")) {
      return false;
    }
  }
  
  std::cout << "\nSingle Motor Basic Function Demo Completed!\n";
  return true;
}

// Multiple motor control demo
bool runMultipleMotorsDemo(std::vector<std::shared_ptr<v161_motor_control::MotorV161>>& motors) {
  if (motors.empty()) {
    std::cerr << "No motors found.\n";
    return false;
  }
  
  std::cout << "\n--- Multiple Motor Control Demo ---\n";
  std::cout << motors.size() << " motors to control.\n";
  
  // Read all motors' status information
  std::cout << "\n[1] Read All Motors' Status Information\n";
  for (size_t i = 0; i < motors.size(); ++i) {
    std::cout << "Motor #" << i+1 << " Status:\n";
    
    v161_motor_control::types::Status1DataV161 status1_data;
    if (!checkStatusOr(motors[i]->readStatus1(), status1_data, "Read Motor #" + std::to_string(i+1) + " Status 1")) {
      continue;  // Continue even if this motor fails
    }
    printStatus1(status1_data);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  // Stop all motors
  std::cout << "\n[2] Stop All Motors\n";
  for (size_t i = 0; i < motors.size(); ++i) {
    std::cout << "Stopping Motor #" << i+1 << "..." << '\n';
    if (!checkStatus(motors[i]->motorStop(), "Stop Motor #" + std::to_string(i+1))) {
      continue;  // Continue even if this motor fails
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  // Prepare all motors
  std::cout << "\n[3] Turn On All Motors\n";
  for (size_t i = 0; i < motors.size(); ++i) {
    std::cout << "Turning On Motor #" << i+1 << "..." << '\n';
    if (!checkStatus(motors[i]->motorRun(), "Turn Motor #" + std::to_string(i+1) + " On")) {
      continue;  // Continue even if this motor fails
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  // All motors rotate at the same speed
  std::cout << "\n[4] Rotate All Motors at the Same Speed\n";
  for (size_t i = 0; i < motors.size(); ++i) {
    std::cout << "Setting Motor #" << i+1 << " Speed (20 dps)...\n";
    v161_motor_control::types::Status2DataV161 feedback;
    if (!checkStatusOr(motors[i]->setSpeedControl(2000), feedback, 
                      "Set Motor #" + std::to_string(i+1) + " Speed Control")) {
      continue;  // Continue even if this motor fails
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  // Wait for all motors to move
  std::cout << "All motors are rotating...\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  
  // Stop all motors
  std::cout << "\n[5] Stop All Motors\n";
  for (size_t i = 0; i < motors.size(); ++i) {
    std::cout << "Stopping Motor #" << i+1 << "..." << '\n';
    if (!checkStatus(motors[i]->motorStop(), "Stop Motor #" + std::to_string(i+1))) {
      continue;  // Continue even if this motor fails
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  // All motors move to different positions simultaneously
  std::cout << "\n[6] Move All Motors Simultaneously to Different Positions\n";
  {
    // Prepare commands for all motors
    std::vector<std::function<void()>> position_commands;
    for (size_t i = 0; i < motors.size(); ++i) {
      // Different positions for each motor (30 degrees * (i+1))
      int32_t target_angle = static_cast<int32_t>(3000 * (i + 1));
      
      position_commands.push_back([&motors, i, target_angle]() {
        std::cout << "Setting Motor #" << i+1 << " Position (" << target_angle / 100.0 << " degrees)...\n";
        v161_motor_control::types::Status2DataV161 feedback;
        if (!checkStatusOr(motors[i]->setPositionControl2(target_angle, 50), feedback,
                          "Set Motor #" + std::to_string(i+1) + " Position Control")) {
          return;  // Continue even if this motor fails
        }
      });
    }
    
    // Execute all motor commands at once
    for (auto& cmd : position_commands) {
      cmd();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Very short delay
    }
  }
  
  // Wait for position movement to complete
  std::cout << "All motors are moving to specified positions...\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  
  // Shutdown all motors
  std::cout << "\n[7] Shutdown All Motors\n";
  for (size_t i = 0; i < motors.size(); ++i) {
    std::cout << "Turning Off Motor #" << i+1 << "..." << '\n';
    if (!checkStatus(motors[i]->motorOff(), "Turn Motor #" + std::to_string(i+1) + " Off")) {
      continue;  // Continue even if this motor fails
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  std::cout << "\nMultiple Motor Control Demo Completed!\n";
  return true;
}

// Error handling demo
void errorHandlingDemo() {
  std::cout << "\n--- Error Handling Demo ---\n";
  
  std::cout << "This demo shows error handling patterns using absl::Status and absl::StatusOr.\n";
  
  // Attempt to use a non-existent CAN interface
  std::cout << "\n[1] Attempt to Use a Non-Existent CAN Interface\n";
  try {
    std::cout << "Attempting to connect to non-existent CAN interface 'invalid_can'...\n";
    auto can_interface = std::make_shared<v161_motor_control::CanInterface>("invalid_can");
    
    // This part should not be reached (exception expected)
    std::cout << "CAN interface creation succeeded (should not be reached)\n";
  } catch (const std::exception& e) {
    std::cout << "Expected exception: " << e.what() << "\n";
    std::cout << "This is because the current implementation throws an exception if CAN interface creation fails.\n";
  }
  
  // Attempt to add a wrong ID to the motor registry
  std::cout << "\n[2] Attempt to Add a Wrong ID to the Motor Registry\n";
  {
    auto motor_registry = std::make_shared<v161_motor_control::MotorRegistry>();
    
    // ID out of valid range (1-32)
    uint8_t invalid_id = 33;
    std::cout << "Attempting to add invalid ID " << static_cast<int>(invalid_id) << " to the motor registry...\n";
    
    auto status = motor_registry->addMotorId(invalid_id);
    if (!status.ok()) {
      std::cout << "Expected error: " << status.message() << "\n";
    } else {
      std::cout << "ID addition succeeded (should not be reached)\n";
    }
  }
  
  std::cout << "\n[3] StatusOr Handling Pattern\n";
  std::cout << "StatusOr<T> contains T type value and Status on success, and Status on failure.\n";
  std::cout << "Correct handling pattern:\n";
  std::cout << "  auto result_or = someFunction();\n";
  std::cout << "  if (!result_or.ok()) {\n";
  std::cout << "    // Error handling\n";
  std::cout << "    std::cerr << \"Error: \" << result_or.status().message() << std::endl;\n";
  std::cout << "    return; // Or other error handling\n";
  std::cout << "  }\n";
  std::cout << "  // Use value on success\n";
  std::cout << "  auto result = result_or.value();\n";
  
  std::cout << "\n[4] General Error Status Codes\n";
  std::cout << "- absl::InvalidArgumentError: Invalid argument passed to method\n";
  std::cout << "- absl::UnavailableError: CAN communication error (connection issue, etc.)\n";
  std::cout << "- absl::InternalError: Internal processing error (response parsing failure, etc.)\n";
  std::cout << "- absl::DeadlineExceededError: Command timeout (no response received)\n";
  std::cout << "- absl::AlreadyExistsError: Attempt to add already existing resource (motor ID, etc.)\n";
  
  std::cout << "\nError Handling Demo Completed!\n";
}

int main(int argc, char* argv[]) {
  // Extract CAN interface name and motor ID from command line arguments
  std::string can_interface_name = "can0";  // Default value
  std::vector<uint8_t> motor_ids = {1};     // Default value
  bool demo_multi_motor = false;            // Whether to run multiple motor demo
  bool demo_error_handling = false;         // Whether to run error handling demo
  
  // Process command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--can" && i + 1 < argc) {
      can_interface_name = argv[++i];
    } else if (arg == "--motor-id" && i + 1 < argc) {
      motor_ids.clear();  // Remove existing default value
      motor_ids.push_back(static_cast<uint8_t>(std::stoi(argv[++i])));
    } else if (arg == "--add-motor-id" && i + 1 < argc) {
      motor_ids.push_back(static_cast<uint8_t>(std::stoi(argv[++i])));
    } else if (arg == "--demo-multi-motor") {
      demo_multi_motor = true;
    } else if (arg == "--demo-error-handling") {
      demo_error_handling = true;
    } else if (arg == "--help") {
      std::cout << "Usage: " << argv[0] << " [options]\n";
      std::cout << "Options:\n";
      std::cout << "  --can <interface>       Use CAN interface (default: can0)\n";
      std::cout << "  --motor-id <ID>         Control motor ID (default: 1)\n";
      std::cout << "  --add-motor-id <ID>     Additional motor ID (for multiple motor control)\n";
      std::cout << "  --demo-multi-motor      Run multiple motor control demo\n";
      std::cout << "  --demo-error-handling   Run error handling demo\n";
      std::cout << "  --help                  Show this help message\n";
      return 0;
    }
  }

  // Clear output buffer
  std::cout << std::unitbuf;

  std::cout << "MyActuator RMD Motor V161 Control Example\n";
  std::cout << "CAN Interface: " << can_interface_name << "\n";
  std::cout << "Motor ID: ";
  for (auto id : motor_ids) {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << "\n\n";
  
  // Run error handling demo only if no multi-motor demo or single motor
  if (demo_error_handling && !demo_multi_motor && motor_ids.size() <= 1) {
    errorHandlingDemo();
    return 0;
  }

  try {
    // Create a CAN interface (managed by shared_ptr)
    std::cout << "Initializing CAN Interface(" << can_interface_name << ")...\n";
    auto can_interface =
        std::make_shared<v161_motor_control::CanInterface>(can_interface_name);
    
    // Create a Motor Registry (managed by shared_ptr)
    std::cout << "Initializing Motor Registry...\n";
    auto motor_registry = 
        std::make_shared<v161_motor_control::MotorRegistry>();
    
    // Add motor ID to registry
    for (auto id : motor_ids) {
      std::cout << "Registering Motor ID " << static_cast<int>(id) << "...\n";
      auto status = motor_registry->addMotorId(id);
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
    }
    
    // Create Motor with both CAN Interface and Motor Registry
    std::vector<std::shared_ptr<v161_motor_control::MotorV161>> motors;
    for (auto id : motor_ids) {
      std::cout << "Creating Motor #" << static_cast<int>(id) << " object...\n";
      motors.push_back(std::make_shared<v161_motor_control::MotorV161>(
          can_interface, motor_registry, id));
    }
    
    std::cout << "Motor initialization completed!\n\n";
    
    // Single motor demo
    if (!demo_multi_motor || motors.size() <= 1) {
      if (!runSingleMotorDemo(*motors[0])) {
        std::cerr << "Single motor demo execution failed\n";
        return 1;
      }
    }
    
    // Multiple motor demo
    if (demo_multi_motor && motors.size() > 1) {
      if (!runMultipleMotorsDemo(motors)) {
        std::cerr << "Multiple motor demo execution failed\n";
        return 1;
      }
    }
    
    // Error handling demo
    if (demo_error_handling) {
      errorHandlingDemo();
    }

  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << '\n';
    return 1;
  }

  std::cout << "\nProgram finished\n";
  return 0;
}
