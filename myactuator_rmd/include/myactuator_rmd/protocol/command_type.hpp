/**
 * \file command_type.hpp
 * \mainpage
 *    Contains enum for all supported command types for MyActuator RMD motor drivers
 * \author
 *    Tobit Flatscher (github.com/2b-t)
 * 
 * \note
 *    Updated 2025-05-18 for V1.61 protocol compatibility
*/

#ifndef MYACTUATOR_RMD__PROTOCOL__COMMAND_TYPE
#define MYACTUATOR_RMD__PROTOCOL__COMMAND_TYPE
#pragma once

#include <cstdint>


namespace myactuator_rmd {

  /**\enum CommandType
   * \brief
   *    Strongly typed enum for all supported command types based on V1.61 protocol
   * \details
   *    Based on the GYEMS/MyActuator Motor Control Protocol (V1.61)
   *    Contains all command codes for motor control, status reading, and configuration
  */
  enum class CommandType: std::uint8_t {
    // PID and acceleration related commands (0x30-0x34)
    READ_PID_PARAMETERS = 0x30,             // Read position/speed/torque loop PID parameters
    WRITE_PID_PARAMETERS_TO_RAM = 0x31,     // Write PID parameters to RAM (volatile)
    WRITE_PID_PARAMETERS_TO_ROM = 0x32,     // Write PID parameters to ROM (non-volatile)
    READ_ACCELERATION = 0x33,               // Read acceleration data (Updated from 0x42 in V3.8)
    WRITE_ACCELERATION_TO_RAM = 0x34,       // Write acceleration data to RAM (Updated from 0x43 in V3.8)
    
    // Zero position setting (0x19)
    WRITE_CURRENT_POSITION_TO_ROM_AS_ZERO = 0x19, // Set current position as zero point in ROM
    
    // Motor control commands (0x80-0x88)
    MOTOR_OFF = 0x80,                       // Turn motor off (Renamed from SHUTDOWN_MOTOR)
    MOTOR_STOP = 0x81,                      // Stop motor
    MOTOR_RUNNING = 0x88,                   // Run motor
    
    // Encoder and motor status commands (0x90-0x9D)
    READ_ENCODER_DATA = 0x90,               // Read encoder data
    WRITE_ENCODER_OFFSET = 0x91,            // Write encoder offset
    READ_MULTI_TURNS_ANGLE = 0x92,          // Read multi-turn angle position
    READ_SINGLE_CIRCLE_ANGLE = 0x94,        // Read single-turn angle position
    READ_MOTOR_STATUS_1 = 0x9A,             // Read motor status and error flags
    CLEAR_MOTOR_ERROR_FLAG = 0x9B,          // Clear motor error flags
    READ_MOTOR_STATUS_2 = 0x9C,             // Read additional motor status
    READ_MOTOR_STATUS_3 = 0x9D,             // Read extended motor status
    
    // Closed-loop control commands (0xA1-0xA6)
    TORQUE_CLOSED_LOOP_CONTROL = 0xA1,       // Torque closed-loop control mode
    SPEED_CLOSED_LOOP_CONTROL = 0xA2,        // Speed closed-loop control mode
    POSITION_CLOSED_LOOP_CONTROL_1 = 0xA3,   // Position closed-loop control mode 1
    POSITION_CLOSED_LOOP_CONTROL_2 = 0xA4,   // Position closed-loop control mode 2
    POSITION_CLOSED_LOOP_CONTROL_3 = 0xA5,   // Position closed-loop control mode 3
    POSITION_CLOSED_LOOP_CONTROL_4 = 0xA6    // Position closed-loop control mode 4
  };

  // Symmetric comparison operators
  constexpr bool operator == (CommandType const& c, std::uint8_t const i) noexcept {
    return i == static_cast<std::uint8_t>(c);
  }
  constexpr bool operator == (std::uint8_t const i, CommandType const& c) noexcept {
    return operator == (c, i);
  }

  constexpr bool operator != (CommandType const& c, std::uint8_t const i) noexcept {
    return !(operator == (c, i));
  }
  constexpr bool operator != (std::uint8_t const i, CommandType const& c) noexcept {
    return operator != (c, i);
  }

}

#endif // MYACTUATOR_RMD__PROTOCOL__COMMAND_TYPE
