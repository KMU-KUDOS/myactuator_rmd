/**
 * \file command_type.hpp
 * \mainpage
 *    Contains enum for all supported command types for V1.61 protocol
 * \author
 *    Tobit Flatscher (github.com/2b-t)
 *    Taehun Jung (github.com/KMU-KUDOS)
*/

#ifndef MYACTUATOR_RMD__PROTOCOL__COMMAND_TYPE
#define MYACTUATOR_RMD__PROTOCOL__COMMAND_TYPE

#include <cstdint>


namespace myactuator_rmd {

  /**\enum CommandType
   * \brief
   *    Strongly typed enum for all supported command types in V1.61 protocol
  */
  enum class CommandType: std::uint8_t {
    // PID Commands (V1.61)  
    READ_PID_PARAMETERS = 0x30,
    WRITE_PID_PARAMETERS_TO_RAM = 0x31,
    WRITE_PID_PARAMETERS_TO_ROM = 0x32,
    
    // Encoder Commands (V1.61)
    WRITE_CURRENT_MULTI_TURN_POSITION_TO_ROM_AS_ZERO = 0x64,
    READ_SINGLE_TURN_ENCODER = 0x90,
    READ_MULTI_TURN_ANGLE = 0x92,
    READ_SINGLE_TURN_ANGLE = 0x94,
    
    // Motor Status Commands (V1.61)
    READ_MOTOR_STATUS_1_AND_ERROR_FLAG = 0x9A,
    READ_MOTOR_STATUS_2 = 0x9C,
    READ_MOTOR_STATUS_3 = 0x9D,
    
    // Motor Control Commands (V1.61)
    SHUTDOWN_MOTOR = 0x80,
    STOP_MOTOR = 0x81,
    TORQUE_CLOSED_LOOP_CONTROL = 0xA1,
    SPEED_CLOSED_LOOP_CONTROL = 0xA2,
    ABSOLUTE_POSITION_CLOSED_LOOP_CONTROL = 0xA4
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
