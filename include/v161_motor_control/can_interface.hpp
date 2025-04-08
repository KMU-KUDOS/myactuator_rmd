#ifndef V161_MOTOR_CONTROL__CAN_INTERFACE_HPP
#define V161_MOTOR_CONTROL__CAN_INTERFACE_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <memory> // for std::unique_ptr
#include <string>
#include <vector>

#include "myactuator_rmd/can/exceptions.hpp"
#include "myactuator_rmd/can/node.hpp"

namespace v161_motor_control {

class CanInterface {
public:
  /**
   * @brief Constructor : Initialize the CAN interface
   * @param ifname : CAN interface name (e.g. "can0", "vcan0")
   * @param send_timeout_us : Send timeout (microseconds)
   * @param receive_timeout_us : Receive timeout (microseconds)
   */
  CanInterface(const std::string &ifname,
               long send_timeout_us = 1000000,   // 1s
               long receive_timeout_us = 1000000 // 1s
  );
  ~CanInterface();

  /**
   * @brief : Add the motor IDs to communicate with and set the receive filter
   * Listen to the response ID (0x140 + ID) of the V1.61 protocol
   * @param motor_id : Motor ID to add (1 to 32)
   */
  bool addMotorId(uint8_t motor_id);

  /**
   * @brief : Send a CAN frame
   * @param can_id : Target CAN ID
   * @param data : 8 bytes of data to send
   * @return : true on success, false on failure
   */
  bool sendFrame(uint32_t can_id, const std::array<uint8_t, 8> &data);

  /**
   * @brief : Receive frames from a specific CAN ID, Wait for timeout
   * @param expected_can_id : The CAN ID you expect to receive
   * @param data_out : Array to store the received data
   * @return : true on success, false on failure
   */
  bool receiveFrame(uint32_t expected_can_id, std::array<uint8_t, 8> &data_out);

  // Disable copy and move constructor/assignment operators (because of
  // unique_ptr member)
  CanInterface(const CanInterface &) = delete;
  CanInterface &operator=(const CanInterface &) = delete;
  CanInterface(CanInterface &&) = delete;
  CanInterface &operator=(CanInterface &&) = delete;

private:
  std::unique_ptr<myactuator_rmd::can::Node> can_node_;
  std::vector<uint8_t> registered_motor_ids_;
  std::string ifname_;

  /**
   * @brief : Update the incoming filter for the currently registered motor IDs
   */
  void updateReceiveFilters();
};

} // namespace v161_motor_control

#endif // V161_MOTOR_CONTROL__CAN_INTERFACE_HPP
