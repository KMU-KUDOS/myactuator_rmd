#ifndef V161_MOTOR_CONTROL__CAN_INTERFACE_HPP
#define V161_MOTOR_CONTROL__CAN_INTERFACE_HPP

#include <array>
#include <chrono>
#include <memory>  // for std::unique_ptr
#include <string>
#include <vector>

#include <cstdint>

#include "myactuator_rmd/can/exceptions.h"
#include "myactuator_rmd/can/node.h"
#include "absl/status/status.h"

namespace v161_motor_control {

class CanInterface {
 public:
  /**
   * @brief Constructor : Initialize the CAN interface
   * @param ifname : CAN interface name (e.g. "can0", "vcan0")
   * @param send_timeout_us : Send timeout (microseconds)
   * @param receive_timeout_us : Receive timeout (microseconds)
   */
  CanInterface(const std::string& ifname,
               long send_timeout_us = 1000000,    // 1s
               long receive_timeout_us = 1000000  // 1s
  );
  ~CanInterface();

  /**
   * @brief : Send a CAN frame
   * @param can_id : Target CAN ID
   * @param data : 8 bytes of data to send
   * @return : Status indicating success or failure with error details
   */
  absl::Status sendFrame(uint32_t can_id, const std::array<uint8_t, 8>& data);

  /**
   * @brief : Receive frames from a specific CAN ID, Wait for timeout
   * @param expected_can_id : The CAN ID you expect to receive
   * @param data_out : Array to store the received data
   * @return : Status indicating success or failure with error details
   */
  absl::Status receiveFrame(uint32_t expected_can_id, std::array<uint8_t, 8>& data_out);

  /**
   * @brief : Set the CAN receive filters to listen for specific IDs
   * @param ids : Vector of CAN IDs to filter for
   * @return : Status indicating success or failure with error details
   */
  absl::Status setReceiveFilters(const std::vector<uint32_t>& ids);

  // Disable copy and move constructor/assignment operators (because of
  // unique_ptr member)
  CanInterface(const CanInterface&) = delete;
  CanInterface& operator=(const CanInterface&) = delete;
  CanInterface(CanInterface&&) = delete;
  CanInterface& operator=(CanInterface&&) = delete;

 private:
  std::unique_ptr<myactuator_rmd::can::Node> can_node_;
  std::string ifname_;
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__CAN_INTERFACE_HPP
