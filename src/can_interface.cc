#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/protocol/protocol_v161.h" // for getV161ResponseId

#include <algorithm> // for std::find
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

namespace v161_motor_control {

CanInterface::CanInterface(const std::string &ifname, long send_timeout_us,
                           long receive_timeout_us)
    : ifname_(ifname) {
  try {
    can_node_ = std::make_unique<myactuator_rmd::can::Node>(ifname_);
    can_node_->setSendTimeout(std::chrono::microseconds(send_timeout_us));
    can_node_->setRecvTimeout(std::chrono::microseconds(receive_timeout_us));

    can_node_->setErrorFilters(true); // Set to also receive error frames by
                                      // default (change to false if needed)

    can_node_->setLoopback(
        false); // Disable loopback (don't receive messages from yourself)

    std::cout << "Can interface '" << ifname << "' initialized successfully"
              << '\n';
  } catch (const myactuator_rmd::can::SocketException &e) {
    std::cerr << "Failed to initialized CAN interface '" << ifname
              << "': " << e.what() << " (Error Code: " << e.code() << ")"
              << '\n';

    can_node_ = nullptr; // Pointer nulling on initialization failure

    throw; // Throw an exception again to signal a failed creation
  }
}

CanInterface::~CanInterface() {
  // unique_ptr automatically frees memory and calls destructor (closeSocket()
  // called in can::Node destructor)
  std::cout << "Closing CAN interface '" << ifname_ << "'." << '\n';
}

bool CanInterface::addMotorId(uint8_t motor_id) {
  if (!can_node_) {
    std::cerr << "CAN node is not initialized" << '\n';
    return false;
  }

  if (motor_id < 1 || motor_id > 32) {
    std::cerr << "Invalid motor ID: " << static_cast<int>(motor_id)
              << ". Must be between 1 & 32" << '\n';
    return false;
  }

  // Check motor_id
  if (std::find(registered_motor_ids_.begin(), registered_motor_ids_.end(),
                motor_id) == registered_motor_ids_.end()) {
    registered_motor_ids_.push_back(motor_id);
    updateReceiveFilters();

    std::cout << "Motor ID " << static_cast<int>(motor_id)
              << " registered. Updating CAN filters" << '\n';
    return true;
  } else {
    std::cout << "Motor ID " << static_cast<int>(motor_id)
              << " is already registered" << '\n';
    return true;
  }
}

void CanInterface::updateReceiveFilters() {
  if (!can_node_ || registered_motor_ids_.empty()) {
    return;
  }

  std::vector<uint32_t> receive_ids;

  for (uint8_t id : registered_motor_ids_) {
    uint32_t response_id = protocol::getV161ResponseId(id);

    if (response_id != 0) {
      receive_ids.push_back(response_id);
    }
  }

  if (!receive_ids.empty()) {
    try {
      can_node_->setRecvFilter(receive_ids);
      std::cout << "CAN receive filters updated for IDs:";

      for (uint32_t id : receive_ids) {
        std::cout << " 0x" << std::hex << id << std::dec;
      }

      std::cout << '\n';
    } catch (const myactuator_rmd::can::SocketException &e) {
      std::cerr << "Failed to set CAN receive filters: " << e.what() << '\n';
    }
  } else {
    std::cout << "No valid motor IDs registered, clearing CAN filters" << '\n';

    try {
      can_node_->setRecvFilter({});
    } catch (const myactuator_rmd::can::SocketException &e) {
      std::cerr << "Failed to clear CAN receive filters: " << e.what() << '\n';
    }
  }
}

bool CanInterface::sendFrame(uint32_t can_id,
                             const std::array<uint8_t, 8> &data) {
  if (!can_node_) {
    std::cerr << "CAN node is not initialized. Cannot send frame" << '\n';
    return false;
  }

  if (can_id == 0) {
  std:
    std::cerr << "Invalid CAN ID 0. Cannot send frame" << '\n';
    return false;
  }

  try {
    // Debugging
    // std::cout << "Sending Frame - ID: 0x" << std::hex << can_id << ", Data:
    // "; for(int i = 0; i < 8; ++i) std::cout << std::hex << std::setfill('0')
    // << std::setw(2) << static_cast<int>(data[i]) << " "; std::cout <<
    // std::dec << '\n';

    can_node_->write(can_id, data);
    return true;
  } catch (const myactuator_rmd::can::SocketException &e) {
    std::cerr << "Failed to send CAN frame (ID: 0x" << std::hex << can_id
              << std::dec << "): " << e.what() << '\n';
  } catch (const myactuator_rmd::can::Exception
               &e) { // Other CAN-specific exceptions (TxTimeout, etc.)
    std::cerr << "CAN Error during send (ID: 0x" << std::hex << can_id
              << std::dec << "): " << e.what() << '\n';
  } catch (const std::exception &e) {
    std::cerr << "Unknown error during send (ID: 0x" << std::hex << can_id
              << std::dec << "): " << e.what() << '\n';
  }
  return false;
}

bool CanInterface::receiveFrame(uint32_t expected_can_id,
                                std::array<uint8_t, 8> &data_out) {
  if (!can_node_) {
    std::cerr << "CAN node is not initialized. Cannot receive frame" << '\n';
    return false;
  }

  try {
    // read() function throws a SocketException (errno=EAGAIN or EWOULDBLOCK) on
    // timeout
    myactuator_rmd::can::Frame received_frame = can_node_->read();

    // Checking the ID of a received frame
    if (received_frame.getId() == expected_can_id) {
      data_out = received_frame.getData();

      // Debugging
      // std::cout << "Received Frame - ID: 0x" << std::hex <<
      // received_frame.getId() << ", Data: "; for(int i = 0; i < 8; ++i)
      // std::cout << std::hex << std::setfill('0') << std::setw(2) <<
      // static_cast<int>(data_out[i]) << " "; std::cout << std::dec << '\n';
      return true;
    } else {
      // Receiving a frame with an unexpected ID (filtering is not perfect,
      // error frames, etc.)
      std::cerr << "Warning: Received frame with unexpected ID: 0x" << std::hex
                << received_frame.getId() << " (Expected: 0x" << expected_can_id
                << ")" << std::dec << '\n';

      // Need to make a policy decision on whether to return false or retry
      // read() here Return false once to let the calling side decide if it
      // wants to retry
      return false;
    }
  } catch (const myactuator_rmd::can::SocketException &e) {
    // EAGAIN or EWOULDBLOCK means timeout
    if (e.code().value() == EAGAIN || e.code().value() == EWOULDBLOCK) {
      // std::cout << "CAN receive timeout (Expected ID: 0x" << std::hex <<
      // expected_can_id << "): " << std::dec << '\n';
    } else {
      std::cerr << "Failed to receive CAN frame (Expected ID: 0x" << std::hex
                << expected_can_id << std::dec << "): " << e.what() << '\n';
    }
  } catch (const myactuator_rmd::can::Exception &e) {
    std::cerr << "CAN error during receive (Expected ID: 0x" << std::hex
              << expected_can_id << std::dec << "): " << e.what() << '\n';
  } catch (const std::exception &e) {
    std::cerr << "Unknown error during receive (Expected ID: 0x" << std::hex
              << expected_can_id << std::dec << "): " << e.what() << '\n';
  }
  return false;
}

} // namespace v161_motor_control
