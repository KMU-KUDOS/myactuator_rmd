#include "myactuator_rmd/can_interface.h"

#include <algorithm>  // for std::find
#include <iostream>
#include <memory>
#include <vector>

#include <cstdint>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

namespace v161_motor_control {

CanInterface::CanInterface(const std::string& ifname,
                           long send_timeout_us,
                           long receive_timeout_us)
    : ifname_(ifname) {
  try {
    can_node_ = std::make_unique<myactuator_rmd::can::Node>(ifname_);
    can_node_->setSendTimeout(std::chrono::microseconds(send_timeout_us));
    can_node_->setRecvTimeout(std::chrono::microseconds(receive_timeout_us));

    can_node_->setErrorFilters(true);  // Set to also receive error frames by
                                       // default (change to false if needed)

    can_node_->setLoopback(
        false);  // Disable loopback (don't receive messages from yourself)

    std::cout << "Can interface '" << ifname << "' initialized successfully"
              << '\n';
  } catch (const myactuator_rmd::can::SocketException& e) {
    std::cerr << "Failed to initialized CAN interface '" << ifname
              << "': " << e.what() << " (Error Code: " << e.code() << ")"
              << '\n';

    can_node_ = nullptr;  // Pointer nulling on initialization failure

    throw;  // Throw an exception again to signal a failed creation
  }
}

CanInterface::~CanInterface() {
  // unique_ptr automatically frees memory and calls destructor (closeSocket()
  // called in can::Node destructor)
  std::cout << "Closing CAN interface '" << ifname_ << "'." << '\n';
}

absl::Status CanInterface::sendFrame(uint32_t can_id,
                           const std::array<uint8_t, 8>& data) {
  if (!can_node_) {
    return absl::UnavailableError("CAN node is not initialized. Cannot send frame");
  }

  if (can_id == 0) {
    return absl::InvalidArgumentError("Invalid CAN ID 0. Cannot send frame");
  }

  try {
    // Debugging
    // std::cout << "Sending Frame - ID: 0x" << std::hex << can_id << ", Data:
    // "; for(int i = 0; i < 8; ++i) std::cout << std::hex << std::setfill('0')
    // << std::setw(2) << static_cast<int>(data[i]) << " "; std::cout <<
    // std::dec << '\n';

    can_node_->write(can_id, data);
    return absl::OkStatus();
  } catch (const myactuator_rmd::can::SocketException& e) {
    return absl::UnavailableError(
        absl::StrCat("Failed to send CAN frame (ID: 0x", 
                      absl::Hex(can_id), "): ", e.what()));
  } catch (const myactuator_rmd::can::Exception& e) {
    // Other CAN-specific exceptions (TxTimeout, etc.)
    return absl::DeadlineExceededError(
        absl::StrCat("CAN Error during send (ID: 0x", 
                      absl::Hex(can_id), "): ", e.what()));
  } catch (const std::exception& e) {
    return absl::InternalError(
        absl::StrCat("Unknown error during send (ID: 0x", 
                      absl::Hex(can_id), "): ", e.what()));
  }
}

absl::Status CanInterface::receiveFrame(uint32_t expected_can_id,
                              std::array<uint8_t, 8>& data_out) {
  if (!can_node_) {
    return absl::UnavailableError("CAN node is not initialized. Cannot receive frame");
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
      return absl::OkStatus();
    } else {
      // Receiving a frame with an unexpected ID (filtering is not perfect,
      // error frames, etc.)
      return absl::InvalidArgumentError(
          absl::StrCat("Received frame with unexpected ID: 0x", 
                        absl::Hex(received_frame.getId()), 
                        " (Expected: 0x", absl::Hex(expected_can_id), ")"));
    }
  } catch (const myactuator_rmd::can::SocketException& e) {
    // EAGAIN or EWOULDBLOCK means timeout
    if (e.code().value() == EAGAIN || e.code().value() == EWOULDBLOCK) {
      return absl::DeadlineExceededError(
          absl::StrCat("CAN receive timeout (Expected ID: 0x", 
                         absl::Hex(expected_can_id), ")"));
    } else {
      return absl::UnavailableError(
          absl::StrCat("Failed to receive CAN frame (Expected ID: 0x", 
                         absl::Hex(expected_can_id), "): ", e.what()));
    }
  } catch (const myactuator_rmd::can::Exception& e) {
    return absl::InternalError(
        absl::StrCat("CAN error during receive (Expected ID: 0x", 
                       absl::Hex(expected_can_id), "): ", e.what()));
  } catch (const std::exception& e) {
    return absl::InternalError(
        absl::StrCat("Unknown error during receive (Expected ID: 0x", 
                       absl::Hex(expected_can_id), "): ", e.what()));
  }
}

absl::Status CanInterface::setReceiveFilters(const std::vector<uint32_t>& ids) {
  if (!can_node_) {
    return absl::UnavailableError("CAN node is not initialized. Cannot set receive filters");
  }

  try {
    can_node_->setRecvFilter(ids);
    
    if (!ids.empty()) {
      std::cout << "CAN receive filters updated for IDs:";
      for (uint32_t id : ids) {
        std::cout << " 0x" << std::hex << id << std::dec;
      }
      std::cout << '\n';
    } else {
      std::cout << "Clearing CAN filters" << '\n';
    }
    
    return absl::OkStatus();
  } catch (const myactuator_rmd::can::SocketException& e) {
    return absl::InternalError(
        absl::StrCat("Failed to set CAN receive filters: ", e.what()));
  }
}

}  // namespace v161_motor_control
