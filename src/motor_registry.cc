#include "myactuator_rmd/motor_registry.h"

#include <algorithm>  // for std::find
#include <iostream>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "myactuator_rmd/protocol/protocol_v161.h"  // for getV161ResponseId

namespace v161_motor_control {

MotorRegistry::MotorRegistry() {
  // Initialize empty registry
}

absl::Status MotorRegistry::addMotorId(uint8_t motor_id) {
  if (motor_id < 1 || motor_id > 32) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid motor ID: ", static_cast<int>(motor_id),
                    ". Must be between 1 & 32"));
  }

  // Check if motor_id already exists
  if (std::find(registered_motor_ids_.begin(),
                registered_motor_ids_.end(),
                motor_id) == registered_motor_ids_.end()) {
    registered_motor_ids_.push_back(motor_id);
    return absl::OkStatus();
  } else {
    return absl::AlreadyExistsError(
        absl::StrCat("Motor ID ", static_cast<int>(motor_id),
                    " is already registered"));
  }
}

std::vector<uint8_t> MotorRegistry::getRegisteredMotorIds() const {
  return registered_motor_ids_;
}

std::vector<uint32_t> MotorRegistry::getFilterIds() const {
  std::vector<uint32_t> filter_ids;
  
  // Populate filter IDs for all registered motor IDs
  for (uint8_t id : registered_motor_ids_) {
    uint32_t response_id = protocol::getV161ResponseId(id);
    
    if (response_id != 0) {
      filter_ids.push_back(response_id);
    }
  }
  
  return filter_ids;
}

}  // namespace v161_motor_control 