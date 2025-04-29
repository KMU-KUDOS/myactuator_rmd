#ifndef V161_MOTOR_CONTROL__MOTOR_REGISTRY_H_
#define V161_MOTOR_CONTROL__MOTOR_REGISTRY_H_

#include <vector>
#include <cstdint>

namespace v161_motor_control {

/**
 * @brief Manages motor IDs and their associated CAN filter IDs
 * 
 * This class is responsible for registering and tracking motor IDs,
 * as well as generating appropriate CAN filter IDs for registered motors.
 */
class MotorRegistry {
 public:
  /**
   * @brief Constructor: Initialize an empty motor registry
   */
  MotorRegistry();
  
  /**
   * @brief Add a motor ID to the registry
   * @param motor_id Motor ID to add (1 to 32)
   * @return true if successfully added or already exists, false otherwise
   */
  bool addMotorId(uint8_t motor_id);
  
  /**
   * @brief Get the list of registered motor IDs
   * @return Vector of registered motor IDs
   */
  std::vector<uint8_t> getRegisteredMotorIds() const;
  
  /**
   * @brief Generate CAN filter IDs for all registered motors
   * @return Vector of CAN filter IDs
   */
  std::vector<uint32_t> getFilterIds() const;
  
 private:
  std::vector<uint8_t> registered_motor_ids_;
};

}  // namespace v161_motor_control

#endif  // V161_MOTOR_CONTROL__MOTOR_REGISTRY_H_ 