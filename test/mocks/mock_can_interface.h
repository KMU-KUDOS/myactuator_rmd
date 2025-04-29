#ifndef MOCK_CAN_INTERFACE_H
#define MOCK_CAN_INTERFACE_H

#include <array>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

// 테스트를 위해 인터페이스만 정의하고 사용합니다
namespace v161_motor_control {
namespace testing {

/**
 * @brief Interface class for CAN communication (used for testing)
 */
class ICanInterface {
 public:
  virtual ~ICanInterface() = default;
  
  // Define the interface methods
  virtual bool addMotorId(uint8_t motor_id) = 0;
  virtual bool sendFrame(uint32_t can_id, const std::array<uint8_t, 8>& data) = 0;
  virtual bool receiveFrame(uint32_t expected_can_id, std::array<uint8_t, 8>& data_out) = 0;
};

/**
 * @brief Mock class for ICanInterface to be used in unit tests
 */
class MockCanInterface : public ICanInterface {
 public:
  // Constructor
  MockCanInterface() {}
  
  // Mock methods using Google Mock
  MOCK_METHOD1(addMotorId, bool(uint8_t motor_id));
  MOCK_METHOD2(sendFrame, bool(uint32_t can_id, const std::array<uint8_t, 8>& data));
  MOCK_METHOD2(receiveFrame, bool(uint32_t expected_can_id, std::array<uint8_t, 8>& data_out));
};

}  // namespace testing
}  // namespace v161_motor_control

#endif  // MOCK_CAN_INTERFACE_H 