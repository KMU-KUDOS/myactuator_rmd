#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>

#include "../mocks/mock_can_interface.h"

namespace v161_motor_control {
namespace testing {

using ::testing::Return;
using ::testing::_;

/**
 * @brief Test fixture for MotorV161 tests
 */
class MotorV161Test : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a shared_ptr to the mock interface
    mock_can_interface_ = std::make_shared<MockCanInterface>();
    
    // Set up default behaviors for mock methods
    ON_CALL(*mock_can_interface_, addMotorId(_))
        .WillByDefault(Return(true));
    ON_CALL(*mock_can_interface_, sendFrame(_, _))
        .WillByDefault(Return(true));
    ON_CALL(*mock_can_interface_, receiveFrame(_, _))
        .WillByDefault(Return(true));
  }

  void TearDown() override {
    // Clean up any resources used by the test
  }

  // Shared mock CAN interface that can be used by all tests
  std::shared_ptr<MockCanInterface> mock_can_interface_;
};

/**
 * @brief Simple placeholder test to verify test framework works
 */
TEST_F(MotorV161Test, Placeholder) {
  // Basic assertion that should always pass
  EXPECT_TRUE(true);
  
  // Verify we can create our mock object
  EXPECT_NE(mock_can_interface_, nullptr);
  
  // Test that the default behaviors are working
  EXPECT_TRUE(mock_can_interface_->addMotorId(1));
  
  std::array<uint8_t, 8> dummy_data = {0, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_TRUE(mock_can_interface_->sendFrame(0x100, dummy_data));
  
  std::array<uint8_t, 8> output_data;
  EXPECT_TRUE(mock_can_interface_->receiveFrame(0x100, output_data));
}

}  // namespace testing
}  // namespace v161_motor_control

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} 