#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "myactuator_rmd/motor_registry.h"
#include "myactuator_rmd/protocol/protocol_v161.h"
#include "absl/status/status.h"

namespace v161_motor_control {
namespace {

using ::testing::ElementsAre;
using ::testing::ElementsAreArray;
using ::testing::IsEmpty;

class MotorRegistryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 각 테스트 전에 실행됨
  }

  void TearDown() override {
    // 각 테스트 후에 실행됨
  }

  MotorRegistry registry_;
};

TEST_F(MotorRegistryTest, InitiallyEmpty) {
  // 초기 상태에서는 등록된 모터가 없어야 함
  EXPECT_THAT(registry_.getRegisteredMotorIds(), IsEmpty());
  EXPECT_THAT(registry_.getFilterIds(), IsEmpty());
}

TEST_F(MotorRegistryTest, AddValidMotorId) {
  // 유효한 모터 ID를 추가하는 테스트
  EXPECT_TRUE(registry_.addMotorId(1).ok());
  EXPECT_THAT(registry_.getRegisteredMotorIds(), ElementsAre(1));
  
  // 여러 개의 유효한 모터 ID 추가
  EXPECT_TRUE(registry_.addMotorId(5).ok());
  EXPECT_TRUE(registry_.addMotorId(32).ok());
  EXPECT_THAT(registry_.getRegisteredMotorIds(), ElementsAre(1, 5, 32));
}

TEST_F(MotorRegistryTest, AddInvalidMotorId) {
  // 0은 유효하지 않은 ID (1~32 범위 외)
  auto status = registry_.addMotorId(0);
  EXPECT_FALSE(status.ok());
  EXPECT_TRUE(absl::IsInvalidArgument(status));
  EXPECT_THAT(registry_.getRegisteredMotorIds(), IsEmpty());
  
  // 33은 유효하지 않은 ID (1~32 범위 외)
  status = registry_.addMotorId(33);
  EXPECT_FALSE(status.ok());
  EXPECT_TRUE(absl::IsInvalidArgument(status));
  EXPECT_THAT(registry_.getRegisteredMotorIds(), IsEmpty());
  
  // 255는 유효하지 않은 ID (1~32 범위 외)
  status = registry_.addMotorId(255);
  EXPECT_FALSE(status.ok());
  EXPECT_TRUE(absl::IsInvalidArgument(status));
  EXPECT_THAT(registry_.getRegisteredMotorIds(), IsEmpty());
}

TEST_F(MotorRegistryTest, AddDuplicateMotorId) {
  // 처음 추가할 때는 성공
  EXPECT_TRUE(registry_.addMotorId(10).ok());
  EXPECT_THAT(registry_.getRegisteredMotorIds(), ElementsAre(10));
  
  // 동일한 ID를 다시 추가하면 AlreadyExists 상태 반환
  auto status = registry_.addMotorId(10);
  EXPECT_FALSE(status.ok());
  EXPECT_TRUE(absl::IsAlreadyExists(status));
  // 중복 추가되지는 않음
  EXPECT_THAT(registry_.getRegisteredMotorIds(), ElementsAre(10));
}

TEST_F(MotorRegistryTest, GetFilterIds) {
  // 등록된 모터가 없을 때는 빈 필터 ID 목록 반환
  EXPECT_THAT(registry_.getFilterIds(), IsEmpty());
  
  // 모터 ID 1 추가
  EXPECT_TRUE(registry_.addMotorId(1).ok());
  
  // 예상되는 필터 ID는 protocol::getV161ResponseId(1)
  uint32_t expected_filter_id = protocol::getV161ResponseId(1);
  EXPECT_THAT(registry_.getFilterIds(), ElementsAre(expected_filter_id));
  
  // 모터 ID 5 추가
  EXPECT_TRUE(registry_.addMotorId(5).ok());
  
  // 예상되는 필터 ID 목록은 protocol::getV161ResponseId(1)와 protocol::getV161ResponseId(5)
  std::vector<uint32_t> expected_filter_ids = {
    protocol::getV161ResponseId(1),
    protocol::getV161ResponseId(5)
  };
  
  EXPECT_THAT(registry_.getFilterIds(), ElementsAreArray(expected_filter_ids));
}

// 범위 경계값 테스트
TEST_F(MotorRegistryTest, BoundaryValues) {
  // 하한 경계값 (1은 유효)
  EXPECT_TRUE(registry_.addMotorId(1).ok());
  
  // 상한 경계값 (32는 유효)
  EXPECT_TRUE(registry_.addMotorId(32).ok());
  
  // 하한 외부 경계값 (0은 유효하지 않음)
  auto status = registry_.addMotorId(0);
  EXPECT_FALSE(status.ok());
  EXPECT_TRUE(absl::IsInvalidArgument(status));
  
  // 상한 외부 경계값 (33은 유효하지 않음)
  status = registry_.addMotorId(33);
  EXPECT_FALSE(status.ok());
  EXPECT_TRUE(absl::IsInvalidArgument(status));
  
  // 최종 등록된 ID 확인
  EXPECT_THAT(registry_.getRegisteredMotorIds(), ElementsAre(1, 32));
}

}  // namespace
}  // namespace v161_motor_control 