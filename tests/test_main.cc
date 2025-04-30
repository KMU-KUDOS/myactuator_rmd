#include <gtest/gtest.h>
#include <gmock/gmock.h>

// 모든 GoogleTest 테스트에서 사용할 메인 함수
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
} 