# GoogleTest를 위한 디렉토리 설정
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${gtest_SOURCE_DIR}/include
  ${gmock_SOURCE_DIR}/include
)

# 유닛 테스트용 라이브러리 설정
add_library(test_main STATIC
  test_main.cc
)

target_link_libraries(test_main
  PRIVATE
    gtest
    gmock
)

# 모터 레지스트리 테스트
add_executable(motor_registry_tests
  motor_registry_tests.cc
)

target_link_libraries(motor_registry_tests
  PRIVATE
    test_main
    myactuator_rmd
    gtest
    gmock
    Threads::Threads
)

# CAN 인터페이스 테스트
add_executable(can_interface_tests
  can_interface_tests.cc
)

target_link_libraries(can_interface_tests
  PRIVATE
    test_main
    myactuator_rmd
    gtest
    gmock
    Threads::Threads
)

# 테스트 추가
add_test(NAME MotorRegistryTests COMMAND motor_registry_tests)
add_test(NAME CanInterfaceTests COMMAND can_interface_tests) 