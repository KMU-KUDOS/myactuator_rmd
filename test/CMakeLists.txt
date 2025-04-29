# Test CMakeLists.txt for myactuator_rmd project
cmake_minimum_required(VERSION 3.16)

# Include GoogleTest
include(GoogleTest)

# Add the test executable
add_executable(motor_v161_tests
    unit/motor_v161_test.cc
)

# Link the test executable with GTest, GMock, and the myactuator_rmd library
target_link_libraries(motor_v161_tests PRIVATE
    gtest
    gtest_main
    gmock
    gmock_main
    myactuator_rmd
)

# Discover and add tests to CTest
gtest_discover_tests(motor_v161_tests) 