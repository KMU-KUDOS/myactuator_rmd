cmake_minimum_required(VERSION 3.16)
project(myactuator_rmd VERSION 0.1.0 LANGUAGES CXX)

# C++17 requirement
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
endif()

# Find ROS2 ament_cmake and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED) # As per PRD FR6.6
find_package(pluginlib REQUIRED)         # As per PRD C5
find_package(angles REQUIRED)            # As per PRD C5
find_package(std_msgs REQUIRED)          # Added in package.xml

# External non-ROS dependencies (using FetchContent)
include(FetchContent)

# MoodyCamel ConcurrentQueue
FetchContent_Declare(
  concurrentqueue
  GIT_REPOSITORY https://github.com/cameron314/concurrentqueue.git
  GIT_TAG v1.0.3 # Using a specific tag for stability
)
FetchContent_MakeAvailable(concurrentqueue)

# Library definition for the core C++ library
add_library(myactuator_rmd_lib SHARED
  src/core/can_interface.cc
  # Add other source files here as they are created
)

target_include_directories(myactuator_rmd_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> # For installed headers
  PRIVATE
    ${concurrentqueue_SOURCE_DIR} # For concurrentqueue headers
)

# Link external non-ROS libraries
target_link_libraries(myactuator_rmd_lib
  PUBLIC
    concurrentqueue::concurrentqueue
)

# Link ROS2 dependencies to the library
ament_target_dependencies(myactuator_rmd_lib
  PUBLIC
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
    angles
    std_msgs
)

# Set library properties
set_target_properties(myactuator_rmd_lib PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  # PUBLIC_HEADER "myactuator_rmd/myactuator_rmd.hpp" # This will be handled by install(DIRECTORY...)
)

# Installation rules
# Install the library
install(TARGETS myactuator_rmd_lib
  EXPORT export_myactuator_rmd_lib # Export for ament_cmake
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install public headers from the include/myactuator_rmd directory
install(DIRECTORY include/myactuator_rmd/
  DESTINATION include/myactuator_rmd
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.launch.xml"
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.param.yaml"
)

# Export targets for downstream CMake projects (if this were a pure CMake lib, not primarily ROS2 pkg)
# This was from the previous version, for ROS2 ament_export_targets is preferred.
# install(EXPORT MyActuatorRMDLibTargets
#   FILE MyActuatorRMDLibTargets.cmake
#   NAMESPACE MyActuatorRMD::
#   DESTINATION lib/cmake/myactuator_rmd_lib
# )
# For ROS2, ament_export_targets handles this better.
ament_export_targets(export_myactuator_rmd_lib HAS_LIBRARY_TARGET)

# Testing configuration (Google Test with ament_cmake_gtest)
option(BUILD_TESTING "Build tests" ON)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  # Add the test executable for CanInterface
  add_executable(can_interface_test
    test/core/can_interface_test.cc
  )
  # Link the test executable against our library and GTest/GMock
  target_link_libraries(can_interface_test
    PRIVATE
    myactuator_rmd_lib
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
    GTest::gmock_main
    # Add Abseil libraries needed by tests if not already linked by myactuator_rmd_lib
    # For example, if status/time are used directly in test assertions:
    absl::status
    absl::time
    absl::log_initialize # If Abseil logging is used directly in tests
  )

  # Add the test to CTest using ament_cmake_gtest
  ament_add_gtest(can_interface_test
    test/core/can_interface_test.cc
  )
  # If you have multiple test source files for this target, list them all in add_executable
  # and in the ament_add_gtest call or use a glob if preferred (though explicit is better).

  # Add linters if desired (example)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
  # ament_lint_auto_add_test(myactuator_rmd_lib)

endif()

# This ensures all build, install, and export logic is correctly processed by ament.
ament_package()

# Remove the old message, ament_package provides its own.
# message(STATUS "Core C++ library CMake configuration complete.") 