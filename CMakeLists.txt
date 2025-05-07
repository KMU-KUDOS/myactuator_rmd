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
# Placeholder source files - these will be created later
file(GLOB_RECURSE LIB_SRC "src/*.cpp") # This will be empty for now
add_library(myactuator_rmd_lib SHARED
  # Add source files here once they exist, e.g., src/my_file.cpp
  # For now, if LIB_SRC is empty, this might cause issues.
  # Consider adding a dummy source file or conditional compilation for now.
  # Or, more simply, list expected files:
  # src/can_interface.cpp 
  # src/rmd_motor.cpp
  # src/protocol_v1_61.cpp
  # For now, let's assume placeholder files will be added to src/ to avoid build errors.
  # If src/ is empty, CMake might complain or create an empty library.
  # For a robust setup, create placeholder .cpp files in src/ later.
  ${LIB_SRC} 
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
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
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
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  # Google Test (already declared with FetchContent, ensure it's available)
  # FetchContent_MakeAvailable(googletest) # Already done above if not conditional

  # Example for a test executable (placeholder)
  # add_executable(example_test test/example_test.cpp)
  # ament_target_dependencies(example_test
  #   myactuator_rmd_lib
  #   # GTest::gtest_main # Modern CMake target for Google Test
  # )
  # target_link_libraries(example_test PRIVATE GTest::gtest_main GTest::gmock) # If using gmock
  # 
  # ament_add_gtest(example_test test/example_test.cpp)
  # target_link_libraries(example_test myactuator_rmd_lib GTest::gtest_main)

  # Linters
  ament_lint_auto_find_test_dependencies()
  ament_lint_common() # Add common linters
endif()

# This ensures all build, install, and export logic is correctly processed by ament.
ament_package()

# Remove the old message, ament_package provides its own.
# message(STATUS "Core C++ library CMake configuration complete.") 