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

# External dependencies
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
file(GLOB_RECURSE LIB_SRC "src/*.cpp")
add_library(myactuator_rmd_lib SHARED
  ${LIB_SRC}
)

target_include_directories(myactuator_rmd_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    # Add concurrentqueue source to include path if its headers are not installed globally
    # concurrentqueue_SOURCE_DIR is available after FetchContent_MakeAvailable
    ${concurrentqueue_SOURCE_DIR} # This provides headers like concurrentqueue.h
)

# Link libraries
target_link_libraries(myactuator_rmd_lib
  PUBLIC
    concurrentqueue::concurrentqueue # Modern CMake target for concurrentqueue
)

# Set library properties
set_target_properties(myactuator_rmd_lib PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR} # For ABI compatibility
  PUBLIC_HEADER "myactuator_rmd/myactuator_rmd.hpp" # Example main public header
)

# Installation rules for the library
install(TARGETS myactuator_rmd_lib
  EXPORT MyActuatorRMDLibTargets # Exporting the library target
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  PUBLIC_HEADER DESTINATION include/myactuator_rmd # Install public headers to include/myactuator_rmd
)

# Install public headers from the include directory
install(DIRECTORY include/myactuator_rmd/
  DESTINATION include/myactuator_rmd
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Generate and install export file for downstream consumption
install(EXPORT MyActuatorRMDLibTargets
  FILE MyActuatorRMDLibTargets.cmake
  NAMESPACE MyActuatorRMD::
  DESTINATION lib/cmake/myactuator_rmd_lib
)

# Testing configuration (Google Test)
option(BUILD_TESTING "Build tests" ON)

if(BUILD_TESTING)
  enable_testing()

  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1 # Using a specific tag for stability
  )
  FetchContent_MakeAvailable(googletest)

  # Placeholder test source file - this will be created later
  # add_executable(myactuator_rmd_tests
  #   test/example_test.cpp
  # )
  # 
  # target_link_libraries(myactuator_rmd_tests
  #   PRIVATE
  #     myactuator_rmd_lib
  #     GTest::gtest_main # Modern CMake target for Google Test
  #     GTest::gmock # If using Google Mock
  # )
  # 
  # include(GoogleTest)
  # gtest_discover_tests(myactuator_rmd_tests)
endif()

# Print a message to indicate the end of the core library CMakeLists.txt
message(STATUS "Core C++ library CMake configuration complete.") 