cmake_minimum_required(VERSION 3.16)
project(myactuator_rmd VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)
enable_testing()

if(NOT UNIX)
    message(FATAL_ERROR "Currently this driver only supports Linux!")
endif()

# Find required packages
find_package(Threads REQUIRED)

# GoogleTest Integration
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG    release-1.11.0 # 안정적인 버전을 위해 특정 태그 사용
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Abseil Integration
FetchContent_Declare(
  abseil
  GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
  GIT_TAG        20230125.3  # 안정적인 버전을 위해 특정 태그 사용
)

# Position Independent Code 활성화
set(ABSL_BUILD_TESTING OFF CACHE BOOL "Disable abseil tests")
set(ABSL_USE_SYSTEM_INCLUDES ON CACHE BOOL "Use system includes" FORCE)
set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL "Propagate C++ standard to dependencies" FORCE)

# Make all fetched content available
FetchContent_MakeAvailable(googletest abseil)

# --- Build the myactuator_rmd shared library ---
add_library(myactuator_rmd SHARED
    # CAN implementation files
    src/can/node.cc
    src/can/utilities.cc
    # Protocol implementation files
    src/protocol/motor_v161.cc
    src/protocol/packing_v161.cc
    src/protocol/parsing_v161.cc
    # New SRP-compliant classes
    src/protocol/motor_configurator.cc
    src/protocol/motor_status_querier.cc
    src/protocol/motor_actuator.cc
    # Interface implementation
    src/can_interface.cc
    # Motor Registry implementation
    src/motor_registry.cc
)

# Header file paths
target_include_directories(myactuator_rmd PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link with pthread for Linux SocketCAN and abseil libraries
target_link_libraries(myactuator_rmd PUBLIC
    Threads::Threads
    absl::status
    absl::statusor
    absl::strings
)

# --- Build v161_example executable ---
add_executable(v161_example
    examples/v161_motor/main.cc
)

# Link with myactuator_rmd library
target_link_libraries(v161_example PRIVATE
    myactuator_rmd
)

# --- Installation settings ---
install(TARGETS myactuator_rmd v161_example
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
)

add_subdirectory(tests)

message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
