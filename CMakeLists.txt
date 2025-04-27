cmake_minimum_required(VERSION 3.16)
project(v161_motor_control VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT UNIX)
    message(FATAL_ERROR "Currently this driver only supports Linux!")
endif()

# --- third_party CAN Library ---
add_library(myactuator_rmd_can STATIC
    third_party/myactuator_rmd_can/src/node.cc
    third_party/myactuator_rmd_can/src/utilities.cc
)

# Header file path
target_include_directories(myactuator_rmd_can PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/myactuator_rmd_can/include
)

# Requires a pthread link to use Linux SocketCAN
target_link_libraries(myactuator_rmd_can PUBLIC pthread)


# --- Build the main library ---
add_library(v161_motor_control_lib STATIC
    src/can_interface.cc
    src/packing_v161.cc
    src/parsing_v161.cc
    src/motor_v161.cc
)

# Header file path
target_include_directories(v161_motor_control_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# CAN library link
target_link_libraries(v161_motor_control_lib PUBLIC
    myactuator_rmd_can
)


# --- Build an executable ---
add_executable(main_app src/main.cc)

# Main library link
target_link_libraries(main_app PRIVATE v161_motor_control_lib)

message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
