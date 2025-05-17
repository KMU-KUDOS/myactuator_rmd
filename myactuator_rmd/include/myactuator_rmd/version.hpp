/**
 * \file version.hpp
 * \mainpage
 *    Contains macros for the driver protocol version
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__VERSION
#define MYACTUATOR_RMD__VERSION
#pragma once

// Package version
#define MYACTUATOR_RMD_VERSION_MAJOR 2  // Major version bump due to protocol change
#define MYACTUATOR_RMD_VERSION_MINOR 0
#define MYACTUATOR_RMD_VERSION_PATCH 0

// Protocol version
#define MYACTUATOR_RMD_PROTOCOL_MAJOR_VERSION 1  // Changed from 3.8 to 1.61 to support newer V1.61 protocol
#define MYACTUATOR_RMD_PROTOCOL_MINOR_VERSION 61

// Protocol version string (for reporting and display purposes)
#define MYACTUATOR_RMD_PROTOCOL_VERSION "V1.61"

// Version string for the entire package
#define MYACTUATOR_RMD_VERSION_STR "2.0.0"

/**
 * Enum for supported protocol versions
 * This allows for more type-safe protocol version checks throughout the codebase
 */
enum class ProtocolVersion {
  V1_61 = 161  // Version 1.61 protocol
};

// Current protocol version set to V1.61
const ProtocolVersion CURRENT_PROTOCOL_VERSION = ProtocolVersion::V1_61;

#endif // MYACTUATOR_RMD__VERSION
