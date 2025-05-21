/**
 * \file version_test.cpp
 * \mainpage
 *    Test the protocol version information
 * \author
 *    Taehun Jung (github.com/KMU-KUDOS)
 */

#include <gtest/gtest.h>

#include "myactuator_rmd/version.hpp"

namespace myactuator_rmd::test {

  TEST(VersionTest, ProtocolVersion) {
    // Protocol version is set to V1.61
    EXPECT_EQ(MYACTUATOR_RMD_PROTOCOL_MAJOR_VERSION, 1);
    EXPECT_EQ(MYACTUATOR_RMD_PROTOCOL_MINOR_VERSION, 61);
  }

}
