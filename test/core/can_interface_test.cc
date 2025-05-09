#include "gtest/gtest.h"
#include "gmock/gmock.h" // For future mocking if needed

#include "myactuator_rmd/core/can_interface.h"
#include "myactuator_rmd/core/can_frame.h"
#include "absl/status/status.h"
#include "absl/time/time.h"

// To run these tests, a virtual CAN interface (e.g., vcan0) should be up and running.
// You can set it up using commands like:
// sudo modprobe vcan
// sudo ip link add dev vcan0 type vcan
// sudo ip link set up vcan0
//
// And tear it down after tests:
// sudo ip link delete vcan0

namespace myactuator_rmd::core::testing
{

const char* const kTestCanInterface = "vcan0"; // Default virtual CAN interface for testing

class CanInterfaceTest : public ::testing::Test
{
protected:
    std::unique_ptr<CanInterface> can_interface_;

    void SetUp() override
    {
        // It's generally better to create the object under test in each test case
        // or use a factory, to ensure a fresh state for each test.
        // However, for simplicity in these initial tests, we can create it here
        // and ensure it's properly managed.
    }

    void TearDown() override
    {
        if (can_interface_ && can_interface_->is_open())
        {
            ASSERT_TRUE(can_interface_->close().ok());
        }
    }

    // Helper to create an interface for tests that need one pre-opened.
    void CreateAndOpenInterface(const std::string& interface_name = kTestCanInterface)
    {
        can_interface_ = std::make_unique<CanInterface>(interface_name);
        ASSERT_NE(can_interface_, nullptr);
        absl::Status status = can_interface_->open();
        ASSERT_TRUE(status.ok()) << "Failed to open " << interface_name << ": " << status;
        ASSERT_TRUE(can_interface_->is_open());
    }
};

TEST_F(CanInterfaceTest, ConstructAndDestruct)
{
    // Test that the CanInterface can be constructed and destructed without crashing.
    // If SetUp/TearDown create/destroy, this test is implicitly covered for the default case.
    // Explicitly test construction with a different name to ensure no hardcoding issues.
    auto interface = std::make_unique<CanInterface>("can_test_construct");
    EXPECT_NE(interface, nullptr);
    // Destructor will be called when 'interface' goes out of scope.
}

TEST_F(CanInterfaceTest, OpenAndCloseSuccessfully)
{
    can_interface_ = std::make_unique<CanInterface>(kTestCanInterface);
    ASSERT_NE(can_interface_, nullptr);
    
    EXPECT_FALSE(can_interface_->is_open());
    
    absl::Status status = can_interface_->open();
    EXPECT_TRUE(status.ok()) << "Status: " << status;
    EXPECT_TRUE(can_interface_->is_open());

    status = can_interface_->close();
    EXPECT_TRUE(status.ok()) << "Status: " << status;
    EXPECT_FALSE(can_interface_->is_open());
}

TEST_F(CanInterfaceTest, OpenAlreadyOpenInterface)
{
    CreateAndOpenInterface(); // Opens kTestCanInterface
    ASSERT_TRUE(can_interface_->is_open());

    absl::Status status = can_interface_->open();
    EXPECT_FALSE(status.ok());
    EXPECT_EQ(status.code(), absl::StatusCode::kAlreadyExists) << "Status: " << status;
    EXPECT_TRUE(can_interface_->is_open()); // Should still be open
}

TEST_F(CanInterfaceTest, CloseUnopenedInterface)
{
    can_interface_ = std::make_unique<CanInterface>(kTestCanInterface);
    ASSERT_NE(can_interface_, nullptr);
    EXPECT_FALSE(can_interface_->is_open());

    absl::Status status = can_interface_->close();
    EXPECT_TRUE(status.ok()) << "Status: " << status; // Closing an unopened interface should be OK
    EXPECT_FALSE(can_interface_->is_open());
}

TEST_F(CanInterfaceTest, OpenNonExistentInterface)
{
    can_interface_ = std::make_unique<CanInterface>("nonexistentcan");
    ASSERT_NE(can_interface_, nullptr);
    
    absl::Status status = can_interface_->open();
    EXPECT_FALSE(status.ok());
    // The specific error code might depend on the system/kernel behavior for ioctl SIOCGIFINDEX.
    // absl::StatusCode::kNotFound is a common expectation.
    EXPECT_EQ(status.code(), absl::StatusCode::kNotFound) << "Status: " << status;
    EXPECT_FALSE(can_interface_->is_open());
}

TEST_F(CanInterfaceTest, SendAndReceiveSingleFrame)
{
    CreateAndOpenInterface(); // Opens kTestCanInterface

    // Create a second interface to act as a peer
    CanInterface peer_interface(kTestCanInterface); // Connect to the same vcan interface
    ASSERT_TRUE(peer_interface.open().ok()) << "Failed to open peer interface";
    ASSERT_TRUE(peer_interface.is_open());

    const uint32_t test_can_id = 0x123;
    const std::array<uint8_t, 2> test_data = {0xAB, 0xCD};
    CanFrame frame_to_send(test_can_id, reinterpret_cast<const uint8_t*>(test_data.data()), test_data.size());
    frame_to_send.is_extended_id = false;

    // Send the frame from the main interface
    absl::Status send_status = can_interface_->send_frame(frame_to_send, absl::Milliseconds(100));
    EXPECT_TRUE(send_status.ok()) << "send_frame failed: " << send_status;

    // Try to receive the frame on the peer interface
    absl::StatusOr<CanFrame> received_frame_status = peer_interface.receive_frame(absl::Milliseconds(200)); // Generous timeout
    ASSERT_TRUE(received_frame_status.ok()) << "receive_frame failed: " << received_frame_status.status();

    CanFrame received_frame = received_frame_status.value();
    EXPECT_EQ(received_frame.id, test_can_id);
    EXPECT_EQ(received_frame.dlc, test_data.size());
    for (size_t i = 0; i < test_data.size(); ++i)
    {
        EXPECT_EQ(received_frame.data[i], test_data[i]);
    }
    EXPECT_FALSE(received_frame.is_extended_id);

    ASSERT_TRUE(peer_interface.close().ok());
}

TEST_F(CanInterfaceTest, SendFrameTimeout)
{
    // For this test, we don't open the interface fully, or use a non-listening peer.
    // A simpler way to test send timeout without a peer might be to fill the kernel buffer,
    // but that's harder to control reliably in a unit test.
    // Instead, we can try to send to an interface that has no listener and expect a timeout
    // if the send buffer fills and blocks, or rely on the poll timeout for non-blocking send.
    // However, a true send *timeout* on a non-blocking socket typically means the poll itself timed out.

    can_interface_ = std::make_unique<CanInterface>(kTestCanInterface, absl::Milliseconds(5), absl::Milliseconds(5)); // Short timeouts
    ASSERT_TRUE(can_interface_->open().ok());

    const uint32_t test_can_id = 0x456;
    const std::array<uint8_t, 1> test_data = {0xFF};
    CanFrame frame_to_send(test_can_id, test_data.data(), test_data.size());

    // Continuously send until a timeout or error occurs. 
    // The kernel buffer for vcan might be large, so this might take many sends if it doesn't block immediately.
    // A more direct test of poll timeout would involve mocking poll.
    // Given we are not mocking, we rely on poll() timing out if the socket isn't writable.
    // If the socket is always writable (vcan might be), then send() itself won't block or timeout directly in a typical sense.
    // The timeout in send_frame is for poll().
    // For a vcan interface, it might accept frames very quickly.
    // This test might be flaky or not test the timeout as intended without a way to block the vcan output.
    // Let's assume for now that trying to send to a busy or non-existent peer on the vcan will eventually lead to poll timeout.
    // A better approach would be to use a mock or fill the buffer.

    // To simulate a more direct poll timeout, let's assume the interface is writable
    // but we pass a zero timeout to send_frame which should then use the default (5ms) for poll.
    // This is more a test of the poll() timeout mechanism within send_frame.
    absl::Status send_status;
    int attempts = 0;
    const int max_attempts = 5; // Try a few times, expecting poll to timeout
    bool timed_out = false;
    for (attempts = 0; attempts < max_attempts; ++attempts) {
        // Use a very short explicit timeout for the send operation itself to test that path in poll.
        send_status = can_interface_->send_frame(frame_to_send, absl::Milliseconds(1)); 
        if (send_status.code() == absl::StatusCode::kDeadlineExceeded) {
            timed_out = true;
            break;
        }
        // If it sends ok, try again (vcan might be consuming them)
    }
    // This test is inherently tricky without mocking socket behavior or filling kernel buffers.
    // We expect DeadlineExceeded if poll times out.
    EXPECT_TRUE(timed_out) << "Expected DeadlineExceeded, but got " << send_status << " after " << attempts << " attempts";
}

TEST_F(CanInterfaceTest, ReceiveFrameTimeout)
{
    CreateAndOpenInterface(); // Opens kTestCanInterface
    // No frames are sent, so receive should timeout.
    absl::StatusOr<CanFrame> received_frame_status = can_interface_->receive_frame(absl::Milliseconds(50)); // Short timeout
    
    ASSERT_FALSE(received_frame_status.ok());
    EXPECT_EQ(received_frame_status.status().code(), absl::StatusCode::kDeadlineExceeded) << "Status: " << received_frame_status.status();
}

// TODO: Test receiving CAN error frames (will require ability to inject error frames on vcan0)
// TODO: Test with extended CAN IDs for send and receive.
// TODO: Test send/receive with zero timeout for true non-blocking behavior.

} // namespace myactuator_rmd::core::testing 