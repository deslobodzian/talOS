#include "udp.h"

#include <array>
#include <cstring>

#include "gtest/gtest.h"

namespace talos::protocol {
namespace {

// Both send paths (copying and in-buffer) reach the peer intact and number
// their frames from one sequence.
TEST(RuntimeUdpTest, SendsFramesOverLoopback) {
  RuntimeUdpPeer mac;
  RuntimeUdpPeer rio;
  UdpSocketOptions options;
  options.non_blocking = false;

  ASSERT_EQ(rio.Open("127.0.0.1", 59101, "127.0.0.1", 59100, options),
            UdpStatus::kOk);
  ASSERT_EQ(mac.Open("127.0.0.1", 59100, "127.0.0.1", 59101, options),
            UdpStatus::kOk);

  const std::array<uint8_t, 6> payload = {1, 2, 3, 4, 5, 6};
  ASSERT_EQ(mac.SendFrame(FrameType::kHardwareCommand, kFlagNone, 1000,
                          payload.data(), payload.size()),
            UdpStatus::kOk);

  DecodedFrame received;
  ASSERT_EQ(rio.TryReceive(&received), UdpStatus::kOk);
  EXPECT_EQ(received.header.type, FrameType::kHardwareCommand);
  EXPECT_EQ(received.header.sequence, 1u);
  EXPECT_EQ(received.header.monotonic_time_us, 1000u);
  ASSERT_EQ(received.payload_size, payload.size());
  EXPECT_EQ(std::memcmp(received.payload, payload.data(), payload.size()), 0);

  constexpr std::size_t kSize = 16;
  uint8_t* buffered = mac.MutablePayloadBuffer(kSize);
  ASSERT_NE(buffered, nullptr);
  for (std::size_t i = 0; i < kSize; ++i) {
    buffered[i] = static_cast<uint8_t>(99 + i);
  }
  ASSERT_EQ(
      mac.SendBufferedFrame(FrameType::kHardwareState, kFlagNone, 22, kSize),
      UdpStatus::kOk);

  ASSERT_EQ(rio.TryReceive(&received), UdpStatus::kOk);
  EXPECT_EQ(received.header.type, FrameType::kHardwareState);
  EXPECT_EQ(received.header.sequence, 2u);
  ASSERT_EQ(received.payload_size, kSize);
  for (std::size_t i = 0; i < kSize; ++i) {
    EXPECT_EQ(received.payload[i], static_cast<uint8_t>(99 + i));
  }
}

TEST(RuntimeUdpTest, NonBlockingReceiveReportsWouldBlock) {
  RuntimeUdpPeer peer;
  ASSERT_EQ(peer.Open("127.0.0.1", 59102, "127.0.0.1", 59103), UdpStatus::kOk);

  DecodedFrame received;
  EXPECT_EQ(peer.TryReceive(&received), UdpStatus::kWouldBlock);
}

}  // namespace
}  // namespace talos::protocol
