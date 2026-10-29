#include "udp.h"

#include <array>
#include <cstring>

#include "gtest/gtest.h"

namespace talos::protocol {
namespace {

TEST(RuntimeUdpTest, SendsRuntimeFrameOverLoopback) {
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
  EXPECT_EQ(mac.stats().sent_frames, 1u);
  EXPECT_EQ(rio.stats().received_frames, 1u);
}

TEST(RuntimeUdpTest, NonBlockingReceiveReportsWouldBlock) {
  RuntimeUdpPeer peer;
  ASSERT_EQ(peer.Open("127.0.0.1", 59102, "127.0.0.1", 59103), UdpStatus::kOk);

  DecodedFrame received;
  EXPECT_EQ(peer.TryReceive(&received), UdpStatus::kWouldBlock);
  EXPECT_EQ(peer.stats().receive_would_block, 1u);
}

TEST(RuntimeUdpTest, SendsBufferedPayloadWithoutExtraCopy) {
  RuntimeUdpPeer mac;
  RuntimeUdpPeer rio;
  UdpSocketOptions options;
  options.non_blocking = false;

  ASSERT_EQ(rio.Open("127.0.0.1", 59105, "127.0.0.1", 59104, options),
            UdpStatus::kOk);
  ASSERT_EQ(mac.Open("127.0.0.1", 59104, "127.0.0.1", 59105, options),
            UdpStatus::kOk);

  constexpr std::size_t kSize = 16;
  uint8_t* payload = mac.MutablePayloadBuffer(kSize);
  ASSERT_NE(payload, nullptr);
  for (std::size_t i = 0; i < kSize; ++i) {
    payload[i] = static_cast<uint8_t>(99 + i);
  }

  ASSERT_EQ(
      mac.SendBufferedFrame(FrameType::kHardwareState, kFlagNone, 22, kSize),
      UdpStatus::kOk);

  DecodedFrame received;
  ASSERT_EQ(rio.TryReceive(&received), UdpStatus::kOk);
  EXPECT_EQ(received.header.type, FrameType::kHardwareState);
  ASSERT_EQ(received.payload_size, kSize);
  for (std::size_t i = 0; i < kSize; ++i) {
    EXPECT_EQ(received.payload[i], static_cast<uint8_t>(99 + i));
  }
}

}  // namespace
}  // namespace talos::protocol
