#include "talos/protocol/runtime_udp.h"

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

  struct Payload {
    RuntimeValuesHeader header;
    SignalValue value;
  } payload{};
  payload.header.manifest_id = 10;
  payload.header.value_count = 1;
  payload.value.id = 7;
  payload.value.value_type = ValueType::kFloat64;
  payload.value.float64_value = 12.25;

  ASSERT_EQ(mac.SendFrame(FrameType::kRioCommand, kFlagNone, 1000, &payload,
                          sizeof(payload)),
            UdpStatus::kOk);

  DecodedFrame received;
  ASSERT_EQ(rio.TryReceive(&received), UdpStatus::kOk);
  EXPECT_EQ(received.header.type, FrameType::kRioCommand);
  EXPECT_EQ(received.header.sequence, 1u);
  EXPECT_EQ(received.header.monotonic_time_us, 1000u);
  ASSERT_EQ(received.payload_size, sizeof(payload));

  Payload decoded{};
  std::memcpy(&decoded, received.payload, sizeof(decoded));
  EXPECT_EQ(decoded.header.manifest_id, 10u);
  EXPECT_EQ(decoded.value.id, 7u);
  EXPECT_DOUBLE_EQ(decoded.value.float64_value, 12.25);
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

  struct Payload {
    RuntimeValuesHeader header;
    SignalValue value;
  };

  auto* payload =
      reinterpret_cast<Payload*>(mac.MutablePayloadBuffer(sizeof(Payload)));
  ASSERT_NE(payload, nullptr);
  payload->header.manifest_id = 99;
  payload->header.value_count = 1;
  payload->value.id = 4;
  payload->value.value_type = ValueType::kBool;
  payload->value.bool_value = true;

  ASSERT_EQ(mac.SendBufferedFrame(FrameType::kRioCommand, kFlagNone, 22,
                                  sizeof(Payload)),
            UdpStatus::kOk);

  DecodedFrame received;
  ASSERT_EQ(rio.TryReceive(&received), UdpStatus::kOk);
  const auto* decoded = reinterpret_cast<const Payload*>(received.payload);
  EXPECT_EQ(decoded->header.manifest_id, 99u);
  EXPECT_EQ(decoded->value.id, 4u);
  EXPECT_TRUE(decoded->value.bool_value);
}

}  // namespace
}  // namespace talos::protocol
