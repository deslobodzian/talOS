#include "frame.h"

#include <array>
#include <cstring>

#include "gtest/gtest.h"

namespace talos::protocol {
namespace {

TEST(FrameTest, EncodesAndDecodesHeader) {
  std::array<uint8_t, kMaxFrameSize> frame{};
  const std::size_t size =
      EncodeFrame(FrameType::kHardwareConfigAck, kFlagAckRequested, 42, 123456,
                  nullptr, 0, frame.data(), frame.size());

  ASSERT_EQ(size, kFrameHeaderSize);

  DecodedFrame decoded;
  ASSERT_EQ(DecodeFrame(frame.data(), size, &decoded), DecodeStatus::kOk);
  EXPECT_EQ(decoded.header.magic, kFrameMagic);
  EXPECT_EQ(decoded.header.version, kProtocolVersion);
  EXPECT_EQ(decoded.header.type, FrameType::kHardwareConfigAck);
  EXPECT_EQ(decoded.header.flags, kFlagAckRequested);
  EXPECT_EQ(decoded.header.sequence, 42u);
  EXPECT_EQ(decoded.header.monotonic_time_us, 123456u);
  EXPECT_EQ(decoded.payload_size, 0u);
}

TEST(FrameTest, RoundTripsPayloadBytes) {
  std::array<uint8_t, kMaxPayloadSize> payload{};
  for (std::size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<uint8_t>(i * 7);
  }

  std::array<uint8_t, kMaxFrameSize> frame{};
  const std::size_t size =
      EncodeFrame(FrameType::kHardwareCommand, kFlagNone, 9, 1000,
                  payload.data(), payload.size(), frame.data(), frame.size());
  ASSERT_EQ(size, kMaxFrameSize);

  DecodedFrame decoded;
  ASSERT_EQ(DecodeFrame(frame.data(), size, &decoded), DecodeStatus::kOk);
  EXPECT_EQ(decoded.header.type, FrameType::kHardwareCommand);
  ASSERT_EQ(decoded.payload_size, payload.size());
  EXPECT_EQ(std::memcmp(decoded.payload, payload.data(), payload.size()), 0);
}

TEST(FrameTest, RejectsOversizePayload) {
  std::array<uint8_t, kMaxPayloadSize + 1> payload{};
  std::array<uint8_t, kMaxFrameSize + 1> frame{};

  EXPECT_EQ(
      EncodeFrame(FrameType::kHardwareState, kFlagNone, 1, 2, payload.data(),
                  payload.size(), frame.data(), frame.size()),
      0u);
}

TEST(FrameTest, EncodesInPlaceWithoutPayloadCopy) {
  std::array<uint8_t, kMaxFrameSize> frame{};
  const double value = 4.75;
  std::memcpy(frame.data() + kFrameHeaderSize, &value, sizeof(value));

  const std::size_t size =
      EncodeFrameInPlace(FrameType::kHardwareState, kFlagNone, 3, 10,
                         sizeof(value), frame.data(), frame.size());

  DecodedFrame decoded;
  ASSERT_EQ(DecodeFrame(frame.data(), size, &decoded), DecodeStatus::kOk);
  ASSERT_EQ(decoded.payload_size, sizeof(value));
  double decoded_value = 0;
  std::memcpy(&decoded_value, decoded.payload, sizeof(decoded_value));
  EXPECT_DOUBLE_EQ(decoded_value, 4.75);
}

TEST(FrameTest, ReservedHeaderWordIsZero) {
  std::array<uint8_t, kMaxFrameSize> frame{};
  frame.fill(0xff);
  const std::size_t size =
      EncodeFrameInPlace(FrameType::kDriverStation, kFlagNone, 8, 20, 4,
                         frame.data(), frame.size());

  DecodedFrame decoded;
  ASSERT_EQ(DecodeFrame(frame.data(), size, &decoded), DecodeStatus::kOk);
  EXPECT_EQ(decoded.header.reserved, 0u);
  EXPECT_EQ(decoded.payload_size, 4u);
}

}  // namespace
}  // namespace talos::protocol
