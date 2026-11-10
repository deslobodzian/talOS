#include "frame.h"

#include <array>
#include <cstring>

#include "gtest/gtest.h"

namespace talos::protocol {
namespace {

TEST(FrameTest, RoundTripsHeaderAndMaximumPayload) {
  std::array<uint8_t, kMaxPayloadSize> payload{};
  for (std::size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<uint8_t>(i * 7);
  }
  // The codec carries the raw 16-bit field even though no flag is defined.
  constexpr uint16_t kRawFlags = 0xA5C3;

  std::array<uint8_t, kMaxFrameSize> frame{};
  const std::size_t size =
      EncodeFrame(FrameType::kHardwareCommand, kRawFlags, 42, 123456,
                  payload.data(), payload.size(), frame.data(), frame.size());
  ASSERT_EQ(size, kMaxFrameSize);

  DecodedFrame decoded;
  ASSERT_EQ(DecodeFrame(frame.data(), size, &decoded), DecodeStatus::kOk);
  EXPECT_EQ(decoded.header.magic, kFrameMagic);
  EXPECT_EQ(decoded.header.version, kProtocolVersion);
  EXPECT_EQ(decoded.header.type, FrameType::kHardwareCommand);
  EXPECT_EQ(decoded.header.flags, kRawFlags);
  EXPECT_EQ(decoded.header.sequence, 42u);
  EXPECT_EQ(decoded.header.monotonic_time_us, 123456u);
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

TEST(FrameTest, EncodesInPlaceAroundThePayloadAndZeroesReserved) {
  std::array<uint8_t, kMaxFrameSize> frame{};
  frame.fill(0xff);
  const double value = 4.75;
  std::memcpy(frame.data() + kFrameHeaderSize, &value, sizeof(value));

  const std::size_t size =
      EncodeFrameInPlace(FrameType::kHardwareState, kFlagNone, 3, 10,
                         sizeof(value), frame.data(), frame.size());

  DecodedFrame decoded;
  ASSERT_EQ(DecodeFrame(frame.data(), size, &decoded), DecodeStatus::kOk);
  EXPECT_EQ(decoded.header.reserved, 0u);
  ASSERT_EQ(decoded.payload_size, sizeof(value));
  double decoded_value = 0;
  std::memcpy(&decoded_value, decoded.payload, sizeof(decoded_value));
  EXPECT_DOUBLE_EQ(decoded_value, 4.75);
}

TEST(FrameTest, RejectsPreviousProtocolVersion) {
  std::array<uint8_t, kMaxFrameSize> frame{};
  const auto size = EncodeFrame(FrameType::kHardwareState, kFlagNone, 1, 2,
                                nullptr, 0, frame.data(), frame.size());
  frame[8] = static_cast<uint8_t>(kProtocolVersion - 1);
  frame[9] = 0;
  DecodedFrame decoded;
  EXPECT_EQ(DecodeFrame(frame.data(), size, &decoded),
            DecodeStatus::kUnsupportedVersion);
}

}  // namespace
}  // namespace talos::protocol
