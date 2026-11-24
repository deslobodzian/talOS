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

// The header layout the RoboRIO and the companion agree on, byte for byte,
// whatever either host's byte order.
// REQ-CONTRACT-03
TEST(FrameTest, HeaderIsLittleEndianOnTheWire) {
  std::array<uint8_t, kMaxFrameSize> frame{};
  const std::size_t size = EncodeFrame(
      FrameType::kHardwareConfig, 0x0102, 0x1112131415161718ull,
      0x2122232425262728ull, nullptr, 0, frame.data(), frame.size());
  ASSERT_EQ(size, kFrameHeaderSize);
  const std::array<uint8_t, kFrameHeaderSize> expected{
      'T',  'A',  'L',  'O',  'S',  'R',  'I',  'O',   // magic
      12,   0,    40,   0,    22,   0,    0x02, 0x01,  // version..flags
      0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11,  // sequence
      0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21,  // monotonic_time_us
      0,    0,    0,    0,    0,    0,    0,    0};    // payload_size, reserved
  static_assert(kProtocolVersion == 12 && kFrameHeaderSize == 40);
  EXPECT_EQ(std::memcmp(frame.data(), expected.data(), expected.size()), 0);

  frame[32] = 0x02;  // payload_size 0x0102 = 258, decoded little-endian
  frame[33] = 0x01;
  DecodedFrame decoded;
  ASSERT_EQ(DecodeFrame(frame.data(), kFrameHeaderSize + 258, &decoded),
            DecodeStatus::kOk);
  EXPECT_EQ(decoded.header.sequence, 0x1112131415161718ull);
  EXPECT_EQ(decoded.payload_size, 258u);
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

// REQ-CONTRACT-01: EncodeFrameInPlace must reject null output buffers,
// payloads larger than kMaxPayloadSize, and capacities shorter than header +
// payload without writing past the buffer.
TEST(FrameTest, EncodeFrameInPlaceRejectsInvalidArguments) {
  std::array<uint8_t, kMaxFrameSize + 16> frame{};
  frame.fill(0xCC);

  EXPECT_EQ(EncodeFrameInPlace(FrameType::kHardwareState, kFlagNone, 1, 2,
                               kMaxPayloadSize + 1, frame.data(), frame.size()),
            0u);
  EXPECT_EQ(EncodeFrameInPlace(FrameType::kHardwareState, kFlagNone, 1, 2, 0,
                               nullptr, kMaxFrameSize),
            0u);
  EXPECT_EQ(EncodeFrameInPlace(FrameType::kHardwareState, kFlagNone, 1, 2, 0,
                               frame.data(), 0),
            0u);
  EXPECT_EQ(EncodeFrameInPlace(FrameType::kHardwareState, kFlagNone, 1, 2, 0,
                               frame.data(), kFrameHeaderSize - 1),
            0u);
  EXPECT_EQ(EncodeFrameInPlace(FrameType::kHardwareState, kFlagNone, 1, 2, 16,
                               frame.data(), kFrameHeaderSize + 15),
            0u);
  for (const uint8_t byte : frame) {
    EXPECT_EQ(byte, 0xCCu);
  }
}

// REQ-CONTRACT-01
TEST(FrameTest, RejectsInvalidHeaderVersionMagicAndSize) {
  std::array<uint8_t, kMaxFrameSize> valid{};
  const auto size = EncodeFrame(FrameType::kHardwareState, kFlagNone, 1, 2,
                                nullptr, 0, valid.data(), valid.size());
  ASSERT_EQ(size, kFrameHeaderSize);

  DecodedFrame decoded{};
  // Too small: 0 B and 39 B (one short of kFrameHeaderSize).
  EXPECT_EQ(DecodeFrame(valid.data(), 0, &decoded), DecodeStatus::kTooSmall);
  EXPECT_EQ(DecodeFrame(valid.data(), kFrameHeaderSize - 1, &decoded),
            DecodeStatus::kTooSmall);

  // Bad magic.
  {
    auto frame = valid;
    frame[0] ^= 0x01;
    EXPECT_EQ(DecodeFrame(frame.data(), size, &decoded),
              DecodeStatus::kBadMagic);
  }

  // Unsupported version: both one below and one above kProtocolVersion.
  for (const uint16_t bad_version :
       {static_cast<uint16_t>(kProtocolVersion - 1),
        static_cast<uint16_t>(kProtocolVersion + 1)}) {
    auto frame = valid;
    frame[8] = static_cast<uint8_t>(bad_version & 0xFF);
    frame[9] = static_cast<uint8_t>(bad_version >> 8);
    decoded = {};
    EXPECT_EQ(DecodeFrame(frame.data(), size, &decoded),
              DecodeStatus::kUnsupportedVersion);
    EXPECT_EQ(decoded.header.version, bad_version);
  }

  // Bad header size: 39 B and 41 B.
  for (const uint16_t bad_header_size :
       {static_cast<uint16_t>(kFrameHeaderSize - 1),
        static_cast<uint16_t>(kFrameHeaderSize + 1)}) {
    auto frame = valid;
    frame[10] = static_cast<uint8_t>(bad_header_size & 0xFF);
    frame[11] = static_cast<uint8_t>(bad_header_size >> 8);
    EXPECT_EQ(DecodeFrame(frame.data(), size, &decoded),
              DecodeStatus::kBadHeaderSize);
  }

  // Payload larger than kMaxPayloadSize even when buffer size matches header.
  {
    std::array<uint8_t, kMaxFrameSize + 1> oversize{};
    std::memcpy(oversize.data(), valid.data(), kFrameHeaderSize);
    constexpr uint32_t kTooLarge = kMaxPayloadSize + 1;
    oversize[32] = static_cast<uint8_t>(kTooLarge & 0xFF);
    oversize[33] = static_cast<uint8_t>((kTooLarge >> 8) & 0xFF);
    oversize[34] = static_cast<uint8_t>((kTooLarge >> 16) & 0xFF);
    oversize[35] = static_cast<uint8_t>((kTooLarge >> 24) & 0xFF);
    EXPECT_EQ(DecodeFrame(oversize.data(), oversize.size(), &decoded),
              DecodeStatus::kPayloadTooLarge);
  }
}

// REQ-CONTRACT-02
TEST(FrameTest, RejectsTruncatedAndTrailingBytes) {
  const std::array<uint8_t, 4> payload = {1, 2, 3, 4};
  std::array<uint8_t, kMaxFrameSize> frame{};
  const std::size_t size =
      EncodeFrame(FrameType::kHardwareCommand, kFlagNone, 7, 100,
                  payload.data(), payload.size(), frame.data(), frame.size());
  ASSERT_EQ(size, kFrameHeaderSize + payload.size());

  DecodedFrame decoded;
  EXPECT_EQ(DecodeFrame(frame.data(), size - 1, &decoded),
            DecodeStatus::kTruncated);
  frame[size] = 0xAA;
  EXPECT_EQ(DecodeFrame(frame.data(), size + 1, &decoded),
            DecodeStatus::kTruncated);
}

}  // namespace
}  // namespace talos::protocol
