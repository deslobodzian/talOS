#include "frame.h"

#include <array>
#include <cstring>

#include "gtest/gtest.h"

namespace talos::protocol {
namespace {

TEST(FrameTest, EncodesAndDecodesHeartbeat) {
  std::array<uint8_t, kMaxFrameSize> frame{};
  const std::size_t size =
      EncodeFrame(FrameType::kHeartbeat, kFlagAckRequested, 42, 123456, nullptr,
                  0, frame.data(), frame.size());

  ASSERT_EQ(size, kFrameHeaderSize);

  DecodedFrame decoded;
  EXPECT_EQ(DecodeFrame(frame.data(), size, &decoded), DecodeStatus::kOk);
  EXPECT_EQ(decoded.header.magic, kFrameMagic);
  EXPECT_EQ(decoded.header.version, kProtocolVersion);
  EXPECT_EQ(decoded.header.type, FrameType::kHeartbeat);
  EXPECT_EQ(decoded.header.flags, kFlagAckRequested);
  EXPECT_EQ(decoded.header.sequence, 42u);
  EXPECT_EQ(decoded.header.monotonic_time_us, 123456u);
  EXPECT_EQ(decoded.payload_size, 0u);
}

TEST(FrameTest, EncodesAndDecodesRuntimePayload) {
  struct Payload {
    RuntimeValuesHeader header;
    SignalValue values[2];
  } payload{};
  payload.header.manifest_id = 77;
  payload.header.value_count = 2;
  payload.values[0].id = 1;
  payload.values[0].value_type = ValueType::kFloat64;
  payload.values[0].float64_value = 3.5;
  payload.values[1].id = 2;
  payload.values[1].value_type = ValueType::kBool;
  payload.values[1].bool_value = true;

  std::array<uint8_t, kMaxFrameSize> frame{};
  const std::size_t size =
      EncodeFrame(FrameType::kRioCommand, kFlagNone, 9, 1000, &payload,
                  sizeof(payload), frame.data(), frame.size());

  DecodedFrame decoded;
  ASSERT_EQ(DecodeFrame(frame.data(), size, &decoded), DecodeStatus::kOk);
  ASSERT_EQ(decoded.payload_size, sizeof(payload));

  Payload decoded_payload{};
  std::memcpy(&decoded_payload, decoded.payload, sizeof(decoded_payload));
  EXPECT_EQ(decoded_payload.header.manifest_id, 77u);
  EXPECT_EQ(decoded_payload.header.value_count, 2u);
  EXPECT_EQ(decoded_payload.values[0].id, 1u);
  EXPECT_DOUBLE_EQ(decoded_payload.values[0].float64_value, 3.5);
  EXPECT_TRUE(decoded_payload.values[1].bool_value);
}

TEST(FrameTest, RejectsOversizePayload) {
  std::array<uint8_t, kMaxPayloadSize + 1> payload{};
  std::array<uint8_t, kMaxFrameSize + 1> frame{};

  EXPECT_EQ(EncodeFrame(FrameType::kRioState, kFlagNone, 1, 2, payload.data(),
                        payload.size(), frame.data(), frame.size()),
            0u);
}

TEST(FrameTest, EncodesInPlaceWithoutPayloadCopy) {
  std::array<uint8_t, kMaxFrameSize> frame{};
  auto* payload =
      reinterpret_cast<SignalValue*>(frame.data() + kFrameHeaderSize);
  payload->id = 9;
  payload->value_type = ValueType::kFloat64;
  payload->float64_value = 4.75;

  const std::size_t size =
      EncodeFrameInPlace(FrameType::kRioState, kFlagNone, 3, 10,
                         sizeof(SignalValue), frame.data(), frame.size());

  DecodedFrame decoded;
  ASSERT_EQ(DecodeFrame(frame.data(), size, &decoded), DecodeStatus::kOk);
  ASSERT_EQ(decoded.payload_size, sizeof(SignalValue));
  const auto* decoded_value =
      reinterpret_cast<const SignalValue*>(decoded.payload);
  EXPECT_EQ(decoded_value->id, 9u);
  EXPECT_DOUBLE_EQ(decoded_value->float64_value, 4.75);
}

TEST(FrameTest, ReservedHeaderWordIsZero) {
  std::array<uint8_t, kMaxFrameSize> frame{};
  auto* payload =
      reinterpret_cast<SignalValue*>(frame.data() + kFrameHeaderSize);
  payload->id = 11;
  payload->value_type = ValueType::kUint64;
  payload->uint64_value = 1234;

  const std::size_t size =
      EncodeFrameInPlace(FrameType::kRioState, kFlagNone, 8, 20,
                         sizeof(SignalValue), frame.data(), frame.size());

  DecodedFrame decoded;
  ASSERT_EQ(DecodeFrame(frame.data(), size, &decoded), DecodeStatus::kOk);
  EXPECT_EQ(decoded.header.reserved, 0u);
  const auto* decoded_value =
      reinterpret_cast<const SignalValue*>(decoded.payload);
  EXPECT_EQ(decoded_value->id, 11u);
  EXPECT_EQ(decoded_value->uint64_value, 1234u);
}

}  // namespace
}  // namespace talos::protocol
