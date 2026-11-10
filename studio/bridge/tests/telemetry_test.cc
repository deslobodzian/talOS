#include "studio/bridge/telemetry.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace studio {
namespace {

// Builds one size-prefixed v2 frame the way the producer does.
std::string FrameBytes(bool with_dictionary, double velocity, double inhibit,
                       int value_count = 3,
                       std::uint64_t stream_id = 9007199254740993ull,
                       std::uint32_t dictionary_id = 4242,
                       bool missing_chassis = false, double yaw = 0.5,
                       const char* third_name = "pose/valid") {
  flatbuffers::FlatBufferBuilder builder;
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<Talos::Telemetry::ChannelInfo>>>
      dictionary = 0;
  if (with_dictionary) {
    auto labels = builder.CreateVector(
        std::vector<flatbuffers::Offset<flatbuffers::String>>{
            builder.CreateString("DRIVE"),
            builder.CreateString("TARGET_STALE")});
    auto first = Talos::Telemetry::CreateChannelInfo(
        builder, builder.CreateString("drive/velocity"),
        builder.CreateString("m/s"), 0);
    auto second = Talos::Telemetry::CreateChannelInfo(
        builder, builder.CreateString("intake/inhibit"), 0, labels);
    auto third = Talos::Telemetry::CreateChannelInfo(
        builder, builder.CreateString(third_name), 0, 0);
    dictionary = builder.CreateVector(std::vector{first, second, third});
  }
  flatbuffers::Offset<flatbuffers::Vector<double>> values = 0;
  if (value_count >= 0) {
    std::vector<double> samples{velocity, inhibit, 1.0};
    samples.resize(static_cast<std::size_t>(value_count));
    values = builder.CreateVector(samples);
  }
  Talos::Telemetry::Pose chassis{1.5, 2.5, 0, 0, 0, yaw};
  Talos::Telemetry::FinishSizePrefixedFrameBuffer(
      builder, Talos::Telemetry::CreateFrame(
                   builder, stream_id, 7, 123456789, dictionary_id,
                   missing_chassis ? nullptr : &chassis, values, dictionary));
  return {reinterpret_cast<const char*>(builder.GetBufferPointer()),
          builder.GetSize()};
}

TEST(TelemetryJson, SelfDescribesTheLatestFrame) {
  TelemetryState state;
  ASSERT_TRUE(
      IngestTelemetry(state, FrameBytes(true, 0.75, 1.0), 1'000'000'000));
  ASSERT_TRUE(state.has_dictionary);
  ASSERT_TRUE(state.has_frame);

  const std::string json = TelemetryJson(state, 1'250'000'000);
  EXPECT_NE(json.find("\"kind\":\"talos.telemetry\""), std::string::npos);
  EXPECT_NE(json.find("\"version\":2"), std::string::npos);
  // uint64 past 2^53 survives as a decimal string.
  EXPECT_NE(json.find("\"stream_id\":\"9007199254740993\""), std::string::npos);
  EXPECT_NE(json.find("\"sequence_id\":\"7\""), std::string::npos);
  EXPECT_NE(json.find("\"timestamp_ns\":\"123456789\""), std::string::npos);
  EXPECT_NE(json.find("\"dictionary_id\":4242"), std::string::npos);
  EXPECT_NE(json.find("\"age_ms\":250"), std::string::npos);
  EXPECT_NE(json.find("\"x\":1.5"), std::string::npos);
  EXPECT_NE(json.find("\"drive/velocity\":{\"value\":0.75,\"unit\":\"m/s\"}"),
            std::string::npos);
  // A unitless channel still carries "unit", so readers never branch on it.
  EXPECT_NE(json.find("\"pose/valid\":{\"value\":1,\"unit\":\"\"}"),
            std::string::npos);
}

TEST(TelemetryJson, NaNLeavesAsNullAndEnumLabelsOnlyInRange) {
  TelemetryState state;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(IngestTelemetry(state, FrameBytes(true, nan, 1.0), 0));
  std::string json = TelemetryJson(state, 0);
  EXPECT_NE(json.find("\"drive/velocity\":{\"value\":null,"),
            std::string::npos);
  EXPECT_NE(json.find("\"intake/inhibit\":{\"value\":1,\"unit\":\"\","
                      "\"label\":\"TARGET_STALE\"}"),
            std::string::npos);

  // Out of range: a number with no invented label.
  TelemetryState out_of_range;
  ASSERT_TRUE(IngestTelemetry(out_of_range, FrameBytes(true, 1.0, 9.0), 0));
  json = TelemetryJson(out_of_range, 0);
  EXPECT_NE(json.find("\"intake/inhibit\":{\"value\":9,\"unit\":\"\"}"),
            std::string::npos);
  EXPECT_EQ(json.find("\"label\""), std::string::npos);
}

TEST(TelemetryJson, ValueFrameWithoutDictionaryIsNotUsable) {
  TelemetryState state;
  ASSERT_TRUE(IngestTelemetry(state, FrameBytes(false, 1.0, 0.0), 0));
  EXPECT_FALSE(state.has_dictionary);
  EXPECT_FALSE(state.has_frame);
  // A dictionary frame for the same stream unlocks later value frames.
  ASSERT_TRUE(IngestTelemetry(state, FrameBytes(true, 1.0, 0.0), 0));
  ASSERT_TRUE(IngestTelemetry(state, FrameBytes(false, 2.0, 0.0), 5));
  EXPECT_TRUE(state.has_frame);
  EXPECT_EQ(state.values[0], 2.0);
}

TEST(TelemetryJson, RejectsMismatchedDictionaryValuesAtomically) {
  TelemetryState state;
  const std::string original = FrameBytes(true, 0.75, 1.0);
  ASSERT_TRUE(IngestTelemetry(state, original, 10));
  const std::string before = TelemetryJson(state, 10);
  EXPECT_FALSE(
      IngestTelemetry(state, FrameBytes(true, 9.0, 0.0, 2, 55, 99), 20));
  EXPECT_TRUE(state.has_dictionary);
  EXPECT_TRUE(state.has_frame);
  EXPECT_EQ(state.dictionary_frame, original);
  EXPECT_EQ(TelemetryJson(state, 10), before);
}

TEST(TelemetryJson, RejectsInvalidCandidatesWithoutChangingCachedState) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::string> invalid{
      FrameBytes(true, 1.0, 0.0, 3, 55, 99, true),
      FrameBytes(true, 1.0, 0.0, 3, 55, 99, false, nan),
      FrameBytes(true, 1.0, 0.0, 3, 55, 99, false, infinity),
      FrameBytes(true, infinity, 0.0, 3, 55, 99),
      FrameBytes(true, -infinity, 0.0, 3, 55, 99),
      FrameBytes(true, 1.0, 0.0, 3, 55, 99, false, 0.5, "drive/velocity"),
      FrameBytes(true, 1.0, 0.0, 3, 55, 99, false, 0.5, "derived/spoof"),
      FrameBytes(false, 1.0, 0.0, 3, 9007199254740993ull, 4242, true),
      FrameBytes(false, infinity, 0.0)};
  for (const auto& bytes : invalid) {
    TelemetryState state;
    const std::string original = FrameBytes(true, 0.75, 1.0);
    ASSERT_TRUE(IngestTelemetry(state, original, 10));
    const std::string before = TelemetryJson(state, 10);
    EXPECT_FALSE(IngestTelemetry(state, bytes, 20));
    EXPECT_EQ(state.dictionary_frame, original);
    EXPECT_EQ(TelemetryJson(state, 10), before);
    ASSERT_TRUE(IngestTelemetry(state, FrameBytes(false, 2.0, 0.0), 30));
    EXPECT_EQ(state.values[0], 2.0);
  }
}

TEST(TelemetryJson, DictionaryWithoutValuesInvalidatesThePreviousFrame) {
  // Both a repeat of the current dictionary and a replacement must stop
  // exposing the old values until a matching value frame arrives.
  for (const bool changed_stream : {false, true}) {
    for (const bool changed_dictionary : {false, true}) {
      TelemetryState state;
      ASSERT_TRUE(IngestTelemetry(state, FrameBytes(true, 0.75, 1.0), 10));
      const std::uint64_t stream_id = changed_stream ? 55 : 9007199254740993ull;
      const std::uint32_t dictionary_id = changed_dictionary ? 99 : 4242;
      ASSERT_TRUE(IngestTelemetry(
          state, FrameBytes(true, 0.0, 0.0, -1, stream_id, dictionary_id), 20));
      EXPECT_TRUE(state.has_dictionary);
      EXPECT_FALSE(state.has_frame);
      EXPECT_TRUE(state.values.empty());
      EXPECT_EQ(state.stream_id, stream_id);
      EXPECT_EQ(state.dictionary_id, dictionary_id);
      ASSERT_TRUE(IngestTelemetry(
          state, FrameBytes(false, 2.0, 0.0, 3, stream_id, dictionary_id), 30));
      EXPECT_TRUE(state.has_frame);
      ASSERT_EQ(state.values.size(), 3);
      EXPECT_EQ(state.values[0], 2.0);
      EXPECT_EQ(state.received_monotonic_ns, 30);
    }
  }
}

TEST(ChannelsJson, DescribesTheDictionaryOnItsOwn) {
  TelemetryState state;
  ASSERT_TRUE(IngestTelemetry(state, FrameBytes(true, 0.75, 1.0), 0));
  const std::string json = ChannelsJson(state);
  EXPECT_NE(json.find("\"kind\":\"talos.telemetry.channels\""),
            std::string::npos);
  EXPECT_NE(json.find("\"version\":2"), std::string::npos);
  EXPECT_NE(
      json.find("\"name\":\"drive/velocity\",\"unit\":\"m/s\",\"labels\":[]"),
      std::string::npos);
  EXPECT_NE(json.find("\"labels\":[\"DRIVE\",\"TARGET_STALE\"]"),
            std::string::npos);
}

TEST(TelemetryJson, RejectsBytesThatAreNotV2Frames) {
  TelemetryState state;
  EXPECT_FALSE(IngestTelemetry(state, "not a frame", 0));
  EXPECT_FALSE(state.has_dictionary);
  EXPECT_FALSE(state.has_frame);
}

}  // namespace
}  // namespace studio
