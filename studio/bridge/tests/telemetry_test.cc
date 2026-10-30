#include "studio/bridge/telemetry.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace studio {
namespace {

// Builds one size-prefixed v2 frame the way the producer does.
std::string FrameBytes(bool with_dictionary, double velocity, double inhibit) {
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
        builder, builder.CreateString("pose/valid"), 0, 0);
    dictionary = builder.CreateVector(std::vector{first, second, third});
  }
  auto values =
      builder.CreateVector(std::vector<double>{velocity, inhibit, 1.0});
  Talos::Telemetry::Pose chassis{1.5, 2.5, 0, 0, 0, 0.5};
  Talos::Telemetry::FinishSizePrefixedFrameBuffer(
      builder,
      Talos::Telemetry::CreateFrame(builder, 9007199254740993ull, 7, 123456789,
                                    4242, &chassis, values, dictionary));
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
  EXPECT_NE(
      json.find("\"name\":\"drive/velocity\",\"value\":0.75,\"unit\":\"m/s\""),
      std::string::npos);
}

TEST(TelemetryJson, NaNLeavesAsNullAndEnumLabelsOnlyInRange) {
  TelemetryState state;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(IngestTelemetry(state, FrameBytes(true, nan, 1.0), 0));
  std::string json = TelemetryJson(state, 0);
  EXPECT_NE(json.find("\"name\":\"drive/velocity\",\"value\":null"),
            std::string::npos);
  EXPECT_NE(json.find("\"label\":\"TARGET_STALE\""), std::string::npos);

  // Out of range: a number with no invented label.
  TelemetryState out_of_range;
  ASSERT_TRUE(IngestTelemetry(out_of_range, FrameBytes(true, 1.0, 9.0), 0));
  json = TelemetryJson(out_of_range, 0);
  EXPECT_NE(json.find("\"name\":\"intake/inhibit\",\"value\":9"),
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
