#include "talOS/telemetry/node.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "geometry/geometry_generated.h"
#include "schema_test/shapes_generated.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/node/testing.h"
#include "talOS/rtms/rtms.h"
#include "talOS/schema/identity.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {
namespace {

using namespace std::chrono_literals;
using Talos::Telemetry::Frame;
using testing::Sim;

// Test node that publishes schemas with nested structs, arrays, enums, bools.
class ShapesNode {
 public:
  explicit ShapesNode(talos::Node& node)
      : mixed_{node.publish<schema_test::Mixed>("/demo/state")},
        arrays_{node.publish<schema_test::Arrays>("/demo/status")} {}

  talos::Publisher<schema_test::Mixed> mixed_;
  talos::Publisher<schema_test::Arrays> arrays_;
};

// Test node that publishes PoseEstimate.
class EstimatorNode {
 public:
  explicit EstimatorNode(talos::Node& node)
      : pose_{node.publish<::geometry::PoseEstimate>("/pose_estimator/state")} {
  }

  talos::Publisher<::geometry::PoseEstimate> pose_;
};

// Test node that publishes hardware driver station packet.
class HardwareNode {
 public:
  explicit HardwareNode(talos::Node& node)
      : ds_{node.publish<hardware::Packet>("/hw/state/driver_station")},
        raw_{node.publish<hardware::Packet>("/hw/state")} {}

  talos::Publisher<hardware::Packet> ds_;
  talos::Publisher<hardware::Packet> raw_;
};

struct FrameHolder {
  std::vector<std::byte> bytes;

  const Frame* get() const {
    if (bytes.empty()) return nullptr;
    const auto payload = telemetry_wire::payload(bytes);
    if (payload.size() != bytes.size()) return nullptr;
    flatbuffers::Verifier verifier{
        reinterpret_cast<const std::uint8_t*>(payload.data()), payload.size()};
    if (!Talos::Telemetry::VerifySizePrefixedFrameBuffer(verifier)) {
      return nullptr;
    }
    return Talos::Telemetry::GetSizePrefixedFrame(bytes.data());
  }

  const Frame* operator->() const { return get(); }
  explicit operator const Frame*() const { return get(); }
  bool operator==(std::nullptr_t) const { return get() == nullptr; }
  bool operator!=(std::nullptr_t) const { return get() != nullptr; }
};

FrameHolder LatestFrame(const Sim& sim, std::string_view topic) {
  auto bytes = sim.latest_bytes(topic);
  if (!bytes) return FrameHolder{};
  return FrameHolder{std::move(*bytes)};
}

std::map<std::string, double> DecodeChannels(const Telemetry& telemetry,
                                             const Frame* frame) {
  std::map<std::string, double> out;
  if (!frame || !frame->values()) return out;
  const auto& dict = telemetry.dictionary();
  for (std::size_t i = 0; i < dict.size(); ++i) {
    const double val = frame->values()->Get(i);
    if (!std::isnan(val)) {
      out[dict[i].name] = val;
    }
  }
  return out;
}

std::map<std::string, double> DecodeChannels(const Telemetry& telemetry,
                                             const FrameHolder& holder) {
  return DecodeChannels(telemetry, holder.get());
}

std::map<std::string, const ChannelDef*> DictionaryMap(
    const Telemetry& telemetry) {
  std::map<std::string, const ChannelDef*> out;
  for (const auto& def : telemetry.dictionary()) {
    out[def.name] = &def;
  }
  return out;
}

TEST(TelemetryTest, SchemaLessStateIsSilentlyExcluded) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.demo]
node = "//demo:node"

[subsystems.hw]
node = "//hw:node"

[subsystems.hw.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  Sim sim = Sim::FromToml(kConfig);
  sim.add<HardwareNode>("hw");
  auto& shapes = sim.add<ShapesNode>("demo");

  ::testing::internal::CaptureStderr();
  auto& telemetry = sim.add<Telemetry>("telemetry");
  const std::string stderr_output = ::testing::internal::GetCapturedStderr();
  // Only the budget report; nothing about the excluded schema-less topic.
  EXPECT_EQ(stderr_output.find("hw/state"), std::string::npos) << stderr_output;
  EXPECT_NE(stderr_output.find("channels; worst-case value frame"),
            std::string::npos)
      << stderr_output;

  const auto dict = DictionaryMap(telemetry);
  EXPECT_TRUE(dict.contains("demo/state/position_m"));
  EXPECT_TRUE(dict.contains("demo/status/samples_m/0"));
  for (const auto& [name, channel] : dict) {
    EXPECT_NE(name, "hw/state");
    EXPECT_FALSE(name.starts_with("hw/state/") &&
                 !name.starts_with("hw/state/driver_station/"));
  }

  shapes.mixed_.send(schema_test::Mixed{true,
                                        4.5,
                                        schema_test::Level::TOP,
                                        1234,
                                        {schema_test::Mode::HIGH, 1.25f},
                                        0.8f});
  sim.step(20ms);
  const auto frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  EXPECT_DOUBLE_EQ(DecodeChannels(telemetry, frame).at("demo/state/position_m"),
                   4.5);
}

TEST(TelemetryTest, SchemaLessPoseTopicIsStillAnError) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"
pose_topic = "/hw/state"

[subsystems.hw]
node = "//hw:node"

[subsystems.hw.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  Sim sim = Sim::FromToml(kConfig);
  sim.add<HardwareNode>("hw");
  EXPECT_THROW(sim.add<Telemetry>("telemetry"), std::invalid_argument);
}

TEST(TelemetryTest, FlatteningAndUnits) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.demo]
node = "//demo:node"

[subsystems.demo.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";

  Sim sim = Sim::FromToml(kConfig);
  auto& shapes = sim.add<ShapesNode>("demo");
  auto& telemetry = sim.add<Telemetry>("telemetry");

  const auto dict = DictionaryMap(telemetry);

  // Check nested struct and scalar field names and derived units
  ASSERT_TRUE(dict.contains("demo/state/flag"));
  EXPECT_EQ(dict.at("demo/state/flag")->unit, "");

  ASSERT_TRUE(dict.contains("demo/state/position_m"));
  EXPECT_EQ(dict.at("demo/state/position_m")->unit, "m");

  ASSERT_TRUE(dict.contains("demo/state/level"));
  EXPECT_EQ(dict.at("demo/state/level")->unit, "");
  EXPECT_FALSE(dict.at("demo/state/level")->labels.empty());

  ASSERT_TRUE(dict.contains("demo/state/sample_count"));
  EXPECT_EQ(dict.at("demo/state/sample_count")->unit, "");

  ASSERT_TRUE(dict.contains("demo/state/inner/mode"));
  EXPECT_EQ(dict.at("demo/state/inner/mode")->unit, "");
  EXPECT_FALSE(dict.at("demo/state/inner/mode")->labels.empty());

  ASSERT_TRUE(dict.contains("demo/state/inner/offset_m"));
  EXPECT_EQ(dict.at("demo/state/inner/offset_m")->unit, "m");

  ASSERT_TRUE(dict.contains("demo/state/speed_frac"));
  EXPECT_EQ(dict.at("demo/state/speed_frac")->unit, "");

  // Check array elements
  ASSERT_TRUE(dict.contains("demo/status/samples_m/0"));
  ASSERT_TRUE(dict.contains("demo/status/samples_m/1"));
  ASSERT_TRUE(dict.contains("demo/status/samples_m/2"));
  EXPECT_EQ(dict.at("demo/status/samples_m/0")->unit, "m");
  EXPECT_EQ(dict.at("demo/status/samples_m/1")->unit, "m");
  EXPECT_EQ(dict.at("demo/status/samples_m/2")->unit, "m");

  ASSERT_TRUE(dict.contains("demo/status/modes/0"));
  ASSERT_TRUE(dict.contains("demo/status/modes/1"));
  EXPECT_EQ(dict.at("demo/status/modes/0")->unit, "");

  ASSERT_TRUE(dict.contains("demo/status/inners/0/offset_m"));
  ASSERT_TRUE(dict.contains("demo/status/inners/1/offset_m"));
  EXPECT_EQ(dict.at("demo/status/inners/0/offset_m")->unit, "m");

  // Publish values and verify they are decoded into values array
  schema_test::Mixed mixed{true,
                           4.5,
                           schema_test::Level::TOP,
                           1234,
                           {schema_test::Mode::HIGH, 1.25f},
                           0.8f};
  shapes.mixed_.send(mixed);

  sim.step(20ms);

  const auto frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  const auto channels = DecodeChannels(telemetry, frame);

  EXPECT_DOUBLE_EQ(channels.at("demo/state/flag"), 1.0);
  EXPECT_DOUBLE_EQ(channels.at("demo/state/position_m"), 4.5);
  EXPECT_DOUBLE_EQ(channels.at("demo/state/level"),
                   static_cast<double>(schema_test::Level::TOP));
  EXPECT_DOUBLE_EQ(channels.at("demo/state/sample_count"), 1234.0);
  EXPECT_DOUBLE_EQ(channels.at("demo/state/inner/mode"),
                   static_cast<double>(schema_test::Mode::HIGH));
  EXPECT_NEAR(channels.at("demo/state/inner/offset_m"), 1.25, 1e-5);
  EXPECT_NEAR(channels.at("demo/state/speed_frac"), 0.8, 1e-5);
}

TEST(TelemetryTest, IncludeExcludeFiltering) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.demo]
node = "//demo:node"

[subsystems.demo.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
include = ["*/status"]
exclude = ["*state*"]
)";

  Sim sim = Sim::FromToml(kConfig);
  sim.add<ShapesNode>("demo");
  auto& telemetry = sim.add<Telemetry>("telemetry");

  const auto dict = DictionaryMap(telemetry);

  // Status topics included
  EXPECT_TRUE(dict.contains("demo/status/samples_m/0"));
  // State topics excluded
  EXPECT_FALSE(dict.contains("demo/state/flag"));
  EXPECT_FALSE(dict.contains("demo/state/position_m"));
}

TEST(TelemetryTest, ChannelExcludePreservesOtherChannelsAndShrinksBudget) {
  const std::string config = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.demo]
node = "//demo:node"

[subsystems.demo.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  Sim baseline = Sim::FromToml(config);
  baseline.add<ShapesNode>("demo");
  const auto& original = baseline.add<Telemetry>("telemetry");
  auto expected = DictionaryMap(original);
  ASSERT_EQ(expected.erase("demo/state/sample_count"), 1u);
  const auto original_bytes =
      MeasureValueFrameBytes(original.dictionary().size());

  for (const std::string pattern :
       {"demo/state/sample_count", "/demo/state/sample_count"}) {
    std::string filtered_config = config;
    filtered_config += "exclude = [\"";
    filtered_config += pattern;
    filtered_config += "\"]\n";
    Sim filtered = Sim::FromToml(filtered_config);
    auto& shapes = filtered.add<ShapesNode>("demo");
    const auto& telemetry = filtered.add<Telemetry>("telemetry");
    const auto actual = DictionaryMap(telemetry);
    ASSERT_EQ(actual.size(), expected.size());
    for (const auto& [name, definition] : expected) {
      EXPECT_TRUE(actual.contains(name)) << name;
    }
    EXPECT_FALSE(actual.contains("demo/state/sample_count"));
    EXPECT_EQ(MeasureValueFrameBytes(telemetry.dictionary().size()),
              original_bytes - sizeof(double));

    shapes.mixed_.send(schema_test::Mixed{true,
                                          4.5,
                                          schema_test::Level::TOP,
                                          1234,
                                          {schema_test::Mode::HIGH, 1.25f},
                                          0.8f});
    filtered.step(20ms);
    const auto frame = LatestFrame(filtered, "/talos/telemetry");
    ASSERT_NE(frame, nullptr);
    const auto values = DecodeChannels(telemetry, frame);
    EXPECT_DOUBLE_EQ(values.at("demo/state/position_m"), 4.5);
    EXPECT_FALSE(values.contains("demo/state/sample_count"));
    filtered.step(200ms);
    shapes.arrays_.send(schema_test::Arrays{});
    filtered.step(20ms);
    const auto stale = LatestFrame(filtered, "/talos/telemetry");
    ASSERT_NE(stale, nullptr);
    EXPECT_FALSE(
        DecodeChannels(telemetry, stale).contains("demo/state/position_m"));
  }
}

TEST(TelemetryTest, NaNOnStale) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.demo]
node = "//demo:node"

[subsystems.demo.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
stale_timeout_us = 50000
)";

  Sim sim = Sim::FromToml(kConfig);
  auto& shapes = sim.add<ShapesNode>("demo");
  auto& telemetry = sim.add<Telemetry>("telemetry");

  schema_test::Mixed mixed{true,
                           4.5,
                           schema_test::Level::TOP,
                           1234,
                           {schema_test::Mode::HIGH, 1.25f},
                           0.8f};
  shapes.mixed_.send(mixed);

  sim.step(20ms);
  const auto frame1 = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame1, nullptr);
  auto channels1 = DecodeChannels(telemetry, frame1);
  EXPECT_TRUE(channels1.contains("demo/state/position_m"));

  // Advance time past stale_timeout_us without new publications
  sim.step(100ms);
  // Send arrays message to trigger a frame while mixed is now stale
  schema_test::Arrays arrays{};
  shapes.arrays_.send(arrays);
  sim.step(20ms);

  const auto frame2 = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame2, nullptr);
  auto channels2 = DecodeChannels(telemetry, frame2);
  // demo/state was stale, so its channels are NaN (DecodeChannels drops NaNs)
  EXPECT_FALSE(channels2.contains("demo/state/position_m"));
  EXPECT_TRUE(channels2.contains("demo/status/samples_m/0"));
}

TEST(TelemetryTest, PoseTopicExtraction) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"
pose_topic = "/pose_estimator/state"

[subsystems.pose_estimator]
node = "//estimator:node"

[subsystems.pose_estimator.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";

  Sim sim = Sim::FromToml(kConfig);
  auto& estimator = sim.add<EstimatorNode>("pose_estimator");
  auto& telemetry = sim.add<Telemetry>("telemetry");

  const auto dict = DictionaryMap(telemetry);
  ASSERT_TRUE(dict.contains("pose/valid"));

  ::geometry::PoseEstimate estimate{
      0,
      ::geometry::Pose2d{3.25, 1.5, 0.75},
      ::geometry::Twist2d{0.0, 0.0, 0.0},
      true,
      true,
      0,
      0,
  };
  estimator.pose_.send(estimate);

  sim.step(20ms);

  const auto frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->chassis(), nullptr);
  EXPECT_DOUBLE_EQ(frame->chassis()->x(), 3.25);
  EXPECT_DOUBLE_EQ(frame->chassis()->y(), 1.5);
  EXPECT_DOUBLE_EQ(frame->chassis()->yaw(), 0.75);

  const auto channels = DecodeChannels(telemetry, frame);
  EXPECT_DOUBLE_EQ(channels.at("pose/valid"), 1.0);

  // Send invalid pose
  estimate = ::geometry::PoseEstimate{
      0,
      ::geometry::Pose2d{3.25, 1.5, 0.75},
      ::geometry::Twist2d{0.0, 0.0, 0.0},
      false,
      true,
      0,
      0,
  };
  estimator.pose_.send(estimate);
  sim.step(20ms);

  const auto invalid_frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(invalid_frame, nullptr);
  EXPECT_DOUBLE_EQ(invalid_frame->chassis()->x(), 0.0);
  EXPECT_DOUBLE_EQ(invalid_frame->chassis()->y(), 0.0);
  EXPECT_DOUBLE_EQ(invalid_frame->chassis()->yaw(), 0.0);
  const auto invalid_channels = DecodeChannels(telemetry, invalid_frame);
  EXPECT_DOUBLE_EQ(invalid_channels.at("pose/valid"), 0.0);
}

TEST(TelemetryTest, DriverStationDecoding) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.hw]
node = "//hw:node"

[subsystems.hw.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";

  Sim sim = Sim::FromToml(kConfig);
  auto& hw = sim.add<HardwareNode>("hw");
  auto& telemetry = sim.add<Telemetry>("telemetry");

  const auto dict = DictionaryMap(telemetry);
  ASSERT_TRUE(dict.contains("hw/state/driver_station/connected"));
  ASSERT_TRUE(dict.contains("hw/state/driver_station/enabled"));
  ASSERT_TRUE(dict.contains("hw/state/driver_station/teleop"));
  ASSERT_TRUE(dict.contains("hw/state/driver_station/autonomous"));
  ASSERT_TRUE(dict.contains("hw/state/driver_station/alliance"));
  ASSERT_TRUE(dict.contains("hw/state/driver_station/match_time_s"));
  EXPECT_EQ(dict.at("hw/state/driver_station/match_time_s")->unit, "s");
  EXPECT_EQ(dict.at("hw/state/driver_station/alliance")->labels,
            (std::vector<std::string>{"unknown", "red", "blue"}));

  driver_station::DriverStationData ds_data{};
  ds_data.fms.flags = driver_station::kDsAttached | driver_station::kEnabled |
                      driver_station::kTeleop;
  ds_data.fms.alliance = driver_station::Alliance::kRed;
  ds_data.fms.match_time_s = 75.5f;

  hardware::Packet packet{};
  packet.size =
      static_cast<uint32_t>(driver_station::Encode(ds_data, packet.data));
  hw.ds_.send(packet);

  sim.step(20ms);

  const auto frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  const auto channels = DecodeChannels(telemetry, frame);

  EXPECT_DOUBLE_EQ(channels.at("hw/state/driver_station/connected"), 1.0);
  EXPECT_DOUBLE_EQ(channels.at("hw/state/driver_station/enabled"), 1.0);
  EXPECT_DOUBLE_EQ(channels.at("hw/state/driver_station/teleop"), 1.0);
  EXPECT_DOUBLE_EQ(channels.at("hw/state/driver_station/autonomous"), 0.0);
  EXPECT_DOUBLE_EQ(channels.at("hw/state/driver_station/alliance"), 1.0);
  EXPECT_NEAR(channels.at("hw/state/driver_station/match_time_s"), 75.5, 1e-3);
}

// Node with 150 channels to exceed 1200-byte frame budget.
class GiantNode {
 public:
  explicit GiantNode(talos::Node& node)
      : a_{node.publish<schema_test::Arrays>("/giant/state/a")},
        b_{node.publish<schema_test::Arrays>("/giant/state/b")},
        c_{node.publish<schema_test::Arrays>("/giant/state/c")},
        d_{node.publish<schema_test::Arrays>("/giant/state/d")},
        e_{node.publish<schema_test::Arrays>("/giant/state/e")},
        f_{node.publish<schema_test::Arrays>("/giant/state/f")},
        g_{node.publish<schema_test::Arrays>("/giant/state/g")},
        h_{node.publish<schema_test::Arrays>("/giant/state/h")},
        i_{node.publish<schema_test::Arrays>("/giant/state/i")},
        j_{node.publish<schema_test::Arrays>("/giant/state/j")},
        k_{node.publish<schema_test::Arrays>("/giant/state/k")},
        l_{node.publish<schema_test::Arrays>("/giant/state/l")} {}

  talos::Publisher<schema_test::Arrays> a_, b_, c_, d_, e_, f_, g_, h_, i_, j_,
      k_, l_;
};

TEST(TelemetryTest, BudgetCheckFailureMessage) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.giant]
node = "//giant:node"

[subsystems.giant.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";

  Sim sim = Sim::FromToml(kConfig);
  sim.add<GiantNode>("giant");

  EXPECT_THROW(sim.add<Telemetry>("telemetry"), std::runtime_error);
}

// Each topic contributes one channel after excluding its offset field.
class BoundaryChannelsNode {
 public:
  explicit BoundaryChannelsNode(talos::Node& node) {
    const auto count = node.param<std::int64_t>("channel_count", 0);
    for (std::int64_t i = 0; i < count; ++i) {
      publishers_.push_back(node.publish<schema_test::Inner>(
          "/budget/state/" + std::to_string(i)));
    }
  }

  void send() {
    for (auto& publisher : publishers_) {
      publisher.send(schema_test::Inner{schema_test::Mode::LOW, 1.0f});
    }
  }

 private:
  std::vector<talos::Publisher<schema_test::Inner>> publishers_;
};

TEST(TelemetryTest, LargestAcceptedLiveValueFrameFitsBudget) {
  std::size_t channel_count = 1;
  while (MeasureValueFrameBytes(channel_count + 1) <=
         telemetry_wire::kMaxValueFrameBytes) {
    ++channel_count;
  }
  ASSERT_LE(MeasureValueFrameBytes(channel_count),
            telemetry_wire::kMaxValueFrameBytes);
  ASSERT_GT(MeasureValueFrameBytes(channel_count + 1),
            telemetry_wire::kMaxValueFrameBytes);

  const auto config = [](std::size_t count) {
    return std::string{R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.budget]
node = "//budget:node"
channel_count = )"} +
           std::to_string(count) + R"(
[subsystems.budget.motors.m]
can_id = 1
[subsystems.telemetry]
node = "//talOS/telemetry:node"
exclude = ["budget/state/*/offset_m"]
)";
  };

  // The built-in pose/valid channel accounts for the remaining entry.
  Sim sim = Sim::FromToml(config(channel_count - 1));
  auto& source = sim.add<BoundaryChannelsNode>("budget");
  const auto& telemetry = sim.add<Telemetry>("telemetry");
  ASSERT_EQ(telemetry.dictionary().size(), channel_count);
  source.send();
  sim.step(20ms);
  source.send();
  sim.step(20ms);
  const auto frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  ASSERT_EQ(frame->dictionary(), nullptr);
  ASSERT_NE(frame->stream_id(), 0u);
  ASSERT_NE(frame->sequence_id(), 0u);
  ASSERT_NE(frame->timestamp_ns(), 0u);
  ASSERT_NE(frame->dictionary_id(), 0u);
  ASSERT_EQ(frame->values()->size(), channel_count);
  EXPECT_LE(frame.bytes.size(), telemetry_wire::kMaxValueFrameBytes);
  EXPECT_LE(frame.bytes.size(), MeasureValueFrameBytes(channel_count));

  Sim oversized = Sim::FromToml(config(channel_count));
  oversized.add<BoundaryChannelsNode>("budget");
  EXPECT_THROW(oversized.add<Telemetry>("telemetry"), std::runtime_error);
}

// `/budget/state` as a struct of `field_count` doubles, each named
// `name_bytes` long; the schema is built by hand so a test sets any length.
class LongNamesNode {
 public:
  explicit LongNamesNode(talos::Node& node) {
    const auto count =
        static_cast<std::size_t>(node.param<std::int64_t>("field_count", 1));
    const auto name_bytes =
        static_cast<std::size_t>(node.param<std::int64_t>("name_bytes", 1));
    flatbuffers::FlatBufferBuilder builder;
    std::vector<flatbuffers::Offset<reflection::Field>> fields;
    for (std::size_t i = 0; i < count; ++i) {
      std::string name = std::to_string(i);
      name.resize(name_bytes, 'x');
      const auto field_name = builder.CreateString(name);
      reflection::TypeBuilder type_builder{builder};
      type_builder.add_base_type(reflection::BaseType::Double);
      type_builder.add_index(-1);
      const auto type = type_builder.Finish();
      reflection::FieldBuilder field_builder{builder};
      field_builder.add_name(field_name);
      field_builder.add_type(type);
      field_builder.add_id(static_cast<std::uint16_t>(i));
      field_builder.add_offset(static_cast<std::uint16_t>(i * sizeof(double)));
      fields.push_back(field_builder.Finish());
    }
    const auto object_name = builder.CreateString("budget.State");
    const auto field_vector = builder.CreateVector(fields);
    reflection::ObjectBuilder object_builder{builder};
    object_builder.add_name(object_name);
    object_builder.add_fields(field_vector);
    object_builder.add_is_struct(true);
    object_builder.add_minalign(8);
    object_builder.add_bytesize(
        static_cast<std::int32_t>(count * sizeof(double)));
    const auto object = object_builder.Finish();
    const auto objects = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Object>>{object});
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder schema_builder{builder};
    schema_builder.add_objects(objects);
    schema_builder.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, schema_builder.Finish());
    schema_.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                   builder.GetSize());
    talos::detail::NodeAccess::add_publisher(
        node, "/budget/state", count * sizeof(double), 8,
        talos::schema::Identity{"budget.State", 12345, schema_});
  }

 private:
  std::string schema_;
};

// LongNamesNode's channels are this prefix plus the field name, so this is
// the longest field name whose channel Studio still decodes.
constexpr std::string_view kLongNamesPrefix = "budget/state/";
constexpr std::size_t kLongestFieldName =
    telemetry_wire::kMaxDictionaryStringBytes - kLongNamesPrefix.size();

// The channel LongNamesNode's first field produces.
std::string LongChannelName(std::size_t name_bytes) {
  std::string field = "0";
  field.resize(name_bytes, 'x');
  return std::string{kLongNamesPrefix} + field;
}

std::string LongNamesConfig(std::size_t field_count, std::size_t name_bytes,
                            std::string_view telemetry_keys = "") {
  return R"(
[robot]
period_us = 10000
[subsystems.budget]
node = "//budget:node"
field_count = )" +
         std::to_string(field_count) +
         "\nname_bytes = " + std::to_string(name_bytes) + R"(
[subsystems.budget.motors.m]
can_id = 1
[subsystems.telemetry]
node = "//talOS/telemetry:node"
)" + std::string{telemetry_keys};
}

TEST(TelemetryTest, DictionaryFrameMustFitSharedMemorySlot) {
  const std::vector<ChannelDef> small{{"pose/valid", "", {}}};
  EXPECT_LE(MeasureDictionaryFrameBytes(small), telemetry_wire::slot_bytes);
  const std::vector<ChannelDef> large{
      {"status", "", {std::string(telemetry_wire::slot_bytes, 'x')}}};
  EXPECT_LT(MeasureValueFrameBytes(large.size()),
            telemetry_wire::kMaxValueFrameBytes);
  EXPECT_GT(MeasureDictionaryFrameBytes(large), telemetry_wire::slot_bytes);

  // 64 channels of the longest name Studio decodes: a small value frame, but
  // a dictionary frame over the slot. The refusal names the largest channels.
  Sim sim = Sim::FromToml(LongNamesConfig(64, kLongestFieldName));
  sim.add<LongNamesNode>("budget");
  try {
    sim.add<Telemetry>("telemetry");
    ADD_FAILURE() << "an over-slot dictionary was accepted";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(
        std::string{error.what()}.find(LongChannelName(kLongestFieldName)),
        std::string::npos)
        << error.what();
  }
}

// Owns a topic whose channels fall under Studio's derived/ prefix.
class DerivedOwnerNode {
 public:
  explicit DerivedOwnerNode(talos::Node& node)
      : state_{node.publish<schema_test::Inner>("/derived/state")} {}

  talos::Publisher<schema_test::Inner> state_;
};

// Studio refuses a whole dictionary over one channel it cannot decode, which
// blanks the stream; construction refuses that channel instead, by name.
TEST(TelemetryTest, ChannelStudioCannotDecodeIsRefusedByName) {
  const auto refusal = [](Sim& sim) -> std::string {
    try {
      sim.add<Telemetry>("telemetry");
    } catch (const std::invalid_argument& error) {
      return error.what();
    }
    return "";
  };
  Sim at_limit = Sim::FromToml(LongNamesConfig(1, kLongestFieldName));
  at_limit.add<LongNamesNode>("budget");
  EXPECT_EQ(refusal(at_limit), "");

  const std::string over_limit = LongChannelName(kLongestFieldName + 1);
  Sim over = Sim::FromToml(LongNamesConfig(1, kLongestFieldName + 1));
  over.add<LongNamesNode>("budget");
  const std::string message = refusal(over);
  EXPECT_NE(message.find("'" + over_limit + "'"), std::string::npos) << message;

  // The remedy the refusal points at: excluding the channel.
  Sim excluded = Sim::FromToml(LongNamesConfig(
      1, kLongestFieldName + 1, "exclude = [\"" + over_limit + "\"]\n"));
  excluded.add<LongNamesNode>("budget");
  EXPECT_EQ(refusal(excluded), "");

  Sim derived = Sim::FromToml(R"(
[robot]
period_us = 10000
[subsystems.derived]
node = "//derived:node"
[subsystems.derived.motors.m]
can_id = 1
[subsystems.telemetry]
node = "//talOS/telemetry:node"
)");
  derived.add<DerivedOwnerNode>("derived");
  const std::string reserved = refusal(derived);
  EXPECT_NE(reserved.find("'derived/state/mode'"), std::string::npos)
      << reserved;
}

TEST(TelemetryTest, PoseTopicMissingErrors) {
  const std::string kAbsentConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"
pose_topic = "/absent/state"

[subsystems.demo.motors.m_demo]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  Sim sim1 = Sim::FromToml(kAbsentConfig);
  EXPECT_THROW(sim1.add<Telemetry>("telemetry"), std::invalid_argument);

  const std::string kNoPoseFieldConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"
pose_topic = "/demo/state"

[subsystems.demo]
node = "//demo:node"

[subsystems.demo.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  Sim sim2 = Sim::FromToml(kNoPoseFieldConfig);
  sim2.add<ShapesNode>("demo");
  EXPECT_THROW(sim2.add<Telemetry>("telemetry"), std::invalid_argument);
}

class BadBfbsNode {
 public:
  explicit BadBfbsNode(talos::Node& node) {
    using talos::detail::NodeAccess;
    NodeAccess::add_publisher(
        node, "/bad/state", 16, 8,
        talos::schema::Identity{"bad.State", 12345, "invalid schema bytes"});
  }
};

class MissingObjectNode {
 public:
  explicit MissingObjectNode(talos::Node& node) {
    using talos::detail::NodeAccess;
    const auto schema = talos::schema::Of<schema_test::Mixed>();
    NodeAccess::add_publisher(
        node, "/missing_obj/state", 16, 8,
        talos::schema::Identity{"bad.NonExistent", 12345, schema.bfbs});
  }
};

TEST(TelemetryTest, BadBfbsAndMissingObjectErrors) {
  const std::string kBadBfbsConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.bad]
node = "//bad:node"

[subsystems.bad.motors.m_bad]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  Sim sim1 = Sim::FromToml(kBadBfbsConfig);
  sim1.add<BadBfbsNode>("bad");
  EXPECT_THROW(sim1.add<Telemetry>("telemetry"), std::invalid_argument);

  const std::string kMissingObjConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.missing]
node = "//missing:node"

[subsystems.missing.motors.m_missing]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  Sim sim2 = Sim::FromToml(kMissingObjConfig);
  sim2.add<MissingObjectNode>("missing");
  EXPECT_THROW(sim2.add<Telemetry>("telemetry"), std::invalid_argument);
}

TEST(TelemetryTest, EnumLabelsExtraction) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.demo]
node = "//demo:node"

[subsystems.demo.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  Sim sim = Sim::FromToml(kConfig);
  sim.add<ShapesNode>("demo");
  auto& telemetry = sim.add<Telemetry>("telemetry");

  const auto dict = DictionaryMap(telemetry);
  ASSERT_TRUE(dict.contains("demo/state/inner/mode"));
  const auto& mode_labels = dict.at("demo/state/inner/mode")->labels;
  ASSERT_GE(mode_labels.size(), 5u);
  EXPECT_EQ(mode_labels[0], "OFF");
  EXPECT_EQ(mode_labels[3], "LOW");
  EXPECT_EQ(mode_labels[4], "HIGH");
}

TEST(TelemetryTest, AllSourcesStaleEmitsOneClearingFrameThenResumes) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"
pose_topic = "/pose_estimator/state"

[subsystems.pose_estimator]
node = "//estimator:node"

[subsystems.pose_estimator.motors.m_est]
can_id = 1

[subsystems.demo]
node = "//demo:node"

[subsystems.demo.motors.m_demo]
can_id = 2
[subsystems.telemetry]
node = "//talOS/telemetry:node"
stale_timeout_us = 50000
)";
  Sim sim = Sim::FromToml(kConfig);
  auto& estimator = sim.add<EstimatorNode>("pose_estimator");
  auto& shapes = sim.add<ShapesNode>("demo");
  auto& telemetry = sim.add<Telemetry>("telemetry");

  ::geometry::PoseEstimate estimate{
      0,
      ::geometry::Pose2d{2.0, 3.0, 1.0},
      ::geometry::Twist2d{0.0, 0.0, 0.0},
      true,
      true,
      0,
      0,
  };
  estimator.pose_.send(estimate);
  sim.step(40ms);

  auto frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  EXPECT_DOUBLE_EQ(frame->chassis()->x(), 2.0);
  auto channels = DecodeChannels(telemetry, frame);
  EXPECT_DOUBLE_EQ(channels.at("pose/valid"), 1.0);
  const auto fresh_sequence = frame->sequence_id();

  // Advance time past stale_timeout_us (50ms) without sending new pose
  sim.step(100ms);
  // The last fresh source becoming stale emits one clearing frame, even when
  // there is no other source left to trigger normal publication.
  frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  const auto stale_sequence = frame->sequence_id();
  EXPECT_EQ(stale_sequence, fresh_sequence + 1);
  EXPECT_DOUBLE_EQ(frame->chassis()->x(), 0.0);
  EXPECT_DOUBLE_EQ(frame->chassis()->y(), 0.0);
  EXPECT_DOUBLE_EQ(frame->chassis()->z(), 0.0);
  EXPECT_DOUBLE_EQ(frame->chassis()->roll(), 0.0);
  EXPECT_DOUBLE_EQ(frame->chassis()->pitch(), 0.0);
  EXPECT_DOUBLE_EQ(frame->chassis()->yaw(), 0.0);
  for (std::size_t i = 0; i < telemetry.dictionary().size(); ++i) {
    if (telemetry.dictionary()[i].name == "pose/valid") {
      EXPECT_DOUBLE_EQ(frame->values()->Get(i), 0.0);
    } else {
      EXPECT_TRUE(std::isnan(frame->values()->Get(i)));
    }
  }
  sim.step(100ms);
  frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  EXPECT_EQ(frame->sequence_id(), stale_sequence);

  // Publish arrays on demo node so telemetry produces a new frame while pose is
  // stale
  schema_test::Arrays arrays{};
  shapes.arrays_.send(arrays);
  sim.step(20ms);

  frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  EXPECT_GT(frame->sequence_id(), stale_sequence);
  EXPECT_DOUBLE_EQ(frame->chassis()->x(), 0.0);
  EXPECT_DOUBLE_EQ(frame->chassis()->y(), 0.0);
  EXPECT_DOUBLE_EQ(frame->chassis()->yaw(), 0.0);
  channels = DecodeChannels(telemetry, frame);
  EXPECT_DOUBLE_EQ(channels.at("pose/valid"), 0.0);
  // Pose estimator channels are NaN when stale
  EXPECT_FALSE(channels.contains("pose_estimator/state/twist/vx_mps"));
}

// The viewer stream outlives its viewers: a reader that registered and then
// stopped reading (a stalled Studio bridge) is lapped, not obeyed. Under
// drop_newest the writer refused every frame past the ring, for every reader.
// testing::Sim keeps byte topics in memory without a ring, so the topic is
// exercised on RTMS directly, as Node::publish_bytes opens it without
// drop_newest.
TEST(TelemetryTopic, AStoppedReaderDoesNotStopPublication) {
  ASSERT_FALSE(kTopicOptions.drop_newest);
  const std::string topic = "/test/telemetry/p" + std::to_string(::getpid());
  RTMSQueue writer{topic,
                   telemetry_wire::slot_bytes,
                   kMessageAlignment,
                   schema::Bytes(),
                   kTopicOptions.slots,
                   {.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                    .read_mode = ReadMode::SEQUENCE,
                    .publisher = true}};
  RTMSQueue stalled{topic,
                    telemetry_wire::slot_bytes,
                    kMessageAlignment,
                    schema::Bytes(),
                    telemetry_wire::slot_count,
                    {.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                     .read_mode = ReadMode::SEQUENCE}};
  const auto reader = stalled.register_reader();
  ASSERT_TRUE(reader.has_value());

  const std::uint64_t frames = 3 * telemetry_wire::slot_count;
  std::vector<std::byte> frame(64, std::byte{1});
  for (std::uint64_t i = 0; i < frames; ++i) {
    EXPECT_EQ(writer.write(frame).result, WriteResult::SUCCESS) << i;
  }

  // Resuming, the reader gets the oldest intact frame and the count it missed.
  std::vector<std::byte> slot(telemetry_wire::slot_bytes);
  MessageInfo info{};
  ASSERT_EQ(stalled.read_next(*reader, slot, info), ReadResult::OK);
  EXPECT_EQ(info.dropped, frames - (telemetry_wire::slot_count - 1));
  stalled.release_reader(*reader);
}

}  // namespace
}  // namespace talos::telemetry
