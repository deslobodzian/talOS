#include "talOS/telemetry/node.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "geometry/geometry_generated.h"
#include "schema_test/shapes_generated.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/node/testing.h"
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
  EXPECT_TRUE(stderr_output.empty()) << stderr_output;

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
  try {
    sim.add<Telemetry>("telemetry");
    FAIL() << "expected exception for schema-less pose_topic";
  } catch (const std::invalid_argument& err) {
    EXPECT_NE(
        std::string{err.what()}.find("pose topic '/hw/state' has no schema"),
        std::string::npos);
  }
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

  try {
    sim.add<Telemetry>("telemetry");
    FAIL() << "expected frame size budget failure";
  } catch (const std::runtime_error& err) {
    const std::string msg = err.what();
    EXPECT_NE(msg.find("worst-case value frame is"), std::string::npos);
    EXPECT_NE(msg.find("over the 1200-byte budget"), std::string::npos);
    EXPECT_NE(msg.find("Largest sources:"), std::string::npos);
    EXPECT_NE(msg.find("/giant/state/a"), std::string::npos);
  }
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

// A single scalar keeps the value frame small while its metadata exceeds a
// dictionary slot. BFBS permits long field names even with short topic names.
class LargeDictionaryNode {
 public:
  explicit LargeDictionaryNode(talos::Node& node) {
    flatbuffers::FlatBufferBuilder builder;
    const auto field_name =
        builder.CreateString(std::string(telemetry_wire::slot_bytes, 'x'));
    const auto object_name = builder.CreateString("budget.State");
    reflection::TypeBuilder type_builder{builder};
    type_builder.add_base_type(reflection::BaseType::Double);
    type_builder.add_index(-1);
    const auto type = type_builder.Finish();
    reflection::FieldBuilder field_builder{builder};
    field_builder.add_name(field_name);
    field_builder.add_type(type);
    const auto field = field_builder.Finish();
    const auto fields = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Field>>{field});
    reflection::ObjectBuilder object_builder{builder};
    object_builder.add_name(object_name);
    object_builder.add_fields(fields);
    object_builder.add_is_struct(true);
    object_builder.add_minalign(8);
    object_builder.add_bytesize(8);
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
        node, "/budget/state", 8, 8,
        talos::schema::Identity{"budget.State", 12345, schema_});
  }

 private:
  std::string schema_;
};

TEST(TelemetryTest, DictionaryFrameMustFitSharedMemorySlot) {
  const std::vector<ChannelDef> small{{"pose/valid", "", {}}};
  EXPECT_LE(MeasureDictionaryFrameBytes(small), telemetry_wire::slot_bytes);
  const std::vector<ChannelDef> large{
      {"status", "", {std::string(telemetry_wire::slot_bytes, 'x')}}};
  EXPECT_LT(MeasureValueFrameBytes(large.size()),
            telemetry_wire::kMaxValueFrameBytes);
  EXPECT_GT(MeasureDictionaryFrameBytes(large), telemetry_wire::slot_bytes);

  Sim sim = Sim::FromToml(R"(
[robot]
period_us = 10000
[subsystems.budget]
node = "//budget:node"
[subsystems.budget.motors.m]
can_id = 1
[subsystems.telemetry]
node = "//talOS/telemetry:node"
)");
  sim.add<LargeDictionaryNode>("budget");
  try {
    sim.add<Telemetry>("telemetry");
    FAIL() << "expected dictionary slot budget failure";
  } catch (const std::runtime_error& err) {
    const std::string message{err.what()};
    EXPECT_NE(message.find("dictionary frame is"), std::string::npos);
    EXPECT_NE(message.find("16384-byte shared-memory slot budget"),
              std::string::npos);
  }
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
  try {
    sim1.add<Telemetry>("telemetry");
    FAIL() << "expected exception for absent pose_topic";
  } catch (const std::invalid_argument& err) {
    EXPECT_NE(std::string{err.what()}.find("/absent/state"), std::string::npos);
  }

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
  try {
    sim2.add<Telemetry>("telemetry");
    FAIL() << "expected exception for topic lacking geometry.Pose2d field";
  } catch (const std::invalid_argument& err) {
    EXPECT_NE(std::string{err.what()}.find("/demo/state"), std::string::npos);
  }
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
  try {
    sim1.add<Telemetry>("telemetry");
    FAIL() << "expected bad schema buffer exception";
  } catch (const std::invalid_argument& err) {
    EXPECT_NE(std::string{err.what()}.find("/bad/state"), std::string::npos);
  }

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
  try {
    sim2.add<Telemetry>("telemetry");
    FAIL() << "expected missing root object exception";
  } catch (const std::invalid_argument& err) {
    EXPECT_NE(std::string{err.what()}.find("/missing_obj/state"),
              std::string::npos);
  }
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

}  // namespace
}  // namespace talos::telemetry
