#include "talOS/telemetry/node.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "geometry/geometry_generated.h"
#include "schema_test/shapes_generated.h"
#include "sim/sim_generated.h"
#include "sim/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/describe.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/node/internal.h"
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

// Test node that publishes the MuJoCo gateway's game pieces.
class PiecesNode {
 public:
  explicit PiecesNode(talos::Node& node)
      : pieces_{node.publish<::sim::GamePieces>(::sim::topics::kPieces)} {}

  talos::Publisher<::sim::GamePieces> pieces_;
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

std::map<std::string, const ChannelDef*> DictionaryMap(
    const Telemetry& telemetry) {
  std::map<std::string, const ChannelDef*> out;
  for (const auto& def : telemetry.dictionary()) {
    out[def.name] = &def;
  }
  return out;
}

// The raw samples one value page carries, by channel name. A keyframe page
// carries every present channel of a single-page tick; absent ones are missing.
std::map<std::string, std::uint64_t> DecodeRaw(const Telemetry& telemetry,
                                               const Frame* frame) {
  std::map<std::string, std::uint64_t> out;
  if (!frame || !frame->channels() || !frame->values()) return out;
  const auto& dict = telemetry.dictionary();
  for (std::size_t i = 0; i < frame->channels()->size(); ++i) {
    out[dict.at(frame->channels()->Get(i)).name] = frame->values()->Get(i);
  }
  return out;
}

// The same samples as numbers, read per channel kind.
std::map<std::string, double> DecodeChannels(const Telemetry& telemetry,
                                             const Frame* frame) {
  std::map<std::string, double> out;
  const auto by_name = DictionaryMap(telemetry);
  for (const auto& [name, raw] : DecodeRaw(telemetry, frame)) {
    switch (by_name.at(name)->kind) {
      case Talos::Telemetry::ChannelKind_Int:
        out[name] = static_cast<double>(static_cast<std::int64_t>(raw));
        break;
      case Talos::Telemetry::ChannelKind_UInt:
      case Talos::Telemetry::ChannelKind_Bool:
        out[name] = static_cast<double>(raw);
        break;
      default:
        out[name] = DoubleFromRaw(raw);
    }
  }
  return out;
}

std::map<std::string, double> DecodeChannels(const Telemetry& telemetry,
                                             const FrameHolder& holder) {
  return DecodeChannels(telemetry, holder.get());
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
  EXPECT_NE(stderr_output.find("value page(s) per tick"), std::string::npos)
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

// REQ-DEV-43: only state and status topics become channels, so the game
// pieces' telemetry topic (its 2,560 fields would also overrun the channel
// cap) puts nothing in the feed or its recording.
TEST(TelemetryTest, TelemetryRoleTopicIsNotObserved) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.demo]
node = "//demo:node"

[subsystems.gateway]
node = "//gateway:node"

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  Sim sim = Sim::FromToml(kConfig);
  sim.add<ShapesNode>("demo");
  sim.add<PiecesNode>("gateway");
  auto& telemetry = sim.add<Telemetry>("telemetry");
  const auto dict = DictionaryMap(telemetry);
  EXPECT_TRUE(dict.contains("demo/state/position_m"));
  for (const auto& [name, channel] : dict) {
    EXPECT_FALSE(name.starts_with("sim/")) << name;
  }
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

TEST(TelemetryTest, ChannelExcludePreservesOtherChannels) {
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

// Absent means only that the topic was not published (or went stale); a NaN
// or infinite reading is a reading, and travels with its bits.
TEST(TelemetryTest, StaleIsAbsentWhileNaNAndInfinityAreReadings) {
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
                           std::numeric_limits<double>::quiet_NaN(),
                           schema_test::Level::TOP,
                           1234,
                           {schema_test::Mode::HIGH, 1.25f},
                           -std::numeric_limits<float>::infinity()};
  shapes.mixed_.send(mixed);

  sim.step(20ms);
  const auto frame1 = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame1, nullptr);
  auto channels1 = DecodeChannels(telemetry, frame1);
  ASSERT_TRUE(channels1.contains("demo/state/position_m"));
  EXPECT_TRUE(std::isnan(channels1.at("demo/state/position_m")));
  ASSERT_TRUE(channels1.contains("demo/state/speed_frac"));
  EXPECT_EQ(channels1.at("demo/state/speed_frac"),
            -std::numeric_limits<double>::infinity());

  // Advance time past stale_timeout_us without new publications
  sim.step(100ms);
  // Send arrays message to trigger a frame while mixed is now stale
  schema_test::Arrays arrays{};
  shapes.arrays_.send(arrays);
  sim.step(20ms);

  const auto frame2 = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame2, nullptr);
  auto channels2 = DecodeChannels(telemetry, frame2);
  // demo/state went stale, so its channels are absent: not carried.
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
  ASSERT_TRUE(dict.contains("hw/state/driver_station/match_type"));
  EXPECT_EQ(dict.at("hw/state/driver_station/match_type")->labels,
            (std::vector<std::string>{"none", "practice", "qualification",
                                      "elimination"}));

  driver_station::DriverStationData ds_data{};
  ds_data.fms.flags = driver_station::kDsAttached | driver_station::kEnabled |
                      driver_station::kTeleop;
  ds_data.fms.alliance = driver_station::Alliance::kRed;
  ds_data.fms.match_time_s = 75.5f;
  ds_data.fms.station = 3;
  ds_data.fms.match_type = driver_station::MatchType::kQualification;
  ds_data.fms.match_number = 412;

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
  EXPECT_DOUBLE_EQ(channels.at("hw/state/driver_station/station"), 3.0);
  EXPECT_DOUBLE_EQ(channels.at("hw/state/driver_station/match_type"), 2.0);
  EXPECT_DOUBLE_EQ(channels.at("hw/state/driver_station/match_number"), 412.0);
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

// A tick past one page's worth of channels publishes as several pages of one
// sequence, each within the page cap. (The Studio bridge's telemetry_test
// reassembles such pages channel by channel.)
TEST(TelemetryTest, ValuesPastOnePagePublishInPages) {
  Sim sim = Sim::FromToml(R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.budget]
node = "//budget:node"
channel_count = 300
[subsystems.budget.motors.m]
can_id = 1
[subsystems.telemetry]
node = "//talOS/telemetry:node"
exclude = ["budget/state/*/offset_m"]
)");
  auto& source = sim.add<BoundaryChannelsNode>("budget");
  const auto& telemetry = sim.add<Telemetry>("telemetry");
  ASSERT_EQ(telemetry.dictionary().size(), 301u);
  const PageBudget budget = MeasurePages(telemetry.dictionary());
  ASSERT_GE(budget.value_pages, 3u);
  EXPECT_LE(budget.largest_value_page, telemetry_wire::kMaxValueFrameBytes);

  source.send();
  sim.step(20ms);
  const auto last = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(last, nullptr);
  ASSERT_EQ(last->dictionary(), nullptr);
  EXPECT_TRUE(last->keyframe());
  EXPECT_EQ(last->channel_count(), 301u);
  EXPECT_EQ(last->page_count(), budget.value_pages);
  EXPECT_EQ(last->page(), last->page_count() - 1);
  EXPECT_LE(last.bytes.size(), telemetry_wire::kMaxValueFrameBytes);
}

// The channel cap is exact: a dictionary of kMaxChannels constructs, one more
// is refused naming the largest source (a keyframe of it could not fit the
// Studio bridge's burst, so no client could ever be sent one whole).
// REQ-CONTRACT-28
TEST(TelemetryTest, ChannelCapAdmitsExactlyMaxChannels) {
  const auto config = [](std::string_view exclude) {
    return std::string{R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.budget]
node = "//budget:node"
channel_count = )"} +
           std::to_string(telemetry_wire::kMaxChannels / 2) + R"(
[subsystems.budget.motors.m]
can_id = 1
[subsystems.telemetry]
node = "//talOS/telemetry:node"
exclude = [)" +
           std::string{exclude} + "]\n";
  };
  // Two channels per topic plus pose/valid, less one excluded.
  Sim at_cap = Sim::FromToml(config(R"("budget/state/0/offset_m")"));
  at_cap.add<BoundaryChannelsNode>("budget");
  EXPECT_EQ(at_cap.add<Telemetry>("telemetry").dictionary().size(),
            telemetry_wire::kMaxChannels);

  Sim over = Sim::FromToml(config(""));
  over.add<BoundaryChannelsNode>("budget");
  try {
    over.add<Telemetry>("telemetry");
    ADD_FAILURE() << "a dictionary past the channel cap constructed";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string_view{error.what()}.find("2049 channels"),
              std::string_view::npos)
        << error.what();
    EXPECT_NE(std::string_view{error.what()}.find("budget/state"),
              std::string_view::npos)
        << error.what();
  }
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

// A node may declare a message of more scalars than a dictionary holds; the
// config's exclude brings it within the cap, as for any large source. Here
// /budget/state has 2100 doubles (fields "0xxx" to "2099") and excluding
// the names starting with 1
// leaves 989: telemetry constructs and serves exactly those. Counting the
// excluded channels against the topic cap refused the whole node.
// REQ-CONTRACT-28
TEST(TelemetryTest, ExcludedChannelsDoNotCountAgainstATopicsCap) {
  Sim sim = Sim::FromToml(
      LongNamesConfig(2100, 4, R"(exclude = ["budget/state/1*"])"));
  sim.add<LongNamesNode>("budget");
  const auto& telemetry = sim.add<Telemetry>("telemetry");
  std::size_t served = 0;
  for (const ChannelDef& channel : telemetry.dictionary()) {
    if (!channel.name.starts_with(kLongNamesPrefix)) continue;
    ++served;
    EXPECT_NE(channel.name[kLongNamesPrefix.size()], '1') << channel.name;
  }
  EXPECT_EQ(served, 2100u - 1111u);

  // Excluding nothing keeps all 2100: the refusal names that many, not the
  // cap plus one, so the author knows how much to exclude.
  Sim over = Sim::FromToml(LongNamesConfig(2100, 4));
  over.add<LongNamesNode>("budget");
  try {
    over.add<Telemetry>("telemetry");
    ADD_FAILURE() << "a dictionary past the channel cap constructed";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string_view{error.what()}.find("budget/state (2100)"),
              std::string_view::npos)
        << error.what();
  }
}

TEST(TelemetryTest, DictionaryPastOneSlotPagesAndOneChannelMustFitAPage) {
  // 64 channels of the longest name Studio decodes: more dictionary than one
  // slot holds, so it takes several pages.
  Sim sim = Sim::FromToml(LongNamesConfig(64, kLongestFieldName));
  sim.add<LongNamesNode>("budget");
  const auto& telemetry = sim.add<Telemetry>("telemetry");
  EXPECT_GE(MeasurePages(telemetry.dictionary()).dictionary_pages, 2u);

  // One channel whose entry alone outgrows a page cannot be paged; the
  // refusal names it.
  const std::vector<ChannelDef> oversized{
      {"status/huge", "",
       std::vector<std::string>(1024, std::string(256, 'x'))}};
  try {
    MeasurePages(oversized);
    ADD_FAILURE() << "a channel larger than a dictionary page was accepted";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string{error.what()}.find("'status/huge'"),
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

// REQ-CONTRACT-28: enum labels admit exactly kMaxEnumLabels (1024) labels at
// construction (and fit the dictionary page cap on tick 0) and refuse one more
// (1025) cleanly at construction.
TEST(TelemetryTest, EnumLabelCapAdmitsMaxLabelsAndRefusesOneMore) {
  const auto make_labels = [](std::size_t count) {
    std::string array = "[";
    for (std::size_t i = 0; i < count; ++i) {
      if (i != 0) array += ", ";
      array += "\"" + std::to_string(i) + "\"";
    }
    array += "]";
    return array;
  };
  const auto config = [&](std::size_t count) {
    return R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.demo]
node = "//demo:node"
[subsystems.demo.motors.m]
can_id = 1
[subsystems.autonomous]
node = "//autonomous:node"
routines = )" +
           make_labels(count) + R"(
[subsystems.telemetry]
node = "//talOS/telemetry:node"
[[subsystems.telemetry.labels]]
channel = "demo/state/sample_count"
from = "autonomous.routines"
)";
  };

  Sim at_cap = Sim::FromToml(config(telemetry_wire::kMaxEnumLabels));
  auto& shapes = at_cap.add<ShapesNode>("demo");
  const auto& telemetry = at_cap.add<Telemetry>("telemetry");
  const auto dict = DictionaryMap(telemetry);
  ASSERT_TRUE(dict.contains("demo/state/sample_count"));
  const auto& labels = dict.at("demo/state/sample_count")->labels;
  ASSERT_EQ(labels.size(), telemetry_wire::kMaxEnumLabels);
  EXPECT_EQ(labels.front(), "0");
  EXPECT_EQ(labels.back(), std::to_string(telemetry_wire::kMaxEnumLabels - 1));
  const PageBudget budget = MeasurePages(telemetry.dictionary());
  EXPECT_GE(budget.dictionary_pages, 1u);
  EXPECT_LE(budget.dictionary_pages + budget.value_pages,
            telemetry_wire::slot_count);
  shapes.mixed_.send(schema_test::Mixed{true,
                                        1.0,
                                        schema_test::Level::TOP,
                                        1023,
                                        {schema_test::Mode::LOW, 0.0f},
                                        0.0f});
  at_cap.step(20ms);
  const auto frame = LatestFrame(at_cap, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  EXPECT_EQ(DecodeRaw(telemetry, frame.get()).at("demo/state/sample_count"),
            1023u);

  Sim over = Sim::FromToml(config(telemetry_wire::kMaxEnumLabels + 1));
  over.add<ShapesNode>("demo");
  try {
    over.add<Telemetry>("telemetry");
    ADD_FAILURE() << "a channel past kMaxEnumLabels constructed";
  } catch (const std::invalid_argument& error) {
    const std::string_view msg{error.what()};
    EXPECT_NE(msg.find("labels[0].from"), std::string_view::npos) << msg;
    EXPECT_NE(msg.find("1025 labels"), std::string_view::npos) << msg;
    EXPECT_NE(msg.find("1024-label limit"), std::string_view::npos) << msg;
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

// Integer fields travel as their exact value, not through a double: 2^53 + 1
// has no double representation.
TEST(TelemetryTest, IntegerAbove2To53TravelsExactly) {
  Sim sim = Sim::FromToml(R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.pose_estimator]
node = "//estimator:node"
[subsystems.pose_estimator.motors.m]
can_id = 1
[subsystems.telemetry]
node = "//talOS/telemetry:node"
)");
  auto& estimator = sim.add<EstimatorNode>("pose_estimator");
  const auto& telemetry = sim.add<Telemetry>("telemetry");
  const auto dict = DictionaryMap(telemetry);
  ASSERT_EQ(dict.at("pose_estimator/state/sample_time_ns")->kind,
            Talos::Telemetry::ChannelKind_UInt);
  ASSERT_EQ(dict.at("pose_estimator/state/valid")->kind,
            Talos::Telemetry::ChannelKind_Bool);

  constexpr std::uint64_t kExact = (std::uint64_t{1} << 53) + 1;
  ASSERT_NE(static_cast<std::uint64_t>(static_cast<double>(kExact)), kExact);
  estimator.pose_.send(::geometry::PoseEstimate{
      kExact, ::geometry::Pose2d{1.0, 2.0, 0.5},
      ::geometry::Twist2d{0.0, 0.0, 0.0}, true, true, 0, 0});
  sim.step(20ms);
  const auto raw =
      DecodeRaw(telemetry, LatestFrame(sim, "/talos/telemetry").get());
  EXPECT_EQ(raw.at("pose_estimator/state/sample_time_ns"), kExact);
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
  // Every other channel is absent: a keyframe carries only pose/valid.
  const auto cleared = DecodeChannels(telemetry, frame);
  ASSERT_EQ(cleared.size(), 1u);
  EXPECT_DOUBLE_EQ(cleared.at("pose/valid"), 0.0);
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
  // Pose estimator channels are absent (not carried) when stale
  EXPECT_FALSE(channels.contains("pose_estimator/state/twist/vx_mps"));
}

// Test node whose status carries an alert of each level (schema_test.Alerts).
class AlertsNode {
 public:
  explicit AlertsNode(talos::Node& node)
      : status_{node.publish<schema_test::Alerts>("/shooter/status")} {}

  talos::Publisher<schema_test::Alerts> status_;
};

constexpr std::string_view kAlertsConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"

[subsystems.shooter]
node = "//shooter:node"

[subsystems.shooter.motors.m]
can_id = 1

[subsystems.demo]
node = "//demo:node"

[subsystems.autonomous]
node = "//autonomous:node"
routines = ["preload", "none", "taxi"]

[subsystems.telemetry]
node = "//talOS/telemetry:node"
stale_timeout_us = 50000

[[subsystems.telemetry.labels]]
channel = "shooter/status/routine_index"
from = "autonomous.routines"
)";

// Observer matrix (docs/TESTING.md §7): the node observes every state and
// status topic; an input's "validity" is its channels' presence.

// REQ-OPS-37, REQ-OPS-38: each alert field's channel carries its level in
// the dictionary (other fields none), a labelled integer channel carries
// another subsystem's string array as its labels, and every value is carried
// each tick, so a viewer names both without parsing a name or reading TOML.
TEST(TelemetryMatrix, PublishesFromALiveInput) {
  Sim sim = Sim::FromToml(std::string{kAlertsConfig});
  auto& shooter = sim.add<AlertsNode>("shooter");
  auto& telemetry = sim.add<Telemetry>("telemetry");

  const auto dict = DictionaryMap(telemetry);
  using Talos::Telemetry::AlertLevel;
  for (const auto& [name, level] :
       std::initializer_list<std::pair<std::string_view, AlertLevel>>{
           {"shooter/status/alert_error_link_lost",
            Talos::Telemetry::AlertLevel_Error},
           {"shooter/status/alert_warning_hot",
            Talos::Telemetry::AlertLevel_Warning},
           {"shooter/status/alert_info_homed",
            Talos::Telemetry::AlertLevel_Info},
           {"shooter/status/routine_index", Talos::Telemetry::AlertLevel_None},
           {"shooter/status/supply_v", Talos::Telemetry::AlertLevel_None},
       }) {
    ASSERT_TRUE(dict.contains(std::string{name})) << name;
    EXPECT_EQ(static_cast<int>(dict.at(std::string{name})->alert),
              static_cast<int>(level))
        << name;
  }

  EXPECT_EQ(dict.at("shooter/status/routine_index")->labels,
            (std::vector<std::string>{"preload", "none", "taxi"}));

  shooter.status_.send(schema_test::Alerts{true, false, true, 2, 12.5});
  sim.step(20ms);
  const auto raw =
      DecodeRaw(telemetry, LatestFrame(sim, "/talos/telemetry").get());
  EXPECT_EQ(raw.at("shooter/status/alert_error_link_lost"), 1u);
  EXPECT_EQ(raw.at("shooter/status/alert_warning_hot"), 0u);
  EXPECT_EQ(raw.at("shooter/status/alert_info_homed"), 1u);
  EXPECT_EQ(raw.at("shooter/status/routine_index"), 2u);
}

// Test node publishing the alert-shaped schema on its `state` topic.
class StateAlertsNode {
 public:
  explicit StateAlertsNode(talos::Node& node)
      : state_{node.publish<schema_test::Alerts>("/arm/state")} {}

  talos::Publisher<schema_test::Alerts> state_;
};

// REQ-OPS-37, REQ-CONTRACT-33: an alert is a `status` topic's field, as
// session_report reads it; an alert-shaped bool on another role is a plain
// bool, so the dashboard's alert feed (and its budget count) and the
// post-match report agree on which channels are alerts.
TEST(TelemetryMatrix, TagsAlertsOnlyOnStatusTopics) {
  std::string config{kAlertsConfig};
  config += R"(
[subsystems.arm]
node = "//arm:node"
)";
  Sim sim = Sim::FromToml(config);
  sim.add<AlertsNode>("shooter");
  sim.add<StateAlertsNode>("arm");
  auto& telemetry = sim.add<Telemetry>("telemetry");

  const auto dict = DictionaryMap(telemetry);
  ASSERT_TRUE(dict.contains("arm/state/alert_warning_hot"));
  EXPECT_EQ(static_cast<int>(dict.at("arm/state/alert_warning_hot")->alert),
            static_cast<int>(Talos::Telemetry::AlertLevel_None));
  EXPECT_EQ(
      static_cast<int>(dict.at("shooter/status/alert_warning_hot")->alert),
      static_cast<int>(Talos::Telemetry::AlertLevel_Warning));
  // What the `alerts` widget adds to the feed mask and its budget.
  std::size_t alerts = 0;
  for (const auto& [name, info] : dict) {
    if (info->alert != Talos::Telemetry::AlertLevel_None) ++alerts;
  }
  EXPECT_EQ(alerts, 3u);
}

// The channels a dashboard layout names, each from one of its key shapes.
struct LayoutChannels {
  std::string routine = "shooter/status/routine_index";
  std::string phase = "shooter/status/supply_v";  // optional key
  std::string pose = "pose_estimator/state/pose";
  std::string series = "pose_estimator/state/localized";  // channels[1]
  std::string item = "shooter/status/alert_error_link_lost";
};

std::string DashboardLayoutText(const LayoutChannels& c) {
  return R"([[widgets]]
kind = "match"
at = [0, 0, 4, 2]
[[widgets]]
kind = "routine"
at = [4, 0, 3, 2]
channel = ")" +
         c.routine + R"("
phase = ")" +
         c.phase +
         R"("
[[widgets]]
kind = "field"
at = [7, 0, 5, 4]
pose = ")" +
         c.pose +
         R"("
valid = "pose_estimator/state/valid"
[[widgets]]
kind = "graph"
at = [0, 2, 4, 2]
channels = ["shooter/status/supply_v", ")" +
         c.series + R"("]
[[widgets]]
kind = "checks"
at = [0, 4, 4, 2]
items = [{ label = "Link", channel = ")" +
         c.item + R"(", equals = false }]
[[widgets]]
kind = "alerts"
at = [4, 2, 3, 2]
)";
}

// REQ-OPS-38: a [[subsystem.labels]] entry that cannot label an integer
// channel with a string array refuses construction, naming its key, rather
// than leaving a routine shown as a bare number.
// REQ-OPS-39, REQ-OPS-35: so does a dashboard layout ([studio] dashboard)
// the drive feed could not fill, naming the file and the key: a channel
// absent from this dictionary, whichever key named it (optional keys
// too), and a feed over its budget once the dictionary's alert channels,
// which an `alerts` widget adds, are counted.
TEST(TelemetryMatrix, BadParameterFailsConstruction) {
  std::string exact_max = "[";
  for (std::size_t i = 0; i < telemetry_wire::kMaxEnumLabels; ++i) {
    exact_max += (i == 0 ? "\"" : ", \"") + std::to_string(i) + "\"";
  }
  exact_max += "]";
  std::string many = "[";
  for (std::size_t i = 0; i <= telemetry_wire::kMaxEnumLabels; ++i) {
    many += (i == 0 ? "\"r" : ", \"r") + std::to_string(i) + "\"";
  }
  many += "]";
  const std::string max_label_str(telemetry_wire::kMaxDictionaryStringBytes,
                                  'a');
  const std::string overlong_label_str(
      telemetry_wire::kMaxDictionaryStringBytes + 1, 'a');

  // REQ-OPS-38: exactly kMaxEnumLabels (1024) labels and a 256-byte label are
  // accepted at Telemetry construction.
  {
    Sim sim = Sim::FromToml(R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.shooter]
node = "//shooter:node"
[subsystems.shooter.motors.m]
can_id = 1
[subsystems.autonomous]
node = "//autonomous:node"
exact_max = )" + exact_max +
                            R"(
[subsystems.telemetry]
node = "//talOS/telemetry:node"
[[subsystems.telemetry.labels]]
channel = "shooter/status/routine_index"
from = "autonomous.exact_max"
)");
    sim.add<AlertsNode>("shooter");
    const auto& telemetry = sim.add<Telemetry>("telemetry");
    const auto dict = DictionaryMap(telemetry);
    ASSERT_TRUE(dict.contains("shooter/status/routine_index"));
    const auto& labels = dict.at("shooter/status/routine_index")->labels;
    ASSERT_EQ(labels.size(), telemetry_wire::kMaxEnumLabels);
    EXPECT_EQ(labels.front(), "0");
    EXPECT_EQ(labels.back(),
              std::to_string(telemetry_wire::kMaxEnumLabels - 1));
  }
  {
    Sim sim = Sim::FromToml(R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.shooter]
node = "//shooter:node"
[subsystems.shooter.motors.m]
can_id = 1
[subsystems.autonomous]
node = "//autonomous:node"
exact_len = [")" + max_label_str +
                            R"("]
[subsystems.telemetry]
node = "//talOS/telemetry:node"
[[subsystems.telemetry.labels]]
channel = "shooter/status/routine_index"
from = "autonomous.exact_len"
)");
    sim.add<AlertsNode>("shooter");
    const auto& telemetry = sim.add<Telemetry>("telemetry");
    const auto dict = DictionaryMap(telemetry);
    ASSERT_TRUE(dict.contains("shooter/status/routine_index"));
    EXPECT_EQ(dict.at("shooter/status/routine_index")->labels,
              (std::vector<std::string>{max_label_str}));
  }

  struct Case {
    const char* what;
    std::string labels;  // the [[subsystems.telemetry.labels]] body
    const char* key;     // the key the error names
  };
  const auto entry = [](std::string_view channel, std::string_view from) {
    return "channel = \"" + std::string{channel} + "\"\nfrom = \"" +
           std::string{from} + "\"\n";
  };
  const std::string kIndex = "shooter/status/routine_index";
  for (const auto& [what, labels, key] : std::vector<Case>{
           {"unknown channel",
            entry("shooter/status/nope", "autonomous.routines"),
            "labels[0].channel"},
           {"double channel",
            entry("shooter/status/supply_v", "autonomous.routines"),
            "labels[0].channel"},
           {"bool channel",
            entry("shooter/status/alert_warning_hot", "autonomous.routines"),
            "labels[0].channel"},
           {"missing subsystem", entry(kIndex, "ghost.routines"),
            "labels[0].from"},
           {"missing key", entry(kIndex, "autonomous.missing"),
            "labels[0].from"},
           {"empty array", entry(kIndex, "autonomous.empty"), "labels[0].from"},
           {"mixed string and int array", entry(kIndex, "autonomous.mixed"),
            "labels[0].from"},
           {"257-byte label", entry(kIndex, "autonomous.overlong"),
            "labels[0].from"},
           {"number array", entry(kIndex, "autonomous.speeds"),
            "labels[0].from"},
           {"not an array", entry(kIndex, "autonomous.name_text"),
            "labels[0].from"},
           {"no key", entry(kIndex, "autonomous"), "labels[0].from"},
           {"too many labels", entry(kIndex, "autonomous.many"),
            "labels[0].from"},
           {"missing from", "channel = \"" + kIndex + "\"\n", "labels[0].from"},
           {"missing channel", "from = \"autonomous.routines\"\n",
            "labels[0].channel"},
           {"empty entry", "", "labels[0].channel"},
           {"unknown key",
            entry(kIndex, "autonomous.routines") + "typo = \"x\"\n",
            "labels[0].typo"},
           {"non-string channel",
            "channel = 7\nfrom = \"autonomous.routines\"\n",
            "labels[0].channel"},
           {"duplicate labelled channel",
            entry(kIndex, "autonomous.routines") +
                "[[subsystems.telemetry.labels]]\n" +
                entry(kIndex, "autonomous.routines"),
            "labels[1].channel"},
       }) {
    SCOPED_TRACE(what);
    const std::string config = R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.shooter]
node = "//shooter:node"
[subsystems.shooter.motors.m]
can_id = 1
[subsystems.autonomous]
node = "//autonomous:node"
routines = ["preload", "none", "taxi"]
empty = []
mixed = ["a", 1]
overlong = [")" + overlong_label_str +
                               R"("]
speeds = [1.0, 2.0]
name_text = "preload"
many = )" + many + R"(
[subsystems.telemetry]
node = "//talOS/telemetry:node"
[[subsystems.telemetry.labels]]
)" + labels;
    Sim sim = Sim::FromToml(config);
    sim.add<AlertsNode>("shooter");
    try {
      sim.add<Telemetry>("telemetry");
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string_view{error.what()}.find(key),
                std::string_view::npos)
          << error.what();
    }
  }

  const char* tmp = std::getenv("TEST_TMPDIR");
  ASSERT_NE(tmp, nullptr);
  const std::string file = std::string{tmp} + "/dashboard.toml";
  // Writes `layout` to `file` and constructs every node of a config that
  // names it, with `telemetry` added to [subsystems.telemetry].
  const auto construct = [&file](const std::string& layout,
                                 const std::string& telemetry) {
    std::ofstream(file) << layout;
    Sim sim = Sim::FromToml(R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
dashboard = ")" + file + R"("
[subsystems.shooter]
node = "//shooter:node"
[subsystems.shooter.motors.m]
can_id = 1
[subsystems.hw]
node = "//hw:node"
[subsystems.pose_estimator]
node = "//pose_estimator:node"
[subsystems.telemetry]
node = "//talOS/telemetry:node"
)" + telemetry);
    sim.add<AlertsNode>("shooter");
    sim.add<HardwareNode>("hw");
    sim.add<EstimatorNode>("pose_estimator");
    sim.add<Telemetry>("telemetry");
  };
  // Every key shape naming a channel the dictionary holds constructs.
  EXPECT_NO_THROW(construct(DashboardLayoutText({}), ""));

  struct LayoutCase {
    const char* what;
    std::string layout;
    std::string telemetry;  // keys of [subsystems.telemetry]
    const char* key;        // the key the error names
  };
  const auto with = [](auto change) {
    LayoutChannels channels;
    change(channels);
    return DashboardLayoutText(channels);
  };
  // One number channel at 10 Hz fits 12,532 B/s exactly (dashboard_layout
  // test); the three alert channels an `alerts` widget adds do not.
  const std::string over_budget = R"([dashboard]
update_hz = 10
budget_bytes_per_s = 12532
[[widgets]]
kind = "number"
at = [0, 0, 1, 1]
channel = "shooter/status/supply_v"
[[widgets]]
kind = "alerts"
at = [1, 0, 1, 1]
)";
  for (const auto& [what, layout, telemetry, key] : std::vector<LayoutCase>{
           {"absent channel",
            with([](auto& c) { c.routine = "shooter/status/nope"; }), "",
            "widgets[1].channel"},
           {"absent optional channel",
            with([](auto& c) { c.phase = "autonomous/state/phase"; }), "",
            "widgets[1].phase"},
           {"absent pose fields",
            with([](auto& c) { c.pose = "pose_estimator/state/nope"; }), "",
            "widgets[2].pose"},
           {"absent graph series",
            with([](auto& c) { c.series = "shooter/status/nope"; }), "",
            "widgets[3].channels[1]"},
           {"absent check channel",
            with([](auto& c) { c.item = "shooter/status/nope"; }), "",
            "widgets[4].items[0].channel"},
           {"excluded fixed channel", DashboardLayoutText({}),
            "exclude = [\"hw/state/driver_station/alliance\"]\n",
            "widgets[0].kind"},
           {"over budget with alert channels", over_budget, "",
            "[dashboard].budget_bytes_per_s"},
       }) {
    SCOPED_TRACE(what);
    try {
      construct(layout, telemetry);
      ADD_FAILURE() << "constructed";
    } catch (const DashboardLayoutError& error) {
      const std::string_view message = error.what();
      EXPECT_NE(message.find(file), std::string_view::npos) << message;
      EXPECT_NE(message.find(key), std::string_view::npos) << message;
    }
  }
}

// Test node: the hardware bridge's /hw/status with an alert of each level
// (schema_test.Alerts), so the dictionary holds
// hw/status/alert_error_link_lost as a graph with the bridge does.
class HardwareStatusNode {
 public:
  explicit HardwareStatusNode(talos::Node& node)
      : status_{node.publish<schema_test::Alerts>("/hw/status")} {}

  talos::Publisher<schema_test::Alerts> status_;
};

// REQ-OPS-35, REQ-OPS-36: the drive feed carries the RoboRIO link's alert
// channel whatever the layout's widgets, so construction counts it in the
// worst case once the dictionary holds it, and once only beside an `alerts`
// widget; a graph without /hw/status adds nothing.
TEST(TelemetryMatrix, DriveFeedBudgetCountsTheRoboRioLinkChannel) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  ASSERT_NE(tmp, nullptr);
  const std::string file = std::string{tmp} + "/link_dashboard.toml";
  const auto construct = [&file](std::uint32_t budget, bool alerts_widget,
                                 bool hardware_status) {
    std::ofstream(file) << "[dashboard]\nupdate_hz = 10\n"
                        << "budget_bytes_per_s = " << budget << "\n"
                        << "[[widgets]]\nkind = \"number\"\n"
                        << "at = [0, 0, 1, 1]\n"
                        << "channel = \"shooter/status/supply_v\"\n"
                        << (alerts_widget ? "[[widgets]]\nkind = \"alerts\"\n"
                                            "at = [1, 0, 1, 1]\n"
                                          : "");
    Sim sim = Sim::FromToml(R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
dashboard = ")" + file + R"("
[subsystems.shooter]
node = "//shooter:node"
[subsystems.hw]
node = "//hw:node"
[subsystems.telemetry]
node = "//talOS/telemetry:node"
)");
    sim.add<AlertsNode>("shooter");
    if (hardware_status) sim.add<HardwareStatusNode>("hw");
    sim.add<Telemetry>("telemetry");
  };
  // update_hz 10: 10 x (12 x channels + 210) + 10,312 B/s, so one channel
  // needs 12,532, two 12,652 and seven (the number, the shooter's three
  // alerts and /hw/status's three) 13,252.
  struct Case {
    const char* what;
    std::uint32_t budget;
    bool alerts_widget;
    bool hardware_status;
    bool constructs;
  };
  for (const Case& c : {
           Case{"no /hw/status: the named channel only", 12532, false, false,
                true},
           Case{"the link channel counts", 12532, false, true, false},
           Case{"the link channel fits", 12652, false, true, true},
           Case{"beside an alerts widget, counted once", 13252, true, true,
                true},
       }) {
    SCOPED_TRACE(c.what);
    if (c.constructs) {
      EXPECT_NO_THROW(construct(c.budget, c.alerts_widget, c.hardware_status));
      continue;
    }
    try {
      construct(c.budget, c.alerts_widget, c.hardware_status);
      ADD_FAILURE() << "constructed";
    } catch (const DashboardLayoutError& error) {
      EXPECT_NE(
          std::string_view{error.what()}.find("[dashboard].budget_bytes_per_s"),
          std::string_view::npos)
          << error.what();
    }
  }
}

// REQ-OPS-37: an alert whose topic went stale is absent, neither a frozen
// "active" nor a "clear", while the node keeps publishing its other inputs.
TEST(TelemetryMatrix, StaleInputClearsValidity) {
  Sim sim = Sim::FromToml(std::string{kAlertsConfig});
  auto& shooter = sim.add<AlertsNode>("shooter");
  auto& shapes = sim.add<ShapesNode>("demo");
  auto& telemetry = sim.add<Telemetry>("telemetry");

  shooter.status_.send(schema_test::Alerts{true, true, false, 0, 12.0});
  shapes.arrays_.send(schema_test::Arrays{});
  sim.step(20ms);
  auto raw = DecodeRaw(telemetry, LatestFrame(sim, "/talos/telemetry").get());
  ASSERT_EQ(raw.at("shooter/status/alert_error_link_lost"), 1u);
  ASSERT_EQ(raw.at("shooter/status/alert_info_homed"), 0u);

  // Past stale_timeout_us (50 ms) with the demo topic still fresh.
  for (int i = 0; i < 8; ++i) {
    shapes.arrays_.send(schema_test::Arrays{});
    sim.step(10ms);
  }
  const auto frame = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(frame, nullptr);
  ASSERT_TRUE(frame->keyframe());
  raw = DecodeRaw(telemetry, frame.get());
  EXPECT_TRUE(raw.contains("demo/status/samples_m/0"));
  for (const auto* name : {"shooter/status/alert_error_link_lost",
                           "shooter/status/alert_warning_hot",
                           "shooter/status/alert_info_homed"}) {
    EXPECT_FALSE(raw.contains(name)) << name;
  }
}

// Sources whose every topic changes on each of their ticks: nested structs,
// arrays and enums, a pose, a Driver Station packet, and enough topics that
// one telemetry tick takes several value pages.
class LiveSources {
 public:
  static constexpr int kPagingTopics = 300;

  explicit LiveSources(talos::Node& node)
      : node_{node},
        mixed_{node.publish<schema_test::Mixed>("/demo/state")},
        arrays_{node.publish<schema_test::Arrays>("/demo/status")},
        alerts_{node.publish<schema_test::Alerts>("/demo/status/alerts")},
        pose_{node.publish<::geometry::PoseEstimate>("/pose_estimator/state")},
        driver_station_{
            node.publish<hardware::Packet>("/hw/state/driver_station")} {
    for (int i = 0; i < kPagingTopics; ++i) {
      paging_.push_back(
          node.publish<schema_test::Inner>("/demo/state/" + std::to_string(i)));
    }
  }

  void update() {
    const auto tick = static_cast<double>(node_.tick());
    mixed_.send(schema_test::Mixed{
        node_.tick() % 2 == 0, tick, schema_test::Level::TOP,
        static_cast<std::uint32_t>(node_.tick()),
        schema_test::Inner{schema_test::Mode::HIGH, static_cast<float>(tick)},
        0.5f});
    arrays_.send(schema_test::Arrays{});
    const bool odd = node_.tick() % 2 != 0;
    alerts_.send(schema_test::Alerts{
        odd, !odd, odd, static_cast<std::uint8_t>(node_.tick()), tick});
    pose_.send(::geometry::PoseEstimate{0, ::geometry::Pose2d{tick, 1.0, 0.5},
                                        ::geometry::Twist2d{0.0, 0.0, 0.0},
                                        true, true, 0, 0});
    driver_station::DriverStationData data{};
    data.fms.flags = driver_station::kDsAttached | driver_station::kEnabled |
                     driver_station::kTeleop;
    data.fms.match_time_s = static_cast<float>(tick);
    hardware::Packet packet{};
    packet.size =
        static_cast<std::uint32_t>(driver_station::Encode(data, packet.data));
    driver_station_.send(packet);
    for (auto& publisher : paging_) {
      publisher.send(
          schema_test::Inner{schema_test::Mode::LOW, static_cast<float>(tick)});
    }
  }

 private:
  talos::Node& node_;
  talos::Publisher<schema_test::Mixed> mixed_;
  talos::Publisher<schema_test::Arrays> arrays_;
  talos::Publisher<schema_test::Alerts> alerts_;
  talos::Publisher<::geometry::PoseEstimate> pose_;
  talos::Publisher<hardware::Packet> driver_station_;
  std::vector<talos::Publisher<schema_test::Inner>> paging_;
};

// RULES.md 3: with every source changing each tick (alerts toggling among
// them), the producer's reads, value pages and its dictionary resend (every
// 100th tick, alert levels and configured labels included) allocate nothing.
TEST(TelemetryMatrix, UpdateDoesNotAllocate) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  Sim sim = Sim::FromToml(R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
pose_topic = "/pose_estimator/state"
[subsystems.demo]
node = "//demo:node"
[subsystems.demo.motors.m]
can_id = 1
[subsystems.autonomous]
node = "//autonomous:node"
routines = ["preload", "none", "taxi"]
[subsystems.telemetry]
node = "//talOS/telemetry:node"
[[subsystems.telemetry.labels]]
channel = "demo/status/alerts/routine_index"
from = "autonomous.routines"
)");
  sim.add<LiveSources>("demo");
  const auto& telemetry = sim.add<Telemetry>("telemetry");
  ASSERT_GE(MeasurePages(telemetry.dictionary()).value_pages, 2u);
  ASSERT_EQ(DictionaryMap(telemetry)
                .at("demo/status/alerts/routine_index")
                ->labels.size(),
            3u);
  // Warm up through the dictionary resend at sequence 100: the first send,
  // at sequence 0, omits the zero sequence field, so its pages are smaller.
  sim.step(1030ms);
  const auto before = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(before, nullptr);
  const std::uint64_t first_sequence = before->sequence_id();

  talos::testing::AllocationProbe probe;
  probe.start();
  // Past the next dictionary resend, at sequence 200.
  sim.step(1100ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();

  const auto after = LatestFrame(sim, "/talos/telemetry");
  ASSERT_NE(after, nullptr);
  EXPECT_GE(after->sequence_id(), first_sequence + 105);
  EXPECT_GE(after->page_count(), 2u);
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

// Builds a minimal verified reflection schema containing `geometry.Pose2d`
// (at object index 0) and `pose_test.Estimate` (at object index 1, sorted by
// name for FlatBuffers LookupByKey) with configurable pose field type and
// optional `valid` offset.
std::string CraftPoseSchema(reflection::BaseType pose_field_type,
                            std::uint16_t pose_field_bytes, bool include_valid,
                            std::uint16_t valid_offset) {
  flatbuffers::FlatBufferBuilder fbb;

  auto make_field = [&](const char* name, reflection::BaseType base,
                        std::int32_t obj_index, std::uint16_t id,
                        std::uint16_t offset) {
    const auto type =
        reflection::CreateType(fbb, base, reflection::BaseType::None, obj_index,
                               /*fixed_length=*/0, /*base_size=*/1,
                               /*element_size=*/1);
    return reflection::CreateFieldDirect(fbb, name, type, id, offset);
  };

  std::vector<flatbuffers::Offset<reflection::Field>> pose_fields;
  pose_fields.push_back(make_field("x_m", pose_field_type, -1, 0, 0));
  pose_fields.push_back(
      make_field("y_m", pose_field_type, -1, 1, pose_field_bytes));
  pose_fields.push_back(
      make_field("yaw_rad", pose_field_type, -1, 2,
                 static_cast<std::uint16_t>(pose_field_bytes * 2)));
  const auto pose_obj = reflection::CreateObjectDirect(
      fbb, "geometry.Pose2d", &pose_fields, /*is_struct=*/true,
      /*minalign=*/pose_field_bytes,
      /*bytesize=*/pose_field_bytes * 3);

  std::vector<flatbuffers::Offset<reflection::Field>> est_fields;
  est_fields.push_back(
      make_field("pose", reflection::BaseType::Obj, /*obj_index=*/0, 0, 0));
  if (include_valid) {
    est_fields.push_back(
        make_field("valid", reflection::BaseType::Bool, -1, 1, valid_offset));
  }
  const std::int32_t est_bytes =
      include_valid
          ? std::max<std::int32_t>(pose_field_bytes * 3, valid_offset + 1)
          : pose_field_bytes * 3;
  const auto est_obj = reflection::CreateObjectDirect(
      fbb, "pose_test.Estimate", &est_fields, /*is_struct=*/true,
      /*minalign=*/pose_field_bytes, /*bytesize=*/est_bytes);

  std::vector<flatbuffers::Offset<reflection::Object>> objects{pose_obj,
                                                               est_obj};
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  reflection::FinishSchemaBuffer(
      fbb, reflection::CreateSchemaDirect(fbb, &objects, &enums));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

class FloatPoseTopicNode {
 public:
  explicit FloatPoseTopicNode(talos::Node& node) {
    using talos::detail::NodeAccess;
    const std::string bfbs =
        CraftPoseSchema(reflection::BaseType::Float, sizeof(float), false, 0);
    NodeAccess::add_publisher(
        node, "/pose_estimator/state", 24, 8,
        talos::schema::Identity{"pose_test.Estimate", 111, bfbs});
  }
};

class TruncatedPoseTopicNode {
 public:
  explicit TruncatedPoseTopicNode(talos::Node& node) {
    using talos::detail::NodeAccess;
    NodeAccess::add_publisher(node, "/pose_estimator/state", 16, 8,
                              talos::schema::Of<::geometry::PoseEstimate>());
  }
};

class OutOfBoundsValidPoseTopicNode {
 public:
  explicit OutOfBoundsValidPoseTopicNode(talos::Node& node) {
    using talos::detail::NodeAccess;
    const std::string bfbs =
        CraftPoseSchema(reflection::BaseType::Double, sizeof(double), true, 24);
    NodeAccess::add_publisher(
        node, "/pose_estimator/state", 24, 8,
        talos::schema::Identity{"pose_test.Estimate", 222, bfbs});
  }
};

// REQ-OPS-22: SetupPoseExtraction must verify field base types (Double for
// x_m/y_m/yaw_rad, Bool for valid) and message_bytes bounds before recording
// offsets read by memcpy in update().
TEST(TelemetryTest, PoseExtractionRefusesWrongTypesAndOutOfBoundsOffsets) {
  const std::string kConfig = R"(
[robot]
period_us = 10000

[studio]
topic = "/talos/telemetry"
pose_topic = "/pose_estimator/state"

[subsystems.pose_estimator]
node = "//pose_estimator:node"

[subsystems.pose_estimator.motors.m]
can_id = 1

[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";

  // 1. Float fields inside geometry.Pose2d instead of Double.
  {
    Sim sim = Sim::FromToml(kConfig);
    sim.add<FloatPoseTopicNode>("pose_estimator");
    EXPECT_THROW(sim.add<Telemetry>("telemetry"), std::invalid_argument);
  }

  // 2. Valid Double fields, but declared message_bytes truncates yaw_rad.
  {
    Sim sim = Sim::FromToml(kConfig);
    sim.add<TruncatedPoseTopicNode>("pose_estimator");
    EXPECT_THROW(sim.add<Telemetry>("telemetry"), std::invalid_argument);
  }

  // 3. Valid Double geometry.Pose2d fits in 24 bytes, but Bool `valid` sits at
  // offset 24 (past message_bytes = 24).
  {
    Sim sim = Sim::FromToml(kConfig);
    sim.add<OutOfBoundsValidPoseTopicNode>("pose_estimator");
    EXPECT_THROW(sim.add<Telemetry>("telemetry"), std::invalid_argument);
  }
}

// ByteTopic sink that appends every frame published on `/talos/telemetry` in
// order, unlike testing::Sim's LatestBytes which retains only the last page of
// each tick.
class CollectingByteTopic final : public talos::detail::ByteTopic {
 public:
  explicit CollectingByteTopic(std::vector<FrameHolder>& frames)
      : frames_{frames} {}

  bool write(std::span<const std::byte> message) override {
    frames_.push_back(
        FrameHolder{std::vector<std::byte>(message.begin(), message.end())});
    return true;
  }

 private:
  std::vector<FrameHolder>& frames_;
};

class CollectingPort final
    : public talos::detail::LoopPortFor<event::SimulatedEventLoop<>> {
 public:
  CollectingPort(event::SimulatedEventLoop<>& loop,
                 std::vector<FrameHolder>& frames)
      : LoopPortFor{loop}, frames_{frames} {}

  std::unique_ptr<talos::detail::ByteTopic> open_byte_topic(
      std::string_view, std::size_t, const BytePublishOptions&) override {
    return std::make_unique<CollectingByteTopic>(frames_);
  }

 private:
  std::vector<FrameHolder>& frames_;
};

// Builds a flat reflection schema for a struct of `count` doubles, each field
// named with `name_bytes` characters.
std::string MakeDoubleStructSchema(std::string_view object_name,
                                   std::size_t count, std::size_t name_bytes) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<reflection::Field>> fields;
  fields.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    std::string name = std::to_string(i);
    if (name.size() < name_bytes) name.resize(name_bytes, 'x');
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
  const auto obj_name = builder.CreateString(object_name);
  const auto field_vector = builder.CreateVector(fields);
  reflection::ObjectBuilder object_builder{builder};
  object_builder.add_name(obj_name);
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
  return {reinterpret_cast<const char*>(builder.GetBufferPointer()),
          builder.GetSize()};
}

// Publishes `/budget/state` with enough long-named double channels that both
// dictionary pages and value pages span multiple frames each tick.
class MultiPageSourceNode {
 public:
  static constexpr std::size_t kFields = 250;

  explicit MultiPageSourceNode(talos::Node& node)
      : node_{node},
        schema_{MakeDoubleStructSchema("budget.MultiPage", kFields,
                                       kLongestFieldName)},
        pub_index_{talos::detail::NodeAccess::add_publisher(
            node, "/budget/state", kFields * sizeof(double), 8,
            talos::schema::Identity{"budget.MultiPage", 54321, schema_})},
        payload_(kFields, 1.5) {}

  void update() {
    payload_[0] = static_cast<double>(node_.tick());
    talos::detail::NodeAccess::publisher_send(
        node_, pub_index_, payload_.data(), payload_.size() * sizeof(double));
  }

 private:
  talos::Node& node_;
  std::string schema_;
  std::size_t pub_index_;
  std::vector<double> payload_;
};

// REQ-OPS-22: on tick 0 and every 100th tick thereafter (tick 100), Telemetry
// emits every dictionary page before its value pages; ticks 1..99 emit value
// pages only.
TEST(TelemetryTest, DictionaryPagesEmitFirstOnTick0AndEvery100thTick) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  ASSERT_NE(tmp, nullptr);
  const std::string config_path = std::string{tmp} + "/telemetry_cadence_" +
                                  std::to_string(::getpid()) + ".toml";
  {
    std::ofstream out{config_path};
    out << R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.budget]
node = "//budget:node"
[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  }
  const auto config = std::make_shared<const config::RobotConfig>(
      config::ParseRobotConfig(config_path));

  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> src_loop{env};
  talos::detail::LoopPortFor<event::SimulatedEventLoop<>> src_port{src_loop};
  auto src_node = talos::detail::NodeAccess::create(talos::detail::NodeSetup{
      .name = "budget",
      .config_path = config_path,
      .config = config,
      .loop = &src_port,
  });
  MultiPageSourceNode src{*src_node};
  talos::detail::NodeAccess::set_update(*src_node, [&] { src.update(); });
  talos::detail::NodeAccess::start(*src_node);

  talos::detail::DeclaredGraph graph;
  graph.push_back(talos::detail::DescribeNode(
      *src_node, "//budget:node", src_loop.manifest(),
      introspect::LoopSourceFlags(src_loop)));

  std::vector<FrameHolder> collected;
  event::SimulatedEventLoop<> tel_loop{env};
  CollectingPort tel_port{tel_loop, collected};
  auto tel_node = talos::detail::NodeAccess::create(talos::detail::NodeSetup{
      .name = "telemetry",
      .config_path = config_path,
      .config = config,
      .loop = &tel_port,
      .graph = [&graph] { return &graph; },
  });
  Telemetry telemetry{*tel_node};
  talos::detail::NodeAccess::set_update(*tel_node, [&] { telemetry.update(); });
  talos::detail::NodeAccess::start(*tel_node);

  const PageBudget budget = MeasurePages(telemetry.dictionary());
  ASSERT_GE(budget.dictionary_pages, 2u);
  ASSERT_GE(budget.value_pages, 2u);

  constexpr std::uint64_t kTicks = 101;
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    SCOPED_TRACE("tick " + std::to_string(tick));
    const std::size_t before = collected.size();
    const auto at = event::MonotonicTime::from_nanos(
        static_cast<std::int64_t>((tick + 1) * 10'000'000LL));
    src_loop.run_until(at);
    tel_loop.run_until(at);

    const std::size_t emitted = collected.size() - before;
    const bool expect_dict = (tick % 100 == 0);
    const std::size_t expected_dict = expect_dict ? budget.dictionary_pages : 0;
    ASSERT_EQ(emitted, expected_dict + budget.value_pages);

    std::vector<std::string> dict_names;
    for (std::size_t i = 0; i < expected_dict; ++i) {
      const Frame* frame = collected[before + i].get();
      ASSERT_NE(frame, nullptr);
      EXPECT_EQ(frame->sequence_id(), tick);
      EXPECT_EQ(frame->stream_id(), telemetry.stream_id());
      EXPECT_EQ(frame->dictionary_id(), telemetry.dictionary_id());
      EXPECT_EQ(frame->channel_count(), telemetry.dictionary().size());
      ASSERT_NE(frame->dictionary(), nullptr);
      EXPECT_EQ(frame->channels(), nullptr);
      EXPECT_EQ(frame->page(), i);
      EXPECT_EQ(frame->page_count(), budget.dictionary_pages);
      for (const auto* info : *frame->dictionary()) {
        dict_names.push_back(info->name()->str());
      }
    }
    if (expect_dict) {
      ASSERT_EQ(dict_names.size(), telemetry.dictionary().size());
      for (std::size_t c = 0; c < dict_names.size(); ++c) {
        EXPECT_EQ(dict_names[c], telemetry.dictionary()[c].name);
      }
    }
    for (std::size_t i = 0; i < budget.value_pages; ++i) {
      const Frame* frame = collected[before + expected_dict + i].get();
      ASSERT_NE(frame, nullptr);
      EXPECT_EQ(frame->sequence_id(), tick);
      EXPECT_EQ(frame->stream_id(), telemetry.stream_id());
      EXPECT_EQ(frame->dictionary_id(), telemetry.dictionary_id());
      EXPECT_EQ(frame->channel_count(), telemetry.dictionary().size());
      EXPECT_EQ(frame->dictionary(), nullptr);
      EXPECT_TRUE(frame->keyframe());
      EXPECT_EQ(frame->page(), i);
      EXPECT_EQ(frame->page_count(), budget.value_pages);
    }
  }
  std::remove(config_path.c_str());
}

// Publishes four topics with distinct channel counts whose alphabetical order
// disagrees with their descending size order:
//   /alpha/state  (10)   - alphabetically first, third largest
//   /middle/state (1500) - largest
//   /tiny/state   (1)    - smallest (tied with pose/valid; omitted from top 3)
//   /zeta/state   (600)  - alphabetically last, second largest
class DistinctSizedTopicsNode {
 public:
  explicit DistinctSizedTopicsNode(talos::Node& node) {
    const struct {
      const char* topic;
      const char* schema;
      std::size_t fields;
      std::uint64_t hash;
    } specs[] = {
        {"/alpha/state", "sized.Alpha", 10, 1001},
        {"/middle/state", "sized.Middle", 1500, 1002},
        {"/tiny/state", "sized.Tiny", 1, 1003},
        {"/zeta/state", "sized.Zeta", 600, 1004},
    };
    schemas_.reserve(4);
    for (const auto& s : specs) {
      schemas_.push_back(MakeDoubleStructSchema(s.schema, s.fields, 4));
      talos::detail::NodeAccess::add_publisher(
          node, s.topic, s.fields * sizeof(double), 8,
          talos::schema::Identity{s.schema, s.hash, schemas_.back()});
    }
  }

 private:
  std::vector<std::string> schemas_;
};

// REQ-OPS-22: when the dictionary exceeds kMaxChannels, construction fails
// listing the largest contributing topics in descending channel-count order
// and omitting smaller topics beyond the top three.
TEST(TelemetryTest, ChannelCapErrorOrdersLargestSourcesDescending) {
  Sim sim = Sim::FromToml(R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.sized]
node = "//sized:node"
[subsystems.telemetry]
node = "//talOS/telemetry:node"
)");
  sim.add<DistinctSizedTopicsNode>("sized");
  try {
    sim.add<Telemetry>("telemetry");
    ADD_FAILURE() << "a dictionary past the channel cap constructed";
  } catch (const std::runtime_error& error) {
    const std::string_view msg = error.what();
    // 10 + 1500 + 1 + 600 + 1 (pose/valid) = 2112 channels.
    EXPECT_NE(msg.find("2112 channels"), std::string_view::npos) << msg;
    const std::size_t pos_largest = msg.find("middle/state (1500)");
    const std::size_t pos_second = msg.find("zeta/state (600)");
    const std::size_t pos_third = msg.find("alpha/state (10)");
    ASSERT_NE(pos_largest, std::string_view::npos) << msg;
    ASSERT_NE(pos_second, std::string_view::npos) << msg;
    ASSERT_NE(pos_third, std::string_view::npos) << msg;
    EXPECT_LT(pos_largest, pos_second) << msg;
    EXPECT_LT(pos_second, pos_third) << msg;
    // Smallest topics fall outside the top three and are omitted.
    EXPECT_EQ(msg.find("tiny/state"), std::string_view::npos) << msg;
    EXPECT_EQ(msg.find("pose/valid"), std::string_view::npos) << msg;
  }
}

// Publishes `/pose_estimator/state` with a verified reflection.Schema whose
// `objects` vector is empty (0 objects) or omitted.
class EmptyObjectsPoseTopicNode {
 public:
  explicit EmptyObjectsPoseTopicNode(talos::Node& node) {
    flatbuffers::FlatBufferBuilder fbb;
    std::vector<flatbuffers::Offset<reflection::Object>> objects;
    std::vector<flatbuffers::Offset<reflection::Enum>> enums;
    reflection::FinishSchemaBuffer(
        fbb, reflection::CreateSchemaDirect(fbb, &objects, &enums));
    const std::string bfbs{
        reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
    talos::detail::NodeAccess::add_publisher(
        node, "/pose_estimator/state", 24, 8,
        talos::schema::Identity{"pose_test.EmptySchema", 333, bfbs});
  }
};

class OmittedObjectsPoseTopicNode {
 public:
  explicit OmittedObjectsPoseTopicNode(talos::Node& node) {
    flatbuffers::FlatBufferBuilder fbb;
    const auto start = fbb.StartTable();
    const flatbuffers::Offset<reflection::Schema> empty{fbb.EndTable(start)};
    reflection::FinishSchemaBuffer(fbb, empty);
    const std::string bfbs{
        reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
    talos::detail::NodeAccess::add_publisher(
        node, "/pose_estimator/state", 24, 8,
        talos::schema::Identity{"pose_test.OmittedObjects", 334, bfbs});
  }
};

// REQ-OPS-22, REQ-CONTRACT-20: a schema with empty or omitted `objects()` must
// be refused with std::invalid_argument naming the schema and/or pose topic,
// never dereferenced.
TEST(TelemetryTest, PoseExtractionRefusesSchemaWithEmptyOrOmittedObjects) {
  constexpr const char* kConfig = R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
pose_topic = "/pose_estimator/state"
[subsystems.pose_estimator]
node = "//pose_estimator:node"
[subsystems.telemetry]
node = "//talOS/telemetry:node"
)";
  {
    Sim sim = Sim::FromToml(kConfig);
    sim.add<EmptyObjectsPoseTopicNode>("pose_estimator");
    try {
      sim.add<Telemetry>("telemetry");
      ADD_FAILURE() << "constructed despite empty schema->objects()";
    } catch (const std::invalid_argument& error) {
      const std::string_view msg = error.what();
      EXPECT_NE(msg.find("pose_test.EmptySchema"), std::string_view::npos)
          << msg;
      EXPECT_NE(msg.find("/pose_estimator/state"), std::string_view::npos)
          << msg;
    }
  }
  {
    Sim sim = Sim::FromToml(kConfig);
    sim.add<OmittedObjectsPoseTopicNode>("pose_estimator");
    try {
      sim.add<Telemetry>("telemetry");
      ADD_FAILURE() << "constructed despite omitted schema->objects()";
    } catch (const std::invalid_argument& error) {
      const std::string_view msg = error.what();
      EXPECT_NE(msg.find("/pose_estimator/state"), std::string_view::npos)
          << msg;
    }
  }
}

// REQ-CONTRACT-28, REQ-OPS-22: on tick 0 (and every 100th tick), Telemetry
// emits every dictionary page followed by every value page in one tick into
// the 64-slot `/talos/telemetry` ring. A dictionary whose combined page count
// exceeds `telemetry_wire::slot_count` would silently lap its own dictionary
// pages on tick 0, so construction must refuse it naming the page counts and
// `slot_count`.
TEST(TelemetryTest, CombinedPagesExceedingRingSlotCountIsRefused) {
  // 2000 long-named channels + pose/valid = 2001 channels (<= kMaxChannels):
  // fits in <= 64 combined pages (dictionary + value).
  {
    Sim fits = Sim::FromToml(LongNamesConfig(1900, kLongestFieldName));
    fits.add<LongNamesNode>("budget");
    const auto& telemetry = fits.add<Telemetry>("telemetry");
    const PageBudget budget = MeasurePages(telemetry.dictionary());
    EXPECT_LE(budget.dictionary_pages + budget.value_pages,
              telemetry_wire::slot_count);
  }

  // 2047 long-named channels + pose/valid = 2048 channels (== kMaxChannels),
  // but 41 dictionary pages + 25 value pages = 66 > 64 slots.
  Sim over = Sim::FromToml(LongNamesConfig(2047, kLongestFieldName));
  over.add<LongNamesNode>("budget");
  try {
    over.add<Telemetry>("telemetry");
    ADD_FAILURE() << "accepted dictionary whose resend tick exceeds 64 slots";
  } catch (const std::runtime_error& error) {
    const std::string_view msg = error.what();
    EXPECT_NE(msg.find(std::to_string(telemetry_wire::slot_count)),
              std::string_view::npos)
        << msg;
    EXPECT_NE(msg.find("dictionary"), std::string_view::npos) << msg;
    EXPECT_NE(msg.find("value"), std::string_view::npos) << msg;
  }
}

// Publishes a 3-segment topic `/big/state/sub` with 2100 doubles so excess
// channels past the flattening cap must bucket under `big/state` together with
// the built channels rather than splitting into `big/state/sub`.
class ThreeSegmentOversizedNode {
 public:
  explicit ThreeSegmentOversizedNode(talos::Node& node)
      : schema_{MakeDoubleStructSchema("big.Sub", 2100, 4)} {
    talos::detail::NodeAccess::add_publisher(
        node, "/big/state/sub", 2100 * sizeof(double), 8,
        talos::schema::Identity{"big.Sub", 6789, schema_});
  }

 private:
  std::string schema_;
};

// REQ-OPS-22: excess channels on a 3-segment topic bucket under `owner/role`
// together with its built channels in the cap refusal.
TEST(TelemetryTest, ExcessChannelsBucketByOwnerAndRole) {
  Sim sim = Sim::FromToml(R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.big]
node = "//big:node"
[subsystems.telemetry]
node = "//talOS/telemetry:node"
)");
  sim.add<ThreeSegmentOversizedNode>("big");
  try {
    sim.add<Telemetry>("telemetry");
    ADD_FAILURE() << "a dictionary past the channel cap constructed";
  } catch (const std::runtime_error& error) {
    const std::string_view msg = error.what();
    EXPECT_NE(msg.find("2101 channels"), std::string_view::npos) << msg;
    EXPECT_NE(msg.find("big/state (2100)"), std::string_view::npos) << msg;
    EXPECT_EQ(msg.find("big/state/sub"), std::string_view::npos) << msg;
  }
}

// REQ-OPS-38, REQ-CONTRACT-10: an explicit empty `labels = []` in
// `[subsystems.telemetry]` constructs cleanly and records a read of `"labels"`
// so `LintParameterKeys` never flags it as unread.
TEST(TelemetryTest, EmptyLabelsArrayRecordsParamRead) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  ASSERT_NE(tmp, nullptr);
  const std::string config_path = std::string{tmp} +
                                  "/telemetry_empty_labels_" +
                                  std::to_string(::getpid()) + ".toml";
  {
    std::ofstream out{config_path};
    out << R"(
[robot]
period_us = 10000
[studio]
topic = "/talos/telemetry"
[subsystems.telemetry]
node = "//talOS/telemetry:node"
labels = []
)";
  }
  const auto config = std::make_shared<const config::RobotConfig>(
      config::ParseRobotConfig(config_path));
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  talos::detail::LoopPortFor<event::SimulatedEventLoop<>> port{loop};
  auto node = talos::detail::NodeAccess::create(talos::detail::NodeSetup{
      .name = "telemetry",
      .config_path = config_path,
      .config = config,
      .loop = &port,
  });
  Telemetry telemetry{*node};
  const auto desc = talos::detail::DescribeNode(
      *node, "//talOS/telemetry:node", loop.manifest(),
      introspect::LoopSourceFlags(loop));
  ASSERT_TRUE(desc.params.has_value());
  bool saw_labels = false;
  for (const auto& read : *desc.params) {
    if (read.subsystem == "telemetry" && read.key == "labels") {
      saw_labels = true;
    }
  }
  EXPECT_TRUE(saw_labels);
  std::remove(config_path.c_str());
}

}  // namespace
}  // namespace talos::telemetry
