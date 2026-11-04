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
      : ds_{node.publish<hardware::Packet>("/hw/state/driver_station")} {}

  talos::Publisher<hardware::Packet> ds_;
};

struct FrameHolder {
  std::vector<std::byte> bytes;

  const Frame* get() const {
    if (bytes.empty()) return nullptr;
    const auto payload = telemetry_wire::payload(bytes);
    if (payload.size() != bytes.size()) return nullptr;
    flatbuffers::Verifier verifier{
        reinterpret_cast<const std::uint8_t*>(payload.data()), payload.size()};
    if (!Talos::Telemetry::VerifySizePrefixedFrameBuffer(verifier))
      return nullptr;
    return Talos::Telemetry::GetSizePrefixedFrame(bytes.data());
  }

  const Frame* operator->() const { return get(); }
  operator const Frame*() const { return get(); }
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
  ASSERT_TRUE(dict.contains("hw/state/driver_station/match_time_s"));
  EXPECT_EQ(dict.at("hw/state/driver_station/match_time_s")->unit, "s");

  driver_station::DriverStationData ds_data{};
  ds_data.fms.flags = driver_station::kDsAttached | driver_station::kEnabled |
                      driver_station::kTeleop;
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

}  // namespace
}  // namespace talos::telemetry
