#include "telemetry/node.h"

#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {
namespace {

using namespace std::chrono_literals;
using Talos::Telemetry::Frame;
using testing::Sim;

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

odometry::OdometryState Odometry(double x, double y, double yaw_rot) {
  return odometry::OdometryState{0, x, y, yaw_rot, 1.5, -0.5, 0.25};
}

shooter::ShooterState Shooter() {
  return shooter::ShooterState{0, 42.0, 42.0, true, true, true};
}

intake::IntakeState Intake(intake::Inhibit inhibit) {
  return intake::IntakeState{true, 20.0f, false, 0.0f, inhibit};
}

class TelemetryTest : public ::testing::Test {
 protected:
  // The newest frame on the Studio topic, checked the way the bridge reads
  // it; nullptr when none was published.
  const Frame* Latest() {
    latest_ =
        sim.latest_bytes(sim.node("telemetry").param<std::string>("topic"));
    if (!latest_) return nullptr;
    const auto payload = telemetry_wire::payload(*latest_);
    EXPECT_EQ(payload.size(), latest_->size());
    flatbuffers::Verifier verifier{
        reinterpret_cast<const std::uint8_t*>(payload.data()), payload.size()};
    EXPECT_TRUE(Talos::Telemetry::VerifySizePrefixedFrameBuffer(verifier));
    return Talos::Telemetry::GetSizePrefixedFrame(latest_->data());
  }

  std::size_t LatestBytes() {
    auto bytes =
        sim.latest_bytes(sim.node("telemetry").param<std::string>("topic"));
    return bytes ? bytes->size() : 0;
  }

  // Present channels only: NaN is the wire's absence marker, so an absent
  // or stale source simply has no entry here.
  std::map<std::string, double> Channels() {
    const Frame* frame = Latest();
    std::map<std::string, double> channels;
    if (frame == nullptr || frame->values() == nullptr) return channels;
    const auto& dictionary = telemetry.dictionary();
    EXPECT_EQ(frame->values()->size(), dictionary.size());
    for (std::size_t i = 0; i < dictionary.size(); ++i) {
      const double value = frame->values()->Get(i);
      if (std::isnan(value)) continue;
      EXPECT_TRUE(channels.emplace(dictionary[i].name, value).second)
          << "duplicate channel " << dictionary[i].name;
    }
    return channels;
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  Telemetry& telemetry = sim.add<Telemetry>("telemetry");
  talos::testing::SimHardware hw{sim};

 private:
  std::optional<std::vector<std::byte>> latest_;
};

TEST_F(TelemetryTest, PublishesChassisPoseAndChannelsFromOdometry) {
  sim.publish(odometry::topics::kState, Odometry(3.25, 1.5, 0.25));
  sim.publish(drivetrain::topics::kState,
              drivetrain::DrivetrainState{0, true, 1.5, -0.5, 0.25, 0.25, 0.1});
  sim.step(50ms);

  const Frame* frame = Latest();
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->chassis(), nullptr);
  EXPECT_NEAR(frame->chassis()->x(), 3.25, 1e-12);
  EXPECT_NEAR(frame->chassis()->y(), 1.5, 1e-12);
  EXPECT_NEAR(frame->chassis()->yaw(), 0.25 * 2 * std::numbers::pi, 1e-12);
  EXPECT_EQ(frame->sequence_id(), 4u);

  const auto channels = Channels();
  EXPECT_NEAR(channels.at("pose/valid"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("odometry/vx"), 1.5, 1e-12);
  EXPECT_NEAR(channels.at("odometry/omega"), 0.25, 1e-12);
  EXPECT_EQ(channels.count("shooter/flywheel"), 0u);
  EXPECT_NEAR(channels.at("drivetrain/vx"), 1.5, 1e-12);
  EXPECT_NEAR(channels.at("drivetrain/enabled"), 1.0, 1e-12);
}

TEST_F(TelemetryTest, PublishesNothingUntilAPoseExists) {
  sim.publish(drivetrain::topics::kState,
              drivetrain::DrivetrainState{0, true, 1, 0, 0, 0, 0});
  sim.step(50ms);
  EXPECT_EQ(Latest(), nullptr);
}

TEST_F(TelemetryTest, PublishesMechanismChannelsWithoutOdometry) {
  sim.publish(shooter::topics::kState, Shooter());
  sim.step(50ms);

  const auto channels = Channels();
  EXPECT_NEAR(channels.at("shooter/flywheel"), 42.0, 1e-12);
  EXPECT_NEAR(channels.at("pose/valid"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("mechanism/stale"), 0.0, 1e-12);
  EXPECT_EQ(channels.count("odometry/vx"), 0u);
  const Frame* frame = Latest();
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->chassis(), nullptr);
  EXPECT_EQ(frame->chassis()->x(), 0.0);
  EXPECT_EQ(frame->chassis()->y(), 0.0);
  EXPECT_EQ(frame->chassis()->yaw(), 0.0);
}

TEST_F(TelemetryTest, PoseValidTracksFreshPose) {
  sim.publish(shooter::topics::kState, Shooter());
  sim.step(20ms);
  EXPECT_NEAR(Channels().at("pose/valid"), 0.0, 1e-12);

  sim.publish(odometry::topics::kState, Odometry(3.25, 1.5, 0.25));
  sim.step(20ms);
  EXPECT_NEAR(Channels().at("pose/valid"), 1.0, 1e-12);
  const Frame* frame = Latest();
  ASSERT_NE(frame, nullptr);
  EXPECT_NEAR(frame->chassis()->x(), 3.25, 1e-12);
}

TEST_F(TelemetryTest, OmitsStaleMechanismRowsButKeepsPublishingPose) {
  sim.publish(shooter::topics::kState, Shooter());
  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(50ms);
  EXPECT_GT(Channels().count("shooter/flywheel"), 0u);

  sim.step(150ms);
  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(30ms);
  const auto channels = Channels();
  EXPECT_EQ(channels.count("shooter/flywheel"), 0u);
  EXPECT_NEAR(channels.at("mechanism/stale"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("pose/valid"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("odometry/vx"), 1.5, 1e-12);
}

TEST_F(TelemetryTest, MarksStaleOdometryAndHoldsFieldView) {
  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(50ms);
  ASSERT_NE(Latest(), nullptr);

  sim.step(150ms);
  sim.publish(shooter::topics::kState, Shooter());
  sim.step(30ms);
  const auto channels = Channels();
  EXPECT_NEAR(channels.at("pose/valid"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("odometry/stale"), 1.0, 1e-12);
  EXPECT_EQ(channels.count("odometry/vx"), 0u);
  EXPECT_NEAR(channels.at("shooter/flywheel"), 42.0, 1e-12);
  const Frame* frame = Latest();
  ASSERT_NE(frame, nullptr);
  EXPECT_EQ(frame->chassis()->x(), 0.0);
  EXPECT_EQ(frame->chassis()->y(), 0.0);
}

TEST_F(TelemetryTest, DropsAFrameWhosePoseIsNaN) {
  sim.publish(odometry::topics::kState,
              Odometry(std::numeric_limits<double>::quiet_NaN(), 0, 0));
  sim.step(50ms);
  EXPECT_EQ(Latest(), nullptr);

  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(50ms);
  EXPECT_NEAR(Channels().at("pose/valid"), 1.0, 1e-12);
}

TEST_F(TelemetryTest, NaNValuesMarkAbsentChannels) {
  const shooter::ShooterState shooter{
      0, std::numeric_limits<double>::quiet_NaN(), 42.0, true, true, true};
  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.publish(shooter::topics::kState, shooter);
  sim.step(50ms);
  const auto channels = Channels();
  const Frame* frame = Latest();
  ASSERT_NE(frame, nullptr);
  EXPECT_EQ(channels.count("shooter/flywheel"), 0u);
  EXPECT_NEAR(channels.at("pose/valid"), 1.0, 1e-12);
  // The slot itself carries NaN, aligned with the dictionary.
  const auto& dictionary = telemetry.dictionary();
  for (std::size_t i = 0; i < dictionary.size(); ++i) {
    if (dictionary[i].name == "shooter/flywheel") {
      EXPECT_TRUE(std::isnan(frame->values()->Get(i)));
    }
  }
}

TEST_F(TelemetryTest, ForwardsShooterStateIntoMechanismChannels) {
  sim.publish(odometry::topics::kState, Odometry(3.25, 1.5, 0.25));
  sim.publish(shooter::topics::kState, Shooter());
  sim.step(50ms);

  const auto channels = Channels();
  EXPECT_NEAR(channels.at("shooter/flywheel"), 42.0, 1e-12);
  EXPECT_NEAR(channels.at("shooter/beam_broken"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("odometry/vx"), 1.5, 1e-12);
}

TEST_F(TelemetryTest, IntakeInhibitIsAnEnumChannelWithLabels) {
  sim.publish(odometry::topics::kState, Odometry(3.25, 1.5, 0.25));
  sim.publish(intake::topics::kState, Intake(intake::Inhibit::TARGET_STALE));
  sim.step(10ms);  // one tick: the latest frame is the dictionary frame

  const auto channels = Channels();
  EXPECT_NEAR(channels.at("intake/requested"), 0.0, 1e-6);
  EXPECT_NEAR(channels.at("intake/measured"), 20.0, 1e-6);
  EXPECT_NEAR(channels.at("intake/inhibit"),
              static_cast<double>(intake::Inhibit::TARGET_STALE), 1e-12);

  // The label lives in the dictionary, not the unit string.
  const Frame* frame = Latest();
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->dictionary(), nullptr);  // first frame
  bool found = false;
  for (const auto* info : *frame->dictionary()) {
    if (info->name()->str() != "intake/inhibit") continue;
    found = true;
    EXPECT_TRUE(info->unit() == nullptr || info->unit()->str().empty());
    ASSERT_NE(info->labels(), nullptr);
    ASSERT_EQ(info->labels()->size(), 6u);
    EXPECT_EQ(info->labels()->Get(0)->str(), "DRIVE");
    EXPECT_EQ(info->labels()->Get(3)->str(), "TARGET_STALE");
    EXPECT_EQ(info->labels()->Get(5)->str(), "DEMAND_NONFINITE");
  }
  EXPECT_TRUE(found);
}

TEST_F(TelemetryTest, DropsStaleShooterWhileIntakeStaysLive) {
  sim.publish(shooter::topics::kState, Shooter());
  for (int i = 0; i < 8; ++i) {
    sim.publish(intake::topics::kState, Intake(intake::Inhibit::DRIVE));
    sim.step(20ms);
  }

  const auto channels = Channels();
  EXPECT_EQ(channels.count("shooter/flywheel"), 0u);
  EXPECT_NEAR(channels.at("shooter/stale"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("intake/measured"), 20.0, 1e-6);
  EXPECT_NEAR(channels.at("intake/stale"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("mechanism/stale"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("pose/valid"), 0.0, 1e-12);
}

TEST_F(TelemetryTest, ForwardsBridgeHealthAndMarksItStale) {
  sim.publish(::bridge::topics::kStatus,
              ::bridge::BridgeStatus{100, 90, 2, 3, 50, 1500, 0b101, 3, true});
  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(30ms);
  auto channels = Channels();
  EXPECT_NEAR(channels.at("bridge/state_age_us"), 1500.0, 1e-12);
  EXPECT_NEAR(channels.at("bridge/identity_rejected"), 2.0, 1e-12);
  EXPECT_NEAR(channels.at("bridge/ownership_rejected"), 3.0, 1e-12);
  EXPECT_NEAR(channels.at("bridge/timed_out_mask"), 5.0, 1e-12);
  EXPECT_NEAR(channels.at("bridge/configured"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("bridge/stale"), 0.0, 1e-12);
  // Units live in the dictionary now.
  for (const ChannelDef& channel : telemetry.dictionary()) {
    if (channel.name == "bridge/state_age_us") EXPECT_EQ(channel.unit, "us");
  }

  for (int i = 0; i < 8; ++i) {
    sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
    sim.step(20ms);
  }
  channels = Channels();
  EXPECT_EQ(channels.count("bridge/state_age_us"), 0u);
  EXPECT_EQ(channels.count("bridge/configured"), 0u);
  EXPECT_NEAR(channels.at("bridge/stale"), 1.0, 1e-12);
}

TEST_F(TelemetryTest, ReadsDriverStationFromHardwareTopic) {
  // No DS packet forwarded yet: stale, and the mode channels stay absent.
  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(30ms);
  auto channels = Channels();
  EXPECT_NEAR(channels.at("ds/stale"), 1.0, 1e-12);
  EXPECT_EQ(channels.count("ds/enabled"), 0u);

  driver_station::DriverStationData data{};
  data.fms.flags = driver_station::kDsAttached | driver_station::kEnabled |
                   driver_station::kTeleop;
  data.fms.match_time_s = 42.0f;
  hw.set_driver_station(data);
  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(50ms);
  channels = Channels();
  EXPECT_NEAR(channels.at("ds/stale"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("ds/enabled"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("ds/teleop"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("ds/autonomous"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("ds/match_time"), 42.0, 1e-9);
}

TEST_F(TelemetryTest, StreamAndDictionaryIdsAreStable) {
  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(30ms);
  const Frame* first = Latest();
  ASSERT_NE(first, nullptr);
  EXPECT_NE(telemetry.stream_id(), 0u);
  EXPECT_EQ(first->stream_id(), telemetry.stream_id());
  EXPECT_EQ(first->dictionary_id(), telemetry.dictionary_id());
  EXPECT_EQ(first->dictionary_id(), DictionaryId(telemetry.dictionary()));
  const std::uint64_t stream = first->stream_id();
  const std::uint32_t dictionary = first->dictionary_id();

  sim.publish(odometry::topics::kState, Odometry(2.0, 2.0, 0.0));
  sim.step(30ms);
  const Frame* later = Latest();
  ASSERT_NE(later, nullptr);
  EXPECT_EQ(later->stream_id(), stream);
  EXPECT_EQ(later->dictionary_id(), dictionary);
}

TEST_F(TelemetryTest, DictionaryRidesFirstAndEveryHundredthFrame) {
  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  std::map<std::uint64_t, bool> has_dictionary;
  std::uint64_t last_sequence = 0;
  bool saw_any = false;
  for (int i = 0; i < 210; ++i) {
    sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
    sim.step(10ms);
    const Frame* frame = Latest();
    ASSERT_NE(frame, nullptr);
    if (saw_any && frame->sequence_id() == last_sequence) continue;
    saw_any = true;
    last_sequence = frame->sequence_id();
    has_dictionary[frame->sequence_id()] = frame->dictionary() != nullptr;
    // Value-only frames obey the radio budget; dictionary frames fit a slot.
    const std::size_t bytes = LatestBytes();
    if (frame->dictionary() == nullptr) {
      EXPECT_LE(bytes, telemetry_wire::kMaxValueFrameBytes)
          << "sequence " << frame->sequence_id();
    } else {
      EXPECT_LE(bytes, telemetry_wire::slot_bytes);
    }
  }
  ASSERT_TRUE(has_dictionary.count(0));
  EXPECT_TRUE(has_dictionary.at(0));
  EXPECT_FALSE(has_dictionary.at(1));
  EXPECT_FALSE(has_dictionary.at(99));
  ASSERT_TRUE(has_dictionary.count(100));
  EXPECT_TRUE(has_dictionary.at(100));
  EXPECT_FALSE(has_dictionary.at(101));
  ASSERT_TRUE(has_dictionary.count(200));
  EXPECT_TRUE(has_dictionary.at(200));
}

TEST_F(TelemetryTest, WorstCaseValueFrameFitsTheBudget) {
  const std::size_t worst =
      MeasureValueFrameBytes(telemetry.dictionary().size());
  std::cout << "telemetry: worst-case value frame " << worst
            << " bytes (budget " << telemetry_wire::kMaxValueFrameBytes
            << "), dictionary " << telemetry.dictionary().size()
            << " channels\n";
  EXPECT_LE(worst, telemetry_wire::kMaxValueFrameBytes);

  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.publish(shooter::topics::kState, Shooter());
  sim.publish(intake::topics::kState, Intake(intake::Inhibit::DRIVE));
  sim.step(10ms);  // one tick: the latest frame is the dictionary frame
  const std::size_t first_bytes = LatestBytes();  // dictionary frame
  const Frame* frame = Latest();
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->dictionary(), nullptr);
  std::cout << "telemetry: measured dictionary frame " << first_bytes
            << " bytes\n";
  EXPECT_LE(first_bytes, telemetry_wire::slot_bytes);

  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(20ms);
  const std::size_t value_bytes = LatestBytes();
  std::cout << "telemetry: measured value frame " << value_bytes << " bytes\n";
  EXPECT_LE(value_bytes, telemetry_wire::kMaxValueFrameBytes);
  EXPECT_GT(value_bytes, 0u);
}

TEST(Telemetry, PublishesTomlConfiguredSubsystemStateChannels) {
  Sim sim = Sim::FromToml(R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.telemetry]
period_us = 10000
topic = "/talos/telemetry"
stale_timeout_us = 100000

[subsystems.wrist]
period_us = 20000
teleop_button = 3
teleop_demand = 25.0
telemetry = true

[subsystems.wrist.motors.wrist_motor]
can_id = 1
)");
  Telemetry& telemetry = sim.add<Telemetry>("telemetry");

  sim.publish("/wrist/state", GenericState{1, {}, 25.0, 1, {}});
  sim.step(10ms);  // one tick: the latest frame is the dictionary frame

  const auto latest = sim.latest_bytes("/talos/telemetry");
  ASSERT_TRUE(latest.has_value());
  const Frame* frame = Talos::Telemetry::GetSizePrefixedFrame(latest->data());
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->dictionary(), nullptr);
  ASSERT_NE(frame->values(), nullptr);

  std::map<std::string, double> channels;
  std::map<std::string, std::string> units;
  ASSERT_EQ(frame->dictionary()->size(), frame->values()->size());
  for (std::size_t i = 0; i < frame->dictionary()->size(); ++i) {
    const auto* info = frame->dictionary()->Get(i);
    const double value = frame->values()->Get(i);
    if (info->unit() != nullptr)
      units[info->name()->str()] = info->unit()->str();
    if (!std::isnan(value)) channels[info->name()->str()] = value;
  }
  EXPECT_NEAR(channels.at("wrist/value"), 25.0, 1e-12);
  EXPECT_EQ(units.at("wrist/value"), "rev/s");
  EXPECT_NEAR(channels.at("wrist/connected"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("wrist/stale"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("mechanism/stale"), 0.0, 1e-12);
  EXPECT_EQ(telemetry.dictionary().size(), frame->dictionary()->size());

  sim.step(150ms);
  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(20ms);

  const auto stale_bytes = sim.latest_bytes("/talos/telemetry");
  ASSERT_TRUE(stale_bytes.has_value());
  const Frame* stale_frame =
      Talos::Telemetry::GetSizePrefixedFrame(stale_bytes->data());
  ASSERT_NE(stale_frame, nullptr);
  std::map<std::string, double> stale_channels;
  const auto& dictionary = telemetry.dictionary();
  for (std::size_t i = 0; i < dictionary.size(); ++i) {
    const double value = stale_frame->values()->Get(i);
    if (!std::isnan(value)) stale_channels[dictionary[i].name] = value;
  }
  EXPECT_EQ(stale_channels.count("wrist/value"), 0u);
  EXPECT_NEAR(stale_channels.at("wrist/stale"), 1.0, 1e-12);
  EXPECT_NEAR(stale_channels.at("mechanism/stale"), 1.0, 1e-12);
}

// The Studio feed bypasses the loop, so --describe must still declare it.
TEST(Binary, DescribeDeclaresTheStudioFeedAsAnExternalSender) {
  const std::string command = "'" + Runfile("TALOS_TEST_NODE") +
                              "' --describe --config '" +
                              Runfile("TALOS_TEST_CONFIG") + "' 2>&1";
  FILE* pipe = ::popen(command.c_str(), "r");
  ASSERT_NE(pipe, nullptr);
  std::string output;
  char buffer[4096];
  while (const std::size_t n = std::fread(buffer, 1, sizeof buffer, pipe)) {
    output.append(buffer, n);
  }
  const int status = ::pclose(pipe);
  ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0) << output;
  EXPECT_NE(
      output.find(
          R"({"kind": "SENDER", "name": "/talos/telemetry", "message_bytes": )" +
          std::to_string(telemetry_wire::slot_bytes) +
          R"(, "external": true, "optional": false})"),
      std::string::npos)
      << output;
  for (const char* watched :
       {"/odometry/state", "/drivetrain/state", "/shooter/state",
        "/intake/state", "/hw/state/driver_station"}) {
    EXPECT_NE(output.find(std::string{R"({"kind": "WATCHER", "name": ")"} +
                          watched + "\""),
              std::string::npos)
        << watched << "\n"
        << output;
  }
}

}  // namespace
}  // namespace talos::telemetry
