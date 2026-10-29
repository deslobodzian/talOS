#include "telemetry/node.h"

#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "studio/schema/wire.h"
#include "talOS/node/testing.h"

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
  return intake::IntakeState{0, true, 20.0, false, 0.0, inhibit};
}

class TelemetryTest : public ::testing::Test {
 protected:
  // The newest frame on the Studio topic, checked the way the bridge and
  // Studio read it; nullptr when none was published.
  const Frame* Latest() {
    latest_ =
        sim.latest_bytes(sim.node("telemetry").param<std::string>("topic"));
    if (!latest_) return nullptr;
    // The bridge forwards prefix plus buffer; Studio parses it size-prefixed.
    const auto payload = studio::payload(*latest_);
    EXPECT_EQ(payload.size(), latest_->size());
    flatbuffers::Verifier verifier{
        reinterpret_cast<const std::uint8_t*>(payload.data()), payload.size()};
    EXPECT_TRUE(Talos::Telemetry::VerifySizePrefixedFrameBuffer(verifier));
    return Talos::Telemetry::GetSizePrefixedFrame(latest_->data());
  }

  std::map<std::string, double> Channels() {
    const Frame* frame = Latest();
    std::map<std::string, double> channels;
    if (frame == nullptr || frame->channels() == nullptr) return channels;
    for (const auto* channel : *frame->channels()) {
      EXPECT_TRUE(
          channels.emplace(channel->name()->str(), channel->value()).second)
          << "Studio rejects a frame with a duplicate channel name";
    }
    return channels;
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  Telemetry& telemetry = sim.add<Telemetry>("telemetry");

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
  // yaw_rot is rotations on the robot's wire; Studio poses are radians.
  EXPECT_NEAR(frame->chassis()->yaw(), 0.25 * 2 * std::numbers::pi, 1e-12);
  // 10 ms ticks from 10 ms: the frame at 50 ms is the fifth.
  EXPECT_EQ(frame->sequence_id(), 4u);

  const auto channels = Channels();
  EXPECT_NEAR(channels.at("pose/valid"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("odometry/vx"), 1.5, 1e-12);
  EXPECT_NEAR(channels.at("odometry/omega"), 0.25, 1e-12);
  // No shooter publisher: its rows stay out rather than holding the frame.
  EXPECT_EQ(channels.count("shooter/flywheel"), 0u);
  EXPECT_NEAR(channels.at("drivetrain/vx"), 1.5, 1e-12);
  EXPECT_NEAR(channels.at("drivetrain/enabled"), 1.0, 1e-12);
}

TEST_F(TelemetryTest, PublishesNothingUntilAPoseExists) {
  // Drivetrain alone carries no field position. Publishing here would draw
  // the robot at the origin as though that were measured.
  sim.publish(drivetrain::topics::kState,
              drivetrain::DrivetrainState{0, true, 1, 0, 0, 0, 0});
  sim.step(50ms);
  EXPECT_EQ(Latest(), nullptr);
}

TEST_F(TelemetryTest, PublishesMechanismChannelsWithoutOdometry) {
  // Mechanism rows publish with no pose at all: the operator sees the
  // shooter spinning up while localization is still converging.
  sim.publish(shooter::topics::kState, Shooter());
  sim.step(50ms);

  const auto channels = Channels();
  EXPECT_NEAR(channels.at("shooter/flywheel"), 42.0, 1e-12);
  EXPECT_NEAR(channels.at("pose/valid"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("mechanism/stale"), 0.0, 1e-12);
  EXPECT_EQ(channels.count("odometry/vx"), 0u);
  // Chassis is zeros by contract until pose/valid reads 1; the field view
  // must gate drawing on that flag rather than on the struct's presence.
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

  // Both sources age out past the 100 ms horizon; then odometry alone
  // recovers. The frame must carry the live pose without the frozen
  // shooter row, and say so in-stream.
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

  // Odometry ages out; a fresh mechanism push keeps frames flowing, but the
  // field view must wait: pose/valid drops, odometry rows go missing, and
  // chassis holds zeros rather than a frozen pose drawn as live.
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

TEST_F(TelemetryTest, DropsAFrameWhoseValuesWouldFailStudioValidation) {
  // Studio throws on a non-finite number and discards the whole frame, so a
  // NaN pose must be dropped here rather than shipped.
  sim.publish(odometry::topics::kState,
              Odometry(std::numeric_limits<double>::quiet_NaN(), 0, 0));
  sim.step(50ms);
  EXPECT_EQ(Latest(), nullptr);

  sim.publish(odometry::topics::kState, Odometry(1.0, 2.0, 0.0));
  sim.step(50ms);
  EXPECT_NEAR(Channels().at("pose/valid"), 1.0, 1e-12);
}

TEST_F(TelemetryTest, ForwardsShooterStateIntoMechanismChannels) {
  sim.publish(odometry::topics::kState, Odometry(3.25, 1.5, 0.25));
  sim.publish(shooter::topics::kState, Shooter());
  sim.step(50ms);

  const auto channels = Channels();
  EXPECT_NEAR(channels.at("shooter/flywheel"), 42.0, 1e-12);
  EXPECT_NEAR(channels.at("shooter/beam_broken"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("odometry/vx"), 1.5, 1e-12);
  const Frame* frame = Latest();
  ASSERT_NE(frame, nullptr);
  EXPECT_NEAR(frame->chassis()->x(), 3.25, 1e-12);
}

TEST_F(TelemetryTest, ForwardsIntakeStudioViewWithReadableInhibit) {
  sim.publish(odometry::topics::kState, Odometry(3.25, 1.5, 0.25));
  // Inhibited (stale target): measured 20, requested 0.
  sim.publish(intake::topics::kState, Intake(intake::Inhibit::TARGET_STALE));
  sim.step(50ms);

  const auto channels = Channels();
  EXPECT_NEAR(channels.at("intake/requested"), 0.0, 1e-6);
  EXPECT_NEAR(channels.at("intake/measured"), 20.0, 1e-6);
  // The numeric code stays the wire contract; the unit names it for Studio,
  // which renders each channel's unit next to its value.
  const Frame* frame = Latest();
  ASSERT_NE(frame, nullptr);
  bool found = false;
  for (const auto* channel : *frame->channels()) {
    if (channel->name()->str() != "intake/inhibit") continue;
    found = true;
    EXPECT_NEAR(channel->value(),
                static_cast<double>(intake::Inhibit::TARGET_STALE), 1e-12);
    ASSERT_NE(channel->unit(), nullptr);
    EXPECT_EQ(channel->unit()->str(), "TARGET_STALE");
  }
  EXPECT_TRUE(found);
}

TEST_F(TelemetryTest, DropsStaleShooterWhileIntakeStaysLive) {
  // Shooter speaks once; intake keeps reporting every 20 ms. Past the
  // 100 ms horizon the shooter rows must vanish -- frozen flywheel numbers
  // beside a live intake would read as healthy -- with a stale marker in
  // their place. No odometry: mechanism traffic alone publishes.
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

// The Studio feed bypasses the loop, so --describe must still declare it:
// the bridge reads it, and a graph without it reports the bridge's input as
// published by nobody.
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
          R"({"kind": "SENDER", "name": "/talos/telemetry", "message_bytes": 65536, "external": true, "optional": false})"),
      std::string::npos)
      << output;
  for (const char* watched :
       {"/odometry/state", "/drivetrain/state", "/driver_station/state",
        "/shooter/state", "/intake/state"}) {
    EXPECT_NE(output.find(std::string{R"({"kind": "WATCHER", "name": ")"} +
                          watched + "\""),
              std::string::npos)
        << watched << "\n"
        << output;
  }
}

}  // namespace
}  // namespace talos::telemetry
