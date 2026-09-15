#include "talos_nodes/year2026_telemetry/node.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <map>
#include <numbers>
#include <optional>
#include <string>

#include "talOS/events/simulated_event_loop.h"

namespace talos::telemetry::year2026 {
namespace {

using namespace std::chrono_literals;

// Real shared memory, so each run needs its own name, and the `test` owner is
// reserved for exactly this: the launcher refuses to spawn a node that uses it,
// so a fixture name can never be mistaken for a topic the robot runs on.
//
// RTMS caps a topic at 30 characters after the leading slash and
// `test/telemetry/` spends 15 of them, which is why the suffixes here are short
// enough to leave room for a long pid.
std::string TestTopic(const char* suffix) {
  return "/test/telemetry/" + std::string{suffix} + "_" +
         std::to_string(::getpid());
}

TelemetryConfig TestConfig(const std::string& topic) {
  TelemetryConfig config{};
  config.topic = topic;
  config.period_us = 10000;
  return config;
}

odometry::OdometryState Odometry(double x, double y, double yaw_rot) {
  return odometry::OdometryState{0, x, y, yaw_rot, 1.5, -0.5, 0.25};
}

// Attaches as a second reader on the node's topic and decodes the newest frame.
class Consumer {
 public:
  explicit Consumer(const std::string& topic)
      : queue_{topic, studio::slot_bytes, 8, studio::slot_count,
               RTMSOptions{OverflowPolicy::DROP_NEWEST, ReadMode::SEQUENCE}},
        reader_{queue_.register_reader()} {
    EXPECT_TRUE(reader_.has_value());
  }

  // Returns the last frame available, or nullopt when nothing was published.
  std::optional<std::vector<std::byte>> Drain() {
    std::optional<std::vector<std::byte>> latest;
    std::vector<std::byte> slot(studio::slot_bytes);
    MessageInfo info{};
    while (queue_.read_next(*reader_, slot, info) == ReadResult::OK) {
      latest = slot;
    }
    return latest;
  }

 private:
  RTMSQueue queue_;
  std::optional<std::size_t> reader_;
};

const Talos::Telemetry::Frame* Decode(const std::vector<std::byte>& slot) {
  const auto payload = studio::payload(slot);
  EXPECT_FALSE(payload.empty());
  if (payload.empty()) return nullptr;
  // The bridge forwards prefix plus buffer; Studio parses it size-prefixed.
  return flatbuffers::GetSizePrefixedRoot<Talos::Telemetry::Frame>(
      payload.data());
}

std::map<std::string, double> Channels(const Talos::Telemetry::Frame* frame) {
  std::map<std::string, double> channels;
  for (const auto* channel : *frame->channels()) {
    EXPECT_TRUE(
        channels.emplace(channel->name()->str(), channel->value()).second)
        << "Studio rejects a frame with a duplicate channel name";
  }
  return channels;
}

TEST(YearTelemetry, ForwardsShooterStateIntoMechanismChannels) {
  const auto topic = TestTopic("shoot");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  YearTelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  loop.inject(odometry::kOdometryTopic, Odometry(3.25, 1.5, 0.25));
  loop.inject(talos::shooter::kShooterStateTopic,
              talos::shooter::ShooterState{0, 42.0, 42.0, true, true, true});
  loop.run_for(50ms);

  ASSERT_GT(node.frames_published(), 0u);
  const auto slot = consumer.Drain();
  ASSERT_TRUE(slot.has_value());
  const auto* frame = Decode(*slot);
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->channels(), nullptr);

  const auto channels = Channels(frame);
  EXPECT_NEAR(channels.at("shooter/flywheel"), 42.0, 1e-12);
  EXPECT_NEAR(channels.at("shooter/beam_broken"), 1.0, 1e-12);
  // The composed core still owns its pose channels in the same frame.
  EXPECT_NEAR(frame->chassis()->x(), 3.25, 1e-12);
  EXPECT_NEAR(channels.at("odometry/vx"), 1.5, 1e-12);
}

TEST(YearTelemetry, ForwardsIntakeStudioViewIntoMechanismChannels) {
  const auto topic = TestTopic("intake");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  YearTelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  loop.inject(odometry::kOdometryTopic, Odometry(3.25, 1.5, 0.25));
  // inhibited here (stale target): measured 20, requested 0, reason 3.
  loop.inject(talos::intake::kIntakeStateTopic,
              talos::intake::IntakeState{0, true, 20.0, false, 0.0,
                                         talos::intake::kInhibitTargetStale});
  loop.run_for(50ms);

  ASSERT_GT(node.frames_published(), 0u);
  const auto slot = consumer.Drain();
  ASSERT_TRUE(slot.has_value());
  const auto* frame = Decode(*slot);
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->channels(), nullptr);

  const auto channels = Channels(frame);
  EXPECT_NEAR(channels.at("intake/requested"), 0.0, 1e-6);
  EXPECT_NEAR(channels.at("intake/measured"), 20.0, 1e-6);
  EXPECT_NEAR(channels.at("intake/inhibit"),
              static_cast<double>(talos::intake::kInhibitTargetStale), 1e-12);
}

TEST(YearTelemetry, PublishesShooterWithoutOdometry) {
  const auto topic = TestTopic("noodom");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  YearTelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  // No odometry at all: mechanism rows still publish, while the field view
  // waits on pose/valid.
  loop.inject(talos::shooter::kShooterStateTopic,
              talos::shooter::ShooterState{0, 42.0, 42.0, true, true, true});
  loop.run_for(50ms);

  ASSERT_GT(node.frames_published(), 0u);
  const auto slot = consumer.Drain();
  ASSERT_TRUE(slot.has_value());
  const auto* frame = Decode(*slot);
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->channels(), nullptr);

  const auto channels = Channels(frame);
  EXPECT_NEAR(channels.at("shooter/flywheel"), 42.0, 1e-12);
  EXPECT_NEAR(channels.at("pose/valid"), 0.0, 1e-12);
  EXPECT_EQ(channels.count("odometry/vx"), 0u);
  ASSERT_NE(frame->chassis(), nullptr);
  EXPECT_EQ(frame->chassis()->x(), 0.0);
}

TEST(YearTelemetry, InhibitReasonIsHumanReadable) {
  const auto topic = TestTopic("reason");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  YearTelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  loop.inject(odometry::kOdometryTopic, Odometry(3.25, 1.5, 0.25));
  // inhibited here (stale target): measured 20, requested 0, reason 3.
  loop.inject(talos::intake::kIntakeStateTopic,
              talos::intake::IntakeState{0, true, 20.0, false, 0.0,
                                         talos::intake::kInhibitTargetStale});
  loop.run_for(50ms);

  ASSERT_GT(node.frames_published(), 0u);
  const auto slot = consumer.Drain();
  ASSERT_TRUE(slot.has_value());
  const auto* frame = Decode(*slot);
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->channels(), nullptr);

  // The numeric code stays the wire contract; the unit names it for Studio,
  // which renders each channel's unit next to its value.
  bool found = false;
  for (const auto* channel : *frame->channels()) {
    if (channel->name()->str() == "intake/inhibit") {
      found = true;
      EXPECT_NEAR(channel->value(), 3.0, 1e-12);
      ASSERT_NE(channel->unit(), nullptr);
      EXPECT_EQ(channel->unit()->str(), "TARGET_STALE");
    }
  }
  EXPECT_TRUE(found);
}

TEST(YearTelemetry, DropsStaleShooterWhileIntakeStaysLive) {
  const auto topic = TestTopic("split");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  YearTelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  // Shooter speaks once; intake keeps reporting every 20 ms. Past the
  // 100 ms horizon the shooter rows must vanish -- frozen flywheel numbers
  // beside a live intake would read as healthy -- with a stale marker in
  // their place. No odometry is injected: mechanism traffic alone publishes.
  loop.inject(talos::shooter::kShooterStateTopic,
              talos::shooter::ShooterState{0, 42.0, 42.0, true, true, true});
  for (int i = 0; i < 8; ++i) {
    loop.inject(talos::intake::kIntakeStateTopic,
                talos::intake::IntakeState{0, true, 20.0, false, 0.0,
                                           talos::intake::kInhibitDrive});
    loop.run_for(20ms);
    // The test queue holds 8 frames; drain as we go so the newest frame is
    // never the one DROP_NEWEST discards. Keep the last iteration's frames
    // for the assertions below.
    if (i + 1 < 8) (void)consumer.Drain();
  }

  ASSERT_GT(node.frames_published(), 0u);
  const auto slot = consumer.Drain();
  ASSERT_TRUE(slot.has_value());
  const auto* frame = Decode(*slot);
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->channels(), nullptr);

  const auto channels = Channels(frame);
  EXPECT_EQ(channels.count("shooter/flywheel"), 0u);
  EXPECT_NEAR(channels.at("shooter/stale"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("intake/measured"), 20.0, 1e-6);
  EXPECT_NEAR(channels.at("intake/stale"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("mechanism/stale"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("pose/valid"), 0.0, 1e-12);
}

TEST(YearTelemetry, OmitsMechanismChannelsUntilShooterPublishes) {
  const auto topic = TestTopic("nomech");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  YearTelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  // Pose alone still publishes: a missing mechanism publisher leaves those
  // channels out rather than holding the whole frame.
  loop.inject(odometry::kOdometryTopic, Odometry(1.0, 2.0, 0.0));
  loop.run_for(50ms);

  ASSERT_GT(node.frames_published(), 0u);
  const auto slot = consumer.Drain();
  ASSERT_TRUE(slot.has_value());
  const auto* frame = Decode(*slot);
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(frame->channels(), nullptr);
  EXPECT_EQ(Channels(frame).count("shooter/flywheel"), 0u);
}

}  // namespace
}  // namespace talos::telemetry::year2026
