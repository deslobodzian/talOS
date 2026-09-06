#include "2026-robot/main_processor/telemetry/node.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <numbers>
#include <optional>
#include <string>

#include "talOS/events/simulated_event_loop.h"

namespace talos::telemetry {
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
  return odometry::OdometryState{0, x, y, yaw_rot, 1.5, -0.5, 0.25, 42.0, true};
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

TEST(Telemetry, PublishesChassisPoseAndChannelsFromOdometry) {
  const auto topic = TestTopic("pose");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  TelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  loop.inject(odometry::kOdometryTopic, Odometry(3.25, 1.5, 0.25));
  loop.inject(
      talos::drive::kDrivetrainStateTopic,
      talos::drive::DrivetrainState{0, true, 1.5, -0.5, 0.25, 0.25, 0.1});
  loop.run_for(50ms);

  ASSERT_GT(node.frames_published(), 0u);
  EXPECT_EQ(node.frames_rejected(), 0u);

  const auto slot = consumer.Drain();
  ASSERT_TRUE(slot.has_value());
  const auto* frame = Decode(*slot);
  ASSERT_NE(frame, nullptr);

  ASSERT_NE(frame->chassis(), nullptr);
  EXPECT_NEAR(frame->chassis()->x(), 3.25, 1e-12);
  EXPECT_NEAR(frame->chassis()->y(), 1.5, 1e-12);
  // yaw_rot is rotations on the robot's wire; Studio poses are radians.
  EXPECT_NEAR(frame->chassis()->yaw(), 0.25 * 2 * std::numbers::pi, 1e-12);

  ASSERT_NE(frame->channels(), nullptr);
  std::map<std::string, double> channels;
  for (const auto* channel : *frame->channels()) {
    ASSERT_NE(channel->name(), nullptr);
    EXPECT_TRUE(
        channels.emplace(channel->name()->str(), channel->value()).second)
        << "Studio rejects a frame with a duplicate channel name";
  }
  EXPECT_NEAR(channels.at("odometry/vx"), 1.5, 1e-12);
  EXPECT_NEAR(channels.at("odometry/omega"), 0.25, 1e-12);
  EXPECT_NEAR(channels.at("shooter/flywheel"), 42.0, 1e-12);
  EXPECT_NEAR(channels.at("shooter/beam_broken"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("drivetrain/vx"), 1.5, 1e-12);
}

TEST(Telemetry, PublishesNothingUntilAPoseExists) {
  const auto topic = TestTopic("nopose");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  TelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  // Driver station and drivetrain alone carry no field position. Publishing
  // here would draw the robot at the origin as though that were measured.
  loop.inject(talos::drive::kDrivetrainStateTopic,
              talos::drive::DrivetrainState{0, true, 1, 0, 0, 0, 0});
  loop.run_for(50ms);
  EXPECT_EQ(node.frames_published(), 0u);
  EXPECT_FALSE(consumer.Drain().has_value());
}

TEST(Telemetry, DropsAFrameWhoseValuesWouldFailStudioValidation) {
  const auto topic = TestTopic("nan");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  TelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  // Studio throws on a non-finite number and discards the whole frame, so a
  // NaN pose must be dropped here rather than shipped.
  loop.inject(odometry::kOdometryTopic,
              Odometry(std::numeric_limits<double>::quiet_NaN(), 0, 0));
  loop.run_for(50ms);
  EXPECT_EQ(node.frames_published(), 0u);
  EXPECT_GT(node.frames_rejected(), 0u);

  loop.inject(odometry::kOdometryTopic, Odometry(1.0, 2.0, 0.0));
  loop.run_for(50ms);
  EXPECT_GT(node.frames_published(), 0u);
}

}  // namespace
}  // namespace talos::telemetry
