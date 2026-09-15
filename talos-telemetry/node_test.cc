#include "talos_nodes/telemetry/node.h"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <map>
#include <numbers>
#include <optional>
#include <string>

#include "talOS/events/simulated_event_loop.h"
#include "talOS/memory/shared_memory_ptr.h"

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
  // Year mechanism channels are the year wrapper's job (see
  // 2026-robot/main_processor/telemetry): the core frame carries no
  // shooter/* names at all.
  EXPECT_EQ(channels.count("shooter/flywheel"), 0u);
  EXPECT_NEAR(channels.at("drivetrain/vx"), 1.5, 1e-12);
}

// The Studio segment opens on the first publishable frame, not in the
// constructor and not in Start: --describe builds the node on a simulated
// loop and never ticks, so describing (or merely constructing) the node must
// leave no shared-memory segment behind. shm_open with O_RDWR attaches
// without creating, so ENOENT proves absence.
bool ShmSegmentExists(const std::string& topic) {
  const int fd = ::shm_open(ShmObjectName(topic).c_str(), O_RDWR, 0666);
  if (fd == -1) return false;
  ::close(fd);
  return true;
}

TEST(Telemetry, ConstructorAndStartCreateNoSharedMemory) {
  const auto topic = TestTopic("defer");
  EXPECT_FALSE(ShmSegmentExists(topic));

  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  TelemetryNode node{loop, TestConfig(topic)};
  EXPECT_FALSE(ShmSegmentExists(topic));

  node.Start(event::MonotonicTime::from_nanos(1'000'000));
  // Ticks fire with no pose: nothing publishable, still no segment.
  loop.run_for(50ms);
  EXPECT_EQ(node.frames_published(), 0u);
  EXPECT_FALSE(ShmSegmentExists(topic));

  // First pose publishes, opening the segment on first use.
  loop.inject(odometry::kOdometryTopic, Odometry(1.0, 2.0, 0.0));
  loop.run_for(50ms);
  EXPECT_GT(node.frames_published(), 0u);
  EXPECT_TRUE(ShmSegmentExists(topic));
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

std::map<std::string, double> Channels(const Talos::Telemetry::Frame* frame) {
  std::map<std::string, double> channels;
  for (const auto* channel : *frame->channels()) {
    EXPECT_TRUE(
        channels.emplace(channel->name()->str(), channel->value()).second)
        << "Studio rejects a frame with a duplicate channel name";
  }
  return channels;
}

TEST(Telemetry, PublishesMechanismChannelsWithoutOdometry) {
  const auto topic = TestTopic("mechonly");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  TelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  // Mechanism rows publish with no pose at all: the operator sees the
  // shooter spinning up while localization is still converging.
  node.SetMechanismChannels({{"shooter/flywheel", 42.0, "rev/s"}});
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
  EXPECT_NEAR(channels.at("mechanism/stale"), 0.0, 1e-12);
  EXPECT_EQ(channels.count("odometry/vx"), 0u);
  // Chassis is zeros by contract until pose/valid reads 1; the field view
  // must gate drawing on that flag rather than on the struct's presence.
  ASSERT_NE(frame->chassis(), nullptr);
  EXPECT_EQ(frame->chassis()->x(), 0.0);
  EXPECT_EQ(frame->chassis()->y(), 0.0);
  EXPECT_EQ(frame->chassis()->yaw(), 0.0);
}

TEST(Telemetry, PoseValidTracksFreshPose) {
  const auto topic = TestTopic("posevalid");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  TelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  node.SetMechanismChannels({{"shooter/flywheel", 42.0, "rev/s"}});
  loop.run_for(20ms);
  ASSERT_GT(node.frames_published(), 0u);
  const auto first_slot = consumer.Drain();
  ASSERT_TRUE(first_slot.has_value());
  const auto* first_frame = Decode(*first_slot);
  ASSERT_NE(first_frame, nullptr);
  EXPECT_NEAR(Channels(first_frame).at("pose/valid"), 0.0, 1e-12);

  loop.inject(odometry::kOdometryTopic, Odometry(3.25, 1.5, 0.25));
  loop.run_for(20ms);
  const auto slot = consumer.Drain();
  ASSERT_TRUE(slot.has_value());
  const auto* frame = Decode(*slot);
  ASSERT_NE(frame, nullptr);
  EXPECT_NEAR(Channels(frame).at("pose/valid"), 1.0, 1e-12);
  EXPECT_NEAR(frame->chassis()->x(), 3.25, 1e-12);
}

TEST(Telemetry, OmitsStaleMechanismRowsButKeepsPublishingPose) {
  const auto topic = TestTopic("mechstale");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  TelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  node.SetMechanismChannels({{"shooter/flywheel", 42.0, "rev/s"}});
  loop.inject(odometry::kOdometryTopic, Odometry(1.0, 2.0, 0.0));
  loop.run_for(50ms);
  ASSERT_GT(node.frames_published(), 0u);
  const auto fresh_slot = consumer.Drain();
  ASSERT_TRUE(fresh_slot.has_value());
  const auto* fresh_frame = Decode(*fresh_slot);
  ASSERT_NE(fresh_frame, nullptr);
  EXPECT_GT(Channels(fresh_frame).count("shooter/flywheel"), 0u);

  // Both sources age out past the 100 ms horizon; then odometry alone
  // recovers. The frame must carry the live pose without the frozen
  // shooter row, and say so in-stream.
  loop.run_for(150ms);
  (void)consumer.Drain();
  loop.inject(odometry::kOdometryTopic, Odometry(1.0, 2.0, 0.0));
  loop.run_for(30ms);
  const auto slot = consumer.Drain();
  ASSERT_TRUE(slot.has_value());
  const auto* frame = Decode(*slot);
  ASSERT_NE(frame, nullptr);
  const auto channels = Channels(frame);
  EXPECT_EQ(channels.count("shooter/flywheel"), 0u);
  EXPECT_NEAR(channels.at("mechanism/stale"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("pose/valid"), 1.0, 1e-12);
  EXPECT_NEAR(channels.at("odometry/vx"), 1.5, 1e-12);
}

TEST(Telemetry, MarksStaleOdometryAndHoldsFieldView) {
  const auto topic = TestTopic("odomstale");
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  TelemetryNode node{loop, TestConfig(topic)};
  Consumer consumer{topic};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  loop.inject(odometry::kOdometryTopic, Odometry(1.0, 2.0, 0.0));
  loop.run_for(50ms);
  ASSERT_GT(node.frames_published(), 0u);
  (void)consumer.Drain();

  // Odometry ages out; a fresh mechanism push keeps frames flowing, but the
  // field view must wait: pose/valid drops, odometry rows go missing, and
  // chassis holds zeros rather than a frozen pose drawn as live.
  loop.run_for(150ms);
  node.SetMechanismChannels({{"shooter/flywheel", 42.0, "rev/s"}});
  loop.run_for(30ms);
  const auto slot = consumer.Drain();
  ASSERT_TRUE(slot.has_value());
  const auto* frame = Decode(*slot);
  ASSERT_NE(frame, nullptr);
  const auto channels = Channels(frame);
  EXPECT_NEAR(channels.at("pose/valid"), 0.0, 1e-12);
  EXPECT_NEAR(channels.at("odometry/stale"), 1.0, 1e-12);
  EXPECT_EQ(channels.count("odometry/vx"), 0u);
  EXPECT_NEAR(channels.at("shooter/flywheel"), 42.0, 1e-12);
  EXPECT_EQ(frame->chassis()->x(), 0.0);
  EXPECT_EQ(frame->chassis()->y(), 0.0);
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
