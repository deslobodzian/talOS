#include "talOS/devices/target.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "talOS/devices/devices.h"
#include "talOS/devices/testing.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/node/access.h"
#include "talOS/node/sim_access.h"
#include "talOS/node/testing.h"
#include "target_test/target_test_message_generated.h"

namespace talos {
namespace {

using namespace std::chrono_literals;
using testing::DriverStationGeneration;
using testing::DsPacket;
using testing::LatestTarget;
using testing::PublishTarget;
using testing::ReleaseTarget;
using testing::Sim;
using testing::SimHardware;
namespace ds = driver_station;

// A double-aligned message and a 4-byte one, so both alignments run
// (target_test_message.fbs).
using target_test::Goal;
using target_test::Small;
static_assert(sizeof(TargetEnvelope<Goal>) == 24);
static_assert(sizeof(TargetEnvelope<Small>) == 20);
// Neither handle exists without its node: get() and send() always have one.
static_assert(!std::is_default_constructible_v<Target<Goal>>);
static_assert(!std::is_default_constructible_v<TargetLane<Goal>>);

// The lift lets auto-align preempt the driver in teleop (robot default); the
// wrist overrides teleop and opts into test mode.
constexpr const char* kRobotToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[robot.targets]
teleop = ["align", "teleop"]

[subsystems.lift.motors.m]
can_id = 1

[subsystems.wrist.targets]
teleop = ["driver"]
test = ["pit"]
[subsystems.wrist.motors.w]
can_id = 2
)";

// What a consumer saw on its last update().
template <class M>
struct Consumer {
  explicit Consumer(Node& node) : target{node} {}
  void update() {
    const M* t = target.get();
    seen = t != nullptr ? std::optional<M>{*t} : std::nullopt;
    source = std::string{target.source()};
  }
  Target<M> target;
  std::optional<M> seen;
  std::string source;
};

constexpr std::uint32_t kTeleop = ds::kEnabled | ds::kTeleop;
constexpr std::uint32_t kAuto = ds::kEnabled | ds::kAutonomous;

class TargetTest : public ::testing::Test {
 protected:
  TargetTest() {
    hw.enable();
    Mode(kTeleop);
  }
  // SimHardware forwards the packet every robot period from now on.
  void Mode(std::uint32_t flags) {
    hw.set_driver_station(DsPacket(flags));
    sim.step(10ms);
  }
  // A producer's lane, republished every 5 ms for `duration`.
  void Hold(const char* source, double position, std::chrono::milliseconds d) {
    for (auto t = 0ms; t < d; t += 5ms) {
      PublishTarget(sim, "lift", source, Goal{position});
      sim.step(5ms);
    }
  }
  // Disabled, then teleop again: the packet that re-enables lands just
  // after the lift's tick, so it waits for the next drain with whatever the
  // producers queued meanwhile.
  void ReenableBetweenTicks() {
    Mode(0);
    sim.step(1ms);  // off the 5 ms grid the packets and ticks share
    hw.set_driver_station(DsPacket(kTeleop));
    sim.step(4ms);
  }
  std::optional<double> Seen() const {
    return lift.seen ? std::optional{lift.seen->position_rot()} : std::nullopt;
  }

  Sim sim = Sim::FromToml(kRobotToml);
  Consumer<Goal>& lift = sim.add<Consumer<Goal>>("lift");
  SimHardware hw{sim};
};

TEST_F(TargetTest, NeverSentIsNull) {
  sim.step(50ms);
  EXPECT_EQ(Seen(), std::nullopt);
  EXPECT_EQ(lift.source, "");
}

TEST_F(TargetTest, HigherPriorityLaneWinsAndReleaseFallsThroughInOneTick) {
  PublishTarget(sim, "lift", "teleop", Goal{1.0});
  PublishTarget(sim, "lift", "align", Goal{2.0});
  sim.step(5ms);
  EXPECT_EQ(Seen(), 2.0);
  EXPECT_EQ(lift.source, "align");

  // Released, not stale: the driver gets control back on the next tick, not
  // a command timeout later.
  ReleaseTarget<Goal>(sim, "lift", "align");
  sim.step(5ms);
  EXPECT_EQ(Seen(), 1.0);
  EXPECT_EQ(lift.source, "teleop");

  ReleaseTarget<Goal>(sim, "lift", "teleop");
  sim.step(5ms);
  EXPECT_EQ(Seen(), std::nullopt);
}

TEST_F(TargetTest, StaleLaneIsNull) {
  Hold("teleop", 1.0, 20ms);
  ASSERT_EQ(Seen(), 1.0);
  sim.step(90ms);
  EXPECT_EQ(Seen(), 1.0);  // still younger than command_timeout_us
  sim.step(10ms);          // 100 ms since it arrived
  EXPECT_EQ(Seen(), std::nullopt);
}

struct PausableProducer {
  explicit PausableProducer(Node& node) : lane{node, "lift", "teleop"} {}
  void update() { lane.send(Goal{0.5}); }
  TargetLane<Goal> lane;
};

struct ActuatingConsumer {
  explicit ActuatingConsumer(Node& node) : target{node}, motor{node, "m"} {}
  void update() {
    if (const auto* goal = target.get())
      motor.set_duty_cycle(goal->position_rot());
    ++updates;
  }
  Target<Goal> target;
  Motor motor;
  int updates = 0;
};

TEST(Target,
     PausedProducerExpiresToNeutralWhileConsumerAndHardwareKeepRunning) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<PausableProducer>("wrist");
  sim.add<ActuatingConsumer>("lift");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(20ms);
  ASSERT_EQ(hw.motor_request("lift", "m").mode, hardware::Mode::kDutyCycle);
  sim.pause("wrist", 200ms);
  sim.step(105ms);
  EXPECT_TRUE(robot_enabled(sim.node("lift")));
  EXPECT_EQ(hw.motor_request("lift", "m").mode, hardware::Mode::kNeutral);
  sim.resume("wrist");
  sim.step(5ms);
  EXPECT_EQ(hw.motor_request("lift", "m").mode, hardware::Mode::kDutyCycle);
  hw.enable(false);
  sim.step(10ms);
  EXPECT_EQ(hw.motor_request("lift", "m").mode, hardware::Mode::kNeutral);
}

// REQ-SAFE-31: the first request after a gap, as after a restart, is neutral
// on a lane that expired before the consumer drained it
TEST(Target, PausedConsumerNeverActuatesOnExpiredQueuedTargetAfterResume) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& lift = sim.add<ActuatingConsumer>("lift");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(20ms);
  const int updates = lift.updates;
  sim.pause("lift", 125ms);
  PublishTarget(sim, "lift", "teleop", Goal{0.5});
  sim.step(124ms);
  EXPECT_EQ(lift.updates, updates);
  sim.step(1ms);
  EXPECT_EQ(lift.updates, updates + 1);
  EXPECT_TRUE(robot_enabled(sim.node("lift")));
  EXPECT_EQ(lift.target.get(), nullptr);
  EXPECT_EQ(hw.motor_request("lift", "m").mode, hardware::Mode::kNeutral);
}

// Each update() that selects a lane logs one READ record of that lane's
// message (its publish time), the dispatch-log evidence of the target age a
// tick acted on; a stale or released lane, or a disabled robot, selects
// nothing and logs nothing.
// Asks its Target twice per update(), as a node that also reports source().
struct RepeatedlyAskingConsumer {
  explicit RepeatedlyAskingConsumer(Node& node)
      : target{node}, motor{node, "m"} {}
  void update() {
    for (int i = 0; i < 2; ++i) {
      if (const auto* goal = target.get())
        motor.set_duty_cycle(goal->position_rot());
      static_cast<void>(target.source());
    }
    ++updates;
  }
  Target<Goal> target;
  Motor motor;
  int updates = 0;
};

TEST(Target, EachSelectingUpdateLogsOneReadOfTheLane) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& lift = sim.add<RepeatedlyAskingConsumer>("lift");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(12ms);  // off the tick grid
  const auto reads = [&] { return detail::SimAccess::reads(sim, "lift"); };
  ASSERT_EQ(reads().count, 0U);
  // 20 ms of updates that log no READ.
  const auto expect_no_reads = [&](const char* why) {
    const std::uint64_t count = reads().count;
    const int updates = lift.updates;
    sim.step(20ms);
    EXPECT_GT(lift.updates, updates) << why;
    EXPECT_EQ(reads().count, count) << why;
  };

  const Time published = sim.now();
  PublishTarget(sim, "lift", "teleop", Goal{0.5});
  int updates = lift.updates;
  sim.step(15ms);
  ASSERT_EQ(hw.motor_request("lift", "m").mode, hardware::Mode::kDutyCycle);
  EXPECT_GT(lift.updates, updates);
  EXPECT_EQ(reads().count, static_cast<std::uint64_t>(lift.updates - updates));
  EXPECT_EQ(reads().topic, "/lift/target/teleop");
  EXPECT_EQ(reads().publish_time, published);

  sim.step(100ms);  // the lane passes command_timeout_us
  expect_no_reads("stale lane");

  PublishTarget(sim, "lift", "teleop", Goal{0.5});
  sim.step(5ms);
  const std::uint64_t fresh = reads().count;
  ReleaseTarget<Goal>(sim, "lift", "teleop");
  sim.step(5ms);
  EXPECT_GT(fresh, 0U);
  expect_no_reads("released lane");

  PublishTarget(sim, "lift", "teleop", Goal{0.5});
  updates = lift.updates;
  const std::uint64_t before = reads().count;
  sim.step(5ms);
  EXPECT_EQ(reads().count - before,
            static_cast<std::uint64_t>(lift.updates - updates));
  hw.enable(false);
  sim.step(5ms);
  PublishTarget(sim, "lift", "teleop", Goal{0.5});
  expect_no_reads("disabled");
}

// REQ-SAFE-31: a node's first drain judges a lane by its publication time
TEST(Target, FirstDrainAfterCommandTimeoutDoesNotReviveAnOldTarget) {
  auto sim = Sim::FromToml(R"(
[robot]
period_us = 5000
command_timeout_us = 100000
[subsystems.lift]
period_us = 200000
[subsystems.mech.motors.m]
can_id = 1
)");
  auto& lift = sim.add<Consumer<Goal>>("lift");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(10ms);
  PublishTarget(sim, "lift", "teleop", Goal{2.0});
  sim.step(190ms);  // First consumer drain; the producer has been silent.
  EXPECT_TRUE(robot_enabled(sim.node("lift")));
  EXPECT_EQ(lift.target.get(), nullptr);
  EXPECT_FALSE(lift.seen);
  EXPECT_EQ(lift.source, "");
}

TEST_F(TargetTest, ModeSwitchIgnoresTheOldModesLane) {
  Mode(kAuto);
  Hold("auto", 3.0, 20ms);
  ASSERT_EQ(Seen(), 3.0);
  EXPECT_EQ(lift.source, "auto");

  // Teleop does not list auto, and the auto routine is still talking.
  hw.set_driver_station(DsPacket(kTeleop));
  Hold("auto", 3.0, 20ms);
  EXPECT_EQ(Seen(), std::nullopt);
}

TEST_F(TargetTest, LaneFromBeforeTheModeChangeIsIgnored) {
  Hold("teleop", 1.0, 20ms);
  ASSERT_EQ(Seen(), 1.0);
  Mode(kAuto);
  // Back to teleop within the command timeout: the teleop lane is still
  // fresh, but it was said before this teleop period began.
  Mode(kTeleop);
  EXPECT_EQ(Seen(), std::nullopt);
  Hold("teleop", 1.25, 5ms);
  EXPECT_EQ(Seen(), 1.25);
}

// REQ-SAFE-09
TEST_F(TargetTest, TargetQueuedBeforeTheModeChangeIsRejectedInTheSameDrain) {
  Hold("teleop", 1.0, 20ms);
  ASSERT_EQ(Seen(), 1.0);
  const std::uint64_t teleop = DriverStationGeneration(sim);
  ReenableBetweenTicks();
  ASSERT_NE(DriverStationGeneration(sim), teleop);
  // Said under the first teleop period and delivered in the same drain as
  // the packet that starts the second: both are received at one instant,
  // so only the generation tells them apart.
  PublishTarget(sim, "lift", "teleop", Goal{2.0}, teleop);
  sim.step(5ms);
  EXPECT_EQ(Seen(), std::nullopt);
  EXPECT_EQ(lift.source, "");
}

TEST_F(TargetTest, FlapBetweenPublishedSamplesRejectsTheOldGeneration) {
  hw.stop_driver_station();
  ds::ModeTracker tracker;
  auto publish_ds = [&] {
    auto data = DsPacket(kTeleop);
    data.sample_time_us = static_cast<uint64_t>(sim.now().nanos() / 1000);
    tracker.Stamp(data);
    hardware::Packet packet{};
    packet.size = static_cast<uint32_t>(ds::Encode(data, packet.data));
    sim.publish(hardware::kDriverStationTopic, packet);
    return data.mode_since_us;
  };

  const uint64_t before = publish_ds();
  PublishTarget(sim, "lift", "teleop", Goal{1.0}, before);
  sim.step(5ms);
  ASSERT_EQ(Seen(), 1.0);

  // No DS packet is published during these two hardware observations.
  tracker.Observe(ds::kDsAttached | ds::kTeleop,
                  static_cast<uint64_t>(sim.now().nanos() / 1000));
  sim.step(5ms);
  tracker.Observe(ds::kDsAttached | kTeleop,
                  static_cast<uint64_t>(sim.now().nanos() / 1000));
  sim.step(10ms);
  const uint64_t after = publish_ds();  // 20 ms after the previous packet
  ASSERT_GT(after, before);
  // Still fresh, but stamped before the unreported disable/re-enable.
  PublishTarget(sim, "lift", "teleop", Goal{2.0}, before);
  sim.step(5ms);
  EXPECT_EQ(Seen(), std::nullopt);

  PublishTarget(sim, "lift", "teleop", Goal{3.0}, after);
  sim.step(5ms);
  EXPECT_EQ(Seen(), 3.0);
}

// A RoboRIO reboot restarts the gateway clock: boot B's generations are low
// where boot A's were high. A lane stamped under A's generation never drives
// after the consumer has taken B's packets, not even after a delayed A packet
// arrives, and B's own generation drives only while B is enabled and fresh.
TEST_F(TargetTest, RebootedGatewayRetiresTheOldBootsGeneration) {
  hw.stop_driver_station();
  ds::ModeTracker boot_a, boot_b;
  auto publish_ds = [&](ds::ModeTracker& tracker, uint64_t boot_id,
                        uint64_t sample_time_us, uint32_t flags) {
    auto data = DsPacket(flags);
    data.boot_id = boot_id;
    data.sample_time_us = sample_time_us;
    tracker.Stamp(data);
    hardware::Packet packet{};
    packet.size = static_cast<uint32_t>(ds::Encode(data, packet.data));
    sim.publish(hardware::kDriverStationTopic, packet);
    return data.mode_since_us;
  };
  constexpr uint64_t kBootA = 0xA1, kBootB = 0xB1;

  const uint64_t a = publish_ds(boot_a, kBootA, 900'000'000, kTeleop);
  PublishTarget(sim, "lift", "teleop", Goal{1.0}, a);
  sim.step(5ms);
  ASSERT_EQ(Seen(), 1.0);

  const uint64_t b = publish_ds(boot_b, kBootB, 1000, kTeleop);
  ASSERT_LT(b, a);
  PublishTarget(sim, "lift", "teleop", Goal{2.0}, a);
  sim.step(5ms);
  EXPECT_EQ(Seen(), std::nullopt);

  // A's last packet, delayed past the reboot, cannot bring A's generation
  // back.
  publish_ds(boot_a, kBootA, 900'020'000, kTeleop);
  PublishTarget(sim, "lift", "teleop", Goal{2.5}, a);
  sim.step(5ms);
  EXPECT_EQ(Seen(), std::nullopt);

  publish_ds(boot_b, kBootB, 21000, kTeleop);
  PublishTarget(sim, "lift", "teleop", Goal{3.0}, b);
  sim.step(5ms);
  EXPECT_EQ(Seen(), 3.0);

  // Disabled in boot B: neutral, whatever generation the lane carries.
  const uint64_t disabled = publish_ds(boot_b, kBootB, 41000, 0);
  PublishTarget(sim, "lift", "teleop", Goal{3.5}, b);
  sim.step(5ms);
  EXPECT_EQ(Seen(), std::nullopt);
  PublishTarget(sim, "lift", "teleop", Goal{3.5}, disabled);
  sim.step(5ms);
  EXPECT_EQ(Seen(), std::nullopt);

  // Enabled again, then silent past command_timeout_us: stale is neutral.
  const uint64_t again = publish_ds(boot_b, kBootB, 61000, kTeleop);
  PublishTarget(sim, "lift", "teleop", Goal{4.0}, again);
  sim.step(5ms);
  ASSERT_EQ(Seen(), 4.0);
  for (int i = 0; i < 30; ++i) {
    PublishTarget(sim, "lift", "teleop", Goal{4.0}, again);
    sim.step(5ms);
  }
  EXPECT_EQ(Seen(), std::nullopt);
}

TEST_F(TargetTest, TargetOfTheNewGenerationIsAcceptedInTheSameDrain) {
  ReenableBetweenTicks();
  // The operator interface derived this target from the re-enabling packet;
  // it lands in the same drain.
  PublishTarget(sim, "lift", "teleop", Goal{0.5});
  sim.step(5ms);
  EXPECT_EQ(Seen(), 0.5);
}

TEST_F(TargetTest, TargetFromAProducerWithNoModeIsRejected) {
  // A producer whose Driver Station view is stale or disabled stamps 0.
  PublishTarget(sim, "lift", "teleop", Goal{1.0}, 0);
  sim.step(5ms);
  EXPECT_EQ(Seen(), std::nullopt);
}

TEST_F(TargetTest, RobotDisabledIsNull) {
  Hold("teleop", 1.0, 20ms);
  ASSERT_EQ(Seen(), 1.0);
  hw.enable(false);
  Hold("teleop", 1.0, 20ms);
  EXPECT_EQ(Seen(), std::nullopt);
  EXPECT_EQ(lift.source, "");
}

TEST_F(TargetTest, DriverStationLossIsNull) {
  Hold("teleop", 1.0, 20ms);
  ASSERT_EQ(Seen(), 1.0);
  hw.stop_driver_station();
  Hold("teleop", 1.0, 110ms);  // the producer keeps talking
  EXPECT_EQ(Seen(), std::nullopt);
}

TEST_F(TargetTest, SameModeRecoveryKeepsTheGeneration) {
  Hold("teleop", 1.0, 20ms);
  const std::uint64_t teleop = DriverStationGeneration(sim);
  hw.stop_driver_station();
  sim.step(150ms);
  // While its view was stale the producer stamped 0.
  PublishTarget(sim, "lift", "teleop", Goal{3.0}, 0);
  // The link comes back with the Rio still enabled in teleop: the same
  // generation, so the producer drives again as soon as it resends.
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(10ms);  // the lift has drained the returning packet
  EXPECT_EQ(DriverStationGeneration(sim), teleop);
  EXPECT_EQ(Seen(), std::nullopt);
  Hold("teleop", 1.5, 5ms);
  EXPECT_EQ(Seen(), 1.5);
}

TEST_F(TargetTest, ReattachStartsANewGenerationTheOldLaneCannotDrive) {
  Hold("teleop", 1.0, 20ms);
  const std::uint64_t teleop = DriverStationGeneration(sim);
  ds::DriverStationData detached{};
  detached.fms.flags = kTeleop;  // enabled bits latched, DS not attached
  hw.set_driver_station(detached);
  sim.step(10ms);
  EXPECT_EQ(Seen(), std::nullopt);
  Mode(kTeleop);
  ASSERT_NE(DriverStationGeneration(sim), teleop);
  // A producer that has not drained the re-attach yet resends the old word.
  PublishTarget(sim, "lift", "teleop", Goal{1.0}, teleop);
  sim.step(5ms);
  EXPECT_EQ(Seen(), std::nullopt);
  Hold("teleop", 1.25, 5ms);
  EXPECT_EQ(Seen(), 1.25);
}

TEST_F(TargetTest, StaleHigherPriorityLaneFallsBackToTheNext) {
  PublishTarget(sim, "lift", "align", Goal{2.0});
  Hold("teleop", 1.0, 50ms);
  EXPECT_EQ(Seen(), 2.0);
  EXPECT_EQ(lift.source, "align");
  // Auto-align stopped talking without a release; the driver gets control
  // back once its lane is older than command_timeout_us.
  Hold("teleop", 1.0, 60ms);
  EXPECT_EQ(Seen(), 1.0);
  EXPECT_EQ(lift.source, "teleop");
}

TEST_F(TargetTest, DisabledOrEStoppedDriverStationIsNull) {
  Mode(ds::kTeleop);  // teleop, not enabled
  Hold("teleop", 1.0, 20ms);
  EXPECT_EQ(Seen(), std::nullopt);
  Mode(kTeleop | ds::kEStop);
  Hold("teleop", 1.0, 20ms);
  EXPECT_EQ(Seen(), std::nullopt);
}

TEST_F(TargetTest, TestModeDrivesNothingByDefault) {
  Mode(ds::kEnabled | ds::kTest);
  Hold("teleop", 1.0, 20ms);
  EXPECT_EQ(Seen(), std::nullopt);
}

TEST(TargetPolicy, SubsystemTargetsReplaceTheListedModes) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& wrist = sim.add<Consumer<Small>>("wrist");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(10ms);

  // Teleop is the wrist's own list; the robot's align and teleop lanes are
  // not even subscribed.
  PublishTarget(sim, "wrist", "teleop", Small{9.0f});
  sim.step(5ms);
  EXPECT_EQ(wrist.seen, std::nullopt);
  PublishTarget(sim, "wrist", "driver", Small{2.5f});
  sim.step(5ms);
  if (!wrist.seen.has_value()) {
    FAIL() << "Expected a wrist target";
  }
  const auto& wrist_target = *wrist.seen;
  EXPECT_FLOAT_EQ(wrist_target.speed_rps(), 2.5f);

  // Test mode opted in; autonomous kept the robot default.
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTest));
  sim.step(10ms);
  PublishTarget(sim, "wrist", "pit", Small{1.0f});
  sim.step(5ms);
  ASSERT_TRUE(wrist.seen.has_value());
  EXPECT_EQ(wrist.source, "pit");
  hw.set_driver_station(DsPacket(kAuto));
  sim.step(10ms);
  PublishTarget(sim, "wrist", "auto", Small{4.0f});
  sim.step(5ms);
  EXPECT_EQ(wrist.source, "auto");
}

// The lift's teleop lane comes from a topic of the config's choosing; its
// align lane, and every other node's lanes, stay where they were. The
// blind producer's Driver Station is remapped onto a topic nothing writes.
constexpr const char* kRemapToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[robot.targets]
teleop = ["align", "teleop"]

[subsystems.lift.motors.m]
can_id = 1
[subsystems.lift.remap]
"target/teleop" = "/left_lift/target/teleop"

[subsystems.blind_producer.remap]
"/hw/state/driver_station" = "/hw/state/driver_station_unplugged"
)";

TEST(TargetPolicy, RemapRedirectsOneLane) {
  auto sim = Sim::FromToml(kRemapToml);
  auto& lift = sim.add<Consumer<Goal>>("lift");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(10ms);

  PublishTarget(sim, "lift", "teleop", Goal{1.0});
  sim.step(5ms);
  EXPECT_EQ(lift.seen, std::nullopt);  // not the lift's teleop lane any more
  PublishTarget(sim, "left_lift", "teleop", Goal{2.0});
  sim.step(5ms);
  if (!lift.seen.has_value()) {
    FAIL() << "Expected a remapped lift target";
  }
  const auto& remapped_target = *lift.seen;
  EXPECT_EQ(remapped_target.position_rot(), 2.0);
  EXPECT_EQ(lift.source, "teleop");
  // Still the second priority: align, not remapped, preempts it.
  PublishTarget(sim, "lift", "align", Goal{3.0});
  PublishTarget(sim, "left_lift", "teleop", Goal{2.0});
  sim.step(5ms);
  EXPECT_EQ(lift.source, "align");
}

// Who drives, as the node's registry row says it: only get() runs, so the
// report cannot depend on anyone asking for source().
struct Driven {
  explicit Driven(Node& n) : node{n}, target{n} {}
  void update() { target.get(); }
  std::string driver() const {
    const std::int32_t index = detail::NodeAccess::selected(node, 0);
    if (index < 0) return "";
    return std::string{detail::NodeAccess::selection_sources(
        node, 0)[static_cast<std::size_t>(index)]};
  }
  Node& node;
  Target<Goal> target;
};

TEST(TargetSelection, TheNodeReportsWhoDrivesAndNobodyWhenStaleOrDisabled) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& lift = sim.add<Driven>("lift");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(10ms);

  ASSERT_EQ(detail::NodeAccess::selection_count(lift.node), 1u);
  const auto sources = detail::NodeAccess::selection_sources(lift.node, 0);
  EXPECT_EQ((std::vector<std::string>{sources.begin(), sources.end()}),
            (std::vector<std::string>{"align", "teleop", "auto"}));
  EXPECT_EQ(lift.driver(), "");

  PublishTarget(sim, "lift", "teleop", Goal{1.0});
  sim.step(5ms);
  EXPECT_EQ(lift.driver(), "teleop");
  PublishTarget(sim, "lift", "align", Goal{2.0});
  sim.step(5ms);
  EXPECT_EQ(lift.driver(), "align");
  ReleaseTarget<Goal>(sim, "lift", "align");
  PublishTarget(sim, "lift", "teleop", Goal{1.0});
  sim.step(5ms);
  EXPECT_EQ(lift.driver(), "teleop");

  // Stale: the teleop lane outlives command_timeout_us unrepeated.
  sim.step(110ms);
  EXPECT_EQ(lift.driver(), "");

  // Disabled: the producer keeps talking, and nobody drives.
  PublishTarget(sim, "lift", "teleop", Goal{1.0});
  sim.step(5ms);
  ASSERT_EQ(lift.driver(), "teleop");
  hw.enable(false);
  for (auto t = 0ms; t < 20ms; t += 5ms) {
    PublishTarget(sim, "lift", "teleop", Goal{1.0});
    sim.step(5ms);
  }
  EXPECT_EQ(lift.driver(), "");
}

// A producer node writes the same envelopes the helpers do.
struct Producer {
  explicit Producer(Node& node) : lane{node, "lift", "teleop"} {}
  void update() {
    if (talking) {
      lane.send(Goal{0.75});
    } else {
      lane.release();
    }
  }
  TargetLane<Goal> lane;
  bool talking = true;
};

TEST(TargetLane, SendsAndReleasesTheEnvelope) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& producer = sim.add<Producer>("producer");
  auto& lift = sim.add<Consumer<Goal>>("lift");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(20ms);

  auto envelope = LatestTarget<Goal>(sim, "lift", "teleop");
  if (!envelope.has_value()) {
    FAIL() << "Expected an active target envelope";
  }
  const auto& active_envelope = *envelope;
  EXPECT_EQ(active_envelope.active, 1);
  EXPECT_EQ(active_envelope.message.position_rot(), 0.75);
  if (!lift.seen.has_value()) {
    FAIL() << "Expected a lift target";
  }
  const auto& seen_target = *lift.seen;
  EXPECT_EQ(seen_target.position_rot(), 0.75);

  producer.talking = false;
  sim.step(10ms);
  envelope = LatestTarget<Goal>(sim, "lift", "teleop");
  if (!envelope.has_value()) {
    FAIL() << "Expected a released target envelope";
  }
  const auto& released_envelope = *envelope;
  EXPECT_EQ(released_envelope.active, 0);
  EXPECT_EQ(released_envelope.message.position_rot(), 0.0);
  EXPECT_EQ(lift.seen, std::nullopt);
}

// REQ-SAFE-12: a lane stamps 0 while the Driver Station is disabled
TEST(TargetLane, StampsTheGenerationItsNodeDrained) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<Producer>("producer");
  auto& lift = sim.add<Consumer<Goal>>("lift");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(20ms);
  const std::uint64_t first = DriverStationGeneration(sim);
  ASSERT_NE(first, 0u);
  auto envelope = LatestTarget<Goal>(sim, "lift", "teleop");
  if (!envelope.has_value()) {
    FAIL() << "Expected the first target envelope";
  }
  const auto& first_envelope = *envelope;
  EXPECT_EQ(first_envelope.mode_since_us(), first);

  // Disabled: the producer keeps sending, stamped 0.
  hw.set_driver_station(DsPacket(0));
  sim.step(20ms);
  envelope = LatestTarget<Goal>(sim, "lift", "teleop");
  if (!envelope.has_value()) {
    FAIL() << "Expected a disabled target envelope";
  }
  const auto& disabled_envelope = *envelope;
  EXPECT_EQ(disabled_envelope.mode_since_us(), 0u);
  EXPECT_EQ(lift.seen, std::nullopt);

  // Re-enabled: a new generation, which the producer picks up on its own.
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(20ms);
  const std::uint64_t second = DriverStationGeneration(sim);
  EXPECT_NE(second, first);
  envelope = LatestTarget<Goal>(sim, "lift", "teleop");
  if (!envelope.has_value()) {
    FAIL() << "Expected the second target envelope";
  }
  const auto& second_envelope = *envelope;
  EXPECT_EQ(second_envelope.mode_since_us(), second);
  if (!lift.seen.has_value()) {
    FAIL() << "Expected a lift target";
  }
  const auto& seen_target = *lift.seen;
  EXPECT_EQ(seen_target.position_rot(), 0.75);
}

// A producer with its own lane, for the remapped config.
struct BlindProducer {
  explicit BlindProducer(Node& node) : lane{node, "lift", "align"} {}
  void update() { lane.send(Goal{5.0}); }
  TargetLane<Goal> lane;
};

TEST(TargetLane, AProducerThatSeesNoDriverStationNeverDrives) {
  auto sim = Sim::FromToml(kRemapToml);
  sim.add<BlindProducer>("blind_producer");
  auto& lift = sim.add<Consumer<Goal>>("lift");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(50ms);
  // The lift is enabled in teleop, and align is its first lane, fresh and
  // active; but its producer never saw the mode it would be driving.
  const auto envelope = LatestTarget<Goal>(sim, "lift", "align");
  if (!envelope.has_value()) {
    FAIL() << "Expected a target envelope without driver station state";
  }
  const auto& blind_envelope = *envelope;
  EXPECT_EQ(blind_envelope.active, 1);
  EXPECT_EQ(blind_envelope.mode_since_us(), 0u);
  EXPECT_EQ(lift.seen, std::nullopt);
  PublishTarget(sim, "left_lift", "teleop", Goal{2.0});
  sim.step(5ms);
  EXPECT_EQ(lift.source, "teleop");
}

struct BadLane {
  explicit BadLane(Node& node) : lane{node, "lift", "Auto"} {}
  TargetLane<Goal> lane;
};

TEST(TargetLane, RejectsASourceThatIsNotATopicSegment) {
  auto sim = Sim::FromToml(kRobotToml);
  EXPECT_THROW(sim.add<BadLane>("producer"), std::invalid_argument);
}

// An operator producer that sends whenever joystick 0's button 1 is held and
// ignores what evaluate() says: its arming alone holds the lane released.
// Its 20 ms period samples the Driver Station less often than the 5 ms
// packets arrive.
struct Operator {
  explicit Operator(Node& node)
      : ds{node}, arming{node}, lane{node, "lift", "teleop", arming} {}
  void update() {
    if (!ds.connected()) saw_stale = true;
    const bool held = ds.joystick(0).button(1);
    if (!paused) arming.evaluate(/*neutral=*/!held);
    if (held) {
      lane.send(Goal{0.5});
    } else {
      lane.release();
    }
  }
  DriverStation ds;
  OperatorArming arming;
  TargetLane<Goal> lane;
  bool paused = false;
  bool saw_stale = false;
};

// A button held through any break in what the operator saw must be released
// once, in the new view, before the lane drives again.
// REQ-SAFE-31: started with the input held, it never arms
// REQ-SAFE-13: a held input is released once after each break, then re-arms
TEST(OperatorArming, HeldInputWaitsForNeutralAfterEveryBreak) {
  enum class Break { kDisable, kReboot, kEvaluationGap, kUnsampledRecovery };
  for (const Break disturbance :
       {Break::kDisable, Break::kReboot, Break::kEvaluationGap,
        Break::kUnsampledRecovery}) {
    SCOPED_TRACE(static_cast<int>(disturbance));
    auto sim = Sim::FromToml(std::string{kRobotToml} +
                             "[subsystems.operator]\nperiod_us = 20000\n");
    auto& op = sim.add<Operator>("operator");
    auto& lift = sim.add<Consumer<Goal>>("lift");
    SimHardware hw{sim};
    hw.enable();
    const auto packet = [](std::uint32_t flags, bool held) {
      auto data = DsPacket(flags);
      data.joysticks[0].connected = true;
      data.joysticks[0].button_count = 1;
      data.joysticks[0].buttons = held ? 1u : 0u;
      return data;
    };
    const auto active = [&] {
      const auto envelope = LatestTarget<Goal>(sim, "lift", "teleop");
      return envelope.has_value() && envelope->active != 0;
    };

    // Held from startup: never armed.
    hw.set_driver_station(packet(kTeleop, true));
    sim.step(60ms);
    EXPECT_FALSE(active());
    EXPECT_EQ(lift.seen, std::nullopt);
    hw.set_driver_station(packet(kTeleop, false));
    sim.step(40ms);
    hw.set_driver_station(packet(kTeleop, true));
    sim.step(40ms);
    ASSERT_TRUE(active());
    ASSERT_NE(lift.seen, std::nullopt);

    switch (disturbance) {
      case Break::kDisable:
        hw.set_driver_station(packet(0, true));
        sim.step(40ms);
        hw.set_driver_station(packet(kTeleop, true));
        break;
      case Break::kReboot:
        hw.reboot();
        break;
      case Break::kEvaluationGap:  // the Driver Station stays fresh
        op.paused = true;
        sim.step(120ms);
        op.paused = false;
        break;
      case Break::kUnsampledRecovery: {
        // Silent just past command_timeout_us, stale only between two of
        // the operator's 20 ms evaluations (the last packet near 150 ms and
        // the next near 255 ms, evaluations at 240 and 260 ms): the same
        // generation and boot come back.
        const std::uint64_t generation = DriverStationGeneration(sim);
        sim.step(10ms);
        op.saw_stale = false;
        hw.stop_driver_station();
        sim.step(102ms);
        hw.set_driver_station(packet(kTeleop, true));
        sim.step(20ms);
        ASSERT_FALSE(op.saw_stale);
        ASSERT_EQ(DriverStationGeneration(sim), generation);
        break;
      }
    }
    sim.step(60ms);
    EXPECT_FALSE(active());
    EXPECT_EQ(lift.seen, std::nullopt);

    hw.set_driver_station(packet(kTeleop, false));
    sim.step(40ms);
    hw.set_driver_station(packet(kTeleop, true));
    sim.step(40ms);
    EXPECT_TRUE(active());
    EXPECT_NE(lift.seen, std::nullopt);
  }
}

// The live-target gate at the devices flush (RULES.md 1): a tick drives only
// when one of the node's Targets returned a target from a get() made during
// that update(). Every node here owns a motor, a digital output and a PWM
// output and sets them in update(); the robot is enabled in teleop.
constexpr const char* kGateToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.gate.motors.m]
can_id = 1
[subsystems.gate.digital_outputs.led]
dio = 0
[subsystems.gate.pwm_outputs.servo]
channel = 0
)";

// The gate node's outputs as its last request carried them.
struct GateOutputs {
  hardware::Mode mode;
  bool led;
  std::optional<double> servo;  // nullopt: no pulse
};
GateOutputs Requested(const SimHardware& hw) {
  return {hw.motor_request("gate", "m").mode, hw.digital_output("gate", "led"),
          hw.pwm("gate", "servo")};
}
void ExpectNeutral(const SimHardware& hw) {
  const GateOutputs out = Requested(hw);
  EXPECT_EQ(out.mode, hardware::Mode::kNeutral);
  EXPECT_FALSE(out.led);
  EXPECT_EQ(out.servo, std::nullopt);
}
void ExpectDriving(const SimHardware& hw, double value) {
  const GateOutputs out = Requested(hw);
  EXPECT_EQ(out.mode, hardware::Mode::kDutyCycle);
  EXPECT_DOUBLE_EQ(hw.motor_request("gate", "m").demand, value);
  EXPECT_TRUE(out.led);
  EXPECT_EQ(out.servo, value);
}

std::uint64_t GateRequests(const Sim& sim) {
  return detail::SimAccess::message_count(sim, "/hw/request/gate",
                                          sizeof(hardware::Packet));
}

// Every output from one value: what a node drives from its target.
struct GateDevices {
  explicit GateDevices(Node& n)
      : motor{n, "m"}, led{n, "led"}, servo{n, "servo"} {}
  void Drive(double value) {
    motor.set_duty_cycle(value);
    led.set(true);
    servo.set(value);
  }
  Motor motor;
  DigitalOutput led;
  PwmOutput servo;
};

// A joystick-direct node: it reads the Driver Station and has no Target.
struct StickDirect {
  explicit StickDirect(Node& n)
      : ds{n}, motor{n, "m"}, led{n, "led"}, servo{n, "servo"} {}
  void update() {
    const Joystick stick = ds.joystick(0);
    motor.set_voltage(12.0 * stick.axis(0));
    led.set(stick.button(1));
    servo.set(stick.axis(0));
  }
  DriverStation ds;
  Motor motor;
  DigitalOutput led;
  PwmOutput servo;
};

// REQ-SAFE-41
TEST(TargetGate, OutputsSetWithoutATargetAreSentNeutral) {
  auto sim = Sim::FromToml(kGateToml);
  sim.add<StickDirect>("gate");
  SimHardware hw{sim};
  auto held = DsPacket(kTeleop);
  held.joysticks[0].connected = true;
  held.joysticks[0].axis_count = 1;
  held.joysticks[0].axes = {0.5f};
  held.joysticks[0].button_count = 1;
  held.joysticks[0].buttons = 1;
  hw.set_driver_station(held);
  ::testing::internal::CaptureStderr();
  hw.enable();
  sim.step(20ms);
  ASSERT_TRUE(robot_enabled(sim.node("gate")));
  ExpectNeutral(hw);
  // The request still goes out every tick.
  const std::uint64_t sent = GateRequests(sim);
  sim.step(20ms);
  EXPECT_EQ(GateRequests(sim), sent + 4);
  ExpectNeutral(hw);

  // One line per enabled spell: this one, and the next after a disable.
  hw.enable(false);
  sim.step(20ms);
  hw.enable();
  sim.step(20ms);
  ExpectNeutral(hw);
  const std::string err = ::testing::internal::GetCapturedStderr();
  std::size_t lines = 0;
  for (std::size_t at = err.find("gate: "); at != std::string::npos;
       at = err.find("gate: ", at + 1)) {
    ++lines;
  }
  EXPECT_EQ(lines, 2u) << err;
}

// What a Target-driven node does: drive only from what get() returned. It
// has a second Target it never asks: any one of the node's Targets
// returning a target lets the tick drive.
struct Targeted {
  explicit Targeted(Node& n) : unasked{n}, target{n}, devices{n} {}
  void update() {
    if (const Goal* goal = target.get()) devices.Drive(goal->position_rot());
  }
  Target<Goal> unasked;
  Target<Goal> target;
  GateDevices devices;
};

// REQ-SAFE-41
TEST(TargetGate, ALiveTargetDrivesEveryOutputItsTickSet) {
  auto sim = Sim::FromToml(kGateToml);
  sim.add<Targeted>("gate");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(10ms);
  for (auto t = 0ms; t < 20ms; t += 5ms) {
    PublishTarget(sim, "gate", "teleop", Goal{0.5});
    sim.step(5ms);
  }
  ExpectDriving(hw, 0.5);
}

// Keeps driving from the last target it got, whatever get() says now.
struct Latching {
  explicit Latching(Node& n) : target{n}, devices{n} {}
  void update() {
    if (const Goal* goal = target.get()) latched = *goal;
    if (latched) devices.Drive(latched->position_rot());
  }
  Target<Goal> target;
  GateDevices devices;
  std::optional<Goal> latched;
};

// REQ-SAFE-41
TEST(TargetGate, ATickWithoutALiveTargetIsNeutralWhateverUpdateSet) {
  enum class Loss { kReleased, kStale, kEarlierGeneration };
  for (const Loss loss :
       {Loss::kReleased, Loss::kStale, Loss::kEarlierGeneration}) {
    SCOPED_TRACE(static_cast<int>(loss));
    auto sim = Sim::FromToml(kGateToml);
    auto& gate = sim.add<Latching>("gate");
    SimHardware hw{sim};
    hw.enable();
    hw.set_driver_station(DsPacket(kTeleop));
    sim.step(10ms);
    for (auto t = 0ms; t < 20ms; t += 5ms) {
      PublishTarget(sim, "gate", "teleop", Goal{0.5});
      sim.step(5ms);
    }
    ExpectDriving(hw, 0.5);
    switch (loss) {
      case Loss::kReleased:
        ReleaseTarget<Goal>(sim, "gate", "teleop");
        sim.step(5ms);  // the next tick
        break;
      case Loss::kStale:
        // The last publication is 5 ms old: at 95 ms it still drives, and
        // the first tick at an age of command_timeout_us does not.
        sim.step(90ms);
        ExpectDriving(hw, 0.5);
        sim.step(5ms);
        break;
      case Loss::kEarlierGeneration: {
        const std::uint64_t before = DriverStationGeneration(sim);
        hw.set_driver_station(DsPacket(kAuto));
        sim.step(10ms);
        ASSERT_NE(DriverStationGeneration(sim), before);
        PublishTarget(sim, "gate", "auto", Goal{0.5}, before);
        sim.step(5ms);  // the drain that takes it
        break;
      }
    }
    ASSERT_TRUE(gate.latched.has_value());
    EXPECT_TRUE(robot_enabled(sim.node("gate")));
    ExpectNeutral(hw);
  }
}

// Asks its Target only where the test says: from every(), or in update() on
// some ticks; it drives every tick from what was seen last.
struct Asking {
  explicit Asking(Node& n) : target{n}, devices{n} {
    n.every(5ms, [this] {
      if (ask_in_every && target.get() != nullptr) seen_in_every = true;
    });
  }
  void update() {
    asked = ask_in_update && (tick++ % 2 == 0);
    if (asked && target.get() != nullptr) seen = true;
    if (seen || seen_in_every) devices.Drive(0.5);
  }
  Target<Goal> target;
  GateDevices devices;
  bool ask_in_every = false;
  bool ask_in_update = false;
  bool seen = false;
  bool seen_in_every = false;
  bool asked = false;
  int tick = 0;
};

// REQ-SAFE-41
TEST(TargetGate, OnlyAGetInsideThisUpdateCounts) {
  auto sim = Sim::FromToml(kGateToml);
  auto& gate = sim.add<Asking>("gate");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(10ms);
  const auto hold = [&](std::chrono::milliseconds d) {
    for (auto t = 0ms; t < d; t += 5ms) {
      PublishTarget(sim, "gate", "teleop", Goal{0.5});
      sim.step(5ms);
    }
  };

  // A live get() in every() alone leaves every tick neutral.
  gate.ask_in_every = true;
  hold(20ms);
  ASSERT_TRUE(gate.seen_in_every);
  ExpectNeutral(hw);

  // Asked on alternate ticks: each tick drives exactly when it asked.
  gate.ask_in_every = false;
  gate.seen_in_every = false;
  gate.ask_in_update = true;
  hold(10ms);
  ASSERT_TRUE(gate.seen);
  for (int i = 0; i < 6; ++i) {
    SCOPED_TRACE(i);
    hold(5ms);
    const auto packet = sim.latest<hardware::Packet>("/hw/request/gate");
    ASSERT_TRUE(packet.has_value());
    hardware::Command command{};
    ASSERT_TRUE(hardware::Decode(packet->bytes(), command));
    EXPECT_EQ(command.motors[0].mode, gate.asked ? hardware::Mode::kDutyCycle
                                                 : hardware::Mode::kNeutral);
  }
}

// Drives from a live target each tick until told to throw: that update()
// asks its Target, sets every output from the target, then throws. The
// next update() asks and sets only as `ask` and `set` say.
struct Throwing {
  explicit Throwing(Node& n) : target{n}, devices{n} {}
  void update() {
    ++updates;
    if (throw_now) {
      throw_now = false;
      if (target.get() != nullptr) devices.Drive(0.5);
      throw std::runtime_error{"gate: update() failed"};
    }
    const bool live = ask && target.get() != nullptr;
    if (set && (live || !ask)) devices.Drive(0.5);
  }
  Target<Goal> target;
  GateDevices devices;
  bool throw_now = false;
  bool ask = true;
  bool set = true;
  int updates = 0;
};

// An update() that throws after its Target returned a target and it set
// every output leaves neither to the next tick's flush: the next update()
// has an index of its own, so a tick that sets outputs without asking, or
// that asks but sets nothing, is neutral. Only a harness that keeps
// stepping after the throw (testing::Sim) reaches that tick.
// REQ-SAFE-41, REQ-SAFE-03
TEST(TargetGate, AThrownUpdateLeavesNothingToTheNextTick) {
  struct Case {
    const char* label;
    bool ask;
    bool set;
  };
  for (const auto& [label, ask, set] :
       {Case{"sets without asking", false, true},
        Case{"asks and sets nothing", true, false}}) {
    SCOPED_TRACE(label);
    auto sim = Sim::FromToml(kGateToml);
    auto& gate = sim.add<Throwing>("gate");
    SimHardware hw{sim};
    hw.enable();
    hw.set_driver_station(DsPacket(kTeleop));
    sim.step(10ms);
    for (auto t = 0ms; t < 20ms; t += 5ms) {
      PublishTarget(sim, "gate", "teleop", Goal{0.5});
      sim.step(5ms);
    }
    ExpectDriving(hw, 0.5);

    gate.throw_now = true;
    PublishTarget(sim, "gate", "teleop", Goal{0.5});
    EXPECT_THROW(sim.step(5ms), std::runtime_error);
    gate.ask = ask;
    gate.set = set;
    // Up to the next update(), wherever the throw left the Sim's clock.
    const int before = gate.updates;
    for (int i = 0; i < 4 && gate.updates == before; ++i) {
      PublishTarget(sim, "gate", "teleop", Goal{0.5});
      sim.step(5ms);
    }
    ASSERT_EQ(gate.updates, before + 1) << "one update() after the throw";
    // Applied, not requested: an output the request omits keeps its last
    // requested value in SimHardware's inspection view.
    EXPECT_EQ(hw.applied_motor("gate", "m").mode, hardware::Mode::kNeutral);
    EXPECT_FALSE(hw.applied_digital_output("gate", "led"));
    EXPECT_EQ(hw.applied_pwm("gate", "servo"), std::nullopt);
  }
}

// Sets every output each tick while `set`, from its target when get()
// returned one and from a constant when it did not.
struct Insisting {
  explicit Insisting(Node& n) : node{n}, target{n}, devices{n} {}
  void update() {
    const Goal* goal = target.get();
    if (set) devices.Drive(goal != nullptr ? goal->position_rot() : 0.25);
  }
  Node& node;
  Target<Goal> target;
  GateDevices devices;
  bool set = true;
};

// What Studio's system view shows of the gate (the registry row's
// kFlagOutputsHeld): held while the flush sends neutral what update() set
// with no live target, and cleared by a tick that drives, a tick that sets
// nothing, and a disable. A disabled robot holds nothing back.
// REQ-SAFE-41
TEST(TargetGate, TheNodeReportsOutputsHeldOnlyWhileTheFlushHoldsThem) {
  auto sim = Sim::FromToml(kGateToml);
  auto& gate = sim.add<Insisting>("gate");
  SimHardware hw{sim};
  hw.set_driver_station(DsPacket(kTeleop));
  const auto held = [&gate] {
    return detail::NodeAccess::outputs_held(gate.node);
  };
  sim.step(10ms);
  ASSERT_FALSE(robot_enabled(gate.node));
  ExpectNeutral(hw);
  EXPECT_FALSE(held());

  hw.enable();
  sim.step(10ms);
  ASSERT_TRUE(robot_enabled(gate.node));
  ExpectNeutral(hw);
  EXPECT_TRUE(held());

  PublishTarget(sim, "gate", "teleop", Goal{0.5});
  sim.step(5ms);
  ExpectDriving(hw, 0.5);
  EXPECT_FALSE(held());

  ReleaseTarget<Goal>(sim, "gate", "teleop");
  sim.step(5ms);
  ExpectNeutral(hw);
  EXPECT_TRUE(held());

  gate.set = false;
  sim.step(5ms);
  EXPECT_FALSE(held());
  gate.set = true;
  sim.step(5ms);
  EXPECT_TRUE(held());

  hw.enable(false);
  sim.step(20ms);
  ASSERT_FALSE(robot_enabled(gate.node));
  EXPECT_FALSE(held());
}

// A PWM output set to 0 is the 1.5 ms centre pulse, which moves a servo to
// centre: an actuation like any other. Set with no live target, the flush
// sends it as no pulse and the node reports outputs held.
// REQ-SAFE-41
TEST(TargetGate, ACentrePulseWithoutATargetIsHeld) {
  struct Centring {
    explicit Centring(Node& n) : node{n}, target{n}, servo{n, "servo"} {}
    void update() {
      if (target.get() == nullptr) servo.set(0.0);
    }
    Node& node;
    Target<Goal> target;
    PwmOutput servo;
  };
  auto sim = Sim::FromToml(kGateToml);
  auto& gate = sim.add<Centring>("gate");
  SimHardware hw{sim};
  hw.set_driver_station(DsPacket(kTeleop));
  hw.enable();
  sim.step(20ms);
  ASSERT_TRUE(robot_enabled(gate.node));
  EXPECT_EQ(hw.pwm("gate", "servo"), std::nullopt);
  EXPECT_EQ(hw.applied_pwm("gate", "servo"), std::nullopt);
  EXPECT_TRUE(detail::NodeAccess::outputs_held(gate.node));
}

TEST(TargetEnvelope, IsTheHeaderThenTheMessage) {
  EXPECT_EQ(detail::TargetEnvelopeBytes(24, 8), 40u);
  EXPECT_EQ(detail::TargetEnvelopeBytes(4, 4), 20u);
  EXPECT_EQ(detail::TargetEnvelopeBytes(1, 1), 17u);
  EXPECT_THROW(detail::TargetEnvelopeBytes(12, 8), std::invalid_argument);
  EXPECT_THROW(detail::TargetEnvelopeBytes(16, 16), std::invalid_argument);
  EXPECT_THROW(detail::TargetEnvelopeBytes(8, 3), std::invalid_argument);

  // The bytes EncodeTargetEnvelope writes are the struct's layout.
  TargetEnvelope<Small> envelope{};
  const Small message{1.5f};
  detail::EncodeTargetEnvelope(0x0102030405060708u, true, &message,
                               sizeof message, alignof(Small),
                               std::as_writable_bytes(std::span{&envelope, 1}));
  EXPECT_EQ(envelope.mode_since_us(), 0x0102030405060708u);
  EXPECT_EQ(envelope.active, 1);
  EXPECT_EQ(envelope.message.speed_rps(), 1.5f);
  const auto header = detail::DecodeTargetEnvelopeHeader(
      std::as_bytes(std::span{&envelope, 1}).data());
  EXPECT_EQ(header.mode_since_us, 0x0102030405060708u);
  EXPECT_TRUE(header.active);
}

// REQ-SAFE-31: a consumer that starts (or restarts) after a target was
// published never drives on it, though that target stays active, younger than
// command_timeout_us and stamped with the consumer's own generation: a
// subscription delivers only what is published after it attached.
TEST(Target, AConsumerStartedAfterATargetWasPublishedNeverDrivesOnIt) {
  // The generation SimHardware stamps on its first teleop packet, from a run
  // of the same setup: the target below carries it before anyone attached.
  const std::uint64_t generation = [] {
    auto sim = Sim::FromToml(kRobotToml);
    sim.add<ActuatingConsumer>("lift");
    SimHardware hw{sim};
    hw.enable();
    hw.set_driver_station(DsPacket(kTeleop));
    sim.step(10ms);
    return DriverStationGeneration(sim);
  }();
  ASSERT_NE(generation, 0U);

  auto sim = Sim::FromToml(kRobotToml);
  const Time published = sim.now();
  PublishTarget(sim, "lift", "teleop", Goal{0.5}, generation);
  auto& lift = sim.add<ActuatingConsumer>("lift");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(kTeleop));
  sim.step(10ms);
  ASSERT_EQ(DriverStationGeneration(sim), generation);
  ASSERT_TRUE(robot_enabled(sim.node("lift")));
  // Every tick while the target is young enough for Target::get.
  for (; sim.now() - published < 95ms; sim.step(5ms)) {
    EXPECT_EQ(hw.applied_motor("lift", "m").mode, hardware::Mode::kNeutral)
        << "target " << (sim.now() - published).count() / 1'000'000
        << " ms old, update " << lift.updates;
  }
  // The same envelope, published once the consumer has attached, drives it.
  PublishTarget(sim, "lift", "teleop", Goal{0.5}, generation);
  sim.step(5ms);
  EXPECT_EQ(hw.applied_motor("lift", "m").mode, hardware::Mode::kDutyCycle);
}

}  // namespace
}  // namespace talos
