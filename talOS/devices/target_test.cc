#include "talOS/devices/target.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "talOS/devices/devices.h"
#include "talOS/devices/testing.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/node/access.h"
#include "talOS/node/sim_access.h"
#include "talOS/node/testing.h"

namespace talos {
namespace {

using namespace std::chrono_literals;
using testing::DsPacket;
using testing::LatestTarget;
using testing::PublishTarget;
using testing::ReleaseTarget;
using testing::Sim;
using testing::SimHardware;
namespace ds = driver_station;

// A double-aligned message and a 4-byte one, so both envelope paddings run.
struct Goal {
  double position_rot;
};
struct Small {
  float speed_rps;
};
static_assert(sizeof(TargetEnvelope<Goal>) == 16);
static_assert(sizeof(TargetEnvelope<Small>) == 8);

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
  std::optional<double> Seen() const {
    return lift.seen ? std::optional{lift.seen->position_rot} : std::nullopt;
  }

  Sim sim = Sim::FromToml(kRobotToml);
  Consumer<Goal>& lift = sim.add<Consumer<Goal>>("lift");
  SimHardware hw{sim};
};

TEST_F(TargetTest, TeleopLaneDrivesInTeleop) {
  Hold("teleop", 1.5, 20ms);
  EXPECT_EQ(Seen(), 1.5);
  EXPECT_EQ(lift.source, "teleop");
}

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

TEST_F(TargetTest, LaneInTheSameDrainAsTheModeChangeIsAccepted) {
  // Disabled: nothing drives, whatever the lanes say.
  Mode(0);
  sim.step(1ms);  // off the 5 ms grid the packets and ticks share
  hw.set_driver_station(DsPacket(kTeleop));
  // SimHardware forwards the mode-change packet at the end of this step,
  // just after the node's tick, so it waits for the next drain.
  sim.step(4ms);
  // The operator interface derived this target from that same packet; it
  // lands in the same drain.
  PublishTarget(sim, "lift", "teleop", Goal{0.5});
  sim.step(5ms);
  EXPECT_EQ(Seen(), 0.5);
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
  ASSERT_TRUE(wrist.seen.has_value());
  EXPECT_FLOAT_EQ(wrist.seen->speed_rps, 2.5f);

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
    if (talking)
      lane.send(Goal{0.75});
    else
      lane.release();
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
  ASSERT_TRUE(envelope.has_value());
  EXPECT_EQ(envelope->active, 1);
  EXPECT_EQ(envelope->message.position_rot, 0.75);
  ASSERT_TRUE(lift.seen.has_value());
  EXPECT_EQ(lift.seen->position_rot, 0.75);

  producer.talking = false;
  sim.step(10ms);
  envelope = LatestTarget<Goal>(sim, "lift", "teleop");
  ASSERT_TRUE(envelope.has_value());
  EXPECT_EQ(envelope->active, 0);
  EXPECT_EQ(envelope->message.position_rot, 0.0);
  EXPECT_EQ(lift.seen, std::nullopt);
}

struct BadLane {
  explicit BadLane(Node& node) : lane{node, "lift", "Auto"} {}
  TargetLane<Goal> lane;
};

TEST(TargetLane, RejectsASourceThatIsNotATopicSegment) {
  auto sim = Sim::FromToml(kRobotToml);
  EXPECT_THROW(sim.add<BadLane>("producer"), std::invalid_argument);
}

TEST(TargetEnvelope, BytesFollowTheAlignment) {
  EXPECT_EQ(detail::TargetEnvelopeBytes(24, 8), 32u);
  EXPECT_EQ(detail::TargetEnvelopeBytes(4, 4), 8u);
  EXPECT_EQ(detail::TargetEnvelopeBytes(1, 1), 2u);
  EXPECT_THROW(detail::TargetEnvelopeBytes(12, 8), std::invalid_argument);
  EXPECT_THROW(detail::TargetEnvelopeBytes(16, 16), std::invalid_argument);
  EXPECT_THROW(detail::TargetEnvelopeBytes(8, 3), std::invalid_argument);
}

}  // namespace
}  // namespace talos
