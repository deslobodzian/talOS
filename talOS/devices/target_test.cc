#include "talOS/devices/target.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
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

}  // namespace
}  // namespace talos
