// RollerWithSensor on a simulated robot (component tier): a node drives it
// from its teleop lane, as a robot node does, with SimHardware as the bridge.
//
// Node test matrix (docs/TESTING.md §7) for the library: 1, 2, 3, 7, 9 and
// 12 below. n/a 4, 5, 6: released, never-sent and Driver-Station-lost lanes
// reach the library exactly as a stale one does, as stop(). 7 is the piece
// sensor (an invalid sample is no reading; intaking is neutral while it
// lasts); the roller reads no motor feedback (the gateway neutralizes a
// motor whose feedback is invalid, REQ-SAFE-22). n/a 8: a command is an
// enum, there is no number to be non-finite.
// n/a 10, 11: lane selection and generations are the owner's talos::Target.

#include "talOS/mechanisms/roller_with_sensor.h"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "roller_test/roller_test_message_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/devices/testing.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/messages.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/node/testing.h"

namespace talos::mechanisms {
namespace {

using namespace std::chrono_literals;
using hardware::Mode;
using roller_test::Command;
using testing::DsPacket;
using testing::PublishTarget;
using testing::ReleaseTarget;
using testing::Sim;
using testing::SimHardware;
namespace ds = driver_station;

// The intake holds with hold_v and runs an eject or feed asked in kIdle; the
// indexer has no hold_v (neutral while holding), an inverted sensor, a 20 ms
// debounce, and ignores an eject asked in kIdle (the default).
constexpr const char* kRobotToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.intake.motors.roller]
can_id = 1
[subsystems.intake.digital_inputs.beam]
dio = 0
[subsystems.intake.roller]
sensor = "beam"
idle_runs = true
intake_v = 6.0
hold_v = 1.0
eject_v = -8.0
eject_us = 200000
feed_v = 10.0
feed_us = 100000

[subsystems.indexer.motors.belt]
can_id = 2
[subsystems.indexer.digital_inputs.gate]
dio = 1
[subsystems.indexer.belt]
sensor = "gate"
sensor_inverted = true
sensor_debounce_us = 20000
intake_v = 4.0
eject_v = -4.0
eject_us = 100000
)";

// The intake's roller is "roller", the indexer's "belt".
std::string_view RollerOf(std::string_view subsystem) {
  return subsystem == "intake" ? "roller" : "belt";
}

// A robot node around one RollerWithSensor, driven by its teleop lane.
struct RollerNode {
  explicit RollerNode(Node& node)
      : roller{node, RollerOf(node.name())}, target{node} {}
  void update() {
    if (const roller_test::Goal* g = target.get()) {
      roller.set_goal(static_cast<RollerCommand>(g->command()));
    } else {
      roller.stop();
    }
    roller.update();
  }
  RollerWithSensor roller;
  Target<roller_test::Goal> target;
};

static_assert(static_cast<int>(Command::IDLE) ==
              static_cast<int>(RollerCommand::kIdle));
static_assert(static_cast<int>(Command::INTAKE) ==
              static_cast<int>(RollerCommand::kIntake));
static_assert(static_cast<int>(Command::EJECT) ==
              static_cast<int>(RollerCommand::kEject));
static_assert(static_cast<int>(Command::FEED) ==
              static_cast<int>(RollerCommand::kFeed));

struct Rig {
  explicit Rig(std::string_view subsystem)
      : sim{Sim::FromToml(kRobotToml)},
        node{sim.add<RollerNode>(subsystem)},
        hw{sim},
        name{subsystem},
        motor{RollerOf(subsystem)} {
    hw.enable();
  }
  void Start() {
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(10ms);
  }
  // The teleop lane, republished every 5 ms for `duration`.
  void Drive(Command command, std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 5ms) {
      PublishTarget(sim, name, "teleop", roller_test::Goal{command});
      sim.step(5ms);
    }
  }
  void Sensor(std::string_view device, bool value) {
    hw.set_digital_input(name, device, value);
  }
  hardware::MotorRequest Applied() const {
    return hw.applied_motor(name, motor);
  }
  RollerWithSensor& roller() { return node.roller; }

  Sim sim;
  RollerNode& node;
  SimHardware hw;
  std::string name;
  std::string motor;
};

// REQ-SAFE-01: intake until the sensor sees a piece, hold it with hold_v, eject
// for eject_us, then idle neutral.
// REQ-SAFE-38
TEST(RollerWithSensor, DrivesAtALiveLane) {
  Rig intake{"intake"};
  intake.Start();
  intake.Drive(Command::INTAKE, 40ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kIntaking);
  EXPECT_EQ(intake.Applied().mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(intake.Applied().demand, 6.0);
  EXPECT_FALSE(intake.roller().has_piece());

  intake.Sensor("beam", true);
  intake.Drive(Command::INTAKE, 20ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kHolding);
  EXPECT_TRUE(intake.roller().has_piece());
  EXPECT_DOUBLE_EQ(intake.Applied().demand, 1.0);

  intake.Drive(Command::EJECT, 20ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kEjecting);
  EXPECT_DOUBLE_EQ(intake.Applied().demand, -8.0);
  EXPECT_DOUBLE_EQ(intake.roller().output_v(), -8.0);
  intake.Sensor("beam", false);
  intake.Drive(Command::EJECT, 150ms);  // still inside eject_us
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kEjecting);
  EXPECT_DOUBLE_EQ(intake.Applied().demand, -8.0);
  intake.Drive(Command::EJECT, 80ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kIdle);
  EXPECT_EQ(intake.Applied().mode, Mode::kNeutral);
}

// With idle_runs, an eject asked in kIdle runs once however long it is
// held; asking again (after another command) runs it again. Without it
// (the indexer) kIdle ignores an eject.
// REQ-SAFE-38
TEST(RollerWithSensor, IdleRunsAnEjectOncePerAsk) {
  Rig intake{"intake"};
  intake.Start();
  intake.Drive(Command::EJECT, 20ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kEjecting);
  intake.Drive(Command::EJECT, 300ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kIdle);
  EXPECT_EQ(intake.Applied().mode, Mode::kNeutral);
  intake.Drive(Command::IDLE, 10ms);
  intake.Drive(Command::EJECT, 20ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kEjecting);

  Rig indexer{"indexer"};
  indexer.Sensor("gate", true);  // empty
  indexer.Start();
  indexer.Drive(Command::EJECT, 50ms);
  EXPECT_EQ(indexer.roller().phase(), RollerPhase::kIdle);
  EXPECT_EQ(indexer.Applied().mode, Mode::kNeutral);
}

// Feeding a held piece runs feed_v for feed_us, then idles.
TEST(RollerWithSensor, FeedsAHeldPiece) {
  Rig intake{"intake"};
  intake.Sensor("beam", true);
  intake.Start();
  intake.Drive(Command::INTAKE, 20ms);
  ASSERT_EQ(intake.roller().phase(), RollerPhase::kHolding);
  intake.Drive(Command::FEED, 20ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kFeeding);
  EXPECT_DOUBLE_EQ(intake.Applied().demand, 10.0);
  intake.Drive(Command::FEED, 120ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kIdle);
  EXPECT_EQ(intake.Applied().mode, Mode::kNeutral);
}

// sensor_inverted and sensor_debounce_us decide has_piece, and with
// no hold_v a held piece leaves the roller neutral.
TEST(RollerWithSensor, InvertedDebouncedSensorAndNoHoldVoltage) {
  Rig indexer{"indexer"};
  indexer.Sensor("gate", true);  // inverted: true reads empty
  indexer.Start();
  indexer.Drive(Command::INTAKE, 20ms);
  EXPECT_EQ(indexer.roller().phase(), RollerPhase::kIntaking);
  EXPECT_DOUBLE_EQ(indexer.Applied().demand, 4.0);

  indexer.Sensor("gate", false);  // a blip shorter than the debounce
  indexer.Drive(Command::INTAKE, 10ms);
  EXPECT_FALSE(indexer.roller().has_piece());
  indexer.Sensor("gate", true);
  indexer.Drive(Command::INTAKE, 10ms);
  EXPECT_EQ(indexer.roller().phase(), RollerPhase::kIntaking);

  indexer.Sensor("gate", false);
  indexer.Drive(Command::INTAKE, 40ms);
  EXPECT_TRUE(indexer.roller().has_piece());
  EXPECT_EQ(indexer.roller().phase(), RollerPhase::kHolding);
  EXPECT_EQ(indexer.Applied().mode, Mode::kNeutral);
  EXPECT_FALSE(indexer.roller().driving());
}

// A held piece survives an invalid sensor sample that reads "no piece"
// (SimHardware's invalid sample reads false): has_piece() keeps its last
// valid value and the hold keeps hold_v; a valid empty reading drops it.
// REQ-SAFE-38
TEST(RollerWithSensor, InvalidSensorSampleKeepsAHeldPiece) {
  Rig intake{"intake"};
  intake.Sensor("beam", true);
  intake.Start();
  intake.Drive(Command::INTAKE, 20ms);
  ASSERT_EQ(intake.roller().phase(), RollerPhase::kHolding);

  intake.hw.set_connected("intake", "beam", false);
  intake.Drive(Command::INTAKE, 30ms);
  EXPECT_FALSE(intake.roller().sensor_connected());
  EXPECT_TRUE(intake.roller().has_piece());
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kHolding);
  EXPECT_EQ(intake.Applied().mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(intake.Applied().demand, 1.0);

  intake.Sensor("beam", false);  // valid and empty: the piece is gone
  intake.hw.set_connected("intake", "beam", true);
  intake.Drive(Command::IDLE, 20ms);
  EXPECT_FALSE(intake.roller().has_piece());
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kIdle);

  // A confirmed piece is not a debounce run: back valid with the piece still
  // there, the indexer (20 ms debounce) keeps holding from the first reading.
  Rig indexer{"indexer"};
  indexer.Sensor("gate", false);  // a piece
  indexer.Start();
  indexer.Drive(Command::INTAKE, 40ms);
  ASSERT_EQ(indexer.roller().phase(), RollerPhase::kHolding);
  indexer.hw.set_connected("indexer", "gate", false);
  indexer.Drive(Command::INTAKE, 10ms);
  indexer.hw.set_connected("indexer", "gate", true);
  for (int i = 0; i < 6; ++i) {
    SCOPED_TRACE(i);
    indexer.Drive(Command::INTAKE, 5ms);
    EXPECT_TRUE(indexer.roller().has_piece());
    EXPECT_EQ(indexer.roller().phase(), RollerPhase::kHolding);
  }
}

// Matrix 7. Intaking toward a sensor it cannot read is neutral (still
// kIntaking) whatever the invalid sample reads (here "piece": the indexer is
// inverted); back valid with a piece, the roller intakes until the debounce
// completes, counted from the first valid piece reading.
// REQ-SAFE-38
TEST(RollerWithSensor, DisconnectedFeedbackGoesNeutral) {
  Rig indexer{"indexer"};
  indexer.Sensor("gate", true);  // empty
  indexer.Start();
  indexer.Drive(Command::INTAKE, 20ms);
  ASSERT_EQ(indexer.roller().phase(), RollerPhase::kIntaking);
  ASSERT_DOUBLE_EQ(indexer.Applied().demand, 4.0);

  indexer.hw.set_connected("indexer", "gate", false);
  indexer.Drive(Command::INTAKE, 40ms);  // twice the debounce
  EXPECT_FALSE(indexer.roller().sensor_connected());
  EXPECT_FALSE(indexer.roller().has_piece());
  EXPECT_EQ(indexer.roller().phase(), RollerPhase::kIntaking);
  EXPECT_FALSE(indexer.roller().driving());
  EXPECT_EQ(indexer.Applied().mode, Mode::kNeutral);

  indexer.Sensor("gate", false);  // a piece, read validly
  indexer.hw.set_connected("indexer", "gate", true);
  indexer.Drive(Command::INTAKE, 15ms);  // inside the 20 ms debounce
  EXPECT_FALSE(indexer.roller().has_piece());
  EXPECT_EQ(indexer.roller().phase(), RollerPhase::kIntaking);
  EXPECT_EQ(indexer.Applied().mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(indexer.Applied().demand, 4.0);
  indexer.Drive(Command::INTAKE, 25ms);
  EXPECT_TRUE(indexer.roller().has_piece());
  EXPECT_EQ(indexer.roller().phase(), RollerPhase::kHolding);
  EXPECT_EQ(indexer.Applied().mode, Mode::kNeutral);
}

// Valid piece readings interrupted by invalid samples never complete the
// debounce: each invalid sample starts the run over, though the valid
// readings add up to three times sensor_debounce_us.
// REQ-SAFE-38
TEST(RollerWithSensor, InvalidSensorSampleRestartsTheDebounce) {
  Rig indexer{"indexer"};
  indexer.Sensor("gate", true);  // empty
  indexer.Start();
  indexer.Drive(Command::INTAKE, 20ms);
  ASSERT_EQ(indexer.roller().phase(), RollerPhase::kIntaking);

  indexer.Sensor("gate", false);  // a piece
  for (int i = 0; i < 6; ++i) {
    SCOPED_TRACE(i);
    indexer.hw.set_connected("indexer", "gate", true);
    indexer.Drive(Command::INTAKE, 10ms);  // two valid readings
    EXPECT_FALSE(indexer.roller().has_piece());
    indexer.hw.set_connected("indexer", "gate", false);
    indexer.Drive(Command::INTAKE, 5ms);  // one invalid
    EXPECT_FALSE(indexer.roller().has_piece());
  }
  EXPECT_EQ(indexer.roller().phase(), RollerPhase::kIntaking);
}

// REQ-SAFE-02: a disabled robot drives nothing, while the state machine keeps
// the held piece.
// REQ-SAFE-38
TEST(RollerWithSensor, DisabledRobotGoesNeutral) {
  Rig intake{"intake"};
  intake.Sensor("beam", true);
  intake.Start();
  intake.Drive(Command::INTAKE, 20ms);
  ASSERT_EQ(intake.roller().phase(), RollerPhase::kHolding);
  ASSERT_EQ(intake.Applied().mode, Mode::kVoltage);
  intake.hw.enable(false);
  intake.Drive(Command::INTAKE, 50ms);
  EXPECT_EQ(intake.Applied().mode, Mode::kNeutral);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kHolding);
  EXPECT_TRUE(intake.roller().has_piece());
}

// REQ-SAFE-07: a stale or released target drives nothing; a timed eject still
// finishes its clock, without driving.
// REQ-SAFE-38
TEST(RollerWithSensor, StaleLaneGoesNeutral) {
  Rig intake{"intake"};
  intake.Start();
  intake.Drive(Command::INTAKE, 20ms);
  ASSERT_EQ(intake.Applied().mode, Mode::kVoltage);
  ReleaseTarget<roller_test::Goal>(intake.sim, "intake", "teleop");
  intake.sim.step(20ms);
  EXPECT_EQ(intake.Applied().mode, Mode::kNeutral);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kIdle);

  intake.Drive(Command::EJECT, 20ms);
  ASSERT_EQ(intake.roller().phase(), RollerPhase::kEjecting);
  intake.sim.step(150ms);  // past command_timeout_us
  EXPECT_EQ(intake.Applied().mode, Mode::kNeutral);
  intake.sim.step(100ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kIdle);
  EXPECT_EQ(intake.Applied().mode, Mode::kNeutral);
}

// Losing enable cancels a timed run: re-enabled within eject_us with the
// eject still asked, the roller does not pick the run up again.
// REQ-SAFE-38
TEST(RollerWithSensor, DisableCancelsATimedRun) {
  Rig intake{"intake"};
  intake.Sensor("beam", true);
  intake.Start();
  intake.Drive(Command::INTAKE, 20ms);
  intake.Drive(Command::EJECT, 20ms);
  ASSERT_EQ(intake.roller().phase(), RollerPhase::kEjecting);
  intake.Sensor("beam", false);  // the piece is leaving
  intake.hw.enable(false);
  intake.Drive(Command::EJECT, 20ms);
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kIdle);
  intake.hw.enable();
  intake.Drive(Command::IDLE, 50ms);  // well inside eject_us
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kIdle);
  EXPECT_EQ(intake.Applied().mode, Mode::kNeutral);
}

// With idle_runs, a release ends an ask: press, release (the lane released,
// as declarative teleop does with no binding live), press runs twice, while
// the held press before it ran once.
// REQ-SAFE-38
TEST(RollerWithSensor, ReleaseThenPressRunsAgain) {
  Rig intake{"intake"};
  intake.Start();
  intake.Drive(Command::EJECT, 20ms);
  ASSERT_EQ(intake.roller().phase(), RollerPhase::kEjecting);
  intake.Drive(Command::EJECT, 300ms);  // held: the run ends, no second
  ASSERT_EQ(intake.roller().phase(), RollerPhase::kIdle);
  ReleaseTarget<roller_test::Goal>(intake.sim, "intake", "teleop");
  intake.sim.step(20ms);
  intake.Drive(Command::EJECT, 20ms);  // pressed again
  EXPECT_EQ(intake.roller().phase(), RollerPhase::kEjecting);
  EXPECT_EQ(intake.Applied().mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(intake.Applied().demand, -8.0);
}

// A request is refused outside update() (here every()), naming the roller,
// and lasts only the update() it was made in.
// REQ-SAFE-38
TEST(RollerWithSensor, RequestIsTiedToItsUpdate) {
  struct TimerNode {
    explicit TimerNode(Node& node) : roller{node, "roller"} {
      node.every(20ms, [this] { roller.set_goal(RollerCommand::kIntake); });
    }
    void update() { roller.update(); }
    RollerWithSensor roller;
  };
  {
    auto sim = Sim::FromToml(kRobotToml);
    sim.add<TimerNode>("intake");
    try {
      sim.step(50ms);
      ADD_FAILURE() << "set_goal() from every() was accepted";
    } catch (const std::logic_error& e) {
      EXPECT_NE(std::string{e.what()}.find("RollerWithSensor 'roller'"),
                std::string::npos)
          << e.what();
    }
  }
  struct SkippingNode {
    explicit SkippingNode(Node& node) : roller{node, "roller"} {}
    void update() {
      if (armed && step++ == 0) {
        roller.set_goal(RollerCommand::kEject);  // update() skipped
        return;
      }
      roller.update();
    }
    RollerWithSensor roller;
    bool armed = false;
    int step = 0;
  };
  auto sim = Sim::FromToml(kRobotToml);
  auto& node = sim.add<SkippingNode>("intake");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(10ms);
  node.armed = true;
  sim.step(15ms);
  ASSERT_GE(node.step, 2);
  EXPECT_EQ(node.roller.phase(), RollerPhase::kIdle);  // the eject was dropped
  EXPECT_EQ(hw.applied_motor("intake", "roller").mode, Mode::kNeutral);
}

// REQ-RT-06
TEST(RollerWithSensor, UpdateDoesNotAllocate) {
  if (const auto why = testing::AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  Rig intake{"intake"};
  intake.Sensor("beam", true);
  intake.Start();
  intake.Drive(Command::INTAKE, 40ms);
  ASSERT_EQ(intake.roller().phase(), RollerPhase::kHolding);
  PublishTarget(intake.sim, "intake", "teleop",
                roller_test::Goal{Command::INTAKE});
  testing::AllocationProbe probe;
  probe.start();
  intake.sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(intake.Applied().mode, Mode::kVoltage);
}

// Each bad parameter throws at construction, naming the key.
// REQ-SAFE-38
TEST(RollerWithSensor, BadParameterFailsConstruction) {
  const auto add = [](const std::string& table) {
    auto sim = Sim::FromToml(std::string{R"(
[robot]
period_us = 5000
command_timeout_us = 100000
[subsystems.intake.motors.roller]
can_id = 1
[subsystems.intake.digital_inputs.beam]
dio = 0
[subsystems.intake.roller]
intake_v = 6.0
eject_v = -6.0
)"} + table);
    sim.add<RollerNode>("intake");
  };
  EXPECT_NO_THROW(add("sensor = \"beam\"\neject_us = 100000\n"));
  const std::vector<std::pair<std::string, std::string>> bad{
      {"sensor", "eject_us = 100000\n"},
      {"beam_typo", "sensor = \"beam_typo\"\neject_us = 100000\n"},
      {"eject_us", "sensor = \"beam\"\neject_us = 0\n"},
      {"eject_us", "sensor = \"beam\"\n"},
      {"hold_v", "sensor = \"beam\"\neject_us = 100000\nhold_v = inf\n"},
  };
  for (const auto& [key, table] : bad) {
    SCOPED_TRACE(table);
    try {
      add(table);
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string{e.what()}.find(key), std::string::npos) << e.what();
    }
  }
}

}  // namespace
}  // namespace talos::mechanisms
