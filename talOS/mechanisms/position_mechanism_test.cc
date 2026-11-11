// PositionMechanism on a simulated robot (component tier): a node drives it
// from its teleop lane, as a robot node does, with SimHardware as the bridge.
//
// Node test matrix (docs/TESTING.md §7) for the library: 1, 2, 3, 7, 8, 9 and
// 12 below. n/a 4, 5, 6: released, never-sent and Driver-Station-lost lanes
// reach the library exactly as a stale one does, as stop() (no set_goal()
// this tick); the owning node's matrix covers its Target. n/a 10, 11: lane
// selection and generations are the owner's talos::Target, not the
// library's.
//
// Limit-switch homing (consecutive samples, timeout, stall, stuck switch,
// Driver Station clear, re-homing after a reset) and the travel clamp are
// covered through the library's first consumer,
// examples/superstructure/cpp/superstructure_test.cc (ElevatorAlone,
// ElevatorHoming, ElevatorConfig); they are not repeated here.

#include "talOS/mechanisms/position_mechanism.h"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "control/control_generated.h"
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
using testing::DsPacket;
using testing::PublishTarget;
using testing::Sim;
using testing::SimHardware;
namespace ds = driver_station;

// Each subsystem's mechanism is named like the subsystem: [motors.<name>]
// and the [<name>] parameter table. arm, turret and lift command a remote
// CANcoder's absolute frame (homing = "none"); hood and wrist home a rezero
// frame. Device names resolve across the whole config, and a remote sensor
// must be its own subsystem's, so each CANcoder has its own name.
constexpr const char* kRobotToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.arm.motors.arm]
can_id = 1
feedback = "RemoteCANcoder"
feedback_sensor = "arm_cc"
position_frame = "absolute"
[subsystems.arm.sensors.arm_cc]
type = "CANcoder"
can_id = 2
[subsystems.arm.arm]
tolerance_rot = 0.01
gravity = "arm"
gravity_v = 1.0

[subsystems.turret.motors.turret]
can_id = 3
feedback = "RemoteCANcoder"
feedback_sensor = "turret_cc"
position_frame = "absolute"
[subsystems.turret.sensors.turret_cc]
type = "CANcoder"
can_id = 4
[subsystems.turret.turret]
tolerance_rot = 0.01
wrap_rot = 1.0
min_rot = -0.75
max_rot = 0.75

[subsystems.lift.motors.lift]
can_id = 5
feedback = "RemoteCANcoder"
feedback_sensor = "lift_cc"
position_frame = "absolute"
[subsystems.lift.sensors.lift_cc]
type = "CANcoder"
can_id = 6
[subsystems.lift.lift]
tolerance_rot = 0.05
at_goal_debounce_us = 30000
gravity_v = 0.5
stall_time_us = 200000
stall_velocity_rps = 0.05
move_timeout_us = 1000000

[subsystems.hood.motors.hood]
can_id = 7
[subsystems.hood.hood]
tolerance_rot = 0.01
control = "position"
homing = "stall"
homing_v = -1.0
min_rot = 0.0
homing_timeout_us = 500000
homing_stall_rps = 0.05
homing_stall_us = 50000

[subsystems.wrist.motors.wrist]
can_id = 8
[subsystems.wrist.sensors.angle]
type = "CANcoder"
can_id = 9
[subsystems.wrist.wrist]
tolerance_rot = 0.01
homing = "absolute"
absolute_sensor = "angle"
absolute_offset_rot = 0.25
)";

// A robot node around one PositionMechanism: its goal is the teleop lane's,
// and with no live target it stops. With `hold_mode` a live target means
// "hold where you are".
struct PositionNode {
  explicit PositionNode(Node& node) : mech{node, node.name()}, target{node} {}
  void update() {
    if (const control::PositionTarget* t = target.get()) {
      if (hold_mode) {
        mech.hold(t->feedforward_v());
      } else {
        mech.set_goal(t->position_rot(), t->feedforward_v());
      }
    } else {
      mech.stop();
    }
    mech.update();
  }
  PositionMechanism mech;
  Target<control::PositionTarget> target;
  bool hold_mode = false;
};

// One subsystem of kRobotToml on an enabled robot. Set feedback, then
// Start() puts the Driver Station in enabled teleop.
struct Rig {
  explicit Rig(std::string_view subsystem)
      : sim{Sim::FromToml(kRobotToml)},
        node{sim.add<PositionNode>(subsystem)},
        hw{sim},
        name{subsystem} {
    hw.enable();
  }
  void Start() {
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(10ms);
  }
  // The teleop lane, republished every 5 ms for `duration`.
  void Drive(double position_rot, std::chrono::milliseconds duration,
             double feedforward_v = 0) {
    for (auto t = 0ms; t < duration; t += 5ms) {
      PublishTarget(sim, name, "teleop",
                    control::PositionTarget{position_rot, feedforward_v});
      sim.step(5ms);
    }
  }
  void SetMotor(double position_rot, double velocity_rps = 0) {
    hw.set_motor(name, name, position_rot, velocity_rps);
  }
  hardware::MotorRequest Request() const {
    return hw.motor_request(name, name);
  }
  hardware::MotorRequest Applied() const {
    return hw.applied_motor(name, name);
  }
  // Neutral as applied, and the state says it drives nothing.
  void ExpectNeutral() const {
    EXPECT_EQ(Applied().mode, Mode::kNeutral) << name;
    const control::PositionState state = node.mech.state();
    EXPECT_FALSE(state.enabled()) << name;
    EXPECT_FALSE(state.at_goal()) << name;
    EXPECT_DOUBLE_EQ(state.goal_rot(), 0.0) << name;
  }
  PositionMechanism& mech() { return node.mech; }

  Sim sim;
  PositionNode& node;
  SimHardware hw;
  std::string name;
};

// REQ-SAFE-01: a live goal drives the motor's motion-magic loop; the state
// reflects it.
// REQ-SAFE-36
TEST(PositionMechanism, DrivesAtALiveLane) {
  Rig arm{"arm"};
  arm.hw.ideal_motors();
  arm.Start();
  arm.Drive(0.2, 30ms);
  EXPECT_EQ(arm.Applied().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(arm.Applied().demand, 0.2);
  EXPECT_EQ(arm.mech().phase(), PositionPhase::kAtGoal);
  const control::PositionState state = arm.mech().state();
  EXPECT_TRUE(state.enabled());
  EXPECT_TRUE(state.at_goal());
  EXPECT_TRUE(state.connected());
  EXPECT_DOUBLE_EQ(state.goal_rot(), 0.2);
  EXPECT_DOUBLE_EQ(state.position_rot(), 0.2);
}

// REQ-SAFE-02
// REQ-SAFE-36
TEST(PositionMechanism, DisabledRobotGoesNeutral) {
  Rig arm{"arm"};
  arm.hw.ideal_motors();
  arm.Start();
  arm.Drive(0.2, 30ms);
  ASSERT_EQ(arm.Applied().mode, Mode::kMotionMagic);
  arm.hw.enable(false);
  arm.Drive(0.2, 30ms);
  arm.ExpectNeutral();
  EXPECT_EQ(arm.mech().phase(), PositionPhase::kReady);
}

// REQ-SAFE-07: no live target is stop(): neutral, never the old goal.
// REQ-SAFE-36
TEST(PositionMechanism, StaleLaneGoesNeutral) {
  Rig arm{"arm"};
  arm.hw.ideal_motors();
  arm.Start();
  arm.Drive(0.2, 30ms);
  ASSERT_EQ(arm.Applied().mode, Mode::kMotionMagic);
  arm.sim.step(110ms);  // past command_timeout_us
  arm.ExpectNeutral();
}

// REQ-SAFE-36
TEST(PositionMechanism, DisconnectedFeedbackGoesNeutral) {
  Rig arm{"arm"};
  arm.Start();
  arm.hw.set_connected("arm", "arm", false);
  arm.Drive(0.2, 30ms);
  arm.ExpectNeutral();
  EXPECT_FALSE(arm.mech().state().connected());
}

// REQ-SAFE-04
// REQ-SAFE-36
TEST(PositionMechanism, NonFiniteDemandGoesNeutral) {
  Rig arm{"arm"};
  arm.Start();
  arm.Drive(std::numeric_limits<double>::quiet_NaN(), 30ms);
  arm.ExpectNeutral();
  arm.Drive(0.2, 30ms, std::numeric_limits<double>::infinity());
  arm.ExpectNeutral();
}

// Each bad [m] table throws at construction, naming the key.
// REQ-SAFE-36
TEST(PositionMechanism, BadParameterFailsConstruction) {
  const auto toml =
      [](const std::string& table, const std::string& motor = "") {
        return std::string{R"(
[robot]
period_us = 5000
command_timeout_us = 100000
[subsystems.m.motors.m]
can_id = 1
)"} + motor + R"(
[subsystems.m.digital_inputs.bottom]
dio = 0
[subsystems.m.sensors.cc]
type = "CANcoder"
can_id = 2
[subsystems.m.m]
)" + table;
      };
  // A valid stall-homed mechanism, less homing_v and homing_stall_us.
  const std::string stall =
      "tolerance_rot = 0.1\nhoming = \"stall\"\nmin_rot = 0.0\n"
      "homing_timeout_us = 100000\nhoming_stall_rps = 0.1\n";
  const std::string ok = stall + "homing_v = -1.0\nhoming_stall_us = 50000\n";
  {
    auto sim = Sim::FromToml(toml(ok));
    EXPECT_NO_THROW(sim.add<PositionNode>("m"));
  }
  const std::vector<std::pair<std::string, std::string>> bad{
      {"tolerance_rot", "homing = \"absolute\"\nabsolute_sensor = \"cc\"\n"},
      {"tolerance_rot",
       "tolerance_rot = 0.0\nhoming = \"absolute\"\n"
       "absolute_sensor = \"cc\"\n"},
      {"homing", "tolerance_rot = 0.1\nhoming = \"magic\"\n"},
      // "none" on a rezero-frame motor would never drive.
      {"homing", "tolerance_rot = 0.1\n"},
      {"limit_switch",
       "tolerance_rot = 0.1\nhoming = \"limit_switch\"\nhoming_v = -1.0\n"},
      // Pointing away from home_rot (0, at min_rot).
      {"homing_v", stall + "homing_v = 1.0\nhoming_stall_us = 50000\n"},
      {"homing_stall_us", stall + "homing_v = -1.0\n"},
      {"max_rot", ok + "max_rot = 0.0\n"},
      {"gravity", ok + "gravity = \"down\"\n"},
      {"absolute_sensor", "tolerance_rot = 0.1\nhoming = \"absolute\"\n"},
  };
  // Gravity from one source: the motor's slot g, or gravity_v, not both.
  const std::string slot_g = "slot0 = { g = 0.3 }\n";
  {
    auto sim = Sim::FromToml(toml(ok, slot_g));
    EXPECT_NO_THROW(sim.add<PositionNode>("m"));
  }
  {
    auto sim = Sim::FromToml(toml(ok + "gravity_v = 0.5\n", slot_g));
    try {
      sim.add<PositionNode>("m");
      ADD_FAILURE() << "constructed with slot0 g and gravity_v";
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string{e.what()}.find("gravity_v"), std::string::npos)
          << e.what();
    }
  }
  for (const auto& [key, table] : bad) {
    SCOPED_TRACE(key + " in:\n" + table);
    auto sim = Sim::FromToml(toml(table));
    try {
      sim.add<PositionNode>("m");
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string{e.what()}.find(key), std::string::npos) << e.what();
    }
  }
}

// REQ-RT-06
TEST(PositionMechanism, UpdateDoesNotAllocate) {
  if (const auto why = testing::AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  Rig lift{"lift"};
  lift.hw.ideal_motors();
  lift.Start();
  lift.Drive(1.0, 40ms);
  PublishTarget(lift.sim, "lift", "teleop", control::PositionTarget{1.0, 0.0});
  testing::AllocationProbe probe;
  probe.start();
  lift.sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(lift.mech().phase(), PositionPhase::kAtGoal);
}

// Stall homing pushes homing_v until the mechanism has been still for
// homing_stall_us, then sets the frame there (home_rot); control =
// "position" uses the position loop.
// REQ-SAFE-36
TEST(PositionMechanism, StallHomingSetsTheFrameAtTheHardStop) {
  Rig hood{"hood"};
  hood.SetMotor(2.0);
  hood.hw.ideal_motors();  // a voltage request does not move it
  hood.Start();
  hood.Drive(0.5, 30ms);
  EXPECT_EQ(hood.mech().phase(), PositionPhase::kHoming);
  EXPECT_EQ(hood.Applied().mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(hood.Applied().demand, -1.0);
  EXPECT_FALSE(hood.mech().homed());
  hood.Drive(0.5, 40ms);
  EXPECT_TRUE(hood.mech().homed());
  EXPECT_EQ(hood.Applied().mode, Mode::kPosition);
  EXPECT_DOUBLE_EQ(hood.Applied().demand, 2.5);  // 0.5 past the stop
  EXPECT_EQ(hood.mech().phase(), PositionPhase::kAtGoal);
}

// Absolute homing takes the frame from the CANcoder (scaled, offset) without
// moving, waits while it is disconnected, and takes it again after a device
// reset drops the zero.
// REQ-SAFE-36
TEST(PositionMechanism, AbsoluteHomingFollowsTheCancoderAcrossAReset) {
  Rig wrist{"wrist"};
  wrist.SetMotor(5.0);
  wrist.hw.set_cancoder("wrist", "angle", 0.1, 0.0);
  wrist.hw.set_connected("wrist", "angle", false);
  wrist.Start();
  wrist.Drive(0.35, 20ms);
  EXPECT_EQ(wrist.mech().phase(), PositionPhase::kUnhomed);
  wrist.ExpectNeutral();

  wrist.hw.set_connected("wrist", "angle", true);
  wrist.Drive(0.35, 20ms);
  EXPECT_TRUE(wrist.mech().homed());
  EXPECT_NEAR(wrist.mech().position_rot(), 0.35, 1e-9);
  EXPECT_EQ(wrist.Applied().mode, Mode::kMotionMagic);
  EXPECT_NEAR(wrist.Applied().demand, 5.0, 1e-9);  // raw frame

  // The TalonFX resets: its count restarts at 1.0 where it is.
  wrist.hw.reset_device("wrist", "wrist");
  wrist.SetMotor(1.0);
  wrist.Drive(0.35, 40ms);
  EXPECT_TRUE(wrist.mech().homed());
  EXPECT_EQ(wrist.Applied().mode, Mode::kMotionMagic);
  EXPECT_NEAR(wrist.Applied().demand, 1.0, 1e-9);
}

// A goal short of the target that stops moving faults (stall), latched
// neutral; a stale link does not clear it, a Driver Station disable does.
// REQ-SAFE-36
TEST(PositionMechanism, StallFaultLatchesUntilTheDriverStationDisables) {
  Rig lift{"lift"};
  lift.Start();
  lift.Drive(1.0, 150ms);  // the motor never moves (velocity 0)
  EXPECT_EQ(lift.mech().phase(), PositionPhase::kMoving);
  EXPECT_EQ(lift.Applied().mode, Mode::kMotionMagic);
  lift.Drive(1.0, 100ms);
  EXPECT_EQ(lift.mech().phase(), PositionPhase::kFaulted);
  EXPECT_EQ(lift.mech().fault(), PositionFault::kStall);
  lift.ExpectNeutral();

  lift.hw.lose_states(30);  // robot_enabled false for 150 ms
  lift.Drive(1.0, 250ms);
  EXPECT_EQ(lift.mech().fault(), PositionFault::kStall);
  lift.ExpectNeutral();

  lift.hw.set_driver_station(DsPacket(ds::kTeleop));  // disabled
  lift.sim.step(20ms);
  lift.hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  lift.sim.step(10ms);
  EXPECT_EQ(lift.mech().fault(), PositionFault::kNone);
  lift.Drive(1.0, 20ms);
  EXPECT_EQ(lift.Applied().mode, Mode::kMotionMagic);
}

// A goal not reached within move_timeout_us faults, even while the motor
// keeps moving (no stall).
// REQ-SAFE-36
TEST(PositionMechanism, MoveTimeoutFaults) {
  Rig lift{"lift"};
  lift.SetMotor(0.2, 1.0);  // moving, never arriving
  lift.Start();
  lift.Drive(1.5, 900ms);
  EXPECT_EQ(lift.mech().phase(), PositionPhase::kMoving);
  lift.Drive(1.5, 200ms);
  EXPECT_EQ(lift.mech().fault(), PositionFault::kMoveTimeout);
  lift.ExpectNeutral();
}

// Elevator gravity is a constant +gravity_v wherever the carriage is, added
// to the target's own feedforward.
TEST(PositionMechanism, ElevatorGravityIsConstantAndUp) {
  Rig lift{"lift"};
  lift.Start();
  lift.Drive(1.0, 20ms, 0.25);
  EXPECT_DOUBLE_EQ(lift.Applied().feedforward_v, 0.75);
  lift.SetMotor(1.8);
  lift.Drive(1.0, 20ms);
  EXPECT_DOUBLE_EQ(lift.Applied().feedforward_v, 0.5);
}

// Arm gravity follows cos(position - gravity_zero_rot): +gravity_v level on
// one side, 0 straight up, -gravity_v level on the other.
TEST(PositionMechanism, ArmGravityFollowsTheCosine) {
  Rig arm{"arm"};
  arm.Start();
  const auto ff_at = [&](double position_rot) {
    arm.SetMotor(position_rot);
    arm.Drive(position_rot, 20ms);
    EXPECT_EQ(arm.Applied().mode, Mode::kMotionMagic);
    return arm.Applied().feedforward_v;
  };
  EXPECT_NEAR(ff_at(0.0), 1.0, 1e-9);
  EXPECT_NEAR(ff_at(0.125), 0.70710678, 1e-6);
  EXPECT_NEAR(ff_at(0.25), 0.0, 1e-9);
  EXPECT_NEAR(ff_at(0.5), -1.0, 1e-9);
  EXPECT_NEAR(ff_at(-0.25), 0.0, 1e-9);
}

// With wrap_rot a turret goes to the equivalent of the goal nearest where it
// is, inside its limits.
// REQ-SAFE-36
TEST(PositionMechanism, TurretWrapPicksTheNearestEquivalentInRange) {
  Rig turret{"turret"};
  turret.Start();
  const auto demand_for = [&](double position_rot, double goal_rot) {
    SCOPED_TRACE(std::to_string(position_rot) + " -> " +
                 std::to_string(goal_rot));
    turret.SetMotor(position_rot);
    turret.Drive(goal_rot, 20ms);
    EXPECT_NEAR(turret.mech().goal_rot(), turret.Applied().demand, 1e-12);
    return turret.Applied().demand;
  };
  EXPECT_NEAR(demand_for(0.6, -0.3), 0.7, 1e-9);   // across the seam
  EXPECT_NEAR(demand_for(0.6, 0.3), 0.3, 1e-9);    // already nearest
  EXPECT_NEAR(demand_for(0.7, -0.9), 0.1, 1e-9);   // 1.1 is past max_rot
  EXPECT_NEAR(demand_for(-0.6, 0.5), -0.5, 1e-9);  // the other way
}

// at_goal needs at_goal_debounce_us of continuous tolerance, and leaves at
// once.
TEST(PositionMechanism, AtGoalDebounceEdges) {
  Rig lift{"lift"};
  lift.Start();
  lift.Drive(1.0, 20ms);
  EXPECT_EQ(lift.mech().phase(), PositionPhase::kMoving);
  lift.SetMotor(1.02);  // inside tolerance_rot
  lift.Drive(1.0, 20ms);
  EXPECT_FALSE(lift.mech().state().at_goal());
  lift.Drive(1.0, 30ms);
  EXPECT_TRUE(lift.mech().state().at_goal());
  lift.SetMotor(1.2);
  lift.Drive(1.0, 10ms);
  EXPECT_EQ(lift.mech().phase(), PositionPhase::kMoving);
}

// hold() keeps the position it had when the hold began, not where the
// mechanism drifts to; a new hold starts from the new position.
// REQ-SAFE-36
TEST(PositionMechanism, HoldLatchesThePositionAtTheStart) {
  Rig arm{"arm"};
  arm.node.hold_mode = true;
  arm.SetMotor(0.3);
  arm.Start();
  arm.Drive(9.0, 20ms);  // the goal is ignored while holding
  EXPECT_DOUBLE_EQ(arm.Applied().demand, 0.3);
  arm.SetMotor(0.32);
  arm.Drive(9.0, 20ms);
  EXPECT_DOUBLE_EQ(arm.Applied().demand, 0.3);
  arm.sim.step(150ms);  // stale: neutral, the hold ends
  arm.ExpectNeutral();
  arm.Drive(9.0, 20ms);
  EXPECT_DOUBLE_EQ(arm.Applied().demand, 0.32);
}

}  // namespace
}  // namespace talos::mechanisms
