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
// Limit-switch homing toward a negative homing_v (consecutive samples,
// timeout, stall, stuck switch, Driver Station clear, re-homing after a
// reset) and the travel clamp are also covered through the library's first
// consumer, examples/superstructure/cpp/superstructure_test.cc
// (ElevatorAlone, ElevatorHoming, ElevatorConfig).

#include "talOS/mechanisms/position_mechanism.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <optional>
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
#include "talOS/hardware/sim_physics.h"
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
// CANcoder's absolute frame (homing = "none"); hood, wrist and top home a
// rezero frame. Device names resolve across the whole config, and a remote
// sensor must be its own subsystem's, so each CANcoder has its own name.
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
min_rot = 0.0
max_rot = 1.0
homing = "stall"
homing_v = -1.0
homing_timeout_us = 500000
homing_stall_rps = 0.05
homing_stall_us = 50000
homing_stall_current_a = 20.0
homing_min_push_us = 200000
homing_min_travel_rot = 0.05

[subsystems.wrist.motors.wrist]
can_id = 8
[subsystems.wrist.sensors.angle]
type = "CANcoder"
can_id = 9
[subsystems.wrist.wrist]
tolerance_rot = 0.01
min_rot = 0.0
max_rot = 0.5
homing = "absolute"
absolute_sensor = "angle"
absolute_offset_rot = 0.25
absolute_margin_rot = 0.05

[subsystems.top.motors.top]
can_id = 10
[subsystems.top.digital_inputs.top_switch]
dio = 0
[subsystems.top.top]
tolerance_rot = 0.01
min_rot = 0.0
max_rot = 2.0
homing = "limit_switch"
limit_switch = "top_switch"
limit_switch_closed_reads = true
homing_v = 1.0
home_rot = 2.0
home_samples = 2
home_margin_rot = 0.2
homing_timeout_us = 500000
homing_stall_rps = 0.05
homing_stall_us = 200000

[subsystems.carriage.motors.carriage]
can_id = 11
[subsystems.carriage.digital_inputs.carriage_home]
dio = 1
[subsystems.carriage.carriage]
tolerance_rot = 0.01
min_rot = 0.0
max_rot = 2.0
homing = "limit_switch"
limit_switch = "carriage_home"
homing_v = -1.0
home_samples = 2
home_margin_rot = 0.2
homing_timeout_us = 500000
homing_stall_rps = 0.05
homing_stall_us = 200000
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
template <class N = PositionNode>
struct Rig {
  explicit Rig(std::string_view subsystem)
      : sim{Sim::FromToml(kRobotToml)},
        node{sim.add<N>(subsystem)},
        hw{sim},
        name{subsystem} {
    hw.enable();
  }
  void Start() {
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(10ms);
  }
  // The teleop lane, republished every 5 ms (one tick) for `duration`.
  void Drive(double position_rot, std::chrono::milliseconds duration,
             double feedforward_v = 0) {
    for (auto t = 0ms; t < duration; t += 5ms) {
      PublishTarget(sim, name, "teleop",
                    control::PositionTarget{position_rot, feedforward_v});
      sim.step(5ms);
    }
  }
  void SetMotor(double position_rot, double velocity_rps = 0,
                double stator_current_a = 0) {
    hw.set_motor(name, name, position_rot, velocity_rps, 0, 0, 0, 0,
                 stator_current_a);
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
  void ExpectHoming(double homing_v) const {
    EXPECT_EQ(node.mech.phase(), PositionPhase::kHoming) << name;
    EXPECT_EQ(Applied().mode, Mode::kVoltage) << name;
    EXPECT_DOUBLE_EQ(Applied().demand, homing_v) << name;
    EXPECT_FALSE(node.mech.homed()) << name;
  }
  PositionMechanism& mech() { return node.mech; }

  Sim sim;
  N& node;
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

// A request outside update() (here a message callback, which would act on
// a goal no Target selected) is refused, naming the mechanism.
// REQ-SAFE-36
TEST(PositionMechanism, RequestOutsideUpdateThrows) {
  struct CallbackNode {
    explicit CallbackNode(Node& node)
        : mech{node, "arm"},
          goal{node.subscribe<control::PositionTarget>(
              "/test/goal", [this](const control::PositionTarget& t) {
                mech.set_goal(t.position_rot());
              })} {}
    void update() { mech.update(); }
    PositionMechanism mech;
    Subscription<control::PositionTarget> goal;
  };
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<CallbackNode>("arm");
  SimHardware hw{sim};
  hw.enable();
  sim.step(10ms);
  sim.publish("/test/goal", control::PositionTarget{0.2, 0.0});
  try {
    sim.step(10ms);
    ADD_FAILURE() << "set_goal() from a callback was accepted";
  } catch (const std::logic_error& e) {
    EXPECT_NE(std::string{e.what()}.find("PositionMechanism 'arm'"),
              std::string::npos)
        << e.what();
  }
}

// A request lasts the update() it was made in: made in tick N by an owner
// whose update() then skips the mechanism's update(), it does not drive in
// tick N + 1.
// REQ-SAFE-36
TEST(PositionMechanism, RequestDoesNotCarryIntoTheNextTick) {
  struct SkippingNode {
    explicit SkippingNode(Node& node) : mech{node, "arm"} {}
    void update() {
      if (!armed) {
        mech.update();
        return;
      }
      if (step == 0) {
        mech.set_goal(0.2);  // tick N: requested, but update() is skipped
      } else {
        mech.update();  // tick N + 1 on: nothing asked this tick
      }
      ++step;
    }
    PositionMechanism mech;
    bool armed = false;
    int step = 0;
  };
  Rig<SkippingNode> arm{"arm"};
  arm.Start();
  arm.node.armed = true;
  arm.sim.step(15ms);
  ASSERT_GE(arm.node.step, 2);
  EXPECT_FALSE(arm.mech().driving());
  EXPECT_EQ(arm.Applied().mode, Mode::kNeutral);
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
      "max_rot = 1.0\nhoming_timeout_us = 500000\nhoming_stall_rps = 0.1\n"
      "homing_stall_current_a = 20.0\nhoming_min_push_us = 50000\n"
      "homing_min_travel_rot = 0.05\n";
  const std::string ok = stall + "homing_v = -1.0\nhoming_stall_us = 50000\n";
  const std::string absolute =
      "tolerance_rot = 0.1\nhoming = \"absolute\"\nabsolute_sensor = \"cc\"\n"
      "absolute_margin_rot = 0.05\n";
  {
    auto sim = Sim::FromToml(toml(ok));
    EXPECT_NO_THROW(sim.add<PositionNode>("m"));
  }
  const auto expect_refused = [&](const std::string& key,
                                  const std::string& config) {
    SCOPED_TRACE(key + " in:\n" + config);
    auto sim = Sim::FromToml(config);
    try {
      sim.add<PositionNode>("m");
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string{e.what()}.find(key), std::string::npos) << e.what();
    }
  };
  const std::vector<std::pair<std::string, std::string>> bad{
      {"tolerance_rot", "homing = \"absolute\"\nabsolute_sensor = \"cc\"\n"},
      {"tolerance_rot", "tolerance_rot = 0.0\n"},
      {"homing", "tolerance_rot = 0.1\nhoming = \"magic\"\n"},
      // "none" on a rezero-frame motor would never drive.
      {"homing", "tolerance_rot = 0.1\n"},
      {"limit_switch",
       "tolerance_rot = 0.1\nhoming = \"limit_switch\"\nmin_rot = 0.0\n"
       "max_rot = 1.0\nhoming_v = -1.0\n"},
      // Pointing away from home_rot (0, at min_rot).
      {"homing_v", stall + "homing_v = 1.0\nhoming_stall_us = 50000\n"},
      // More than half the motor's max_voltage (12 V by default).
      {"homing_v", stall + "homing_v = -7.0\nhoming_stall_us = 50000\n"},
      {"homing_stall_us", stall + "homing_v = -1.0\n"},
      {"homing_stall_current_a",
       "tolerance_rot = 0.1\nhoming = \"stall\"\nmin_rot = 0.0\n"
       "max_rot = 1.0\nhoming_timeout_us = 100000\nhoming_stall_rps = 0.1\n"
       "homing_v = -1.0\nhoming_stall_us = 50000\n"
       "homing_min_push_us = 50000\n"},
      {"homing_min_push_us",
       "tolerance_rot = 0.1\nhoming = \"stall\"\nmin_rot = 0.0\n"
       "max_rot = 1.0\nhoming_timeout_us = 100000\nhoming_stall_rps = 0.1\n"
       "homing_v = -1.0\nhoming_stall_us = 50000\n"
       "homing_stall_current_a = 20.0\n"},
      {"homing_timeout_us",
       "tolerance_rot = 0.1\nhoming = \"stall\"\nmin_rot = 0.0\n"
       "max_rot = 1.0\nhoming_stall_rps = 0.1\nhoming_v = -1.0\n"
       "homing_stall_us = 50000\nhoming_stall_current_a = 20.0\n"
       "homing_min_push_us = 50000\n"},
      {"gravity", ok + "gravity = \"down\"\n"},
      // A homed mechanism needs a finite travel.
      {"max_rot",
       "tolerance_rot = 0.1\nhoming = \"stall\"\nmin_rot = 0.0\n"
       "homing_timeout_us = 100000\nhoming_stall_rps = 0.1\n"
       "homing_v = -1.0\nhoming_stall_us = 50000\n"
       "homing_stall_current_a = 20.0\nhoming_min_push_us = 50000\n"},
      {"min_rot", absolute + "max_rot = 0.5\n"},
      // Wrap needs a finite travel, or it winds a turret's cables.
      {"min_rot", "tolerance_rot = 0.1\nwrap_rot = 1.0\n"},
      {"absolute_sensor",
       "tolerance_rot = 0.1\nhoming = \"absolute\"\nmin_rot = 0.0\n"
       "max_rot = 0.5\n"},
      // 0.5 rot of travel, 0.1 rot per sensor rotation: ambiguous at boot.
      {"absolute_offset_rot", absolute + "min_rot = 0.0\nmax_rot = 0.5\n"
                                         "absolute_rot_per_sensor_rot = 0.1\n"},
      // Offset 0, ratio 1: readings in [-0.5, 0.5) map to [-0.5, 0.5), so a
      // travel to 0.8 is not inside it (0.6 would read as -0.4).
      {"absolute_offset_rot", absolute + "min_rot = 0.0\nmax_rot = 0.8\n"},
      // A negative ratio maps the window the other way round: still
      // offset +- 0.5, so a travel reaching -0.5 is outside it.
      {"absolute_offset_rot", absolute +
                                  "min_rot = -0.5\nmax_rot = 0.4\n"
                                  "absolute_rot_per_sensor_rot = -1.0\n"},
      // On the window's edge (offset 0.4 maps (-0.1, 0.9)): a hood resting
      // on its stop just below min_rot would read across the discontinuity.
      {"absolute_margin_rot", absolute + "absolute_offset_rot = 0.4\n"
                                         "min_rot = -0.1\nmax_rot = 0.8\n"},
      {"absolute_margin_rot",
       "tolerance_rot = 0.1\nhoming = \"absolute\"\nabsolute_sensor = \"cc\"\n"
       "min_rot = 0.0\nmax_rot = 0.4\n"},
      // At or above the motor's stator_limit_a (80 A by default): never met.
      {"homing_stall_current_a",
       "tolerance_rot = 0.1\nhoming = \"stall\"\nmin_rot = 0.0\n"
       "max_rot = 1.0\nhoming_timeout_us = 500000\nhoming_stall_rps = 0.1\n"
       "homing_v = -1.0\nhoming_stall_us = 50000\n"
       "homing_stall_current_a = 80.0\nhoming_min_push_us = 50000\n"
       "homing_min_travel_rot = 0.05\n"},
      {"homing_min_travel_rot",
       "tolerance_rot = 0.1\nhoming = \"stall\"\nmin_rot = 0.0\n"
       "max_rot = 1.0\nhoming_timeout_us = 500000\nhoming_stall_rps = 0.1\n"
       "homing_v = -1.0\nhoming_stall_us = 50000\n"
       "homing_stall_current_a = 20.0\nhoming_min_push_us = 50000\n"
       "homing_min_travel_rot = 1.0\n"},
      // No time left to find the stop before the timeout faults.
      {"homing_timeout_us",
       "tolerance_rot = 0.1\nhoming = \"stall\"\nmin_rot = 0.0\n"
       "max_rot = 1.0\nhoming_timeout_us = 100000\nhoming_stall_rps = 0.1\n"
       "homing_v = -1.0\nhoming_stall_us = 50000\n"
       "homing_stall_current_a = 20.0\nhoming_min_push_us = 50000\n"
       "homing_min_travel_rot = 0.05\n"},
  };
  for (const auto& [key, table] : bad) expect_refused(key, toml(table));
  {  // a negative ratio with the margin inside its window is accepted
    auto sim = Sim::FromToml(toml(absolute + "min_rot = -0.4\nmax_rot = 0.4\n"
                                             "absolute_rot_per_sensor_rot = "
                                             "-1.0\n"));
    EXPECT_NO_THROW(sim.add<PositionNode>("m"));
  }
  // Gravity from one source: the motor's slot g, or gravity_v, not both.
  const std::string slot_g = "slot0 = { g = 0.3 }\n";
  {
    auto sim = Sim::FromToml(toml(ok, slot_g));
    EXPECT_NO_THROW(sim.add<PositionNode>("m"));
  }
  expect_refused("gravity_v", toml(ok + "gravity_v = 0.5\n", slot_g));
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

// Stall homing: the hood moves down under homing_v, reaches its stop and
// stays still under load for homing_stall_us; that position becomes home
// (home_rot 0), and control = "position" then uses the position loop.
// REQ-SAFE-36
TEST(PositionMechanism, StallHomingSetsTheFrameWhereTheMechanismStopped) {
  Rig hood{"hood"};
  hood.SetMotor(2.3, -0.5, 5.0);  // moving down, light load
  hood.Start();
  hood.Drive(0.5, 60ms);
  hood.ExpectHoming(-1.0);
  hood.SetMotor(2.0, 0.0, 30.0);  // at the stop: still, high current
  hood.Drive(0.5, 30ms);          // less than homing_stall_us still
  hood.ExpectHoming(-1.0);
  hood.Drive(0.5, 40ms);
  EXPECT_TRUE(hood.mech().homed());
  EXPECT_NEAR(hood.mech().position_rot(), 0.0, 1e-9);
  EXPECT_EQ(hood.Applied().mode, Mode::kPosition);
  EXPECT_DOUBLE_EQ(hood.Applied().demand, 2.5);  // 0.5 above the stop
}

// REQ-SAFE-05: a stall-homing rezero refused for a late motor position
// (20 ms old, over RezeroAgeBound's 10 ms at 200 Hz, yet valid at the
// gateway) is not home: the push goes on, and the frame is taken as soon
// as a recent position arrives.
// REQ-SAFE-36
TEST(PositionMechanism, StallHomingKeepsPushingWhileTheRezeroIsRefused) {
  Rig hood{"hood"};
  hood.SetMotor(2.3, -0.5, 5.0);
  hood.Start();
  hood.Drive(0.5, 60ms);
  hood.hw.set_motor("hood", "hood", 2.0, 0.0, 20'000, 0, 0, 0, 30.0);
  hood.Drive(0.5, 70ms);  // still under load past homing_stall_us
  hood.ExpectHoming(-1.0);
  hood.SetMotor(2.0, 0.0, 30.0);
  hood.Drive(0.5, 10ms);
  EXPECT_TRUE(hood.mech().homed());
  EXPECT_NEAR(hood.mech().position_rot(), 0.0, 1e-9);
}

// Motion belongs to the push that made it: a push that moved the hood, then
// a released goal leaving it 0.2 rot above its stop, then a new push that
// starts slowly under load does not zero before homing_min_push_us.
// REQ-SAFE-36
TEST(PositionMechanism, StallHomingForgetsMotionFromAnEarlierPush) {
  Rig hood{"hood"};
  hood.SetMotor(2.5, -0.5, 5.0);
  hood.Start();
  hood.Drive(0.5, 30ms);
  hood.SetMotor(2.2, -0.5, 5.0);  // 0.3 rot of travel in this push
  hood.Drive(0.5, 30ms);
  hood.ExpectHoming(-1.0);
  hood.sim.step(150ms);             // the goal is lost: the push ends
  hood.SetMotor(2.2, -0.02, 25.0);  // a new push, slow under load
  hood.Drive(0.5, 150ms);  // three times homing_stall_us, under min push
  EXPECT_FALSE(hood.mech().homed());
  hood.ExpectHoming(-1.0);
}

// A position step faster than the motor can turn the hood (a frame jump)
// is not motion: the push starts over there, so a slow, loaded hood after
// the jump does not zero before homing_min_push_us.
// REQ-SAFE-36
TEST(PositionMechanism, StallHomingTreatsAFrameJumpAsANewPush) {
  Rig hood{"hood"};
  hood.SetMotor(3.0, -0.5, 5.0);
  hood.Start();
  hood.Drive(0.5, 30ms);
  hood.ExpectHoming(-1.0);
  // 1.1 rot in one 5 ms tick: 220 rot/s, past max_velocity_rps (100).
  hood.SetMotor(1.9, 0.0, 30.0);
  hood.Drive(0.5, 150ms);  // three times homing_stall_us, under min push
  EXPECT_FALSE(hood.mech().homed());
  hood.ExpectHoming(-1.0);
}

// Homing can succeed with no motion at all: a hood already resting on its
// stop at boot homes once the push has lasted homing_min_push_us and it has
// stayed still under load for homing_stall_us.
// REQ-SAFE-36
TEST(PositionMechanism, StallHomingFromRestOnTheStopHomesAfterTheMinimumPush) {
  Rig hood{"hood"};
  hood.SetMotor(2.0, 0.0, 30.0);
  hood.Start();
  hood.Drive(0.5, 150ms);
  hood.ExpectHoming(-1.0);
  hood.Drive(0.5, 150ms);  // past 200 ms of push plus 50 ms still
  EXPECT_TRUE(hood.mech().homed());
  EXPECT_DOUBLE_EQ(hood.Applied().demand, 2.5);
}

// A mechanism that never moves and shows no load (a slipping belt, an
// unpowered motor) is no evidence of a stop: homing times out and faults,
// it never homes.
// REQ-SAFE-36
TEST(PositionMechanism, StallHomingThatNeverMovesFaultsInsteadOfHoming) {
  Rig hood{"hood"};
  hood.SetMotor(2.0, 0.0, 0.0);
  hood.Start();
  hood.Drive(0.5, 450ms);
  hood.ExpectHoming(-1.0);
  hood.Drive(0.5, 100ms);
  EXPECT_FALSE(hood.mech().homed());
  EXPECT_EQ(hood.mech().fault(), PositionFault::kHomingTimeout);
  hood.ExpectNeutral();
}

// Slow to get going (under load, below homing_stall_rps) is not a stop: no
// zero before homing_min_push_us of push unless it has moved.
// REQ-SAFE-36
TEST(PositionMechanism, StallHomingDoesNotZeroOnASlowStart) {
  Rig hood{"hood"};
  hood.SetMotor(2.3, -0.02, 25.0);  // breaking away: slow, loaded
  hood.Start();
  hood.Drive(0.5, 150ms);  // three times homing_stall_us, under min push
  hood.ExpectHoming(-1.0);
  hood.SetMotor(2.2, -0.5, 5.0);  // now it moves
  hood.Drive(0.5, 20ms);
  hood.SetMotor(2.0, 0.0, 30.0);  // and stops at the hard stop
  hood.Drive(0.5, 80ms);
  EXPECT_TRUE(hood.mech().homed());
  EXPECT_DOUBLE_EQ(hood.Applied().demand, 2.5);
}

// One subsystem of kRobotToml on the physics simulator (sim_gateway
// --physics). Each Tick() the physics backend runs what the gateway applied
// last tick, and its motor sample, and the home switch's reading once
// SetHomeSwitch() binds one, reach the node through SimHardware (an input
// SimHardware has never been given reads false: closed, for a normally open
// switch).
struct PhysicsRig {
  explicit PhysicsRig(std::string_view subsystem)
      : rig{subsystem},
        config{rig.sim.node(subsystem)
                   .extension<HardwareLink>()
                   .config()
                   .hardware} {
    config.commissioned = true;
    for (std::size_t i = 0; i < config.motors.size(); ++i) {
      if (config.motors[i].name == subsystem) motor_index = i;
    }
    requests.resize(config.motors.size());
    state.motor_count = static_cast<std::uint16_t>(requests.size());
  }
  hardware::MotorConfig& motor() { return config.motors.at(motor_index); }
  void SetHomeSwitch(const hardware::sim::HomeSwitch& home) {
    for (std::size_t i = 0; i < config.digital_inputs.size(); ++i) {
      if (config.digital_inputs[i].name == home.input) home_input = i;
    }
    physics.SetHomeSwitches({home});
  }
  // Configures the physics backend and hands the node its first reading,
  // before the Driver Station enables it.
  void Configure() {
    ASSERT_TRUE(physics.Configure(config));
    Relay();
  }
  void Tick(double goal_rot) {
    for (std::size_t i = 0; i < requests.size(); ++i) {
      requests[i] = {.id = config.motors[i].id};
    }
    requests[motor_index] = rig.Applied();
    ASSERT_TRUE(physics.Apply(requests));
    Relay();
    rig.Drive(goal_rot, 5ms);
  }
  void Relay() {
    state.sample_time_us = now_us += 5000;
    physics.Read(state);
    const hardware::MotorSample& sample = state.motors[motor_index];
    rig.SetMotor(sample.position_rot, sample.velocity_rps,
                 sample.stator_current_a);
    if (home_input) {
      rig.hw.set_digital_input(rig.name,
                               config.digital_inputs[*home_input].name,
                               state.digital_inputs[*home_input].value);
    }
  }
  double position_rot() const { return state.motors[motor_index].position_rot; }

  Rig<> rig;
  hardware::Config config;
  std::size_t motor_index = 0;
  std::optional<std::size_t> home_input;
  hardware::sim::PhysicsSimBackend physics;
  std::vector<hardware::MotorRequest> requests;
  hardware::State state{};
  std::uint64_t now_us = 0;
};

// Stall homing on the physics simulator's motor (sim_gateway --physics):
// the hood's 1 V push runs it down into a hard stop 0.3 rot below where it
// starts (the physics soft limit), where the winding model draws 1 V / R,
// about 30 A on a Kraken X60, past its realistic 20 A homing_stall_current_a,
// so it homes there well inside homing_timeout_us.
// REQ-SAFE-36
TEST(PositionMechanism, StallHomingCompletesOnThePhysicsMotor) {
  PhysicsRig hood{"hood"};
  ASSERT_EQ(hood.motor().motor_type.value, hardware::MotorType::kKrakenX60);
  hood.motor().soft_limits = true;  // the physical stop
  hood.motor().reverse_limit_rot = -0.3;
  hood.motor().forward_limit_rot = 10.0;
  hood.Configure();
  hood.rig.Start();
  for (int i = 0; i < 100 && !hood.rig.mech().homed(); ++i) hood.Tick(0.5);
  ASSERT_TRUE(hood.rig.mech().homed())
      << "fault " << static_cast<int>(hood.rig.mech().fault());
  EXPECT_LT(hood.now_us, 400'000u);  // homing_timeout_us is 500 ms
  EXPECT_NEAR(hood.position_rot(), -0.3, 1e-9);
  EXPECT_NEAR(hood.rig.mech().position_rot(), 0.0, 1e-9);
}

// Limit-switch homing on the physics simulator's home switch: the carriage
// starts at 0, above its switch, which closes at -0.25 and below. Its 1 V
// push runs it down with the switch open, and it zeroes past the switch
// point, by no more than it travels in the few ticks home_samples and the
// relay take (about 0.04 rot each at about 8 rot/s), not where it started.
// REQ-SAFE-36
TEST(PositionMechanism, LimitSwitchHomingFindsThePhysicsHomeSwitch) {
  PhysicsRig carriage{"carriage"};
  carriage.SetHomeSwitch(
      {.input = "carriage_home", .motor = "carriage", .closed_at_rot = -0.25});
  carriage.Configure();
  carriage.rig.Start();
  carriage.Tick(0.5);
  carriage.Tick(0.5);
  carriage.rig.ExpectHoming(-1.0);
  for (int i = 0; i < 100 && !carriage.rig.mech().homed(); ++i) {
    carriage.Tick(0.5);
  }
  ASSERT_TRUE(carriage.rig.mech().homed())
      << "fault " << static_cast<int>(carriage.rig.mech().fault());
  // With the relay paused, the node reads the last relayed sample again: the
  // frame's zero, in the motor's hardware frame, is that position less the
  // node's.
  carriage.rig.Drive(0.5, 10ms);
  const double zero_rot =
      carriage.position_rot() - carriage.rig.mech().position_rot();
  EXPECT_LE(zero_rot, -0.25);
  EXPECT_GT(zero_rot, -0.25 - 0.15);
  EXPECT_EQ(carriage.rig.mech().fault(), PositionFault::kNone);
}

// The first move after limit-switch homing starts from rest. The physics
// carriage reaches its switch (-0.25, no hard stop there) at the homing
// speed, 1 V / (12 V / 100 rps) = 8.3 rps; a Motion Magic profile started at
// that speed with acceleration_rps2 = 2 would run on v^2 / (2 a), about 17
// rotations, past the switch before turning back. Settling in neutral
// instead stops it near the switch: it pushes on for the ticks home_samples
// and the relay take (the zero lands within 0.15 rot past the switch, as
// above), then the brake's decay (time constant 20 ms, 25% of the speed per
// 5 ms step) carries it v * dt * 0.75 / 0.25 = 0.125 rot further. So it
// never goes more than 0.3 rot past the switch, and once at rest the profile
// takes it to its goal.
// REQ-SAFE-36
TEST(PositionMechanism, LimitSwitchHomingSettlesBeforeTheFirstMove) {
  PhysicsRig carriage{"carriage"};
  carriage.SetHomeSwitch(
      {.input = "carriage_home", .motor = "carriage", .closed_at_rot = -0.25});
  carriage.motor().acceleration_rps2 = 2.0;
  carriage.motor().slots[0] = {.p = 1.0, .d = 0.05, .v = 0.12};
  carriage.Configure();
  carriage.rig.Start();
  double lowest_rot = 0.0;
  // 3 s: the move from rest, 0.9 rot at 2 rot/s^2, takes about 1.3 s.
  for (int i = 0; i < 600 && !carriage.rig.mech().at_goal(); ++i) {
    carriage.Tick(0.5);
    lowest_rot = std::min(lowest_rot, carriage.position_rot());
  }
  EXPECT_GE(lowest_rot, -0.25 - 0.3);
  EXPECT_TRUE(carriage.rig.mech().at_goal())
      << "phase " << static_cast<int>(carriage.rig.mech().phase()) << " fault "
      << static_cast<int>(carriage.rig.mech().fault()) << " at "
      << carriage.position_rot();
}

// While it settles the mechanism is still homing: neutral, not homed, and
// bounded by homing_timeout_us. One that never comes to rest (back-driven
// past its switch at 0.5 rot/s, ten times homing_stall_rps) faults with the
// homing timeout and never takes a goal. Invalid feedback on the way, which
// SimHardware sends as velocity 0, is no reading, not rest.
// REQ-SAFE-36
TEST(PositionMechanism, LimitSwitchHomingThatNeverSettlesTimesOut) {
  Rig carriage{"carriage"};
  carriage.SetMotor(1.0, -0.5);  // moving down: no stall
  carriage.hw.set_digital_input("carriage", "carriage_home", true);  // open
  carriage.Start();
  carriage.Drive(0.5, 30ms);
  carriage.ExpectHoming(-1.0);
  carriage.hw.set_digital_input("carriage", "carriage_home", false);
  carriage.Drive(0.5, 30ms);  // the switch has set the frame
  EXPECT_EQ(carriage.mech().phase(), PositionPhase::kHoming);
  EXPECT_FALSE(carriage.mech().homed());
  carriage.ExpectNeutral();
  bool moved = false;
  for (int i = 0; i < 100; ++i) {  // 500 ms, homing_timeout_us
    carriage.hw.set_connected("carriage", "carriage", i < 40 || i >= 50);
    carriage.Drive(0.5, 5ms);
    moved |= carriage.Applied().mode != Mode::kNeutral;
  }
  EXPECT_FALSE(moved);
  EXPECT_FALSE(carriage.mech().homed());
  EXPECT_EQ(carriage.mech().fault(), PositionFault::kHomingTimeout);
  carriage.ExpectNeutral();
}

// A Driver Station disable while settling ends the homing run, as it does
// during the push: neutral, unhomed, and the timeout starts over, so 400 ms
// settling before the disable and 300 ms after it (more than
// homing_timeout_us in all) does not fault. Re-enabled, it still takes no
// goal until it has come to rest.
// REQ-SAFE-36
TEST(PositionMechanism, DisableWhileSettlingEndsTheHomingRun) {
  Rig carriage{"carriage"};
  carriage.SetMotor(1.0, -0.5);
  carriage.hw.set_digital_input("carriage", "carriage_home", true);
  carriage.Start();
  carriage.Drive(0.5, 30ms);
  carriage.hw.set_digital_input("carriage", "carriage_home", false);
  carriage.Drive(0.5, 400ms);  // settling, still moving
  ASSERT_EQ(carriage.mech().phase(), PositionPhase::kHoming);

  carriage.hw.set_driver_station(DsPacket(ds::kTeleop));
  carriage.sim.step(20ms);
  EXPECT_EQ(carriage.mech().phase(), PositionPhase::kUnhomed);
  EXPECT_FALSE(carriage.mech().homed());
  carriage.ExpectNeutral();

  carriage.Start();
  carriage.Drive(0.5, 300ms);
  EXPECT_EQ(carriage.mech().fault(), PositionFault::kNone);
  EXPECT_EQ(carriage.mech().phase(), PositionPhase::kHoming);
  EXPECT_FALSE(carriage.mech().homed());
  carriage.ExpectNeutral();

  carriage.SetMotor(0.9);  // at rest
  carriage.Drive(0.5, 30ms);
  EXPECT_TRUE(carriage.mech().homed());
  EXPECT_EQ(carriage.Applied().mode, Mode::kMotionMagic);
}

// A mechanism that keeps moving (no stop within reach) times out and faults.
// REQ-SAFE-36
TEST(PositionMechanism, StallHomingTimesOutWhileStillMoving) {
  Rig hood{"hood"};
  hood.SetMotor(2.3, -0.5, 5.0);
  hood.Start();
  hood.Drive(0.5, 550ms);
  EXPECT_FALSE(hood.mech().homed());
  EXPECT_EQ(hood.mech().fault(), PositionFault::kHomingTimeout);
  hood.ExpectNeutral();
}

// Losing the goal pauses homing's push, not its timeout: the push stops at
// once, and a goal back after the timeout has passed faults.
// REQ-SAFE-36
TEST(PositionMechanism, HomingPausesWithoutAGoalButKeepsItsTimeout) {
  Rig hood{"hood"};
  hood.SetMotor(2.3, -0.5, 5.0);  // moving: no stall
  hood.Start();
  hood.Drive(0.5, 300ms);
  hood.ExpectHoming(-1.0);
  hood.sim.step(300ms);  // stale after command_timeout_us
  EXPECT_EQ(hood.mech().phase(), PositionPhase::kUnhomed);
  hood.ExpectNeutral();
  hood.Drive(0.5, 20ms);
  EXPECT_EQ(hood.mech().fault(), PositionFault::kHomingTimeout);
  hood.ExpectNeutral();
}

// hold() has no position to hold in an unknown frame: an unhomed mechanism
// stays neutral and does not start homing.
// REQ-SAFE-36
TEST(PositionMechanism, HoldNeverStartsHoming) {
  Rig hood{"hood"};
  hood.node.hold_mode = true;
  hood.SetMotor(2.3, -0.5, 5.0);
  hood.Start();
  hood.Drive(0.5, 50ms);
  EXPECT_EQ(hood.mech().phase(), PositionPhase::kUnhomed);
  EXPECT_FALSE(hood.mech().driving());
  hood.ExpectNeutral();
}

// Limit-switch homing upward (positive homing_v, home_rot at max_rot 2.0,
// a switch that reads true when closed): the closed switch sets the frame
// (here at a stop, so the top is at rest and settles at once), a closed
// switch at home is fine, and one closed well below home (into the travel)
// is stuck.
// REQ-SAFE-36
TEST(PositionMechanism, LimitSwitchHomingUpwardAndAStuckSwitch) {
  Rig top{"top"};
  top.SetMotor(5.0, 0.5);  // moving up: no stall
  top.Start();
  top.Drive(1.0, 30ms);
  top.ExpectHoming(1.0);
  top.hw.set_digital_input("top", "top_switch", true);  // closed
  top.SetMotor(5.0);                                    // and stopped
  top.Drive(1.0, 20ms);  // home_samples (2) closed, and slow, readings
  EXPECT_TRUE(top.mech().homed());
  EXPECT_EQ(top.mech().fault(), PositionFault::kNone);  // closed at home
  EXPECT_EQ(top.Applied().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(top.Applied().demand, 4.0);  // 1.0 is 1 below raw 5.0
  top.SetMotor(4.0, 0.0);  // 1.0 below home, switch still closed
  top.Drive(1.0, 20ms);
  EXPECT_EQ(top.mech().fault(), PositionFault::kSwitchStuck);
  top.ExpectNeutral();
}

// A switch reading counts only while its sample is valid
// (DigitalInput::connected()). An input that stops answering, or a gateway
// still configuring, sends an invalid sample; SimHardware's reads false,
// which is closed for the carriage's normally open switch: it must not home
// the carriage where it is, an invalid reading breaks a run of closed ones
// as a disconnected motor ends a homing push's evidence, and on a homed
// carriage it is not a stuck switch.
// REQ-SAFE-36
TEST(PositionMechanism, LimitSwitchHomingIgnoresAnInvalidSwitchSample) {
  Rig carriage{"carriage"};
  carriage.SetMotor(1.0, -0.5);  // moving down: no stall
  carriage.hw.set_digital_input("carriage", "carriage_home", true);  // open
  carriage.Start();
  carriage.Drive(0.5, 30ms);
  carriage.ExpectHoming(-1.0);

  carriage.hw.set_connected("carriage", "carriage_home", false);
  carriage.Drive(0.5, 50ms);  // ten invalid "closed" readings; home_samples 2
  carriage.ExpectHoming(-1.0);
  EXPECT_EQ(carriage.mech().fault(), PositionFault::kNone);

  // Valid closed, invalid, valid closed: not two consecutive closed readings.
  // The node reads each State on its tick after SimHardware publishes it, so
  // these ticks read: invalid, valid closed, invalid twice, valid closed.
  carriage.hw.set_digital_input("carriage", "carriage_home", false);  // closed
  carriage.hw.set_connected("carriage", "carriage_home", true);
  carriage.Drive(0.5, 5ms);
  carriage.hw.set_connected("carriage", "carriage_home", false);
  carriage.Drive(0.5, 10ms);
  carriage.hw.set_connected("carriage", "carriage_home", true);
  carriage.Drive(0.5, 10ms);
  carriage.ExpectHoming(-1.0);
  carriage.Drive(0.5, 5ms);  // the second consecutive valid closed reading
  EXPECT_NEAR(carriage.mech().position_rot(), 0.0, 1e-9);  // the frame
  EXPECT_FALSE(carriage.mech().homed());  // still moving: it settles first
  carriage.SetMotor(1.0);                 // at rest, read on the next tick
  carriage.Drive(0.5, 15ms);              // then home_samples (2) slow readings
  ASSERT_TRUE(carriage.mech().homed());

  // Homed and 0.5 into its travel, the switch open, then invalid.
  carriage.SetMotor(1.5);
  carriage.hw.set_digital_input("carriage", "carriage_home", true);
  carriage.Drive(0.5, 20ms);
  EXPECT_EQ(carriage.mech().phase(), PositionPhase::kAtGoal);
  carriage.hw.set_connected("carriage", "carriage_home", false);
  carriage.Drive(0.5, 20ms);
  EXPECT_EQ(carriage.mech().fault(), PositionFault::kNone);
  EXPECT_EQ(carriage.mech().phase(), PositionPhase::kAtGoal);
  EXPECT_EQ(carriage.Applied().mode, Mode::kMotionMagic);
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

// REQ-SAFE-05: absolute homing pairs the motor's position with the
// CANcoder's, so it takes the frame only from samples acquired within
// RezeroAgeBound (10 ms at 200 Hz) of each other. With the CANcoder 30 ms
// behind (late, yet valid at the gateway) a wrist turning at 1 rot/s would
// take a frame 0.03 rot off: it stays unhomed and neutral, and homes once
// the samples line up.
// REQ-SAFE-36
TEST(PositionMechanism, AbsoluteHomingWaitsForAlignedSamples) {
  Rig wrist{"wrist"};
  wrist.SetMotor(5.0, 1.0);
  wrist.hw.set_cancoder("wrist", "angle", 0.1, 1.0, 30'000, 0);
  wrist.Start();
  wrist.Drive(0.35, 80ms);  // the bridge's clock mapping converges in it
  EXPECT_EQ(wrist.mech().phase(), PositionPhase::kUnhomed);
  wrist.ExpectNeutral();

  wrist.hw.set_cancoder("wrist", "angle", 0.1, 1.0, 4'000, 0);
  wrist.Drive(0.35, 20ms);
  EXPECT_TRUE(wrist.mech().homed());
  EXPECT_NEAR(wrist.mech().position_rot(), 0.35, 1e-9);
  EXPECT_EQ(wrist.Applied().mode, Mode::kMotionMagic);
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
