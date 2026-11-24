// VelocityMechanism on a simulated robot (component tier): a node drives it
// from its teleop lane, as a robot node does, with SimHardware as the bridge.
//
// Node test matrix (docs/TESTING.md §7) for the library: 1, 2, 3, 7, 8, 9 and
// 12 below. n/a 4, 5, 6: released, never-sent and Driver-Station-lost lanes
// reach the library exactly as a stale one does, as stop(); the owning
// node's matrix covers its Target. n/a 10, 11: lane selection and
// generations are the owner's talos::Target.

#include "talOS/mechanisms/velocity_mechanism.h"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
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
namespace detail {
double StepSpinDown(double goal_rps, double previous_rps, double max_rps,
                    double spin_down_rps2, double dt_s);
bool StepVelocityReadiness(bool was_at_speed, int& entry_ticks,
                           double velocity_rps, double goal_rps,
                           double entry_rps, double exit_rps,
                           int entry_tick_count, bool velocity_recent);
}  // namespace detail
namespace {

using namespace std::chrono_literals;
using hardware::Mode;
using testing::DsPacket;
using testing::PublishTarget;
using testing::Sim;
using testing::SimHardware;
namespace ds = driver_station;

// Exit band: max(2, 0.1 * |goal|), 6 rps at 60. The neighbor's wheel stays
// connected, so a lost flywheel is the loss of one device, as on a robot,
// and only the mechanism's own feedback gate stops it; losing every device
// is the gateway's fault (SimHardwareTotalLoss), which disables the robot.
constexpr const char* kRobotToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.shooter.motors.flywheel]
can_id = 1
max_velocity_rps = 80.0
[subsystems.shooter.flywheel]
at_speed_entry_rps = 2.0
at_speed_exit_frac = 0.1
at_speed_entry_tick_count = 3
spin_down_rps2 = 100.0

[subsystems.neighbor.motors.wheel]
can_id = 2
)";

// A robot node around one VelocityMechanism, driven by its teleop lane.
struct Shooter {
  explicit Shooter(Node& node) : flywheel{node, "flywheel"}, target{node} {}
  void update() {
    if (const control::VelocityTarget* t = target.get()) {
      flywheel.set_goal(t->velocity_rps(), t->feedforward_v());
    } else {
      flywheel.stop();
    }
    flywheel.update();
  }
  VelocityMechanism flywheel;
  Target<control::VelocityTarget> target;
};

class VelocityMechanismTest : public ::testing::Test {
 protected:
  VelocityMechanismTest() {
    hw.enable();
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(10ms);
  }
  // The teleop lane, republished every 5 ms (one tick) for `duration`.
  void Drive(double velocity_rps, std::chrono::milliseconds duration,
             double feedforward_v = 0) {
    for (auto t = 0ms; t < duration; t += 5ms) {
      PublishTarget(sim, "shooter", "teleop",
                    control::VelocityTarget{velocity_rps, feedforward_v});
      sim.step(5ms);
    }
  }
  // Feedback, published before the next tick reads it.
  void SetVelocity(double velocity_rps) {
    hw.set_motor("shooter", "flywheel", 0.0, velocity_rps);
    sim.step(5ms);
  }
  hardware::MotorRequest Applied() const {
    return hw.applied_motor("shooter", "flywheel");
  }
  void ExpectNeutral() {
    EXPECT_EQ(Applied().mode, Mode::kNeutral);
    EXPECT_FALSE(flywheel().state().enabled());
    EXPECT_DOUBLE_EQ(flywheel().state().requested_rps(), 0.0);
    EXPECT_FALSE(flywheel().at_speed());
  }
  VelocityMechanism& flywheel() { return shooter.flywheel; }

  Sim sim = Sim::FromToml(kRobotToml);
  Shooter& shooter = sim.add<Shooter>("shooter");
  SimHardware hw{sim};
};

// REQ-SAFE-01: a live goal drives the velocity loop with its feedforward
// and reaches at_speed.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, DrivesAtALiveLane) {
  hw.ideal_motors();
  Drive(60.0, 40ms, 1.5);
  EXPECT_EQ(Applied().mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(Applied().demand, 60.0);
  EXPECT_DOUBLE_EQ(Applied().feedforward_v, 1.5);
  EXPECT_TRUE(flywheel().at_speed());
  const control::VelocityState state = flywheel().state();
  EXPECT_DOUBLE_EQ(state.velocity_rps(), 60.0);
  EXPECT_DOUBLE_EQ(state.requested_rps(), 60.0);
  EXPECT_TRUE(state.enabled());
  EXPECT_TRUE(state.connected());
}

// Goals clamp to the motor's own max_velocity_rps, and readiness is judged
// against the clamped goal.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, ClampsToTheMotorMaxAndIsReadyThere) {
  hw.ideal_motors();
  Drive(120.0, 40ms);
  EXPECT_DOUBLE_EQ(Applied().demand, 80.0);
  EXPECT_DOUBLE_EQ(flywheel().goal_rps(), 80.0);
  EXPECT_TRUE(flywheel().at_speed());
}

// Entry takes at_speed_entry_tick_count consecutive ticks in the entry band.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, AtSpeedNeedsConsecutiveEntryTicks) {
  SetVelocity(59.0);  // inside at_speed_entry_rps
  Drive(60.0, 10ms);  // two ticks
  EXPECT_FALSE(flywheel().at_speed());
  EXPECT_EQ(flywheel().phase(), VelocityPhase::kSpinningUp);
  Drive(60.0, 5ms);  // the third
  EXPECT_TRUE(flywheel().at_speed());
}

// Once at speed it stays inside the exit band, leaves past it, and must
// re-enter the entry band for the full count.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, ExitBandHysteresis) {
  SetVelocity(60.0);
  Drive(60.0, 20ms);
  ASSERT_TRUE(flywheel().at_speed());
  SetVelocity(55.0);  // 5 off: outside entry, inside exit (6)
  Drive(60.0, 20ms);
  EXPECT_TRUE(flywheel().at_speed());
  SetVelocity(53.0);  // 7 off: past exit
  Drive(60.0, 5ms);
  EXPECT_FALSE(flywheel().at_speed());
  SetVelocity(55.0);  // inside exit but not entry: stays out
  Drive(60.0, 30ms);
  EXPECT_FALSE(flywheel().at_speed());
  SetVelocity(59.0);
  Drive(60.0, 5ms);  // at most two ticks in the entry band
  EXPECT_FALSE(flywheel().at_speed());
  Drive(60.0, 10ms);
  EXPECT_TRUE(flywheel().at_speed());
}

// A changed goal restarts readiness, even inside the entry band.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, ChangedGoalRestartsReadiness) {
  SetVelocity(60.0);
  Drive(60.0, 20ms);
  ASSERT_TRUE(flywheel().at_speed());
  Drive(61.0, 5ms);
  EXPECT_FALSE(flywheel().at_speed());
  Drive(61.0, 10ms);
  EXPECT_TRUE(flywheel().at_speed());
}

// Changing an out-of-range request that still clamps to the same goal_rps_
// does not spuriously reset readiness; moving back inside range does.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, UnchangedClampedGoalKeepsReadiness) {
  SetVelocity(80.0);
  Drive(100.0, 20ms);
  ASSERT_DOUBLE_EQ(flywheel().goal_rps(), 80.0);
  ASSERT_TRUE(flywheel().at_speed());
  Drive(120.0, 5ms);
  EXPECT_DOUBLE_EQ(flywheel().goal_rps(), 80.0);
  EXPECT_TRUE(flywheel().at_speed());
  Drive(79.0, 5ms);
  EXPECT_DOUBLE_EQ(flywheel().goal_rps(), 79.0);
  EXPECT_FALSE(flywheel().at_speed());

  SetVelocity(-80.0);
  Drive(-100.0, 20ms);
  ASSERT_DOUBLE_EQ(flywheel().goal_rps(), -80.0);
  ASSERT_TRUE(flywheel().at_speed());
  Drive(-120.0, 5ms);
  EXPECT_DOUBLE_EQ(flywheel().goal_rps(), -80.0);
  EXPECT_TRUE(flywheel().at_speed());
}

// A CAN gap freezes the flywheel's velocity: the gateway keeps reporting the
// last value, in band, with a growing age, and keeps it valid until that age
// reaches command_timeout_us (100 ms). at_speed must not ride on it: a
// sample at most four status periods old at its State (ReadinessAgeBound,
// 20 ms at the default 200 Hz) keeps it, one 1 us older drops it on that
// tick while the goal still drives, an unknown age (UINT32_MAX, which the
// gateway also marks invalid) drops it, and a fresh sample must then qualify
// again for the full entry count.
// REQ-DEV-22
TEST_F(VelocityMechanismTest, AtSpeedDropsOnAVelocitySampleOlderThanTheBound) {
  EXPECT_FALSE(flywheel().at_speed()) << "no velocity sample yet";
  struct Row {
    std::uint32_t velocity_age_us;
    bool at_speed;
    bool driving;
  };
  for (const Row& row : {Row{20'000, true, true}, Row{20'001, false, true},
                         Row{UINT32_MAX, false, false}}) {
    SCOPED_TRACE(row.velocity_age_us);
    SetVelocity(60.0);
    Drive(60.0, 20ms);
    ASSERT_TRUE(flywheel().at_speed());
    hw.set_motor("shooter", "flywheel", 0.0, 60.0, 0, row.velocity_age_us);
    Drive(60.0, 10ms);
    // The neighbor keeps the robot enabled: the flywheel's gate decides.
    ASSERT_TRUE(robot_enabled(sim.node("shooter")));
    EXPECT_EQ(flywheel().at_speed(), row.at_speed);
    EXPECT_EQ(flywheel().driving(), row.driving);
    if (row.driving) {
      EXPECT_EQ(Applied().mode, Mode::kVelocity);
      EXPECT_DOUBLE_EQ(Applied().demand, 60.0);
    }
  }
  SetVelocity(60.0);  // fresh again: the entry count starts over
  Drive(60.0, 10ms);
  EXPECT_FALSE(flywheel().at_speed());
  Drive(60.0, 5ms);
  EXPECT_TRUE(flywheel().at_speed());
}

// The velocity age bound follows [robot] status_hz (four status periods), not
// a hardcoded 20 ms default: at 100 Hz, 40'000 us keeps at_speed and 40'001 us
// drops it while still driving.
// REQ-DEV-22
TEST(VelocityMechanismStatusRate, ReadinessAgeBoundFollowsConfiguredStatusHz) {
  auto sim = Sim::FromToml(R"(
[robot]
period_us = 5000
command_timeout_us = 100000
status_hz = 100.0

[subsystems.shooter.motors.flywheel]
can_id = 1
max_velocity_rps = 80.0
[subsystems.shooter.flywheel]
at_speed_entry_rps = 2.0
at_speed_exit_frac = 0.1
at_speed_entry_tick_count = 3
spin_down_rps2 = 100.0

[subsystems.neighbor.motors.wheel]
can_id = 2
)");
  auto& shooter = sim.add<Shooter>("shooter");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(10ms);

  const auto drive = [&](std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 5ms) {
      PublishTarget(sim, "shooter", "teleop",
                    control::VelocityTarget{60.0, 0.0});
      sim.step(5ms);
    }
  };

  hw.set_motor("shooter", "flywheel", 0.0, 60.0);
  sim.step(5ms);
  drive(20ms);
  ASSERT_TRUE(shooter.flywheel.at_speed());

  hw.set_motor("shooter", "flywheel", 0.0, 60.0, 0, 40'000);
  drive(10ms);
  ASSERT_TRUE(robot_enabled(sim.node("shooter")));
  EXPECT_TRUE(shooter.flywheel.at_speed());
  EXPECT_TRUE(shooter.flywheel.driving());

  hw.set_motor("shooter", "flywheel", 0.0, 60.0, 0, 40'001);
  drive(10ms);
  ASSERT_TRUE(robot_enabled(sim.node("shooter")));
  EXPECT_FALSE(shooter.flywheel.at_speed());
  EXPECT_TRUE(shooter.flywheel.driving());
}

// The velocity's age comes from the State, so at_speed does not wait for
// the bridge's clock mapping: three in-band ticks right after the first
// States reach it while HardwareLink::sample_time() is still unknown.
// REQ-DEV-22
TEST(VelocityMechanismMapping, AtSpeedBeforeTheClockMappingConverges) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& shooter = sim.add<Shooter>("shooter");
  const auto& link = sim.node("shooter").extension<HardwareLink>();
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  hw.set_motor("shooter", "flywheel", 0.0, 60.0);
  sim.step(5ms);
  for (int tick = 0; tick < 3; ++tick) {
    PublishTarget(sim, "shooter", "teleop", control::VelocityTarget{60.0, 0});
    sim.step(5ms);
  }
  ASSERT_FALSE(link.sample_time()) << "the mapping has converged already";
  EXPECT_TRUE(shooter.flywheel.at_speed());
}

// A live goal that drops winds down at spin_down_rps2; losing the goal is
// neutral at once.
TEST_F(VelocityMechanismTest, SpinDownLimitsALiveDropButNotNeutral) {
  hw.ideal_motors();
  Drive(60.0, 40ms);
  ASSERT_DOUBLE_EQ(Applied().demand, 60.0);
  Drive(0.0, 100ms);  // 100 rps/s for 0.1 s: about 50 rps
  EXPECT_EQ(Applied().mode, Mode::kVelocity);
  EXPECT_NEAR(Applied().demand, 50.0, 1.0) << "spin_down_rps2 = 100";
  EXPECT_FALSE(flywheel().at_speed());
  EXPECT_DOUBLE_EQ(flywheel().goal_rps(), 0.0);
  sim.step(110ms);  // stale
  ExpectNeutral();
}

// Negative dt during spin-down must never accelerate away from goal or past
// +-max_rps, and a NaN velocity while at_speed must drop readiness immediately.
// REQ-SAFE-37
TEST(VelocityMechanismGuards, NegativeDtSpinDownAndNaNReadiness) {
  // Positive speed winding down with negative dt must hold previous, not ramp
  // up past max_rps.
  EXPECT_DOUBLE_EQ(
      detail::StepSpinDown(/*goal_rps=*/0.0, /*previous_rps=*/80.0,
                           /*max_rps=*/80.0,
                           /*spin_down_rps2=*/100.0, /*dt_s=*/-0.5),
      80.0);
  // Negative speed winding down with negative dt must hold previous, not ramp
  // more negative past -max_rps.
  EXPECT_DOUBLE_EQ(
      detail::StepSpinDown(/*goal_rps=*/0.0,
                           /*previous_rps=*/-80.0,
                           /*max_rps=*/80.0,
                           /*spin_down_rps2=*/100.0, /*dt_s=*/-0.5),
      -80.0);
  // Out-of-bounds previous command is clamped to [-max_rps, max_rps].
  EXPECT_DOUBLE_EQ(detail::StepSpinDown(/*goal_rps=*/0.0,
                                        /*previous_rps=*/120.0,
                                        /*max_rps=*/80.0,
                                        /*spin_down_rps2=*/100.0, /*dt_s=*/0.0),
                   80.0);
  // Opposite-sign tiny velocities whose product underflows to -0.0 still
  // engage spin-down rather than jumping instantaneously across zero on dt=0.
  EXPECT_DOUBLE_EQ(detail::StepSpinDown(/*goal_rps=*/-1e-200,
                                        /*previous_rps=*/1e-200,
                                        /*max_rps=*/80.0,
                                        /*spin_down_rps2=*/100.0, /*dt_s=*/0.0),
                   1e-200);

  // NaN or Inf velocity while latched at_speed drops readiness and resets
  // entry_ticks.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad_vel : {nan, inf, -inf}) {
    int entry_ticks = 3;
    EXPECT_FALSE(detail::StepVelocityReadiness(
        /*was_at_speed=*/true, entry_ticks, bad_vel, /*goal_rps=*/60.0,
        /*entry_rps=*/2.0, /*exit_rps=*/6.0, /*entry_tick_count=*/3,
        /*velocity_recent=*/true));
    EXPECT_EQ(entry_ticks, 0);
  }
}

// Matrix case 2: while disabled the mechanism reports itself disabled,
// asks for nothing and is not at speed. The neutral output is the devices
// flush's (REQ-SAFE-02, proved in talOS/devices).
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, DisabledRobotGoesNeutral) {
  hw.ideal_motors();
  Drive(60.0, 40ms);
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  hw.enable(false);
  Drive(60.0, 30ms);
  ExpectNeutral();
}

// Matrix case 3: once the lane is stale the mechanism is stopped and reports
// nothing asked. The neutral request is the devices flush's (Target::get
// returns no target: REQ-SAFE-41, proved in talOS/devices).
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, StaleLaneGoesNeutral) {
  hw.ideal_motors();
  Drive(60.0, 40ms);
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  sim.step(110ms);
  ExpectNeutral();
  EXPECT_EQ(flywheel().phase(), VelocityPhase::kStopped);
}

// REQ-SAFE-37
TEST_F(VelocityMechanismTest, DisconnectedFeedbackGoesNeutral) {
  hw.set_connected("shooter", "flywheel", false);
  Drive(60.0, 30ms);
  // The neighbor keeps the robot enabled: the flywheel's gate decides.
  ASSERT_TRUE(robot_enabled(sim.node("shooter")));
  ExpectNeutral();
  EXPECT_FALSE(flywheel().state().connected());
}

// REQ-SAFE-04
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, NonFiniteDemandGoesNeutral) {
  Drive(std::numeric_limits<double>::quiet_NaN(), 30ms);
  ExpectNeutral();
  Drive(60.0, 30ms, std::numeric_limits<double>::infinity());
  ExpectNeutral();
}

// Requests and update() are refused outside update() (the constructor, an
// every() timer, a message callback, and off the loop thread while update()
// is running), throwing std::logic_error naming the mechanism, and a request
// lasts only the update() it was made in.
// REQ-SAFE-37
TEST(VelocityMechanismRequest, RequestIsTiedToItsUpdate) {
  struct Case {
    const char* name;
    void (*call)(VelocityMechanism&);
  };
  static constexpr std::array<Case, 3> kCases{{
      {"set_goal", [](VelocityMechanism& m) { m.set_goal(60.0); }},
      {"stop", [](VelocityMechanism& m) { m.stop(); }},
      {"update", [](VelocityMechanism& m) { m.update(); }},
  }};
  constexpr std::string_view kExpectedName = "VelocityMechanism 'flywheel'";

  // Constructor (before the event loop runs update()).
  {
    struct CtorNode {
      explicit CtorNode(Node& node) : flywheel{node, "flywheel"} {
        for (std::size_t i = 0; i < kCases.size(); ++i) {
          try {
            kCases[i].call(flywheel);
            errors[i] = "accepted";
          } catch (const std::logic_error& e) {
            errors[i] = e.what();
          }
        }
      }
      void update() { flywheel.update(); }
      VelocityMechanism flywheel;
      std::array<std::string, kCases.size()> errors{};
    };
    auto sim = Sim::FromToml(kRobotToml);
    const auto& node = sim.add<CtorNode>("shooter");
    for (std::size_t i = 0; i < kCases.size(); ++i) {
      SCOPED_TRACE(std::string{"constructor: "} + kCases[i].name);
      EXPECT_NE(node.errors[i].find(kExpectedName), std::string::npos)
          << node.errors[i];
    }
  }

  // every() timer callback.
  for (const Case& c : kCases) {
    SCOPED_TRACE(std::string{"every: "} + c.name);
    struct TimerNode {
      explicit TimerNode(Node& node) : flywheel{node, "flywheel"} {
        node.every(20ms, [this] { call(flywheel); });
      }
      void update() { flywheel.update(); }
      VelocityMechanism flywheel;
      void (*call)(VelocityMechanism&) = nullptr;
    };
    auto sim = Sim::FromToml(kRobotToml);
    sim.add<TimerNode>("shooter").call = c.call;
    try {
      sim.step(50ms);
      ADD_FAILURE() << c.name << " from every() was accepted";
    } catch (const std::logic_error& e) {
      EXPECT_NE(std::string{e.what()}.find(kExpectedName), std::string::npos)
          << e.what();
    }
  }

  // Subscription callback.
  for (const Case& c : kCases) {
    SCOPED_TRACE(std::string{"callback: "} + c.name);
    struct CallbackNode {
      explicit CallbackNode(Node& node)
          : flywheel{node, "flywheel"},
            goal{node.subscribe<control::VelocityTarget>(
                "/test/goal",
                [this](const control::VelocityTarget&) { call(flywheel); })} {}
      void update() { flywheel.update(); }
      VelocityMechanism flywheel;
      void (*call)(VelocityMechanism&) = nullptr;
      Subscription<control::VelocityTarget> goal;
    };
    auto sim = Sim::FromToml(kRobotToml);
    sim.add<CallbackNode>("shooter").call = c.call;
    sim.publish("/test/goal", control::VelocityTarget{60.0, 0.0});
    try {
      sim.step(10ms);
      ADD_FAILURE() << c.name << " from a callback was accepted";
    } catch (const std::logic_error& e) {
      EXPECT_NE(std::string{e.what()}.find(kExpectedName), std::string::npos)
          << e.what();
    }
  }

  // Off the loop thread while update() is running (joined before update()
  // returns so there is no concurrent access to node or mechanism state).
  {
    struct ThreadedNode {
      explicit ThreadedNode(Node& node) : flywheel{node, "flywheel"} {}
      void update() {
        if (!ran) {
          ran = true;
          std::thread other{[this] {
            for (std::size_t i = 0; i < kCases.size(); ++i) {
              try {
                kCases[i].call(flywheel);
                errors[i] = "accepted";
              } catch (const std::logic_error& e) {
                errors[i] = e.what();
              }
            }
          }};
          other.join();
        }
        flywheel.update();
      }
      VelocityMechanism flywheel;
      bool ran = false;
      std::array<std::string, kCases.size()> errors{};
    };
    auto sim = Sim::FromToml(kRobotToml);
    const auto& node = sim.add<ThreadedNode>("shooter");
    sim.step(10ms);
    ASSERT_TRUE(node.ran);
    for (std::size_t i = 0; i < kCases.size(); ++i) {
      SCOPED_TRACE(std::string{"off-thread: "} + kCases[i].name);
      EXPECT_NE(node.errors[i].find(kExpectedName), std::string::npos)
          << node.errors[i];
    }
  }

  struct SkippingNode {
    explicit SkippingNode(Node& node) : flywheel{node, "flywheel"} {}
    void update() {
      if (armed && step++ == 0) {
        flywheel.set_goal(60.0);  // update() skipped this tick
        return;
      }
      flywheel.update();
    }
    VelocityMechanism flywheel;
    bool armed = false;
    int step = 0;
  };
  auto sim = Sim::FromToml(kRobotToml);
  auto& node = sim.add<SkippingNode>("shooter");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(10ms);
  node.armed = true;
  sim.step(15ms);
  ASSERT_GE(node.step, 2);
  EXPECT_FALSE(node.flywheel.driving());
  EXPECT_EQ(hw.applied_motor("shooter", "flywheel").mode, Mode::kNeutral);
}

// A request lasts one tick even where nothing else gates it: with
// AllowUntargetedOutputs the devices flush does not neutralize a tick with no
// target, so the mechanism alone must stop driving when the owner stops
// asking.
// REQ-SAFE-37
TEST(VelocityMechanismRequest, OneRequestDrivesOneTick) {
  struct OnceNode {
    explicit OnceNode(Node& node) : flywheel{node, "flywheel"} {
      AllowUntargetedOutputs(node);
    }
    void update() {
      if (ask) flywheel.set_goal(60.0);
      flywheel.update();
    }
    VelocityMechanism flywheel;
    bool ask = true;
  };
  auto sim = Sim::FromToml(kRobotToml);
  auto& node = sim.add<OnceNode>("shooter");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(10ms);
  ASSERT_TRUE(node.flywheel.driving());
  ASSERT_EQ(hw.applied_motor("shooter", "flywheel").mode, Mode::kVelocity);
  node.ask = false;
  sim.step(10ms);
  EXPECT_FALSE(node.flywheel.driving());
  EXPECT_EQ(hw.applied_motor("shooter", "flywheel").mode, Mode::kNeutral);
}

// The entry count is of consecutive ticks: one tick outside the entry band
// starts it over, so two ticks before and two after are not three.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, EntryCountResetsWhenTheBandIsBroken) {
  SetVelocity(59.0);
  Drive(60.0, 5ms);  // in band
  ASSERT_FALSE(flywheel().at_speed());
  hw.set_motor("shooter", "flywheel", 0.0, 50.0);
  Drive(60.0, 10ms);  // out of band
  ASSERT_FALSE(flywheel().at_speed());
  hw.set_motor("shooter", "flywheel", 0.0, 59.0);
  Drive(60.0, 10ms);  // in band again, short of a full count
  EXPECT_FALSE(flywheel().at_speed());
  Drive(60.0, 10ms);
  EXPECT_TRUE(flywheel().at_speed());
}

// A goal of 1e-3 rps or less is never at speed, even with the flywheel
// exactly on it; just above that it can be.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, TinyGoalIsNeverAtSpeed) {
  SetVelocity(0.001);
  Drive(0.001, 40ms);
  EXPECT_DOUBLE_EQ(Applied().demand, 0.001);
  EXPECT_FALSE(flywheel().at_speed());
  SetVelocity(0.002);
  Drive(0.002, 40ms);
  EXPECT_TRUE(flywheel().at_speed());
}

// REQ-RT-06
TEST_F(VelocityMechanismTest, UpdateDoesNotAllocate) {
  if (const auto why = testing::AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  hw.ideal_motors();
  Drive(60.0, 40ms);
  PublishTarget(sim, "shooter", "teleop", control::VelocityTarget{60.0, 0.0});
  testing::AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_TRUE(flywheel().at_speed());
}

// Each bad parameter throws at construction, naming the key.
// REQ-SAFE-37
TEST(VelocityMechanismConfig, BadParameterFailsConstruction) {
  const auto add = [](const std::string& table) {
    auto sim = Sim::FromToml(std::string{R"(
[robot]
period_us = 5000
command_timeout_us = 100000
[subsystems.shooter.motors.flywheel]
can_id = 1
[subsystems.shooter.flywheel]
)"} + table);
    sim.add<Shooter>("shooter");
  };
  EXPECT_NO_THROW(add(""));
  const std::vector<std::pair<std::string, std::string>> bad{
      {"at_speed_entry_rps", "at_speed_entry_rps = 0.0\n"},
      {"at_speed_entry_rps", "at_speed_entry_rps = nan\n"},
      {"at_speed_exit_frac", "at_speed_exit_frac = -0.1\n"},
      {"at_speed_exit_frac", "at_speed_exit_frac = 1.5\n"},
      {"at_speed_entry_tick_count", "at_speed_entry_tick_count = 0\n"},
      {"spin_down_rps2", "spin_down_rps2 = -1.0\n"},
      {"slot", "slot = -1\n"},
      {"slot", "slot = 3\n"},
  };
  for (const auto& [key, table] : bad) {
    SCOPED_TRACE(table);
    try {
      add(table);
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string{e.what()}.find("flywheel." + key),
                std::string::npos)
          << e.what();
    }
  }
}

}  // namespace
}  // namespace talos::mechanisms
