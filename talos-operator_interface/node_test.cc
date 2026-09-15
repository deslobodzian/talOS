#include "talos_nodes/operator_interface/node.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>

#include "talOS/events/simulated_event_loop.h"

namespace talos::oi {
namespace {

using namespace std::chrono_literals;
using driver_station::DriverStationState;
using driver_station::FmsState;
using driver_station::JoystickState;

JoystickState Stick(float axis0, float axis1, float axis2, uint32_t buttons = 0,
                    bool connected = true) {
  return JoystickState{connected, 4, 12, 1, buttons, axis0, axis1, axis2, 0, 0};
}

DriverStationState DsState(uint32_t flags, JoystickState stick) {
  return DriverStationState{0, 0, FmsState{flags, 0, 0, 0, 0, 0, 0.0f}, stick,
                            JoystickState{}};
}

constexpr uint32_t kTeleopEnabled = driver_station::kEnabled |
                                    driver_station::kTeleop |
                                    driver_station::kDsAttached;

OperatorInterfaceConfig NoDeadband(bool field_oriented = true) {
  OperatorInterfaceConfig config{};
  config.deadband = 0.0;
  config.field_oriented = field_oriented;
  return config;
}

class Harness {
 public:
  explicit Harness(OperatorInterfaceConfig config = {})
      : loop_{env_}, node_{loop_, config} {}

  void Push(const DriverStationState& state) {
    loop_.inject(driver_station::kDsStateTopic, state);
    loop_.run_for(10ms);
  }
  void PublishHeading(double yaw_rot) {
    loop_.inject(talos::drive::kDrivetrainStateTopic,
                 talos::drive::DrivetrainState{0, true, 0, 0, 0, yaw_rot, 0});
    loop_.run_for(10ms);
  }
  void Advance(event::Duration duration) { loop_.run_for(duration); }
  event::MonotonicTime now() const { return loop_.monotonic_now(); }
  uint64_t chassis_count() {
    return env_
        .channel(talos::drive::kTeleopTargetTopic,
                 sizeof(talos::drive::ChassisTarget))
        .next_sequence();
  }
  talos::drive::ChassisTarget last_chassis() {
    auto& channel = env_.channel(talos::drive::kTeleopTargetTopic,
                                 sizeof(talos::drive::ChassisTarget));
    talos::drive::ChassisTarget target{};
    EXPECT_GT(channel.next_sequence(), 0u);
    if (channel.next_sequence() == 0) return target;
    channel.copy_to(channel.next_sequence() - 1,
                    {reinterpret_cast<std::byte*>(&target), sizeof(target)});
    return target;
  }
  OperatorInterfaceNode<event::SimulatedEventLoop<>>& node() { return node_; }

 private:
  event::SimulationEnvironment env_;
  event::SimulatedEventLoop<> loop_;
  OperatorInterfaceNode<event::SimulatedEventLoop<>> node_;
};

TEST(OperatorInterface, MapsStickAxesToChassisRequests) {
  Harness harness{NoDeadband(/*field_oriented=*/false)};
  // Forward is negative axis 1, left is negative axis 0, CCW is negative 2.
  harness.Push(DsState(kTeleopEnabled, Stick(-0.5f, -0.75f, -0.25f,
                                             /*buttons=*/1)));

  const OperatorInterfaceConfig defaults{};
  const auto target = harness.last_chassis();
  EXPECT_NEAR(target.vx_mps(), 0.75 * defaults.max_linear_mps, 1e-9);
  EXPECT_NEAR(target.vy_mps(), 0.5 * defaults.max_linear_mps, 1e-9);
  EXPECT_NEAR(target.omega_radps(), 0.25 * defaults.max_angular_radps, 1e-9);
  EXPECT_TRUE(target.enabled());
}

TEST(OperatorInterface, DeadbandIsZeroInsideAndContinuousAtTheThreshold) {
  OperatorInterfaceConfig config{};
  config.field_oriented = false;
  config.deadband = 0.05;
  Harness harness{config};

  harness.Push(DsState(kTeleopEnabled, Stick(0, -0.04f, 0)));
  EXPECT_NEAR(harness.last_chassis().vx_mps(), 0.0, 1e-12);

  // Just outside: the output ramps from zero rather than stepping to full.
  harness.Push(DsState(kTeleopEnabled, Stick(0, -0.10f, 0)));
  EXPECT_NEAR(harness.last_chassis().vx_mps(),
              (0.10 - 0.05) / 0.95 * config.max_linear_mps, 1e-6);
}

TEST(OperatorInterface, FieldOrientedRotatesTheRequestByDrivetrainHeading) {
  Harness harness{NoDeadband(/*field_oriented=*/true)};
  harness.PublishHeading(0.25);  // A quarter turn CCW: nose points to field +y.
  EXPECT_TRUE(harness.node().field_oriented_active(harness.now()));

  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));

  // Driving downfield while facing left means strafing to the robot's right.
  const OperatorInterfaceConfig defaults{};
  const auto target = harness.last_chassis();
  EXPECT_NEAR(target.vx_mps(), 0.0, 1e-9);
  EXPECT_NEAR(target.vy_mps(), -defaults.max_linear_mps, 1e-9);
}

TEST(OperatorInterface, FieldOrientedIsRobotRelativeUntilAHeadingArrives) {
  Harness harness{NoDeadband(/*field_oriented=*/true)};
  // Rotating by an assumed yaw of zero looks right only while the robot
  // happens to face downfield, so the request passes through untouched.
  EXPECT_FALSE(harness.node().field_oriented_active(harness.now()));
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));

  const OperatorInterfaceConfig defaults{};
  EXPECT_NEAR(harness.last_chassis().vx_mps(), defaults.max_linear_mps, 1e-9);
  EXPECT_NEAR(harness.last_chassis().vy_mps(), 0.0, 1e-9);
}

TEST(OperatorInterface, StaleHeadingFallsBackToRobotRelative) {
  Harness harness{NoDeadband(/*field_oriented=*/true)};
  harness.PublishHeading(0.25);
  ASSERT_TRUE(harness.node().field_oriented_active(harness.now()));

  // The drivetrain stops publishing: after the timeout the latched quarter
  // turn must not steer the robot anymore.
  harness.Advance(1s);
  EXPECT_FALSE(harness.node().field_oriented_active(harness.now()));
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));

  const OperatorInterfaceConfig defaults{};
  EXPECT_NEAR(harness.last_chassis().vx_mps(), defaults.max_linear_mps, 1e-9);
  EXPECT_NEAR(harness.last_chassis().vy_mps(), 0.0, 1e-9);
}

TEST(OperatorInterface, HeadingWithinTimeoutStaysFieldOriented) {
  Harness harness{NoDeadband(/*field_oriented=*/true)};
  harness.PublishHeading(0.25);

  // One dropped packet at the 20 ms rate is not a stale publisher.
  harness.Advance(20ms);
  EXPECT_TRUE(harness.node().field_oriented_active(harness.now()));
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));

  const OperatorInterfaceConfig defaults{};
  const auto target = harness.last_chassis();
  EXPECT_NEAR(target.vx_mps(), 0.0, 1e-9);
  EXPECT_NEAR(target.vy_mps(), -defaults.max_linear_mps, 1e-9);
}

TEST(OperatorInterface, FreshHeadingResumesFieldOrientedAfterStale) {
  Harness harness{NoDeadband(/*field_oriented=*/true)};
  harness.PublishHeading(0.25);
  harness.Advance(1s);
  ASSERT_FALSE(harness.node().field_oriented_active(harness.now()));

  harness.PublishHeading(0.25);
  EXPECT_TRUE(harness.node().field_oriented_active(harness.now()));
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));

  const OperatorInterfaceConfig defaults{};
  const auto target = harness.last_chassis();
  EXPECT_NEAR(target.vx_mps(), 0.0, 1e-9);
  EXPECT_NEAR(target.vy_mps(), -defaults.max_linear_mps, 1e-9);
}

TEST(OperatorInterface, NonFiniteHeadingFallsBackToRobotRelative) {
  Harness harness{NoDeadband(/*field_oriented=*/true)};
  // A NaN yaw carries no direction: it must not latch, so control stays
  // robot-relative exactly as if no heading had ever arrived.
  harness.PublishHeading(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(harness.node().field_oriented_active(harness.now()));
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));

  const OperatorInterfaceConfig defaults{};
  EXPECT_NEAR(harness.last_chassis().vx_mps(), defaults.max_linear_mps, 1e-9);
  EXPECT_NEAR(harness.last_chassis().vy_mps(), 0.0, 1e-9);
}

TEST(OperatorInterface, RobotRelativeModeIgnoresHeading) {
  Harness harness{NoDeadband(/*field_oriented=*/false)};
  harness.PublishHeading(0.25);
  EXPECT_FALSE(harness.node().field_oriented_active(harness.now()));
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));

  const OperatorInterfaceConfig defaults{};
  EXPECT_NEAR(harness.last_chassis().vx_mps(), defaults.max_linear_mps, 1e-9);
  EXPECT_NEAR(harness.last_chassis().vy_mps(), 0.0, 1e-9);
}

TEST(OperatorInterface, RequestsNothingWhenDisabledAutonomousOrDisconnected) {
  const JoystickState deflected = Stick(0, -0.9f, 0);
  const std::pair<uint32_t, JoystickState> cases[]{
      {driver_station::kDsAttached, deflected},
      {driver_station::kEnabled | driver_station::kAutonomous, deflected},
      {kTeleopEnabled, Stick(0, -0.9f, 0, 0, /*connected=*/false)},
  };
  for (const auto& [flags, stick] : cases) {
    Harness harness{NoDeadband(false)};
    harness.Push(DsState(flags, stick));
    EXPECT_EQ(harness.chassis_count(), 0u);
  }
}

TEST(OperatorInterface, PublishesOneDisabledRequestOnTheDisableEdge) {
  Harness harness{NoDeadband(false)};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));
  const auto commanding = harness.chassis_count();
  ASSERT_GT(commanding, 0u);
  ASSERT_TRUE(harness.node().commanding());

  // Disabled with the stick still deflected: the drivetrain is told to stop
  // rather than left to time the last request out.
  harness.Push(DsState(driver_station::kDsAttached, Stick(0, -1.0f, 0)));
  EXPECT_EQ(harness.chassis_count(), commanding + 1);
  EXPECT_FALSE(harness.node().commanding());
  const auto released = harness.last_chassis();
  EXPECT_FALSE(released.enabled());
  EXPECT_NEAR(released.vx_mps(), 0.0, 1e-12);

  // Staying disabled is not another edge.
  const auto after = harness.chassis_count();
  harness.Push(DsState(driver_station::kDsAttached, Stick(0, -1.0f, 0)));
  EXPECT_EQ(harness.chassis_count(), after);
}

}  // namespace
}  // namespace talos::oi
