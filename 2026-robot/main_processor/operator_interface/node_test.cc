#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "operator_interface/operator_interface.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"

namespace operator_interface {
namespace {

using namespace std::chrono_literals;
using talos::testing::Sim;
using talos::testing::SimHardware;
namespace ds = talos::driver_station;

constexpr double kMaxLinear = 4.0;  // the compiled defaults
constexpr double kMaxAngular = 6.0;

ds::JoystickData Stick(float axis0, float axis1, float axis2,
                       std::uint32_t buttons = 0, bool connected = true) {
  ds::JoystickData stick{};
  stick.connected = connected;
  stick.axis_count = 4;
  stick.button_count = 12;
  stick.pov_count = 1;
  stick.buttons = buttons;
  stick.axes[0] = axis0;
  stick.axes[1] = axis1;
  stick.axes[2] = axis2;
  return stick;
}

// The packet the bridge forwards on /hw/state/driver_station.
ds::DriverStationData DsState(std::uint32_t flags,
                              const ds::JoystickData& stick) {
  ds::DriverStationData data{};
  data.fms.flags = flags;
  data.joysticks[0] = stick;
  return data;
}

constexpr std::uint32_t kTeleopEnabled =
    ds::kEnabled | ds::kTeleop | ds::kDsAttached;

// Counts the chassis requests on the teleop lane, and the stops among them.
struct Probe {
  explicit Probe(talos::Node& node) {
    node.subscribe<drivetrain::ChassisTarget>(
        drivetrain::topics::kTeleopTarget,
        [this](const drivetrain::ChassisTarget& target) {
          ++chassis;
          if (!target.enabled()) ++stops;
        });
  }
  int chassis = 0;
  int stops = 0;
};

// One operator interface whose [subsystems.operator_interface] table holds
// `keys` and nothing else, so every other setting is the compiled default.
class Harness {
 public:
  explicit Harness(std::string_view keys = "")
      : sim_{Sim::FromToml(std::string{R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.mechanism.motors.unused]
can_id = 1

[subsystems.operator_interface]
period_us = 20000
)"} + std::string{keys})} {}

  // SimHardware forwards this packet every robot period from now on.
  void Push(const ds::DriverStationData& state) {
    hw_->set_driver_station(state);
    sim_.step(10ms);
  }
  // The bridge goes silent: no more Driver Station packets.
  void Silence() { hw_.reset(); }
  void PublishHeading(double yaw_rot) {
    sim_.publish(drivetrain::topics::kState,
                 drivetrain::DrivetrainState{0, true, 0, 0, 0, yaw_rot, 0});
    sim_.step(10ms);
  }
  void Advance(talos::Duration d) { sim_.step(d); }
  int chassis_count() const { return probe_.chassis; }
  int stop_count() const { return probe_.stops; }
  drivetrain::ChassisTarget chassis() const {
    return Last<drivetrain::ChassisTarget>(drivetrain::topics::kTeleopTarget);
  }
  shooter::ShooterTarget shooter() const {
    return Last<shooter::ShooterTarget>(shooter::topics::kTeleopTarget);
  }
  intake::IntakeTarget intake() const {
    return Last<intake::IntakeTarget>(intake::topics::kTeleopTarget);
  }

 private:
  template <class M>
  M Last(const char* topic) const {
    const auto message = sim_.latest<M>(topic);
    EXPECT_TRUE(message.has_value()) << topic;
    return message.value_or(M{});
  }

  Sim sim_;
  OperatorInterface& node_ = sim_.add<OperatorInterface>("operator_interface");
  Probe& probe_ = sim_.add<Probe>("probe");
  std::unique_ptr<SimHardware> hw_ = std::make_unique<SimHardware>(sim_);
};

constexpr const char* kNoDeadbandRobotRelative =
    "deadband = 0.0\nfield_oriented = false\n";
constexpr const char* kNoDeadbandFieldOriented =
    "deadband = 0.0\nfield_oriented = true\n";

TEST(OperatorInterface, MapsStickAxesToChassisRequests) {
  Harness harness{kNoDeadbandRobotRelative};
  // Forward is negative axis 1, left is negative axis 0, CCW is negative 2.
  harness.Push(DsState(kTeleopEnabled, Stick(-0.5f, -0.75f, -0.25f, 1)));
  const auto target = harness.chassis();
  EXPECT_NEAR(target.vx_mps(), 0.75 * kMaxLinear, 1e-9);
  EXPECT_NEAR(target.vy_mps(), 0.5 * kMaxLinear, 1e-9);
  EXPECT_NEAR(target.omega_radps(), 0.25 * kMaxAngular, 1e-9);
  EXPECT_TRUE(target.enabled());
}

TEST(OperatorInterface, DeadbandIsZeroInsideAndContinuousAtTheThreshold) {
  Harness harness{"deadband = 0.05\nfield_oriented = false\n"};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -0.04f, 0)));
  EXPECT_NEAR(harness.chassis().vx_mps(), 0.0, 1e-12);
  // Just outside: the output ramps from zero rather than stepping to full.
  harness.Push(DsState(kTeleopEnabled, Stick(0, -0.10f, 0)));
  EXPECT_NEAR(harness.chassis().vx_mps(), (0.10 - 0.05) / 0.95 * kMaxLinear,
              1e-6);
}

TEST(OperatorInterface, TuningAndBindingsComeFromTheSubsystemTable) {
  Harness harness{
      "deadband = 0.0\nfield_oriented = false\nmax_linear_mps = 2.0\n"
      "shooter_target_rps = 75.0\nshoot_button = 3\n"
      "intake_target_rps = 12.5\nintake_button = 4\n"};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0, /*buttons=*/1)));
  EXPECT_NEAR(harness.chassis().vx_mps(), 2.0, 1e-9);
  EXPECT_FALSE(harness.shooter().enabled());  // button 1 is no longer shoot
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/4 | 8)));
  EXPECT_TRUE(harness.shooter().enabled());
  EXPECT_NEAR(harness.shooter().target_velocity_rps(), 75.0, 1e-9);
  EXPECT_TRUE(harness.intake().enabled());
  EXPECT_NEAR(harness.intake().roller_velocity_rps(), 12.5, 1e-6);
}

TEST(OperatorInterface, FieldOrientedRotatesTheRequestByDrivetrainHeading) {
  Harness harness{kNoDeadbandFieldOriented};
  harness.PublishHeading(0.25);  // a quarter turn CCW: nose to field +y
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));
  // Driving downfield while facing left means strafing to the robot's right.
  EXPECT_NEAR(harness.chassis().vx_mps(), 0.0, 1e-9);
  EXPECT_NEAR(harness.chassis().vy_mps(), -kMaxLinear, 1e-9);
}

TEST(OperatorInterface, HeadingWithinTimeoutStaysFieldOriented) {
  Harness harness{kNoDeadbandFieldOriented};
  harness.PublishHeading(0.25);
  harness.Advance(20ms);  // one dropped packet is not a stale publisher
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));
  EXPECT_NEAR(harness.chassis().vy_mps(), -kMaxLinear, 1e-9);
}

TEST(OperatorInterface, WithoutAFreshFiniteHeadingDrivesRobotRelative) {
  // Rotating by an assumed or stale yaw looks right only while the robot
  // happens to face that way, so the request passes through untouched.
  const auto robot_relative = [](Harness& harness) {
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));
    EXPECT_NEAR(harness.chassis().vx_mps(), kMaxLinear, 1e-9);
    EXPECT_NEAR(harness.chassis().vy_mps(), 0.0, 1e-9);
  };
  {
    Harness never{kNoDeadbandFieldOriented};  // no heading yet
    robot_relative(never);
  }
  {
    Harness stale{kNoDeadbandFieldOriented};
    stale.PublishHeading(0.25);
    stale.Advance(1s);
    robot_relative(stale);
  }
  {
    Harness nan{kNoDeadbandFieldOriented};
    nan.PublishHeading(std::numeric_limits<double>::quiet_NaN());
    robot_relative(nan);
  }
  {
    Harness off{kNoDeadbandRobotRelative};
    off.PublishHeading(0.25);
    robot_relative(off);
  }
}

TEST(OperatorInterface, AFreshHeadingResumesFieldOrientedAfterStale) {
  Harness harness{kNoDeadbandFieldOriented};
  harness.PublishHeading(0.25);
  harness.Advance(1s);
  harness.PublishHeading(0.25);
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));
  EXPECT_NEAR(harness.chassis().vy_mps(), -kMaxLinear, 1e-9);
}

TEST(OperatorInterface, RequestsNothingWhenDisabledAutonomousOrDisconnected) {
  const ds::JoystickData deflected = Stick(0, -0.9f, 0);
  const std::pair<std::uint32_t, ds::JoystickData> cases[]{
      {ds::kDsAttached, deflected},
      {ds::kEnabled | ds::kAutonomous, deflected},
      {kTeleopEnabled, Stick(0, -0.9f, 0, 0, /*connected=*/false)},
  };
  for (const auto& [flags, stick] : cases) {
    Harness harness{kNoDeadbandRobotRelative};
    harness.Push(DsState(flags, stick));
    EXPECT_EQ(harness.chassis_count(), 0);
  }
}

TEST(OperatorInterface, PublishesOneStopPerTargetOnTheDisableEdge) {
  Harness harness{kNoDeadbandRobotRelative};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0, /*buttons=*/3)));
  ASSERT_GT(harness.chassis_count(), 0);
  ASSERT_EQ(harness.stop_count(), 0);
  EXPECT_TRUE(harness.shooter().enabled());
  EXPECT_TRUE(harness.intake().enabled());

  // Disabled with the stick still deflected: every subsystem is told to stop
  // rather than left to time the last request out.
  harness.Push(DsState(ds::kDsAttached, Stick(0, -1.0f, 0, 3)));
  EXPECT_EQ(harness.stop_count(), 1);
  EXPECT_FALSE(harness.chassis().enabled());
  EXPECT_NEAR(harness.chassis().vx_mps(), 0.0, 1e-12);
  EXPECT_FALSE(harness.shooter().enabled());
  EXPECT_FALSE(harness.intake().enabled());

  // Staying disabled is not another edge.
  harness.Push(DsState(ds::kDsAttached, Stick(0, -1.0f, 0)));
  EXPECT_EQ(harness.stop_count(), 1);
}

TEST(OperatorInterface, ASilentDriverStationGetsOneStopPerTarget) {
  Harness harness{kNoDeadbandRobotRelative};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0, /*buttons=*/3)));
  ASSERT_GT(harness.chassis_count(), 0);
  ASSERT_EQ(harness.stop_count(), 0);
  ASSERT_TRUE(harness.chassis().enabled());

  // No packet arrives to say "disabled"; the last one ages past the command
  // timeout and update() stops every target once.
  harness.Silence();
  harness.Advance(300ms);
  EXPECT_EQ(harness.stop_count(), 1);
  EXPECT_FALSE(harness.chassis().enabled());
  EXPECT_NEAR(harness.chassis().vx_mps(), 0.0, 1e-12);
  EXPECT_FALSE(harness.shooter().enabled());
  EXPECT_FALSE(harness.intake().enabled());
}

TEST(OperatorInterface, RequestsNothingWithoutTheDsAttachedFlag) {
  // Enabled teleop flags from a packet that does not say the DS is attached
  // read as disabled.
  Harness harness{kNoDeadbandRobotRelative};
  harness.Push(DsState(ds::kEnabled | ds::kTeleop, Stick(0, -0.9f, 0)));
  harness.Advance(100ms);
  EXPECT_EQ(harness.chassis_count(), 0);
}

TEST(OperatorInterface, RunsShooterOnButtonOneAndIntakeOnButtonTwo) {
  Harness harness;
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/1)));
  EXPECT_TRUE(harness.shooter().enabled());
  EXPECT_NEAR(harness.shooter().target_velocity_rps(), 60.0, 1e-9);
  EXPECT_FALSE(harness.intake().enabled());
  EXPECT_TRUE(harness.chassis().enabled());

  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/2)));
  EXPECT_FALSE(harness.shooter().enabled());
  EXPECT_TRUE(harness.intake().enabled());
  EXPECT_NEAR(harness.intake().roller_velocity_rps(), 30.0, 1e-6);
}

TEST(OperatorInterface, PublishesTomlConfiguredSubsystemTeleopTarget) {
  Sim sim = Sim::FromToml(R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.operator_interface]
period_us = 20000

[subsystems.wrist]
period_us = 20000
teleop_button = 3
teleop_demand = 25.0
telemetry = true

[subsystems.wrist.motors.wrist_motor]
can_id = 1
)");
  sim.add<OperatorInterface>("operator_interface");
  SimHardware hw{sim};

  // Button 3 (bit 2) pressed in teleop -> enabled with demand 25.0.
  hw.set_driver_station(
      DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/1u << 2)));
  sim.step(10ms);
  auto target = sim.latest<GenericTarget>("/wrist/target/teleop");
  ASSERT_TRUE(target.has_value());
  EXPECT_EQ(target->enabled, 1u);
  EXPECT_DOUBLE_EQ(target->demand, 25.0);

  // Button 3 released -> disabled with demand 0.0.
  hw.set_driver_station(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/0)));
  sim.step(10ms);
  target = sim.latest<GenericTarget>("/wrist/target/teleop");
  ASSERT_TRUE(target.has_value());
  EXPECT_EQ(target->enabled, 0u);
  EXPECT_DOUBLE_EQ(target->demand, 0.0);

  // Press button 3 again, then disable robot -> falling-edge stop emitted.
  hw.set_driver_station(
      DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/1u << 2)));
  sim.step(10ms);
  ASSERT_EQ(sim.latest<GenericTarget>("/wrist/target/teleop")->enabled, 1u);

  hw.set_driver_station(
      DsState(ds::kDsAttached, Stick(0, 0, 0, /*buttons=*/1u << 2)));
  sim.step(10ms);
  target = sim.latest<GenericTarget>("/wrist/target/teleop");
  ASSERT_TRUE(target.has_value());
  EXPECT_EQ(target->enabled, 0u);
  EXPECT_DOUBLE_EQ(target->demand, 0.0);
}

}  // namespace
}  // namespace operator_interface
