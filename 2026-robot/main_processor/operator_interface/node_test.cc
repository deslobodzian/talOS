#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "drivetrain/drivetrain.h"
#include "operator_interface/operator_interface.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"

namespace operator_interface {
namespace {

using namespace std::chrono_literals;
using talos::testing::LatestTarget;
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
  // Envelopes written on /drivetrain/target/teleop so far.
  std::uint64_t chassis_count() const {
    return talos::detail::SimAccess::message_count(
        sim_, "/drivetrain/target/teleop",
        sizeof(talos::TargetEnvelope<drivetrain::ChassisTarget>));
  }
  // The newest envelope on each teleop lane (nullopt: never written).
  std::optional<talos::TargetEnvelope<drivetrain::ChassisTarget>> chassis_lane()
      const {
    return LatestTarget<drivetrain::ChassisTarget>(sim_, "drivetrain",
                                                   "teleop");
  }
  std::optional<talos::TargetEnvelope<shooter::ShooterTarget>> shooter_lane()
      const {
    return LatestTarget<shooter::ShooterTarget>(sim_, "shooter", "teleop");
  }
  std::optional<talos::TargetEnvelope<intake::IntakeTarget>> intake_lane()
      const {
    return LatestTarget<intake::IntakeTarget>(sim_, "intake", "teleop");
  }
  // The message of an active lane; fails the test when it is released.
  drivetrain::ChassisTarget chassis() const { return Active(chassis_lane()); }
  shooter::ShooterTarget shooter() const { return Active(shooter_lane()); }
  intake::IntakeTarget intake() const { return Active(intake_lane()); }
  bool shooter_active() const { return IsActive(shooter_lane()); }
  bool intake_active() const { return IsActive(intake_lane()); }
  bool chassis_active() const { return IsActive(chassis_lane()); }

 private:
  template <class M>
  static bool IsActive(const std::optional<talos::TargetEnvelope<M>>& e) {
    EXPECT_TRUE(e.has_value());
    return e.has_value() && e->active != 0;
  }
  template <class M>
  static M Active(const std::optional<talos::TargetEnvelope<M>>& e) {
    EXPECT_TRUE(IsActive(e));
    return e.has_value() ? e->message : M{};
  }

  Sim sim_;
  OperatorInterface& node_ = sim_.add<OperatorInterface>("operator_interface");
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
  EXPECT_FALSE(harness.shooter_active());  // button 1 is no longer shoot
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/4 | 8)));
  EXPECT_NEAR(harness.shooter().target_velocity_rps(), 75.0, 1e-9);
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
    EXPECT_EQ(harness.chassis_count(), 0u);
  }
}

TEST(OperatorInterface, ReleasesEveryLaneOnceOnTheDisableEdge) {
  Harness harness{kNoDeadbandRobotRelative};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0, /*buttons=*/3)));
  ASSERT_TRUE(harness.chassis_active());
  ASSERT_TRUE(harness.shooter_active());
  ASSERT_TRUE(harness.intake_active());

  // Disabled with the stick still deflected: every lane is released rather
  // than left to time the last request out.
  harness.Push(DsState(ds::kDsAttached, Stick(0, -1.0f, 0, 3)));
  EXPECT_FALSE(harness.chassis_active());
  EXPECT_FALSE(harness.shooter_active());
  EXPECT_FALSE(harness.intake_active());
  const std::uint64_t after_release = harness.chassis_count();

  // Staying disabled is not another edge.
  harness.Push(DsState(ds::kDsAttached, Stick(0, -1.0f, 0)));
  EXPECT_EQ(harness.chassis_count(), after_release);
}

TEST(OperatorInterface, ASilentDriverStationReleasesEveryLaneOnce) {
  Harness harness{kNoDeadbandRobotRelative};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0, /*buttons=*/3)));
  ASSERT_TRUE(harness.chassis_active());

  // No packet arrives to say "disabled"; the last one ages past the command
  // timeout and update() releases every lane once.
  harness.Silence();
  harness.Advance(300ms);
  EXPECT_FALSE(harness.chassis_active());
  EXPECT_FALSE(harness.shooter_active());
  EXPECT_FALSE(harness.intake_active());
  const std::uint64_t after_release = harness.chassis_count();
  harness.Advance(100ms);
  EXPECT_EQ(harness.chassis_count(), after_release);
}

TEST(OperatorInterface, RequestsNothingWithoutTheDsAttachedFlag) {
  // Enabled teleop flags from a packet that does not say the DS is attached
  // read as disabled.
  Harness harness{kNoDeadbandRobotRelative};
  harness.Push(DsState(ds::kEnabled | ds::kTeleop, Stick(0, -0.9f, 0)));
  harness.Advance(100ms);
  EXPECT_EQ(harness.chassis_count(), 0u);
}

TEST(OperatorInterface, RunsShooterOnButtonOneAndIntakeOnButtonTwo) {
  Harness harness;
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/1)));
  EXPECT_NEAR(harness.shooter().target_velocity_rps(), 60.0, 1e-9);
  EXPECT_FALSE(harness.intake_active());
  EXPECT_TRUE(harness.chassis_active());

  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/2)));
  EXPECT_FALSE(harness.shooter_active());
  EXPECT_NEAR(harness.intake().roller_velocity_rps(), 30.0, 1e-6);
}

// --- the teleop chain on the real robot config ---------------------------

std::string RobotConfig() {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv("TALOS_TEST_CONFIG");
  if (!runfiles || location == nullptr) {
    throw std::runtime_error("robot config unavailable: " + error);
  }
  return runfiles->Rlocation(location);
}

// The whole teleop path, with the same nodes the launcher starts and
// nothing stubbed: a joystick packet lands on /hw/state/driver_station (sent
// by SimHardware, as the bridge forwards it), the operator interface writes
// it to /drivetrain/target/teleop, the drivetrain's Target selects that lane
// because the mode says teleop, and resolves it into per-module requests.
class TeleopChain : public ::testing::Test {
 protected:
  void Push(const ds::DriverStationData& data) {
    hw.set_driver_station(data);
    sim.step(20ms);
  }
  static ds::DriverStationData ForwardStick() {
    return DsState(kTeleopEnabled, Stick(0, -1.0f, 0));  // full forward
  }
  void ExpectModulesNeutral() const {
    for (const auto& module : geometry.modules) {
      EXPECT_EQ(hw.motor_request("drivetrain", module.drive).mode,
                talos::hardware::Mode::kNeutral)
          << module.name;
    }
  }

  Sim sim{RobotConfig()};
  OperatorInterface& operator_interface_node =
      sim.add<OperatorInterface>("operator_interface");
  drivetrain::Drivetrain& drivetrain_node =
      sim.add<drivetrain::Drivetrain>("drivetrain");
  SimHardware hw{sim};
  const drivetrain::SwerveGeometry geometry = drivetrain::BuildSwerveGeometry(
      talos::config::ParseRobotConfig(RobotConfig()), "drivetrain");

  void SetUp() override { hw.enable(); }
};

TEST_F(TeleopChain, JoystickDeflectionReachesTheSwerveModulesAsVelocity) {
  Push(ForwardStick());
  // 4 m/s over a 0.0508 m wheel, below the 20 rps drive ceiling, so the
  // request is not desaturated.
  const double expected_rps = 4.0 / (2 * std::numbers::pi * 0.0508);
  for (const auto& module : geometry.modules) {
    const auto drive = hw.motor_request("drivetrain", module.drive);
    const auto steer = hw.motor_request("drivetrain", module.steer);
    EXPECT_EQ(drive.mode, talos::hardware::Mode::kVelocity) << module.name;
    EXPECT_NEAR(drive.demand, expected_rps, 1e-9) << module.name;
    EXPECT_EQ(steer.mode, talos::hardware::Mode::kPosition) << module.name;
    EXPECT_NEAR(steer.demand, 0.0, 1e-9) << module.name;
  }
}

TEST_F(TeleopChain, ReleasingTheStickStopsTheModulesWithoutWaitingForTimeout) {
  Push(ForwardStick());
  ASSERT_EQ(hw.motor_request("drivetrain", geometry.modules[0].drive).mode,
            talos::hardware::Mode::kVelocity);
  Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
  for (const auto& module : geometry.modules) {
    EXPECT_NEAR(hw.motor_request("drivetrain", module.drive).demand, 0.0, 1e-9)
        << module.name;
  }
}

TEST_F(TeleopChain, DisablingOrLosingTheDriverStationStopsTheModules) {
  Push(ForwardStick());
  ASSERT_EQ(hw.motor_request("drivetrain", geometry.modules[0].drive).mode,
            talos::hardware::Mode::kVelocity);
  Push(DsState(ds::kDsAttached, Stick(0, -1.0f, 0)));
  ExpectModulesNeutral();

  Push(ForwardStick());
  ASSERT_EQ(hw.motor_request("drivetrain", geometry.modules[0].drive).mode,
            talos::hardware::Mode::kVelocity);
  hw.stop_driver_station();
  sim.step(200ms);
  ExpectModulesNeutral();
}

TEST_F(TeleopChain, AutonomousIgnoresTheSticks) {
  Push(DsState(ds::kEnabled | ds::kAutonomous | ds::kDsAttached,
               Stick(0, -1.0f, 0)));
  sim.step(40ms);
  ExpectModulesNeutral();
}

}  // namespace
}  // namespace operator_interface
