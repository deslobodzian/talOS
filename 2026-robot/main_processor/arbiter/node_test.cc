#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include "arbiter/arbiter.h"
#include "drivetrain/drivetrain.h"
#include "operator_interface/operator_interface.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"

namespace arbiter {
namespace {

using namespace std::chrono_literals;
using drivetrain::ChassisTarget;
using intake::IntakeTarget;
using shooter::ShooterTarget;
using talos::testing::Sim;
using talos::testing::SimHardware;
namespace ds = talos::driver_station;

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

// The packet the bridge forwards on /hw/state/driver_station.
ds::DriverStationData DsState(std::uint32_t flags) {
  ds::DriverStationData data{};
  data.fms.flags = flags;
  return data;
}

constexpr std::uint32_t kTeleopEnabled =
    ds::kEnabled | ds::kTeleop | ds::kDsAttached;
constexpr std::uint32_t kAutoEnabled =
    ds::kEnabled | ds::kAutonomous | ds::kDsAttached;

ChassisTarget Chassis(double vx) { return ChassisTarget{vx, 0, 0, true}; }

// Counts what the arbiter writes on each subsystem's single target topic.
struct Probe {
  explicit Probe(talos::Node& node) {
    node.subscribe<ChassisTarget>(drivetrain::topics::kTarget,
                                  [this](const ChassisTarget&) { ++chassis; });
    node.subscribe<ShooterTarget>(shooter::topics::kTarget,
                                  [this](const ShooterTarget&) { ++shooter; });
    node.subscribe<IntakeTarget>(intake::topics::kTarget,
                                 [this](const IntakeTarget&) { ++intake; });
  }
  int chassis = 0;
  int shooter = 0;
  int intake = 0;
};

class ArbiterTest : public ::testing::Test {
 protected:
  // SimHardware forwards this packet every robot period from now on.
  void Mode(std::uint32_t flags) {
    hw->set_driver_station(DsState(flags));
    sim.step(10ms);
  }
  template <talos::Message M>
  void Send(const char* topic, const M& message) {
    sim.publish(topic, message);
    sim.step(10ms);
  }
  template <talos::Message M>
  M Last(const char* topic) const {
    const auto message = sim.latest<M>(topic);
    EXPECT_TRUE(message.has_value()) << topic;
    return message.value_or(M{});
  }

  Sim sim{RobotConfig()};
  Arbiter& arbiter = sim.add<Arbiter>("arbiter");
  Probe& out = sim.add<Probe>("probe");
  std::unique_ptr<SimHardware> hw = std::make_unique<SimHardware>(sim);
};

TEST_F(ArbiterTest, ForwardsOnlyTheProducerTheMatchModeSelects) {
  Mode(kTeleopEnabled);
  const int after_mode = out.chassis;
  // The unselected producer is free to keep publishing; none of it gets out.
  Send(drivetrain::topics::kAutoTarget, Chassis(9.0));
  EXPECT_EQ(out.chassis, after_mode);
  Send(drivetrain::topics::kTeleopTarget, Chassis(1.5));
  EXPECT_EQ(out.chassis, after_mode + 1);
  EXPECT_NEAR(Last<ChassisTarget>(drivetrain::topics::kTarget).vx_mps(), 1.5,
              1e-12);

  Mode(kAutoEnabled);
  const int after_switch = out.chassis;
  Send(drivetrain::topics::kTeleopTarget, Chassis(1.5));
  EXPECT_EQ(out.chassis, after_switch);
  Send(drivetrain::topics::kAutoTarget, Chassis(-2.0));
  EXPECT_NEAR(Last<ChassisTarget>(drivetrain::topics::kTarget).vx_mps(), -2.0,
              1e-12);
}

TEST_F(ArbiterTest, StopsOnceOnEveryModeChangeSoNoTargetOutlivesItsMode) {
  Mode(kAutoEnabled);
  Send(drivetrain::topics::kAutoTarget, Chassis(3.0));
  ASSERT_NEAR(Last<ChassisTarget>(drivetrain::topics::kTarget).vx_mps(), 3.0,
              1e-12);

  // Switching to teleop must not leave the autonomous request live on the
  // topic for a command timeout while the driver is already in control.
  const int before = out.chassis;
  Mode(kTeleopEnabled);
  EXPECT_EQ(out.chassis, before + 1);
  const auto released = Last<ChassisTarget>(drivetrain::topics::kTarget);
  EXPECT_FALSE(released.enabled());
  EXPECT_NEAR(released.vx_mps(), 0.0, 1e-12);
  EXPECT_NEAR(released.vy_mps(), 0.0, 1e-12);
  EXPECT_NEAR(released.omega_radps(), 0.0, 1e-12);
  EXPECT_FALSE(Last<ShooterTarget>(shooter::topics::kTarget).enabled());
  EXPECT_FALSE(Last<IntakeTarget>(intake::topics::kTarget).enabled());

  // Repeating the same mode is not a change, so it does not re-emit.
  const int after = out.chassis;
  Mode(kTeleopEnabled);
  EXPECT_EQ(out.chassis, after);
}

TEST_F(ArbiterTest, ForwardsNothingBeforeTheFirstDriverStationPacket) {
  Send(drivetrain::topics::kTeleopTarget, Chassis(1.0));
  Send(drivetrain::topics::kAutoTarget, Chassis(1.0));
  Send(shooter::topics::kTeleopTarget, ShooterTarget{60.0, true});
  EXPECT_EQ(out.chassis, 0);
  EXPECT_EQ(out.shooter, 0);
}

TEST_F(ArbiterTest, RoutesShooterTargetsOnTheSameSelection) {
  Mode(kTeleopEnabled);
  const int before = out.shooter;
  Send(shooter::topics::kAutoTarget, ShooterTarget{99.0, true});
  EXPECT_EQ(out.shooter, before);
  Send(shooter::topics::kTeleopTarget, ShooterTarget{60.0, true});
  EXPECT_EQ(out.shooter, before + 1);
  EXPECT_NEAR(
      Last<ShooterTarget>(shooter::topics::kTarget).target_velocity_rps(), 60.0,
      1e-12);
}

TEST_F(ArbiterTest, RoutesIntakeTargetsOnTheSameSelection) {
  Mode(kTeleopEnabled);
  const int before = out.intake;
  Send(intake::topics::kAutoTarget, IntakeTarget{true, 99.0f});
  EXPECT_EQ(out.intake, before);
  Send(intake::topics::kTeleopTarget, IntakeTarget{true, 30.0f});
  EXPECT_EQ(out.intake, before + 1);
  EXPECT_NEAR(Last<IntakeTarget>(intake::topics::kTarget).roller_velocity_rps(),
              30.0, 1e-6);
}

TEST_F(ArbiterTest, SelectsNothingOnceTheDriverStationGoesSilent) {
  Mode(kTeleopEnabled);
  Send(drivetrain::topics::kTeleopTarget, Chassis(1.5));
  ASSERT_TRUE(Last<ChassisTarget>(drivetrain::topics::kTarget).enabled());

  // The bridge stops forwarding packets. No packet says "disabled", but the
  // last one ages past the command timeout, so update() drops to kNone and
  // stops every lane once.
  hw.reset();
  const int before = out.chassis;
  sim.step(300ms);
  EXPECT_EQ(out.chassis, before + 1);
  EXPECT_FALSE(Last<ChassisTarget>(drivetrain::topics::kTarget).enabled());
  EXPECT_FALSE(Last<ShooterTarget>(shooter::topics::kTarget).enabled());
  EXPECT_FALSE(Last<IntakeTarget>(intake::topics::kTarget).enabled());

  // The teleop producer keeps talking; none of it gets out.
  Send(drivetrain::topics::kTeleopTarget, Chassis(1.5));
  EXPECT_EQ(out.chassis, before + 1);
}

TEST(Arbiter, DisabledEStopTestAndDetachedDriveNothing) {
  for (const std::uint32_t flags :
       {std::uint32_t{ds::kDsAttached},
        std::uint32_t{ds::kEnabled | ds::kTeleop | ds::kEStop |
                      ds::kDsAttached},
        std::uint32_t{ds::kEnabled | ds::kTest | ds::kDsAttached},
        // Enabled teleop from a packet that does not say the DS is attached.
        std::uint32_t{ds::kEnabled | ds::kTeleop}}) {
    Sim sim{RobotConfig()};
    sim.add<Arbiter>("arbiter");
    Probe& out = sim.add<Probe>("probe");
    SimHardware hw{sim};
    hw.set_driver_station(DsState(flags));
    sim.step(10ms);
    sim.publish(drivetrain::topics::kTeleopTarget, Chassis(1.0));
    sim.publish(drivetrain::topics::kAutoTarget, Chassis(1.0));
    sim.step(10ms);
    EXPECT_EQ(out.chassis, 0) << flags;
  }
}

// The whole teleop path, with the same nodes the launcher starts and
// nothing stubbed: a joystick packet lands on /hw/state/driver_station (sent
// by SimHardware, as the bridge forwards it), the operator interface maps it
// to a request, the arbiter admits it because the mode says teleop, and the
// drivetrain resolves it into per-module swerve requests.
class TeleopChain : public ::testing::Test {
 protected:
  void Push(const ds::DriverStationData& data) {
    hw.set_driver_station(data);
    sim.step(20ms);
  }
  static ds::DriverStationData ForwardStick() {
    ds::DriverStationData data{};
    data.fms.flags = ds::kEnabled | ds::kTeleop | ds::kDsAttached;
    data.joysticks[0].connected = true;
    data.joysticks[0].axis_count = 4;
    data.joysticks[0].axes[1] = -1.0f;  // full forward
    return data;
  }

  Sim sim{RobotConfig()};
  operator_interface::OperatorInterface& operator_interface_node =
      sim.add<operator_interface::OperatorInterface>("operator_interface");
  Arbiter& arbiter_node = sim.add<Arbiter>("arbiter");
  drivetrain::Drivetrain& drivetrain_node =
      sim.add<drivetrain::Drivetrain>("drivetrain");
  talos::testing::SimHardware hw{sim};
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
  ds::DriverStationData centered = ForwardStick();
  centered.joysticks[0].axes[1] = 0.0f;
  Push(centered);
  for (const auto& module : geometry.modules) {
    EXPECT_NEAR(hw.motor_request("drivetrain", module.drive).demand, 0.0, 1e-9)
        << module.name;
  }
}

TEST(Arbiter, RoutesTomlConfiguredSubsystemFromOperatorInterfaceToTarget) {
  Sim sim = Sim::FromToml(R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.arbiter]
period_us = 20000

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
  sim.add<operator_interface::OperatorInterface>("operator_interface");
  sim.add<Arbiter>("arbiter");
  SimHardware hw{sim};

  ds::DriverStationData pressed = DsState(kTeleopEnabled);
  pressed.joysticks[0].connected = true;
  pressed.joysticks[0].axis_count = 4;
  pressed.joysticks[0].button_count = 12;
  pressed.joysticks[0].buttons = 1u << 2;  // button 3
  hw.set_driver_station(pressed);
  sim.step(10ms);

  const auto teleop = sim.latest<GenericTarget>("/wrist/target");
  ASSERT_TRUE(teleop.has_value());
  EXPECT_EQ(teleop->enabled, 1u);
  EXPECT_DOUBLE_EQ(teleop->demand, 25.0);

  // Mode switch to autonomous emits a stop on /wrist/target before auto speaks.
  hw.set_driver_station(DsState(kAutoEnabled));
  sim.step(10ms);
  const auto stopped = sim.latest<GenericTarget>("/wrist/target");
  ASSERT_TRUE(stopped.has_value());
  EXPECT_EQ(stopped->enabled, 0u);
  EXPECT_DOUBLE_EQ(stopped->demand, 0.0);

  sim.publish("/wrist/target/auto", GenericTarget{1, {}, -12.5});
  sim.step(10ms);
  const auto auto_target = sim.latest<GenericTarget>("/wrist/target");
  ASSERT_TRUE(auto_target.has_value());
  EXPECT_EQ(auto_target->enabled, 1u);
  EXPECT_DOUBLE_EQ(auto_target->demand, -12.5);
}

}  // namespace
}  // namespace arbiter
