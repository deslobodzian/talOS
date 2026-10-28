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
#include "talOS/driver_station/driver_station_node.h"
#include "talOS/node/testing.h"

namespace arbiter {
namespace {

using namespace std::chrono_literals;
using drivetrain::ChassisTarget;
using driver_station::DriverStationState;
using driver_station::FmsState;
using intake::IntakeTarget;
using shooter::ShooterTarget;
using talos::testing::Sim;
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

DriverStationState DsState(std::uint32_t flags) {
  return DriverStationState{0, 0, FmsState{flags, 0, 0, 0, 0, 0, 0.0f},
                            driver_station::JoystickState{},
                            driver_station::JoystickState{}};
}

constexpr std::uint32_t kTeleopEnabled = ds::kEnabled | ds::kTeleop;
constexpr std::uint32_t kAutoEnabled = ds::kEnabled | ds::kAutonomous;

ChassisTarget Chassis(double vx) { return ChassisTarget{vx, 0, 0, 0, true}; }

// Counts what the arbiter writes on each subsystem's single target topic.
struct Probe {
  explicit Probe(talos::Node& node) {
    node.subscribe<ChassisTarget>(drivetrain::kTargetTopic,
                                  [this](const ChassisTarget&) { ++chassis; });
    node.subscribe<ShooterTarget>(shooter::kTargetTopic,
                                  [this](const ShooterTarget&) { ++shooter; });
    node.subscribe<IntakeTarget>(intake::kTargetTopic,
                                 [this](const IntakeTarget&) { ++intake; });
  }
  int chassis = 0;
  int shooter = 0;
  int intake = 0;
};

class ArbiterTest : public ::testing::Test {
 protected:
  void Mode(std::uint32_t flags) { Send(driver_station::kStateTopic, DsState(flags)); }
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
};

TEST_F(ArbiterTest, ForwardsOnlyTheProducerTheMatchModeSelects) {
  Mode(kTeleopEnabled);
  const int after_mode = out.chassis;
  // The unselected producer is free to keep publishing; none of it gets out.
  Send(drivetrain::kAutoTargetTopic, Chassis(9.0));
  EXPECT_EQ(out.chassis, after_mode);
  Send(drivetrain::kTeleopTargetTopic, Chassis(1.5));
  EXPECT_EQ(out.chassis, after_mode + 1);
  EXPECT_NEAR(Last<ChassisTarget>(drivetrain::kTargetTopic).vx_mps(), 1.5,
              1e-12);

  Mode(kAutoEnabled);
  const int after_switch = out.chassis;
  Send(drivetrain::kTeleopTargetTopic, Chassis(1.5));
  EXPECT_EQ(out.chassis, after_switch);
  Send(drivetrain::kAutoTargetTopic, Chassis(-2.0));
  EXPECT_NEAR(Last<ChassisTarget>(drivetrain::kTargetTopic).vx_mps(), -2.0,
              1e-12);
}

TEST_F(ArbiterTest, StopsOnceOnEveryModeChangeSoNoTargetOutlivesItsMode) {
  Mode(kAutoEnabled);
  Send(drivetrain::kAutoTargetTopic, Chassis(3.0));
  ASSERT_NEAR(Last<ChassisTarget>(drivetrain::kTargetTopic).vx_mps(), 3.0,
              1e-12);

  // Switching to teleop must not leave the autonomous request live on the
  // topic for a command timeout while the driver is already in control.
  const int before = out.chassis;
  Mode(kTeleopEnabled);
  EXPECT_EQ(out.chassis, before + 1);
  const auto released = Last<ChassisTarget>(drivetrain::kTargetTopic);
  EXPECT_FALSE(released.enabled());
  EXPECT_NEAR(released.vx_mps(), 0.0, 1e-12);
  EXPECT_NEAR(released.vy_mps(), 0.0, 1e-12);
  EXPECT_NEAR(released.omega_radps(), 0.0, 1e-12);
  EXPECT_FALSE(Last<ShooterTarget>(shooter::kTargetTopic).enabled());
  EXPECT_FALSE(Last<IntakeTarget>(intake::kTargetTopic).enabled());

  // Repeating the same mode is not a change, so it does not re-emit.
  const int after = out.chassis;
  Mode(kTeleopEnabled);
  EXPECT_EQ(out.chassis, after);
}

TEST_F(ArbiterTest, ForwardsNothingBeforeTheFirstDriverStationPacket) {
  Send(drivetrain::kTeleopTargetTopic, Chassis(1.0));
  Send(drivetrain::kAutoTargetTopic, Chassis(1.0));
  Send(shooter::kTeleopTargetTopic, ShooterTarget{60.0, 0, true});
  EXPECT_EQ(out.chassis, 0);
  EXPECT_EQ(out.shooter, 0);
}

TEST_F(ArbiterTest, RoutesShooterTargetsOnTheSameSelection) {
  Mode(kTeleopEnabled);
  const int before = out.shooter;
  Send(shooter::kAutoTargetTopic, ShooterTarget{99.0, 0, true});
  EXPECT_EQ(out.shooter, before);
  Send(shooter::kTeleopTargetTopic, ShooterTarget{60.0, 0, true});
  EXPECT_EQ(out.shooter, before + 1);
  EXPECT_NEAR(Last<ShooterTarget>(shooter::kTargetTopic).target_velocity_rps(),
              60.0, 1e-12);
}

TEST_F(ArbiterTest, RoutesIntakeTargetsOnTheSameSelection) {
  Mode(kTeleopEnabled);
  const int before = out.intake;
  Send(intake::kAutoTargetTopic, IntakeTarget{0, true, 99.0f});
  EXPECT_EQ(out.intake, before);
  Send(intake::kTeleopTargetTopic, IntakeTarget{0, true, 30.0f});
  EXPECT_EQ(out.intake, before + 1);
  EXPECT_NEAR(Last<IntakeTarget>(intake::kTargetTopic).roller_velocity_rps(),
              30.0, 1e-6);
}

TEST(Arbiter, DisabledEStopAndTestDriveNothing) {
  for (const std::uint32_t flags :
       {std::uint32_t{ds::kDsAttached},
        std::uint32_t{ds::kEnabled | ds::kTeleop | ds::kEStop},
        std::uint32_t{ds::kEnabled | ds::kTest}}) {
    Sim sim{RobotConfig()};
    sim.add<Arbiter>("arbiter");
    Probe& out = sim.add<Probe>("probe");
    sim.publish(driver_station::kStateTopic, DsState(flags));
    sim.step(10ms);
    sim.publish(drivetrain::kTeleopTargetTopic, Chassis(1.0));
    sim.publish(drivetrain::kAutoTargetTopic, Chassis(1.0));
    sim.step(10ms);
    EXPECT_EQ(out.chassis, 0) << flags;
  }
}

// The whole teleop path, with the same four nodes the launcher starts and
// nothing stubbed: a joystick packet lands on /hw/state/driver_station, the
// driver station node decodes it, the operator interface maps it to a
// request, the arbiter admits it because the mode says teleop, and the
// drivetrain resolves it into per-module swerve requests.
class TeleopChain : public ::testing::Test {
 protected:
  void Push(const ds::DriverStationData& data) {
    talos::hardware::Packet packet{};
    packet.size = static_cast<std::uint32_t>(ds::Encode(data, packet.data));
    sim.publish(talos::hardware::kDriverStationTopic, packet);
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
  driver_station::DriverStation& driver_station_node =
      sim.add<driver_station::DriverStation>("driver_station");
  operator_interface::OperatorInterface& operator_interface_node =
      sim.add<operator_interface::OperatorInterface>("operator_interface");
  Arbiter& arbiter_node = sim.add<Arbiter>("arbiter");
  drivetrain::Drivetrain& drivetrain_node =
      sim.add<drivetrain::Drivetrain>("drivetrain");
  talos::testing::SimHardware hw{sim};
  const drivetrain::SwerveGeometry geometry =
      drivetrain::BuildSwerveGeometry(talos::config::ParseRobotConfig(RobotConfig()),
                                      "drivetrain");

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
  for (const auto& module : geometry.modules)
    EXPECT_NEAR(hw.motor_request("drivetrain", module.drive).demand, 0.0, 1e-9)
        << module.name;
}

}  // namespace
}  // namespace arbiter
