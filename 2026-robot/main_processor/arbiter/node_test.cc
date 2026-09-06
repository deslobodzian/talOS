#include "2026-robot/main_processor/arbiter/node.h"
#include "2026-robot/main_processor/driver_station/node.h"

#include <gtest/gtest.h>

#include <chrono>

#include "2026-robot/main_processor/drivetrain/geometry.h"
#include "2026-robot/main_processor/drivetrain/node.h"
#include "2026-robot/main_processor/operator_interface/node.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/hardware/sim_backend.h"

namespace talos::arbiter {
namespace {

using namespace std::chrono_literals;
using driver_station::DriverStationState;
using driver_station::FmsState;
using talos::drive::ChassisTarget;
using talos::intake::IntakeTarget;
using talos::shooter::ShooterTarget;

DriverStationState DsState(uint32_t flags) {
  return DriverStationState{0, 0, FmsState{flags, 0, 0, 0, 0, 0, 0.0f},
                            driver_station::JoystickState{},
                            driver_station::JoystickState{}};
}

constexpr uint32_t kTeleopEnabled =
    driver_station::kEnabled | driver_station::kTeleop;
constexpr uint32_t kAutoEnabled =
    driver_station::kEnabled | driver_station::kAutonomous;

ChassisTarget Chassis(double vx) { return ChassisTarget{vx, 0, 0, 0, true}; }

class Harness {
 public:
  Harness() : loop_{env_}, node_{loop_} {}

  void Mode(uint32_t flags) {
    loop_.inject(driver_station::kDsStateTopic, DsState(flags));
    loop_.run_for(10ms);
  }
  void Send(const char* topic, const ChassisTarget& target) {
    loop_.inject(topic, target);
    loop_.run_for(10ms);
  }
  void Send(const char* topic, const ShooterTarget& target) {
    loop_.inject(topic, target);
    loop_.run_for(10ms);
  }
  void Send(const char* topic, const IntakeTarget& target) {
    loop_.inject(topic, target);
    loop_.run_for(10ms);
  }

  uint64_t chassis_count() {
    return env_.channel(talos::drive::kTargetTopic, sizeof(ChassisTarget))
        .next_sequence();
  }
  uint64_t shooter_count() {
    return env_
        .channel(talos::shooter::kShooterTargetTopic, sizeof(ShooterTarget))
        .next_sequence();
  }
  uint64_t intake_count() {
    return env_
        .channel(talos::intake::kTargetTopic, sizeof(IntakeTarget))
        .next_sequence();
  }
  ChassisTarget last_chassis() {
    auto& channel =
        env_.channel(talos::drive::kTargetTopic, sizeof(ChassisTarget));
    ChassisTarget target{};
    EXPECT_GT(channel.next_sequence(), 0u);
    if (channel.next_sequence() == 0) return target;
    channel.copy_to(channel.next_sequence() - 1,
                    {reinterpret_cast<std::byte*>(&target), sizeof(target)});
    return target;
  }
  ShooterTarget last_shooter() {
    auto& channel = env_.channel(talos::shooter::kShooterTargetTopic,
                                 sizeof(ShooterTarget));
    ShooterTarget target{};
    EXPECT_GT(channel.next_sequence(), 0u);
    if (channel.next_sequence() == 0) return target;
    channel.copy_to(channel.next_sequence() - 1,
                    {reinterpret_cast<std::byte*>(&target), sizeof(target)});
    return target;
  }
  IntakeTarget last_intake() {
    auto& channel = env_.channel(talos::intake::kTargetTopic,
                                 sizeof(IntakeTarget));
    IntakeTarget target{};
    EXPECT_GT(channel.next_sequence(), 0u);
    if (channel.next_sequence() == 0) return target;
    channel.copy_to(channel.next_sequence() - 1,
                    {reinterpret_cast<std::byte*>(&target), sizeof(target)});
    return target;
  }
  ArbiterNode<event::SimulatedEventLoop<>>& node() { return node_; }

 private:
  event::SimulationEnvironment env_;
  event::SimulatedEventLoop<> loop_;
  ArbiterNode<event::SimulatedEventLoop<>> node_;
};

TEST(Arbiter, ForwardsOnlyTheProducerTheMatchModeSelects) {
  Harness harness;
  harness.Mode(kTeleopEnabled);
  ASSERT_EQ(harness.node().source(), Source::kTeleop);
  const auto after_mode = harness.chassis_count();

  // The unselected producer is free to keep publishing; none of it gets out.
  harness.Send(kAutoChassisTopic, Chassis(9.0));
  EXPECT_EQ(harness.chassis_count(), after_mode);

  harness.Send(kTeleopChassisTopic, Chassis(1.5));
  EXPECT_EQ(harness.chassis_count(), after_mode + 1);
  EXPECT_NEAR(harness.last_chassis().vx_mps(), 1.5, 1e-12);

  harness.Mode(kAutoEnabled);
  ASSERT_EQ(harness.node().source(), Source::kAutonomous);
  const auto after_switch = harness.chassis_count();
  harness.Send(kTeleopChassisTopic, Chassis(1.5));
  EXPECT_EQ(harness.chassis_count(), after_switch);
  harness.Send(kAutoChassisTopic, Chassis(-2.0));
  EXPECT_NEAR(harness.last_chassis().vx_mps(), -2.0, 1e-12);
}

TEST(Arbiter, StopsOnceOnEveryModeChangeSoNoTargetOutlivesItsMode) {
  Harness harness;
  harness.Mode(kAutoEnabled);
  harness.Send(kAutoChassisTopic, Chassis(3.0));
  ASSERT_NEAR(harness.last_chassis().vx_mps(), 3.0, 1e-12);

  // Switching to teleop must not leave the autonomous request live on the
  // topic for a command timeout while the driver is already in control.
  const auto before = harness.chassis_count();
  harness.Mode(kTeleopEnabled);
  EXPECT_EQ(harness.chassis_count(), before + 1);
  const auto released_intake = harness.last_intake();
  EXPECT_FALSE(released_intake.enabled());
  const auto released = harness.last_chassis();
  EXPECT_FALSE(released.enabled());
  EXPECT_NEAR(released.vx_mps(), 0.0, 1e-12);
  EXPECT_NEAR(released.vy_mps(), 0.0, 1e-12);
  EXPECT_NEAR(released.omega_radps(), 0.0, 1e-12);
  EXPECT_FALSE(harness.last_shooter().enabled());

  // Repeating the same mode is not a change, so it does not re-emit.
  const auto after = harness.chassis_count();
  harness.Mode(kTeleopEnabled);
  EXPECT_EQ(harness.chassis_count(), after);
}

TEST(Arbiter, DisabledEStopAndTestDriveNothing) {
  for (const uint32_t flags :
       {uint32_t{driver_station::kDsAttached},
        uint32_t{driver_station::kEnabled | driver_station::kTeleop |
                 driver_station::kEStop},
        uint32_t{driver_station::kEnabled | driver_station::kTest}}) {
    Harness harness;
    harness.Mode(flags);
    EXPECT_EQ(harness.node().source(), Source::kNone);
    const auto before = harness.chassis_count();
    harness.Send(kTeleopChassisTopic, Chassis(1.0));
    harness.Send(kAutoChassisTopic, Chassis(1.0));
    EXPECT_EQ(harness.chassis_count(), before);
  }
}

TEST(Arbiter, ForwardsNothingBeforeTheFirstDriverStationPacket) {
  Harness harness;
  EXPECT_EQ(harness.node().source(), Source::kNone);
  harness.Send(kTeleopChassisTopic, Chassis(1.0));
  harness.Send(kAutoChassisTopic, Chassis(1.0));
  EXPECT_EQ(harness.chassis_count(), 0u);
  EXPECT_EQ(harness.shooter_count(), 0u);
}

TEST(Arbiter, RoutesShooterTargetsOnTheSameSelection) {
  Harness harness;
  harness.Mode(kTeleopEnabled);
  const auto before = harness.shooter_count();
  harness.Send(kAutoShooterTopic, ShooterTarget{99.0, 0, true});
  EXPECT_EQ(harness.shooter_count(), before);
  harness.Send(kTeleopShooterTopic, ShooterTarget{60.0, 0, true});
  EXPECT_EQ(harness.shooter_count(), before + 1);
  EXPECT_NEAR(harness.last_shooter().target_velocity_rps(), 60.0, 1e-12);
}

TEST(Arbiter, RoutesIntakeTargetsOnTheSameSelection) {
  Harness harness;
  harness.Mode(kTeleopEnabled);
  const auto before = harness.intake_count();
  harness.Send(kAutoIntakeTopic, IntakeTarget{0, true, 99.0f});
  EXPECT_EQ(harness.intake_count(), before);
  harness.Send(kTeleopIntakeTopic, IntakeTarget{0, true, 30.0f});
  EXPECT_EQ(harness.intake_count(), before + 1);
  EXPECT_NEAR(harness.last_intake().roller_velocity_rps(), 30.0, 1e-6);
}

// A minimal enabled-teleop packet with the stick pushed straight downfield.
driver_station::DriverStationData ForwardStick() {
  driver_station::DriverStationData data{};
  data.fms.flags = driver_station::kEnabled | driver_station::kTeleop |
                   driver_station::kDsAttached;
  data.joysticks[0].connected = true;
  data.joysticks[0].axis_count = 4;
  data.joysticks[0].axes[1] = -1.0f;  // Full forward: raw vx = +1.
  return data;
}

talos::drive::Packet CreatePacket(const driver_station::DriverStationData& d) {
  talos::drive::Packet packet{};
  packet.size = static_cast<uint32_t>(driver_station::Encode(d, packet.data));
  return packet;
}

// The whole teleop path in one loop: a joystick packet lands on
// /hw/state/driver_station, the driver station node decodes it, the operator
// interface maps it to a request, the arbiter admits it because the mode says
// teleop, and the drivetrain resolves it into per-module swerve requests on
// /hw/request/drivetrain. Nothing here is stubbed -- these are the same four
// nodes the launcher starts.
class TeleopChain {
 public:
  TeleopChain()
      : robot_config_{Parse()},
        loop_{env_},
        driver_station_node_{loop_},
        operator_interface_node_{loop_, RobotRelativeTeleop()},
        arbiter_node_{loop_},
        drivetrain_node_{loop_, robot_config_.hardware,
                         talos::drive::BuildSwerveGeometry(robot_config_),
                         Devices(robot_config_)} {
    drivetrain_node_.Start(event::MonotonicTime::from_nanos(1'000'000));
    loop_.inject(talos::drive::kStateTopic, InitialStatePacket());
    loop_.run_for(10ms);
  }

  event::SimulatedEventLoop<>& loop() { return loop_; }
  event::SimulationEnvironment& env() { return env_; }

  hardware::Command Push(const driver_station::DriverStationData& data) {
    // One packet is enough: the driver station node publishes its state before
    // its target, so the arbiter knows the match mode by the time the target
    // reaches it and nothing is dropped on the first frame after enable.
    loop_.inject(driver_station::kHwDsTopic, CreatePacket(data));
    loop_.run_for(10ms);
    auto& channel = env_.channel(talos::drive::kDriveRequestTopic,
                                 sizeof(talos::drive::Packet));
    EXPECT_GT(channel.next_sequence(), 0u);
    talos::drive::Packet packet{};
    channel.copy_to(channel.next_sequence() - 1,
                    {reinterpret_cast<std::byte*>(&packet), sizeof(packet)});
    hardware::Command command{};
    EXPECT_TRUE(hardware::Decode(packet.bytes(), command));
    return command;
  }

 private:
  static config::RobotConfig Parse() {
    auto config = config::ParseRobotConfig(
        "2026-robot/main_processor/configuration/robot.toml");
    config.hardware.commissioned = true;
    return config;
  }
  static hardware::Devices Devices(const config::RobotConfig& config) {
    const auto* devices = config.GetDevices("drivetrain");
    return devices ? *devices : hardware::Devices{};
  }
  static talos::oi::OperatorInterfaceConfig RobotRelativeTeleop() {
    talos::oi::OperatorInterfaceConfig teleop{};
    teleop.deadband = 0.0;
    teleop.field_oriented = false;
    return teleop;
  }
  talos::drive::Packet InitialStatePacket() {
    hardware::SimBackend backend;
    hardware::Gateway gateway{robot_config_.hardware, backend, 42};
    gateway.Tick(1000, true);
    talos::drive::Packet packet{};
    packet.size = static_cast<uint32_t>(
        hardware::Encode(gateway.snapshot(), packet.data));
    return packet;
  }

  config::RobotConfig robot_config_;
  event::SimulationEnvironment env_;
  event::SimulatedEventLoop<> loop_;
  driver_station::DriverStationNode<event::SimulatedEventLoop<>>
      driver_station_node_;
  talos::oi::OperatorInterfaceNode<event::SimulatedEventLoop<>>
      operator_interface_node_;
  ArbiterNode<event::SimulatedEventLoop<>> arbiter_node_;
  talos::drive::DrivetrainNode<event::SimulatedEventLoop<>> drivetrain_node_;
};

TEST(TeleopChainTest, JoystickDeflectionReachesTheSwerveModulesAsVelocity) {
  TeleopChain chain;
  const auto command = chain.Push(ForwardStick());

  // 4 m/s over a 0.0508 m wheel, and below the 20 rps drive ceiling, so the
  // request is not desaturated.
  const double expected_rps = 4.0 / (2 * std::numbers::pi * 0.0508);
  for (int module = 0; module < 4; ++module) {
    EXPECT_EQ(command.motors[2 * module].mode, hardware::Mode::kVelocity);
    EXPECT_NEAR(command.motors[2 * module].demand, expected_rps, 1e-9);
    EXPECT_EQ(command.motors[2 * module + 1].mode, hardware::Mode::kPosition);
    EXPECT_NEAR(command.motors[2 * module + 1].demand, 0.0, 1e-9);
  }
}

TEST(TeleopChainTest,
     ReleasingTheStickStopsTheModulesWithoutWaitingForTimeout) {
  TeleopChain chain;
  ASSERT_EQ(chain.Push(ForwardStick()).motors[0].mode,
            hardware::Mode::kVelocity);

  driver_station::DriverStationData centered = ForwardStick();
  centered.joysticks[0].axes[1] = 0.0f;
  const auto command = chain.Push(centered);
  for (int module = 0; module < 4; ++module)
    EXPECT_NEAR(command.motors[2 * module].demand, 0.0, 1e-9);
}

}  // namespace
}  // namespace talos::arbiter
