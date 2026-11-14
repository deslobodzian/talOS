#include "talOS/teleop/node.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

#include "motor/motor_message_generated.h"
#include "swerve_module/swerve_module.h"
#include "talOS/devices/testing.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/node/testing.h"

namespace swerve_module {
namespace {
using namespace std::chrono_literals;
using talos::hardware::Mode;
using talos::testing::AllocationProbe;
using talos::testing::DriverStationGeneration;
using talos::testing::DsPacket;
using talos::testing::PublishTarget;
using talos::testing::ReleaseTarget;
namespace ds = talos::driver_station;
// The bench config plus the swerve module's subsystem file (see the file).
constexpr const char* kConfig =
    "bench/main_processor/swerve_module/testdata/robot.toml";
constexpr const char* kNode = "swerve_module";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kTwoPi = 2.0 * std::numbers::pi;
// Tolerance for a setpoint that goes through a cosine and a rotation wrap.
constexpr double kTight = 1e-12;

ModuleTarget Drive(double velocity_rps, double steer_rot) {
  return {ModuleMode::DRIVE, velocity_rps, steer_rot};
}

// The node test matrix, docs/TESTING.md §7: the case names are the matrix's,
// which //tools:check_node_matrix looks for. This node drives the bench
// swerve module, so the safety cases assert what the gateway applies, after
// its gates, besides what the node requested, on both motors.
class SwerveModuleTest : public ::testing::Test {
 protected:
  void Enable() {
    hw.enable();
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(50ms);
  }
  void Send(ModuleTarget target) {
    PublishTarget(sim, kNode, "teleop", target);
    sim.step(10ms);
  }
  // Holds a lane the way a producer does: republished every 10 ms.
  void Hold(ModuleTarget target, std::chrono::milliseconds duration,
            const char* source = "teleop") {
    for (auto t = 0ms; t < duration; t += 10ms) {
      PublishTarget(sim, kNode, source, target);
      sim.step(10ms);
    }
  }
  ModuleState State() {
    return sim.latest<ModuleState>(topics::kState).value();
  }
  talos::hardware::MotorRequest Applied(const char* motor) const {
    return hw.applied_motor(kNode, motor);
  }
  // Both motors driven: the drive in velocity, the steer in position.
  void Driving() {
    ASSERT_EQ(Applied("drive").mode, Mode::kVelocity);
    ASSERT_EQ(Applied("steer").mode, Mode::kPosition);
  }
  void Neutral() {
    for (const char* motor : {"drive", "steer"}) {
      SCOPED_TRACE(motor);
      EXPECT_EQ(hw.motor_request(kNode, motor).mode, Mode::kNeutral);
      EXPECT_EQ(Applied(motor).mode, Mode::kNeutral);
    }
    EXPECT_EQ(State().mode(), ModuleMode::NEUTRAL);
    EXPECT_EQ(State().source(), TargetSource::NONE);
    EXPECT_EQ(State().commanded_drive_velocity_rps(), 0.0);
    EXPECT_EQ(State().commanded_steer_position_rot(), 0.0);
    EXPECT_FALSE(State().reversed());
  }
  talos::testing::Sim sim{kConfig};
  BenchSwerveModule& module = sim.add<BenchSwerveModule>(kNode);
  talos::testing::SimHardware hw{sim};
};

// REQ-SAFE-07, REQ-DEV-26: the steer motor (absolute frame, so no rezero)
// goes to the asked heading by the short way, and the drive runs at the
// cosine-scaled speed; the state shows the feedback and the setpoint.
TEST_F(SwerveModuleTest, DrivesAtALiveLane) {
  hw.set_motor(kNode, "steer", 0.1, 0.0);
  hw.set_motor(kNode, "drive", 12.0, 3.0);
  hw.set_cancoder(kNode, "encoder", 0.1, 0.0);
  Enable();
  Send(Drive(5.0, 0.15));
  const double drive_rps = 5.0 * std::cos(kTwoPi * 0.05);
  EXPECT_EQ(Applied("drive").mode, Mode::kVelocity);
  EXPECT_NEAR(Applied("drive").demand, drive_rps, kTight);
  EXPECT_EQ(Applied("steer").mode, Mode::kPosition);
  EXPECT_NEAR(Applied("steer").demand, 0.15, kTight);
  const ModuleState state = State();
  EXPECT_EQ(state.mode(), ModuleMode::DRIVE);
  EXPECT_EQ(state.source(), TargetSource::TELEOP);
  EXPECT_TRUE(state.enabled());
  EXPECT_FALSE(state.reversed());
  EXPECT_NEAR(state.commanded_drive_velocity_rps(), drive_rps, kTight);
  EXPECT_NEAR(state.commanded_steer_position_rot(), 0.15, kTight);
  EXPECT_DOUBLE_EQ(state.drive_position_rot(), 12.0);
  EXPECT_DOUBLE_EQ(state.drive_velocity_rps(), 3.0);
  EXPECT_DOUBLE_EQ(state.steer_position_rot(), 0.1);
  EXPECT_DOUBLE_EQ(state.encoder_absolute_position_rot(), 0.1);
  EXPECT_TRUE(state.drive_connected());
  EXPECT_TRUE(state.steer_connected());
  EXPECT_TRUE(state.encoder_connected());
}

// REQ-DEV-26: asked 0.4 rot with the steer 3 turns along (unwrapped), the
// module turns 0.1 rot back to the opposite heading, 2.9 rot, instead of
// 0.4 rot forward, and reverses the drive.
TEST_F(SwerveModuleTest, ReversesPastAQuarterTurn) {
  hw.set_motor(kNode, "steer", 3.0, 0.0);
  Enable();
  Send(Drive(5.0, 0.4));
  const double drive_rps = -5.0 * std::cos(kTwoPi * 0.1);
  EXPECT_EQ(Applied("drive").mode, Mode::kVelocity);
  EXPECT_NEAR(Applied("drive").demand, drive_rps, 1e-9);
  EXPECT_EQ(Applied("steer").mode, Mode::kPosition);
  EXPECT_NEAR(Applied("steer").demand, 2.9, 1e-9);
  EXPECT_TRUE(State().reversed());
}

// REQ-SAFE-02
TEST_F(SwerveModuleTest, DisabledRobotGoesNeutral) {
  Enable();
  Send(Drive(5.0, 0.0));
  Driving();
  hw.enable(false);
  Hold(Drive(5.0, 0.0), 30ms);
  Neutral();
  EXPECT_FALSE(State().enabled());  // state keeps publishing
  EXPECT_TRUE(State().drive_connected());
}

// REQ-SAFE-07
TEST_F(SwerveModuleTest, StaleLaneGoesNeutral) {
  hw.set_cancoder(kNode, "encoder", -0.2, 0.0);
  Enable();
  Send(Drive(5.0, 0.0));
  Driving();
  // The lane was published 10 ms ago; 95 ms on it is past
  // command_timeout_us (100 ms) by one period (5 ms).
  sim.step(95ms);
  Neutral();
  EXPECT_TRUE(State().enabled());  // the robot is enabled; the lane is not
  EXPECT_DOUBLE_EQ(State().encoder_absolute_position_rot(), -0.2);
}

// REQ-SAFE-07
TEST_F(SwerveModuleTest, ReleasedLaneGoesNeutral) {
  Enable();
  Send(Drive(5.0, 0.0));
  Driving();
  ReleaseTarget<ModuleTarget>(sim, kNode, "teleop");
  sim.step(10ms);
  Neutral();
}

// REQ-SAFE-07
TEST_F(SwerveModuleTest, NeverSentGoesNeutral) {
  Enable();
  sim.step(50ms);
  Neutral();
  EXPECT_TRUE(State().enabled());
}

// REQ-SAFE-07
TEST_F(SwerveModuleTest, DriverStationLossGoesNeutral) {
  Enable();
  Send(Drive(5.0, 0.0));
  Driving();
  hw.stop_driver_station();      // the bridge stops forwarding DS packets
  Hold(Drive(5.0, 0.0), 150ms);  // past command_timeout_us: the DS is stale
  Neutral();
}

// REQ-CONTRACT-25: losing any one of the three devices stops both motors,
// the CANcoder alone included (the steer motor's own feedback stays valid,
// so only the node's gate stops the drive). The other two keep the robot
// enabled.
TEST_F(SwerveModuleTest, DisconnectedFeedbackGoesNeutral) {
  struct Device {
    const char* name;
    bool (ModuleState::*connected)() const;
  };
  Enable();
  for (const Device& device :
       {Device{"encoder", &ModuleState::encoder_connected},
        Device{"drive", &ModuleState::drive_connected},
        Device{"steer", &ModuleState::steer_connected}}) {
    SCOPED_TRACE(device.name);
    Send(Drive(5.0, 0.0));
    Driving();
    hw.set_connected(kNode, device.name, false);
    Hold(Drive(5.0, 0.0), 20ms);
    Neutral();
    EXPECT_TRUE(State().enabled());
    EXPECT_FALSE((State().*device.connected)());
    hw.set_connected(kNode, device.name, true);
  }
}

// REQ-SAFE-04
TEST_F(SwerveModuleTest, NonFiniteDemandGoesNeutral) {
  Enable();
  for (const ModuleTarget& target :
       {Drive(kNaN, 0.0), Drive(std::numeric_limits<double>::infinity(), 0.0),
        Drive(1.0, kNaN),
        Drive(1.0, -std::numeric_limits<double>::infinity())}) {
    SCOPED_TRACE(::testing::Message()
                 << "drive_velocity_rps " << target.drive_velocity_rps()
                 << " steer_angle_rot " << target.steer_angle_rot());
    Send(Drive(5.0, 0.0));
    Driving();
    Send(target);
    Neutral();
  }
}

// REQ-CONTRACT-18
TEST(SwerveModuleParameters, BadParameterFailsConstruction) {
  for (const char* value : {"0.0", "-1.0", "nan", "inf", "'ten'"}) {
    SCOPED_TRACE(value);
    auto sim = talos::testing::Sim::FromToml(
        std::string{"[robot]\nperiod_us = 5000\ncommand_timeout_us = 100000\n"
                    "[subsystems.swerve_module]\nmax_drive_velocity_rps = "} +
        value +
        "\n[subsystems.swerve_module.motors.drive]\ntype = 'TalonFX'\n"
        "can_id = 1\n"
        "[subsystems.swerve_module.motors.steer]\ntype = 'TalonFX'\n"
        "can_id = 2\n"
        "[subsystems.swerve_module.sensors.encoder]\ntype = 'CANcoder'\n"
        "can_id = 3\n");
    try {
      sim.add<BenchSwerveModule>(kNode);
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string{error.what()}.find("max_drive_velocity_rps"),
                std::string::npos)
          << error.what();
    }
  }
}

// REQ-SAFE-07
TEST_F(SwerveModuleTest, OtherModesLaneIsIgnored) {
  Enable();
  // The bench policy lists teleop's lane in teleop and none in autonomous.
  // An autonomous lane, live, in teleop:
  Hold(Drive(5.0, 0.0), 30ms, "auto");
  Neutral();
  ReleaseTarget<ModuleTarget>(sim, kNode, "auto");
  // The teleop lane, driving in teleop, then live in autonomous:
  Send(Drive(5.0, 0.0));
  Driving();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kAutonomous));
  sim.step(20ms);
  Hold(Drive(5.0, 0.0), 30ms);
  Neutral();
}

// REQ-SAFE-09
TEST_F(SwerveModuleTest, EarlierGenerationTargetIsIgnored) {
  Enable();
  Send(Drive(5.0, 0.0));
  Driving();
  const std::uint64_t before = DriverStationGeneration(sim);
  hw.set_driver_station(DsPacket(ds::kTeleop));  // disabled
  sim.step(40ms);
  // Re-enabled. SimHardware publishes the packet at its next 20 ms hook,
  // after the node's tick at that instant, so the packet waits for the
  // node's next drain; a target its producer queued under the earlier
  // enable, published before that drain, arrives in the same one.
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(20ms);
  ASSERT_NE(DriverStationGeneration(sim), before);
  PublishTarget(sim, kNode, "teleop", Drive(5.0, 0.0), before);
  sim.step(10ms);
  Neutral();
  // The same target stamped with the current generation drives at once.
  Send(Drive(5.0, 0.0));
  Driving();
}

// REQ-RT-06: the steady-state tick (target read, setpoint, two requests,
// flush, state publish) allocates nothing. The target stays fresh through
// the window (60 ms < command_timeout_us).
TEST_F(SwerveModuleTest, UpdateDoesNotAllocate) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  hw.set_motor(kNode, "steer", 0.1, 0.0);
  Enable();
  Hold(Drive(5.0, 0.4), 30ms);
  PublishTarget(sim, kNode, "teleop", Drive(5.0, 0.4));
  AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  Driving();
}

// The node's own cap (max_drive_velocity_rps, 10) holds below the drive
// motor's configured max_velocity_rps (30), either way.
TEST_F(SwerveModuleTest, DriveVelocityIsClamped) {
  Enable();
  for (const double asked : {25.0, -25.0}) {
    SCOPED_TRACE(asked);
    Send(Drive(asked, 0.0));
    EXPECT_EQ(Applied("drive").mode, Mode::kVelocity);
    EXPECT_DOUBLE_EQ(Applied("drive").demand, std::copysign(10.0, asked));
  }
}

// Unlike the drivetrain, a module asked for no speed still steers to its
// heading: the bench uses it to check the steering alone.
TEST_F(SwerveModuleTest, ZeroVelocityStillSteers) {
  Enable();
  Send(Drive(0.0, 0.2));
  EXPECT_EQ(Applied("steer").mode, Mode::kPosition);
  EXPECT_NEAR(Applied("steer").demand, 0.2, kTight);
  EXPECT_EQ(Applied("drive").mode, Mode::kVelocity);
  EXPECT_EQ(Applied("drive").demand, 0.0);
}

// The bench config binds the motor's lane too, and teleop needs a reader of
// every bound lane; this stands in for the motor node, whose package this
// one may not depend on.
struct MotorLaneReader {
  explicit MotorLaneReader(talos::Node& node) : target{node} {}
  void update() {}
  talos::Target<motor::MotorTarget> target;
};

// The subsystem file's bindings: the module moves only while button 6 is
// held, axis 5 drives, axis 4 steers, and release or disable stops it.
TEST(SwerveModuleTeleop, ConfiguredHoldToRunAxesReleaseAndDisable) {
  talos::testing::Sim sim{kConfig};
  sim.add<BenchSwerveModule>(kNode);
  sim.add<MotorLaneReader>("motor");
  sim.add<talos::teleop::Teleop>("teleop");
  talos::testing::SimHardware hw{sim};
  hw.enable();
  auto packet = DsPacket(ds::kEnabled | ds::kTeleop);
  auto& stick = packet.joysticks[0];
  stick.connected = true;
  stick.axis_count = 6;
  stick.button_count = 6;
  const auto push = [&] {
    hw.set_driver_station(packet);
    sim.step(20ms);
  };
  const auto drive = [&] { return hw.motor_request(kNode, "drive"); };
  const auto steer = [&] { return hw.motor_request(kNode, "steer"); };
  constexpr std::uint32_t kButton6 = 1U << 5;
  push();  // Neutral evaluation arms the producer.
  stick.axes[5] = -1.0F;
  push();  // An axis without the button sends a NEUTRAL target.
  EXPECT_EQ(drive().mode, Mode::kNeutral);
  EXPECT_EQ(steer().mode, Mode::kNeutral);
  stick.buttons = kButton6;
  push();
  EXPECT_EQ(drive().mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(drive().demand, 10.0);
  EXPECT_EQ(steer().mode, Mode::kPosition);
  EXPECT_DOUBLE_EQ(steer().demand, 0.0);
  // Halfway past the deadband: an eighth of a turn, with the drive scaled by
  // its cosine while the steer has not moved (no ideal motors). Axes travel
  // as floats, so 0.55 is off by about 1e-8.
  stick.axes[4] = 0.55F;
  push();
  EXPECT_NEAR(steer().demand, 0.125, 1e-6);
  EXPECT_NEAR(drive().demand, 10.0 * std::cos(kTwoPi * 0.125), 1e-6);
  stick.buttons = 0;
  push();
  EXPECT_EQ(drive().mode, Mode::kNeutral);
  EXPECT_EQ(steer().mode, Mode::kNeutral);
  stick.buttons = kButton6;
  push();
  ASSERT_EQ(drive().mode, Mode::kVelocity);
  packet.fms.flags = ds::kTeleop | ds::kDsAttached;
  hw.enable(false);
  push();
  EXPECT_EQ(drive().mode, Mode::kNeutral);
  EXPECT_EQ(steer().mode, Mode::kNeutral);
}

}  // namespace
}  // namespace swerve_module
