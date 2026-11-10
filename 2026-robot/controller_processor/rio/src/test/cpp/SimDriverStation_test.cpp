// The desktop-only simulated Driver Station that headless runs (sim.sh
// --headless, tools/test_drivetrain.py --wpilib) enable the robot with.
#include "SimDriverStation.h"

#ifndef __FRC_ROBORIO__

#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>
#include <hal/HAL.h>

#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "DriverStationReader.h"

namespace {

using talos::rio::ParseSimJoystickSchedule;
using talos::rio::ParseSimSchedule;
using talos::rio::SimDriverStation;
using namespace std::chrono_literals;

// Settings built by assignment: GCC's -Wmissing-field-initializers rejects
// a designated initializer that leaves any member out.
SimDriverStation::Settings Active() {
  SimDriverStation::Settings settings;
  settings.active = true;
  return settings;
}

struct Mode {
  bool enabled;
  bool autonomous;
};

Mode Published() {
  return {frc::sim::DriverStationSim::GetEnabled(),
          frc::sim::DriverStationSim::GetAutonomous()};
}

// The Driver Station simulation is process-wide; leave it as the
// PhoenixBackend tests expect it, disabled teleop.
class SimDriverStationTest : public ::testing::Test {
 protected:
  void SetUp() override { HAL_Initialize(500, 0); }
  void TearDown() override {
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::SetJoystickAxisCount(0, 0);
    frc::sim::DriverStationSim::SetJoystickButtonCount(0, 0);
    frc::sim::DriverStationSim::SetJoystickPOVCount(0, 0);
    frc::sim::DriverStationSim::NotifyNewData();
  }
};

// A schedule must not enable before the gateway is configured (the first
// configuration push needs disabled operation), then runs each phase from
// that moment, autonomous included, and holds the last one.
TEST_F(SimDriverStationTest, ScheduleStartsAtConfigurationAndHoldsLastPhase) {
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h;
  auto settings = Active();
  settings.schedule =
      ParseSimSchedule("teleop:1,auto:0.5,disabled:0.25,teleop:2");
  SimDriverStation ds{settings, t0};
  ds.Update(t0 + 10s, false);
  EXPECT_FALSE(Published().enabled);

  const auto configured = t0 + 10s + 5ms;
  ds.Update(configured, true);
  EXPECT_TRUE(Published().enabled);
  EXPECT_FALSE(Published().autonomous);
  ds.Update(configured + 999ms, true);
  EXPECT_FALSE(Published().autonomous);
  ds.Update(configured + 1s, true);
  EXPECT_TRUE(Published().enabled);
  EXPECT_TRUE(Published().autonomous);
  ds.Update(configured + 1500ms, true);
  EXPECT_FALSE(Published().enabled);
  ds.Update(configured + 1750ms, true);
  EXPECT_TRUE(Published().enabled);
  EXPECT_FALSE(Published().autonomous);
  ds.Update(configured + 1h, true);
  EXPECT_TRUE(Published().enabled);
}

// TALOS_SIM_DS=auto with a disable deadline (tools/test_drivetrain.py
// --wpilib) must stay disabled once it disables, not re-enable on the next
// tick because the gateway is still configured.
TEST_F(SimDriverStationTest, AutoEnableDisablesOnceAtDeadline) {
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h;
  auto settings = Active();
  settings.enable_when_configured = true;
  settings.disable_after = std::chrono::milliseconds{1000};
  SimDriverStation ds{settings, t0};
  ds.Update(t0 + 100ms, true);
  EXPECT_TRUE(Published().enabled);
  ds.Update(t0 + 1s, true);
  EXPECT_FALSE(Published().enabled);
  ds.Update(t0 + 1005ms, true);
  EXPECT_FALSE(Published().enabled);
  ds.Update(t0 + 1010ms, true);
  EXPECT_FALSE(Published().enabled);
}

// What the gateway would send: the Driver Station packet sampled the way
// Robot.cpp samples it, through WPILib.
talos::driver_station::DriverStationData Sampled() {
  frc::DriverStation::RefreshData();
  return talos::driver_station::SampleDriverStation(1);
}

// A scripted joystick must reach the gateway's Driver Station packet as a
// real stick would: absent until the gateway is configured (no deflection
// before the robot can be enabled), each snapshot at its time from that
// moment, and still reported after a disable, which the packet's flags (not
// a vanished stick) carry to the driver.
TEST_F(SimDriverStationTest, JoystickScheduleReachesDriverStationPacket) {
  namespace ds = talos::driver_station;
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h;
  auto settings = Active();
  settings.schedule = ParseSimSchedule("teleop:2,disabled:1");
  settings.joysticks = ParseSimJoystickSchedule(
      "0:axes=0,0,0;buttons=0;pov=-1|"
      "1:axes=0.25,-0.5,0;buttons=5;pov=90");
  SimDriverStation sim{settings, t0};
  sim.Update(t0 + 10s, false);
  EXPECT_FALSE(Sampled().joysticks[0].connected);

  const auto configured = t0 + 11s;
  sim.Update(configured, true);
  auto packet = Sampled();
  ASSERT_TRUE(packet.joysticks[0].connected);
  EXPECT_NE(packet.fms.flags & ds::kEnabled, 0u);
  EXPECT_EQ(packet.joysticks[0].axis_count, 3);
  EXPECT_EQ(packet.joysticks[0].axes[1], 0.0f);

  sim.Update(configured + 999ms, true);
  EXPECT_EQ(Sampled().joysticks[0].axes[1], 0.0f);

  sim.Update(configured + 1s, true);
  packet = Sampled();
  EXPECT_FLOAT_EQ(packet.joysticks[0].axes[0], 0.25f);
  EXPECT_FLOAT_EQ(packet.joysticks[0].axes[1], -0.5f);
  EXPECT_EQ(packet.joysticks[0].buttons, 5u);
  ASSERT_EQ(packet.joysticks[0].pov_count, 1);
  EXPECT_EQ(packet.joysticks[0].povs[0], 90);

  sim.Update(configured + 2s, true);
  packet = Sampled();
  EXPECT_EQ(packet.fms.flags & ds::kEnabled, 0u);
  EXPECT_FLOAT_EQ(packet.joysticks[0].axes[1], -0.5f);
}

// Without a named alliance the pose estimator never trusts its seed, so
// autonomous DRIVE aborts: the stand-in must always name one, blue unless
// TALOS_SIM_DS_ALLIANCE says red, and refuse anything else.
TEST_F(SimDriverStationTest, NamesTheConfiguredAlliance) {
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h;
  {
    SimDriverStation blue{Active(), t0};
    frc::DriverStation::RefreshData();
    EXPECT_EQ(frc::DriverStation::GetAlliance(),
              frc::DriverStation::Alliance::kBlue);
  }
  ::setenv("TALOS_SIM_DS", "0", 1);
  ::setenv("TALOS_SIM_DS_ALLIANCE", "red", 1);
  {
    SimDriverStation red{SimDriverStation::SettingsFromEnvironment(), t0};
    frc::DriverStation::RefreshData();
    EXPECT_EQ(frc::DriverStation::GetAlliance(),
              frc::DriverStation::Alliance::kRed);
  }
  ::setenv("TALOS_SIM_DS_ALLIANCE", "green", 1);
  EXPECT_THROW(SimDriverStation::SettingsFromEnvironment(),
               std::invalid_argument);
  ::unsetenv("TALOS_SIM_DS_ALLIANCE");
  ::unsetenv("TALOS_SIM_DS");
}

TEST(SimScheduleParse, RejectsMalformedJoystickSchedules) {
  constexpr const char* kStick = "axes=0;buttons=0;pov=-1";
  for (const std::string& spec :
       {std::string{}, std::string{"0"}, std::string{"x:"} + kStick,
        std::string{"-1:"} + kStick, std::string{"nan:"} + kStick,
        std::string{"1:"} + kStick + "|1:" + kStick,
        std::string{"0:"} + kStick + "|", std::string{"0:axes=0;buttons=0"},
        std::string{"0:axes=1.5;buttons=0;pov=-1"},
        std::string{"0:axes=0,0,0,0,0,0,0,0,0,0,0,0,0;buttons=0;pov=-1"},
        std::string{"0:axes=0;buttons=-1;pov=-1"},
        std::string{"0:axes=0;buttons=4294967296;pov=-1"},
        std::string{"0:axes=0;buttons=0;pov=360"},
        std::string{"0:axes=0;buttons=0;pov=45.5"},
        std::string{"0:axes=0;axes=0;buttons=0;pov=-1"},
        std::string{"0:axes=0;buttons=0;pov=-1;rumble=1"}}) {
    EXPECT_THROW(ParseSimJoystickSchedule(spec), std::invalid_argument) << spec;
  }
  EXPECT_EQ(
      ParseSimJoystickSchedule(std::string{"0:"} + kStick + "|0.5:" + kStick)
          .size(),
      2u);
}

TEST(SimScheduleParse, RejectsMalformedSchedules) {
  for (const char* spec : {"", "teleop", "fly:1", "teleop:0", "teleop:-1",
                           "teleop:nan", "teleop:1x", "teleop:1,", ",auto:1"}) {
    EXPECT_THROW(ParseSimSchedule(spec), std::invalid_argument) << spec;
  }
}

}  // namespace

#endif  // __FRC_ROBORIO__
