// The desktop-only simulated Driver Station that headless runs (sim.sh
// --headless, tools/test_rio_in_the_loop.py --wpilib) enable the robot with.
#include "SimDriverStation.h"

#ifndef __FRC_ROBORIO__

#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>
#include <hal/HAL.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "DriverStationReader.h"
#include "RioLog.h"

namespace {

using talos::driver_station::ParseJoystickSchedule;
using talos::driver_station::ParseModeSchedule;
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

// A schedule must not enable before the gateway acknowledges a
// configuration push (a push needs disabled operation), then runs each phase
// from that moment, autonomous included, and holds the last one.
TEST_F(SimDriverStationTest, ScheduleStartsAtConfigurationAndHoldsLastPhase) {
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h;
  auto settings = Active();
  settings.schedule =
      ParseModeSchedule("teleop:1,auto:0.5,disabled:0.25,teleop:2");
  SimDriverStation ds{settings, t0};
  ds.Update(t0 + 10s, false);
  EXPECT_FALSE(Published().enabled);

  const auto configured = t0 + 10s + 5ms;
  ds.ConfigurationAcknowledged();
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

// TALOS_SIM_DS=auto with a disable deadline (tools/test_rio_in_the_loop.py
// --wpilib) must stay disabled once it disables, not re-enable on the next
// tick because the gateway is still configured.
TEST_F(SimDriverStationTest, AutoEnableDisablesOnceAtDeadline) {
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h;
  auto settings = Active();
  settings.enable_when_configured = true;
  settings.disable_after = std::chrono::milliseconds{1000};
  SimDriverStation ds{settings, t0};
  ds.ConfigurationAcknowledged();
  ds.Update(t0 + 100ms, true);
  EXPECT_TRUE(Published().enabled);
  ds.Update(t0 + 1s, true);
  EXPECT_FALSE(Published().enabled);
  ds.Update(t0 + 1005ms, true);
  EXPECT_FALSE(Published().enabled);
  ds.Update(t0 + 1010ms, true);
  EXPECT_FALSE(Published().enabled);
}

// A gateway configured from the configuration a previous run left on disk
// is not yet configured by this run's main processor: enabling then would
// make the gateway refuse that push if it differs (a push needs disabled
// operation), so neither TALOS_SIM_DS=auto nor a schedule may start until a
// push is acknowledged in this boot, and a schedule's clock starts then.
// REQ-DEV-06
TEST_F(SimDriverStationTest, ARestoredConfigurationEnablesNothing) {
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h;
  auto automatic = Active();
  automatic.enable_when_configured = true;
  {
    SimDriverStation ds{automatic, t0};
    ds.Update(t0 + 1s, true);
    EXPECT_FALSE(Published().enabled)
        << "TALOS_SIM_DS=auto enabled on a restored configuration";
    ds.ConfigurationAcknowledged();
    ds.Update(t0 + 2s, true);
    EXPECT_TRUE(Published().enabled);
  }

  auto scheduled = Active();
  scheduled.schedule = ParseModeSchedule("teleop:1,disabled:1");
  SimDriverStation ds{scheduled, t0};
  ds.Update(t0 + 2s, true);
  EXPECT_FALSE(Published().enabled)
      << "the schedule started on a restored configuration";
  const auto acknowledged = t0 + 5s;
  ds.ConfigurationAcknowledged();
  ds.Update(acknowledged, true);
  EXPECT_TRUE(Published().enabled);
  ds.Update(acknowledged + 999ms, true);
  EXPECT_TRUE(Published().enabled)
      << "the schedule's clock started at the restore, not the push";
}

// What the gateway would send: a refresh, as the Driver Station reader's
// thread makes it, and its snapshot (SnapshotDriverStation).
talos::driver_station::DriverStationData Sampled() {
  frc::DriverStation::RefreshData();
  return talos::driver_station::SampleDriverStation(1);
}

// The Driver Station packet is one refresh whole only while the reader's
// thread is the program's one refresher of WPILib's cache
// (DriverStationReader.h). A scripted mode or joystick event runs on the
// loop thread, so it must hand its data to the HAL and wake the reader, not
// refresh the cache itself (DriverStationSim::NotifyNewData does): a refresh
// there could land between the reader's read of the control word and of the
// joysticks, a snapshot with one refresh's mode and another's sticks. So the
// cache shows the event only after a refresh, and the next refresh shows it
// whole.
// REQ-SAFE-13
TEST_F(SimDriverStationTest, ScriptedEventsLeaveTheRefreshToTheReader) {
  namespace ds = talos::driver_station;
  frc::DriverStation::RefreshData();
  ASSERT_FALSE(frc::DriverStation::IsEnabled());
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h;
  auto settings = Active();
  settings.enable_when_configured = true;
  settings.joysticks = ParseJoystickSchedule("0:axes=0.25;buttons=1;pov=-1");
  SimDriverStation sim{settings, t0};
  sim.ConfigurationAcknowledged();
  sim.Update(t0 + 1s, true);
  ASSERT_TRUE(Published().enabled);
  EXPECT_FALSE(frc::DriverStation::IsEnabled())
      << "the simulated Driver Station refreshed WPILib's cache itself";

  const auto packet = Sampled();
  EXPECT_NE(packet.fms.flags & ds::kEnabled, 0u);
  ASSERT_TRUE(packet.joysticks[0].connected);
  EXPECT_FLOAT_EQ(packet.joysticks[0].axes[0], 0.25f);
  EXPECT_EQ(packet.joysticks[0].buttons, 1u);
}

// A scripted joystick must reach the gateway's Driver Station packet as a
// real stick would: absent until a configuration push is acknowledged (no
// deflection before the robot can be enabled), each snapshot at its time from
// that moment, and still reported after a disable, which the packet's flags
// (not a vanished stick) carry to the driver.
TEST_F(SimDriverStationTest, JoystickScheduleReachesDriverStationPacket) {
  namespace ds = talos::driver_station;
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h;
  auto settings = Active();
  settings.schedule = ParseModeSchedule("teleop:2,disabled:1");
  settings.joysticks = ParseJoystickSchedule(
      "0:axes=0,0,0;buttons=0;pov=-1|"
      "1:axes=0.25,-0.5,0;buttons=5;pov=90");
  SimDriverStation sim{settings, t0};
  sim.Update(t0 + 10s, false);
  EXPECT_FALSE(Sampled().joysticks[0].connected);

  const auto configured = t0 + 11s;
  sim.ConfigurationAcknowledged();
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

// The console time prefix of a line printed at `now`, the loop's clock.
std::string ConsoleTimeAt(SimDriverStation::Clock::time_point now) {
  return talos::rio::ConsoleTime(static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          now.time_since_epoch())
          .count()));
}

// Every console line the stand-in prints (a schedule phase, a joystick
// event, enabling on a push, disabling at its deadline) starts with the
// console time prefix of the tick that printed it, like the program's own
// lines, so a run's log places each against them.
// REQ-OPS-24
TEST_F(SimDriverStationTest, EveryConsoleLineStartsWithTheConsoleTime) {
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h + 123ms;
  std::vector<std::string> expected;
  testing::internal::CaptureStdout();
  {
    auto settings = Active();
    settings.schedule = ParseModeSchedule("teleop:1,disabled:1");
    settings.joysticks = ParseJoystickSchedule(
        "0:axes=0;buttons=0;pov=-1|0.5:axes=0.5;buttons=1;pov=-1");
    SimDriverStation sim{settings, t0};
    sim.ConfigurationAcknowledged();
    const auto configured = t0 + 7ms;
    sim.Update(configured, true);          // joystick event 1, then teleop
    sim.Update(configured + 500ms, true);  // joystick event 2
    sim.Update(configured + 1s, true);     // disabled
    expected = {ConsoleTimeAt(configured), ConsoleTimeAt(configured),
                ConsoleTimeAt(configured + 500ms),
                ConsoleTimeAt(configured + 1s)};
  }
  {
    auto settings = Active();
    settings.enable_when_configured = true;
    settings.disable_after = std::chrono::milliseconds{1000};
    SimDriverStation sim{settings, t0};
    sim.ConfigurationAcknowledged();
    sim.Update(t0 + 101ms, true);   // enabling
    sim.Update(t0 + 1001ms, true);  // disabling
    expected.push_back(ConsoleTimeAt(t0 + 101ms));
    expected.push_back(ConsoleTimeAt(t0 + 1001ms));
  }
  std::istringstream output{testing::internal::GetCapturedStdout()};
  std::vector<std::string> lines;
  for (std::string line; std::getline(output, line);) {
    // Only the stand-in's: WPILib may print lines of its own.
    if (line.find("simulated Driver Station:") != std::string::npos) {
      lines.push_back(line);
    }
  }
  ASSERT_EQ(lines.size(), expected.size());
  for (std::size_t i = 0; i < lines.size(); ++i) {
    EXPECT_EQ(lines[i].rfind(expected[i], 0), 0u)
        << "line " << i << " lacks " << expected[i] << ": " << lines[i];
  }
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

// Malformed TALOS_SIM_DS (e.g. "false" or "") must throw std::invalid_argument
// rather than silently enabling the simulated robot, and malformed or negative
// TALOS_SIM_DS_DISABLE_AFTER_MS must be refused instead of parsed via atoi().
// REQ-DEV-06
TEST_F(SimDriverStationTest,
       SettingsFromEnvironmentRejectsMalformedModeAndDisableAfter) {
  struct ResetSimEnv {
    ~ResetSimEnv() {
      ::unsetenv("TALOS_SIM_DS");
      ::unsetenv("TALOS_SIM_DS_SCHEDULE");
      ::unsetenv("TALOS_SIM_DS_DISABLE_AFTER_MS");
      ::unsetenv("TALOS_SIM_DS_ALLIANCE");
      ::unsetenv("TALOS_SIM_JOYSTICK_SCHEDULE");
    }
  } reset_env;

  ::unsetenv("TALOS_SIM_DS_SCHEDULE");
  ::unsetenv("TALOS_SIM_DS_DISABLE_AFTER_MS");
  ::unsetenv("TALOS_SIM_DS_ALLIANCE");
  ::unsetenv("TALOS_SIM_JOYSTICK_SCHEDULE");

  ::setenv("TALOS_SIM_DS", "1", 1);
  const auto enabled = SimDriverStation::SettingsFromEnvironment();
  EXPECT_TRUE(enabled.active);
  EXPECT_TRUE(enabled.enabled);
  EXPECT_FALSE(enabled.enable_when_configured);

  ::setenv("TALOS_SIM_DS", "0", 1);
  const auto disabled = SimDriverStation::SettingsFromEnvironment();
  EXPECT_TRUE(disabled.active);
  EXPECT_FALSE(disabled.enabled);

  ::setenv("TALOS_SIM_DS", "auto", 1);
  ::setenv("TALOS_SIM_DS_DISABLE_AFTER_MS", "1500", 1);
  const auto automatic = SimDriverStation::SettingsFromEnvironment();
  EXPECT_TRUE(automatic.active);
  EXPECT_TRUE(automatic.enable_when_configured);
  ASSERT_TRUE(automatic.disable_after.has_value());
  EXPECT_EQ(*automatic.disable_after, std::chrono::milliseconds{1500});

  ::unsetenv("TALOS_SIM_DS_DISABLE_AFTER_MS");
  for (const char* bad_mode : {"", "false", "true", "2", "AUTO"}) {
    ::setenv("TALOS_SIM_DS", bad_mode, 1);
    EXPECT_THROW(SimDriverStation::SettingsFromEnvironment(),
                 std::invalid_argument)
        << "mode='" << bad_mode << "'";
  }

  ::setenv("TALOS_SIM_DS", "1", 1);
  for (const char* bad_after :
       {"", "-1", "+10", " 10", "abc", "10ms", "99999999999999999999"}) {
    ::setenv("TALOS_SIM_DS_DISABLE_AFTER_MS", bad_after, 1);
    EXPECT_THROW(SimDriverStation::SettingsFromEnvironment(),
                 std::invalid_argument)
        << "disable_after='" << bad_after << "'";
  }
}

}  // namespace

#endif  // __FRC_ROBORIO__
