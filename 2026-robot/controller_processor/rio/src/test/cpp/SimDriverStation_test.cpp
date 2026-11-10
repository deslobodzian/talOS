// The desktop-only simulated Driver Station that headless runs (sim.sh
// --headless, tools/test_drivetrain.py --wpilib) enable the robot with.
#include "SimDriverStation.h"

#ifndef __FRC_ROBORIO__

#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>
#include <hal/HAL.h>

#include <chrono>
#include <stdexcept>

namespace {

using talos::rio::ParseSimSchedule;
using talos::rio::SimDriverStation;
using namespace std::chrono_literals;

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
    frc::sim::DriverStationSim::NotifyNewData();
  }
};

// A schedule must not enable before the gateway is configured (the first
// configuration push needs disabled operation), then runs each phase from
// that moment, autonomous included, and holds the last one.
TEST_F(SimDriverStationTest, ScheduleStartsAtConfigurationAndHoldsLastPhase) {
  const auto t0 = SimDriverStation::Clock::time_point{} + 1h;
  SimDriverStation ds{{.active = true,
                       .schedule = ParseSimSchedule(
                           "teleop:1,auto:0.5,disabled:0.25,teleop:2")},
                      t0};
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
  SimDriverStation ds{{.active = true,
                       .enable_when_configured = true,
                       .disable_after = std::chrono::milliseconds{1000}},
                      t0};
  ds.Update(t0 + 100ms, true);
  EXPECT_TRUE(Published().enabled);
  ds.Update(t0 + 1s, true);
  EXPECT_FALSE(Published().enabled);
  ds.Update(t0 + 1005ms, true);
  EXPECT_FALSE(Published().enabled);
  ds.Update(t0 + 1010ms, true);
  EXPECT_FALSE(Published().enabled);
}

TEST(SimScheduleParse, RejectsMalformedSchedules) {
  for (const char* spec : {"", "teleop", "fly:1", "teleop:0", "teleop:-1",
                           "teleop:nan", "teleop:1x", "teleop:1,", ",auto:1"}) {
    EXPECT_THROW(ParseSimSchedule(spec), std::invalid_argument) << spec;
  }
}

}  // namespace

#endif  // __FRC_ROBORIO__
