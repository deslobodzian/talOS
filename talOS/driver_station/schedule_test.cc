#include "talOS/driver_station/schedule.h"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <string>

namespace talos::driver_station {
namespace {

using std::chrono::microseconds;

// Phases end cumulatively and carry the flags the gateway's packet sends.
TEST(Schedule, ModePhasesAccumulateWithTheirFlags) {
  const auto phases = ParseModeSchedule("teleop:1,auto:0.5,disabled:0.25");
  ASSERT_EQ(phases.size(), 3u);
  EXPECT_EQ(phases[0].flags, kDsAttached | kEnabled | kTeleop);
  EXPECT_EQ(phases[1].flags, kDsAttached | kEnabled | kAutonomous);
  EXPECT_EQ(phases[1].end, microseconds{1'500'000});
  EXPECT_EQ(phases[2].flags, kDsAttached);
  EXPECT_EQ(phases[2].end, microseconds{1'750'000});
}

TEST(Schedule, RejectsMalformedModeSchedules) {
  for (const char* spec : {"", "teleop", "fly:1", "teleop:0", "teleop:-1",
                           "teleop:nan", "teleop:1x", "teleop:1e999",
                           "teleop:1,", ",auto:1", "teleop:6e8,auto:6e8"}) {
    EXPECT_THROW(ParseModeSchedule(spec), std::invalid_argument) << spec;
  }
}

TEST(Schedule, JoystickSnapshotsCarryAxesButtonsAndPov) {
  const auto events = ParseJoystickSchedule(
      "0:axes=0,0,0;buttons=0;pov=-1|1.5:axes=0,-0.5,1;buttons=5;pov=90");
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[1].start, microseconds{1'500'000});
  const auto& stick = events[1].joystick;
  EXPECT_TRUE(stick.connected);
  EXPECT_EQ(stick.button_count, kMaxButtons);
  EXPECT_EQ(stick.axis_count, 3);
  EXPECT_FLOAT_EQ(stick.axes[1], -0.5f);
  EXPECT_FLOAT_EQ(stick.axes[2], 1.0f);
  EXPECT_EQ(stick.buttons, 5u);
  ASSERT_EQ(stick.pov_count, 1);
  EXPECT_EQ(stick.povs[0], 90);
  EXPECT_EQ(events[0].joystick.povs[0], -1);
}

// A malformed schedule must fail the run, never be partly applied.
TEST(Schedule, RejectsMalformedJoystickSchedules) {
  constexpr const char* kStick = "axes=0;buttons=0;pov=-1";
  for (const std::string& spec :
       {std::string{}, std::string{"0"}, std::string{"x:"} + kStick,
        std::string{"-1:"} + kStick, std::string{"nan:"} + kStick,
        std::string{"2e9:"} + kStick,
        std::string{"1:"} + kStick + "|1:" + kStick,
        // Distinct as text, equal at microsecond resolution.
        std::string{"0.0000001:"} + kStick + "|0.0000002:" + kStick,
        std::string{"0:"} + kStick + "|", std::string{"0:axes=0;buttons=0"},
        std::string{"0:axes=1.5;buttons=0;pov=-1"},
        std::string{"0:axes=0,0,0,0,0,0,0,0,0,0,0,0,0;buttons=0;pov=-1"},
        std::string{"0:axes=0;buttons=-1;pov=-1"},
        std::string{"0:axes=0;buttons=+1;pov=-1"},
        std::string{"0:axes=0;buttons= 1;pov=-1"},
        std::string{"0:axes=0;buttons=4294967296;pov=-1"},
        std::string{"0:axes=0;buttons=0;pov=360"},
        std::string{"0:axes=0;buttons=0;pov=45.5"},
        std::string{"0:axes=0;axes=0;buttons=0;pov=-1"},
        std::string{"0:axes=0;buttons=0;pov=-1;rumble=1"}}) {
    EXPECT_THROW(ParseJoystickSchedule(spec), std::invalid_argument) << spec;
  }
}

}  // namespace
}  // namespace talos::driver_station
