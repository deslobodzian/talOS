#pragma once

// An automated stand-in for the Driver Station, for runs of this program
// headless on a desktop. It exists only in the desktop build: on the RoboRIO
// the enable state comes from the real Driver Station and nothing here can
// forge it. It is off unless TALOS_SIM_DS or TALOS_SIM_DS_SCHEDULE is set, so
// an interactive `./gradlew simulateNative` behaves like any other WPILib
// robot: it starts disabled and you enable it in the simulation GUI.
//
//   TALOS_SIM_DS=1                      enable in teleop at startup
//   TALOS_SIM_DS=0                      attach a Driver Station, stay disabled
//   TALOS_SIM_DS=auto                   stay disabled until the gateway is
//                                       configured, then enable in teleop
//   TALOS_SIM_DS_DISABLE_AFTER_MS=2000  ... then disable two seconds after
//                                       program start
//   TALOS_SIM_DS_SCHEDULE=teleop:60,auto:15,disabled:5
//                                       stay disabled until the gateway is
//                                       configured, then run these
//                                       mode:seconds phases and hold the last;
//                                       replaces TALOS_SIM_DS
//   TALOS_SIM_JOYSTICK_SCHEDULE='2:axes=0,-0.5;buttons=0;pov=-1'
//                                       script joystick 0 as seconds:fields
//                                       events (needs TALOS_SIM_DS or a
//                                       schedule), timed like the mode
//                                       schedule from configuration
//   TALOS_SIM_DS_ALLIANCE=red           the alliance (blue or red; default
//                                       blue, as sim_gateway's --alliance),
//                                       station 1
//   TALOS_SIM_RUN_MS=10000              stop after ten seconds and print a
//                                       summary (read by Robot.cpp)
//
// Schedule modes are disabled, auto (or autonomous), teleop and test, with
// positive, finite durations: the syntax of sim_gateway's --mode-schedule.
// A schedule cannot be combined with TALOS_SIM_DS_DISABLE_AFTER_MS.
//
// The joystick schedule is sim_gateway's --joystick-schedule: `|`-separated
// `seconds:axes=A0,A1,...;buttons=MASK;pov=DEG` events at nonnegative,
// strictly increasing times, each a complete joystick-0 snapshot (one to
// twelve axes in [-1,1], a decimal uint32 button mask with bit 0 as button 1,
// POV -1 or integer degrees [0,359]) held until the next. Joystick 0 is
// disconnected before the first event, including until the gateway is
// configured. Like a real Driver Station it keeps reporting the stick in
// every mode; disabling is what neutralizes the robot.

#ifndef __FRC_ROBORIO__

#include <chrono>
#include <optional>
#include <string_view>
#include <vector>

#include "driver_station/driver_station.h"

namespace talos::rio {

struct SimMode {
  bool enabled{false};
  bool autonomous{false};
  bool test{false};

  bool operator==(const SimMode&) const = default;
};

struct SimPhase {
  SimMode mode;
  // Cumulative: the phase ends this long after the schedule starts.
  std::chrono::microseconds end;
};

// Parses `mode:seconds,...`; throws std::invalid_argument naming the problem.
std::vector<SimPhase> ParseSimSchedule(std::string_view spec);

struct SimJoystickEvent {
  // From the schedule's start, like SimPhase::end.
  std::chrono::microseconds start;
  driver_station::JoystickData joystick;
};

// Parses `seconds:axes=...;buttons=MASK;pov=DEG|...`; throws
// std::invalid_argument naming the problem.
std::vector<SimJoystickEvent> ParseSimJoystickSchedule(std::string_view spec);

class SimDriverStation {
 public:
  using Clock = std::chrono::steady_clock;

  struct Settings {
    bool active{false};
    // TALOS_SIM_DS: 1 enables at once, auto once the gateway is configured.
    bool enabled{false};
    bool enable_when_configured{false};
    std::optional<std::chrono::milliseconds> disable_after;
    // When nonempty, replaces the three fields above.
    std::vector<SimPhase> schedule;
    // Joystick 0 over time, from the same start as the schedule (the
    // gateway's configuration); empty leaves it connected and centered.
    std::vector<SimJoystickEvent> joysticks;
    // Never kUnknown: the pose estimator trusts its seed only after a
    // disabled Driver Station view that names an alliance.
    driver_station::Alliance alliance{driver_station::Alliance::kBlue};
  };

  // Reads the TALOS_SIM_DS* variables; throws std::invalid_argument for a
  // malformed or contradictory setting.
  static Settings SettingsFromEnvironment();

  // Attaches the simulated Driver Station and publishes the initial mode when
  // the settings are active.
  SimDriverStation(Settings settings, Clock::time_point started);

  // Called every loop iteration with whether the gateway is configured.
  void Update(Clock::time_point now, bool configured);

 private:
  void Publish(SimMode mode);
  void UpdateJoystick(Clock::duration elapsed);

  Settings settings_;
  Clock::time_point started_;
  std::optional<Clock::time_point> configured_at_;
  SimMode published_;
  std::size_t phase_{0};
  std::size_t joystick_event_{0};
};

}  // namespace talos::rio

#endif  // __FRC_ROBORIO__
