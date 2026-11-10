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
//   TALOS_SIM_RUN_MS=10000              stop after ten seconds and print a
//                                       summary (read by Robot.cpp)
//
// Schedule modes are disabled, auto (or autonomous), teleop and test, with
// positive, finite durations: the syntax of sim_gateway's --mode-schedule.
// A schedule cannot be combined with TALOS_SIM_DS_DISABLE_AFTER_MS.

#ifndef __FRC_ROBORIO__

#include <chrono>
#include <optional>
#include <string_view>
#include <vector>

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

  Settings settings_;
  Clock::time_point started_;
  std::optional<Clock::time_point> configured_at_;
  SimMode published_;
  std::size_t phase_{0};
};

}  // namespace talos::rio

#endif  // __FRC_ROBORIO__
