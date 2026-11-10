#include "SimDriverStation.h"

#ifndef __FRC_ROBORIO__

#include <frc/simulation/DriverStationSim.h>
#include <hal/DriverStationTypes.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace talos::rio {

namespace {

SimMode ModeOf(uint32_t flags) {
  namespace ds = driver_station;
  return {.enabled = (flags & ds::kEnabled) != 0,
          .autonomous = (flags & ds::kAutonomous) != 0,
          .test = (flags & ds::kTest) != 0};
}

const char* ModeName(const SimMode& mode) {
  if (!mode.enabled) return "disabled";
  if (mode.autonomous) return "autonomous";
  if (mode.test) return "test";
  return "teleop";
}

}  // namespace

SimDriverStation::Settings SimDriverStation::SettingsFromEnvironment() {
  Settings settings;
  const char* requested = std::getenv("TALOS_SIM_DS");
  const char* schedule = std::getenv("TALOS_SIM_DS_SCHEDULE");
  const char* after = std::getenv("TALOS_SIM_DS_DISABLE_AFTER_MS");
  if (schedule != nullptr) {
    if (after != nullptr) {
      throw std::invalid_argument(
          "TALOS_SIM_DS_SCHEDULE cannot be combined with "
          "TALOS_SIM_DS_DISABLE_AFTER_MS; end the schedule with a disabled "
          "phase instead");
    }
    settings.active = true;
    settings.schedule = driver_station::ParseModeSchedule(schedule);
  } else if (requested != nullptr) {
    settings.active = true;
    const std::string_view mode{requested};
    if (mode == "auto") {
      // Without a persisted configuration the first push still requires
      // disabled operation. A restored configuration can start enabled.
      settings.enable_when_configured = true;
    } else {
      settings.enabled = mode != "0";
    }
    if (after != nullptr) {
      settings.disable_after = std::chrono::milliseconds{std::atoi(after)};
    }
  }
  // Otherwise leave the Driver Station to the GUI or to a real one.
  if (const char* alliance = std::getenv("TALOS_SIM_DS_ALLIANCE")) {
    const std::string_view name{alliance};
    if (name == "red") {
      settings.alliance = driver_station::Alliance::kRed;
    } else if (name != "blue") {
      throw std::invalid_argument(
          "TALOS_SIM_DS_ALLIANCE must be blue or red, got '" +
          std::string{name} + "'");
    }
  }
  if (const char* joysticks = std::getenv("TALOS_SIM_JOYSTICK_SCHEDULE")) {
    if (!settings.active) {
      throw std::invalid_argument(
          "TALOS_SIM_JOYSTICK_SCHEDULE needs TALOS_SIM_DS or "
          "TALOS_SIM_DS_SCHEDULE; without them the GUI owns the joysticks");
    }
    settings.joysticks = driver_station::ParseJoystickSchedule(joysticks);
  }
  return settings;
}

SimDriverStation::SimDriverStation(Settings settings, Clock::time_point started)
    : settings_(std::move(settings)), started_(started) {
  if (!settings_.active) {
    return;
  }
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetAllianceStationId(
      settings_.alliance == driver_station::Alliance::kRed
          ? HAL_AllianceStationID_kRed1
          : HAL_AllianceStationID_kBlue1);
  // A scripted joystick is disconnected until its first event.
  const bool centered = settings_.joysticks.empty();
  frc::sim::DriverStationSim::SetJoystickAxisCount(0, centered ? 4 : 0);
  frc::sim::DriverStationSim::SetJoystickButtonCount(0, centered ? 10 : 0);
  frc::sim::DriverStationSim::SetJoystickPOVCount(0, centered ? 1 : 0);
  Publish({.enabled = settings_.schedule.empty() && settings_.enabled,
           .autonomous = false,
           .test = false});
}

void SimDriverStation::Update(Clock::time_point now, bool configured) {
  if (!settings_.active) {
    return;
  }
  // The schedules' clock starts when the gateway is configured, since the
  // first configuration push requires disabled operation.
  if (!configured_at_ && configured) {
    configured_at_ = now;
  }
  if (configured_at_) {
    UpdateJoystick(now - *configured_at_);
  }
  if (!settings_.schedule.empty()) {
    if (!configured_at_) return;
    const auto elapsed = now - *configured_at_;
    while (phase_ + 1 < settings_.schedule.size() &&
           elapsed >= settings_.schedule[phase_].end) {
      ++phase_;
    }
    const SimMode mode = ModeOf(settings_.schedule[phase_].flags);
    if (mode != published_) {
      std::printf("simulated Driver Station: %s (schedule phase %zu)\n",
                  ModeName(mode), phase_ + 1);
      std::fflush(stdout);
      Publish(mode);
    }
    return;
  }
  if (settings_.enable_when_configured && !published_.enabled && configured) {
    std::printf("simulated Driver Station: enabling (gateway configured)\n");
    settings_.enable_when_configured = false;
    Publish({.enabled = true, .autonomous = false, .test = false});
  }
  if (!published_.enabled || !settings_.disable_after) {
    return;
  }
  if (now - started_ < *settings_.disable_after) {
    return;
  }
  std::printf("simulated Driver Station: disabling\n");
  Publish({});
}

void SimDriverStation::UpdateJoystick(Clock::duration elapsed) {
  const auto& events = settings_.joysticks;
  const std::size_t before = joystick_event_;
  while (joystick_event_ < events.size() &&
         elapsed >= events[joystick_event_].start) {
    ++joystick_event_;
  }
  if (joystick_event_ == before) return;
  // Only the latest due event matters: each is a complete snapshot.
  const auto& stick = events[joystick_event_ - 1].joystick;
  using frc::sim::DriverStationSim;
  DriverStationSim::SetJoystickAxisCount(0, stick.axis_count);
  for (int axis = 0; axis < stick.axis_count; ++axis) {
    DriverStationSim::SetJoystickAxis(0, axis, stick.axes[axis]);
  }
  DriverStationSim::SetJoystickButtonCount(0, stick.button_count);
  DriverStationSim::SetJoystickButtons(0, stick.buttons);
  DriverStationSim::SetJoystickPOVCount(0, stick.pov_count);
  for (int pov = 0; pov < stick.pov_count; ++pov) {
    DriverStationSim::SetJoystickPOV(0, pov, stick.povs[pov]);
  }
  DriverStationSim::NotifyNewData();
  std::printf("simulated Driver Station: joystick event %zu\n",
              joystick_event_);
  std::fflush(stdout);
}

void SimDriverStation::Publish(SimMode mode) {
  published_ = mode;
  frc::sim::DriverStationSim::SetAutonomous(mode.autonomous);
  frc::sim::DriverStationSim::SetTest(mode.test);
  frc::sim::DriverStationSim::SetEnabled(mode.enabled);
  frc::sim::DriverStationSim::NotifyNewData();
}

}  // namespace talos::rio

#endif  // __FRC_ROBORIO__
