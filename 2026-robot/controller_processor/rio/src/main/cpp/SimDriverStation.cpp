#include "SimDriverStation.h"

#ifndef __FRC_ROBORIO__

#include <frc/simulation/DriverStationSim.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>

namespace talos::rio {

namespace {

double Seconds(const std::string& text) {
  std::size_t used = 0;
  double value = 0;
  try {
    value = std::stod(text, &used);
  } catch (const std::exception&) {
    used = 0;
  }
  if (used == 0 || used != text.size() || !std::isfinite(value) || value <= 0) {
    throw std::invalid_argument(
        "mode schedule seconds must be finite and positive: '" + text + "'");
  }
  return value;
}

const char* ModeName(const SimMode& mode) {
  if (!mode.enabled) return "disabled";
  if (mode.autonomous) return "autonomous";
  if (mode.test) return "test";
  return "teleop";
}

}  // namespace

std::vector<SimPhase> ParseSimSchedule(std::string_view spec) {
  std::vector<SimPhase> phases;
  double end_s = 0;
  std::size_t start = 0;
  while (true) {
    const auto comma = spec.find(',', start);
    const std::string part{spec.substr(start, comma - start)};
    const auto colon = part.find(':');
    if (colon == std::string::npos) {
      throw std::invalid_argument(
          "mode schedule needs mode:seconds phases, got '" + part + "'");
    }
    const auto name = part.substr(0, colon);
    SimMode mode;
    if (name == "auto" || name == "autonomous") {
      mode = {.enabled = true, .autonomous = true};
    } else if (name == "teleop") {
      mode = {.enabled = true};
    } else if (name == "test") {
      mode = {.enabled = true, .test = true};
    } else if (name != "disabled") {
      throw std::invalid_argument("unknown mode in mode schedule: '" + name +
                                  "'");
    }
    end_s += Seconds(part.substr(colon + 1));
    if (!std::isfinite(end_s) || end_s > 1e9) {
      throw std::invalid_argument("mode schedule is too long");
    }
    phases.push_back(
        {mode, std::chrono::microseconds{std::llround(end_s * 1e6)}});
    if (comma == std::string_view::npos) break;
    start = comma + 1;
  }
  return phases;
}

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
    settings.schedule = ParseSimSchedule(schedule);
    return settings;
  }
  if (requested == nullptr) {
    return settings;  // Leave the Driver Station to the GUI or to a real one.
  }
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
  return settings;
}

SimDriverStation::SimDriverStation(Settings settings, Clock::time_point started)
    : settings_(std::move(settings)), started_(started) {
  if (!settings_.active) {
    return;
  }
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetJoystickAxisCount(0, 4);
  frc::sim::DriverStationSim::SetJoystickButtonCount(0, 10);
  frc::sim::DriverStationSim::SetJoystickPOVCount(0, 1);
  Publish({.enabled = settings_.schedule.empty() && settings_.enabled});
}

void SimDriverStation::Update(Clock::time_point now, bool configured) {
  if (!settings_.active) {
    return;
  }
  if (!settings_.schedule.empty()) {
    // The schedule's clock starts when the gateway is configured, since the
    // first configuration push requires disabled operation.
    if (!configured_at_) {
      if (!configured) return;
      configured_at_ = now;
    }
    const auto elapsed = now - *configured_at_;
    while (phase_ + 1 < settings_.schedule.size() &&
           elapsed >= settings_.schedule[phase_].end) {
      ++phase_;
    }
    const SimMode mode = settings_.schedule[phase_].mode;
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
    Publish({.enabled = true});
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

void SimDriverStation::Publish(SimMode mode) {
  published_ = mode;
  frc::sim::DriverStationSim::SetAutonomous(mode.autonomous);
  frc::sim::DriverStationSim::SetTest(mode.test);
  frc::sim::DriverStationSim::SetEnabled(mode.enabled);
  frc::sim::DriverStationSim::NotifyNewData();
}

}  // namespace talos::rio

#endif  // __FRC_ROBORIO__
