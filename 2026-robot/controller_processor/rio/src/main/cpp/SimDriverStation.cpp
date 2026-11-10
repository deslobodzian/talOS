#include "SimDriverStation.h"

#ifndef __FRC_ROBORIO__

#include <frc/simulation/DriverStationSim.h>
#include <hal/DriverStationTypes.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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

// Splits on `delimiter`, refusing an empty field.
std::vector<std::string> Split(std::string_view text, char delimiter) {
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (true) {
    const auto end = text.find(delimiter, start);
    std::string part{text.substr(start, end - start)};
    if (part.empty()) {
      throw std::invalid_argument("empty joystick schedule field in '" +
                                  std::string{text} + "'");
    }
    parts.push_back(std::move(part));
    if (end == std::string_view::npos) return parts;
    start = end + 1;
  }
}

double JoystickNumber(const std::string& text) {
  std::size_t used = 0;
  double value = 0;
  try {
    value = std::stod(text, &used);
  } catch (const std::exception&) {
    used = 0;
  }
  if (used == 0 || used != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("joystick schedule numbers must be finite: '" +
                                text + "'");
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
      mode = {.enabled = true, .autonomous = true, .test = false};
    } else if (name == "teleop") {
      mode = {.enabled = true, .autonomous = false, .test = false};
    } else if (name == "test") {
      mode = {.enabled = true, .autonomous = false, .test = true};
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

std::vector<SimJoystickEvent> ParseSimJoystickSchedule(std::string_view spec) {
  namespace ds = talos::driver_station;
  std::vector<SimJoystickEvent> events;
  for (const auto& entry : Split(spec, '|')) {
    const auto colon = entry.find(':');
    if (colon == std::string::npos) {
      throw std::invalid_argument("joystick event needs seconds:fields, got '" +
                                  entry + "'");
    }
    const double start_s = JoystickNumber(entry.substr(0, colon));
    if (start_s < 0 || start_s > 1e9) {
      throw std::invalid_argument("joystick event time out of range: '" +
                                  entry + "'");
    }
    SimJoystickEvent event{
        std::chrono::microseconds{std::llround(start_s * 1e6)}, {}};
    if (!events.empty() && event.start <= events.back().start) {
      throw std::invalid_argument(
          "joystick event times must strictly increase: '" + entry + "'");
    }
    auto& stick = event.joystick;
    stick.connected = true;
    stick.button_count = ds::kMaxButtons;
    bool axes = false;
    bool buttons = false;
    bool pov = false;
    for (const auto& field :
         Split(std::string_view{entry}.substr(colon + 1), ';')) {
      const auto equal = field.find('=');
      if (equal == std::string::npos) {
        throw std::invalid_argument("joystick field needs key=value, got '" +
                                    field + "'");
      }
      const auto key = field.substr(0, equal);
      const auto value = field.substr(equal + 1);
      if (key == "axes" && !axes) {
        axes = true;
        for (const auto& axis : Split(value, ',')) {
          const double x = JoystickNumber(axis);
          if (stick.axis_count == stick.axes.size() || x < -1 || x > 1) {
            throw std::invalid_argument(
                "joystick axes need one to twelve values in [-1,1], got '" +
                value + "'");
          }
          stick.axes[stick.axis_count++] = static_cast<float>(x);
        }
      } else if (key == "buttons" && !buttons) {
        buttons = true;
        std::size_t used = 0;
        unsigned long long mask = 0;
        try {
          mask = std::stoull(value, &used);
        } catch (const std::exception&) {
          used = 0;
        }
        if (used == 0 || used != value.size() || value.front() == '-' ||
            value.front() == '+' || mask > UINT32_MAX) {
          throw std::invalid_argument(
              "joystick buttons must be a decimal uint32 mask, got '" + value +
              "'");
        }
        stick.buttons = static_cast<uint32_t>(mask);
      } else if (key == "pov" && !pov) {
        pov = true;
        const double angle = JoystickNumber(value);
        if (angle != std::floor(angle) ||
            (angle != -1 && (angle < 0 || angle >= 360))) {
          throw std::invalid_argument(
              "joystick pov must be -1 or integer degrees [0,359], got '" +
              value + "'");
        }
        stick.pov_count = 1;
        stick.povs[0] = static_cast<int16_t>(angle);
      } else {
        throw std::invalid_argument("unknown or repeated joystick field '" +
                                    key + "'");
      }
    }
    if (!axes || !buttons || !pov) {
      throw std::invalid_argument(
          "joystick event needs axes, buttons and pov, got '" + entry + "'");
    }
    events.push_back(event);
  }
  return events;
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
    settings.joysticks = ParseSimJoystickSchedule(joysticks);
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
