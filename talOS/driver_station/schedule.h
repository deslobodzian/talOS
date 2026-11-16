#pragma once

// The scripted Driver Station schedules both simulators accept: sim_gateway's
// --mode-schedule and --joystick-schedule, and the RoboRIO program's
// TALOS_SIM_DS_SCHEDULE and TALOS_SIM_JOYSTICK_SCHEDULE (desktop build). Header
// only and free of other talOS dependencies, so GradleRIO compiles it as is.
//
// Mode schedule: `mode:seconds,...`, modes disabled, auto (or autonomous),
// teleop and test, each with a positive, finite duration; the phases total at
// most 1e9 s, and the last one is held.
//
// Joystick schedule: `|`-separated
// `seconds:axes=A0,A1,...;buttons=MASK;pov=DEG` events at nonnegative times of
// at most 1e9 s that strictly increase (at microsecond resolution), each a
// complete joystick-0 snapshot (one to twelve axes in [-1,1], a decimal uint32
// button mask with bit 0 as button 1, POV -1 or integer degrees [0,359]) held
// until the next.
//
// Both parsers throw std::invalid_argument naming the problem.

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "driver_station.h"

namespace talos::driver_station {

struct SchedulePhase {
  // kDsAttached plus, unless disabled, kEnabled and the mode's flag.
  uint32_t flags;
  // Cumulative: the phase ends this long after the schedule starts.
  std::chrono::microseconds end;
};

struct JoystickEvent {
  // From the schedule's start, like SchedulePhase::end.
  std::chrono::microseconds start;
  JoystickData joystick;
};

namespace schedule_detail {

inline constexpr double kMaxSeconds = 1e9;

// Splits on `delimiter`, refusing an empty field.
inline std::vector<std::string> Split(std::string_view text, char delimiter) {
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (true) {
    const auto end = text.find(delimiter, start);
    std::string part{text.substr(start, end - start)};
    if (part.empty()) {
      throw std::invalid_argument("empty schedule field in '" +
                                  std::string{text} + "'");
    }
    parts.push_back(std::move(part));
    if (end == std::string_view::npos) return parts;
    start = end + 1;
  }
}

// The whole of `text` as a finite number; std::stod's own exceptions are
// reported as std::invalid_argument too.
inline double Finite(const std::string& text, const char* what) {
  std::size_t used = 0;
  double value = 0;
  try {
    value = std::stod(text, &used);
  } catch (const std::exception&) {
    used = 0;
  }
  if (used == 0 || used != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(std::string{what} + " must be finite: '" +
                                text + "'");
  }
  return value;
}

inline std::chrono::microseconds Micros(double seconds) {
  return std::chrono::microseconds{std::llround(seconds * 1e6)};
}

}  // namespace schedule_detail

inline std::vector<SchedulePhase> ParseModeSchedule(std::string_view spec) {
  namespace detail = schedule_detail;
  std::vector<SchedulePhase> phases;
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
    uint32_t flags = kDsAttached;
    if (name == "auto" || name == "autonomous") {
      flags |= kEnabled | kAutonomous;
    } else if (name == "teleop") {
      flags |= kEnabled | kTeleop;
    } else if (name == "test") {
      flags |= kEnabled | kTest;
    } else if (name != "disabled") {
      throw std::invalid_argument("unknown mode in mode schedule: '" + name +
                                  "'");
    }
    const double seconds =
        detail::Finite(part.substr(colon + 1), "mode schedule seconds");
    if (seconds <= 0) {
      throw std::invalid_argument("mode schedule seconds must be positive: '" +
                                  part + "'");
    }
    end_s += seconds;
    if (end_s > detail::kMaxSeconds) {
      throw std::invalid_argument("mode schedule is too long");
    }
    phases.push_back({flags, detail::Micros(end_s)});
    if (comma == std::string_view::npos) return phases;
    start = comma + 1;
  }
}

inline std::vector<JoystickEvent> ParseJoystickSchedule(std::string_view spec) {
  namespace detail = schedule_detail;
  std::vector<JoystickEvent> events;
  for (const auto& entry : detail::Split(spec, '|')) {
    const auto colon = entry.find(':');
    if (colon == std::string::npos) {
      throw std::invalid_argument("joystick event needs seconds:fields, got '" +
                                  entry + "'");
    }
    const double start_s =
        detail::Finite(entry.substr(0, colon), "joystick event time");
    if (start_s < 0 || start_s > detail::kMaxSeconds) {
      throw std::invalid_argument("joystick event time out of range: '" +
                                  entry + "'");
    }
    JoystickEvent event{detail::Micros(start_s), {}};
    if (!events.empty() && event.start <= events.back().start) {
      throw std::invalid_argument(
          "joystick event times must strictly increase: '" + entry + "'");
    }
    auto& stick = event.joystick;
    stick.connected = true;
    stick.button_count = kMaxButtons;
    bool axes = false;
    bool buttons = false;
    bool pov = false;
    for (const auto& field :
         detail::Split(std::string_view{entry}.substr(colon + 1), ';')) {
      const auto equal = field.find('=');
      if (equal == std::string::npos) {
        throw std::invalid_argument("joystick field needs key=value, got '" +
                                    field + "'");
      }
      const auto key = field.substr(0, equal);
      const auto value = field.substr(equal + 1);
      if (key == "axes" && !axes) {
        axes = true;
        for (const auto& axis : detail::Split(value, ',')) {
          const double x = detail::Finite(axis, "joystick axes");
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
        // Digits only: std::stoull would accept a sign or leading space.
        if (!value.empty() && value.front() >= '0' && value.front() <= '9') {
          try {
            mask = std::stoull(value, &used);
          } catch (const std::exception&) {
            used = 0;
          }
        }
        if (used == 0 || used != value.size() || mask > UINT32_MAX) {
          throw std::invalid_argument(
              "joystick buttons must be a decimal uint32 mask, got '" + value +
              "'");
        }
        stick.buttons = static_cast<uint32_t>(mask);
      } else if (key == "pov" && !pov) {
        pov = true;
        const double angle = detail::Finite(value, "joystick pov");
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

}  // namespace talos::driver_station
