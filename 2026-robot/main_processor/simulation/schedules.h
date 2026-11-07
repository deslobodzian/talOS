#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/sim_physics.h"

namespace robot::simulation {

inline std::vector<std::string> Split(const std::string& text, char delimiter) {
  std::vector<std::string> parts;
  std::size_t start = 0;
  do {
    const auto end = text.find(delimiter, start);
    auto part = text.substr(start, end - start);
    if (part.empty()) throw std::invalid_argument("empty schedule field");
    parts.push_back(std::move(part));
    if (end == std::string::npos) break;
    start = end + 1;
  } while (true);
  return parts;
}

inline double Number(const std::string& text) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("schedule numbers must be finite");
  }
  return value;
}

struct JoystickEvent {
  double start_s;
  talos::driver_station::JoystickData joystick;
};

// Every event is a complete joystick-0 snapshot, held until the next event.
inline std::vector<JoystickEvent> ParseJoysticks(const std::string& spec) {
  std::vector<JoystickEvent> events;
  for (const auto& entry : Split(spec, '|')) {
    const auto colon = entry.find(':');
    if (colon == std::string::npos) {
      throw std::invalid_argument("joystick event needs time:fields");
    }
    JoystickEvent event{Number(entry.substr(0, colon)), {}};
    if (event.start_s < 0 ||
        (!events.empty() && event.start_s <= events.back().start_s)) {
      throw std::invalid_argument("joystick times must increase from zero");
    }
    auto& stick = event.joystick;
    stick.connected = true;
    stick.button_count = 32;
    bool axes = false, buttons = false, pov = false;
    for (const auto& field : Split(entry.substr(colon + 1), ';')) {
      const auto equal = field.find('=');
      const auto key = field.substr(0, equal);
      if (equal == std::string::npos) {
        throw std::invalid_argument("joystick field needs key=value");
      }
      const auto value = field.substr(equal + 1);
      if (key == "axes" && !axes) {
        axes = true;
        for (const auto& axis : Split(value, ',')) {
          const double x = Number(axis);
          if (stick.axis_count == stick.axes.size() || x < -1 || x > 1) {
            throw std::invalid_argument("axes need at most 12 values in [-1,1]");
          }
          stick.axes[stick.axis_count++] = static_cast<float>(x);
        }
      } else if (key == "buttons" && !buttons) {
        buttons = true;
        std::size_t used = 0;
        const auto mask = std::stoull(value, &used);
        if (used != value.size() || value.starts_with('-') || mask > UINT32_MAX) {
          throw std::invalid_argument("buttons must be a uint32 decimal mask");
        }
        stick.buttons = static_cast<uint32_t>(mask);
      } else if (key == "pov" && !pov) {
        pov = true;
        const double angle = Number(value);
        if (angle != std::floor(angle) ||
            (angle != -1 && (angle < 0 || angle >= 360))) {
          throw std::invalid_argument("pov must be -1 or integer degrees [0,359]");
        }
        stick.pov_count = 1;
        stick.povs[0] = static_cast<int16_t>(angle);
      } else {
        throw std::invalid_argument("unknown or repeated joystick field: " + key);
      }
    }
    if (!axes || !buttons || !pov) {
      throw std::invalid_argument("joystick event needs axes, buttons and pov");
    }
    events.push_back(event);
  }
  return events;
}

enum class FaultKind {
  kMotorDisconnect,
  kSensorDisconnect,
  kFreeze,
  kSupply,
  kReset,
  kBlackout
};

struct FaultEvent {
  double start_s;
  double end_s;
  FaultKind kind;
  std::string target;
  double volts{12};
  uint16_t id{};
  bool active{false};
  bool finished{false};
};

inline std::vector<FaultEvent> ParseFaults(const std::string& spec) {
  std::vector<FaultEvent> events;
  for (const auto& entry : Split(spec, '|')) {
    const auto fields = Split(entry, ':');
    if (fields.size() != 4 && fields.size() != 5) {
      throw std::invalid_argument("fault needs start:end:kind:target[:volts]");
    }
    FaultEvent event{Number(fields[0]), Number(fields[1]), FaultKind::kReset,
                     fields[3]};
    if (event.start_s < 0 || event.end_s <= event.start_s) {
      throw std::invalid_argument("fault needs 0 <= start < end");
    }
    const auto& kind = fields[2];
    if (kind == "motor_disconnect") {
      event.kind = FaultKind::kMotorDisconnect;
    } else if (kind == "sensor_disconnect") {
      event.kind = FaultKind::kSensorDisconnect;
    } else if (kind == "freeze") {
      event.kind = FaultKind::kFreeze;
    } else if (kind == "supply_voltage") {
      event.kind = FaultKind::kSupply;
    } else if (kind == "reset") {
      event.kind = FaultKind::kReset;
    } else if (kind == "udp_blackout") {
      event.kind = FaultKind::kBlackout;
    } else {
      throw std::invalid_argument("unknown fault: " + kind);
    }
    if (event.kind == FaultKind::kSupply) {
      if (fields.size() != 5 || event.target != "all") {
        throw std::invalid_argument("supply_voltage needs target all and volts");
      }
      event.volts = Number(fields[4]);
      if (event.volts < 0 || event.volts > 16) {
        throw std::invalid_argument("supply voltage must be in [0,16]");
      }
    } else if (fields.size() != 4) {
      throw std::invalid_argument("only supply_voltage accepts volts");
    }
    if (event.kind == FaultKind::kBlackout && event.target != "bridge") {
      throw std::invalid_argument("udp_blackout target must be bridge");
    }
    for (const auto& previous : events) {
      if (event.kind == previous.kind && event.target == previous.target &&
          event.start_s < previous.end_s && previous.start_s < event.end_s) {
        throw std::invalid_argument("overlapping faults for the same target");
      }
    }
    events.push_back(event);
  }
  // Apply an ending interval's restoration before an adjacent interval's
  // activation, regardless of the order supplied on the command line.
  std::sort(events.begin(), events.end(), [](const auto& a, const auto& b) {
    return a.start_s < b.start_s;
  });
  return events;
}

inline void ResolveFaults(std::vector<FaultEvent>& events,
                          const talos::hardware::Config& config) {
  for (auto& event : events) {
    if (event.kind == FaultKind::kSupply || event.kind == FaultKind::kBlackout) {
      continue;
    }
    bool found = false;
    if (event.kind != FaultKind::kSensorDisconnect) {
      for (const auto& motor : config.motors) {
        if (motor.name == event.target) {
          event.id = motor.id;
          found = true;
        }
      }
    }
    if (event.kind == FaultKind::kSensorDisconnect ||
        event.kind == FaultKind::kReset) {
      for (const auto& sensor : config.sensors) {
        if (sensor.name == event.target) {
          event.id = sensor.id;
          found = true;
        }
      }
    }
    if (!found) {
      throw std::invalid_argument("unknown fault device: " + event.target);
    }
  }
}

inline bool ApplyFaults(std::vector<FaultEvent>& events, double elapsed_s,
                        talos::hardware::sim::PhysicsSimBackend& backend) {
  bool blackout = false;
  for (auto& event : events) {
    if (event.finished || elapsed_s < event.start_s) continue;
    const bool active = elapsed_s < event.end_s;
    // Reset is an edge, not a reset every tick. Other hooks are reapplied so
    // a device restore cannot silently erase a still-scheduled fault.
    if (event.kind == FaultKind::kReset) {
      if (!event.active) backend.SimulateDeviceReset(event.id);
    } else if (event.kind == FaultKind::kMotorDisconnect) {
      backend.SetMotorConnected(event.id, !active);
    } else if (event.kind == FaultKind::kSensorDisconnect) {
      backend.SetSensorConnected(event.id, !active);
    } else if (event.kind == FaultKind::kFreeze) {
      if (active) {
        backend.FreezeFeedback(event.id);
      } else {
        backend.UnfreezeFeedback(event.id);
      }
    } else if (event.kind == FaultKind::kSupply) {
      backend.SetSupplyVoltage(active ? event.volts : 12.0);
    } else {
      blackout |= active;
    }
    event.active = active;
    event.finished = !active;
  }
  return blackout;
}

}  // namespace robot::simulation
