#pragma once

// Advisory hardware-configuration checks run by the launcher beside the graph
// lint (talOS/launcher/README.md). Split out of names.h so
// //talOS/introspection :names has no dependency on //talOS/hardware.

#include <cstddef>
#include <cstdio>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/hardware/config.h"
#include "talOS/introspection/names.h"

namespace talos::introspect::naming {

// A CAN bus's estimated utilization (0 to 1) above which
// CheckHardwareConfig warns.
inline constexpr double kCanBusWarnFrac = 0.70;

// A CAN bus's estimated utilization from its devices (CheckHardwareConfig).
// Per device at a 100 Hz status rate, from CTR Electronics' Phoenix 6 "CAN
// Bus Utilization" page
// (https://v6.docs.ctr-electronics.com/en/stable/docs/migration/canbus-utilization.html),
// which also advises keeping a bus below 90 %: on CAN FD (a CANivore: any
// bus but "rio") a Talon FX 0.018, a CANcoder 0.009, a Pigeon 2 0.013; on
// classic CAN ("rio") 0.041, 0.017 and 0.031. Each device's figure scales
// with its status rate over 100 Hz, and every CAN device sends at the
// config's `status_hz` (no device sets its own rate). Each bus adds 0.05
// for the vendor's diagnostic server. Calibration: the bench's three Talon
// FX, CANcoder and Pigeon 2 on a CANivore at 200 Hz estimate 0.152 plus
// that 0.05, against 0.16 to 0.18 measured on /hw/status.
struct CanBusLoad {
  std::string bus;
  std::size_t motors{0}, cancoders{0}, gyros{0};
  double estimate_frac{0};
};

inline std::vector<CanBusLoad> EstimateCanBusLoads(
    const hardware::Config& config) {
  std::map<std::string, CanBusLoad, std::less<>> buses;
  const auto at = [&](std::string_view bus) -> CanBusLoad& {
    auto it = buses.find(bus);
    if (it == buses.end()) {
      it = buses.emplace(std::string{bus}, CanBusLoad{}).first;
      it->second.bus = it->first;
    }
    return it->second;
  };
  for (const auto& motor : config.motors) ++at(motor.bus).motors;
  for (const auto& sensor : config.sensors) {
    CanBusLoad& load = at(sensor.bus);
    ++(sensor.kind == hardware::SensorKind::kPigeon2 ? load.gyros
                                                     : load.cancoders);
  }
  std::vector<CanBusLoad> out;
  out.reserve(buses.size());
  for (auto& [name, load] : buses) {
    const bool classic = name == "rio";
    const double per_100_hz =
        static_cast<double>(load.motors) * (classic ? 0.041 : 0.018) +
        static_cast<double>(load.cancoders) * (classic ? 0.017 : 0.009) +
        static_cast<double>(load.gyros) * (classic ? 0.031 : 0.013);
    load.estimate_frac = per_100_hz * config.status_hz / 100.0 + 0.05;
    out.push_back(std::move(load));
  }
  return out;
}

// Advisory hardware limits, not configuration errors.
inline std::vector<Diagnostic> CheckHardwareConfig(
    const hardware::Config& config) {
  std::vector<Diagnostic> out;
  for (const auto& motor : config.motors) {
    const double sensor_velocity =
        motor.max_velocity_rps * motor.sensor_to_mechanism_ratio;
    if (sensor_velocity > 105.0) {
      out.push_back(
          {Severity::WARNING, motor.name.str(),
           "max_velocity_rps * sensor_to_mechanism_ratio is " +
               std::to_string(sensor_velocity) +
               " rps, exceeding the recommended 105 rps; check gearing and "
               "the configured maximum velocity"});
    }
  }
  for (const auto& load : EstimateCanBusLoads(config)) {
    if (load.estimate_frac <= kCanBusWarnFrac) continue;
    char line[256];
    std::snprintf(line, sizeof line,
                  "estimated CAN bus utilization %.2f (%zu motors, %zu "
                  "CANcoders, %zu gyros at status_hz %g, on %s) is above "
                  "%.2f; reduce status_hz or move devices to another CAN bus",
                  load.estimate_frac, load.motors, load.cancoders, load.gyros,
                  config.status_hz,
                  load.bus == "rio" ? "classic CAN" : "CAN FD",
                  kCanBusWarnFrac);
    out.push_back({Severity::WARNING, load.bus, line});
  }
  return out;
}

}  // namespace talos::introspect::naming
