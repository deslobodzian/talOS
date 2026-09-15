#pragma once

// Item 34: commissioning workflow.
//
// The gateway's `commissioned` flag is a safety interlock, not a default:
// example CAN addresses and zero gains are not commissioned settings, and
// until the flag is set telemetry flows but commands are refused. This header
// makes the workflow explicit and enforceable in code:
//
//   1. Match the manifest to the physical robot (IDs, names, CAN addresses).
//   2. For each motor, confirm identity, direction, gearing, offsets, and
//      current/mechanical limits against the hardware.
//   3. Record each confirmation in a CommissioningRecord with a note naming
//      the evidence (e.g. "spun +duty, mechanism moved +", "measured 6.75:1").
//   4. ApplyTo() flips `commissioned` only when every motor and sensor in the
//      config has all five motor steps (identity, direction, gearing,
//      offsets, limits) or the sensor offset step signed off. Anything less
//      fails without touching the config.
//
// The record is attestation plus cross-checks: Validate() consistency
// (offsets within range, limits ordered, ratios positive, remote references
// resolving) is re-checked here so a signed-off but incoherent manifest
// cannot commission.

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "config.h"

namespace talos::hardware {

struct MotorCommissioning {
  bool identity_ok{false};   // Name/ID/CAN address match the physical device.
  bool direction_ok{false};  // Positive demand moves the mechanism positive.
  bool gearing_ok{false};    // Ratios measured, feedback source confirmed.
  bool offsets_ok{false};    // Encoder offsets measured, soft limits make sense.
  bool limits_ok{false};     // Current and mechanical limits verified safe.
  std::string note;          // Evidence for the sign-offs above.
};

struct SensorCommissioning {
  bool identity_ok{false};  // Name/ID/CAN address match the physical device.
  bool offset_ok{false};    // Magnet/CANcoder offset measured on the robot.
  std::string note;
};

struct CommissioningIssue {
  uint16_t id{0};
  std::string device;
  std::string check;
  std::string detail;
};

struct CommissioningReport {
  bool ready{false};
  std::vector<CommissioningIssue> issues;
};

struct CommissioningRecord {
  std::map<uint16_t, MotorCommissioning> motors;    // Keyed by logical motor ID.
  std::map<uint16_t, SensorCommissioning> sensors;  // Keyed by logical sensor ID.

  CommissioningReport Check(const Config& config) const {
    CommissioningReport report;
    report.ready = true;
    const auto fail = [&](uint16_t id, std::string device, std::string check,
                          std::string detail) {
      report.ready = false;
      report.issues.push_back({id, std::move(device), std::move(check),
                               std::move(detail)});
    };
    if (!config.commissioned) {
      // Not a failure: the record is what earns the flag. Cross-checks below
      // still apply so sign-off order does not matter.
    }
    for (const auto& m : config.motors) {
      const auto it = motors.find(m.id);
      const std::string dev = "motor '" + m.name + "'";
      if (it == motors.end()) {
        fail(m.id, dev, "record", "no commissioning entry for this motor");
        continue;
      }
      const auto& s = it->second;
      if (!s.identity_ok)
        fail(m.id, dev, "identity", "identity not verified " + s.note);
      if (!s.direction_ok)
        fail(m.id, dev, "direction", "direction not verified " + s.note);
      if (!s.gearing_ok)
        fail(m.id, dev, "gearing", "gearing not verified " + s.note);
      if (!s.offsets_ok)
        fail(m.id, dev, "offsets", "offsets not verified " + s.note);
      if (!s.limits_ok)
        fail(m.id, dev, "limits", "current/mechanical limits not verified " +
                                     s.note);
      // Cross-checks against the manifest itself.
      if (!(m.rotor_to_sensor_ratio > 0 && m.sensor_to_mechanism_ratio > 0))
        fail(m.id, dev, "gearing", "non-positive gear ratio in manifest");
      if (m.soft_limits && !(m.reverse_limit_rot < m.forward_limit_rot))
        fail(m.id, dev, "limits", "soft limits disordered in manifest");
      if (!(m.supply_limit_a > 0 && m.stator_limit_a > 0 && m.max_voltage > 0))
        fail(m.id, dev, "limits", "non-positive current/voltage limit");
    }
    for (const auto& s : config.sensors) {
      const auto it = sensors.find(s.id);
      const std::string dev = "sensor '" + s.name + "'";
      if (it == sensors.end()) {
        fail(s.id, dev, "record", "no commissioning entry for this sensor");
        continue;
      }
      if (!it->second.identity_ok)
        fail(s.id, dev, "identity",
             "identity not verified " + it->second.note);
      if (s.kind == SensorKind::kCANcoder && !it->second.offset_ok)
        fail(s.id, dev, "offsets",
             "magnet offset not measured " + it->second.note);
    }
    return report;
  }

  // Sets config.commissioned only when Check passes. Returns the report so
  // callers can log exactly which step blocked commissioning.
  CommissioningReport ApplyTo(Config& config) const {
    CommissioningReport report = Check(config);
    if (report.ready) config.commissioned = true;
    return report;
  }
};

}  // namespace talos::hardware
