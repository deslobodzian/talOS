#pragma once

// Precise configuration failures. Backends that can name the device, bus,
// address, operation, and vendor status behind a failed Configure() return it
// here; Gateway latches the most recent failure and exposes it via
// Gateway::config_error() until the next successful configuration clears it.

#include <optional>
#include <string>

namespace talos::hardware {

struct ConfigError {
  std::string device_name;  // Config name, e.g. "front_left_drive".
  std::string bus;          // CAN bus, e.g. "rio".
  int can_id{-1};           // CAN address, or -1 when not a CAN device.
  std::string operation;    // What was attempted, e.g. "apply_talonfx_config".
  int vendor_status{0};  // Raw vendor status value (0 often means OK/unknown).
  std::string message;   // Human-readable detail, never empty on failure.
};

inline std::string FormatConfigError(const ConfigError& e) {
  std::string out = "config failed for '" + e.device_name + "'";
  if (!e.bus.empty()) out += " on bus '" + e.bus + "'";
  if (e.can_id >= 0) out += " CAN " + std::to_string(e.can_id);
  if (!e.operation.empty()) out += " during " + e.operation;
  out +=
      " (vendor status " + std::to_string(e.vendor_status) + "): " + e.message;
  return out;
}

}  // namespace talos::hardware
