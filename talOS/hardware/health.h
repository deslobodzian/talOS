#pragma once

// Item 39: essential hardware health.
//
// State samples already carry per-device motion/current telemetry and validity
// flags, but three essentials were missing: the supply rail the whole robot
// shares, how many device resets have been observed since boot, and whether
// the gateway itself has latched a fault. This snapshot answers those without
// changing the State wire format: backends report what they can measure
// (unknowns stay invalid/zero), and Gateway::health() overlays the
// gateway-owned reset count and fault latch.

#include <cstdint>

namespace talos::hardware {

struct HealthSnapshot {
  double supply_voltage_v{0.0};
  bool supply_valid{false};  // False when the backend cannot measure the rail.
  uint64_t reset_count{0};   // Device resets observed since gateway boot.
  bool fault_latched{false};  // Gateway fault latch (apply/config failure).
  bool configured{false};
  bool enabled{false};
};

}  // namespace talos::hardware
