#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace talos::testing {
class Sim;
}  // namespace talos::testing

namespace talos::telemetry {

// Adds the telemetry node to `sim` without exposing the internal
// talOS/telemetry/node.h (and talOS/node/access.h) header to robot/bench tests.
void AddTelemetry(talos::testing::Sim& sim,
                  const std::string& name = "telemetry");

// Adds the telemetry node wrapped to record each update() callback's
// entry-to-exit monotonic wall duration (nanoseconds) into `samples_ns` while
// `recording` is true (REQ-RT-07; docs/METHODOLOGY.md §10). Both references
// must outlive `sim`.
void AddTimedTelemetry(talos::testing::Sim& sim,
                       std::vector<std::int64_t>& samples_ns,
                       const bool& recording,
                       const std::string& name = "telemetry");

}  // namespace talos::telemetry
