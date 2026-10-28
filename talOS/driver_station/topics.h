#pragma once

// The driver station node's topic: the decoded Driver Station state,
// republished verbatim. Its input, /hw/state/driver_station, is the bridge's
// (talos::hardware::kDriverStationTopic).
namespace driver_station {
inline constexpr const char* kStateTopic = "/driver_station/state";
}  // namespace driver_station
