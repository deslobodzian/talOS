#pragma once

#include "talOS/hardware/packet.h"

// Published topic constants and the message-type alias for the driver station
// node, following 2026-robot/main_processor/drivetrain/packet.h: consumers of
// `/driver_station/state` include this header rather than node.h, so reading a
// topic string never pulls in the node class (ARCHITECTURE.md Rule 5).
namespace talos::driver_station {
// The hardware envelope is a framework type; this alias keeps a single
// spelling at the call sites that publish or inject it.
using Packet = hardware::Packet;

// `/hw/state/driver_station`, and an alias rather than a second spelling of it:
// the bridge owns that name and this node is only the first reader, so the two
// ends cannot drift the way `/hw/req/drive` drifted from `/hw/req/drivetrain`.
inline constexpr const char* kHwDsTopic = hardware::kDriverStationTopic;
inline constexpr const char* kDsStateTopic = "/driver_station/state";
}  // namespace talos::driver_station
