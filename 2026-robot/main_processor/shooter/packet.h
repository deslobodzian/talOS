#pragma once

#include <string>

#include "talOS/hardware/packet.h"

namespace talos::shooter {
using Packet = hardware::Packet;

inline constexpr const char* kHwStateTopic = hardware::kStateTopic;
inline constexpr const char* kShooterStateTopic = "/shooter/state";
// Written only by the arbiter; producers use the per-source topics below.
inline constexpr const char* kShooterTargetTopic = "/shooter/target";
inline constexpr const char* kTeleopShooterTargetTopic =
    "/shooter/target/teleop";
inline constexpr const char* kAutoShooterTargetTopic = "/shooter/target/auto";
// Spelled once, behind hardware::RequestTopic: the bridge derives the same
// string from the `[subsystems.shooter]` key in the config, so calling the
// helper is what makes the two ends provably the same topic.
inline const std::string kShooterRequestTopic =
    hardware::RequestTopic("shooter");
}  // namespace talos::shooter
