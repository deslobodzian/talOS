#pragma once

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
// hardware::kRequestTopicPrefix followed by the `[subsystems.shooter]` key,
// which is how the bridge spells its end of the same topic.
inline constexpr const char* kShooterRequestTopic = "/hw/request/shooter";
}  // namespace talos::shooter
