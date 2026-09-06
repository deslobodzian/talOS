#pragma once

#include "talOS/hardware/packet.h"

namespace talos::shooter {
using Packet = hardware::Packet;

inline constexpr const char* kHwStateTopic = hardware::kStateTopic;
inline constexpr const char* kShooterStateTopic = "/shooter/state";
// Written only by the arbiter; producers use the per-source topics below.
inline constexpr const char* kShooterTargetTopic = "/shooter/tgt";
inline constexpr const char* kTeleopShooterTargetTopic = "/shooter/tgt/teleop";
inline constexpr const char* kAutoShooterTargetTopic = "/shooter/tgt/auto";
inline constexpr const char* kShooterRequestTopic = "/hw/req/shooter";
}  // namespace talos::shooter
