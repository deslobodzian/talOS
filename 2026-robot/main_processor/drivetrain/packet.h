#pragma once

#include "talOS/hardware/packet.h"

namespace talos::drive {
// The hardware envelope is a framework type; this alias keeps the historical
// spelling at the call sites that predate the talOS/robot split.
using Packet = hardware::Packet;

inline constexpr const char* kStateTopic = hardware::kStateTopic;
inline constexpr const char* kCommandTopic = hardware::kCommandTopic;
inline constexpr const char* kDriveRequestTopic = "/hw/req/drive";
// The arbiter is the only writer of kTargetTopic. Producers publish to their
// own topic below and never coordinate; see 2026-robot/main_processor/arbiter.
inline constexpr const char* kTargetTopic = "/drivetrain/tgt";
inline constexpr const char* kTeleopTargetTopic = "/drivetrain/tgt/teleop";
inline constexpr const char* kAutoTargetTopic = "/drivetrain/tgt/auto";
inline constexpr const char* kDrivetrainStateTopic = "/drivetrain/state";
}  // namespace talos::drive
