#pragma once

#include <string>

#include "talOS/hardware/packet.h"

namespace talos::drive {
// The hardware envelope is a framework type; this alias keeps the historical
// spelling at the call sites that predate the talOS/robot split.
using Packet = hardware::Packet;

inline constexpr const char* kStateTopic = hardware::kStateTopic;
inline constexpr const char* kCommandTopic = hardware::kCommandTopic;
// Spelled once, behind hardware::RequestTopic: the bridge derives the same
// string from the `[subsystems.drivetrain]` key in the config, so calling the
// helper is what makes the two ends provably the same topic. (It once read
// `/hw/req/drive` here and only worked because a special case in the bridge
// subscribed to both spellings.)
inline const std::string kDriveRequestTopic =
    hardware::RequestTopic("drivetrain");
// The arbiter is the only writer of kTargetTopic. Producers publish to their
// own topic below and never coordinate; see 2026-robot/main_processor/arbiter.
inline constexpr const char* kTargetTopic = "/drivetrain/target";
inline constexpr const char* kTeleopTargetTopic = "/drivetrain/target/teleop";
inline constexpr const char* kAutoTargetTopic = "/drivetrain/target/auto";
inline constexpr const char* kDrivetrainStateTopic = "/drivetrain/state";
}  // namespace talos::drive
