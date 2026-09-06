#pragma once

#include "talOS/hardware/packet.h"

namespace talos::drive {
// The hardware envelope is a framework type; this alias keeps the historical
// spelling at the call sites that predate the talOS/robot split.
using Packet = hardware::Packet;

inline constexpr const char* kStateTopic = hardware::kStateTopic;
inline constexpr const char* kCommandTopic = hardware::kCommandTopic;
// hardware::kRequestTopicPrefix followed by the `[subsystems.drivetrain]` key,
// which is the string the bridge builds from the config for the other end. It
// read `/hw/req/drive` for a season and only worked because a special case in
// the bridge subscribed to both spellings; spelling the subsystem key out is
// what makes the two ends provably the same topic.
inline constexpr const char* kDriveRequestTopic = "/hw/request/drivetrain";
// The arbiter is the only writer of kTargetTopic. Producers publish to their
// own topic below and never coordinate; see 2026-robot/main_processor/arbiter.
inline constexpr const char* kTargetTopic = "/drivetrain/target";
inline constexpr const char* kTeleopTargetTopic = "/drivetrain/target/teleop";
inline constexpr const char* kAutoTargetTopic = "/drivetrain/target/auto";
inline constexpr const char* kDrivetrainStateTopic = "/drivetrain/state";
}  // namespace talos::drive
