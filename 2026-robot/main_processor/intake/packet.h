#pragma once

#include <cstdint>
#include <string>

#include "talOS/hardware/packet.h"

namespace talos::intake {
using Packet = hardware::Packet;

inline constexpr const char* kHwStateTopic = hardware::kStateTopic;
inline constexpr const char* kIntakeStateTopic = "/intake/state";
// Written only by the arbiter; producers use the per-source topics below.
inline constexpr const char* kTargetTopic = "/intake/target";
inline constexpr const char* kTeleopIntakeTargetTopic = "/intake/target/teleop";
inline constexpr const char* kAutoIntakeTargetTopic = "/intake/target/auto";
// Spelled once, behind hardware::RequestTopic: the bridge derives the same
// string from the `[subsystems.intake]` key in the config, so calling the
// helper is what makes the two ends provably the same topic.
inline const std::string kIntakeRequestTopic = hardware::RequestTopic("intake");

// IntakeState.inhibit codes: why the wheel is NOT driving. Shared with
// intake_message.fbs (comment) and talos_nodes/intake/packet.py; the numeric
// values are the wire contract, so append-only, never renumbered.
inline constexpr uint8_t kInhibitDrive = 0;
inline constexpr uint8_t kInhibitGateway = 1;
inline constexpr uint8_t kInhibitTargetDisabled = 2;
inline constexpr uint8_t kInhibitTargetStale = 3;
inline constexpr uint8_t kInhibitFeedbackInvalid = 4;
inline constexpr uint8_t kInhibitDemandNonfinite = 5;
}  // namespace talos::intake
