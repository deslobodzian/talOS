#pragma once

#include "talOS/hardware/packet.h"

namespace talos::intake {
using Packet = hardware::Packet;

inline constexpr const char* kHwStateTopic = hardware::kStateTopic;
inline constexpr const char* kIntakeStateTopic = "/intake/state";
// Written only by the arbiter; producers use the per-source topics below.
inline constexpr const char* kTargetTopic = "/intake/target";
inline constexpr const char* kTeleopIntakeTargetTopic = "/intake/target/teleop";
inline constexpr const char* kAutoIntakeTargetTopic = "/intake/target/auto";
// hardware::kRequestTopicPrefix followed by the `[subsystems.intake]` key,
// which is how the bridge spells its end of the same topic.
inline constexpr const char* kIntakeRequestTopic = "/hw/request/intake";
}  // namespace talos::intake
