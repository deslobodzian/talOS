#pragma once

#include "talos_nodes/drivetrain/packet.h"

namespace talos::odometry {
// The fused field pose is odometry's own state -- what this node is, not
// something a producer asks it to become -- so the role is `state` and the
// owner segment is this node's name.
inline constexpr const char* kOdometryTopic = "/odometry/state";

// Alias, not a second declaration. Drivetrain owns this name, and a consumer
// that spells it itself is the shape the /hw/req/drive bug had: two spellings
// that agree until one of them is edited.
inline constexpr const char* kDrivetrainStateTopic =
    talos::drive::kDrivetrainStateTopic;
}  // namespace talos::odometry
