#pragma once

#include "2026-robot/main_processor/drivetrain/packet.h"
#include "2026-robot/main_processor/shooter/packet.h"

namespace talos::odometry {
// The fused field pose is odometry's own state -- what this node is, not
// something a producer asks it to become -- so the role is `state` and the
// owner segment is this node's name.
inline constexpr const char* kOdometryTopic = "/odometry/state";

// Aliases, not second declarations. Drivetrain and shooter own these names, and
// a consumer that spells one itself is the shape the /hw/req/drive bug had: two
// spellings that agree until one of them is edited.
inline constexpr const char* kDrivetrainStateTopic =
    talos::drive::kDrivetrainStateTopic;
inline constexpr const char* kShooterStateTopic =
    talos::shooter::kShooterStateTopic;
}  // namespace talos::odometry
