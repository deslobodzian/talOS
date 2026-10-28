#pragma once

// The fused field pose is odometry's own state: the role is `state` and the
// owner segment is this node's name.
namespace odometry {
inline constexpr const char* kStateTopic = "/odometry/state";
}  // namespace odometry
