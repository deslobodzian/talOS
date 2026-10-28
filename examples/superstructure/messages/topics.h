#pragma once

// The example's topics. Each mechanism publishes its own /<name>/state and
// reads /<name>/target; the superstructure is the only writer of the
// mechanism targets.
namespace demo {
inline constexpr const char* kIntakeGoalTopic = "/intake/target";
inline constexpr const char* kIntakeStatusTopic = "/intake/state";
inline constexpr const char* kElevatorGoalTopic = "/elevator/target";
inline constexpr const char* kElevatorStatusTopic = "/elevator/state";
inline constexpr const char* kSuperstructureGoalTopic = "/superstructure/target";
inline constexpr const char* kSuperstructureStatusTopic = "/superstructure/state";
}  // namespace demo
