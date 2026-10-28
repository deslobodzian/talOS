#pragma once

// The drivetrain's topics. /drivetrain/target has one writer, the arbiter;
// teleop and autonomous publish on their own lanes and the arbiter picks one.
namespace drivetrain {
inline constexpr const char* kStateTopic = "/drivetrain/state";
inline constexpr const char* kTargetTopic = "/drivetrain/target";
inline constexpr const char* kTeleopTargetTopic = "/drivetrain/target/teleop";
inline constexpr const char* kAutoTargetTopic = "/drivetrain/target/auto";
}  // namespace drivetrain
