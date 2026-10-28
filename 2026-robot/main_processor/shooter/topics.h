#pragma once

// The shooter's topics. /shooter/target has one writer, the arbiter; teleop
// and autonomous publish on their own lanes and the arbiter picks one.
namespace shooter {
inline constexpr const char* kStateTopic = "/shooter/state";
inline constexpr const char* kTargetTopic = "/shooter/target";
inline constexpr const char* kTeleopTargetTopic = "/shooter/target/teleop";
inline constexpr const char* kAutoTargetTopic = "/shooter/target/auto";
}  // namespace shooter
