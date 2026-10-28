#pragma once

#include <cstdint>

// The intake's topics. /intake/target has one writer, the arbiter; teleop
// and autonomous publish on their own lanes and the arbiter picks one.
namespace intake {
inline constexpr const char* kStateTopic = "/intake/state";
inline constexpr const char* kTargetTopic = "/intake/target";
inline constexpr const char* kTeleopTargetTopic = "/intake/target/teleop";
inline constexpr const char* kAutoTargetTopic = "/intake/target/auto";

// IntakeState.inhibit: why the wheel is NOT driving. Wire values, shared with
// the Python node; append-only, never renumbered.
inline constexpr std::uint8_t kInhibitDrive = 0;
inline constexpr std::uint8_t kInhibitGateway = 1;
inline constexpr std::uint8_t kInhibitTargetDisabled = 2;
inline constexpr std::uint8_t kInhibitTargetStale = 3;
inline constexpr std::uint8_t kInhibitFeedbackInvalid = 4;
inline constexpr std::uint8_t kInhibitDemandNonfinite = 5;
}  // namespace intake
