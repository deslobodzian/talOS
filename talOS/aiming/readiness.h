#pragma once

// Ready-to-fire decision. Every condition is checked in a fixed order and
// the first one that fails is the reason; ready needs every condition to
// hold on settle_tick_count consecutive control ticks, numbered by the
// caller (nothing here reads a clock). A single failing tick, or a gap in
// the tick numbers, drops readiness at once.

#include <cstdint>
#include <limits>
#include <string_view>

namespace talos::aiming {

// In the order they are checked.
enum class ReadinessReason : std::uint8_t {
  kReady,
  kStale,                 // pose_age_us unknown, negative or > max_pose_age_us
  kNoSolution,            // Solution::valid is false
  kNotConverged,          // Solution::converged is false
  kOutOfRange,            // Solution::in_range is false
  kTurretLimit,           // Solution::turret_reachable is false
  kRobotTooFast,          // |robot_speed_mps| > max_robot_speed_mps
  kRobotRotatingTooFast,  // |robot_omega_radps| > max_robot_omega_radps
  kAimError,              // |aim_error_rad| > aim_tolerance_rad
  kFlywheelNotAtSpeed,    // |flywheel_error_rps| > flywheel_tolerance_rps
  kHoodNotAtGoal,         // |hood_error_rad| > hood_tolerance_rad
  kSettling,              // all hold, not yet for settle_tick_count ticks
};

std::string_view ToString(ReadinessReason reason);

// Tolerances are >= 0 (a NaN or negative one is refused); speed limits may
// be infinite (no limit).
struct ReadinessLimits {
  // Oldest pose (and so Solution) the gate accepts, > 0.
  std::int64_t max_pose_age_us = 0;
  double aim_tolerance_rad = 0.0;
  double flywheel_tolerance_rps = 0.0;
  double hood_tolerance_rad = 0.0;
  double max_robot_speed_mps = 0.0;
  double max_robot_omega_radps = 0.0;
  // Consecutive passing ticks before ready, >= 1 (1: no debounce).
  int settle_tick_count = 1;
};

// One tick's measurements. Every field defaults to failing its check, so a
// field the caller forgets to fill blocks the shot: the flags are false,
// the numbers NaN, the pose age unknown. A robot without a hood sets
// hood_error_rad = 0 explicitly.
//
// pose_age_us: the caller's now minus the sample time of the pose the
// Solution was computed from. The flags come from that Solution. Errors are
// goal minus measured: the aim error is the robot heading error (fixed
// shooter) or turret angle error (turret).
struct ReadinessInput {
  static constexpr std::int64_t kUnknownAge_us =
      std::numeric_limits<std::int64_t>::max();

  std::int64_t pose_age_us = kUnknownAge_us;
  bool solution_valid = false;
  bool converged = false;
  bool in_range = false;
  bool turret_reachable = false;
  double robot_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double robot_omega_radps = std::numeric_limits<double>::quiet_NaN();
  double aim_error_rad = std::numeric_limits<double>::quiet_NaN();
  double flywheel_error_rps = std::numeric_limits<double>::quiet_NaN();
  double hood_error_rad = std::numeric_limits<double>::quiet_NaN();
};

// The first failing condition, without debounce (never kSettling).
ReadinessReason CheckReadiness(const ReadinessInput& input,
                               const ReadinessLimits& limits);

struct Readiness {
  bool ready = false;
  ReadinessReason reason = ReadinessReason::kNoSolution;
  // Consecutive ticks every condition has held, saturating at
  // settle_tick_count.
  int passing_tick_count = 0;
};

class ReadinessGate {
 public:
  // Throws std::invalid_argument naming the bad key.
  explicit ReadinessGate(const ReadinessLimits& limits);

  // Call once per control tick. tick_index is the caller's own tick
  // counter, increasing by one per tick it runs: a passing streak continues
  // only from tick_index - 1, so ticks the caller skipped (disabled,
  // paused, not solving) restart the settle count instead of counting.
  Readiness Update(const ReadinessInput& input, std::uint64_t tick_index);
  // Forget the passing streak (e.g. on a new target).
  void Reset() { passing_tick_count_ = 0; }

  const ReadinessLimits& limits() const { return limits_; }

 private:
  ReadinessLimits limits_;
  int passing_tick_count_ = 0;
  std::uint64_t last_passing_tick_index_ = 0;
};

}  // namespace talos::aiming
