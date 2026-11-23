#include "talOS/aiming/readiness.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace talos::aiming {
namespace {

// Written so that NaN and +-inf measurements fail even against an infinite
// speed limit, and non-finite tolerances fail even for direct CheckReadiness()
// callers.
bool Within(double value, double limit, bool allow_infinite_limit = false) {
  return std::isfinite(value) &&
         (allow_infinite_limit || std::isfinite(limit)) &&
         std::abs(value) <= limit;
}

void RequireLimit(double value, const char* key, bool allow_infinity) {
  if (!(value >= 0.0) || (!allow_infinity && !std::isfinite(value))) {
    throw std::invalid_argument(
        std::string("aiming: ") + key +
        (allow_infinity ? " must be >= 0" : " must be finite and >= 0"));
  }
}

}  // namespace

std::string_view ToString(ReadinessReason reason) {
  switch (reason) {
    case ReadinessReason::kReady:
      return "ready";
    case ReadinessReason::kStale:
      return "stale";
    case ReadinessReason::kNoSolution:
      return "no solution";
    case ReadinessReason::kOutOfRange:
      return "out of range";
    case ReadinessReason::kNotConverged:
      return "not converged";
    case ReadinessReason::kTurretLimit:
      return "turret limit";
    case ReadinessReason::kRobotTooFast:
      return "robot too fast";
    case ReadinessReason::kRobotRotatingTooFast:
      return "robot rotating too fast";
    case ReadinessReason::kAimError:
      return "aim error";
    case ReadinessReason::kFlywheelNotAtSpeed:
      return "flywheel not at speed";
    case ReadinessReason::kHoodNotAtGoal:
      return "hood not at goal";
    case ReadinessReason::kSettling:
      return "settling";
  }
  return "unknown";
}

ReadinessReason CheckReadiness(const ReadinessInput& input,
                               const ReadinessLimits& limits) {
  if (limits.max_pose_age_us <= 0 || input.pose_age_us < 0 ||
      input.pose_age_us > limits.max_pose_age_us) {
    return ReadinessReason::kStale;
  }
  if (!input.solution_valid) return ReadinessReason::kNoSolution;
  // A non-converged iterate's distance, and so its in_range, means nothing.
  if (!input.converged) return ReadinessReason::kNotConverged;
  if (!input.in_range) return ReadinessReason::kOutOfRange;
  if (!input.turret_reachable) return ReadinessReason::kTurretLimit;
  if (!Within(input.robot_speed_mps, limits.max_robot_speed_mps,
              /*allow_infinite_limit=*/true)) {
    return ReadinessReason::kRobotTooFast;
  }
  if (!Within(input.robot_omega_radps, limits.max_robot_omega_radps,
              /*allow_infinite_limit=*/true)) {
    return ReadinessReason::kRobotRotatingTooFast;
  }
  if (!Within(input.aim_error_rad, limits.aim_tolerance_rad)) {
    return ReadinessReason::kAimError;
  }
  if (!Within(input.flywheel_error_rps, limits.flywheel_tolerance_rps)) {
    return ReadinessReason::kFlywheelNotAtSpeed;
  }
  if (!Within(input.hood_error_rad, limits.hood_tolerance_rad)) {
    return ReadinessReason::kHoodNotAtGoal;
  }
  return ReadinessReason::kReady;
}

ReadinessGate::ReadinessGate(const ReadinessLimits& limits) : limits_(limits) {
  if (limits.max_pose_age_us <= 0) {
    throw std::invalid_argument("aiming: max_pose_age_us must be > 0");
  }
  RequireLimit(limits.aim_tolerance_rad, "aim_tolerance_rad",
               /*allow_infinity=*/false);
  RequireLimit(limits.flywheel_tolerance_rps, "flywheel_tolerance_rps",
               /*allow_infinity=*/false);
  RequireLimit(limits.hood_tolerance_rad, "hood_tolerance_rad",
               /*allow_infinity=*/false);
  RequireLimit(limits.max_robot_speed_mps, "max_robot_speed_mps",
               /*allow_infinity=*/true);
  RequireLimit(limits.max_robot_omega_radps, "max_robot_omega_radps",
               /*allow_infinity=*/true);
  if (limits.settle_tick_count < 1) {
    throw std::invalid_argument("aiming: settle_tick_count must be >= 1");
  }
}

Readiness ReadinessGate::Update(const ReadinessInput& input,
                                std::uint64_t tick_index) {
  const ReadinessReason reason = CheckReadiness(input, limits_);
  if (reason != ReadinessReason::kReady) {
    passing_tick_count_ = 0;
    return {false, reason, 0};
  }
  // A streak continues only from the immediately preceding tick; a gap, a
  // repeat, a step back, or a uint64 wraparound starts it over.
  const bool continues = passing_tick_count_ > 0 &&
                         tick_index > last_passing_tick_index_ &&
                         tick_index - last_passing_tick_index_ == 1;
  if (!continues) {
    passing_tick_count_ = 1;
  } else if (passing_tick_count_ < limits_.settle_tick_count) {
    ++passing_tick_count_;
  }
  last_passing_tick_index_ = tick_index;
  if (passing_tick_count_ < limits_.settle_tick_count) {
    return {false, ReadinessReason::kSettling, passing_tick_count_};
  }
  return {true, ReadinessReason::kReady, passing_tick_count_};
}

}  // namespace talos::aiming
