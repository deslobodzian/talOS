#pragma once

// A straight move between two field poses as a Trajectory (README.md
// "Straight lines"), for a path built inside update() from the live pose.

#include <cstddef>

#include "talOS/geometry/se2.h"
#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {

// Each limit must be finite and positive.
struct StraightLineLimits {
  double max_linear_mps = 0;
  double max_linear_accel_mps2 = 0;
  double max_angular_radps = 0;
  double max_angular_accel_radps2 = 0;
};

// Samples MakeStraightLine() writes at most: one at each acceleration change
// of either profile, plus the ends. `out` must have reserved this many.
inline constexpr std::size_t kStraightLineSamples = 6;

enum class StraightLineError {
  kNone,
  kNonFinitePose,   // a start or end coordinate or heading is NaN or infinite
  kInvalidLimits,   // a limit is not finite and positive
  kCapacity,        // out.capacity() < kStraightLineSamples
  kUnrepresentable  // the profile overflowed (e.g. tiny limits, huge move)
};

struct StraightLineResult {
  StraightLineError error = StraightLineError::kNone;
  explicit operator bool() const { return error == StraightLineError::kNone; }
};

// A static string naming the error ("" for kNone).
const char* ToString(StraightLineError error);

// Fills `out` with a move from `start` to `end` (field frame) along the
// straight segment between them, the heading turning the short way
// (WrapAngle of the difference; exactly pi turns CCW). Translation (distance
// along the segment) and heading each follow a trapezoidal profile from rest
// to rest, triangular when the move is too short to reach the speed limit.
// The duration is the longer of the two axes' minimum times; the other axis
// keeps its acceleration limit and lowers its cruise speed so both finish
// together. Samples sit at every acceleration change, with
// Interpolation::kConstantAcceleration, so sample() reproduces the profile
// exactly between them (to rounding). The heading is continuous from
// start.angle(); the last sample is exactly `end`'s position at the total time
// with zero velocity. A move with no translation and no turn is one sample at
// t = 0.
//
// On an error `out` is cleared (empty; the follower then commands zero) and
// the result names why. Never throws; never allocates while out.capacity()
// >= kStraightLineSamples (reserve it in the node's constructor).
StraightLineResult MakeStraightLine(const geometry::SE2<double>& start,
                                    const geometry::SE2<double>& end,
                                    const StraightLineLimits& limits,
                                    Trajectory& out);

}  // namespace talos::trajectory
