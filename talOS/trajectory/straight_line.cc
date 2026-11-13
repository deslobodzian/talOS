#include "talOS/trajectory/straight_line.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>

#include "talOS/geometry/se2.h"
#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {
namespace {

bool Positive(double value) { return std::isfinite(value) && value > 0; }

// Shortest rest-to-rest time to cover `distance` (>= 0) with speed limit
// `speed` and acceleration limit `accel`.
double MinimumTime(double distance, double speed, double accel) {
  if (distance <= 0) return 0;
  if (distance * accel <= speed * speed) return 2 * std::sqrt(distance / accel);
  return distance / speed + speed / accel;
}

// One axis, rest to rest over [0, total_s]: accelerate at `accel` for
// ramp_s, cruise, decelerate for ramp_s.
struct Profile {
  double distance = 0;
  double accel = 0;
  double cruise = 0;
  double ramp_s = 0;
  double total_s = 0;

  double position(double t_s) const {
    if (cruise == 0) return 0;  // distance is 0 (Consistent)
    if (t_s <= ramp_s) return 0.5 * accel * t_s * t_s;
    if (t_s < total_s - ramp_s) {
      return 0.5 * cruise * ramp_s + cruise * (t_s - ramp_s);
    }
    const double left_s = total_s - t_s;
    return distance - 0.5 * accel * left_s * left_s;
  }
  double velocity(double t_s) const {
    if (cruise == 0) return 0;
    if (t_s <= ramp_s) return accel * t_s;
    if (t_s < total_s - ramp_s) return cruise;
    return accel * (total_s - t_s);
  }
  double acceleration(double t_s) const {
    if (cruise == 0) return 0;
    if (t_s < ramp_s) return accel;
    if (t_s < total_s - ramp_s) return 0;
    return -accel;
  }
};

// The profile covering `distance` in exactly `total_s` (at least the
// axis's MinimumTime) at full acceleration and the lowest cruise speed that
// does it. The axis that sets total_s cruises at its limit, or peaks at
// sqrt(accel distance) (a triangle). The other solves cruise^2 - accel
// total cruise + accel distance = 0 for the smaller root, written to avoid
// cancellation.
Profile Fit(double distance, double speed, double accel, double total_s) {
  Profile p{distance, accel, 0, 0, total_s};
  if (distance <= 0 || total_s <= 0) return p;
  if (total_s <= MinimumTime(distance, speed, accel)) {
    p.cruise =
        distance * accel <= speed * speed ? std::sqrt(distance * accel) : speed;
  } else {
    const double at = accel * total_s;
    const double discriminant = std::max(0.0, at * at - 4 * accel * distance);
    p.cruise =
        std::min(speed, 2 * accel * distance / (at + std::sqrt(discriminant)));
  }
  p.ramp_s = std::min(p.cruise / accel, total_s / 2);
  return p;
}

// The profile covers its distance (to rounding): false when the arithmetic
// overflowed or underflowed.
bool Consistent(const Profile& p) {
  if (p.distance <= 0) return true;
  const double covered = p.cruise * (p.total_s - p.ramp_s);
  return std::isfinite(covered) &&
         std::abs(covered - p.distance) <= 1e-9 * p.distance;
}

bool Finite(const TrajectorySample& s) {
  for (const double value :
       {s.t_s, s.x_m, s.y_m, s.heading_rad, s.vx_mps, s.vy_mps, s.omega_radps,
        s.ax_mps2, s.ay_mps2, s.alpha_radps2}) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

}  // namespace

const char* ToString(StraightLineError error) {
  switch (error) {
    case StraightLineError::kNone:
      return "";
    case StraightLineError::kNonFinitePose:
      return "start or end pose is not finite";
    case StraightLineError::kInvalidLimits:
      return "a straight-line limit is not finite and positive";
    case StraightLineError::kCapacity:
      return "trajectory capacity is below kStraightLineSamples";
    case StraightLineError::kUnrepresentable:
      return "straight-line profile is not representable";
  }
  return "unknown straight-line error";
}

StraightLineResult MakeStraightLine(const geometry::SE2<double>& start,
                                    const geometry::SE2<double>& end,
                                    const StraightLineLimits& limits,
                                    Trajectory& out) {
  const auto fail = [&out](StraightLineError error) {
    out.clear();
    return StraightLineResult{error};
  };
  const double x0 = start.translation().x();
  const double y0 = start.translation().y();
  const double h0 = start.angle();
  const double x1 = end.translation().x();
  const double y1 = end.translation().y();
  const double h1 = end.angle();
  for (const double value : {x0, y0, h0, x1, y1, h1}) {
    if (!std::isfinite(value)) return fail(StraightLineError::kNonFinitePose);
  }
  if (!(Positive(limits.max_linear_mps) &&
        Positive(limits.max_linear_accel_mps2) &&
        Positive(limits.max_angular_radps) &&
        Positive(limits.max_angular_accel_radps2))) {
    return fail(StraightLineError::kInvalidLimits);
  }
  if (out.capacity() < kStraightLineSamples) {
    return fail(StraightLineError::kCapacity);
  }

  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double distance = std::hypot(dx, dy);
  const double turn = WrapAngle(h1 - h0);
  if (!(std::isfinite(distance) && std::isfinite(turn))) {
    return fail(StraightLineError::kUnrepresentable);
  }
  const double ux = distance > 0 ? dx / distance : 0;
  const double uy = distance > 0 ? dy / distance : 0;
  const double direction = turn < 0 ? -1 : 1;
  const double total_s =
      std::max(MinimumTime(distance, limits.max_linear_mps,
                           limits.max_linear_accel_mps2),
               MinimumTime(std::abs(turn), limits.max_angular_radps,
                           limits.max_angular_accel_radps2));
  if (!std::isfinite(total_s)) return fail(StraightLineError::kUnrepresentable);
  const Profile linear = Fit(distance, limits.max_linear_mps,
                             limits.max_linear_accel_mps2, total_s);
  const Profile angular = Fit(std::abs(turn), limits.max_angular_radps,
                              limits.max_angular_accel_radps2, total_s);
  if (!(Consistent(linear) && Consistent(angular))) {
    return fail(StraightLineError::kUnrepresentable);
  }

  // Every instant where either axis changes acceleration, in order, each
  // once; total_s is last.
  std::array<double, kStraightLineSamples> breaks{0,
                                                  linear.ramp_s,
                                                  total_s - linear.ramp_s,
                                                  angular.ramp_s,
                                                  total_s - angular.ramp_s,
                                                  total_s};
  std::sort(breaks.begin(), breaks.end());
  std::array<double, kStraightLineSamples> times{};
  std::size_t count = 0;
  for (const double t_s : breaks) {
    if (count == 0 || t_s > times[count - 1]) times[count++] = t_s;
  }

  std::array<TrajectorySample, kStraightLineSamples> samples{};
  for (std::size_t i = 0; i < count; ++i) {
    const double t_s = times[i];
    TrajectorySample& s = samples[i];
    s.t_s = t_s;
    if (i + 1 == count) {
      // Exactly the end, at rest.
      s.x_m = x1;
      s.y_m = y1;
      s.heading_rad = h0 + turn;
      continue;
    }
    const double along_m = linear.position(t_s);
    const double speed_mps = linear.velocity(t_s);
    s.x_m = x0 + ux * along_m;
    s.y_m = y0 + uy * along_m;
    s.heading_rad = h0 + direction * angular.position(t_s);
    s.vx_mps = ux * speed_mps;
    s.vy_mps = uy * speed_mps;
    s.omega_radps = direction * angular.velocity(t_s);
    // The acceleration held until the next sample: the one in force at the
    // interval's midpoint, so it is the same piece on both sides.
    const double middle_s = 0.5 * (t_s + times[i + 1]);
    const double accel_mps2 = linear.acceleration(middle_s);
    s.ax_mps2 = ux * accel_mps2;
    s.ay_mps2 = uy * accel_mps2;
    s.alpha_radps2 = direction * angular.acceleration(middle_s);
  }
  const std::span<const TrajectorySample> written{samples.data(), count};
  for (const TrajectorySample& s : written) {
    if (!Finite(s)) return fail(StraightLineError::kUnrepresentable);
  }
  // Finite, non-negative and strictly increasing times, within capacity:
  // assign() neither throws nor allocates.
  out.assign(written, Interpolation::kConstantAcceleration);
  return {};
}

}  // namespace talos::trajectory
