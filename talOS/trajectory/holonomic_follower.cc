#include "talOS/trajectory/holonomic_follower.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "talOS/geometry/se2.h"
#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {
namespace {

void RequireNonNegative(double value, const std::string& name) {
  if (!(std::isfinite(value) && value >= 0)) {
    throw std::invalid_argument("HolonomicFollower: " + name +
                                " must be finite and non-negative");
  }
}

void RequireGains(const PidGains& gains, const std::string& name) {
  RequireNonNegative(gains.kp, name + ".kp");
  RequireNonNegative(gains.ki, name + ".ki");
  RequireNonNegative(gains.kd, name + ".kd");
  RequireNonNegative(gains.integral_limit, name + ".integral_limit");
}

}  // namespace

HolonomicFollower::HolonomicFollower(const FollowerConfig& config)
    : config_(config) {
  RequireGains(config.translation, "translation");
  RequireGains(config.heading, "heading");
  RequireNonNegative(config.max_linear_mps, "max_linear_mps");
  RequireNonNegative(config.max_angular_radps, "max_angular_radps");
  RequireNonNegative(config.position_tolerance_m, "position_tolerance_m");
  RequireNonNegative(config.heading_tolerance_rad, "heading_tolerance_rad");
  if (config.max_linear_mps == 0 || config.max_angular_radps == 0) {
    throw std::invalid_argument(
        "HolonomicFollower: max_linear_mps and max_angular_radps must be "
        "positive");
  }
}

void HolonomicFollower::reset() {
  x_ = {};
  y_ = {};
  heading_ = {};
}

double HolonomicFollower::Axis::update(const PidGains& gains, double error,
                                       double dt_s, bool angular) {
  if (dt_s > 0 && gains.ki > 0 && gains.integral_limit > 0) {
    const double limit = gains.integral_limit / gains.ki;
    const double delta = error * dt_s;
    // Anti-windup: hold the integral term within its limit without letting
    // error * dt_s overflow poison the accumulator.
    if (std::isfinite(limit)) {
      if (std::isfinite(delta)) {
        integral = std::clamp(integral + delta, -limit, limit);
      } else if (!std::isnan(delta)) {
        integral = delta > 0 ? limit : -limit;
      }
    } else if (std::isfinite(delta)) {
      const double next = integral + delta;
      if (std::isfinite(next)) integral = next;
    }
  }
  pending_dt_s += dt_s;
  if (!std::isfinite(error)) {
    // Do not latch an overflowing error into derivative history.
    previous_error = 0.0;
    pending_dt_s = 0.0;
    derivative = 0.0;
    has_previous = false;
  } else if (!has_previous) {
    previous_error = error;
    pending_dt_s = 0.0;
    has_previous = true;
  } else if (pending_dt_s >= kMinDerivativePeriod_s) {
    if (gains.kd > 0) {
      const double raw_change =
          (error == previous_error) ? 0.0 : error - previous_error;
      if (std::isfinite(raw_change)) {
        const double change = angular ? WrapAngle(raw_change) : raw_change;
        const double next_derivative = change / pending_dt_s;
        // Guard non-finite derivative or d_term so +-inf never latches across
        // subsequent dt_s < kMinDerivativePeriod_s ticks.
        derivative = (std::isfinite(next_derivative) &&
                      std::isfinite(gains.kd * next_derivative))
                         ? next_derivative
                         : 0.0;
      } else {
        derivative = 0.0;
      }
    } else {
      derivative = 0.0;
    }
    previous_error = error;
    pending_dt_s = 0.0;
  }
  const double p_term = gains.kp > 0 ? gains.kp * error : 0.0;
  const double i_term =
      (gains.ki > 0 && gains.integral_limit > 0)
          ? std::clamp(gains.ki * integral, -gains.integral_limit,
                       gains.integral_limit)
          : 0.0;
  const double d_term = gains.kd > 0 ? gains.kd * derivative : 0.0;
  return p_term + i_term + d_term;
}

FollowerOutput HolonomicFollower::step(const Trajectory& trajectory, double t_s,
                                       const geometry::SE2<double>& measured,
                                       double dt_s, SpeedsFrame frame) {
  FollowerOutput out;
  const double measured_x_m = measured.translation().x();
  const double measured_y_m = measured.translation().y();
  const double measured_heading_rad = measured.angle();
  if (!(std::isfinite(t_s) && std::isfinite(dt_s) && dt_s >= 0 &&
        std::isfinite(measured_x_m) && std::isfinite(measured_y_m) &&
        std::isfinite(measured_heading_rad)) ||
      trajectory.empty()) {
    return out;
  }
  out.reference = trajectory.sample(t_s);
  if (!(std::isfinite(out.reference.x_m) && std::isfinite(out.reference.y_m) &&
        std::isfinite(out.reference.heading_rad) &&
        std::isfinite(out.reference.vx_mps) &&
        std::isfinite(out.reference.vy_mps) &&
        std::isfinite(out.reference.omega_radps))) {
    return FollowerOutput{};
  }
  out.x_error_m = out.reference.x_m - measured_x_m;
  out.y_error_m = out.reference.y_m - measured_y_m;
  const double raw_heading_diff =
      out.reference.heading_rad - measured_heading_rad;
  out.heading_error_rad = std::isfinite(raw_heading_diff)
                              ? WrapAngle(raw_heading_diff)
                              : WrapAngle(WrapAngle(out.reference.heading_rad) -
                                          WrapAngle(measured_heading_rad));

  Axis next_x = x_;
  Axis next_y = y_;
  Axis next_heading = heading_;
  double vx_mps =
      out.reference.vx_mps +
      next_x.update(config_.translation, out.x_error_m, dt_s, false);
  double vy_mps =
      out.reference.vy_mps +
      next_y.update(config_.translation, out.y_error_m, dt_s, false);
  const double omega_radps =
      out.reference.omega_radps +
      next_heading.update(config_.heading, out.heading_error_rad, dt_s, true);

  if (std::isnan(vx_mps) || std::isnan(vy_mps) || std::isnan(omega_radps)) {
    return FollowerOutput{};
  }

  // Scale, not clip per axis, so the command keeps its direction. Guard
  // against overflow in kp * error (+-inf) and in hypot(vx, vy) (where
  // max_linear_mps / +inf == 0 and inf * 0 == NaN).
  if (std::isinf(vx_mps) || std::isinf(vy_mps)) {
    const double ux = std::isinf(vx_mps) ? std::copysign(1.0, vx_mps) : 0.0;
    const double uy = std::isinf(vy_mps) ? std::copysign(1.0, vy_mps) : 0.0;
    const double scale = config_.max_linear_mps / std::hypot(ux, uy);
    vx_mps = ux * scale;
    vy_mps = uy * scale;
  } else {
    const double max_component = std::max(std::abs(vx_mps), std::abs(vy_mps));
    if (max_component > 0) {
      const double ux = vx_mps / max_component;
      const double uy = vy_mps / max_component;
      const double unit_norm = std::hypot(ux, uy);
      const double speed_mps = max_component * unit_norm;
      if (!(speed_mps <= config_.max_linear_mps)) {
        const double scale = config_.max_linear_mps / unit_norm;
        vx_mps = ux * scale;
        vy_mps = uy * scale;
      }
    }
  }
  out.speeds.omega_radps = std::clamp(omega_radps, -config_.max_angular_radps,
                                      config_.max_angular_radps);
  if (frame == SpeedsFrame::kRobot) {
    const double c = std::cos(measured_heading_rad);
    const double s = std::sin(measured_heading_rad);
    out.speeds.vx_mps = vx_mps * c + vy_mps * s;
    out.speeds.vy_mps = -vx_mps * s + vy_mps * c;
  } else {
    out.speeds.vx_mps = vx_mps;
    out.speeds.vy_mps = vy_mps;
  }
  if (!(std::isfinite(out.speeds.vx_mps) && std::isfinite(out.speeds.vy_mps) &&
        std::isfinite(out.speeds.omega_radps))) {
    return FollowerOutput{};
  }
  x_ = next_x;
  y_ = next_y;
  heading_ = next_heading;
  out.finished =
      t_s >= trajectory.total_time_s() &&
      std::hypot(out.x_error_m, out.y_error_m) <=
          config_.position_tolerance_m &&
      std::abs(out.heading_error_rad) <= config_.heading_tolerance_rad;
  out.valid = true;
  return out;
}

}  // namespace talos::trajectory
