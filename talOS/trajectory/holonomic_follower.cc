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
  if (dt_s > 0 && gains.ki > 0) {
    integral += error * dt_s;
    // Anti-windup: hold the integral term within its limit.
    const double limit = gains.integral_limit / gains.ki;
    integral = std::clamp(integral, -limit, limit);
  }
  pending_dt_s += dt_s;
  if (!has_previous) {
    previous_error = error;
    pending_dt_s = 0;
    has_previous = true;
  } else if (pending_dt_s >= kMinDerivativePeriod_s) {
    const double change =
        angular ? WrapAngle(error - previous_error) : error - previous_error;
    derivative = change / pending_dt_s;
    previous_error = error;
    pending_dt_s = 0;
  }
  return gains.kp * error + gains.ki * integral + gains.kd * derivative;
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
  out.x_error_m = out.reference.x_m - measured_x_m;
  out.y_error_m = out.reference.y_m - measured_y_m;
  out.heading_error_rad =
      WrapAngle(out.reference.heading_rad - measured_heading_rad);

  double vx_mps = out.reference.vx_mps +
                  x_.update(config_.translation, out.x_error_m, dt_s, false);
  double vy_mps = out.reference.vy_mps +
                  y_.update(config_.translation, out.y_error_m, dt_s, false);
  const double omega_radps =
      out.reference.omega_radps +
      heading_.update(config_.heading, out.heading_error_rad, dt_s, true);

  // Scale, not clip per axis, so the command keeps its direction.
  const double speed_mps = std::hypot(vx_mps, vy_mps);
  if (speed_mps > config_.max_linear_mps) {
    const double scale = config_.max_linear_mps / speed_mps;
    vx_mps *= scale;
    vy_mps *= scale;
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
  out.finished =
      t_s >= trajectory.total_time_s() &&
      std::hypot(out.x_error_m, out.y_error_m) <=
          config_.position_tolerance_m &&
      std::abs(out.heading_error_rad) <= config_.heading_tolerance_rad;
  out.valid = true;
  return out;
}

}  // namespace talos::trajectory
