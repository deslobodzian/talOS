#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <span>
#include <stdexcept>

#include "drivetrain/drive_message_generated.h"
#include "geometry/geometry_generated.h"

namespace autonomous {

inline double WrapAngle(double angle) {
  return std::remainder(angle, 2 * std::numbers::pi);
}

struct TrajectorySample {
  double time_s{};
  double x_m{};
  double y_m{};
  double yaw_rad{};
  double vx_mps{};
  double vy_mps{};
  double omega_radps{};
};

// Owns bounded storage: no heap allocation, including while sampling/flipping.
class Trajectory {
 public:
  static constexpr std::size_t kCapacity = 64;
  Trajectory() = default;
  explicit Trajectory(std::span<const TrajectorySample> samples) {
    if (samples.empty() || samples.size() > kCapacity) {
      throw std::invalid_argument("trajectory sample count");
    }
    for (const auto& s : samples) {
      for (double value : {s.time_s, s.x_m, s.y_m, s.yaw_rad, s.vx_mps,
                           s.vy_mps, s.omega_radps}) {
        if (!std::isfinite(value))
          throw std::invalid_argument("trajectory value");
      }
      if (s.time_s < 0 || (count_ && s.time_s <= samples_[count_ - 1].time_s)) {
        throw std::invalid_argument("trajectory sample times");
      }
      samples_[count_++] = s;
    }
  }

  // A synchronized trapezoid: accelerate for 1/4, cruise for 1/2,
  // decelerate for 1/4 of the duration. Hermite interpolation is exact
  // on these constant-acceleration segments.
  static Trajectory FromPoses(const geometry::Pose2d& start,
                              const geometry::Pose2d& target,
                              double max_linear_mps, double max_angular_radps) {
    if (!(std::isfinite(max_linear_mps) && max_linear_mps > 0 &&
          std::isfinite(max_angular_radps) && max_angular_radps > 0)) {
      throw std::invalid_argument("trajectory speed limits");
    }
    const double dx = target.x_m() - start.x_m();
    const double dy = target.y_m() - start.y_m();
    const double da = WrapAngle(target.yaw_rad() - start.yaw_rad());
    const double duration =
        std::max({std::hypot(dx, dy) / (0.75 * max_linear_mps),
                  std::abs(da) / (0.75 * max_angular_radps), 0.001});
    std::array<TrajectorySample, 4> samples{};
    constexpr std::array<double, 4> times{0, 0.25, 0.75, 1};
    constexpr std::array<double, 4> positions{0, 1.0 / 6, 5.0 / 6, 1};
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const double velocity = (i == 1 || i == 2) ? 1 / (0.75 * duration) : 0;
      samples[i] = {times[i] * duration,
                    start.x_m() + positions[i] * dx,
                    start.y_m() + positions[i] * dy,
                    start.yaw_rad() + positions[i] * da,
                    velocity * dx,
                    velocity * dy,
                    velocity * da};
    }
    return Trajectory{samples};
  }

  double total_time_s() const {
    return count_ ? samples_[count_ - 1].time_s : 0;
  }
  TrajectorySample SampleAt(double elapsed_s) const {
    if (!count_) return {};
    if (elapsed_s <= samples_[0].time_s) return samples_[0];
    if (elapsed_s >= total_time_s()) return samples_[count_ - 1];
    std::size_t i = 1;
    while (samples_[i].time_s < elapsed_s) ++i;
    const auto& a = samples_[i - 1];
    const auto& b = samples_[i];
    const double dt = b.time_s - a.time_s;
    const double u = (elapsed_s - a.time_s) / dt;
    const auto interpolate = [u, dt](double p, double q, double v, double w) {
      return (2 * u * u * u - 3 * u * u + 1) * p +
             (u * u * u - 2 * u * u + u) * dt * v +
             (-2 * u * u * u + 3 * u * u) * q + (u * u * u - u * u) * dt * w;
    };
    const auto velocity = [u, dt](double p, double q, double v, double w) {
      return (6 * u * u - 6 * u) * (p - q) / dt + (3 * u * u - 4 * u + 1) * v +
             (3 * u * u - 2 * u) * w;
    };
    const double yaw = a.yaw_rad + WrapAngle(b.yaw_rad - a.yaw_rad);
    return {elapsed_s,
            interpolate(a.x_m, b.x_m, a.vx_mps, b.vx_mps),
            interpolate(a.y_m, b.y_m, a.vy_mps, b.vy_mps),
            interpolate(a.yaw_rad, yaw, a.omega_radps, b.omega_radps),
            velocity(a.x_m, b.x_m, a.vx_mps, b.vx_mps),
            velocity(a.y_m, b.y_m, a.vy_mps, b.vy_mps),
            velocity(a.yaw_rad, yaw, a.omega_radps, b.omega_radps)};
  }
  void FlipForRed(double field_length_m, double field_width_m) {
    for (std::size_t i = 0; i < count_; ++i) {
      auto& s = samples_[i];
      s.x_m = field_length_m - s.x_m;
      s.y_m = field_width_m - s.y_m;
      s.yaw_rad = WrapAngle(s.yaw_rad + std::numbers::pi);
      s.vx_mps = -s.vx_mps;
      s.vy_mps = -s.vy_mps;
    }
  }

 private:
  std::array<TrajectorySample, kCapacity> samples_{};
  std::size_t count_{};
};

struct HolonomicController {
  double kp_xy = 1;
  double kp_yaw = 1;
  double max_linear_mps = 2;
  double max_angular_radps = 3;

  drivetrain::ChassisTarget Calculate(const TrajectorySample& desired,
                                      const geometry::Pose2d& measured) const {
    double vx = desired.vx_mps + kp_xy * (desired.x_m - measured.x_m());
    double vy = desired.vy_mps + kp_xy * (desired.y_m - measured.y_m());
    const double omega =
        desired.omega_radps +
        kp_yaw * WrapAngle(desired.yaw_rad - measured.yaw_rad());
    if (!(std::isfinite(vx) && std::isfinite(vy) && std::isfinite(omega) &&
          std::isfinite(measured.yaw_rad())))
      return {};
    const double speed = std::hypot(vx, vy);
    if (speed > max_linear_mps) {
      vx *= max_linear_mps / speed;
      vy *= max_linear_mps / speed;
    }
    const double c = std::cos(measured.yaw_rad()),
                 s = std::sin(measured.yaw_rad());
    return {vx * c + vy * s, -vx * s + vy * c,
            std::clamp(omega, -max_angular_radps, max_angular_radps)};
  }
};

}  // namespace autonomous
