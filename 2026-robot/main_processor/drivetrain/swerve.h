#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "talOS/hardware/config.h"
#include "talOS/hardware/messages.h"

namespace talos::drive {

struct SwerveModuleGeometry {
  const char* name;
  uint16_t drive_id, steer_id, encoder_id;
  double x_m, y_m;
  double wheel_radius_m;
  double drive_max_velocity_rps{20.0};
};

inline constexpr std::array<SwerveModuleGeometry, 4> kSwerveModules{{
    {"back_left", 1, 3, 2, -0.30, 0.30, 0.0508, 20.0},
    {"back_right", 4, 6, 5, -0.30, -0.30, 0.0508, 20.0},
    {"front_left", 8, 10, 9, 0.30, 0.30, 0.0508, 20.0},
    {"front_right", 11, 13, 12, 0.30, -0.30, 0.0508, 20.0},
}};

inline void ValidateSwerve(const hardware::Config& config) {
  hardware::Validate(config);
  for (const auto& module : kSwerveModules) {
    if (!std::isfinite(module.x_m) || !std::isfinite(module.y_m) ||
        !std::isfinite(module.wheel_radius_m) || module.wheel_radius_m <= 0)
      throw std::invalid_argument("invalid swerve geometry");
    bool drive = false, steer = false;
    for (const auto& motor : config.motors) {
      drive |= motor.id == module.drive_id;
      steer |= motor.id == module.steer_id && motor.continuous_wrap;
    }
    if (!drive || !steer)
      throw std::invalid_argument(
          "swerve module mapping or steering wrap is missing");
  }
}

// Computes robot-relative inverse kinematics and takes the shorter steering
// path. Device-side ratios make position/velocity units mechanism rotations.
inline hardware::Command SwerveCommand(const hardware::Config& config,
                                       const hardware::State& state, double vx,
                                       double vy, double omega) {
  hardware::Command out{};
  if (config.motors.size() > hardware::kMaxMotors ||
      state.motor_count > hardware::kMaxMotors)
    return out;
  out.config_id = state.config_id;
  out.boot_id = state.boot_id;
  out.epoch = state.epoch;
  out.observed_time_us = state.sample_time_us;
  out.count = static_cast<uint16_t>(config.motors.size());
  for (std::size_t i = 0; i < config.motors.size(); ++i)
    out.motors[i].id = config.motors[i].id;
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega))
    return out;
  constexpr double tau = 2 * std::numbers::pi;
  std::array<double, 4> speeds{}, angles{};
  double scale = 1;
  for (std::size_t i = 0; i < kSwerveModules.size(); ++i) {
    const auto& module = kSwerveModules[i];
    const double x = vx - omega * module.y_m;
    const double y = vy + omega * module.x_m;
    speeds[i] = std::hypot(x, y) / (tau * module.wheel_radius_m);
    if (!std::isfinite(speeds[i])) return out;
    angles[i] = std::atan2(y, x) / tau;
    for (const auto& motor : config.motors) {
      if (motor.id == module.drive_id && speeds[i] > motor.max_velocity_rps)
        scale = std::min(scale, motor.max_velocity_rps / speeds[i]);
    }
  }
  for (std::size_t i = 0; i < kSwerveModules.size(); ++i) {
    const auto& module = kSwerveModules[i];
    double current{};
    bool found = false;
    for (std::size_t j = 0; j < state.motor_count; ++j) {
      if (state.motors[j].id == module.steer_id && state.motors[j].valid) {
        current = state.motors[j].position_rot;
        found = true;
      }
    }
    if (!found) return hardware::Command{};
    double delta = std::remainder(angles[i] - current, 1.0);
    if (std::abs(delta) > 0.25) {
      delta -= std::copysign(0.5, delta);
      speeds[i] = -speeds[i];
    }
    if (std::abs(speeds[i]) < 1e-6) delta = 0;
    for (std::size_t j = 0; j < config.motors.size(); ++j) {
      auto& r = out.motors[j];
      if (r.id == module.drive_id) {
        r.mode = hardware::Mode::kVelocity;
        r.demand = speeds[i] * scale;
      }
      if (r.id == module.steer_id) {
        r.mode = hardware::Mode::kPosition;
        r.demand = current + delta;
      }
    }
  }
  return out;
}

inline void ComputeForwardKinematics(const hardware::State& state,
                                     double& vx, double& vy, double& omega) {
  constexpr double tau = 2 * std::numbers::pi;
  double sum_vx = 0, sum_vy = 0, sum_omega = 0;
  int valid_modules = 0;

  for (const auto& module : kSwerveModules) {
    double steer_angle_rad = 0;
    double drive_rps = 0;
    bool has_steer = false, has_drive = false;
    for (std::size_t j = 0; j < state.motor_count; ++j) {
      if (state.motors[j].id == module.steer_id && state.motors[j].valid) {
        steer_angle_rad = state.motors[j].position_rot * tau;
        has_steer = true;
      }
      if (state.motors[j].id == module.drive_id && state.motors[j].valid) {
        drive_rps = state.motors[j].velocity_rps;
        has_drive = true;
      }
    }
    if (has_steer && has_drive) {
      double v_wheel = drive_rps * tau * module.wheel_radius_m;
      double v_ix = v_wheel * std::cos(steer_angle_rad);
      double v_iy = v_wheel * std::sin(steer_angle_rad);
      sum_vx += v_ix;
      sum_vy += v_iy;
      double r2 = module.x_m * module.x_m + module.y_m * module.y_m;
      if (r2 > 1e-6) {
        sum_omega += (-module.y_m * v_ix + module.x_m * v_iy) / r2;
      }
      ++valid_modules;
    }
  }
  if (valid_modules > 0) {
    vx = sum_vx / valid_modules;
    vy = sum_vy / valid_modules;
    omega = sum_omega / valid_modules;
  } else {
    vx = 0;
    vy = 0;
    omega = 0;
  }
}
}  // namespace talos::drive
