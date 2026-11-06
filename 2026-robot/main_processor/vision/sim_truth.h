#pragma once

#include <cmath>
#include <cstdint>

#include "drivetrain/drive_message_generated.h"
#include "talOS/geometry/se2.h"
#include "talOS/vision/field_sim/sim_camera.h"

namespace vision {

// Integrates the simulator's measured chassis motion, never a localization
// estimate. Consume each drivetrain sample through an on_message subscription.
class SimTruth {
 public:
  SimTruth(double x_m, double y_m, double yaw_rad)
      : pose_(yaw_rad, Eigen::Vector2d{x_m, y_m}), start_yaw_(yaw_rad) {}

  void Observe(const drivetrain::DrivetrainState& state) {
    const auto stamp = state.sample_time_ns();
    if (have_sample_ && stamp <= previous_ns_) return;
    if (!state.modules_valid() || !std::isfinite(state.vx_mps()) ||
        !std::isfinite(state.vy_mps()) || !std::isfinite(state.omega_radps())) {
      have_sample_ = false;
      return;
    }
    if (have_sample_) {
      const double dt = static_cast<double>(stamp - previous_ns_) * 1e-9;
      if (dt > 0 && dt <= 0.1) {
        pose_ = pose_ * talos::geometry::SE2<double>::Exp(Eigen::Vector3d{
                            state.omega_radps() * dt, state.vx_mps() * dt,
                            state.vy_mps() * dt});
      }
    }
    // Simulated drivetrain yaw starts at zero and integrates valid modules.
    // The configured field start heading is therefore an additive offset.
    if (state.yaw_valid() && std::isfinite(state.yaw_rad())) {
      pose_ = talos::geometry::SE2<double>{start_yaw_ + state.yaw_rad(),
                                           pose_.translation()};
    }
    previous_ns_ = stamp;
    have_sample_ = true;
  }

  talos::geometry::SE3<double> pose() const {
    return talos::vision::field_sim::RobotPoseOnField(
        pose_.translation().x(), pose_.translation().y(), pose_.angle());
  }

 private:
  talos::geometry::SE2<double> pose_;
  double start_yaw_;
  std::uint64_t previous_ns_ = 0;
  bool have_sample_ = false;
};

}  // namespace vision
