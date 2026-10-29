#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

#include "drivetrain/drive_message_generated.h"
#include "drivetrain/topics.h"
#include "odometry/odometry_message_generated.h"
#include "odometry/topics.h"
#include "talOS/node/node.h"

namespace odometry {

// Dead reckoning: integrates each drivetrain state's field-rotated chassis
// velocity into a field pose, and publishes it as that state arrives.
class Odometry {
 public:
  explicit Odometry(talos::Node& node)
      : pose_{node.publish<OdometryState>(topics::kState)} {
    node.subscribe<drivetrain::DrivetrainState>(
        drivetrain::topics::kState,
        [this](const drivetrain::DrivetrainState& s) { Integrate(s); });
  }

 private:
  void Integrate(const drivetrain::DrivetrainState& state) {
    // The drivetrain's own clock, so a delayed delivery does not stretch the
    // step it covers.
    const double dt_s =
        have_last_ && state.timestamp_ns() >= last_ns_
            ? static_cast<double>(state.timestamp_ns() - last_ns_) * 1e-9
            : 0.0;
    have_last_ = true;
    last_ns_ = state.timestamp_ns();

    const double theta = state.yaw_rot() * 2.0 * std::numbers::pi;
    x_m_ +=
        (state.vx_mps() * std::cos(theta) - state.vy_mps() * std::sin(theta)) *
        dt_s;
    y_m_ +=
        (state.vx_mps() * std::sin(theta) + state.vy_mps() * std::cos(theta)) *
        dt_s;
    pose_.send(OdometryState{state.timestamp_ns(), x_m_, y_m_, state.yaw_rot(),
                             state.vx_mps(), state.vy_mps(),
                             state.omega_radps()});
  }

  double x_m_ = 0;
  double y_m_ = 0;
  std::uint64_t last_ns_ = 0;
  bool have_last_ = false;
  talos::Publisher<OdometryState> pose_;
};

}  // namespace odometry
