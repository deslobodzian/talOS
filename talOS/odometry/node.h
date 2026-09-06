#pragma once

#include <cmath>
#include <cstdint>

#include "talOS/drivetrain/drive_message_generated.h"
#include "talOS/events/handles.h"
#include "talOS/odometry/odometry_message_generated.h"
#include "talOS/odometry/packet.h"
#include "talOS/shooter/shooter_message_generated.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace talos::odometry {

template <typename Loop>
class OdometryNode {
 public:
  explicit OdometryNode(Loop& loop, double initial_x = 0.0,
                        double initial_y = 0.0, double initial_yaw_rot = 0.0)
      : x_m_{initial_x}, y_m_{initial_y}, yaw_rot_{initial_yaw_rot} {
    event::watch<talos::drive::DrivetrainState,
                 &OdometryNode::OnDrivetrainState>(
        loop, kDrivetrainStateTopic, this);
    event::watch<talos::shooter::ShooterState, &OdometryNode::OnShooterState>(
        loop, kShooterStateTopic, this);
    sender_ = event::make_sender<OdometryState>(loop, kOdometryTopic);
  }

  void Start() {}
  void Start(event::MonotonicTime) {}

  void Reset(double x = 0.0, double y = 0.0, double yaw_rot = 0.0) {
    x_m_ = x;
    y_m_ = y;
    yaw_rot_ = yaw_rot;
    have_last_ts_ = false;
    last_ts_ns_ = 0;
  }

  double x() const { return x_m_; }
  double y() const { return y_m_; }
  double yaw_rot() const { return yaw_rot_; }
  double vx_mps() const { return vx_mps_; }
  double vy_mps() const { return vy_mps_; }
  double omega_radps() const { return omega_radps_; }
  double flywheel_rps() const { return flywheel_rps_; }
  bool beam_broken() const { return beam_broken_; }
  const OdometryState& state() const { return current_state_; }

 private:
  void OnDrivetrainState(const event::Context&,
                         const talos::drive::DrivetrainState& state) {
    double dt = 0.0;
    if (have_last_ts_) {
      if (state.timestamp_ns() >= last_ts_ns_) {
        dt = static_cast<double>(state.timestamp_ns() - last_ts_ns_) * 1e-9;
      }
    } else {
      have_last_ts_ = true;
    }
    last_ts_ns_ = state.timestamp_ns();

    const double theta = state.yaw_rot() * 2.0 * M_PI;
    const double dx = (state.vx_mps() * std::cos(theta) -
                       state.vy_mps() * std::sin(theta)) *
                      dt;
    const double dy = (state.vx_mps() * std::sin(theta) +
                       state.vy_mps() * std::cos(theta)) *
                      dt;
    x_m_ += dx;
    y_m_ += dy;
    yaw_rot_ = state.yaw_rot();
    vx_mps_ = state.vx_mps();
    vy_mps_ = state.vy_mps();
    omega_radps_ = state.omega_radps();

    current_state_ = OdometryState(
        state.timestamp_ns(),
        x_m_,
        y_m_,
        yaw_rot_,
        vx_mps_,
        vy_mps_,
        omega_radps_,
        flywheel_rps_,
        beam_broken_);
    sender_.send(current_state_);
  }

  void OnShooterState(const event::Context&,
                      const talos::shooter::ShooterState& state) {
    flywheel_rps_ = state.flywheel_velocity_rps();
    beam_broken_ = state.beam_broken();
    current_state_ = OdometryState(
        current_state_.timestamp_ns(),
        x_m_,
        y_m_,
        yaw_rot_,
        vx_mps_,
        vy_mps_,
        omega_radps_,
        flywheel_rps_,
        beam_broken_);
  }

  double x_m_{0.0};
  double y_m_{0.0};
  double yaw_rot_{0.0};
  double vx_mps_{0.0};
  double vy_mps_{0.0};
  double omega_radps_{0.0};
  double flywheel_rps_{0.0};
  bool beam_broken_{false};

  uint64_t last_ts_ns_{0};
  bool have_last_ts_{false};

  OdometryState current_state_{};
  event::Sender<Loop, OdometryState> sender_;
};

}  // namespace talos::odometry
