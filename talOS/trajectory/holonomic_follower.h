#pragma once

// Follows a Trajectory with a holonomic (swerve) drive: the sampled field
// velocities as feedforward plus PID on the field-frame x and y errors and on
// the heading error (the shorter way round). See README.md.

#include "talOS/geometry/se2.h"
#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {

// One PID axis. Output units are the controlled rate's (m/s for x and y,
// rad/s for heading); the gains convert from the error's units (m or rad).
struct PidGains {
  double kp = 0;  // output per unit error
  double ki = 0;  // output per unit error-second
  double kd = 0;  // output per unit error per second
  // |ki * integral| is held at or below this, in output units (anti-windup);
  // 0 turns the integral term off.
  double integral_limit = 0;
};

struct FollowerConfig {
  PidGains translation{.kp = 2.0};  // applied to x and y alike
  PidGains heading{.kp = 3.0};
  double max_linear_mps = 4.0;  // the command's speed is scaled down to this
  double max_angular_radps = 6.0;
  double position_tolerance_m = 0.05;
  double heading_tolerance_rad = 0.05;
};

// Planar chassis velocity with the fields of a robot's chassis target
// message (the 2026 drivetrain::ChassisTarget), so `as<M>()` builds one
// without this library depending on the robot.
struct ChassisSpeeds {
  double vx_mps = 0;
  double vy_mps = 0;
  double omega_radps = 0;

  template <class M>
  M as() const {
    return M{vx_mps, vy_mps, omega_radps};
  }
};

enum class SpeedsFrame {
  kField,  // x downfield, y left (the trajectory's frame)
  kRobot,  // x forward, y left of the measured heading (drivetrain targets)
};

struct FollowerOutput {
  ChassisSpeeds speeds;        // zero when !valid
  TrajectorySample reference;  // the trajectory's state at the step's time
  double x_error_m = 0;        // reference minus measured, field frame
  double y_error_m = 0;
  double heading_error_rad = 0;  // wrapped to (-pi, pi]
  // The step's time has reached the end of the trajectory and the measured
  // pose is within both tolerances of its final pose.
  bool finished = false;
  // False when the time, period or measured pose is not finite: the speeds
  // are zero and the controller's state is left unchanged.
  bool valid = false;
};

class HolonomicFollower {
 public:
  // Throws std::invalid_argument naming the field when a gain, limit or
  // tolerance is negative or not finite, or a maximum speed is zero.
  explicit HolonomicFollower(const FollowerConfig& config = {});

  // Clears the integrators and derivative history. Call when starting a
  // trajectory or after the drive was disabled.
  void reset();

  // One control step at `t_s` along `trajectory`, with the pose measured now
  // and `dt_s` since the previous step (integral and derivative terms use it;
  // 0 skips them). Deterministic; no allocation.
  FollowerOutput step(const Trajectory& trajectory, double t_s,
                      const geometry::SE2<double>& measured, double dt_s,
                      SpeedsFrame frame);

  // The same, for any pose with x_m(), y_m() and yaw_rad() accessors
  // (geometry::Pose2d from talOS/msgs).
  template <class Pose>
  FollowerOutput step_pose(const Trajectory& trajectory, double t_s,
                           const Pose& measured, double dt_s,
                           SpeedsFrame frame) {
    const geometry::SE2<double>::Vector2 translation(measured.x_m(),
                                                     measured.y_m());
    return step(trajectory, t_s,
                geometry::SE2<double>(measured.yaw_rad(), translation), dt_s,
                frame);
  }

  const FollowerConfig& config() const { return config_; }

 private:
  struct Axis {
    double integral = 0;
    double previous_error = 0;
    bool has_previous = false;

    double update(const PidGains& gains, double error, double dt_s);
  };

  FollowerConfig config_;
  Axis x_;
  Axis y_;
  Axis heading_;
};

}  // namespace talos::trajectory
