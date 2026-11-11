#pragma once

// Aiming a shooter at a field point, standing or moving. Frames, signs and
// the maths are in talOS/aiming/README.md; in brief:
//
// - Field and robot frames are those of talOS/msgs/geometry.fbs: x forward,
//   y left, angles CCW+ in radians; the robot pose is field_to_robot.
// - The shooter (a fixed shooter's exit point, or a turret's axis) sits at
//   ShooterMount (x_m, y_m) in the robot frame and points along mount
//   yaw_rad (a turret: at turret angle 0).
// - A shot is planar: the ball leaves the shooter with the robot-relative
//   horizontal velocity of a standing shot at the commanded distance (that
//   distance over the table's time of flight), plus the shooter point's
//   field velocity, and keeps it until it lands.
//
// Every solve is deterministic, allocation-free, and returns no NaN.

#include <Eigen/Dense>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>

#include "geometry/geometry_generated.h"
#include "talOS/aiming/interpolating_table.h"
#include "talOS/geometry/se2.h"

namespace talos::aiming {

// One measured standing shot: from distance_m (shooter to target, horizontal)
// it scores at flywheel_rps and hood_rad, and the ball takes
// time_of_flight_s from release to the target.
struct ShotRow {
  double distance_m = 0.0;
  double flywheel_rps = 0.0;
  double hood_rad = 0.0;
  double time_of_flight_s = 0.0;
};

// A table lookup. Outside the table the end row's values are returned and
// in_range is false.
struct Shot {
  double flywheel_rps = 0.0;
  double hood_rad = 0.0;
  double time_of_flight_s = 0.0;
  bool in_range = false;
};

// distance_m -> (flywheel_rps, hood_rad, time_of_flight_s), interpolated
// linearly. Rows as InterpolatingTable requires, every time_of_flight_s > 0.
class ShotTable {
 public:
  static constexpr std::size_t kCapacity = 32;

  // Throws std::invalid_argument naming the row and the problem.
  explicit ShotTable(std::span<const ShotRow> rows);

  Shot at(double distance_m) const;
  std::size_t size() const { return table_.size(); }
  double min_distance_m() const { return table_.min_key(); }
  double max_distance_m() const { return table_.max_key(); }

 private:
  using Table = InterpolatingTable<3, kCapacity>;
  static Table Build(std::span<const ShotRow> rows);

  Table table_;
};

// Where the shooter is on the robot, in the robot frame.
struct ShooterMount {
  double x_m = 0.0;
  double y_m = 0.0;
  // Direction a fixed shooter fires, or a turret fires at turret angle 0,
  // CCW+ from robot forward.
  double yaw_rad = 0.0;
};

// A turret's travel, in turret angle (CCW+ from the mount yaw). The limits
// may span more than a turn; infinite limits mean a continuous turret.
struct TurretTravel {
  double min_rad = -std::numeric_limits<double>::infinity();
  double max_rad = std::numeric_limits<double>::infinity();
  // A needed angle at most this far past a limit holds that limit
  // (unreachable) instead of swinging the turret to the equivalent angle on
  // the far side, so noise on the seam of a full-turn travel cannot sweep
  // it across its travel every tick. >= 0.
  double seam_hysteresis_rad = 0.05;
};

struct SolverConfig {
  ShooterMount mount;
  // Present: the shooter is on a turret, aimed by turret angle while the
  // robot keeps its heading. Absent: the shooter is fixed to the robot and
  // aimed by the robot's heading.
  std::optional<TurretTravel> turret;
  // Bound on time-of-flight iterations; the first is the standing solve.
  // At least 2: convergence is two successive iterations agreeing.
  int max_iteration_count = 20;
  // Two successive times of flight this close have converged.
  double time_of_flight_tolerance_s = 1e-3;
};

struct Solution {
  // False when an input is not finite, or no aim exists (the target is
  // inside the shooter's lateral offset, behind its exit point, or on it).
  // Every number below is then 0.
  bool valid = false;
  // The time of flight settled within tolerance. False: the numbers are the
  // last iteration's, close to the target but not compensated exactly.
  bool converged = false;
  // distance_m is inside the shot table.
  bool in_range = false;
  // The turret angle is inside its travel (always true for a fixed shooter).
  // False: turret_rad is a held travel limit (TurretTravel, README).
  bool turret_reachable = false;
  int iteration_count = 0;

  // Horizontal distance from the shooter to the virtual target; the table
  // is looked up here.
  double distance_m = 0.0;
  // Field direction the shooter must point, (-pi, pi].
  double shot_heading_rad = 0.0;
  // Field yaw the robot must hold, (-pi, pi]. Fixed shooter: the solved
  // heading (the robot turns in place about its origin). Turret: the
  // current yaw.
  double robot_heading_rad = 0.0;
  // Turret: the turret angle to command, inside the travel when reachable,
  // nearest the measured turret angle among equivalent angles. Fixed: 0.
  double turret_rad = 0.0;
  double flywheel_rps = 0.0;
  double hood_rad = 0.0;
  double time_of_flight_s = 0.0;
  // Field position of the shooter with the robot at robot_heading_rad.
  double shooter_x_m = 0.0;
  double shooter_y_m = 0.0;
  // The point a standing shot is aimed at so that the moving shot lands on
  // the target: target - shooter_velocity * time_of_flight_s.
  double virtual_target_x_m = 0.0;
  double virtual_target_y_m = 0.0;
};

// Robot velocity in the field frame: vx_mps, vy_mps along the field axes,
// omega_radps CCW+. Its own type so a robot-relative geometry.Twist2d
// (DrivetrainState, PoseEstimate.velocity) cannot be passed where a field
// velocity is meant: there is no conversion from Twist2d.
struct FieldTwist {
  double vx_mps = 0.0;
  double vy_mps = 0.0;
  double omega_radps = 0.0;
};

// Rotates a robot-relative chassis velocity into the field frame by the
// pose's yaw; omega is unchanged.
FieldTwist ToFieldTwist(const talos::geometry::SE2<double>& field_to_robot,
                        const ::geometry::Twist2d& chassis_velocity);

class TargetSolver {
 public:
  // Throws std::invalid_argument naming the bad key.
  TargetSolver(const SolverConfig& config, const ShotTable& table);

  // field_to_robot: robot pose. chassis_velocity: robot-relative velocity
  // as the drivetrain and pose estimator publish it (geometry.Twist2d: x
  // forward, y left, omega CCW+). field_target_m: the target's field
  // position. measured_turret_rad: the turret's current angle, used to pick
  // among equivalent turret angles (ignored without a turret).
  Solution SolveWithChassisVelocity(
      const talos::geometry::SE2<double>& field_to_robot,
      const ::geometry::Twist2d& chassis_velocity,
      const Eigen::Vector2d& field_target_m,
      double measured_turret_rad = 0.0) const;

  // The same, with the velocity already in the field frame.
  Solution SolveWithFieldVelocity(
      const talos::geometry::SE2<double>& field_to_robot,
      const FieldTwist& field_velocity, const Eigen::Vector2d& field_target_m,
      double measured_turret_rad = 0.0) const;

  // The robot standing still.
  Solution SolveStatic(const talos::geometry::SE2<double>& field_to_robot,
                       const Eigen::Vector2d& field_target_m,
                       double measured_turret_rad = 0.0) const;

  const SolverConfig& config() const { return config_; }
  const ShotTable& table() const { return table_; }

 private:
  struct Aim {
    bool valid = false;
    double distance_m = 0.0;
    double shot_heading_rad = 0.0;
    double robot_heading_rad = 0.0;
    Eigen::Vector2d shooter_m = Eigen::Vector2d::Zero();
  };

  Aim AimAt(const talos::geometry::SE2<double>& field_to_robot,
            const Eigen::Vector2d& point_m) const;

  SolverConfig config_;
  ShotTable table_;
  Eigen::Vector2d mount_m_;
  // The mount offset in the shot frame (x along the shot): a fixed
  // shooter's exit point is along_m ahead of and lateral_m left of the line
  // through the robot origin parallel to the shot.
  double along_m_ = 0.0;
  double lateral_m_ = 0.0;
};

}  // namespace talos::aiming
