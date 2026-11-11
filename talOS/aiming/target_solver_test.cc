// Tests for talOS/aiming TargetSolver: standing solves against hand-computed
// geometry, moving solves against a brute-force simulation of the ball,
// turret travel, the non-convergence path, and the no-allocation rule.

#include "talOS/aiming/target_solver.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "talOS/geometry/se2.h"
#include "talOS/geometry/so2.h"
#include "talOS/node/allocation_probe.h"

namespace talos::aiming {
namespace {

using talos::geometry::SE2;
using talos::geometry::SO2;
using Twist = ::geometry::Twist2d;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTight = 1e-9;
// A compensated moving shot must land this close to the target.
constexpr double kLandingTolerance_m = 0.02;
// An uncompensated moving shot in these scenarios misses by more than this.
constexpr double kNaiveMiss_m = 0.3;

SE2<double> Pose(double x_m, double y_m, double yaw_rad) {
  return SE2<double>(yaw_rad, Eigen::Vector2d(x_m, y_m));
}

double Wrap(double angle_rad) { return SO2<double>(angle_rad).angle(); }

// A smooth, made-up table: time of flight grows faster than distance, as a
// real lobbed shot's does.
ShotTable RealisticTable() {
  std::array<ShotRow, 20> rows{};
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const double d_m = 0.5 + 0.5 * static_cast<double>(i);
    rows[i] = {d_m, 35.0 + 6.0 * d_m, 0.3 + 0.05 * d_m,
               0.35 + 0.12 * d_m + 0.004 * d_m * d_m};
  }
  return ShotTable(rows);
}

SolverConfig Fixed(double x_m, double y_m, double yaw_rad) {
  SolverConfig config;
  config.mount = {x_m, y_m, yaw_rad};
  return config;
}

SolverConfig Turret(double x_m, double y_m, double yaw_rad,
                    TurretTravel travel = {}) {
  SolverConfig config = Fixed(x_m, y_m, yaw_rad);
  config.turret = travel;
  return config;
}

// Brute force: what the actuators were told (turret angle or robot heading,
// and the commanded distance) is turned into a ball and stepped forward in
// 0.1 ms increments. The ball leaves the shooter's exit point with the
// standing shot's horizontal velocity for the commanded distance (that
// distance over the table's time of flight), plus the exit point's field
// velocity, and lands when that time of flight has elapsed. The exit
// point's velocity is not the solver's omega x r formula: it is the central
// difference of where the exit point is as the robot pose moves along the
// twist (translation + v t, yaw + omega t), so a sign error in the solver's
// tangential lead is not repeated here.
Eigen::Vector2d SimulateLanding(const SolverConfig& config,
                                const ShotTable& table,
                                const SE2<double>& pose_now,
                                const FieldTwist& field_velocity,
                                const Solution& solution) {
  const ShooterMount& mount = config.mount;
  const Eigen::Vector2d mount_m(mount.x_m, mount.y_m);
  const double yaw_rad =
      config.turret ? pose_now.angle() : solution.robot_heading_rad;
  const Eigen::Vector2d robot_velocity_mps(field_velocity.vx_mps,
                                           field_velocity.vy_mps);
  const auto exit_at = [&](double t_s) {
    const Eigen::Vector2d robot_m =
        pose_now.translation() + robot_velocity_mps * t_s;
    return Eigen::Vector2d(
        robot_m +
        SO2<double>(yaw_rad + field_velocity.omega_radps * t_s).act(mount_m));
  };
  constexpr double kDifference_s = 1e-6;
  const Eigen::Vector2d exit_m = exit_at(0.0);
  const Eigen::Vector2d exit_velocity_mps =
      (exit_at(kDifference_s) - exit_at(-kDifference_s)) /
      (2.0 * kDifference_s);
  const double heading_rad =
      yaw_rad + mount.yaw_rad + (config.turret ? solution.turret_rad : 0.0);
  const double flight_s = table.at(solution.distance_m).time_of_flight_s;
  const Eigen::Vector2d relative_mps =
      (solution.distance_m / flight_s) *
      Eigen::Vector2d(std::cos(heading_rad), std::sin(heading_rad));
  const Eigen::Vector2d ball_mps = relative_mps + exit_velocity_mps;

  constexpr double kStep_s = 1e-4;
  Eigen::Vector2d ball_m = exit_m;
  double elapsed_s = 0.0;
  while (elapsed_s + kStep_s <= flight_s) {
    ball_m += ball_mps * kStep_s;
    elapsed_s += kStep_s;
  }
  ball_m += ball_mps * (flight_s - elapsed_s);
  return ball_m;
}

// --- standing shots ----------------------------------------------------------

// a centred fixed shooter aims along the line to the target.
TEST(TargetSolverStatic, CentredFixedShooter) {
  const ShotTable table = RealisticTable();
  const TargetSolver solver(Fixed(0.0, 0.0, 0.0), table);
  const Solution s =
      solver.SolveStatic(Pose(0.0, 0.0, 0.0), Eigen::Vector2d(3.0, 4.0));
  ASSERT_TRUE(s.valid);
  EXPECT_TRUE(s.converged);
  EXPECT_TRUE(s.in_range);
  EXPECT_TRUE(s.turret_reachable);
  EXPECT_EQ(s.iteration_count, 2);
  EXPECT_NEAR(s.distance_m, 5.0, kTight);
  EXPECT_NEAR(s.shot_heading_rad, 0.92729521800161219, kTight);  // atan2(4,3)
  EXPECT_NEAR(s.robot_heading_rad, 0.92729521800161219, kTight);
  EXPECT_EQ(s.turret_rad, 0.0);
  EXPECT_NEAR(s.flywheel_rps, 65.0, kTight);
  EXPECT_NEAR(s.hood_rad, 0.55, kTight);
  EXPECT_NEAR(s.time_of_flight_s, 0.35 + 0.6 + 0.1, kTight);
  EXPECT_NEAR(s.virtual_target_x_m, 3.0, kTight);
  EXPECT_NEAR(s.virtual_target_y_m, 4.0, kTight);
}

// headings are reported in (-pi, pi] and stay continuous
// through the branch cut directly behind the robot.
TEST(TargetSolverStatic, HeadingWrapsAtPi) {
  const ShotTable table = RealisticTable();
  const TargetSolver solver(Fixed(0.0, 0.0, 0.0), table);
  const Solution behind =
      solver.SolveStatic(Pose(5.0, 0.0, 0.0), Eigen::Vector2d(2.0, 0.0));
  ASSERT_TRUE(behind.valid);
  EXPECT_NEAR(behind.shot_heading_rad, kPi, kTight);
  EXPECT_NEAR(behind.distance_m, 3.0, kTight);
  const Solution just_below =
      solver.SolveStatic(Pose(5.0, 0.0, 0.0), Eigen::Vector2d(2.0, -1e-6));
  EXPECT_NEAR(just_below.shot_heading_rad, -kPi + 1e-6 / 3.0, kTight);
  const Solution just_above =
      solver.SolveStatic(Pose(5.0, 0.0, 0.0), Eigen::Vector2d(2.0, 1e-6));
  EXPECT_NEAR(just_above.shot_heading_rad, kPi - 1e-6 / 3.0, kTight);
  // A turret on a robot facing 3.0 rad, target at field heading -3.0 rad:
  // the turret turns -6.0 + 2 pi = +0.2832 rad, not -6.0.
  const TargetSolver turret(Turret(0.0, 0.0, 0.0), table);
  const Eigen::Vector2d target(4.0 * std::cos(-3.0), 4.0 * std::sin(-3.0));
  const Solution t = turret.SolveStatic(Pose(0.0, 0.0, 3.0), target);
  ASSERT_TRUE(t.valid);
  EXPECT_NEAR(t.shot_heading_rad, -3.0, kTight);
  EXPECT_NEAR(t.robot_heading_rad, 3.0, kTight);
  EXPECT_NEAR(t.turret_rad, 2.0 * kPi - 6.0, kTight);
  EXPECT_TRUE(t.turret_reachable);
  // An unwrapped robot yaw (7 pi, facing -x) is accepted: target behind
  // the field origin side, turret straight ahead.
  const Solution wound =
      turret.SolveStatic(Pose(5.0, 0.0, 7.0 * kPi), Eigen::Vector2d(2.0, 0.0));
  ASSERT_TRUE(wound.valid);
  EXPECT_NEAR(std::abs(wound.robot_heading_rad), kPi, kTight);
  EXPECT_NEAR(wound.turret_rad, 0.0, kTight);
}

// a fixed shooter mounted off the robot's centreline is aimed
// so its own line of fire, not the robot's, passes through the target.
TEST(TargetSolverStatic, OffsetFixedShooter) {
  const ShotTable table = RealisticTable();
  // Exit point 0.5 m left of the robot origin. Target 4 m ahead: the robot
  // turns right by asin(0.5 / 4) and the shot is sqrt(4^2 - 0.5^2) long.
  const TargetSolver left(Fixed(0.0, 0.5, 0.0), table);
  const Solution s =
      left.SolveStatic(Pose(0.0, 0.0, 0.0), Eigen::Vector2d(4.0, 0.0));
  ASSERT_TRUE(s.valid);
  EXPECT_NEAR(s.robot_heading_rad, -0.1253278311680654, kTight);
  EXPECT_NEAR(s.shot_heading_rad, -0.1253278311680654, kTight);
  EXPECT_NEAR(s.distance_m, 3.968626966596886, kTight);
  EXPECT_NEAR(s.shooter_x_m, 0.0625, kTight);               // -0.5 sin(phi)
  EXPECT_NEAR(s.shooter_y_m, 0.49607837082461076, kTight);  // 0.5 cos(phi)
  // The ray from the exit point along the shot heading hits the target.
  EXPECT_NEAR(std::atan2(0.0 - s.shooter_y_m, 4.0 - s.shooter_x_m),
              s.shot_heading_rad, kTight);

  // Shooter on the back, firing backward (mount yaw pi), 0.3 m behind the
  // origin. Target 5 m up the field's +y: the robot faces -y and the exit
  // point is 0.3 m nearer the target.
  const TargetSolver rear(Fixed(-0.3, 0.0, kPi), table);
  const Solution r =
      rear.SolveStatic(Pose(1.0, 1.0, 0.0), Eigen::Vector2d(1.0, 6.0));
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.shot_heading_rad, kPi / 2.0, kTight);
  EXPECT_NEAR(r.robot_heading_rad, -kPi / 2.0, kTight);
  EXPECT_NEAR(r.distance_m, 4.7, kTight);
  EXPECT_NEAR(r.shooter_x_m, 1.0, kTight);
  EXPECT_NEAR(r.shooter_y_m, 1.3, kTight);
}

// a turret's axis is where the current pose puts it, and its
// angle is measured from the mount yaw.
TEST(TargetSolverStatic, OffsetTurret) {
  const ShotTable table = RealisticTable();
  // Axis 0.2 m behind the origin; robot facing +y, so the axis is at
  // (1, 0.8) and the target 4.2 m straight ahead.
  const TargetSolver behind(Turret(-0.2, 0.0, 0.0), table);
  const Solution s =
      behind.SolveStatic(Pose(1.0, 1.0, kPi / 2.0), Eigen::Vector2d(1.0, 5.0));
  ASSERT_TRUE(s.valid);
  EXPECT_NEAR(s.shooter_x_m, 1.0, kTight);
  EXPECT_NEAR(s.shooter_y_m, 0.8, kTight);
  EXPECT_NEAR(s.distance_m, 4.2, kTight);
  EXPECT_NEAR(s.shot_heading_rad, kPi / 2.0, kTight);
  EXPECT_NEAR(s.robot_heading_rad, kPi / 2.0, kTight);
  EXPECT_NEAR(s.turret_rad, 0.0, kTight);

  // Turret zero points left (mount yaw pi/2).
  const TargetSolver sideways(Turret(0.0, 0.0, kPi / 2.0), table);
  EXPECT_NEAR(sideways.SolveStatic(Pose(0.0, 0.0, 0.0), {0.0, 3.0}).turret_rad,
              0.0, kTight);
  EXPECT_NEAR(sideways.SolveStatic(Pose(0.0, 0.0, 0.0), {3.0, 0.0}).turret_rad,
              -kPi / 2.0, kTight);
}

// no aim exists -> valid is false and no number is NaN.
// REQ-DEV-17
TEST(TargetSolverStatic, InvalidInputsAndDegenerateGeometry) {
  const ShotTable table = RealisticTable();
  const TargetSolver centred(Fixed(0.0, 0.0, 0.0), table);
  const Solution on_robot =
      centred.SolveStatic(Pose(1.0, 1.0, 0.0), Eigen::Vector2d(1.0, 1.0));
  EXPECT_FALSE(on_robot.valid);
  EXPECT_EQ(on_robot.distance_m, 0.0);

  // Target closer than the exit point's lateral offset.
  const TargetSolver left(Fixed(0.0, 0.5, 0.0), table);
  EXPECT_FALSE(
      left.SolveStatic(Pose(0.0, 0.0, 0.0), Eigen::Vector2d(0.3, 0.0)).valid);
  // Target behind a far-forward exit point.
  const TargetSolver forward(Fixed(1.0, 0.0, 0.0), table);
  EXPECT_FALSE(
      forward.SolveStatic(Pose(0.0, 0.0, 0.0), Eigen::Vector2d(0.5, 0.0))
          .valid);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Solution bad_pose =
      centred.SolveStatic(Pose(nan, 0.0, 0.0), Eigen::Vector2d(3.0, 0.0));
  EXPECT_FALSE(bad_pose.valid);
  EXPECT_EQ(bad_pose.flywheel_rps, 0.0);
  EXPECT_FALSE(centred
                   .SolveWithFieldVelocity(Pose(0.0, 0.0, 0.0),
                                           FieldTwist{nan, 0.0, 0.0},
                                           Eigen::Vector2d(3.0, 0.0))
                   .valid);
  EXPECT_FALSE(
      centred.SolveStatic(Pose(0.0, 0.0, 0.0), Eigen::Vector2d(3.0, nan))
          .valid);
  // The measured turret angle matters only with a turret.
  EXPECT_TRUE(
      centred.SolveStatic(Pose(0.0, 0.0, 0.0), Eigen::Vector2d(3.0, 0.0), nan)
          .valid);
  const TargetSolver turret(Turret(0.0, 0.0, 0.0), table);
  EXPECT_FALSE(
      turret.SolveStatic(Pose(0.0, 0.0, 0.0), Eigen::Vector2d(3.0, 0.0), nan)
          .valid);
}

// a target beyond the table is solved but flagged out of range.
TEST(TargetSolverStatic, OutOfTableIsFlagged) {
  const ShotTable table = RealisticTable();  // 0.5 .. 10 m
  const TargetSolver solver(Fixed(0.0, 0.0, 0.0), table);
  const Solution far =
      solver.SolveStatic(Pose(0.0, 0.0, 0.0), Eigen::Vector2d(12.0, 0.0));
  ASSERT_TRUE(far.valid);
  EXPECT_FALSE(far.in_range);
  EXPECT_NEAR(far.distance_m, 12.0, kTight);
  EXPECT_NEAR(far.flywheel_rps, 95.0, kTight);  // the 10 m row
}

// bad configuration is refused, naming the key.
TEST(TargetSolver, RejectsBadConfig) {
  const ShotTable table = RealisticTable();
  SolverConfig one_iteration = Fixed(0.0, 0.0, 0.0);
  one_iteration.max_iteration_count = 1;
  EXPECT_THROW(TargetSolver(one_iteration, table), std::invalid_argument);
  SolverConfig no_tolerance = Fixed(0.0, 0.0, 0.0);
  no_tolerance.time_of_flight_tolerance_s = 0.0;
  EXPECT_THROW(TargetSolver(no_tolerance, table), std::invalid_argument);
  EXPECT_THROW(TargetSolver(Fixed(std::nan(""), 0.0, 0.0), table),
               std::invalid_argument);
  try {
    const TargetSolver solver(Turret(0.0, 0.0, 0.0, {1.0, -1.0}), table);
    FAIL() << "expected a throw";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("turret.min_rad"), std::string::npos);
  }
}

// A robot-relative geometry.Twist2d (what the drivetrain and pose estimator
// publish) cannot be passed as a field velocity: the frame mix-up does not
// compile.
template <typename Velocity>
concept FieldSolvable =
    requires(const TargetSolver& solver, const Velocity& velocity) {
      solver.SolveWithFieldVelocity(SE2<double>(), velocity, Eigen::Vector2d());
    };
static_assert(FieldSolvable<FieldTwist>);
static_assert(!FieldSolvable<Twist>);
static_assert(!std::is_constructible_v<FieldTwist, Twist>);

// A chassis velocity is rotated into the field frame by the pose's yaw, and
// the chassis entry point applies that rotation.
TEST(TargetSolver, ChassisVelocityIsRotatedIntoTheFieldFrame) {
  const FieldTwist field =
      ToFieldTwist(Pose(3.0, 2.0, kPi / 2.0), Twist(1.0, 0.5, 0.3));
  EXPECT_NEAR(field.vx_mps, -0.5, kTight);
  EXPECT_NEAR(field.vy_mps, 1.0, kTight);
  EXPECT_EQ(field.omega_radps, 0.3);

  // Robot facing +y driving forward at 2 m/s toward a target ahead: the
  // virtual target is pulled back along field +y, not along field +x.
  const TargetSolver solver(Fixed(0.0, 0.0, 0.0), RealisticTable());
  const Solution s = solver.SolveWithChassisVelocity(Pose(0.0, 0.0, kPi / 2.0),
                                                     Twist(2.0, 0.0, 0.0),
                                                     Eigen::Vector2d(0.0, 5.0));
  ASSERT_TRUE(s.converged);
  EXPECT_NEAR(s.virtual_target_x_m, 0.0, kTight);
  EXPECT_NEAR(s.virtual_target_y_m, 5.0 - 2.0 * s.time_of_flight_s, 2e-3);
}

// --- turret travel -----------------------------------------------------------

// among equivalent turret angles, the one inside the travel and
// nearest the measured angle; outside the travel, the nearest limit and
// turret_reachable false.
// REQ-DEV-18
TEST(TargetSolverTurret, TravelLimits) {
  const ShotTable table = RealisticTable();
  const Eigen::Vector2d at_minus_3(4.0 * std::cos(-3.0), 4.0 * std::sin(-3.0));
  const SE2<double> origin = Pose(0.0, 0.0, 0.0);

  // Travel past +-pi: both -3.0 and 2 pi - 3.0 are reachable; the one
  // nearer the measured angle wins.
  const TargetSolver wide(Turret(0.0, 0.0, 0.0, {-3.5, 3.5}), table);
  Solution s = wide.SolveStatic(origin, at_minus_3, 3.2);
  EXPECT_TRUE(s.turret_reachable);
  EXPECT_NEAR(s.turret_rad, 2.0 * kPi - 3.0, kTight);
  s = wide.SolveStatic(origin, at_minus_3, -2.9);
  EXPECT_TRUE(s.turret_reachable);
  EXPECT_NEAR(s.turret_rad, -3.0, kTight);
  s = wide.SolveStatic(origin, at_minus_3, 0.0);
  EXPECT_NEAR(s.turret_rad, -3.0, kTight);

  // Travel [0, 4]: -3.0 is out, 2 pi - 3.0 is in.
  const TargetSolver positive(Turret(0.0, 0.0, 0.0, {0.0, 4.0}), table);
  s = positive.SolveStatic(origin, at_minus_3, 0.0);
  EXPECT_TRUE(s.turret_reachable);
  EXPECT_NEAR(s.turret_rad, 2.0 * kPi - 3.0, kTight);

  // Travel [-pi/2, pi/2], target behind-left (needs ~2.976 rad): the +pi/2
  // limit is nearer than -pi/2.
  const TargetSolver front(Turret(0.0, 0.0, 0.0, {-kPi / 2.0, kPi / 2.0}),
                           table);
  s = front.SolveStatic(origin, Eigen::Vector2d(-3.0, 0.5), 0.0);
  ASSERT_TRUE(s.valid);
  EXPECT_FALSE(s.turret_reachable);
  EXPECT_EQ(s.turret_rad, kPi / 2.0);
  s = front.SolveStatic(origin, Eigen::Vector2d(-3.0, -0.5), 0.0);
  EXPECT_FALSE(s.turret_reachable);
  EXPECT_EQ(s.turret_rad, -kPi / 2.0);
  s = front.SolveStatic(origin, Eigen::Vector2d(3.0, 0.5), 0.0);
  EXPECT_TRUE(s.turret_reachable);
  EXPECT_NEAR(s.turret_rad, std::atan2(0.5, 3.0), kTight);
}

// An unreachable target holds the limit the turret is at, so noise that
// moves the needed angle across the middle of the dead zone cannot swing it
// to the other limit.
// REQ-DEV-18
TEST(TargetSolverTurret, UnreachableHoldsTheLimitNearestTheTurret) {
  const ShotTable table = RealisticTable();
  const SE2<double> origin = Pose(0.0, 0.0, 0.0);
  const TargetSolver front(Turret(0.0, 0.0, 0.0, {-kPi / 2.0, kPi / 2.0}),
                           table);
  // Directly behind, either side of the dead zone's middle, turret parked at
  // +pi/2: it stays there for both.
  for (const double y_m : {0.05, -0.05}) {
    SCOPED_TRACE(y_m);
    const Solution s =
        front.SolveStatic(origin, Eigen::Vector2d(-3.0, y_m), kPi / 2.0);
    ASSERT_TRUE(s.valid);
    EXPECT_FALSE(s.turret_reachable);
    EXPECT_EQ(s.turret_rad, kPi / 2.0);
  }
  const Solution at_min =
      front.SolveStatic(origin, Eigen::Vector2d(-3.0, 0.05), -kPi / 2.0);
  EXPECT_FALSE(at_min.turret_reachable);
  EXPECT_EQ(at_min.turret_rad, -kPi / 2.0);
}

// A full-turn travel: the needed angle jittering across the +-pi seam while
// the turret sits there holds the turret at the seam limit instead of
// sweeping it to the other end; past the hysteresis it does go around.
// REQ-DEV-18
TEST(TargetSolverTurret, SeamHysteresisStopsAFullTurnSweep) {
  const ShotTable table = RealisticTable();
  const SE2<double> origin = Pose(0.0, 0.0, 0.0);
  const TargetSolver full(Turret(0.0, 0.0, 0.0, {-kPi, kPi, 0.05}), table);
  double turret_rad = kPi - 0.01;
  for (int tick = 0; tick < 20; ++tick) {
    // The needed heading alternates pi - 0.005 and -pi + 0.005.
    const double needed_rad = (tick % 2 == 0) ? kPi - 0.005 : -kPi + 0.005;
    const Eigen::Vector2d target(4.0 * std::cos(needed_rad),
                                 4.0 * std::sin(needed_rad));
    const Solution s = full.SolveStatic(origin, target, turret_rad);
    ASSERT_TRUE(s.valid);
    EXPECT_GT(s.turret_rad, kPi - 0.02) << "tick " << tick;
    turret_rad = s.turret_rad;
  }
  // Well past the seam (0.2 rad > 0.05): the turret goes around.
  const Solution around = full.SolveStatic(
      origin,
      Eigen::Vector2d(4.0 * std::cos(-kPi + 0.2), 4.0 * std::sin(-kPi + 0.2)),
      kPi - 0.01);
  EXPECT_TRUE(around.turret_reachable);
  EXPECT_NEAR(around.turret_rad, -kPi + 0.2, kTight);
}

// --- moving shots ------------------------------------------------------------

struct MovingCase {
  const char* name;
  SolverConfig config;
  SE2<double> pose;
  FieldTwist field_velocity;
  Eigen::Vector2d target_m;
};

// shooting on the move lands on the target (the compensated
// shot hits where the naive standing solution misses), including the lead
// for the exit point's tangential velocity when the robot rotates.
// REQ-DEV-17
TEST(TargetSolverMoving, CompensatedShotLandsWhereNaiveShotMisses) {
  const ShotTable table = RealisticTable();
  const std::array<MovingCase, 4> cases = {{
      {"turret, offset, translating and rotating", Turret(-0.15, 0.1, 0.0),
       Pose(2.0, 1.0, 0.4), FieldTwist{1.5, -2.0, 1.2},
       Eigen::Vector2d(6.5, 4.0)},
      {"fixed rear shooter, offset, translating and rotating",
       Fixed(0.25, -0.2, kPi), Pose(3.0, 5.0, -2.5),
       FieldTwist{-1.5, 0.8, -0.8}, Eigen::Vector2d(8.0, 4.1)},
      {"fixed centred shooter, driving at the target", Fixed(0.0, 0.0, 0.0),
       Pose(1.0, 1.0, 0.0), FieldTwist{2.0, 2.0, 0.0},
       Eigen::Vector2d(6.0, 6.0)},
      {"turret far off centre, spinning in place (tangential lead only)",
       Turret(0.0, 0.6, 0.0), Pose(4.0, 4.0, 1.0), FieldTwist{0.0, 0.0, 4.0},
       Eigen::Vector2d(9.0, 2.0)},
  }};
  for (const MovingCase& c : cases) {
    SCOPED_TRACE(c.name);
    const TargetSolver solver(c.config, table);
    const Solution moving =
        solver.SolveWithFieldVelocity(c.pose, c.field_velocity, c.target_m);
    ASSERT_TRUE(moving.valid);
    EXPECT_TRUE(moving.converged);
    EXPECT_TRUE(moving.in_range);
    EXPECT_LT(moving.iteration_count, solver.config().max_iteration_count);

    const Eigen::Vector2d hit_m =
        SimulateLanding(c.config, table, c.pose, c.field_velocity, moving);
    EXPECT_LT((hit_m - c.target_m).norm(), kLandingTolerance_m)
        << "landed at " << hit_m.transpose();

    const Solution naive = solver.SolveStatic(c.pose, c.target_m);
    ASSERT_TRUE(naive.valid);
    const Eigen::Vector2d naive_hit_m =
        SimulateLanding(c.config, table, c.pose, c.field_velocity, naive);
    EXPECT_GT((naive_hit_m - c.target_m).norm(), kNaiveMiss_m);
  }
}

// the iteration is bounded, and when it does not settle the
// result says so (converged false, iteration_count at the bound) without
// NaN, so readiness refuses the shot.
// REQ-DEV-17
TEST(TargetSolverMoving, NonConvergenceIsBoundedAndFlagged) {
  // Time of flight jumps from 0.1 s to 2.0 s over one metre: at 2 m/s
  // toward the target, the virtual target leaps between 3.3 m ahead and
  // 0.5 m behind, a two-cycle that never settles.
  const std::array<ShotRow, 2> rows = {{
      {1.0, 40.0, 0.2, 0.1},
      {2.0, 50.0, 0.3, 2.0},
  }};
  const ShotTable steep(rows);
  SolverConfig config = Turret(0.0, 0.0, 0.0);
  config.max_iteration_count = 7;
  const TargetSolver solver(config, steep);
  const Solution s = solver.SolveWithFieldVelocity(Pose(0.0, 0.0, 0.0),
                                                   FieldTwist{2.0, 0.0, 0.0},
                                                   Eigen::Vector2d(3.5, 0.0));
  ASSERT_TRUE(s.valid);
  EXPECT_FALSE(s.converged);
  EXPECT_EQ(s.iteration_count, 7);
  for (const double value :
       {s.distance_m, s.shot_heading_rad, s.turret_rad, s.flywheel_rps,
        s.hood_rad, s.time_of_flight_s, s.virtual_target_x_m}) {
    EXPECT_TRUE(std::isfinite(value));
  }
  // The same table converges standing still.
  const Solution standing =
      solver.SolveStatic(Pose(0.0, 0.0, 0.0), Eigen::Vector2d(1.5, 0.0));
  EXPECT_TRUE(standing.converged);
  EXPECT_NEAR(standing.time_of_flight_s, 1.05, kTight);
}

// --- allocation --------------------------------------------------------------

// Solving runs inside a control node's update(), so it allocates nothing
// (RULES.md 3).
// REQ-DEV-17
TEST(TargetSolverAllocation, SolveDoesNotAllocate) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  const ShotTable table = RealisticTable();
  const TargetSolver fixed(Fixed(0.25, -0.2, kPi), table);
  const TargetSolver turret(Turret(-0.15, 0.1, 0.0, {-3.5, 3.5}), table);
  double sink_rps = 0.0;
  talos::testing::AllocationProbe probe;
  probe.start();
  for (int i = 0; i < 1000; ++i) {
    const double t = 0.001 * i;
    const SE2<double> pose = Pose(2.0 + t, 1.0 - t, Wrap(0.4 + 3.0 * t));
    const Twist chassis(1.5, -2.0, 1.2);
    sink_rps +=
        fixed.SolveWithChassisVelocity(pose, chassis, {6.5, 4.0}).flywheel_rps;
    sink_rps += turret.SolveWithChassisVelocity(pose, chassis, {6.5, 4.0}, 0.5)
                    .flywheel_rps;
  }
  const std::uint64_t allocations = probe.stop();
  EXPECT_EQ(allocations, 0u) << probe.sites();
  EXPECT_GT(sink_rps, 0.0);
}

}  // namespace
}  // namespace talos::aiming
