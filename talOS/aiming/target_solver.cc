#include "talOS/aiming/target_solver.h"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include "talOS/geometry/so2.h"

namespace talos::aiming {
namespace {

using talos::geometry::SE2;
using talos::geometry::SO2;

constexpr double kTau = 6.28318530717958647692;
// Below this the shooter is on its aim point and no direction exists.
constexpr double kMinDistance_m = 1e-6;

double Wrap(double angle_rad) { return SO2<double>(angle_rad).angle(); }

bool Finite(const Eigen::Vector2d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y());
}

struct TurretChoice {
  double turret_rad = 0.0;
  bool reachable = false;
};

// Among needed_rad + 2 pi k, the one inside [min, max] nearest measured.
// Just past a limit, or when none is inside, a limit is held, unreachable.
TurretChoice ChooseTurret(double needed_rad, double measured_rad,
                          const TurretTravel& travel) {
  const double nearest = measured_rad + Wrap(needed_rad - measured_rad);
  if (nearest >= travel.min_rad && nearest <= travel.max_rad) {
    return {nearest, true};
  }
  // Seam hysteresis: a needed angle that has only just crossed a limit holds
  // that limit rather than jumping to the far side of the travel.
  if (nearest > travel.max_rad &&
      nearest - travel.max_rad <= travel.seam_hysteresis_rad) {
    return {travel.max_rad, false};
  }
  if (nearest < travel.min_rad &&
      travel.min_rad - nearest <= travel.seam_hysteresis_rad) {
    return {travel.min_rad, false};
  }
  // Candidates further from `measured` are further from `nearest` too, so
  // the first one shifted into the travel is the closest in it.
  if (nearest > travel.max_rad) {
    const double shifted =
        nearest - kTau * std::ceil((nearest - travel.max_rad) / kTau);
    if (shifted >= travel.min_rad) return {shifted, true};
  } else {
    const double shifted =
        nearest + kTau * std::ceil((travel.min_rad - nearest) / kTau);
    if (shifted <= travel.max_rad) return {shifted, true};
  }
  // Unreachable: hold the limit the turret is nearer, so noise in a needed
  // angle in the dead zone cannot swing it from one limit to the other. A
  // tie (turret mid-travel) goes to the limit nearer the needed angle.
  const double turret_to_min_rad = std::abs(measured_rad - travel.min_rad);
  const double turret_to_max_rad = std::abs(measured_rad - travel.max_rad);
  if (turret_to_min_rad != turret_to_max_rad) {
    return {
        turret_to_min_rad < turret_to_max_rad ? travel.min_rad : travel.max_rad,
        false};
  }
  const double needed_to_min_rad = std::abs(Wrap(travel.min_rad - needed_rad));
  const double needed_to_max_rad = std::abs(Wrap(travel.max_rad - needed_rad));
  return {
      needed_to_min_rad <= needed_to_max_rad ? travel.min_rad : travel.max_rad,
      false};
}

void Require(bool ok, const char* key, const char* rule) {
  if (!ok) {
    throw std::invalid_argument(std::string("aiming: ") + key + " " + rule);
  }
}

}  // namespace

ShotTable::ShotTable(std::span<const ShotRow> rows) : table_(Build(rows)) {}

ShotTable::Table ShotTable::Build(std::span<const ShotRow> rows) {
  if (rows.size() > kCapacity) {
    throw std::invalid_argument("shot table: " + std::to_string(rows.size()) +
                                " rows, capacity " + std::to_string(kCapacity));
  }
  std::array<Table::Row, kCapacity> converted{};
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const ShotRow& row = rows[i];
    if (!(row.time_of_flight_s > 0.0)) {
      throw std::invalid_argument("shot table: row " + std::to_string(i) +
                                  ": time_of_flight_s must be > 0");
    }
    converted[i] = {row.distance_m,
                    {row.flywheel_rps, row.hood_rad, row.time_of_flight_s}};
  }
  return Table(std::span<const Table::Row>(converted.data(), rows.size()));
}

Shot ShotTable::at(double distance_m) const {
  const Table::Lookup lookup = table_.at(distance_m);
  return {lookup.values[0], lookup.values[1], lookup.values[2],
          lookup.in_range};
}

FieldTwist ToFieldTwist(const SE2<double>& field_to_robot,
                        const ::geometry::Twist2d& chassis_velocity) {
  const Eigen::Vector2d field_mps = field_to_robot.rotation().act(
      Eigen::Vector2d(chassis_velocity.vx_mps(), chassis_velocity.vy_mps()));
  return {field_mps.x(), field_mps.y(), chassis_velocity.omega_radps()};
}

TargetSolver::TargetSolver(const SolverConfig& config, const ShotTable& table)
    : config_(config),
      table_(table),
      mount_m_(config.mount.x_m, config.mount.y_m) {
  Require(std::isfinite(config.mount.x_m), "mount.x_m", "must be finite");
  Require(std::isfinite(config.mount.y_m), "mount.y_m", "must be finite");
  Require(std::isfinite(config.mount.yaw_rad), "mount.yaw_rad",
          "must be finite");
  if (config.turret) {
    Require(!std::isnan(config.turret->min_rad), "turret.min_rad",
            "must be a number");
    Require(!std::isnan(config.turret->max_rad), "turret.max_rad",
            "must be a number");
    Require(config.turret->min_rad < config.turret->max_rad, "turret.min_rad",
            "must be less than turret.max_rad");
    Require(std::isfinite(config.turret->seam_hysteresis_rad) &&
                config.turret->seam_hysteresis_rad >= 0.0,
            "turret.seam_hysteresis_rad", "must be finite and >= 0");
  }
  Require(config.max_iteration_count >= 2, "max_iteration_count",
          "must be at least 2");
  Require(config.time_of_flight_tolerance_s > 0.0 &&
              std::isfinite(config.time_of_flight_tolerance_s),
          "time_of_flight_tolerance_s", "must be finite and > 0");
  const Eigen::Vector2d in_shot_frame =
      SO2<double>(-config.mount.yaw_rad).act(mount_m_);
  along_m_ = in_shot_frame.x();
  lateral_m_ = in_shot_frame.y();
}

TargetSolver::Aim TargetSolver::AimAt(const SE2<double>& field_to_robot,
                                      const Eigen::Vector2d& point_m) const {
  Aim aim;
  const Eigen::Vector2d& robot_m = field_to_robot.translation();
  if (config_.turret) {
    // The turret axis stays where the robot's current pose puts it.
    aim.robot_heading_rad = Wrap(field_to_robot.angle());
    aim.shooter_m = field_to_robot.act(mount_m_);
    const Eigen::Vector2d to_point_m = point_m - aim.shooter_m;
    aim.distance_m = to_point_m.norm();
    if (!(aim.distance_m > kMinDistance_m)) return aim;
    aim.shot_heading_rad = std::atan2(to_point_m.y(), to_point_m.x());
    aim.valid = true;
    return aim;
  }
  // Fixed shooter: find the shot heading phi for which the point lies on the
  // ray from the exit point along phi. In the frame rotated by phi the exit
  // point is (along, lateral), so the point's lateral coordinate there must
  // equal `lateral`: |d| sin(alpha - phi) = lateral.
  const Eigen::Vector2d to_point_m = point_m - robot_m;
  const double range_m = to_point_m.norm();
  if (!(range_m > std::abs(lateral_m_) + kMinDistance_m)) return aim;
  const double bearing_rad = std::atan2(to_point_m.y(), to_point_m.x());
  const double shot_rad = bearing_rad - std::asin(lateral_m_ / range_m);
  aim.distance_m =
      std::sqrt(range_m * range_m - lateral_m_ * lateral_m_) - along_m_;
  if (!(aim.distance_m > kMinDistance_m)) return aim;
  aim.shot_heading_rad = Wrap(shot_rad);
  aim.robot_heading_rad = Wrap(shot_rad - config_.mount.yaw_rad);
  aim.shooter_m = robot_m + SO2<double>(aim.robot_heading_rad).act(mount_m_);
  aim.valid = true;
  return aim;
}

Solution TargetSolver::SolveWithFieldVelocity(
    const SE2<double>& field_to_robot, const FieldTwist& field_velocity,
    const Eigen::Vector2d& field_target_m, double measured_turret_rad) const {
  Solution out;
  const Eigen::Vector2d velocity_mps(field_velocity.vx_mps,
                                     field_velocity.vy_mps);
  const double omega_radps = field_velocity.omega_radps;
  if (!std::isfinite(field_to_robot.angle()) ||
      !Finite(field_to_robot.translation()) || !Finite(velocity_mps) ||
      !std::isfinite(omega_radps) || !Finite(field_target_m) ||
      (config_.turret && !std::isfinite(measured_turret_rad))) {
    return out;
  }
  const Eigen::Vector2d& robot_m = field_to_robot.translation();

  // Fixed-point iteration on time of flight t: aim a standing shot at
  // V(t) = target - shooter_velocity * t, look up t' = tof(|V - shooter|),
  // repeat until t' and t agree.
  Eigen::Vector2d virtual_target_m = field_target_m;
  double previous_time_of_flight_s = 0.0;
  Aim aim;
  Shot shot;
  int iteration_count = 0;
  bool converged = false;
  for (int k = 1; k <= config_.max_iteration_count; ++k) {
    iteration_count = k;
    aim = AimAt(field_to_robot, virtual_target_m);
    if (!aim.valid) return out;
    shot = table_.at(aim.distance_m);
    if (k > 1 && std::abs(shot.time_of_flight_s - previous_time_of_flight_s) <=
                     config_.time_of_flight_tolerance_s) {
      converged = true;
      break;
    }
    if (k == config_.max_iteration_count) break;
    // The exit point's field velocity: the robot's, plus omega x r for the
    // exit point's offset r from the robot origin (the tangential lead).
    const Eigen::Vector2d offset_m = aim.shooter_m - robot_m;
    const Eigen::Vector2d shooter_velocity_mps =
        velocity_mps +
        omega_radps * Eigen::Vector2d(-offset_m.y(), offset_m.x());
    previous_time_of_flight_s = shot.time_of_flight_s;
    virtual_target_m =
        field_target_m - shooter_velocity_mps * previous_time_of_flight_s;
  }

  out.valid = true;
  out.converged = converged;
  out.in_range = shot.in_range;
  out.iteration_count = iteration_count;
  out.distance_m = aim.distance_m;
  out.shot_heading_rad = aim.shot_heading_rad;
  out.robot_heading_rad = aim.robot_heading_rad;
  out.flywheel_rps = shot.flywheel_rps;
  out.hood_rad = shot.hood_rad;
  out.time_of_flight_s = shot.time_of_flight_s;
  out.shooter_x_m = aim.shooter_m.x();
  out.shooter_y_m = aim.shooter_m.y();
  out.virtual_target_x_m = virtual_target_m.x();
  out.virtual_target_y_m = virtual_target_m.y();
  if (config_.turret) {
    const double needed_rad = Wrap(
        aim.shot_heading_rad - field_to_robot.angle() - config_.mount.yaw_rad);
    const TurretChoice choice =
        ChooseTurret(needed_rad, measured_turret_rad, *config_.turret);
    out.turret_rad = choice.turret_rad;
    out.turret_reachable = choice.reachable;
  } else {
    out.turret_reachable = true;
  }
  return out;
}

Solution TargetSolver::SolveStatic(const SE2<double>& field_to_robot,
                                   const Eigen::Vector2d& field_target_m,
                                   double measured_turret_rad) const {
  return SolveWithFieldVelocity(field_to_robot, FieldTwist{}, field_target_m,
                                measured_turret_rad);
}

Solution TargetSolver::SolveWithChassisVelocity(
    const SE2<double>& field_to_robot,
    const ::geometry::Twist2d& chassis_velocity,
    const Eigen::Vector2d& field_target_m, double measured_turret_rad) const {
  return SolveWithFieldVelocity(field_to_robot,
                                ToFieldTwist(field_to_robot, chassis_velocity),
                                field_target_m, measured_turret_rad);
}

}  // namespace talos::aiming
