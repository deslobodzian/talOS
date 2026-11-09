#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "drivetrain/drive_message_generated.h"
#include "drivetrain/geometry.h"
#include "drivetrain/swerve.h"
#include "drivetrain/topics.h"
#include "geometry/geometry_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace drivetrain {

// Swerve drive: turns the selected chassis target lane into a drive velocity
// and a steer position per module, and reports, every tick, the chassis
// motion and module positions the modules measure together with the robot
// heading, each with a validity flag.
class Drivetrain {
 public:
  explicit Drivetrain(talos::Node& node)
      : node_{node},
        link_{node.extension<talos::HardwareLink>()},
        geometry_{LoadSwerveGeometry(node)},
        kinematics_{geometry_},
        gyro_{node, "drivetrain_imu"},
        // WPILib/ideal sims report a permanently zero Pigeon yaw, so --sim
        // defaults to module-integrated heading. robot_physics_sim.toml sets
        // this false to use the physics gateway's modeled Pigeon (and drift).
        integrate_heading_{
            node.param<bool>("integrate_heading", node.simulation())},
        max_linear_accel_mps2_{
            node.param<double>("max_linear_accel_mps2", 0.0)},
        max_angular_accel_radps2_{
            node.param<double>("max_angular_accel_radps2", 0.0)},
        status_{node.subscribe<bridge::BridgeStatus>(bridge::topics::kStatus)},
        target_{node},
        state_{node.publish<DrivetrainState>(topics::kState)} {
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      drive_[i] = talos::Motor{node, geometry_.modules[i].drive};
      steer_[i] = talos::Motor{node, geometry_.modules[i].steer};
      encoder_[i] = talos::CANcoder{node, geometry_.modules[i].encoder};
    }
  }

  void update() {
    // Acquisition times, not packet assembly time, describe the measurements.
    // Timing validity never changes the existing connectivity/target drive
    // gate.
    const auto now = node_.now();
    MeasurementTimes module_times{now};
    const bool reset =
        status_.fresh() &&
        status_.value().gateway_device_reset_count() != reset_count_;
    if (status_.fresh())
      reset_count_ = status_.value().gateway_device_reset_count();
    unsigned live_mask = 0;
    std::array<ModuleState, kModuleCount> measured{};
    const double period_s =
        std::chrono::duration<double>{node_.period()}.count();
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      const bool drive_valid = drive_[i].connected();
      if (drive_valid) {
        const double raw = drive_[i].position();
        // A reset can precede its slower status counter. A physically
        // impossible wheel step is also a new encoder frame, never travel.
        const double candidate = raw + drive_offset_[i];
        const double limit = geometry_.modules[i].max_drive_rps * period_s * 4;
        if (drive_seen_[i] &&
            (reset || !drive_valid_[i] ||
             std::abs(candidate - drive_position_[i]) > limit))
          drive_offset_[i] = drive_position_[i] - raw;
        drive_position_[i] = raw + drive_offset_[i];
        drive_seen_[i] = true;
      }
      drive_valid_[i] = drive_valid;
      if (!drive_valid || !steer_[i].connected() || !encoder_[i].connected())
        continue;
      live_mask |= 1U << i;
      measured[i] = {drive_[i].velocity(), steer_[i].position()};
      module_times.add(drive_[i].position_time());
      module_times.add(drive_[i].velocity_time());
      module_times.add(steer_[i].position_time());
      modules_[i] = geometry::SwerveModulePosition{
          WheelTravel(drive_position_[i], geometry_.modules[i].wheel_radius_m),
          ToRadians(measured[i].steer_rot)};
    }
    const bool connected = live_mask == 15;
    const bool can_drive = kinematics_.UsableSubset(live_mask);
    const bool modules_valid = connected && module_times.valid();

    ChassisSpeeds speeds{};
    const ChassisTarget* target = target_.get();
    const bool valid_target =
        can_drive && target != nullptr && std::isfinite(target->vx_mps()) &&
        std::isfinite(target->vy_mps()) && std::isfinite(target->omega_radps());
    if (!valid_target) commanded_speeds_ = {0.0, 0.0, 0.0};
    if (can_drive) {
      speeds = kinematics_.ToChassisSpeeds(measured, live_mask);
      if (valid_target) {
        const double dt_s =
            std::chrono::duration<double>{node_.period()}.count();
        double dx = target->vx_mps() - commanded_speeds_.vx_mps;
        double dy = target->vy_mps() - commanded_speeds_.vy_mps;
        if (std::isfinite(max_linear_accel_mps2_) &&
            max_linear_accel_mps2_ > 0.0 && dt_s > 0.0) {
          const double delta = std::hypot(dx, dy);
          const double limit = max_linear_accel_mps2_ * dt_s;
          if (delta > limit) {
            dx *= limit / delta;
            dy *= limit / delta;
          }
        }
        double domega = target->omega_radps() - commanded_speeds_.omega_radps;
        if (std::isfinite(max_angular_accel_radps2_) &&
            max_angular_accel_radps2_ > 0.0 && dt_s > 0.0) {
          const double limit = max_angular_accel_radps2_ * dt_s;
          domega = std::clamp(domega, -limit, limit);
        }
        commanded_speeds_.vx_mps += dx;
        commanded_speeds_.vy_mps += dy;
        commanded_speeds_.omega_radps += domega;
        const auto setpoints = kinematics_.ToModuleStates(
            commanded_speeds_, measured, dt_s, live_mask);
        for (std::size_t i = 0; setpoints && i < kModuleCount; ++i) {
          if ((live_mask & (1U << i)) == 0) continue;
          drive_[i].set_velocity((*setpoints)[i].drive_rps);
          steer_[i].set_position((*setpoints)[i].steer_rot);
        }
      }
    }  // anything not set goes out neutral

    // yaw_rad_ keeps the last valid heading while the source is missing.
    bool yaw_valid = false;
    if (integrate_heading_) {
      if (modules_valid) {
        const double dt_s =
            std::chrono::duration<double>{node_.period()}.count();
        {
          const auto sample_ns =
              static_cast<std::uint64_t>(module_times.oldest_ns);
          if (sample_ns > last_integrated_sample_ns_) {
            const double elapsed_s =
                last_integrated_sample_ns_ == 0
                    ? dt_s
                    : std::min(static_cast<double>(sample_ns -
                                                   last_integrated_sample_ns_) *
                                   1e-9,
                               dt_s * 4.0);
            yaw_rad_ += speeds.omega_radps * elapsed_s;
            last_integrated_sample_ns_ = sample_ns;
          }
        }
        yaw_valid = true;
      } else {
        last_integrated_sample_ns_ = 0;
        // Preserve the ideal-sim heading accumulated during clock convergence,
        // but never integrate measurements rejected for their signal ages.
        if (connected && !link_.sample_time())
          yaw_rad_ += speeds.omega_radps * period_s;
      }
    } else {
      MeasurementTimes yaw_times{now};
      yaw_times.add(gyro_.yaw_time());
      bool valid = yaw_times.valid();
      // One timestamp serves both products. Prefer the module solution when
      // independently usable yaw is too far from any module acquisition.
      if (valid && modules_valid) {
        valid = std::max(module_times.newest_ns, yaw_times.newest_ns) -
                    std::min(module_times.oldest_ns, yaw_times.oldest_ns) <=
                kMaxMeasurementSkewNs;
      }
      if (valid) {
        const double raw = ToRadians(gyro_.yaw());
        if (gyro_seen_ &&
            (reset || !gyro_valid_ ||
             std::abs(std::remainder(raw + yaw_offset_rad_ - yaw_rad_,
                                     kTwoPi)) > 25.0 * period_s))
          yaw_offset_rad_ = yaw_rad_ - raw;
        yaw_rad_ = raw + yaw_offset_rad_;
        gyro_seen_ = true;
        yaw_valid = true;
        if (!modules_valid || yaw_times.oldest_ns < module_times.oldest_ns)
          module_times.oldest_ns = yaw_times.oldest_ns;
      }
      gyro_valid_ = valid;
    }

    const auto sample_ns =
        modules_valid || yaw_valid
            ? static_cast<std::uint64_t>(module_times.oldest_ns)
            : 0;
    state_.send(DrivetrainState{
        sample_ns, talos::robot_enabled(node_), modules_valid, yaw_valid,
        speeds.vx_mps, speeds.vy_mps, speeds.omega_radps, yaw_rad_, modules_[0],
        modules_[1], modules_[2], modules_[3]});
  }

 private:
  // Ten control ticks of age and four of skew (at the usual 5 ms period)
  // bound the error from publishing asynchronous values without extrapolation.
  static constexpr std::int64_t kMaxMeasurementAgeNs = 50'000'000;
  static constexpr std::int64_t kMaxMeasurementSkewNs = 20'000'000;

  struct MeasurementTimes {
    explicit MeasurementTimes(talos::Time now)
        : now_ns{now.nanos()}, oldest_ns{now_ns} {}

    void add(std::optional<talos::Time> time) {
      if (!time || time->nanos() <= 0 || time->nanos() > now_ns ||
          now_ns - time->nanos() > kMaxMeasurementAgeNs) {
        known = false;
        return;
      }
      oldest_ns = std::min(oldest_ns, time->nanos());
      newest_ns = std::max(newest_ns, time->nanos());
    }

    bool valid() const {
      return known && newest_ns > 0 &&
             newest_ns - oldest_ns <= kMaxMeasurementSkewNs;
    }

    std::int64_t now_ns;
    std::int64_t oldest_ns;
    std::int64_t newest_ns{0};
    bool known{true};
  };

  talos::Node& node_;
  talos::HardwareLink& link_;
  SwerveGeometry geometry_;
  SwerveKinematics kinematics_;
  std::array<talos::Motor, kModuleCount> drive_;
  std::array<talos::Motor, kModuleCount> steer_;
  std::array<talos::CANcoder, kModuleCount> encoder_;
  std::array<double, kModuleCount> drive_position_{}, drive_offset_{};
  std::array<bool, kModuleCount> drive_seen_{}, drive_valid_{};
  std::uint32_t reset_count_{0};
  bool gyro_seen_{false}, gyro_valid_{false};
  double yaw_offset_rad_{0};
  talos::Gyro gyro_;
  bool integrate_heading_;
  double max_linear_accel_mps2_;
  double max_angular_accel_radps2_;
  talos::Subscription<bridge::BridgeStatus> status_;
  ChassisSpeeds commanded_speeds_{0.0, 0.0, 0.0};
  double yaw_rad_ = 0;
  std::uint64_t last_integrated_sample_ns_{0};
  std::array<geometry::SwerveModulePosition, kModuleCount> modules_{};
  talos::Target<ChassisTarget> target_;
  talos::Publisher<DrivetrainState> state_;
};

}  // namespace drivetrain
