#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

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
        // A simulated Pigeon reads a valid, permanently zero yaw, so under
        // --sim the heading integrates the rotation the modules measure. Off
        // on a robot: an integrated heading drifts with no absolute reference.
        integrate_heading_{
            node.param<bool>("integrate_heading", node.simulation())},
        max_linear_accel_mps2_{
            node.param<double>("max_linear_accel_mps2", 0.0)},
        max_angular_accel_radps2_{
            node.param<double>("max_angular_accel_radps2", 0.0)},
        target_{node},
        state_{node.publish<DrivetrainState>(topics::kState)} {
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      drive_[i] = talos::Motor{node, geometry_.modules[i].drive};
      steer_[i] = talos::Motor{node, geometry_.modules[i].steer};
    }
  }

  void update() {
    // Node-clock time the hardware sample this tick reads was taken; nullopt
    // until the bridge's clock mapping converges or while hardware is stale.
    // A measurement without a time is not usable, so it clears both flags;
    // driving does not need it.
    const std::optional<talos::Time> sample_time = link_.sample_time();
    // Every module or none: until all eight motors report, nothing is driven
    // and the module fields are not believed.
    bool connected = true;
    std::array<ModuleState, kModuleCount> measured{};
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      if (!drive_[i].connected() || !steer_[i].connected()) {
        connected = false;
        continue;
      }
      measured[i] = {drive_[i].velocity(), steer_[i].position()};
    }
    const bool modules_valid = connected && sample_time.has_value();

    ChassisSpeeds speeds{};
    const ChassisTarget* target = target_.get();
    const bool valid_target =
        connected && target != nullptr && std::isfinite(target->vx_mps()) &&
        std::isfinite(target->vy_mps()) && std::isfinite(target->omega_radps());
    if (!valid_target) commanded_speeds_ = {0.0, 0.0, 0.0};
    if (connected) {
      for (std::size_t i = 0; i < kModuleCount; ++i) {
        modules_[i] = geometry::SwerveModulePosition{
            WheelTravel(drive_[i].position(),
                        geometry_.modules[i].wheel_radius_m)
                .value(),
            ToRadians(measured[i].steer_rot).value()};
      }
      speeds = kinematics_.ToChassisSpeeds(measured);
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
        const auto setpoints =
            kinematics_.ToModuleStates(commanded_speeds_, measured, dt_s);
        for (std::size_t i = 0; setpoints && i < kModuleCount; ++i) {
          drive_[i].set_velocity((*setpoints)[i].drive_rps);
          steer_[i].set_position((*setpoints)[i].steer_rot);
        }
      }
    }  // anything not set goes out neutral

    // yaw_rad_ keeps the last valid heading while the source is missing.
    bool yaw_valid = false;
    if (integrate_heading_) {
      if (connected) {
        yaw_rad_ += speeds.omega_radps *
                    std::chrono::duration<double>{node_.period()}.count();
        yaw_valid = sample_time.has_value();
      }
    } else if (sample_time.has_value() && gyro_.connected()) {
      yaw_rad_ = ToRadians(gyro_.yaw()).value();
      yaw_valid = true;
    }

    state_.send(DrivetrainState{
        sample_time ? static_cast<std::uint64_t>(sample_time->nanos()) : 0,
        talos::robot_enabled(node_), modules_valid, yaw_valid, speeds.vx_mps,
        speeds.vy_mps, speeds.omega_radps, yaw_rad_, modules_[0], modules_[1],
        modules_[2], modules_[3]});
  }

 private:
  talos::Node& node_;
  talos::HardwareLink& link_;
  SwerveGeometry geometry_;
  SwerveKinematics kinematics_;
  std::array<talos::Motor, kModuleCount> drive_;
  std::array<talos::Motor, kModuleCount> steer_;
  talos::Gyro gyro_;
  bool integrate_heading_;
  double max_linear_accel_mps2_;
  double max_angular_accel_radps2_;
  ChassisSpeeds commanded_speeds_{0.0, 0.0, 0.0};
  double yaw_rad_ = 0;
  std::array<geometry::SwerveModulePosition, kModuleCount> modules_{};
  talos::Target<ChassisTarget> target_;
  talos::Publisher<DrivetrainState> state_;
};

}  // namespace drivetrain
