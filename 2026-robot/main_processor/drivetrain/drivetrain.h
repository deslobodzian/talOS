#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <numbers>

#include "drivetrain/drive_message_generated.h"
#include "drivetrain/geometry.h"
#include "drivetrain/swerve.h"
#include "drivetrain/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace drivetrain {

// Swerve drive: turns the arbiter's chassis target into a drive velocity and
// a steer position per module, and reports the chassis motion the modules
// measure together with the robot heading.
class Drivetrain {
 public:
  explicit Drivetrain(talos::Node& node)
      : node_{node},
        geometry_{LoadSwerveGeometry(node)},
        gyro_{node, "drivetrain_imu"},
        // A simulated Pigeon reads a valid, permanently zero yaw, so under
        // --sim the heading integrates the rotation the modules measure. Off
        // on a robot: an integrated heading drifts with no absolute reference.
        integrate_heading_{
            node.param<bool>("integrate_heading", node.simulation())},
        target_{node.subscribe<ChassisTarget>(kTargetTopic)},
        state_{node.publish<DrivetrainState>(kStateTopic)} {
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      drive_[i] = talos::Motor{node, geometry_.modules[i].drive};
      steer_[i] = talos::Motor{node, geometry_.modules[i].steer};
    }
  }

  void update() {
    std::array<ModuleState, kModuleCount> measured{};
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      // Every module or none: until all eight motors report, nothing is
      // driven and nothing is reported.
      if (!drive_[i].connected() || !steer_[i].connected()) return;
      measured[i] = {drive_[i].velocity(), steer_[i].position()};
    }
    if (target_.fresh() && target_->enabled()) {
      const auto setpoints = ToModuleStates(
          geometry_,
          {target_->vx_mps(), target_->vy_mps(), target_->omega_radps()},
          measured);
      for (std::size_t i = 0; setpoints && i < kModuleCount; ++i) {
        drive_[i].set_velocity((*setpoints)[i].drive_rps);
        steer_[i].set_position((*setpoints)[i].steer_rot);
      }
    }  // anything not set goes out neutral

    const ChassisSpeeds speeds = ToChassisSpeeds(geometry_, measured);
    // DrivetrainState carries the yaw rate in rotations/s, like the Pigeon.
    double yaw_rot = 0;
    double yaw_rate_rps = speeds.omega_radps / (2 * std::numbers::pi);
    if (integrate_heading_) {
      integrated_yaw_rot_ +=
          yaw_rate_rps * std::chrono::duration<double>{node_.period()}.count();
      yaw_rot = integrated_yaw_rot_;
    } else if (gyro_.connected()) {
      yaw_rot = gyro_.yaw();
      yaw_rate_rps = gyro_.yaw_rate();
    }
    state_.send(DrivetrainState{static_cast<std::uint64_t>(node_.now().nanos()),
                                talos::robot_enabled(node_), speeds.vx_mps,
                                speeds.vy_mps, speeds.omega_radps, yaw_rot,
                                yaw_rate_rps});
  }

 private:
  talos::Node& node_;
  SwerveGeometry geometry_;
  std::array<talos::Motor, kModuleCount> drive_;
  std::array<talos::Motor, kModuleCount> steer_;
  talos::Gyro gyro_;
  bool integrate_heading_;
  double integrated_yaw_rot_ = 0;
  talos::Subscription<ChassisTarget> target_;
  talos::Publisher<DrivetrainState> state_;
};

}  // namespace drivetrain
