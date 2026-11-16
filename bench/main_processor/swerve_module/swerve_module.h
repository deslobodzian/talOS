#pragma once

// The bench swerve module: a drive TalonFX, a steer TalonFX closing on a
// remote CANcoder, and that CANcoder. A DRIVE target asks a wheel speed and
// heading; geometry::OptimizeSwerveModule, the setpoint the 2026 drivetrain
// uses, turns it into the drive velocity and steer position.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "swerve_module/module_message_generated.h"
#include "swerve_module/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/geometry/swerve_module.h"
#include "talOS/node/node.h"

namespace swerve_module {
static_assert(sizeof(ModuleState) == 120);

class BenchSwerveModule {
 public:
  explicit BenchSwerveModule(talos::Node& node)
      : node_{node},
        drive_{node, "drive"},
        steer_{node, "steer"},
        encoder_{node, "encoder"},
        target_{node},
        state_{node.publish<ModuleState>(topics::kState)},
        max_drive_velocity_rps_{
            node.param<double>("max_drive_velocity_rps", 10.0)} {
    if (!std::isfinite(max_drive_velocity_rps_) || max_drive_velocity_rps_ <= 0)
      throw std::invalid_argument(
          "max_drive_velocity_rps must be finite and > 0");
  }

  void update() {
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    const bool enabled = talos::robot_enabled(node_);
    const auto* target = target_.get();
    auto mode = ModuleMode::NEUTRAL;
    double drive_rps = 0;
    double steer_rot = 0;
    bool reversed = false;
    if (enabled && target && target->mode() == ModuleMode::DRIVE &&
        drive_.connected() && steer_.connected() && encoder_.connected() &&
        std::isfinite(target->drive_velocity_rps()) &&
        std::isfinite(target->steer_angle_rot())) {
      const double asked =
          std::clamp(target->drive_velocity_rps(), -max_drive_velocity_rps_,
                     max_drive_velocity_rps_);
      const double measured_rot = steer_.position();
      const auto setpoint = talos::geometry::OptimizeSwerveModule(
          asked, kTwoPi * target->steer_angle_rot(), kTwoPi * measured_rot);
      drive_rps = setpoint.speed;
      steer_rot = measured_rot + setpoint.steer_delta_rad / kTwoPi;
      reversed = setpoint.reversed;
      drive_.set_velocity(drive_rps);
      steer_.set_position(steer_rot);
      mode = ModuleMode::DRIVE;
    }
    auto& link = node_.extension<talos::HardwareLink>();
    constexpr auto unknown = std::numeric_limits<std::uint64_t>::max();
    state_.send(ModuleState{
        static_cast<std::uint64_t>(node_.now().nanos()),
        link.fresh() ? static_cast<std::uint64_t>(link.age().count() / 1000)
                     : unknown,
        drive_.position(),
        drive_.velocity(),
        drive_.voltage(),
        drive_.current(),
        steer_.position(),
        steer_.velocity(),
        steer_.voltage(),
        steer_.current(),
        encoder_.position(),
        encoder_.velocity(),
        drive_rps,
        steer_rot,
        drive_.connected(),
        steer_.connected(),
        encoder_.connected(),
        enabled,
        mode,
        mode == ModuleMode::DRIVE && target_.source() == "teleop"
            ? TargetSource::TELEOP
            : TargetSource::NONE,
        reversed});
  }

 private:
  talos::Node& node_;
  talos::Motor drive_;
  talos::Motor steer_;
  talos::CANcoder encoder_;
  talos::Target<ModuleTarget> target_;
  talos::Publisher<ModuleState> state_;
  double max_drive_velocity_rps_;
};
}  // namespace swerve_module
