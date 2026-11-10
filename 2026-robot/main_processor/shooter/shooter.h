#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace shooter {

// Shared-memory slot contract for /shooter/state: docs/DATA_FLOW.md.
static_assert(sizeof(ShooterState) == 32,
              "ShooterState size changed; update docs/DATA_FLOW.md");

// Spins the flywheel to the selected target lane. Readiness requires
// consecutive ticks in the entry band of the clamped demand, then tolerates the
// wider exit band until the target changes or actuation is no longer allowed.
class Shooter {
 public:
  static constexpr double kTargetEpsilonRps = 1e-3;
  explicit Shooter(talos::Node& node)
      : node_{node},
        flywheel_{node, "flywheel"},
        beam_break_{node, "beam_break"},
        target_{node},
        state_{node.publish<ShooterState>(topics::kState)},
        max_velocity_rps_{MaxVelocity(node)},
        entry_band_rps_{node.param<double>("at_speed_entry_rps", 2.0)},
        exit_band_frac_{node.param<double>("at_speed_exit_frac", 0.1)},
        entry_tick_count_{node.param<int>("at_speed_entry_tick_count", 3)} {
    if (!std::isfinite(entry_band_rps_) || entry_band_rps_ <= 0)
      throw std::invalid_argument("at_speed_entry_rps must be finite and > 0");
    if (!std::isfinite(exit_band_frac_) || exit_band_frac_ <= 0 ||
        exit_band_frac_ > 1)
      throw std::invalid_argument("at_speed_exit_frac must be in (0, 1]");
    if (entry_tick_count_ <= 0)
      throw std::invalid_argument("at_speed_entry_tick_count must be > 0");
  }

  void update() {
    const bool enabled = talos::robot_enabled(node_);
    const ShooterTarget* t = target_.get();
    const double requested = t ? t->target_velocity_rps() : 0.0;
    const bool spin =
        enabled && t && flywheel_.connected() && std::isfinite(requested);
    const double goal =
        spin ? std::clamp(requested, -max_velocity_rps_, max_velocity_rps_)
             : 0.0;
    if (spin)
      flywheel_.set_velocity(goal);  // otherwise the flush sends neutral

    const double velocity = flywheel_.velocity();
    if (!spin || std::abs(goal) <= kTargetEpsilonRps) {
      at_speed_ = false;
      entry_ticks_ = 0;
      readiness_target_rps_ = 0.0;
    } else {
      if (std::abs(requested - readiness_target_rps_) > kTargetEpsilonRps) {
        at_speed_ = false;
        entry_ticks_ = 0;
        readiness_target_rps_ = requested;
      }
      const double error_rps = std::abs(velocity - goal);
      const double exit_band_rps =
          std::max(entry_band_rps_, exit_band_frac_ * std::abs(goal));
      if (at_speed_) {
        if (error_rps > exit_band_rps) {
          at_speed_ = false;
          entry_ticks_ = 0;
        }
      } else if (error_rps <= entry_band_rps_) {
        at_speed_ = ++entry_ticks_ >= entry_tick_count_;
      } else {
        entry_ticks_ = 0;
      }
    }
    state_.send(ShooterState{static_cast<std::uint64_t>(node_.now().nanos()),
                             velocity, goal, at_speed_, beam_break_.get(),
                             enabled});
  }

 private:
  static double MaxVelocity(talos::Node& node) {
    const auto& config = node.extension<talos::HardwareLink>().config();
    const auto* owned = config.GetDevices(std::string{node.name()});
    if (owned) {
      for (const auto& motor : config.hardware.motors) {
        if (motor.name == "flywheel" &&
            std::find(owned->motors.begin(), owned->motors.end(), motor.id) !=
                owned->motors.end())
          return motor.max_velocity_rps;
      }
    }
    throw std::invalid_argument("shooter has no flywheel motor");
  }

  talos::Node& node_;
  talos::Motor flywheel_;
  talos::DigitalInput beam_break_;
  talos::Target<ShooterTarget> target_;
  talos::Publisher<ShooterState> state_;
  double max_velocity_rps_;
  double entry_band_rps_;
  double exit_band_frac_;
  int entry_tick_count_;
  double readiness_target_rps_{0.0};
  int entry_ticks_{0};
  bool at_speed_{false};
};

}  // namespace shooter
