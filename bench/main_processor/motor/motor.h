#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "bridge/bridge_status_generated.h"
#include "motor/motor_message_generated.h"
#include "motor/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace motor {
class BenchMotor {
 public:
  explicit BenchMotor(talos::Node& node)
      : node_{node},
        motor_{node, "kraken"},
        target_{node},
        state_{node.publish<MotorState>(topics::kState)},
        status_{node.subscribe<bridge::BridgeStatus>("/hw/status")},
        max_voltage_v_{node.param<double>("max_bench_voltage_v", 2.0)} {
    if (!std::isfinite(max_voltage_v_) || max_voltage_v_ <= 0)
      throw std::invalid_argument("max_bench_voltage_v must be finite and > 0");
  }

  void update() {
    const bool enabled = talos::robot_enabled(node_);
    const auto* target = target_.get();
    auto mode = ControlMode::NEUTRAL;
    if (enabled && target && motor_.connected()) {
      if (target->mode() == ControlMode::VOLTAGE &&
          std::isfinite(target->voltage_v()) &&
          std::isfinite(target->voltage_frac())) {
        const double volts =
            target->voltage_v() +
            std::clamp(target->voltage_frac(), -1.0, 1.0) * max_voltage_v_;
        motor_.set_voltage(std::clamp(volts, -max_voltage_v_, max_voltage_v_));
        mode = ControlMode::VOLTAGE;
      } else if (target->mode() == ControlMode::VELOCITY &&
                 std::isfinite(target->velocity_rps())) {
        motor_.set_velocity(target->velocity_rps());
        mode = ControlMode::VELOCITY;
      }
    }
    auto& link = node_.extension<talos::HardwareLink>();
    const auto position_time = motor_.position_time();
    const auto velocity_time = motor_.velocity_time();
    constexpr auto unknown = std::numeric_limits<std::uint64_t>::max();
    const auto age = [&](const auto& time) -> std::uint64_t {
      return time ? static_cast<std::uint64_t>((node_.now() - *time).count() /
                                               1000)
                  : unknown;
    };
    const bool fault_valid = status_.fresh() && status_->health_valid();
    state_.send(MotorState{
        static_cast<std::uint64_t>(node_.now().nanos()), motor_.position(),
        motor_.velocity(), motor_.voltage(), motor_.current(),
        link.fresh() ? static_cast<std::uint64_t>(link.age().count() / 1000)
                     : unknown,
        age(position_time), age(velocity_time),
        position_time.has_value() && velocity_time.has_value(),
        motor_.connected(), fault_valid && status_->motor_fault_mask() != 0,
        fault_valid, enabled, mode,
        mode != ControlMode::NEUTRAL && target_.source() == "teleop"
            ? TargetSource::TELEOP
            : TargetSource::NONE});
  }

 private:
  talos::Node& node_;
  talos::Motor motor_;
  talos::Target<MotorTarget> target_;
  talos::Publisher<MotorState> state_;
  talos::Subscription<bridge::BridgeStatus> status_;
  double max_voltage_v_;
};
}  // namespace motor
