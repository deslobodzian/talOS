#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <span>
#include <utility>

#include "talos_nodes/shooter/packet.h"
#include "talos_nodes/shooter/shooter_message_generated.h"
#include "talOS/events/handles.h"
#include "talOS/hardware/gate.h"
#include "talOS/hardware/messages.h"

namespace talos::shooter {

template <typename Loop>
class ShooterNode {
 public:
  explicit ShooterNode(Loop& loop, hardware::Config config,
                       hardware::Devices devices = {})
      : config_{std::move(config)},
        devices_{std::move(devices)},
        config_id_{hardware::ConfigurationId(config_)} {
    // Determine flywheel motor ID
    for (const auto& m : config_.motors) {
      if (m.name == "flywheel") {
        flywheel_id_ = m.id;
        break;
      }
    }
    if (flywheel_id_ == 0 && !devices_.motors.empty()) {
      flywheel_id_ = devices_.motors[0];
    }
    if (devices_.motors.empty() && flywheel_id_ != 0) {
      devices_.motors.push_back(flywheel_id_);
    }

    // Determine beam break digital input ID
    for (const auto& di : config_.digital_inputs) {
      if (di.name == "beam_break") {
        beam_break_id_ = di.id;
        break;
      }
    }
    if (beam_break_id_ == 0 && !devices_.digital_inputs.empty()) {
      beam_break_id_ = devices_.digital_inputs[0];
    }
    if (devices_.digital_inputs.empty() && beam_break_id_ != 0) {
      devices_.digital_inputs.push_back(beam_break_id_);
    }

    event::watch<Packet, &ShooterNode::OnState>(loop, kHwStateTopic, this);
    event::watch<ShooterTarget, &ShooterNode::OnTarget>(
        loop, kShooterTargetTopic, this);
    request_ = event::make_sender<Packet>(loop, kShooterRequestTopic);
    shooter_state_ = event::make_sender<ShooterState>(loop, kShooterStateTopic);
    timer_ = event::make_timer<&ShooterNode::Tick>(loop, "shooter", this);
  }

  void Start(event::MonotonicTime first) {
    timer_.setup_periodic(first, std::chrono::microseconds{config_.period_us});
  }

  uint16_t timer_id() const { return timer_.id(); }
  uint16_t flywheel_id() const { return flywheel_id_; }
  uint16_t beam_break_id() const { return beam_break_id_; }

 private:
  void OnState(const event::Context& context, const Packet& packet) {
    hardware::State state;
    if (!hardware::Decode(packet.bytes(), state)) return;
    if (!hardware::AcceptState(have_state_, state_, config_id_, state)) return;
    state_ = state;
    state_received_ = context.now;
    have_state_ = true;
  }

  void OnTarget(const event::Context&, const ShooterTarget& target) {
    target_ = target;
  }

  void Tick(const event::Context& context) {
    if (!have_state_) return;

    const auto timeout = std::chrono::microseconds{config_.command_timeout_us};
    const auto issued = event::MonotonicTime::from_nanos(target_.issued_ns());

    const bool gateway_ok = hardware::GatewayReady(state_);

    const bool target_fresh = hardware::TargetFresh(
        context.now, issued, target_.enabled(), state_received_, timeout);

    // Find flywheel sample in state_
    bool flywheel_sample_valid = false;
    double flywheel_velocity_rps = 0.0;
    if (const hardware::MotorSample* sample =
            hardware::FindMotor(state_, flywheel_id_)) {
      flywheel_sample_valid = sample->valid;
      flywheel_velocity_rps = sample->velocity_rps;
    }

    // Find beam break value in state_
    bool beam_broken = false;
    for (std::size_t i = 0; i < state_.digital_input_count; ++i) {
      if (state_.digital_inputs[i].id == beam_break_id_) {
        beam_broken = state_.digital_inputs[i].value;
        break;
      }
    }

    bool valid = gateway_ok && target_fresh && flywheel_sample_valid &&
                 std::isfinite(target_.target_velocity_rps());

    // Build command slice for flywheel
    hardware::Command command = hardware::NeutralCommand(
        state_, std::span<const uint16_t>(&flywheel_id_, 1));
    command.motors[0].slot = 0;
    if (valid) {
      command.motors[0].mode = hardware::Mode::kVelocity;
      command.motors[0].demand = target_.target_velocity_rps();
      command.motors[0].feedforward_v = 0.0;
    } else {
      command.motors[0].mode = hardware::Mode::kNeutral;
      command.motors[0].demand = 0.0;
      command.motors[0].feedforward_v = 0.0;
    }

    Packet packet{};
    packet.size = static_cast<uint32_t>(hardware::Encode(command, packet.data));
    if (packet.size > 0) {
      request_.send(packet);
    }

    // Publish ShooterState
    double target_vel = valid ? target_.target_velocity_rps() : 0.0;
    constexpr double kAtSpeedToleranceRps = 2.0;
    bool at_speed =
        valid && std::abs(target_.target_velocity_rps()) > 1e-3 &&
        std::abs(flywheel_velocity_rps - target_.target_velocity_rps()) <=
            kAtSpeedToleranceRps;
    bool enabled =
        (state_.flags & (hardware::kConfigured | hardware::kEnabled)) ==
        (hardware::kConfigured | hardware::kEnabled);
    uint64_t ts_ns = state_.sample_time_us * 1000;

    ShooterState shooter_st{ts_ns,    flywheel_velocity_rps, target_vel,
                            at_speed, beam_broken,           enabled};
    shooter_state_.send(shooter_st);
  }

  hardware::Config config_;
  hardware::Devices devices_;
  uint64_t config_id_{0};
  uint16_t flywheel_id_{0};
  uint16_t beam_break_id_{0};
  hardware::State state_{};
  bool have_state_{false};
  event::MonotonicTime state_received_{};
  ShooterTarget target_{};
  event::Sender<Loop, Packet> request_;
  event::Sender<Loop, ShooterState> shooter_state_;
  event::Timer<Loop> timer_;
};

}  // namespace talos::shooter
