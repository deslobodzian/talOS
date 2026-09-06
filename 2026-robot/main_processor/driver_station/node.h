#pragma once

#include <cmath>
#include <cstdint>

#include "talOS/driver_station/driver_station.h"
#include "2026-robot/main_processor/driver_station/driver_station_message_generated.h"
#include "2026-robot/main_processor/drivetrain/drive_message_generated.h"
#include "2026-robot/main_processor/drivetrain/packet.h"
#include "talOS/events/handles.h"
#include "2026-robot/main_processor/shooter/packet.h"
#include "2026-robot/main_processor/shooter/shooter_message_generated.h"

namespace talos::driver_station {

inline constexpr const char* kHwDsTopic = "/hw/ds";
inline constexpr const char* kDsStateTopic = "/driver_station/state";

// Teleop mapping policy, read from [subsystems.driver_station]. The defaults
// are the values the mapping shipped with, so an absent table changes nothing.
struct TeleopConfig {
  double max_linear_mps{4.0};
  double max_angular_radps{6.0};
  double deadband{0.05};
  double shooter_target_rps{60.0};
};

template <typename Loop>
class DriverStationNode {
 public:
  explicit DriverStationNode(Loop& loop, TeleopConfig teleop = {})
      : teleop_{teleop},
        driver_station_state_{
            event::make_sender<DriverStationState>(loop, kDsStateTopic)},
        chassis_target_{event::make_sender<talos::drive::ChassisTarget>(
            loop, talos::drive::kTargetTopic)},
        shooter_target_{event::make_sender<talos::shooter::ShooterTarget>(
            loop, talos::shooter::kShooterTargetTopic)} {
    event::watch<talos::drive::Packet, &DriverStationNode::OnDsPacket>(
        loop, kHwDsTopic, this);
  }

  void Start(event::MonotonicTime = {}) {}

  const DriverStationState& last_state() const { return last_state_; }
  bool has_state() const { return has_state_; }

 private:
  // Zero inside the deadband, then rescaled so the output ramps from 0 at the
  // threshold to full scale at 1.0 instead of stepping.
  double Deadband(double value) const {
    const double magnitude = std::abs(value);
    if (magnitude <= teleop_.deadband || teleop_.deadband >= 1.0) return 0.0;
    const double scaled =
        (magnitude - teleop_.deadband) / (1.0 - teleop_.deadband);
    return std::copysign(scaled, value);
  }

  void OnDsPacket(const event::Context& ctx, const talos::drive::Packet& pkt) {
    DriverStationData ds_data{};
    if (!Decode(pkt.bytes(), ds_data)) {
      return;
    }

    FmsState fms{
        ds_data.fms.flags,
        static_cast<uint8_t>(ds_data.fms.alliance),
        ds_data.fms.station,
        static_cast<uint8_t>(ds_data.fms.match_type),
        ds_data.fms.match_number,
        ds_data.fms.replay_number,
        ds_data.fms.match_time_s,
    };

    auto to_joystick_state = [](const JoystickData& j) {
      return JoystickState{
          j.connected,
          j.axis_count,
          j.button_count,
          j.pov_count,
          j.buttons,
          j.axes[0],
          j.axes[1],
          j.axes[2],
          j.axes[3],
          j.povs[0],
      };
    };

    // timestamp_ns is the loop clock every other node stamps with; the Rio
    // sample stamp rides along in its own field rather than aliasing it.
    DriverStationState state{static_cast<uint64_t>(ctx.now.nanos()),
                             ds_data.sample_time_us,
                             fms,
                             to_joystick_state(ds_data.joysticks[0]),
                             to_joystick_state(ds_data.joysticks[1])};
    last_state_ = state;
    has_state_ = true;
    driver_station_state_.send(state);

    const bool enabled = (ds_data.fms.flags & kEnabled) != 0;
    const bool teleop = (ds_data.fms.flags & kTeleop) != 0;
    if (enabled && teleop && ds_data.joysticks[0].connected) {
      const double raw_vx = -static_cast<double>(ds_data.joysticks[0].axes[1]);
      const double raw_vy = -static_cast<double>(ds_data.joysticks[0].axes[0]);
      const double raw_omega = -static_cast<double>(ds_data.joysticks[0].axes[2]);

      const double vx = Deadband(raw_vx) * teleop_.max_linear_mps;
      const double vy = Deadband(raw_vy) * teleop_.max_linear_mps;
      const double omega = Deadband(raw_omega) * teleop_.max_angular_radps;

      talos::drive::ChassisTarget target{vx, vy, omega, ctx.now.nanos(), true};
      chassis_target_.send(target);

      // Shooter trigger (button 1, bit 0)
      const bool shoot_btn = (ds_data.joysticks[0].buttons & 1) != 0;
      const double shooter_velocity_rps =
          shoot_btn ? teleop_.shooter_target_rps : 0.0;
      talos::shooter::ShooterTarget shoot_target{shooter_velocity_rps,
                                                 ctx.now.nanos(), shoot_btn};
      shooter_target_.send(shoot_target);
    }
  }

  TeleopConfig teleop_;
  event::Sender<Loop, DriverStationState> driver_station_state_;
  event::Sender<Loop, talos::drive::ChassisTarget> chassis_target_;
  event::Sender<Loop, talos::shooter::ShooterTarget> shooter_target_;

  DriverStationState last_state_{};
  bool has_state_{false};
};

}  // namespace talos::driver_station
