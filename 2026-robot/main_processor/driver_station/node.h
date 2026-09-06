#pragma once

#include <cstdint>

#include "2026-robot/main_processor/driver_station/driver_station_message_generated.h"
#include "2026-robot/main_processor/drivetrain/packet.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/handles.h"

// Decodes the Driver Station packet the bridge forwards from the controller
// processor and republishes it verbatim. Nothing here decides what the robot
// should do: joystick mapping and the requests that come out of it live in
// 2026-robot/main_processor/operator_interface, so that the wire format has one
// reader and the game logic can change without touching it.
namespace talos::driver_station {

inline constexpr const char* kHwDsTopic = "/hw/ds";
inline constexpr const char* kDsStateTopic = "/driver_station/state";

template <typename Loop>
class DriverStationNode {
 public:
  explicit DriverStationNode(Loop& loop)
      : driver_station_state_{
            event::make_sender<DriverStationState>(loop, kDsStateTopic)} {
    event::watch<talos::drive::Packet, &DriverStationNode::OnDsPacket>(
        loop, kHwDsTopic, this);
  }

  void Start(event::MonotonicTime = {}) {}

  const DriverStationState& last_state() const { return last_state_; }
  bool has_state() const { return has_state_; }
  uint64_t packets_decoded() const { return decoded_; }
  uint64_t packets_rejected() const { return rejected_; }

 private:
  void OnDsPacket(const event::Context& ctx, const talos::drive::Packet& pkt) {
    DriverStationData ds_data{};
    if (!Decode(pkt.bytes(), ds_data)) {
      ++rejected_;
      return;
    }

    FmsState fms{
        ds_data.fms.flags,        static_cast<uint8_t>(ds_data.fms.alliance),
        ds_data.fms.station,      static_cast<uint8_t>(ds_data.fms.match_type),
        ds_data.fms.match_number, ds_data.fms.replay_number,
        ds_data.fms.match_time_s,
    };

    // Only the first two sticks fit the published message. axis_count and
    // button_count are carried through unchanged so a consumer can tell a
    // centred axis from one the joystick never reported.
    auto to_joystick_state = [](const JoystickData& j) {
      return JoystickState{
          j.connected, j.axis_count, j.button_count, j.pov_count, j.buttons,
          j.axes[0],   j.axes[1],    j.axes[2],      j.axes[3],   j.povs[0],
      };
    };

    // timestamp_ns is the loop clock every other node stamps with; the Rio
    // sample stamp rides along in its own field rather than aliasing it.
    DriverStationState state{static_cast<uint64_t>(ctx.now.nanos()),
                             ds_data.sample_time_us, fms,
                             to_joystick_state(ds_data.joysticks[0]),
                             to_joystick_state(ds_data.joysticks[1])};
    last_state_ = state;
    has_state_ = true;
    ++decoded_;
    driver_station_state_.send(state);
  }

  event::Sender<Loop, DriverStationState> driver_station_state_;
  DriverStationState last_state_{};
  bool has_state_{false};
  uint64_t decoded_{0}, rejected_{0};
};

}  // namespace talos::driver_station
