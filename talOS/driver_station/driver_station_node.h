#pragma once

#include <cstdint>

#include "driver_station/driver_station_message_generated.h"
#include "driver_station/topics.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/node/access.h"
#include "talOS/node/node.h"

namespace driver_station {

// Decodes the Driver Station packet the bridge forwards from the controller
// processor and republishes it, as it arrives. Nothing here decides what the
// robot should do: joystick mapping lives in the operator interface, so the
// wire format has one reader and game logic can change without touching it.
class DriverStation {
 public:
  explicit DriverStation(talos::Node& node)
      : node_{node}, state_{node.publish<DriverStationState>(kStateTopic)} {
    // The bridge owns this topic and its layout (hardware::Packet, aligned
    // as the bridge registers it), so it is subscribed raw rather than as a
    // talos message.
    using talos::hardware::Packet;
    talos::detail::NodeAccess::add_subscription(
        node, talos::hardware::kDriverStationTopic, sizeof(Packet),
        alignof(Packet), /*optional=*/false, [this](const void* packet) {
          OnPacket(*static_cast<const Packet*>(packet));
        });
  }

 private:
  void OnPacket(const talos::hardware::Packet& packet) {
    talos::driver_station::DriverStationData data{};
    if (!talos::driver_station::Decode(packet.bytes(), data)) return;
    const auto& fms = data.fms;
    // Only the first two sticks fit the message. axis_count and button_count
    // ride along so a consumer can tell a centred axis from a missing one.
    const auto stick = [](const talos::driver_station::JoystickData& j) {
      return JoystickState{j.connected, j.axis_count, j.button_count,
                           j.pov_count, j.buttons,    j.axes[0],
                           j.axes[1],   j.axes[2],    j.axes[3],
                           j.povs[0]};
    };
    // timestamp_ns is the node clock every other node stamps with; the Rio
    // sample stamp rides in its own field.
    state_.send(DriverStationState{
        static_cast<std::uint64_t>(node_.now().nanos()), data.sample_time_us,
        FmsState{fms.flags, static_cast<std::uint8_t>(fms.alliance),
                 fms.station, static_cast<std::uint8_t>(fms.match_type),
                 fms.match_number, fms.replay_number, fms.match_time_s},
        stick(data.joysticks[0]), stick(data.joysticks[1])});
  }

  talos::Node& node_;
  talos::Publisher<DriverStationState> state_;
};

}  // namespace driver_station
