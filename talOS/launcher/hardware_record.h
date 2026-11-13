#pragma once

// The hardware-layer packet topics (/hw/state, /hw/command,
// /hw/request/<node>, /hw/state/driver_station) carry a
// talos.hardware.Packet with no FlatBuffers schema, so the descriptor decoder
// (talOS/schema/decode.h) can show only their bytes. This decodes them with
// the wire codecs themselves (hardware::Decode, driver_station::Decode) into
// one JSON object: what log_dump prints for those records.
//
//   /hw/state                 {"type":"state","config_id":"...","enabled":true,
//                              "motors":[{"id":1,"name":"flywheel",...}],...}
//   /hw/command, /hw/request  {"type":"command",...,"motors":[{"id":1,
//                              "name":"flywheel","mode":"velocity",...}]}
//   /hw/state/driver_station  {"type":"driver_station","mode":"teleop",
//                              "enabled":true,...,"joysticks":[...]}
//   a payload that will not   {"type":"state","error":"..."}
//   decode
//
// Device names come from the robot config the session ran (DeviceNames) and
// are given only for a packet whose config_id is that config's, so a config
// edited after the match never mislabels a device; the logical id always is.
//
// The codecs are this build's. The Packet's schema identity hashes only its
// size, so a log recorded before a wire change that kept sizeof(Packet)
// decodes with today's layout: wrongly, or as an error when the lengths no
// longer agree. payload_hex keeps the recorded bytes either way.

#include <cstddef>
#include <cstdint>
#include <map>
#include <span>
#include <string>
#include <string_view>

#include "talOS/hardware/config.h"

namespace talos::launcher {

enum class HardwareRecord : std::uint8_t {
  kNone,
  kState,
  kCommand,
  kDriverStation
};

// Which packet a recorded topic carries; kNone unless `schema_name` is the
// hardware Packet's (talOS/schema/identity.h kPacketName).
HardwareRecord HardwareRecordOf(std::string_view topic,
                                std::string_view schema_name);

// Logical device id to config name, per device kind, for the configuration
// ids this config can run under: as written, and commissioned (the bridge
// commissions it in memory under --sim, talOS/bridge/node.cc).
struct DeviceNames {
  std::uint64_t config_id{};
  std::uint64_t commissioned_config_id{};
  std::map<std::uint16_t, std::string> motors, sensors, digital_inputs,
      digital_outputs, analog_inputs, encoders, pwm_outputs;

  static DeviceNames FromConfig(const hardware::Config& config);
  bool Matches(std::uint64_t id) const {
    return id != 0 && (id == config_id || id == commissioned_config_id);
  }
};

// The JSON object for one recorded Packet payload (sizeof(hardware::Packet)
// bytes). Never throws: a payload of the wrong size or one the codec rejects
// yields an "error" member in place of the fields. `names` may be null.
std::string HardwareRecordJson(HardwareRecord kind,
                               std::span<const std::byte> payload,
                               const DeviceNames* names);

}  // namespace talos::launcher
