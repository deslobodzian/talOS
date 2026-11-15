#pragma once

// The hardware-layer packet topics (/hw/state, /hw/command,
// /hw/request/<node>, /hw/state/driver_station) carry a
// talos.hardware.Packet with no FlatBuffers schema, so the descriptor decoder
// (talOS/schema/decode.h) can show only their bytes. This decodes them with
// the wire codecs themselves (hardware::Decode, driver_station::Decode) into
// one JSON object: what log_dump prints for those records. tlog_to_wpilog
// takes the decoded structs (DecodeHardwareRecord) and writes them as
// entries.
//
//   /hw/state                 {"type":"state","config_id":"...","enabled":true,
//                              "motors":[{"id":1,"name":"flywheel",...}],...,
//                              "health":{...,"can_bus":{"index":1,
//                              "name":"Shooter","read_state":"ok",...}}}
//   /hw/command, /hw/request  {"type":"command",...,"motors":[{"id":1,
//                              "name":"flywheel","mode":"velocity",...}]}
//   /hw/state/driver_station  {"type":"driver_station","mode":"teleop",
//                              "enabled":true,...,"joysticks":[...]}
//   a payload that will not   {"type":"state","error":"..."}
//   decode
//
// Device and CAN bus names come from the robot config the session ran
// (DeviceNames) and
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
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/messages.h"

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
  // The State's CAN bus slots, in slot order (hardware::CanBuses).
  std::vector<std::string> can_buses;

  static DeviceNames FromConfig(const hardware::Config& config);
  bool Matches(std::uint64_t id) const {
    return id != 0 && (id == config_id || id == commissioned_config_id);
  }
};

// The robot config whose device names label a session's hardware packets:
// `config` when given, else the config_path in the manifest.json of the
// first directory among `inputs` that names one; empty when neither does. A
// relative path is the user's or the launcher's, relative to where they ran:
// under `bazel run` that is BUILD_WORKING_DIRECTORY, since a tool then runs
// in its runfiles directory.
std::string SessionConfigPath(const std::string& config,
                              const std::vector<std::string>& inputs);

// DeviceNames from the robot config at `path`; nullopt, with `error` set,
// when it cannot be read or parsed.
std::optional<DeviceNames> LoadDeviceNames(const std::string& path,
                                           std::string& error);

// One recorded Packet decoded with its wire codec: `state` for kState,
// `command` for kCommand, `driver_station` for kDriverStation.
struct HardwarePacket {
  hardware::State state;
  hardware::Command command;
  driver_station::DriverStationData driver_station;
};

// Decodes one recorded Packet payload (sizeof(hardware::Packet) bytes) into
// `out`. Empty on success, else why it does not decode: a payload of the
// wrong size, a Packet size past its buffer, or bytes the codec rejects.
// Reads only inside `payload`; never throws.
std::string DecodeHardwareRecord(HardwareRecord kind,
                                 std::span<const std::byte> payload,
                                 HardwarePacket& out);

// The names log_dump gives these values, shared so every tool spells them
// alike.
const char* HardwareModeName(hardware::Mode mode);
const char* CanBusReadStateName(hardware::CanBusReadState state);
// The mode the Driver Station selected (its flags), whether or not the robot
// is enabled in it: "teleop", "autonomous", "test" or "none".
const char* DriverStationModeName(std::uint32_t flags);
const char* AllianceName(driver_station::Alliance alliance);
const char* MatchTypeName(driver_station::MatchType type);

// The JSON object for one recorded Packet payload (sizeof(hardware::Packet)
// bytes). Never throws: a payload of the wrong size or one the codec rejects
// yields an "error" member in place of the fields. `names` may be null.
std::string HardwareRecordJson(HardwareRecord kind,
                               std::span<const std::byte> payload,
                               const DeviceNames* names);

}  // namespace talos::launcher
