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
// The codecs are this build's (protocol::kProtocolVersion). The logs do not
// store the protocol version a session was recorded under (a recorded
// Packet holds a frame's payload without its header, and the Packet's schema
// identity hashes only its size); the session's manifest.json does
// (hardware_protocol_version, RecordedProtocolVersion), for a session
// recorded since it was added. HardwareProtocolCheck runs once over a
// session before any packet is decoded:
//   - a recorded version other than this build's refuses every packet, none
//     tried;
//   - otherwise every packet is tried with this build's codecs. Every
//     hardware packet decoded under the build that recorded it (the bridge
//     publishes only a State or Driver Station packet its codec accepted;
//     requests and commands are its codec's encodings), so a rejected one
//     means, with no recorded version, a session of another protocol, and,
//     with this build's version recorded, a packet its codec did not write.
// Either way a tool then decodes none of the session's hardware packets.
// A session without a recorded version, of another protocol whose packets
// all happen to decode (a change that kept every length), cannot be told
// apart, and decodes with this build's layout.

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

// The hardware protocol version a session's manifest.json records, for the
// first of `inputs` that has one: a directory's own manifest.json, or for a
// log file the manifest.json beside it when its session_id is the log
// header's (not when it names another session, or has none, or the file has
// no log header). nullopt when none does: a session recorded before the
// field, or a log with no manifest of its session beside it.
std::optional<std::uint16_t> RecordedProtocolVersion(
    const std::vector<std::string>& inputs);

// Whether a session's hardware packets can be decoded with this build's
// codecs (see above): from the version its manifest records, if any, then
// from every recorded hardware packet payload Add()ed to it before the tool
// decodes one. Every tool that decodes recorded hardware packets runs it
// and, when foreign(), prints Warning() once and decodes none.
class HardwareProtocolCheck {
 public:
  // `recorded`: RecordedProtocolVersion of the session's inputs.
  explicit HardwareProtocolCheck(
      std::optional<std::uint16_t> recorded = std::nullopt);
  // One recorded payload of a hardware packet topic (`kind` not kNone).
  // Not tried when the recorded version is not this build's.
  void Add(HardwareRecord kind, std::string_view topic,
           std::span<const std::byte> payload);
  // The recorded version is not this build's, or a packet this build's
  // codecs reject was added.
  bool foreign() const { return refused_by_version() || rejected_ != 0; }
  // The one line a tool prints after its own name when foreign(), else
  // empty. It starts "warning: hardware packets not decoded: ", then:
  //   another version recorded: "recorded under hardware protocol <R>;
  //     this build's is <N> (manifest.json hardware_protocol_version)";
  //   none recorded: "recorded under a hardware protocol other than this
  //     build's protocol <N> (the session records no protocol version);
  //     this build's codecs reject <k> of <n>, the first on <topic>: <why>";
  //   this build's recorded: "recorded under this build's hardware
  //     protocol <N>, yet this build's codecs reject <k> of <n>, ... (packets
  //     its codec did not write, not another protocol)".
  std::string Warning() const;

 private:
  bool refused_by_version() const;

  std::optional<std::uint16_t> recorded_;
  std::uint64_t packets_{0};
  std::uint64_t rejected_{0};
  std::string first_topic_;
  std::string first_why_;
  HardwarePacket scratch_;
};

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
