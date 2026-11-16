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
// The codecs are this build's (protocol::kProtocolVersion). A recorded
// Packet holds a frame's payload without its header, and the Packet's schema
// identity hashes only its size, so the packets do not say which protocol
// they are of. A session records it twice (RecordedProtocolVersion): the
// bridge's own build's version in every /hw/status it publishes
// (bridge.BridgeStatus protocol_version_id), and the launcher's build's in
// manifest.json (hardware_protocol_version). The bridge's, when recorded, is
// the session's version; the manifest's only when no log of the session
// records a /hw/status of this build's BridgeStatus schema (a session from
// before the field, one with no bridge, or a .tlog read without its
// session's). HardwareProtocolCheck runs once over a session before any
// packet is decoded:
//   - a recorded version other than this build's, or a bridge that recorded
//     two, refuses every packet, none tried;
//   - otherwise every packet is tried with this build's codecs. Every
//     hardware packet decoded under the build that recorded it (the bridge
//     publishes only a State or Driver Station packet its codec accepted;
//     requests and commands are its codec's encodings), so a rejected one
//     means, with no recorded version, a session of another protocol, and,
//     with this build's version recorded, a packet a codec of another build
//     wrote (a node binary of another build, or, with only the launcher's
//     version, the bridge's), or one made by hand.
// Either way a tool then decodes none of the session's hardware packets.
// A session without a recorded version, of another protocol whose packets
// all happen to decode (a change that kept every length), cannot be told
// apart, and decodes with this build's layout.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
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

// The hardware protocol versions a session records (see above).
struct RecordedProtocol {
  // manifest.json's hardware_protocol_version: the launcher build's.
  std::optional<std::uint16_t> manifest;
  // The bridge's: the protocol_version_id of the first /hw/status SEND in
  // each of the session's logs whose manifest registers /hw/status as a
  // sender of this build's BridgeStatus schema, the distinct values in log
  // path order. One bridge build writes one value, so two mean bridge runs
  // of different builds in one session (which one bundle cannot launch).
  // A /hw/status of another schema (a bridge from before the field), or
  // whose field is 0 (no version: one a test wrote), adds none.
  std::vector<std::uint16_t> bridge;

  // The version the session's packets are read under: the bridge's when it
  // recorded exactly one, else the manifest's when the bridge recorded none.
  std::optional<std::uint16_t> version() const;
  // Its packets are refused untried: the bridge recorded two versions, or
  // version() is not this build's.
  bool refused() const;
  // Why, when refused(), else empty: "recorded under hardware protocol <R>;
  // this build's is <N> (<where>)", <where> "/hw/status protocol_version_id,
  // the bridge's" or "manifest.json hardware_protocol_version"; for two
  // bridge versions "recorded under hardware protocols <A> and <B> by bridge
  // runs of different builds (/hw/status protocol_version_id); this build's
  // is <N>".
  std::string Refusal() const;
  // The one line a tool prints after its own name when the manifest's and
  // the bridge's one version differ, else empty: "warning: the session's
  // bridge recorded hardware protocol <B> (/hw/status protocol_version_id),
  // its launcher <L> (manifest.json hardware_protocol_version): read under
  // the bridge's".
  std::string Disagreement() const;
};

// The hardware protocol versions a session records, for the first of
// `inputs` that records any. A directory is a session: its manifest.json and
// its logs (event::log::FindLogFiles). A log file is read with its session
// when the manifest.json beside it has the log header's session_id: that
// manifest and the logs beside it of that session; otherwise (another
// session's manifest, none, or no log header) the file alone. Each log is
// read only to its first /hw/status record, and a log that registers no
// /hw/status sender only to its manifest: the bridge publishes a status on
// its first tick, so this costs about one header read per log, not a pass
// over the session. Never throws: an unreadable log records nothing.
RecordedProtocol RecordedProtocolVersion(
    const std::vector<std::string>& inputs);

// Whether a session's hardware packets can be decoded with this build's
// codecs (see above): from the versions it records, if any, then from every
// recorded hardware packet payload Add()ed to it before the tool decodes
// one. Every tool that decodes recorded hardware packets runs it, prints
// the RecordedProtocol's Disagreement() if any, and, when foreign(), prints
// Warning() once and decodes none.
class HardwareProtocolCheck {
 public:
  // `recorded`: RecordedProtocolVersion of the session's inputs.
  explicit HardwareProtocolCheck(RecordedProtocol recorded = {});
  // One recorded payload of a hardware packet topic (`kind` not kNone).
  // `reader_copy` for a reader's MESSAGE record when the caller also adds
  // the writer's SEND records: each publication's bytes are the same in
  // every copy, so a reader's copy of a topic whose SEND was already tried
  // is not tried again (a caller that adds one copy per publication passes
  // false). No packet is tried when the recorded version is not this
  // build's. Returns the packet decoded when it was tried and decoded, else
  // null; valid until the next Add.
  const HardwarePacket* Add(HardwareRecord kind, std::string_view topic,
                            bool reader_copy,
                            std::span<const std::byte> payload);
  // The recorded versions refuse the packets (RecordedProtocol::refused), or
  // a packet this build's codecs reject was added.
  bool foreign() const { return recorded_.refused() || rejected_ != 0; }
  // The one line a tool prints after its own name when foreign(), else
  // empty. It starts "warning: hardware packets not decoded: ", then:
  //   refused by the recorded versions: RecordedProtocol::Refusal();
  //   none recorded: "recorded under a hardware protocol other than this
  //     build's protocol <N> (the session records no protocol version);
  //     this build's codecs reject <k> of the <n> tried, the first on
  //     <topic>: <why>";
  //   this build's recorded: "recorded under this build's hardware
  //     protocol <N> (<whose>), yet this build's codecs reject <k> of the
  //     <n> tried, the first on <topic>: <why>; the session's <what> may be
  //     of another build", <whose> and <what> "the bridge's" and "nodes"
  //     when the bridge recorded it, else "the launcher's" and "bridge or
  //     nodes".
  std::string Warning() const;

 private:
  RecordedProtocol recorded_;
  std::uint64_t packets_{0};
  std::uint64_t rejected_{0};
  std::string first_topic_;
  std::string first_why_;
  // Hardware topics a record other than a reader's copy has been tried of.
  std::set<std::string, std::less<>> sent_;
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
