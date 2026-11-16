// stop_paths_fixture <directory> [foreign-protocol]
//
// Records the session stop_paths_test reads: the bench's stop path 2 (the
// teleop node stopped for 300 ms with the stick held) followed by stop path 1
// (a Driver Station disable), as the bridge, `motor` and `teleop` log it.
// Every stream ticks every 5 ms from 0 to 500 ms; times below are ms on the
// session's log clock, each record's event, now and publish time alike.
//
//   hardware_node  a Driver Station packet at 0 (teleop, enabled) and at 400
//                  (disabled); a State every tick, enabled and active until
//                  400, the motor at 1 V until 405 and at 0 V from 410; and
//                  1 ms after each request, a command of the same mode.
//   motor          a request every tick: kraken at 1 V while its lane is
//                  fresh (to 150 ms, the lane's last message plus 100 ms),
//                  neutral from 155. Its log loses the requests of 200 to
//                  235 ms to one GAP record, as a full writer pool does.
//   teleop         /motor/target/teleop active to 50 ms, silent while
//                  stopped, released (active false) from 350.
//
// Device names come from the robot.toml it writes beside manifest.json,
// which names that config. foreign-protocol: every State 13 bytes shorter,
// as protocol 7 wrote them, and no protocol in the manifest, so log_dump
// decodes none of the session's hardware packets.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "continuity_test/continuity_message_descriptor.h"
#include "continuity_test/continuity_message_generated.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/log/gap_testing.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/launcher/session.h"
#include "talOS/schema/identity.h"

namespace {

constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.motor]
period_us = 5000

[subsystems.motor.motors.kraken]
type = "TalonFX"
can_id = 1
)";

constexpr std::uint64_t kSampleOriginUs = 1'000'000;  // the RoboRIO's clock
constexpr std::uint64_t kModeSinceUs = 900'000;

using talos::event::EventKind;
using talos::event::log::LogWriter;

// One log of `process` whose records are each their own dispatch at `ms`.
class Log {
 public:
  Log(const std::string& directory, const std::string& process,
      const talos::event::Manifest& sources)
      : writer_{directory + "/" + process + ".tlog", process,
                talos::event::log::LogWriterOptions{1 << 16, 1, false}, 44} {
    writer_.start(sources, Time(0));
  }

  void Send(int ms, std::uint16_t id, std::span<const std::byte> payload) {
    talos::event::Context context;
    context.kind = EventKind::SEND;
    context.source_id = id;
    context.dispatch_index = ++index_;
    context.now = Time(ms);
    context.event_time = context.now;
    context.publish_time = context.now;
    context.sequence = index_;
    writer_.send(context, id, payload, 0, index_);
  }

  bool Finish() {
    writer_.flush();
    return !writer_.incomplete();
  }

 private:
  static talos::event::MonotonicTime Time(int ms) {
    return talos::event::MonotonicTime::from_nanos(1'000'000'000) +
           std::chrono::milliseconds{ms};
  }

  LogWriter writer_;
  std::uint64_t index_{0};
};

talos::event::Registration Sender(std::uint16_t id, std::string name,
                                  talos::schema::Identity schema,
                                  std::uint32_t bytes) {
  talos::event::Registration source;
  source.id = id;
  source.kind = talos::event::SourceKind::SENDER;
  source.name = std::move(name);
  source.message_bytes = bytes;
  source.schema = std::move(schema);
  return source;
}

talos::event::Registration PacketSender(std::uint16_t id, std::string name) {
  return Sender(id, std::move(name),
                talos::schema::Of<talos::hardware::Packet>(),
                sizeof(talos::hardware::Packet));
}

std::span<const std::byte> Bytes(const talos::hardware::Packet& packet) {
  return std::as_bytes(std::span{&packet, 1});
}

std::uint64_t SampleUs(int ms) {
  return kSampleOriginUs + static_cast<std::uint64_t>(ms) * 1000;
}

talos::hardware::Packet DriverStation(int ms, bool enabled) {
  namespace ds = talos::driver_station;
  ds::DriverStationData data;
  data.sample_time_us = SampleUs(ms);
  data.mode_since_us = kModeSinceUs;
  data.fms.flags = ds::kDsAttached | ds::kTeleop | (enabled ? ds::kEnabled : 0);
  talos::hardware::Packet packet;
  packet.size = static_cast<std::uint32_t>(ds::Encode(data, packet.data));
  return packet;
}

talos::hardware::Packet Request(std::uint64_t config_id, std::uint16_t motor,
                                int ms, bool driving) {
  talos::hardware::Command command;
  command.config_id = config_id;
  command.boot_id = 7;
  command.epoch = 1;
  command.observed_time_us = SampleUs(ms);
  command.count = 1;
  command.motors[0].id = motor;
  command.motors[0].mode = driving ? talos::hardware::Mode::kVoltage
                                   : talos::hardware::Mode::kNeutral;
  command.motors[0].demand = driving ? 1.0 : 0.0;
  talos::hardware::Packet packet;
  packet.size =
      static_cast<std::uint32_t>(talos::hardware::Encode(command, packet.data));
  return packet;
}

talos::hardware::Packet State(std::uint64_t config_id, std::uint16_t motor,
                              int ms, bool foreign) {
  talos::hardware::State state;
  state.config_id = config_id;
  state.boot_id = 7;
  state.epoch = 1;
  state.sample_time_us = SampleUs(ms);
  state.sequence = static_cast<std::uint64_t>(ms / 5);
  state.last_command_sequence = static_cast<std::uint64_t>(ms / 5);
  state.flags = talos::hardware::kConfigured;
  if (ms < 400) {
    state.flags |= talos::hardware::kEnabled | talos::hardware::kCommandActive;
  }
  state.motor_count = 1;
  state.motors[0].id = motor;
  state.motors[0].valid = true;
  state.motors[0].voltage = ms <= 405 ? 1.0 : 0.0;
  state.motors[0].velocity_rps = ms <= 405 ? 10.0 : 0.0;
  talos::hardware::Packet packet;
  packet.size =
      static_cast<std::uint32_t>(talos::hardware::Encode(state, packet.data));
  if (foreign) packet.size -= 13;
  return packet;
}

// The teleop lane's envelope (talOS/devices/target.h) around a Count.
std::array<std::byte,
           talos::schema::kEnvelopeHeaderBytes + sizeof(continuity_test::Count)>
Lane(bool active) {
  std::array<std::byte, talos::schema::kEnvelopeHeaderBytes +
                            sizeof(continuity_test::Count)>
      envelope{};
  const std::uint64_t mode_since_us = kModeSinceUs;
  std::memcpy(envelope.data(), &mode_since_us, sizeof mode_since_us);
  envelope[talos::schema::kEnvelopeActiveOffset] =
      active ? std::byte{1} : std::byte{0};
  const continuity_test::Count count{1, 2};
  std::memcpy(envelope.data() + talos::schema::kEnvelopeHeaderBytes, &count,
              sizeof count);
  return envelope;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) return 1;
  const std::string directory = argv[1];
  const std::string mode = argc == 3 ? argv[2] : "";
  if (!mode.empty() && mode != "foreign-protocol") return 1;
  const bool foreign = mode == "foreign-protocol";
  std::filesystem::create_directories(directory);
  std::ofstream{directory + "/robot.toml"} << kToml;
  const auto config = talos::config::ParseRobotConfigString(kToml).hardware;
  const std::uint64_t config_id = talos::hardware::ConfigurationId(config);
  const std::uint16_t kraken = config.motors.at(0).id;

  talos::launcher::SessionManifest manifest;
  manifest.session_id = 44;
  manifest.config_path = directory + "/robot.toml";
  manifest.end_wall_ns = 1;
  if (foreign) manifest.hardware_protocol_version.reset();
  for (const char* name : {"hardware_node", "motor", "teleop"}) {
    talos::launcher::NodeProcessInfo node;
    node.name = name;
    node.exited = true;
    manifest.nodes.push_back(node);
  }
  std::ofstream{directory + "/manifest.json"} << manifest.ToJson();

  using D = talos::Descriptor<continuity_test::Count>;
  const talos::schema::Identity count{
      std::string{D::kName}, D::kHash,
      std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                  D::kBfbs.size()}};
  Log bridge{directory,
             "hardware_node",
             {PacketSender(0, talos::hardware::kStateTopic),
              PacketSender(1, talos::hardware::kDriverStationTopic),
              PacketSender(2, talos::hardware::kCommandTopic)}};
  Log motor{directory,
            "motor",
            {PacketSender(0, talos::hardware::RequestTopic("motor"))}};
  Log teleop{directory,
             "teleop",
             {Sender(0, "/motor/target/teleop", talos::schema::Envelope(count),
                     talos::schema::kEnvelopeHeaderBytes +
                         sizeof(continuity_test::Count))}};

  bridge.Send(0, 1, Bytes(DriverStation(0, true)));
  for (int ms = 0; ms <= 500; ms += 5) {
    if (ms == 400) bridge.Send(ms, 1, Bytes(DriverStation(ms, false)));
    bridge.Send(ms, 0, Bytes(State(config_id, kraken, ms, foreign)));
    const bool driving = ms <= 150;
    motor.Send(ms, 0, Bytes(Request(config_id, kraken, ms, driving)));
    bridge.Send(ms + 1, 2, Bytes(Request(config_id, kraken, ms, driving)));
    if (ms <= 50 || ms >= 350) {
      const auto lane = Lane(ms <= 50);
      teleop.Send(ms, 0, lane);
    }
  }
  if (!bridge.Finish() || !motor.Finish() || !teleop.Finish()) return 1;
  // motor's records 40 to 47 are its requests of 200 to 235 ms.
  const std::string log = directory + "/motor.tlog";
  talos::event::log::CopyWithGap(log, log + ".gapped", 40, 48);
  std::filesystem::rename(log + ".gapped", log);
  return 0;
}
