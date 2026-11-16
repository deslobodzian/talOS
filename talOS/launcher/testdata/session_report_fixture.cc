#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>

#include "talOS/driver_station/driver_station.h"
#include "talOS/events/log/gap_testing.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/launcher/session.h"

namespace {

talos::hardware::Packet CommandPacket(std::uint64_t observed_time_us) {
  talos::hardware::Command command;
  command.boot_id = 7;
  command.observed_time_us = observed_time_us;
  talos::hardware::Packet packet;
  packet.size =
      static_cast<std::uint32_t>(talos::hardware::Encode(command, packet.data));
  return packet;
}

// A State this build's codec accepts, as every State the bridge publishes
// is; with `foreign`, one of the length protocol 7 gave it (13 bytes
// shorter: protocol 8 added one CAN bus sample per State), as a session
// recorded under that protocol holds.
talos::hardware::Packet StatePacket(bool foreign = false) {
  talos::hardware::Packet packet;
  packet.size = static_cast<std::uint32_t>(
      talos::hardware::Encode(talos::hardware::State{}, packet.data));
  if (foreign) packet.size -= 13;
  return packet;
}

// A disabled session of an event-driven bridge, which has no timer, and one
// control node: the bridge publishes a State every 5 ms after a disabled
// Driver Station packet; the node's tick 1 ms after each State sends a
// request observing it; the bridge sends the merged command 0.25 ms after
// that request, then the last one again as a heartbeat 20 ms later.
int DisabledBridgeSession(const std::string& directory) {
  using namespace talos::event;
  talos::launcher::SessionManifest manifest;
  manifest.session_id = 43;
  manifest.end_wall_ns = 1;
  for (const char* name : {"hardware_node", "motor"}) {
    talos::launcher::NodeProcessInfo node;
    node.name = name;
    node.exited = true;
    manifest.nodes.push_back(node);
  }
  std::ofstream{directory + "/manifest.json"} << manifest.ToJson();
  Registration state;
  state.id = 0;
  state.kind = SourceKind::SENDER;
  state.name = talos::hardware::kStateTopic;
  state.message_bytes = sizeof(talos::hardware::Packet);
  state.schema = talos::schema::Of<talos::hardware::Packet>();
  Registration ds = state;
  ds.id = 1;
  ds.name = talos::hardware::kDriverStationTopic;
  Registration command = state;
  command.id = 2;
  command.name = talos::hardware::kCommandTopic;
  Registration timer;
  timer.id = 0;
  timer.kind = SourceKind::TIMER;
  timer.name = "motor";
  Registration request = state;
  request.id = 1;
  request.name = talos::hardware::RequestTopic("motor");
  const auto start = MonotonicTime::from_nanos(1'000'000'000);
  using talos::event::log::LogWriter;
  const talos::event::log::LogWriterOptions options{4096, 1, false};
  LogWriter bridge{directory + "/hardware_node.tlog", "hardware_node", options,
                   43};
  LogWriter motor{directory + "/motor.tlog", "motor", options, 43};
  bridge.start({state, ds, command}, start);
  motor.start({timer, request}, start);
  std::uint64_t index = 0;
  auto record = [&](LogWriter& writer, int us, std::uint16_t id, EventKind kind,
                    const talos::hardware::Packet& packet) {
    Context context;
    context.kind = kind;
    context.source_id = id;
    context.dispatch_index = ++index;
    context.now = start + std::chrono::microseconds{us};
    context.event_time = context.now;
    context.publish_time = context.now;
    context.sequence = index;
    if (kind == EventKind::SEND) {
      writer.send(context, id, std::as_bytes(std::span{&packet, 1}), 0, index);
    } else {
      writer.dispatch(context, {});
    }
  };
  talos::driver_station::DriverStationData disabled;
  disabled.fms.flags = talos::driver_station::kDsAttached;
  talos::hardware::Packet ds_packet;
  ds_packet.size = talos::driver_station::Encode(disabled, ds_packet.data);
  record(bridge, 0, 1, EventKind::SEND, ds_packet);
  for (int us = 0; us <= 20'000; us += 5'000) {
    const auto observed_us = static_cast<std::uint64_t>(us);
    record(bridge, us, 0, EventKind::SEND, StatePacket());
    record(motor, us + 1'000, 0, EventKind::TIMER, {});
    record(motor, us + 1'000, 1, EventKind::SEND, CommandPacket(observed_us));
    record(bridge, us + 1'250, 2, EventKind::SEND, CommandPacket(observed_us));
  }
  record(bridge, 41'250, 2, EventKind::SEND, CommandPacket(20'000));
  bridge.flush();
  motor.flush();
  // A gap no mode asked for is a fixture fault, as a failed write is.
  return bridge.incomplete() || motor.incomplete() ? 1 : 0;
}

// The default session's log (see main), as one node "fixture" recorded it;
// false when a write failed or the writer dropped records.
bool RecordFixtureLog(const std::string& directory, bool foreign) {
  using namespace talos::event;
  Registration state;
  state.id = 0;
  state.kind = SourceKind::SENDER;
  state.name = talos::hardware::kStateTopic;
  state.message_bytes = sizeof(talos::hardware::Packet);
  state.schema = talos::schema::Of<talos::hardware::Packet>();
  Registration ds = state;
  ds.id = 1;
  ds.name = talos::hardware::kDriverStationTopic;
  Registration timer;
  timer.id = 2;
  timer.kind = SourceKind::TIMER;
  timer.name = "update";
  Registration target = state;
  target.id = 3;
  target.kind = SourceKind::WATCHER;
  target.name = "/fixture/target/test";
  talos::event::log::LogWriter writer{
      directory + "/fixture.tlog", "fixture",
      talos::event::log::LogWriterOptions{4096, 1, false}, 42};
  writer.start({state, ds, timer, target},
               MonotonicTime::from_nanos(1'000'000'000));
  std::uint64_t index = 0;
  auto record = [&](int ms, std::uint16_t id, EventKind kind,
                    const talos::hardware::Packet& packet) {
    Context context;
    context.kind = kind;
    context.source_id = id;
    context.dispatch_index = ++index;
    context.now = MonotonicTime::from_nanos(1'000'000'000 + ms * 1'000'000);
    context.event_time = context.now - std::chrono::milliseconds{1};
    context.publish_time = context.now - std::chrono::milliseconds{3};
    context.sequence = index;
    if (kind == EventKind::SEND) {
      writer.send(context, id, std::as_bytes(std::span{&packet, 1}), 0, index);
    } else {
      writer.dispatch(context, kind == EventKind::TIMER
                                   ? std::span<const std::byte>{}
                                   : std::as_bytes(std::span{&packet, 1}));
    }
    return context;
  };
  auto driver_station = [&](int ms, bool enabled) {
    talos::driver_station::DriverStationData data;
    data.fms.flags = talos::driver_station::kDsAttached |
                     (enabled ? talos::driver_station::kEnabled : 0);
    talos::hardware::Packet packet;
    packet.size = talos::driver_station::Encode(data, packet.data);
    record(ms, 1, EventKind::SEND, packet);
  };
  // Records 0 to 12, each its own dispatch but the READ (12).
  driver_station(0, true);
  for (int ms : {0, 5, 10, 130, 135}) {
    record(ms, 0, EventKind::SEND, StatePacket(foreign));
  }
  driver_station(140, false);
  for (int ms : {140, 340, 345}) {
    record(ms, 0, EventKind::SEND, StatePacket(foreign));
  }
  const Context lane = record(346, 3, EventKind::MESSAGE, {});
  const Context update = record(347, 2, EventKind::TIMER, {});
  // Target::get selecting that lane inside the update: a READ record with an
  // empty payload in the update's dispatch (its index, now and event time),
  // carrying the selected message's publish time and sequence, 4 ms before.
  Context read = update;
  read.kind = EventKind::READ;
  read.source_id = 3;
  read.publish_time = lane.publish_time;
  read.sequence = lane.sequence;
  writer.dispatch(read, {});
  writer.flush();
  // A gap no mode asked for is a fixture fault, as a failed write is.
  return !writer.incomplete();
}

}  // namespace

// session_report_fixture <directory>
//                        [disabled-bridge | foreign-protocol | log-gap]
//
// foreign-protocol: the default session recorded before manifests named
// their hardware protocol, its States recorded under protocol 7
// (StatePacket); its Driver Station packets still decode.
// log-gap: the default session with its log's records 2 to 5 (the States
// sent at 5, 10, 130 and 135 ms, all while enabled) replaced by the GAP record
// a full writer pool leaves, and its manifest marking the gap as the launcher
// does (log_gap, recorder_failed).
int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) return 1;
  const std::string directory = argv[1];
  std::filesystem::create_directories(directory);
  const std::string mode = argc == 3 ? argv[2] : "";
  if (mode == "disabled-bridge") return DisabledBridgeSession(directory);
  if (!mode.empty() && mode != "foreign-protocol" && mode != "log-gap") {
    return 1;
  }
  const bool foreign = mode == "foreign-protocol";
  const bool gap = mode == "log-gap";
  talos::launcher::SessionManifest manifest;
  manifest.session_id = 42;
  manifest.config_sha256 = "fixture-sha256";
  manifest.end_wall_ns = 1;
  if (foreign) manifest.hardware_protocol_version.reset();
  talos::launcher::NodeProcessInfo node;
  node.name = "fixture";
  node.exit_code = -1;   // a signal ended it
  node.exit_signal = 9;  // SIGKILL
  node.exited = true;
  if (gap) {
    node.log_path = directory + "/fixture.tlog";
    node.log_paths = {node.log_path};
    node.log_gap = true;
    node.failed = true;
  }
  manifest.nodes.push_back(node);
  std::ofstream{directory + "/manifest.json"} << manifest.ToJson();
  if (!RecordFixtureLog(directory, foreign)) return 1;
  if (gap) {
    const std::string log = directory + "/fixture.tlog";
    talos::event::log::CopyWithGap(log, log + ".gapped", 2, 6);
    std::filesystem::rename(log + ".gapped", log);
  }
  return 0;
}
