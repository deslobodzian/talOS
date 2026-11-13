#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>

#include "talOS/driver_station/driver_station.h"
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
    record(bridge, us, 0, EventKind::SEND, {});
    record(motor, us + 1'000, 0, EventKind::TIMER, {});
    record(motor, us + 1'000, 1, EventKind::SEND, CommandPacket(observed_us));
    record(bridge, us + 1'250, 2, EventKind::SEND, CommandPacket(observed_us));
  }
  record(bridge, 41'250, 2, EventKind::SEND, CommandPacket(20'000));
  bridge.flush();
  motor.flush();
  return bridge.failed() || motor.failed() ? 1 : 0;
}

}  // namespace

// session_report_fixture <directory> [disabled-bridge]
int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) return 1;
  const std::string directory = argv[1];
  std::filesystem::create_directories(directory);
  if (argc == 3) {
    return std::string{argv[2]} == "disabled-bridge"
               ? DisabledBridgeSession(directory)
               : 1;
  }
  talos::launcher::SessionManifest manifest;
  manifest.session_id = 42;
  manifest.config_sha256 = "fixture-sha256";
  manifest.end_wall_ns = 1;
  talos::launcher::NodeProcessInfo node;
  node.name = "fixture";
  node.exit_code = -1;   // a signal ended it
  node.exit_signal = 9;  // SIGKILL
  node.exited = true;
  manifest.nodes.push_back(node);
  std::ofstream{directory + "/manifest.json"} << manifest.ToJson();
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
  driver_station(0, true);
  for (int ms : {0, 5, 10, 130, 135}) record(ms, 0, EventKind::SEND, {});
  driver_station(140, false);
  for (int ms : {140, 340, 345}) record(ms, 0, EventKind::SEND, {});
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
  return writer.failed() ? 1 : 0;
}
