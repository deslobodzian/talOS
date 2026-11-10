#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>

#include "talOS/driver_station/driver_station.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/packet.h"
#include "talOS/launcher/launcher.h"

int main(int argc, char** argv) {
  if (argc != 2) return 1;
  const std::string directory = argv[1];
  std::filesystem::create_directories(directory);
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
