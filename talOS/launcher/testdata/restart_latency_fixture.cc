// Writes one restarted run's event log for restart_latency_test: origin at
// 1 s, its 5 ms tick armed for one period after the origin, run() 1.5 ms
// after the origin (so the first deadline is 6.5 ms), a first tick dispatched
// at 6.6 ms that sends nothing, and a second at 11.6 ms that sends
// /hw/request/drivetrain.
#include <chrono>
#include <cstdint>
#include <span>

#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/packet.h"

int main(int argc, char** argv) {
  if (argc != 2) return 1;
  using namespace talos::event;
  constexpr std::int64_t kOrigin = 1'000'000'000;
  Registration tick;
  tick.id = 0;
  tick.kind = SourceKind::TIMER;
  tick.name = "drivetrain";
  tick.armed = true;
  tick.period_ns = 5'000'000;
  tick.offset_ns = 5'000'000;
  Registration request;
  request.id = 1;
  request.kind = SourceKind::SENDER;
  request.name = "/hw/request/drivetrain";
  request.message_bytes = sizeof(talos::hardware::Packet);
  request.schema = talos::schema::Of<talos::hardware::Packet>();
  talos::event::log::LogWriter writer{
      argv[1], "drivetrain",
      talos::event::log::LogWriterOptions{4096, 1, false}, 7};
  writer.start({tick, request}, MonotonicTime::from_nanos(kOrigin));
  std::uint64_t index = 0;
  auto tick_at = [&](std::int64_t now_ns) {
    Context context;
    context.kind = EventKind::TIMER;
    context.source_id = 0;
    context.dispatch_index = ++index;
    context.now = MonotonicTime::from_nanos(kOrigin + now_ns);
    context.event_time = context.now - std::chrono::microseconds{100};
    writer.dispatch(context, {});
    return context;
  };
  tick_at(6'600'000);
  const Context second = tick_at(11'600'000);
  const talos::hardware::Packet packet{};
  writer.send(second, 1, std::as_bytes(std::span{&packet, 1}), 0, 1);
  writer.flush();
  return writer.failed() ? 1 : 0;
}
