#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

#include "studio/schema/telemetry_generated.h"
#include "talOS/rtms/rtms.h"
#include "studio/schema/wire.h"
namespace {
volatile std::sig_atomic_t stop = 0;
void signal_handler(int) { stop = 1; }
}  // namespace
int main(int argc, char** argv) try {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  RTMSQueue queue(argc > 1 ? argv[1] : "/talos/telemetry", studio::slot_bytes,
                  8, studio::slot_count,
                  {OverflowPolicy::DROP_NEWEST, ReadMode::SEQUENCE});
  std::array<std::byte, studio::slot_bytes> slot{};
  auto next = std::chrono::steady_clock::now();
  for (std::uint64_t seq = 0; !stop; ++seq) {
    flatbuffers::FlatBufferBuilder b;
    const double t = seq * 0.005;
    Talos::Telemetry::Pose pose(8 + 3 * std::cos(t * .25),
                                4 + 2 * std::sin(t * .25), 0, 0, 0,
                                t * .25 + 1.5707963267948966);
    Talos::Telemetry::Pose raw(pose.x() + 0.15 * std::sin(t), pose.y(), 0, 0, 0,
                               pose.yaw());
    auto ghost =
        Talos::Telemetry::CreateGhost(b, b.CreateString("RawOdometry"), &raw);
    auto ghosts = b.CreateVector(std::vector{ghost});
    auto speed = Talos::Telemetry::CreateChannel(
        b, b.CreateString("drive/velocity"), 0.75, b.CreateString("m/s"));
    auto channels = b.CreateVector(std::vector{speed});
    auto frame = Talos::Telemetry::CreateFrame(
        b, seq * 5000000ULL, seq, &pose, ghosts, channels, 0,
        0.5 + 0.4 * std::sin(t), 0.5 * std::sin(t), &pose);
    Talos::Telemetry::FinishFrameBuffer(b, frame);
    auto size = b.GetSize();
    if (size > slot.size() - 4) throw std::runtime_error("frame exceeds slot");
    for (unsigned i = 0; i < 4; ++i)
      slot[i] = std::byte((size >> (8 * i)) & 255);
    std::memcpy(slot.data() + 4, b.GetBufferPointer(), size);
    queue.write({slot.size(), slot.data()});
    next += std::chrono::milliseconds(5);
    std::this_thread::sleep_until(next);
  }
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
