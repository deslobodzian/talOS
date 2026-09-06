#include <atomic>
#include <csignal>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/odometry/node.h"
#include "talOS/odometry/packet.h"

namespace talos::odometry {
inline std::atomic<bool> stop_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free);
inline void RequestStop(int) {
  stop_requested.store(true, std::memory_order_relaxed);
}
inline void InstallStopHandlers() {
  std::signal(SIGINT, RequestStop);
  std::signal(SIGTERM, RequestStop);
}
}  // namespace talos::odometry

int main(int argc, char** argv) {
  try {
    std::string log_path = "/tmp/odometry.tlog", replay_path;
    int duration_s = 0;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--log" && i + 1 < argc)
        log_path = argv[++i];
      else if (arg == "--replay" && i + 1 < argc)
        replay_path = argv[++i];
      else if (arg == "--duration-s" && i + 1 < argc)
        duration_s = std::stoi(argv[++i]);
      else
        throw std::invalid_argument(
            "usage: node [--duration-s N] [--log PATH] [--replay PATH]");
    }
    if (duration_s < 0)
      throw std::invalid_argument("duration must be nonnegative");

    if (!replay_path.empty()) {
      talos::event::log::LogReader reader{replay_path};
      talos::event::ReplayEventLoop<> loop{reader};
      talos::odometry::OdometryNode node{loop};
      loop.run();
      std::printf("replay: diverged=%d clean_exit=%d\n", loop.diverged(),
                  loop.reached_exit());
      return loop.diverged() || !loop.reached_exit() ? 1 : 0;
    }

    talos::event::RealtimeEventLoop<talos::event::log::LogWriter> loop{
        talos::event::log::LogWriter{log_path, "odometry"}};
    talos::odometry::OdometryNode node{loop};
    talos::odometry::InstallStopHandlers();
    std::jthread stopper{[&](std::stop_token stop) {
      while (!stop.stop_requested()) {
        if (talos::odometry::stop_requested.load() && loop.running()) {
          loop.exit();
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
      }
    }};
    if (duration_s)
      loop.run_for(std::chrono::seconds{duration_s});
    else
      loop.run();
    stopper.request_stop();
    std::printf("odometry: dispatches=%llu recording_failed=%d\n",
                static_cast<unsigned long long>(loop.dispatch_count()),
                loop.recorder().failed());
    if (loop.recorder().failed()) {
      std::fprintf(stderr, "%s\n", loop.recorder().error().c_str());
      return 1;
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
