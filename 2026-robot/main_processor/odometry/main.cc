#include <atomic>
#include <csignal>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include "2026-robot/main_processor/odometry/node.h"
#include "2026-robot/main_processor/odometry/packet.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/reporter.h"

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

namespace {
// This node's identity, written once. `--describe` and the live registry row
// have to name the same node and the same build target, or a viewer cannot tell
// a graph that changed from a graph it is reading two different names for.
constexpr const char* kNodeName = "odometry";
constexpr const char* kNodeTarget = "//2026-robot/main_processor/odometry:node";
}  // namespace

int main(int argc, char** argv) {
  try {
    std::string log_path = "/tmp/odometry.tlog", replay_path;
    int duration_s = 0;
    uint64_t session_id = 0;
    bool simulation = false;
    bool describe = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      // The launcher passes the same flags to every node it starts. This one
      // reads no hardware and no configuration, so --sim and --config are
      // accepted rather than refused; --sim additionally tells the registry
      // which mode the session is running in.
      if (arg == "--sim") {
        simulation = true;
        continue;
      } else if (arg == "--describe") {
        describe = true;
        continue;
      } else if (arg == "--config" && i + 1 < argc)
        ++i;
      else if (arg == "--session-id" && i + 1 < argc)
        session_id = std::stoull(argv[++i]);
      else if (arg == "--log" && i + 1 < argc)
        log_path = argv[++i];
      else if (arg == "--replay" && i + 1 < argc)
        replay_path = argv[++i];
      else if (arg == "--duration-s" && i + 1 < argc)
        duration_s = std::stoi(argv[++i]);
      else
        throw std::invalid_argument(
            "usage: node [--sim] [--describe] [--duration-s N] [--log PATH] "
            "[--session-id ID] [--config PATH] [--replay PATH]");
    }
    if (duration_s < 0)
      throw std::invalid_argument("duration must be nonnegative");

    // --describe builds the node for real and prints what its constructor
    // registered. The loop is the only difference: a simulated loop's channels
    // live in this process, so no shared memory, no hardware and no network is
    // touched, and the launcher can ask what a node is without starting a
    // robot. Describing from the same constructor is the point -- a topology
    // written down a second time is a topology to forget to update.
    if (describe) {
      talos::event::SimulationEnvironment environment;
      talos::event::SimulatedEventLoop<> loop{environment};
      talos::odometry::OdometryNode node{loop};
      std::printf("%s", talos::introspect::DescribeToJson(
                            talos::introspect::DescribeManifest(
                                kNodeName, kNodeTarget, loop.manifest()))
                            .c_str());
      return 0;
    }

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
        talos::event::log::LogWriter{log_path, "odometry",
                                     talos::event::log::LogWriterOptions{},
                                     session_id}};
    talos::odometry::OdometryNode node{loop};
    talos::odometry::InstallStopHandlers();
    // Publishes this node's topics and activity into the live registry, so
    // Studio and any agent can see what is running without being told.
    // Declared after the loop: reverse destruction order joins its thread
    // before the counters it samples are destroyed.
    talos::introspect::Reporter reporter{loop,
                                         {.name = kNodeName,
                                          .target = kNodeTarget,
                                          .session_id = session_id,
                                          .simulation = simulation}};

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
