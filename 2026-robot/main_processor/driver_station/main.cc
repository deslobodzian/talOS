#include <atomic>
#include <csignal>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include "talOS/configuration/config_parser.h"
#include "2026-robot/main_processor/driver_station/node.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/replay_event_loop.h"

namespace {
std::atomic<bool> stop_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free);
void RequestStop(int) {
  stop_requested.store(true, std::memory_order_relaxed);
}
void InstallStopHandlers() {
  std::signal(SIGINT, RequestStop);
  std::signal(SIGTERM, RequestStop);
}
}  // namespace

int main(int argc, char** argv) {
  try {
    std::string log_path = "/tmp/driver_station.tlog", replay_path;
    std::string config_path = "2026-robot/main_processor/configuration/robot.toml";
    int duration_s = 0;
    uint64_t session_id = 0;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      // --sim only gates hardware commissioning, and this node owns no
      // hardware, so the launcher's flag is accepted and ignored.
      if (arg == "--sim")
        continue;
      else if (arg == "--log" && i + 1 < argc)
        log_path = argv[++i];
      else if (arg == "--session-id" && i + 1 < argc)
        session_id = std::stoull(argv[++i]);
      else if (arg == "--config" && i + 1 < argc)
        config_path = argv[++i];
      else if (arg == "--replay" && i + 1 < argc)
        replay_path = argv[++i];
      else if (arg == "--duration-s" && i + 1 < argc)
        duration_s = std::stoi(argv[++i]);
      else
        throw std::invalid_argument(
            "usage: node [--sim] [--duration-s N] [--log PATH] [--session-id ID] [--config PATH] [--replay "
            "PATH]");
    }
    if (duration_s < 0)
      throw std::invalid_argument("duration must be nonnegative");
    const auto robot_config = talos::config::ParseRobotConfig(config_path);
    talos::driver_station::TeleopConfig teleop{};
    if (const auto* tbl = robot_config.GetSubsystemTable("driver_station")) {
      teleop.max_linear_mps =
          (*tbl)["max_linear_mps"].value_or(teleop.max_linear_mps);
      teleop.max_angular_radps =
          (*tbl)["max_angular_radps"].value_or(teleop.max_angular_radps);
      teleop.deadband = (*tbl)["deadband"].value_or(teleop.deadband);
      teleop.shooter_target_rps =
          (*tbl)["shooter_target_rps"].value_or(teleop.shooter_target_rps);
    }
    if (!replay_path.empty()) {
      talos::event::log::LogReader reader{replay_path};
      talos::event::ReplayEventLoop<> loop{reader};
      talos::driver_station::DriverStationNode node{loop, teleop};
      node.Start(loop.monotonic_now());
      loop.run();
      std::printf("replay: diverged=%d clean_exit=%d\n", loop.diverged(),
                  loop.reached_exit());
      return loop.diverged() || !loop.reached_exit() ? 1 : 0;
    }
    talos::event::RealtimeEventLoop<talos::event::log::LogWriter> loop{
        talos::event::log::LogWriter{log_path, "driver_station",
                                     talos::event::log::LogWriterOptions{},
                                     session_id}};
    talos::driver_station::DriverStationNode node{loop, teleop};
    node.Start(loop.monotonic_now());
    InstallStopHandlers();
    std::jthread stopper{[&](std::stop_token stop) {
      while (!stop.stop_requested()) {
        if (stop_requested.load() && loop.running()) {
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
    std::printf("driver_station: dispatches=%llu recording_failed=%d\n",
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
