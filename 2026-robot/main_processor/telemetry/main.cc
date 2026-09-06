#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include "2026-robot/main_processor/telemetry/node.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/process/process.h"

int main(int argc, char** argv) {
  try {
    std::string log_path = "/tmp/telemetry.tlog", replay_path;
    std::string config_path =
        "2026-robot/main_processor/configuration/robot.toml";
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
            "usage: node [--sim] [--duration-s N] [--log PATH] [--session-id "
            "ID] [--config PATH] [--replay "
            "PATH]");
    }
    if (duration_s < 0)
      throw std::invalid_argument("duration must be nonnegative");
    const auto robot_config = talos::config::ParseRobotConfig(config_path);
    talos::telemetry::TelemetryConfig telemetry{};
    if (const auto* tbl = robot_config.GetSubsystemTable("telemetry")) {
      telemetry.period_us = (*tbl)["period_us"].value_or(telemetry.period_us);
      telemetry.topic = (*tbl)["topic"].value_or(telemetry.topic);
    }
    if (!replay_path.empty()) {
      talos::event::log::LogReader reader{replay_path};
      talos::event::ReplayEventLoop<> loop{reader};
      talos::telemetry::TelemetryNode node{loop, telemetry};
      node.Start(loop.monotonic_now() +
                 std::chrono::microseconds{telemetry.period_us});
      loop.run();
      std::printf("replay: diverged=%d clean_exit=%d\n", loop.diverged(),
                  loop.reached_exit());
      return loop.diverged() || !loop.reached_exit() ? 1 : 0;
    }
    talos::event::RealtimeEventLoop<talos::event::log::LogWriter> loop{
        talos::event::log::LogWriter{log_path, "telemetry",
                                     talos::event::log::LogWriterOptions{},
                                     session_id}};
    talos::telemetry::TelemetryNode node{loop, telemetry};
    node.Start(loop.monotonic_now() +
               std::chrono::microseconds{telemetry.period_us});
    talos::process::InstallStopHandlers();
    std::jthread stopper{[&](std::stop_token stop) {
      while (!stop.stop_requested()) {
        if (talos::process::stop_requested.load() && loop.running()) {
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
    std::printf(
        "telemetry: dispatches=%llu published=%llu dropped=%llu rejected=%llu "
        "recording_failed=%d\n",
        static_cast<unsigned long long>(loop.dispatch_count()),
        static_cast<unsigned long long>(node.frames_published()),
        static_cast<unsigned long long>(node.frames_dropped()),
        static_cast<unsigned long long>(node.frames_rejected()),
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
