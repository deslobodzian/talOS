#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include "talOS/configuration/config_parser.h"
#include "talOS/drivetrain/node.h"
#include "talOS/drivetrain/process.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/replay_event_loop.h"

int main(int argc, char** argv) {
  try {
    bool simulation = false;
    std::string log_path = "/tmp/drivetrain.tlog", replay_path;
    std::string config_path = "talOS/configuration/robot.toml";
    int duration_s = 0;
    uint64_t session_id = 0;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--sim")
        simulation = true;
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
    auto robot_config = talos::config::ParseRobotConfig(config_path);
    if (simulation) {
      robot_config.hardware.commissioned = true;
    }
    const auto* dev_ptr = robot_config.GetDevices("drivetrain");
    talos::hardware::Devices devices = dev_ptr ? *dev_ptr : talos::hardware::Devices{};
    auto config = robot_config.hardware;
    if (!replay_path.empty()) {
      talos::event::log::LogReader reader{replay_path};
      talos::event::ReplayEventLoop<> loop{reader};
      talos::drive::DrivetrainNode node{loop, config, devices};
      // Identical to the live setup below: the replay loop's clock already
      // reads the recorded origin, so the same line produces the recorded
      // schedule.
      node.Start(loop.monotonic_now() +
                 std::chrono::microseconds{config.period_us});
      loop.run();
      std::printf("replay: diverged=%d clean_exit=%d\n", loop.diverged(),
                  loop.reached_exit());
      return loop.diverged() || !loop.reached_exit() ? 1 : 0;
    }
    talos::event::RealtimeEventLoop<talos::event::log::LogWriter> loop{
        talos::event::log::LogWriter{log_path, "drivetrain",
                                     talos::event::log::LogWriterOptions{},
                                     session_id}};
    talos::drive::DrivetrainNode node{loop, config, devices};
    node.Start(loop.monotonic_now() +
               std::chrono::microseconds{config.period_us});
    talos::drive::InstallStopHandlers();
    std::jthread stopper{[&](std::stop_token stop) {
      while (!stop.stop_requested()) {
        if (talos::drive::stop_requested.load() && loop.running()) {
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
    std::printf("drivetrain: dispatches=%llu recording_failed=%d\n",
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
