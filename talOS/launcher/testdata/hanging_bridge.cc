// A Studio bridge stub that claims a registry row under the launcher's
// --session-id, sets LoopTier::kObserver, advances dispatch_count and
// heartbeat, and then hangs (stops updating both) on command, for
// supervision_test's check of hung-bridge detection and restart (REQ-OPS-30,
// REQ-OPS-45).
//
// hanging_bridge <runs_file> <hang_file> <mode> <launcher args...>
//
// Appends its pid to <runs_file> once its stop handlers and registry row are
// ready. <mode> is:
// - `once`: run 1 advances until <hang_file> exists, then creates
//   <hang_file>.hung once and freezes (stops updating its row) until SIGKILL;
//   later runs keep advancing until stop.
// - `always`: every run advances twice, records its pid, then freezes until
//   SIGKILL.

#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <thread>

#include "talOS/events/manifest.h"
#include "talOS/introspection/registry.h"
#include "talOS/process/realtime_outcome.h"

namespace {

volatile std::sig_atomic_t stop = 0;

void OnStop(int /*signal*/) { stop = 1; }

std::size_t CountLines(const std::string& path) {
  std::ifstream in{path};
  std::size_t count = 0;
  for (std::string line; std::getline(in, line);) ++count;
  return count;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) return 2;
  const std::string runs_file = argv[1];
  const std::string hang_file = argv[2];
  const std::string_view mode = argv[3];
  std::uint64_t session_id = 0;
  std::string topic = "/talos/telemetry";
  if (argc > 4 && !std::string_view{argv[4]}.starts_with("--")) {
    topic = argv[4];
  }
  for (int i = 4; i + 1 < argc; ++i) {
    if (std::string_view{argv[i]} == "--session-id") {
      session_id = std::stoull(argv[i + 1]);
    }
  }
  if (session_id == 0) return 2;

  std::signal(SIGINT, OnStop);
  std::signal(SIGTERM, OnStop);

  talos::introspect::NodeRegistration row{{
      .name = "studio_bridge",
      .target = "//studio/bridge:studio_bridge",
      .session_id = session_id,
      .flags = 0,
  }};
  talos::event::Registration watcher;
  watcher.id = 0;
  watcher.kind = talos::event::SourceKind::WATCHER;
  watcher.name = topic;
  watcher.message_bytes = 4096;
  watcher.alignment = 8;
  row.publish(talos::event::Manifest{watcher});
  row.set_realtime({.tier = talos::process::LoopTier::kObserver});

  const auto sample = [&row](std::uint64_t dispatch) {
    row.begin_sample();
    row.heartbeat();
    row.set_dispatch_count(dispatch);
    row.end_sample();
  };

  const std::size_t run_number = CountLines(runs_file) + 1;
  std::uint64_t dispatch = 1;
  sample(dispatch++);
  if (mode == "always") {
    sample(dispatch++);
    {
      std::ofstream out{runs_file, std::ios::app};
      out << ::getpid() << '\n';
    }
    while (stop == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    return 0;
  }

  {
    std::ofstream out{runs_file, std::ios::app};
    out << ::getpid() << '\n';
  }
  bool hung_acked = false;
  while (stop == 0) {
    const bool hung = run_number == 1 && std::filesystem::exists(hang_file);
    if (hung) {
      if (!hung_acked) {
        std::ofstream{hang_file + ".hung"}.close();
        hung_acked = true;
      }
    } else {
      sample(dispatch++);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  return 0;
}
