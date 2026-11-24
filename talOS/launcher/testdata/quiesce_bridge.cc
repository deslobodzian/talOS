// A hardware_node stub that claims a registry row under the launcher's
// --session-id so supervision_test can exercise two-phase shutdown with
// kFlagQuiesced (REQ-OPS-20, REQ-OPS-06).
//
// quiesce_bridge <mode> <ready_file> <events_file> <release_file>
//                <launcher args...>
//
// Appends "hw\n" to <ready_file> once stop handlers and the registry row are
// active. On SIGINT/SIGTERM appends "hw_int\n" to <events_file>:
// - `quiesce`: calls mark_quiesced(), appends "hw_quiesced\n", waits until
//   <release_file> exists (or SIGKILL), appends "hw_exit\n", and exits 0.
// - `hang`: does not call mark_quiesced(), waits until <release_file> exists
//   (or SIGKILL), appends "hw_exit\n", and exits 0.

#include <poll.h>

#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

#include "talOS/introspection/registry.h"

namespace {

volatile std::sig_atomic_t stop = 0;

void OnStop(int /*signal*/) { stop = 1; }

void AppendLine(const std::string& path, std::string_view line) {
  std::ofstream out{path, std::ios::app};
  out << line << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 5) return 2;
  const std::string_view mode = argv[1];
  const std::string ready_file = argv[2];
  const std::string events_file = argv[3];
  const std::string release_file = argv[4];
  std::uint64_t session_id = 0;
  for (int i = 5; i + 1 < argc; ++i) {
    if (std::string_view{argv[i]} == "--session-id") {
      session_id = std::stoull(argv[i + 1]);
    }
  }
  if (session_id == 0) return 2;

  std::signal(SIGINT, OnStop);
  std::signal(SIGTERM, OnStop);

  talos::introspect::NodeRegistration row{{
      .name = "hardware_node",
      .target = "//talOS/bridge:hardware_node",
      .session_id = session_id,
      .flags = 0,
  }};
  row.activate();
  std::uint64_t dispatch = 1;
  const auto sample = [&row](std::uint64_t count) {
    row.begin_sample();
    row.heartbeat();
    row.set_dispatch_count(count);
    row.end_sample();
  };
  sample(dispatch++);
  AppendLine(ready_file, "hw");

  while (stop == 0) {
    sample(dispatch++);
    ::poll(nullptr, 0, 5);
  }

  AppendLine(events_file, "hw_int");
  if (mode == "quiesce") {
    row.mark_quiesced();
    AppendLine(events_file, "hw_quiesced");
  }
  while (!std::filesystem::exists(release_file)) {
    ::poll(nullptr, 0, 5);
  }
  AppendLine(events_file, "hw_exit");
  return 0;
}
