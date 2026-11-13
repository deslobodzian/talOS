// A roster member that only keeps a registry row, carrying the delivery-loss
// counts its command line gives, for supervision_test's check of what the
// launcher records of them.
//
// delivery_loss_node <name> <target> <run>... <launcher arguments>
//
// One <run> per run, the first for the first run (`<log>.tlog`), the next
// for restart 1 (`<log>.restart1.tlog`), and so on: wake_failures,
// stalled_arms, wake_fallbacks, the requests skipped on a watcher of
// `/hw/request/alpha`, then how the run ends, comma separated. `run` runs
// until SIGINT or SIGTERM, then exits 0; `stop` does too, but its row shows
// zeros until the stop, then the counts, in one last sample before it
// releases the row; `exit` exits 3 once the session's manifest.json shows
// wake_failures at this run's (nonzero) count, as the launcher posts it
// when a count first leaves 0.

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/introspection/registry.h"

namespace {

volatile std::sig_atomic_t stop = 0;

void OnStop(int /*signal*/) { stop = 1; }

struct Run {
  std::uint64_t wake_failures{0};
  std::uint64_t stalled_arms{0};
  std::uint64_t wake_fallbacks{0};
  std::uint64_t requests_skipped{0};
  bool exits{false};
  bool late{false};  // The counts appear only once the run is stopped.
};

Run ParseRun(const std::string& spec) {
  std::istringstream in{spec};
  std::vector<std::string> fields;
  for (std::string field; std::getline(in, field, ',');)
    fields.push_back(field);
  if (fields.size() != 5) std::exit(2);
  return {.wake_failures = std::stoull(fields[0]),
          .stalled_arms = std::stoull(fields[1]),
          .wake_fallbacks = std::stoull(fields[2]),
          .requests_skipped = std::stoull(fields[3]),
          .exits = fields[4] == "exit",
          .late = fields[4] == "stop"};
}

// Restart k's event log is `<name>.restart<k>.tlog`; the first run's has no
// restart number.
std::size_t RunIndex(std::string_view log) {
  const auto at = log.rfind(".restart");
  if (at == std::string_view::npos) return 0;
  return std::stoul(std::string{log.substr(at + 8)});
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) return 2;
  const std::string name = argv[1];
  const std::string target = argv[2];
  std::vector<Run> runs;
  int arg = 3;
  for (; arg < argc && !std::string_view{argv[arg]}.starts_with("--"); ++arg)
    runs.push_back(ParseRun(argv[arg]));
  std::uint64_t session_id = 0;
  std::string log;
  for (; arg + 1 < argc; ++arg) {
    const std::string_view flag = argv[arg];
    if (flag == "--session-id") session_id = std::stoull(argv[arg + 1]);
    if (flag == "--log") log = argv[arg + 1];
  }
  const std::size_t index = RunIndex(log);
  if (runs.empty() || log.empty()) return 2;
  const Run run = runs[index < runs.size() ? index : runs.size() - 1];
  std::signal(SIGINT, OnStop);
  std::signal(SIGTERM, OnStop);

  talos::introspect::NodeRegistration row{
      {.name = name, .target = target, .session_id = session_id}};
  if (run.requests_skipped != 0) {
    talos::event::Registration request;
    request.kind = talos::event::SourceKind::WATCHER;
    request.name = "/hw/request/alpha";
    request.message_bytes = 8;
    request.alignment = 8;
    row.publish(talos::event::Manifest{request});
  }
  row.activate();
  const std::string manifest =
      std::filesystem::path{log}.parent_path() / "manifest.json";
  const std::string recorded =
      "\"wake_failures\": " + std::to_string(run.wake_failures);
  const auto give_up =
      std::chrono::steady_clock::now() + std::chrono::seconds{10};
  const Run none{};
  const auto sample = [&row, &run](std::uint64_t dispatch, const Run& shown) {
    row.begin_sample();
    row.heartbeat();
    row.set_dispatch_count(dispatch);
    row.set_wake_loss(shown.wake_failures, shown.stalled_arms,
                      shown.wake_fallbacks);
    if (run.requests_skipped != 0)
      row.set_source(0, dispatch, shown.requests_skipped, dispatch, 0, 0, 0);
    row.end_sample();
  };
  std::uint64_t dispatch = 1;
  for (; stop == 0; ++dispatch) {
    sample(dispatch, run.late ? none : run);
    if (run.exits) {
      std::ifstream in{manifest};
      const std::string text{std::istreambuf_iterator<char>{in}, {}};
      if (text.find(recorded) != std::string::npos) return 3;
      if (std::chrono::steady_clock::now() > give_up) return 4;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  // As a node's Reporter takes its final sample after its loop ends; the
  // row is released on return.
  if (run.late) sample(dispatch, run);
  return 0;
}
