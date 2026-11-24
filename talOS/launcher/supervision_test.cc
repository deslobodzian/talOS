// Launcher supervision: clear failures, isolated sessions, and exit-code
// propagation from critical nodes.
//
// Every session runs in a pid-unique temp directory with shell-stub nodes,
// so no two tests share state, ports, or shared memory. Stubs describe an
// empty graph unless a test explicitly exercises a failed probe.

#include <arpa/inet.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/introspection/lock_file.h"
#include "talOS/launcher/launcher.h"
#include "talOS/launcher/testdata/first_exec.h"
#include "talOS/launcher/testdata/supervision_events.h"
#include "talOS/memory/shared_memory_ptr.h"
#include "talOS/protocol/types.h"
#include "talOS/rtms/process_liveness.h"
#include "talOS/rtms/rtms.h"
#include "talOS/schema/identity.h"

namespace talos::launcher {
namespace {

using namespace std::chrono_literals;

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_supervision_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

// A session that ends on an event, not a duration, sets this: it only
// bounds a hang (docs/TESTING.md §6.3).
constexpr int kHangSeconds = 20;

// Framework-owned fixture config: robot-agnostic, two stub subsystems.
constexpr std::string_view kFixtureToml = R"(
[robot]
name = "delivery_fixture"
period_us = 5000

[subsystems.alpha]
node = "//example/alpha:node"
period_us = 5000

[subsystems.alpha.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.beta]
node = "//example/beta:node"
period_us = 10000

[subsystems.beta.motors.m2]
type = "TalonFX"
bus = "rio"
can_id = 2
)";

// `robot_keys` lands inside the fixture's [robot] table.
std::string WriteFixtureToml(std::string_view tag,
                             std::string_view robot_keys = "") {
  const std::string path = TempPath(tag) + ".toml";
  std::string toml{kFixtureToml};
  const std::string header = "[robot]\n";
  toml.insert(toml.find(header) + header.size(), robot_keys);
  std::ofstream out{path};
  out << toml;
  return path;
}

std::string WriteStub(std::string_view tag, std::string_view body,
                      std::string_view node = "",
                      bool answers_describe = true) {
  const std::string path = TempPath(tag) + ".sh";
  if (node.empty()) node = tag.ends_with("_beta") ? "beta" : "alpha";
  introspect::Description description;
  description.name = std::string{node};
  description.target = node == "hardware_node"
                           ? "//talOS/bridge:hardware_node"
                           : "//example/" + std::string{node} + ":node";
  std::ostringstream out;
  out << "#!/bin/sh\n"
      << "for arg in \"$@\"; do\n"
      << "  if [ \"$arg\" = \"--describe\" ]; then\n";
  if (answers_describe) {
    out << "    cat <<'TALOS_DESCRIBE'\n"
        << introspect::DescribeToJson(description) << "TALOS_DESCRIBE\n"
        << "    exit 0\n";
  } else {
    out << "    exit 1\n";
  }
  out << "  fi\n"
      << "done\n"
      << body;
  return testing::WriteWarmStub(path, out.str());
}

std::string ReadFile(const std::string& path) {
  std::ifstream in{path};
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

LauncherOptions BaseOptions(std::string_view tag,
                            std::string_view robot_keys = "") {
  LauncherOptions options;
  options.session_id = GenerateSessionId();
  options.output_dir = TempPath(tag);
  options.config_path = WriteFixtureToml(tag, robot_keys);
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub(std::string{tag} + "_hardware", "exit 0\n", "hardware_node");
  return options;
}

void MakeAlphaNonCritical(const LauncherOptions& options) {
  auto config = toml::parse_file(options.config_path);
  auto* alpha = config["subsystems"]["alpha"].as_table();
  alpha->erase("motors");
  alpha->insert_or_assign("critical", false);
  std::ofstream{options.config_path} << config;
}

int RunSession(LauncherOptions& options) {
  const int result = Launcher{options}.Run();
  options.output_dir += "/" + std::to_string(options.session_id);
  return result;
}

// A stub body that appends its --log argument to `record` (one line per run),
// then ends with `ending`.
std::string RecordLogArgThen(const std::string& record,
                             std::string_view ending) {
  return "prev=\"\"\n"
         "log=\"\"\n"
         "for arg in \"$@\"; do\n"
         "  if [ \"$prev\" = \"--log\" ]; then log=\"$arg\"; fi\n"
         "  prev=\"$arg\"\n"
         "done\n"
         "echo \"$log\" >> " +
         record + "\n" + std::string{ending};
}

std::vector<std::string> ReadLines(const std::string& path) {
  std::ifstream in{path};
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) lines.push_back(line);
  return lines;
}

// One node's object in manifest.json, from its name to its run history.
std::string ManifestEntry(const std::string& manifest, std::string_view name) {
  const std::size_t start =
      manifest.find("\"name\": \"" + std::string{name} + "\"");
  if (start == std::string::npos) return {};
  // Its run history comes last; its runs repeat keys such as exit_code.
  return manifest.substr(start, manifest.find("\"runs\": [", start) - start);
}

// One node's run history in manifest.json: each run's object, first first.
std::vector<std::string> ManifestRuns(const std::string& manifest,
                                      std::string_view name) {
  const std::size_t node =
      manifest.find("\"name\": \"" + std::string{name} + "\"");
  if (node == std::string::npos) return {};
  const std::size_t start = manifest.find("\"runs\": [", node);
  if (start == std::string::npos) return {};
  const std::string runs =
      manifest.substr(start, manifest.find(']', start) - start);
  std::vector<std::string> out;
  for (std::size_t at = runs.find('{'); at != std::string::npos;
       at = runs.find('{', at + 1)) {
    out.push_back(runs.substr(at, runs.find('}', at) - at + 1));
  }
  return out;
}

// The integer `key` of a member's manifest object (ManifestEntry), or -1.
int64_t EntryInteger(const std::string& entry, std::string_view key) {
  const std::string name = "\"" + std::string{key} + "\": ";
  const std::size_t at = entry.find(name);
  if (at == std::string::npos) return -1;
  return std::strtoll(entry.c_str() + at + name.size(), nullptr, 10);
}

LauncherOptions ConstructionOptions(std::string_view tag, int delay_ms) {
  auto options = BaseOptions(tag);
  options.max_restarts = 0;
  auto config = toml::parse_file(options.config_path);
  config["subsystems"]["alpha"].as_table()->insert_or_assign("construction_ms",
                                                             delay_ms);
  std::ofstream{options.config_path} << config;
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  if (!runfiles) throw std::runtime_error(error);
  const char* construction_node = std::getenv("TALOS_CONSTRUCTION_NODE");
  if (construction_node == nullptr)
    throw std::runtime_error("missing construction node runfile");
  options.binary_overrides["//example/alpha:node"] =
      runfiles->Rlocation(construction_node);
  options.binary_overrides["//example/beta:node"] =
      WriteStub(std::string{tag} + "_beta", "exit 0\n");
  return options;
}

// The drift fixture's /alpha/event is no session topic (no probe declares
// it), so its node creates the segment and, killed for the mismatch, never
// unlinks it: unlinked after the last test, in this test's namespace only.
class DriftTopicCleanup : public ::testing::Environment {
 public:
  void TearDown() override {
    ::shm_unlink(ShmObjectName("/alpha/event").c_str());
  }
};
[[maybe_unused]] ::testing::Environment* const kDriftTopicCleanup =
    ::testing::AddGlobalTestEnvironment(new DriftTopicCleanup);

// alpha is the endpoint drift fixture (testdata/endpoint_drift_node.cc),
// which publishes /alpha/event only live, and only with `drift` set.
LauncherOptions EndpointOptions(std::string_view tag, bool drift) {
  auto options = BaseOptions(tag);
  options.max_restarts = 0;
  options.duration_s = kHangSeconds;
  auto config = toml::parse_file(options.config_path);
  config["subsystems"]["alpha"].as_table()->insert_or_assign("drift",
                                                             drift ? 1 : 0);
  std::ofstream{options.config_path} << config;
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  if (!runfiles) throw std::runtime_error(error);
  const char* node = std::getenv("TALOS_ENDPOINT_DRIFT_NODE");
  if (node == nullptr) throw std::runtime_error("missing drift node runfile");
  options.binary_overrides["//example/alpha:node"] = runfiles->Rlocation(node);
  options.binary_overrides["//example/beta:node"] =
      WriteStub(std::string{tag} + "_beta", "exit 0\n");
  return options;
}

// How long a finished session ran, from its manifest: from just before its
// first spawn (after the probes) to its end.
std::chrono::nanoseconds SessionLength(const std::string& manifest) {
  const auto field = [&manifest](std::string_view key) -> int64_t {
    // Top level: its runs repeat these keys.
    const std::string name = "\n  \"" + std::string{key} + "\": ";
    const std::size_t at = manifest.find(name);
    if (at == std::string::npos) return 0;
    return std::stoll(manifest.substr(at + name.size()));
  };
  return std::chrono::nanoseconds{field("end_wall_ns") -
                                  field("start_wall_ns")};
}

// Whether `manifest` is the running session's: its end is not recorded yet.
bool Running(const std::string& manifest) {
  return manifest.find("\n  \"end_wall_ns\": 0,") != std::string::npos;
}

// RunSession, stopped as an operator stops it (SIGTERM, which the launcher's
// handler takes) once `ready` has held for the running session's manifest
// and one whole supervision pass more has run (woken for it, not waited
// for at the loop's next deadline), so whatever that pass reads
// of a running member the session has recorded. `seen` gets the manifest
// `ready` held for: empty when the session ended first. Set
// options.duration_s to kHangSeconds. `ready` must also wait until every
// member still running handles the stop (a talos node or
// delivery_loss_node once Registered, a shell stub once its trap is set)
// when the test asserts how a run ended: one stopped before that dies of
// the launcher's SIGINT instead of exiting as its handler would.
int RunSessionUntil(LauncherOptions& options,
                    const std::function<bool(const std::string&)>& ready,
                    std::string& seen) {
  // RunSession rewrites options.output_dir on return; not read concurrently.
  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  testing::SupervisionPasses passes;
  std::jthread stopper{[&](const std::stop_token& stop) {
    std::optional<uint64_t> stop_at;
    for (; !stop.stop_requested(); std::this_thread::sleep_for(2ms)) {
      if (!stop_at) {
        std::string text = ReadFile(live);
        if (!Running(text) || !ready(text)) continue;
        seen = std::move(text);
        // Datagrams sent before now are drained here; two more mean the
        // first of them started a pass that has ended.
        stop_at = passes.Started() + 2;
      }
      if (passes.Started() >= *stop_at) {
        ::kill(::getpid(), SIGTERM);
        return;
      }
      // Those passes now, not at the loop's next deadline.
      SupervisionWakeup::Wake();
    }
  }};
  const int status = RunSession(options);
  stopper.request_stop();
  return status;
}

// Whether the registry holds an active row named `name` of `session` that
// is no longer constructing.
bool Registered(std::string_view name, uint64_t session) {
  const auto registry = introspect::RegistryReader::open();
  if (!registry) return false;
  const auto nodes = registry->nodes();
  return std::any_of(nodes.begin(), nodes.end(), [&](const auto& node) {
    return node.name == name && node.session_id == session &&
           (node.flags & introspect::kFlagConstructing) == 0;
  });
}

// Whether the registry holds an active row for `pid` named `name` of `session`
// that is no longer constructing and has published at least one heartbeat and
// dispatch, with `pid` still present in the process table (`kill(pid, 0)`).
// Unlike Registered(), matching `node.pid` against the caller's PID (from the
// new run's own registration line) prevents matching an earlier SIGKILLed run's
// still-ACTIVE row; `kill(pid, 0)` rejects an already-reaped PID (an unreaped
// zombie still answers `kill(pid, 0)`).
bool RegisteredPid(std::string_view name, uint64_t session, pid_t pid) {
  if (pid <= 0 || ::kill(pid, 0) != 0) return false;
  const auto registry = introspect::RegistryReader::open();
  if (!registry) return false;
  const auto nodes = registry->nodes();
  return std::any_of(nodes.begin(), nodes.end(), [&](const auto& node) {
    return node.name == name && node.session_id == session &&
           node.pid == static_cast<uint64_t>(pid) &&
           node.heartbeat_monotonic_ns > 0 && node.dispatch_count > 0 &&
           (node.flags & introspect::kFlagConstructing) == 0;
  });
}

// Parses a positive pid line written by a test fixture, or 0 if incomplete.
pid_t ParsePidLine(std::string_view line) {
  pid_t pid = 0;
  const auto [ptr, ec] =
      std::from_chars(line.data(), line.data() + line.size(), pid);
  if (ec != std::errc{} || ptr != line.data() + line.size() || pid <= 0) {
    return 0;
  }
  return pid;
}

// Waits until `done()` holds, `stop` is requested, or `bound` elapses.
bool WaitFor(const std::stop_token& stop, const std::function<bool()>& done,
             std::chrono::steady_clock::duration bound = std::chrono::seconds{
                 kHangSeconds}) {
  const auto give_up = std::chrono::steady_clock::now() + bound;
  while (!stop.stop_requested() && !done() &&
         std::chrono::steady_clock::now() < give_up)
    std::this_thread::sleep_for(2ms);
  return !stop.stop_requested() && done();
}

// Stop trap paired with kLoopUntilStopped: exits 0 on SIGINT or SIGTERM and
// terminates the background sleep so it cannot linger or hold pipes open.
constexpr std::string_view kStopTrap =
    "trap 'p=${_sleep_pid:-$!}; [ -n \"$p\" ] && kill \"$p\" 2>/dev/null; "
    "exit 0' INT TERM\n";

// A stub body that sets its stop trap (exit 0 on SIGINT or SIGTERM), then
// appends its --log argument to `record`, then runs `then`: a recorded run
// can be stopped without dying of the signal.
std::string TrappedRun(const std::string& record, std::string_view then) {
  return std::string{kStopTrap} + RecordLogArgThen(record, then);
}

// A stub's tail that runs until the session stops it. Only safe after an
// active INT/TERM trap (such as TrappedRun): without a trap, bash's
// wait-and-cooperative-exit can ignore SIGINT when it lands in a child
// sleep's fork window; untrapped stubs must use `exec sleep 86400`.
// Uses background sleep + `wait` instead of `while :; do sleep 0.05; done`
// because macOS `/bin/sh` (bash 3.2) defers trapped signals while foreground
// `sleep` runs/forks, which can miss the 500 ms bridge stop timeout, whereas
// POSIX `wait` is interrupted immediately by a trapped signal.
constexpr std::string_view kLoopUntilStopped =
    "while :; do sleep 86400 </dev/null >/dev/null 2>&1 & _sleep_pid=$!; "
    "wait $_sleep_pid; done\n";

// A run whose live endpoints differ from its probe's is reported, recorded
// in the manifest, and stops the session (exit 1) on its own, before its
// duration; --allow-graph-errors records it and the session runs on until
// it is stopped, alpha's run ending with that stop, not the mismatch.
// REQ-CONTRACT-11: live endpoints differing from the probe stop the session
TEST(Supervision, LiveEndpointsThatDifferFromTheProbeStopTheSession) {
  auto options = EndpointOptions("endpoint_drift", true);
  EXPECT_EQ(RunSession(options), 1);
  auto manifest = ReadFile(options.output_dir + "/manifest.json");
  EXPECT_LT(SessionLength(manifest), std::chrono::seconds{kHangSeconds});
  auto alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"endpoint_mismatch\": true"), std::string::npos)
      << alpha;
  std::filesystem::remove_all(options.output_dir);

  auto allowed = EndpointOptions("endpoint_drift_allowed", true);
  allowed.allow_graph_errors = true;
  allowed.simulation = true;  // The flag requires --sim.
  std::string seen;
  // Its exit status is alpha's, which a slow log sync at shutdown can turn
  // into a SIGKILL under disk load.
  EXPECT_NE(RunSessionUntil(
                allowed,
                [](const std::string& live) {
                  return ManifestEntry(live, "alpha")
                             .find("\"endpoint_mismatch\": true") !=
                         std::string::npos;
                },
                seen),
            1);
  ASSERT_FALSE(seen.empty()) << "no running session recorded the mismatch";
  manifest = ReadFile(allowed.output_dir + "/manifest.json");
  alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"endpoint_mismatch\": true"), std::string::npos)
      << alpha;
  const auto runs = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(runs.size(), 1u) << manifest;
  // Ended by the session's stop, not by the mismatch: "stop", or
  // "stop_timeout" when that slow sync outlasted the stop timeout.
  EXPECT_NE(runs[0].find("\"reason\": \"stop"), std::string::npos) << runs[0];
  std::filesystem::remove_all(allowed.output_dir);
}

// A non-critical member's mismatch costs only that member: it is killed and,
// though restarts are enabled, not restarted (no retry pending in the
// manifest that records its kill), while the session (beta, which runs
// until stopped) runs on until it is stopped.
TEST(Supervision, LiveEndpointMismatchStopsOnlyANonCriticalMember) {
  auto options = EndpointOptions("endpoint_drift_optional", true);
  MakeAlphaNonCritical(options);
  options.max_restarts = 5;
  options.binary_overrides["//example/beta:node"] =
      WriteStub("endpoint_drift_optional_beta", "exec sleep 30\n");
  std::string seen;
  // 1 is a session the mismatch stopped (Launcher::CheckEndpoints).
  EXPECT_NE(
      RunSessionUntil(
          options,
          [](const std::string& live) {
            return ManifestEntry(live, "alpha").find("\"exit_signal\": 9") !=
                   std::string::npos;
          },
          seen),
      1);
  ASSERT_FALSE(seen.empty()) << "no running session recorded alpha's kill";
  const auto killed = ManifestEntry(seen, "alpha");
  EXPECT_NE(killed.find("\"retry_at_monotonic_ns\": 0"), std::string::npos)
      << killed;
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"endpoint_mismatch\": true"), std::string::npos)
      << alpha;
  EXPECT_NE(alpha.find("\"exit_signal\": 9"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"restart_count\": 0"), std::string::npos) << alpha;
  const auto beta = ManifestRuns(manifest, "beta");
  ASSERT_EQ(beta.size(), 1u) << manifest;
  EXPECT_NE(beta[0].find("\"reason\": \"stop\""), std::string::npos) << beta[0];
  std::filesystem::remove_all(options.output_dir);
}

// The same node without drift: its live ends (a publication and a
// latest-only subscription, which the loop flags and the probe does not)
// compare equal to its probe's. The session is stopped once a whole
// supervision pass has run after alpha's row was constructed, the pass that
// compares them (Launcher::WatchRun), and nothing was reported.
TEST(Supervision, LiveEndpointsThatMatchTheProbeAreNotReported) {
  auto options = EndpointOptions("endpoint_match", false);
  const uint64_t session = options.session_id;
  std::string seen;
  RunSessionUntil(
      options,
      [session](const std::string&) { return Registered("alpha", session); },
      seen);
  ASSERT_FALSE(seen.empty()) << "alpha's row was never constructed";
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"endpoint_mismatch\": false"), std::string::npos)
      << alpha;
  std::filesystem::remove_all(options.output_dir);
}

// A real constructor that never returns is killed at the construction
// deadline, not as stalled: its reporting thread's construction heartbeat
// keeps it alive past the 2 s grace, and the launcher reads that heartbeat
// and the constructing flag from its registry row until the 3 s deadline.
// The deadline's arithmetic is SupervisionPolicy's; this is the real kill,
// stepped on the supervision clock so host load cannot race registration or
// the grace window.
// REQ-OPS-05
TEST(Supervision, HeartbeatingConstructorStillHasAFiniteDeadline) {
  auto options = ConstructionOptions("hung_construction", 60000);
  options.heartbeat_startup_grace_ms = 2000;
  options.construction_timeout = 3000ms;
  options.duration_s = kHangSeconds;
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  options.supervision_clock = clock.clock();
  const uint64_t session = options.session_id;
  testing::SupervisionPasses passes;
  bool observed_constructing = false;
  bool alive_past_grace = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    const auto constructing_heartbeat = [&]() -> std::optional<int64_t> {
      const auto registry = introspect::RegistryReader::open();
      if (!registry) return std::nullopt;
      for (const auto& node : registry->nodes()) {
        if (node.name == "alpha" && node.session_id == session &&
            (node.flags & introspect::kFlagConstructing) != 0 &&
            ::kill(static_cast<pid_t>(node.pid), 0) == 0) {
          return node.heartbeat_monotonic_ns;
        }
      }
      return std::nullopt;
    };
    const auto observe_pass = [&] {
      const uint64_t next = passes.Started() + 2;
      return WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= next;
      });
    };
    // Wait for alpha's constructing row AND a supervision pass at `start` so
    // Supervise() has recorded `started_at_ == start` before advancing the
    // clock.
    if (!WaitFor(stop, [&] { return constructing_heartbeat().has_value(); }) ||
        !observe_pass()) {
      return;
    }
    // Hold inside the 2 s startup grace (1800 ms), then wait for the
    // reporting thread's construction heartbeat to advance and observe a pass
    // at 1800 ms so progress_changed_at_ == start + 1800 ms.
    clock.HoldAt(start + 1800ms);
    if (!observe_pass()) return;
    const auto beat_at_1800 = constructing_heartbeat();
    if (!beat_at_1800) return;
    if (!WaitFor(stop,
                 [&] {
                   const auto beat = constructing_heartbeat();
                   return beat && *beat > *beat_at_1800;
                 }) ||
        !observe_pass()) {
      return;
    }
    observed_constructing = true;
    // Step across the 2 s grace in a < kMinHeartbeatTimeout (500 ms) increment
    // (2200 ms - 1800 ms = 400 ms < 500 ms): the constructing heartbeat keeps
    // alpha alive past grace without tripping stalled.
    clock.HoldAt(start + 2200ms);
    if (!observe_pass()) return;
    alive_past_grace = constructing_heartbeat().has_value();
    // Step to the 3 s construction deadline: alpha is killed for
    // construction_deadline, never stalled. Keep the clock held until the
    // session exits so ReleaseOnExit cannot snap time back before the pass.
    clock.HoldAt(start + options.construction_timeout);
    WaitFor(stop, [&] {
      SupervisionWakeup::Wake();
      return false;
    });
  }};
  EXPECT_NE(RunSession(options), 0);
  driver.request_stop();
  driver.join();
  EXPECT_TRUE(observed_constructing)
      << "alpha never published a moving construction heartbeat";
  EXPECT_TRUE(alive_past_grace)
      << "alpha did not stay alive past startup grace";
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 0"), std::string::npos) << alpha;
  const auto history = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(history.size(), 1u) << manifest;
  EXPECT_NE(history[0].find("\"reason\": \"construction_deadline\""),
            std::string::npos)
      << history[0];
  std::filesystem::remove_all(options.output_dir);
}

TEST(Supervision, ConstructorFailureExitsNonzeroAndIsReported) {
  auto options = ConstructionOptions("failed_construction", -1);
  options.duration_s = kHangSeconds;
  EXPECT_NE(RunSession(options), 0);
  const auto entry =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"), "alpha");
  EXPECT_NE(entry.find("\"exit_code\": 1"), std::string::npos) << entry;
  std::filesystem::remove_all(options.output_dir);
}

// A run whose event log stops early is in the live manifest while it still
// runs, not first at shutdown: its reporter flags the registry row and the
// launcher records it. A 2-byte log_max_bytes leaves this session 1 byte, so
// each logging member gets the 1-byte minimum share and the real node's log
// is capped before its header; a cap is log_capped, not recorder_failed.
TEST(Supervision, StoppedEventLogReachesTheLiveManifest) {
  auto options = ConstructionOptions("stopped_log", 0);
  auto config = toml::parse_file(options.config_path);
  config["robot"].as_table()->insert_or_assign("log_max_bytes", 2);
  std::ofstream{options.config_path} << config;
  options.duration_s = kHangSeconds;
  std::string seen;
  EXPECT_EQ(
      RunSessionUntil(
          options,
          [](const std::string& live) {
            return ManifestEntry(live, "alpha").find("\"log_capped\": true") !=
                   std::string::npos;
          },
          seen),
      0);
  ASSERT_FALSE(seen.empty()) << "no live manifest recorded the capped log";
  const auto live = ManifestEntry(seen, "alpha");
  EXPECT_NE(live.find("\"recorder_failed\": false"), std::string::npos) << live;
  // The real node kept no bytes: a 1-byte share admits no record or prefix
  // of one, in any file of its event log, segments included.
  std::uintmax_t kept = 0;
  for (const auto& file :
       std::filesystem::directory_iterator{options.output_dir}) {
    if (file.path().filename().string().starts_with("alpha.tlog"))
      kept += file.file_size();
  }
  EXPECT_LE(kept, 1u) << "alpha's event log kept more than its share";
  std::filesystem::remove_all(options.output_dir);
}

// What each run's registry row says it lost reaches the manifest per run and
// summed per node, and outlives the processes. alpha's first run shows lost
// wakes, waits until the live manifest carries them (posted when a count
// first leaves 0) and crashes; its restart shows other nonzero counts, from
// its own row, so only a sum over both runs gives alpha's totals (not the
// first run's, the last's or the larger). The bridge's row shows requests
// skipped on alpha's request topic. beta loses nothing and records zeros.
// The session stops once the manifest carries the restart's counts and the
// bridge's, and a pass has read beta's row. The stubs
// (testdata/delivery_loss_node.cc) declare no endpoints but the bridge
// publishes its request watcher, so the session runs with
// --allow-graph-errors.
// REQ-OPS-43, REQ-OPS-16, REQ-RT-19
TEST(Supervision, DeliveryLossOfEachRunReachesTheManifest) {
  auto options = BaseOptions("delivery_loss");
  options.allow_graph_errors = true;
  options.simulation = true;  // The flag requires --sim.
  options.duration_s = kHangSeconds;
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  ASSERT_NE(runfiles, nullptr) << error;
  const char* stub = std::getenv("TALOS_DELIVERY_LOSS_NODE");
  ASSERT_NE(stub, nullptr);
  const std::string binary = runfiles->Rlocation(stub);
  // Each run: wake_failures, stalled_arms, wake_fallbacks, requests skipped
  // on /hw/request/alpha, and how it ends.
  const auto member = [&](std::string_view name, std::string_view target,
                          std::string_view runs) {
    return WriteStub("delivery_loss_" + std::string{name},
                     "exec " + binary + " " + std::string{name} + " " +
                         std::string{target} + " " + std::string{runs} +
                         " \"$@\"\n",
                     name);
  };
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      member("hardware_node", "//talOS/bridge:hardware_node", "0,0,0,4,run");
  options.binary_overrides["//example/alpha:node"] =
      member("alpha", "//example/alpha:node", "3,1,2,0,exit 5,4,7,0,run");
  options.binary_overrides["//example/beta:node"] =
      member("beta", "//example/beta:node", "0,0,0,0,run");
  const uint64_t session = options.session_id;
  std::string seen;
  EXPECT_EQ(RunSessionUntil(
                options,
                [session](const std::string& live) {
                  const auto runs = ManifestRuns(live, "alpha");
                  return runs.size() == 2 &&
                         runs[1].find("\"wake_failures\": 5,") !=
                             std::string::npos &&
                         ManifestEntry(live, "hardware_node")
                                 .find("\"requests_skipped\": 4,\n") !=
                             std::string::npos &&
                         Registered("beta", session);
                },
                seen),
            0);
  ASSERT_FALSE(seen.empty()) << "no live manifest carried every count";
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");

  const auto alpha = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(alpha.size(), 2u) << manifest;
  EXPECT_NE(
      alpha[0].find("\"reason\": \"crash\", \"stopped_by_session\": false, "
                    "\"wake_failures\": 3, "
                    "\"stalled_arms\": 1, \"wake_fallbacks\": 2, "
                    "\"requests_skipped\": 0}"),
      std::string::npos)
      << "the crashed run keeps its last counts: " << alpha[0];
  EXPECT_NE(alpha[1].find("\"wake_failures\": 5, \"stalled_arms\": 4, "
                          "\"wake_fallbacks\": 7, \"requests_skipped\": 0}"),
            std::string::npos)
      << "the restart counts from its own row: " << alpha[1];
  const auto alpha_entry = ManifestEntry(manifest, "alpha");
  for (const char* sum : {"\"wake_failures\": 8,\n", "\"stalled_arms\": 5,\n",
                          "\"wake_fallbacks\": 9,\n"}) {
    EXPECT_NE(alpha_entry.find(sum), std::string::npos)
        << sum << " in " << alpha_entry;
  }

  const auto beta = ManifestEntry(manifest, "beta");
  for (const char* zero :
       {"\"wake_failures\": 0,\n", "\"stalled_arms\": 0,\n",
        "\"wake_fallbacks\": 0,\n", "\"requests_skipped\": 0,\n"}) {
    EXPECT_NE(beta.find(zero), std::string::npos) << zero << " in " << beta;
  }

  const auto bridge = ManifestEntry(manifest, "hardware_node");
  EXPECT_NE(bridge.find("\"requests_skipped\": 4,\n"), std::string::npos)
      << bridge;
  EXPECT_NE(bridge.find("\"requests_skipped_by_subsystem\": {\"alpha\": 4}"),
            std::string::npos)
      << bridge;
  std::filesystem::remove_all(options.output_dir);
}

// What a run loses while the session stops still reaches the final
// manifest (TODO.md RT-7): the supervision pass stops reading rows for
// liveness once the session is stopping, so the launcher reads each run's
// row once more when it reaps the run. alpha and the bridge stubs show
// zeros until they are stopped, then store their counts in one last sample
// and release their rows as they exit, as a node's Reporter and the bridge
// do; the bridge's skipped requests survive that release.
// REQ-OPS-43
TEST(Supervision, DeliveryLossDuringTheStopReachesTheFinalManifest) {
  auto options = BaseOptions("stop_loss");
  options.allow_graph_errors = true;  // The bridge stub's request watcher.
  options.simulation = true;          // The flag requires --sim.
  options.duration_s = kHangSeconds;
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  ASSERT_NE(runfiles, nullptr) << error;
  const char* stub = std::getenv("TALOS_DELIVERY_LOSS_NODE");
  ASSERT_NE(stub, nullptr);
  const std::string binary = runfiles->Rlocation(stub);
  const auto member = [&](std::string_view name, std::string_view target,
                          std::string_view run) {
    return WriteStub("stop_loss_" + std::string{name},
                     "exec " + binary + " " + std::string{name} + " " +
                         std::string{target} + " " + std::string{run} +
                         " \"$@\"\n",
                     name);
  };
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      member("hardware_node", "//talOS/bridge:hardware_node", "0,0,0,6,stop");
  options.binary_overrides["//example/alpha:node"] =
      member("alpha", "//example/alpha:node", "2,1,3,0,stop");
  options.binary_overrides["//example/beta:node"] =
      member("beta", "//example/beta:node", "0,0,0,0,run");
  const uint64_t session = options.session_id;
  std::string seen;
  EXPECT_EQ(RunSessionUntil(
                options,
                [session](const std::string&) {
                  // beta too: every member ends as its stop handler has
                  // it (RunSessionUntil).
                  return Registered("alpha", session) &&
                         Registered("hardware_node", session) &&
                         Registered("beta", session);
                },
                seen),
            0);
  ASSERT_FALSE(seen.empty()) << "a stub never registered";
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");

  const auto alpha = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(alpha.size(), 1u) << manifest;
  EXPECT_NE(
      alpha[0].find("\"reason\": \"stop\", \"stopped_by_session\": false, "
                    "\"wake_failures\": 2, "
                    "\"stalled_arms\": 1, \"wake_fallbacks\": 3, "
                    "\"requests_skipped\": 0}"),
      std::string::npos)
      << "the run's counts as it ended: " << alpha[0];
  const auto bridge = ManifestEntry(manifest, "hardware_node");
  EXPECT_NE(bridge.find("\"requests_skipped_by_subsystem\": {\"alpha\": 6}"),
            std::string::npos)
      << bridge;
  std::filesystem::remove_all(options.output_dir);
}

// A run killed during a log gap leaves no GAP record, so its registry row
// is the only signal: the launcher marks the member log_gap and
// recorder_failed from the row's kFlagLogGap and names the run's log in one
// line. alpha's row shows the bit while it runs (a supervision pass reads
// it); beta's first run sets it in one sample and exits at once, so the
// launcher reads it from the row the run left when it reaps it; its restart
// shows none and is not named. Neither stub writes a log.
// REQ-OPS-52
TEST(Supervision, ALogGapInTheRowMarksTheMemberThoughItsLogHasNoGapRecord) {
  auto options = BaseOptions("row_gap");
  options.simulation = true;
  options.duration_s = kHangSeconds;
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  ASSERT_NE(runfiles, nullptr) << error;
  const char* stub = std::getenv("TALOS_DELIVERY_LOSS_NODE");
  ASSERT_NE(stub, nullptr);
  const std::string binary = runfiles->Rlocation(stub);
  const auto member = [&](std::string_view name, std::string_view target,
                          std::string_view runs) {
    return WriteStub("row_gap_" + std::string{name},
                     "exec " + binary + " " + std::string{name} + " " +
                         std::string{target} + " " + std::string{runs} +
                         " \"$@\"\n",
                     name);
  };
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      member("hardware_node", "//talOS/bridge:hardware_node", "0,0,0,0,run");
  options.binary_overrides["//example/alpha:node"] =
      member("alpha", "//example/alpha:node", "0,0,0,0,gap");
  options.binary_overrides["//example/beta:node"] =
      member("beta", "//example/beta:node", "0,0,0,0,gapexit 0,0,0,0,run");
  std::string seen;
  ::testing::internal::CaptureStderr();
  const int status = RunSessionUntil(
      options,
      [](const std::string& live) {
        return ManifestEntry(live, "alpha").find("\"log_gap\": true") !=
                   std::string::npos &&
               ManifestRuns(live, "beta").size() == 2 &&
               ManifestEntry(live, "beta").find("\"log_gap\": true") !=
                   std::string::npos;
      },
      seen);
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(status, 0);
  ASSERT_FALSE(seen.empty()) << "no live manifest carried both gaps";
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  for (const char* name : {"alpha", "beta"}) {
    const auto entry = ManifestEntry(manifest, name);
    EXPECT_NE(entry.find("\"log_gap\": true"), std::string::npos) << entry;
    EXPECT_NE(entry.find("\"recorder_failed\": true"), std::string::npos)
        << entry;
    EXPECT_NE(entry.find("\"recorder_failed_logs\": []"), std::string::npos)
        << entry;
    const std::string expected_gap =
        "\"gap_logs\": [\"" + options.output_dir + "/" + name + ".tlog\"]";
    EXPECT_NE(entry.find(expected_gap), std::string::npos) << entry;
  }
  std::vector<std::string> named;
  std::istringstream text{stderr_text};
  for (std::string line; std::getline(text, line);) {
    if (line.find("lost records") == std::string::npos) continue;
    const std::size_t at = line.rfind('/');
    named.push_back(at == std::string::npos
                        ? line
                        : line.substr(at + 1, line.find(' ', at) - at - 1));
  }
  std::sort(named.begin(), named.end());
  EXPECT_EQ(named, (std::vector<std::string>{"alpha.tlog", "beta.tlog"}))
      << stderr_text;
  for (const char* log : {"/alpha.tlog", "/beta.tlog"}) {
    EXPECT_FALSE(std::filesystem::exists(options.output_dir + log)) << log;
  }
  std::filesystem::remove_all(options.output_dir);
}

// Run 0 and run 1 of a restarted node both suffer live kFlagRecorderFailed
// (their --log paths are directories so LogWriter fails with EISDIR). Both
// logs reach recorder_failed_logs in the live manifest (NewRun clears the
// per-run recorder failure tracker independently of recording_reported),
// while the stderr warning is still emitted once per member.
// REQ-OPS-14
TEST(Supervision, RecorderFailedLogsTrackEveryRunAcrossRestarts) {
  auto options = ConstructionOptions("rec_failed_runs", 0);
  options.simulation = true;
  options.max_restarts = 2;
  options.duration_s = kHangSeconds;
  std::string rf_error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &rf_error)};
  ASSERT_NE(runfiles, nullptr) << rf_error;
  const char* stub = std::getenv("TALOS_DELIVERY_LOSS_NODE");
  ASSERT_NE(stub, nullptr);
  const std::string dl_binary = runfiles->Rlocation(stub);
  const auto keeper = [&](std::string_view name, std::string_view target) {
    return WriteStub("rec_failed_" + std::string{name},
                     "exec " + dl_binary + " " + std::string{name} + " " +
                         std::string{target} + " 0,0,0,0,run \"$@\"\n",
                     name);
  };
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      keeper("hardware_node", "//talOS/bridge:hardware_node");
  options.binary_overrides["//example/beta:node"] =
      keeper("beta", "//example/beta:node");
  const std::string real_node =
      options.binary_overrides["//example/alpha:node"];
  const std::string wrapper = TempPath("rec_failed_alpha") + ".sh";
  {
    std::ofstream out{wrapper};
    out << "#!/bin/sh\n"
        << "for arg in \"$@\"; do\n"
        << "  if [ \"$arg\" = \"--describe\" ]; then\n"
        << "    exec \"" << real_node << "\" \"$@\"\n"
        << "  fi\n"
        << "done\n"
        << "log=\"\"\n"
        << "prev=\"\"\n"
        << "for arg in \"$@\"; do\n"
        << "  if [ \"$prev\" = \"--log\" ]; then log=\"$arg\"; break; fi\n"
        << "  prev=\"$arg\"\n"
        << "done\n"
        << "mkdir -p \"$log\"\n"
        << "case \"$log\" in\n"
        << "  *.restart*)\n"
        << "    exec \"" << real_node << "\" \"$@\"\n"
        << "    ;;\n"
        << "  *)\n"
        << "    parent=$$\n"
        << "    manifest=\"$(dirname \"$log\")/manifest.json\"\n"
        << "    (\n"
        << "      i=0\n"
        << "      while [ $i -lt 200 ]; do\n"
        << "        if grep -q '\"recorder_failed_logs\": \\[\"' "
           "\"$manifest\" 2>/dev/null; then\n"
        << "          kill -9 $parent 2>/dev/null\n"
        << "          exit 0\n"
        << "        fi\n"
        << "        sleep 0.02\n"
        << "        i=$((i + 1))\n"
        << "      done\n"
        << "    ) &\n"
        << "    exec \"" << real_node << "\" \"$@\"\n"
        << "    ;;\n"
        << "esac\n";
  }
  ::chmod(wrapper.c_str(), 0755);
  options.binary_overrides["//example/alpha:node"] = wrapper;

  std::string seen;
  ::testing::internal::CaptureStderr();
  const int status = RunSessionUntil(
      options,
      [](const std::string& live) {
        const std::string alpha = ManifestEntry(live, "alpha");
        const std::size_t key = alpha.find("\"recorder_failed_logs\": [");
        if (key == std::string::npos) return false;
        const std::size_t end = alpha.find(']', key);
        if (end == std::string::npos) return false;
        const std::string arr = alpha.substr(key, end - key + 1);
        return arr.find("alpha.tlog") != std::string::npos &&
               arr.find("alpha.restart1.tlog") != std::string::npos;
      },
      seen);
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  // alpha's run 1 exits 1 because its event log failed (incomplete()).
  EXPECT_EQ(status, 1);
  ASSERT_FALSE(seen.empty())
      << "live manifest never recorded both runs in recorder_failed_logs";
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha = ManifestEntry(manifest, "alpha");
  const std::string expected = "\"recorder_failed_logs\": [\"" +
                               options.output_dir + "/alpha.tlog\", \"" +
                               options.output_dir + "/alpha.restart1.tlog\"]";
  EXPECT_NE(alpha.find(expected), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"recorder_failed\": true"), std::string::npos)
      << alpha;
  // Per-member stderr-once preserved across runs.
  std::size_t warnings = 0;
  for (std::size_t pos = 0;
       (pos = stderr_text.find("failed; the rest of this run is not recorded",
                               pos)) != std::string::npos;
       ++warnings, ++pos) {
  }
  EXPECT_EQ(warnings, 1u) << stderr_text;
  std::filesystem::remove_all(options.output_dir);
  ::remove(wrapper.c_str());
}

// A run that ends before it claims a registry row has no counts of its own,
// however a row left in the session by an earlier run whose pid it reuses
// reads: the launcher's read at the reap takes only a row claimed since the
// run was spawned. alpha's one run never registers; while it runs, the test
// leaves a released row of this session carrying alpha's pid, lost wakes,
// and a heartbeat from before the session, as an earlier run of the same
// pid would, then lets alpha exit.
// REQ-OPS-43
TEST(Supervision, AnEarlierRunsRowWithTheSamePidIsNotTheRunsCounts) {
  auto options = BaseOptions("stale_row");
  options.max_restarts = 0;
  options.duration_s = kHangSeconds;
  const auto pid_file = TempPath("stale_row_pid");
  const auto go_file = TempPath("stale_row_go");
  ::remove(pid_file.c_str());
  ::remove(go_file.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "stale_row_alpha", "echo $$ > " + pid_file + "\nwhile [ ! -e " + go_file +
                             " ]; do sleep 0.01; done\nexit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("stale_row_beta", "exit 0\n");
  const int64_t before = introspect::monotonic_now_ns();
  std::atomic<bool> planted{false};
  std::jthread plant{[&, session_id =
                             options.session_id](const std::stop_token& stop) {
    pid_t pid = 0;
    while (!stop.stop_requested()) {
      std::ifstream input{pid_file};
      if (input >> pid) break;
      std::this_thread::sleep_for(5ms);
    }
    if (pid <= 0) return;
    {
      introspect::NodeRegistration registration{
          {"alpha", "//example/alpha:node", session_id, 0}};
      introspect::RegistryMapping mapping;
      for (auto& row : mapping.header()->nodes) {
        if (row.state.load(std::memory_order_acquire) !=
                static_cast<uint64_t>(introspect::SlotState::ACTIVE) ||
            row.session_id != session_id ||
            row.pid != static_cast<uint64_t>(::getpid()))
          continue;
        row.state.store(static_cast<uint64_t>(introspect::SlotState::CLAIMING),
                        std::memory_order_release);
        row.pid = static_cast<uint64_t>(pid);
        row.pid_start = 0;
        row.heartbeat_monotonic_ns.store(before);
        row.wake_failures.store(3);
        row.stalled_arms.store(1);
        row.wake_fallbacks.store(2);
        row.state.store(static_cast<uint64_t>(introspect::SlotState::FREE),
                        std::memory_order_release);
        planted = true;
        break;
      }
    }
    std::ofstream{go_file} << "go\n";
  }};
  RunSession(options);
  // alpha exits only once the row is planted; a run never spawned ends the
  // wait for its pid.
  plant.request_stop();
  plant.join();
  ASSERT_TRUE(planted) << "no row to leave behind";
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(alpha.size(), 1u) << manifest;
  const auto alpha_entry = ManifestEntry(manifest, "alpha");
  for (const char* key :
       {"\"wake_failures\"", "\"stalled_arms\"", "\"wake_fallbacks\"",
        "\"requests_skipped\"", "\"requests_skipped_by_subsystem\""}) {
    EXPECT_EQ(alpha_entry.find(key), std::string::npos)
        << "unseen row must omit " << key << " on node: " << alpha_entry;
    EXPECT_EQ(alpha[0].find(key), std::string::npos)
        << "unseen row must omit " << key << " on run: " << alpha[0];
  }
  std::filesystem::remove_all(options.output_dir);
}

// The same row left active, as a SIGKILLed run leaves it until the next
// claim frees it, earlier in the registry than the row the run that reuses
// its pid claims late (a Python node's interpreter start). The supervision
// pass's read must take the run's own row, never one older than the run's
// spawn: the earlier row's dispatch count never moves, so taking it charges
// the run the earlier run's counts and kills it as stalled once the startup
// grace (5 s) is over, though its own row shows progress at every look. The
// test plays both rows and moves the supervision clock 400 ms (inside the
// 500 ms heartbeat timeout) per step, with progress in the run's row each
// step, until the grace is well over; then alpha exits.
// REQ-OPS-05, REQ-OPS-43
TEST(Supervision, AnEarlierRunsActiveRowIsNotTheRunsRow) {
  auto options = BaseOptions("stale_active_row");
  options.max_restarts = 0;
  options.duration_s = kHangSeconds;
  testing::SteppedClock clock;
  options.supervision_clock = clock.clock();
  const testing::SteppedClock::ReleaseOnExit release{clock};
  const auto pid_file = TempPath("stale_active_row_pid");
  const auto go_file = TempPath("stale_active_row_go");
  ::remove(pid_file.c_str());
  ::remove(go_file.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "stale_active_row_alpha",
      "echo $$ > " + pid_file + ".tmp && mv " + pid_file + ".tmp " + pid_file +
          "\nwhile [ ! -e " + go_file + " ]; do sleep 0.01; done\nexit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("stale_active_row_beta", "exit 0\n");
  const int64_t before = introspect::monotonic_now_ns();
  testing::SupervisionPasses passes;
  bool planted = false;
  bool passed = false;
  std::jthread driver{[&, session_id =
                              options.session_id](const std::stop_token& stop) {
    const auto give_up = std::chrono::steady_clock::now() + 20s;
    pid_t pid = 0;
    while (!stop.stop_requested() &&
           std::chrono::steady_clock::now() < give_up) {
      std::ifstream input{pid_file};
      if (input >> pid) break;
      std::this_thread::sleep_for(2ms);
    }
    introspect::RegistryMapping mapping;
    // Claims a row here and hands it to alpha's pid: the registration no
    // longer owns it, so its destructor leaves it active. A claim takes the
    // lowest free row whose heartbeat is old (never used, here), so the
    // earlier run's comes first.
    const auto plant = [&](int64_t heartbeat, uint64_t wake_failures) {
      const introspect::NodeRegistration registration{
          {"alpha", "//example/alpha:node", session_id, 0}};
      auto& rows = mapping.header()->nodes;
      for (auto& row : rows) {
        if (row.state.load(std::memory_order_acquire) !=
                static_cast<uint64_t>(introspect::SlotState::ACTIVE) ||
            row.session_id != session_id ||
            row.pid != static_cast<uint64_t>(::getpid()))
          continue;
        row.state.store(static_cast<uint64_t>(introspect::SlotState::CLAIMING),
                        std::memory_order_release);
        row.pid = static_cast<uint64_t>(pid);
        row.pid_start = 0;
        row.flags.store(0, std::memory_order_release);
        row.heartbeat_monotonic_ns.store(heartbeat);
        row.dispatch_count.store(7);
        row.wake_failures.store(wake_failures);
        row.state.store(static_cast<uint64_t>(introspect::SlotState::ACTIVE),
                        std::memory_order_release);
        return static_cast<std::ptrdiff_t>(&row - &rows[0]);
      }
      return std::ptrdiff_t{-1};
    };
    const std::ptrdiff_t earlier = pid > 0 ? plant(before, 3) : -1;
    const std::ptrdiff_t own =
        earlier >= 0 ? plant(introspect::monotonic_now_ns(), 0) : -1;
    planted = earlier >= 0 && own > earlier;
    // A whole pass has run since now (RunSessionUntil).
    const auto pass = [&] {
      const uint64_t after = passes.Started() + 2;
      while (passes.Started() < after &&
             std::chrono::steady_clock::now() < give_up) {
        SupervisionWakeup::Wake();
        std::this_thread::sleep_for(2ms);
      }
      return passes.Started() >= after;
    };
    passed = planted;
    for (int step = 0; passed && step < 15; ++step) {
      auto& row = mapping.header()->nodes[own];
      row.dispatch_count.fetch_add(1);
      row.heartbeat_monotonic_ns.store(introspect::monotonic_now_ns());
      clock.Advance(400ms);
      passed = pass();
    }
    std::ofstream{go_file} << "go\n";
    // Their pid is gone: freed, as the next claim would.
    for (const std::ptrdiff_t row : {earlier, own}) {
      if (row >= 0)
        mapping.header()->nodes[row].state.store(
            static_cast<uint64_t>(introspect::SlotState::FREE),
            std::memory_order_release);
    }
  }};
  RunSession(options);
  driver.join();
  ASSERT_TRUE(planted) << "no rows to plant";
  EXPECT_TRUE(passed) << "the supervision passes did not run";
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(alpha.size(), 1u) << manifest;
  EXPECT_NE(alpha[0].find("\"reason\": \"exit\""), std::string::npos)
      << "judged by the earlier run's row: " << alpha[0];
  EXPECT_EQ(alpha[0].find("\"wake_failures\": 3"), std::string::npos)
      << "an earlier run's row counted as this run's: " << alpha[0];
  std::filesystem::remove_all(options.output_dir);
  ::remove(pid_file.c_str());
  ::remove(go_file.c_str());
}

// session_report --strict on a finished session directory: its exit status
// and its stderr, where it names each reason it rejects the session.
struct StrictReport {
  int status{-1};
  std::string stderr_text;
};

StrictReport RunStrictReport(const rules_cc::cc::runfiles::Runfiles& runfiles,
                             const std::string& session_dir) {
  // The report finds log_dump through these, in this test's runfiles.
  for (const auto& [key, value] : runfiles.EnvVars())
    ::setenv(key.c_str(), value.c_str(), 1);
  const char* report = std::getenv("TALOS_SESSION_REPORT");
  if (report == nullptr) return {};
  const std::string errors = TempPath("strict_report.stderr");
  const std::string command = "'" + runfiles.Rlocation(report) +
                              "' --strict '" + session_dir +
                              "' >/dev/null 2>'" + errors + "'";
  const int status = std::system(command.c_str());
  return {.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1,
          .stderr_text = ReadFile(errors)};
}

// A wake notification lost on a topic its node's loop does not write -- a
// byte topic (Node::publish_bytes), as /talos/telemetry is -- fails the
// session end to end: the fixture node (testdata/byte_wake_loss_node.cc)
// cannot send one notification on /alpha/telemetry for want of a
// descriptor, its registry row counts it, the launcher records it in the
// session manifest, and session_report --strict rejects the session naming
// alpha (TODO.md RT-7). The bridge and beta are stubs that lose nothing.
// REQ-OPS-43, REQ-RT-19
TEST(Supervision, AWakeLostOnAByteTopicFailsTheStrictReport) {
#if defined(__SANITIZE_ADDRESS__)
  GTEST_SKIP() << "ASan runtime calls pipe() while byte_wake_loss exhausts "
                  "RLIMIT_NOFILE";
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
  GTEST_SKIP() << "ASan runtime calls pipe() while byte_wake_loss exhausts "
                  "RLIMIT_NOFILE";
#endif
#endif
  auto options = BaseOptions("byte_wake_loss");
  options.max_restarts = 0;
  options.duration_s = kHangSeconds;
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  ASSERT_NE(runfiles, nullptr) << error;
  const char* node = std::getenv("TALOS_BYTE_WAKE_LOSS_NODE");
  const char* stub = std::getenv("TALOS_DELIVERY_LOSS_NODE");
  ASSERT_NE(node, nullptr);
  ASSERT_NE(stub, nullptr);
  const auto lossless = [&](std::string_view name, std::string_view target) {
    return WriteStub("byte_wake_loss_" + std::string{name},
                     "exec " + runfiles->Rlocation(stub) + " " +
                         std::string{name} + " " + std::string{target} +
                         " 0,0,0,0,run \"$@\"\n",
                     name);
  };
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      lossless("hardware_node", "//talOS/bridge:hardware_node");
  options.binary_overrides["//example/beta:node"] =
      lossless("beta", "//example/beta:node");
  options.binary_overrides["//example/alpha:node"] = runfiles->Rlocation(node);
  const uint64_t session = options.session_id;
  std::string seen;
  EXPECT_EQ(RunSessionUntil(
                options,
                [session](const std::string& live) {
                  // The lossless stubs too: every member ends as its stop
                  // handler has it (RunSessionUntil).
                  return ManifestEntry(live, "alpha")
                                 .find("\"wake_failures\": 1,\n") !=
                             std::string::npos &&
                         Registered("hardware_node", session) &&
                         Registered("beta", session);
                },
                seen),
            0);
  ASSERT_FALSE(seen.empty()) << "no live manifest carried alpha's lost wake";

  const StrictReport report = RunStrictReport(*runfiles, options.output_dir);
  EXPECT_EQ(report.status, 2) << report.stderr_text;
  EXPECT_NE(report.stderr_text.find(
                "session_report: strict: alpha: wake_failures=1\n"),
            std::string::npos)
      << report.stderr_text;
  std::filesystem::remove_all(options.output_dir);
}

TEST(Supervision,
     NonCriticalDescribeFailureWarnsAndRestartsWithoutBlockingPeers) {
  auto options = BaseOptions("optional_probe");
  MakeAlphaNonCritical(options);
  const auto runs = TempPath("optional_probe_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "optional_alpha",
      RecordLogArgThen(runs,
                       "case \"$log\" in *restart1.tlog) exit 0;; esac\n"
                       "exit 3\n"),
      "alpha", false);
  options.binary_overrides["//example/beta:node"] =
      WriteStub("optional_beta", "exit 0\n");
  const auto config = config::ParseRobotConfig(options.config_path);
  std::vector<NodeDescription> described;
  const auto findings =
      DescribeGraph(DiscoverNodes(config, options), options, described);
  EXPECT_FALSE(naming::HasError(findings));
  EXPECT_TRUE(std::any_of(findings.begin(), findings.end(), [](const auto& d) {
    return d.subject == "alpha" && d.severity == naming::Severity::WARNING;
  }));
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs).size(), 2u);
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  EXPECT_NE(ManifestEntry(manifest, "beta").find("\"exit_code\": 0"),
            std::string::npos);
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// A long-lived stub that exits 0 when the launcher stops it.
constexpr std::string_view kRunUntilStopped =
    "trap 'p=${_sleep_pid:-$!}; [ -n \"$p\" ] && kill \"$p\" 2>/dev/null; "
    "exit 0' INT TERM\n"
    "while :; do sleep 86400 </dev/null >/dev/null 2>&1 & _sleep_pid=$!; "
    "wait $_sleep_pid; done\n";

// A critical node that exits 0 while its peers run on is lost, not finished:
// it restarts, once, into its next log. Every stub records its --log
// argument once its stop trap is set, and the session is stopped once all
// four runs have. That a clean exit while stopping is not restarted is
// RequestedStopDoesNotRestartCleanExits's; the session deadline takes the
// same path: its pass sends the stop (stop_sent) before any member can
// react, so every later exit is reaped with the session stopping.
// REQ-SAFE-29
TEST(Supervision, CriticalCleanExitRestarts) {
  auto options = BaseOptions("clean_exit");
  options.duration_s = kHangSeconds;
  const auto runs = TempPath("clean_exit_runs");
  const auto peers = TempPath("clean_exit_peers");
  ::remove(runs.c_str());
  ::remove(peers.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("clean_exit_hardware", TrappedRun(peers, kLoopUntilStopped),
                "hardware_node");
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "clean_exit_alpha",
      TrappedRun(runs, "case \"$log\" in *alpha.tlog) exit 0;; esac\n" +
                           std::string{kLoopUntilStopped}));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("clean_exit_beta", TrappedRun(peers, kLoopUntilStopped));
  std::string seen;
  EXPECT_EQ(RunSessionUntil(
                options,
                [&](const std::string&) {
                  return ReadLines(runs).size() == 2 &&
                         ReadLines(peers).size() == 2;
                },
                seen),
            0);
  ASSERT_FALSE(seen.empty()) << "alpha never restarted";
  EXPECT_EQ(ReadLines(runs), (std::vector<std::string>{
                                 options.output_dir + "/alpha.tlog",
                                 options.output_dir + "/alpha.restart1.tlog"}));
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 1"), std::string::npos) << alpha;
  // The restarted run's pid is printed as the first run's is, so an operator
  // can find the process without reading the manifest (TODO.md DEPLOY-17).
  const auto history = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(history.size(), 2u) << manifest;
  const long long restarted_pid = std::strtoll(
      history[1].c_str() + std::string_view{"{\"pid\": "}.size(), nullptr, 10);
  ASSERT_GT(restarted_pid, 0) << history[1];
  const std::string output =
      ReadFile(options.output_dir + "/" + std::string{kLauncherOutputFile});
  EXPECT_NE(output.find("alpha (pid " + std::to_string(restarted_pid) + ","),
            std::string::npos)
      << "restarted pid " << restarted_pid << " not in " << output;
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
  ::remove(peers.c_str());
}

// A critical peer that exited 0 once and restarted is now actively running:
// its prior clean exit must not cause RosterContinues to skip it and drop
// another critical member's clean-exit restart.
// REQ-SAFE-29
TEST(Supervision, CriticalCleanExitRestartsWhileRestartedPeerRuns) {
  auto options = BaseOptions("clean_exit_restarted_peer");
  options.duration_s = kHangSeconds;
  const auto hw_restarted = TempPath("clean_exit_restarted_peer_hw");
  const auto alpha_restarted = TempPath("clean_exit_restarted_peer_alpha");
  const auto beta_runs = TempPath("clean_exit_restarted_peer_beta");
  ::remove(hw_restarted.c_str());
  ::remove(alpha_restarted.c_str());
  ::remove(beta_runs.c_str());
  const auto restart_once_then_signal = [](const std::string& marker) {
    return std::string{kStopTrap} +
           "prev=\"\"\n"
           "log=\"\"\n"
           "for arg in \"$@\"; do\n"
           "  if [ \"$prev\" = \"--log\" ]; then log=\"$arg\"; fi\n"
           "  prev=\"$arg\"\n"
           "done\n"
           "case \"$log\" in\n"
           "  *restart1.tlog)\n"
           "    : > " +
           marker + "\n" + std::string{kLoopUntilStopped} +
           "    ;;\n"
           "  *)\n"
           "    exit 0\n"
           "    ;;\n"
           "esac\n";
  };
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("clean_exit_restarted_peer_hardware",
                restart_once_then_signal(hw_restarted), "hardware_node");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("clean_exit_restarted_peer_alpha",
                restart_once_then_signal(alpha_restarted), "alpha");
  options.binary_overrides["//example/beta:node"] = WriteStub(
      "clean_exit_restarted_peer_beta",
      TrappedRun(beta_runs,
                 "case \"$log\" in\n"
                 "  *beta.tlog)\n"
                 "    while [ ! -f " +
                     hw_restarted + " ] || [ ! -f " + alpha_restarted +
                     " ]; do sleep 0.01; done\n"
                     "    exit 0\n"
                     "    ;;\n"
                     "esac\n" +
                     std::string{kLoopUntilStopped}),
      "beta");
  std::string seen;
  EXPECT_EQ(RunSessionUntil(
                options,
                [&](const std::string& live) {
                  if (ReadLines(beta_runs).size() >= 2) return true;
                  // Stop promptly if beta's clean-exit restart was wrongly
                  // cancelled so a regression fails fast instead of hanging.
                  const auto beta = ManifestEntry(live, "beta");
                  return beta.find("\"exit_code\": 0") != std::string::npos &&
                         beta.find("\"restart_count\": 0") !=
                             std::string::npos &&
                         beta.find("\"retry_at_monotonic_ns\": 0") !=
                             std::string::npos;
                },
                seen),
            0);
  ASSERT_FALSE(seen.empty())
      << "session ended before beta finished or restarted";
  EXPECT_EQ(
      ReadLines(beta_runs),
      (std::vector<std::string>{options.output_dir + "/beta.tlog",
                                options.output_dir + "/beta.restart1.tlog"}));
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto beta = ManifestEntry(manifest, "beta");
  EXPECT_NE(beta.find("\"restart_count\": 1"), std::string::npos) << beta;
  const std::string output =
      ReadFile(options.output_dir + "/" + std::string{kLauncherOutputFile});
  EXPECT_EQ(output.find("beta finished with the rest of the roster"),
            std::string::npos)
      << output;
  std::filesystem::remove_all(options.output_dir);
  ::remove(hw_restarted.c_str());
  ::remove(alpha_restarted.c_str());
  ::remove(beta_runs.c_str());
}

// A critical member whose clean-exit restart fails to spawn must still retry
// even if every peer exits 0 while that retry is pending:
// RestartSchedule::Started() cleared last_exit_clean_ on the restart attempt,
// so neither the member nor its peers see the stale clean-exit flag and drop
// the retry as "finished with the rest of the roster". Because the retrying
// member is not running during its retry window, RosterContinues' live-peer
// check alone cannot save it.
// REQ-SAFE-29
TEST(Supervision, FailedSpawnAfterCleanExitRetriesWhenPeersExitCleanly) {
  auto options = BaseOptions("clean_exit_failed_spawn");
  options.duration_s = kHangSeconds;
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  options.supervision_clock = clock.clock();
  const auto alpha_runs = TempPath("clean_exit_failed_spawn_alpha_runs");
  const auto peers_exit = TempPath("clean_exit_failed_spawn_peers_exit");
  ::remove(alpha_runs.c_str());
  ::remove(peers_exit.c_str());
  const std::string peer_body =
      "trap 'exit 0' INT TERM\n"
      "prev=\"\"\n"
      "log=\"\"\n"
      "for arg in \"$@\"; do\n"
      "  if [ \"$prev\" = \"--log\" ]; then log=\"$arg\"; fi\n"
      "  prev=\"$arg\"\n"
      "done\n"
      "case \"$log\" in\n"
      "  *restart1.tlog) exit 0 ;;\n"
      "esac\n"
      "while [ ! -f " +
      peers_exit +
      " ]; do sleep 0.01; done\n"
      "exit 0\n";
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("clean_exit_failed_spawn_hardware", peer_body, "hardware_node");
  const std::string alpha_bin =
      WriteStub("clean_exit_failed_spawn_alpha",
                RecordLogArgThen(alpha_runs,
                                 "case \"$log\" in\n"
                                 "  *alpha.tlog)\n"
                                 "    chmod a-x \"$0\"\n"
                                 "    exit 0\n"
                                 "    ;;\n"
                                 "esac\n"
                                 "exit 0\n"),
                "alpha");
  options.binary_overrides["//example/alpha:node"] = alpha_bin;
  options.binary_overrides["//example/beta:node"] =
      WriteStub("clean_exit_failed_spawn_beta", peer_body, "beta");
  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    const auto reaped_runs = [&](const char* name, std::size_t count) {
      const auto runs = ManifestRuns(ReadFile(live), name);
      return runs.size() >= count &&
             runs.back().find("\"reason\": \"running\"") == std::string::npos;
    };
    // Wait for alpha run 0 to exit 0 and be reaped while peers are running.
    if (!WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          return reaped_runs("alpha", 1);
        })) {
      return;
    }
    // Step to alpha's 50 ms clean-exit restart; exec fails with EACCES and
    // schedules a retry at start + 150 ms.
    clock.HoldAt(start + 50ms);
    if (!WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          const auto runs = ManifestRuns(ReadFile(live), "alpha");
          return runs.size() >= 2 &&
                 runs[1].find("\"reason\": \"spawn_failed\"") !=
                     std::string::npos;
        })) {
      return;
    }
    // Restore alpha's execute bit for the retry, then let both peers exit 0
    // cleanly while alpha's retry is pending.
    ::chmod(alpha_bin.c_str(), 0755);
    { std::ofstream touch{peers_exit}; }
    if (!WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          return reaped_runs("hardware_node", 1) && reaped_runs("beta", 1);
        })) {
      return;
    }
    // Step to start + 150 ms: both peers' 50 ms restarts (due at +100 ms) and
    // alpha's retry (due at +150 ms) are due. Because Started() cleared
    // alpha's last_exit_clean_, alpha retries and spawns restart1.tlog.
    clock.HoldAt(start + 150ms);
    if (!WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          return reaped_runs("alpha", 3) && reaped_runs("hardware_node", 2) &&
                 reaped_runs("beta", 2);
        })) {
      return;
    }
    // All three restart1 runs have exited 0 cleanly; step past the longest
    // clean-exit backoff (200 ms for alpha's third start) so RosterContinues
    // ends the session cleanly.
    clock.HoldAt(start + 450ms);
    WaitFor(stop, [&] {
      SupervisionWakeup::Wake();
      return false;
    });
  }};
  EXPECT_EQ(RunSession(options), 0);
  driver.request_stop();
  driver.join();
  ::chmod(alpha_bin.c_str(), 0755);
  EXPECT_EQ(
      ReadLines(alpha_runs),
      (std::vector<std::string>{options.output_dir + "/alpha.tlog",
                                options.output_dir + "/alpha.restart1.tlog"}));
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 1"), std::string::npos) << alpha;
  const auto history = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(history.size(), 3u) << manifest;
  EXPECT_NE(history[0].find("\"reason\": \"exit\""), std::string::npos)
      << history[0];
  EXPECT_NE(history[1].find("\"reason\": \"spawn_failed\""), std::string::npos)
      << history[1];
  EXPECT_NE(history[2].find("\"reason\": \"exit\""), std::string::npos)
      << history[2];
  std::filesystem::remove_all(options.output_dir);
  ::remove(alpha_runs.c_str());
  ::remove(peers_exit.c_str());
}

// A run that cannot be spawned -- fork or pipe failing, as under EAGAIN -- is
// retried on the restart backoff instead of abandoned: a critical member whose
// first spawn and first retry both fail still runs, as its first run. The
// session is stopped once every member's run has set its stop trap.
TEST(Supervision, FailedSpawnIsRetriedNotAbandoned) {
  auto options = BaseOptions("spawn_failure");
  options.duration_s = kHangSeconds;
  options.spawn_failures["alpha"] = 2;
  const auto runs = TempPath("spawn_failure_runs");
  const auto peers = TempPath("spawn_failure_peers");
  ::remove(runs.c_str());
  ::remove(peers.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("spawn_failure_hardware", TrappedRun(peers, kLoopUntilStopped),
                "hardware_node");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("spawn_failure_alpha", TrappedRun(runs, kLoopUntilStopped));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("spawn_failure_beta", TrappedRun(peers, kLoopUntilStopped));
  std::string seen;
  EXPECT_EQ(RunSessionUntil(
                options,
                [&](const std::string&) {
                  return !ReadLines(runs).empty() &&
                         ReadLines(peers).size() == 2;
                },
                seen),
            0);
  ASSERT_FALSE(seen.empty()) << "alpha never ran";
  EXPECT_EQ(ReadLines(runs),
            std::vector<std::string>{options.output_dir + "/alpha.tlog"});
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
  ::remove(peers.c_str());
}

// A run whose exec fails is a spawn failure that names its cause, as the
// child reports it, rather than a run that exited 127: alpha's first run takes
// its binary's execute permission away, so its restart cannot exec, and that
// restart is retried, not counted as a run, and marks alpha recorder_failed.
// REQ-OPS-01
TEST(Supervision, ExecFailureIsASpawnFailureThatNamesItsCause) {
  auto options = BaseOptions("exec_failure", "max_restarts = 1\n");
  MakeAlphaNonCritical(options);
  const std::string alpha =
      WriteStub("exec_failure_alpha", "chmod a-x \"$0\"\nexit 3\n");
  options.binary_overrides["//example/alpha:node"] = alpha;
  options.binary_overrides["//example/beta:node"] =
      WriteStub("exec_failure_beta", kRunUntilStopped);
  const auto log = TempPath("exec_failure_stderr");
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) {
    const int fd = ::open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0 || ::dup2(fd, STDERR_FILENO) < 0) ::_exit(100);
    ::_exit(Launcher{options}.Run());
  }
  // The launcher's line for the failed restart: execv, alpha's binary, and the
  // errno.
  const std::string cause = std::strerror(EACCES);
  const auto report = [&] {
    std::istringstream lines{ReadFile(log)};
    for (std::string line; std::getline(lines, line);) {
      if (line.find(alpha) != std::string::npos &&
          line.find(cause) != std::string::npos)
        return line;
    }
    return std::string{};
  };
  std::string reported = report();
  for (const auto give_up = std::chrono::steady_clock::now() + 20s;
       reported.empty() && std::chrono::steady_clock::now() < give_up;
       reported = report())
    std::this_thread::sleep_for(10ms);
  ::kill(launcher, SIGTERM);
  while (::waitpid(launcher, nullptr, 0) < 0 && errno == EINTR) {
  }
  EXPECT_FALSE(reported.empty()) << ReadFile(log);
  EXPECT_NE(reported.find("execv"), std::string::npos) << reported;
  const auto manifest =
      ReadFile(options.output_dir + "/" + std::to_string(options.session_id) +
               "/manifest.json");
  const auto entry = ManifestEntry(manifest, "alpha");
  EXPECT_NE(entry.find("\"restart_count\": 0,"), std::string::npos) << entry;
  EXPECT_NE(entry.find("\"recorder_failed\": true"), std::string::npos)
      << entry;
  EXPECT_NE(entry.find("\"recorder_failed_logs\": []"), std::string::npos)
      << entry;
  // The failed spawn is a run of its own, with no exit of its own: not the
  // previous run's exit code 3.
  const auto history = ManifestRuns(manifest, "alpha");
  ASSERT_GE(history.size(), 2u) << manifest;
  EXPECT_NE(history[1].find("{\"pid\": -1,"), std::string::npos) << history[1];
  EXPECT_NE(history[1].find("\"exit_code\": -1, \"exit_signal\": 0, "
                            "\"reason\": \"spawn_failed\""),
            std::string::npos)
      << history[1];
  std::filesystem::remove_all(options.output_dir);
  ::remove(log.c_str());
}

// A requested stop (SIGTERM to the launcher) restarts none of the members
// that exit 0 while stopping.
TEST(Supervision, RequestedStopDoesNotRestartCleanExits) {
  auto options = BaseOptions("clean_stop");
  const auto runs = TempPath("clean_stop_runs");
  ::remove(runs.c_str());
  // Every roster member must stay alive until shutdown, with its stop
  // handler set: an exit-0 hardware stub otherwise restarts, and the stop's
  // SIGINT can reach the restart before its trap, ending it by the signal
  // rather than the exit 0 asserted below.
  for (const std::string name : {"hardware_node", "alpha", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + name + ":node";
    options.binary_overrides[target] = WriteStub(
        "clean_stop_" + name, TrappedRun(runs, kLoopUntilStopped), name);
  }
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) ::_exit(Launcher{options}.Run());
  const auto give_up =
      std::chrono::steady_clock::now() + std::chrono::seconds{kHangSeconds};
  // A recorded log argument acknowledges an installed stop handler, not just
  // a fork. Wait for all members rather than guessing with a fixed sleep.
  while (ReadLines(runs).size() < 3 &&
         std::chrono::steady_clock::now() < give_up)
    std::this_thread::sleep_for(10ms);
  const bool ready = ReadLines(runs).size() == 3;
  ::kill(launcher, SIGTERM);
  int status = 0;
  while (::waitpid(launcher, &status, 0) < 0 && errno == EINTR) {
  }
  EXPECT_TRUE(ready) << "not every node installed its stop handler";
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  const std::string dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  auto recorded = ReadLines(runs);
  std::sort(recorded.begin(), recorded.end());
  EXPECT_EQ(recorded,
            (std::vector<std::string>{dir + "/alpha.tlog", dir + "/beta.tlog",
                                      dir + "/hardware_node.tlog"}));
  const auto manifest = ReadFile(dir + "/manifest.json");
  for (const auto* name : {"hardware_node", "alpha", "beta"}) {
    const auto entry = ManifestEntry(manifest, name);
    EXPECT_NE(entry.find("\"restart_count\": 0"), std::string::npos) << entry;
    EXPECT_NE(entry.find("\"exit_code\": 0"), std::string::npos) << entry;
  }
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// A stop that reaches a member before it handles one: between its exec and
// process::InstallStopHandlers() (a C++ node's first lines, a Python node's
// whole interpreter start) the stop's SIGINT kills it. That is the session's
// own stop, not a failed member: an operator's Ctrl-C during startup ends
// the session with status 0, the member is not restarted, and its run says
// how it ended (reason stop, exit_signal SIGINT). alpha never installs a
// handler, which holds that window open; its peers handle the stop.
// REQ-OPS-06
TEST(Supervision, AMemberEndedByTheStopsOwnSignalLeavesTheSessionClean) {
  auto options = BaseOptions("unhandled_stop");
  options.duration_s = kHangSeconds;
  const auto runs = TempPath("unhandled_stop_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("unhandled_stop_hardware", TrappedRun(runs, kLoopUntilStopped),
                "hardware_node");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("unhandled_stop_beta", TrappedRun(runs, kLoopUntilStopped));
  // exec: the member is the sleep, which the SIGINT ends whatever the shell
  // would do with it.
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "unhandled_stop_alpha", RecordLogArgThen(runs, "exec sleep 600\n"));
  std::string seen;
  const int status = RunSessionUntil(
      options, [&](const std::string&) { return ReadLines(runs).size() == 3; },
      seen);
  EXPECT_FALSE(seen.empty()) << "the session ended before every member ran";
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  EXPECT_EQ(status, 0) << manifest;
  const std::string alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 0"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"exit_code\": -1"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"exit_signal\": " + std::to_string(SIGINT)),
            std::string::npos)
      << alpha;
  const auto alpha_runs = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(alpha_runs.size(), 1u) << manifest;
  EXPECT_NE(alpha_runs[0].find("\"reason\": \"stop\", "
                               "\"stopped_by_session\": true"),
            std::string::npos)
      << alpha_runs[0];
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// A stop that reaches a member while its runtime is still starting, before
// the node framework handles a stop, and that the runtime would turn into an
// ordinary exit 1 rather than a death by the signal (CPython importing
// `site`: KeyboardInterrupt, "Fatal Python error: init_import_site", exit
// 1), is held until the framework's handlers are in, and the member stops
// cleanly: exit 0, reason stop, and the session ends 0. alpha is such a
// runtime (testdata/held_stop_node.cc: a SIGINT exits 1 until one is
// pending), and declares holds_stop through the framework's --describe. The
// stop is the launcher's SIGINT, raised once alpha's runtime handler is in
// (its `.started` marker), before beta's spawn, so beta never starts.
// Without the hold alpha exits 1 and the run is a crash.
// REQ-OPS-06
TEST(Supervision, AStopDuringAMembersRuntimeStartUpIsHeldUntilItHandlesIt) {
  auto options = BaseOptions("held_stop");
  options.duration_s = kHangSeconds;
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  ASSERT_TRUE(runfiles) << error;
  const char* held_stop = std::getenv("TALOS_HELD_STOP_NODE");
  ASSERT_NE(held_stop, nullptr) << "missing held stop node runfile";
  options.binary_overrides["//example/alpha:node"] =
      runfiles->Rlocation(held_stop);
  options.binary_overrides["//example/beta:node"] =
      WriteStub("held_stop_beta", "exit 0\n");
  const std::string started = options.output_dir + "/" +
                              std::to_string(options.session_id) +
                              "/alpha.tlog.started";
  bool marked = false;
  options.member_spawned = [&](const std::string& member) {
    if (member != "alpha") return;
    const auto give_up =
        std::chrono::steady_clock::now() + std::chrono::seconds{kHangSeconds};
    while (!(marked = std::filesystem::exists(started)) &&
           std::chrono::steady_clock::now() < give_up)
      std::this_thread::sleep_for(1ms);
    ::raise(SIGINT);
  };
  const int status = RunSession(options);
  EXPECT_TRUE(marked) << "alpha never reached its runtime's start-up";
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha_runs = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(alpha_runs.size(), 1u) << manifest;
  EXPECT_NE(alpha_runs[0].find("\"exit_code\": 0, \"exit_signal\": 0, "
                               "\"reason\": \"stop\""),
            std::string::npos)
      << alpha_runs[0];
  EXPECT_EQ(status, 0) << manifest;
  std::filesystem::remove_all(options.output_dir);
}

// A signal from elsewhere that ends a member during the stop, before the
// stop's own SIGINT could (systemd's default KillMode=control-group sends
// every process of the unit SIGTERM along with the launcher), is recorded
// with reason stop but not as stopped by the session, and the session ends
// nonzero: the manifest's stopped_by_session and the exit status agree.
// alpha ignores SIGINT, so only the test's SIGTERM can end it, and the
// supervision clock is held, so the stop timeout's SIGKILL cannot come
// first. Its peers handle the stop and exit 0.
// REQ-OPS-06
TEST(Supervision, AStopSignalFromElsewhereIsNotTheSessionsStop) {
  auto options = BaseOptions("foreign_stop");
  options.duration_s = kHangSeconds;
  testing::SteppedClock clock;
  clock.Freeze();
  options.supervision_clock = clock.clock();
  const auto runs = TempPath("foreign_stop_runs");
  const auto alpha_pid = TempPath("foreign_stop_alpha_pid");
  ::remove(runs.c_str());
  ::remove(alpha_pid.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("foreign_stop_hardware", TrappedRun(runs, kLoopUntilStopped),
                "hardware_node");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("foreign_stop_beta", TrappedRun(runs, kLoopUntilStopped));
  // Builtins only until the exec; the sleep inherits the ignored SIGINT.
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("foreign_stop_alpha", "trap '' INT\necho $$ > " + alpha_pid +
                                          "\necho ready >> " + alpha_pid +
                                          "\nexec sleep 600\n");
  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  const auto ended_by_stop = [](const std::string& manifest,
                                std::string_view name) {
    const auto history = ManifestRuns(manifest, name);
    return history.size() == 1 &&
           history[0].find("\"reason\": \"stop\"") != std::string::npos;
  };
  bool interrupted = false;
  // SIGTERMs alpha's group once the stop's SIGINT has ended its peers.
  std::jthread interrupter{[&](const std::stop_token& stop) {
    // A give-up lets the clock run on, so the stop timeout ends alpha.
    const testing::SteppedClock::ReleaseOnExit release{clock};
    const auto give_up =
        std::chrono::steady_clock::now() + std::chrono::seconds{kHangSeconds};
    while (!stop.stop_requested() &&
           std::chrono::steady_clock::now() < give_up) {
      const std::string manifest = ReadFile(live);
      const auto lines = ReadLines(alpha_pid);
      pid_t pid = 0;
      if (ended_by_stop(manifest, "hardware_node") &&
          ended_by_stop(manifest, "beta") && lines.size() == 2 &&
          std::from_chars(lines[0].data(), lines[0].data() + lines[0].size(),
                          pid)
                  .ec == std::errc{}) {
        interrupted = ::kill(-pid, SIGTERM) == 0;
        return;
      }
      std::this_thread::sleep_for(2ms);
    }
  }};
  std::string seen;
  const int status = RunSessionUntil(
      options,
      [&](const std::string&) {
        const auto lines = ReadLines(alpha_pid);
        return ReadLines(runs).size() == 2 && lines.size() == 2 &&
               lines[1] == "ready";
      },
      seen);
  interrupter.request_stop();
  interrupter.join();
  EXPECT_FALSE(seen.empty()) << "the session ended before every member ran";
  EXPECT_TRUE(interrupted) << "alpha was never sent the SIGTERM";
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  EXPECT_NE(status, 0) << manifest;
  const auto alpha_runs = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(alpha_runs.size(), 1u) << manifest;
  EXPECT_NE(alpha_runs[0].find("\"exit_signal\": " + std::to_string(SIGTERM) +
                               ", \"reason\": \"stop\", "
                               "\"stopped_by_session\": false"),
            std::string::npos)
      << alpha_runs[0];
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
  ::remove(alpha_pid.c_str());
}

// The same unhandled SIGINT from elsewhere, before any stop is requested, is
// a member that crashed: the launcher did not send it, so the session that
// is then stopped ends nonzero, with the crash as alpha's last run.
// REQ-OPS-06
TEST(Supervision, ASigintTheStopDidNotSendIsStillACrash) {
  auto options = BaseOptions("foreign_sigint");
  options.duration_s = kHangSeconds;
  options.max_restarts = 0;
  const auto runs = TempPath("foreign_sigint_runs");
  const auto alpha_pid = TempPath("foreign_sigint_alpha_pid");
  ::remove(runs.c_str());
  ::remove(alpha_pid.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("foreign_sigint_hardware", TrappedRun(runs, kLoopUntilStopped),
                "hardware_node");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("foreign_sigint_beta", TrappedRun(runs, kLoopUntilStopped));
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("foreign_sigint_alpha",
                // Builtins only until the exec: a shell that is waiting for
                // a child when the SIGINT arrives (bash, macOS's /bin/sh)
                // survives it if that child exits normally.
                "echo $$ > " + alpha_pid + "\necho ready >> " + alpha_pid +
                    "\nexec sleep 600\n");
  // SIGINTs alpha's group, as the launcher's stop would, once it runs.
  std::jthread interrupter{[&](const std::stop_token& stop) {
    for (; !stop.stop_requested(); std::this_thread::sleep_for(2ms)) {
      const auto lines = ReadLines(alpha_pid);
      pid_t pid = 0;
      // The second line says the first is whole.
      if (lines.size() < 2 || lines[1] != "ready" ||
          std::from_chars(lines[0].data(), lines[0].data() + lines[0].size(),
                          pid)
                  .ec != std::errc{})
        continue;
      ::kill(-pid, SIGINT);
      return;
    }
  }};
  std::string seen;
  const int status = RunSessionUntil(
      options,
      [&](const std::string& manifest) {
        const auto alpha = ManifestRuns(manifest, "alpha");
        return ReadLines(runs).size() == 2 && alpha.size() == 1 &&
               alpha[0].find("\"reason\": \"crash\"") != std::string::npos;
      },
      seen);
  interrupter.request_stop();
  EXPECT_FALSE(seen.empty()) << "the session ended before alpha crashed";
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  EXPECT_NE(status, 0) << manifest;
  const std::string alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 0"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"exit_signal\": " + std::to_string(SIGINT)),
            std::string::npos)
      << alpha;
  const auto alpha_runs = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(alpha_runs.size(), 1u) << manifest;
  EXPECT_NE(alpha_runs[0].find("\"reason\": \"crash\""), std::string::npos)
      << alpha_runs[0];
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
  ::remove(alpha_pid.c_str());
}

// A UDP port nothing holds on any interface now.
uint16_t FreeUdpPort() {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  socklen_t length = sizeof address;
  const bool bound =
      fd >= 0 &&
      ::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof address) == 0 &&
      ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length) == 0;
  if (fd >= 0) ::close(fd);
  if (!bound) throw std::runtime_error("no free UDP port");
  return ntohs(address.sin_port);
}

// The pid of `session`'s hardware_node once its loop has started (its
// registry row reports the loop's realtime outcome, which the bridge does
// right before it starts counting its own --duration-s), else 0.
pid_t BridgeLoopStarted(uint64_t session) {
  const auto registry = introspect::RegistryReader::open();
  if (!registry) return 0;
  for (const auto& node : registry->nodes()) {
    if (node.name == "hardware_node" && node.session_id == session &&
        node.alive && node.realtime())
      return static_cast<pid_t>(node.pid);
  }
  return 0;
}

// A bounded session (--duration-s) ends at the launcher's stop, never at a
// member's own duration: the real bridge, with no controller listening (so
// no State ever arrives), would exit 1 at a --duration-s of its own, and the
// session would record that run as a crash, and restart it while the
// deadline has not yet passed on the launcher's clock. Scenario: that clock
// is held short of the 1 s deadline, as a launcher that wakes late (or, on
// macOS, a host that slept: a member's loop clock counts the sleep) has it,
// while the bridge's loop runs the session's 1 s and a quarter second more
// of real time (or until it exits on its own); then the deadline passes.
// The bridge's one run must end as the stop asks (exit 0, reason stop, not
// restarted) and the session exit 0.
// REQ-OPS-06
TEST(Supervision, ABoundedSessionEndsAtItsStopNotAtAMembersOwnDuration) {
  auto options = BaseOptions("bounded_stop");
  options.duration_s = 1;
  options.simulation = true;
  options.remote = "127.0.0.1";
  options.remote_port = FreeUdpPort();
  options.local_port = FreeUdpPort();
  // The stubs never take a registry row; nothing here waits for one.
  options.heartbeat_startup_grace_ms = 60000;
  // The stubs send no hardware request, so they own no device the real
  // bridge would read one for.
  auto config = toml::parse_file(options.config_path);
  for (const char* name : {"alpha", "beta"})
    config["subsystems"][name].as_table()->erase("motors");
  std::ofstream{options.config_path} << config;
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  ASSERT_TRUE(runfiles) << error;
  const char* bridge = std::getenv("TALOS_BRIDGE");
  ASSERT_NE(bridge, nullptr) << "missing bridge runfile";
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      runfiles->Rlocation(bridge);
  for (const std::string name : {"alpha", "beta"}) {
    options.binary_overrides["//example/" + name + ":node"] =
        WriteStub("bounded_stop_" + name, kRunUntilStopped, name);
  }
  testing::SteppedClock clock;
  const SupervisionTime held = clock.Freeze();
  options.supervision_clock = clock.clock();
  bool ran_its_duration = false;
  std::jthread mover{[&](const std::stop_token& stop) {
    // Every exit lets the clock run on, so the deadline still ends the
    // session and the assertions below say what went wrong.
    const testing::SteppedClock::ReleaseOnExit release{clock};
    const auto give_up = std::chrono::steady_clock::now() + 20s;
    pid_t pid = 0;
    while (!stop.stop_requested() &&
           (pid = BridgeLoopStarted(options.session_id)) == 0 &&
           std::chrono::steady_clock::now() < give_up)
      std::this_thread::sleep_for(2ms);
    if (pid == 0) return;
    const auto own_end = std::chrono::steady_clock::now() +
                         std::chrono::seconds{options.duration_s} + 250ms;
    while (!stop.stop_requested() && ::kill(pid, 0) == 0 &&
           std::chrono::steady_clock::now() < own_end)
      std::this_thread::sleep_for(2ms);
    ran_its_duration = true;
    clock.HoldAt(held + std::chrono::seconds{options.duration_s});
  }};
  const int status = RunSession(options);
  mover.request_stop();
  mover.join();
  // The ports were free when picked and released before the launch
  // (docs/TESTING.md §6.4): a bridge that failed and restarted may have
  // lost one to another process, which its own log names.
  const std::string bridge_log =
      "\nthe bridge's ports " + std::to_string(options.local_port) + " and " +
      std::to_string(options.remote_port) +
      " were released before the launch, so another process may have bound "
      "one; hardware_node.log:\n" +
      ReadFile(NodeOutputPath(options.output_dir, "hardware_node"));
  ASSERT_TRUE(ran_its_duration)
      << "the bridge's loop never started" << bridge_log;
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  EXPECT_EQ(status, 0) << manifest << bridge_log;
  const std::string entry = ManifestEntry(manifest, "hardware_node");
  EXPECT_NE(entry.find("\"restart_count\": 0"), std::string::npos)
      << entry << bridge_log;
  const auto runs = ManifestRuns(manifest, "hardware_node");
  ASSERT_EQ(runs.size(), 1u) << manifest << bridge_log;
  EXPECT_NE(runs[0].find("\"exit_code\": 0, \"exit_signal\": 0, "
                         "\"reason\": \"stop\""),
            std::string::npos)
      << runs[0] << bridge_log;
  std::filesystem::remove_all(options.output_dir);
}

// Releasing a SteppedClock held behind steady_clock::now() must advance from
// the held time, not jump forward by the elapsed wall time (which would
// expire kBridgeStopTimeout immediately after HoldAt(deadline) + Release()).
TEST(Supervision, SteppedClockReleaseContinuesFromHeldWithoutJumpingAhead) {
  SupervisionTime base_now{100s};
  testing::SteppedClock clock{[&] { return base_now; }};
  const SupervisionTime held = clock.Freeze();
  // Simulate wall time advancing past the held deadline while frozen (e.g.
  // slow ASan startup + wait), without ever moving HoldAt backwards.
  base_now += 10s;
  const SupervisionTime deadline = held + 1s;
  clock.HoldAt(deadline);
  clock.Release();
  EXPECT_EQ(clock.clock()(), deadline);
  base_now += 250ms;
  EXPECT_EQ(clock.clock()(), deadline + 250ms);
}

// A config that cannot be read refuses the launch before its session
// directory, <output-dir>/<session_id>/, holds a graph.json (README "Two
// phases": nothing was described).
// REQ-OPS-01
TEST(Supervision, MissingConfigFailsFastWithClearError) {
  LauncherOptions options;
  options.session_id = GenerateSessionId();
  options.output_dir = TempPath("missing_cfg_out");
  options.config_path = TempPath("does_not_exist") + ".toml";
  EXPECT_EQ(RunSession(options), 1);
  EXPECT_FALSE(std::filesystem::exists(options.output_dir + "/graph.json"))
      << options.output_dir;
  std::filesystem::remove_all(TempPath("missing_cfg_out"));
}

// Registry rows held by live processes outside the session are never swept, so
// a roster that cannot get a row per node is refused before anything starts
// rather than run until its unregistered nodes are killed.
// REQ-OPS-19
TEST(Supervision, RosterThatDoesNotFitTheRegistryIsRefused) {
  auto options = BaseOptions("registry_full");
  const auto runs = TempPath("registry_full_runs");
  ::remove(runs.c_str());
  for (const std::string name : {"alpha", "beta"}) {
    options.binary_overrides["//example/" + name + ":node"] = WriteStub(
        "registry_full_" + name, RecordLogArgThen(runs, "exit 0\n"), name);
  }
  // The roster registers three nodes (hardware_node, alpha, beta): one too
  // many once 30 of the 32 rows are held.
  std::vector<std::unique_ptr<introspect::NodeRegistration>> held;
  held.reserve(30);
  for (int i = 0; i < 30; ++i) {
    held.push_back(std::make_unique<introspect::NodeRegistration>(
        introspect::NodeRegistration::Identity{.name = "held" +
                                                       std::to_string(i)}));
  }
  ::testing::internal::CaptureStderr();
  const int refused_status = RunSession(options);
  const std::string refused_stderr = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(refused_status, 1);
  EXPECT_TRUE(ReadLines(runs).empty());
  EXPECT_NE(refused_stderr.find("held0"), std::string::npos) << refused_stderr;
  EXPECT_NE(refused_stderr.find("held29"), std::string::npos) << refused_stderr;
  EXPECT_NE(refused_stderr.find("pid " + std::to_string(::getpid())),
            std::string::npos)
      << refused_stderr;
  held.pop_back();
  // An unanswered Studio bridge is dropped before checking capacity, so 29
  // held rows + 3 active nodes still fit even with --start-studio-bridge.
  options.start_studio_bridge = true;
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("registry_full_bridge", "exit 1\n", "studio_bridge",
                /*answers_describe=*/false);
  options.output_dir = TempPath("registry_full");
  options.session_id = GenerateSessionId();
  options.duration_s = kHangSeconds;
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_GE(ReadLines(runs).size(), 2u);
  std::filesystem::remove_all(TempPath("registry_full"));
  ::remove(runs.c_str());
}

// REQ-OPS-19: The launcher prepares the registry before probing any node and
// refuses a launch (exit 1, 0 probes) when the registry segment (0444) or its
// lock file (0000) cannot be opened, naming the file, owner and mode on stderr.
TEST(Supervision, UnopenableRegistryOrLockFileRefusesLaunchBeforeProbing) {
  if (::geteuid() == 0) GTEST_SKIP() << "root can open 0444/0000 modes";
  const std::string mapped = ShmObjectName(introspect::kRegistryPath);
  const std::string lock_path = introspect::LockFilePath(mapped);
  const std::string uid_owner = "owner uid " + std::to_string(::geteuid());

  const auto cleanup_registry = [&] {
    ::chmod(lock_path.c_str(), 0644);
    ::unlink(lock_path.c_str());
    introspect::RegistryMapping::unlink();
  };
  struct CleanupGuard {
    std::function<void()> fn;
    ~CleanupGuard() { fn(); }
  } guard{cleanup_registry};

  const auto make_probe_recorder = [&](std::string_view tag,
                                       const std::string& record,
                                       std::string_view node) {
    const std::string path = TempPath(tag) + ".sh";
    introspect::Description description;
    description.name = std::string{node};
    description.target = node == "hardware_node"
                             ? "//talOS/bridge:hardware_node"
                             : "//example/" + std::string{node} + ":node";
    std::ostringstream out;
    out << "#!/bin/sh\n"
        << "printf '%s\\n' \"$*\" >> '" << record << "'\n"
        << "for arg in \"$@\"; do\n"
        << "  if [ \"$arg\" = \"--describe\" ]; then\n"
        << "    cat <<'TALOS_DESCRIBE'\n"
        << introspect::DescribeToJson(description) << "TALOS_DESCRIBE\n"
        << "    exit 0\n"
        << "  fi\n"
        << "done\n"
        << "exit 0\n";
    return testing::WriteWarmStub(path, out.str());
  };

  struct Case {
    const char* name;
    bool lock_case;
  };
  for (const Case c :
       {Case{"segment_0444", false}, Case{"lock_file_0000", true}}) {
    SCOPED_TRACE(c.name);
    cleanup_registry();
    const std::string probed = TempPath(std::string{"reg_perm_"} + c.name);
    auto options = BaseOptions(std::string{"reg_perm_"} + c.name);
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        make_probe_recorder(std::string{"reg_perm_hw_"} + c.name, probed,
                            "hardware_node");
    options.binary_overrides["//example/alpha:node"] = make_probe_recorder(
        std::string{"reg_perm_alpha_"} + c.name, probed, "alpha");
    options.binary_overrides["//example/beta:node"] = make_probe_recorder(
        std::string{"reg_perm_beta_"} + c.name, probed, "beta");
    // Clear warm-up probe lines written by WriteWarmStub before launching.
    ::remove(probed.c_str());

    if (c.lock_case) {
      const int fd = ::open(lock_path.c_str(),
                            O_CREAT | O_EXCL | O_RDWR | O_CLOEXEC, 0000);
      ASSERT_GE(fd, 0) << std::strerror(errno);
      ASSERT_EQ(::fchmod(fd, 0000), 0);
      ::close(fd);
    } else {
      const int fd =
          ::shm_open(mapped.c_str(), O_CREAT | O_EXCL | O_RDWR, 0444);
      ASSERT_GE(fd, 0) << std::strerror(errno);
      // Linux applies umask to shm_open; macOS applies none and fchmod refuses
      // shm descriptors with EINVAL (talOS/introspection/registry.h).
#ifndef __APPLE__
      ASSERT_EQ(::fchmod(fd, 0444), 0) << std::strerror(errno);
#endif
      ::close(fd);
    }

    ::testing::internal::CaptureStderr();
    const int status = RunSession(options);
    const std::string stderr_text = ::testing::internal::GetCapturedStderr();
    cleanup_registry();

    EXPECT_EQ(status, 1) << stderr_text;
    EXPECT_TRUE(ReadLines(probed).empty())
        << "probes ran despite unopenable registry:\n"
        << ReadFile(probed);
    EXPECT_NE(stderr_text.find(c.lock_case ? lock_path : mapped),
              std::string::npos)
        << stderr_text;
    EXPECT_NE(stderr_text.find(uid_owner), std::string::npos) << stderr_text;
    EXPECT_NE(stderr_text.find(c.lock_case ? "mode 0000" : "mode 0444"),
              std::string::npos)
        << stderr_text;

    std::filesystem::remove_all(TempPath(std::string{"reg_perm_"} + c.name));
    ::remove(probed.c_str());
  }
}

TEST(Supervision, SessionsAreIsolatedAndCleanOnlyOwnedDirs) {
  const std::string sentinel = TempPath("sentinel");
  {
    std::ofstream out{sentinel};
    out << "do not touch\n";
  }

  LauncherOptions a = BaseOptions("iso_a_out");
  a.binary_overrides["//example/alpha:node"] =
      WriteStub("iso_a_alpha", "exit 0\n");
  a.binary_overrides["//example/beta:node"] =
      WriteStub("iso_a_beta", "exit 0\n");
  LauncherOptions b = BaseOptions("iso_b_out");
  b.binary_overrides["//example/alpha:node"] =
      WriteStub("iso_b_alpha", "exit 0\n");
  b.binary_overrides["//example/beta:node"] =
      WriteStub("iso_b_beta", "exit 0\n");

  EXPECT_EQ(RunSession(a), 0);
  EXPECT_EQ(RunSession(b), 0);
  EXPECT_TRUE(std::filesystem::exists(a.output_dir + "/manifest.json"));
  EXPECT_TRUE(std::filesystem::exists(b.output_dir + "/manifest.json"));
  // Distinct sessions: manifests name their own output dir, never each
  // other's, and neither run touched the sibling sentinel file.
  EXPECT_NE(ReadFile(a.output_dir + "/manifest.json").find(a.output_dir),
            std::string::npos);
  EXPECT_EQ(ReadFile(a.output_dir + "/manifest.json").find(b.output_dir),
            std::string::npos);
  EXPECT_EQ(ReadFile(sentinel), "do not touch\n");

  std::filesystem::remove_all(a.output_dir);
  std::filesystem::remove_all(b.output_dir);
  ::remove(sentinel.c_str());
}

// A persistent fault is quarantined at the rolling limit, and the live
// manifest shows it. Stopping the session cancels its one pending retry. The
// session stops once alpha is quarantined, not at a deadline, which a stalled
// host can reach before alpha's third run; that the quarantine waits out the
// window is SupervisionPolicy's. The stubs never register, so their startup
// grace outlasts the 20 s give-up.
// REQ-OPS-04
TEST(Supervision, CrashingNodeQuarantinesUntilTheSessionStops) {
  LauncherOptions options = BaseOptions("restart_out", "max_restarts = 2\n");
  MakeAlphaNonCritical(options);
  options.heartbeat_startup_grace_ms = 60'000;
  const std::string alpha_runs = TempPath("restart_alpha_runs");
  const std::string beta_runs = TempPath("restart_beta_runs");
  ::remove(alpha_runs.c_str());
  ::remove(beta_runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "restart_alpha", RecordLogArgThen(alpha_runs, "kill -KILL $$\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("restart_beta", RecordLogArgThen(beta_runs, kRunUntilStopped));

  // RunSession rewrites options.output_dir on return; don't read it
  // concurrently.
  const auto live_manifest = options.output_dir + "/" +
                             std::to_string(options.session_id) +
                             "/manifest.json";
  bool quarantined = false;
  std::string quarantined_entry;
  std::int64_t seen_quarantined_ns = 0;
  std::jthread stopper{[&](const std::stop_token& stop) {
    const auto give_up = std::chrono::steady_clock::now() + 20s;
    while (!stop.stop_requested() &&
           std::chrono::steady_clock::now() < give_up) {
      std::string entry = ManifestEntry(ReadFile(live_manifest), "alpha");
      if (entry.find("\"quarantined\": true") != std::string::npos) {
        seen_quarantined_ns = introspect::monotonic_now_ns();
        quarantined_entry = std::move(entry);
        quarantined = true;
        break;
      }
      std::this_thread::sleep_for(2ms);
    }
    if (!stop.stop_requested()) ::kill(::getpid(), SIGTERM);
  }};
  EXPECT_NE(RunSession(options), 0);
  stopper.request_stop();
  stopper.join();
  EXPECT_TRUE(quarantined);
  // The quarantined row names an absolute retry deadline, about one rolling
  // window ahead, not a delay or zero.
  const std::string retry_key = "\"retry_at_monotonic_ns\": ";
  const auto retry_pos = quarantined_entry.find(retry_key);
  ASSERT_NE(retry_pos, std::string::npos) << quarantined_entry;
  const std::int64_t retry_at_ns = std::strtoll(
      quarantined_entry.c_str() + retry_pos + retry_key.size(), nullptr, 10);
  EXPECT_GT(retry_at_ns, seen_quarantined_ns) << quarantined_entry;

  const std::string dir = options.output_dir;
  const std::vector<std::string> expected_logs{dir + "/alpha.tlog",
                                               dir + "/alpha.restart1.tlog",
                                               dir + "/alpha.restart2.tlog"};
  EXPECT_EQ(ReadLines(alpha_runs), expected_logs);
  EXPECT_EQ(ReadLines(beta_runs), std::vector<std::string>{dir + "/beta.tlog"});

  const std::string manifest = ReadFile(dir + "/manifest.json");
  const std::string alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 2"), std::string::npos) << alpha;
  // Stopping cancelled the pending retry.
  EXPECT_NE(alpha.find("\"quarantined\": false"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"retry_at_monotonic_ns\": 0"), std::string::npos)
      << alpha;
  EXPECT_NE(alpha.find("\"log_path\": \"" + expected_logs[0] + "\""),
            std::string::npos)
      << alpha;
  EXPECT_NE(alpha.find("\"log_paths\": [\"" + expected_logs[0] + "\", \"" +
                       expected_logs[1] + "\", \"" + expected_logs[2] + "\"]"),
            std::string::npos)
      << alpha;
  const std::string beta = ManifestEntry(manifest, "beta");
  EXPECT_NE(beta.find("\"restart_count\": 0"), std::string::npos) << beta;
  EXPECT_NE(beta.find("\"log_paths\": [\"" + dir + "/beta.tlog\"]"),
            std::string::npos)
      << beta;

  std::filesystem::remove_all(dir);
  ::remove(alpha_runs.c_str());
  ::remove(beta_runs.c_str());
}

// REQ-OPS-04: a quarantined non-critical member does not spawn 1 ms before its
// retry_at_monotonic_ns deadline; at retry_at_monotonic_ns it spawns once,
// clearing `quarantined`, and re-enters quarantine if that retry crashes.
TEST(Supervision,
     QuarantinedNodeRetriesOnlyAtDeadlineThenRequarantinesOnCrash) {
  LauncherOptions options =
      BaseOptions("quarantine_retry_out", "max_restarts = 1\n");
  MakeAlphaNonCritical(options);
  options.duration_s = 3600;
  options.heartbeat_periods = 1'000'000;
  options.heartbeat_startup_grace_ms = 3'600'000;
  options.construction_timeout = 1h;
  testing::SteppedClock clock;
  clock.Freeze();
  options.supervision_clock = clock.clock();
  const std::string alpha_runs = TempPath("quarantine_retry_alpha_runs");
  const std::string crash_retry = TempPath("quarantine_retry_crash");
  ::remove(alpha_runs.c_str());
  ::remove(crash_retry.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("quarantine_retry_hw", kRunUntilStopped, "hardware_node");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("quarantine_retry_alpha",
                RecordLogArgThen(alpha_runs,
                                 "case \"$log\" in\n"
                                 "  *restart2.tlog)\n"
                                 "    while [ ! -e '" +
                                     crash_retry +
                                     "' ]; do sleep 0.01; done\n"
                                     "    exit 3\n"
                                     "    ;;\n"
                                     "esac\n"
                                     "exit 3\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("quarantine_retry_beta", kRunUntilStopped);

  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  testing::SupervisionPasses passes;
  int64_t quarantine_retry_at = 0;
  bool stayed_quarantined_before_deadline = false;
  bool retried_and_cleared_quarantine = false;
  bool requarantined_after_retry_crash = false;
  int64_t requarantine_retry_at = 0;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    // Drive run 0's crash into restart 1 (the 1 restart allowed in the window).
    int64_t first_retry_at = 0;
    if (!WaitFor(stop, [&] {
          const std::string manifest = ReadFile(live);
          const auto history = ManifestRuns(manifest, "alpha");
          first_retry_at = EntryInteger(ManifestEntry(manifest, "alpha"),
                                        "retry_at_monotonic_ns");
          return history.size() == 1 &&
                 history.back().find("\"reason\": \"crash\"") !=
                     std::string::npos &&
                 first_retry_at > 0;
        })) {
      ::kill(::getpid(), SIGTERM);
      return;
    }
    clock.HoldAt(SupervisionTime{std::chrono::nanoseconds{first_retry_at}});
    SupervisionWakeup::Wake();

    // Run 1 (restart 1) crashes -> enters quarantine.
    const bool entered_quarantine = WaitFor(stop, [&] {
      const std::string manifest = ReadFile(live);
      const std::string entry = ManifestEntry(manifest, "alpha");
      const auto history = ManifestRuns(manifest, "alpha");
      quarantine_retry_at = EntryInteger(entry, "retry_at_monotonic_ns");
      return history.size() == 2 &&
             entry.find("\"quarantined\": true") != std::string::npos &&
             quarantine_retry_at > 0;
    });
    if (entered_quarantine) {
      // Advance 1 ms short of retry_at_monotonic_ns: must NOT spawn.
      clock.HoldAt(
          SupervisionTime{std::chrono::nanoseconds{quarantine_retry_at}} - 1ms);
      const uint64_t after_before = passes.Started() + 2;
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after_before;
      });
      const std::string before_manifest = ReadFile(live);
      stayed_quarantined_before_deadline =
          ReadLines(alpha_runs).size() == 2 &&
          ManifestRuns(before_manifest, "alpha").size() == 2 &&
          ManifestEntry(before_manifest, "alpha")
                  .find("\"quarantined\": true") != std::string::npos;

      // Advance to retry_at_monotonic_ns: spawns once and clears quarantined.
      clock.HoldAt(
          SupervisionTime{std::chrono::nanoseconds{quarantine_retry_at}});
      SupervisionWakeup::Wake();
      retried_and_cleared_quarantine = WaitFor(stop, [&] {
        const std::string manifest = ReadFile(live);
        const std::string entry = ManifestEntry(manifest, "alpha");
        const auto history = ManifestRuns(manifest, "alpha");
        return ReadLines(alpha_runs).size() == 3 && history.size() == 3 &&
               history.back().find("\"reason\": \"running\"") !=
                   std::string::npos &&
               entry.find("\"quarantined\": false") != std::string::npos;
      });
    }
    if (retried_and_cleared_quarantine) {
      // Crash the retry run -> re-quarantines with a later retry deadline.
      std::ofstream{crash_retry}.close();
      requarantined_after_retry_crash = WaitFor(stop, [&] {
        const std::string manifest = ReadFile(live);
        const std::string entry = ManifestEntry(manifest, "alpha");
        const auto history = ManifestRuns(manifest, "alpha");
        requarantine_retry_at = EntryInteger(entry, "retry_at_monotonic_ns");
        return history.size() == 3 &&
               history.back().find("\"reason\": \"crash\"") !=
                   std::string::npos &&
               entry.find("\"quarantined\": true") != std::string::npos &&
               requarantine_retry_at > quarantine_retry_at;
      });
    }
    ::kill(::getpid(), SIGTERM);
  }};

  EXPECT_NE(RunSession(options), 0);
  driver.join();
  EXPECT_GT(quarantine_retry_at, 0);
  EXPECT_TRUE(stayed_quarantined_before_deadline)
      << "quarantined alpha spawned before retry_at_monotonic_ns";
  EXPECT_TRUE(retried_and_cleared_quarantine)
      << "quarantined alpha did not spawn once and clear quarantined at "
         "retry_at_monotonic_ns";
  EXPECT_TRUE(requarantined_after_retry_crash)
      << "alpha did not re-quarantine after its retry run crashed";
  EXPECT_GT(requarantine_retry_at, quarantine_retry_at);
  EXPECT_EQ(ReadLines(alpha_runs).size(), 3u);

  std::filesystem::remove_all(options.output_dir);
  ::remove(alpha_runs.c_str());
  ::remove(crash_retry.c_str());
}

// A critical member (here alpha, an actuator owner) that crashes repeatedly
// past max_restarts is never quarantined: it keeps restarting on the capped
// backoff, its manifest entry stays quarantined: false with restart_count past
// max_restarts, and each crashed run records reason "crash".
// REQ-SAFE-29
TEST(Supervision, CriticalMemberCrashesPastMaxRestartsWithoutQuarantine) {
  LauncherOptions options =
      BaseOptions("critical_restart_loop", "max_restarts = 2\n");
  options.duration_s = kHangSeconds;
  options.heartbeat_startup_grace_ms = 60'000;
  testing::SteppedClock clock;
  clock.Freeze();
  options.supervision_clock = clock.clock();
  const std::string alpha_runs = TempPath("critical_loop_alpha_runs");
  ::remove(alpha_runs.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("critical_loop_hardware", kRunUntilStopped, "hardware_node");
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "critical_loop_alpha", RecordLogArgThen(alpha_runs, "exit 3\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("critical_loop_beta", kRunUntilStopped);

  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  constexpr std::size_t kTargetRuns = 5;  // 4 restarts, past max_restarts = 2.
  bool reached_past_limit = false;
  bool ever_quarantined = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    for (std::size_t run = 1; run <= kTargetRuns; ++run) {
      int64_t retry_at = 0;
      const bool crashed = WaitFor(stop, [&] {
        const std::string manifest = ReadFile(live);
        const std::string entry = ManifestEntry(manifest, "alpha");
        if (entry.find("\"quarantined\": true") != std::string::npos) {
          ever_quarantined = true;
        }
        const auto history = ManifestRuns(manifest, "alpha");
        if (history.size() < run ||
            history[run - 1].find("\"reason\": \"crash\"") ==
                std::string::npos) {
          return false;
        }
        if (history.size() > run) return true;
        retry_at = EntryInteger(entry, "retry_at_monotonic_ns");
        return retry_at > 0;
      });
      if (!crashed) break;
      if (run == kTargetRuns) {
        reached_past_limit = true;
        break;
      }
      if (retry_at > 0) {
        clock.HoldAt(SupervisionTime{std::chrono::nanoseconds{retry_at}});
      }
    }
    ::kill(::getpid(), SIGTERM);
  }};
  EXPECT_NE(RunSession(options), 0);
  driver.request_stop();
  driver.join();

  ASSERT_TRUE(reached_past_limit)
      << "critical alpha did not keep restarting past max_restarts = 2";
  EXPECT_FALSE(ever_quarantined) << "critical alpha must never be quarantined";
  const std::string dir = options.output_dir;
  const auto recorded = ReadLines(alpha_runs);
  ASSERT_GE(recorded.size(), kTargetRuns);
  EXPECT_EQ(recorded[0], dir + "/alpha.tlog");
  for (std::size_t i = 1; i < kTargetRuns; ++i) {
    EXPECT_EQ(recorded[i],
              dir + "/alpha.restart" + std::to_string(i) + ".tlog");
  }
  const std::string manifest = ReadFile(dir + "/manifest.json");
  const std::string alpha = ManifestEntry(manifest, "alpha");
  EXPECT_GE(EntryInteger(alpha, "restart_count"),
            static_cast<int64_t>(kTargetRuns - 1))
      << alpha;
  EXPECT_NE(alpha.find("\"quarantined\": false"), std::string::npos) << alpha;
  const auto history = ManifestRuns(manifest, "alpha");
  ASSERT_GE(history.size(), kTargetRuns) << manifest;
  for (std::size_t i = 0; i < kTargetRuns; ++i) {
    EXPECT_NE(history[i].find("\"reason\": \"crash\""), std::string::npos)
        << "run " << i << ": " << history[i];
  }

  std::filesystem::remove_all(dir);
  ::remove(alpha_runs.c_str());
}

// --max-restarts 0 wins over the config and disables restarts.
// REQ-SAFE-29
TEST(Supervision, MaxRestartsZeroDisablesRestarts) {
  LauncherOptions options = BaseOptions("norestart_out", "max_restarts = 4\n");
  options.max_restarts = 0;
  const std::string alpha_runs = TempPath("norestart_alpha_runs");
  ::remove(alpha_runs.c_str());
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("norestart_alpha", RecordLogArgThen(alpha_runs, "exit 3\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("norestart_beta", "exit 0\n");

  EXPECT_EQ(RunSession(options), 3);
  EXPECT_EQ(ReadLines(alpha_runs),
            std::vector<std::string>{options.output_dir + "/alpha.tlog"});
  const std::string alpha =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"), "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 0"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"exit_code\": 3"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"quarantined\": false"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"retry_at_monotonic_ns\": 0"), std::string::npos)
      << alpha;

  std::filesystem::remove_all(options.output_dir);
  ::remove(alpha_runs.c_str());
}

// A run a signal ended says which: exit_code -1 alone cannot tell a crash
// from a stop the node never handled, or from the launcher's own SIGKILL.
TEST(Supervision, SignalDeathRecordsTheSignal) {
  LauncherOptions options = BaseOptions("signal_death");
  options.max_restarts = 0;
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("signal_death_alpha", "kill -KILL $$\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("signal_death_beta", "exit 0\n");

  EXPECT_NE(RunSession(options), 0);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"exit_code\": -1"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"exit_signal\": " + std::to_string(SIGKILL)),
            std::string::npos)
      << alpha;
  const std::string beta = ManifestEntry(manifest, "beta");
  EXPECT_NE(beta.find("\"exit_signal\": 0"), std::string::npos) << beta;
  std::filesystem::remove_all(options.output_dir);
}

// A session records its config as an absolute path, in manifest.json and
// graph.json, even when --config was relative: a relative one names nothing
// once the launcher's working directory is forgotten.
TEST(Supervision, ConfigPathIsRecordedAbsolute) {
  LauncherOptions options = BaseOptions("absolute_config");
  options.max_restarts = 0;
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("absolute_config_alpha", "exit 0\n", "alpha");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("absolute_config_beta", "exit 0\n");
  const std::filesystem::path config = options.config_path;
  const auto cwd = std::filesystem::current_path();
  std::filesystem::current_path(config.parent_path());
  options.config_path = config.filename().string();
  // The working directory as getcwd spells it, symlinks resolved.
  const auto expected = std::filesystem::current_path() / config.filename();
  RunSession(options);
  std::filesystem::current_path(cwd);

  const std::string absolute = "\"config_path\": \"" + expected.string() + "\"";
  EXPECT_NE(ReadFile(options.output_dir + "/manifest.json").find(absolute),
            std::string::npos);
  EXPECT_NE(ReadFile(options.output_dir + "/graph.json").find(absolute),
            std::string::npos);
  std::filesystem::remove_all(options.output_dir);
}

// A node's output never waits for the launcher's stdout: alpha writes 2 MiB
// to stdout while the launcher's stdout is a pipe nobody reads, then records
// that it got past the write. Inherited, that stdout would block it at 64 KiB.
// Its output lands in alpha.log after the line naming the run, all of it or
// with what was dropped marked.
TEST(Supervision, NodeOutputNeverWaitsForTheLaunchersStdout) {
  auto options = BaseOptions("output_drain");
  const std::string written = TempPath("output_drain_written");
  ::remove(written.c_str());
  constexpr int kBytes = 2 * 1024 * 1024;
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("output_drain_hardware", kRunUntilStopped, "hardware_node");
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "output_drain_alpha", "head -c " + std::to_string(kBytes) +
                                " /dev/zero | tr '\\0' x\necho done > " +
                                written + "\n" + std::string{kRunUntilStopped});
  options.binary_overrides["//example/beta:node"] =
      WriteStub("output_drain_beta", kRunUntilStopped);
  int launcher_stdout[2];
  ASSERT_EQ(::pipe(launcher_stdout), 0);
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) {
    ::close(launcher_stdout[0]);
    if (::dup2(launcher_stdout[1], STDOUT_FILENO) < 0) ::_exit(100);
    ::close(launcher_stdout[1]);
    ::_exit(Launcher{options}.Run());
  }
  ::close(launcher_stdout[1]);
  bool got_past = false;
  for (const auto give_up = std::chrono::steady_clock::now() + 20s;
       !got_past && std::chrono::steady_clock::now() < give_up;) {
    std::this_thread::sleep_for(20ms);
    got_past = std::filesystem::exists(written);
  }
  // Now let the launcher's own lines out, so that it can stop.
  std::thread drain{[fd = launcher_stdout[0]] {
    char bytes[4096];
    while (::read(fd, bytes, sizeof bytes) > 0) {
    }
  }};
  ::kill(launcher, SIGTERM);
  while (::waitpid(launcher, nullptr, 0) < 0 && errno == EINTR) {
  }
  drain.join();
  ::close(launcher_stdout[0]);
  EXPECT_TRUE(got_past) << "alpha blocked on its output";
  const std::string log =
      ReadFile(options.output_dir + "/" + std::to_string(options.session_id) +
               "/alpha.log");
  EXPECT_EQ(log.rfind("[launcher] alpha run 0, pid ", 0), 0u)
      << log.substr(0, 200);
  auto accounted = std::count(log.begin(), log.end(), 'x');
  const std::regex dropped{"\\[launcher\\] ([0-9]+) bytes of output dropped"};
  for (std::sregex_iterator it{log.begin(), log.end(), dropped}, end; it != end;
       ++it)
    accounted += std::stol((*it)[1].str());
  EXPECT_EQ(accounted, kBytes);
  std::filesystem::remove_all(options.output_dir);
  ::remove(written.c_str());
}

// A run's last output stays under its own header when the next run starts
// at once: the old pipe still holds "last words" when the restart's pipe is
// added, and the reader moves them out before writing the new header.
TEST(Supervision, CrashOutputIsNotCreditedToTheRestart) {
  const std::string path = TempPath("output_restart.log");
  ::remove(path.c_str());
  int first[2];
  int second[2];
  ASSERT_EQ(::pipe(first), 0);
  ASSERT_EQ(::pipe(second), 0);
  const std::string last_words = "last words\n";
  ASSERT_EQ(::write(first[1], last_words.data(), last_words.size()),
            static_cast<ssize_t>(last_words.size()));
  {
    ChildOutput output;
    output.Add(first[0], path, "alpha", 100, false);
    output.Add(second[0], path, "alpha", 101, false);
    ::close(first[1]);
    ::close(second[1]);
    output.Stop();
  }
  EXPECT_EQ(ReadFile(path),
            "[launcher] alpha run 0, pid 100\nlast words\n"
            "[launcher] alpha run 1, pid 101\n");
  ::remove(path.c_str());
}

// A negative [robot] max_restarts is a config error, caught before launch.
TEST(Supervision, NegativeMaxRestartsRefusesToLaunch) {
  LauncherOptions options =
      BaseOptions("badrestart_out", "max_restarts = -1\n");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("badrestart_alpha", "exit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("badrestart_beta", "exit 0\n");
  EXPECT_EQ(RunSession(options), 1);
  EXPECT_FALSE(std::filesystem::exists(options.output_dir + "/manifest.json"));
  std::filesystem::remove_all(options.output_dir);
}

TEST(Supervision, StalledNodeGroupKillStopsItsHelper) {
  auto options = BaseOptions("helper_out");
  options.max_restarts = 0;
  options.heartbeat_startup_grace_ms = 0;
  options.duration_s = kHangSeconds;
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  options.supervision_clock = clock.clock();
  const auto helper_pid = TempPath("helper_pid");
  ::remove(helper_pid.c_str());
  int lifetime[2];
  ASSERT_EQ(::pipe(lifetime), 0);
  // The helper inherits this writer across exec. EOF proves it stopped, even
  // on hosts where an orphaned helper briefly remains as a zombie.
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "helper_alpha", "sleep 30 &\necho $! > " + helper_pid + ".tmp && mv " +
                          helper_pid + ".tmp " + helper_pid + "\nwait\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("helper_beta", "exit 0\n");
  testing::SupervisionPasses passes;
  bool helper_started = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    helper_started = WaitFor(stop, [&] {
      pid_t pid = 0;
      std::ifstream in{helper_pid};
      return (in >> pid) && pid > 0;
    });
    if (!helper_started) return;
    const uint64_t next = passes.Started() + 2;
    if (!WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          return passes.Started() >= next;
        })) {
      return;
    }
    clock.HoldAt(start + kMinHeartbeatTimeout);
  }};
  EXPECT_NE(RunSession(options), 0);
  driver.request_stop();
  driver.join();
  EXPECT_TRUE(helper_started) << "alpha's helper never wrote its pid";
  ::close(lifetime[1]);
  pollfd ready{lifetime[0], POLLIN, 0};
  const int result = ::poll(&ready, 1, kHangSeconds * 1000);
  char byte = 0;
  EXPECT_GT(result, 0);
  if (result > 0) {
    EXPECT_EQ(::read(lifetime[0], &byte, 1), 0);
  }
  ::close(lifetime[0]);
  pid_t helper = 0;
  std::ifstream{helper_pid} >> helper;
  EXPECT_GT(helper, 0);
  // Also clean up on a regression, rather than leaking the fixture's helper.
  if (helper > 0) ::kill(helper, SIGKILL);
  std::filesystem::remove_all(options.output_dir);
  ::remove(helper_pid.c_str());
}

TEST(Supervision, KillingLauncherKillsNodeAndItsHelper) {
  int output[2];
  ASSERT_EQ(::pipe(output), 0);
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) {
    ::close(output[0]);
    std::string error;
    if (::dup2(output[1], STDOUT_FILENO) < 0) ::_exit(1);
    ::close(output[1]);
    const auto node =
        SpawnProcess({"/bin/sh", "-c", "sleep 30 & echo $$; wait"}, error);
    if (node < 0) ::_exit(1);
    for (;;) ::pause();
  }
  ::close(output[1]);
  pollfd ready{output[0], POLLIN, 0};
  const int started = ::poll(&ready, 1, kHangSeconds * 1000);
  char text[64]{};
  ssize_t size = 0;
  if (started > 0) size = ::read(output[0], text, sizeof(text) - 1);
  ::kill(launcher, SIGKILL);
  int status = 0;
  while (::waitpid(launcher, &status, 0) < 0 && errno == EINTR) {
  }
  const int stopped = ::poll(&ready, 1, kHangSeconds * 1000);
  char byte = 0;
  EXPECT_GT(started, 0);
  EXPECT_GT(size, 0);
  EXPECT_GT(stopped, 0);
  if (stopped > 0) {
    EXPECT_EQ(::read(output[0], &byte, 1), 0);
  }
  if (size > 0) {
    SignalProcessGroup(static_cast<pid_t>(std::strtol(text, nullptr, 10)),
                       SIGKILL);
  }
  ::close(output[0]);
}

// A guardian never execs, so a descriptor its launcher's process held at the
// spawn would stay open in it for as long as the node runs, close-on-exec or
// not: another thread's registry lock (close-on-exec, like this one) held that
// way would block every later registration.
TEST(Supervision, GuardianKeepsNoLockHeldAtSpawn) {
  const std::string path = TempPath("guardian_lock");
  const int held = ::open(path.c_str(), O_CREAT | O_RDONLY | O_CLOEXEC, 0600);
  const int probe = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  ASSERT_GE(held, 0);
  ASSERT_GE(probe, 0);
  ASSERT_EQ(::flock(held, LOCK_EX), 0);
  std::string error;
  const pid_t node = SpawnProcess({"/bin/sh", "-c", "exec sleep 30"}, error);
  ::close(held);
  ASSERT_GT(node, 0) << error;
  // The node's own copy closes at its exec; then nothing may hold the lock.
  bool released = ::flock(probe, LOCK_EX | LOCK_NB) == 0;
  // The exec is the event; the deadline only bounds a hang (§6.3).
  for (const auto give_up = std::chrono::steady_clock::now() +
                            std::chrono::seconds{kHangSeconds};
       !released && std::chrono::steady_clock::now() < give_up;) {
    std::this_thread::sleep_for(10ms);
    released = ::flock(probe, LOCK_EX | LOCK_NB) == 0;
  }
  EXPECT_TRUE(released) << "the node's guardian still holds the lock";
  ::close(probe);
  SignalProcessGroup(node, SIGKILL);
  while (::waitpid(node, nullptr, 0) < 0 && errno == EINTR) {
  }
  FinishProcessGroup(node);
  ::remove(path.c_str());
}

// The real supervision loop, not a watchdog thread, must deliver READY before
// its first WATCHDOG. A hung loop consequently cannot keep the unit healthy.
// REQ-OPS-07
TEST(Supervision, SendsReadyThenWatchdogDatagrams) {
  const std::string path = "/tmp/tn_" + std::to_string(::getpid());
  ::unlink(path.c_str());
  const int socket = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  ASSERT_GE(socket, 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  ASSERT_EQ(::bind(socket, reinterpret_cast<const sockaddr*>(&address),
                   static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                          path.size() + 1)),
            0);
  auto options = BaseOptions("notify_out");
  options.max_restarts = 0;
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("notify_alpha", "sleep 0.1\nexit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("notify_beta", "exit 0\n");
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) {
    ::close(socket);
    ::setenv("NOTIFY_SOCKET", path.c_str(), 1);
    ::_exit(Launcher{options}.Run());
  }
  std::vector<std::string> messages;
  for (int i = 0; i < 2; ++i) {
    pollfd ready{socket, POLLIN, 0};
    if (::poll(&ready, 1, kHangSeconds * 1000) <= 0) break;
    char buffer[64];
    const auto size = ::recv(socket, buffer, sizeof(buffer), 0);
    if (size > 0) messages.emplace_back(buffer, static_cast<std::size_t>(size));
  }
  int status = 0;
  while (::waitpid(launcher, &status, 0) < 0 && errno == EINTR) {
  }
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_EQ(messages, (std::vector<std::string>{"READY=1", "WATCHDOG=1"}));
  ::close(socket);
  ::unlink(path.c_str());
  std::filesystem::remove_all(options.output_dir);
}

// The service watchdog is what bounds a deployed session whose launcher
// hangs (README "Supervision passes"): with the loop held inside a pass (its
// clock read blocked), no WATCHDOG=1 goes out for three watchdog intervals,
// and the pings resume once the loop does. A watchdog thread, or any ping not
// sent by the loop itself, fails it.
// REQ-OPS-07
TEST(Supervision, AStalledLoopStopsFeedingTheServiceWatchdog) {
  const std::string path = "/tmp/tw_" + std::to_string(::getpid());
  ::unlink(path.c_str());
  const int socket = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  ASSERT_GE(socket, 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  ASSERT_EQ(::bind(socket, reinterpret_cast<const sockaddr*>(&address),
                   static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                          path.size() + 1)),
            0);
  // 100 ms between pings (half of WATCHDOG_USEC).
  constexpr auto kInterval = 100ms;
  auto options = BaseOptions("watchdog_stall");
  // Only bounds a hang: the test stops the session once it has judged it,
  // and its own waits add up to at most about 60 s.
  options.duration_s = 4 * kHangSeconds;
  // The stubs never register; no heartbeat kill may end them meanwhile.
  options.heartbeat_startup_grace_ms = 60'000;
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("watchdog_stall_hardware", kRunUntilStopped, "hardware_node");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("watchdog_stall_alpha", kRunUntilStopped);
  options.binary_overrides["//example/beta:node"] =
      WriteStub("watchdog_stall_beta", kRunUntilStopped);
  std::mutex mutex;
  std::condition_variable changed;
  bool stall = false;
  bool stalled = false;
  options.supervision_clock = [&] {
    std::unique_lock lock{mutex};
    if (stall) {
      stalled = true;
      changed.notify_all();
      changed.wait(lock, [&] { return !stall; });
    }
    return std::chrono::steady_clock::now();
  };
  // Waits at most `bound` for one datagram: its text, or empty.
  const auto receive =
      [socket](std::chrono::milliseconds bound) -> std::string {
    pollfd ready{socket, POLLIN, 0};
    if (::poll(&ready, 1, static_cast<int>(bound.count())) <= 0) return {};
    char buffer[64];
    const auto size = ::recv(socket, buffer, sizeof(buffer), 0);
    return size > 0 ? std::string(buffer, static_cast<std::size_t>(size))
                    : std::string{};
  };
  bool pinged = false;
  bool held = false;
  std::string while_stalled;
  bool resumed = false;
  std::jthread driver{[&] {
    const auto hang = std::chrono::seconds{kHangSeconds};
    for (int i = 0; i < 2 && !pinged; ++i)
      pinged = receive(hang) == "WATCHDOG=1";
    if (pinged) {
      {
        std::unique_lock lock{mutex};
        stall = true;
      }
      SupervisionWakeup::Wake();
      std::unique_lock lock{mutex};
      held = changed.wait_for(lock, hang, [&] { return stalled; });
    }
    if (held) {
      // Sent before the loop stopped: queued already, drained here.
      while (!receive(0ms).empty()) {
      }
      while_stalled = receive(3 * kInterval);
    }
    {
      const std::lock_guard lock{mutex};
      stall = false;
    }
    changed.notify_all();
    if (held) resumed = receive(hang) == "WATCHDOG=1";
    // Unpinged, the session is stuck before its loop; its deadline ends it.
    if (pinged) ::kill(::getpid(), SIGTERM);
  }};
  ::setenv("NOTIFY_SOCKET", path.c_str(), 1);
  ::setenv("WATCHDOG_USEC", "200000", 1);
  const int status = RunSession(options);
  ::unsetenv("NOTIFY_SOCKET");
  ::unsetenv("WATCHDOG_USEC");
  driver.join();
  EXPECT_EQ(status, 0);
  ASSERT_TRUE(pinged) << "no WATCHDOG=1 before the stall";
  ASSERT_TRUE(held) << "the loop never read its clock once told to stall";
  EXPECT_EQ(while_stalled, "")
      << "a ping went out while the loop was held for "
      << (3 * kInterval).count() << " ms (3 watchdog intervals)";
  EXPECT_TRUE(resumed) << "no WATCHDOG=1 after the loop resumed";
  ::close(socket);
  ::unlink(path.c_str());
  std::filesystem::remove_all(options.output_dir);
}

#ifdef __linux__
TEST(Supervision, NotifierSupportsAbstractSocketNames) {
  // Abstract names live in the network namespace, which concurrent sandboxed
  // runs of this test share while their pids repeat across PID namespaces.
  const std::string path = "@tn_" + std::to_string(GenerateSessionId());
  const int socket = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  ASSERT_GE(socket, 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.data(), path.size());
  address.sun_path[0] = '\0';
  ASSERT_EQ(::bind(socket, reinterpret_cast<const sockaddr*>(&address),
                   static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                          path.size())),
            0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(socket);
    ::setenv("NOTIFY_SOCKET", path.c_str(), 1);
    ServiceNotifier notifier;
    notifier.Send("WATCHDOG=1");
    ::_exit(0);
  }
  pollfd ready{socket, POLLIN, 0};
  const int received = ::poll(&ready, 1, kHangSeconds * 1000);
  char buffer[64]{};
  ssize_t size = 0;
  if (received > 0) size = ::recv(socket, buffer, sizeof(buffer), 0);
  int status = 0;
  ::waitpid(child, &status, 0);
  EXPECT_EQ(
      std::string(buffer, static_cast<std::size_t>(std::max<ssize_t>(size, 0))),
      "WATCHDOG=1");
  ::close(socket);
}
#endif

// The registry lock is a node's while it creates or claims in the registry,
// and a node can hold it for good: stopped or stuck mid-registration. The
// launcher's loop, which feeds the service watchdog, must not wait on it.
// With the lock held all session, every member starts and the session still
// stops when asked (SIGTERM, once every member has set its stop trap).
// REQ-OPS-07
TEST(Supervision, HeldRegistryLockDoesNotStallSupervision) {
  auto options = BaseOptions("held_lock");
  options.duration_s = kHangSeconds;
  const auto runs = TempPath("held_lock_runs");
  ::remove(runs.c_str());
  for (const std::string name : {"hardware_node", "alpha", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + name + ":node";
    options.binary_overrides[target] = WriteStub(
        "held_lock_" + name, TrappedRun(runs, kLoopUntilStopped), name);
  }
  // The launcher waits on this lock at startup only to create the registry
  // (PrepareRegistry); here it exists, as after any earlier session.
  introspect::PrepareRegistry();
  const introspect::RegistryLock held{ShmObjectName(introspect::kRegistryPath)};
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) ::_exit(Launcher{options}.Run());
  const auto give_up =
      std::chrono::steady_clock::now() + std::chrono::seconds{kHangSeconds};
  while (ReadLines(runs).size() < 3 &&
         std::chrono::steady_clock::now() < give_up)
    std::this_thread::sleep_for(10ms);
  const bool started = ReadLines(runs).size() == 3;
  ::kill(launcher, SIGTERM);
  int status = 0;
  pid_t reaped = 0;
  while ((reaped = ::waitpid(launcher, &status, WNOHANG)) == 0 &&
         std::chrono::steady_clock::now() <
             give_up + std::chrono::seconds{kHangSeconds})
    std::this_thread::sleep_for(10ms);
  if (reaped != launcher) {
    ::kill(launcher, SIGKILL);
    while (::waitpid(launcher, &status, 0) < 0 && errno == EINTR) {
    }
  }
  EXPECT_TRUE(started) << "not every member started";
  EXPECT_EQ(reaped, launcher) << "supervision stalled on the registry lock";
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_EQ(ReadLines(runs).size(), 3u);
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// One session per shared-memory namespace: a second launcher would unlink the
// running session's topics. It is refused before it writes or starts
// anything, naming the holder; --describe-only, which starts nothing, still
// runs; and the lock dies with its holder.
// REQ-OPS-02
TEST(Supervision, SecondLaunchInTheNamespaceIsRefused) {
  int ready[2];
  ASSERT_EQ(::pipe(ready), 0);
  const pid_t holder = ::fork();
  ASSERT_GE(holder, 0);
  if (holder == 0) {
    ::close(ready[0]);
    std::string error;
    const auto lock = SessionLock::Acquire(error);
    const char held = lock ? 1 : 0;
    if (::write(ready[1], &held, 1) != 1 || !lock) ::_exit(1);
    for (;;) ::pause();
  }
  ::close(ready[1]);
  char held = 0;
  const bool reported = ::read(ready[0], &held, 1) == 1;
  ::close(ready[0]);
  const auto with_stubs = [](std::string_view tag, bool describe_only) {
    auto options = BaseOptions(tag);
    options.describe_only = describe_only;
    options.binary_overrides["//example/alpha:node"] =
        WriteStub(std::string{tag} + "_alpha", "exit 0\n");
    options.binary_overrides["//example/beta:node"] =
        WriteStub(std::string{tag} + "_beta", "exit 0\n");
    return options;
  };
  auto second = with_stubs("second_launch", false);
  auto describe = with_stubs("second_describe", true);
  ::testing::internal::CaptureStderr();
  const int refused = RunSession(second);
  const std::string error = ::testing::internal::GetCapturedStderr();
  const int described = RunSession(describe);
  ::kill(holder, SIGKILL);
  while (::waitpid(holder, nullptr, 0) < 0 && errno == EINTR) {
  }
  ASSERT_TRUE(reported && held == 1);
  EXPECT_EQ(refused, 1);
  EXPECT_NE(error.find("pid " + std::to_string(holder)), std::string::npos)
      << error;
  EXPECT_FALSE(std::filesystem::exists(second.output_dir));
  EXPECT_EQ(described, 0);
  auto after = with_stubs("after_holder", false);
  EXPECT_EQ(RunSession(after), 0);
  for (const auto* options : {&second, &describe, &after}) {
    std::filesystem::remove_all(options->output_dir);
  }
}

// The talos-*.lock files in the lock directory (introspect::LockDirectory:
// this test's TEST_TMPDIR under Bazel, /dev/shm under TALOS_SHM_NAMESPACE).
std::vector<std::string> LockFiles() {
  std::vector<std::string> names;
  std::error_code error;
  for (const auto& entry : std::filesystem::directory_iterator{
           introspect::LockDirectory(), error}) {
    const std::string name = entry.path().filename().string();
    if (name.starts_with("talos-") && name.ends_with(".lock"))
      names.push_back(name);
  }
  std::sort(names.begin(), names.end());
  return names;
}

// Whether the shared-memory object `mapped` (a ShmObjectName) exists.
bool SegmentExists(const std::string& mapped) {
  const int fd = ::shm_open(mapped.c_str(), O_RDONLY, 0);
  if (fd >= 0) ::close(fd);
  return fd >= 0;
}

// A session in a shared-memory namespace leaves none of its names behind:
// the registry segment it prepared, the registry's writer lock file, and the
// session lock file all exist while it runs and are gone once it has
// returned, every member reaped (a host that runs namespaced sessions all
// day otherwise collects three per session).
// REQ-OPS-03
TEST(Supervision, ANamespacedSessionLeavesNoNamesBehind) {
  const std::string registry = ShmObjectName(introspect::kRegistryPath);
  ASSERT_TRUE(registry.starts_with("/~"))
      << "not in a test namespace: " << registry;
  const std::string registry_lock = introspect::LockFilePath(registry);
  const std::string session_lock =
      introspect::LockFilePath(ShmObjectName(kSessionLockName));
  auto options = BaseOptions("leaves_no_names");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("leaves_no_names_alpha", "exit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("leaves_no_names_beta", "exit 0\n");
  const std::vector<std::string> before = LockFiles();
  bool held_all = false;
  options.member_spawned = [&](const std::string&) {
    held_all = SegmentExists(registry) &&
               std::filesystem::exists(registry_lock) &&
               std::filesystem::exists(session_lock);
  };
  EXPECT_EQ(RunSession(options), 0);
  ASSERT_TRUE(held_all) << "the running session did not hold " << registry
                        << ", " << registry_lock << " and " << session_lock;
  EXPECT_FALSE(SegmentExists(registry)) << registry;
  EXPECT_FALSE(std::filesystem::exists(registry_lock)) << registry_lock;
  EXPECT_FALSE(std::filesystem::exists(session_lock)) << session_lock;
  const std::vector<std::string> after = LockFiles();
  std::vector<std::string> added;
  std::set_difference(after.begin(), after.end(), before.begin(), before.end(),
                      std::back_inserter(added));
  EXPECT_TRUE(added.empty()) << "left in " << introspect::LockDirectory()
                             << ": " << ::testing::PrintToString(added);
  std::filesystem::remove_all(options.output_dir);
}

// Removes the namespace's registry segment and its lock file, which a test
// that keeps them leaves behind, so the tests after it start as this binary's
// first does.
void RemoveRegistry() {
  introspect::RegistryMapping::unlink();
  ::unlink(introspect::LockFilePath(ShmObjectName(introspect::kRegistryPath))
               .c_str());
}

// A namespaced session's end leaves the registry while a live process of
// another session holds a row in it: removing the segment would take that
// node's row out of every viewer, and its next restart would register in a
// new segment. The row's claimant is this test process, which runs.
// REQ-OPS-03
TEST(Supervision, ALiveRowOfAnotherSessionKeepsTheRegistry) {
  const std::string registry = ShmObjectName(introspect::kRegistryPath);
  const std::string registry_lock = introspect::LockFilePath(registry);
  auto options = BaseOptions("foreign_row");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("foreign_row_alpha", "exit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("foreign_row_beta", "exit 0\n");
  std::optional<introspect::NodeRegistration> foreign;
  foreign.emplace(introspect::NodeRegistration::Identity{
      "foreign", "//example/foreign:node", options.session_id + 1, 0});
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_TRUE(SegmentExists(registry))
      << "removed under another session's live row: " << registry;
  EXPECT_TRUE(std::filesystem::exists(registry_lock)) << registry_lock;
  foreign.reset();
  RemoveRegistry();
  std::filesystem::remove_all(options.output_dir);
}

// The same while another process holds the registry's writer lock (a node of
// another session claiming a row): the session does not wait for it and
// leaves the registry. Held here from before the session, once the registry
// is prepared, so the launcher's own preparation needs no lock.
// REQ-OPS-03
TEST(Supervision, AHeldRegistryLockKeepsTheRegistry) {
  const std::string registry = ShmObjectName(introspect::kRegistryPath);
  const std::string registry_lock = introspect::LockFilePath(registry);
  auto options = BaseOptions("held_registry_lock");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("held_registry_lock_alpha", "exit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("held_registry_lock_beta", "exit 0\n");
  introspect::PrepareRegistry();
  std::optional<introspect::RegistryLock> held;
  held.emplace(registry, 0ms);
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_TRUE(SegmentExists(registry))
      << "removed while its lock was held: " << registry;
  EXPECT_TRUE(std::filesystem::exists(registry_lock)) << registry_lock;
  held.reset();
  RemoveRegistry();
  std::filesystem::remove_all(options.output_dir);
}

// A row an earlier session left active, its claimant gone (SIGKILLed: its
// heartbeat still fresh, its row not yet swept by a claim), is no live
// session's and does not keep the registry for good. The planted row's
// claimant word is this process's with its start time changed, which reads
// as the pid reused by a later process (process_liveness.h).
// REQ-OPS-03
TEST(Supervision, ARowWhoseProcessIsGoneDoesNotKeepTheRegistry) {
  const std::string registry = ShmObjectName(introspect::kRegistryPath);
  auto options = BaseOptions("gone_row");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("gone_row_alpha", "exit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("gone_row_beta", "exit 0\n");
  const uint64_t gone = talos::OwnProcessWord() ^ (1ULL << kProcessPidBits);
  ASSERT_FALSE(talos::ProcessWordAlive(gone));
  // Held across the session and released at its end, so the row is freed
  // whether or not the registry was removed.
  const introspect::NodeRegistration registration{
      {"gone", "//example/gone:node", options.session_id + 1, 0}};
  bool planted = false;
  {
    introspect::RegistryMapping mapping;
    for (auto& row : mapping.header()->nodes) {
      if (row.state.load(std::memory_order_acquire) !=
              static_cast<uint64_t>(introspect::SlotState::ACTIVE) ||
          row.session_id != options.session_id + 1)
        continue;
      row.pid_start = gone >> kProcessPidBits;
      planted = true;
      break;
    }
  }
  ASSERT_TRUE(planted) << "no row to plant";
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_FALSE(SegmentExists(registry))
      << "kept for a row whose process is gone: " << registry;
  RemoveRegistry();
  std::filesystem::remove_all(options.output_dir);
}

// The loop never waits for the disk, yet every mid-session manifest is
// synced. With every sync stuck from alpha's first run on, the critical
// alpha's crash is still restarted at once; once the disk recovers, a synced
// manifest of the running session records the restart (PersistManifest,
// ManifestWriter).
// REQ-OPS-08
TEST(Supervision, StuckManifestSyncDelaysNoRestartAndSyncsItLater) {
  auto options = BaseOptions("stuck_sync");
  // The stubs never register; no heartbeat kill may restart them meanwhile.
  options.heartbeat_startup_grace_ms = 60'000;
  const auto runs = TempPath("stuck_sync_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("stuck_sync_hardware", kRunUntilStopped, "hardware_node");
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "stuck_sync_alpha",
      RecordLogArgThen(runs, "case \"$log\" in *alpha.tlog) exit 3;; esac\n" +
                                 std::string{kRunUntilStopped}));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("stuck_sync_beta", kRunUntilStopped);
  // RunSession rewrites options.output_dir on return; don't read it
  // concurrently.
  const auto live_manifest = options.output_dir + "/" +
                             std::to_string(options.session_id) +
                             "/manifest.json";
  std::mutex mutex;
  std::condition_variable release;
  bool released = false;
  std::atomic<bool> synced_restart{false};
  options.sync_file = [&](int fd) {
    if (!ReadLines(runs).empty()) {
      std::unique_lock lock{mutex};
      release.wait(lock, [&] { return released; });
    }
    const int result = ::fsync(fd);
    // The directory is synced after the rename: manifest.json is the
    // version just made durable.
    struct stat status {};
    if (::fstat(fd, &status) == 0 && S_ISDIR(status.st_mode)) {
      const std::string text = ReadFile(live_manifest);
      if (ManifestEntry(text, "alpha").find("\"restart_count\": 1,") !=
              std::string::npos &&
          text.find("\n  \"end_wall_ns\": 0,") != std::string::npos)
        synced_restart = true;
    }
    return result;
  };
  bool restarted = false;
  std::jthread stopper{[&](const std::stop_token& stop) {
    const auto give_up = std::chrono::steady_clock::now() + 20s;
    while (!stop.stop_requested() &&
           std::chrono::steady_clock::now() < give_up) {
      if (ReadLines(runs).size() >= 2) {
        restarted = true;
        break;
      }
      std::this_thread::sleep_for(2ms);
    }
    {
      const std::lock_guard lock{mutex};
      released = true;
    }
    release.notify_all();
    while (!stop.stop_requested() && !synced_restart &&
           std::chrono::steady_clock::now() < give_up) {
      std::this_thread::sleep_for(2ms);
    }
    if (!stop.stop_requested()) ::kill(::getpid(), SIGTERM);
  }};
  RunSession(options);
  stopper.request_stop();
  stopper.join();
  EXPECT_TRUE(restarted) << "the restart waited for a manifest sync";
  EXPECT_TRUE(synced_restart)
      << "no manifest synced mid-session recorded the restart";
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// REQ-OPS-08: manifest.json is written before spawn 0 returns (verified inside
// member_spawned) and records each member's binary_sha256 matching
// FileSha256(binary_path).
TEST(Supervision, ManifestExistsBeforeFirstSpawnWithBinaryDigests) {
  auto options = BaseOptions("manifest_before_spawn");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("manifest_before_spawn_alpha", "exit 0\n", "alpha");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("manifest_before_spawn_beta", "exit 0\n", "beta");
  const std::string live_manifest = options.output_dir + "/" +
                                    std::to_string(options.session_id) +
                                    "/manifest.json";
  const std::string hw_sha =
      FileSha256(options.binary_overrides["//talOS/bridge:hardware_node"]);
  const std::string alpha_sha =
      FileSha256(options.binary_overrides["//example/alpha:node"]);
  const std::string beta_sha =
      FileSha256(options.binary_overrides["//example/beta:node"]);
  ASSERT_EQ(hw_sha.size(), 64u);
  ASSERT_EQ(alpha_sha.size(), 64u);
  ASSERT_EQ(beta_sha.size(), 64u);

  int spawn_calls = 0;
  bool existed_at_first_spawn = false;
  std::string manifest_at_first_spawn;
  options.member_spawned = [&](const std::string&) {
    if (spawn_calls++ == 0) {
      existed_at_first_spawn = std::filesystem::exists(live_manifest);
      manifest_at_first_spawn = ReadFile(live_manifest);
    }
  };
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(spawn_calls, 3);
  EXPECT_TRUE(existed_at_first_spawn)
      << "manifest.json did not exist inside member_spawned before spawn 0 "
         "returned";
  const std::string final_manifest =
      ReadFile(options.output_dir + "/manifest.json");
  const std::array<std::pair<std::string_view, const std::string*>, 3> expected{
      {{"hardware_node", &hw_sha}, {"alpha", &alpha_sha}, {"beta", &beta_sha}}};
  for (const auto& [name, sha] : expected) {
    SCOPED_TRACE(name);
    const std::string needle = "\"binary_sha256\": \"" + *sha + "\"";
    EXPECT_NE(ManifestEntry(manifest_at_first_spawn, name).find(needle),
              std::string::npos)
        << manifest_at_first_spawn;
    EXPECT_NE(ManifestEntry(final_manifest, name).find(needle),
              std::string::npos)
        << final_manifest;
  }
  std::filesystem::remove_all(options.output_dir);
}

// Supplies a shell stub's registry row from the fixture, without giving the
// launcher a writable mapping or requiring a real framework node binary: once
// the stub has written its pid to `pid_file`, claims a row, hands it to that
// pid, and then calls `tick` on it every 5 ms, as a running node would.
std::jthread PublishStubRow(const LauncherOptions& options,
                            const std::string& pid_file,
                            std::function<void(introspect::NodeRecord&)> tick) {
  return std::jthread{[session_id = options.session_id, pid_file,
                       tick = std::move(tick)](const std::stop_token& stop) {
    pid_t pid = 0;
    while (!stop.stop_requested()) {
      std::ifstream input{pid_file};
      if (input >> pid) break;
      std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
    if (pid <= 0) return;
    introspect::NodeRegistration registration{
        {"alpha", "//example/alpha:node", session_id, 0}};
    introspect::RegistryMapping mapping;
    introspect::NodeRecord* stub = nullptr;
    for (auto& row : mapping.header()->nodes) {
      if (row.state.load(std::memory_order_acquire) !=
          static_cast<uint64_t>(introspect::SlotState::ACTIVE)) {
        continue;
      }
      if (row.session_id == session_id &&
          row.pid == static_cast<uint64_t>(::getpid())) {
        row.state.store(static_cast<uint64_t>(introspect::SlotState::CLAIMING),
                        std::memory_order_release);
        row.pid = static_cast<uint64_t>(pid);
        row.pid_start = 0;  // The stub's start is unknown: compare its pid.
        row.heartbeat_monotonic_ns.store(introspect::monotonic_now_ns());
        row.dispatch_count.store(1);
        row.state.store(static_cast<uint64_t>(introspect::SlotState::ACTIVE),
                        std::memory_order_release);
        stub = &row;
        break;
      }
    }
    while (!stop.stop_requested()) {
      if (stub != nullptr) tick(*stub);
      std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
  }};
}

// A loop blocked inside the node while its reporting thread still heartbeats:
// the dispatch count, which only the loop moves, stops, and the launcher kills
// the run as stalled and restarts it.
// REQ-SAFE-29, REQ-OPS-05
TEST(Supervision, StalledLoopIsKilledAndRestartedThoughItsHeartbeatMoves) {
  auto options = BaseOptions("stalled_out", "max_restarts = 1\n");
  constexpr auto kGrace = 1000ms;
  options.heartbeat_startup_grace_ms = static_cast<int>(kGrace.count());
  options.duration_s = kHangSeconds;
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  options.supervision_clock = clock.clock();
  const auto pid_file = TempPath("stalled_pid");
  const auto runs = TempPath("stalled_runs");
  ::remove(pid_file.c_str());
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "stalled_alpha",
      RecordLogArgThen(runs,
                       "case \"$log\" in *restart1.tlog) exit 0;; esac\n"
                       "echo $$ > " +
                           pid_file + ".tmp && mv " + pid_file + ".tmp " +
                           pid_file + "\nexec sleep 60\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("stalled_beta", "exit 0\n");
  auto publisher =
      PublishStubRow(options, pid_file, [](introspect::NodeRecord& row) {
        row.heartbeat_monotonic_ns.store(introspect::monotonic_now_ns());
      });
  const uint64_t session = options.session_id;
  const std::string live =
      options.output_dir + "/" + std::to_string(session) + "/manifest.json";
  testing::SupervisionPasses passes;
  bool stepped = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    if (!WaitFor(stop, [&] { return Registered("alpha", session); })) return;
    const uint64_t next = passes.Started() + 2;
    if (!WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          return passes.Started() >= next;
        })) {
      return;
    }
    // Step run 0 past both startup grace and kMinHeartbeatTimeout so run 0 is
    // killed as stalled and restarted cleanly; while the clock stays frozen at
    // start + kGrace, run 1 and its peers exit cleanly without hitting any
    // heartbeat timeout or starting a second restart.
    clock.HoldAt(start + kGrace);
    if (!WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          const std::string manifest = ReadFile(live);
          const auto alpha_runs = ManifestRuns(manifest, "alpha");
          if (alpha_runs.size() < 2) return false;
          for (const char* name : {"hardware_node", "alpha", "beta"}) {
            const auto member_runs = ManifestRuns(manifest, name);
            if (member_runs.empty() ||
                member_runs.back().find("\"reason\": \"running\"") !=
                    std::string::npos) {
              return false;
            }
          }
          return true;
        })) {
      return;
    }
    stepped = true;
    // Past every clean-exit backoff (100 ms for restart 2) so RosterContinues
    // cancels every pending restart and ends the session at once.
    clock.HoldAt(start + kGrace + 200ms);
    WaitFor(stop, [&] {
      SupervisionWakeup::Wake();
      return false;
    });
  }};
  EXPECT_EQ(RunSession(options), 0);
  driver.request_stop();
  driver.join();
  publisher.request_stop();
  publisher.join();
  EXPECT_TRUE(stepped) << "stalled alpha was not killed and restarted";
  EXPECT_EQ(ReadLines(runs).size(), 2u);
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 1"), std::string::npos);
  // The kill is in the session, not only on the launcher's stderr: the
  // stalled run ended by the launcher's SIGKILL, for a stall.
  const auto history = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(history.size(), 2u) << manifest;
  EXPECT_NE(history[0].find("\"exit_signal\": " + std::to_string(SIGKILL) +
                            ", \"reason\": \"stalled\""),
            std::string::npos)
      << history[0];
  EXPECT_NE(history[1].find("\"reason\": \"exit\""), std::string::npos)
      << history[1];
  std::filesystem::remove_all(options.output_dir);
  ::remove(pid_file.c_str());
  ::remove(runs.c_str());
}

// A run that never registers is killed as heartbeat_missing and restarted,
// even when --allow-graph-errors let its failed probe through.
// REQ-OPS-05
TEST(Supervision, UndescribedNodeAllowedExplicitlyStillMustRegister) {
  auto options = BaseOptions("undescribed_out", "max_restarts = 1\n");
  options.allow_graph_errors = true;
  options.simulation = true;  // The flag requires --sim.
  options.heartbeat_startup_grace_ms = 0;
  options.duration_s = kHangSeconds;
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  options.supervision_clock = clock.clock();
  const auto runs = TempPath("undescribed_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "undescribed_alpha",
      RecordLogArgThen(runs,
                       "case \"$log\" in *restart1.tlog) exit 0;; esac\n"
                       "exec sleep 60\n"),
      "alpha", false);
  options.binary_overrides["//example/beta:node"] =
      WriteStub("undescribed_beta", "exit 0\n");
  const uint64_t session = options.session_id;
  const std::string live =
      options.output_dir + "/" + std::to_string(session) + "/manifest.json";
  testing::SupervisionPasses passes;
  bool stepped = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    // Wait until alpha run 0 is up and its peers' run 0s have exited cleanly
    // and been reaped, so advancing to start + kMinHeartbeatTimeout cannot
    // SIGKILL a still-exiting peer under heartbeat_startup_grace_ms = 0.
    if (!WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          if (ReadLines(runs).empty()) return false;
          const std::string manifest = ReadFile(live);
          for (const char* name : {"hardware_node", "beta"}) {
            const auto member_runs = ManifestRuns(manifest, name);
            if (member_runs.empty() ||
                member_runs.back().find("\"reason\": \"running\"") !=
                    std::string::npos) {
              return false;
            }
          }
          return true;
        })) {
      return;
    }
    const uint64_t next = passes.Started() + 2;
    if (!WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          return passes.Started() >= next;
        })) {
      return;
    }
    // Step run 0 past kMinHeartbeatTimeout so unregistered alpha is killed as
    // heartbeat_missing and restarted cleanly; while the clock stays frozen at
    // start + kMinHeartbeatTimeout, run 1 of alpha and its peers exit cleanly
    // without hitting any heartbeat timeout or starting a second restart.
    clock.HoldAt(start + kMinHeartbeatTimeout);
    if (!WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          const std::string manifest = ReadFile(live);
          for (const char* name : {"hardware_node", "alpha", "beta"}) {
            const auto member_runs = ManifestRuns(manifest, name);
            if (member_runs.size() < 2 ||
                member_runs.back().find("\"reason\": \"running\"") !=
                    std::string::npos) {
              return false;
            }
          }
          return true;
        })) {
      return;
    }
    stepped = true;
    // Past every clean-exit backoff (100 ms for restart 2) so RosterContinues
    // cancels every pending restart and ends the session at once.
    clock.HoldAt(start + kMinHeartbeatTimeout + 200ms);
    WaitFor(stop, [&] {
      SupervisionWakeup::Wake();
      return false;
    });
  }};
  EXPECT_EQ(RunSession(options), 0);
  driver.request_stop();
  driver.join();
  EXPECT_TRUE(stepped) << "unregistered alpha was not killed and restarted";
  EXPECT_EQ(ReadLines(runs).size(), 2u);
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"described\": false"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"restart_count\": 1"), std::string::npos) << alpha;
  const auto history = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(history.size(), 2u) << manifest;
  EXPECT_NE(history[0].find("\"reason\": \"heartbeat_missing\""),
            std::string::npos)
      << history[0];
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// An idle session -- every member running, none registered, nothing due --
// is supervised at its members' look interval, not on a fixed poll: no two
// passes start closer than one look interval (LivenessLookInterval of the
// 500 ms heartbeat timeout every member here has, its floor), so a window in
// which k passes started is at least k - 1 intervals long. Load only spaces
// passes further apart. The window ends at an event (the kth pass), not a
// duration. The 20 ms poll this replaced started 8 passes in 163 ms.
// REQ-OPS-45
TEST(Supervision, IdleSessionPassesOncePerLookIntervalNotOnAPoll) {
  auto options = BaseOptions("idle_passes");
  options.max_restarts = 0;
  options.duration_s = kHangSeconds;
  // The stubs never register; no heartbeat_missing kill ends the window.
  options.heartbeat_startup_grace_ms = 60'000;
  for (const std::string name : {"hardware_node", "alpha", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + name + ":node";
    options.binary_overrides[target] =
        WriteStub("idle_passes_" + name, "exec sleep 60\n", name);
  }
  // The shortest period here (alpha's, the robot's) gives the shortest
  // timeout, and with it the shortest look interval.
  const std::chrono::nanoseconds look = LivenessLookInterval(
      {HeartbeatTimeout(5000, options.heartbeat_periods), {}, {}});
  constexpr uint64_t kPasses = 8;
  testing::SupervisionPasses passes;
  uint64_t counted = 0;
  std::chrono::nanoseconds window{};
  std::jthread stopper{[&](const std::stop_token& stop) {
    const auto give_up =
        std::chrono::steady_clock::now() + std::chrono::seconds{kHangSeconds};
    const auto wait = [&](const std::function<bool()>& done) {
      while (!stop.stop_requested() && !done() &&
             std::chrono::steady_clock::now() < give_up)
        std::this_thread::sleep_for(1ms);
    };
    // The first pass runs once every member has spawned.
    wait([&] { return passes.Started() > 0; });
    const uint64_t base = passes.Started();
    const auto from = std::chrono::steady_clock::now();
    wait([&] { return passes.Started() - base >= kPasses; });
    counted = passes.Started() - base;
    window = std::chrono::steady_clock::now() - from;
    if (!stop.stop_requested()) ::kill(::getpid(), SIGTERM);
  }};
  RunSession(options);
  stopper.request_stop();
  stopper.join();
  ASSERT_GE(counted, kPasses) << "the session made too few passes to judge";
  const auto ms = [](std::chrono::nanoseconds d) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
  };
  EXPECT_GE(window, (static_cast<int64_t>(counted) - 1) * look)
      << counted << " passes started within " << ms(window)
      << " ms; one look per " << ms(look) << " ms allows at most "
      << ms(window) / ms(look) + 1;
  std::filesystem::remove_all(options.output_dir);
}

// With nothing due for a minute -- heartbeat timeouts of hours, the startup
// grace and construction deadline an hour off, the session's duration 60 s
// -- the loop sleeps, and still a child's exit and a stop signal each start
// a pass at once: alpha exits (no restart: max_restarts 0) and the live
// manifest records the crash; then SIGTERM ends the session. The launcher
// runs on a thread that blocks both signals, so their handlers run on
// another thread, as they may in a launcher with its output threads, and
// cannot wake the loop by interrupting its sleep: only the wake they write
// can. Each is awaited for 10 s, which only bounds a hang: a loop they did
// not wake would see either only at its next deadline, a minute after the
// start.
// REQ-OPS-06, REQ-OPS-45
TEST(Supervision, ChildExitAndStopSignalWakeTheSleepingLoop) {
  auto options = BaseOptions("long_sleep");
  options.max_restarts = 0;
  options.duration_s = 60;
  options.heartbeat_periods = 1'000'000;
  options.heartbeat_startup_grace_ms = 3'600'000;
  options.construction_timeout = 1h;
  const auto runs = TempPath("long_sleep_runs");
  const auto trigger = TempPath("long_sleep_exit");
  ::remove(runs.c_str());
  ::remove(trigger.c_str());
  for (const std::string name : {"hardware_node", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + name + ":node";
    options.binary_overrides[target] = WriteStub(
        "long_sleep_" + name, RecordLogArgThen(runs, "exec sleep 60\n"), name);
  }
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "long_sleep_alpha",
      RecordLogArgThen(runs, "while [ ! -e " + trigger +
                                 " ]; do sleep 0.01; done\nexit 3\n"));
  // RunSession rewrites options.output_dir on return; not read concurrently.
  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  std::atomic<bool> returned{false};
  std::thread launcher{[&] {
    sigset_t blocked;
    sigemptyset(&blocked);
    sigaddset(&blocked, SIGCHLD);
    sigaddset(&blocked, SIGTERM);
    ::pthread_sigmask(SIG_BLOCK, &blocked, nullptr);
    RunSession(options);
    returned = true;
  }};
  const auto wait = [](const std::function<bool()>& done) {
    const auto give_up =
        std::chrono::steady_clock::now() + std::chrono::seconds{kHangSeconds};
    while (!done() && std::chrono::steady_clock::now() < give_up)
      std::this_thread::sleep_for(2ms);
    return done();
  };
  bool reaped = false;
  bool stopped = false;
  if (wait([&] { return ReadLines(runs).size() == 3; })) {
    std::ofstream{trigger}.close();
    reaped = wait([&live] {
      const auto history = ManifestRuns(ReadFile(live), "alpha");
      return !history.empty() &&
             history.back().find("\"reason\": \"crash\"") != std::string::npos;
    });
  }
  if (reaped) {
    ::kill(::getpid(), SIGTERM);
    stopped = wait([&] { return returned.load(); });
  }
  launcher.join();
  EXPECT_TRUE(reaped) << "alpha's exit waited for the loop's next deadline";
  EXPECT_TRUE(stopped) << "SIGTERM waited for the loop's next deadline";
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
  ::remove(trigger.c_str());
}

// LOG-9: a launch holds its session directory's session.lock for the whole
// session, which can outlast a /tmp cleaner's age (3 unused days on macOS,
// 10 under systemd-tmpfiles) in the default log root /tmp/talos_logs; with
// the file deleted, a later launch's retention would create one, take it
// and remove the running session. The supervision loop touches it with the
// session lock, once per kLockFileRefresh on the supervision clock: aged by
// the test, it stays aged through a pass half a refresh later and is
// touched by the pass a whole refresh later. Removed under the running
// session (as a cleaner that ran before a refresh would), it is locked again
// at the next refresh, so retention finds it held, and that is reported
// once in talos.log however often it happens. Only the test moves the
// clock; each wait only bounds a hang.
// REQ-OPS-11, REQ-OPS-45
TEST(Supervision, SessionDirectoryLockIsRefreshedAndRetakenWhileHeld) {
  auto options = BaseOptions("lock_refresh");
  options.max_restarts = 0;
  // Hours pass on the supervision clock: nothing else may fall due.
  options.duration_s = 1'000'000;
  options.heartbeat_periods = 1'000'000;
  options.heartbeat_startup_grace_ms = 1'000'000'000;
  options.construction_timeout = 1000h;
  testing::SteppedClock clock;
  options.supervision_clock = clock.clock();
  const auto runs = TempPath("lock_refresh_runs");
  ::remove(runs.c_str());
  for (const std::string name : {"hardware_node", "alpha", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + name + ":node";
    options.binary_overrides[target] = WriteStub(
        "lock_refresh_" + name, TrappedRun(runs, kLoopUntilStopped), name);
  }
  const std::string dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string lock = dir + "/" + std::string{kSessionDirectoryLockFile};
  const std::string talos_log = dir + "/" + std::string{kLauncherOutputFile};
  constexpr time_t kAged = 1'000'000;  // 1970: older than any cleaner's age.
  const auto age = [&lock] {
    const timespec times[2] = {{kAged, 0}, {kAged, 0}};
    return ::utimensat(AT_FDCWD, lock.c_str(), times, 0) == 0;
  };
  const auto aged = [&lock] {
    struct stat status {};
    return ::stat(lock.c_str(), &status) == 0 && status.st_mtime == kAged;
  };
  constexpr std::string_view kRetaken = "was removed while this session held";
  const auto reports = [&talos_log, kRetaken] {
    const std::string text = ReadFile(talos_log);
    std::size_t count = 0;
    for (std::size_t at = text.find(kRetaken); at != std::string::npos;
         at = text.find(kRetaken, at + 1))
      ++count;
    return count;
  };
  testing::SupervisionPasses passes;
  bool started = false;
  bool aged_before_due = false;
  bool touched_when_due = false;
  bool retaken = false;
  bool retaken_again = false;
  std::jthread driver{[&] {
    const auto give_up = std::chrono::steady_clock::now() + 20s;
    const auto wait = [&](const std::function<bool()>& done) {
      while (!done() && std::chrono::steady_clock::now() < give_up)
        std::this_thread::sleep_for(2ms);
      return done();
    };
    // A whole pass has run since now (RunSessionUntil).
    const auto pass = [&] {
      const uint64_t after = passes.Started() + 2;
      return wait([&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after;
      });
    };
    started =
        wait([&] { return ReadLines(runs).size() == 3; }) && pass() && age();
    const std::chrono::nanoseconds half =
        std::chrono::nanoseconds{kLockFileRefresh} / 2;
    if (started) {
      clock.Advance(half);
      aged_before_due = pass() && aged();
      clock.Advance(half);
      touched_when_due = pass() && !aged();
      // Removed twice, each time refreshed into a lock retention sees.
      for (bool* outcome : {&retaken, &retaken_again}) {
        ::unlink(lock.c_str());
        clock.Advance(kLockFileRefresh);
        *outcome = pass() && SessionDirectoryLock::Held(dir, false);
      }
      wait([&] { return reports() > 0; });
      pass();
    }
    ::kill(::getpid(), SIGTERM);
  }};
  RunSession(options);
  driver.join();
  ASSERT_TRUE(started) << "not every member started";
  EXPECT_TRUE(aged_before_due) << "touched before its refresh was due";
  EXPECT_TRUE(touched_when_due) << lock << " was not touched when due";
  EXPECT_TRUE(retaken) << "a removed " << lock << " was not locked again";
  EXPECT_TRUE(retaken_again) << "removed again, it was not locked again";
  EXPECT_EQ(reports(), 1u) << ReadFile(talos_log);
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// LOG-10 / REQ-OPS-11, REQ-OPS-30: lock-loss and retake events during
// supervision persist immediately into the live manifest and survive into the
// final manifest, and per-member `critical` and `studio_bridge` flags reflect
// each member's role (including non-critical subsystems and the launcher-added
// Studio bridge).
TEST(Supervision, ManifestRecordsLockLossRetakeCriticalAndStudioBridge) {
  auto options = BaseOptions("lock_manifest");
  MakeAlphaNonCritical(options);
  options.max_restarts = 0;
  options.duration_s = 1'000'000;
  options.heartbeat_periods = 1'000'000;
  options.heartbeat_startup_grace_ms = 1'000'000'000;
  options.construction_timeout = 1000h;
  options.start_studio_bridge = true;
  testing::SteppedClock clock;
  options.supervision_clock = clock.clock();
  std::atomic<bool> block_dir_lock{false};
  options.lock_session_directory = [&block_dir_lock](int fd, int op) {
    if (block_dir_lock.load()) {
      errno = EWOULDBLOCK;
      return -1;
    }
    return ::flock(fd, op);
  };
  const auto runs = TempPath("lock_manifest_runs");
  ::remove(runs.c_str());
  for (const std::string name : {"hardware_node", "alpha", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + name + ":node";
    options.binary_overrides[target] = WriteStub(
        "lock_manifest_" + name, TrappedRun(runs, kLoopUntilStopped), name);
  }
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("lock_manifest_bridge", kRunUntilStopped, "studio_bridge");

  const std::string dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string dir_lock =
      dir + "/" + std::string{kSessionDirectoryLockFile};
  const std::string session_lock =
      introspect::LockFilePath(ShmObjectName(kSessionLockName));
  const std::string live = dir + "/manifest.json";

  testing::SupervisionPasses passes;
  bool started = false;
  bool initial_false = false;
  bool live_retaken = false;
  bool live_dir_lost = false;
  bool live_session_lost = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const auto pass = [&] {
      const uint64_t after = passes.Started() + 2;
      return WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after;
      });
    };
    started =
        WaitFor(stop, [&] { return ReadLines(runs).size() == 3; }) && pass();
    if (started) {
      const std::string before = ReadFile(live);
      initial_false =
          before.find("\"session_lock_lost\": false") != std::string::npos &&
          before.find("\"directory_lock_lost\": false") != std::string::npos &&
          before.find("\"directory_lock_retaken\": false") != std::string::npos;

      ::unlink(dir_lock.c_str());
      clock.Advance(kLockFileRefresh);
      SupervisionWakeup::Wake();
      live_retaken = WaitFor(stop, [&] {
        return ReadFile(live).find("\"directory_lock_retaken\": true") !=
               std::string::npos;
      });

      block_dir_lock.store(true);
      ::unlink(dir_lock.c_str());
      clock.Advance(kLockFileRefresh);
      SupervisionWakeup::Wake();
      live_dir_lost = WaitFor(stop, [&] {
        return ReadFile(live).find("\"directory_lock_lost\": true") !=
               std::string::npos;
      });

      ::unlink(session_lock.c_str());
      clock.Advance(kLockFileRefresh);
      SupervisionWakeup::Wake();
      live_session_lost = WaitFor(stop, [&] {
        return ReadFile(live).find("\"session_lock_lost\": true") !=
               std::string::npos;
      });
    }
    ::kill(::getpid(), SIGTERM);
  }};
  RunSession(options);
  driver.join();

  ASSERT_TRUE(started) << "not every member started";
  EXPECT_TRUE(initial_false);
  EXPECT_TRUE(live_retaken)
      << "directory_lock_retaken did not reach live manifest";
  EXPECT_TRUE(live_dir_lost)
      << "directory_lock_lost did not reach live manifest";
  EXPECT_TRUE(live_session_lost)
      << "session_lock_lost did not reach live manifest";

  const std::string final_manifest =
      ReadFile(options.output_dir + "/manifest.json");
  EXPECT_NE(final_manifest.find("\"session_lock_lost\": true"),
            std::string::npos)
      << final_manifest;
  EXPECT_NE(final_manifest.find("\"directory_lock_lost\": true"),
            std::string::npos)
      << final_manifest;
  EXPECT_NE(final_manifest.find("\"directory_lock_retaken\": true"),
            std::string::npos)
      << final_manifest;

  const std::string hw = ManifestEntry(final_manifest, "hardware_node");
  EXPECT_NE(hw.find("\"critical\": true"), std::string::npos) << hw;
  EXPECT_NE(hw.find("\"studio_bridge\": false"), std::string::npos) << hw;

  const std::string alpha = ManifestEntry(final_manifest, "alpha");
  EXPECT_NE(alpha.find("\"critical\": false"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"studio_bridge\": false"), std::string::npos) << alpha;

  const std::string beta = ManifestEntry(final_manifest, "beta");
  EXPECT_NE(beta.find("\"critical\": true"), std::string::npos) << beta;
  EXPECT_NE(beta.find("\"studio_bridge\": false"), std::string::npos) << beta;

  const std::string bridge = ManifestEntry(final_manifest, "studio_bridge");
  EXPECT_NE(bridge.find("\"critical\": false"), std::string::npos) << bridge;
  EXPECT_NE(bridge.find("\"studio_bridge\": true"), std::string::npos)
      << bridge;

  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// Options for a session in which nothing falls due on its own for an hour
// (the session lock's refresh): no member is looked at, stalls, misses its
// grace or construction deadline, or ends, and the duration is an hour. A
// test adds the one deadline it checks; the loop must wake for it alone.
LauncherOptions NothingDueOptions(std::string_view tag, const std::string& runs,
                                  std::string_view member_prefix = "") {
  auto options = BaseOptions(tag);
  options.max_restarts = 0;
  options.duration_s = 3600;
  options.heartbeat_periods = 1'000'000;
  options.heartbeat_startup_grace_ms = 3'600'000;
  options.construction_timeout = 1h;
  for (const std::string name : {"hardware_node", "alpha", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + name + ":node";
    options.binary_overrides[target] = WriteStub(
        std::string{tag} + "_" + name,
        std::string{member_prefix} + RecordLogArgThen(runs, "exec sleep 60\n"),
        name);
  }
  return options;
}

// The stop timeout's SIGKILL is a deadline of its own: with members that
// ignore the stop's SIGINT and nothing else due, the loop wakes for it, so
// the session ends one stop timeout after SIGTERM, each run ended
// stop_timeout. A loop that did not sleep until it would sleep for an hour;
// the test then wakes it itself after kStopTimeout + 10 s (the hang bound,
// docs/TESTING.md §6.3) and fails.
// REQ-OPS-06, REQ-OPS-45
TEST(Supervision, StopTimeoutAloneWakesTheLoop) {
  const auto runs = TempPath("stop_wake_runs");
  ::remove(runs.c_str());
  auto options = NothingDueOptions("stop_wake", runs, "trap '' INT HUP\n");
  std::atomic<bool> returned{false};
  bool rescued = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const auto wait = [&](const std::function<bool()>& done,
                          std::chrono::steady_clock::duration bound) {
      const auto give_up = std::chrono::steady_clock::now() + bound;
      while (!stop.stop_requested() && !done() &&
             std::chrono::steady_clock::now() < give_up)
        std::this_thread::sleep_for(2ms);
      return done();
    };
    wait([&] { return ReadLines(runs).size() == 3; },
         std::chrono::seconds{kHangSeconds});
    ::kill(::getpid(), SIGTERM);
    if (!wait([&] { return returned.load(); },
              kStopTimeout + std::chrono::seconds{kHangSeconds})) {
      rescued = true;
      SupervisionWakeup::Wake();
    }
  }};
  RunSession(options);
  returned = true;
  driver.join();
  EXPECT_FALSE(rescued) << "the stop timeout's SIGKILL waited for a wake";
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto history = ManifestRuns(manifest, "alpha");
  ASSERT_FALSE(history.empty()) << manifest;
  EXPECT_NE(history.back().find("\"reason\": \"stop_timeout\""),
            std::string::npos)
      << history.back();
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// The Studio bridge's restart is a deadline of its own: its first run
// exits at once, nothing else is due, and the loop wakes one restart
// backoff later to spawn the second. A loop that did not sleep until it
// would not restart it for an hour; 10 s bounds the wait (docs/TESTING.md
// §6.3).
// REQ-OPS-30, REQ-OPS-45
TEST(Supervision, StudioBridgeRestartAloneWakesTheLoop) {
  const auto runs = TempPath("studio_wake_runs");
  const auto attempts = TempPath("studio_wake_attempts");
  ::remove(runs.c_str());
  ::remove(attempts.c_str());
  auto options = NothingDueOptions("studio_wake", runs);
  // One restart: its second run's end leaves it, and nothing else is due.
  options.max_restarts = 1;
  options.start_studio_bridge = true;
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("studio_wake_bridge", "echo x >> " + attempts + "\nexit 1\n",
                "studio_bridge");
  bool retried = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const auto give_up =
        std::chrono::steady_clock::now() + std::chrono::seconds{kHangSeconds};
    while (!stop.stop_requested() && ReadLines(attempts).size() < 2 &&
           std::chrono::steady_clock::now() < give_up)
      std::this_thread::sleep_for(2ms);
    retried = ReadLines(attempts).size() >= 2;
    ::kill(::getpid(), SIGTERM);
  }};
  RunSession(options);
  driver.join();
  EXPECT_TRUE(retried) << "the bridge's restart waited for an unrelated wake";
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
  ::remove(attempts.c_str());
}

// Pure unit test of every wake-deadline term in NextSupervisionWake: deleting
// any single term (watchdog, lock refresh, bridge stop timeout, shutdown
// timeout, session duration, sim schedule poll, bridge protocol poll, member
// liveness look, pending member restart, Studio bridge next check) fails here.
// REQ-OPS-45
TEST(Supervision, NextSupervisionWakeSelectsEarliestActiveTerm) {
  constexpr SupervisionTime kT0 = SupervisionTime{} + 1h;
  const SupervisionTime kFar = kT0 + 10h;

  // Nothing due -> SupervisionTime::max().
  EXPECT_EQ(NextSupervisionWake(SupervisionWakeInputs{.now = kT0}),
            SupervisionTime::max());

  // Baseline in kNone with every term far in the future except the one tested.
  const auto baseline = [&] {
    SupervisionWakeInputs inputs;
    inputs.now = kT0;
    inputs.watchdog = std::chrono::hours{12};
    inputs.next_lock_refresh = kFar;
    inputs.session_deadline = kFar;
    inputs.studio_next_check = kFar;
    return inputs;
  };

  // Watchdog term alone.
  {
    auto inputs = baseline();
    inputs.watchdog = 100ms;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + 100ms);
  }

  // Lock refresh term alone (e.g. 1 h kLockFileRefresh).
  {
    auto inputs = baseline();
    inputs.next_lock_refresh = kT0 + kLockFileRefresh;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + kLockFileRefresh);
  }

  // Session duration deadline term alone (kNone only).
  {
    auto inputs = baseline();
    inputs.session_deadline = kT0 + 5s;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + 5s);

    inputs.stop_phase = SupervisionStopPhase::kBridge;
    inputs.bridge_killed = true;
    EXPECT_EQ(NextSupervisionWake(inputs), kFar);

    inputs.stop_phase = SupervisionStopPhase::kAllSignaled;
    inputs.shutdown_deadline = kT0;
    EXPECT_EQ(NextSupervisionWake(inputs), kFar);
  }

  // Sim schedule start poll term (kNone only).
  {
    auto inputs = baseline();
    inputs.sim_schedule_polling = true;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + kSimScheduleStartPoll);

    inputs.stop_phase = SupervisionStopPhase::kAllSignaled;
    inputs.shutdown_deadline = kT0;
    EXPECT_EQ(NextSupervisionWake(inputs), kFar);
  }

  // Bridge protocol poll term (kNone only).
  {
    auto inputs = baseline();
    inputs.bridge_protocol_polling = true;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + kBridgeProtocolPoll);

    inputs.stop_phase = SupervisionStopPhase::kBridge;
    inputs.bridge_killed = true;
    EXPECT_EQ(NextSupervisionWake(inputs), kFar);
  }

  // Running members' liveness NextLook terms (kNone only).
  {
    const std::array looks{kT0 + 400ms, kT0 + 125ms, kT0 + 300ms};
    auto inputs = baseline();
    inputs.member_next_looks = looks;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + 125ms);

    inputs.stop_phase = SupervisionStopPhase::kAllSignaled;
    inputs.shutdown_deadline = kT0;
    EXPECT_EQ(NextSupervisionWake(inputs), kFar);
  }

  // Pending member restarts (active in every phase).
  {
    const std::array restarts{kT0 + 200ms, kT0 + 50ms};
    auto inputs = baseline();
    inputs.pending_restarts = restarts;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + 50ms);

    inputs.stop_phase = SupervisionStopPhase::kBridge;
    inputs.bridge_killed = true;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + 50ms);
  }

  // Studio bridge next check (settle interval, liveness, or restart).
  {
    auto inputs = baseline();
    inputs.studio_next_check = kT0 + kStudioBridgeSettleInterval;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + kStudioBridgeSettleInterval);
  }

  // Two-phase bridge stop deadline (kBridge, polling for kFlagQuiesced at
  // kBridgeQuiescePoll capped at bridge_stop_deadline, only when
  // !bridge_killed and now < bridge_stop_deadline).
  {
    auto inputs = baseline();
    inputs.stop_phase = SupervisionStopPhase::kBridge;
    inputs.bridge_stop_deadline = kT0 + kBridgeStopTimeout;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + kBridgeQuiescePoll);

    inputs.bridge_stop_deadline = kT0 + 2ms;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + 2ms);

    inputs.bridge_killed = true;
    EXPECT_EQ(NextSupervisionWake(inputs), kFar);

    inputs.bridge_killed = false;
    inputs.bridge_stop_deadline = kT0;
    EXPECT_EQ(NextSupervisionWake(inputs), kFar);
  }

  // Final stop timeout deadline (kAllSignaled, only when
  // now < shutdown_deadline).
  {
    auto inputs = baseline();
    inputs.stop_phase = SupervisionStopPhase::kAllSignaled;
    inputs.shutdown_deadline = kT0 + kStopTimeout;
    EXPECT_EQ(NextSupervisionWake(inputs), kT0 + kStopTimeout);

    inputs.shutdown_deadline = kT0;
    EXPECT_EQ(NextSupervisionWake(inputs), kFar);
  }
}

// The service watchdog is a wake deadline of its own: with nothing else due
// for an hour (NothingDueOptions) and WATCHDOG_USEC = 200 ms (100 ms wake
// interval), the sleeping loop wakes on its own to send repeated WATCHDOG=1
// pings. Deleting the watchdog wake term leaves the loop asleep for an hour
// after its first pass.
// REQ-OPS-07, REQ-OPS-45
TEST(Supervision, WatchdogAloneWakesTheLoop) {
  const std::string path = "/tmp/tw_wake_" + std::to_string(::getpid());
  ::unlink(path.c_str());
  const int sock = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  ASSERT_GE(sock, 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  ASSERT_EQ(::bind(sock, reinterpret_cast<const sockaddr*>(&address),
                   static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                          path.size() + 1)),
            0);
  const auto runs = TempPath("watchdog_wake_runs");
  ::remove(runs.c_str());
  auto options = NothingDueOptions("watchdog_wake", runs);
  int pings = 0;
  std::jthread driver{[&](const std::stop_token& stop) {
    // Pass 0 sends READY=1 and WATCHDOG=1; receiving 3 WATCHDOG=1 pings
    // requires at least two loop wakes driven by the watchdog deadline alone.
    for (int datagrams = 0;
         !stop.stop_requested() && pings < 3 && datagrams < 10; ++datagrams) {
      pollfd ready{sock, POLLIN, 0};
      if (::poll(&ready, 1, kHangSeconds * 1000) <= 0) break;
      char buffer[64];
      const ssize_t size = ::recv(sock, buffer, sizeof(buffer), 0);
      if (size > 0 && std::string_view(buffer, static_cast<std::size_t>(
                                                   size)) == "WATCHDOG=1") {
        ++pings;
      }
    }
    ::kill(::getpid(), SIGTERM);
  }};
  ::setenv("NOTIFY_SOCKET", path.c_str(), 1);
  ::setenv("WATCHDOG_USEC", "200000", 1);
  const int status = RunSession(options);
  ::unsetenv("NOTIFY_SOCKET");
  ::unsetenv("WATCHDOG_USEC");
  driver.join();
  EXPECT_EQ(status, 0);
  EXPECT_GE(pings, 3) << "WATCHDOG=1 waited for an unrelated wake";
  ::close(sock);
  ::unlink(path.c_str());
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// The session duration (--duration-s) is a wake deadline of its own: with
// nothing else due for an hour (NothingDueOptions) and duration_s = 1, the
// sleeping loop wakes on its own at 1 s and ends the session cleanly.
// REQ-OPS-45
TEST(Supervision, SessionDurationAloneWakesTheLoop) {
  const auto runs = TempPath("duration_wake_runs");
  ::remove(runs.c_str());
  auto options = NothingDueOptions("duration_wake", runs);
  options.duration_s = 1;
  std::atomic<bool> returned{false};
  bool rescued = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    if (!WaitFor(stop, [&] { return returned.load(); })) {
      rescued = !returned.load();
      ::kill(::getpid(), SIGTERM);
    }
  }};
  const int status = RunSession(options);
  returned = true;
  driver.join();
  EXPECT_FALSE(rescued) << "--duration-s waited for an unrelated wake";
  EXPECT_EQ(status, 0);
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// A regular member's pending restart backoff is a wake deadline of its own:
// with nothing else due for an hour (NothingDueOptions), alpha crashes on its
// first run (critical immediate restart, 0 ms) and its second run (100 ms
// backoff, sleeping in the loop), and must start its third run without any
// peer look or synthetic wake.
// REQ-OPS-04, REQ-OPS-45
TEST(Supervision, MemberRestartAloneWakesTheLoop) {
  const auto runs = TempPath("member_restart_wake_runs");
  const auto alpha_runs = TempPath("member_restart_wake_alpha");
  ::remove(runs.c_str());
  ::remove(alpha_runs.c_str());
  auto options = NothingDueOptions("member_restart_wake", runs);
  options.max_restarts = 2;
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("member_restart_wake_alpha",
                "echo x >> '" + alpha_runs +
                    "'\n"
                    "if [ \"$(wc -l < '" +
                    alpha_runs +
                    "')\" -le 2 ]; then exit 1; fi\n"
                    "exec sleep 60\n",
                "alpha");
  bool restarted = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    restarted =
        WaitFor(stop, [&] { return ReadLines(alpha_runs).size() >= 3; });
    ::kill(::getpid(), SIGTERM);
  }};
  RunSession(options);
  driver.join();
  EXPECT_TRUE(restarted)
      << "alpha's backed-off restart waited for an unrelated wake";
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
  ::remove(alpha_runs.c_str());
}

// The Studio bridge's settle interval (kStudioBridgeSettleInterval) is a wake
// deadline of its own: with nothing else due for an hour (NothingDueOptions),
// the sleeping loop wakes on its own after 1 s and announces the settled
// bridge in talos.log without any peer look or synthetic wake.
// REQ-OPS-30, REQ-OPS-45
TEST(Supervision, StudioBridgeSettleAloneWakesTheLoop) {
  const auto runs = TempPath("studio_settle_wake_runs");
  ::remove(runs.c_str());
  auto options = NothingDueOptions("studio_settle_wake", runs);
  options.start_studio_bridge = true;
  options.studio_http_port = 8080;
  options.binary_overrides["//studio/bridge:studio_bridge"] = WriteStub(
      "studio_settle_wake_bridge", "exec sleep 60\n", "studio_bridge");
  const std::string talos_log = options.output_dir + "/" +
                                std::to_string(options.session_id) + "/" +
                                std::string{kLauncherOutputFile};
  bool settled = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    settled = WaitFor(stop, [&] {
      return ReadFile(talos_log).find("Started studio_bridge (pid ") !=
             std::string::npos;
    });
    ::kill(::getpid(), SIGTERM);
  }};
  RunSession(options);
  driver.join();
  EXPECT_TRUE(settled) << "studio_bridge settle waited for an unrelated wake\n"
                       << ReadFile(talos_log);
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// The hourly lock refresh (next_lock_refresh) is a wake deadline of its own:
// with every other deadline hours further out, advancing the clock once to
// 300 ms before kLockFileRefresh runs one pass that leaves session.lock aged
// and puts the loop to sleep; with no further synthetic wake, the sleeping
// loop must wake on its own 300 ms later and touch session.lock.
// REQ-OPS-11, REQ-OPS-45
TEST(Supervision, LockRefreshAloneWakesTheLoop) {
  const auto runs = TempPath("lock_refresh_wake_runs");
  ::remove(runs.c_str());
  auto options = NothingDueOptions("lock_refresh_wake", runs);
  // Hours past kLockFileRefresh so only next_lock_refresh falls due.
  options.duration_s = 1'000'000;
  options.heartbeat_periods = 10'000'000;
  options.heartbeat_startup_grace_ms = 1'000'000'000;
  options.construction_timeout = 1000h;
  testing::SteppedClock clock;
  options.supervision_clock = clock.clock();
  const std::string dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string lock = dir + "/" + std::string{kSessionDirectoryLockFile};
  constexpr time_t kAged = 1'000'000;
  const auto age = [&lock] {
    const timespec times[2] = {{kAged, 0}, {kAged, 0}};
    return ::utimensat(AT_FDCWD, lock.c_str(), times, 0) == 0;
  };
  const auto aged = [&lock] {
    struct stat status {};
    return ::stat(lock.c_str(), &status) == 0 && status.st_mtime == kAged;
  };
  testing::SupervisionPasses passes;
  bool started = false;
  bool aged_before_due = false;
  bool touched_on_own_wake = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const auto pass = [&] {
      const uint64_t after = passes.Started() + 2;
      return WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after;
      });
    };
    started = WaitFor(stop, [&] { return ReadLines(runs).size() == 3; }) &&
              pass() && age();
    if (started) {
      const uint64_t before_advance = passes.Started();
      // One advance + wake lands 300 ms before next_lock_refresh; no further
      // wake is sent, so only next_lock_refresh can wake the sleeping loop.
      clock.Advance(kLockFileRefresh - 300ms);
      if (WaitFor(stop, [&] { return passes.Started() > before_advance; })) {
        aged_before_due = aged();
        touched_on_own_wake = WaitFor(stop, [&] { return !aged(); });
      }
    }
    ::kill(::getpid(), SIGTERM);
  }};
  RunSession(options);
  driver.join();
  ASSERT_TRUE(started) << "not every member started";
  EXPECT_TRUE(aged_before_due) << "lock was touched before next_lock_refresh";
  EXPECT_TRUE(touched_on_own_wake)
      << "next_lock_refresh waited for an unrelated wake";
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// A Studio bridge stub's body: once its stop trap is set, appends its pid
// to `record` (one line per run), then runs `then`.
std::string BridgeRun(const std::string& record, std::string_view then) {
  return std::string{kStopTrap} + "echo $$ >> " + record + "\n" +
         std::string{then};
}

// Studio bridge sessions for the tests below: NothingDueOptions (no member
// ends, stalls or misses a deadline on its own) with the bridge started, a
// restart budget of 3 and a supervision clock the test holds from before
// the launch, so every restart decision reads a time the test chose.
LauncherOptions BridgeOptions(std::string_view tag, const std::string& runs,
                              testing::SteppedClock& clock) {
  auto options = NothingDueOptions(tag, runs);
  options.max_restarts = 3;
  options.start_studio_bridge = true;
  options.supervision_clock = clock.clock();
  return options;
}

// A Studio bridge that crashes mid-session (SIGKILL, once it has been up a
// settle interval) is started again on the restart backoff, and no node is
// disturbed: each keeps its one run and its pid, with restart_count 0, and
// the operator's stop ends the session with 0. The supervision clock is held
// until the crash has been reaped, then runs on, so the backoff passes.
// REQ-OPS-30
TEST(Supervision, CrashedStudioBridgeRestartsAndNoNodeIsDisturbed) {
  const auto runs = TempPath("bridge_crash_runs");
  const auto bridge_runs = TempPath("bridge_crash_bridge_runs");
  const auto crash = TempPath("bridge_crash_now");
  for (const auto* path : {&runs, &bridge_runs, &crash})
    ::remove(path->c_str());
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  auto options = BridgeOptions("bridge_crash", runs, clock);
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("bridge_crash_bridge",
                BridgeRun(bridge_runs, "if [ \"$(wc -l < '" + bridge_runs +
                                           "')\" -eq 1 ]; then\n"
                                           "  while [ ! -e '" +
                                           crash +
                                           "' ]; do sleep 0.01; done\n"
                                           "  kill -KILL $$\n"
                                           "fi\n" +
                                           std::string{kLoopUntilStopped}),
                "studio_bridge");
  // RunSession rewrites options.output_dir on return; not read concurrently.
  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  constexpr std::array kNodes{"hardware_node", "alpha", "beta"};
  testing::SupervisionPasses passes;
  std::string before;  // The live manifest once every member ran.
  bool crashed = false;
  bool restarted = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    const auto every_member_ran = [&] {
      if (ReadLines(runs).size() != 3 || ReadLines(bridge_runs).size() != 1)
        return false;
      before = ReadFile(live);
      return std::ranges::all_of(kNodes, [&](const char* name) {
        return EntryInteger(ManifestEntry(before, name), "pid") > 0;
      });
    };
    if (WaitFor(stop, every_member_ran)) {
      // Up one settle interval: a running bridge, not one still starting.
      clock.HoldAt(start + kStudioBridgeSettleInterval);
      const uint64_t settled = passes.Started() + 2;
      if (WaitFor(stop, [&] {
            SupervisionWakeup::Wake();
            return passes.Started() >= settled;
          })) {
        std::ofstream{crash}.close();
        crashed = WaitFor(stop, [&] {
          const auto history = ManifestRuns(ReadFile(live), "studio_bridge");
          return !history.empty() &&
                 history.front().find("\"reason\": \"crash\"") !=
                     std::string::npos;
        });
      }
    }
    if (crashed) {
      clock.Release();
      restarted =
          WaitFor(stop, [&] { return ReadLines(bridge_runs).size() == 2; });
    }
    ::kill(::getpid(), SIGTERM);
  }};
  const int status = RunSession(options);
  driver.join();
  ASSERT_TRUE(crashed) << "the bridge's crash was not recorded";
  EXPECT_TRUE(restarted) << "the crashed bridge was not started again";
  EXPECT_EQ(status, 0);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string bridge = ManifestEntry(manifest, "studio_bridge");
  EXPECT_EQ(EntryInteger(bridge, "restart_count"), 1) << bridge;
  EXPECT_EQ(ManifestRuns(manifest, "studio_bridge").size(), 2u) << manifest;
  EXPECT_EQ(ReadLines(runs).size(), 3u) << "a node was started again";
  for (const char* name : kNodes) {
    SCOPED_TRACE(name);
    const std::string entry = ManifestEntry(manifest, name);
    EXPECT_EQ(EntryInteger(entry, "restart_count"), 0) << entry;
    EXPECT_EQ(EntryInteger(entry, "pid"),
              EntryInteger(ManifestEntry(before, name), "pid"))
        << entry;
    EXPECT_EQ(ManifestRuns(manifest, name).size(), 1u) << manifest;
  }
  std::filesystem::remove_all(options.output_dir);
  for (const auto* path : {&runs, &bridge_runs, &crash})
    ::remove(path->c_str());
}

// Ephemeral `--studio-http-port 0`: the bridge's `listening <port>` line is
// captured from its output pipe (even split across writes and preceded by
// other lines), recorded on the studio_bridge entry in manifest.json as
// `http_port`, announced in `Started studio_bridge ... on http://localhost:P`
// (never port 0), and passed as argv[2] on restart so the restart rebinds the
// same port.
// REQ-OPS-30, REQ-OPS-08, REQ-OPS-34
TEST(Supervision,
     StudioBridgeEphemeralPortRecordedInManifestAndReusedOnRestart) {
  const auto runs = TempPath("bridge_port_runs");
  const auto bridge_runs = TempPath("bridge_port_bridge_runs");
  const auto bridge_ports = TempPath("bridge_port_argv");
  const auto crash = TempPath("bridge_port_crash");
  for (const auto* path : {&runs, &bridge_runs, &bridge_ports, &crash})
    ::remove(path->c_str());
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  auto options = BridgeOptions("bridge_port", runs, clock);
  options.studio_http_port = 0;
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("bridge_port_bridge",
                BridgeRun(bridge_runs, "echo \"$2\" >> '" + bridge_ports +
                                           "'\n"
                                           "if [ \"$(wc -l < '" +
                                           bridge_runs +
                                           "')\" -eq 1 ]; then\n"
                                           "  echo 'bridge starting up'\n"
                                           "  printf 'list'\n"
                                           "  printf 'ening 43210\\n'\n"
                                           "  while [ ! -e '" +
                                           crash +
                                           "' ]; do sleep 0.01; done\n"
                                           "  kill -KILL $$\n"
                                           "fi\n"
                                           "printf 'listening %s\\n' \"$2\"\n" +
                                           std::string{kLoopUntilStopped}),
                "studio_bridge");
  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string live = session_dir + "/manifest.json";
  const std::string talos_log =
      session_dir + "/" + std::string{kLauncherOutputFile};
  bool port_recorded = false;
  bool settled = false;
  bool crashed = false;
  bool restarted = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    port_recorded = WaitFor(stop, [&] {
      if (ReadLines(runs).size() != 3 || ReadLines(bridge_runs).size() != 1)
        return false;
      const std::string bridge = ManifestEntry(ReadFile(live), "studio_bridge");
      return EntryInteger(bridge, "http_port") == 43210;
    });
    if (port_recorded) {
      clock.HoldAt(start + kStudioBridgeSettleInterval);
      SupervisionWakeup::Wake();
      settled = WaitFor(stop, [&] {
        return ReadFile(talos_log).find("Started studio_bridge (pid ") !=
               std::string::npos;
      });
    }
    if (settled) {
      std::ofstream{crash}.close();
      crashed = WaitFor(stop, [&] {
        const auto history = ManifestRuns(ReadFile(live), "studio_bridge");
        return !history.empty() &&
               history.front().find("\"reason\": \"crash\"") !=
                   std::string::npos;
      });
    }
    if (crashed) {
      clock.Release();
      restarted =
          WaitFor(stop, [&] { return ReadLines(bridge_ports).size() == 2; });
    }
    ::kill(::getpid(), SIGTERM);
  }};
  const int status = RunSession(options);
  driver.join();
  ASSERT_TRUE(port_recorded) << "manifest.json did not record http_port 43210";
  ASSERT_TRUE(settled) << "studio_bridge did not settle";
  ASSERT_TRUE(crashed) << "studio_bridge crash was not recorded";
  ASSERT_TRUE(restarted) << "studio_bridge did not restart";
  EXPECT_EQ(status, 0);
  const auto ports = ReadLines(bridge_ports);
  ASSERT_EQ(ports.size(), 2u);
  EXPECT_EQ(ports[0], "0");
  EXPECT_EQ(ports[1], "43210");
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string bridge = ManifestEntry(manifest, "studio_bridge");
  EXPECT_EQ(EntryInteger(bridge, "http_port"), 43210) << bridge;
  EXPECT_EQ(EntryInteger(ManifestEntry(manifest, "alpha"), "http_port"), -1);
  const std::string log_text =
      ReadFile(options.output_dir + "/" + std::string{kLauncherOutputFile});
  EXPECT_NE(log_text.find("on http://localhost:43210\n"), std::string::npos)
      << log_text;
  EXPECT_EQ(log_text.find("http://localhost:0"), std::string::npos) << log_text;
  std::filesystem::remove_all(options.output_dir);
  for (const auto* path : {&runs, &bridge_runs, &bridge_ports, &crash})
    ::remove(path->c_str());
}

// A Studio bridge that ends at once on every run, even with exit 0 (a server
// that ends before the stop has failed), is restarted max_restarts times
// (3) on the non-critical member's backoff, 50, 100 and 200 ms, then left
// for the rest of the session: recorder_failed, nothing pending, and no
// retry even a restart window later, where a quarantined node would have
// one. Each gap is read from the manifest's retry_at_monotonic_ns on a
// supervision clock held at every reap, so it is exact. No node is touched,
// and the operator's stop ends the session with 0, as without Studio.
// REQ-OPS-30
TEST(Supervision, FailingStudioBridgeRestartsBoundedTimesThenIsLeft) {
  const auto runs = TempPath("bridge_bound_runs");
  const auto bridge_runs = TempPath("bridge_bound_bridge_runs");
  for (const auto* path : {&runs, &bridge_runs}) ::remove(path->c_str());
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  auto options = BridgeOptions("bridge_bound", runs, clock);
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("bridge_bound_bridge", BridgeRun(bridge_runs, "exit 0\n"),
                "studio_bridge");
  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  testing::SupervisionPasses passes;
  std::vector<SupervisionTime> retries;  // Each restart's scheduled start.
  bool left = false;
  bool looked_later = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    const auto bridge_runs_ended = [&](std::size_t count, int64_t& retry_at) {
      const std::string manifest = ReadFile(live);
      retry_at = EntryInteger(ManifestEntry(manifest, "studio_bridge"),
                              "retry_at_monotonic_ns");
      const auto history = ManifestRuns(manifest, "studio_bridge");
      return history.size() == count &&
             history.back().find("\"reason\": \"running\"") ==
                 std::string::npos;
    };
    for (std::size_t run = 1; run <= 3; ++run) {
      int64_t retry_at = 0;
      if (!WaitFor(stop, [&] {
            return bridge_runs_ended(run, retry_at) && retry_at > 0;
          }))
        break;
      retries.emplace_back(std::chrono::nanoseconds{retry_at});
      clock.HoldAt(retries.back());
    }
    if (retries.size() == 3) {
      left = WaitFor(stop, [&] {
        int64_t retry_at = 0;
        return bridge_runs_ended(4, retry_at) && retry_at == 0 &&
               ManifestEntry(ReadFile(live), "studio_bridge")
                       .find("\"recorder_failed\": true") != std::string::npos;
      });
    }
    if (left) {
      clock.HoldAt(retries.back() +
                   std::chrono::milliseconds{options.restart_window_ms} +
                   kRestartMaxBackoff);
      const uint64_t later = passes.Started() + 2;
      looked_later = WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= later;
      });
    }
    ::kill(::getpid(), SIGTERM);
  }};
  const int status = RunSession(options);
  driver.join();
  ASSERT_EQ(retries.size(), 3u)
      << "restart " << retries.size() + 1 << " was never scheduled";
  // The non-critical backoff: kRestartInitialBackoff, doubling.
  const std::array<std::chrono::nanoseconds, 3> gaps{
      retries[0] - start, retries[1] - retries[0], retries[2] - retries[1]};
  EXPECT_EQ(gaps[0], 50ms);
  EXPECT_EQ(gaps[1], 100ms);
  EXPECT_EQ(gaps[2], 200ms);
  ASSERT_TRUE(left) << "the fourth run's end did not leave the bridge";
  EXPECT_TRUE(looked_later);
  EXPECT_EQ(ReadLines(bridge_runs).size(), 4u)
      << "a bridge that was left was started again";
  EXPECT_EQ(status, 0);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string bridge = ManifestEntry(manifest, "studio_bridge");
  EXPECT_EQ(EntryInteger(bridge, "restart_count"), 3) << bridge;
  EXPECT_EQ(EntryInteger(bridge, "retry_at_monotonic_ns"), 0) << bridge;
  EXPECT_NE(bridge.find("\"recorder_failed\": true"), std::string::npos)
      << bridge;
  const std::string talos_log = ReadFile(options.output_dir + "/talos.log");
  EXPECT_NE(talos_log.find("will not be restarted again"), std::string::npos)
      << talos_log;
  EXPECT_EQ(ReadLines(runs).size(), 3u) << "a node was started again";
  for (const char* name : {"hardware_node", "alpha", "beta"}) {
    SCOPED_TRACE(name);
    EXPECT_EQ(EntryInteger(ManifestEntry(manifest, name), "restart_count"), 0);
    EXPECT_EQ(ManifestRuns(manifest, name).size(), 1u) << manifest;
  }
  std::filesystem::remove_all(options.output_dir);
  for (const auto* path : {&runs, &bridge_runs}) ::remove(path->c_str());
}

// REQ-OPS-30: `--max-restarts 0` disables Studio bridge restarts while peers
// keep running: 1 run, 0 restarts, `recorder_failed: true`, give-up line in
// talos.log, and session exit status 0.
TEST(Supervision, StudioBridgeWithZeroMaxRestartsGivesUpWithoutRestarting) {
  const auto runs = TempPath("bridge_zero_restarts_runs");
  const auto bridge_runs = TempPath("bridge_zero_restarts_bridge_runs");
  for (const auto* path : {&runs, &bridge_runs}) ::remove(path->c_str());
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  auto options = BridgeOptions("bridge_zero_restarts", runs, clock);
  options.max_restarts = 0;
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("bridge_zero_restarts_bridge",
                BridgeRun(bridge_runs, "exit 1\n"), "studio_bridge");
  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  testing::SupervisionPasses passes;
  bool left = false;
  bool looked_later = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    left = WaitFor(stop, [&] {
      if (ReadLines(runs).size() != 3) return false;
      const std::string manifest = ReadFile(live);
      const std::string entry = ManifestEntry(manifest, "studio_bridge");
      const auto history = ManifestRuns(manifest, "studio_bridge");
      return history.size() == 1 &&
             history.back().find("\"reason\": \"running\"") ==
                 std::string::npos &&
             EntryInteger(entry, "retry_at_monotonic_ns") == 0 &&
             entry.find("\"recorder_failed\": true") != std::string::npos;
    });
    if (left) {
      clock.HoldAt(start +
                   std::chrono::milliseconds{options.restart_window_ms} +
                   kRestartMaxBackoff);
      const uint64_t later = passes.Started() + 2;
      looked_later = WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= later;
      });
    }
    ::kill(::getpid(), SIGTERM);
  }};
  const int status = RunSession(options);
  driver.join();

  ASSERT_TRUE(left) << "bridge with max_restarts=0 was not left";
  EXPECT_TRUE(looked_later);
  EXPECT_EQ(status, 0);
  EXPECT_EQ(ReadLines(bridge_runs).size(), 1u);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string bridge = ManifestEntry(manifest, "studio_bridge");
  EXPECT_EQ(EntryInteger(bridge, "restart_count"), 0) << bridge;
  EXPECT_EQ(EntryInteger(bridge, "retry_at_monotonic_ns"), 0) << bridge;
  EXPECT_EQ(ManifestRuns(manifest, "studio_bridge").size(), 1u) << manifest;
  EXPECT_NE(bridge.find("\"recorder_failed\": true"), std::string::npos)
      << bridge;
  const std::string talos_log = ReadFile(options.output_dir + "/talos.log");
  EXPECT_NE(talos_log.find("will not be restarted again"), std::string::npos)
      << talos_log;
  EXPECT_EQ(ReadLines(runs).size(), 3u) << "a peer node was disturbed";
  for (const char* name : {"hardware_node", "alpha", "beta"}) {
    SCOPED_TRACE(name);
    EXPECT_EQ(EntryInteger(ManifestEntry(manifest, name), "restart_count"), 0);
    EXPECT_EQ(ManifestRuns(manifest, name).size(), 1u) << manifest;
  }
  std::filesystem::remove_all(options.output_dir);
  for (const auto* path : {&runs, &bridge_runs}) ::remove(path->c_str());
}

// REQ-OPS-30: a Studio bridge that answers --describe and then fails to spawn
// (here its --describe handler replaces the script with an executable script
// whose interpreter does not exist, so execv fails with ENOENT) is retried on
// the non-critical backoff (50, 100, 200 ms) up to max_restarts times, then
// left with recorder_failed: true without disturbing any peer or changing the
// session's exit status.
TEST(Supervision,
     StudioBridgeSpawnFailureAfterDescribeRestartsOnNonCriticalBackoff) {
  const auto runs = TempPath("bridge_spawn_fail_runs");
  ::remove(runs.c_str());
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  auto options = BridgeOptions("bridge_spawn_fail", runs, clock);
  introspect::Description desc;
  desc.name = "studio_bridge";
  desc.target = "//studio/bridge:studio_bridge";
  const std::string bridge_bin = TempPath("bridge_spawn_fail_bridge") + ".sh";
  std::ostringstream script;
  script << "#!/bin/sh\n"
         << "for arg in \"$@\"; do\n"
         << "  if [ \"$arg\" = \"--describe\" ]; then\n"
         << "    cat <<'TALOS_DESCRIBE'\n"
         << introspect::DescribeToJson(desc) << "TALOS_DESCRIBE\n"
         << "    printf '#!/nonexistent/talos_studio_interp\\n' > \"$0.tmp\"\n"
         << "    chmod +x \"$0.tmp\"\n"
         << "    mv \"$0.tmp\" \"$0\"\n"
         << "    exit 0\n"
         << "  fi\n"
         << "done\n"
         << "exit 0\n";
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      testing::WriteWarmStub(bridge_bin, script.str());

  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  std::vector<SupervisionTime> retries;
  bool left = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    const auto bridge_failed = [&](std::size_t count, int64_t& retry_at) {
      const std::string manifest = ReadFile(live);
      retry_at = EntryInteger(ManifestEntry(manifest, "studio_bridge"),
                              "retry_at_monotonic_ns");
      const auto history = ManifestRuns(manifest, "studio_bridge");
      return history.size() == count &&
             history.back().find("\"reason\": \"spawn_failed\"") !=
                 std::string::npos;
    };
    for (std::size_t attempt = 1; attempt <= 3; ++attempt) {
      int64_t retry_at = 0;
      if (!WaitFor(stop, [&] {
            return bridge_failed(attempt, retry_at) && retry_at > 0;
          })) {
        break;
      }
      retries.emplace_back(std::chrono::nanoseconds{retry_at});
      clock.HoldAt(retries.back());
      SupervisionWakeup::Wake();
    }
    if (retries.size() == 3) {
      left = WaitFor(stop, [&] {
        int64_t retry_at = 0;
        return bridge_failed(4, retry_at) && retry_at == 0 &&
               ManifestEntry(ReadFile(live), "studio_bridge")
                       .find("\"recorder_failed\": true") != std::string::npos;
      });
    }
    WaitFor(stop, [&] { return ReadLines(runs).size() == 3; });
    ::kill(::getpid(), SIGTERM);
  }};

  const int status = RunSession(options);
  driver.join();
  ASSERT_EQ(retries.size(), 3u)
      << "spawn-failed restart " << retries.size() + 1 << " was not scheduled";
  const std::array<std::chrono::nanoseconds, 3> gaps{
      retries[0] - start, retries[1] - retries[0], retries[2] - retries[1]};
  EXPECT_EQ(gaps[0], 50ms);
  EXPECT_EQ(gaps[1], 100ms);
  EXPECT_EQ(gaps[2], 200ms);
  ASSERT_TRUE(left) << "studio_bridge was not left after exhausting retries";
  EXPECT_EQ(status, 0);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string bridge = ManifestEntry(manifest, "studio_bridge");
  EXPECT_EQ(EntryInteger(bridge, "restart_count"), 3) << bridge;
  EXPECT_EQ(EntryInteger(bridge, "retry_at_monotonic_ns"), 0) << bridge;
  EXPECT_NE(bridge.find("\"recorder_failed\": true"), std::string::npos)
      << bridge;
  const auto history = ManifestRuns(manifest, "studio_bridge");
  ASSERT_EQ(history.size(), 4u) << manifest;
  for (const auto& run : history) {
    EXPECT_NE(run.find("{\"pid\": -1,"), std::string::npos) << run;
    EXPECT_NE(run.find("\"reason\": \"spawn_failed\""), std::string::npos)
        << run;
  }
  for (const char* name : {"hardware_node", "alpha", "beta"}) {
    SCOPED_TRACE(name);
    EXPECT_EQ(EntryInteger(ManifestEntry(manifest, name), "restart_count"), 0);
    EXPECT_EQ(ManifestRuns(manifest, name).size(), 1u) << manifest;
  }
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
  ::remove(bridge_bin.c_str());
}

// A stop that arrives while the bridge waits out its restart backoff cancels
// the restart: no bridge is spawned once the stop has begun, and the session
// ends as soon as its members have, waiting neither for the backoff nor for
// the stop timeout. The supervision clock is held at the bridge's exit until
// the stop is under way (alpha ignores the stop's SIGINT and holds
// the session open until the test releases it), then moved past the
// restart's due time but short of the stop timeout, so a pass that still
// started the restart would start a bridge during the stop.
// REQ-OPS-30
TEST(Supervision, StopDuringStudioBridgeBackoffStartsNoBridge) {
  const auto runs = TempPath("bridge_stop_runs");
  const auto bridge_runs = TempPath("bridge_stop_bridge_runs");
  const auto release = TempPath("bridge_stop_release");
  for (const auto* path : {&runs, &bridge_runs, &release})
    ::remove(path->c_str());
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  auto options = BridgeOptions("bridge_stop", runs, clock);
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "bridge_stop_alpha",
      "trap '' INT\n" +
          RecordLogArgThen(runs, "while [ ! -e '" + release +
                                     "' ]; do sleep 0.01; done\nexit 0\n"),
      "alpha");
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("bridge_stop_bridge", BridgeRun(bridge_runs, "exit 1\n"),
                "studio_bridge");
  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  testing::SupervisionPasses passes;
  std::atomic<bool> returned{false};
  bool pending = false;
  bool stopping = false;
  bool rescued = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release_clock{clock};
    const auto stopped = [&](const std::string& manifest, const char* name) {
      const auto history = ManifestRuns(manifest, name);
      return !history.empty() &&
             history.back().find("\"reason\": \"stop\"") != std::string::npos;
    };
    pending = WaitFor(stop, [&] {
      return ReadLines(runs).size() == 3 &&
             EntryInteger(ManifestEntry(ReadFile(live), "studio_bridge"),
                          "retry_at_monotonic_ns") > 0;
    });
    ::kill(::getpid(), SIGTERM);
    if (pending) {
      stopping = WaitFor(stop, [&] {
        const std::string manifest = ReadFile(live);
        return stopped(manifest, "hardware_node") && stopped(manifest, "beta");
      });
    }
    if (stopping) {
      // Past the restart's due time, short of the stop timeout.
      clock.HoldAt(start + kRestartMaxBackoff);
      const uint64_t later = passes.Started() + 2;
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= later;
      });
    }
    std::ofstream{release}.close();
    if (!WaitFor(stop, [&] { return returned.load(); })) rescued = true;
  }};
  const int status = RunSession(options);
  returned = true;
  driver.join();
  ASSERT_TRUE(pending) << "the bridge's restart was never scheduled";
  ASSERT_TRUE(stopping) << "the stop did not end hardware_node and beta";
  EXPECT_FALSE(rescued);
  EXPECT_EQ(ReadLines(bridge_runs).size(), 1u)
      << "a bridge was started after the stop began";
  EXPECT_EQ(status, 0);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string bridge = ManifestEntry(manifest, "studio_bridge");
  EXPECT_EQ(EntryInteger(bridge, "retry_at_monotonic_ns"), 0) << bridge;
  EXPECT_EQ(EntryInteger(bridge, "restart_count"), 0) << bridge;
  const auto alpha = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(alpha.size(), 1u) << manifest;
  EXPECT_NE(alpha.back().find("\"reason\": \"stop\""), std::string::npos)
      << alpha.back();
  std::filesystem::remove_all(options.output_dir);
  for (const auto* path : {&runs, &bridge_runs, &release})
    ::remove(path->c_str());
}

std::string RunfileBinary(const char* env_var) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  if (!runfiles) throw std::runtime_error(error);
  const char* path = std::getenv(env_var);
  if (path == nullptr)
    throw std::runtime_error(std::string{"missing "} + env_var);
  return runfiles->Rlocation(path);
}

// Configures `hardware_node`, `alpha`, and `beta` as registering, heartbeating
// stubs (`delivery_loss_node`) so real liveness timeouts apply to studio_bridge
// without killing the other roster members.
void UseHealthyRegisteringNodes(LauncherOptions& options,
                                std::string_view tag) {
  const std::string binary = RunfileBinary("TALOS_DELIVERY_LOSS_NODE");
  for (const std::string name : {"hardware_node", "alpha", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + name + ":node";
    options.binary_overrides[target] = WriteStub(
        std::string{tag} + "_" + name,
        "exec " + binary + " " + name + " " + target + " 0,0,0,0,run \"$@\"\n",
        name);
  }
}

// Advances `clock` from `now` to `target` in increments strictly below the
// 500 ms `kMinHeartbeatTimeout` of `hardware_node`, `alpha`, and `beta`,
// refreshing each healthy node's observed `dispatch_count` before every step
// so an extra supervision pass never mistakes a clock jump for a node stall.
bool AdvanceClockWithHealthyNodes(const std::stop_token& stop,
                                  testing::SteppedClock& clock,
                                  testing::SupervisionPasses& passes,
                                  uint64_t session, SupervisionTime& now,
                                  SupervisionTime target) {
  const auto node_dispatches = [session] {
    std::array<uint64_t, 3> counts{0, 0, 0};
    const auto reader = introspect::RegistryReader::open();
    if (!reader) return counts;
    constexpr std::array<std::string_view, 3> kNames{"hardware_node", "alpha",
                                                     "beta"};
    for (const auto& row : reader->nodes()) {
      if (row.session_id != session) continue;
      for (std::size_t idx = 0; idx < kNames.size(); ++idx) {
        if (row.name == kNames[idx]) counts[idx] = row.dispatch_count;
      }
    }
    return counts;
  };
  const auto observe_pass = [&] {
    const uint64_t next = passes.Started() + 2;
    return WaitFor(stop, [&] {
      SupervisionWakeup::Wake();
      return passes.Started() >= next;
    });
  };
  const auto refresh_healthy_nodes = [&] {
    const auto before_step = node_dispatches();
    if (!WaitFor(stop, [&] {
          const auto current = node_dispatches();
          return current[0] > before_step[0] && current[1] > before_step[1] &&
                 current[2] > before_step[2];
        }))
      return false;
    return observe_pass();
  };
  while (now < target) {
    if (!refresh_healthy_nodes()) return false;
    const auto remaining = target - now;
    now += remaining < 250ms ? remaining : 250ms;
    clock.HoldAt(now);
    SupervisionWakeup::Wake();
  }
  return true;
}

// A Studio bridge that registers its row, advances dispatch_count, and then
// hangs without exiting is detected by the launcher's liveness watch, killed
// as `stalled`, and restarted on the bridge's schedule without disturbing any
// other node; holding the restarted run before registration while the clock
// stays at `retry_at` must not kill it across supervision passes.
// REQ-OPS-30
TEST(Supervision, HungStudioBridgeIsKilledAndRestartedWithoutDisturbingNodes) {
  const auto bridge_runs = TempPath("bridge_hang_bridge_runs");
  const auto hang = TempPath("bridge_hang_now");
  const auto hung_ack = hang + ".hung";
  const auto hold_run2 = TempPath("bridge_hang_hold_run2");
  const auto hold_ready = hold_run2 + ".ready";
  for (const auto* path :
       {&bridge_runs, &hang, &hung_ack, &hold_run2, &hold_ready}) {
    ::remove(path->c_str());
  }
  struct HoldCleanup {
    std::string hold;
    std::string ready;
    ~HoldCleanup() {
      ::remove(hold.c_str());
      ::remove(ready.c_str());
    }
  } hold_cleanup{hold_run2, hold_ready};
  std::ofstream{hold_run2}.close();
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  auto options = BaseOptions("bridge_hang");
  options.duration_s = 3600;
  options.max_restarts = 2;
  options.heartbeat_periods = 1;
  options.heartbeat_startup_grace_ms = 500;
  options.construction_timeout = 1h;
  options.start_studio_bridge = true;
  options.supervision_clock = clock.clock();
  UseHealthyRegisteringNodes(options, "bridge_hang");
  const std::string binary = RunfileBinary("TALOS_HANGING_BRIDGE");
  options.binary_overrides["//studio/bridge:studio_bridge"] = WriteStub(
      "bridge_hang_bridge",
      "TALOS_HOLD_RUN2_BEFORE_REGISTER='" + hold_run2 + "' exec " + binary +
          " '" + bridge_runs + "' '" + hang + "' once \"$@\"\n",
      "studio_bridge");
  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  const uint64_t session = options.session_id;
  constexpr std::array kNodes{"hardware_node", "alpha", "beta"};
  testing::SupervisionPasses passes;
  std::string before;
  bool held_before_register = false;
  std::vector<std::string> held_history;
  bool restarted = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    SupervisionTime now = start;
    const auto observe_pass = [&] {
      const uint64_t next = passes.Started() + 2;
      return WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= next;
      });
    };
    const auto ready = [&] {
      const auto lines = ReadLines(bridge_runs);
      if (lines.size() != 1) return false;
      if (!RegisteredPid("studio_bridge", session, ParsePidLine(lines[0])) ||
          !Registered("hardware_node", session) ||
          !Registered("alpha", session) || !Registered("beta", session))
        return false;
      before = ReadFile(live);
      return std::ranges::all_of(kNodes, [&](const char* name) {
        return EntryInteger(ManifestEntry(before, name), "pid") > 0;
      });
    };
    if (WaitFor(stop, ready)) {
      // Request hang and wait for run 1 to acknowledge it has frozen its
      // dispatch_count while the clock is still held at `start`, then let one
      // supervision pass observe run 1's final dispatch_count so
      // `progress_changed_at_ == start` deterministically.
      std::ofstream{hang}.close();
      if (WaitFor(stop, [&] { return std::filesystem::exists(hung_ack); }) &&
          observe_pass() &&
          AdvanceClockWithHealthyNodes(stop, clock, passes, session, now,
                                       start + kStudioBridgePeriod)) {
        // Hold the clock at `start + kStudioBridgePeriod` (before the 50 ms
        // restart backoff) until run 1 is reaped and ManifestWriter flushes
        // `stalled` + `retry_at_monotonic_ns`: never keep stepping the clock
        // while waiting on disk sync or child startup.
        int64_t retry_at = 0;
        const bool stalled = WaitFor(stop, [&] {
          const std::string manifest = ReadFile(live);
          retry_at = EntryInteger(ManifestEntry(manifest, "studio_bridge"),
                                  "retry_at_monotonic_ns");
          const auto history = ManifestRuns(manifest, "studio_bridge");
          return history.size() == 1 &&
                 history[0].find("\"reason\": \"stalled\"") !=
                     std::string::npos &&
                 retry_at > 0;
        });
        // Advance exactly to `retry_at` and hold there while run 2 is parked
        // before claiming its registry row: two supervision passes must leave
        // the unregistered restart running rather than killing it. Then
        // release `hold_run2` and wait until run 2 registers its own row
        // (matched by run 2's PID, distinct from run 1's SIGKILLed row) and
        // one supervision pass observes it.
        if (stalled &&
            AdvanceClockWithHealthyNodes(
                stop, clock, passes, session, now,
                SupervisionTime{std::chrono::nanoseconds{retry_at}})) {
          held_before_register =
              WaitFor(stop,
                      [&] {
                        return std::filesystem::exists(hold_ready) &&
                               ManifestRuns(ReadFile(live), "studio_bridge")
                                       .size() >= 2;
                      }) &&
              observe_pass() && observe_pass();
          if (held_before_register) {
            held_history = ManifestRuns(ReadFile(live), "studio_bridge");
          }
          ::remove(hold_run2.c_str());
          if (held_before_register) {
            restarted = WaitFor(stop,
                                [&] {
                                  const auto lines = ReadLines(bridge_runs);
                                  if (lines.size() != 2) return false;
                                  return RegisteredPid("studio_bridge", session,
                                                       ParsePidLine(lines[1]));
                                }) &&
                        observe_pass();
          }
        }
      }
    }
    ::kill(::getpid(), SIGTERM);
  }};
  const int status = RunSession(options);
  driver.join();
  EXPECT_TRUE(held_before_register)
      << "restarted studio_bridge did not park before registering";
  EXPECT_EQ(held_history.size(), 2u);
  if (held_history.size() >= 2u) {
    EXPECT_NE(held_history[1].find("\"reason\": \"running\""),
              std::string::npos)
        << held_history[1];
  }
  EXPECT_TRUE(restarted) << "hung studio_bridge was not killed and restarted";
  EXPECT_EQ(status, 0);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string bridge = ManifestEntry(manifest, "studio_bridge");
  EXPECT_EQ(EntryInteger(bridge, "restart_count"), 1) << bridge;
  EXPECT_NE(bridge.find("\"loop_tier\": \"observer\""), std::string::npos)
      << bridge;
  EXPECT_NE(bridge.find("\"realtime\": \"not_requested\""), std::string::npos)
      << bridge;
  const auto history = ManifestRuns(manifest, "studio_bridge");
  ASSERT_EQ(history.size(), 2u) << manifest;
  EXPECT_NE(history[0].find("\"reason\": \"stalled\""), std::string::npos)
      << history[0];
  EXPECT_NE(history[0].find("\"exit_signal\": 9"), std::string::npos)
      << history[0];
  for (const char* name : kNodes) {
    SCOPED_TRACE(name);
    const std::string entry = ManifestEntry(manifest, name);
    EXPECT_EQ(EntryInteger(entry, "restart_count"), 0) << entry;
    EXPECT_EQ(EntryInteger(entry, "pid"),
              EntryInteger(ManifestEntry(before, name), "pid"))
        << entry;
    EXPECT_EQ(ManifestRuns(manifest, name).size(), 1u) << manifest;
  }
  std::filesystem::remove_all(options.output_dir);
  for (const auto* path : {&bridge_runs, &hang, &hung_ack})
    ::remove(path->c_str());
}

// A Studio bridge that hangs on every run is killed as `stalled` and restarted
// up to max_restarts times on the non-critical backoff, then left for the rest
// of the session (`recorder_failed`, no retry pending) without disturbing any
// other node or affecting the session's exit status.
// REQ-OPS-30
TEST(Supervision, RepeatedlyHangingStudioBridgeExhaustsRestartsAndIsLeft) {
  const auto bridge_runs = TempPath("bridge_hang_bound_bridge_runs");
  const auto hang = TempPath("bridge_hang_bound_now");
  for (const auto* path : {&bridge_runs, &hang}) ::remove(path->c_str());
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  auto options = BaseOptions("bridge_hang_bound");
  options.duration_s = 3600;
  options.max_restarts = 2;
  options.heartbeat_periods = 1;
  options.heartbeat_startup_grace_ms = 500;
  options.construction_timeout = 1h;
  options.start_studio_bridge = true;
  options.supervision_clock = clock.clock();
  UseHealthyRegisteringNodes(options, "bridge_hang_bound");
  const std::string binary = RunfileBinary("TALOS_HANGING_BRIDGE");
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("bridge_hang_bound_bridge",
                "exec " + binary + " '" + bridge_runs + "' '" + hang +
                    "' always \"$@\"\n",
                "studio_bridge");
  const std::string live = options.output_dir + "/" +
                           std::to_string(options.session_id) +
                           "/manifest.json";
  const uint64_t session = options.session_id;
  testing::SupervisionPasses passes;
  bool left = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    SupervisionTime now = start;
    const auto run_stalled = [&](std::size_t count, int64_t& retry_at) {
      const std::string manifest = ReadFile(live);
      retry_at = EntryInteger(ManifestEntry(manifest, "studio_bridge"),
                              "retry_at_monotonic_ns");
      const auto history = ManifestRuns(manifest, "studio_bridge");
      return history.size() == count &&
             history.back().find("\"reason\": \"stalled\"") !=
                 std::string::npos;
    };
    const auto observe_pass = [&] {
      const uint64_t target = passes.Started() + 2;
      return WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= target;
      });
    };
    const bool nodes_up = WaitFor(stop, [&] {
      return Registered("hardware_node", session) &&
             Registered("alpha", session) && Registered("beta", session);
    });
    for (std::size_t run = 1; nodes_up && run <= 3; ++run) {
      if (!WaitFor(stop, [&] {
            const auto lines = ReadLines(bridge_runs);
            if (lines.size() != run) return false;
            return RegisteredPid("studio_bridge", session,
                                 ParsePidLine(lines[run - 1]));
          })) {
        break;
      }
      // Let one pass observe the registered bridge row at `now`.
      if (!observe_pass()) break;
      if (!AdvanceClockWithHealthyNodes(stop, clock, passes, session, now,
                                        now + 1000ms))
        break;
      int64_t retry_at = 0;
      if (run <= 2) {
        if (!WaitFor(stop, [&] {
              return run_stalled(run, retry_at) && retry_at > 0;
            }))
          break;
        if (!AdvanceClockWithHealthyNodes(
                stop, clock, passes, session, now,
                SupervisionTime{std::chrono::nanoseconds{retry_at}}))
          break;
      } else {
        left = WaitFor(stop, [&] {
          return run_stalled(run, retry_at) && retry_at == 0 &&
                 ManifestEntry(ReadFile(live), "studio_bridge")
                         .find("\"recorder_failed\": true") !=
                     std::string::npos;
        });
      }
    }
    ::kill(::getpid(), SIGTERM);
  }};
  const int status = RunSession(options);
  driver.join();
  ASSERT_TRUE(left)
      << "repeatedly hanging bridge was not left after exhaustion";
  EXPECT_EQ(status, 0);
  EXPECT_EQ(ReadLines(bridge_runs).size(), 3u);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string bridge = ManifestEntry(manifest, "studio_bridge");
  EXPECT_EQ(EntryInteger(bridge, "restart_count"), 2) << bridge;
  EXPECT_EQ(EntryInteger(bridge, "retry_at_monotonic_ns"), 0) << bridge;
  EXPECT_NE(bridge.find("\"recorder_failed\": true"), std::string::npos)
      << bridge;
  const auto history = ManifestRuns(manifest, "studio_bridge");
  ASSERT_EQ(history.size(), 3u) << manifest;
  for (const auto& entry : history) {
    EXPECT_NE(entry.find("\"reason\": \"stalled\""), std::string::npos)
        << entry;
  }
  for (const char* name : {"hardware_node", "alpha", "beta"}) {
    SCOPED_TRACE(name);
    EXPECT_EQ(EntryInteger(ManifestEntry(manifest, name), "restart_count"), 0);
    EXPECT_EQ(ManifestRuns(manifest, name).size(), 1u) << manifest;
  }
  std::filesystem::remove_all(options.output_dir);
  for (const auto* path : {&bridge_runs, &hang}) ::remove(path->c_str());
}

// The Studio bridge's liveness look is a wake deadline of its own (REQ-OPS-45):
// once the bridge has settled and every other roster member has exited cleanly
// (with max_restarts = 0 and duration_s = 3600, so no other member's liveness,
// restart, settle deadline, or session deadline is due, and no external
// SupervisionWakeup::Wake() is ever sent), hanging studio_bridge causes
// StudioBridge::NextCheck alone to wake the supervision loop, kill it as
// `stalled`, and end the session cleanly. Deleting the liveness term in
// StudioBridge::NextCheck leaves the loop asleep for an hour and fails this
// test.
// REQ-OPS-30, REQ-OPS-45
TEST(Supervision, StudioBridgeLivenessAloneWakesTheLoop) {
  const auto bridge_runs = TempPath("bridge_liveness_wake_runs");
  const auto hang = TempPath("bridge_liveness_wake_hang");
  const auto hung_ack = hang + ".hung";
  for (const auto* path : {&bridge_runs, &hang, &hung_ack})
    ::remove(path->c_str());
  auto options = BaseOptions("bridge_liveness_wake");
  options.duration_s = 3600;
  options.max_restarts = 0;
  options.heartbeat_periods = 1;
  options.heartbeat_startup_grace_ms = 2000;
  options.construction_timeout = 1h;
  options.start_studio_bridge = true;
  UseHealthyRegisteringNodes(options, "bridge_liveness_wake");
  const std::string binary = RunfileBinary("TALOS_HANGING_BRIDGE");
  options.binary_overrides["//studio/bridge:studio_bridge"] = WriteStub(
      "bridge_liveness_wake_bridge",
      "exec " + binary + " '" + bridge_runs + "' '" + hang + "' once \"$@\"\n",
      "studio_bridge");
  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string live = session_dir + "/manifest.json";
  const std::string talos_log = session_dir + "/talos.log";
  const uint64_t session = options.session_id;
  constexpr std::array kNodes{"hardware_node", "alpha", "beta"};
  std::atomic<bool> returned{false};
  bool settled = false;
  bool nodes_exited = false;
  bool rescued = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    // Wait until studio_bridge has settled (so Starting() is false and its
    // settle deadline is past) and every other member has registered and
    // recorded its pid.
    std::array<pid_t, 3> pids{0, 0, 0};
    settled = WaitFor(stop, [&] {
      if (ReadLines(bridge_runs).size() != 1 ||
          !Registered("studio_bridge", session) ||
          !Registered("hardware_node", session) ||
          !Registered("alpha", session) || !Registered("beta", session))
        return false;
      if (ReadFile(talos_log).find("Started studio_bridge (pid ") ==
          std::string::npos)
        return false;
      const std::string manifest = ReadFile(live);
      for (std::size_t i = 0; i < kNodes.size(); ++i) {
        const int64_t pid =
            EntryInteger(ManifestEntry(manifest, kNodes[i]), "pid");
        if (pid <= 0) return false;
        pids[i] = static_cast<pid_t>(pid);
      }
      return true;
    });
    if (settled) {
      // Stop every other member cleanly so none of them has a liveness look
      // or restart pending; only the settled studio_bridge remains running.
      for (const pid_t pid : pids) ::kill(pid, SIGTERM);
      nodes_exited = WaitFor(stop, [&] {
        const std::string manifest = ReadFile(live);
        return std::ranges::all_of(kNodes, [&](const char* name) {
          const auto runs = ManifestRuns(manifest, name);
          return !runs.empty() && runs.back().find("\"reason\": \"running\"") ==
                                      std::string::npos;
        });
      });
    }
    if (nodes_exited) {
      // Now hang studio_bridge; nothing except StudioBridge::NextCheck can
      // wake the supervision loop for the next hour.
      std::ofstream{hang}.close();
      if (!WaitFor(stop, [&] { return returned.load(); })) {
        rescued = true;
        ::kill(::getpid(), SIGTERM);
      }
    } else {
      ::kill(::getpid(), SIGTERM);
    }
  }};
  const int status = RunSession(options);
  returned = true;
  driver.join();
  ASSERT_TRUE(settled) << "studio_bridge did not settle";
  ASSERT_TRUE(nodes_exited) << "other roster members did not exit cleanly";
  EXPECT_FALSE(rescued) << "hung studio_bridge waited for an unrelated wake "
                           "(StudioBridge::NextCheck)";
  EXPECT_EQ(status, 0);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto history = ManifestRuns(manifest, "studio_bridge");
  ASSERT_EQ(history.size(), 1u) << manifest;
  EXPECT_NE(history[0].find("\"reason\": \"stalled\""), std::string::npos)
      << history[0];
  EXPECT_NE(history[0].find("\"exit_signal\": 9"), std::string::npos)
      << history[0];
  std::filesystem::remove_all(options.output_dir);
  for (const auto* path : {&bridge_runs, &hang, &hung_ack})
    ::remove(path->c_str());
}

// The launcher passes --schedule-start-file <session>/sim_schedule_start to a
// launcher-managed sim_gateway and holds off writing that file until every
// non-observer member's loop has started (not constructing), ignoring an
// observer (studio_bridge) that has not registered.
// REQ-DEV-05, REQ-DEV-10
TEST(Supervision, SimGatewayScheduleWaitsForEveryNonObserverLoop) {
  const auto gateway_args = TempPath("sim_sched_gw_args");
  ::remove(gateway_args.c_str());
  auto options = BaseOptions("sim_sched",
                             "sim_gateway = \"//example/sim_gateway:node\"\n");
  options.simulation = true;
  options.remote = "127.0.0.1";
  options.start_sim_gateway = true;
  options.start_studio_bridge = true;
  options.max_restarts = 0;
  options.duration_s = kHangSeconds;
  // alpha is a real talos node whose constructor sleeps 1.2 s while
  // heartbeating kFlagConstructing.
  {
    auto config = toml::parse_file(options.config_path);
    config["subsystems"]["alpha"].as_table()->insert_or_assign(
        "construction_ms", 1200);
    std::ofstream{options.config_path} << config;
  }
  const std::string loss_binary = RunfileBinary("TALOS_DELIVERY_LOSS_NODE");
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStub(
      "sim_sched_hardware",
      "exec " + loss_binary +
          " hardware_node //talOS/bridge:hardware_node 0,0,0,0,run \"$@\"\n",
      "hardware_node");
  options.binary_overrides["//example/alpha:node"] =
      RunfileBinary("TALOS_CONSTRUCTION_NODE");
  options.binary_overrides["//example/beta:node"] = WriteStub(
      "sim_sched_beta",
      "exec " + loss_binary + " beta //example/beta:node 0,0,0,0,run \"$@\"\n",
      "beta");
  options.binary_overrides["//example/sim_gateway:node"] = WriteStub(
      "sim_sched_gateway",
      "printf '%s\\n' \"$*\" > '" + gateway_args + "'\nexec " + loss_binary +
          " sim_gateway //example/sim_gateway:node 0,0,0,0,run --log "
          "/dev/null \"$@\"\n",
      "sim_gateway");
  // An observer that never registers must not hold sim_gateway's schedule.
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("sim_sched_studio", "trap 'exit 0' INT TERM\nexec sleep 30\n",
                "studio_bridge");

  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string start_file =
      session_dir + "/" + std::string{kSimScheduleStartFile};
  const uint64_t session = options.session_id;
  bool saw_constructing_without_file = false;
  bool wrote_early = false;
  std::string seen;
  EXPECT_EQ(RunSessionUntil(
                options,
                [&](const std::string&) {
                  const bool file_exists = std::filesystem::exists(start_file);
                  if (!Registered("alpha", session)) {
                    if (Registered("sim_gateway", session) &&
                        Registered("hardware_node", session) &&
                        Registered("beta", session)) {
                      if (file_exists) wrote_early = true;
                      saw_constructing_without_file = true;
                    }
                    return false;
                  }
                  return file_exists;
                },
                seen),
            0);
  ASSERT_FALSE(seen.empty()) << "sim_schedule_start was never written";
  EXPECT_TRUE(saw_constructing_without_file)
      << "never observed peers running while alpha was constructing";
  EXPECT_FALSE(wrote_early)
      << "sim_schedule_start appeared before alpha finished constructing";
  EXPECT_FALSE(Registered("studio_bridge", session));

  const std::string passed = ReadFile(gateway_args);
  EXPECT_NE(passed.find("--schedule-start-file " + start_file),
            std::string::npos)
      << passed;
  std::string stamp = ReadFile(start_file);
  ASSERT_FALSE(stamp.empty());
  if (stamp.ends_with('\n')) stamp.pop_back();
  ASSERT_FALSE(stamp.empty());
  EXPECT_TRUE(std::all_of(stamp.begin(), stamp.end(), [](char c) {
    return c >= '0' && c <= '9';
  })) << stamp;
  EXPECT_GT(std::stoll(stamp), 0);

  std::filesystem::remove_all(options.output_dir);
  ::remove(gateway_args.c_str());
}

// When the caller passes --schedule-start-file through --sim-gateway-arg, the
// launcher forwards it untouched and neither passes nor writes its own
// sim_schedule_start file.
// REQ-DEV-05
TEST(Supervision, CallerScheduleStartFileDisablesLauncherCoordination) {
  const auto gateway_args = TempPath("custom_sched_gw_args");
  const auto custom_file = TempPath("custom_sched_start");
  for (const auto* path : {&gateway_args, &custom_file})
    ::remove(path->c_str());
  auto options = BaseOptions("custom_sched",
                             "sim_gateway = \"//example/sim_gateway:node\"\n");
  options.simulation = true;
  options.remote = "127.0.0.1";
  options.start_sim_gateway = true;
  options.sim_gateway_args = {"--schedule-start-file", custom_file};
  options.max_restarts = 0;
  options.duration_s = kHangSeconds;
  UseHealthyRegisteringNodes(options, "custom_sched");
  const std::string loss_binary = RunfileBinary("TALOS_DELIVERY_LOSS_NODE");
  options.binary_overrides["//example/sim_gateway:node"] = WriteStub(
      "custom_sched_gateway",
      "printf '%s\\n' \"$*\" > '" + gateway_args + "'\nexec " + loss_binary +
          " sim_gateway //example/sim_gateway:node 0,0,0,0,run --log "
          "/dev/null \"$@\"\n",
      "sim_gateway");

  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string launcher_file =
      session_dir + "/" + std::string{kSimScheduleStartFile};
  const uint64_t session = options.session_id;
  std::string seen;
  EXPECT_EQ(RunSessionUntil(
                options,
                [session](const std::string&) {
                  return Registered("sim_gateway", session) &&
                         Registered("hardware_node", session) &&
                         Registered("alpha", session) &&
                         Registered("beta", session);
                },
                seen),
            0);
  ASSERT_FALSE(seen.empty());
  const std::string passed = ReadFile(gateway_args);
  EXPECT_NE(passed.find("--schedule-start-file " + custom_file),
            std::string::npos)
      << passed;
  EXPECT_EQ(passed.find(std::string{kSimScheduleStartFile}), std::string::npos)
      << passed;
  EXPECT_FALSE(std::filesystem::exists(launcher_file));
  EXPECT_FALSE(std::filesystem::exists(custom_file));

  std::filesystem::remove_all(options.output_dir);
  for (const auto* path : {&gateway_args, &custom_file})
    ::remove(path->c_str());
}

// When sim_gateway exits without a pending restart before
// sim_schedule_start is written, supervision abandons schedule-start
// coordination: sim_schedule_start is not written and the loop sleeps until
// its next real deadline instead of waking every 20 ms.
// REQ-DEV-10, REQ-DEV-24
TEST(Supervision, SimGatewayEarlyExitStopsScheduleStartPolling) {
  auto options = BaseOptions("sim_sched_gw_exit",
                             "sim_gateway = \"//example/sim_gateway:node\"\n");
  options.simulation = true;
  options.remote = "127.0.0.1";
  options.start_sim_gateway = true;
  options.max_restarts = 0;
  // Nothing else falls due on its own for an hour, so any 20 ms wakes after
  // sim_gateway exits can come only from schedule-start polling.
  options.duration_s = 3600;
  options.heartbeat_periods = 1'000'000;
  options.heartbeat_startup_grace_ms = 3'600'000;
  options.construction_timeout = 1h;
  for (const std::string_view name : {"hardware_node", "alpha", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + std::string{name} + ":node";
    options.binary_overrides[target] = WriteStub(
        "sim_sched_gw_exit_" + std::string{name}, "exec sleep 60\n", name);
  }
  // sim_gateway exits early before registering; the remaining stubs never
  // register, so roster_ready is never reached.
  options.binary_overrides["//example/sim_gateway:node"] =
      WriteStub("sim_sched_gw_exit_gateway", "exit 1\n", "sim_gateway");

  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string live = session_dir + "/manifest.json";
  const std::string start_file =
      session_dir + "/" + std::string{kSimScheduleStartFile};
  testing::SupervisionPasses passes;
  bool gateway_exited = false;
  uint64_t passes_after_exit = 0;
  std::jthread driver{[&](const std::stop_token& stop) {
    gateway_exited = WaitFor(stop, [&] {
      const auto runs = ManifestRuns(ReadFile(live), "sim_gateway");
      return !runs.empty() &&
             runs.back().find("\"reason\": \"crash\"") != std::string::npos;
    });
    if (gateway_exited) {
      // 300 ms would produce ~15 passes under a 20 ms poll, and at most 1
      // in-flight wake once schedule-start polling stops.
      const uint64_t base = passes.Started();
      WaitFor(stop, [&] { return passes.Started() - base >= 4; }, 300ms);
      passes_after_exit = passes.Started() - base;
    }
    ::kill(::getpid(), SIGTERM);
  }};
  EXPECT_EQ(RunSession(options), 1);
  driver.join();
  ASSERT_TRUE(gateway_exited) << "sim_gateway did not exit as crashed";
  EXPECT_LT(passes_after_exit, 4u)
      << "Supervise kept waking (" << passes_after_exit
      << " passes in 300 ms) after sim_gateway exited";
  EXPECT_FALSE(std::filesystem::exists(start_file));
  std::filesystem::remove_all(options.output_dir);
}

// When a non-critical member exits cleanly or is quarantined before
// registering, sim_schedule_start is still written once the remaining active
// non-observer members have started their loops.
// REQ-DEV-10, REQ-DEV-24
TEST(Supervision, NonCriticalExitOrQuarantineDoesNotBlockSimGatewaySchedule) {
  const std::string loss_binary = RunfileBinary("TALOS_DELIVERY_LOSS_NODE");

  // Case 1: non-critical alpha exits cleanly (exit 0) before registering;
  // restarts are enabled, but non-critical clean exits are not restarted.
  {
    auto options = BaseOptions(
        "sim_sched_nc_clean", "sim_gateway = \"//example/sim_gateway:node\"\n");
    options.simulation = true;
    options.remote = "127.0.0.1";
    options.start_sim_gateway = true;
    options.max_restarts = 5;
    options.duration_s = kHangSeconds;
    MakeAlphaNonCritical(options);
    UseHealthyRegisteringNodes(options, "sim_sched_nc_clean");
    options.binary_overrides["//example/alpha:node"] =
        WriteStub("sim_sched_nc_clean_alpha", "exit 0\n", "alpha");
    options.binary_overrides["//example/sim_gateway:node"] = WriteStub(
        "sim_sched_nc_clean_gateway",
        "exec " + loss_binary +
            " sim_gateway //example/sim_gateway:node 0,0,0,0,run --log "
            "/dev/null \"$@\"\n",
        "sim_gateway");

    const std::string session_dir =
        options.output_dir + "/" + std::to_string(options.session_id);
    const std::string start_file =
        session_dir + "/" + std::string{kSimScheduleStartFile};
    std::string seen;
    EXPECT_EQ(RunSessionUntil(
                  options,
                  [&](const std::string& live) {
                    const auto runs = ManifestRuns(live, "alpha");
                    return !runs.empty() &&
                           runs.back().find("\"reason\": \"exit\"") !=
                               std::string::npos &&
                           std::filesystem::exists(start_file);
                  },
                  seen),
              0);
    ASSERT_FALSE(seen.empty()) << "sim_schedule_start was not written after "
                                  "non-critical clean exit";
    std::string stamp = ReadFile(start_file);
    if (stamp.ends_with('\n')) stamp.pop_back();
    ASSERT_FALSE(stamp.empty());
    EXPECT_GT(std::stoll(stamp), 0);
    const std::string manifest =
        ReadFile(options.output_dir + "/manifest.json");
    EXPECT_EQ(ManifestRuns(manifest, "alpha").size(), 1u) << manifest;
    std::filesystem::remove_all(options.output_dir);
  }

  // Case 2: non-critical alpha crashes before registering until quarantined;
  // sim_schedule_start is written once alpha enters quarantine without waiting
  // out the 60 s restart window.
  {
    auto options = BaseOptions(
        "sim_sched_nc_quar", "sim_gateway = \"//example/sim_gateway:node\"\n");
    options.simulation = true;
    options.remote = "127.0.0.1";
    options.start_sim_gateway = true;
    options.max_restarts = 1;
    options.restart_window_ms = 60000;
    options.duration_s = kHangSeconds;
    MakeAlphaNonCritical(options);
    UseHealthyRegisteringNodes(options, "sim_sched_nc_quar");
    options.binary_overrides["//example/alpha:node"] =
        WriteStub("sim_sched_nc_quar_alpha", "exit 1\n", "alpha");
    options.binary_overrides["//example/sim_gateway:node"] = WriteStub(
        "sim_sched_nc_quar_gateway",
        "exec " + loss_binary +
            " sim_gateway //example/sim_gateway:node 0,0,0,0,run --log "
            "/dev/null \"$@\"\n",
        "sim_gateway");

    const std::string session_dir =
        options.output_dir + "/" + std::to_string(options.session_id);
    const std::string start_file =
        session_dir + "/" + std::string{kSimScheduleStartFile};
    std::string seen;
    EXPECT_EQ(RunSessionUntil(
                  options,
                  [&](const std::string& live) {
                    return ManifestEntry(live, "alpha")
                                   .find("\"quarantined\": true") !=
                               std::string::npos &&
                           std::filesystem::exists(start_file);
                  },
                  seen),
              1);
    ASSERT_FALSE(seen.empty()) << "sim_schedule_start was not written after "
                                  "non-critical quarantine";
    std::string stamp = ReadFile(start_file);
    if (stamp.ends_with('\n')) stamp.pop_back();
    ASSERT_FALSE(stamp.empty());
    EXPECT_GT(std::stoll(stamp), 0);
    const std::string manifest =
        ReadFile(options.output_dir + "/manifest.json");
    EXPECT_EQ(ManifestRuns(manifest, "alpha").size(), 2u) << manifest;
    std::filesystem::remove_all(options.output_dir);
  }
}

// When sim_gateway crashes before registering and waits in a pending restart
// backoff (including past max_restarts; launcher-managed sim_gateway is
// critical and therefore retries on capped backoff rather than quarantining),
// schedule-start coordination stays armed without writing sim_schedule_start
// while the gateway is down, and writes sim_schedule_start cleanly once the
// retried gateway starts its loop.
// REQ-DEV-10, REQ-DEV-24
TEST(Supervision, SimGatewayCrashAndRestartStillStartsSchedule) {
  const auto gw_runs = TempPath("sim_sched_gw_retry_runs");
  ::remove(gw_runs.c_str());
  testing::SteppedClock clock;
  clock.Freeze();
  auto options = BaseOptions("sim_sched_gw_retry",
                             "sim_gateway = \"//example/sim_gateway:node\"\n");
  options.simulation = true;
  options.remote = "127.0.0.1";
  options.start_sim_gateway = true;
  options.max_restarts = 1;
  options.duration_s = 3600;
  options.heartbeat_periods = 1'000'000;
  options.heartbeat_startup_grace_ms = 3'600'000;
  options.construction_timeout = 1h;
  options.supervision_clock = clock.clock();
  UseHealthyRegisteringNodes(options, "sim_sched_gw_retry");
  const std::string loss_binary = RunfileBinary("TALOS_DELIVERY_LOSS_NODE");
  // Runs 1 and 2 crash before registering (run 1 restarts immediately as a
  // critical first crash; run 2 enters a 100 ms pending backoff). Run 3 starts
  // its loop normally.
  options.binary_overrides["//example/sim_gateway:node"] = WriteStub(
      "sim_sched_gw_retry_gateway",
      "echo run >> '" + gw_runs + "'\n" + "if [ \"$(wc -l < '" + gw_runs +
          "')\" -le 2 ]; then exit 1; fi\nexec " + loss_binary +
          " sim_gateway //example/sim_gateway:node 0,0,0,0,run --log "
          "/dev/null \"$@\"\n",
      "sim_gateway");

  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string live = session_dir + "/manifest.json";
  const std::string start_file =
      session_dir + "/" + std::string{kSimScheduleStartFile};
  const uint64_t session = options.session_id;
  testing::SupervisionPasses passes;
  bool saw_pending_backoff = false;
  bool wrote_while_down = false;
  bool started_after_retry = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    int64_t retry_at = 0;
    saw_pending_backoff = WaitFor(stop, [&] {
      if (!Registered("hardware_node", session) ||
          !Registered("alpha", session) || !Registered("beta", session))
        return false;
      const std::string manifest = ReadFile(live);
      const auto runs = ManifestRuns(manifest, "sim_gateway");
      if (runs.size() != 2 ||
          runs.back().find("\"reason\": \"crash\"") == std::string::npos)
        return false;
      retry_at = EntryInteger(ManifestEntry(manifest, "sim_gateway"),
                              "retry_at_monotonic_ns");
      return retry_at > 0;
    });
    if (saw_pending_backoff) {
      // Run two supervision passes while sim_gateway is down in backoff so
      // every healthy peer's loop_started is observed and we confirm the start
      // file is not written before sim_gateway comes back up.
      const uint64_t after_down = passes.Started() + 2;
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after_down;
      });
      wrote_while_down = std::filesystem::exists(start_file);
      // Advance to the pending restart deadline and wake Supervise.
      clock.HoldAt(SupervisionTime{std::chrono::nanoseconds{retry_at}});
      SupervisionWakeup::Wake();
      started_after_retry = WaitFor(stop, [&] {
        return Registered("sim_gateway", session) &&
               std::filesystem::exists(start_file);
      });
    }
    ::kill(::getpid(), SIGTERM);
  }};

  EXPECT_EQ(RunSession(options), 0);
  driver.join();
  ASSERT_TRUE(saw_pending_backoff)
      << "sim_gateway never entered pending restart backoff";
  EXPECT_FALSE(wrote_while_down)
      << "sim_schedule_start was written while sim_gateway was down";
  ASSERT_TRUE(started_after_retry)
      << "sim_schedule_start was not written after sim_gateway retried";
  std::string stamp = ReadFile(start_file);
  if (stamp.ends_with('\n')) stamp.pop_back();
  ASSERT_FALSE(stamp.empty());
  EXPECT_GT(std::stoll(stamp), 0);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const auto runs = ManifestRuns(manifest, "sim_gateway");
  ASSERT_EQ(runs.size(), 3u) << manifest;
  EXPECT_NE(runs[0].find("\"reason\": \"crash\""), std::string::npos)
      << runs[0];
  EXPECT_NE(runs[1].find("\"reason\": \"crash\""), std::string::npos)
      << runs[1];
  EXPECT_NE(runs[2].find("\"reason\": \"stop\""), std::string::npos) << runs[2];
  EXPECT_EQ(
      EntryInteger(ManifestEntry(manifest, "sim_gateway"), "restart_count"), 2);
  std::filesystem::remove_all(options.output_dir);
  ::remove(gw_runs.c_str());
}

// Helper: writes a shell stub whose --describe output is `description`.
std::string WriteDescribedStub(std::string_view tag,
                               const introspect::Description& description,
                               std::string_view body) {
  const std::string path = TempPath(tag) + ".sh";
  std::ostringstream out;
  out << "#!/bin/sh\n"
      << "for arg in \"$@\"; do\n"
      << "  if [ \"$arg\" = \"--describe\" ]; then\n"
      << "    cat <<'TALOS_DESCRIBE'\n"
      << introspect::DescribeToJson(description) << "TALOS_DESCRIBE\n"
      << "    exit 0\n"
      << "  fi\n"
      << "done\n"
      << body;
  return testing::WriteWarmStub(path, out.str());
}

// LOG-16: DescribeGraph counts the launcher as +1 observer reader on /hw/status
// when the bridge declares it, and Supervise opens /hw/status as an observer
// reader using the bridge's declared shape, warning once per bridge run to
// stderr + talos.log when protocol_version_id != protocol::kProtocolVersion
// and re-warning after a bridge restart.
// REQ-OPS-51
TEST(Supervision, BridgeProtocolMismatchWarnsOncePerRunAndRewarnsOnRestart) {
  const auto status_schema = talos::schema::Of<bridge::BridgeStatus>();
  introspect::naming::SourceShape status_sender;
  status_sender.kind = event::SourceKind::SENDER;
  status_sender.name = std::string{bridge::topics::kStatus};
  status_sender.message_bytes = sizeof(bridge::BridgeStatus);
  status_sender.alignment = alignof(bridge::BridgeStatus);
  status_sender.slots = MAX_SLOTS;
  status_sender.schema = status_schema.name;
  status_sender.schema_hash = status_schema.hash;
  status_sender.bfbs = status_schema.bfbs;

  introspect::Description bridge_desc;
  bridge_desc.name = "hardware_node";
  bridge_desc.target = "//talOS/bridge:hardware_node";
  bridge_desc.sources.push_back(status_sender);

  // Part 1: DescribeGraph counts the launcher's observer reader on /hw/status
  // against MAX_READERS (32 other readers + 1 launcher = 33 -> error;
  // 31 other readers + 1 launcher = 32 -> ok).
  {
    const auto has_reader_limit_error =
        [](const std::vector<introspect::naming::Diagnostic>& diags) {
          return std::any_of(diags.begin(), diags.end(), [](const auto& d) {
            return d.severity == introspect::naming::Severity::ERROR &&
                   d.subject == bridge::topics::kStatus &&
                   d.message.find("reader limit") != std::string::npos;
          });
        };
    const std::string bridge_bin =
        WriteDescribedStub("proto_limit_hw", bridge_desc, "exit 0\n");
    for (const std::size_t readers : {MAX_READERS - 1, MAX_READERS}) {
      introspect::Description reader_desc;
      reader_desc.name = "alpha";
      reader_desc.target = "//example/alpha:node";
      introspect::naming::SourceShape reader_shape = status_sender;
      reader_shape.kind = event::SourceKind::FETCHER;
      for (std::size_t i = 0; i < readers; ++i) {
        reader_desc.sources.push_back(reader_shape);
      }
      const std::string alpha_bin =
          WriteDescribedStub("proto_limit_alpha_" + std::to_string(readers),
                             reader_desc, "exit 0\n");
      auto opts = BaseOptions("proto_limit_" + std::to_string(readers));
      const std::vector<NodeSpec> specs{
          {.name = "hardware_node",
           .target = "//talOS/bridge:hardware_node",
           .binary_path = bridge_bin,
           .critical = true},
          {.name = "alpha",
           .target = "//example/alpha:node",
           .binary_path = alpha_bin,
           .critical = true},
      };
      std::vector<NodeDescription> described;
      const auto diags = DescribeGraph(specs, opts, described);
      EXPECT_EQ(has_reader_limit_error(diags), readers == MAX_READERS)
          << "readers=" << readers;
    }
  }

  // Part 2: Live mismatch warns once per bridge run and re-warns after restart.
  const auto hw_runs = TempPath("proto_warn_hw_runs");
  const auto crash_run1 = TempPath("proto_warn_crash1");
  ::remove(hw_runs.c_str());
  ::remove(crash_run1.c_str());

  auto options = BaseOptions("proto_warn");
  options.duration_s = kHangSeconds;
  options.max_restarts = 2;
  // Run 1 records its pid and waits for `crash_run1`, then exits 1 (triggers
  // immediate restart for critical hardware_node). Run 2 records its pid and
  // execs sleep until stopped (no bash WCE loop that could swallow SIGINT).
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteDescribedStub("proto_warn_hw", bridge_desc,
                         "echo $$ >> '" + hw_runs +
                             "'\n"
                             "if [ \"$(wc -l < '" +
                             hw_runs +
                             "')\" -eq 1 ]; then\n"
                             "  while [ ! -e '" +
                             crash_run1 +
                             "' ]; do sleep 0.01; done\n"
                             "  exit 1\n"
                             "fi\n"
                             "exec sleep 86400\n");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("proto_warn_alpha", "exec sleep 86400\n", "alpha");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("proto_warn_beta", "exec sleep 86400\n", "beta");

  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string talos_log = session_dir + "/talos.log";
  const std::uint16_t wrong_version =
      static_cast<std::uint16_t>(talos::protocol::kProtocolVersion - 1);
  const std::string needle = "reports hardware protocol " +
                             std::to_string(wrong_version) + " on /hw/status";
  const auto count_warnings = [&] {
    const std::string text = ReadFile(talos_log);
    std::size_t count = 0;
    for (std::size_t at = text.find(needle); at != std::string::npos;
         at = text.find(needle, at + needle.size())) {
      ++count;
    }
    return count;
  };

  testing::SupervisionPasses passes;
  bool warned_run1 = false;
  std::size_t count_after_extra_publishes = 0;
  bool warned_run2 = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    if (!WaitFor(stop, [&] { return ReadLines(hw_runs).size() == 1; })) {
      ::kill(::getpid(), SIGTERM);
      return;
    }
    RTMSQueue status_pub{bridge::topics::kStatus,
                         sizeof(bridge::BridgeStatus),
                         alignof(bridge::BridgeStatus),
                         status_schema,
                         MAX_SLOTS,
                         RTMSOptions{.publisher = true}};
    bridge::BridgeStatus msg{};
    msg.mutate_protocol_version_id(wrong_version);
    const auto payload = std::as_bytes(std::span{&msg, 1});
    status_pub.write(payload);
    SupervisionWakeup::Wake();
    warned_run1 = WaitFor(stop, [&] { return count_warnings() == 1; });
    if (warned_run1) {
      // Publish additional mismatched statuses during the same bridge run:
      // must not warn a second time.
      status_pub.write(payload);
      status_pub.write(payload);
      const uint64_t target_pass = passes.Started() + 2;
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= target_pass;
      });
      count_after_extra_publishes = count_warnings();

      // Crash run 1 and wait for run 2 to start, then publish again.
      std::ofstream{crash_run1}.close();
      if (WaitFor(stop, [&] { return ReadLines(hw_runs).size() == 2; })) {
        // Wait for Supervise to have spawned run 2 and updated manifest.
        WaitFor(stop, [&] {
          return ManifestRuns(ReadFile(session_dir + "/manifest.json"),
                              "hardware_node")
                     .size() == 2;
        });
        status_pub.write(payload);
        SupervisionWakeup::Wake();
        warned_run2 = WaitFor(stop, [&] { return count_warnings() == 2; });
      }
    }
    ::kill(::getpid(), SIGTERM);
  }};

  EXPECT_EQ(RunSession(options), 0);
  driver.join();
  EXPECT_TRUE(warned_run1) << ReadFile(talos_log);
  EXPECT_EQ(count_after_extra_publishes, 1u) << ReadFile(talos_log);
  EXPECT_TRUE(warned_run2) << ReadFile(talos_log);
  EXPECT_EQ(count_warnings(), 2u) << ReadFile(talos_log);
  const auto pids = ReadLines(hw_runs);
  ASSERT_EQ(pids.size(), 2u);
  const std::string log_text = ReadFile(talos_log);
  EXPECT_NE(log_text.find("hardware_node (pid " + pids[0] + ") " + needle),
            std::string::npos)
      << log_text;
  EXPECT_NE(log_text.find("hardware_node (pid " + pids[1] + ") " + needle),
            std::string::npos)
      << log_text;

  std::filesystem::remove_all(options.output_dir);
  ::remove(hw_runs.c_str());
  ::remove(crash_run1.c_str());
}

// RT-12(b) / REQ-OPS-20: two-phase shutdown stops `hardware_node` first and
// waits for it to exit (up to kBridgeStopTimeout before SIGKILLing a hung
// bridge) before signaling the remaining actuator-owning nodes.
// REQ-OPS-20
TEST(Supervision, ShutdownStopsHardwareNodeBeforeActuatorOwners) {
  // Case 1: hardware_node exits cleanly on kStopSignal; actuator owners
  // (alpha, beta) receive kStopSignal only after hardware_node has exited.
  {
    const auto ready = TempPath("twophase_ready");
    const auto events = TempPath("twophase_events");
    const auto release_hw = TempPath("twophase_release_hw");
    for (const auto* p : {&ready, &events, &release_hw}) ::remove(p->c_str());

    testing::SteppedClock clock;
    clock.Freeze();
    auto options = BaseOptions("twophase_clean");
    options.duration_s = 3600;
    options.max_restarts = 0;
    options.heartbeat_periods = 1'000'000;
    options.heartbeat_startup_grace_ms = 3'600'000;
    options.construction_timeout = 1h;
    options.supervision_clock = clock.clock();
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStub("twophase_clean_hw",
                  "got_int=0\n"
                  "trap 'got_int=1' INT\n"
                  "echo hw >> '" +
                      ready +
                      "'\n"
                      "while [ \"$got_int\" -eq 0 ]; do sleep 0.01; done\n"
                      "echo hw_int >> '" +
                      events +
                      "'\n"
                      "while [ ! -e '" +
                      release_hw +
                      "' ]; do sleep 0.01; done\n"
                      "echo hw_exit >> '" +
                      events +
                      "'\n"
                      "exit 0\n",
                  "hardware_node");
    const auto owner_body = [&](std::string_view name) {
      return "got_int=0\n"
             "trap 'got_int=1' INT\n"
             "echo " +
             std::string{name} + " >> '" + ready +
             "'\n"
             "while [ \"$got_int\" -eq 0 ]; do sleep 0.01; done\n"
             "echo " +
             std::string{name} + "_int >> '" + events +
             "'\n"
             "exit 0\n";
    };
    options.binary_overrides["//example/alpha:node"] =
        WriteStub("twophase_clean_alpha", owner_body("alpha"), "alpha");
    options.binary_overrides["//example/beta:node"] =
        WriteStub("twophase_clean_beta", owner_body("beta"), "beta");

    testing::SupervisionPasses passes;
    std::vector<std::string> events_while_hw_alive;
    std::jthread driver{[&](const std::stop_token& stop) {
      const testing::SteppedClock::ReleaseOnExit release_clock{clock};
      if (!WaitFor(stop, [&] { return ReadLines(ready).size() == 3; })) {
        ::kill(::getpid(), SIGTERM);
        return;
      }
      ::kill(::getpid(), SIGTERM);
      if (WaitFor(stop, [&] {
            const auto lines = ReadLines(events);
            return !lines.empty() && lines[0] == "hw_int";
          })) {
        const uint64_t after = passes.Started() + 2;
        WaitFor(stop, [&] {
          SupervisionWakeup::Wake();
          return passes.Started() >= after;
        });
        events_while_hw_alive = ReadLines(events);
      }
      std::ofstream{release_hw}.close();
      // Keep the stepped clock frozen while hardware_node sees release_hw,
      // writes hw_exit, and exits, and alpha/beta receive kStopSignal; without
      // this wait, ReleaseOnExit unfreezes clock to wall time (which on a slow
      // host can already be >= start + kBridgeStopTimeout) and SIGKILLs
      // hardware_node before it finishes writing hw_exit.
      WaitFor(stop, [&] { return ReadLines(events).size() >= 4; });
    }};

    EXPECT_EQ(RunSession(options), 0);
    driver.join();
    EXPECT_EQ(events_while_hw_alive, (std::vector<std::string>{"hw_int"}))
        << "actuator owners were signaled before hardware_node exited";
    const auto all_events = ReadLines(events);
    ASSERT_EQ(all_events.size(), 4u);
    EXPECT_EQ(all_events[0], "hw_int");
    EXPECT_EQ(all_events[1], "hw_exit");
    std::vector<std::string> owners(all_events.begin() + 2, all_events.end());
    std::sort(owners.begin(), owners.end());
    EXPECT_EQ(owners, (std::vector<std::string>{"alpha_int", "beta_int"}));

    std::filesystem::remove_all(options.output_dir);
    for (const auto* p : {&ready, &events, &release_hw}) ::remove(p->c_str());
  }

  // Case 2: a hung hardware_node ignoring kStopSignal is SIGKILLed at
  // kBridgeStopTimeout (500 ms), after which alpha and beta are signaled and
  // exit cleanly with reason "stop".
  {
    const auto ready = TempPath("twophase_hung_ready");
    ::remove(ready.c_str());
    testing::SteppedClock clock;
    const SupervisionTime start = clock.Freeze();
    auto options = BaseOptions("twophase_hung");
    options.duration_s = 3600;
    options.max_restarts = 0;
    options.heartbeat_periods = 1'000'000;
    options.heartbeat_startup_grace_ms = 3'600'000;
    options.construction_timeout = 1h;
    options.supervision_clock = clock.clock();
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStub("twophase_hung_hw",
                  "trap '' INT\necho hw >> '" + ready + "'\nexec sleep 60\n",
                  "hardware_node");
    options.binary_overrides["//example/alpha:node"] =
        WriteStub("twophase_hung_alpha",
                  "echo alpha >> '" + ready + "'\nexec sleep 60\n", "alpha");
    options.binary_overrides["//example/beta:node"] =
        WriteStub("twophase_hung_beta",
                  "echo beta >> '" + ready + "'\nexec sleep 60\n", "beta");

    const std::string live = options.output_dir + "/" +
                             std::to_string(options.session_id) +
                             "/manifest.json";
    testing::SupervisionPasses passes;
    bool held_before_timeout = false;
    std::jthread driver{[&](const std::stop_token& stop) {
      const testing::SteppedClock::ReleaseOnExit release_clock{clock};
      if (!WaitFor(stop, [&] {
            if (ReadLines(ready).size() != 3) return false;
            const std::string init = ReadFile(live);
            return !ManifestRuns(init, "hardware_node").empty() &&
                   !ManifestRuns(init, "alpha").empty();
          })) {
        ::kill(::getpid(), SIGTERM);
        return;
      }
      // Keep clock frozen at `start` while Supervise() handles SIGTERM so
      // bridge_stop_deadline is deterministically `start + kBridgeStopTimeout`.
      ::kill(::getpid(), SIGTERM);
      const uint64_t after_sigterm = passes.Started() + 2;
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after_sigterm;
      });
      clock.HoldAt(start + kBridgeStopTimeout - 50ms);
      const uint64_t after = passes.Started() + 2;
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after;
      });
      const std::string mid = ReadFile(live);
      const auto mid_hw = ManifestRuns(mid, "hardware_node");
      const auto mid_alpha = ManifestRuns(mid, "alpha");
      held_before_timeout =
          !mid_hw.empty() &&
          mid_hw.back().find("\"running\"") != std::string::npos &&
          !mid_alpha.empty() &&
          mid_alpha.back().find("\"running\"") != std::string::npos;
      clock.HoldAt(start + kBridgeStopTimeout);
      SupervisionWakeup::Wake();
    }};

    EXPECT_EQ(RunSession(options), -1);
    driver.join();
    EXPECT_TRUE(held_before_timeout)
        << "hardware_node or alpha ended before kBridgeStopTimeout";
    const std::string manifest =
        ReadFile(options.output_dir + "/manifest.json");
    const auto hw = ManifestRuns(manifest, "hardware_node");
    ASSERT_EQ(hw.size(), 1u);
    EXPECT_NE(hw.back().find("\"reason\": \"stop_timeout\""), std::string::npos)
        << hw.back();
    for (const char* name : {"alpha", "beta"}) {
      const auto runs = ManifestRuns(manifest, name);
      ASSERT_EQ(runs.size(), 1u);
      EXPECT_NE(runs.back().find("\"reason\": \"stop\""), std::string::npos)
          << name << ": " << runs.back();
    }
    std::filesystem::remove_all(options.output_dir);
    ::remove(ready.c_str());
  }
}

// REQ-OPS-20, REQ-OPS-06: two-phase shutdown decouples bridge quiescence
// (kBridgeStopTimeout, 500 ms) from `.tlog` flush (kStopTimeout, 2 s):
// (a) a quiesced bridge stalling past 500 ms and exiting 0 at 800 ms lets
//     actuator owners stop immediately on kFlagQuiesced without a 500 ms
//     SIGKILL and exits 0;
// (b) a quiesced bridge hanging past 2 s is SIGKILLed with stop_timeout;
// (c) an un-quiesced hung bridge is still SIGKILLed at 500 ms before actuator
//     owners are signaled.
TEST(Supervision, QuiescedBridgeDecouplesActuatorStopFromLogFlushTimeout) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  ASSERT_NE(runfiles, nullptr) << error;
  const char* quiesce_env = std::getenv("TALOS_QUIESCE_BRIDGE");
  ASSERT_NE(quiesce_env, nullptr);
  const std::string quiesce_bin = runfiles->Rlocation(quiesce_env);

  const auto make_owner = [](std::string_view tag, std::string_view name,
                             const std::string& ready,
                             const std::string& events) {
    return WriteStub(tag,
                     "got_int=0\n"
                     "trap 'got_int=1' INT\n"
                     "echo " +
                         std::string{name} + " >> '" + ready +
                         "'\n"
                         "while [ \"$got_int\" -eq 0 ]; do sleep 0.01; done\n"
                         "echo " +
                         std::string{name} + "_int >> '" + events +
                         "'\n"
                         "exit 0\n",
                     name);
  };

  const auto make_bridge =
      [&](std::string_view tag, std::string_view mode, const std::string& ready,
          const std::string& events, const std::string& release) {
        return WriteStub(tag,
                         "exec '" + quiesce_bin + "' " + std::string{mode} +
                             " '" + ready + "' '" + events + "' '" + release +
                             "' \"$@\"\n",
                         "hardware_node");
      };

  // Case (a): quiesced bridge stalls past 500 ms and exits 0 at 800 ms.
  {
    const auto ready = TempPath("quiesce_clean_ready");
    const auto events = TempPath("quiesce_clean_events");
    const auto release_hw = TempPath("quiesce_clean_release");
    for (const auto* p : {&ready, &events, &release_hw}) ::remove(p->c_str());

    testing::SteppedClock clock;
    const SupervisionTime start = clock.Freeze();
    auto options = BaseOptions("quiesce_clean");
    options.duration_s = 3600;
    options.max_restarts = 0;
    options.heartbeat_periods = 1'000'000;
    options.heartbeat_startup_grace_ms = 3'600'000;
    options.construction_timeout = 1h;
    options.supervision_clock = clock.clock();
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        make_bridge("quiesce_clean_hw", "quiesce", ready, events, release_hw);
    options.binary_overrides["//example/alpha:node"] =
        make_owner("quiesce_clean_alpha", "alpha", ready, events);
    options.binary_overrides["//example/beta:node"] =
        make_owner("quiesce_clean_beta", "beta", ready, events);

    const uint64_t session = options.session_id;
    const std::string live =
        options.output_dir + "/" + std::to_string(session) + "/manifest.json";
    testing::SupervisionPasses passes;
    bool owners_signaled_on_quiesced = false;
    bool hw_alive_at_800ms = false;
    std::atomic<bool> session_done{false};
    std::jthread driver{[&](const std::stop_token& stop) {
      const testing::SteppedClock::ReleaseOnExit release_clock{clock};
      if (!WaitFor(stop, [&] {
            const auto init_hw = ManifestRuns(ReadFile(live), "hardware_node");
            return ReadLines(ready).size() == 3 &&
                   Registered("hardware_node", session) && !init_hw.empty() &&
                   init_hw.back().find("\"running\"") != std::string::npos;
          })) {
        std::ofstream{release_hw}.close();
        ::kill(::getpid(), SIGTERM);
        return;
      }
      ::kill(::getpid(), SIGTERM);
      // Actuator owners must be signaled immediately upon kFlagQuiesced while
      // hardware_node is still alive (clock frozen at start, release_hw
      // absent).
      owners_signaled_on_quiesced = WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return ReadLines(events).size() >= 4;
      });
      // Advance past kBridgeStopTimeout (500 ms) to 800 ms (< kStopTimeout 2
      // s): the quiesced bridge must NOT be SIGKILLed at 500 ms.
      clock.HoldAt(start + 800ms);
      const uint64_t after_800 = passes.Started() + 2;
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after_800;
      });
      const auto mid_hw = ManifestRuns(ReadFile(live), "hardware_node");
      hw_alive_at_800ms =
          !mid_hw.empty() &&
          mid_hw.back().find("\"running\"") != std::string::npos;
      // Release hardware_node at 800 ms so it exits 0 cleanly; keep clock
      // held until RunSession finishes so ReleaseOnExit cannot jump past
      // kStopTimeout while children are still being reaped.
      std::ofstream{release_hw}.close();
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return session_done.load(std::memory_order_acquire);
      });
    }};

    EXPECT_EQ(RunSession(options), 0);
    session_done.store(true, std::memory_order_release);
    driver.request_stop();
    driver.join();
    EXPECT_TRUE(owners_signaled_on_quiesced)
        << "actuator owners were not signaled immediately on kFlagQuiesced";
    EXPECT_TRUE(hw_alive_at_800ms)
        << "quiesced hardware_node was killed before 800 ms";
    const auto all_events = ReadLines(events);
    ASSERT_EQ(all_events.size(), 5u);
    EXPECT_EQ(all_events[0], "hw_int");
    // quiesce_bridge marks kFlagQuiesced before writing "hw_quiesced", so
    // signaled owners can log their SIGINT lines concurrently with it.
    std::vector<std::string> middle(all_events.begin() + 1,
                                    all_events.begin() + 4);
    std::sort(middle.begin(), middle.end());
    EXPECT_EQ(middle, (std::vector<std::string>{"alpha_int", "beta_int",
                                                "hw_quiesced"}));
    EXPECT_EQ(all_events[4], "hw_exit");

    const std::string manifest =
        ReadFile(options.output_dir + "/manifest.json");
    const auto hw = ManifestRuns(manifest, "hardware_node");
    ASSERT_EQ(hw.size(), 1u);
    EXPECT_NE(hw.back().find("\"reason\": \"stop\""), std::string::npos)
        << hw.back();
    EXPECT_NE(hw.back().find("\"exit_code\": 0"), std::string::npos)
        << hw.back();

    std::filesystem::remove_all(options.output_dir);
    for (const auto* p : {&ready, &events, &release_hw}) ::remove(p->c_str());
  }

  // Case (b): quiesced bridge hanging past kStopTimeout (2 s) IS SIGKILLed
  // with reason stop_timeout.
  {
    const auto ready = TempPath("quiesce_timeout_ready");
    const auto events = TempPath("quiesce_timeout_events");
    const auto release_hw = TempPath("quiesce_timeout_release");
    for (const auto* p : {&ready, &events, &release_hw}) ::remove(p->c_str());

    testing::SteppedClock clock;
    const SupervisionTime start = clock.Freeze();
    auto options = BaseOptions("quiesce_timeout");
    options.duration_s = 3600;
    options.max_restarts = 0;
    options.heartbeat_periods = 1'000'000;
    options.heartbeat_startup_grace_ms = 3'600'000;
    options.construction_timeout = 1h;
    options.supervision_clock = clock.clock();
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        make_bridge("quiesce_timeout_hw", "quiesce", ready, events, release_hw);
    options.binary_overrides["//example/alpha:node"] =
        make_owner("quiesce_timeout_alpha", "alpha", ready, events);
    options.binary_overrides["//example/beta:node"] =
        make_owner("quiesce_timeout_beta", "beta", ready, events);

    const uint64_t session = options.session_id;
    const std::string live =
        options.output_dir + "/" + std::to_string(session) + "/manifest.json";
    testing::SupervisionPasses passes;
    bool hw_alive_before_2s = false;
    std::atomic<bool> session_done{false};
    std::jthread driver{[&](const std::stop_token& stop) {
      const testing::SteppedClock::ReleaseOnExit release_clock{clock};
      if (!WaitFor(stop, [&] {
            const auto init_hw = ManifestRuns(ReadFile(live), "hardware_node");
            return ReadLines(ready).size() == 3 &&
                   Registered("hardware_node", session) && !init_hw.empty() &&
                   init_hw.back().find("\"running\"") != std::string::npos;
          })) {
        ::kill(::getpid(), SIGTERM);
        return;
      }
      ::kill(::getpid(), SIGTERM);
      // Wait for kFlagQuiesced -> kAllSignaled (shutdown_deadline = start +
      // 2s) and for alpha/beta to exit cleanly before advancing to
      // kStopTimeout.
      if (!WaitFor(stop, [&] {
            SupervisionWakeup::Wake();
            if (ReadLines(events).size() < 4) return false;
            const std::string cur = ReadFile(live);
            const auto alpha = ManifestRuns(cur, "alpha");
            const auto beta = ManifestRuns(cur, "beta");
            return !alpha.empty() &&
                   alpha.back().find("\"running\"") == std::string::npos &&
                   !beta.empty() &&
                   beta.back().find("\"running\"") == std::string::npos;
          })) {
        return;
      }
      clock.HoldAt(start + kStopTimeout - 100ms);
      const uint64_t after_1900 = passes.Started() + 2;
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after_1900;
      });
      const auto mid_hw = ManifestRuns(ReadFile(live), "hardware_node");
      hw_alive_before_2s =
          !mid_hw.empty() &&
          mid_hw.back().find("\"running\"") != std::string::npos;
      clock.HoldAt(start + kStopTimeout);
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return session_done.load(std::memory_order_acquire);
      });
    }};

    EXPECT_EQ(RunSession(options), -1);
    session_done.store(true, std::memory_order_release);
    driver.request_stop();
    driver.join();
    EXPECT_TRUE(hw_alive_before_2s)
        << "quiesced hardware_node ended before kStopTimeout (2 s)";
    const std::string manifest =
        ReadFile(options.output_dir + "/manifest.json");
    const auto hw = ManifestRuns(manifest, "hardware_node");
    ASSERT_EQ(hw.size(), 1u);
    EXPECT_NE(hw.back().find("\"reason\": \"stop_timeout\""), std::string::npos)
        << hw.back();
    EXPECT_NE(hw.back().find("\"exit_signal\": 9"), std::string::npos)
        << hw.back();
    for (const char* name : {"alpha", "beta"}) {
      const auto runs = ManifestRuns(manifest, name);
      ASSERT_EQ(runs.size(), 1u);
      EXPECT_NE(runs.back().find("\"reason\": \"stop\""), std::string::npos)
          << name << ": " << runs.back();
    }

    std::filesystem::remove_all(options.output_dir);
    for (const auto* p : {&ready, &events, &release_hw}) ::remove(p->c_str());
  }

  // Case (c): un-quiesced hung bridge (registered row without kFlagQuiesced)
  // is STILL SIGKILLed at kBridgeStopTimeout (500 ms) before actuator owners
  // are signaled.
  {
    const auto ready = TempPath("quiesce_hung_ready");
    const auto events = TempPath("quiesce_hung_events");
    const auto release_hw = TempPath("quiesce_hung_release");
    for (const auto* p : {&ready, &events, &release_hw}) ::remove(p->c_str());

    testing::SteppedClock clock;
    const SupervisionTime start = clock.Freeze();
    auto options = BaseOptions("quiesce_hung");
    options.duration_s = 3600;
    options.max_restarts = 0;
    options.heartbeat_periods = 1'000'000;
    options.heartbeat_startup_grace_ms = 3'600'000;
    options.construction_timeout = 1h;
    options.supervision_clock = clock.clock();
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        make_bridge("quiesce_hung_hw", "hang", ready, events, release_hw);
    options.binary_overrides["//example/alpha:node"] =
        make_owner("quiesce_hung_alpha", "alpha", ready, events);
    options.binary_overrides["//example/beta:node"] =
        make_owner("quiesce_hung_beta", "beta", ready, events);

    const uint64_t session = options.session_id;
    const std::string live =
        options.output_dir + "/" + std::to_string(session) + "/manifest.json";
    testing::SupervisionPasses passes;
    std::vector<std::string> events_before_500ms;
    bool held_before_500ms = false;
    std::atomic<bool> session_done{false};
    std::jthread driver{[&](const std::stop_token& stop) {
      const testing::SteppedClock::ReleaseOnExit release_clock{clock};
      if (!WaitFor(stop, [&] {
            const auto init_hw = ManifestRuns(ReadFile(live), "hardware_node");
            return ReadLines(ready).size() == 3 &&
                   Registered("hardware_node", session) && !init_hw.empty() &&
                   init_hw.back().find("\"running\"") != std::string::npos;
          })) {
        ::kill(::getpid(), SIGTERM);
        return;
      }
      ::kill(::getpid(), SIGTERM);
      if (!WaitFor(stop, [&] {
            const auto lines = ReadLines(events);
            return !lines.empty() && lines[0] == "hw_int";
          })) {
        return;
      }
      clock.HoldAt(start + kBridgeStopTimeout - 50ms);
      const uint64_t after_450 = passes.Started() + 2;
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after_450;
      });
      events_before_500ms = ReadLines(events);
      const std::string mid = ReadFile(live);
      const auto mid_hw = ManifestRuns(mid, "hardware_node");
      const auto mid_alpha = ManifestRuns(mid, "alpha");
      held_before_500ms =
          !mid_hw.empty() &&
          mid_hw.back().find("\"running\"") != std::string::npos &&
          !mid_alpha.empty() &&
          mid_alpha.back().find("\"running\"") != std::string::npos;
      // Hold clock at start + kBridgeStopTimeout until RunSession finishes so
      // ReleaseOnExit cannot jump to wall time (> kStopTimeout) while alpha
      // and beta are handling SIGINT and exiting.
      clock.HoldAt(start + kBridgeStopTimeout);
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return session_done.load(std::memory_order_acquire);
      });
    }};

    EXPECT_EQ(RunSession(options), -1);
    session_done.store(true, std::memory_order_release);
    driver.request_stop();
    driver.join();
    EXPECT_TRUE(held_before_500ms)
        << "un-quiesced hardware_node or alpha ended before 500 ms";
    EXPECT_EQ(events_before_500ms, (std::vector<std::string>{"hw_int"}))
        << "actuator owners were signaled before un-quiesced bridge SIGKILL";
    const std::string manifest =
        ReadFile(options.output_dir + "/manifest.json");
    const auto hw = ManifestRuns(manifest, "hardware_node");
    ASSERT_EQ(hw.size(), 1u);
    EXPECT_NE(hw.back().find("\"reason\": \"stop_timeout\""), std::string::npos)
        << hw.back();
    EXPECT_NE(hw.back().find("\"exit_signal\": 9"), std::string::npos)
        << hw.back();
    for (const char* name : {"alpha", "beta"}) {
      const auto runs = ManifestRuns(manifest, name);
      ASSERT_EQ(runs.size(), 1u);
      EXPECT_NE(runs.back().find("\"reason\": \"stop\""), std::string::npos)
          << name << ": " << runs.back();
    }

    std::filesystem::remove_all(options.output_dir);
    for (const auto* p : {&ready, &events, &release_hw}) ::remove(p->c_str());
  }
}

// RT-2 (software part): late ring mlock refusal recorded into a live
// registry row via NodeRegistration::note_realtime_refused, and warned/merged
// by Launcher::NoteRealtime (both after initial full and partial grants, and at
// reap time via FinalDeliveryCounts) and NodeProcessInfo::RecordRealtime.
// REQ-RT-09, REQ-OPS-18
TEST(Supervision, LateRingLockRefusalWarnsAndMergesIntoManifest) {
  // Part 1: NodeRegistration::note_realtime_refused guards.
  {
    const std::string reg_path =
        "/late_rt_test_" + std::to_string(::getpid()) + ".reg";
    introspect::RegistryMapping::unlink(reg_path);
    {
      introspect::NodeRegistration reg{{.name = "alpha", .session_id = 1},
                                       reg_path};
      auto reader = introspect::RegistryReader::open(reg_path);
      ASSERT_TRUE(reader.has_value());
      // Before set_realtime: note_realtime_refused must be a no-op.
      reg.note_realtime_refused(process::kStepRingLock);
      EXPECT_EQ(reader->nodes().front().realtime(), std::nullopt);
      // Requested=false: note_realtime_refused must be a no-op.
      reg.set_realtime({.tier = process::LoopTier::kControl,
                        .requested = false,
                        .refused = 0});
      reg.note_realtime_refused(process::kStepRingLock);
      ASSERT_TRUE(reader->nodes().front().realtime().has_value());
      EXPECT_EQ(reader->nodes().front().realtime()->refused, 0u);
    }
    introspect::RegistryMapping::unlink(reg_path);
    {
      introspect::NodeRegistration reg{{.name = "alpha", .session_id = 2},
                                       reg_path};
      auto reader = introspect::RegistryReader::open(reg_path);
      ASSERT_TRUE(reader.has_value());
      reg.set_realtime({.tier = process::LoopTier::kControl,
                        .requested = true,
                        .refused = process::kStepStackLock});
      reg.note_realtime_refused(process::kStepRingLock);
      const auto rt = reader->nodes().front().realtime();
      ASSERT_TRUE(rt.has_value());
      EXPECT_EQ(rt->grant(), process::RealtimeGrant::kPartial);
      EXPECT_EQ(rt->refused, process::kStepStackLock | process::kStepRingLock);
    }
    introspect::RegistryMapping::unlink(reg_path);
  }

  // Part 2: live supervision + reap-time merge into manifest.json & talos.log.
  auto options = BaseOptions("late_ring_mlock");
  options.duration_s = kHangSeconds;
  options.max_restarts = 0;
  UseHealthyRegisteringNodes(options, "late_ring_mlock");

  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string live = session_dir + "/manifest.json";
  const std::string talos_log = session_dir + "/talos.log";
  const uint64_t session = options.session_id;

  const auto set_member_flags = [&](std::string_view name, uint32_t bits) {
    introspect::RegistryMapping mapping{introspect::kRegistryPath,
                                        /*read_only=*/false};
    for (auto& row : mapping.header()->nodes) {
      if (row.state.load(std::memory_order_acquire) !=
          static_cast<uint64_t>(introspect::SlotState::ACTIVE)) {
        continue;
      }
      if (row.session_id == session &&
          introspect::read_name(row.name, introspect::kMaxNameBytes) == name) {
        row.flags.fetch_or(bits, std::memory_order_release);
        return true;
      }
    }
    return false;
  };

  bool initial_seen = false;
  bool late_seen = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    if (!WaitFor(stop, [&] {
          return Registered("hardware_node", session) &&
                 Registered("alpha", session) && Registered("beta", session);
        })) {
      ::kill(::getpid(), SIGTERM);
      return;
    }
    // Initial grants: hardware_node = full, alpha = full, beta = partial
    // (stack mlock).
    set_member_flags("hardware_node", introspect::RealtimeFlags(
                                          {.tier = process::LoopTier::kBridge,
                                           .requested = true,
                                           .refused = 0}));
    set_member_flags(
        "alpha", introspect::RealtimeFlags({.tier = process::LoopTier::kControl,
                                            .requested = true,
                                            .refused = 0}));
    set_member_flags("beta", introspect::RealtimeFlags(
                                 {.tier = process::LoopTier::kControl,
                                  .requested = true,
                                  .refused = process::kStepStackLock}));
    SupervisionWakeup::Wake();
    initial_seen = WaitFor(stop, [&] {
      const std::string m = ReadFile(live);
      return ManifestEntry(m, "hardware_node").find("\"realtime\": \"full\"") !=
                 std::string::npos &&
             ManifestEntry(m, "alpha").find("\"realtime\": \"full\"") !=
                 std::string::npos &&
             ManifestEntry(m, "beta").find(
                 "\"realtime_refused\": \"stack mlock\"") != std::string::npos;
    });
    if (initial_seen) {
      // Late kStepRingLock refusal on alpha (after full) and beta (after
      // partial stack mlock) observed during supervision.
      const uint32_t late_ring = static_cast<uint32_t>(process::kStepRingLock)
                                 << (introspect::kFlagRealtimeShift + 3);
      set_member_flags("alpha", late_ring);
      set_member_flags("beta", late_ring);
      SupervisionWakeup::Wake();
      late_seen = WaitFor(stop, [&] {
        const std::string m = ReadFile(live);
        return ManifestEntry(m, "alpha")
                       .find("\"realtime_refused\": \"ring mlock\"") !=
                   std::string::npos &&
               ManifestEntry(m, "beta").find(
                   "\"realtime_refused\": \"ring mlock, stack mlock\"") !=
                   std::string::npos;
      });
      // Late kStepRingLock refusal on hardware_node right at shutdown so
      // reap-time FinalDeliveryCounts -> NoteRealtime merges it.
      set_member_flags("hardware_node", late_ring);
    }
    ::kill(::getpid(), SIGTERM);
  }};

  EXPECT_EQ(RunSession(options), 0);
  driver.join();
  ASSERT_TRUE(initial_seen);
  ASSERT_TRUE(late_seen);

  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string log_text = ReadFile(talos_log);

  const std::string alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"realtime\": \"partial\""), std::string::npos)
      << alpha;
  EXPECT_NE(alpha.find("\"realtime_refused\": \"ring mlock\""),
            std::string::npos)
      << alpha;
  EXPECT_NE(log_text.find("alpha (pid "), std::string::npos) << log_text;
  EXPECT_NE(log_text.find("partial (ring mlock)"), std::string::npos)
      << log_text;

  const std::string beta = ManifestEntry(manifest, "beta");
  EXPECT_NE(beta.find("\"realtime\": \"partial\""), std::string::npos) << beta;
  EXPECT_NE(beta.find("\"realtime_refused\": \"ring mlock, stack mlock\""),
            std::string::npos)
      << beta;
  EXPECT_NE(log_text.find("partial (ring mlock, stack mlock)"),
            std::string::npos)
      << log_text;

  const std::string hw = ManifestEntry(manifest, "hardware_node");
  EXPECT_NE(hw.find("\"realtime\": \"partial\""), std::string::npos) << hw;
  EXPECT_NE(hw.find("\"realtime_refused\": \"ring mlock\""), std::string::npos)
      << hw;

  std::filesystem::remove_all(options.output_dir);
}

// REQ-RT-18: the launcher's stall check reads the loop's dispatch_count, never
// the reporter's heartbeat_monotonic_ns. When alpha's dispatch_count keeps
// advancing while its heartbeat_monotonic_ns stays frozen past both the stall
// timeout (startup grace + kMinHeartbeatTimeout) and
// introspect::kLivenessTimeout (5 s), the launcher does NOT stall-kill alpha.
TEST(Supervision, AdvancingDispatchCountPreventsStallKillWithFrozenHeartbeat) {
  auto options = BaseOptions("frozen_hb_out", "max_restarts = 1\n");
  constexpr auto kGrace = 1000ms;
  options.heartbeat_periods = 1;
  options.heartbeat_startup_grace_ms = static_cast<int>(kGrace.count());
  options.construction_timeout = 1h;
  options.duration_s = 3600;
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  options.supervision_clock = clock.clock();
  UseHealthyRegisteringNodes(options, "frozen_hb");

  const auto pid_file = TempPath("frozen_hb_pid");
  const auto runs = TempPath("frozen_hb_runs");
  ::remove(pid_file.c_str());
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("frozen_hb_alpha",
                TrappedRun(runs, "echo $$ > '" + pid_file + ".tmp' && mv '" +
                                     pid_file + ".tmp' '" + pid_file + "'\n" +
                                     std::string{kLoopUntilStopped}));
  // PublishStubRow stamps heartbeat_monotonic_ns once when claiming the row;
  // this tick advances only dispatch_count so heartbeat_monotonic_ns remains
  // frozen for the entire session.
  auto publisher =
      PublishStubRow(options, pid_file, [](introspect::NodeRecord& row) {
        row.dispatch_count.fetch_add(1, std::memory_order_relaxed);
      });

  const uint64_t session = options.session_id;
  const std::string live =
      options.output_dir + "/" + std::to_string(session) + "/manifest.json";
  const auto read_alpha_row = [&] {
    std::pair<int64_t, uint64_t> out{0, 0};
    introspect::RegistryMapping mapping{introspect::kRegistryPath,
                                        /*read_only=*/true};
    for (const auto& row : mapping.header()->nodes) {
      if (row.state.load(std::memory_order_acquire) !=
          static_cast<uint64_t>(introspect::SlotState::ACTIVE)) {
        continue;
      }
      if (row.session_id == session &&
          introspect::read_name(row.name, introspect::kMaxNameBytes) ==
              "alpha") {
        out.first = row.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
        out.second = row.dispatch_count.load(std::memory_order_relaxed);
        break;
      }
    }
    return out;
  };

  testing::SupervisionPasses passes;
  int64_t initial_heartbeat = 0;
  int64_t final_heartbeat = 0;
  uint64_t initial_dispatch = 0;
  uint64_t final_dispatch = 0;
  bool survived_past_liveness_timeout = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    if (!WaitFor(stop, [&] {
          return Registered("hardware_node", session) &&
                 Registered("alpha", session) && Registered("beta", session);
        })) {
      ::kill(::getpid(), SIGTERM);
      return;
    }
    const auto initial = read_alpha_row();
    initial_heartbeat = initial.first;
    initial_dispatch = initial.second;
    SupervisionTime now = start;
    const SupervisionTime target =
        start + kGrace + introspect::kLivenessTimeout + 1s;
    if (AdvanceClockWithHealthyNodes(stop, clock, passes, session, now,
                                     target)) {
      const uint64_t after = passes.Started() + 2;
      WaitFor(stop, [&] {
        SupervisionWakeup::Wake();
        return passes.Started() >= after;
      });
      const auto current = read_alpha_row();
      final_heartbeat = current.first;
      final_dispatch = current.second;
      const auto history = ManifestRuns(ReadFile(live), "alpha");
      survived_past_liveness_timeout =
          history.size() == 1 &&
          history.front().find("\"reason\": \"running\"") != std::string::npos;
    }
    ::kill(::getpid(), SIGTERM);
  }};

  EXPECT_EQ(RunSession(options), 0);
  driver.join();
  publisher.request_stop();
  publisher.join();

  EXPECT_GT(initial_heartbeat, 0);
  EXPECT_EQ(final_heartbeat, initial_heartbeat)
      << "alpha's heartbeat_monotonic_ns was supposed to stay frozen";
  EXPECT_GT(final_dispatch, initial_dispatch);
  EXPECT_TRUE(survived_past_liveness_timeout)
      << "alpha was stall-killed despite advancing dispatch_count";
  EXPECT_EQ(ReadLines(runs).size(), 1u);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  EXPECT_EQ(EntryInteger(ManifestEntry(manifest, "alpha"), "restart_count"), 0)
      << manifest;
  const auto history = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(history.size(), 1u) << manifest;
  EXPECT_NE(history[0].find("\"reason\": \"stop\""), std::string::npos)
      << history[0];
  std::filesystem::remove_all(options.output_dir);
  ::remove(pid_file.c_str());
  ::remove(runs.c_str());
}

// REQ-SAFE-30: SpawnProcess's guardian must ignore SIGHUP alongside SIGINT and
// SIGTERM so a process-group SIGHUP cannot disarm orphan cleanup before the
// launcher dies.
TEST(Supervision, GuardianIgnoresSighupAndStillKillsOrphanGroup) {
  int info_pipe[2];
  int gc_alive[2];
  int gc_block[2];
  ASSERT_EQ(::pipe(info_pipe), 0);
  ASSERT_EQ(::pipe(gc_alive), 0);
  ASSERT_EQ(::pipe(gc_block), 0);

  // Move gc_alive[1] and gc_block[0] to fds 7 and 8 (below 10 so /bin/sh
  // never reuses them for its own script fd) inside the helper parent so the
  // spawned child and its grandchild inherit them while the guardian closes
  // every descriptor except lifetime[0].
  const std::string script = TempPath("guardian_sighup.sh");
  {
    std::ofstream out{script};
    // Only the grandchild ignores SIGHUP; the child leaves SIGHUP at its
    // default action so group SIGHUP terminates it synchronously in the kernel
    // (avoiding a bash race where a trapped SIGHUP arriving between echo and
    // read leaves pending_traps deferred while read blocks forever).
    out << "#!/bin/sh\n"
           "(trap '' HUP; echo \"gc_ready\"; read ignored <&8) &\n"
           "echo \"ready $!\"\n"
           "read ignored <&8\n";
  }
  ASSERT_EQ(::chmod(script.c_str(), 0755), 0);

  std::fflush(nullptr);
  const pid_t helper = ::fork();
  ASSERT_GE(helper, 0);
  if (helper == 0) {
    ::close(info_pipe[0]);
    ::close(gc_alive[0]);
    ::close(gc_block[1]);
    // Move every kept descriptor above 8 first so dup2(..., 7/8) never collides
    // with pipe fds, and mark info_out close-on-exec so spawned processes do
    // not inherit it.
    const int info_out = ::fcntl(info_pipe[1], F_DUPFD_CLOEXEC, 12);
    const int alive_tmp = ::fcntl(gc_alive[1], F_DUPFD, 12);
    const int block_tmp = ::fcntl(gc_block[0], F_DUPFD, 12);
    ::close(info_pipe[1]);
    ::close(gc_alive[1]);
    ::close(gc_block[0]);
    if (info_out < 0 || alive_tmp < 0 || block_tmp < 0 ||
        ::dup2(alive_tmp, 7) < 0 || ::dup2(block_tmp, 8) < 0)
      ::_exit(100);
    ::close(alive_tmp);
    ::close(block_tmp);
    std::string error;
    int child_out = -1;
    const pid_t child = SpawnProcess({script}, error, SpawnOutput::kPipe,
                                     &child_out, nullptr, /*hold_stop=*/false);
    if (child <= 0) ::_exit(101);
    // Wait for both "ready <gc_pid>\n" and "gc_ready\n" so the grandchild has
    // installed its SIGHUP trap before we send group SIGHUP.
    std::string lines;
    char buf[128];
    while (lines.find("ready ") == std::string::npos ||
           lines.find("gc_ready\n") == std::string::npos) {
      const ssize_t n = ::read(child_out, buf, sizeof buf);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) ::_exit(102);
      lines.append(buf, static_cast<std::size_t>(n));
    }
    // Close our copies of fds 7/8 so only the spawned group holds them.
    ::close(7);
    ::close(8);
    const std::string msg =
        "child " + std::to_string(child) + "\n" + lines + "done\n";
    [[maybe_unused]] const ssize_t w =
        ::write(info_out, msg.data(), msg.size());
    // Wait for group SIGHUP to terminate child, then report "hup\n" to the
    // test and wait until SIGKILLed.
    int child_status = 0;
    while (::waitpid(child, &child_status, 0) < 0 && errno == EINTR) {
    }
    if (WIFSIGNALED(child_status) && WTERMSIG(child_status) == SIGHUP) {
      [[maybe_unused]] const ssize_t w2 = ::write(info_out, "hup\n", 4);
    }
    for (;;) ::pause();
  }

  ::close(info_pipe[1]);
  ::close(gc_alive[1]);
  ::close(gc_block[0]);

  const auto read_until = [&](int fd, std::string& acc,
                              std::string_view marker) {
    while (acc.find(marker) == std::string::npos) {
      pollfd pfd{fd, POLLIN, 0};
      const int r = ::poll(&pfd, 1, kHangSeconds * 1000);
      if (r < 0 && errno == EINTR) continue;
      if (r <= 0) return false;
      char buf[128];
      const ssize_t n = ::read(fd, buf, sizeof buf);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) return false;
      acc.append(buf, static_cast<std::size_t>(n));
    }
    return true;
  };

  std::string info;
  ASSERT_TRUE(read_until(info_pipe[0], info, "done\n")) << info;
  pid_t child_pid = -1;
  pid_t gc_pid = -1;
  {
    std::istringstream in{info};
    std::string tok;
    while (in >> tok) {
      if (tok == "child") in >> child_pid;
      if (tok == "ready") in >> gc_pid;
    }
  }
  ASSERT_GT(child_pid, 0) << info;
  ASSERT_GT(gc_pid, 0) << info;

  // Send SIGHUP to the child's process group (child, grandchild, guardian).
  ASSERT_EQ(::kill(-child_pid, SIGHUP), 0);
  ASSERT_TRUE(read_until(info_pipe[0], info, "hup\n")) << info;

  // Grandchild must still be alive before we kill helper.
  pollfd before_kill{gc_alive[0], POLLIN, 0};
  ASSERT_EQ(::poll(&before_kill, 1, 0), 0)
      << "grandchild exited before parent SIGKILL";

  // Now SIGKILL the helper parent: its death closes lifetime[1]. If the
  // guardian survived SIGHUP, it immediately SIGKILLs the process group,
  // killing the grandchild and closing gc_alive[1] (EOF on gc_alive[0]).
  ::kill(helper, SIGKILL);
  int helper_status = 0;
  while (::waitpid(helper, &helper_status, 0) < 0 && errno == EINTR) {
  }
  ::close(info_pipe[0]);

  pollfd alive_pfd{gc_alive[0], POLLIN, 0};
  int poll_rc = 0;
  do {
    poll_rc = ::poll(&alive_pfd, 1, 2000);
  } while (poll_rc < 0 && errno == EINTR);

  bool gc_killed = false;
  if (poll_rc > 0) {
    char c = 0;
    gc_killed = ::read(gc_alive[0], &c, 1) == 0;
  }

  // Clean up any surviving orphan process group on failure.
  if (!gc_killed) ::kill(-child_pid, SIGKILL);
  ::close(gc_alive[0]);
  ::close(gc_block[1]);
  ::remove(script.c_str());

  EXPECT_TRUE(gc_killed)
      << "grandchild survived parent SIGKILL after group SIGHUP (guardian "
         "likely died on SIGHUP)";
}

// REQ-OPS-07, REQ-OPS-45: ServiceNotifier rejects negative, zero, 1-us,
// overflowing, or malformed WATCHDOG_USEC values instead of turning them into a
// <= 0 watchdog interval busy-spin, and accepts valid values >= 2 us.
TEST(Supervision, ServiceNotifierRejectsInvalidOrTinyWatchdogUsec) {
  const std::string sock_path = "/tmp/twp_" + std::to_string(::getpid());
  ::unlink(sock_path.c_str());
  const int sock = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  ASSERT_GE(sock, 0);
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::memcpy(addr.sun_path, sock_path.c_str(), sock_path.size() + 1);
  ASSERT_EQ(::bind(sock, reinterpret_cast<const sockaddr*>(&addr),
                   static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                          sock_path.size() + 1)),
            0);

  struct Case {
    const char* raw;
    std::optional<std::chrono::microseconds> expected;
  };
  const std::vector<Case> cases = {
      {"-100000", std::nullopt},
      {"0", std::nullopt},
      {"1", std::nullopt},
      {"2", std::chrono::microseconds{1}},
      {"99999999999999999999999", std::nullopt},
      {"1000000", std::chrono::microseconds{500000}},
      {"", std::nullopt},
      {" 1000000", std::nullopt},
      {"+1000000", std::nullopt},
      {"1000000abc", std::nullopt},
      {"9223372036854775808", std::nullopt},
      {"9223372036854775807",
       std::chrono::microseconds{std::numeric_limits<int64_t>::max() / 2}},
  };

  ::setenv("NOTIFY_SOCKET", sock_path.c_str(), 1);
  for (const auto& c : cases) {
    SCOPED_TRACE(std::string{"WATCHDOG_USEC="} + c.raw);
    ::setenv("WATCHDOG_USEC", c.raw, 1);
    ServiceNotifier notifier;
    const auto interval = notifier.watchdog_interval();
    EXPECT_EQ(interval, c.expected);
    if (interval) {
      EXPECT_GE(*interval, std::chrono::microseconds{1});
    }
  }
  ::unsetenv("NOTIFY_SOCKET");
  ::unsetenv("WATCHDOG_USEC");
  ::close(sock);
  ::unlink(sock_path.c_str());
}

}  // namespace
}  // namespace talos::launcher
