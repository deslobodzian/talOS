// Regression tests for four launcher supervision edge cases:
// 1. Critical node clean exit followed by respawn SpawnFailed (REQ-OPS-06,
//    REQ-OPS-08, REQ-SAFE-29).
// 2. RefuseLaunch draining refusal studio_bridge stdout past pipe capacity
//    after port announcement and refreshing session_lock_ and
//    session_directory_ every kLockFileRefresh (REQ-OPS-33, REQ-OPS-02,
//    REQ-OPS-11, REQ-OPS-34, REQ-OPS-45).
// 3. StudioBridge::Finish preserving exit_code when a settling bridge exited
//    normally before SIGKILL (REQ-OPS-08, REQ-OPS-30).
// 4. PublishedEndpoints catching a running node that declared endpoints in
//    --describe but published zero sources at runtime (REQ-CONTRACT-11).

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/lock_file.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/registry.h"
#include "talOS/launcher/child_output.h"
#include "talOS/launcher/launcher.h"
#include "talOS/launcher/session_lock.h"
#include "talOS/launcher/studio_bridge.h"
#include "talOS/launcher/testdata/first_exec.h"
#include "talOS/launcher/testdata/supervision_events.h"
#include "talOS/memory/shared_memory_ptr.h"

namespace talos::launcher {
namespace {

using namespace std::chrono_literals;

constexpr int kHangSeconds = 20;
constexpr int kWaitPollMs = 5;
constexpr int kMaxWaitPolls = 1000;  // 5 s upper bound on event polls.

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_launcher_edge_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

std::string ReadFile(const std::string& path) {
  std::ifstream in{path};
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

std::vector<std::string> ReadLines(const std::string& path) {
  std::ifstream in{path};
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) lines.push_back(line);
  return lines;
}

bool PollUntil(const std::function<bool()>& done,
               int max_polls = kMaxWaitPolls) {
  for (int i = 0; i < max_polls; ++i) {
    if (done()) return true;
    ::poll(nullptr, 0, kWaitPollMs);
  }
  return done();
}

std::string ManifestEntry(const std::string& manifest, std::string_view name) {
  const std::size_t start =
      manifest.find("\"name\": \"" + std::string{name} + "\"");
  if (start == std::string::npos) return {};
  const std::size_t runs = manifest.find("\"runs\": [", start);
  if (runs == std::string::npos) return {};
  return manifest.substr(start, runs - start);
}

std::vector<std::string> ManifestRuns(const std::string& manifest,
                                      std::string_view name) {
  const std::size_t node =
      manifest.find("\"name\": \"" + std::string{name} + "\"");
  if (node == std::string::npos) return {};
  const std::size_t start = manifest.find("\"runs\": [", node);
  if (start == std::string::npos) return {};
  const std::size_t end = manifest.find(']', start);
  if (end == std::string::npos) return {};
  const std::string runs = manifest.substr(start, end - start);
  std::vector<std::string> out;
  for (std::size_t at = runs.find('{'); at != std::string::npos;
       at = runs.find('{', at + 1)) {
    const std::size_t close = runs.find('}', at);
    if (close == std::string::npos) break;
    out.push_back(runs.substr(at, close - at + 1));
  }
  return out;
}

constexpr std::string_view kFixtureToml = R"(
[robot]
name = "edge_fixture"
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

std::string WriteFixtureToml(std::string_view tag) {
  const std::string path = TempPath(tag) + ".toml";
  std::ofstream out{path};
  out << kFixtureToml;
  return path;
}

naming::SourceShape MakeTopicSource(event::SourceKind kind,
                                    std::string_view topic) {
  naming::SourceShape shape;
  shape.kind = kind;
  shape.name = std::string{topic};
  shape.message_bytes = 8;
  shape.alignment = 8;
  shape.slots = 16;
  shape.schema = "edge.Test";
  shape.schema_hash = 0x123456789abcdef0ULL;
  return shape;
}

std::string WriteDescribingStub(std::string_view tag, std::string_view node,
                                std::string_view target,
                                const std::vector<naming::SourceShape>& sources,
                                std::string_view body) {
  introspect::Description description;
  description.name = std::string{node};
  description.target = std::string{target};
  description.sources = sources;
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
  return testing::WriteWarmStub(TempPath(tag) + ".sh", out.str());
}

// Captures sd_notify datagrams over NOTIFY_SOCKET with poll-based waits.
class NotifyCapture {
 public:
  NotifyCapture() {
    static int serial = 0;
    path_ = "/tmp/tle_notify_" + std::to_string(::getpid()) + "_" +
            std::to_string(serial++);
    ::unlink(path_.c_str());
    fd_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::memcpy(addr.sun_path, path_.c_str(), path_.size() + 1);
    if (fd_ < 0 || ::fcntl(fd_, F_SETFL, O_NONBLOCK) < 0 ||
        ::bind(fd_, reinterpret_cast<const sockaddr*>(&addr),
               static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                      path_.size() + 1)) != 0) {
      throw std::runtime_error("cannot bind notify socket " + path_);
    }
    ::setenv("NOTIFY_SOCKET", path_.c_str(), 1);
  }

  ~NotifyCapture() {
    ::unsetenv("NOTIFY_SOCKET");
    if (fd_ >= 0) ::close(fd_);
    ::unlink(path_.c_str());
  }

  NotifyCapture(const NotifyCapture&) = delete;
  NotifyCapture& operator=(const NotifyCapture&) = delete;

  bool WaitFor(std::string_view expected, int timeout_ms) const {
    int remaining = timeout_ms;
    while (remaining > 0) {
      ::pollfd pfd{fd_, POLLIN, 0};
      const int step = remaining < 50 ? remaining : 50;
      const int rc = ::poll(&pfd, 1, step);
      if (rc > 0) {
        char buf[128];
        for (ssize_t n; (n = ::recv(fd_, buf, sizeof(buf), 0)) > 0;) {
          if (std::string_view{buf, static_cast<std::size_t>(n)} == expected) {
            return true;
          }
        }
      }
      remaining -= step;
    }
    return false;
  }

 private:
  std::string path_;
  int fd_{-1};
};

// Reads one line from `fifo_path` (opening O_RDONLY | O_NONBLOCK then polling)
// within `timeout_ms`.
bool ReadLineFromFifo(const std::string& fifo_path, std::string& line,
                      int timeout_ms) {
  const int fd = ::open(fifo_path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) return false;
  std::string buf;
  int remaining = timeout_ms;
  while (remaining > 0) {
    ::pollfd pfd{fd, POLLIN, 0};
    const int step = remaining < 50 ? remaining : 50;
    const int rc = ::poll(&pfd, 1, step);
    if (rc > 0) {
      char chunk[256];
      const ssize_t n = ::read(fd, chunk, sizeof(chunk));
      if (n > 0) {
        buf.append(chunk, static_cast<std::size_t>(n));
        const std::size_t nl = buf.find('\n');
        if (nl != std::string::npos) {
          line = buf.substr(0, nl);
          ::close(fd);
          return true;
        }
      } else if (n == 0) {
        break;
      }
    }
    remaining -= step;
  }
  ::close(fd);
  return false;
}

bool RegistryRowDispatched(std::string_view name, uint64_t session_id,
                           uint64_t min_dispatch) {
  auto reader = introspect::RegistryReader::open();
  if (!reader) return false;
  for (const auto& node : reader->nodes()) {
    if (node.name == name && node.session_id == session_id &&
        (node.flags & introspect::kFlagConstructing) == 0 &&
        node.dispatch_count >= min_dispatch) {
      return true;
    }
  }
  return false;
}

// REQ-OPS-06, REQ-OPS-08, REQ-SAFE-29: A critical node that exits 0
// mid-session and then fails to respawn (SpawnFailed) must leave proc.pid == -1
// and proc.exit_code == -1 (matching runs.back()), so stopping the session
// before recovery returns nonzero from Launcher::Run() rather than 0.
TEST(LauncherEdgeTest,
     CriticalCleanExitThenRespawnFailureReturnsNonzeroAndRecordsFailure) {
  LauncherOptions options;
  options.session_id = GenerateSessionId();
  options.output_dir = TempPath("crit_spawn_fail");
  options.config_path = WriteFixtureToml("crit_spawn_fail");
  options.duration_s = kHangSeconds;
  options.heartbeat_startup_grace_ms = 60000;
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  testing::SteppedClock::ReleaseOnExit release{clock};
  options.supervision_clock = clock.clock();

  const std::string peers_ready = TempPath("crit_spawn_fail_peers");
  ::unlink(peers_ready.c_str());
  const std::string peer_body =
      "trap 'exit 0' INT TERM\n"
      "echo ready >> \"" +
      peers_ready +
      "\"\n"
      "while :; do sleep 0.05; done\n";

  // Run 0 of critical alpha removes its own execute permission and exits 0 so
  // its respawn fails in SpawnNode -> SpawnFailed.
  const std::string alpha_bin = WriteDescribingStub(
      "crit_spawn_fail_alpha", "alpha", "//example/alpha:node", {},
      "chmod a-x \"$0\"\nexit 0\n");
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteDescribingStub("crit_spawn_fail_hw", "hardware_node",
                          "//talOS/bridge:hardware_node", {}, peer_body);
  options.binary_overrides["//example/alpha:node"] = alpha_bin;
  options.binary_overrides["//example/beta:node"] = WriteDescribingStub(
      "crit_spawn_fail_beta", "beta", "//example/beta:node", {}, peer_body);

  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string live = session_dir + "/manifest.json";

  bool saw_spawn_failed = false;
  std::jthread driver{[&] {
    const bool run0_exited = PollUntil([&] {
      SupervisionWakeup::Wake();
      if (ReadLines(peers_ready).size() < 2) return false;
      const auto runs = ManifestRuns(ReadFile(live), "alpha");
      return !runs.empty() &&
             runs[0].find("\"reason\": \"exit\"") != std::string::npos;
    });
    if (run0_exited) {
      // Advance past the 50 ms clean-exit restart backoff so Restart() tries
      // to spawn alpha and hits EACCES -> SpawnFailed().
      clock.HoldAt(start + 100ms);
      saw_spawn_failed = PollUntil([&] {
        SupervisionWakeup::Wake();
        const auto runs = ManifestRuns(ReadFile(live), "alpha");
        return runs.size() >= 2 &&
               runs[1].find("\"reason\": \"spawn_failed\"") !=
                   std::string::npos;
      });
    }
    ::kill(::getpid(), SIGTERM);
  }};

  const int status = Launcher{options}.Run();
  driver.join();
  ::chmod(alpha_bin.c_str(), 0755);

  ASSERT_TRUE(saw_spawn_failed) << ReadFile(live);
  EXPECT_NE(status, 0) << "session must exit nonzero when critical respawn "
                          "failed before stop";
  const std::string manifest = ReadFile(live);
  const std::string alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"pid\": -1,"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"exit_code\": -1,"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"exit_signal\": 0,"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"recorder_failed\": true"), std::string::npos)
      << alpha;
  const auto runs = ManifestRuns(manifest, "alpha");
  ASSERT_EQ(runs.size(), 2u) << manifest;
  EXPECT_NE(runs[0].find("\"reason\": \"exit\""), std::string::npos) << runs[0];
  EXPECT_NE(runs[1].find("\"reason\": \"spawn_failed\""), std::string::npos)
      << runs[1];

  std::filesystem::remove_all(options.output_dir);
  ::unlink(peers_ready.c_str());
}

// REQ-OPS-33, REQ-OPS-02, REQ-OPS-34: RefuseLaunch must keep draining the
// refusal studio_bridge's stdout even after port_announced (so writing > 64 KiB
// after announcing its port never deadlocks) and must refresh session_lock_
// every kLockFileRefresh.
TEST(LauncherEdgeTest,
     RefuseLaunchDrainsStdoutPastPipeCapacityAndRefreshesSessionLock) {
  NotifyCapture notify;
  const std::string go_fifo = TempPath("refuse_drain_go.fifo");
  const std::string done_fifo = TempPath("refuse_drain_done.fifo");
  ::unlink(go_fifo.c_str());
  ::unlink(done_fifo.c_str());
  ASSERT_EQ(::mkfifo(go_fifo.c_str(), 0600), 0);
  ASSERT_EQ(::mkfifo(done_fifo.c_str(), 0600), 0);

  LauncherOptions options;
  options.session_id = GenerateSessionId();
  options.output_dir = TempPath("refuse_drain");
  options.config_path = WriteFixtureToml("refuse_drain");
  options.start_studio_bridge = true;
  options.serve_refusal = true;
  options.studio_http_port = 0;
  options.duration_s = 1'000'000;
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  testing::SteppedClock::ReleaseOnExit release{clock};
  options.supervision_clock = clock.clock();

  // Two writers on /alpha/state triggers a phase-1 graph refusal.
  const auto writer =
      MakeTopicSource(event::SourceKind::SENDER, "/alpha/state");
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteDescribingStub("refuse_drain_hw", "hardware_node",
                          "//talOS/bridge:hardware_node", {}, "exit 0\n");
  options.binary_overrides["//example/alpha:node"] =
      WriteDescribingStub("refuse_drain_alpha", "alpha", "//example/alpha:node",
                          {writer}, "exit 0\n");
  options.binary_overrides["//example/beta:node"] = WriteDescribingStub(
      "refuse_drain_beta", "beta", "//example/beta:node", {writer}, "exit 0\n");

  // Refusal bridge stub: announces port, waits for trigger (after READY=1),
  // writes 256 KiB (> 64 KiB pipe capacity) to stdout, signals done_fifo,
  // then waits for SIGINT/SIGTERM.
  const std::string bridge_body =
      "trap 'exit 0' INT TERM\n"
      "printf 'listening 18080\\n'\n"
      "read _ < \"" +
      go_fifo +
      "\"\n"
      "dd if=/dev/zero bs=4096 count=64 2>/dev/null | tr '\\0' 'x'\n"
      "printf '\\n'\n"
      "printf 'drained\\n' > \"" +
      done_fifo +
      "\"\n"
      "while :; do sleep 0.05; done\n";
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteDescribingStub("refuse_drain_bridge", "studio_bridge",
                          "//studio/bridge:studio_bridge", {}, bridge_body);

  int exit_code = -1;
  std::thread runner([&] { exit_code = Launcher{options}.Run(); });

  const bool ready = notify.WaitFor("READY=1", 10000);
  // Open go_fifo to unblock the bridge's 256 KiB write (only after READY=1
  // proves port_announced == true).
  {
    const int fd = ::open(go_fifo.c_str(), O_WRONLY);
    if (fd >= 0) {
      constexpr char kGo[] = "go\n";
      [[maybe_unused]] const ssize_t w = ::write(fd, kGo, sizeof(kGo) - 1);
      ::close(fd);
    }
  }

  std::string done_line;
  const bool drained = ReadLineFromFifo(done_fifo, done_line, 2000);

  // Verify session_lock_ is refreshed every kLockFileRefresh during refusal.
  const std::string lock_path =
      introspect::LockFilePath(ShmObjectName(kSessionLockName));
  constexpr time_t kAged = 1'000'000;
  const auto age_lock = [&lock_path] {
    const timespec times[2] = {{kAged, 0}, {kAged, 0}};
    return ::utimensat(AT_FDCWD, lock_path.c_str(), times, 0) == 0;
  };
  const auto lock_aged = [&lock_path] {
    struct stat st {};
    return ::stat(lock_path.c_str(), &st) == 0 && st.st_mtime == kAged;
  };

  bool aged_ok = false;
  bool stayed_aged_before_due = false;
  bool refreshed_when_due = false;
  if (ready && drained) {
    aged_ok = age_lock();
    if (aged_ok) {
      clock.HoldAt(start + std::chrono::nanoseconds{kLockFileRefresh} / 2);
      // Give the refusal loop a couple of wakeups; mtime must stay aged.
      SupervisionWakeup::Wake();
      ::poll(nullptr, 0, 20);
      stayed_aged_before_due = lock_aged();

      clock.HoldAt(start + kLockFileRefresh);
      refreshed_when_due = PollUntil(
          [&] {
            SupervisionWakeup::Wake();
            return !lock_aged();
          },
          200);
    }
  }

  ::kill(::getpid(), SIGINT);
  runner.join();

  EXPECT_TRUE(ready) << "RefuseLaunch did not send READY=1";
  EXPECT_TRUE(drained)
      << "refusal studio_bridge deadlocked writing 256 KiB to stdout after "
         "port_announced";
  EXPECT_EQ(done_line, "drained");
  EXPECT_TRUE(aged_ok) << "failed to age session lock " << lock_path;
  EXPECT_TRUE(stayed_aged_before_due)
      << "session lock refreshed before kLockFileRefresh elapsed";
  EXPECT_TRUE(refreshed_when_due)
      << "RefuseLaunch did not refresh session_lock_ at kLockFileRefresh";
  EXPECT_EQ(exit_code, 1);

  ::unlink(go_fifo.c_str());
  ::unlink(done_fifo.c_str());
  std::filesystem::remove_all(options.output_dir);
}

// REQ-OPS-11, REQ-OPS-45: RefuseLaunch must refresh session_directory_ (its
// session.lock and quiet files such as graph.json) every kLockFileRefresh and
// retake a removed session.lock, logging the retake once.
TEST(LauncherEdgeTest,
     RefuseLaunchRefreshesAndRetakesSessionDirectoryLockAndQuietFiles) {
  NotifyCapture notify;

  LauncherOptions options;
  options.session_id = GenerateSessionId();
  options.output_dir = TempPath("refuse_dir_lock");
  options.config_path = WriteFixtureToml("refuse_dir_lock");
  options.start_studio_bridge = true;
  options.serve_refusal = true;
  options.studio_http_port = 0;
  options.duration_s = 1'000'000;
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
  testing::SteppedClock::ReleaseOnExit release{clock};
  options.supervision_clock = clock.clock();

  // Two writers on /alpha/state triggers a phase-1 graph refusal after
  // CreateSessionDirectory, talos.log setup, and graph.json write.
  const auto writer =
      MakeTopicSource(event::SourceKind::SENDER, "/alpha/state");
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteDescribingStub("refuse_dir_lock_hw", "hardware_node",
                          "//talOS/bridge:hardware_node", {}, "exit 0\n");
  options.binary_overrides["//example/alpha:node"] =
      WriteDescribingStub("refuse_dir_lock_alpha", "alpha",
                          "//example/alpha:node", {writer}, "exit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteDescribingStub("refuse_dir_lock_beta", "beta", "//example/beta:node",
                          {writer}, "exit 0\n");

  const std::string bridge_body =
      "trap 'exit 0' INT TERM\n"
      "printf 'listening 18080\\n'\n"
      "while :; do sleep 0.05; done\n";
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteDescribingStub("refuse_dir_lock_bridge", "studio_bridge",
                          "//studio/bridge:studio_bridge", {}, bridge_body);

  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string graph_path = session_dir + "/graph.json";
  const std::string dir_lock_path =
      session_dir + "/" + std::string{kSessionDirectoryLockFile};
  const std::string talos_log =
      session_dir + "/" + std::string{kLauncherOutputFile};

  int exit_code = -1;
  std::thread runner([&] { exit_code = Launcher{options}.Run(); });

  const bool ready = notify.WaitFor("READY=1", 10000);

  constexpr time_t kAged = 1'000'000;
  const auto age_graph = [&graph_path] {
    const timespec times[2] = {{kAged, 0}, {kAged, 0}};
    return ::utimensat(AT_FDCWD, graph_path.c_str(), times, 0) == 0;
  };
  const auto graph_aged = [&graph_path] {
    struct stat st {};
    return ::stat(graph_path.c_str(), &st) == 0 && st.st_mtime == kAged;
  };
  constexpr std::string_view kRetaken = "was removed while this session held";
  const auto retake_reports = [&talos_log, kRetaken] {
    const std::string text = ReadFile(talos_log);
    std::size_t count = 0;
    for (std::size_t at = text.find(kRetaken); at != std::string::npos;
         at = text.find(kRetaken, at + 1)) {
      ++count;
    }
    return count;
  };

  bool aged_ok = false;
  bool stayed_aged_before_due = false;
  bool unlinked_ok = false;
  bool refreshed_and_logged = false;
  bool dir_lock_held_after_retake = false;
  bool second_aged_ok = false;
  bool second_unlinked_ok = false;
  bool retaken_again = false;

  if (ready) {
    aged_ok = age_graph();
    if (aged_ok) {
      clock.HoldAt(start + std::chrono::nanoseconds{kLockFileRefresh} / 2);
      SupervisionWakeup::Wake();
      ::poll(nullptr, 0, 20);
      stayed_aged_before_due = graph_aged();

      unlinked_ok = ::unlink(dir_lock_path.c_str()) == 0;
      if (unlinked_ok) {
        clock.HoldAt(start + kLockFileRefresh);
        refreshed_and_logged = PollUntil(
            [&] {
              SupervisionWakeup::Wake();
              return !graph_aged() && retake_reports() > 0;
            },
            2000);
        dir_lock_held_after_retake =
            SessionDirectoryLock::Held(session_dir, false);

        // Unlink a second time: Refresh retakes it again and logs the retake
        // only once across the refusal.
        second_aged_ok = age_graph();
        second_unlinked_ok = ::unlink(dir_lock_path.c_str()) == 0;
        if (second_aged_ok && second_unlinked_ok) {
          clock.HoldAt(start + 2 * kLockFileRefresh);
          retaken_again = PollUntil(
              [&] {
                SupervisionWakeup::Wake();
                return !graph_aged() &&
                       SessionDirectoryLock::Held(session_dir, false);
              },
              2000);
        }
      }
    }
  }

  ::kill(::getpid(), SIGINT);
  runner.join();

  EXPECT_TRUE(ready) << "RefuseLaunch did not send READY=1";
  EXPECT_TRUE(aged_ok) << "failed to age " << graph_path;
  EXPECT_TRUE(stayed_aged_before_due)
      << "graph.json refreshed before kLockFileRefresh elapsed";
  EXPECT_TRUE(unlinked_ok) << "failed to unlink " << dir_lock_path;
  EXPECT_TRUE(refreshed_and_logged)
      << "RefuseLaunch did not refresh graph.json and log session.lock retake "
         "at kLockFileRefresh:\n"
      << ReadFile(talos_log);
  EXPECT_TRUE(dir_lock_held_after_retake)
      << "session.lock was not retaken by RefuseLaunch";
  EXPECT_TRUE(second_aged_ok) << "failed to re-age " << graph_path;
  EXPECT_TRUE(second_unlinked_ok)
      << "failed to unlink " << dir_lock_path << " a second time";
  EXPECT_TRUE(retaken_again)
      << "session.lock was not retaken on a second refresh";
  EXPECT_EQ(retake_reports(), 1u) << ReadFile(talos_log);
  EXPECT_EQ(exit_code, 1);

  std::filesystem::remove_all(options.output_dir);
}

// REQ-OPS-08, REQ-OPS-30: StudioBridge::Finish must record proc.exit_code from
// WIFEXITED(status) when a settling bridge exited normally right before
// SIGKILL, and -1 with exit_signal == SIGKILL when killed while running.
TEST(LauncherEdgeTest,
     StudioBridgeFinishRecordsExitCodeWhenSettlingBridgeExitedNormally) {
  const std::string out_dir = TempPath("studio_finish");
  std::filesystem::remove_all(out_dir);
  std::filesystem::create_directories(out_dir);
  const std::string config_path = WriteFixtureToml("studio_finish");
  const config::RobotConfig robot_config =
      config::ParseRobotConfig(config_path);

  // Case 1: settling bridge exits with code 7 before Finish() runs.
  {
    const std::string ready_fifo = TempPath("studio_finish_exit7.fifo");
    ::unlink(ready_fifo.c_str());
    ASSERT_EQ(::mkfifo(ready_fifo.c_str(), 0600), 0);

    const std::string bin = WriteDescribingStub(
        "studio_finish_exit7", "studio_bridge", "//studio/bridge:studio_bridge",
        {}, "printf 'bye\\n' > \"" + ready_fifo + "\"\nexit 7\n");

    LauncherOptions options;
    options.session_id = GenerateSessionId();
    options.output_dir = out_dir;
    options.config_path = config_path;
    options.studio_http_port = 0;

    NodeSpec spec;
    spec.name = std::string{kStudioBridgeName};
    spec.target = "//studio/bridge:studio_bridge";
    spec.binary_path = bin;
    spec.critical = false;
    spec.studio_bridge = true;

    NodeProcessInfo proc;
    proc.name = spec.name;
    proc.target = spec.target;
    proc.critical = false;
    proc.studio_bridge = true;
    proc.binary_path = spec.binary_path;

    NodeDescription desc;
    desc.roster_name = spec.name;
    desc.target = spec.target;
    desc.answered = true;

    testing::SteppedClock clock;
    clock.Freeze();
    ChildOutput output;
    StudioBridge bridge{0,       spec,
                        proc,    robot_config,
                        options, RestartLimits{5, 60000ms},
                        {desc},  out_dir + "/graph.json",
                        output};
    bridge.Start(proc, clock.clock());
    ASSERT_GT(proc.pid, 0);

    std::string bye;
    ASSERT_TRUE(ReadLineFromFifo(ready_fifo, bye, 2000));
    // Wait until the child is a zombie without reaping it (WNOWAIT), so
    // Finish() deterministically reaps the normal exit status (code 7).
    const bool exited_before_finish = PollUntil([&] {
      siginfo_t info{};
      return ::waitid(P_PID, static_cast<id_t>(proc.pid), &info,
                      WEXITED | WNOHANG | WNOWAIT) == 0 &&
             info.si_pid == proc.pid;
    });
    ASSERT_TRUE(exited_before_finish);

    bridge.Finish(proc, /*stop_requested=*/true);
    output.Stop();

    EXPECT_TRUE(proc.exited);
    EXPECT_EQ(proc.exit_code, 7);
    EXPECT_EQ(proc.exit_signal, 0);
    ASSERT_EQ(proc.runs.size(), 1u);
    EXPECT_EQ(proc.runs[0].exit_code, 7);
    EXPECT_EQ(proc.runs[0].exit_signal, 0);
    ::unlink(ready_fifo.c_str());
  }

  // Case 2: settling bridge is still running -> Finish() sends SIGKILL.
  {
    const std::string ready_fifo = TempPath("studio_finish_kill.fifo");
    ::unlink(ready_fifo.c_str());
    ASSERT_EQ(::mkfifo(ready_fifo.c_str(), 0600), 0);

    const std::string bin = WriteDescribingStub(
        "studio_finish_kill", "studio_bridge", "//studio/bridge:studio_bridge",
        {}, "printf 'up\\n' > \"" + ready_fifo + "\"\nexec sleep 30\n");

    LauncherOptions options;
    options.session_id = GenerateSessionId();
    options.output_dir = out_dir;
    options.config_path = config_path;
    options.studio_http_port = 0;

    NodeSpec spec;
    spec.name = std::string{kStudioBridgeName};
    spec.target = "//studio/bridge:studio_bridge";
    spec.binary_path = bin;
    spec.critical = false;
    spec.studio_bridge = true;

    NodeProcessInfo proc;
    proc.name = spec.name;
    proc.target = spec.target;
    proc.critical = false;
    proc.studio_bridge = true;
    proc.binary_path = spec.binary_path;

    NodeDescription desc;
    desc.roster_name = spec.name;
    desc.target = spec.target;
    desc.answered = true;

    testing::SteppedClock clock;
    clock.Freeze();
    ChildOutput output;
    StudioBridge bridge{0,       spec,
                        proc,    robot_config,
                        options, RestartLimits{5, 60000ms},
                        {desc},  out_dir + "/graph.json",
                        output};
    bridge.Start(proc, clock.clock());
    ASSERT_GT(proc.pid, 0);

    std::string up;
    ASSERT_TRUE(ReadLineFromFifo(ready_fifo, up, 2000));

    bridge.Finish(proc, /*stop_requested=*/true);
    output.Stop();

    EXPECT_TRUE(proc.exited);
    EXPECT_EQ(proc.exit_code, -1);
    EXPECT_EQ(proc.exit_signal, SIGKILL);
    ASSERT_EQ(proc.runs.size(), 1u);
    EXPECT_EQ(proc.runs[0].exit_code, -1);
    EXPECT_EQ(proc.runs[0].exit_signal, SIGKILL);
    ::unlink(ready_fifo.c_str());
  }

  std::filesystem::remove_all(out_dir);
}

// REQ-CONTRACT-11: A node that declares endpoints in --describe but publishes
// zero sources at runtime (once kFlagConstructing == 0 and its loop has
// started) must trigger CheckEndpoints() and stop the session with
// endpoint_mismatch = true, while a node declaring zero sources that also
// publishes zero sources must not trigger a false mismatch.
TEST(LauncherEdgeTest,
     ZeroRuntimeSourcesWhenProbeDeclaredEndpointsTriggersMismatch) {
  std::string rf_error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &rf_error)};
  ASSERT_TRUE(runfiles != nullptr) << rf_error;
  const char* loss_node_env = std::getenv("TALOS_DELIVERY_LOSS_NODE");
  ASSERT_NE(loss_node_env, nullptr);
  const std::string loss_node = runfiles->Rlocation(loss_node_env);
  ASSERT_FALSE(loss_node.empty());

  // Case 1: alpha declares /alpha/state in --describe, but runs with 0 sources.
  {
    LauncherOptions options;
    options.session_id = GenerateSessionId();
    options.output_dir = TempPath("zero_sources_mismatch");
    options.config_path = WriteFixtureToml("zero_sources_mismatch");
    options.max_restarts = 0;
    options.duration_s = kHangSeconds;
    options.heartbeat_startup_grace_ms = 60000;

    const auto sender =
        MakeTopicSource(event::SourceKind::SENDER, "/alpha/state");
    const auto watcher =
        MakeTopicSource(event::SourceKind::WATCHER, "/alpha/state");
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteDescribingStub("zero_src_mm_hw", "hardware_node",
                            "//talOS/bridge:hardware_node", {}, "exit 0\n");
    options.binary_overrides["//example/alpha:node"] = WriteDescribingStub(
        "zero_src_mm_alpha", "alpha", "//example/alpha:node", {sender},
        "exec \"" + loss_node +
            "\" alpha //example/alpha:node 0,0,0,0,run \"$@\"\n");
    options.binary_overrides["//example/beta:node"] =
        WriteDescribingStub("zero_src_mm_beta", "beta", "//example/beta:node",
                            {watcher}, "exit 0\n");

    const uint64_t session_id = options.session_id;
    testing::SupervisionPasses passes;
    std::atomic<bool> finished{false};
    std::jthread fallback_stopper{[&] {
      // Wait until alpha has dispatched >= 2 times and 3 supervision passes
      // have observed it. With the fix, Launcher stops the session itself;
      // without the fix, this stops the session quickly so RED fails fast.
      const bool observed = PollUntil([&] {
        if (finished.load()) return true;
        return RegistryRowDispatched("alpha", session_id, 2);
      });
      if (observed && !finished.load()) {
        const uint64_t target_pass = passes.Started() + 3;
        PollUntil(
            [&] {
              if (finished.load()) return true;
              SupervisionWakeup::Wake();
              return passes.Started() >= target_pass;
            },
            200);
      }
      if (!finished.load()) ::kill(::getpid(), SIGTERM);
    }};

    const int status = Launcher{options}.Run();
    finished.store(true);
    fallback_stopper.join();

    const std::string manifest =
        ReadFile(options.output_dir + "/" + std::to_string(session_id) +
                 "/manifest.json");
    const std::string alpha = ManifestEntry(manifest, "alpha");
    EXPECT_EQ(status, 1) << manifest;
    EXPECT_NE(alpha.find("\"endpoint_mismatch\": true"), std::string::npos)
        << alpha;
    std::filesystem::remove_all(options.output_dir);
  }

  // Case 2: alpha declares 0 sources in --describe and runs with 0 sources ->
  // no false mismatch.
  {
    LauncherOptions options;
    options.session_id = GenerateSessionId();
    options.output_dir = TempPath("zero_sources_match");
    options.config_path = WriteFixtureToml("zero_sources_match");
    options.max_restarts = 0;
    options.duration_s = kHangSeconds;
    options.heartbeat_startup_grace_ms = 60000;

    options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteDescribingStub("zero_src_ok_hw", "hardware_node",
                            "//talOS/bridge:hardware_node", {}, "exit 0\n");
    options.binary_overrides["//example/alpha:node"] = WriteDescribingStub(
        "zero_src_ok_alpha", "alpha", "//example/alpha:node", {},
        "exec \"" + loss_node +
            "\" alpha //example/alpha:node 0,0,0,0,run \"$@\"\n");
    options.binary_overrides["//example/beta:node"] = WriteDescribingStub(
        "zero_src_ok_beta", "beta", "//example/beta:node", {}, "exit 0\n");

    const uint64_t session_id = options.session_id;
    testing::SupervisionPasses passes;
    std::jthread stopper{[&] {
      const bool observed = PollUntil(
          [&] { return RegistryRowDispatched("alpha", session_id, 2); });
      if (observed) {
        const uint64_t target_pass = passes.Started() + 2;
        PollUntil(
            [&] {
              SupervisionWakeup::Wake();
              return passes.Started() >= target_pass;
            },
            200);
      }
      ::kill(::getpid(), SIGTERM);
    }};

    const int status = Launcher{options}.Run();
    stopper.join();

    const std::string manifest =
        ReadFile(options.output_dir + "/" + std::to_string(session_id) +
                 "/manifest.json");
    const std::string alpha = ManifestEntry(manifest, "alpha");
    EXPECT_EQ(status, 0) << manifest;
    EXPECT_NE(alpha.find("\"endpoint_mismatch\": false"), std::string::npos)
        << alpha;
    std::filesystem::remove_all(options.output_dir);
  }
}

}  // namespace
}  // namespace talos::launcher
