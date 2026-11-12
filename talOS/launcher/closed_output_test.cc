// The launcher's stdout and stderr readers going away mid-session (an ssh
// session or a `tee` closing): the session runs on, and what it spawns
// starts with default signal dispositions and an empty mask whatever the
// launcher's are.
//
// Every wait here is on an event -- a pipe's data or its end -- bounded by
// kHangMs, which only bounds a hang (docs/TESTING.md §6.3).

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <string_view>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/launcher/launcher.h"

namespace talos::launcher {
namespace {

// How long a wait for one event may see nothing before the test gives up.
constexpr int kHangMs = 20000;

// The descriptor a forked launcher's members find the test's event pipe on.
// They inherit it across exec; their guardians close it.
constexpr int kEventFd = 9;

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_closed_output_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

constexpr std::string_view kFixtureToml = R"(
[robot]
name = "closed_output_fixture"
period_us = 5000

[subsystems.alpha]
node = "//example/alpha:node"
period_us = 5000

[subsystems.alpha.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
)";

// A shell stub for `node` that answers --describe, then runs `body`.
std::string WriteStub(std::string_view tag, std::string_view body,
                      std::string_view node) {
  const std::string path = TempPath(tag) + ".sh";
  introspect::Description description;
  description.name = std::string{node};
  description.target = node == "hardware_node"
                           ? "//talOS/bridge:hardware_node"
                           : "//example/" + std::string{node} + ":node";
  std::ofstream out{path};
  out << "#!/bin/sh\n"
      << "for arg in \"$@\"; do\n"
      << "  if [ \"$arg\" = \"--describe\" ]; then\n"
      << "    cat <<'TALOS_DESCRIBE'\n"
      << introspect::DescribeToJson(description) << "TALOS_DESCRIBE\n"
      << "    exit 0\n"
      << "  fi\n"
      << "done\n"
      << body;
  out.close();
  ::chmod(path.c_str(), 0755);
  return path;
}

std::string ReadFile(const std::string& path) {
  std::ifstream in{path};
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

// A session of the fixture: a hardware stub that runs until stopped, and
// alpha, which prints "tick" every 20 ms. Once its stop trap is set, the
// hardware stub writes "hardware ready" to kEventFd, and each alpha run
// "alpha <run>"; run 1 also writes "alpha 1 ticked" once it has printed
// twenty ticks. Once `crash` exists, run 0 exits 1. A stop signalled before
// both are ready could end a stub by the signal rather than its trap.
struct Fixture {
  LauncherOptions options;
  std::string crash;
  std::string crashed;
  std::string session_dir;

  explicit Fixture(std::string_view tag) {
    options.session_id = GenerateSessionId();
    options.output_dir = TempPath(std::string{tag} + "_session");
    options.describe_timeout_ms = 4000;
    // The stubs never take a registry row; nothing here waits for one.
    options.heartbeat_startup_grace_ms = 60000;
    options.config_path = TempPath(std::string{tag} + "_fixture") + ".toml";
    std::ofstream{options.config_path} << kFixtureToml;
    crash = TempPath(std::string{tag} + "_crash");
    crashed = TempPath(std::string{tag} + "_crashed");
    ::remove(crash.c_str());
    ::remove(crashed.c_str());
    const std::string events = std::to_string(kEventFd);
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStub(std::string{tag} + "_hardware",
                  "trap 'exit 0' INT TERM\necho 'hardware ready' >&" + events +
                      "\nwhile :; do sleep 0.05; done\n",
                  "hardware_node");
    std::string alpha = "trap 'exit 0' INT TERM\nrun=0\n";
    alpha += "[ -e " + crashed + " ] && run=1\n";
    alpha += "echo \"alpha $run\" >&" + events + "\nticks=0\nwhile :; do\n";
    alpha += "  echo tick\n  ticks=$((ticks + 1))\n";
    alpha += "  if [ $run = 1 ] && [ $ticks = 20 ]; then\n";
    alpha += "    echo 'alpha 1 ticked' >&" + events + "\n  fi\n";
    alpha += "  if [ $run = 0 ] && [ -e " + crash + " ]; then\n";
    alpha += "    : > " + crashed + "\n    exit 1\n  fi\n";
    alpha += "  sleep 0.02\ndone\n";
    options.binary_overrides["//example/alpha:node"] =
        WriteStub(std::string{tag} + "_alpha", alpha, "alpha");
    session_dir = options.output_dir + "/" + std::to_string(options.session_id);
  }
  ~Fixture() {
    std::filesystem::remove_all(options.output_dir);
    ::remove(crash.c_str());
    ::remove(crashed.c_str());
  }
  Fixture(const Fixture&) = delete;
  Fixture& operator=(const Fixture&) = delete;
};

// A launcher forked from the test, its stdout and stderr both `output`.
struct Forked {
  pid_t pid{-1};
  int events{-1};  // The read end of its kEventFd; ends once all have exited.
};

// `reader`, when not -1, is the test's end of an `output` pipe: closed in
// the launcher, so that the test's close leaves the pipe without a reader.
// With `ignore_hangup` the launcher starts with SIGHUP ignored, as `nohup`
// starts it.
Forked ForkLauncher(const LauncherOptions& options, int output, int reader = -1,
                    bool ignore_hangup = false) {
  int events[2];
  if (::pipe(events) != 0) return {};
  // Nothing the test buffered may reach the launcher's output twice.
  std::fflush(nullptr);
  const pid_t pid = ::fork();
  if (pid < 0) return {};
  if (pid == 0) {
    if (reader >= 0) ::close(reader);
    ::close(events[0]);
    if (::dup2(output, STDOUT_FILENO) < 0 ||
        ::dup2(output, STDERR_FILENO) < 0 || ::dup2(events[1], kEventFd) < 0)
      ::_exit(100);
    ::close(output);
    ::close(events[1]);
    // Started as a shell starts it, whatever this test inherited (a harness
    // may ignore SIGPIPE).
    std::signal(SIGPIPE, SIG_DFL);
    std::signal(SIGHUP, ignore_hangup ? SIG_IGN : SIG_DFL);
    const int code = Launcher{options}.Run();
    // As main's return would.
    std::cout.flush();
    std::fflush(nullptr);
    ::_exit(code);
  }
  ::close(events[1]);
  return {pid, events[0]};
}

// Reads `fd` into `seen` until `want` appears in it. False when `fd` ends,
// or stays silent for kHangMs, first.
bool AwaitText(int fd, std::string& seen, std::string_view want) {
  while (seen.find(want) == std::string::npos) {
    ::pollfd ready{fd, POLLIN, 0};
    const int polled = ::poll(&ready, 1, kHangMs);
    if (polled < 0 && errno == EINTR) continue;
    if (polled <= 0) return false;
    char buffer[4096];
    const ssize_t got = ::read(fd, buffer, sizeof buffer);
    if (got < 0 && errno == EINTR) continue;
    if (got <= 0) return false;
    seen.append(buffer, static_cast<std::size_t>(got));
  }
  return true;
}

// Reads `fd` until it ends. False when it stays silent for kHangMs first.
bool AwaitEnd(int fd, std::string& seen) {
  for (;;) {
    ::pollfd ready{fd, POLLIN, 0};
    const int polled = ::poll(&ready, 1, kHangMs);
    if (polled < 0 && errno == EINTR) continue;
    if (polled <= 0) return false;
    char buffer[4096];
    const ssize_t got = ::read(fd, buffer, sizeof buffer);
    if (got < 0 && errno == EINTR) continue;
    if (got <= 0) return true;
    seen.append(buffer, static_cast<std::size_t>(got));
  }
}

// Reaps `pid`, which has exited or is about to (its event pipe ended).
int Reap(pid_t pid) {
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
  return status;
}

// REQ-OPS-33
// A session whose launcher stdout and stderr are one pipe (`launcher 2>&1 |
// tee`): once alpha's output has been echoed through it, the test closes the
// pipe's only reader, then makes alpha's first run exit 1. The launcher's
// lines about that exit and the restart, the echo of alpha's output, and its
// lines at the stop and its exit all write into a pipe nobody reads. A
// launcher that dies of SIGPIPE ends the session, and its nodes' guardians
// kill them. It must keep supervising: alpha is restarted, its second run's
// output reaches its log, and SIGTERM still stops the session with status 0.
TEST(ClosedOutput, SessionOutlivesItsStdoutReader) {
  Fixture fixture{"closed"};
  const std::string alpha_log = NodeOutputPath(fixture.session_dir, "alpha");
  int output[2];
  ASSERT_EQ(::pipe(output), 0);
  const Forked launcher = ForkLauncher(fixture.options, output[1], output[0]);
  ::close(output[1]);
  ASSERT_GT(launcher.pid, 0);

  // Read the launcher's output until alpha's echo shows up in it.
  std::string seen;
  const bool echoed = AwaitText(output[0], seen, "tick");
  ::close(output[0]);
  std::ofstream{fixture.crash}.close();

  std::string events;
  const bool restarted = AwaitText(launcher.events, events, "alpha 1 ticked");
  int status = 0;
  const bool alive = ::waitpid(launcher.pid, &status, WNOHANG) == 0;
  if (alive) ::kill(launcher.pid, SIGTERM);
  const bool ended = AwaitEnd(launcher.events, events);
  ::close(launcher.events);
  if (alive) status = Reap(launcher.pid);

  ASSERT_TRUE(echoed) << "alpha's output never reached the launcher's stdout: "
                      << seen;
  EXPECT_TRUE(alive) << "the launcher ended once its stdout reader closed: "
                     << ExitReason(status);
  EXPECT_TRUE(restarted) << "alpha's second run never printed twenty lines; "
                            "events: "
                         << events;
  EXPECT_TRUE(ended) << "the session did not end after SIGTERM";
  const std::string log = ReadFile(alpha_log);
  const std::size_t second_run = log.find("[launcher] alpha run 1,");
  ASSERT_NE(second_run, std::string::npos) << alpha_log << ":\n" << log;
  std::size_t ticks = 0;
  for (std::size_t at = log.find("tick", second_run); at != std::string::npos;
       at = log.find("tick", at + 1))
    ++ticks;
  EXPECT_GE(ticks, 20u) << alpha_log << ":\n" << log;
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "a stop after the close ended with " << ExitReason(status);
}

// REQ-OPS-33
// A terminal hangup (an interactive ssh session or a terminal window
// closing) sends the launcher SIGHUP. Row "hangup": a launcher started with
// SIGHUP at its default stops the session in order on it, as on SIGTERM:
// members stopped (each run's reason "stop"), the manifest finalized (its
// end_wall_ns set), exit status 0. Killed by the default action instead,
// it ends with signal 1, its guardians kill the members and the manifest
// is never finalized. Row "nohup": a launcher started with SIGHUP ignored
// keeps supervising after one: alpha's crash after it is restarted, and
// SIGTERM then stops the session.
TEST(ClosedOutput, HangupStopsTheSessionUnlessIgnoredAtStart) {
  struct Row {
    const char* name;
    bool ignored;
  };
  for (const Row row : {Row{"hangup", false}, Row{"nohup", true}}) {
    SCOPED_TRACE(row.name);
    Fixture fixture{row.name};
    const std::string output_path = TempPath(std::string{row.name} + ".out");
    const int output = ::open(output_path.c_str(),
                              O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    ASSERT_GE(output, 0);
    const Forked launcher =
        ForkLauncher(fixture.options, output, -1, row.ignored);
    ::close(output);
    ASSERT_GT(launcher.pid, 0);
    std::string events;
    // The members start after the launcher has installed its handlers.
    const bool started = AwaitText(launcher.events, events, "alpha 0") &&
                         AwaitText(launcher.events, events, "hardware ready");
    ::kill(launcher.pid, SIGHUP);
    bool restarted = false;
    if (row.ignored) {
      std::ofstream{fixture.crash}.close();
      restarted = AwaitText(launcher.events, events, "alpha 1");
      ::kill(launcher.pid, SIGTERM);
    }
    const bool ended = AwaitEnd(launcher.events, events);
    ::close(launcher.events);
    const int status = Reap(launcher.pid);
    const std::string manifest =
        ReadFile(fixture.session_dir + "/manifest.json");
    ::remove(output_path.c_str());

    ASSERT_TRUE(started) << "alpha never started; events: " << events;
    if (row.ignored)
      EXPECT_TRUE(restarted)
          << "no restart after an ignored SIGHUP; events: " << events;
    EXPECT_TRUE(ended) << "the session did not end";
    EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
        << "the launcher ended with " << ExitReason(status);
    EXPECT_EQ(manifest.find("\n  \"end_wall_ns\": 0,"), std::string::npos)
        << "the manifest was not finalized:\n"
        << manifest;
    std::size_t stopped = 0;
    for (std::size_t at = manifest.find("\"reason\": \"stop\"");
         at != std::string::npos;
         at = manifest.find("\"reason\": \"stop\"", at + 1))
      ++stopped;
    // The hardware stub's run and alpha's last.
    EXPECT_EQ(stopped, 2u) << manifest;
  }
}

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) return {};
  return runfiles->Rlocation(location);
}

// REQ-OPS-33
// Ignored signals stay ignored across exec, and the signal mask passes
// through it. Each row is a way a launcher comes to hold a disposition or
// mask its programs must not get: it ignores SIGPIPE itself; `nohup` ignores
// SIGHUP; `&` from a non-interactive shell ignores SIGINT and SIGQUIT (a
// node that ignores SIGINT no longer stops when the session does); and a
// caller's blocked signals. With all of them set on the spawning thread, a
// spawned program must report no signal ignored and none blocked.
TEST(ClosedOutput, SpawnedChildStartsWithDefaultSignals) {
  const std::string probe = Runfile("TALOS_SIGNAL_PROBE");
  ASSERT_FALSE(probe.empty()) << "TALOS_SIGNAL_PROBE";
  struct Row {
    const char* why;
    int signal;
    bool block;  // Else ignore.
  };
  const Row rows[] = {
      {"the launcher ignores SIGPIPE", SIGPIPE, false},
      {"nohup", SIGHUP, false},
      {"& from a non-interactive shell", SIGINT, false},
      {"& from a non-interactive shell", SIGQUIT, false},
      {"a caller's blocked signal", SIGUSR1, true},
      {"a caller's blocked signal", SIGTERM, true},
  };
  struct sigaction ignore {};
  ignore.sa_handler = SIG_IGN;
  sigemptyset(&ignore.sa_mask);
  struct sigaction previous[NSIG]{};
  sigset_t blocked;
  sigset_t mask;
  sigemptyset(&blocked);
  std::string set;
  for (const Row& row : rows) {
    set += std::string{"\n  "} + row.why + ": " +
           (row.block ? "blocks " : "ignores ") + SignalText(row.signal);
    if (row.block) {
      sigaddset(&blocked, row.signal);
    } else {
      ASSERT_EQ(::sigaction(row.signal, &ignore, &previous[row.signal]), 0);
    }
  }
  ASSERT_EQ(::pthread_sigmask(SIG_BLOCK, &blocked, &mask), 0);
  std::string error;
  int reader = -1;
  const pid_t child =
      SpawnProcess({probe}, error, SpawnOutput::kStdoutPipe, &reader);
  ::pthread_sigmask(SIG_SETMASK, &mask, nullptr);
  for (const Row& row : rows) {
    if (!row.block) ::sigaction(row.signal, &previous[row.signal], nullptr);
  }
  ASSERT_GT(child, 0) << error;
  std::string report;
  ASSERT_TRUE(AwaitEnd(reader, report)) << report;
  ::close(reader);
  const int status = Reap(child);
  FinishProcessGroup(child);
  ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << ExitReason(status);
  EXPECT_EQ(report, "ignored:\nblocked:\n")
      << "spawned from a thread set up so:" << set;
}

}  // namespace
}  // namespace talos::launcher
