// The launcher's stdout and stderr closed or never read, and its terminal
// hanging up: the session runs on (or, on the hangup, stops in order), and
// what it spawns starts with default signal dispositions and an empty mask
// whatever the launcher's are.
//
// Every wait here is on an event -- a pipe's data or its end -- bounded by
// kHangMs, which only bounds a hang (docs/TESTING.md §6.3).

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif
#include <pthread.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/launcher/child_output.h"
#include "talOS/launcher/console_line.h"
#include "talOS/launcher/launcher.h"
#include "talOS/launcher/testdata/first_exec.h"

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
  return talos::launcher::testing::WriteWarmStub(path, out.str());
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
// both are ready could end a stub by the signal rather than its trap. Each
// alpha run starts with a 16 KiB line, more than a terminal holds.
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
    alpha += "head -c 16384 /dev/zero | tr '\\0' x\necho\n";
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

// Waits for `fd` to become readable or hit EOF, up to `timeout_ms`. Uses
// select() when `fd` fits in FD_SETSIZE because macOS's poll() on an
// O_NONBLOCK FIFO can miss wakeups while a large write() refills the FIFO.
bool WaitReadable(int fd, int timeout_ms) {
  for (;;) {
    if (fd >= 0 && fd < FD_SETSIZE) {
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(fd, &rfds);
      timeval tv{};
      tv.tv_sec = timeout_ms / 1000;
      tv.tv_usec = static_cast<decltype(tv.tv_usec)>(timeout_ms % 1000) * 1000;
      const int ready = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
      if (ready < 0 && errno == EINTR) continue;
      return ready > 0;
    }
    ::pollfd ready{fd, POLLIN, 0};
    const int polled = ::poll(&ready, 1, timeout_ms);
    if (polled < 0 && errno == EINTR) continue;
    return polled > 0;
  }
}

// Reads `fd` into `seen` until `want` appears in it. False when `fd` ends,
// or stays silent for kHangMs, first.
bool AwaitText(int fd, std::string& seen, std::string_view want) {
  while (seen.find(want) == std::string::npos) {
    if (!WaitReadable(fd, kHangMs)) return false;
    char buffer[4096];
    const ssize_t got = ::read(fd, buffer, sizeof buffer);
    if (got < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
      continue;
    if (got <= 0) return false;
    seen.append(buffer, static_cast<std::size_t>(got));
  }
  return true;
}

// Reads `fd` until it ends. False when it stays silent for kHangMs first.
bool AwaitEnd(int fd, std::string& seen) {
  for (;;) {
    if (!WaitReadable(fd, kHangMs)) return false;
    char buffer[4096];
    const ssize_t got = ::read(fd, buffer, sizeof buffer);
    if (got < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
      continue;
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

// Whether the session's manifest was finalized: its end_wall_ns is set.
bool Finalized(const std::string& manifest) {
  return manifest.find("\n  \"end_wall_ns\": 0,") == std::string::npos;
}

// The runs a manifest records as ended by the session's stop.
std::size_t StoppedRuns(const std::string& manifest) {
  std::size_t stopped = 0;
  for (std::size_t at = manifest.find("\"reason\": \"stop\"");
       at != std::string::npos;
       at = manifest.find("\"reason\": \"stop\"", at + 1))
    ++stopped;
  return stopped;
}

// Whether `text` has a line holding both `first` and `second`.
bool HasLine(const std::string& text, std::string_view first,
             std::string_view second) {
  std::size_t start = 0;
  while (start < text.size()) {
    std::size_t end = text.find('\n', start);
    if (end == std::string::npos) end = text.size();
    const std::string_view line{text.data() + start, end - start};
    if (line.find(first) != std::string_view::npos &&
        line.find(second) != std::string_view::npos)
      return true;
    start = end + 1;
  }
  return false;
}

// Fills `writer`'s pipe (or terminal) until a write would block, leaving it
// blocking.
void FillPipe(int writer) {
  const int flags = ::fcntl(writer, F_GETFL);
  ::fcntl(writer, F_SETFL, flags | O_NONBLOCK);
  const std::string filler(4096, 'f');
  while (::write(writer, filler.data(), filler.size()) > 0) {
  }
  ::fcntl(writer, F_SETFL, flags);
}

// REQ-OPS-33
// A session whose launcher stdout and stderr are one pipe (`launcher 2>&1 |
// tee`) that is not read. Row "closed": once alpha's output has been echoed
// through it, the test closes the pipe's only reader, so every later write
// fails with EPIPE; a launcher that dies of SIGPIPE ends the session. Row
// "full": the pipe is full before the launcher starts and nobody ever reads
// it (a paused pager, an ssh session sshd has not noticed is gone), so every
// write to it blocks; a launcher that writes its own lines there from its
// supervision thread stops supervising. In both, alpha's first run exits 1
// once it has started: alpha must be restarted, its second run's output
// must reach alpha.log, SIGTERM must still stop the session in order
// (status 0, manifest finalized, the hardware stub's and alpha's last runs
// "stop"), and the session's talos.log must hold the launcher's line about
// alpha's exit. Row "terminal": stdout and stderr are a pseudo-terminal
// nobody reads (an interactive ssh session whose client's network dropped),
// full but for a few bytes: poll() calls it writable, and a blocking write
// of more than that room never returns. The stop must still end the
// launcher within the descriptors' linger.
TEST(ClosedOutput, SessionOutlivesAnUnreadStdout) {
  enum class Output { kClosed, kFull, kTerminal };
  struct Row {
    const char* name;
    Output output;
  };
  for (const Row row :
       {Row{"closed", Output::kClosed}, Row{"full", Output::kFull},
        Row{"terminal", Output::kTerminal}}) {
    SCOPED_TRACE(row.name);
    Fixture fixture{row.name};
    const std::string alpha_log = NodeOutputPath(fixture.session_dir, "alpha");
    // output[0] the test's end, output[1] the launcher's.
    int output[2];
    if (row.output == Output::kTerminal) {
      ASSERT_EQ(::openpty(&output[0], &output[1], nullptr, nullptr, nullptr),
                0);
      FillPipe(output[1]);
      // One read, once: some room, so that poll() calls the terminal
      // writable, but less than the launcher's output will need.
      char taken[4096];
      ASSERT_GT(::read(output[0], taken, sizeof taken), 0);
    } else {
      ASSERT_EQ(::pipe(output), 0);
      if (row.output == Output::kFull) FillPipe(output[1]);
    }
    const Forked launcher = ForkLauncher(fixture.options, output[1], output[0]);
    ::close(output[1]);
    ASSERT_GT(launcher.pid, 0);

    std::string seen;
    std::string events;
    bool started = false;
    if (row.output != Output::kClosed) {
      started = AwaitText(launcher.events, events, "alpha 0") &&
                AwaitText(launcher.events, events, "hardware ready");
    } else {
      // Read the launcher's output until alpha's echo shows up in it.
      started = AwaitText(output[0], seen, "tick");
      ::close(output[0]);
    }
    std::ofstream{fixture.crash}.close();
    // Each wait that fails costs kHangMs: the later ones are skipped.
    const bool restarted =
        started && AwaitText(launcher.events, events, "alpha 1 ticked");
    int status = 0;
    const bool alive = ::waitpid(launcher.pid, &status, WNOHANG) == 0;
    if (alive) ::kill(launcher.pid, SIGTERM);
    const bool ended = AwaitEnd(launcher.events, events);
    ::close(launcher.events);
    if (!ended) ::kill(launcher.pid, SIGKILL);
    if (alive) status = Reap(launcher.pid);
    if (row.output != Output::kClosed) ::close(output[0]);

    ASSERT_TRUE(started) << (row.output != Output::kClosed
                                 ? "alpha never started; events: "
                                 : "alpha's output never reached the "
                                   "launcher's stdout: ")
                         << (row.output != Output::kClosed ? events : seen);
    EXPECT_TRUE(alive) << "the launcher ended: " << ExitReason(status);
    EXPECT_TRUE(restarted) << "alpha's second run never printed twenty "
                              "lines; events: "
                           << events;
    EXPECT_TRUE(ended) << "the launcher did not exit after SIGTERM";
    EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
        << "the stop ended with " << ExitReason(status);
    const std::string manifest =
        ReadFile(fixture.session_dir + "/manifest.json");
    EXPECT_TRUE(Finalized(manifest)) << manifest;
    EXPECT_EQ(StoppedRuns(manifest), 2u) << manifest;
    const std::string log = ReadFile(alpha_log);
    const std::size_t second_run = log.find("[launcher] alpha run 1,");
    ASSERT_NE(second_run, std::string::npos) << alpha_log << ":\n" << log;
    std::size_t ticks = 0;
    for (std::size_t at = log.find("tick", second_run); at != std::string::npos;
         at = log.find("tick", at + 1))
      ++ticks;
    EXPECT_GE(ticks, 20u) << alpha_log << ":\n" << log;
    const std::string launcher_log =
        ReadFile(fixture.session_dir + "/" + std::string{kLauncherOutputFile});
    EXPECT_TRUE(HasLine(launcher_log, "alpha", "exit code 1")) << "talos.log:\n"
                                                               << launcher_log;
  }
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
    if (row.ignored) {
      EXPECT_TRUE(restarted)
          << "no restart after an ignored SIGHUP; events: " << events;
    }
    EXPECT_TRUE(ended) << "the session did not end";
    EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
        << "the launcher ended with " << ExitReason(status);
    EXPECT_TRUE(Finalized(manifest)) << "the manifest was not finalized:\n"
                                     << manifest;
    // The hardware stub's run and alpha's last.
    EXPECT_EQ(StoppedRuns(manifest), 2u) << manifest;
  }
}

// REQ-OPS-33
// A roster member's name is its output and event log file names. One named
// `talos` would drain its output into talos.log, the launcher's own, and so
// would `Talos` on a case-insensitive file system (macOS's default APFS).
// The graph lint refuses both (a reserved owner; a segment of lowercase
// letters), but --allow-graph-errors overrides the lint; the launch must
// still be refused, before any session directory or member exists. Row
// "gamma", the same roster under a legal name, launches and runs its member.
TEST(ClosedOutput, MisnamedMemberIsRefusedEvenAllowingGraphErrors) {
  struct Row {
    const char* name;
    bool refused;
  };
  for (const Row row :
       {Row{"talos", true}, Row{"Talos", true}, Row{"gamma", false}}) {
    SCOPED_TRACE(row.name);
    const std::string name{row.name};
    LauncherOptions options;
    options.session_id = GenerateSessionId();
    options.output_dir = TempPath(name + "_member_session");
    options.allow_graph_errors = true;
    options.simulation = true;  // The flag requires --sim.
    options.config_path = TempPath(name + "_member") + ".toml";
    std::ofstream{options.config_path}
        << "[robot]\nname = \"member_fixture\"\nperiod_us = 5000\n\n"
        << "[subsystems." << name << "]\nnode = \"//example/" << name
        << ":node\"\nperiod_us = 5000\n\n[subsystems." << name
        << ".motors.m1]\ntype = \"TalonFX\"\nbus = \"rio\"\ncan_id = 1\n";
    const std::string ran = TempPath(name + "_member_ran");
    ::remove(ran.c_str());
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStub(name + "_member_hardware", "exit 0\n", "hardware_node");
    options.binary_overrides["//example/" + name + ":node"] =
        WriteStub(name + "_member", ": > " + ran + "\nexit 0\n", name);
    const int status = Launcher{options}.Run();
    const bool started = std::filesystem::exists(ran);
    const bool session = std::filesystem::exists(
        options.output_dir + "/" + std::to_string(options.session_id));
    std::filesystem::remove_all(options.output_dir);
    ::remove(ran.c_str());
    EXPECT_EQ(status, row.refused ? 1 : 0);
    EXPECT_EQ(started, !row.refused);
    EXPECT_EQ(session, !row.refused);
  }
}

// REQ-OPS-33
// Output the launcher's stdout drops mid-session, because its reader fell
// behind, is noted in talos.log as well as in the stream: the stream's note
// may never be read, and talos.log is what session_report reads. A forked
// launcher-to-be echoes 2 MiB, twice what its stdout's buffer holds, into a
// pipe the test does not read yet; the test then reads until the stream's
// note of the drop arrives, and only then lets it stop. talos.log must note
// the same count, in the line session_report parses.
TEST(ClosedOutput, StdoutDropsMidSessionAreNotedInTalosLog) {
  const std::string log = TempPath("dropped_stdout.log");
  ::remove(log.c_str());
  int output[2];
  int echoed[2];
  int stop[2];
  ASSERT_EQ(::pipe(output), 0);
  ASSERT_EQ(::pipe(echoed), 0);
  ASSERT_EQ(::pipe(stop), 0);
  std::fflush(nullptr);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(output[0]);
    ::close(echoed[0]);
    ::close(stop[1]);
    const int null = ::open("/dev/null", O_WRONLY);
    if (null < 0 || ::dup2(output[1], STDOUT_FILENO) < 0 ||
        ::dup2(null, STDERR_FILENO) < 0)
      ::_exit(100);
    ::close(output[1]);
    {
      LauncherOutput launcher;
      launcher.Record(log);
      launcher.Echo(std::string(std::size_t{2} << 20, 'e'));
      // Waits for the test to close its end.
      char go = 0;
      if (::write(echoed[1], "e", 1) != 1 || ::read(stop[0], &go, 1) != 0)
        ::_exit(101);
      launcher.Stop(std::chrono::milliseconds{kHangMs});
    }
    ::_exit(0);
  }
  ::close(output[1]);
  ::close(echoed[1]);
  ::close(stop[0]);
  std::string ready;
  const bool echoed_all = AwaitText(echoed[0], ready, "e");
  std::string seen;
  const bool noted = echoed_all && AwaitText(output[0], seen, " here: ");
  ::close(stop[1]);  // Lets the child stop.
  AwaitEnd(output[0], seen);
  ::close(output[0]);
  ::close(echoed[0]);
  const int status = Reap(child);
  const std::string talos_log = ReadFile(log);
  ::remove(log.c_str());

  ASSERT_TRUE(noted) << "the stream never noted a drop";
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << ExitReason(status);
  const std::regex stream_note{
      R"(\[launcher\] (\d+) bytes of output dropped here: )"};
  std::smatch in_stream;
  ASSERT_TRUE(std::regex_search(seen, in_stream, stream_note));
  // session_report.py's pattern for a drop on the launcher's own stdout.
  const std::regex log_note{
      R"((^|\n)\[launcher\] (\d+) bytes for the launcher's stdout were not written)"};
  std::smatch in_log;
  ASSERT_TRUE(std::regex_search(talos_log, in_log, log_note)) << "talos.log:\n"
                                                              << talos_log;
  EXPECT_EQ(in_log[2].str(), in_stream[1].str()) << talos_log;
}

// REQ-OPS-33
// talos.log is read by line (session_report, a person grepping for a
// member's exit), so the launcher's note about one of its streams never
// lands inside another line. A forked launcher-to-be prints the first piece
// of a line ("[launcher] alpha") to a stdout whose reader is gone, waits
// until the stream's failure has been noted (on stderr, which the test
// reads), then prints the rest. talos.log must hold the whole line, then
// the note on a line of its own.
TEST(ClosedOutput, StreamNoteWaitsForTheLineInProgress) {
  const std::string log = TempPath("note_line.log");
  ::remove(log.c_str());
  int errors[2];
  int resume[2];
  ASSERT_EQ(::pipe(errors), 0);
  ASSERT_EQ(::pipe(resume), 0);
  std::fflush(nullptr);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(errors[0]);
    ::close(resume[1]);
    int closed[2];
    if (::pipe(closed) != 0) ::_exit(100);
    ::close(closed[0]);             // Every write to stdout fails with EPIPE.
    std::signal(SIGPIPE, SIG_IGN);  // As Launcher::Run sets it.
    if (::dup2(closed[1], STDOUT_FILENO) < 0 ||
        ::dup2(errors[1], STDERR_FILENO) < 0)
      ::_exit(101);
    ::close(closed[1]);
    ::close(errors[1]);
    {
      LauncherOutput launcher;
      launcher.Record(log);
      std::cout << "[launcher] alpha";
      // Until the test has seen the failure noted and closes its end.
      char go = 0;
      if (::read(resume[0], &go, 1) != 0) ::_exit(102);
      std::cout << " exited (exit code 1)\n";
      launcher.Stop(std::chrono::milliseconds{kHangMs});
    }
    ::_exit(0);
  }
  ::close(errors[1]);
  ::close(resume[0]);
  std::string seen;
  const bool noted = AwaitText(errors[0], seen, "no longer writes to it");
  ::close(resume[1]);
  AwaitEnd(errors[0], seen);
  ::close(errors[0]);
  const int status = Reap(child);
  const std::string talos_log = ReadFile(log);
  ::remove(log.c_str());

  ASSERT_TRUE(noted) << "the stream's failure was never noted: " << seen;
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << ExitReason(status);
  const std::string line = "[launcher] alpha exited (exit code 1)\n";
  const std::size_t whole = talos_log.find(line);
  ASSERT_NE(whole, std::string::npos) << "talos.log:\n" << talos_log;
  const std::size_t note = talos_log.find("[launcher] stdout: ");
  ASSERT_NE(note, std::string::npos) << "talos.log:\n" << talos_log;
  EXPECT_EQ(note, whole + line.size()) << "talos.log:\n" << talos_log;
}

// REQ-OPS-33
// A member's stdio to its pipe is fully buffered, so one line can reach the
// launcher in two reads. The echo holds the first part, and the launcher's
// stdout shows a launcher line printed between the reads before the member's
// line, whole. A forked launcher-to-be drains two member pipes the test
// writes, `a` and `b`, echoing both. Steps: `a` "abc"; `b` "one\n", which
// the reader reads after "abc" unless poll() scans `a` just before "abc"
// arrives; the launcher prints its line; `a` "def\n" and then more than
// kChildOutputReadBytes of a line without a newline, which must show, ended
// by a newline, before its end once the held part exceeds that bound; `a`
// "tail" and its end, which must show as "tail\n": a partial line forwarded
// is a console line of its own, so the launcher's next line (an "exited")
// never joins it. Before the line hold the stdout read
// "abcone\n[launcher] between\ndef\n...". `a`'s log takes every byte, as
// before, and no newline the member did not write.
TEST(ClosedOutput, EchoForwardsAMembersLinesWhole) {
  const std::string a_log = TempPath("echo_a.log");
  const std::string b_log = TempPath("echo_b.log");
  ::remove(a_log.c_str());
  ::remove(b_log.c_str());
  int output[2];
  int a[2];
  int b[2];
  int go[2];
  ASSERT_EQ(::pipe(output), 0);
  ASSERT_EQ(::pipe(a), 0);
  ASSERT_EQ(::pipe(b), 0);
  ASSERT_EQ(::pipe(go), 0);
  std::fflush(nullptr);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(output[0]);
    ::close(a[1]);
    ::close(b[1]);
    ::close(go[1]);
    const int null = ::open("/dev/null", O_WRONLY);
    if (null < 0 || ::dup2(output[1], STDOUT_FILENO) < 0 ||
        ::dup2(null, STDERR_FILENO) < 0)
      ::_exit(100);
    ::close(output[1]);
    {
      LauncherOutput launcher;
      ChildOutput members{&launcher};
      members.Add(a[0], a_log, "a", 1, /*echo=*/true);
      members.Add(b[0], b_log, "b", 2, /*echo=*/true);
      char byte = 0;
      if (::read(go[0], &byte, 1) != 1) ::_exit(101);
      std::cout << "[launcher] between\n";
      // Until the test has closed every pipe.
      if (::read(go[0], &byte, 1) != 0) ::_exit(102);
      members.Stop(std::chrono::milliseconds{kHangMs});
      launcher.Stop(std::chrono::milliseconds{kHangMs});
    }
    ::_exit(0);
  }
  ::close(output[1]);
  ::close(a[0]);
  ::close(b[0]);
  ::close(go[0]);
  const auto put = [](int fd, std::string_view bytes) {
    return ::write(fd, bytes.data(), bytes.size()) ==
           static_cast<ssize_t>(bytes.size());
  };
  const std::string unended(kChildOutputReadBytes + 1, 'x');
  const std::string lines = "one\n[launcher] between\nabcdef\n";
  std::string seen;
  bool steps = put(a[1], "abc") && put(b[1], "one\n") &&
               AwaitText(output[0], seen, "one\n") && put(go[1], "g") &&
               AwaitText(output[0], seen, "between\n") && put(a[1], "def\n") &&
               put(a[1], unended) && AwaitText(output[0], seen, unended + "\n");
  steps = steps && put(a[1], "tail");
  ::close(a[1]);
  ::close(b[1]);
  ::close(go[1]);
  const bool ended = AwaitEnd(output[0], seen);
  ::close(output[0]);
  const int status = Reap(child);
  const std::string a_written = ReadFile(a_log);
  ::remove(a_log.c_str());
  ::remove(b_log.c_str());

  // Without the bound, the unended line never shows before the end.
  ASSERT_TRUE(steps) << "stdout so far:\n" << seen.substr(0, 200);
  EXPECT_TRUE(ended);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << ExitReason(status);
  // A member's line split on the console, or its end's remainder lost.
  EXPECT_EQ(seen.substr(0, lines.size()), lines);
  EXPECT_TRUE(seen == lines + unended + "\ntail\n")
      << "stdout ends: "
      << seen.substr(seen.size() - std::min<std::size_t>(seen.size(), 16));
  EXPECT_TRUE(a_written ==
              "[launcher] a run 0, pid 1\nabcdef\n" + unended + "tail")
      << a_log << " starts: " << a_written.substr(0, 48);
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

// REQ-OPS-06
// A member whose --describe says holds_stop is spawned with exactly the stop
// signals blocked (SIGINT, SIGTERM, SIGHUP), so a stop sent while its runtime
// starts stays pending; nothing else is, even signals the spawning thread
// blocks itself (SIGCHLD, SIGPIPE), or the node would never see a child exit
// or a broken pipe.
TEST(ClosedOutput, HeldStopSpawnBlocksExactlyTheStopSignals) {
  const std::string probe = Runfile("TALOS_SIGNAL_PROBE");
  ASSERT_FALSE(probe.empty()) << "TALOS_SIGNAL_PROBE";
  sigset_t blocked;
  sigset_t mask;
  sigemptyset(&blocked);
  sigaddset(&blocked, SIGCHLD);
  sigaddset(&blocked, SIGPIPE);
  ASSERT_EQ(::pthread_sigmask(SIG_BLOCK, &blocked, &mask), 0);
  std::string error;
  int reader = -1;
  const pid_t child =
      SpawnProcess({probe}, error, SpawnOutput::kStdoutPipe, &reader, nullptr,
                   /*hold_stop=*/true);
  ::pthread_sigmask(SIG_SETMASK, &mask, nullptr);
  ASSERT_GT(child, 0) << error;
  std::string report;
  ASSERT_TRUE(AwaitEnd(reader, report)) << report;
  ::close(reader);
  const int status = Reap(child);
  FinishProcessGroup(child);
  ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << ExitReason(status);
  int held[] = {SIGINT, SIGTERM, SIGHUP};
  std::sort(std::begin(held), std::end(held));
  std::string expected = "ignored:\nblocked:";
  for (const int signal : held) expected += " " + std::to_string(signal);
  expected += "\n";
  EXPECT_EQ(report, expected);
}

// Writes all bytes to `fd`, retrying EINTR.
bool WriteAllFd(int fd, std::string_view bytes) {
  while (!bytes.empty()) {
    const ssize_t n = ::write(fd, bytes.data(), bytes.size());
    if (n < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    bytes.remove_prefix(static_cast<std::size_t>(n));
  }
  return true;
}

// Reads `fd` into `seen` until `seen.size() >= want_bytes`, EOF, or kHangMs.
bool AwaitSize(int fd, std::string& seen, std::size_t want_bytes) {
  while (seen.size() < want_bytes) {
    if (!WaitReadable(fd, kHangMs)) return false;
    char buffer[4096];
    const ssize_t got = ::read(fd, buffer, sizeof buffer);
    if (got < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
      continue;
    if (got <= 0) return false;
    seen.append(buffer, static_cast<std::size_t>(got));
  }
  return true;
}

// REQ-OPS-10
// Every member's stdout and stderr both land in `<session>/<name>.log`.
TEST(ClosedOutput, MemberStdoutAndStderrBothLandInNodeLog) {
  LauncherOptions options;
  options.session_id = GenerateSessionId();
  options.output_dir = TempPath("stdout_stderr_session");
  options.config_path = TempPath("stdout_stderr") + ".toml";
  options.max_restarts = 0;
  options.heartbeat_startup_grace_ms = 60000;
  std::filesystem::remove_all(options.output_dir);
  std::ofstream{options.config_path} << kFixtureToml;
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("stdout_stderr_hw", "exit 0\n", "hardware_node");
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "stdout_stderr_alpha",
      "echo 'alpha_stdout_payload'\necho 'alpha_stderr_payload' >&2\nexit 0\n",
      "alpha");

  const int status = Launcher{options}.Run();
  const std::string session_dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  const std::string alpha_log = ReadFile(NodeOutputPath(session_dir, "alpha"));
  std::filesystem::remove_all(options.output_dir);
  ::remove(options.config_path.c_str());

  EXPECT_EQ(status, 0);
  EXPECT_NE(alpha_log.find("alpha_stdout_payload\n"), std::string::npos)
      << alpha_log;
  EXPECT_NE(alpha_log.find("alpha_stderr_payload\n"), std::string::npos)
      << alpha_log;
}

// REQ-OPS-10
// When a member's log writer stalls (here writing to an unread FIFO),
// ChildOutput keeps draining the member's pipe into its bounded memory buffer
// without blocking the producer, counts drops past the buffer limit, and writes
// the drop marker once the writer catches up.
TEST(ClosedOutput, ChildOutputDropsAndMarksWhenLogWriterStalls) {
  const std::string fifo_path = TempPath("stalled_writer.fifo");
  ::remove(fifo_path.c_str());
  ASSERT_EQ(::mkfifo(fifo_path.c_str(), 0644), 0);
  const int fifo_rd =
      ::open(fifo_path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  ASSERT_GE(fifo_rd, 0);

  int in[2];
  ASSERT_EQ(::pipe(in), 0);
  std::string drained;
  uint64_t dropped_while_stalled = 0;
  uint64_t dropped_total = 0;
  {
    ChildOutput output{nullptr};
    output.Add(in[0], fifo_path, "alpha", 1, /*echo=*/false);
    ASSERT_TRUE(WriteAllFd(in[1], "writer_open\n"));
    ASSERT_TRUE(AwaitText(fifo_rd, drained, "writer_open\n"));

    // Stop reading the FIFO so WriteLoop blocks inside write(), then push
    // 4 MiB (> 2 MiB, far past kChildOutputBufferBytes = 1 MiB) through the
    // child pipe without blocking.
    const std::string payload(std::size_t{4} << 20, 's');
    ASSERT_TRUE(WriteAllFd(in[1], payload));
    ::close(in[1]);
    dropped_while_stalled = output.Dropped(fifo_path);

    // Unblock WriteLoop by draining the FIFO until EOF while Stop finishes.
    std::jthread drainer{[&] { AwaitEnd(fifo_rd, drained); }};
    output.Stop(std::chrono::milliseconds{kHangMs});
    drainer.join();
    dropped_total = output.Dropped(fifo_path);
  }
  ::close(fifo_rd);
  ::remove(fifo_path.c_str());

  EXPECT_GT(dropped_while_stalled, 0u);
  EXPECT_GE(dropped_total, dropped_while_stalled);
  EXPECT_NE(drained.find(
                " bytes of output dropped here: the launcher's log writer fell "
                "behind\n"),
            std::string::npos);
}

// REQ-OPS-10
// A member's output log is capped at kChildOutputLogMaxBytes (64 MiB) per
// session: once reached, ChildOutput writes the cap notice, drops all later
// output, counts the dropped bytes, and stops growing the file.
TEST(ClosedOutput, ChildOutputCapsLogAtPerSessionMaxBytes) {
  // Alphabetical order: a_log < z_fifo, so WriteLoop flushes a_log before
  // writing each fence token to z_fifo.
  const std::string a_log = TempPath("cap_a.log");
  const std::string z_fifo = TempPath("cap_z.fifo");
  ::remove(a_log.c_str());
  ::remove(z_fifo.c_str());
  ASSERT_EQ(::mkfifo(z_fifo.c_str(), 0644), 0);
  const int z_rd = ::open(z_fifo.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  ASSERT_GE(z_rd, 0);

  int a_pipe[2];
  int z_pipe[2];
  ASSERT_EQ(::pipe(a_pipe), 0);
  ASSERT_EQ(::pipe(z_pipe), 0);

  const std::string cap_note = "\n[launcher] output log reached " +
                               std::to_string(kChildOutputLogMaxBytes) +
                               " bytes; later output is dropped\n";
  uint64_t capped_size = 0;
  uint64_t dropped_at_cap = 0;
  uint64_t dropped_after_extra = 0;
  std::string fence_seen;
  {
    ChildOutput output{nullptr};
    output.Add(a_pipe[0], a_log, "alpha", 1, /*echo=*/false);
    output.Add(z_pipe[0], z_fifo, "fence", 2, /*echo=*/false);

    // 256 KiB per step (< kChildOutputBufferBytes = 1 MiB) so buffer pressure
    // never drops before disk writes reach the 64 MiB cap; 257 steps = 64.25
    // MiB.
    const std::string chunk(std::size_t{256} << 10, 'c');
    for (int i = 0; i < 257; ++i) {
      ASSERT_TRUE(WriteAllFd(a_pipe[1], chunk));
      const std::string tag = "f" + std::to_string(i) + "\n";
      ASSERT_TRUE(WriteAllFd(z_pipe[1], tag));
      ASSERT_TRUE(AwaitText(z_rd, fence_seen, tag));
      fence_seen.clear();
    }
    capped_size = std::filesystem::file_size(a_log);
    dropped_at_cap = output.Dropped(a_log);

    // Write another 64 KiB after capping: file must not grow and every byte
    // must be counted as dropped.
    const std::string extra(std::size_t{64} << 10, 'd');
    ASSERT_TRUE(WriteAllFd(a_pipe[1], extra));
    ASSERT_TRUE(WriteAllFd(z_pipe[1], "extra\n"));
    ASSERT_TRUE(AwaitText(z_rd, fence_seen, "extra\n"));

    ::close(a_pipe[1]);
    ::close(z_pipe[1]);
    std::jthread drainer{[&] { AwaitEnd(z_rd, fence_seen); }};
    output.Stop(std::chrono::milliseconds{kHangMs});
    drainer.join();
    dropped_after_extra = output.Dropped(a_log);
  }
  ::close(z_rd);
  ::remove(z_fifo.c_str());

  const uint64_t final_size = std::filesystem::file_size(a_log);
  std::string tail(cap_note.size(), '\0');
  {
    std::ifstream in{a_log, std::ios::binary};
    ASSERT_TRUE(in.good());
    in.seekg(static_cast<std::streamoff>(final_size - cap_note.size()));
    in.read(tail.data(), static_cast<std::streamsize>(tail.size()));
  }
  ::remove(a_log.c_str());

  EXPECT_EQ(final_size, capped_size);
  EXPECT_LE(final_size, kChildOutputLogMaxBytes + cap_note.size());
  EXPECT_GT(dropped_at_cap, 0u);
  EXPECT_GE(dropped_after_extra, dropped_at_cap + (std::size_t{64} << 10));
  EXPECT_EQ(tail, cap_note);
}

// REQ-OPS-33
// Distinct stdout (actively drained) and stderr (closed or full unread pipe):
// LauncherOutput keeps separate descriptor queues for different inodes, so an
// unread or broken stderr never stalls stdout echo or supervision, alpha still
// restarts and stops cleanly, and talos.log keeps every launcher line.
TEST(ClosedOutput, SessionOutlivesAnUnreadSeparateStderr) {
  struct Row {
    const char* name;
    bool closed;
  };
  for (const Row row :
       {Row{"stderr_closed", true}, Row{"stderr_full", false}}) {
    SCOPED_TRACE(row.name);
    Fixture fixture{row.name};
    // Ensure the launcher writes to std::cerr during the live session so the
    // separate closed/full stderr descriptor is actually exercised.
    fixture.options.member_spawned = [](const std::string& name) {
      ConsoleLine(std::cerr) << "[launcher] stderr probe for " << name << "\n";
    };
    const std::string alpha_log = NodeOutputPath(fixture.session_dir, "alpha");
    int out_pipe[2];
    int err_pipe[2];
    int events[2];
    ASSERT_EQ(::pipe(out_pipe), 0);
    ASSERT_EQ(::pipe(err_pipe), 0);
    ASSERT_EQ(::pipe(events), 0);
    if (!row.closed) FillPipe(err_pipe[1]);

    std::fflush(nullptr);
    const pid_t pid = ::fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      ::close(out_pipe[0]);
      ::close(err_pipe[0]);
      ::close(events[0]);
      if (::dup2(out_pipe[1], STDOUT_FILENO) < 0 ||
          ::dup2(err_pipe[1], STDERR_FILENO) < 0 ||
          ::dup2(events[1], kEventFd) < 0)
        ::_exit(100);
      ::close(out_pipe[1]);
      ::close(err_pipe[1]);
      ::close(events[1]);
      std::signal(SIGPIPE, SIG_DFL);
      std::signal(SIGHUP, SIG_DFL);
      const int code = Launcher{fixture.options}.Run();
      std::cout.flush();
      std::fflush(nullptr);
      ::_exit(code);
    }
    ::close(out_pipe[1]);
    ::close(err_pipe[1]);
    ::close(events[1]);
    if (row.closed) ::close(err_pipe[0]);

    // Actively drain stdout in parallel so stdout stays healthy while stderr is
    // closed or full.
    std::string stdout_seen;
    std::jthread out_drainer{[&] { AwaitEnd(out_pipe[0], stdout_seen); }};

    std::string ev;
    const bool started = AwaitText(events[0], ev, "alpha 0") &&
                         AwaitText(events[0], ev, "hardware ready");
    std::ofstream{fixture.crash}.close();
    const bool restarted =
        started && AwaitText(events[0], ev, "alpha 1 ticked");
    int status = 0;
    const bool alive = ::waitpid(pid, &status, WNOHANG) == 0;
    if (alive) ::kill(pid, SIGTERM);
    const bool ended = AwaitEnd(events[0], ev);
    ::close(events[0]);
    if (!ended) ::kill(pid, SIGKILL);
    if (alive) status = Reap(pid);
    out_drainer.join();
    ::close(out_pipe[0]);
    if (!row.closed) ::close(err_pipe[0]);

    ASSERT_TRUE(started) << ev;
    EXPECT_TRUE(alive) << ExitReason(status);
    EXPECT_TRUE(restarted) << ev;
    EXPECT_TRUE(ended);
    EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
        << ExitReason(status);
    EXPECT_NE(stdout_seen.find("tick\n"), std::string::npos)
        << "drained stdout missed member echo";
    const std::string manifest =
        ReadFile(fixture.session_dir + "/manifest.json");
    EXPECT_TRUE(Finalized(manifest)) << manifest;
    EXPECT_EQ(StoppedRuns(manifest), 2u) << manifest;
    const std::string log = ReadFile(alpha_log);
    EXPECT_NE(log.find("[launcher] alpha run 1,"), std::string::npos) << log;
    const std::string launcher_log =
        ReadFile(fixture.session_dir + "/" + std::string{kLauncherOutputFile});
    EXPECT_TRUE(HasLine(launcher_log, "alpha", "exit code 1")) << launcher_log;
    EXPECT_TRUE(HasLine(launcher_log, "stderr probe for", "alpha"))
        << launcher_log;
    if (row.closed) {
      EXPECT_TRUE(HasLine(launcher_log, "stderr:", "no longer writes to it"))
          << launcher_log;
    } else {
      EXPECT_TRUE(HasLine(launcher_log, "bytes for the launcher's stderr",
                          "were not written"))
          << launcher_log;
    }
  }
}

// REQ-OPS-33
// Concurrent multi-piece ConsoleLine(std::cout) writes and LauncherOutput::Echo
// writes from separate threads enter stdout whole with zero spliced lines.
TEST(ClosedOutput, ConcurrentConsoleLineAndEchoNeverSpliceLines) {
  int output[2];
  ASSERT_EQ(::pipe(output), 0);
  constexpr int kLinesPerWriter = 200;
  std::fflush(nullptr);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(output[0]);
    const int null = ::open("/dev/null", O_WRONLY);
    if (null < 0 || ::dup2(output[1], STDOUT_FILENO) < 0 ||
        ::dup2(null, STDERR_FILENO) < 0)
      ::_exit(100);
    ::close(output[1]);
    ::close(null);
    {
      LauncherOutput launcher;
      std::thread launcher_writer{[&] {
        for (int i = 0; i < kLinesPerWriter; ++i) {
          ConsoleLine(std::cout) << "[launcher] seq=" << i << " "
                                 << std::string(32, 'L') << " end\n";
        }
      }};
      std::thread echo_writer{[&] {
        for (int i = 0; i < kLinesPerWriter; ++i) {
          launcher.Echo("[node] seq=" + std::to_string(i) + " " +
                        std::string(32, 'E') + " end\n");
        }
      }};
      launcher_writer.join();
      echo_writer.join();
      launcher.Stop(std::chrono::milliseconds{kHangMs});
    }
    ::_exit(0);
  }
  ::close(output[1]);
  std::string seen;
  const bool ended = AwaitEnd(output[0], seen);
  ::close(output[0]);
  const int status = Reap(child);

  ASSERT_TRUE(ended);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << ExitReason(status);

  const std::regex launcher_re{R"(^\[launcher\] seq=\d+ L{32} end$)"};
  const std::regex echo_re{R"(^\[node\] seq=\d+ E{32} end$)"};
  std::istringstream in{seen};
  std::string line;
  int launcher_count = 0;
  int echo_count = 0;
  while (std::getline(in, line)) {
    if (std::regex_match(line, launcher_re)) {
      ++launcher_count;
    } else if (std::regex_match(line, echo_re)) {
      ++echo_count;
    } else {
      ADD_FAILURE() << "spliced line: " << line;
    }
  }
  EXPECT_EQ(launcher_count, kLinesPerWriter);
  EXPECT_EQ(echo_count, kLinesPerWriter);
}

// REQ-OPS-33
// Exact partial-line echo threshold: 65,536 bytes (kChildOutputReadBytes)
// without a newline stay held while launcher output passes ahead of them, and
// the 65,537th byte immediately forwards the line (with a console newline)
// while the member's log keeps the exact bytes with no synthetic newline.
TEST(ClosedOutput, EchoHoldsExact64KiBAndForwardsAt64KiBPlusOne) {
  static_assert(kChildOutputReadBytes == 65536u);
  const std::string a_fifo = TempPath("exact64k_a.fifo");
  ::remove(a_fifo.c_str());
  ASSERT_EQ(::mkfifo(a_fifo.c_str(), 0644), 0);
  const int a_log_rd =
      ::open(a_fifo.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  ASSERT_GE(a_log_rd, 0);

  int output[2];
  int a[2];
  int go[2];
  ASSERT_EQ(::pipe(output), 0);
  ASSERT_EQ(::pipe(a), 0);
  ASSERT_EQ(::pipe(go), 0);
  std::fflush(nullptr);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(a_log_rd);
    ::close(output[0]);
    ::close(a[1]);
    ::close(go[1]);
    const int null = ::open("/dev/null", O_WRONLY);
    if (null < 0 || ::dup2(output[1], STDOUT_FILENO) < 0 ||
        ::dup2(null, STDERR_FILENO) < 0)
      ::_exit(100);
    ::close(output[1]);
    ::close(null);
    {
      LauncherOutput launcher;
      ChildOutput members{&launcher};
      members.Add(a[0], a_fifo, "a", 1, /*echo=*/true);
      char byte = 0;
      if (::read(go[0], &byte, 1) != 1) ::_exit(101);
      ConsoleLine(std::cout) << "[launcher] at_64k\n";
      if (::read(go[0], &byte, 1) != 0) ::_exit(102);
      members.Stop(std::chrono::milliseconds{kHangMs});
      launcher.Stop(std::chrono::milliseconds{kHangMs});
    }
    ::_exit(0);
  }
  ::close(output[1]);
  ::close(a[0]);
  ::close(go[0]);

  const std::string header = "[launcher] a run 0, pid 1\n";
  const std::string held_exact(kChildOutputReadBytes, 'h');
  ASSERT_TRUE(WriteAllFd(a[1], held_exact));
  // Waiting until the log FIFO receives all 65,536 bytes proves ReadLoop has
  // already passed every byte through Deliver() into echo_held.
  std::string log_seen;
  ASSERT_TRUE(AwaitSize(a_log_rd, log_seen, header.size() + held_exact.size()));

  // Ask the child to print its launcher line: because 65,536 B is still held,
  // stdout must show ONLY the launcher line first.
  ASSERT_TRUE(WriteAllFd(go[1], "g"));
  std::string stdout_seen;
  ASSERT_TRUE(AwaitText(output[0], stdout_seen, "[launcher] at_64k\n"));
  EXPECT_EQ(stdout_seen, "[launcher] at_64k\n");

  // One more byte (65,537 total) exceeds 65,536 and forwards immediately.
  ASSERT_TRUE(WriteAllFd(a[1], "z"));
  ASSERT_TRUE(AwaitText(output[0], stdout_seen, held_exact + "z\n"));

  ::close(a[1]);
  ::close(go[1]);
  std::jthread log_drainer{[&] { AwaitEnd(a_log_rd, log_seen); }};
  const bool ended = AwaitEnd(output[0], stdout_seen);
  log_drainer.join();
  ::close(output[0]);
  ::close(a_log_rd);
  const int status = Reap(child);
  ::remove(a_fifo.c_str());

  EXPECT_TRUE(ended);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << ExitReason(status);
  EXPECT_EQ(stdout_seen, "[launcher] at_64k\n" + held_exact + "z\n");
  EXPECT_EQ(log_seen, header + held_exact + "z");
}

}  // namespace
}  // namespace talos::launcher
