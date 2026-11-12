// The launcher's stdout and stderr readers going away mid-session (an ssh
// session or a `tee` closing): the session runs on, and what it spawns does
// not inherit the launcher's indifference to SIGPIPE.

#include <gtest/gtest.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <string_view>
#include <thread>

#include "talOS/launcher/launcher.h"

namespace talos::launcher {
namespace {

using namespace std::chrono_literals;

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

uint64_t FileSize(const std::string& path) {
  std::error_code error;
  const auto size = std::filesystem::file_size(path, error);
  return error ? 0 : size;
}

std::string ReadFile(const std::string& path) {
  std::ifstream in{path};
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

// REQ-OPS-33
// A session whose launcher stdout and stderr are one pipe (`launcher 2>&1 |
// tee`): once alpha's output has been echoed through it, the test closes the
// pipe's only reader, then makes alpha's first run exit 1. The launcher's
// lines about that exit and the restart, the echo of alpha's output, and its
// lines at the stop and its exit (main's return flushes std::cout; the child
// here flushes as main would) all write into a pipe nobody reads. A launcher
// that dies of SIGPIPE ends the session, and its nodes' guardians kill them.
// It must keep supervising: alpha is restarted, its second run's output
// reaches its log, and SIGTERM still stops the session with status 0.
TEST(ClosedOutput, SessionOutlivesItsStdoutReader) {
  LauncherOptions options;
  options.session_id = GenerateSessionId();
  options.output_dir = TempPath("session");
  options.describe_timeout_ms = 4000;
  // The stubs never take a registry row; nothing here waits for one.
  options.heartbeat_startup_grace_ms = 60000;
  options.config_path = TempPath("fixture") + ".toml";
  std::ofstream{options.config_path} << kFixtureToml;
  const std::string crash = TempPath("crash");
  const std::string crashed = TempPath("crashed");
  ::remove(crash.c_str());
  ::remove(crashed.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStub(
      "hardware", "trap 'exit 0' INT TERM\nwhile :; do sleep 0.05; done\n",
      "hardware_node");
  // Prints a line every 20 ms; once `crash` exists, its first run exits 1.
  std::string alpha_body = "trap 'exit 0' INT TERM\nwhile :; do\n  echo tick\n";
  alpha_body += "  if [ -e " + crash + " ] && [ ! -e " + crashed + " ]; then\n";
  alpha_body += "    : > " + crashed + "\n    exit 1\n  fi\n";
  alpha_body += "  sleep 0.02\ndone\n";
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("alpha", alpha_body, "alpha");
  const std::string alpha_log = NodeOutputPath(
      options.output_dir + "/" + std::to_string(options.session_id), "alpha");

  int output[2];
  ASSERT_EQ(::pipe(output), 0);
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) {
    ::close(output[0]);
    if (::dup2(output[1], STDOUT_FILENO) < 0 ||
        ::dup2(output[1], STDERR_FILENO) < 0)
      ::_exit(100);
    ::close(output[1]);
    const int code = Launcher{options}.Run();
    std::cout.flush();
    std::fflush(nullptr);
    ::_exit(code);
  }
  ::close(output[1]);

  // Read the launcher's output until alpha's echo shows up in it.
  std::string seen;
  for (const auto give_up = std::chrono::steady_clock::now() + 20s;
       seen.find("tick") == std::string::npos &&
       std::chrono::steady_clock::now() < give_up;) {
    ::pollfd ready{output[0], POLLIN, 0};
    if (::poll(&ready, 1, 100) <= 0) continue;
    char buffer[4096];
    const ssize_t got = ::read(output[0], buffer, sizeof buffer);
    if (got <= 0) break;
    seen.append(buffer, static_cast<std::size_t>(got));
  }
  const bool echoed = seen.find("tick") != std::string::npos;
  ::close(output[0]);
  std::ofstream{crash}.close();

  // Alpha's second run writes twenty lines to its log while the launcher
  // lives.
  const std::string second_run = "[launcher] alpha run 1,";
  const auto second_run_output = [&]() -> std::size_t {
    const std::string log = ReadFile(alpha_log);
    const std::size_t header = log.find(second_run);
    return header == std::string::npos ? 0 : log.size() - header;
  };
  int status = 0;
  pid_t ended = 0;
  for (const auto give_up = std::chrono::steady_clock::now() + 20s;
       second_run_output() < second_run.size() + 20 * 5 &&
       std::chrono::steady_clock::now() < give_up;) {
    ended = ::waitpid(launcher, &status, WNOHANG);
    if (ended == launcher) break;
    std::this_thread::sleep_for(10ms);
  }
  const std::size_t restarted_output = second_run_output();
  if (ended != launcher) ended = ::waitpid(launcher, &status, WNOHANG);
  const bool alive = ended == 0;

  if (alive) ::kill(launcher, SIGTERM);
  if (ended != launcher) {
    while (::waitpid(launcher, &status, 0) < 0 && errno == EINTR) {
    }
  }
  ASSERT_TRUE(echoed) << "alpha's output never reached the launcher's stdout: "
                      << seen;
  EXPECT_TRUE(alive) << "the launcher ended once its stdout reader closed: "
                     << ExitReason(status);
  EXPECT_GE(restarted_output, second_run.size() + 20 * 5)
      << "alpha's restarted run left this in " << alpha_log << ":\n"
      << ReadFile(alpha_log);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "a stop after the close ended with " << ExitReason(status);
  std::filesystem::remove_all(options.output_dir);
  ::remove(crash.c_str());
  ::remove(crashed.c_str());
}

// REQ-OPS-33
// Ignored signals stay ignored across exec. A launcher that ignores SIGPIPE
// must not pass that on: a program writing to a pipe whose reader is gone
// would get EPIPE where it expects to die, and one that ignores write errors
// would run on. A child that sends itself SIGPIPE dies of it with the default
// disposition; with SIGPIPE inherited ignored it exits 3.
TEST(ClosedOutput, SpawnedChildStartsWithDefaultSigpipe) {
  const auto previous = std::signal(SIGPIPE, SIG_IGN);
  ASSERT_NE(previous, SIG_ERR);
  std::string error;
  const pid_t child =
      SpawnProcess({"/bin/sh", "-c", "kill -s PIPE $$; exit 3"}, error);
  std::signal(SIGPIPE, previous);
  ASSERT_GT(child, 0) << error;
  int status = 0;
  while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {
  }
  FinishProcessGroup(child);
  EXPECT_TRUE(WIFSIGNALED(status) && WTERMSIG(status) == SIGPIPE)
      << "the child ended with " << ExitReason(status);
}

}  // namespace
}  // namespace talos::launcher
