// Integration tier (fork/exec, SIGTERM, the live registry): a real node
// binary stopped while its loop waits on nothing. Its own target, out of
// node_test's component lane (docs/TESTING.md §2, "One target, one lane").
#include <fcntl.h>
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/introspection/registry.h"

namespace talos {
namespace {

using namespace std::chrono_literals;

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

std::string TempFile(const std::string& name) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  return std::string{tmp != nullptr ? tmp : "/tmp"} + "/" + name;
}

// A running node whose loop has no timer and no subscription waits on
// nothing but a stop, and no helper thread polls for one: SIGTERM must end
// that wait from the loop's own thread, and the node exit 0 after its run,
// long before --duration-s (60 s), the only other way out. The signal goes
// once the registry row shows the loop running (endpoints are published only
// then).
// REQ-OPS-06
// REQ-RT-21
TEST(Binary, StopEndsARunningNodeThatWaitsOnNothing) {
  const std::string binary = Runfile("TALOS_BLOCKED_NODE");
  const std::string config = Runfile("TALOS_TEST_CONFIG");
  const std::string log = TempFile("idle_node.tlog");
  int output[2];
  ASSERT_EQ(::pipe(output), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    // Its constructor reads stdin to EOF: at once, from /dev/null.
    const int null = ::open("/dev/null", O_RDONLY);
    ::dup2(null, STDIN_FILENO);
    ::dup2(output[1], STDOUT_FILENO);
    ::close(null);
    ::close(output[0]);
    ::close(output[1]);
    ::execl(binary.c_str(), binary.c_str(), "--sim", "--duration-s", "60",
            "--log", log.c_str(), "--config", config.c_str(), nullptr);
    ::_exit(127);
  }
  ::close(output[1]);
  const auto running = [child] {
    const auto reader = introspect::RegistryReader::open();
    if (!reader) return false;
    for (const auto& node : reader->nodes()) {
      if (node.pid == static_cast<std::uint64_t>(child) && node.alive) {
        return true;
      }
    }
    return false;
  };
  int status = 0;
  bool exited = false;
  bool live = running();
  const auto give_up = std::chrono::steady_clock::now() + 20s;
  while (!live && std::chrono::steady_clock::now() < give_up) {
    if (::waitpid(child, &status, WNOHANG) == child) {
      exited = true;
      break;
    }
    std::this_thread::sleep_for(1ms);  // paces the poll; the row is the event
    live = running();
  }
  const auto signalled = std::chrono::steady_clock::now();
  if (!exited) ::kill(child, live ? SIGTERM : SIGKILL);
  std::string printed;
  char buffer[256];
  for (;;) {
    const ssize_t got = ::read(output[0], buffer, sizeof buffer);
    if (got <= 0) break;
    printed.append(buffer, static_cast<std::size_t>(got));
  }
  ::close(output[0]);
  if (!exited) {
    ASSERT_EQ(::waitpid(child, &status, 0), child);
  }
  const auto stopped_after = std::chrono::steady_clock::now() - signalled;

  ASSERT_TRUE(live) << "the node's loop never showed running: " << printed;
  ASSERT_TRUE(WIFEXITED(status))
      << "killed by signal " << (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
  EXPECT_EQ(WEXITSTATUS(status), 0) << printed;
  EXPECT_NE(printed.find("dispatches="), std::string::npos) << printed;
  EXPECT_LT(stopped_after, 30s) << "the stop waited for --duration-s";
}

}  // namespace
}  // namespace talos
