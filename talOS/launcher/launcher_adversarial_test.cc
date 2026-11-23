// Targeted adversarial and boundary tests for launcher supervision, bundle
// loading, CLI parsing, probes, and pipe/signal safety.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/introspection/describe.h"
#include "talOS/launcher/describe_graph.h"
#include "talOS/launcher/launcher.h"
#include "talOS/launcher/roster.h"
#include "talOS/launcher/service_notifier.h"
#include "talOS/launcher/session.h"
#include "talOS/launcher/spawn.h"

namespace talos::launcher {
namespace {

namespace fs = std::filesystem;

fs::path FreshDir(const std::string& name) {
  const fs::path dir =
      fs::path{::testing::TempDir()} /
      ("launcher_adv_" + name + "_" + std::to_string(::getpid()));
  std::error_code ec;
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  return dir;
}

struct BinaryResult {
  int exit_code{-1};
  std::string stderr_output;
};

BinaryResult RunBinary(const std::string& binary,
                       std::vector<std::string> args) {
  args.insert(args.begin(), binary);
  int err_pipe[2]{-1, -1};
  if (::pipe(err_pipe) != 0) return {};
  const pid_t child = ::fork();
  if (child == 0) {
    ::close(err_pipe[0]);
    const int devnull = ::open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      ::dup2(devnull, STDOUT_FILENO);
      ::close(devnull);
    }
    ::dup2(err_pipe[1], STDERR_FILENO);
    ::close(err_pipe[1]);
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (auto& a : args) argv.push_back(a.data());
    argv.push_back(nullptr);
    ::execv(binary.c_str(), argv.data());
    ::_exit(127);
  }
  ::close(err_pipe[1]);
  BinaryResult out;
  if (child < 0) {
    ::close(err_pipe[0]);
    return out;
  }
  char buf[256];
  for (;;) {
    const ssize_t n = ::read(err_pipe[0], buf, sizeof(buf));
    if (n > 0) {
      out.stderr_output.append(buf, static_cast<std::size_t>(n));
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    break;
  }
  ::close(err_pipe[0]);
  int status = 0;
  while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {
  }
  out.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return out;
}

// REQ-OPS-25, REQ-OPS-34: LoadBundle must return false cleanly on hostile
// filesystem paths (symlink loops, overlong names, regular files) instead of
// throwing uncaught filesystem_error, and refuse manifest entries that escape
// the bundle root via absolute paths or `..` traversal.
TEST(LauncherAdversarial, LoadBundleHandlesFilesystemErrorsAndPathTraversal) {
  const fs::path dir = FreshDir("bundle");

  // Symlink loop (ELOOP) must fail cleanly without throwing.
  const fs::path loop_a = dir / "loop_a";
  const fs::path loop_b = dir / "loop_b";
  fs::create_symlink(loop_b, loop_a);
  fs::create_symlink(loop_a, loop_b);
  {
    LauncherOptions options;
    options.bundle_dir = loop_a.string();
    std::string error;
    EXPECT_NO_THROW({ EXPECT_FALSE(LoadBundle(options, error)); });
    EXPECT_FALSE(error.empty());
  }

  // Overlong component (ENAMETOOLONG) must fail cleanly without throwing.
  {
    LauncherOptions options;
    options.bundle_dir = (dir / std::string(4096, 'a')).string();
    std::string error;
    EXPECT_NO_THROW({ EXPECT_FALSE(LoadBundle(options, error)); });
    EXPECT_FALSE(error.empty());
  }

  // A regular file as bundle_dir is not a directory.
  const fs::path regular = dir / "not_a_dir";
  std::ofstream{regular} << "hello\n";
  {
    LauncherOptions options;
    options.bundle_dir = regular.string();
    std::string error;
    EXPECT_FALSE(LoadBundle(options, error));
    EXPECT_FALSE(error.empty());
  }

  // Hostile bundle_manifest.toml entries attempting path traversal or absolute
  // escapes out of the bundle root must be refused.
  const fs::path bundle = dir / "hostile_bundle";
  fs::create_directories(bundle);
  const std::vector<std::string> hostile_manifests{
      "config_path = \"/etc/passwd\"\n",
      "config_path = \"../../outside.toml\"\n",
      "[[nodes]]\nname = \"alpha\"\nbinary = \"/bin/sh\"\n",
      "[[nodes]]\nname = \"alpha\"\nbinary = \"../escape\"\n",
      "[studio]\nbridge = \"/bin/sh\"\nwebroot = \"studio/webroot\"\n",
      "[studio]\nbridge = \"studio/bridge\"\nwebroot = \"/etc\"\n",
      "[studio]\nbridge = \"studio/bridge\"\nwebroot = \"../../etc\"\n",
  };
  for (std::size_t i = 0; i < hostile_manifests.size(); ++i) {
    SCOPED_TRACE("manifest " + std::to_string(i));
    std::ofstream{bundle / "bundle_manifest.toml"} << hostile_manifests[i];
    LauncherOptions options;
    options.bundle_dir = bundle.string();
    std::string error;
    EXPECT_FALSE(LoadBundle(options, error)) << hostile_manifests[i];
    EXPECT_FALSE(error.empty());
  }

  std::error_code ec;
  fs::remove_all(dir, ec);
}

// REQ-SAFE-30, REQ-OPS-33: SpawnProcess rejects empty args cleanly, and
// CloseOnExecPipe relocates ends below fd 3 so closed stdout/stderr never get
// hijacked by lifetime or wakeup pipes.
TEST(LauncherAdversarial, SpawnRejectsEmptyArgsAndRelocatesPipesAboveStdFds) {
  std::string error;
  EXPECT_EQ(SpawnProcess({}, error), -1);
  EXPECT_FALSE(error.empty());
  error.clear();
  EXPECT_EQ(SpawnProcess({""}, error), -1);
  EXPECT_FALSE(error.empty());

  // Fork so closing fd 1 and fd 2 does not disturb the gtest runner.
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(STDOUT_FILENO);
    ::close(STDERR_FILENO);
    int fds[2]{-1, -1};
    if (!CloseOnExecPipe(fds)) ::_exit(2);
    const bool ok = fds[0] >= 3 && fds[1] >= 3;
    ::close(fds[0]);
    ::close(fds[1]);
    ::_exit(ok ? 0 : 3);
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
}

// REQ-OPS-07: huge WATCHDOG_USEC (up to INT64_MAX) must not overflow
// steady_clock::time_point (nanoseconds) when scheduling the next watchdog
// deadline via ServiceNotifier::NextDeadline().
TEST(LauncherAdversarial, HugeWatchdogUsecDoesNotOverflowNanoseconds) {
  const char* prev_sock = std::getenv("NOTIFY_SOCKET");
  const char* prev_wd = std::getenv("WATCHDOG_USEC");
  const std::string saved_sock = prev_sock ? prev_sock : "";
  const std::string saved_wd = prev_wd ? prev_wd : "";

  ASSERT_EQ(::setenv("NOTIFY_SOCKET", "/tmp/talos_test_wd_unused.sock", 1), 0);
  ASSERT_EQ(::setenv("WATCHDOG_USEC", "9223372036854775807", 1), 0);
  {
    const ServiceNotifier notifier;
    const auto interval = notifier.watchdog_interval();
    ASSERT_TRUE(interval.has_value());
    EXPECT_EQ(*interval, std::chrono::microseconds{
                             std::numeric_limits<int64_t>::max() / 2});
    const auto now = std::chrono::steady_clock::now();
    const auto wake = ServiceNotifier::NextDeadline(now, *interval);
    EXPECT_EQ(wake, std::chrono::steady_clock::time_point::max());
    EXPECT_GT(wake, now);
    // Normal intervals still add accurately.
    EXPECT_EQ(
        ServiceNotifier::NextDeadline(now, std::chrono::microseconds{500000}),
        now + std::chrono::microseconds{500000});
  }

  if (prev_sock) {
    ::setenv("NOTIFY_SOCKET", saved_sock.c_str(), 1);
  } else {
    ::unsetenv("NOTIFY_SOCKET");
  }
  if (prev_wd) {
    ::setenv("WATCHDOG_USEC", saved_wd.c_str(), 1);
  } else {
    ::unsetenv("WATCHDOG_USEC");
  }
}

// REQ-OPS-01: ProbeDescribe with an invalid (closed) stop_fd must not block
// forever inside read(), and non-positive timeout must fail immediately.
TEST(LauncherAdversarial, ProbeDescribeHandlesInvalidStopFdAndBadTimeouts) {
  int fds[2]{-1, -1};
  ASSERT_EQ(::pipe(fds), 0);
  const int closed_fd = fds[0];
  ::close(fds[0]);
  ::close(fds[1]);

  introspect::Description out;
  std::string error;
  // A binary that sleeps without printing anything; with a closed stop_fd
  // (POLLNVAL) the probe must still time out cleanly in ~50 ms instead of
  // hanging on a blocking read().
  const bool ok =
      ProbeDescribe({"/bin/sh", "-c", "sleep 1"}, std::chrono::milliseconds{50},
                    out, error, 0, closed_fd);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("50ms"), std::string::npos) << error;

  // Non-positive timeout must not block in poll(-1).
  error.clear();
  EXPECT_FALSE(ProbeDescribe({"/bin/sh", "-c", "sleep 1"},
                             std::chrono::milliseconds{-1}, out, error, 0, -1));
  EXPECT_FALSE(error.empty());
}

// REQ-OPS-01, REQ-OPS-14: Launcher refuses non-positive describe_timeout_ms,
// handles ELOOP in --replay-inputs cleanly without throwing, and CLI rejects
// negative/trailing-garbage integer flags.
TEST(LauncherAdversarial, RefusesBadCliFlagsAndReplaySymlinkLoops) {
  const fs::path dir = FreshDir("cli");
  const fs::path cfg = dir / "robot.toml";
  std::ofstream{cfg} << "[robot]\nperiod_us = 5000\n";

  // Non-positive describe_timeout_ms is refused cleanly.
  {
    LauncherOptions options;
    options.config_path = cfg.string();
    options.output_dir = (dir / "logs").string();
    options.describe_only = true;
    options.describe_timeout_ms = 0;
    Launcher launcher{std::move(options)};
    EXPECT_EQ(launcher.Run(), 1);
  }

  // Symlink loop passed to --replay-inputs or --output-dir during replay must
  // return 1, not throw filesystem_error.
  const fs::path loop_a = dir / "rloop_a";
  const fs::path loop_b = dir / "rloop_b";
  fs::create_symlink(loop_b, loop_a);
  fs::create_symlink(loop_a, loop_b);
  const fs::path real_replay = dir / "real_replay";
  fs::create_directories(real_replay);
  {
    LauncherOptions options;
    options.config_path = cfg.string();
    options.output_dir = (dir / "out").string();
    options.replay_inputs = loop_a.string();
    options.only = "alpha";
    Launcher launcher{std::move(options)};
    EXPECT_NO_THROW({ EXPECT_EQ(launcher.Run(), 1); });
  }
  {
    LauncherOptions options;
    options.config_path = cfg.string();
    options.output_dir = loop_a.string();
    options.replay_inputs = real_replay.string();
    options.only = "alpha";
    Launcher launcher{std::move(options)};
    EXPECT_NO_THROW({ EXPECT_EQ(launcher.Run(), 1); });
  }

  // CLI binary rejects negative session_id, negative duration_s, non-positive
  // describe_timeout_ms, and trailing garbage on numeric flags.
  std::string rf_error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &rf_error));
  ASSERT_NE(runfiles, nullptr) << rf_error;
  const char* env_bin = std::getenv("TALOS_LAUNCHER");
  ASSERT_NE(env_bin, nullptr);
  const std::string binary = runfiles->Rlocation(env_bin);

  const std::vector<std::vector<std::string>> bad_invocations{
      {"--config", cfg.string(), "--describe-only", "--session-id", "-1"},
      {"--config", cfg.string(), "--describe-only", "--session-id", "42xyz"},
      {"--config", cfg.string(), "--describe-only", "--duration-s", "-5"},
      {"--config", cfg.string(), "--describe-only", "--duration-s", "5abc"},
      {"--config", cfg.string(), "--describe-only", "--max-restarts", "0foo"},
      {"--config", cfg.string(), "--describe-only", "--describe-timeout-ms",
       "0"},
      {"--config", cfg.string(), "--describe-only", "--describe-timeout-ms",
       "-10"},
  };
  for (const auto& args : bad_invocations) {
    const BinaryResult result = RunBinary(binary, args);
    EXPECT_EQ(result.exit_code, 1);
    EXPECT_NE(result.stderr_output.find("invalid value for"), std::string::npos)
        << result.stderr_output;
  }

  std::error_code ec;
  fs::remove_all(dir, ec);
}

// REQ-OPS-08: Quoted() in session.cc must escape invalid UTF-8 bytes so
// manifest.json / graph.json / last_refusal.json always stay valid UTF-8 JSON.
TEST(LauncherAdversarial, QuotedEscapesInvalidUtf8Bytes) {
  const std::string encoded = Quoted(std::string_view{"ok\xff\xfe", 4});
  EXPECT_EQ(encoded, "\"ok\\u00ff\\u00fe\"");
  // Valid multibyte UTF-8 passes through unchanged.
  EXPECT_EQ(Quoted("a\xc3\xa9"), "\"a\xc3\xa9\"");
}

}  // namespace
}  // namespace talos::launcher
