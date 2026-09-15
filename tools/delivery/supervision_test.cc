// Launcher delivery tests: isolated sessions, clear failures (item 54) and
// bounded shutdown with critical-vs-viewer separation (item 55).
//
// Every session runs in a pid-unique temp directory with shell-stub nodes,
// so no two tests share state, ports, or shared memory. Stubs answer
// --describe by exiting nonzero (the documented "warning, launch proceeds"
// path), which keeps phase one fast without redeclaring the describe JSON.
//
// Known gap this test does NOT paper over (filed as a follow-up): on the
// stop path the launcher sends one SIGINT per child and never escalates to
// SIGKILL, so a child that ignores SIGINT holds the session open past the
// deadline. The bounded-shutdown test below pins the honored-signal case;
// the SIGKILL-escalation fix belongs to the launcher owner.

#include "talOS/launcher/launcher.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <sys/stat.h>
#include <unistd.h>

namespace talos::launcher {
namespace {

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_delivery_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

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

std::string WriteFixtureToml(std::string_view tag) {
  const std::string path = TempPath(tag) + ".toml";
  std::ofstream out{path};
  out << kFixtureToml;
  return path;
}

std::string WriteStub(std::string_view tag, std::string_view body) {
  const std::string path = TempPath(tag) + ".sh";
  std::ofstream out{path};
  out << "#!/bin/sh\n"
      << "for arg in \"$@\"; do\n"
      << "  if [ \"$arg\" = \"--describe\" ]; then exit 1; fi\n"
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

LauncherOptions BaseOptions(std::string_view tag) {
  LauncherOptions options;
  options.output_dir = TempPath(tag);
  options.describe_timeout_ms = 4000;
  options.config_path = WriteFixtureToml(tag);
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      ::access("/usr/bin/true", X_OK) == 0 ? "/usr/bin/true" : "/bin/echo";
  return options;
}

TEST(DeliverySupervision, MissingConfigFailsFastWithClearError) {
  LauncherOptions options;
  options.output_dir = TempPath("missing_cfg_out");
  options.config_path = TempPath("does_not_exist") + ".toml";
  Launcher launcher{options};
  EXPECT_EQ(launcher.Run(), 1);
  EXPECT_FALSE(std::filesystem::exists(options.output_dir + "/graph.json"));
}

TEST(DeliverySupervision, SessionsAreIsolatedAndCleanOnlyOwnedDirs) {
  const std::string sentinel = TempPath("sentinel");
  { std::ofstream out{sentinel}; out << "do not touch\n"; }

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

  EXPECT_EQ(Launcher{a}.Run(), 0);
  EXPECT_EQ(Launcher{b}.Run(), 0);
  EXPECT_TRUE(std::filesystem::exists(a.output_dir + "/manifest.json"));
  EXPECT_TRUE(std::filesystem::exists(b.output_dir + "/manifest.json"));
  // Distinct sessions: manifests name their own output dir, never each
  // other's, and neither run touched the sibling sentinel file.
  EXPECT_NE(ReadFile(a.output_dir + "/manifest.json").find(a.output_dir),
            std::string::npos);
  EXPECT_EQ(ReadFile(a.output_dir + "/manifest.json").find(b.output_dir),
            std::string::npos);
  EXPECT_EQ(ReadFile(sentinel), "do not touch\n");

  // Cleanup removes only what the test owned.
  std::filesystem::remove_all(a.output_dir);
  std::filesystem::remove_all(b.output_dir);
  EXPECT_TRUE(std::filesystem::exists(sentinel));
  EXPECT_FALSE(std::filesystem::exists(a.output_dir));
  ::remove(sentinel.c_str());
}

TEST(DeliverySupervision, CriticalFailurePropagatesExitCode) {
  LauncherOptions options = BaseOptions("crit_out");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("crit_alpha", "exit 3\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("crit_beta", "exit 0\n");
  EXPECT_EQ(Launcher{options}.Run(), 3);
  const std::string manifest =
      ReadFile(options.output_dir + "/manifest.json");
  EXPECT_NE(manifest.find("\"name\": \"alpha\""), std::string::npos);
  EXPECT_NE(manifest.find("\"exit_code\": 3"), std::string::npos)
      << manifest;
  std::filesystem::remove_all(options.output_dir);
}

TEST(DeliverySupervision, FailingViewerDoesNotMoveTheDeadline) {
  LauncherOptions options = BaseOptions("viewer_out");
  options.simulation = true;
  options.start_studio_bridge = true;
  options.duration_s = 2;
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("viewer_alpha", "exec sleep 30\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("viewer_beta", "exec sleep 30\n");
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStub("viewer_bridge", "exit 1\n");

  const auto started = std::chrono::steady_clock::now();
  Launcher{options}.Run();
  const auto elapsed = std::chrono::steady_clock::now() - started;

  // Bounded: the deadline ends the session even with a failing viewer and
  // long-lived nodes (SIGINT is honored by `exec sleep`).
  EXPECT_GE(elapsed, std::chrono::seconds(2));
  EXPECT_LT(elapsed, std::chrono::seconds(20));
  const std::string manifest =
      ReadFile(options.output_dir + "/manifest.json");
  EXPECT_NE(manifest.find("\"name\": \"studio_bridge\""), std::string::npos)
      << manifest;
  EXPECT_NE(manifest.find("\"name\": \"alpha\""), std::string::npos)
      << manifest;
  std::filesystem::remove_all(options.output_dir);
}

TEST(DeliverySupervision, ShutdownAfterDeadlineIsBounded) {
  LauncherOptions options = BaseOptions("stop_out");
  options.duration_s = 1;
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("stop_alpha", "exec sleep 30\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("stop_beta", "exec sleep 30\n");

  const auto started = std::chrono::steady_clock::now();
  Launcher{options}.Run();
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_LT(elapsed, std::chrono::seconds(20))
      << "deadline + SIGINT must end the session on a bounded horizon";
  EXPECT_TRUE(std::filesystem::exists(options.output_dir + "/manifest.json"));
  std::filesystem::remove_all(options.output_dir);
}

}  // namespace
}  // namespace talos::launcher
