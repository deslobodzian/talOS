// Launcher supervision: clear failures, isolated sessions, and exit-code
// propagation from critical nodes.
//
// Every session runs in a pid-unique temp directory with shell-stub nodes,
// so no two tests share state, ports, or shared memory. Stubs answer
// --describe by exiting nonzero (the documented "warning, launch proceeds"
// path), which keeps phase one fast without redeclaring the describe JSON.

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

#include "talOS/launcher/launcher.h"

namespace talos::launcher {
namespace {

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_supervision_" +
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

TEST(Supervision, MissingConfigFailsFastWithClearError) {
  LauncherOptions options;
  options.output_dir = TempPath("missing_cfg_out");
  options.config_path = TempPath("does_not_exist") + ".toml";
  Launcher launcher{options};
  EXPECT_EQ(launcher.Run(), 1);
  EXPECT_FALSE(std::filesystem::exists(options.output_dir + "/graph.json"));
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

TEST(Supervision, CriticalFailurePropagatesExitCode) {
  LauncherOptions options = BaseOptions("crit_out");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("crit_alpha", "exit 3\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("crit_beta", "exit 0\n");
  EXPECT_EQ(Launcher{options}.Run(), 3);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  EXPECT_NE(manifest.find("\"name\": \"alpha\""), std::string::npos);
  EXPECT_NE(manifest.find("\"exit_code\": 3"), std::string::npos) << manifest;
  std::filesystem::remove_all(options.output_dir);
}

}  // namespace
}  // namespace talos::launcher
