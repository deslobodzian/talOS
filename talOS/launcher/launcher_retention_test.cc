// Retention safety under advisory-lock failures (ENOLCK vs true lockless
// filesystems) and racing ENOENT during directory removal (LOG-10, REQ-OPS-11).

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "talOS/introspection/describe.h"
#include "talOS/launcher/launcher.h"
#include "talOS/launcher/session_lock.h"
#include "talOS/launcher/session_retention.h"
#include "talOS/launcher/testdata/first_exec.h"

namespace talos::launcher {
namespace {

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_launcher_retention_test_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

constexpr std::string_view kFixtureToml = R"(
[robot]
name = "retention_fixture"
log_keep_sessions = 1
period_us = 5000

[subsystems.alpha]
node = "//example/alpha:node"
period_us = 5000
)";

std::string WriteStubNode(const std::string& name, const std::string& body) {
  const std::string path = TempPath(name) + ".sh";
  talos::launcher::testing::WriteWarmStub(path, body, 0700);
  return path;
}

std::string DescribingStub(const std::string& node, const std::string& target) {
  introspect::Description description;
  description.name = node;
  description.target = target;
  std::ostringstream script;
  script << "#!/bin/sh\n"
         << "for arg in \"$@\"; do\n"
         << "  if [ \"$arg\" = \"--describe\" ]; then\n"
         << "    cat <<'TALOS_DESCRIBE'\n"
         << introspect::DescribeToJson(description) << "TALOS_DESCRIBE\n"
         << "    exit 0\n"
         << "  fi\n"
         << "done\n"
         << "exit 0\n";
  return script.str();
}

std::string ReadFile(const std::string& path) {
  std::ifstream in{path};
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

void WriteSessionManifest(const std::string& dir, uint64_t id,
                          int64_t start_wall_ns, int64_t end_wall_ns = 0) {
  std::filesystem::create_directories(dir);
  std::ofstream{dir + "/manifest.json"}
      << "{\n"
      << "  \"session_id\": " << id << ",\n"
      << "  \"simulation\": true,\n"
      << "  \"start_wall_ns\": " << start_wall_ns << ",\n"
      << "  \"end_wall_ns\": " << end_wall_ns << ",\n"
      << "  \"nodes\": []\n"
      << "}\n";
}

LauncherOptions RetentionFixture(std::string_view tag) {
  LauncherOptions options;
  const std::string config_path =
      TempPath(std::string{tag} + "_robot") + ".toml";
  std::ofstream{config_path} << kFixtureToml;
  options.config_path = config_path;
  options.output_dir = TempPath(tag);
  options.session_id = 10;
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      std::string{tag} + "_hardware",
      DescribingStub("hardware_node", "//talOS/bridge:hardware_node"));
  options.binary_overrides["//example/alpha:node"] =
      WriteStubNode(std::string{tag} + "_alpha",
                    DescribingStub("alpha", "//example/alpha:node"));
  options.available_bytes = [](const std::filesystem::path&) {
    return std::optional{std::numeric_limits<uint64_t>::max()};
  };
  return options;
}

// REQ-OPS-11: ENOLCK on .talos-lock-probe (e.g. a network lock daemon down)
// must NOT be treated as a lockless filesystem. Retention prints a warning
// naming ENOLCK and refuses/skips all deletions so a running session in
// another shared-memory namespace can never be deleted.
TEST(LauncherRetention, EnolckOnLockProbeWarnsAndDeletesZeroSessions) {
  for (const bool probe_only : {false, true}) {
    SCOPED_TRACE(probe_only ? "ENOLCK on probe only" : "ENOLCK on all flocks");
    auto options = RetentionFixture(probe_only ? "enolck_probe" : "enolck_all");
    const std::string root = options.output_dir;
    std::filesystem::remove_all(root);

    // Expired recorded sessions, an older abandoned directory, and a leftover
    // .talos-removing-* directory: every one must survive ENOLCK on the probe.
    WriteSessionManifest(root + "/1", 1, 1001, 1101);
    std::filesystem::create_directories(root + "/2");
    std::ofstream{root + "/2/graph.json"} << "{}";
    WriteSessionManifest(root + "/3", 3, 1003, 1103);
    std::filesystem::create_directories(root + "/.talos-removing-99");
    std::ofstream{root + "/.talos-removing-99/stale.txt"} << "leftover";

    // Pre-create .talos-lock-probe so probe_only can match its inode.
    const std::string probe_path = root + "/" + std::string{kLockProbeFile};
    std::ofstream{probe_path}.close();
    struct stat probe_stat {};
    ASSERT_EQ(::stat(probe_path.c_str(), &probe_stat), 0);

    int remove_calls = 0;
    options.remove_all = [&remove_calls](const std::filesystem::path& path,
                                         std::error_code& error) {
      ++remove_calls;
      return std::filesystem::remove_all(path, error);
    };
    options.lock_session_directory = [probe_only, probe_stat](int fd,
                                                              int operation) {
      if (probe_only) {
        struct stat st {};
        if (::fstat(fd, &st) == 0 && st.st_ino == probe_stat.st_ino &&
            st.st_dev == probe_stat.st_dev) {
          errno = ENOLCK;
          return -1;
        }
        return ::flock(fd, operation);
      }
      errno = ENOLCK;
      return -1;
    };

    ::testing::internal::CaptureStderr();
    const int result = Launcher{options}.Run();
    const std::string stderr_text = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(result, 0) << stderr_text;
    EXPECT_EQ(remove_calls, 0) << "ENOLCK must skip all retention deletions";
    EXPECT_TRUE(std::filesystem::exists(root + "/1/manifest.json"));
    EXPECT_TRUE(std::filesystem::exists(root + "/2/graph.json"));
    EXPECT_TRUE(std::filesystem::exists(root + "/3/manifest.json"));
    EXPECT_TRUE(
        std::filesystem::exists(root + "/.talos-removing-99/stale.txt"));
    EXPECT_TRUE(std::filesystem::exists(root + "/10/manifest.json"));
    EXPECT_FALSE(std::filesystem::exists(root + "/10/" +
                                         std::string{kRetentionRecordFile}));
    EXPECT_NE(stderr_text.find("WARNING"), std::string::npos) << stderr_text;
    EXPECT_NE(stderr_text.find("ENOLCK"), std::string::npos) << stderr_text;
    std::filesystem::remove_all(root);
    std::filesystem::remove(options.config_path);
  }
}

// REQ-OPS-11: True lockless filesystems (EOPNOTSUPP, ENOSYS) keep the existing
// warning + lockless behavior (the launch warns and runs, and retention still
// prunes expired sessions).
TEST(LauncherRetention, TrueLocklessFilesystemKeepsLocklessBehavior) {
  for (const int err : {EOPNOTSUPP, ENOSYS}) {
    SCOPED_TRACE(err == EOPNOTSUPP ? "EOPNOTSUPP" : "ENOSYS");
    auto options = RetentionFixture(err == EOPNOTSUPP ? "lockless_eopnotsupp"
                                                      : "lockless_enosys");
    const std::string root = options.output_dir;
    std::filesystem::remove_all(root);
    WriteSessionManifest(root + "/1", 1, 1001, 1101);
    options.lock_session_directory = [err](int, int) {
      errno = err;
      return -1;
    };

    EXPECT_TRUE(FlockUnsupported(err));
    EXPECT_FALSE(FlockUnsupported(ENOLCK));

    ::testing::internal::CaptureStderr();
    const int result = Launcher{options}.Run();
    const std::string stderr_text = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(result, 0) << stderr_text;
    EXPECT_FALSE(std::filesystem::exists(root + "/1"));
    EXPECT_TRUE(std::filesystem::exists(root + "/10/manifest.json"));
    EXPECT_NE(stderr_text.find("takes no advisory locks"), std::string::npos)
        << stderr_text;
    EXPECT_EQ(stderr_text.find("ENOLCK"), std::string::npos) << stderr_text;
    std::filesystem::remove_all(root);
    std::filesystem::remove(options.config_path);
  }
}

// REQ-OPS-11: When a concurrent retention sweep removes a .talos-removing-*
// tree under remove_all (returning ENOENT), both remove_idle and the leftover
// sweep treat it cleanly as removed without printing a "cannot remove" error.
// Non-ENOENT errors (EACCES) are still reported.
TEST(LauncherRetention, RacingEnoentInRemoveAllIsTreatedCleanlyAsRemoved) {
  {
    SCOPED_TRACE("racing ENOENT on remove_idle and leftover sweep");
    auto options = RetentionFixture("racing_enoent");
    const std::string root = options.output_dir;
    std::filesystem::remove_all(root);
    WriteSessionManifest(root + "/1", 1, 1001, 1101);
    std::filesystem::create_directories(root + "/.talos-removing-99");
    std::ofstream{root + "/.talos-removing-99/data.txt"} << "leftover";

    std::vector<std::string> removed_paths;
    options.remove_all = [&removed_paths](const std::filesystem::path& path,
                                          std::error_code& error) {
      removed_paths.push_back(path.filename().string());
      // Remove the directory as a concurrent sweeper would, then report ENOENT
      // as std::filesystem::remove_all does when an entry vanishes mid-walk.
      std::error_code ignored;
      std::filesystem::remove_all(path, ignored);
      error = std::make_error_code(std::errc::no_such_file_or_directory);
      return std::uintmax_t{0};
    };

    ::testing::internal::CaptureStderr();
    const int result = Launcher{options}.Run();
    const std::string stderr_text = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(result, 0) << stderr_text;
    EXPECT_FALSE(std::filesystem::exists(root + "/1"));
    EXPECT_FALSE(std::filesystem::exists(root + "/.talos-removing-99"));
    EXPECT_EQ(stderr_text.find("cannot remove"), std::string::npos)
        << stderr_text;
    EXPECT_NE(std::find(removed_paths.begin(), removed_paths.end(),
                        ".talos-removing-1"),
              removed_paths.end());
    EXPECT_NE(std::find(removed_paths.begin(), removed_paths.end(),
                        ".talos-removing-99"),
              removed_paths.end());
    const std::string retention =
        ReadFile(root + "/10/" + std::string{kRetentionRecordFile});
    EXPECT_NE(retention.find(root + "/1"), std::string::npos) << retention;
    std::filesystem::remove_all(root);
    std::filesystem::remove(options.config_path);
  }
  {
    SCOPED_TRACE("non-ENOENT error (EACCES) is still reported");
    auto options = RetentionFixture("remove_eacces");
    const std::string root = options.output_dir;
    std::filesystem::remove_all(root);
    WriteSessionManifest(root + "/1", 1, 1001, 1101);
    std::filesystem::create_directories(root + "/.talos-removing-99");
    std::ofstream{root + "/.talos-removing-99/data.txt"} << "leftover";

    options.remove_all = [](const std::filesystem::path&,
                            std::error_code& error) {
      error = std::make_error_code(std::errc::permission_denied);
      return std::uintmax_t{0};
    };

    ::testing::internal::CaptureStderr();
    const int result = Launcher{options}.Run();
    const std::string stderr_text = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(result, 0) << stderr_text;
    EXPECT_NE(stderr_text.find("cannot remove " + root + "/.talos-removing-1"),
              std::string::npos)
        << stderr_text;
    EXPECT_NE(stderr_text.find("cannot remove " + root + "/.talos-removing-99"),
              std::string::npos)
        << stderr_text;
    std::filesystem::remove_all(root);
    std::filesystem::remove(options.config_path);
  }
}

}  // namespace
}  // namespace talos::launcher
