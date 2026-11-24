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
#include "talOS/introspection/lock_file.h"
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

// REQ-OPS-11: Retention never removes or follows symlinks — neither a symlink
// in the log root pointing to an external session directory nor symlinks inside
// a kept session pointing to external files or directories — while still
// pruning real expired sessions under log_keep_sessions = 1.
TEST(LauncherRetention, SymlinksInLogRootAndKeptSessionSurviveUntouched) {
  auto options = RetentionFixture("symlinks");
  const std::string root = options.output_dir;
  const std::string external_session = TempPath("symlinks_external_session");
  const std::string external_file = TempPath("symlinks_external_file.txt");
  const std::string external_dir = TempPath("symlinks_external_dir");
  std::filesystem::remove_all(root);
  std::filesystem::remove_all(external_session);
  std::filesystem::remove_all(external_dir);
  std::filesystem::remove(external_file);

  // External session directory linked into the log root as an expired session
  // id ("1"): retention must ignore the symlink and leave its target intact.
  WriteSessionManifest(external_session, 1, 1001, 1101);
  std::ofstream{external_session + "/sentinel.txt"} << "external_session_data";
  std::filesystem::create_directories(root);
  std::filesystem::create_directory_symlink(external_session, root + "/1");

  // Ordinary expired session ("2"): proves retention actively pruned under
  // log_keep_sessions = 1.
  WriteSessionManifest(root + "/2", 2, 1002, 1102);

  // Kept session ("3", protected match) containing symlinks to an external
  // file and an external directory: TreeBytes walks it without following or
  // touching either symlink or its target.
  std::filesystem::create_directories(root + "/3");
  std::ofstream{root + "/3/manifest.json"} << "{\n"
                                           << "  \"session_id\": 3,\n"
                                           << "  \"simulation\": false,\n"
                                           << "  \"start_wall_ns\": 1003,\n"
                                           << "  \"end_wall_ns\": 1103,\n"
                                           << "  \"nodes\": []\n"
                                           << "}\n";
  std::ofstream{root + "/3/" + std::string{kSessionClassFile}} << "match 0\n";
  std::ofstream{external_file} << "external_file_data";
  std::filesystem::create_directories(external_dir);
  std::ofstream{external_dir + "/nested.txt"} << "external_dir_data";
  std::filesystem::create_symlink(external_file, root + "/3/file_link");
  std::filesystem::create_directory_symlink(external_dir, root + "/3/dir_link");

  const int result = Launcher{options}.Run();
  EXPECT_EQ(result, 0);

  // Real expired session 2 was pruned and recorded.
  EXPECT_FALSE(std::filesystem::exists(root + "/2"));
  EXPECT_TRUE(std::filesystem::exists(root + "/10/manifest.json"));
  const std::string retention =
      ReadFile(root + "/10/" + std::string{kRetentionRecordFile});
  EXPECT_NE(retention.find(root + "/2"), std::string::npos) << retention;
  EXPECT_EQ(retention.find(root + "/1"), std::string::npos) << retention;
  EXPECT_EQ(retention.find(root + "/3"), std::string::npos) << retention;

  // Root-level symlink and its external session target survived untouched.
  EXPECT_TRUE(std::filesystem::is_symlink(root + "/1"));
  EXPECT_EQ(std::filesystem::read_symlink(root + "/1"), external_session);
  EXPECT_TRUE(std::filesystem::exists(external_session + "/manifest.json"));
  EXPECT_EQ(ReadFile(external_session + "/sentinel.txt"),
            "external_session_data");

  // Kept session, its symlinks, and their external targets survived untouched.
  EXPECT_TRUE(std::filesystem::exists(root + "/3/manifest.json"));
  EXPECT_TRUE(std::filesystem::is_symlink(root + "/3/file_link"));
  EXPECT_EQ(std::filesystem::read_symlink(root + "/3/file_link"),
            external_file);
  EXPECT_EQ(ReadFile(external_file), "external_file_data");
  EXPECT_TRUE(std::filesystem::is_symlink(root + "/3/dir_link"));
  EXPECT_EQ(std::filesystem::read_symlink(root + "/3/dir_link"), external_dir);
  EXPECT_EQ(ReadFile(external_dir + "/nested.txt"), "external_dir_data");

  std::filesystem::remove_all(root);
  std::filesystem::remove_all(external_session);
  std::filesystem::remove_all(external_dir);
  std::filesystem::remove(external_file);
  std::filesystem::remove(options.config_path);
}

// REQ-OPS-11: SessionDirectoryLock::Held returns false only when the lock file
// is absent (ENOENT/ENOTDIR) or when flock succeeds (or reports
// FlockUnsupported on a lockless root). Non-ENOENT open errors (EACCES, ELOOP)
// and flock errors fail closed (true), and flock retries on EINTR.
TEST(LauncherRetention,
     SessionDirectoryLockHeldFailsClosedOnErrorsAndRetriesEintr) {
  const std::filesystem::path base = TempPath("held_fail_closed");
  std::filesystem::remove_all(base);
  std::filesystem::create_directories(base);

  // Absent lock file (ENOENT) -> not held.
  const std::filesystem::path absent = base / "absent";
  std::filesystem::create_directories(absent);
  EXPECT_FALSE(
      SessionDirectoryLock::Held(absent, /*root_takes_no_locks=*/false));

  // Regular file as session dir (ENOTDIR) -> not held.
  const std::filesystem::path not_a_dir = base / "not_a_dir";
  std::ofstream{not_a_dir} << "x";
  EXPECT_FALSE(
      SessionDirectoryLock::Held(not_a_dir, /*root_takes_no_locks=*/false));

  // Symlinked session.lock fails open(O_NOFOLLOW) with ELOOP -> fail closed.
  const std::filesystem::path symlink_dir = base / "symlink_lock";
  std::filesystem::create_directories(symlink_dir);
  const std::filesystem::path target = base / "target_lock";
  std::ofstream{target} << "";
  std::filesystem::create_symlink(
      target, symlink_dir / std::string{kSessionDirectoryLockFile});
  EXPECT_TRUE(
      SessionDirectoryLock::Held(symlink_dir, /*root_takes_no_locks=*/false));
  EXPECT_TRUE(
      SessionDirectoryLock::Held(symlink_dir, /*root_takes_no_locks=*/true));

  // Unreadable session.lock (EACCES, when not running as root) -> fail closed.
  const std::filesystem::path eacces_dir = base / "eacces_lock";
  std::filesystem::create_directories(eacces_dir);
  const std::filesystem::path eacces_lock =
      eacces_dir / std::string{kSessionDirectoryLockFile};
  std::ofstream{eacces_lock} << "";
  if (::geteuid() != 0) {
    ASSERT_EQ(::chmod(eacces_lock.c_str(), 0000), 0);
    EXPECT_TRUE(
        SessionDirectoryLock::Held(eacces_dir, /*root_takes_no_locks=*/false));
    EXPECT_TRUE(
        SessionDirectoryLock::Held(eacces_dir, /*root_takes_no_locks=*/true));
    ASSERT_EQ(::chmod(eacces_lock.c_str(), 0600), 0);
  }

  // flock returning EINTR is retried before deciding held vs idle.
  const std::filesystem::path eintr_dir = base / "eintr_lock";
  std::error_code create_ec;
  {
    auto held_lock = SessionDirectoryLock::Create(
        eintr_dir,
        [](const std::filesystem::path& p, std::error_code& ec) {
          std::filesystem::create_directories(p, ec);
        },
        create_ec);
    ASSERT_TRUE(held_lock.has_value()) << create_ec.message();

    int calls = 0;
    const auto eintr_twice_then_real = [&calls](int fd, int op) {
      if (calls++ < 2) {
        errno = EINTR;
        return -1;
      }
      return ::flock(fd, op);
    };
    EXPECT_TRUE(SessionDirectoryLock::Held(
        eintr_dir, /*root_takes_no_locks=*/false, eintr_twice_then_real));
    EXPECT_EQ(calls, 3);
  }
  {
    int calls = 0;
    const auto eintr_twice_then_real = [&calls](int fd, int op) {
      if (calls++ < 2) {
        errno = EINTR;
        return -1;
      }
      return ::flock(fd, op);
    };
    EXPECT_FALSE(SessionDirectoryLock::Held(
        eintr_dir, /*root_takes_no_locks=*/false, eintr_twice_then_real));
    EXPECT_EQ(calls, 3);
  }

  // Persistent EINTR and unexpected flock errors fail closed without looping
  // forever.
  EXPECT_TRUE(SessionDirectoryLock::Held(
      eintr_dir, /*root_takes_no_locks=*/false, [](int, int) {
        errno = EINTR;
        return -1;
      }));
  EXPECT_TRUE(SessionDirectoryLock::Held(
      eintr_dir, /*root_takes_no_locks=*/false, [](int, int) {
        errno = ENOMEM;
        return -1;
      }));

  // FlockUnsupported respects root_takes_no_locks.
  const auto unsupported = [](int, int) {
    errno = EOPNOTSUPP;
    return -1;
  };
  EXPECT_TRUE(SessionDirectoryLock::Held(
      eintr_dir, /*root_takes_no_locks=*/false, unsupported));
  EXPECT_FALSE(SessionDirectoryLock::Held(
      eintr_dir, /*root_takes_no_locks=*/true, unsupported));

  std::filesystem::remove_all(base);
}

// REQ-OPS-11, REQ-OPS-02: LockSharesLogFilesystem resolves relative
// non-existent --output-dir paths against the working directory rather than
// walking parent_path() to "" and falsely returning false, so unlocked
// retention still runs when the session lock file cannot be created for ENOSPC.
TEST(LauncherRetention, RelativeNonExistentOutputDirSharesLockFilesystem) {
  const std::filesystem::path lock_dir{introspect::LockDirectory()};
  const std::filesystem::path work_dir =
      lock_dir / ("talos_rel_log_fs_" + std::to_string(::getpid()));
  std::filesystem::remove_all(work_dir);
  std::filesystem::create_directories(work_dir);

  struct stat lock_st {};
  struct stat work_st {};
  ASSERT_EQ(::stat(lock_dir.c_str(), &lock_st), 0);
  ASSERT_EQ(::stat(work_dir.c_str(), &work_st), 0);
  ASSERT_EQ(lock_st.st_dev, work_st.st_dev);

  const std::filesystem::path saved_cwd = std::filesystem::current_path();
  struct CwdGuard {
    std::filesystem::path prev;
    std::filesystem::path cleanup;
    ~CwdGuard() {
      std::error_code ec;
      std::filesystem::current_path(prev, ec);
      std::filesystem::remove_all(cleanup, ec);
    }
  } cwd_guard{saved_cwd, work_dir};
  std::filesystem::current_path(work_dir);

  for (const std::string_view rel_dir : {"logs", "nested/logs"}) {
    SCOPED_TRACE(rel_dir);
    std::filesystem::remove_all(work_dir / "logs");
    std::filesystem::remove_all(work_dir / "nested");

    auto options = RetentionFixture(rel_dir == "logs" ? "rel_one" : "rel_two");
    options.output_dir = std::string{rel_dir};
    int lock_calls = 0;
    options.acquire_session_lock = [&lock_calls](std::string& error,
                                                 bool* no_room) {
      if (lock_calls++ == 0) {
        error = "cannot open the session lock: No space left on device";
        if (no_room != nullptr) *no_room = true;
        return std::optional<SessionLock>{};
      }
      return SessionLock::Acquire(error, no_room);
    };

    ::testing::internal::CaptureStderr();
    const int result = Launcher{options}.Run();
    const std::string stderr_text = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(result, 0) << stderr_text;
    EXPECT_EQ(lock_calls, 2) << stderr_text;
    EXPECT_NE(stderr_text.find("retention runs first, then the lock is taken"),
              std::string::npos)
        << stderr_text;
    EXPECT_TRUE(
        std::filesystem::exists(work_dir / rel_dir / "10" / "manifest.json"));
    std::filesystem::remove(options.config_path);
  }
}

}  // namespace
}  // namespace talos::launcher
