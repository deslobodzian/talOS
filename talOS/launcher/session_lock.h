#pragma once

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>

#include "talOS/introspection/lock_file.h"
#include "talOS/memory/shared_memory_ptr.h"

namespace talos::launcher {

// The logical name the session lock is keyed by. Never a segment: the lock is
// a regular file named after its ShmObjectName (introspect::LockFilePath).
inline constexpr std::string_view kSessionLockName = "/talos_session";

// One launcher session per shared-memory namespace. A session owner replaces
// every declared topic's segment at startup and unlinks them at its end
// (RTMSOptions::session_owner), so a second launcher in the same namespace --
// a hand-run run.sh beside the boot service -- would unlink a running
// session's topics; that session's later restarts would attach to segments
// its other nodes no longer share. So a launch that starts nodes holds this
// for its lifetime, and a second one is refused, naming the holder.
//
// The namespace is ShmObjectName's (TALOS_SHM_NAMESPACE, else Bazel's
// TEST_TMPDIR, else the ordinary names), so concurrent test runs and sessions
// in their own namespaces do not exclude one another.
//
// A POSIX record lock (fcntl F_SETLK), not flock: F_GETLK reports the
// holder's pid, and a forked child -- a node, its guardian -- never inherits
// it, so a launcher's orphaned nodes cannot keep it held. The kernel releases
// it when the launcher exits or is killed. Record locks belong to the
// process, so the launcher opens this file nowhere else.
class SessionLock {
 public:
  // Takes the lock, or returns nullopt with `error` saying who holds it.
  // Locking an existing lock file allocates nothing on disk; creating one
  // does. When `no_room` is given, it is set to whether the failure was that
  // the path named no file and its filesystem had no room to create one
  // (ENOSPC, EDQUOT): macOS keeps the file in /tmp, emptied at boot, on the
  // same container as a log root, so a full log disk can refuse the first
  // launch after a reboot. No launcher holds the lock a new one would take
  // then.
  static std::optional<SessionLock> Acquire(std::string& error,
                                            bool* no_room = nullptr) {
    const std::string path =
        introspect::LockFilePath(ShmObjectName(kSessionLockName));
    if (no_room != nullptr) *no_room = false;
    // A lock taken on a file unlinked meanwhile (a /tmp cleaner, by hand)
    // excludes nobody who opens the path anew: take it again on the file the
    // path names.
    for (int attempt = 0; attempt < 10; ++attempt) {
      auto lock = AcquireOnce(path, error, no_room);
      if (!lock || introspect::LockFileIsCurrent(lock->fd_, path)) return lock;
    }
    error = "cannot lock the session lock " + path +
            ": it was removed every time it was locked";
    return std::nullopt;
  }

  // Keeps the held lock file from looking unused (introspect::TouchLockFile),
  // so a cleaner of old /tmp files does not delete it under a long session.
  // False when the path no longer names the locked file: a second launcher
  // could then start in this namespace. Call it at least daily.
  bool Refresh() const {
    introspect::TouchLockFile(fd_);
    return introspect::LockFileIsCurrent(fd_, path_);
  }

  ~SessionLock() {
    if (fd_ >= 0) ::close(fd_);
  }

  SessionLock(const SessionLock&) = delete;
  SessionLock& operator=(const SessionLock&) = delete;
  SessionLock(SessionLock&& other) noexcept
      : fd_{std::exchange(other.fd_, -1)}, path_{std::move(other.path_)} {}
  SessionLock& operator=(SessionLock&& other) noexcept {
    if (this != &other) {
      if (fd_ >= 0) ::close(fd_);
      fd_ = std::exchange(other.fd_, -1);
      path_ = std::move(other.path_);
    }
    return *this;
  }

  const std::string& path() const { return path_; }

 private:
  SessionLock(int fd, std::string path) : fd_{fd}, path_{std::move(path)} {}

  static std::optional<SessionLock> AcquireOnce(const std::string& path,
                                                std::string& error,
                                                bool* no_room) {
    const int fd = introspect::OpenLockFile(path, O_RDWR);
    if (fd < 0) {
      const int open_error = errno;
      // OpenLockFile opens an existing file without O_CREAT, so only its
      // create of an absent one runs out of room.
      if (no_room != nullptr) {
        *no_room = open_error == ENOSPC || open_error == EDQUOT;
      }
      struct stat status {};
      error = "cannot open the session lock " + path + ": " +
              std::strerror(open_error);
      if (::stat(path.c_str(), &status) == 0) {
        error += " (" + introspect::DescribeOwner(status) +
                 "); remove it as its owner or root";
      }
      return std::nullopt;
    }
    // A holder that exits between the refused set and the query leaves no
    // lock to report, so the set is tried again.
    for (int attempt = 0; attempt < 10; ++attempt) {
      struct flock whole = Whole(F_WRLCK);
      if (::fcntl(fd, F_SETLK, &whole) == 0) return SessionLock{fd, path};
      if (errno == EINTR) continue;
      if (errno != EACCES && errno != EAGAIN) {
        error = "cannot lock the session lock " + path + ": " +
                std::strerror(errno);
        ::close(fd);
        return std::nullopt;
      }
      struct flock holder = Whole(F_WRLCK);
      if (::fcntl(fd, F_GETLK, &holder) == 0 && holder.l_type != F_UNLCK) {
        error = "another launcher (pid " + std::to_string(holder.l_pid) +
                ") is running a session in this shared-memory namespace (" +
                path +
                "); starting a second would remove its topics. Stop it first "
                "(the boot service: systemctl stop talos on Linux, launchctl "
                "bootout system/org.talos.robot on macOS)";
        ::close(fd);
        return std::nullopt;
      }
    }
    error = "cannot lock the session lock " + path;
    ::close(fd);
    return std::nullopt;
  }

  static struct flock Whole(int type) {
    struct flock lock {};
    lock.l_type = static_cast<decltype(lock.l_type)>(type);
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 0;  // To the end of the file, however long it grows.
    return lock;
  }

  int fd_{-1};
  std::string path_;
};

// The file in a session directory whose lock the launch writing that
// directory holds while it runs (SessionDirectoryLock).
inline constexpr std::string_view kSessionDirectoryLockFile = "session.lock";

// Whether a session directory's launch is still running, read from the
// directory itself. SessionLock is per shared-memory namespace, and launches
// in different namespaces (a simulation beside a deployed or bench session,
// two developers' simulations) may write one log root, so holding it says
// nothing about another namespace's sessions there. Every launch holds an
// exclusive flock(2) on its session directory's kSessionDirectoryLockFile
// from before the directory holds anything until the launch ends, and
// retention (launcher_retention.cc) removes a directory only while it holds
// that lock itself.
//
// flock, not a record lock: it belongs to the open file description, so a
// probe from another descriptor conflicts with it even in the holder's own
// process, and closing some other descriptor of the file does not drop it.
// The descriptor is close-on-exec (introspect::OpenLockFile) and a node's
// guardian closes every descriptor, so only the launcher holds it: the
// kernel releases it when the launcher exits or is killed, and nothing the
// launcher started (whose guardian SIGKILLs its process group when the
// launcher dies) or a helper such a process daemonized keeps a dead session
// from being removed.
class SessionDirectoryLock {
 public:
  // Creates a directory and its parents, setting the error when it cannot
  // (LauncherOptions::create_session_directory).
  using CreateDirectory =
      std::function<void(const std::filesystem::path&, std::error_code&)>;

  // Creates `dir` with `create` (nothing when it exists) and takes its lock,
  // or returns nullopt with `error` set: create's error, the lock file's
  // open error, or std::errc::device_or_resource_busy when another launch
  // (one given the same --session-id) holds it. Retention removing `dir`
  // meanwhile holds the lock and moves `dir` away before removing it, so an
  // attempt that finds the lock held, or that locked a file the path no
  // longer names, creates `dir` again and retries: when this returns the
  // lock, `dir` is the directory the launch writes, and no retention can
  // remove it while the lock is held.
  static std::optional<SessionDirectoryLock> Create(
      const std::filesystem::path& dir, const CreateDirectory& create,
      std::error_code& error) {
    const std::string path = (dir / kSessionDirectoryLockFile).string();
    constexpr int kAttempts = 100;
    for (int attempt = 0; attempt < kAttempts; ++attempt) {
      error.clear();
      create(dir, error);
      if (error) return std::nullopt;
      const int fd = introspect::OpenLockFile(path, O_RDONLY);
      if (fd < 0) {
        error = {errno, std::generic_category()};
        // Moved away by retention since it was created: create it again.
        if (errno == ENOENT) continue;
        return std::nullopt;
      }
      if (::flock(fd, LOCK_EX | LOCK_NB) == 0) {
        if (introspect::LockFileIsCurrent(fd, path)) {
          return SessionDirectoryLock{fd};
        }
        ::close(fd);
        continue;
      }
      const int code = errno;
      ::close(fd);
      if (code != EWOULDBLOCK && code != EINTR) {
        error = {code, std::generic_category()};
        return std::nullopt;
      }
      // Retention holds it for the instant it takes to move `dir` away.
      error = std::make_error_code(std::errc::device_or_resource_busy);
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    return std::nullopt;
  }

  // Whether a launch (or a retention removing it) holds `dir`'s lock now.
  // False when `dir` holds no lock file: a launch holds one from before its
  // directory holds anything. Creates nothing.
  static bool Held(const std::filesystem::path& dir) {
    const int fd = ::open((dir / kSessionDirectoryLockFile).c_str(),
                          O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) return false;
    const bool held =
        ::flock(fd, LOCK_SH | LOCK_NB) != 0 && errno == EWOULDBLOCK;
    ::close(fd);
    return held;
  }

  // Takes `dir`'s lock to remove `dir`, creating the lock file when it is
  // absent (a session recorded before launches held one). Returns nullopt
  // with `held` set when a launch, or another retention, holds it or `dir`
  // has become another directory meanwhile; with `error` set when the lock
  // file cannot be opened or `dir` is gone.
  static std::optional<SessionDirectoryLock> TakeIdle(
      const std::filesystem::path& dir, bool& held, std::error_code& error) {
    held = false;
    const std::string path = (dir / kSessionDirectoryLockFile).string();
    const int fd = introspect::OpenLockFile(path, O_RDONLY);
    if (fd < 0) {
      error = {errno, std::generic_category()};
      return std::nullopt;
    }
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
      const int code = errno;
      ::close(fd);
      held = code == EWOULDBLOCK;
      if (!held) error = {code, std::generic_category()};
      return std::nullopt;
    }
    if (!introspect::LockFileIsCurrent(fd, path)) {
      ::close(fd);
      struct stat named {};
      held = ::lstat(path.c_str(), &named) == 0;
      if (!held)
        error = std::make_error_code(std::errc::no_such_file_or_directory);
      return std::nullopt;
    }
    return SessionDirectoryLock{fd};
  }

  ~SessionDirectoryLock() {
    if (fd_ >= 0) ::close(fd_);
  }

  SessionDirectoryLock(const SessionDirectoryLock&) = delete;
  SessionDirectoryLock& operator=(const SessionDirectoryLock&) = delete;
  SessionDirectoryLock(SessionDirectoryLock&& other) noexcept
      : fd_{std::exchange(other.fd_, -1)} {}
  SessionDirectoryLock& operator=(SessionDirectoryLock&& other) noexcept {
    if (this != &other) {
      if (fd_ >= 0) ::close(fd_);
      fd_ = std::exchange(other.fd_, -1);
    }
    return *this;
  }

 private:
  explicit SessionDirectoryLock(int fd) : fd_{fd} {}

  int fd_{-1};
};

}  // namespace talos::launcher
