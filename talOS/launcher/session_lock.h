#pragma once

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
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

}  // namespace talos::launcher
