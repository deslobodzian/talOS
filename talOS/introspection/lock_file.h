#pragma once

// Lock files beside a shared-memory namespace: the registry's writer lock
// (registry.h) and the launcher's one-session-per-namespace lock. Both are
// named after the ShmObjectName of what they guard, so TALOS_SHM_NAMESPACE and
// Bazel's TEST_TMPDIR isolate a lock exactly as they isolate its segment.

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>

namespace talos::introspect {

// Every lock file's mode, whatever the creator's umask: any user that may
// register a node or run a launcher can open it. A lock file holds no data.
inline constexpr mode_t kLockFileMode = 0666;

// Where lock files live. Under Bazel's TEST_TMPDIR namespace (no
// TALOS_SHM_NAMESPACE, the same precedence as ShmObjectName), the test's own
// temporary directory: it is the namespace, every process in it inherits it,
// and Bazel removes it with the test, so a test run's lock files do not
// accumulate until reboot. Otherwise, on Linux, /dev/shm, beside the segments
// themselves: every process that shares the segments shares the directory (a
// service with systemd's PrivateTmp= has a /tmp of its own, not a /dev/shm),
// and the tmpfs is emptied at boot with them. macOS keeps shared memory in the
// kernel, so there it is /tmp, which launchd daemons and logged-in users share
// and which is emptied at boot; its tmp_cleaner deletes files whose atime,
// mtime and ctime are all older than 3 days, so a lock held longer is touched
// by its holder (TouchLockFile) and every acquisition checks it locked the
// file the path still names (LockFileIsCurrent). /dev/shm and /tmp are sticky
// and world-writable; OpenLockFile is why that is safe.
inline const std::string& LockDirectory() {
  static const std::string directory = [] {
    if (std::getenv("TALOS_SHM_NAMESPACE") == nullptr) {
      if (const char* test = std::getenv("TEST_TMPDIR");
          test != nullptr && *test != '\0') {
        return std::string{test};
      }
    }
#if defined(__linux__)
    struct stat status {};
    if (::stat("/dev/shm", &status) == 0 && S_ISDIR(status.st_mode)) {
      return std::string{"/dev/shm"};
    }
#endif
    return std::string{"/tmp"};
  }();
  return directory;
}

// The lock file of the shared-memory object `mapped` (a ShmObjectName
// result): talos-<mapped name>.lock. No segment name contains '-', so a lock
// file never takes a segment's name in /dev/shm.
inline std::string LockFilePath(std::string_view mapped) {
  while (mapped.starts_with('/')) mapped.remove_prefix(1);
  return LockDirectory() + "/talos-" + std::string{mapped} + ".lock";
}

// Opens a lock file for `access` (O_RDONLY or O_RDWR), creating it when
// absent; -1 with errno set on failure. Linux's fs.protected_regular (on by
// default in most distributions) refuses O_CREAT on an existing file that
// another user owns in a sticky directory, even to root, so an existing file
// is opened without O_CREAT, and a create that loses a race to another
// process opens the winner's file. A file this process's user owns is given
// kLockFileMode whatever umask created it, so another user's process can
// still open it.
inline int OpenLockFile(const std::string& path, int access) {
  for (int attempt = 0; attempt < 100; ++attempt) {
    int fd = ::open(path.c_str(), access | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0 && errno == ENOENT) {
      fd = ::open(path.c_str(),
                  access | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                  kLockFileMode);
      // Created by someone else in between, or removed again: open it anew.
      if (fd < 0 && errno == EEXIST) continue;
    }
    if (fd < 0) return -1;
    struct stat status {};
    if (::fstat(fd, &status) == 0 && status.st_uid == ::geteuid() &&
        (status.st_mode & 07777) != kLockFileMode) {
      // Best effort: a mode this cannot fix is reported by whoever it stops.
      (void)::fchmod(fd, kLockFileMode);
    }
    return fd;
  }
  errno = EEXIST;
  return -1;
}

// True while `path` names the file open on `fd`. A lock taken on a file that
// was unlinked meanwhile (by its last holder never, but by a /tmp cleaner, or
// by hand) excludes nobody who opens the path anew, so an acquirer that finds
// this false closes the descriptor and acquires again.
inline bool LockFileIsCurrent(int fd, const std::string& path) {
  struct stat held {};
  struct stat named {};
  return ::fstat(fd, &held) == 0 && ::lstat(path.c_str(), &named) == 0 &&
         held.st_dev == named.st_dev && held.st_ino == named.st_ino;
}

// Marks a lock file just used, so a cleaner of unused /tmp files (macOS's
// tmp_cleaner: atime, mtime and ctime all older than 3 days) passes it over
// while it is held. Needs the file's owner or write access, which kLockFileMode
// gives.
inline bool TouchLockFile(int fd) { return ::futimens(fd, nullptr) == 0; }

// "owner uid U, mode 0MMM" of a file, for a diagnostic.
inline std::string DescribeOwner(const struct stat& status) {
  char mode[8];
  std::snprintf(mode, sizeof(mode), "%04o",
                static_cast<unsigned>(status.st_mode & 07777));
  return "owner uid " + std::to_string(status.st_uid) + ", mode " + mode;
}

}  // namespace talos::introspect
