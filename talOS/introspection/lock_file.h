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
#include <string>
#include <string_view>

namespace talos::introspect {

// Every lock file's mode, whatever the creator's umask: any user that may
// register a node or run a launcher can open it. A lock file holds no data.
inline constexpr mode_t kLockFileMode = 0666;

// Where lock files live. On Linux, /dev/shm, beside the segments themselves:
// every process that shares the segments shares the directory (a service with
// systemd's PrivateTmp= has a /tmp of its own, not a /dev/shm), and the tmpfs
// is emptied at boot with them. macOS keeps shared memory in the kernel, so
// there it is /tmp, which launchd daemons and logged-in users share. Both are
// sticky and world-writable; OpenLockFile is why that is safe.
inline const char* LockDirectory() {
#if defined(__linux__)
  struct stat status {};
  if (::stat("/dev/shm", &status) == 0 && S_ISDIR(status.st_mode)) {
    return "/dev/shm";
  }
#endif
  return "/tmp";
}

// The lock file of the shared-memory object `mapped` (a ShmObjectName
// result): talos-<mapped name>.lock. No segment name contains '-', so a lock
// file never takes a segment's name in /dev/shm.
inline std::string LockFilePath(std::string_view mapped) {
  while (mapped.starts_with('/')) mapped.remove_prefix(1);
  return std::string{LockDirectory()} + "/talos-" + std::string{mapped} +
         ".lock";
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

// "owner uid U, mode 0MMM" of a file, for a diagnostic.
inline std::string DescribeOwner(const struct stat& status) {
  char mode[8];
  std::snprintf(mode, sizeof(mode), "%04o",
                static_cast<unsigned>(status.st_mode & 07777));
  return "owner uid " + std::to_string(status.st_uid) + ", mode " + mode;
}

}  // namespace talos::introspect
