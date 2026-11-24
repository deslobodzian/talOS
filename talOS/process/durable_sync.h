#pragma once

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>

namespace talos::process {

namespace detail {

// Runs `full_sync()` (e.g. fcntl(fd, F_FULLFSYNC) on macOS), retrying EINTR.
// Falls back to `fsync_fn()` (retrying EINTR) only when `full_sync()` fails
// because full sync is unsupported (ENOTSUP, EOPNOTSUPP, EINVAL, ENOTTY).
// Real errors (EIO, ENOSPC, EBADF, ...) return -1 with errno preserved and
// never invoke `fsync_fn()`.
template <typename FullSyncFn, typename FsyncFn>
inline int DurableSyncWithFallback(FullSyncFn&& full_sync,
                                   FsyncFn&& fsync_fn) noexcept {
  int result = 0;
  do {
    result = full_sync();
  } while (result < 0 && errno == EINTR);
  if (result == 0) return 0;
  const int err = errno;
  if (err != ENOTSUP && err != EOPNOTSUPP && err != EINVAL && err != ENOTTY) {
    errno = err;
    return -1;
  }
  do {
    result = fsync_fn();
  } while (result < 0 && errno == EINTR);
  return result;
}

}  // namespace detail

// Syncs an open file or directory so that it survives a power loss: on macOS
// fcntl(F_FULLFSYNC), because fsync() there leaves the data in the drive's
// write cache, falling back to fsync() only when the filesystem reports that
// F_FULLFSYNC is unsupported (ENOTSUP, EOPNOTSUPP, EINVAL, ENOTTY); fsync()
// elsewhere. Retries EINTR. 0, or -1 with errno set by F_FULLFSYNC / fsync().
inline int DurableSync(int fd) noexcept {
#ifdef __APPLE__
  return detail::DurableSyncWithFallback(
      [fd]() noexcept { return ::fcntl(fd, F_FULLFSYNC); },
      [fd]() noexcept { return ::fsync(fd); });
#else
  int result = 0;
  do {
    result = ::fsync(fd);
  } while (result < 0 && errno == EINTR);
  return result;
#endif
}

}  // namespace talos::process
