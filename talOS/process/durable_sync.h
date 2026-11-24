#pragma once

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>

namespace talos::process {

// Syncs an open file or directory so that it survives a power loss: on macOS
// fcntl(F_FULLFSYNC), because fsync() there leaves the data in the drive's
// write cache, falling back to fsync() only when the filesystem reports that
// F_FULLFSYNC is unsupported (ENOTSUP, EOPNOTSUPP, EINVAL, ENOTTY); fsync()
// elsewhere. Retries EINTR. 0, or -1 with errno set by F_FULLFSYNC / fsync().
inline int DurableSync(int fd) noexcept {
  int result;
#ifdef __APPLE__
  do {
    result = ::fcntl(fd, F_FULLFSYNC);
  } while (result < 0 && errno == EINTR);
  if (result == 0) return 0;
  if (errno != ENOTSUP && errno != EOPNOTSUPP && errno != EINVAL &&
      errno != ENOTTY) {
    return -1;
  }
#endif
  do {
    result = ::fsync(fd);
  } while (result < 0 && errno == EINTR);
  return result;
}

}  // namespace talos::process
