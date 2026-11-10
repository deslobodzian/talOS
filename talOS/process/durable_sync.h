#pragma once

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>

namespace talos::process {

// Syncs an open file or directory so that it survives a power loss: on macOS
// fcntl(F_FULLFSYNC), because fsync() there leaves the data in the drive's
// write cache, falling back to fsync() where the filesystem refuses it;
// fsync() elsewhere. Retries EINTR. 0, or -1 with errno set by fsync().
inline int DurableSync(int fd) noexcept {
  int result;
#ifdef __APPLE__
  do {
    result = ::fcntl(fd, F_FULLFSYNC);
  } while (result < 0 && errno == EINTR);
  if (result == 0) return 0;
#endif
  do {
    result = ::fsync(fd);
  } while (result < 0 && errno == EINTR);
  return result;
}

}  // namespace talos::process
