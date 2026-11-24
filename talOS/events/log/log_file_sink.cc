#include "talOS/events/log/log_file_sink.h"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <system_error>
#include <utility>

#include "talOS/events/log/format.h"
#include "talOS/process/durable_sync.h"

namespace talos::event::log {

LogFileSink::LogFileSink(std::string path, LogFileSinkOps ops)
    : path_{std::move(path)}, ops_{std::move(ops)} {}

LogFileSink::~LogFileSink() { close(); }

void LogFileSink::configure_segments(std::vector<std::byte> prefix,
                                     std::uint64_t segment_count) noexcept {
  prefix_ = std::move(prefix);
  segment_count_ = segment_count;
}

int LogFileSink::sys_open(const char* path, int flags,
                          mode_t mode) const noexcept {
  if (ops_.open) return ops_.open(path, flags, mode);
  if ((flags & O_CREAT) != 0) return ::open(path, flags, mode);
  return ::open(path, flags);
}

int LogFileSink::sys_close(int fd) const noexcept {
  if (ops_.close) return ops_.close(fd);
  return ::close(fd);
}

int LogFileSink::sys_durable_sync(int fd) const noexcept {
  if (ops_.durable_sync) return ops_.durable_sync(fd);
  return process::DurableSync(fd);
}

int LogFileSink::sys_periodic_sync(int fd) const noexcept {
  if (ops_.periodic_sync) return ops_.periodic_sync(fd);
  int result = 0;
  do {
    result = ::fsync(fd);
  } while (result < 0 && errno == EINTR);
  return result;
}

int LogFileSink::sys_rename(const char* from, const char* to) const noexcept {
  if (ops_.rename) return ops_.rename(from, to);
  return ::rename(from, to);
}

int LogFileSink::sys_unlink(const char* path) const noexcept {
  if (ops_.unlink) return ops_.unlink(path);
  return ::unlink(path);
}

void LogFileSink::set_error(std::string message) noexcept {
  std::lock_guard lock{error_mutex_};
  failed_.store(true);
  if (error_.empty()) {
    error_ = std::move(message);
  }
}

std::string LogFileSink::error() const {
  std::lock_guard lock{error_mutex_};
  return error_;
}

void LogFileSink::write_all_to(int target_fd, const void* data,
                               std::size_t size) {
  const auto* bytes = reinterpret_cast<const char*>(data);
  std::size_t written = 0;
  while (written < size) {
    const ssize_t n = ::write(target_fd, bytes + written, size - written);
    if (n < 0) {
      if (errno == EINTR) continue;
      throw std::system_error(errno, std::generic_category(),
                              "LogWriter: write failed");
    }
    written += static_cast<std::size_t>(n);
  }
}

void LogFileSink::open_sync() {
  fd_ = sys_open(path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(),
                            "LogWriter: open failed for " + path_);
  }
  needs_durable_sync_ = true;
}

void LogFileSink::write_prefix_sync() {
  write_all_to(fd_, prefix_.data(), prefix_.size());
  needs_durable_sync_ = true;
}

void LogFileSink::abort_for_tiny_cap_sync() noexcept {
  close();
  sys_unlink(path_.c_str());
  mark_capped();
}

void LogFileSink::open_file() noexcept {
  if (prefix_.empty()) {
    mark_capped();
    return;
  }
  fd_ = sys_open(path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd_ < 0) {
    // Sticky: append() accepts nothing more, and write_chunk() drops what
    // was already handed over, so drain() and destruction still return.
    set_error("LogWriter: open failed for " + path_ + ": " +
              std::strerror(errno));
    return;
  }
  needs_durable_sync_ = true;
  try {
    write_all_to(fd_, prefix_.data(), prefix_.size());
  } catch (const std::system_error& e) {
    set_error(e.what());
  }
}

std::size_t LogFileSink::write_bytes(const void* data,
                                     std::size_t size) noexcept {
  if (fd_ < 0) return 0;
  const auto* bytes = reinterpret_cast<const std::byte*>(data);
  std::size_t total = 0;
  while (total < size) {
    const ssize_t n = ::write(fd_, bytes + total, size - total);
    if (n < 0) {
      if (errno == EINTR) continue;
      set_error(std::string{"LogWriter: flush failed: "} +
                std::strerror(errno));
      if (total > 0) needs_durable_sync_ = true;
      return total;
    }
    total += static_cast<std::size_t>(n);
  }
  if (total > 0) needs_durable_sync_ = true;
  return total;
}

// Syncs the directory holding `path_`, so renames, creations and unlinks in
// it survive a power loss. 0, or -1 with errno set.
int LogFileSink::sync_directory() const noexcept {
  const auto slash = path_.rfind('/');
  const std::string directory = slash == std::string::npos
                                    ? "."
                                    : path_.substr(0, slash == 0 ? 1 : slash);
  const int dir_fd = sys_open(directory.c_str(), O_RDONLY | O_CLOEXEC, 0);
  if (dir_fd < 0) return -1;
  const int result = sys_durable_sync(dir_fd);
  const int code = errno;
  sys_close(dir_fd);
  errno = code;
  return result;
}

// A rename and a fresh file rather than a rewrite: every segment stays an
// append-only log that a power loss can only shorten. The order makes every
// cut leave whole segments: the finished one is synced durably; the next one's
// prefix is written and synced under a name no reader takes for a log
// (`.next`) before it is renamed into place; the cap marker is durable before
// the oldest segment goes; the directory is synced last.
void LogFileSink::rotate_to(std::uint64_t next) noexcept {
  sync(/*durable=*/true);
  close();
  if (failed_.load()) return;
  const auto fail = [this](std::string_view step, const std::string& what) {
    set_error("LogWriter: cannot " + std::string{step} + " " + what + ": " +
              std::strerror(errno));
  };

  const std::string staged = path_ + ".next";
  const int next_fd =
      sys_open(staged.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (next_fd < 0) return fail("open", staged);
  try {
    write_all_to(next_fd, prefix_.data(), prefix_.size());
  } catch (const std::system_error& e) {
    set_error(e.what());
  }
  if (failed_.load() || sys_durable_sync(next_fd) != 0) {
    if (!failed_.load()) fail("sync", staged);
    sys_close(next_fd);
    sys_unlink(staged.c_str());
    return;
  }

  // At most segment_count_ segments, the new one included: those the last
  // rotation kept that now fall out are deleted, after the marker that says
  // the log lost its head is on disk.
  const bool drop = next >= segment_count_;
  if (drop && !capped_marked_) {
    mark_capped();
    if (failed_.load() || !capped_marked_ || sync_directory() != 0) {
      if (!failed_.load()) fail("sync the directory of", path_);
      sys_close(next_fd);
      sys_unlink(staged.c_str());
      return;
    }
    head_dropped_.store(true);
  }
  if (sys_rename(path_.c_str(), LogSegmentPath(path_, file_segment_).c_str()) !=
      0) {
    fail("rename", path_);
    sys_close(next_fd);
    sys_unlink(staged.c_str());
    return;
  }
  if (sys_rename(staged.c_str(), path_.c_str()) != 0) {
    fail("rename", staged);
    sys_close(next_fd);
    sys_unlink(staged.c_str());
    return;
  }
  fd_ = next_fd;
  file_segment_ = next;
  needs_durable_sync_ = true;
  if (drop) {
    for (std::uint64_t old = dropped_through_; old <= next - segment_count_;
         ++old) {
      sys_unlink(LogSegmentPath(path_, old).c_str());
    }
    dropped_through_ = next - segment_count_ + 1;
  }
  if (sync_directory() != 0) fail("sync the directory of", path_);
}

void LogFileSink::sync(bool durable, bool capture_stopped) noexcept {
  if (fd_ < 0) {
    // No file (never created, or lost by a failed rotation): nothing to sync,
    // but the periodic sync is still done, or the writer thread would find
    // it due on every pass, spin, and never see `stopping`.
    last_sync_ = std::chrono::steady_clock::now();
    return;
  }
  const int result = durable ? sys_durable_sync(fd_) : sys_periodic_sync(fd_);
  if (result < 0) {
    set_error(std::string{"LogWriter: fsync failed: "} + std::strerror(errno));
  }
  last_sync_ = std::chrono::steady_clock::now();
  const bool need_cap = head_dropped_.load() || capture_stopped;
  if (need_cap) {
    mark_capped();
  }
  if (durable && result == 0 && (!need_cap || capped_marked_)) {
    needs_durable_sync_ = false;
  }
}

void LogFileSink::mark_capped() noexcept {
  if (capped_marked_) return;
  needs_durable_sync_ = true;
  const std::string marker = CappedMarkerPath(path_);
  const int marker_fd =
      sys_open(marker.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (marker_fd < 0) {
    set_error(std::string{"LogWriter: cannot create "} + marker + ": " +
              std::strerror(errno));
    return;
  }
  const int synced = sys_durable_sync(marker_fd);
  if (synced != 0) {
    set_error(std::string{"LogWriter: cannot sync "} + marker + ": " +
              std::strerror(errno));
  }
  sys_close(marker_fd);
  if (synced == 0) {
    capped_marked_ = true;
  }
}

void LogFileSink::close() noexcept {
  if (fd_ >= 0) {
    sys_close(fd_);
    fd_ = -1;
  }
}

}  // namespace talos::event::log
