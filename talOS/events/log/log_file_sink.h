#pragma once

#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace talos::event::log {

// The marker a byte-capped writer leaves next to its log (see
// LogWriterOptions::max_bytes).
inline std::string CappedMarkerPath(std::string_view log_path) {
  return std::string{log_path} + ".capped";
}

// Optional OS syscall seams for unit tests (e.g. verifying rotate_to's durable
// sync ordering without mocking unrelated layers). Empty fields call the
// corresponding POSIX / process::DurableSync functions directly.
struct LogFileSinkOps {
  std::function<int(const char* path, int flags, mode_t mode)> open;
  std::function<int(int fd)> close;
  std::function<int(int fd)> durable_sync;
  std::function<int(int fd)> periodic_sync;
  std::function<int(const char* from, const char* to)> rename;
  std::function<int(const char* path)> unlink;
};

// Owns the on-disk log file descriptor, segment rotation, durable/periodic
// syncing, byte-cap marker creation, and sticky disk error state.
//
// All file I/O methods run on the writer thread (or inline when background
// writing is disabled); failed(), error(), and head_dropped() are thread-safe.
class LogFileSink {
 public:
  explicit LogFileSink(std::string path, LogFileSinkOps ops = {});
  ~LogFileSink();

  LogFileSink(const LogFileSink&) = delete;
  LogFileSink& operator=(const LogFileSink&) = delete;

  // Stores the per-segment prefix (FileHeader + manifest + schema block) and
  // the maximum number of retained segments before the oldest is unlinked.
  void configure_segments(std::vector<std::byte> prefix,
                          std::uint64_t segment_count) noexcept;

  // Synchronous path: opens `path` (throwing std::system_error on failure),
  // writes the configured prefix (throwing std::system_error on failure), or
  // removes the empty file and creates `<path>.capped` when the prefix cannot
  // fit the cap.
  void open_sync();
  void write_prefix_sync();
  void abort_for_tiny_cap_sync() noexcept;

  // Background writer thread entry: creates `path` and writes the prefix, or
  // (when prefix is empty because start() found no room for one) creates only
  // the cap marker. Failures are sticky (failed()/error()), never thrown.
  void open_file() noexcept;

  // Writes `size` bytes to the active descriptor, retrying EINTR. Returns the
  // number of bytes written; on error records a sticky failure and stops.
  std::size_t write_bytes(const void* data, std::size_t size) noexcept;

  // Ends the active segment and starts `next` at `path`, keeping every power
  // cut leaving whole segments: durable-syncs the finished segment, writes and
  // durable-syncs `<path>.next`, durable-syncs `<path>.capped` and the parent
  // directory before unlinking the oldest segment once `next >= segment_count`,
  // renames finished -> segment<k> and `.next` -> `path`, unlinks expired
  // segments, and durable-syncs the parent directory last.
  void rotate_to(std::uint64_t next) noexcept;

  // Syncs the active descriptor (`durable`: process::DurableSync for flush()
  // and destruction; otherwise periodic fsync), updates last_sync(), and
  // creates `<path>.capped` once head_dropped() or `capture_stopped` holds.
  // A successful durable sync clears needs_durable_sync().
  void sync(bool durable, bool capture_stopped = false) noexcept;

  // Creates and durable-syncs `<path>.capped` once.
  void mark_capped() noexcept;

  // Closes the active descriptor if open.
  void close() noexcept;

  const std::string& path() const noexcept { return path_; }
  std::size_t prefix_size() const noexcept { return prefix_.size(); }
  std::uint64_t file_segment() const noexcept { return file_segment_; }
  std::chrono::steady_clock::time_point last_sync() const noexcept {
    return last_sync_;
  }
  bool needs_durable_sync() const noexcept { return needs_durable_sync_; }

  bool failed() const noexcept { return failed_.load(); }
  std::string error() const;
  bool head_dropped() const noexcept { return head_dropped_.load(); }

 private:
  int sys_open(const char* path, int flags, mode_t mode) const noexcept;
  int sys_close(int fd) const noexcept;
  int sys_durable_sync(int fd) const noexcept;
  int sys_periodic_sync(int fd) const noexcept;
  int sys_rename(const char* from, const char* to) const noexcept;
  int sys_unlink(const char* path) const noexcept;
  int sync_directory() const noexcept;
  void write_all_to(int target_fd, const void* data, std::size_t size);
  void set_error(std::string message) noexcept;

  std::string path_;
  LogFileSinkOps ops_;
  int fd_{-1};
  std::vector<std::byte> prefix_;
  std::uint64_t segment_count_{1};
  std::uint64_t file_segment_{0};
  std::uint64_t dropped_through_{0};
  std::atomic<bool> head_dropped_{false};
  bool capped_marked_{false};
  bool needs_durable_sync_{false};
  std::chrono::steady_clock::time_point last_sync_{
      std::chrono::steady_clock::now()};
  std::atomic<bool> failed_{false};
  mutable std::mutex error_mutex_;
  std::string error_;
};

}  // namespace talos::event::log
