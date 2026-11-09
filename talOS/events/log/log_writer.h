#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/events/log/format.h"
#include "talOS/events/manifest.h"
#include "talOS/events/recorder.h"
#include "talOS/events/time.h"

namespace talos::event::log {

// Writes a dispatch log in the on-disk format described by format.h.
//
// Records are copied into preallocated chunks. A
// single-producer/single-consumer ring hands chunks to the background writer
// without locking or waiting on the producer. Once per second the writer also
// writes the committed prefix of the active chunk before fsync, so quiet logs
// do not remain buffered indefinitely. If the pool fills, capture stops
// permanently and failed() becomes true. Previously accepted chunks are
// drained, preserving a prefix without holes.
//
// dispatch()/send()/finish() never throw: a full disk or a transient
// EIO must not bring down a control loop that is otherwise healthy. Such
// failures are sticky and surfaced through failed()/error() instead.
inline constexpr std::size_t DEFAULT_CHUNK_BYTES = 256u << 10;
inline constexpr std::size_t DEFAULT_CHUNK_COUNT = 4;

struct LogWriterOptions {
  // Minimum buffer size. start() grows all chunks to fit the manifest's largest
  // message; records exceeding that capacity fail capture without allocating.
  std::size_t chunk_bytes{DEFAULT_CHUNK_BYTES};

  // How many buffers may be in flight before capture fails.
  std::size_t chunk_count{DEFAULT_CHUNK_COUNT};

  // Write from a background thread. Turn off for tests that want writes to
  // land synchronously, or for a process where an extra thread is not wanted.
  // Without a background thread, the one-second sync is checked on append;
  // an idle synchronous writer must be flushed explicitly.
  bool background{true};

  // Bytes the file may grow to: header, manifest and schema block included.
  // A record that would cross the cap stops capture for good (capped()):
  // the file keeps every whole record accepted before it, so it still reads,
  // and `<path>.capped` (CappedMarkerPath) is created within about a second
  // and at close, so whoever reads the file afterwards knows it is
  // incomplete. Zero is unbounded. If the header/manifest cannot fit, no log
  // is created; only the empty cap marker remains.
  std::uint64_t max_bytes{0};
};

// The marker a byte-capped writer leaves next to its log (see max_bytes).
inline std::string CappedMarkerPath(std::string_view log_path) {
  return std::string{log_path} + ".capped";
}

class LogWriter {
 public:
  using Options = LogWriterOptions;

  static constexpr std::size_t kDefaultBufferBytes = DEFAULT_CHUNK_BYTES;

  // Opens (creating or truncating) `path` for writing. `process_name` is
  // copied into the file header, truncated to MAX_SOURCE_NAME characters.
  // Throws std::system_error if the file cannot be opened.
  explicit LogWriter(std::string_view path, std::string_view process_name,
                     Options options = Options{}, std::uint64_t session_id = 0);

  // Convenience overload: one chunk of `buffer_bytes`, written synchronously.
  LogWriter(std::string_view path, std::string_view process_name,
            std::size_t buffer_bytes, std::uint64_t session_id = 0);

  ~LogWriter();

  LogWriter(const LogWriter&) = delete;
  LogWriter& operator=(const LogWriter&) = delete;
  LogWriter(LogWriter&& other) noexcept = default;
  LogWriter& operator=(LogWriter&& other) noexcept = default;

  // Writes the file header and manifest block, then starts the writer thread.
  // Throws std::system_error if the write fails.
  void start(const Manifest& manifest, MonotonicTime start_time);

  // MESSAGE dispatches preserve the producer's publish time independently of
  // receipt time. SEND records use parent.now, the outgoing publication time.
  void dispatch(const Context& context,
                std::span<const std::byte> payload) noexcept;
  void send(const Context& parent, std::uint16_t source_id,
            std::span<const std::byte> payload, std::uint32_t status,
            std::uint64_t sequence) noexcept;
  // Records loop completion. EXIT may be readable before this writer or its
  // owning process is destroyed; it does not acknowledge resource release.
  void finish(const Context& context) noexcept;

  // Pushes everything buffered to the file and fsyncs it before returning.
  // Destruction also drains and fsyncs before closing the descriptor.
  // Safe to call repeatedly, including after a failure.
  void flush() noexcept;

  // True once anything has gone wrong: the disk refused a write, or capture
  // stopped because the writer fell behind. Treat it as a fault on the robot;
  // a log that stops early is a log you cannot replay the rest of the match
  // from. Reaching the configured byte cap is policy, not a fault: it shows
  // in capped(), not here.
  bool failed() const noexcept;
  std::string error() const;

  // True when capture was abandoned -- the loop outran the writer, a record
  // was too large, or the byte cap was reached -- as opposed to a disk error.
  // Sticky, because capture never resumes: doing so would leave a hole in the
  // middle of the log and replay would silently skip it.
  //
  // This never waits and the producer never blocks on it, which is the whole
  // point: a control loop must not stall behind a slow disk.
  bool capture_stopped() const noexcept;

  // True once a record would have crossed Options::max_bytes.
  bool capped() const noexcept;

  std::uint64_t session_id() const noexcept;

 private:
  struct Chunk {
    std::vector<std::byte> data;
    std::size_t used{0};
    std::atomic<std::size_t> committed{0};
    std::size_t written{0};  // Consumer only.
  };

  // All state lives behind a pointer so that moving a LogWriter never has to
  // move a running thread, a mutex or a condition variable.
  struct State {
    ~State();

    void append(RecordHeader header,
                std::span<const std::byte> payload) noexcept;
    void submit_active() noexcept;
    void acquire_chunk() noexcept;
    void write_chunk(Chunk& chunk) noexcept;
    void write_all(const void* data, std::size_t size);
    void sync() noexcept;
    void mark_capped() noexcept;
    void drain() noexcept;
    void stop_writer() noexcept;
    void set_error(std::string message) noexcept;

    int fd{-1};
    std::string path;
    std::string process_name;
    std::uint64_t session_id{0};
    Options options;
    // File bytes accepted so far. Producer only, after start().
    std::uint64_t accepted_bytes{0};
    bool capped_marked{false};  // Under mutex (or the only thread).

    std::deque<Chunk> pool;
    std::chrono::steady_clock::time_point last_sync{
        std::chrono::steady_clock::now()};
    Chunk* active{nullptr};
    std::atomic<std::uint64_t> submitted{0};
    std::atomic<std::uint64_t> completed{0};

    enum class CaptureError { NONE, POOL_FULL, RECORD_TOO_LARGE, BYTE_CAP };
    std::atomic<CaptureError> capture_error{CaptureError::NONE};

    mutable std::mutex mutex;
    std::condition_variable pending_ready;
    std::condition_variable chunk_ready;
    std::thread writer;
    bool stopping{false};
    std::atomic<bool> failed{false};  // Disk failure, independent of capture.
    // Error reporting also runs while the worker coordination mutex is held
    // (fsync in drain/periodic sync), so it needs independent protection.
    mutable std::mutex error_mutex;
    std::string error;
  };

  std::unique_ptr<State> state_;
};

static_assert(RecorderPolicy<LogWriter>);

}  // namespace talos::event::log
