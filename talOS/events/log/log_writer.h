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

#if defined(__APPLE__)
#include <dispatch/dispatch.h>
#else
#include <semaphore.h>
#endif

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
// without locking or waiting on the producer. The writer sleeps on a
// semaphore until a chunk is handed over or a sync is due; the producer posts
// it on every hand-off, asleep or not, and a post neither blocks nor takes a
// lock (sem_post on Linux, dispatch_semaphore_signal on macOS). The writer
// runs under the normal scheduling policy whatever the thread that
// started it runs under, so it never competes with a realtime loop thread
// for the CPU.
//
// Two losses are bounded separately. A process killed by a signal (the
// launcher's stall kill, a segfault, an abort) runs no flush, so its log
// keeps what the writer had already written to the file, which the kernel
// keeps: the writer writes the committed prefix of a chunk that is not full
// within Options::write_interval (10 ms) of its first unwritten record, so
// such a death loses about that much plus the writer's scheduling delay.
// While records arrive closer together than write_interval the writer wakes
// and writes once per write_interval (at most 100 times a second at the
// default, whatever the tick rate) and a record does not post; a record that
// finds the writer idle posts once and is written at once: one wakeup per
// record. write_interval never delays a full chunk: each hand-off wakes the
// writer, so a flood costs one wakeup per chunk. Once a write_interval passes
// with no record, or after a disk error, the writer sleeps until a record, a
// hand-off or the periodic sync. A power loss also loses what the kernel has
// not put on the disk: once per Options::sync_interval (a second) the writer
// fsyncs, which leaves the data in the drive's write cache on macOS. flush()
// and destruction sync with process::DurableSync, which also flushes that
// cache. If the pool fills, capture stops permanently and failed() becomes
// true. Previously accepted chunks are drained, preserving a prefix without
// holes.
//
// With the background writer the thread that runs the loop never waits on
// the log file: the writer thread creates `path` and writes the header,
// manifest and schema block before the first chunk, so nothing exists at
// `path` until start() and the writer have run (creating a file on a busy
// disk can take hundreds of milliseconds). A file it cannot create is a
// sticky failure like any other disk error: failed() names the path, and the
// records already handed over are dropped, so flush() and destruction return.
//
// A byte cap (Options::max_bytes) keeps the newest records: the log is
// written as up to kLogSegments segments, each a complete log of its own
// (header, manifest and schema block, then whole records). The segment being
// written is always at `path`; a full one is renamed to LogSegmentPath(path,
// k) and the oldest is deleted once the cap would be crossed, all on the
// writer thread. The producer only decides where a segment ends.
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
  // Without a background thread, the periodic sync is checked on append;
  // an idle synchronous writer must be flushed explicitly.
  bool background{true};

  // Bytes the log's segments may take together, each segment's header,
  // manifest and schema block included. Zero is unbounded (one file, never
  // rotated). Capped, the log is split into n = min(kLogSegments,
  // max_bytes / (prefix + largest record)) segments of max_bytes / n bytes;
  // when the newest is full it is renamed and a new one started, and once
  // n segments exist the oldest is deleted, so the log keeps the session's
  // tail. Deleting the first segment makes capped() true and creates
  // `<path>.capped` (CappedMarkerPath), so whoever reads the log afterwards
  // knows its head is missing. Where n < 2 the log cannot rotate: a record
  // that would cross the cap stops capture for good (capped()), keeping
  // every whole record before it. If the header/manifest cannot fit, no log
  // file is left; only the empty cap marker is created.
  std::uint64_t max_bytes{0};

  // How often the writer fsyncs the file, after writing the committed prefix
  // of the active chunk (see LogWriter): what a power loss can lose. Tests
  // shorten it; it must exceed one write and fsync, or a stopping writer
  // finds the next sync already due.
  std::chrono::nanoseconds sync_interval{std::chrono::seconds{1}};

  // How long a committed record may wait in a chunk that is not full before
  // the writer writes it to the file (see LogWriter): what a process killed
  // by a signal can lose, plus the writer's scheduling delay.
  std::chrono::nanoseconds write_interval{std::chrono::milliseconds{10}};
};

// The marker a byte-capped writer leaves next to its log (see max_bytes).
inline std::string CappedMarkerPath(std::string_view log_path) {
  return std::string{log_path} + ".capped";
}

// The most segments a byte-capped log is split into (see max_bytes). Once it
// rotates, the log holds n - 1 full segments and the one being written, so
// it never keeps less than about (n - 1) / n of its cap; each segment repeats
// the header, manifest and schema block.
inline constexpr std::uint64_t kLogSegments = 8;

class LogWriter {
 public:
  using Options = LogWriterOptions;

  static constexpr std::size_t kDefaultBufferBytes = DEFAULT_CHUNK_BYTES;

  // A log written to `path`, created or truncated. `process_name` is copied
  // into the file header, truncated to MAX_SOURCE_NAME characters. Without
  // Options::background this opens the file and throws std::system_error if
  // it cannot; with it, the writer thread opens it once start() runs.
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

  // Builds the file header, manifest and schema block. Without
  // Options::background it writes them and throws std::system_error if the
  // write fails; with it, it starts the writer thread, which creates the file
  // and writes them before any record, and reports a failure through
  // failed().
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

  // Pushes everything buffered to the file and syncs it to disk
  // (process::DurableSync) before returning. Destruction also drains and
  // syncs before closing the descriptor.
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

  // True once the log is missing records because of Options::max_bytes: its
  // oldest segment was deleted, or (where it cannot rotate) a record would
  // have crossed the cap.
  bool capped() const noexcept;

  std::uint64_t session_id() const noexcept;

  // How many times the writer thread has returned from a sleep (a post, a
  // write coming due or the periodic sync), whether or not it then found
  // work: what REQ-RT-21 bounds. A write that blocks is not counted.
  std::uint64_t wakeups() const noexcept;

 private:
  // A counting semaphore: post() never blocks, never takes a lock and is never
  // lost -- a post before the wait makes the wait return at once.
  class Wakeup {
   public:
    Wakeup() noexcept;
    ~Wakeup();
    Wakeup(const Wakeup&) = delete;
    Wakeup& operator=(const Wakeup&) = delete;

    void post() noexcept;
    // Returns after a post (consuming it) or at `deadline`.
    void wait_until(std::chrono::steady_clock::time_point deadline) noexcept;

   private:
#if defined(__APPLE__)
    dispatch_semaphore_t semaphore_;
#else
    sem_t semaphore_{};
#endif
  };

  struct Chunk {
    std::vector<std::byte> data;
    std::size_t used{0};
    std::atomic<std::size_t> committed{0};
    std::size_t written{0};  // Consumer only.
    // The log segment its records belong to. Set by the producer when it
    // takes the chunk, before any record is committed to it.
    std::uint64_t segment{0};
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
    // Writer side: ends the file's segment and starts `segment` at `path`.
    void rotate_to(std::uint64_t segment) noexcept;
    void write_all(const void* data, std::size_t size);
    // `durable`: process::DurableSync (flush(), destruction); otherwise
    // fsync (the periodic sync).
    void sync(bool durable) noexcept;
    void mark_capped() noexcept;
    void drain() noexcept;
    // Writer thread, first: creates `path` and writes the prefix, or (with
    // no prefix: start() found no room for one) only the cap marker.
    void open_file() noexcept;
    void start_writer();
    void stop_writer() noexcept;
    void set_error(std::string message) noexcept;

    int fd{-1};
    std::string path;
    std::string process_name;
    std::uint64_t session_id{0};
    Options options;
    // The header, manifest and schema block every segment starts with.
    std::vector<std::byte> prefix;
    // Bytes per segment; zero when the log does not rotate.
    std::uint64_t segment_bytes{0};
    std::uint64_t segment_count{1};
    // Producer only, after start(): the segment records are accepted into,
    // and its bytes so far, prefix included.
    std::uint64_t segment{0};
    std::uint64_t segment_used{0};
    // Writer only (or the only thread): the segment `fd` holds.
    std::uint64_t file_segment{0};
    // Writer only: the oldest segment rotation has not deleted yet.
    std::uint64_t dropped_through{0};
    std::atomic<bool> head_dropped{false};
    bool capped_marked{false};  // Writer thread, or drain after it is idle.

    std::deque<Chunk> pool;
    std::chrono::steady_clock::time_point last_sync{
        std::chrono::steady_clock::now()};
    // Writer only: when it last wrote (see Options::write_interval).
    std::chrono::steady_clock::time_point last_write{};
    Chunk* active{nullptr};
    std::atomic<std::uint64_t> submitted{0};
    std::atomic<std::uint64_t> completed{0};
    // Set by the writer before it checks for work and sleeps on `wake`; the
    // producer posts `wake` for a record only when it finds it set (and for
    // every hand-off), clearing it. Both
    // sides use sequentially consistent operations on this, `submitted` and
    // a chunk's `committed`, so either the writer sees the submission or
    // record, or the producer sees the writer idle and posts, and the
    // semaphore keeps a post that lands before the wait.
    std::atomic<bool> writer_idle{false};
    std::atomic<std::uint64_t> wakeups{0};  // See LogWriter::wakeups().
    Wakeup wake;

    enum class CaptureError { NONE, POOL_FULL, RECORD_TOO_LARGE, BYTE_CAP };
    std::atomic<CaptureError> capture_error{CaptureError::NONE};

    mutable std::mutex mutex;
    std::condition_variable chunk_ready;
    // Under `mutex`: the writer thread has finished open_file(), so drain()
    // syncs the file it made, not one still being created.
    bool file_ready{false};
    std::thread writer;
    std::atomic<bool> stopping{false};
    std::atomic<bool> failed{false};  // Disk failure, independent of capture.
    // Error reporting also runs while the worker coordination mutex is held
    // (sync in drain/periodic sync), so it needs independent protection.
    mutable std::mutex error_mutex;
    std::string error;
  };

  std::unique_ptr<State> state_;
};

static_assert(RecorderPolicy<LogWriter>);

}  // namespace talos::event::log
