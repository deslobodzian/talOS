#include "talOS/events/log/log_writer.h"

#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <system_error>
#include <utility>

#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/thread_policy.h>
#else
#include <sched.h>
#endif

#include "talOS/process/durable_sync.h"

namespace talos::event::log {

namespace {

std::span<const std::byte> as_bytes(const void* data, std::size_t size) {
  return {reinterpret_cast<const std::byte*>(data), size};
}

// Puts the calling thread (the writer) under the normal time-sharing policy.
// A thread created by one running under process::ConfigureRealtime inherits
// SCHED_FIFO on Linux; the writer must not preempt control loops to write a
// log. Best effort: leaving realtime needs no privilege, and a refusal is
// reported to stderr, not as a log fault.
void UseNormalScheduling() noexcept {
#if defined(__APPLE__)
  // Whether or not a time-constraint policy reached this thread, the
  // standard policy replaces it.
  thread_standard_policy_data_t policy{};
  const kern_return_t result = ::thread_policy_set(
      ::pthread_mach_thread_np(::pthread_self()), THREAD_STANDARD_POLICY,
      reinterpret_cast<thread_policy_t>(&policy), THREAD_STANDARD_POLICY_COUNT);
  if (result != KERN_SUCCESS) {
    std::fprintf(stderr, "talos: log writer kept its scheduling policy (%d)\n",
                 static_cast<int>(result));
  }
#else
  int current = SCHED_OTHER;
  sched_param param{};
  if (::pthread_getschedparam(::pthread_self(), &current, &param) == 0 &&
      current == SCHED_OTHER) {
    return;
  }
  param = sched_param{};
  param.sched_priority = 0;
  const int result =
      ::pthread_setschedparam(::pthread_self(), SCHED_OTHER, &param);
  if (result != 0) {
    std::fprintf(stderr, "talos: log writer kept its scheduling policy (%s)\n",
                 std::strerror(result));
  }
#endif
}

}  // namespace

#if defined(__APPLE__)

LogWriter::Wakeup::Wakeup() noexcept
    : semaphore_{::dispatch_semaphore_create(0)} {}

LogWriter::Wakeup::~Wakeup() { ::dispatch_release(semaphore_); }

void LogWriter::Wakeup::post() noexcept {
  ::dispatch_semaphore_signal(semaphore_);
}

void LogWriter::Wakeup::wait_until(
    std::chrono::steady_clock::time_point deadline) noexcept {
  const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(
      deadline - std::chrono::steady_clock::now());
  ::dispatch_semaphore_wait(
      semaphore_, remaining.count() <= 0
                      ? DISPATCH_TIME_NOW
                      : ::dispatch_time(DISPATCH_TIME_NOW, remaining.count()));
}

#else

LogWriter::Wakeup::Wakeup() noexcept { ::sem_init(&semaphore_, 0, 0); }

LogWriter::Wakeup::~Wakeup() { ::sem_destroy(&semaphore_); }

void LogWriter::Wakeup::post() noexcept { ::sem_post(&semaphore_); }

void LogWriter::Wakeup::wait_until(
    std::chrono::steady_clock::time_point deadline) noexcept {
  const auto to_timespec = [](std::int64_t ns) {
    timespec at{};
    at.tv_sec = static_cast<std::time_t>(ns / 1'000'000'000);
    at.tv_nsec = static_cast<long>(ns % 1'000'000'000);
    return at;
  };
#if defined(__GLIBC__) && \
    (__GLIBC__ > 2 || (__GLIBC__ == 2 && __GLIBC_MINOR__ >= 30))
  // steady_clock is CLOCK_MONOTONIC here, so a wall-clock step cannot stretch
  // the wait past the periodic sync.
  const timespec at =
      to_timespec(std::chrono::duration_cast<std::chrono::nanoseconds>(
                      deadline.time_since_epoch())
                      .count());
  while (::sem_clockwait(&semaphore_, CLOCK_MONOTONIC, &at) != 0 &&
         errno == EINTR) {
  }
#else
  // No sem_clockwait: the same remaining time, against the realtime clock.
  timespec now{};
  ::clock_gettime(CLOCK_REALTIME, &now);
  const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(
      deadline - std::chrono::steady_clock::now());
  const timespec at = to_timespec(
      std::max<std::int64_t>(remaining.count(), 0) +
      static_cast<std::int64_t>(now.tv_sec) * 1'000'000'000 + now.tv_nsec);
  while (::sem_timedwait(&semaphore_, &at) != 0 && errno == EINTR) {
  }
#endif
}

#endif

LogWriter::LogWriter(std::string_view path, std::string_view process_name,
                     Options options, std::uint64_t session_id)
    : state_{std::make_unique<State>()} {
  if (options.chunk_bytes == 0) {
    options.chunk_bytes = DEFAULT_CHUNK_BYTES;
  }
  if (options.chunk_count == 0) {
    options.chunk_count = 1;
  }

  state_->path = std::string{path};
  state_->process_name = std::string{process_name};
  state_->session_id = session_id;
  state_->options = options;

  state_->pool.resize(options.chunk_count);
  for (Chunk& chunk : state_->pool) {
    chunk.data.resize(options.chunk_bytes);
  }

  state_->active = &state_->pool.front();

  // A background writer's file is created by its thread (open_file): the
  // constructor runs on the thread that will run the loop.
  if (options.background) return;
  state_->fd =
      ::open(std::string{path}.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);

  if (state_->fd < 0) {
    throw std::system_error(errno, std::generic_category(),
                            "LogWriter: open failed for " + std::string{path});
  }
}

LogWriter::LogWriter(std::string_view path, std::string_view process_name,
                     std::size_t buffer_bytes, std::uint64_t session_id)
    : LogWriter{path, process_name,
                Options{buffer_bytes, 1, /*background=*/false}, session_id} {}

LogWriter::~LogWriter() = default;

std::uint64_t LogWriter::session_id() const noexcept {
  return state_ ? state_->session_id : 0;
}

LogWriter::State::~State() {
  // Explicit flush is optional; destruction drains the accepted prefix too.
  if (active != nullptr && active->used > 0) {
    submit_active();
  }
  stop_writer();
  sync(/*durable=*/true);

  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

void LogWriter::State::set_error(std::string message) noexcept {
  std::lock_guard lock{error_mutex};
  failed = true;
  if (error.empty()) {
    error = std::move(message);
  }
}

void LogWriter::State::write_all(const void* data, std::size_t size) {
  const auto* bytes = reinterpret_cast<const char*>(data);
  std::size_t written = 0;
  while (written < size) {
    const ssize_t n = ::write(fd, bytes + written, size - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::system_error(errno, std::generic_category(),
                              "LogWriter: write failed");
    }
    written += static_cast<std::size_t>(n);
  }
}

// Runs on the writer thread (or inline when background is off). Never throws:
// a failed write is recorded and the chunk is dropped rather than retried
// forever, because the loop it serves cannot wait.
void LogWriter::State::write_chunk(Chunk& chunk) noexcept {
  const std::size_t size = options.background
                               ? chunk.committed.load(std::memory_order_acquire)
                               : chunk.used;

  if (failed || size == 0) return;
  // The chunk's segment is read only once it holds records: the producer set
  // it before committing the first.
  if (chunk.written == 0 && chunk.segment != file_segment) {
    rotate_to(chunk.segment);
    if (failed) return;
  }

  while (chunk.written < size) {
    const ssize_t n =
        ::write(fd, chunk.data.data() + chunk.written, size - chunk.written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      set_error(std::string{"LogWriter: flush failed: "} +
                std::strerror(errno));
      return;
    }
    chunk.written += static_cast<std::size_t>(n);
  }
}

// Syncs the directory holding `path`, so renames, creations and unlinks in
// it survive a power loss. 0, or -1 with errno set.
static int SyncDirectoryOf(const std::string& path) noexcept {
  const auto slash = path.rfind('/');
  const std::string directory =
      slash == std::string::npos ? "." : path.substr(0, slash == 0 ? 1 : slash);
  const int fd = ::open(directory.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) return -1;
  const int result = process::DurableSync(fd);
  const int code = errno;
  ::close(fd);
  errno = code;
  return result;
}

// Runs where write_chunk does, before the chunk's first byte. A rename and a
// fresh file rather than a rewrite: every segment stays an append-only log
// that a power loss can only shorten. The order makes every cut leave whole
// segments: the finished one is synced durably; the next one's prefix is
// written and synced under a name no reader takes for a log (`.next`) before
// it is renamed into place; the cap marker is durable before the oldest
// segment goes; the directory is synced last.
void LogWriter::State::rotate_to(std::uint64_t next) noexcept {
  sync(/*durable=*/true);
  ::close(fd);
  fd = -1;
  const auto fail = [this](std::string_view step, const std::string& what) {
    set_error("LogWriter: cannot " + std::string{step} + " " + what + ": " +
              std::strerror(errno));
  };

  const std::string staged = path + ".next";
  int next_fd =
      ::open(staged.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (next_fd < 0) return fail("open", staged);
  std::swap(fd, next_fd);  // write_all writes to fd.
  try {
    write_all(prefix.data(), prefix.size());
  } catch (const std::system_error& e) {
    set_error(e.what());
  }
  std::swap(fd, next_fd);
  if (failed || process::DurableSync(next_fd) != 0) {
    if (!failed) fail("sync", staged);
    ::close(next_fd);
    ::unlink(staged.c_str());
    return;
  }

  // At most segment_count segments, the new one included: those the last
  // rotation kept that now fall out are deleted, after the marker that says
  // the log lost its head is on disk.
  const bool drop = next >= segment_count;
  if (drop && !capped_marked) {
    head_dropped.store(true);
    mark_capped();
    if (SyncDirectoryOf(path) != 0) {
      ::close(next_fd);
      return fail("sync the directory of", path);
    }
  }
  if (::rename(path.c_str(), LogSegmentPath(path, file_segment).c_str()) != 0) {
    ::close(next_fd);
    return fail("rename", path);
  }
  if (::rename(staged.c_str(), path.c_str()) != 0) {
    ::close(next_fd);
    return fail("rename", staged);
  }
  fd = next_fd;
  file_segment = next;
  if (drop) {
    for (std::uint64_t old = dropped_through; old <= next - segment_count;
         ++old) {
      ::unlink(LogSegmentPath(path, old).c_str());
    }
    dropped_through = next - segment_count + 1;
  }
  if (SyncDirectoryOf(path) != 0) fail("sync the directory of", path);
}

void LogWriter::State::sync(bool durable) noexcept {
  if (fd < 0) {
    // No file (never created, or lost by a failed rotation): nothing to sync,
    // but the periodic sync is still done, or the writer thread would find
    // it due on every pass, spin, and never see `stopping`.
    last_sync = std::chrono::steady_clock::now();
    return;
  }
  int result;
  if (durable) {
    result = process::DurableSync(fd);
  } else {
    do {
      result = ::fsync(fd);
    } while (result < 0 && errno == EINTR);
  }
  if (result < 0) {
    set_error(std::string{"LogWriter: fsync failed: "} + std::strerror(errno));
  }
  last_sync = std::chrono::steady_clock::now();
  mark_capped();
}

// Off the producer path: the periodic sync, drain and close call it.
void LogWriter::State::mark_capped() noexcept {
  if (capped_marked ||
      (!head_dropped.load() && capture_error.load() != CaptureError::BYTE_CAP))
    return;
  capped_marked = true;
  const std::string marker = CappedMarkerPath(path);
  const int marker_fd =
      ::open(marker.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (marker_fd < 0) {
    set_error(std::string{"LogWriter: cannot create "} + marker + ": " +
              std::strerror(errno));
    return;
  }
  ::fsync(marker_fd);
  ::close(marker_fd);
}

void LogWriter::State::submit_active() noexcept {
  if (active == nullptr || active->used == 0) {
    return;
  }

  Chunk* chunk = active;
  active = nullptr;

  if (!options.background) {
    write_chunk(*chunk);
    chunk->used = 0;
    chunk->written = 0;
    active = chunk;
    return;
  }

  // Publishes both the chunk bytes and used count to the writer. Sequentially
  // consistent, like the writer_idle check after it (see writer_idle).
  submitted.fetch_add(1, std::memory_order_seq_cst);
  // A post only when the writer may be asleep, so a busy writer costs the
  // producer one atomic load per chunk. The post takes no lock and does not
  // block; at most it wakes the writer.
  if (writer_idle.load(std::memory_order_seq_cst) &&
      writer_idle.exchange(false, std::memory_order_seq_cst)) {
    wake.post();
  }
}

// Only the producer selects free chunks. The consumer returns ownership by
// advancing completed after finishing the write.
void LogWriter::State::acquire_chunk() noexcept {
  if (active != nullptr) {
    return;
  }

  const auto next = submitted.load(std::memory_order_relaxed);
  if (next - completed.load(std::memory_order_acquire) >= pool.size()) {
    capture_error.store(CaptureError::POOL_FULL);
    return;
  }

  active = &pool[next % pool.size()];
  active->used = 0;
  active->segment = segment;
}

void LogWriter::State::append(RecordHeader header,
                              std::span<const std::byte> payload) noexcept {
  if (failed || capture_error.load() != CaptureError::NONE) return;

  // Fixed-size messages are mostly trailing padding. Only the bytes up to the
  // last non-zero one are stored; the reader restores the rest.
  std::size_t stored = payload.size();
  while (stored > 0 && payload[stored - 1] == std::byte{0}) --stored;

  header.magic = RECORD_MAGIC;
  header.payload_bytes = static_cast<std::uint32_t>(payload.size());
  header.stored_bytes = static_cast<std::uint32_t>(stored);
  header.crc32 = 0;
  payload = payload.first(stored);

  const std::uint32_t crc_after_header =
      crc32(as_bytes(&header, sizeof(header)));
  header.crc32 =
      payload.empty() ? crc_after_header : crc32(payload, crc_after_header);

  const std::size_t total = sizeof(RecordHeader) + payload.size();
  const std::uint64_t limit =
      segment_bytes != 0 ? segment_bytes : options.max_bytes;
  if (limit != 0 && segment_used + total > limit) {
    if (segment_bytes == 0 || prefix.size() + total > segment_bytes) {
      capture_error.store(CaptureError::BYTE_CAP);
      return;
    }
    // This record starts the next segment, in a chunk of its own; the writer
    // rotates the file before writing it.
    submit_active();
    ++segment;
    segment_used = prefix.size();
    if (active != nullptr) {
      active->segment = segment;  // Synchronous: the same, emptied chunk.
    } else {
      acquire_chunk();
      if (active == nullptr) return;
    }
  }

  if (active == nullptr) {
    acquire_chunk();
    if (active == nullptr) {
      return;
    }
  }

  if (active->used + total > active->data.size()) {
    submit_active();
    acquire_chunk();
    if (active == nullptr) {
      return;
    }
  }

  // Capacity is fixed before dispatch begins. Preserve the accepted prefix
  // and make failure visible instead of allocating or leaving a hole.
  if (total > active->data.size()) {
    capture_error.store(CaptureError::RECORD_TOO_LARGE);
    return;
  }

  std::memcpy(active->data.data() + active->used, &header, sizeof(header));
  if (!payload.empty()) {
    std::memcpy(active->data.data() + active->used + sizeof(header),
                payload.data(), payload.size());
  }
  active->used += total;
  segment_used += total;
  if (options.background) {
    active->committed.store(active->used, std::memory_order_release);
  } else if (std::chrono::steady_clock::now() - last_sync >=
             std::chrono::seconds{1}) {
    submit_active();
    sync(/*durable=*/false);
  }
}

void LogWriter::State::drain() noexcept {
  submit_active();

  if (!options.background) {
    sync(/*durable=*/true);
    return;
  }

  // Not started: nothing was handed over and no file exists.
  if (!writer.joinable()) return;

  wake.post();
  std::unique_lock lock{mutex};
  chunk_ready.wait(lock, [this] {
    return file_ready && completed.load() == submitted.load();
  });
  // All submitted chunks have reached the descriptor; serialize the sync with
  // the worker's periodic one, which also runs under this mutex.
  sync(/*durable=*/true);
}

void LogWriter::State::open_file() noexcept {
  if (prefix.empty()) {
    mark_capped();
    return;
  }
  fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    // Sticky: append() accepts nothing more, and write_chunk() drops what
    // was already handed over, so drain() and destruction still return.
    set_error("LogWriter: open failed for " + path + ": " +
              std::strerror(errno));
    return;
  }
  try {
    write_all(prefix.data(), prefix.size());
  } catch (const std::system_error& e) {
    set_error(e.what());
  }
}

void LogWriter::State::start_writer() {
  if (writer.joinable()) return;

  State* raw = this;
  writer = std::thread{[raw] {
    UseNormalScheduling();
    // Before any chunk, so the prefix leads the file. Records handed over
    // meanwhile wait in the pool; the producer never waits for this.
    raw->open_file();
    std::unique_lock lock{raw->mutex};
    raw->file_ready = true;
    raw->chunk_ready.notify_all();

    for (;;) {
      // Sleep, without the mutex, until a hand-off (the producer posts when
      // it finds the writer idle), flush()/destruction (which always post),
      // or the next periodic sync. A post that lands before the wait is kept
      // by the semaphore, so none is lost; a stale one only makes the next
      // wait return at once and the loop check again.
      const auto sync_at = raw->last_sync + std::chrono::seconds{1};
      lock.unlock();
      raw->writer_idle.store(true, std::memory_order_seq_cst);
      if (raw->completed.load() == raw->submitted.load() &&
          !raw->stopping.load()) {
        raw->wake.wait_until(sync_at);
      }
      raw->writer_idle.store(false, std::memory_order_seq_cst);
      lock.lock();

      const bool sync_due = std::chrono::steady_clock::now() - raw->last_sync >=
                            std::chrono::seconds{1};
      const auto next = raw->completed.load(std::memory_order_relaxed);
      const bool submitted =
          next != raw->submitted.load(std::memory_order_acquire);
      if (!submitted && !sync_due) {
        if (raw->stopping) {
          return;
        }
        continue;
      }

      Chunk* chunk = &raw->pool[next % raw->pool.size()];

      // The mutex is never held across the write, so a slow disk cannot make
      // the loop thread's hand-off block behind it.
      lock.unlock();
      raw->write_chunk(*chunk);
      lock.lock();

      if (submitted) {
        chunk->committed.store(0, std::memory_order_relaxed);
        chunk->written = 0;
        raw->completed.store(next + 1, std::memory_order_release);
        raw->chunk_ready.notify_all();
      }
      if (sync_due) {
        raw->sync(/*durable=*/false);
      }
    }
  }};
}

void LogWriter::State::stop_writer() noexcept {
  if (!writer.joinable()) {
    return;
  }

  stopping.store(true, std::memory_order_seq_cst);
  wake.post();
  writer.join();
}

void LogWriter::start(const Manifest& manifest, MonotonicTime start_time) {
  State& state = *state_;

  std::size_t largest_record = sizeof(RecordHeader);
  for (const auto& reg : manifest) {
    largest_record =
        std::max(largest_record, sizeof(RecordHeader) + reg.message_bytes);
  }
  const std::size_t capacity =
      std::max(state.options.chunk_bytes, largest_record);
  for (auto& chunk : state.pool) chunk.data.resize(capacity);

  std::vector<ManifestEntry> entries;
  entries.reserve(manifest.size());
  std::vector<std::byte> schemas;
  for (const auto& reg : manifest) {
    ManifestEntry entry{};
    entry.id = reg.id;
    entry.kind = static_cast<std::uint16_t>(reg.kind);
    entry.message_bytes = reg.message_bytes;
    entry.alignment = reg.alignment;
    entry.armed = reg.armed;
    entry.period_ns = reg.period_ns;
    entry.offset_ns = reg.offset_ns;
    write_name(entry.name, reg.name);
    entry.schema_hash = reg.schema.hash;
    entry.schema_name_bytes =
        static_cast<std::uint32_t>(reg.schema.name.size());
    entry.schema_bfbs_bytes =
        static_cast<std::uint32_t>(reg.schema.bfbs.size());
    entries.push_back(entry);
    const auto name = std::as_bytes(std::span{reg.schema.name});
    const auto bfbs = std::as_bytes(std::span{reg.schema.bfbs});
    schemas.insert(schemas.end(), name.begin(), name.end());
    schemas.insert(schemas.end(), bfbs.begin(), bfbs.end());
  }

  const std::span<const std::byte> manifest_bytes =
      as_bytes(entries.data(), entries.size() * sizeof(ManifestEntry));

  FileHeader header{};
  std::memcpy(header.magic, FILE_MAGIC, sizeof(header.magic));
  header.version = FORMAT_VERSION;
  header.header_bytes = sizeof(FileHeader);
  header.session_id = state.session_id;
  header.start_monotonic_ns = start_time.nanos();
  header.start_wall_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  header.manifest_count = static_cast<std::uint32_t>(entries.size());
  header.crc32 = crc32(schemas, crc32(manifest_bytes));
  header.schema_bytes = schemas.size();
  write_name(header.process_name, state.process_name);

  const auto prefix_bytes =
      sizeof(header) + manifest_bytes.size() + schemas.size();
  if (state.options.max_bytes != 0 && prefix_bytes > state.options.max_bytes) {
    // No valid log fits. Do not spend the budget on an unreadable prefix,
    // and do not let a policy limit stop the node itself. The thread that
    // would have created the file creates the marker instead: this one when
    // synchronous, else the writer thread (open_file, with no prefix).
    state.capture_error.store(State::CaptureError::BYTE_CAP);
    if (state.options.background) {
      state.start_writer();
      return;
    }
    ::close(state.fd);
    state.fd = -1;
    ::unlink(state.path.c_str());
    state.mark_capped();
    return;
  }

  // Kept for every later segment's start (rotate_to).
  state.prefix.reserve(prefix_bytes);
  const auto header_bytes = as_bytes(&header, sizeof(header));
  state.prefix.insert(state.prefix.end(), header_bytes.begin(),
                      header_bytes.end());
  state.prefix.insert(state.prefix.end(), manifest_bytes.begin(),
                      manifest_bytes.end());
  state.prefix.insert(state.prefix.end(), schemas.begin(), schemas.end());
  state.segment_used = prefix_bytes;
  if (state.options.max_bytes != 0) {
    // Segments that each hold the prefix and the largest record.
    const std::uint64_t smallest = prefix_bytes + largest_record;
    const std::uint64_t count =
        std::min(kLogSegments, state.options.max_bytes / smallest);
    if (count >= 2) {
      state.segment_count = count;
      state.segment_bytes = state.options.max_bytes / count;
    }
  }

  if (state.options.background) {
    // The writer thread opens the file and writes the prefix: this thread
    // runs the loop and must not wait on the disk.
    state.start_writer();
    return;
  }
  // Synchronous: the constructor opened the file, and the prefix leads it.
  state.write_all(state.prefix.data(), state.prefix.size());
}

void LogWriter::dispatch(const Context& context,
                         std::span<const std::byte> payload) noexcept {
  RecordHeader header{};
  header.kind = static_cast<std::uint16_t>(context.kind);
  header.source_id = context.source_id;
  header.dispatch_index = context.dispatch_index;
  header.event_time_ns = context.event_time.nanos();
  header.now_ns = context.now.nanos();
  header.publish_time_ns = context.publish_time.nanos();
  header.sequence = context.sequence;
  header.aux = static_cast<std::uint32_t>(context.dropped);
  state_->append(header, payload);
}

void LogWriter::send(const Context& parent, std::uint16_t source_id,
                     std::span<const std::byte> payload, std::uint32_t status,
                     std::uint64_t sequence) noexcept {
  RecordHeader header{};
  header.kind = static_cast<std::uint16_t>(EventKind::SEND);
  header.source_id = source_id;
  header.dispatch_index = parent.dispatch_index;
  header.event_time_ns = parent.event_time.nanos();
  header.now_ns = parent.now.nanos();
  header.publish_time_ns = parent.now.nanos();
  header.sequence = sequence;
  header.aux = status;
  state_->append(header, payload);
}

void LogWriter::finish(const Context& context) noexcept {
  RecordHeader header{};
  header.kind = static_cast<std::uint16_t>(EventKind::EXIT);
  header.source_id = context.source_id;
  header.dispatch_index = context.dispatch_index;
  header.event_time_ns = context.event_time.nanos();
  header.now_ns = context.now.nanos();
  header.publish_time_ns = context.publish_time.nanos();
  header.sequence = context.sequence;
  header.aux = static_cast<std::uint32_t>(context.dropped);
  state_->append(header, std::span<const std::byte>{});
}

void LogWriter::flush() noexcept { state_->drain(); }

bool LogWriter::failed() const noexcept {
  const auto capture = state_->capture_error.load();
  return state_->failed || (capture != State::CaptureError::NONE &&
                            capture != State::CaptureError::BYTE_CAP);
}

std::string LogWriter::error() const {
  switch (state_->capture_error.load()) {
    case State::CaptureError::POOL_FULL:
      return "LogWriter: buffer pool exhausted; capture stopped";
    case State::CaptureError::RECORD_TOO_LARGE:
      return "LogWriter: record exceeds preallocated capacity; capture stopped";
    case State::CaptureError::BYTE_CAP:
    case State::CaptureError::NONE:
      break;
  }
  std::lock_guard lock{state_->error_mutex};
  if (state_->error.empty() &&
      state_->capture_error.load() == State::CaptureError::BYTE_CAP) {
    return "LogWriter: byte cap of " +
           std::to_string(state_->options.max_bytes) +
           " reached; capture stopped";
  }
  return state_->error;
}

bool LogWriter::capture_stopped() const noexcept {
  return state_->capture_error.load() != State::CaptureError::NONE;
}

bool LogWriter::capped() const noexcept {
  return state_->head_dropped.load() ||
         state_->capture_error.load() == State::CaptureError::BYTE_CAP;
}

}  // namespace talos::event::log
