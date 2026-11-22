#include "talOS/events/log/log_writer.h"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstring>
#include <ctime>
#include <stdexcept>
#include <utility>

#include "talOS/process/process.h"

namespace talos::event::log {

namespace {

std::span<const std::byte> as_bytes(const void* data, std::size_t size) {
  return {reinterpret_cast<const std::byte*>(data), size};
}

}  // namespace

std::chrono::nanoseconds ParseLogWindowUs(const std::string& value) {
  constexpr std::uint64_t kLargest =
      static_cast<std::uint64_t>(std::chrono::nanoseconds::max().count()) /
      1000;
  std::uint64_t us = 0;
  const auto [end, error] =
      std::from_chars(value.data(), value.data() + value.size(), us);
  if (error != std::errc{} || end != value.data() + value.size() ||
      us > kLargest) {
    throw std::invalid_argument(
        "--log-window-us: not a window in "
        "microseconds: " +
        value);
  }
  return std::chrono::microseconds{static_cast<std::int64_t>(us)};
}

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

LogWriter::State::State(std::string path) : sink{std::move(path)} {}

LogWriter::LogWriter(std::string_view path, std::string_view process_name,
                     Options options, std::uint64_t session_id)
    : state_{std::make_unique<State>(std::string{path})} {
  if (options.chunk_bytes == 0) {
    options.chunk_bytes = DEFAULT_CHUNK_BYTES;
  }
  if (options.chunk_count == 0) {
    options.chunk_count = 1;
  }

  state_->process_name = std::string{process_name};
  state_->session_id = session_id;
  state_->options = std::move(options);
  state_->gaps.set_gap_flags(state_->options.gap_flags);

  state_->pool.resize(state_->options.chunk_count);
  for (Chunk& chunk : state_->pool) {
    chunk.data.resize(state_->options.chunk_bytes);
  }

  state_->active = &state_->pool.front();

  // A background writer's file is created by its thread (open_file): the
  // constructor runs on the thread that will run the loop.
  if (state_->options.background) return;
  state_->sink.open_sync();
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
  // Explicit flush is optional; destruction drains the accepted prefix too,
  // and marks an open gap closed, with the writer still running to make room.
  if (gaps.has_pending_stop()) {
    settle();
  } else if (active != nullptr && active->used > 0) {
    submit_active();
  }
  stop_writer();
  sync(/*durable=*/true);
  sink.close();
}

// Runs on the writer thread (or inline when background is off). Never throws:
// a failed write is recorded and the chunk is dropped rather than retried
// forever, because the loop it serves cannot wait.
void LogWriter::State::write_chunk(Chunk& chunk) noexcept {
  const std::size_t size = options.background
                               ? chunk.committed.load(std::memory_order_acquire)
                               : chunk.used;

  if (sink.failed() || size == 0) return;
  if (options.background && options.before_write) options.before_write();
  // The chunk's segment is read only once it holds records: the producer set
  // it before committing the first.
  if (chunk.written == 0 && chunk.segment != sink.file_segment()) {
    sink.rotate_to(chunk.segment);
    if (sink.failed()) return;
  }

  chunk.written +=
      sink.write_bytes(chunk.data.data() + chunk.written, size - chunk.written);
}

void LogWriter::State::sync(bool durable) noexcept {
  sink.sync(durable, capture_error.load() == CaptureError::BYTE_CAP);
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
  // Always a post, idle or not: a writer asleep until its next timed write
  // (writer_idle false) must not leave a full chunk waiting write_interval,
  // or a producer filling the pool within one would exhaust it. One post per
  // chunk; it takes no lock and does not block, and a post to a writer
  // already awake only makes its next wait return at once.
  writer_idle.store(false, std::memory_order_seq_cst);
  wake.post();
}

// Only the producer selects free chunks. The consumer returns ownership by
// advancing completed after finishing the write. False when every chunk is
// handed over and not yet written: the pool is full.
bool LogWriter::State::acquire_chunk() noexcept {
  if (active != nullptr) {
    return true;
  }

  const auto next = submitted.load(std::memory_order_relaxed);
  if (next - completed.load(std::memory_order_acquire) >= pool.size()) {
    return false;
  }

  active = &pool[next % pool.size()];
  active->used = 0;
  active->segment = segment;
  return true;
}

LogWriter::State::Take LogWriter::State::take(
    const RecordHeader& header, std::span<const std::byte> stored) noexcept {
  const std::size_t total = sizeof(RecordHeader) + stored.size();
  // Capacity is fixed before dispatch begins: a record larger than a chunk
  // is dropped (a gap), never allocated for, before it can end a segment or
  // hand a chunk over.
  if (total > pool.front().data.size()) return Take::kTooLarge;
  const std::size_t prefix_size = sink.prefix_size();
  const std::uint64_t limit =
      segment_bytes != 0 ? segment_bytes : options.max_bytes;
  const bool full = limit != 0 && segment_used + total > limit;
  if (full && (segment_bytes == 0 || prefix_size + total > segment_bytes)) {
    capture_error.store(CaptureError::BYTE_CAP);
    return Take::kStopped;
  }
  // Options::window: the segment's records span a window / n of record time.
  // Nonzero only for a rotating log; the segment holds a record (the first
  // set segment_first_ns); a time at or before the first never ends it, and
  // the difference is taken unsigned only once it is positive.
  const bool elapsed = segment_span_ns != 0 && segment_used > prefix_size &&
                       header.now_ns > segment_first_ns &&
                       static_cast<std::uint64_t>(header.now_ns) -
                               static_cast<std::uint64_t>(segment_first_ns) >=
                           segment_span_ns;
  if (full || elapsed) {
    // This record starts the next segment, in a chunk of its own; the writer
    // rotates the file before writing it.
    submit_active();
    ++segment;
    segment_used = prefix_size;
    if (active != nullptr) {
      active->segment = segment;  // Synchronous: the same, emptied chunk.
    }
  }

  if (!acquire_chunk()) return Take::kPoolFull;

  if (active->used + total > active->data.size()) {
    submit_active();
    if (!acquire_chunk()) return Take::kPoolFull;
  }

  std::memcpy(active->data.data() + active->used, &header, sizeof(header));
  if (!stored.empty()) {
    std::memcpy(active->data.data() + active->used + sizeof(header),
                stored.data(), stored.size());
  }
  active->used += total;
  if (segment_used == prefix_size) segment_first_ns = header.now_ns;
  segment_used += total;
  if (options.background) {
    // Sequentially consistent, like the writer_idle check after it: either
    // the writer, about to sleep, sees this record, or this sees it asleep
    // and posts. A busy writer costs one atomic load per record.
    active->committed.store(active->used, std::memory_order_seq_cst);
    if (writer_idle.load(std::memory_order_seq_cst) &&
        writer_idle.exchange(false, std::memory_order_seq_cst)) {
      wake.post();
    }
  } else if (std::chrono::steady_clock::now() - sink.last_sync() >=
             options.sync_interval) {
    submit_active();
    sync(/*durable=*/false);
  }
  return Take::kTaken;
}

// Capture resumes only with half the pool free, rounded up (2 of 3): a gap
// opens only once every chunk is handed over, so between two GAP records of
// a full pool the loop hands over at least half the pool, and the writer
// still has the other half queued when capture resumes.
bool LogWriter::State::room_to_resume() const noexcept {
  if (!options.background) return true;
  const std::uint64_t outstanding = submitted.load(std::memory_order_relaxed) -
                                    completed.load(std::memory_order_acquire);
  const std::uint64_t half = (pool.size() + 1) / 2;
  return outstanding + half <= pool.size();
}

LogWriter::State::Take LogWriter::State::take_gap(
    std::int64_t now_ns) noexcept {
  RecordHeader header{};
  const std::span<const std::byte> stored = gaps.seal_gap(header, now_ns);
  return take(header, stored);
}

void LogWriter::State::append(RecordHeader header,
                              std::span<const std::byte> payload) noexcept {
  if (sink.failed() || capture_error.load() != CaptureError::NONE) return;
  const std::size_t payload_bytes = payload.size();

  if (gaps.is_losing()) {
    // Resume only at a record that starts a new dispatch (or belongs to
    // none), so the records after a GAP are whole dispatches.
    if (!gaps.can_resume_at(header) || !room_to_resume()) {
      gaps.lose(header, payload_bytes);
      return;
    }
    payload = SealRecord(header, payload);
    if (sizeof(RecordHeader) + payload.size() > pool.front().data.size()) {
      gaps.lose(header, payload_bytes);
      return;
    }
    switch (take_gap(header.now_ns)) {
      case Take::kTaken:
        break;
      case Take::kStopped:
        // The byte cap ended capture: that gap stays unmarked.
        gaps.close();
        return;
      case Take::kPoolFull:
      case Take::kTooLarge:
        gaps.lose(header, payload_bytes);  // Still open.
        return;
    }
    gaps.close();
  } else {
    payload = SealRecord(header, payload);
  }

  const Take taken = take(header, payload);
  if (taken == Take::kTaken) {
    gaps.note_accepted(header.dispatch_index);
    return;
  }
  if (taken == Take::kStopped) return;
  // An EXIT that finds no room opens no gap: it is held for the stop.
  if (static_cast<EventKind>(header.kind) != EventKind::EXIT) {
    gaps.open(header, taken == Take::kTooLarge ? GapCause::RECORD_TOO_LARGE
                                               : GapCause::POOL_FULL);
  }
  gaps.lose(header, payload_bytes);
}

bool LogWriter::State::close_at_stop() noexcept {
  const auto open = [this] {
    return !sink.failed() && capture_error.load() == CaptureError::NONE;
  };
  bool took = false;
  if (gaps.is_losing()) {
    gaps.close();
    if (open()) {
      gaps.mark_closed_at_flush();
      took = take_gap(gaps.stop_gap_now_ns()) == Take::kTaken;
    }
  }
  if (std::optional<RecordHeader> exit = gaps.take_held_exit();
      exit.has_value()) {
    if (open() && take(*exit, SealRecord(*exit, {})) == Take::kTaken) {
      gaps.note_accepted(exit->dispatch_index);
      took = true;
    }
  }
  return took;
}

void LogWriter::State::wait_written() noexcept {
  if (!options.background || !writer.joinable()) return;
  wake.post();
  std::unique_lock lock{mutex};
  chunk_ready.wait(lock, [this] {
    return file_ready && completed.load() == submitted.load();
  });
}

void LogWriter::State::settle() noexcept {
  submit_active();
  // Not started: nothing was handed over and no file exists.
  if (options.background && !writer.joinable()) return;
  wait_written();
  // Every chunk is written, so the pool has room for what the stop adds.
  if (close_at_stop()) {
    submit_active();
    wait_written();
  }
}

void LogWriter::State::drain() noexcept {
  settle();

  if (!options.background) {
    sync(/*durable=*/true);
    return;
  }

  // Not started: nothing was handed over and no file exists.
  if (!writer.joinable()) return;

  // All submitted chunks have reached the descriptor; serialize the sync with
  // the worker's periodic one, which also runs under this mutex.
  std::lock_guard lock{mutex};
  sync(/*durable=*/true);
}

void LogWriter::State::start_writer() {
  if (writer.joinable()) return;

  State* raw = this;
  writer = std::thread{[raw] {
    process::ConfigureBackgroundThread();
    // Before any chunk, so the prefix leads the file. Records handed over
    // meanwhile wait in the pool; the producer never waits for this.
    raw->sink.open_file();
    std::unique_lock lock{raw->mutex};
    raw->file_ready = true;
    raw->chunk_ready.notify_all();
    // The last wait was a timed one (see the sleep below), and the last
    // record write was one such a wait held back.
    bool timed = false;
    bool streaming = false;

    for (;;) {
      const auto now = std::chrono::steady_clock::now();
      const auto next = raw->completed.load(std::memory_order_relaxed);
      const bool submitted =
          next != raw->submitted.load(std::memory_order_seq_cst);
      // The oldest chunk not yet completed: a handed-over one, else the one
      // the producer is filling.
      Chunk* chunk = &raw->pool[next % raw->pool.size()];
      // A failed writer drops what is left (write_chunk), so it has nothing
      // to wait for.
      const bool unwritten =
          !raw->sink.failed() &&
          chunk->committed.load(std::memory_order_seq_cst) > chunk->written;
      const bool sync_due =
          now - raw->sink.last_sync() >= raw->options.sync_interval;
      const bool write_due =
          unwritten && (!raw->last_write ||
                        now - *raw->last_write >= raw->options.write_interval);

      if (submitted || write_due || sync_due) {
        // The mutex is never held across the write, so a slow disk cannot
        // make the loop thread's hand-off block behind it.
        lock.unlock();
        raw->write_chunk(*chunk);
        lock.lock();
        // A sync with nothing to write opens no write window.
        if (submitted || unwritten) raw->last_write = now;
        // Records are streaming when the write was one a timed wait held
        // back; a record written at once found the writer idle.
        if (unwritten && !submitted) streaming = timed;
        timed = false;

        if (submitted) {
          chunk->committed.store(0, std::memory_order_relaxed);
          chunk->written = 0;
          raw->completed.store(next + 1, std::memory_order_release);
          raw->chunk_ready.notify_all();
        }
        if (sync_due) {
          raw->sync(/*durable=*/false);
        }
        continue;
      }
      if (raw->options.before_stop_check) {
        raw->options.before_stop_check(raw->stopping);
      }
      if (raw->stopping.load(std::memory_order_seq_cst)) {
        // The owner hands over its last chunk before it sets stopping
        // (~State: submit_active, then stop_writer), so a hand-off this
        // pass's read of `submitted` missed is visible now: write it on
        // another pass rather than return without it.
        if (raw->submitted.load(std::memory_order_seq_cst) != next) continue;
        return;
      }

      // Sleep, without the mutex. A record that arrived within
      // write_interval of the last write waits for the next write to be
      // due; while records are streaming (the last write was such a wait's)
      // the writer also sleeps until then without telling the producer it
      // is idle, so a steady stream is written once per write_interval and
      // its records never post (a hand-off always does, so a full chunk is
      // written at once). Otherwise, with nothing to write, the
      // writer sleeps until a hand-off or a new record (the producer posts
      // when it finds the writer idle, and that record is written at once),
      // flush()/destruction (which always post), or the next periodic sync,
      // so a record sparser than write_interval costs one wakeup. A post
      // that lands before the wait is kept by the semaphore, so none is
      // lost; a stale one only makes the next wait return at once and the
      // loop check again.
      const auto sync_at = raw->sink.last_sync() + raw->options.sync_interval;
      // A writer that has never written has nothing unwritten and no
      // stream here (an unwritten record would have been due above).
      const auto write_at =
          raw->last_write.value_or(now) + raw->options.write_interval;
      lock.unlock();
      if (unwritten || (streaming && now < write_at)) {
        timed = true;
        raw->wake.wait_until(std::min(write_at, sync_at));
        raw->wakeups.fetch_add(1, std::memory_order_relaxed);
      } else {
        // An empty timed wake ends the stream: the next record finds the
        // writer idle and is written at once, with no timed wait after it.
        streaming = false;
        timed = false;
        raw->writer_idle.store(true, std::memory_order_seq_cst);
        if (raw->completed.load() == raw->submitted.load() &&
            (raw->sink.failed() ||
             chunk->committed.load(std::memory_order_seq_cst) ==
                 chunk->written) &&
            !raw->stopping.load()) {
          raw->wake.wait_until(sync_at);
          raw->wakeups.fetch_add(1, std::memory_order_relaxed);
        }
        raw->writer_idle.store(false, std::memory_order_seq_cst);
      }
      lock.lock();
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

  // Every chunk and every segment holds a GAP record.
  std::size_t largest_record = GAP_RECORD_BYTES;
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
    state.sink.abort_for_tiny_cap_sync();
    return;
  }

  // Kept for every later segment's start (rotate_to).
  std::vector<std::byte> prefix;
  prefix.reserve(prefix_bytes);
  const auto header_bytes = as_bytes(&header, sizeof(header));
  prefix.insert(prefix.end(), header_bytes.begin(), header_bytes.end());
  prefix.insert(prefix.end(), manifest_bytes.begin(), manifest_bytes.end());
  prefix.insert(prefix.end(), schemas.begin(), schemas.end());

  std::uint64_t segment_count = 1;
  state.segment_used = prefix_bytes;
  if (state.options.max_bytes != 0) {
    // Segments that each hold the prefix and the largest record.
    const std::uint64_t smallest = prefix_bytes + largest_record;
    const std::uint64_t count =
        std::min(kLogSegments, state.options.max_bytes / smallest);
    if (count >= 2) {
      segment_count = count;
      state.segment_bytes = state.options.max_bytes / count;
      if (state.options.window.count() > 0) {
        // Never sooner than sync_interval: each rotation syncs durably and
        // takes a chunk, so a window whose segments end faster than the
        // writer syncs could exhaust the pool and drop records.
        state.segment_span_ns = std::max<std::uint64_t>(
            static_cast<std::uint64_t>(
                std::max<std::int64_t>(1, state.options.sync_interval.count())),
            static_cast<std::uint64_t>(state.options.window.count()) / count);
      }
    }
  }
  state.sink.configure_segments(std::move(prefix), segment_count);

  if (state.options.background) {
    // The writer thread opens the file and writes the prefix: this thread
    // runs the loop and must not wait on the disk.
    state.start_writer();
    return;
  }
  // Synchronous: the constructor opened the file, and the prefix leads it.
  state.sink.write_prefix_sync();
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

bool LogWriter::failed() const noexcept { return state_->sink.failed(); }

std::string LogWriter::error() const {
  std::string message = state_->sink.error();
  if (message.empty() &&
      state_->capture_error.load() == State::CaptureError::BYTE_CAP) {
    return "LogWriter: byte cap of " +
           std::to_string(state_->options.max_bytes) +
           " reached; capture stopped";
  }
  return message;
}

bool LogWriter::capture_stopped() const noexcept {
  return state_->capture_error.load() == State::CaptureError::BYTE_CAP;
}

std::uint64_t LogWriter::gaps() const noexcept { return state_->gaps.gaps(); }

void LogWriter::set_gap_flags(const GapFlagTarget& target) noexcept {
  state_->options.gap_flags = target;
  state_->gaps.set_gap_flags(target);
}

std::uint64_t LogWriter::lost_records() const noexcept {
  return state_->gaps.lost_records();
}

bool LogWriter::incomplete() const noexcept { return failed() || gaps() > 0; }

bool LogWriter::losing() const noexcept { return state_->gaps.losing(); }

std::uint64_t LogWriter::wakeups() const noexcept {
  return state_->wakeups.load(std::memory_order_relaxed);
}

bool LogWriter::capped() const noexcept {
  return state_->sink.head_dropped() ||
         state_->capture_error.load() == State::CaptureError::BYTE_CAP;
}

}  // namespace talos::event::log
