#include "talOS/events/log/log_writer.h"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <system_error>
#include <utility>

namespace talos::event::log {

namespace {

std::span<const std::byte> as_bytes(const void* data, std::size_t size) {
  return {reinterpret_cast<const std::byte*>(data), size};
}

}  // namespace

LogWriter::LogWriter(std::string_view path, std::string_view process_name,
                     Options options)
    : state_{std::make_unique<State>()} {
  if (options.chunk_bytes == 0) {
    options.chunk_bytes = DEFAULT_CHUNK_BYTES;
  }
  if (options.chunk_count == 0) {
    options.chunk_count = 1;
  }

  state_->process_name = std::string{process_name};
  state_->options = options;

  state_->pool.resize(options.chunk_count);
  for (Chunk& chunk : state_->pool) {
    chunk.data.resize(options.chunk_bytes);
    state_->free_chunks.push_back(&chunk);
  }

  state_->active = state_->free_chunks.front();
  state_->free_chunks.pop_front();

  state_->fd =
      ::open(std::string{path}.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);

  if (state_->fd < 0) {
    throw std::system_error(errno, std::generic_category(),
                            "LogWriter: open failed for " + std::string{path});
  }
}

LogWriter::LogWriter(std::string_view path, std::string_view process_name,
                     std::size_t buffer_bytes)
    : LogWriter{path, process_name,
                Options{buffer_bytes, 1, /*background=*/false}} {}

LogWriter::~LogWriter() = default;

LogWriter::State::~State() {
  // The public destructor runs this after flush() has already drained, but a
  // State destroyed on an error path may still hold data worth keeping.
  if (active != nullptr && active->used > 0) {
    submit_active();
  }
  stop_writer();

  {
    std::unique_lock lock{mutex};
    while (!pending.empty()) {
      Chunk* chunk = pending.front();
      pending.pop_front();
      lock.unlock();
      write_chunk(*chunk);
      lock.lock();
    }
  }

  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

void LogWriter::State::set_error(std::string message) noexcept {
  std::lock_guard lock{mutex};
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
  const std::size_t size = chunk.used;
  chunk.used = 0;

  {
    std::lock_guard lock{mutex};
    if (failed || size == 0) {
      return;
    }
  }

  std::size_t written = 0;
  while (written < size) {
    const ssize_t n = ::write(fd, chunk.data.data() + written, size - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      set_error(std::string{"LogWriter: flush failed: "} +
                std::strerror(errno));
      return;
    }
    written += static_cast<std::size_t>(n);
  }
}

void LogWriter::State::submit_active() noexcept {
  if (active == nullptr || active->used == 0) {
    return;
  }

  Chunk* chunk = active;
  active = nullptr;

  if (!options.background) {
    write_chunk(*chunk);
    active = chunk;
    return;
  }

  {
    std::lock_guard lock{mutex};
    pending.push_back(chunk);
  }
  pending_ready.notify_one();
}

// Takes the next free chunk, waiting if the writer has fallen a whole pool
// behind. The wait is the honest outcome: the alternative is losing records,
// which would make the log unreplayable.
void LogWriter::State::acquire_chunk() noexcept {
  if (active != nullptr) {
    return;
  }

  std::unique_lock lock{mutex};

  if (free_chunks.empty()) {
    ++stalls;
    chunk_ready.wait(lock, [this] { return !free_chunks.empty() || failed; });
  }

  if (free_chunks.empty()) {
    return;  // failed; records are discarded from here on
  }

  active = free_chunks.front();
  free_chunks.pop_front();
  active->used = 0;
}

void LogWriter::State::append(RecordHeader header,
                              std::span<const std::byte> payload) noexcept {
  {
    std::lock_guard lock{mutex};
    if (failed) {
      return;
    }
  }

  header.magic = RECORD_MAGIC;
  header.payload_bytes = static_cast<std::uint32_t>(payload.size());
  header.reserved = 0;
  header.crc32 = 0;

  const std::uint32_t crc_after_header =
      crc32(as_bytes(&header, sizeof(header)));
  header.crc32 =
      payload.empty() ? crc_after_header : crc32(payload, crc_after_header);

  const std::size_t total = sizeof(RecordHeader) + payload.size();

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

  // A single record larger than a whole chunk. Grow this one chunk to fit; it
  // keeps the larger capacity afterwards, so this happens at most once per
  // chunk per record size.
  if (total > active->data.size()) {
    try {
      active->data.resize(total);
    } catch (const std::exception& e) {
      set_error(std::string{"LogWriter: buffer growth failed: "} + e.what());
      return;
    }
  }

  std::memcpy(active->data.data() + active->used, &header, sizeof(header));
  if (!payload.empty()) {
    std::memcpy(active->data.data() + active->used + sizeof(header),
                payload.data(), payload.size());
  }
  active->used += total;
}

void LogWriter::State::drain() noexcept {
  submit_active();
  acquire_chunk();

  if (!options.background) {
    return;
  }

  std::unique_lock lock{mutex};
  pending_ready.notify_all();
  chunk_ready.wait(lock, [this] { return (pending.empty() && !writing); });
}

void LogWriter::State::stop_writer() noexcept {
  if (!writer.joinable()) {
    return;
  }

  {
    std::lock_guard lock{mutex};
    stopping = true;
  }

  pending_ready.notify_all();
  writer.join();
}

void LogWriter::start(const Manifest& manifest, MonotonicTime start_time) {
  State& state = *state_;

  std::vector<ManifestEntry> entries;
  entries.reserve(manifest.size());
  for (const auto& reg : manifest) {
    ManifestEntry entry{};
    entry.id = reg.id;
    entry.kind = static_cast<std::uint16_t>(reg.kind);
    entry.message_bytes = reg.message_bytes;
    entry.alignment = reg.alignment;
    entry.reserved = 0;
    entry.period_ns = reg.period_ns;
    entry.offset_ns = reg.offset_ns;
    write_name(entry.name, reg.name);
    entries.push_back(entry);
  }

  const std::span<const std::byte> manifest_bytes =
      as_bytes(entries.data(), entries.size() * sizeof(ManifestEntry));

  FileHeader header{};
  std::memcpy(header.magic, FILE_MAGIC, sizeof(header.magic));
  header.version = FORMAT_VERSION;
  header.header_bytes = sizeof(FileHeader);
  header.start_monotonic_ns = start_time.nanos();
  header.start_wall_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  header.manifest_count = static_cast<std::uint32_t>(entries.size());
  header.crc32 = crc32(manifest_bytes);
  write_name(header.process_name, state.process_name);

  // Written inline, before the writer thread exists, so the header can never
  // race a buffered chunk to the front of the file.
  state.write_all(&header, sizeof(header));
  if (!manifest_bytes.empty()) {
    state.write_all(manifest_bytes.data(), manifest_bytes.size());
  }

  if (!state.options.background || state.writer.joinable()) {
    return;
  }

  State* raw = state_.get();
  state.writer = std::thread{[raw] {
    std::unique_lock lock{raw->mutex};

    for (;;) {
      raw->pending_ready.wait(
          lock, [raw] { return !raw->pending.empty() || raw->stopping; });

      if (raw->pending.empty()) {
        if (raw->stopping) {
          return;
        }
        continue;
      }

      Chunk* chunk = raw->pending.front();
      raw->pending.pop_front();
      raw->writing = true;

      // The mutex is never held across the write, so a slow disk cannot make
      // the loop thread's hand-off block behind it.
      lock.unlock();
      raw->write_chunk(*chunk);
      lock.lock();

      raw->writing = false;
      raw->free_chunks.push_back(chunk);
      raw->chunk_ready.notify_all();
    }
  }};
}

void LogWriter::dispatch(const Context& context,
                         std::span<const std::byte> payload) noexcept {
  RecordHeader header{};
  header.kind = static_cast<std::uint16_t>(context.kind);
  header.source_id = context.source_id;
  header.dispatch_index = context.dispatch_index;
  header.event_time_ns = context.event_time.nanos();
  header.now_ns = context.now.nanos();
  header.sequence = context.sequence;
  header.aux = static_cast<std::uint32_t>(context.dropped);
  state_->append(header, payload);
}

void LogWriter::fetch(const Context& parent, std::uint16_t source_id,
                      FetchOutcome outcome, std::span<const std::byte> payload,
                      std::uint64_t sequence,
                      [[maybe_unused]] std::uint64_t dropped) noexcept {
  RecordHeader header{};
  header.kind = static_cast<std::uint16_t>(EventKind::FETCH);
  header.source_id = source_id;
  header.dispatch_index = parent.dispatch_index;
  header.event_time_ns = parent.event_time.nanos();
  header.now_ns = parent.now.nanos();
  header.sequence = sequence;
  header.aux = static_cast<std::uint32_t>(outcome);

  // A fetch that saw nothing has nothing to replay; force the payload empty
  // regardless of what the caller passed, so the on-disk record always
  // agrees with its own outcome.
  const std::span<const std::byte> recorded_payload =
      outcome == FetchOutcome::EMPTY ? std::span<const std::byte>{} : payload;
  state_->append(header, recorded_payload);
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
  header.sequence = context.sequence;
  header.aux = static_cast<std::uint32_t>(context.dropped);
  state_->append(header, std::span<const std::byte>{});
}

void LogWriter::flush() noexcept { state_->drain(); }

bool LogWriter::failed() const noexcept {
  std::lock_guard lock{state_->mutex};
  return state_->failed;
}

std::string LogWriter::error() const {
  std::lock_guard lock{state_->mutex};
  return state_->error;
}

std::uint64_t LogWriter::stalls() const noexcept {
  std::lock_guard lock{state_->mutex};
  return state_->stalls;
}

}  // namespace talos::event::log
