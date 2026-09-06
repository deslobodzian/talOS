#include "rtms.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "talOS/memory/shared_memory_ptr.h"

namespace {
std::string_view ValidatePath(std::string_view path) {
  std::string_view name = path;
  if (name.starts_with('/')) {
    name.remove_prefix(1);
  }
  if (name.size() > 30) {
    throw std::invalid_argument(
        "Topic name '" + std::string(path) +
        "' exceeds limit of 30 characters after leading slash");
  }
  return path;
}
}  // namespace

RTMSQueue::RTMSQueue(std::string_view path,
                     std::size_t message_size,
                     std::size_t message_alignment,
                     std::size_t slots,
                     RTMSOptions options
                     )
    : path_{ValidatePath(path)},
      slots_{slots},
      message_size_{message_size},
      message_alignment_{message_alignment},
      data_offset_{align_up(sizeof(RTMSHeader), message_alignment)},
      stride_{align_up(message_size, message_alignment)},
      total_bytes_{data_offset_ + stride_ * slots_},
      options_{options},
      ptr_{path_, total_bytes_} {
  if (!is_pow_2(slots)) {
    throw std::invalid_argument("slots is not a power of 2!");
  }

  // Lap recovery needs at least one slot the writer is not currently in.
  if (slots < 2) {
    throw std::invalid_argument("slots must be at least 2");
  }

  const std::size_t total_size = data_offset_ + stride_ * slots;

  if (ptr_.mode() == SharedMemoryMode::CREATE) {
    // Learned about this, pretty cool, it creates the object at preallocated
    // memory
    header_ = std::construct_at(static_cast<RTMSHeader*>(ptr_.ptr()));

    header_->total_bytes = total_size;
    header_->slots = slots;
    header_->message_bytes = message_size;
    header_->message_alignment = message_alignment;

    header_->data_offset = data_offset_;
    header_->slot_stride = stride_;

    header_->writer.sequence.store(0, std::memory_order_relaxed);
    for (auto& reader : header_->readers) {
      reader.sequence.store(0, std::memory_order_relaxed);
      reader.state.store(ReaderState::FREE, std::memory_order_relaxed);
    }
  } else {
    header_ = static_cast<RTMSHeader*>(ptr_.ptr());
    if (header_->slots != slots_ || header_->message_bytes != message_size_ ||
        header_->message_alignment != message_alignment_ ||
        header_->total_bytes != total_bytes_) {
      throw std::runtime_error("RTMS shared-memory layout mismatch");
    }
  }
}

// Need to find out where the "slowest" reader is.
uint64_t RTMSQueue::minimum_read_position() const {
  std::uint64_t minimum =
      header_->writer.sequence.load(std::memory_order_relaxed);

  for (std::size_t i = 0; i < MAX_READERS; ++i) {
    const auto state =
        header_->readers[i].state.load(std::memory_order_acquire);
    // Reader hasn't started yet, so position isn't relevent
    if (state != ReaderState::ACTIVE) {
      continue;
    }
    const auto position =
        header_->readers[i].sequence.load(std::memory_order_acquire);
    minimum = std::min(minimum, position);
  }
  return minimum;
}

WriteStatus RTMSQueue::write(const RTMSMessage& message) {
  const std::uint64_t slot_mask = slots_ - 1;

  if (message.size > header_->message_bytes) {
    std::cerr << "Cannot write message larger than slot size\n"
              << "message_bytes: " << header_->message_bytes
              << " > message_size: " << message.size << "\n";
    return {WriteResult::ERROR_SIZE_MISSMATCH, 0};
  }

  const uint64_t writer_position =
      header_->writer.sequence.load(std::memory_order_relaxed);

  const uint64_t next_position = writer_position + 1;

  // DROP_NEWEST is the reliable mode: the writer refuses to pass the slowest
  // reader, so no reader ever misses a message. Every other policy keeps the
  // writer running at the cost of lapping slow readers, which read_next()
  // reports back to them as dropped messages. A stuck subscriber must never
  // be able to stall a control loop.
  if (options_.overflow_policy == OverflowPolicy::DROP_NEWEST) {
    const uint64_t slowest_reader = minimum_read_position();

    // example max slots is 10, min reader is at 10 and new position is wrapping
    // to 20 will cause us to be writting over the reader.
    if (next_position - slowest_reader > slots_) {
      return {WriteResult::BUFFER_FULL, 0};
    }
  }

  const std::uint64_t slot_index = writer_position & slot_mask;
  // Cannot add offset to void* should I just make this a std::byte* by defualt?
  // or maybe go back to template SharedMemoryPtr? TODO:?
  auto* base = static_cast<std::byte*>(ptr_.ptr());
  auto* next_segment =
      base + header_->data_offset + (slot_index * header_->slot_stride);

  std::memcpy(next_segment, message.data, message.size);
  header_->writer.sequence.store(next_position, std::memory_order_release);
  return {WriteResult::SUCCESS, writer_position};
}

const std::byte* RTMSQueue::slot_address(
    std::uint64_t message_sequence) const {
  const std::uint64_t slot_index = message_sequence & (header_->slots - 1);

  return static_cast<const std::byte*>(ptr_.ptr()) + header_->data_offset +
         (slot_index * header_->slot_stride);
}

bool RTMSQueue::select_sequence(const Reader& reader, MessageInfo& info) const {
  const std::uint64_t reader_position =
      reader.sequence.load(std::memory_order_relaxed);

  const std::uint64_t writer_position =
      header_->writer.sequence.load(std::memory_order_acquire);

  // reader.sequence and writer.sequence both refer to the next sequence.
  if (reader_position >= writer_position) {
    return false;
  }

  std::uint64_t message_sequence = reader_position;

  // Under DROP_NEWEST the writer refuses to pass the slowest reader, so no
  // unread slot can ever be overwritten and nothing may be skipped. Skipping
  // here would silently break the guarantee that makes that mode worth having.
  //
  // Under an overwriting policy the writer is free to lap us. Only the last
  // `slots` sequences are still intact, and the oldest of those lives in the
  // slot the writer is about to reuse, so recovery targets the one after it.
  if (options_.overflow_policy != OverflowPolicy::DROP_NEWEST &&
      writer_position - message_sequence >= header_->slots) {
    message_sequence = writer_position - header_->slots + 1;
  }

  if (options_.read_mode == ReadMode::LATEST) {
    message_sequence = writer_position - 1;
  }

  info.sequence = message_sequence;
  info.dropped = message_sequence - reader_position;
  return true;
}

bool RTMSQueue::slot_still_valid(std::uint64_t message_sequence) const {
  // A writer that cannot pass the slowest reader cannot have been writing the
  // slot we just read, so there is nothing to re-check.
  if (options_.overflow_policy == OverflowPolicy::DROP_NEWEST) {
    return true;
  }

  // Pairs with the writer's release store. Everything read out of the slot
  // must be ordered before this load, or the check proves nothing.
  std::atomic_thread_fence(std::memory_order_acquire);

  const std::uint64_t writer_position =
      header_->writer.sequence.load(std::memory_order_relaxed);

  // The writer reaches our slot again at message_sequence + slots. If its next
  // write is at or past that point, it may have been writing while we read.
  return writer_position - message_sequence < header_->slots;
}

ReadResult RTMSQueue::read_next(std::uint64_t reader_id,
                                std::span<std::byte> destination,
                                MessageInfo& info) {
  info = MessageInfo{};

  if (reader_id >= MAX_READERS ||
      destination.size() < header_->message_bytes) {
    return ReadResult::INVALID;
  }

  auto& reader = header_->readers[reader_id];

  if (reader.state.load(std::memory_order_acquire) != ReaderState::ACTIVE) {
    return ReadResult::INACTIVE;
  }

  for (int attempt = 0; attempt < MAX_READ_ATTEMPTS; ++attempt) {
    MessageInfo candidate{};

    if (!select_sequence(reader, candidate)) {
      return ReadResult::EMPTY;
    }

    std::memcpy(destination.data(), slot_address(candidate.sequence),
                header_->message_bytes);

    if (!slot_still_valid(candidate.sequence)) {
      continue;
    }

    // The cursor always points to the next sequence to consume.
    reader.sequence.store(candidate.sequence + 1, std::memory_order_release);

    info = candidate;
    return ReadResult::OK;
  }

  return ReadResult::TORN;
}

std::optional<std::size_t> RTMSQueue::register_reader() {
  for (std::size_t i = 0; i < MAX_READERS; ++i) {
    auto& reader = header_->readers[i];
    ReaderState expected = ReaderState::FREE;

    if (!reader.state.compare_exchange_strong(expected, ReaderState::CLAIMING,
                                              std::memory_order_acq_rel,
                                              std::memory_order_relaxed)) {
      continue;
    }

    const auto current =
        header_->writer.sequence.load(std::memory_order_acquire);

    reader.sequence.store(current, std::memory_order_relaxed);

    reader.state.store(ReaderState::ACTIVE, std::memory_order_release);
    return i;
  }
  return std::nullopt;
}

void RTMSQueue::release_reader(std::size_t id) {
  auto& reader = header_->readers[id];
  reader.state.store(ReaderState::FREE, std::memory_order_release);
}
