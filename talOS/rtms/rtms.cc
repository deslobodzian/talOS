#include "rtms.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "talOS/memory/shared_memory_ptr.h"

RTMSQueue::RTMSQueue(std::string_view path,
                     std::size_t message_size,
                     std::size_t message_alignment,
                     std::size_t slots)
    : path_{path},
      slots_{slots},
      message_size_{message_size},
      message_alignment_{message_alignment},
      data_offset_{align_up(sizeof(RTMSHeader), message_alignment)},
      stride_{align_up(message_size, message_alignment)},
      total_bytes_{data_offset_ + stride_ * slots_},
      ptr_{path_, total_bytes_} {
  if (!is_pow_2(slots)) {
    throw std::invalid_argument("slots is not a power of 2!");
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

void RTMSQueue::write(const RTMSMessage& message) {
  const std::uint64_t slot_mask = slots_ - 1;

  if (message.size > header_->message_bytes) {
    std::cerr << "Cannot write message larger than slot size\n"
              << "message_bytes: " << header_->message_bytes
              << " > message_size: " << message.size << "\n";
    return;
  }

  uint64_t writer_position =
      header_->writer.sequence.load(std::memory_order_relaxed);
  // std::printf("Writer position: %llu\n", writer_position);

  const uint64_t slowest_reader = minimum_read_position();
  const uint64_t next_position = writer_position + 1;

  // Producer at the moment cannot pass a slow reader?
  // example max slots is 10, min reader is at 10 and new position is wrapping
  // to 20 will cause us to be writting over the reader.
  if (next_position - slowest_reader > slots_) {
    std::cerr << "writer cannot pass the slowest reader\n";  // kill the reader?
    return;
  }

  const std::uint64_t slot_index = writer_position & slot_mask;
  // Cannot add offset to void* should I just make this a std::byte* by defualt?
  // or maybe go back to template SharedMemoryPtr? TODO:?
  auto* base = static_cast<std::byte*>(ptr_.ptr());
  auto* next_segment =
      base + header_->data_offset + (slot_index * header_->slot_stride);

  std::memcpy(next_segment, message.data, message.size);
  header_->writer.sequence.store(next_position, std::memory_order_release);
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
