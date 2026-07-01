#include "rtms.h"
#include <atomic>
#include <cstdint>
#include <cstring>
#include <iostream>



RTMSQueue::RTMSQueue(std::string_view path,
                     off_t slots,
                     off_t message_size)
    : path_(path),
      message_size_(message_size),
      ptr_(
          path_.c_str(),
          sizeof(RTMSHeader) + (slots * message_size_)) {

    header_ = reinterpret_cast<RTMSHeader*>(ptr_.ptr());

    header_->slots = slots;
    header_->slot_size = message_size_;
}

RTMSQueue::RTMSQueue(std::string_view path, off_t message_size) :
    RTMSQueue(path, MAX_SLOTS, message_size) {
}

// Need to find out where the "slowest" reader is.
uint64_t RTMSQueue::minimum_read_position() const {
    std::uint64_t minimum = header_->writer.sequence.load(std::memory_order_relaxed);

    for (int i = 0; i < MAX_READERS; ++i) {
        const auto state = header_->readers[i].state.load(std::memory_order_acquire);
        // Reader hasn't started yet, so position isn't relevent
        if (state != ReaderState::ACTIVE) {
            continue;
        }
        const auto position = header_->readers[i].sequence.load(std::memory_order::acquire);
        minimum = std::min(minimum, position);
    }
    return minimum;
}

void RTMSQueue::write(const RTMSMessage& message) {
    const std::uint64_t slot_size = header_->slot_size;
    const std::uint64_t slots = header_->slots;
    const std::uint64_t slot_mask = slots - 1;

    if (message.size > slot_size) {
        std::cerr << "Cannot write message larger than slot size\n";
        return;
    }

    uint64_t writer_position = header_->writer.sequence.load(std::memory_order_relaxed);
    std::printf("Writer position: %llu\n", writer_position);

    const uint64_t slowest_reader = minimum_read_position();
    const uint64_t next_position = writer_position + 1;

    // Producer at the moment cannot pass a slow reader?
    // example max slots is 10, min reader is at 10 and new position is wrapping to 20
    // will cause us to be writting over the reader.
    if (next_position - slowest_reader > slots) {
        std::cerr << "writer cannot pass the slowest reader\n"; // kill the reader?
        return;
    }

    const std::uint64_t slot_index = writer_position & slot_mask;
    // Cannot add offset to void* should I just make this a std::byte* by defualt?
    // or maybe go back to template SharedMemoryPtr? TODO:?
    auto* next_segment = static_cast<std::byte*>(ptr_.ptr()) + sizeof(RTMSHeader) + (slot_index * slot_size);
    std::memcpy(next_segment, message.data, message.size);
    header_->writer.sequence.store(next_position, std::memory_order_release);
}

