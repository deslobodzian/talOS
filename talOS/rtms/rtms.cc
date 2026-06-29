#include "rtms.h"
#include <atomic>
#include <cstring>
#include <iostream>


RTMSQueue::RTMSQueue(std::string_view path, off_t message_size) :
    path_(path),
    message_size_(message_size),
    ptr_(
        path_.c_str(),
        static_cast<off_t>(sizeof(RTMSHeader)) + MAX_BUFFER_SIZE) {
    header_ = static_cast<RTMSHeader*>(ptr_.ptr());
    header_->capacity = MAX_BUFFER_SIZE;
    header_->message_size = message_size_;
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
    if (message.size > header_->capacity) {
        std::cerr << "Cannot write message larger than capacity\n";
        return;
    }

    uint64_t writer_position = header_->writer.sequence.load(std::memory_order_relaxed);
    std::printf("Writer position: %llu\n", writer_position);

    const std::uint64_t offset = writer_position & MASK;
    const std::uint64_t remaining = header_->capacity - offset;

    const bool wraps = message.size > remaining;
    const std::uint64_t total_space_needed =
        wraps ? remaining + message.size : message.size;

    const uint64_t slowest_reader = minimum_read_position();

    // Producer cannot pass slowest reader
    if (writer_position + total_space_needed - slowest_reader > header_->capacity) {
        std::cerr << "writer cannot pass the slowest reader\n"; // kill the reader?
        return;
    }

    // Cannot add offset to void* should I just make this a std::byte* by defualt?
    // or maybe go back to template SharedMemoryPtr? TODO:?
    auto* next_segment = static_cast<std::byte*>(ptr_.ptr()) + sizeof(RTMSHeader) + offset;
    std::memcpy(next_segment, message.data, message.size);

    const uint64_t next_position = writer_position + header_->message_size;
    header_->writer.sequence.store(next_position, std::memory_order_release);
}

