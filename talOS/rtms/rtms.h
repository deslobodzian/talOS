#pragma once

/*
 * Real Time Messaging System (RTMS)
 * RTMS is a shared memory single producer multiple consumer IPC method
 * Implementaion is based on https://csg.csail.mit.edu/6.823S17/StudyMaterials/quiz3/handouts/handout13-queue.pdf
*/

#include <cstddef>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <span>
#include <concepts>
#include "talOS/memory/shared_memory_ptr.h"

inline constexpr size_t CACHE_LINE = 64; // C++ I think as a function for this.
inline constexpr size_t MAX_BUFFER_SIZE = 1u << 20; // 1MiB
inline constexpr size_t MASK = MAX_BUFFER_SIZE - 1; // Mask for fixed size
inline constexpr size_t MAX_READERS = 1;

// Mesages will primarity be using flatbuffers but stored this RTMSMessage object
struct RTMSMessage {
    size_t size;
    void* data;
};

enum class ReaderState : std::uint64_t {
    FREE,
    CLAIMING,
    ACTIVE
};

struct alignas(CACHE_LINE) Writer {
    std::atomic<uint64_t> sequence{};
};

struct alignas(CACHE_LINE) Reader {
    std::atomic<uint64_t> sequence{};
    std::atomic<ReaderState> state{};
};

struct RTMSHeader {
    // This will be here incase I want to my different sized buffers in the future
    std::uint64_t capacity;
    std::size_t message_size; // message size will be fixes for each buffer

    Writer writer;
    Reader readers[MAX_READERS];
};

//template <typename T>
//concept NotDerivedFromTable = !std::is_base_of_v<flatbuffers::Table, T>;

// flatbuffer message?
//template <NotDerivedFromTable Message>
template <typename F, class T = const std::byte>
concept TakesSpan = std::invocable<F, std::span<T>>;

// Ring buffer queue, should flatbuffers requirement
// be set here or be generic for any data?
class RTMSQueue {
public:
    explicit RTMSQueue(std::string_view path, off_t message_size);

    std::uint64_t minimum_read_position() const;
    void write(const RTMSMessage& message);
    // We want the option to have copy free interactions, as such we pass a functor
    // you are able to copy the data with the functor if you want or just do a quick operation and leave.
    // e.i schedule an action based on the results of the message.
    template<TakesSpan Callback>
    bool read(std::uint64_t reader_id, Callback&& callback) {
        auto& reader = header_->readers[reader_id];
        const std::uint64_t message_size = header_->message_size;

        std::uint64_t reader_position = reader.sequence.load(std::memory_order_relaxed);
        const std::uint64_t writer_position = header_->writer.sequence.load(std::memory_order_acquire);

        if (reader_position == writer_position) {
            std::cerr << "Reader and Writer at same position!\n";
        }

        std::uint64_t offset = reader_position & MASK;
        const std::uint64_t remaining = header_->capacity - offset;

        // handle wrap around
        if (remaining < message_size) {
            reader_position += message_size;
            offset = 0;
            reader.sequence.store(reader_position, std::memory_order_release);
            // We need to try again since its possible that the reader == writer position
            return read(reader_id, std::forward<Callback>(callback));
        }

        callback(
            std::span<const std::byte>{
                static_cast<std::byte*>(ptr_.ptr()) + sizeof(RTMSHeader) + offset,
                header_->message_size
            }
        );

        const uint64_t next_position = reader_position + message_size;
        reader.sequence.store(next_position, std::memory_order_release);

        return true;

    }
    size_t message_size() const { return message_size_; }
    std::string_view path() const { return path_; }

private:
    std::string path_;
    std::size_t message_size_;
    RTMSHeader* header_;
    SharedMemoryPtr ptr_;
};
