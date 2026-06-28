#pragma once

/*
 * Real Time Messaging System (RTMS)
 * RTMS is a shared memory single producer multiple consumer IPC method
 * Implementaion is based on https://csg.csail.mit.edu/6.823S17/StudyMaterials/quiz3/handouts/handout13-queue.pdf
*/

#include <cstddef>
#include <atomic>
#include <cstdint>
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

// Ring buffer queue, should flatbuffers requirement
// be set here or be generic for any data?
class RTMSQueue {
public:
    explicit RTMSQueue(std::string_view path, off_t message_size);

    std::uint64_t minimum_read_position() const;
    void write(const RTMSMessage& message);
    RTMSMessage read(std::uint64_t reader_id);
    size_t message_size() const { return message_size_; }
    std::string_view path() const { return path_; }

private:
    std::string path_;
    std::size_t message_size_;
    RTMSHeader* header_;
    SharedMemoryPtr ptr_;
};
