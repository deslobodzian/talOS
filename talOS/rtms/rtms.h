#pragma once

/*
 * Real Time Messaging System (RTMS)
 * RTMS is a shared memory single producer multiple consumer IPC method
 * Implementaion is based on https://csg.csail.mit.edu/6.823S17/StudyMaterials/quiz3/handouts/handout13-queue.pdf
*/

#include <cstddef>
#include <atomic>
#include <cstdint>
#include <string>
#include "talOS/memory/shared_memory_ptr.h"

inline constexpr size_t CACHE_LINE = 64; // C++ I think as a function for this.
inline constexpr size_t MAX_BUFFER_SIZE = 1u << 20; // 1MiB
inline constexpr size_t MASK = MAX_BUFFER_SIZE - 1; // Mask for fixed size
inline constexpr size_t MAX_READERS = 8;

// Mesages will primarity be using flatbuffers but stored this RTMSMessage object
struct RTMSMessage {
    size_t size;
    void* data;
};

enum class ReaderStatus : std::uint64_t {
    FREE,
    CLAIMING,
    ACTIVE
};

struct alignas(CACHE_LINE) Writer {
    std::atomic<uint64_t> sequence{};
};

struct alignas(CACHE_LINE) Reader {
    std::atomic<uint64_t> sequence{};
    std::atomic<uint64_t> slot{};
    std::atomic<ReaderStatus> status{};
};

struct RTMSHeader {
    // This will be here incase I want to my different sized buffers in the future
    std::uint64_t capacity;
    std::size_t message_size; // message size will be fixes for each buffer

    Writer writer;
    Reader readers[MAX_READERS];
};

// Ring buffer queue,
template <typename T>
class RTMSQueue {
public:
    explicit RTMSQueue(std::string_view path) :
        path_(path),
        ptr_(path_.c_str(), 100) {
    }
private:
    std::string path_;
    RTMSHeader header_;
    SharedMemoryPtr<void> ptr_;
};
