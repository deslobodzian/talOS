#pragma once

/*
 * Real Time Messaging System (RTMS)
 * RTMS is a shared memory single producer multiple consumer IPC method
 * Implementaion is based on https://csg.csail.mit.edu/6.823S17/StudyMaterials/quiz3/handouts/handout13-queue.pdf
*/

#include <cstddef>
#include <atomic>
#include <cstdint>

inline constexpr size_t CACHE_LINE = 64; // C++ I think as a function for this.
static constexpr size_t MAX_BUFFER_SIZE = 1024;
static constexpr size_t MAX_READERS = 8;

// Mesages will primarity be using flatbuffers but stored this rtms_msg_t objects
struct rtms_msg_t {
    void* data;
    size_t size;
};

enum class reader_status_t : std::uint64_t {
    FREE,
    CLAIMING,
    ACTIVE
};

struct alignas(CACHE_LINE) writer_t {
    std::atomic<uint64_t> sequence{};
};

struct alignas(CACHE_LINE) reader_t {
    std::atomic<uint64_t> sequence{};
    std::atomic<uint64_t> slot{};
    std::atomic<reader_status_t> status{};
};

struct rtms_header_t {
};

// Ring buffer queue,
struct rtms_queue_t {
    //alignas(std::atomic<uint64_t>)
};
