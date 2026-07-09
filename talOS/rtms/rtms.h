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
#include <optional>
#include <cinttypes>

inline constexpr size_t CACHE_LINE = 64; // C++ I think as a function for this.
inline constexpr size_t MAX_SLOTS = 1u << 10; // 1024 slots maximum
inline constexpr size_t MASK = MAX_SLOTS - 1; // Mask for fixed size
inline constexpr size_t MAX_READERS = 8;

inline constexpr std::size_t align_up(
    std::size_t value,
    std::size_t alignment) {
    return (value + alignment - 1) & ~(alignment - 1);
}

static inline bool is_pow_2(std::size_t n) {
    return (n > 0) && (n & (n - 1)) == 0;
}


// Mesages will primarity be using flatbuffers but stored this RTMSMessage object
// These are just views, we essentilaly will only copy the data into the queue
// or we will copy the view out
// TODO: read already uses std::span<const std::byte> should just remove this
struct RTMSMessage {
    size_t size;
    const void* data;
};

enum class ReaderState : std::uint64_t {
    FREE,
    CLAIMING,
    ACTIVE
};

constexpr const char* to_string(ReaderState state) noexcept {
    switch (state) {
        case ReaderState::FREE:     return "FREE";
        case ReaderState::CLAIMING: return "CLAIMING";
        case ReaderState::ACTIVE:   return "ACTIVE";
    }
    return "UNKNOWN";
}

struct alignas(CACHE_LINE) Writer {
    std::atomic<uint64_t> sequence{};
};

struct alignas(CACHE_LINE) Reader {
    std::atomic<uint64_t> sequence{};
    std::atomic<ReaderState> state{};
};

struct RTMSHeader {
    std::uint64_t total_bytes{0};

    std::uint64_t slots{0}; // power of 2 for efficiency
    std::uint64_t slot_bytes{0}; // message size will be fixes for each buffer
    std::uint64_t slot_alignment{0};

    std::uint64_t data_offset{0}; // offset from header
    std::uint64_t slot_stride{0};

    Writer writer;
    Reader readers[MAX_READERS];
};

//template <typepath T>
//concept NotDerivedFromTable = !std::is_base_of_v<flatbuffers::Table, T>;

// flatbuffer message?
//template <NotDerivedFromTable Message>
template <typename F, class T = const std::byte>
concept TakesSpan = std::invocable<F, std::span<T>>;

// Ring buffer queue, should flatbuffers requirement
// be set here or be generic for any data?
class RTMSQueue {
public:
    static RTMSQueue create(
        std::string_view path,
        std::size_t message_size,
        std::size_t slot_alignment,
        std::size_t slots = MAX_SLOTS
    );

    static RTMSQueue attach(
        std::string_view path,
        std::size_t message_size,
        std::size_t slot_alignment,
        std::size_t slots = MAX_SLOTS
    );

    RTMSQueue(RTMSQueue&&) noexcept = default;
    RTMSQueue& operator=(RTMSQueue&&) noexcept = default;

    RTMSQueue(const RTMSQueue&) = delete;
    RTMSQueue& operator=(const RTMSQueue&) = delete;

    std::uint64_t minimum_read_position() const;
    std::optional<std::size_t> register_reader();
    void write(const RTMSMessage& message);
    // We want the option to have copy free interactions, as such we pass a functor
    // you are able to copy the data with the functor if you want or just do a quick operation and leave.
    // e.i schedule an action based on the results of the message.
    template<TakesSpan Callback>
    bool read(std::uint64_t reader_id, Callback&& callback) {
        auto& reader = header_->readers[reader_id];

        std::uint64_t reader_position = reader.sequence.load(std::memory_order_relaxed);
        //const std::uint64_t writer_position = header_->writer.sequence.load(std::memory_order_acquire);

        //if (reader_position == writer_position) {
        //    std::cerr << "Reader and Writer at same position!\n";
        //    return false;
        //}

        const uint64_t mask = header_->slots - 1;
        std::uint64_t slot_index = reader_position & mask;

        callback(
            std::span<const std::byte>{
                static_cast<std::byte*>(ptr_.ptr()) + sizeof(RTMSHeader) + (slot_index * header_->slot_alignment),
                header_->slot_alignment
            }
        );

        reader.sequence.store(reader_position + 1, std::memory_order_release);

        return true;

    }
    size_t message_size() const { return message_size_; }
    std::string_view path() const { return path_; }

    void print_header_state() const {
        if (header_ == nullptr) {
            std::printf("RTMS header is not mapped\n");
            return;
        }

        std::printf("========================================\n");
        std::printf(
            "Writer Sequence: %" PRIu64 "\n",
            header_->writer.sequence.load(std::memory_order_relaxed)
        );

        std::printf("----------------------------------------\n");
        std::printf("Readers:\n");

        for (std::size_t i = 0; i < MAX_READERS; ++i) {
            const std::uint64_t seq =
                header_->readers[i].sequence.load(std::memory_order_relaxed);

            const ReaderState state =
                header_->readers[i].state.load(std::memory_order_relaxed);

            std::printf(
                "  [%zu] Seq: %" PRIu64 " | State: %s\n",
                i,
                seq,
                to_string(state)
            );
        }

        std::printf("========================================\n");
    }

private:
    RTMSQueue(
        std::string_view path,
        std::size_t slots,
        std::size_t message_size,
        std::size_t slot_alignment,
        SharedMemoryMode mode);


    std::string path_{""};
    std::uint64_t slots_{0};
    std::uint64_t message_size_{0};
    std::uint64_t slot_alignment_{0};
    std::uint64_t data_offset_{0};
    std::uint64_t stride_{0};
    std::uint64_t total_bytes_{0};

    RTMSHeader* header_{nullptr};
    SharedMemoryPtr ptr_;
};
