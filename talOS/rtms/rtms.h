#pragma once

/*
 * Real Time Messaging System (RTMS)
 * RTMS is a shared memory single producer multiple consumer IPC method
 * Implementaion is based on https://csg.csail.mit.edu/6.823S17/StudyMaterials/quiz3/handouts/handout13-queue.pdf
*/

#include <cstddef>
#include <atomic>
#include <cstdint>
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

enum class WriteResult {
    SUCCESS,
    FAILURE,
    BUFFER_FULL,
    ERROR_SIZE_MISSMATCH,
};

// Result of a write plus the sequence the message was published under.
// `sequence` is only meaningful when `result == WriteResult::SUCCESS`.
// Implicitly converts to WriteResult so existing call sites keep working.
struct WriteStatus {
    WriteResult result{WriteResult::FAILURE};
    std::uint64_t sequence{0};

    explicit constexpr operator WriteResult() const noexcept { return result; }

    friend constexpr bool operator==(
        const WriteStatus& status,
        WriteResult result) noexcept {
        return status.result == result;
    }
};

enum class ReadResult {
    OK,
    EMPTY,        // Caught up with the writer, nothing new.
    INACTIVE,     // Reader slot is not registered.
    INVALID,      // Reader id out of range or destination too small.
    TORN,         // Writer lapped us mid-copy too many times to recover.
};

// Metadata that accompanies a message handed to a reader.
// `dropped` counts messages the reader skipped past without observing, either
// because the writer lapped it (OVERWRITE_OLDEST) or because ReadMode::LATEST
// intentionally jumps to the newest message.
struct MessageInfo {
    std::uint64_t sequence{0};
    std::uint64_t dropped{0};
};

// For when a reader becomes too slow and our buffer is full
enum class OverflowPolicy {
    DROP_NEWEST,
    OVERWRITE_OLDEST,
    REMOVE_SLOWEST_READER
};

enum class ReadMode {
    LATEST,
    SEQUENCE,
};

struct RTMSOptions {
    OverflowPolicy overflow_policy{OverflowPolicy::OVERWRITE_OLDEST};
    ReadMode read_mode{ReadMode::LATEST};

    // Set by the publisher, which owns a topic's layout. A segment left behind
    // by a killed process from an older build has the wrong shape, and every
    // process that follows would fail to attach to it until someone unlinked
    // it by hand. The owner reclaims such a segment instead. Readers leave it
    // false: a reader disagreeing with a live publisher is a version skew that
    // must be reported, not papered over.
    bool reclaim_mismatched_segment{false};
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
    std::uint64_t message_bytes{0}; // message size will be fixes for each buffer
    std::uint64_t message_alignment{0};

    std::uint64_t data_offset{0}; // offset from header
    std::uint64_t slot_stride{0};

    Writer writer;
    Reader readers[MAX_READERS];
};

template <typename F, class T = const std::byte>
concept TakesSpan = std::invocable<F, std::span<T>>;

// Ring buffer queue, should flatbuffers requirement
// be set here or be generic for any data?
class RTMSQueue {
public:
    RTMSQueue(
        std::string_view path,
        std::size_t message_size,
        std::size_t message_alignment,
        std::size_t slots = MAX_SLOTS,
        RTMSOptions options = RTMSOptions{}
    );

    RTMSQueue(RTMSQueue&&) noexcept = default;
    RTMSQueue& operator=(RTMSQueue&&) noexcept = default;

    RTMSQueue(const RTMSQueue&) = delete;
    RTMSQueue& operator=(const RTMSQueue&) = delete;

    std::uint64_t minimum_read_position() const;
    std::optional<std::size_t> register_reader();
    void release_reader(std::size_t id);
    WriteStatus write(const RTMSMessage& message);

    // Copies the next message for `reader_id` into `destination` and reports
    // its sequence plus how many messages were skipped to get there.
    //
    // Unlike read(), this is safe against a writer that laps the reader: the
    // slot is copied first and the copy is only accepted if the writer did not
    // reach it during the copy (a seqlock, using the writer sequence that the
    // frozen shared-memory layout already provides).
    ReadResult read_next(
        std::uint64_t reader_id,
        std::span<std::byte> destination,
        MessageInfo& info);

    std::uint64_t writer_sequence() const {
        return header_->writer.sequence.load(std::memory_order_acquire);
    }

    std::uint64_t reader_sequence(std::uint64_t reader_id) const {
        return header_->readers[reader_id].sequence.load(
            std::memory_order_acquire);
    }

    std::uint64_t slots() const { return slots_; }
    // We want the option to have copy free interactions, as such we pass a functor
    // you are able to copy the data with the functor if you want or just do a quick operation and leave.
    // e.i schedule an action based on the results of the message.
    // Zero-copy read. The callback is handed a view directly into shared
    // memory, so with OverflowPolicy::OVERWRITE_OLDEST the writer may lap the
    // reader while the callback runs. The overwrite is detected afterwards and
    // the callback is retried, so:
    //
    //   the callback MUST NOT commit side effects; only data copied out of a
    //   read() that returned true is valid.
    //
    // read_next() has no such caveat and reports dropped messages, so prefer
    // it unless the copy genuinely matters.
    template<TakesSpan Callback>
    bool read(std::uint64_t reader_id, Callback&& callback) {
        if (reader_id >= MAX_READERS) {
            return false;
        }

        auto& reader = header_->readers[reader_id];

        if (reader.state.load(std::memory_order_acquire) !=
            ReaderState::ACTIVE) {
            return false;
        }

        for (int attempt = 0; attempt < MAX_READ_ATTEMPTS; ++attempt) {
            MessageInfo info{};

            if (!select_sequence(reader, info)) {
                return false;
            }

            callback(
                std::span<const std::byte>{
                    slot_address(info.sequence),
                    header_->message_bytes
                });

            if (!slot_still_valid(info.sequence)) {
                continue;
            }

            // The cursor always points to the next sequence to consume.
            reader.sequence.store(
                info.sequence + 1,
                std::memory_order_release);

            return true;
        }

        return false;
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
    // Number of times a read retries when the writer laps it mid-read.
    static constexpr int MAX_READ_ATTEMPTS = 8;

    // Chooses the sequence this reader should observe next and fills in how
    // many messages were skipped to reach it. Returns false when the reader is
    // caught up with the writer.
    bool select_sequence(const Reader& reader, MessageInfo& info) const;

    // True when the writer has not yet reached the slot backing
    // `message_sequence`, i.e. the bytes just read were not overwritten.
    bool slot_still_valid(std::uint64_t message_sequence) const;

    const std::byte* slot_address(std::uint64_t message_sequence) const;

    std::string path_{""};
    std::uint64_t slots_{0};
    std::uint64_t message_size_{0};
    std::uint64_t message_alignment_{0};
    std::uint64_t data_offset_{0};
    std::uint64_t stride_{0};
    std::uint64_t total_bytes_{0};

    RTMSOptions options_{};

    RTMSHeader* header_{nullptr};
    SharedMemoryPtr ptr_;
};
