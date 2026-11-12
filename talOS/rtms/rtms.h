#pragma once

/*
 * Real Time Messaging System (RTMS)
 * RTMS is a shared memory single producer multiple consumer IPC method
 * Implementaion is based on
 * https://csg.csail.mit.edu/6.823S17/StudyMaterials/quiz3/handouts/handout13-queue.pdf
 */

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "talOS/memory/shared_memory_ptr.h"
#include "talOS/rtms/process_liveness.h"
#include "talOS/rtms/wake.h"
#include "talOS/schema/identity.h"

inline constexpr size_t CACHE_LINE = 64;  // C++ I think as a function for this.
inline constexpr size_t MAX_SLOTS = 1u << 10;  // 1024 slots maximum
inline constexpr size_t MASK = MAX_SLOTS - 1;  // Mask for fixed size
inline constexpr size_t MAX_READERS = 8;

inline constexpr std::size_t align_up(std::size_t value,
                                      std::size_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

// Each slot keeps the payload first, then an aligned int64 publication time.
inline constexpr std::size_t rtms_slot_alignment(std::size_t alignment) {
  return alignment < alignof(std::int64_t) ? alignof(std::int64_t) : alignment;
}

inline constexpr std::size_t rtms_publish_time_offset(
    std::size_t message_size) {
  return align_up(message_size, alignof(std::int64_t));
}

inline constexpr std::size_t rtms_slot_stride(std::size_t message_size,
                                              std::size_t alignment) {
  return align_up(rtms_publish_time_offset(message_size) + sizeof(std::int64_t),
                  rtms_slot_alignment(alignment));
}

static_assert(sizeof(std::int64_t) == 8);
static_assert(rtms_slot_stride(1, 1) == 16);
static_assert(rtms_slot_stride(8, 4) == 16);
static_assert(rtms_slot_stride(16, 8) == 24);
static_assert(rtms_slot_stride(16, 16) == 32);

static inline bool is_pow_2(std::size_t n) {
  return (n > 0) && (n & (n - 1)) == 0;
}

// A reader slot's state word holds the state in its low two bits and the
// process word (kProcessPidBits) of the process that claimed it above them,
// so a slot whose process died -- killed mid-read, or exited without
// releasing -- can be recognised and freed (see
// RTMSQueue::reclaim_dead_readers). FREE is the word 0.
enum class ReaderState : std::uint64_t { FREE, CLAIMING, ACTIVE };

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

  friend constexpr bool operator==(const WriteStatus& status,
                                   WriteResult result) noexcept {
    return status.result == result;
  }
};

enum class ReadResult {
  OK,
  EMPTY,     // Caught up with the writer, nothing new.
  INACTIVE,  // This queue does not hold the reader slot: never registered,
             // released, or freed as dead (and maybe claimed by another
             // process) since; stays so until register_reader() returns it.
  INVALID,   // Reader id out of range or destination too small.
  TORN,      // Writer lapped us mid-copy too many times to recover.
};

// Metadata that accompanies a message handed to a reader.
// `dropped` counts messages the reader skipped past without observing, either
// because the writer lapped it -- the writer overwrote them, whatever this
// reader's own policy assumed -- or because ReadMode::LATEST intentionally
// jumps to the newest message.
struct MessageInfo {
  std::uint64_t sequence{0};
  std::uint64_t dropped{0};
  // Producer's monotonic clock, captured when this message was published.
  std::int64_t publish_time_ns{0};
};

// For when a reader becomes too slow and our buffer is full
enum class OverflowPolicy {
  DROP_NEWEST,
  OVERWRITE_OLDEST,
};

enum class ReadMode {
  LATEST,
  SEQUENCE,
};

struct RTMSOptions {
  OverflowPolicy overflow_policy{OverflowPolicy::OVERWRITE_OLDEST};
  ReadMode read_mode{ReadMode::LATEST};

  // This end writes the topic: it claims the segment's writer on open, and a
  // second live writer is refused. Like any attach, it fails on a segment of
  // another version, layout or schema rather than replace it.
  bool publisher{false};

  // This queue creates the segment for a session and is the only one that
  // removes it (the launcher; see "Segment lifetime" below). It replaces
  // whatever segment had the name, and its destructor unlinks the segment.
  bool session_owner{false};
};

// The segment format, checked before any other header field is trusted.
inline constexpr std::uint64_t kRtmsMagic =
    0x32534D54534F4C41ULL;  // "ALOSTMS2"
inline constexpr std::uint64_t kRtmsVersion = 8;

// RTMSHeader::lifetime: a session-owned flag, a closing flag, and the count
// of queues attached to a segment that is not session-owned.
inline constexpr std::uint64_t kLifetimeSessionOwned = 1ULL << 63;
inline constexpr std::uint64_t kLifetimeClosing = 1ULL << 62;
inline constexpr std::uint64_t kLifetimeCountMask = (1ULL << 32) - 1;

// The segment records the writer's owner and each reader slot's claimant as
// a process word (kProcessPidBits, process_liveness.h): pid, start time and
// pid namespace. A reader's state word holds a process word above its two
// state bits.
static_assert(kProcessPidBits + kProcessStartBits + kProcessNamespaceBits + 2 ==
              64);

struct alignas(CACHE_LINE) Writer {
  std::atomic<uint64_t> sequence{};
  // Exclusive publisher, 0 when unclaimed; dead owners may be replaced.
  // Despite its name, a process word (process_liveness.h), not a bare pid.
  std::atomic<uint64_t> owner_pid{};
  // Exact completed DROP_NEWEST publication, or 0 when overwriting may have
  // touched its oldest slot. Allows reliable readers to use the full ring.
  std::atomic<uint64_t> reliable_sequence{};
  // Bit i set: reader slot i is asleep until the next publication and wants
  // a wake (RTMSQueue::arm_wake). The writer takes the whole word with one
  // exchange after publishing and notifies each set slot's wake_id.
  std::atomic<uint64_t> waiters{};
};

static_assert(sizeof(Writer) == CACHE_LINE);

struct alignas(CACHE_LINE) Reader {
  std::atomic<uint64_t> sequence{};
  std::atomic<uint64_t> state{};  // (process word << 2) | ReaderState
  // The talos::WakeReceiver id the slot's holder sleeps on, 0 for none.
  std::atomic<uint64_t> wake_id{};
};

static_assert(sizeof(Reader) == CACHE_LINE);

struct RTMSHeader {
  // kRtmsMagic, stored last (release) by the creator: an attacher trusts no
  // other field until it reads this.
  std::atomic<std::uint64_t> magic{0};
  std::uint64_t version{0};
  // Random at creation: tells this segment from a later one under the name.
  std::uint64_t segment_id{0};

  std::uint64_t total_bytes{0};

  std::uint64_t slots{0};          // power of 2 for efficiency
  std::uint64_t message_bytes{0};  // message size will be fixes for each buffer
  std::uint64_t message_alignment{0};

  std::uint64_t data_offset{0};  // offset from header
  std::uint64_t slot_stride{0};

  // The creator's schema identity (talOS/schema/identity.h). Every attach
  // compares its own against it, so two same-size types cannot share a topic.
  std::uint64_t schema_hash{0};
  char schema_name[talos::schema::kMaxSchemaName + 1]{};

  std::atomic<std::uint64_t> lifetime{0};

  Writer writer;
  Reader readers[MAX_READERS];
};

static_assert(sizeof(RTMSHeader) == 832);

// Ring buffer queue over one named shared-memory segment.
//
// Schema identity. The creator writes its schema name and hash into the
// header; every later attach with a different hash throws, naming both
// types, even when the sizes agree.
//
// Segment lifetime. Under the launcher a session owns every declared topic:
// the launcher creates each segment (session_owner) before it spawns a node
// and unlinks it when the session ends, so nodes only attach, a node that
// exits or is killed and restarted leaves its topics in place, and a
// publisher's clean exit never severs its subscribers. Without a launcher
// (a node run directly, Sim-less tools such as send_target or monitor,
// tests) the first queue to open a name creates the segment and the last
// one to close it unlinks it; a segment whose last holder was killed stays
// until a launcher session replaces it, someone unlinks it, or the machine
// restarts, and is reused meanwhile. A last holder killed between
// marking the segment closing and unlinking it never unlinks it: the next
// queue to open the name waits out that unlink and then, when no live
// process holds the segment, unlinks it itself.
class RTMSQueue {
 public:
  RTMSQueue(std::string_view path, std::size_t message_size,
            std::size_t message_alignment,
            const talos::schema::Identity& schema,
            std::size_t slots = MAX_SLOTS, RTMSOptions options = RTMSOptions{});

  RTMSQueue(RTMSQueue&& other) noexcept;
  RTMSQueue& operator=(RTMSQueue&& other) noexcept;
  ~RTMSQueue();

  RTMSQueue(const RTMSQueue&) = delete;
  RTMSQueue& operator=(const RTMSQueue&) = delete;

  std::uint64_t minimum_read_position() const;
  // Claims a reader slot for this process. When none is free, slots held by
  // processes that no longer exist are freed first. The queue keeps the
  // state word it claimed the slot with: once the slot holds another,
  // read_next() reports INACTIVE rather than share another claimant's cursor.
  std::optional<std::size_t> register_reader();
  // Frees reader slot `id` if this queue claimed it and still holds it.
  void release_reader(std::size_t id);

  // Wake channel (README.md "Wake channel"). Records the wake receiver for
  // a reader slot this queue holds; 0 disables.
  void set_reader_wake(std::size_t reader_id, std::uint64_t wake_id);
  // Asks to be notified on the next publication, then reports whether the
  // reader is caught up: true means read_next() would return EMPTY and it is
  // safe to block on the receiver; false means there is something to read
  // (or the slot is no longer held: INACTIVE), so read instead of blocking.
  bool arm_wake(std::size_t reader_id);
  // Notifications this queue object has sent as a writer (process-local).
  std::uint64_t wakes_sent() const { return wakes_sent_; }
  // Of those, the ones lost for a reason other than the receiver being gone
  // (WakeOutcome::FAILED: no descriptor left, a permission refused, an
  // unexpected errno). Process-local; the first is also logged to stderr
  // with the topic and the error. Safe to read from any thread while the
  // writer publishes (a node's registry reporter sums it; RT-7).
  std::uint64_t wake_failures() const {
    return wake_failures_.load(std::memory_order_relaxed);
  }
  // Frees every reader slot whose claiming process has died. Returns how
  // many it freed.
  std::size_t reclaim_dead_readers();
  // Samples the host monotonic clock used by the platform's event Poller.
  WriteStatus write(std::span<const std::byte> message);
  // Explicit timestamp for simulated/replayed publications on their timeline.
  WriteStatus write(std::span<const std::byte> message,
                    std::int64_t publish_time_ns);

  // Copies the next message for `reader_id` into `destination` and reports
  // its sequence, publication time, and how many messages were skipped.
  //
  // Safe against a writer that laps the reader: the slot is copied first and
  // the copy is only accepted if the writer did not reach it during the copy
  // (a seqlock, using the writer sequence that the frozen shared-memory
  // layout already provides).
  // Payload and publication time are accepted under the same validity check.
  ReadResult read_next(std::uint64_t reader_id,
                       std::span<std::byte> destination, MessageInfo& info);

  std::uint64_t writer_sequence() const {
    return header_->writer.sequence.load(std::memory_order_acquire);
  }

  std::uint64_t reader_sequence(std::uint64_t reader_id) const {
    return header_->readers[reader_id].sequence.load(std::memory_order_acquire);
  }

  std::uint64_t slots() const { return slots_; }
  size_t message_size() const { return message_size_; }
  std::string_view path() const { return path_; }
  // The schema the segment's creator recorded.
  std::string_view schema_name() const;
  std::uint64_t schema_hash() const { return header_->schema_hash; }
  bool session_owned() const {
    return (header_->lifetime.load(std::memory_order_acquire) &
            kLifetimeSessionOwned) != 0;
  }

 private:
  // Number of times a read retries when the writer laps it mid-read.
  static constexpr int MAX_READ_ATTEMPTS = 8;

  static bool is_active(const Reader& reader) {
    return (reader.state.load(std::memory_order_acquire) & 3) ==
           static_cast<std::uint64_t>(ReaderState::ACTIVE);
  }

  // Whether reader slot `reader_id` still holds the word this queue claimed
  // it with. Another process that judged this one dead may have freed the
  // slot and claimed it again: it then reads ACTIVE with that process's word.
  // One acquire load. A registration found lost is forgotten, so a later
  // claim of the slot by another queue of this process, with the same word,
  // does not revive it.
  bool holds_reader(std::uint64_t reader_id) {
    const std::uint64_t word =
        header_->readers[reader_id].state.load(std::memory_order_acquire);
    if (word != 0 && word == registered_[reader_id]) return true;
    registered_[reader_id] = 0;
    return false;
  }

  // Maps the segment (creating it when absent), initializes or validates the
  // header, and joins its lifetime. Retries while a closing segment is being
  // unlinked, and unlinks one its closer died before unlinking.
  void open(const talos::schema::Identity& schema);
  // Leaves the segment's lifetime: the last queue of a segment that is not
  // session-owned, or the session owner, unlinks it.
  void close() noexcept;

  // Chooses the sequence this reader should observe next and fills in how
  // many messages were skipped to reach it. Returns false when the reader is
  // caught up with the writer.
  bool select_sequence(const Reader& reader, MessageInfo& info) const;

  // True when the writer has not yet reached the slot backing
  // `message_sequence`, i.e. the bytes just read were not overwritten.
  bool slot_still_valid(std::uint64_t message_sequence) const;

  const std::byte* slot_address(std::uint64_t message_sequence) const;

  // Notifies the wake receiver of each reader slot in `mask` that has one.
  void notify_readers(std::uint64_t mask);
  // Run by a writer that has just claimed the topic: its predecessor may
  // have died between taking the waiters word and notifying, so every slot
  // with a wake receiver is notified, armed or not.
  void wake_all_readers();

  std::string path_{""};
  std::uint64_t slots_{0};
  std::uint64_t message_size_{0};
  std::uint64_t message_alignment_{0};
  std::uint64_t data_offset_{0};
  std::uint64_t publish_time_offset_{0};
  std::uint64_t stride_{0};
  std::uint64_t total_bytes_{0};

  RTMSOptions options_{};

  RTMSHeader* header_{nullptr};
  SharedMemoryPtr ptr_;
  bool counted_{false};  // this queue holds one count of header_->lifetime
  // The state word register_reader() claimed each reader slot with, 0 for a
  // slot this queue does not hold.
  std::array<std::uint64_t, MAX_READERS> registered_{};
  // As a writer: one cached endpoint per reader slot, and how many
  // notifications went out.
  std::array<talos::WakeSender, MAX_READERS> wake_senders_{};
  std::uint64_t wakes_sent_{0};
  std::atomic<std::uint64_t> wake_failures_{0};
};
