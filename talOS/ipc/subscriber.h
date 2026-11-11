#pragma once

#include <flatbuffers/flatbuffers.h>

#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "concepts.h"
#include "talOS/rtms/rtms.h"
#include "talOS/schema/identity.h"

namespace ipc {

// A message plus the metadata a deterministic consumer needs: which sequence
// it was published under, and how many messages were missed before it.
template <typename Message>
struct Received {
  Message message{};
  std::uint64_t sequence{0};
  std::uint64_t dropped{0};
};

template <ipc::NotDerivedFromFlatbufferTable Message>
  requires talos::Described<Message>
class Subscriber {
  static_assert(std::is_trivially_copyable_v<Message>,
                "RTMS messages must be flatbuffer structs, not tables");

 public:
  explicit Subscriber(
      std::string_view topic,
      RTMSOptions options =
          RTMSOptions{
              .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
              .read_mode = ReadMode::SEQUENCE,
          })
      : topic_{topic}, queue_{topic_,           sizeof(Message),
                              alignof(Message), talos::schema::Of<Message>(),
                              MAX_SLOTS,        options} {
    reader_id_ = queue_.register_reader();
    if (!reader_id_) {
      throw std::runtime_error("RTMS topic " + topic_ +
                               " exhausted reader slots (limit " +
                               std::to_string(MAX_READERS) + ")");
    }
    next_sequence_ = queue_.reader_sequence(*reader_id_);
  }

  Subscriber(const Subscriber&) = delete;
  Subscriber& operator=(const Subscriber&) = delete;

  // The reader slot is a shared-memory resource, so exactly one Subscriber may
  // own it. A defaulted move would copy the id and release the slot twice.
  Subscriber(Subscriber&& other) noexcept
      : topic_{std::move(other.topic_)},
        queue_{std::move(other.queue_)},
        reader_id_{std::exchange(other.reader_id_, std::nullopt)},
        rejoining_{std::exchange(other.rejoining_, false)},
        wake_id_{std::exchange(other.wake_id_, 0)},
        next_sequence_{other.next_sequence_},
        lost_{other.lost_},
        failed_reads_{other.failed_reads_} {}

  Subscriber& operator=(Subscriber&& other) noexcept {
    if (this != &other) {
      release();
      topic_ = std::move(other.topic_);
      queue_ = std::move(other.queue_);
      reader_id_ = std::exchange(other.reader_id_, std::nullopt);
      rejoining_ = std::exchange(other.rejoining_, false);
      wake_id_ = std::exchange(other.wake_id_, 0);
      next_sequence_ = other.next_sequence_;
      lost_ = other.lost_;
      failed_reads_ = other.failed_reads_;
    }
    return *this;
  }

  ~Subscriber() { release(); }

  // Reads the next message, reporting the sequence and any messages skipped
  // before it. Prefer this over read() for anything that must be replayable.
  //
  // nullopt when nothing new has been published, and when the read failed:
  // the writer tore it (the next read retries), or another process freed
  // this reader's slot, having judged it dead. A failed read counts in
  // failed_reads(). A freed slot is claimed again by the read that finds it
  // gone, or by a later one while none is free; what was published while
  // this end had no slot counts in the next message's `dropped`.
  std::optional<Received<Message>> read_next() {
    // Left without a slot by a failed rejoin below: keep trying.
    if (rejoining_) {
      rejoin();
    }
    if (!reader_id_) {
      return std::nullopt;
    }

    Received<Message> received{};
    MessageInfo info{};

    const ReadResult result = queue_.read_next(
        reader_id_.value(),
        std::span<std::byte>{reinterpret_cast<std::byte*>(&received.message),
                             sizeof(Message)},
        info);

    if (result != ReadResult::OK) {
      if (result == ReadResult::TORN || result == ReadResult::INACTIVE) {
        ++failed_reads_;
      }
      if (result == ReadResult::INACTIVE) {
        reader_id_.reset();
        rejoining_ = true;
        rejoin();
      }
      return std::nullopt;
    }

    next_sequence_ = info.sequence + 1;
    received.sequence = info.sequence;
    received.dropped = info.dropped + std::exchange(lost_, 0);
    return received;
  }

  std::optional<Message> read() {
    auto received = read_next();
    if (!received) {
      return std::nullopt;
    }
    return received->message;
  }

  // False while a freed reader slot could not be claimed again.
  bool registered() const { return reader_id_.has_value(); }
  std::optional<std::size_t> reader_id() const { return reader_id_; }
  // Reads that failed: torn by the writer, or finding the slot freed.
  std::uint64_t failed_reads() const { return failed_reads_; }
  RTMSQueue& queue() { return queue_; }

  // Wake channel (talOS/rtms README.md "Wake channel"). Names the
  // talos::WakeReceiver this subscriber's reader slot sleeps on, 0 for none;
  // a slot claimed again after being freed names it too.
  void set_wake_receiver(std::uint64_t wake_id) {
    wake_id_ = wake_id;
    if (reader_id_) queue_.set_reader_wake(*reader_id_, wake_id_);
  }
  // Asks for a notification on the next publication. True when caught up,
  // so blocking on the receiver is safe; false when there is something to
  // read, or while no reader slot is held (a read retries the claim).
  bool arm_wake() { return reader_id_ && queue_.arm_wake(*reader_id_); }

 private:
  void release() {
    if (reader_id_) {
      queue_.release_reader(reader_id_.value());
      reader_id_.reset();
    }
  }

  // Claims a reader slot again after this one's was freed under it. The new
  // slot starts at the writer, so what was published since the last message
  // is owed to the next one's dropped count.
  void rejoin() {
    reader_id_ = queue_.register_reader();
    if (!reader_id_) {
      return;
    }
    rejoining_ = false;
    // A claimed slot names no receiver; the next arm covers the new slot.
    if (wake_id_ != 0) queue_.set_reader_wake(*reader_id_, wake_id_);
    const std::uint64_t start = queue_.reader_sequence(*reader_id_);
    if (start > next_sequence_) {
      lost_ += start - next_sequence_;
    }
    next_sequence_ = start;
  }

  std::string topic_;
  RTMSQueue queue_;
  std::optional<std::size_t> reader_id_;
  // Set from a freed slot until a rejoin succeeds; false in a moved-from
  // subscriber, which has no queue to claim one from.
  bool rejoining_{false};
  // The wake receiver this end's slot names (set_wake_receiver), 0 for none.
  std::uint64_t wake_id_{0};
  // The next undelivered sequence, and the messages published while this
  // end had no reader slot, owed to the next message's dropped count.
  std::uint64_t next_sequence_{0};
  std::uint64_t lost_{0};
  std::uint64_t failed_reads_{0};
};
} /* namespace ipc */
