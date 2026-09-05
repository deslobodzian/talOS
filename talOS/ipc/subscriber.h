#pragma once

#include <flatbuffers/flatbuffers.h>

#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <type_traits>
#include <utility>

#include "concepts.h"
#include "talOS/rtms/rtms.h"

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
class Subscriber {
  static_assert(std::is_trivially_copyable_v<Message>,
                "RTMS messages must be flatbuffer structs, not tables");

 public:
  explicit Subscriber(
      std::string_view topic,
      RTMSOptions options = RTMSOptions{
          .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
          .read_mode = ReadMode::SEQUENCE,
      })
      : topic_{topic},
        queue_{
            topic_,
            sizeof(Message),
            alignof(Message),
            MAX_SLOTS,
            options
        } {
    reader_id_ = queue_.register_reader();
  }

  Subscriber(const Subscriber&) = delete;
  Subscriber& operator=(const Subscriber&) = delete;

  // The reader slot is a shared-memory resource, so exactly one Subscriber may
  // own it. A defaulted move would copy the id and release the slot twice.
  Subscriber(Subscriber&& other) noexcept
      : topic_{std::move(other.topic_)},
        queue_{std::move(other.queue_)},
        reader_id_{std::exchange(other.reader_id_, std::nullopt)} {}

  Subscriber& operator=(Subscriber&& other) noexcept {
    if (this != &other) {
      release();
      topic_ = std::move(other.topic_);
      queue_ = std::move(other.queue_);
      reader_id_ = std::exchange(other.reader_id_, std::nullopt);
    }
    return *this;
  }

  ~Subscriber() { release(); }

  // Reads the next message, reporting the sequence and any messages the writer
  // lapped past. Prefer this over read() for anything that must be replayable.
  std::optional<Received<Message>> read_next() {
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
      return std::nullopt;
    }

    received.sequence = info.sequence;
    received.dropped = info.dropped;
    return received;
  }

  std::optional<Message> read() {
    auto received = read_next();
    if (!received) {
      return std::nullopt;
    }
    return received->message;
  }

  bool registered() const { return reader_id_.has_value(); }
  std::optional<std::size_t> reader_id() const { return reader_id_; }
  RTMSQueue& queue() { return queue_; }

 private:
  void release() {
    if (reader_id_) {
      queue_.release_reader(reader_id_.value());
      reader_id_.reset();
    }
  }

  std::string topic_;
  RTMSQueue queue_;
  std::optional<std::size_t> reader_id_;
};
} /* namespace ipc */
