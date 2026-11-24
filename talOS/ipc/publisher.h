#pragma once

#include <flatbuffers/flatbuffers.h>

#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <type_traits>

#include "concepts.h"
#include "talOS/rtms/rtms.h"
#include "talOS/schema/identity.h"

namespace ipc {
// The writing end of a topic of `schema` (talOS/schema/identity.h).
class RawPublisher {
 public:
  // `slots`: the topic's ring, a power of two up to MAX_SLOTS, the one its
  // describe declares (the launcher creates the segment with it).
  explicit RawPublisher(std::string_view topic, size_t size, size_t alignment,
                        const talos::schema::Identity& schema,
                        RTMSOptions options = RTMSOptions{},
                        size_t slots = MAX_SLOTS)
      : topic_{topic},
        queue_{topic, size, alignment, schema, slots, publishing(options)} {}
  WriteStatus write(std::span<const std::byte> message) {
    return queue_.write(message);
  }
  // Stamped `publish_time_ns` (the event Poller's clock), so a writer that
  // logs its publication can record the stamp readers see.
  WriteStatus write(std::span<const std::byte> message,
                    std::int64_t publish_time_ns) {
    return queue_.write(message, publish_time_ns);
  }

  RTMSQueue& queue() { return queue_; }

 private:
  // A publisher claims writer ownership and never acts as session owner
  // (replacing or unlinking the segment), whatever the caller left set in
  // RTMSOptions; see RTMSOptions::publisher and RTMSOptions::session_owner.
  static RTMSOptions publishing(RTMSOptions options) {
    options.publisher = true;
    options.session_owner = false;
    return options;
  }

  std::string topic_;
  RTMSQueue queue_;
};

template <NotDerivedFromFlatbufferTable Message>
  requires talos::Described<Message>
class Publisher {
  static_assert(std::is_trivially_copyable_v<Message>,
                "RTMS messages must be flatbuffer structs, not tables");

 public:
  // Defaults to the policy that keeps the publisher running: a subscriber
  // that stops reading gets lapped instead of stalling this process.
  explicit Publisher(
      std::string_view topic,
      RTMSOptions options =
          RTMSOptions{
              .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
              .read_mode = ReadMode::SEQUENCE,
          },
      size_t slots = MAX_SLOTS)
      : publisher_{topic,
                   sizeof(Message),
                   alignof(Message),
                   talos::schema::Of<Message>(),
                   options,
                   slots} {}

  WriteStatus write(const Message& message) {
    return publisher_.write(
        std::as_bytes(std::span<const Message, 1>{&message, 1}));
  }
  WriteStatus write(const Message& message, std::int64_t publish_time_ns) {
    return publisher_.write(
        std::as_bytes(std::span<const Message, 1>{&message, 1}),
        publish_time_ns);
  }

  RTMSQueue& queue() { return publisher_.queue(); }

 private:
  RawPublisher publisher_;
};
} /* namespace ipc */
