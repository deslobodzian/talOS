#pragma once

#include <flatbuffers/flatbuffers.h>

#include <cstdio>
#include <string>

#include "concepts.h"
#include "talOS/rtms/rtms.h"
#include "talOS/schema/identity.h"

namespace ipc {
// The writing end of a topic of `schema` (talOS/schema/identity.h).
class RawPublisher {
 public:
  explicit RawPublisher(std::string_view topic, size_t size, size_t alignment,
                        const talos::schema::Identity& schema,
                        RTMSOptions options = RTMSOptions{})
      : topic_{topic},
        queue_{topic, size, alignment, schema, MAX_SLOTS, publishing(options)} {
  }
  WriteStatus write(std::span<const std::byte> message) {
    return queue_.write(message);
  }

  RTMSQueue& queue() { return queue_; }

 private:
  // A publisher owns its topic's layout outside a session; see
  // RTMSOptions::publisher.
  static RTMSOptions publishing(RTMSOptions options) {
    options.publisher = true;
    return options;
  }

  std::string topic_;
  RTMSQueue queue_;
};

template <NotDerivedFromFlatbufferTable Message>
  requires talos::Described<Message>
class Publisher {
 public:
  // Defaults to the policy that keeps the publisher running: a subscriber
  // that stops reading gets lapped instead of stalling this process.
  explicit Publisher(
      std::string_view topic,
      RTMSOptions options =
          RTMSOptions{
              .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
              .read_mode = ReadMode::SEQUENCE,
          })
      : publisher_{topic, sizeof(Message), alignof(Message),
                   talos::schema::Of<Message>(), options} {}

  WriteStatus write(const Message& message) {
    return publisher_.write(
        std::as_bytes(std::span<const Message, 1>{&message, 1}));
  }

 private:
  RawPublisher publisher_;
};
} /* namespace ipc */
