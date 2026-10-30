#pragma once

#include <flatbuffers/flatbuffers.h>

#include <cstdio>
#include <string>

#include "concepts.h"
#include "talOS/rtms/rtms.h"

namespace ipc {
class RawPublisher {
 public:
  explicit RawPublisher(std::string_view topic, size_t size, size_t alignment,
                        RTMSOptions options = RTMSOptions{})
      : topic_{topic},
        queue_{topic, size, alignment, MAX_SLOTS, owning(options)} {}
  WriteStatus write(std::span<const std::byte> message) {
    return queue_.write(message);
  }

  RTMSQueue& queue() { return queue_; }

 private:
  // A publisher owns its topic's layout, so it is the one allowed to clear a
  // segment left behind by an older build. See RTMSOptions.
  static RTMSOptions owning(RTMSOptions options) {
    options.reclaim_mismatched_segment = true;
    return options;
  }

  std::string topic_;
  RTMSQueue queue_;
};

template <NotDerivedFromFlatbufferTable Message>
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
      : publisher_{topic, sizeof(Message), alignof(Message), options} {}

  WriteStatus write(const Message& message) {
    return publisher_.write(
        std::as_bytes(std::span<const Message, 1>{&message, 1}));
  }

 private:
  RawPublisher publisher_;
};
} /* namespace ipc */
