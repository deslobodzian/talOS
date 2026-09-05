#pragma once

#include <cstdio>
#include <string>
#include <flatbuffers/flatbuffers.h>
#include "concepts.h"
#include "talOS/rtms/rtms.h"


namespace ipc {
class RawPublisher {
public:
    explicit RawPublisher(
        std::string_view topic,
        size_t size,
        size_t alignment,
        RTMSOptions options = RTMSOptions{})
        : topic_{topic}, queue_{topic, size, alignment, MAX_SLOTS, options}
    {

    }
    WriteStatus write(const RTMSMessage& message) {
        return queue_.write(message);
    }

    RTMSQueue& queue() { return queue_; }
private:
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
        RTMSOptions options = RTMSOptions{
            .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
            .read_mode = ReadMode::SEQUENCE,
        })
        : publisher_{topic, sizeof(Message), alignof(Message), options}
        {
    }

    WriteStatus write(const Message& message) {
        RTMSMessage rtms_message{sizeof(Message), static_cast<const void*>(&message)};
        return publisher_.write(rtms_message);
    }

private:
    RawPublisher publisher_;
};
} /* namespace ipc */
