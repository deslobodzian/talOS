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
        size_t alignment)
        : topic_{topic}, queue_{topic, size, alignment}
    {

    }
    void write(const RTMSMessage& message) {
        queue_.write(message);
    }
private:
    std::string topic_;
    RTMSQueue queue_;
};

template <NotDerivedFromFlatbufferTable Message>
class Publisher {
public:
    explicit Publisher(std::string_view topic) :
        publisher_{topic, sizeof(Message), alignof(Message)}
        {
    }

    void write(const Message& message) {
        RTMSMessage rtms_message{sizeof(Message), static_cast<const void*>(&message)};
        publisher_.write(rtms_message);
    }

private:
    RawPublisher publisher_;
};
} /* namespace ipc */
