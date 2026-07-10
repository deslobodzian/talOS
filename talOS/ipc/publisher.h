#pragma once

#include <cstdio>
#include <string>
#include <flatbuffers/flatbuffers.h>
#include "concepts.h"
#include "talOS/rtms/rtms.h"

namespace ipc {
template <NotDerivedFromFlatbufferTable Message>
class Publisher {
public:
    explicit Publisher(std::string_view topic) : topic_{topic},
        queue_{
            topic,
            sizeof(Message),
            alignof(Message)
        } {
    }

    void write(const Message& message) {
        RTMSMessage rtms_message{sizeof(Message), static_cast<const void*>(&message)};
        queue_.write(rtms_message);
    }

private:
    std::string topic_;
    RTMSQueue queue_;
};
} /* namespace ipc */
