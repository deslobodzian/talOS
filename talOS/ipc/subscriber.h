#pragma once

#include <string>
#include "talOS/rtms/rtms.h"
#include <flatbuffers/flatbuffers.h>
#include "concepts.h"

namespace ipc {

template <ipc::NotDerivedFromFlatbufferTable Message>
class Subscriber {
public:
    explicit Subscriber(std::string_view topic) :
        topic_{topic},
        queue_{
            RTMSQueue::attach(
                topic_,
                sizeof(Message),
                align_up(sizeof(Message), alignof(Message)))
        } {
        reader_id_ = queue_.register_reader();
    }

    Message read() {
        Message msg{};
        if (reader_id_) {
            queue_.read(
                reader_id_.value(),
                [&msg](std::span<const std::byte> bytes) {
                    std::memcpy(&msg, bytes.data(), bytes.size());
                }
            );
        }
        return msg;
    }
private:
    std::string topic_;
    RTMSQueue queue_;
    std::optional<std::size_t> reader_id_;

};
} /* namespace ipc */
