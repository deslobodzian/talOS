#pragma once

#include <string>
#include "talOS/rtms/rtms.h"
#include <flatbuffers/flatbuffers.h>
#include <optional>
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
                alignof(Message))
        } {
        reader_id_ = queue_.register_reader();
    }

    Subscriber(const Subscriber&) = delete;
    Subscriber operator=(const Subscriber&) = delete;

    Subscriber(Subscriber&&) noexcept  = default;
    Subscriber& operator=(Subscriber&& other) noexcept = default;


    ~Subscriber() {
        if (reader_id_) {
            queue_.release_reader(reader_id_.value());
        }
    }


    std::optional<Message> read() {
        Message msg{};
        if (reader_id_) {
            if (queue_.read(
                reader_id_.value(),
                [&msg](std::span<const std::byte> bytes) {
                    std::memcpy(&msg, bytes.data(), bytes.size());
                }
            )) {
                return msg;
            }
        }
        return std::nullopt;
    }
private:
    std::string topic_;
    RTMSQueue queue_;
    std::optional<std::size_t> reader_id_;
};
} /* namespace ipc */
