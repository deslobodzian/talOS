#include "rtms.h"
#include <flatbuffers/flatbuffers.h>
#include "talOS/rtms/test_message_generated.h"

int main() {
    auto path = "/tmp/rtms/test";

    RTMSQueue queue{path, sizeof(Message::TestMessage)};
    Message::TestMessage fb_message{10, 200};
    RTMSMessage message{sizeof(Message::TestMessage), static_cast<void*>(&fb_message)};
    queue.write(message);
    queue.write(message);
    queue.write(message);

    Message::TestMessage new_fb{};
    queue.read(
        0,
        [&new_fb](std::span<const std::byte> bytes) {
            std::memcpy(&new_fb, bytes.data(), bytes.size());
            std::printf("new_fb: %i, %f\n", new_fb.id(), new_fb.value());
         }
    );
    queue.read(
        0,
        [&new_fb](std::span<const std::byte> bytes) {
            std::memcpy(&new_fb, bytes.data(), bytes.size());
            std::printf("new_fb: %i, %f\n", new_fb.id(), new_fb.value());
         }
    );
    queue.read(
        0,
        [&new_fb](std::span<const std::byte> bytes) {
            std::memcpy(&new_fb, bytes.data(), bytes.size());
            std::printf("new_fb: %i, %f\n", new_fb.id(), new_fb.value());
         }
    );
    std::printf("new_fb: %i, %f\n", new_fb.id(), new_fb.value());
    return 0;
}
