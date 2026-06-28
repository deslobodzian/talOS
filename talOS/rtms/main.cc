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

    auto new_message = queue.read(0);
    new_message = queue.read(0);
    new_message = queue.read(0);
    std::printf("new_message: %p, %zu\n", new_message.data, new_message.size);

    auto new_fb = static_cast<const Message::TestMessage*>(new_message.data);
    Message::TestMessage new_stack_fb{new_fb->id(), new_fb->value()};
    std::printf("new_fb: %i, %f\n", new_fb->id(), new_fb->value());
    return 0;
}
