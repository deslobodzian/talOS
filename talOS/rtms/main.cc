#include "rtms.h"
#include <flatbuffers/flatbuffers.h>
#include "talOS/rtms/test_message_generated.h"
#include <random>
#include <future>

void WriteThread(std::string_view path, int iterations, std::promise<void> promise) {
    std::random_device rd;  // a seed source for the random number engine
    std::mt19937 gen(rd()); // mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> distrib(1, 100);
    std::uniform_real_distribution<float> distrib_real(0.0, 300.0);
    RTMSQueue queue = RTMSQueue::create(
        path,
        sizeof(Message::TestMessage),
        align_up(sizeof(Message::TestMessage), alignof(Message::TestMessage))
    );

    promise.set_value();
    for (int i = 0; i < iterations; ++i) {
        Message::TestMessage msg{distrib(gen), distrib_real(gen)};
        RTMSMessage rtms_msg{sizeof(Message::TestMessage), static_cast<void*>(&msg)};
        queue.write(rtms_msg);
    }
}

void ReadThread(std::string_view path, int reader, int iterations) {
    RTMSQueue queue = RTMSQueue::attach(
        path,
        sizeof(Message::TestMessage),
        align_up(sizeof(Message::TestMessage), alignof(Message::TestMessage))
    );
    auto reader_id = queue.register_reader();

    if (reader_id) {
        for (int i = 0; i < iterations; ++i) {
            queue.read(
                reader_id.value(),
                [reader](std::span<const std::byte> bytes) {
                    Message::TestMessage new_fb{};
                    std::memcpy(&new_fb, bytes.data(), bytes.size());
                    std::printf("reader %i: new_fb: %i, %f\n", reader, new_fb.id(), new_fb.value());
                 }
            );
        }
        queue.print_header_state();
    }
}


int main() {
#if defined(__linux__)
    auto path = "/dev/shm/rtms_test";
#else
    auto path = "/tmp/rtms/test";
#endif
    remove(path);
    int iterations = 1000;
    std::promise<void> ready_promise;
    std::future<void> ready_future = ready_promise.get_future();
    std::thread writer{WriteThread, path, iterations, std::move(ready_promise)};

    ready_future.wait();
    std::array<std::thread, 8> readers{};

    for (int i = 0; i < 1; ++i) {
        readers[i] = std::thread{ReadThread, path, i, iterations};
    }

    writer.join();
    for (auto& reader : readers) {
        if (reader.joinable()) {
            reader.join();
        }
    }
}
