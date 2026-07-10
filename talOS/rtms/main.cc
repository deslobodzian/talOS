#include "rtms.h"
#include <chrono>
#include <flatbuffers/flatbuffers.h>
#include "talOS/rtms/test_message_generated.h"
#include <random>
#include <future>

void WriteThread(std::string_view path, int iterations, std::promise<void> promise) {
    std::random_device rd;  // a seed source for the random number engine
    std::mt19937 gen(rd()); // mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> distrib(1, 100);
    RTMSQueue queue{
        path,
        sizeof(Message::TimeMessage),
        alignof(Message::TimeMessage)
    };

    promise.set_value();
    for (int i = 0; i < iterations; ++i) {
        auto now = std::chrono::high_resolution_clock::now();
        auto usec =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()
        ).count();
        Message::TimeMessage msg{distrib(gen), static_cast<uint64_t>(usec)};
        RTMSMessage rtms_msg{sizeof(Message::TimeMessage), static_cast<void*>(&msg)};
        queue.write(rtms_msg);
    }
}

void ReadThread(std::string_view path, int reader, int iterations) {
    RTMSQueue queue{
        path,
        sizeof(Message::TimeMessage),
        alignof(Message::TimeMessage)
    };
    auto reader_id = queue.register_reader();

    if (reader_id) {
        for (int i = 0; i < iterations; ++i) {
            queue.read(
                reader_id.value(),
                [reader](std::span<const std::byte> bytes) {
                    Message::TimeMessage new_fb{};
                    std::memcpy(&new_fb, bytes.data(), bytes.size());
                    std::printf(
                        "reader %i: new_fb: %i, %" PRIu64 "\n",
                        reader,
                        new_fb.id(),
                        new_fb.value()
                    );
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
