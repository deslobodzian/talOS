// C++ peer for the Python RTMS round-trip test.
//
// Publishes or receives IPCMessage::TestMessage on one topic and prints raw
// payload bytes as hex, so the Python side can assert byte-identical
// interchange. New file in the new package only; no existing target touched.
#include <chrono>
#include <cstdio>
#include <string>
#include <string_view>
#include <thread>

#include "talOS/ipc/ipc_test_message_generated.h"
#include "talOS/ipc/publisher.h"
#include "talOS/ipc/subscriber.h"

static_assert(sizeof(IPCMessage::TestMessage) == 8);
static_assert(alignof(IPCMessage::TestMessage) == 4);

namespace {

void PrintHex(const IPCMessage::TestMessage& msg) {
  const auto* bytes = reinterpret_cast<const unsigned char*>(&msg);
  for (std::size_t i = 0; i < sizeof(msg); ++i) {
    std::printf("%02x", bytes[i]);
  }
  std::printf("\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string_view topic = "/py_roundtrip";
  std::string_view mode = "pub";
  int id = 10;
  float value = 200.0F;
  int count = 1;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    const auto rest = [&](std::string_view prefix) {
      return std::string(arg.substr(prefix.size()));
    };
    if (arg.starts_with("--topic=")) {
      topic = arg.substr(8);
    } else if (arg.starts_with("--mode=")) {
      mode = arg.substr(7);
    } else if (arg.starts_with("--id=")) {
      id = std::stoi(rest("--id="));
    } else if (arg.starts_with("--value=")) {
      value = std::stof(rest("--value="));
    } else if (arg.starts_with("--count=")) {
      count = std::stoi(rest("--count="));
    } else {
      std::fprintf(stderr, "unknown arg %s\n", argv[i]);
      return 2;
    }
  }

  if (mode == "pub") {
    ipc::Publisher<IPCMessage::TestMessage> pub{topic};
    IPCMessage::TestMessage last{};
    for (int i = 0; i < count; ++i) {
      last = IPCMessage::TestMessage{id + i, value};
      if (pub.write(last) != WriteResult::SUCCESS) {
        std::fprintf(stderr, "publish failed\n");
        return 1;
      }
    }
    PrintHex(last);
    return 0;
  }

  if (mode == "sub") {
    ipc::Subscriber<IPCMessage::TestMessage> sub{topic};
    if (!sub.registered()) {
      std::fprintf(stderr, "no reader slot\n");
      return 1;
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      if (const auto msg = sub.read_next()) {
        PrintHex(msg->message);
        std::printf("sequence=%llu dropped=%llu\n",
                    (unsigned long long)msg->sequence,
                    (unsigned long long)msg->dropped);
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::fprintf(stderr, "timed out waiting for message\n");
    return 1;
  }

  std::fprintf(stderr, "mode must be pub or sub\n");
  return 2;
}
