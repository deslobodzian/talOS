#include <gtest/gtest.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

#include "ipc_test/ipc_test_message_generated.h"
#include "publisher.h"
#include "subscriber.h"

namespace {

std::string Topic(const char* name) {
  return std::string{"/ipc_"} + name + "_" + std::to_string(::getpid());
}

}  // namespace


TEST(SubscriberTest, ReaderExhaustionRejectsAndRecoversAfterRelease) {
  const std::string topic = Topic("full");
  std::vector<std::unique_ptr<ipc::Subscriber<IPCMessage::TestMessage>>>
      readers;
  for (std::size_t i = 0; i < MAX_READERS; ++i) {
    readers.push_back(
        std::make_unique<ipc::Subscriber<IPCMessage::TestMessage>>(topic));
  }
  EXPECT_THROW((ipc::Subscriber<IPCMessage::TestMessage>{topic}),
               std::runtime_error);
  readers.pop_back();
  EXPECT_NO_THROW((ipc::Subscriber<IPCMessage::TestMessage>{topic}));
}
