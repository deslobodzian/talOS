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

// Another process can free a subscriber's reader slot, having judged this one
// dead. The subscriber must claim a slot again rather than go quiet, count the
// failed read, and report what was published while it had no slot.
TEST(SubscriberTest, ASubscriberWhoseSlotIsFreedClaimsOneAgain) {
  const std::string topic = Topic("freed");
  ipc::Publisher<IPCMessage::TestMessage> publisher{topic};
  ipc::Subscriber<IPCMessage::TestMessage> subscriber{topic};
  int next = 0;
  const auto publish = [&] {
    return publisher.write(IPCMessage::TestMessage{next++, 0.0F}) ==
           WriteResult::SUCCESS;
  };
  ASSERT_TRUE(publish());
  ASSERT_TRUE(subscriber.read_next());
  subscriber.queue().release_reader(
      subscriber.reader_id().value_or(MAX_READERS));
  ASSERT_TRUE(publish());
  ASSERT_TRUE(publish());
  EXPECT_FALSE(subscriber.read_next());  // finds the slot freed, claims one
  EXPECT_TRUE(subscriber.registered());
  ASSERT_TRUE(publish());
  const auto received = subscriber.read_next();
  if (!received) {
    FAIL() << "nothing delivered after the slot was claimed again";
  }
  EXPECT_EQ(received->sequence, 3u);
  EXPECT_EQ(received->dropped, 2u);      // 1 and 2, published without a slot
  EXPECT_FALSE(subscriber.read_next());  // caught up: not a failed read
  EXPECT_EQ(subscriber.failed_reads(), 1u);
}
