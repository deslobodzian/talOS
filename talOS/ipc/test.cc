#include <gtest/gtest.h>
#include <poll.h>
#include <unistd.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ipc_test/ipc_test_message_generated.h"
#include "publisher.h"
#include "subscriber.h"
#include "talOS/rtms/wake.h"

namespace {

std::string Topic(const char* name) {
  return std::string{"/ipc_"} + name + "_" + std::to_string(::getpid());
}

// Whether the receiver's fd turns readable within `ms`.
bool WakePending(const talos::WakeReceiver& receiver, int ms) {
  pollfd entry{.fd = receiver.fd(), .events = POLLIN, .revents = 0};
  return ::poll(&entry, 1, ms) == 1 && (entry.revents & POLLIN) != 0;
}

}  // namespace

// A topic admits 32 subscribers; the 33rd is refused with an error naming
// the topic and the limit, and a released slot admits one again.
// REQ-CONTRACT-36
TEST(SubscriberTest, ReaderExhaustionRejectsAndRecoversAfterRelease) {
  // Written out, not MAX_READERS, so that a smaller table fails here.
  constexpr std::size_t kRequiredReaders = 32;
  const std::string topic = Topic("full");
  std::vector<std::unique_ptr<ipc::Subscriber<IPCMessage::TestMessage>>>
      readers;
  for (std::size_t i = 0; i < kRequiredReaders; ++i) {
    readers.push_back(
        std::make_unique<ipc::Subscriber<IPCMessage::TestMessage>>(topic));
  }
  try {
    ipc::Subscriber<IPCMessage::TestMessage> extra{topic};
    ADD_FAILURE() << "a 33rd subscriber claimed a slot";
  } catch (const std::runtime_error& error) {
    std::string message = error.what();
    const auto at = message.find(topic);
    ASSERT_NE(at, std::string::npos) << message;
    message.erase(at, topic.size());
    EXPECT_NE(message.find(std::to_string(kRequiredReaders)), std::string::npos)
        << message;
  }
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

// A subscriber that sleeps on a wake receiver must still be woken after
// another process freed its reader slot: the slot it claims again starts
// with no receiver, and a reader that armed it would otherwise sleep through
// every publication.
// REQ-RT-19
TEST(SubscriberTest, AWakeReceiverOutlivesAFreedReaderSlot) {
  const std::string topic = Topic("wake");
  ipc::Publisher<IPCMessage::TestMessage> publisher{topic};
  ipc::Subscriber<IPCMessage::TestMessage> subscriber{topic};
  talos::WakeReceiver receiver;
  subscriber.set_wake_receiver(receiver.id());

  subscriber.queue().release_reader(
      subscriber.reader_id().value_or(MAX_READERS));
  ASSERT_EQ(publisher.write(IPCMessage::TestMessage{1, 0.0F}),
            WriteResult::SUCCESS);
  EXPECT_FALSE(subscriber.read_next());  // finds the slot freed, claims one
  ASSERT_TRUE(subscriber.registered());
  ASSERT_TRUE(subscriber.arm_wake()) << "caught up on the new slot";
  receiver.drain();

  ASSERT_EQ(publisher.write(IPCMessage::TestMessage{2, 0.0F}),
            WriteResult::SUCCESS);
  EXPECT_TRUE(WakePending(receiver, 10'000))
      << "a publication after the arm woke nobody";
  EXPECT_FALSE(subscriber.arm_wake()) << "one message unread";
  EXPECT_TRUE(subscriber.read_next());
  EXPECT_TRUE(subscriber.arm_wake());
}
