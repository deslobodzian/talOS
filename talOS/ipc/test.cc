#include <gtest/gtest.h>
#include <poll.h>
#include <unistd.h>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
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

// A subscriber must never claim writer ownership or act as session owner
// (replacing or unlinking the segment), even when the caller passes
// RTMSOptions with publisher=true and session_owner=true.
// REQ-CONTRACT-36, REQ-OPS-03
TEST(SubscriberTest, SubscriberIgnoresPublisherAndSessionOwnerOptions) {
  const RTMSOptions bad_options{
      .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
      .read_mode = ReadMode::SEQUENCE,
      .publisher = true,
      .session_owner = true,
  };

  // Opened before a publisher: must not claim writer ownership.
  {
    const std::string topic = Topic("no_writer");
    ipc::Subscriber<IPCMessage::TestMessage> subscriber{topic, bad_options};
    ipc::Publisher<IPCMessage::TestMessage> publisher{topic};
    ASSERT_EQ(publisher.write(IPCMessage::TestMessage{7, 1.5F}),
              WriteResult::SUCCESS);
    const auto received = subscriber.read_next();
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->message.message_id(), 7);
  }

  // Opened against an existing publisher and then destroyed: must neither
  // fail/replace the publisher's segment nor unlink it on destruction.
  {
    const std::string topic = Topic("no_unlink");
    ipc::Publisher<IPCMessage::TestMessage> publisher{topic};
    {
      ipc::Subscriber<IPCMessage::TestMessage> transient{topic, bad_options};
      ASSERT_EQ(publisher.write(IPCMessage::TestMessage{10, 0.0F}),
                WriteResult::SUCCESS);
      const auto received = transient.read_next();
      ASSERT_TRUE(received.has_value());
      EXPECT_EQ(received->message.message_id(), 10);
    }
    // If `transient` had unlinked the segment, `later` would create a fresh
    // segment and never see `publisher`'s subsequent writes.
    ipc::Subscriber<IPCMessage::TestMessage> later{topic};
    ASSERT_EQ(publisher.write(IPCMessage::TestMessage{11, 2.0F}),
              WriteResult::SUCCESS);
    const auto received = later.read_next();
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->message.message_id(), 11);
  }
}

// When all 32 reader slots are occupied during rejoin(), the subscriber stays
// unregistered (registered() == false, arm_wake() == false) until a slot frees,
// then claims it and reports the exact dropped count for every message
// published while it held no slot.
// REQ-CONTRACT-36
TEST(SubscriberTest, RejoinRecoversAfterFullReaderTableFreesASlot) {
  const std::string topic = Topic("rejoin_full");
  ipc::Publisher<IPCMessage::TestMessage> publisher{topic};
  ipc::Subscriber<IPCMessage::TestMessage> victim{topic};

  ASSERT_EQ(publisher.write(IPCMessage::TestMessage{0, 0.0F}),
            WriteResult::SUCCESS);
  const auto first = victim.read_next();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->sequence, 0u);

  // Another process frees victim's slot and every slot in the table is taken.
  victim.queue().release_reader(victim.reader_id().value_or(MAX_READERS));
  std::vector<std::unique_ptr<ipc::Subscriber<IPCMessage::TestMessage>>>
      fillers;
  fillers.reserve(MAX_READERS);
  for (std::size_t i = 0; i < MAX_READERS; ++i) {
    fillers.push_back(
        std::make_unique<ipc::Subscriber<IPCMessage::TestMessage>>(topic));
  }

  ASSERT_EQ(publisher.write(IPCMessage::TestMessage{1, 0.0F}),
            WriteResult::SUCCESS);
  ASSERT_EQ(publisher.write(IPCMessage::TestMessage{2, 0.0F}),
            WriteResult::SUCCESS);
  // First read sees INACTIVE, counts one failed read, and fails to rejoin.
  EXPECT_FALSE(victim.read_next().has_value());
  EXPECT_FALSE(victim.registered());
  EXPECT_FALSE(victim.arm_wake());
  EXPECT_EQ(victim.failed_reads(), 1u);

  // Another publication while still full; retry still fails cleanly.
  ASSERT_EQ(publisher.write(IPCMessage::TestMessage{3, 0.0F}),
            WriteResult::SUCCESS);
  EXPECT_FALSE(victim.read_next().has_value());
  EXPECT_FALSE(victim.registered());
  EXPECT_FALSE(victim.arm_wake());
  EXPECT_EQ(victim.failed_reads(), 1u);

  // Free one slot: read_next() claims it and catches up at writer sequence 4.
  fillers.pop_back();
  EXPECT_FALSE(victim.read_next().has_value());
  EXPECT_TRUE(victim.registered());
  EXPECT_TRUE(victim.arm_wake());

  ASSERT_EQ(publisher.write(IPCMessage::TestMessage{4, 0.0F}),
            WriteResult::SUCCESS);
  const auto recovered = victim.read_next();
  ASSERT_TRUE(recovered.has_value());
  EXPECT_EQ(recovered->message.message_id(), 4);
  EXPECT_EQ(recovered->sequence, 4u);
  EXPECT_EQ(recovered->dropped, 3u);  // messages 1, 2, and 3
  EXPECT_EQ(victim.failed_reads(), 1u);
}

// Move construction and move assignment transfer the reader slot and pending
// cursor without double-releasing the slot when the moved-from Subscriber is
// destroyed.
// REQ-CONTRACT-36
TEST(SubscriberTest, MovePreservesPendingStateWithoutDoubleRelease) {
  const std::string topic = Topic("move");
  ipc::Publisher<IPCMessage::TestMessage> publisher{topic};

  std::optional<ipc::Subscriber<IPCMessage::TestMessage>> source;
  source.emplace(topic);
  const std::optional<std::size_t> original_id = source->reader_id();
  ASSERT_TRUE(original_id.has_value());

  // Occupy all remaining 31 slots so any accidental release is observable.
  std::vector<std::unique_ptr<ipc::Subscriber<IPCMessage::TestMessage>>>
      fillers;
  fillers.reserve(MAX_READERS - 1);
  for (std::size_t i = 0; i < MAX_READERS - 1; ++i) {
    fillers.push_back(
        std::make_unique<ipc::Subscriber<IPCMessage::TestMessage>>(topic));
  }

  ASSERT_EQ(publisher.write(IPCMessage::TestMessage{0, 0.0F}),
            WriteResult::SUCCESS);
  ASSERT_EQ(publisher.write(IPCMessage::TestMessage{1, 0.0F}),
            WriteResult::SUCCESS);
  ASSERT_EQ(publisher.write(IPCMessage::TestMessage{2, 0.0F}),
            WriteResult::SUCCESS);
  const auto m0 = source->read_next();
  ASSERT_TRUE(m0.has_value());
  EXPECT_EQ(m0->sequence, 0u);

  // Move-construct `moved`, then destroy `source`: slot stays owned by `moved`
  // and messages 1 and 2 remain pending.
  ipc::Subscriber<IPCMessage::TestMessage> moved{std::move(*source)};
  EXPECT_FALSE(source->registered());
  EXPECT_TRUE(moved.registered());
  EXPECT_EQ(moved.reader_id(), original_id);
  source.reset();
  EXPECT_THROW((ipc::Subscriber<IPCMessage::TestMessage>{topic}),
               std::runtime_error);

  const auto m1 = moved.read_next();
  ASSERT_TRUE(m1.has_value());
  EXPECT_EQ(m1->message.message_id(), 1);
  EXPECT_EQ(m1->sequence, 1u);
  EXPECT_EQ(m1->dropped, 0u);

  // Move-assign into a live Subscriber `target`: target's previous slot is
  // released (freeing exactly one slot) while `moved`'s slot and pending msg 2
  // transfer cleanly without double-release when `moved` is destroyed.
  fillers.pop_back();
  ipc::Subscriber<IPCMessage::TestMessage> target{topic};
  EXPECT_THROW((ipc::Subscriber<IPCMessage::TestMessage>{topic}),
               std::runtime_error);

  std::optional<ipc::Subscriber<IPCMessage::TestMessage>> moved_holder;
  moved_holder.emplace(std::move(moved));
  target = std::move(*moved_holder);
  EXPECT_FALSE(moved_holder->registered());
  EXPECT_TRUE(target.registered());
  EXPECT_EQ(target.reader_id(), original_id);
  moved_holder.reset();

  // Exactly one slot was freed (target's old slot): one new subscriber fits,
  // and a second overflows MAX_READERS.
  {
    ipc::Subscriber<IPCMessage::TestMessage> one_extra{topic};
    EXPECT_THROW((ipc::Subscriber<IPCMessage::TestMessage>{topic}),
                 std::runtime_error);
  }

  const auto m2 = target.read_next();
  ASSERT_TRUE(m2.has_value());
  EXPECT_EQ(m2->message.message_id(), 2);
  EXPECT_EQ(m2->sequence, 2u);
  EXPECT_EQ(m2->dropped, 0u);
}
