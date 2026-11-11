#include "talOS/rtms/wake.h"

#include <gtest/gtest.h>
#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdint>

namespace {

// Bounds a hang only (docs/TESTING.md §6.3); never the assertion.
constexpr int kHangBoundMs = 10'000;

// poll(2) on the receiver's fd: 1 when readable, 0 when not within `ms`, -1
// on an error or an fd that turned ready without being readable.
int Readable(const talos::WakeReceiver& receiver, int ms) {
  pollfd entry{.fd = receiver.fd(), .events = POLLIN, .revents = 0};
  const int ready = ::poll(&entry, 1, ms);
  if (ready != 1) return ready;
  return (entry.revents & POLLIN) != 0 ? 1 : -1;
}

// A reader sleeps on the fd, so the fd must turn readable on a notification
// and only then, and drain() must leave it ready to sleep on again.
// REQ-RT-02
TEST(Wake, TheFdIsReadableOnlyBetweenANotificationAndADrain) {
  talos::WakeReceiver receiver;
  ASSERT_NE(receiver.id(), 0u);
  EXPECT_EQ(Readable(receiver, 0), 0) << "readable before any notification";

  talos::WakeSender sender;
  ASSERT_TRUE(sender.notify(receiver.id()));
  EXPECT_EQ(Readable(receiver, kHangBoundMs), 1);
  EXPECT_EQ(Readable(receiver, 0), 1) << "polling consumed the notification";

  receiver.drain();
  EXPECT_EQ(Readable(receiver, 0), 0) << "drain left a notification";
}

// A publisher notifies on its control loop: once a reader has a wake pending,
// further notifications must neither block nor report the receiver gone.
// REQ-RT-02
TEST(Wake, NotificationsToAFullReceiverNeitherBlockNorFail) {
  talos::WakeReceiver receiver;
  talos::WakeSender sender;
  // Far more than any kernel queues for one datagram socket.
  constexpr int kNotifications = 10'000;
  for (int i = 0; i < kNotifications; ++i) {
    ASSERT_TRUE(sender.notify(receiver.id())) << "notification " << i;
  }
  EXPECT_EQ(Readable(receiver, 0), 1);
  receiver.drain();
  EXPECT_EQ(Readable(receiver, 0), 0) << "drain left a notification";
}

// A reader that exits leaves its id in shared memory: notifying it must
// report it gone, through a cached endpoint or a fresh one, and must not
// spoil the sender for the slot's next reader.
// REQ-RT-02
TEST(Wake, ANotificationToADestroyedReceiverReportsItGone) {
  talos::WakeSender cached;
  std::uint64_t gone = 0;
  {
    talos::WakeReceiver receiver;
    gone = receiver.id();
    ASSERT_TRUE(cached.notify(gone));
  }
  EXPECT_FALSE(cached.notify(gone)) << "through the cached endpoint";
  talos::WakeSender fresh;
  EXPECT_FALSE(fresh.notify(gone)) << "through a new endpoint";

  talos::WakeReceiver next;
  EXPECT_NE(next.id(), gone);
  EXPECT_TRUE(cached.notify(next.id()));
  EXPECT_EQ(Readable(next, kHangBoundMs), 1);
}

// Publisher and reader are different processes; the receiver's id is all
// the publisher knows of it.
// REQ-RT-02
TEST(Wake, AnotherProcessWakesAReceiverByItsId) {
  int id_pipe[2];
  ASSERT_EQ(::pipe(id_pipe), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(id_pipe[0]);
    int code = 2;
    try {  // Destroyed before _exit, which would leave macOS's socket file.
      talos::WakeReceiver receiver;
      const std::uint64_t id = receiver.id();
      if (::write(id_pipe[1], &id, sizeof(id)) == sizeof(id)) {
        code = Readable(receiver, kHangBoundMs) == 1 ? 0 : 1;
      }
    } catch (...) {
      code = 3;
    }
    ::_exit(code);
  }
  ::close(id_pipe[1]);
  std::uint64_t id = 0;
  ASSERT_EQ(::read(id_pipe[0], &id, sizeof(id)),
            static_cast<ssize_t>(sizeof(id)));
  ::close(id_pipe[0]);

  talos::WakeSender sender;
  EXPECT_TRUE(sender.notify(id));
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0) << "the child was not woken";
}

}  // namespace
