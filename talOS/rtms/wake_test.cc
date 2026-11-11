#include "talOS/rtms/wake.h"

#include <gtest/gtest.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

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
// REQ-RT-19
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
// REQ-RT-19
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
// REQ-RT-19
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
// REQ-RT-19
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

// Binds an AF_UNIX datagram socket at `path`, as a macOS receiver binds
// /tmp/talos-wake-<id>; -1 on failure.
int BindAt(const std::string& path) {
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  if (path.size() >= sizeof(address.sun_path)) return -1;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  const int fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  if (fd < 0) return -1;
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

bool Exists(const std::string& path) {
  struct stat status {};
  return ::lstat(path.c_str(), &status) == 0;
}

// A process killed with SIGKILL never runs its receiver's destructor, so on
// macOS its socket file stays; a restarted node would add one per crash.
// The reclaim the next receiver runs removes the file nothing is bound to
// and keeps a live receiver's.
// REQ-RT-19
TEST(Wake, ADeadReceiversSocketFileIsReclaimedAndALiveOnesKept) {
  // A directory of its own, short enough for sun_path on every OS, so the
  // reclaim sees only this test's files.
  std::string directory = "/tmp/talos-wake-test-XXXXXX";
  ASSERT_NE(::mkdtemp(directory.data()), nullptr) << std::strerror(errno);
  const std::string dead = directory + "/talos-wake-00000000000000d1";
  const std::string live = directory + "/talos-wake-00000000000000a1";

  int ready[2];
  ASSERT_EQ(::pipe(ready), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(ready[0]);
    const char bound = BindAt(dead) >= 0 ? 1 : 0;
    (void)::write(ready[1], &bound, 1);
    ::pause();
    ::_exit(0);
  }
  ::close(ready[1]);
  char bound = 0;
  ASSERT_EQ(::read(ready[0], &bound, 1), 1);
  ::close(ready[0]);
  ::kill(child, SIGKILL);
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_EQ(bound, 1) << "the child could not bind " << dead;
  ASSERT_TRUE(Exists(dead)) << "SIGKILL removed the file itself";
  const int holder = BindAt(live);
  ASSERT_GE(holder, 0) << live;

  EXPECT_EQ(talos::ReclaimDeadWakeFiles(directory.c_str()), 1u);
  EXPECT_FALSE(Exists(dead)) << dead << " outlived its killed receiver";
  EXPECT_TRUE(Exists(live)) << live << " was removed while bound";

  ::close(holder);
  ::unlink(live.c_str());
  ::unlink(dead.c_str());
  ::rmdir(directory.c_str());
}

// End to end on the address receivers really use: after a receiver's process
// is killed, no file of it remains once the next receiver opens (on Linux
// there never was one).
// REQ-RT-19
TEST(Wake, AReceiverKilledWithSigkillLeavesNoFileOnceTheNextOpens) {
  int id_pipe[2];
  ASSERT_EQ(::pipe(id_pipe), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(id_pipe[0]);
    try {
      talos::WakeReceiver receiver;
      const std::uint64_t id = receiver.id();
      (void)::write(id_pipe[1], &id, sizeof(id));
      ::pause();
    } catch (...) {
    }
    ::_exit(3);
  }
  ::close(id_pipe[1]);
  std::uint64_t id = 0;
  const ssize_t got = ::read(id_pipe[0], &id, sizeof(id));
  ::close(id_pipe[0]);
  ::kill(child, SIGKILL);
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_EQ(got, static_cast<ssize_t>(sizeof(id)));

  char name[64];
  std::snprintf(name, sizeof(name), "/tmp/talos-wake-%016llx",
                static_cast<unsigned long long>(id));
  talos::WakeReceiver next;
  EXPECT_FALSE(Exists(name)) << name << " outlived its killed receiver";
}

}  // namespace
