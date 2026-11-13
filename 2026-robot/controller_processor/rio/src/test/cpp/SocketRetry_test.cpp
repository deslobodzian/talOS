// When the RoboRIO loop retries opening the gateway's socket: soon enough
// after the network comes up that a rebooted RoboRIO's State is back as soon
// as it can be, and seldom enough that a network that stays down costs the
// 5 ms loop nothing. And when it replaces a socket that can no longer send.
#include "SocketRetry.h"

#include <gtest/gtest.h>

#include <cerrno>
#include <cstdint>
#include <cstring>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "protocol/udp.h"
#include "protocol/udp_test_helpers.h"
#endif

namespace {

using talos::rio::SocketRetry;

constexpr uint64_t kLoopPeriodUs = 5'000;
// The program's first tick: an arbitrary steady-clock time.
constexpr uint64_t kStartUs = 7'000'000;

// Scenario (bench, RoboRIO reboot during a session): the program starts
// before the RoboRIO's network has a route to the companion, so the first
// open fails, and the route appears 2.0013 s later, between two loop ticks.
// The socket must open, and State leave, within one retry period and one
// loop tick of the route appearing; the open reports how many attempts
// failed since when, for the console line that times the outage.
// REQ-OPS-28
TEST(SocketRetry, OpensWithinOneRetryPeriodOfTheRouteAppearing) {
  constexpr uint64_t kRouteUpUs = kStartUs + 2'001'300;
  constexpr uint64_t kBoundUs = 100'000 + kLoopPeriodUs;
  SocketRetry retry;
  uint64_t opened_us = 0;
  uint32_t attempts = 0;
  for (uint64_t now = kStartUs; now < kStartUs + 10'000'000;
       now += kLoopPeriodUs) {
    const auto event = retry.Poll(now, [&] {
      ++attempts;
      return now >= kRouteUpUs;
    });
    if (event == SocketRetry::Event::kOpened) {
      opened_us = now;
      break;
    }
  }
  ASSERT_NE(opened_us, 0u) << "never opened";
  EXPECT_LE(opened_us - kRouteUpUs, kBoundUs)
      << "opened " << opened_us - kRouteUpUs
      << " us after the route appeared; bound: one retry period (100 ms) "
         "plus one loop period (5 ms)";
  EXPECT_EQ(retry.failures(), attempts - 1);
  EXPECT_EQ(retry.first_failure_us(), kStartUs);
}

// A network that never comes up: one attempt per retry period, not one per
// loop tick, and none once the socket is open.
TEST(SocketRetry, AttemptsAtMostOncePerPeriod) {
  SocketRetry retry;
  bool up = false;
  uint32_t attempts = 0;
  uint64_t last_attempt_us = 0;
  for (uint64_t now = kStartUs; now < kStartUs + 10'000'000;
       now += kLoopPeriodUs) {
    retry.Poll(now, [&] {
      if (attempts > 0) {
        EXPECT_GE(now - last_attempt_us, SocketRetry::kPeriodUs)
            << "attempt " << attempts << " at " << now;
      }
      ++attempts;
      last_attempt_us = now;
      return up;
    });
  }
  EXPECT_EQ(attempts, 10'000'000 / SocketRetry::kPeriodUs);

  up = true;
  const uint64_t later = kStartUs + 10'000'000 + SocketRetry::kPeriodUs;
  ASSERT_EQ(retry.Poll(later, [&] { return up; }), SocketRetry::Event::kOpened);
  EXPECT_EQ(retry.Poll(later + SocketRetry::kPeriodUs,
                       [&] {
                         ADD_FAILURE() << "attempt on an open socket";
                         return true;
                       }),
            SocketRetry::Event::kNone);
}

// Which send errors lose the socket. One the network clears by itself keeps
// it, and the loop goes on sending on it: a full interface queue (ENOBUFS),
// the companion's port closed (ECONNREFUSED), no interface yet (ENETDOWN),
// an ICMP host unreachable (EHOSTUNREACH), a full socket buffer (EAGAIN).
// One that every later send on the same socket repeats loses it, and the next
// due attempt replaces it: ENETUNREACH is what Linux, the RoboRIO's OS,
// returns once the socket's source address has left the host (a new DHCP
// lease), even after a new address appears.
// REQ-RT-22
TEST(SocketRetry, OnlyASendErrorTheSocketKeepsRepeatingLosesIt) {
  struct Row {
    int error;
    bool lost;
  };
  uint64_t now = kStartUs;
  for (const Row row :
       {Row{ENOBUFS, false}, Row{ECONNREFUSED, false}, Row{ENETDOWN, false},
        Row{EHOSTUNREACH, false}, Row{EAGAIN, false}, Row{ENETUNREACH, true}}) {
    SCOPED_TRACE(std::strerror(row.error));
    SocketRetry retry;
    ASSERT_EQ(retry.Poll(now, [] { return true; }),
              SocketRetry::Event::kOpened);
    now += SocketRetry::kPeriodUs;
    EXPECT_EQ(retry.SendFailed(now, row.error),
              row.lost ? SocketRetry::SendEvent::kLost
                       : SocketRetry::SendEvent::kFailing);
    EXPECT_EQ(retry.open(), !row.lost);
    EXPECT_EQ(
        retry.Poll(now + kLoopPeriodUs, [] { return true; }),
        row.lost ? SocketRetry::Event::kReplaced : SocketRetry::Event::kNone);
  }
}

// Bench 2026-11-13: with the RoboRIO's address removed for 6 s the link was
// lost and came back, and the console showed nothing of the RoboRIO's sends:
// they failed with an errno that keeps the socket (EINVAL, say), or did not
// fail. A spell of failed sends that keep the socket reports once at its
// first failure, naming the errno, and once sends have gone kSpellQuietUs
// without failing, with how many failed, from when to when, and the last
// errno when it changed; the sends between report nothing (Robot.cpp prints
// one console line for each report). A good send between failures does not
// end the spell: while the companion's port is closed every other send fails
// with ECONNREFUSED (the ICMP port unreachable of the one before), which
// would otherwise start and end a spell every two sends. An error that
// replaces the socket still reports the loss; the replacement's good sends
// end that spell without a line of its own, the replaced line having said
// it.
// REQ-RT-22
TEST(SocketRetry, AFailingSpellThatKeepsTheSocketReportsItsStartAndEnd) {
  using SendEvent = SocketRetry::SendEvent;
  SocketRetry retry;
  ASSERT_EQ(retry.Poll(kStartUs, [] { return true; }),
            SocketRetry::Event::kOpened);
  EXPECT_EQ(retry.SendSucceeded(kStartUs), SendEvent::kNone);

  const uint64_t spell_us = kStartUs + 1'000'000;
  constexpr uint64_t kSpellUs = 6'000'000;
  EXPECT_EQ(retry.SendFailed(spell_us, EINVAL), SendEvent::kFailing);
  uint32_t failed = 1;
  uint64_t last_failure_us = spell_us;
  // Every other send fails, the errno changing halfway.
  bool fail = false;
  for (uint64_t now = spell_us + kLoopPeriodUs; now < spell_us + kSpellUs;
       now += kLoopPeriodUs, fail = !fail) {
    if (!fail) {
      EXPECT_EQ(retry.SendSucceeded(now), SendEvent::kNone) << now;
      continue;
    }
    EXPECT_EQ(
        retry.SendFailed(now, now < spell_us + kSpellUs / 2 ? EINVAL : ENOBUFS),
        SendEvent::kNone)
        << now;
    ++failed;
    last_failure_us = now;
  }
  EXPECT_TRUE(retry.open());
  EXPECT_EQ(retry.SendSucceeded(last_failure_us + SocketRetry::kSpellQuietUs -
                                kLoopPeriodUs),
            SendEvent::kNone)
      << "the spell ended before sends went kSpellQuietUs without failing";
  EXPECT_EQ(retry.SendSucceeded(last_failure_us + SocketRetry::kSpellQuietUs),
            SendEvent::kRecovered);
  EXPECT_EQ(retry.spell_failures(), failed);
  EXPECT_EQ(retry.spell_start_us(), spell_us);
  EXPECT_EQ(retry.spell_last_failure_us(), last_failure_us);
  EXPECT_EQ(retry.spell_error(), EINVAL);
  EXPECT_EQ(retry.last_send_error(), ENOBUFS);
  EXPECT_EQ(retry.SendSucceeded(last_failure_us + SocketRetry::kSpellQuietUs +
                                kLoopPeriodUs),
            SendEvent::kNone);

  const uint64_t lost_us = spell_us + 2 * kSpellUs;
  EXPECT_EQ(retry.SendFailed(lost_us, ENETUNREACH), SendEvent::kLost);
  ASSERT_EQ(retry.Poll(lost_us + kLoopPeriodUs, [] { return true; }),
            SocketRetry::Event::kReplaced);
  EXPECT_EQ(retry.SendSucceeded(lost_us + kLoopPeriodUs), SendEvent::kNone);
  EXPECT_EQ(retry.SendSucceeded(lost_us + SocketRetry::kSpellQuietUs),
            SendEvent::kNone);
}

#ifndef _WIN32
// The port a bound socket holds; 0 when it has none.
uint16_t BoundPort(int fd) {
  sockaddr_in address{};
  socklen_t size = sizeof(address);
  if (fd < 0 ||
      ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size) != 0) {
    return 0;
  }
  return ntohs(address.sin_port);
}

// A UDP port the OS hands out, held for the test's lifetime by a wildcard
// socket with SO_REUSEADDR: a socket bound to 127.0.0.1 on it with
// SO_REUSEADDR too (Linux and macOS) still can, and no socket opened without
// it (the default for every talOS peer) can take the port while that socket
// is closed between replacements.
class HeldPort {
 public:
  HeldPort() : fd_{::socket(AF_INET, SOCK_DGRAM, 0)} {
    int enabled = 1;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    if (fd_ < 0 ||
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &enabled,
                     sizeof(enabled)) != 0 ||
        ::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) !=
            0) {
      return;
    }
    port_ = BoundPort(fd_);
  }
  ~HeldPort() {
    if (fd_ >= 0) ::close(fd_);
  }
  HeldPort(const HeldPort&) = delete;
  HeldPort& operator=(const HeldPort&) = delete;
  uint16_t port() const { return port_; }

 private:
  int fd_;
  uint16_t port_{0};
};

// The gateway's socket can no longer send (shut down for writing here: EPIPE,
// as from a socket the kernel marked defunct; every later send on it fails).
// The send that finds it loses it, and the loop's next tick opens a new
// socket on the same ports with the frame sequence carried on: the
// companion, connected to the gateway's port, receives from it. A new socket
// that breaks at once (a condition that outlasts one socket) is replaced
// again only a retry period after the last attempt, not on every tick.
// REQ-RT-22
TEST(SocketRetry, ASocketThatCannotSendIsReplacedOnItsPorts) {
  namespace protocol = talos::protocol;
  const HeldPort gateway_port;
  ASSERT_NE(gateway_port.port(), 0);
  // The companion keeps the port it is given at port 0.
  protocol::RuntimeUdpPeer companion;
  ASSERT_EQ(companion.Open("127.0.0.1", 0, "127.0.0.1", gateway_port.port()),
            protocol::UdpStatus::kOk);
  const uint16_t companion_port = BoundPort(companion.NativeFd());
  ASSERT_NE(companion_port, 0);
  protocol::RuntimeUdpPeer gateway;
  SocketRetry retry;
  // What Robot.cpp's loop passes: Endpoint::Reopen while replacing,
  // Endpoint::Open before the first socket (here with SO_REUSEADDR, beside
  // the socket that holds its port).
  protocol::UdpSocketOptions options;
  options.reuse_address = true;
  const auto open = [&] {
    return (retry.replacing()
                ? gateway.Reopen()
                : gateway.Open("127.0.0.1", gateway_port.port(), "127.0.0.1",
                               companion_port, options)) ==
           protocol::UdpStatus::kOk;
  };
  const auto send = [&](uint64_t now_us) {
    const auto status =
        gateway.SendFrame(protocol::FrameType::kHardwareState,
                          protocol::kFlagNone, now_us, nullptr, 0);
    if (status == protocol::UdpStatus::kSocketError)
      retry.SendFailed(now_us, gateway.stats().last_send_error);
    return status;
  };
  protocol::DecodedFrame received;

  ASSERT_EQ(retry.Poll(kStartUs, open), SocketRetry::Event::kOpened);
  ASSERT_EQ(send(kStartUs), protocol::UdpStatus::kOk);
  ASSERT_EQ(protocol::testing::ReceiveFrame(companion, &received),
            protocol::UdpStatus::kOk);
  EXPECT_EQ(received.header.sequence, 1u);

  const uint64_t lost_us = kStartUs + 1'000'000;
  ASSERT_EQ(::shutdown(gateway.NativeFd(), SHUT_WR), 0);
  ASSERT_EQ(send(lost_us), protocol::UdpStatus::kSocketError);
  EXPECT_EQ(gateway.stats().last_send_error, EPIPE);
  EXPECT_FALSE(retry.open()) << "the socket that cannot send was kept";

  const uint64_t replaced_us = lost_us + kLoopPeriodUs;
  ASSERT_EQ(retry.Poll(replaced_us, open), SocketRetry::Event::kReplaced);
  ASSERT_EQ(send(replaced_us), protocol::UdpStatus::kOk);
  ASSERT_EQ(protocol::testing::ReceiveFrame(companion, &received),
            protocol::UdpStatus::kOk);
  EXPECT_EQ(received.header.monotonic_time_us, replaced_us);
  EXPECT_EQ(received.header.sequence, 2u);

  // Broken again at once: no new socket until a retry period after the
  // last attempt.
  ASSERT_EQ(::shutdown(gateway.NativeFd(), SHUT_WR), 0);
  ASSERT_EQ(send(replaced_us + kLoopPeriodUs),
            protocol::UdpStatus::kSocketError);
  for (uint64_t now = replaced_us + 2 * kLoopPeriodUs;
       now < replaced_us + SocketRetry::kPeriodUs; now += kLoopPeriodUs) {
    EXPECT_EQ(retry.Poll(now, open), SocketRetry::Event::kNone) << now;
  }
  EXPECT_EQ(retry.Poll(replaced_us + SocketRetry::kPeriodUs, open),
            SocketRetry::Event::kReplaced);
  EXPECT_EQ(send(replaced_us + SocketRetry::kPeriodUs),
            protocol::UdpStatus::kOk);
}
#endif

}  // namespace
