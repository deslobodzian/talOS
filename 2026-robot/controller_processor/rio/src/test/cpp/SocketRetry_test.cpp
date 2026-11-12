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

// A send error the network clears by itself keeps the socket: a full
// interface queue (ENOBUFS), the companion's port closed (ECONNREFUSED), no
// interface or route yet (ENETDOWN, ENETUNREACH, EHOSTUNREACH), a full socket
// buffer (EAGAIN). A new socket would fail them the same way, so the loop
// goes on sending on this one and opens nothing.
// REQ-RT-22
TEST(SocketRetry, ATransientSendErrorKeepsTheSocket) {
  SocketRetry retry;
  ASSERT_EQ(retry.Poll(kStartUs, [] { return true; }),
            SocketRetry::Event::kOpened);
  uint64_t now = kStartUs;
  for (const int error :
       {ENOBUFS, ECONNREFUSED, ENETDOWN, ENETUNREACH, EHOSTUNREACH, EAGAIN}) {
    SCOPED_TRACE(std::strerror(error));
    now += SocketRetry::kPeriodUs;
    EXPECT_FALSE(retry.SendFailed(now, error));
    EXPECT_TRUE(retry.open());
    EXPECT_EQ(retry.Poll(now + kLoopPeriodUs,
                         [] {
                           ADD_FAILURE() << "a socket was opened";
                           return true;
                         }),
              SocketRetry::Event::kNone);
  }
}

#ifndef _WIN32
// A loopback UDP port the OS just handed out and released.
uint16_t FreeLoopbackPort() {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return 0;
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  socklen_t size = sizeof(address);
  const bool bound =
      ::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0 &&
      ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size) == 0;
  ::close(fd);
  return bound ? ntohs(address.sin_port) : 0;
}

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
  const uint16_t gateway_port = FreeLoopbackPort();
  const uint16_t companion_port = FreeLoopbackPort();
  ASSERT_NE(gateway_port, 0);
  ASSERT_NE(companion_port, 0);
  protocol::RuntimeUdpPeer companion;
  ASSERT_EQ(
      companion.Open("127.0.0.1", companion_port, "127.0.0.1", gateway_port),
      protocol::UdpStatus::kOk);
  protocol::RuntimeUdpPeer gateway;
  SocketRetry retry;
  // What Robot.cpp's loop passes: Endpoint::Reopen while replacing,
  // Endpoint::Open before the first socket.
  const auto open = [&] {
    return (retry.replacing()
                ? gateway.Reopen()
                : gateway.Open("127.0.0.1", gateway_port, "127.0.0.1",
                               companion_port)) == protocol::UdpStatus::kOk;
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
