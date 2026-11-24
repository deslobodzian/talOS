// When the RoboRIO loop retries opening the gateway's socket: soon enough
// after the network comes up that a rebooted RoboRIO's State is back as soon
// as it can be, and seldom enough that a network that stays down costs the
// 5 ms loop nothing. And when it replaces a socket that can no longer send.
#include "SocketRetry.h"

#include <gtest/gtest.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "DriverStationSampler.h"
#include "LoopConsole.h"
#include "RioLog.h"
#include "SocketLinkTick.h"
#include "driver_station/driver_station.h"
#include "hardware/endpoint.h"
#include "hardware/gateway.h"

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
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
  uint32_t failed_events = 0;
  uint32_t none_events = 0;
  for (uint64_t now = kStartUs; now < kStartUs + 10'000'000;
       now += kLoopPeriodUs) {
    bool attempted = false;
    const auto event = retry.Poll(now, [&] {
      attempted = true;
      ++attempts;
      return now >= kRouteUpUs;
    });
    if (event == SocketRetry::Event::kOpened) {
      EXPECT_TRUE(attempted);
      opened_us = now;
      break;
    }
    if (attempted) {
      EXPECT_EQ(event, SocketRetry::Event::kFailed) << now;
      ++failed_events;
    } else {
      EXPECT_EQ(event, SocketRetry::Event::kNone) << now;
      ++none_events;
    }
  }
  ASSERT_NE(opened_us, 0u) << "never opened";
  EXPECT_LE(opened_us - kRouteUpUs, kBoundUs)
      << "opened " << opened_us - kRouteUpUs
      << " us after the route appeared; bound: one retry period (100 ms) "
         "plus one loop period (5 ms)";
  EXPECT_EQ(failed_events, attempts - 1);
  EXPECT_GT(none_events, 0u);
  EXPECT_EQ(retry.failures(), attempts - 1);
  EXPECT_EQ(retry.first_failure_us(), kStartUs);
}

// A network that never comes up: one attempt per retry period, not one per
// loop tick, and none once the socket is open.
// REQ-OPS-28
TEST(SocketRetry, AttemptsAtMostOncePerPeriod) {
  SocketRetry retry;
  bool up = false;
  uint32_t attempts = 0;
  uint32_t failed_events = 0;
  uint32_t none_events = 0;
  uint64_t last_attempt_us = 0;
  for (uint64_t now = kStartUs; now < kStartUs + 10'000'000;
       now += kLoopPeriodUs) {
    bool attempted = false;
    const auto event = retry.Poll(now, [&] {
      attempted = true;
      if (attempts > 0) {
        EXPECT_GE(now - last_attempt_us, SocketRetry::kPeriodUs)
            << "attempt " << attempts << " at " << now;
      }
      ++attempts;
      last_attempt_us = now;
      return up;
    });
    if (attempted) {
      EXPECT_EQ(event, SocketRetry::Event::kFailed) << now;
      ++failed_events;
    } else {
      EXPECT_EQ(event, SocketRetry::Event::kNone) << now;
      ++none_events;
    }
  }
  EXPECT_EQ(attempts, 10'000'000 / SocketRetry::kPeriodUs);
  EXPECT_EQ(failed_events, attempts);
  EXPECT_GT(none_events, 0u);

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
// REQ-OPS-28
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

// A non-blocking send refused with EAGAIN / EWOULDBLOCK
// (UdpStatus::kWouldBlock, send buffer full) is reported to the loop without
// marking the socket lost, triggering a reopen, or ending an open
// kSocketError spell. REQ-RT-22
TEST(SocketRetry, WouldBlockReportsDropAndKeepsTheSocketOpen) {
  using SendEvent = SocketRetry::SendEvent;
  using UdpStatus = talos::protocol::UdpStatus;
  SocketRetry retry;
  // Before the socket opens, a would-block status is ignored.
  EXPECT_EQ(retry.ObserveSend(kStartUs, UdpStatus::kWouldBlock, 0),
            SendEvent::kNone);

  ASSERT_EQ(retry.Poll(kStartUs, [] { return true; }),
            SocketRetry::Event::kOpened);
  const uint64_t burst_us = kStartUs + SocketRetry::kPeriodUs;
  for (uint32_t i = 1; i <= 5; ++i) {
    EXPECT_EQ(retry.ObserveSend(burst_us + i * kLoopPeriodUs,
                                UdpStatus::kWouldBlock, 0),
              SendEvent::kWouldBlock);
    EXPECT_TRUE(retry.open());
    EXPECT_FALSE(retry.replacing());
    EXPECT_EQ(retry.Poll(burst_us + i * kLoopPeriodUs,
                         [] {
                           ADD_FAILURE()
                               << "kWouldBlock must not trigger reopen";
                           return true;
                         }),
              SocketRetry::Event::kNone);
  }

  // A kWouldBlock during an open kSocketError spell does not end the spell or
  // inflate spell_failures; a later kOk after kSpellQuietUs still recovers it.
  const uint64_t fail_us = burst_us + 1'000'000;
  EXPECT_EQ(retry.ObserveSend(fail_us, UdpStatus::kSocketError, ENOBUFS),
            SendEvent::kFailing);
  EXPECT_EQ(retry.ObserveSend(fail_us + SocketRetry::kSpellQuietUs,
                              UdpStatus::kWouldBlock, 0),
            SendEvent::kWouldBlock);
  EXPECT_EQ(retry.spell_failures(), 1u);
  EXPECT_EQ(retry.spell_error(), ENOBUFS);
  EXPECT_TRUE(retry.open());
  EXPECT_EQ(
      retry.ObserveSend(fail_us + SocketRetry::kSpellQuietUs + kLoopPeriodUs,
                        UdpStatus::kOk, 0),
      SendEvent::kRecovered);
}

// Permanent receive errors lose the socket and schedule replacement on the
// same kPeriodUs cadence as ObserveSend, while non-socket outcomes (kOk,
// kWouldBlock, kDecodeError) and transient errors (ECONNREFUSED) keep it open.
// REQ-RT-22
TEST(SocketRetry, PermanentReceiveErrorReopensWhileTransientKeepsSocket) {
  using UdpStatus = talos::protocol::UdpStatus;

  // Before open, receive errors are ignored.
  {
    SocketRetry retry;
    retry.ObserveReceive(kStartUs, UdpStatus::kSocketError, EBADF);
    EXPECT_FALSE(retry.open());
    EXPECT_FALSE(retry.replacing());
  }

  // Non-socket statuses and transient errors keep the socket open.
  {
    SocketRetry retry;
    ASSERT_EQ(retry.Poll(kStartUs, [] { return true; }),
              SocketRetry::Event::kOpened);
    const uint64_t t = kStartUs + SocketRetry::kPeriodUs;
    for (const auto status :
         {UdpStatus::kOk, UdpStatus::kWouldBlock, UdpStatus::kDecodeError,
          UdpStatus::kInvalidArgument}) {
      retry.ObserveReceive(t, status, EBADF);
      EXPECT_TRUE(retry.open());
      EXPECT_FALSE(retry.replacing());
    }
    for (const int transient : {0, ECONNREFUSED, EAGAIN, EWOULDBLOCK, EINTR}) {
      SCOPED_TRACE(std::strerror(transient));
      retry.ObserveReceive(t, UdpStatus::kSocketError, transient);
      EXPECT_TRUE(retry.open());
      EXPECT_FALSE(retry.replacing());
      EXPECT_EQ(retry.Poll(t + kLoopPeriodUs,
                           [] {
                             ADD_FAILURE()
                                 << "transient receive error triggered reopen";
                             return true;
                           }),
                SocketRetry::Event::kNone);
    }
  }

  // Every permanent receive error transitions into replacing state and the
  // next due Poll replaces the socket on the kPeriodUs cadence.
  uint64_t now = kStartUs;
  for (const int permanent :
       {EBADF, ENOTCONN, ENOTSOCK, EPIPE, ENETUNREACH, EADDRNOTAVAIL}) {
    SCOPED_TRACE(std::strerror(permanent));
    SocketRetry retry;
    ASSERT_EQ(retry.Poll(now, [] { return true; }),
              SocketRetry::Event::kOpened);
    const uint64_t lost_us = now + SocketRetry::kPeriodUs;
    retry.ObserveReceive(lost_us, UdpStatus::kSocketError, permanent);
    EXPECT_FALSE(retry.open());
    EXPECT_TRUE(retry.replacing());
    EXPECT_EQ(retry.lost_us(), lost_us);
    EXPECT_EQ(retry.last_recv_error(), permanent);

    const uint64_t replaced_us = lost_us + kLoopPeriodUs;
    ASSERT_EQ(retry.Poll(replaced_us, [] { return true; }),
              SocketRetry::Event::kReplaced);
    EXPECT_TRUE(retry.open());
    EXPECT_FALSE(retry.replacing());

    // A permanent receive error immediately after replacement respects the
    // kPeriodUs throttle.
    retry.ObserveReceive(replaced_us + kLoopPeriodUs, UdpStatus::kSocketError,
                         permanent);
    EXPECT_FALSE(retry.open());
    EXPECT_TRUE(retry.replacing());
    for (uint64_t tick = replaced_us + 2 * kLoopPeriodUs;
         tick < replaced_us + SocketRetry::kPeriodUs; tick += kLoopPeriodUs) {
      EXPECT_EQ(retry.Poll(tick,
                           [] {
                             ADD_FAILURE() << "reopened before kPeriodUs";
                             return true;
                           }),
                SocketRetry::Event::kNone)
          << tick;
    }
    EXPECT_EQ(
        retry.Poll(replaced_us + SocketRetry::kPeriodUs, [] { return true; }),
        SocketRetry::Event::kReplaced);
    now += 1'000'000;
  }
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

// Unconnected loopback companion socket bound to an ephemeral port, so it
// receives frames from an Endpoint both before and after Endpoint::Reopen()
// without binding any fixed port.
class LoopbackCompanion {
 public:
  LoopbackCompanion() : fd_{::socket(AF_INET, SOCK_DGRAM, 0)} {
    if (fd_ < 0) return;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) != 0 ||
        ::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) !=
            0) {
      ::close(fd_);
      fd_ = -1;
      return;
    }
    port_ = BoundPort(fd_);
  }
  ~LoopbackCompanion() {
    if (fd_ >= 0) ::close(fd_);
  }
  LoopbackCompanion(const LoopbackCompanion&) = delete;
  LoopbackCompanion& operator=(const LoopbackCompanion&) = delete;

  uint16_t port() const { return port_; }

  struct Frame {
    talos::protocol::FrameType type{};
    uint64_t sequence{0};
    uint64_t monotonic_time_us{0};
    std::optional<talos::driver_station::DriverStationData> ds;
  };

  std::vector<Frame> DrainFrames() {
    std::vector<Frame> out;
    std::array<uint8_t, talos::protocol::kMaxFrameSize + 1> buf{};
    for (;;) {
      const ssize_t n =
          ::recvfrom(fd_, buf.data(), buf.size(), 0, nullptr, nullptr);
      if (n <= 0) break;
      talos::protocol::DecodedFrame decoded;
      if (talos::protocol::DecodeFrame(buf.data(), static_cast<std::size_t>(n),
                                       &decoded) !=
          talos::protocol::DecodeStatus::kOk) {
        continue;
      }
      Frame item;
      item.type = decoded.header.type;
      item.sequence = decoded.header.sequence;
      item.monotonic_time_us = decoded.header.monotonic_time_us;
      if (decoded.header.type == talos::protocol::FrameType::kDriverStation) {
        talos::driver_station::DriverStationData ds_data;
        if (talos::driver_station::Decode(
                {decoded.payload, decoded.payload_size}, ds_data)) {
          item.ds = ds_data;
        }
      }
      out.push_back(item);
    }
    return out;
  }

 private:
  int fd_{-1};
  uint16_t port_{0};
};

// Minimal in-memory hardware backend to observe real Gateway::Tick(now, false)
// disabling and neutralizing energized outputs during closed-socket ticks.
class LinkTestBackend final : public talos::hardware::Backend {
 public:
  bool Configure(const talos::hardware::Config&) override { return true; }
  bool Read(talos::hardware::State& state) override {
    for (uint8_t i = 0; i < state.motor_count; ++i) {
      state.motors[i].valid = true;
    }
    return true;
  }
  bool Apply(std::span<const talos::hardware::MotorRequest>) override {
    ++apply_calls;
    energized = true;
    return true;
  }
  std::span<const uint16_t> ApplyFailedDevices() const override { return {}; }
  void Neutral() override {
    ++neutral_calls;
    energized = false;
  }

  bool energized{false};
  int apply_calls{0};
  int neutral_calls{0};
};

// Thin wrapper around a real talos::hardware::Endpoint that counts lifecycle
// calls (Open, Close, Reopen, SendFrame) and supports one-shot send/reopen
// fault injection while forwarding normal ticks directly to the real Endpoint.
class FaultingEndpoint {
 public:
  explicit FaultingEndpoint(talos::hardware::Gateway& gateway)
      : inner_{gateway} {}

  talos::hardware::Endpoint& inner() { return inner_; }

  talos::protocol::UdpStatus Open(const char* local, uint16_t local_port,
                                  const char* remote, uint16_t remote_port) {
    ++open_calls;
    const auto status = inner_.Open(local, local_port, remote, remote_port);
    if (status == talos::protocol::UdpStatus::kOk) closed = false;
    return status;
  }

  talos::protocol::UdpStatus Reopen() {
    ++reopen_calls;
    reopen_times_us.push_back(current_now_us);
    if (fail_reopen) {
      errno = fail_reopen_errno;
      return talos::protocol::UdpStatus::kSocketError;
    }
    const auto status = inner_.Reopen();
    if (status == talos::protocol::UdpStatus::kOk) closed = false;
    return status;
  }

  void Close() {
    ++close_calls;
    closed = true;
    inner_.Close();
  }

  const talos::protocol::UdpStats& udp_stats() const {
    return use_stats_override_ ? stats_override_ : inner_.udp_stats();
  }

  talos::protocol::UdpStatus Tick(uint64_t now_us, bool enabled) {
    use_stats_override_ = false;
    const auto real_status = inner_.Tick(now_us, enabled);
    if (injected_tick_send_.has_value()) {
      const auto fault = *injected_tick_send_;
      injected_tick_send_.reset();
      stats_override_ = inner_.udp_stats();
      if (fault.status == talos::protocol::UdpStatus::kSocketError) {
        stats_override_.last_send_error = fault.err;
      } else if (fault.status == talos::protocol::UdpStatus::kWouldBlock) {
        ++stats_override_.send_would_block;
      }
      use_stats_override_ = true;
      return fault.status;
    }
    return real_status;
  }

  talos::protocol::UdpStatus last_receive_status() const {
    return inner_.last_receive_status();
  }
  int last_receive_error() const { return inner_.last_receive_error(); }
  const talos::hardware::DrainedCommand& drained() const {
    return inner_.drained();
  }

  talos::protocol::UdpStatus SendFrame(talos::protocol::FrameType type,
                                       uint64_t now_us, const void* payload,
                                       std::size_t payload_size) {
    ++send_frame_calls;
    use_stats_override_ = false;
    if (injected_ds_send_.has_value()) {
      const auto fault = *injected_ds_send_;
      injected_ds_send_.reset();
      stats_override_ = inner_.udp_stats();
      if (fault.status == talos::protocol::UdpStatus::kSocketError) {
        stats_override_.last_send_error = fault.err;
      } else if (fault.status == talos::protocol::UdpStatus::kWouldBlock) {
        ++stats_override_.send_would_block;
      }
      use_stats_override_ = true;
      return fault.status;
    }
    return inner_.SendFrame(type, now_us, payload, payload_size);
  }

  struct InjectedSend {
    talos::protocol::UdpStatus status;
    int err{0};
  };

  void InjectNextTickSend(talos::protocol::UdpStatus status, int err = 0) {
    injected_tick_send_ = InjectedSend{status, err};
  }
  void InjectNextDsSend(talos::protocol::UdpStatus status, int err = 0) {
    injected_ds_send_ = InjectedSend{status, err};
  }

  uint64_t current_now_us{0};
  bool fail_reopen{false};
  int fail_reopen_errno{ENETUNREACH};
  bool closed{true};
  int open_calls{0};
  int reopen_calls{0};
  int close_calls{0};
  int send_frame_calls{0};
  std::vector<uint64_t> reopen_times_us;

 private:
  talos::hardware::Endpoint inner_;
  std::optional<InjectedSend> injected_tick_send_;
  std::optional<InjectedSend> injected_ds_send_;
  talos::protocol::UdpStats stats_override_{};
  bool use_stats_override_{false};
};

// Drives talos::rio::TickSocketLink() (Robot.cpp's production socket-loss tick
// path) with a real Gateway, real Endpoint, real loopback UDP companion, real
// SocketRetry, and real DriverStationSampler without NetComm or wall-clock
// sleeps.
struct SocketLinkHarness {
  static constexpr uint64_t kBootId = 0x1020'3040'5060'7081ULL;

  SocketLinkHarness()
      : gateway{MakeConfig(), backend, kBootId}, endpoint{gateway} {}

  static talos::hardware::Config MakeConfig() {
    talos::hardware::Config config;
    config.commissioned = true;
    config.period_us = static_cast<uint32_t>(kLoopPeriodUs);
    config.command_timeout_us = 100'000;
    talos::hardware::MotorConfig motor;
    motor.id = 1;
    motor.name = "drive";
    motor.can_id = 1;
    config.motors.push_back(motor);
    return config;
  }

  void Tick(uint64_t now_us, uint32_t ds_flags) {
    endpoint.current_now_us = now_us;
    talos::driver_station::DriverStationData view{};
    view.fms.flags = ds_flags;
    talos::rio::SocketLinkTickContext ctx{
        .now_us = now_us,
        .boot_id = kBootId,
        .period_us = kLoopPeriodUs,
        .ds_refreshed_us = now_us,
        .driver_station_view = view,
        .socket_retry = socket_retry,
        .ds_sampler = ds_sampler,
        .log = log,
        .loop_console = loop_console,
        .ds_buffer = ds_buffer,
        .ds_packet = ds_packet,
    };
    talos::rio::TickSocketLink(
        ctx, endpoint, gateway,
        [&] {
          return endpoint.Open("127.0.0.1", 0, "127.0.0.1", companion.port());
        },
        [&](uint64_t pre_arm_us) { last_pre_arm_us = pre_arm_us; },
        [&] { return now_us * 1000; });
  }

  bool EnergizeMotor(uint64_t seq, uint64_t now_us) {
    const auto state = gateway.snapshot();
    talos::hardware::Command command{};
    command.config_id = state.config_id;
    command.boot_id = state.boot_id;
    command.epoch = state.epoch;
    command.observed_time_us = state.sample_time_us;
    command.count = state.motor_count;
    command.motors[0] = {state.motors[0].id, talos::hardware::Mode::kDutyCycle,
                         0, 0.25, 0};
    return gateway.Accept(command, seq, now_us);
  }

  LoopbackCompanion companion;
  LinkTestBackend backend;
  talos::hardware::Gateway gateway;
  FaultingEndpoint endpoint;
  SocketRetry socket_retry;
  talos::rio::DriverStationSampler ds_sampler;
  talos::rio::RioLog log;
  talos::rio::LoopConsole loop_console{log};
  std::array<uint8_t, talos::protocol::kMaxPayloadSize> ds_buffer{};
  talos::driver_station::DriverStationData ds_packet{};
  uint64_t last_pre_arm_us{0};
};

int g_injected_recv_errno = 0;

struct ScopedReceiveErrorSeam {
  explicit ScopedReceiveErrorSeam(talos::hardware::Endpoint& endpoint)
      : endpoint_{endpoint} {
    g_injected_recv_errno = 0;
    endpoint_.SetReceiveErrorSeam(
        []() -> int { return std::exchange(g_injected_recv_errno, 0); });
  }
  ~ScopedReceiveErrorSeam() {
    g_injected_recv_errno = 0;
    endpoint_.SetReceiveErrorSeam(nullptr);
  }
  talos::hardware::Endpoint& endpoint_;
};

// Exercises Robot.cpp's actual TickSocketLink() path on receive errors:
// - Transient ECONNREFUSED keeps the real Endpoint open and outputs energized.
// - Permanent EBADF closes the real Endpoint immediately, suppresses the DS
//   send on that tick, ticks Gateway disabled (neutralizing active outputs!)
//   on every closed tick while throttled, retries Reopen() strictly on the
//   100 ms SocketRetry cadence without blocking, and resumes delivering
//   State (continuous sequence) and the pending DS mode generation on reopen.
// REQ-RT-22
TEST(
    SocketRetry,
    RobotTickPermanentReceiveErrorClosesEndpointTicksGatewayDisabledAndReopensOnCadence) {
  namespace ds = talos::driver_station;
  namespace hw = talos::hardware;
  namespace protocol = talos::protocol;
  constexpr uint32_t kTeleopEnabled =
      ds::kDsAttached | ds::kEnabled | ds::kTeleop;
  constexpr uint32_t kAutoEnabled =
      ds::kDsAttached | ds::kEnabled | ds::kAutonomous;

  SocketLinkHarness h;
  ASSERT_NE(h.companion.port(), 0);
  ScopedReceiveErrorSeam recv_seam{h.endpoint.inner()};

  // Tick 0: initial open succeeds, State + DS reach the real companion.
  h.Tick(kStartUs, kTeleopEnabled);
  ASSERT_TRUE(h.socket_retry.open());
  ASSERT_EQ(h.endpoint.open_calls, 1);
  ASSERT_EQ(h.endpoint.close_calls, 0);
  auto frames = h.companion.DrainFrames();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].type, protocol::FrameType::kHardwareState);
  EXPECT_EQ(frames[0].sequence, 1u);
  EXPECT_EQ(frames[1].type, protocol::FrameType::kDriverStation);
  EXPECT_EQ(frames[1].sequence, 2u);

  // Energize the motor so closed-socket Gateway::Tick(now, false)
  // neutralization is directly observable on the backend and gateway flags.
  ASSERT_TRUE(h.EnergizeMotor(/*seq=*/1, kStartUs));
  ASSERT_TRUE(h.backend.energized);
  ASSERT_NE(h.gateway.snapshot().flags & hw::kCommandActive, 0u);

  // Tick 1 (+5 ms): transient receive error (ECONNREFUSED) keeps socket open.
  g_injected_recv_errno = ECONNREFUSED;
  h.Tick(kStartUs + kLoopPeriodUs, kTeleopEnabled);
  EXPECT_TRUE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.close_calls, 0);
  EXPECT_EQ(h.endpoint.reopen_calls, 0);
  EXPECT_TRUE(h.backend.energized);
  frames = h.companion.DrainFrames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].type, protocol::FrameType::kHardwareState);
  EXPECT_EQ(frames[0].sequence, 3u);

  // Tick 2 (+10 ms): permanent receive error (EBADF) on the same tick as a
  // mode transition (teleop -> auto) closes the real Endpoint immediately and
  // suppresses sending the DS frame on the lost socket.
  const uint64_t loss_us = kStartUs + 2 * kLoopPeriodUs;
  g_injected_recv_errno = EBADF;
  const int ds_sends_before_loss = h.endpoint.send_frame_calls;
  h.Tick(loss_us, kAutoEnabled);
  EXPECT_FALSE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.close_calls, 1);
  EXPECT_TRUE(h.endpoint.closed);
  EXPECT_EQ(h.endpoint.send_frame_calls, ds_sends_before_loss)
      << "DS frame must not be sent after ObserveReceive closed the socket";
  // Confirm the underlying real OS socket inside Endpoint is genuinely closed.
  const uint8_t probe = 0;
  EXPECT_EQ(h.endpoint.inner().SendFrame(protocol::FrameType::kDriverStation,
                                         loss_us, &probe, sizeof(probe)),
            protocol::UdpStatus::kSocketError);
  frames = h.companion.DrainFrames();
  uint64_t last_seq_before_loss = 3u;
  for (const auto& f : frames) {
    EXPECT_NE(f.type, protocol::FrameType::kDriverStation)
        << "no DS frame may leave after receive loss";
    last_seq_before_loss = f.sequence;
  }

  // Closed ticks inside the 100 ms throttle window: Gateway must be ticked
  // disabled immediately (neutralizing active outputs!) with zero Reopen()
  // attempts and zero frames sent.
  const int neutral_before_closed = h.backend.neutral_calls;
  for (uint64_t now = loss_us + kLoopPeriodUs;
       now < kStartUs + SocketRetry::kPeriodUs; now += kLoopPeriodUs) {
    h.Tick(now, kAutoEnabled);
    EXPECT_FALSE(h.socket_retry.open()) << now;
    EXPECT_EQ(h.endpoint.reopen_calls, 0)
        << "reopened before 100 ms at " << now;
    EXPECT_FALSE(h.backend.energized)
        << "closed-socket tick did not neutralize energized outputs at " << now;
    EXPECT_EQ(h.gateway.snapshot().flags & hw::kEnabled, 0u) << now;
    EXPECT_EQ(h.gateway.snapshot().flags & hw::kCommandActive, 0u) << now;
    EXPECT_TRUE(h.companion.DrainFrames().empty())
        << "closed-socket tick sent a frame at " << now;
  }
  EXPECT_GT(h.backend.neutral_calls, neutral_before_closed);

  // At +100 ms: first Reopen() attempt fails ENETUNREACH without blocking;
  // socket stays closed and Gateway keeps ticking disabled for another 100 ms.
  h.endpoint.fail_reopen = true;
  const uint64_t first_reopen_us = kStartUs + SocketRetry::kPeriodUs;
  h.Tick(first_reopen_us, kAutoEnabled);
  EXPECT_FALSE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.reopen_calls, 1);
  EXPECT_EQ(h.gateway.snapshot().flags & hw::kEnabled, 0u);
  EXPECT_TRUE(h.companion.DrainFrames().empty());

  for (uint64_t now = first_reopen_us + kLoopPeriodUs;
       now < first_reopen_us + SocketRetry::kPeriodUs; now += kLoopPeriodUs) {
    h.Tick(now, kAutoEnabled);
    EXPECT_FALSE(h.socket_retry.open()) << now;
    EXPECT_EQ(h.endpoint.reopen_calls, 1) << "reopened early at " << now;
    EXPECT_EQ(h.gateway.snapshot().flags & hw::kEnabled, 0u) << now;
    EXPECT_TRUE(h.companion.DrainFrames().empty()) << now;
  }

  // At +200 ms: network returns, Reopen() succeeds, State resumes with
  // continuous frame sequence and the deferred auto-enabled DS packet goes out
  // immediately with the original loss_us mode_since_us generation.
  h.endpoint.fail_reopen = false;
  const uint64_t second_reopen_us = first_reopen_us + SocketRetry::kPeriodUs;
  h.Tick(second_reopen_us, kAutoEnabled);
  EXPECT_TRUE(h.socket_retry.open());
  EXPECT_FALSE(h.endpoint.closed);
  ASSERT_EQ(h.endpoint.reopen_calls, 2);
  EXPECT_EQ(h.endpoint.reopen_times_us,
            (std::vector<uint64_t>{first_reopen_us, second_reopen_us}));
  EXPECT_NE(h.gateway.snapshot().flags & hw::kEnabled, 0u);

  frames = h.companion.DrainFrames();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].type, protocol::FrameType::kHardwareState);
  EXPECT_EQ(frames[0].sequence, last_seq_before_loss + 1);
  EXPECT_EQ(frames[1].type, protocol::FrameType::kDriverStation);
  EXPECT_EQ(frames[1].sequence, last_seq_before_loss + 2);
  ASSERT_TRUE(frames[1].ds.has_value());
  EXPECT_EQ(frames[1].ds->fms.flags, kAutoEnabled);
  EXPECT_EQ(frames[1].ds->mode_since_us, loss_us);
}

// Exercises Robot.cpp's actual TickSocketLink() path when Endpoint::Tick()
// fails to send State with a permanent error (ENETUNREACH, source address
// gone): closes the Endpoint, suppresses DS send on that tick, tries Reopen()
// on the next tick (>= 100 ms since open), ticks Gateway disabled when Reopen()
// fails, and recovers on the 100 ms cadence.
// REQ-RT-22
TEST(
    SocketRetry,
    RobotTickPermanentStateSendErrorClosesEndpointSuppressesDsTicksDisabledAndReopens) {
  namespace ds = talos::driver_station;
  namespace hw = talos::hardware;
  namespace protocol = talos::protocol;
  constexpr uint32_t kTeleopEnabled =
      ds::kDsAttached | ds::kEnabled | ds::kTeleop;
  constexpr uint32_t kAutoEnabled =
      ds::kDsAttached | ds::kEnabled | ds::kAutonomous;

  SocketLinkHarness h;
  ASSERT_NE(h.companion.port(), 0);

  h.Tick(kStartUs, kTeleopEnabled);
  ASSERT_TRUE(h.socket_retry.open());
  ASSERT_EQ(h.companion.DrainFrames().size(), 2u);

  // Advance > 100 ms past initial open and energize the motor.
  const uint64_t steady_us = kStartUs + 200'000;
  h.Tick(steady_us, kTeleopEnabled);
  ASSERT_TRUE(h.EnergizeMotor(/*seq=*/1, steady_us));
  ASSERT_TRUE(h.backend.energized);
  ASSERT_FALSE(h.companion.DrainFrames().empty());

  // State send fails permanently (ENETUNREACH) on the same tick as a mode
  // change (teleop -> auto): must close Endpoint and skip DS send.
  const uint64_t loss_us = steady_us + kLoopPeriodUs;
  const int ds_sends_before = h.endpoint.send_frame_calls;
  h.endpoint.InjectNextTickSend(protocol::UdpStatus::kSocketError, ENETUNREACH);
  h.Tick(loss_us, kAutoEnabled);
  EXPECT_FALSE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.close_calls, 1);
  EXPECT_TRUE(h.endpoint.closed);
  EXPECT_EQ(h.endpoint.send_frame_calls, ds_sends_before)
      << "DS frame must not be sent after SendEvent::kLost closed the socket";
  (void)h.companion.DrainFrames();

  // Next tick (+5 ms, already >= 100 ms since initial open): Reopen() is tried
  // immediately; when it fails ENETUNREACH, Gateway ticks disabled and
  // neutralizes the energized motor, and no further Reopen() runs for 100 ms.
  h.endpoint.fail_reopen = true;
  const uint64_t first_reopen_us = loss_us + kLoopPeriodUs;
  h.Tick(first_reopen_us, kAutoEnabled);
  EXPECT_FALSE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.reopen_calls, 1);
  EXPECT_FALSE(h.backend.energized);
  EXPECT_EQ(h.gateway.snapshot().flags & hw::kEnabled, 0u);
  EXPECT_EQ(h.gateway.snapshot().flags & hw::kCommandActive, 0u);
  EXPECT_TRUE(h.companion.DrainFrames().empty());

  for (uint64_t now = first_reopen_us + kLoopPeriodUs;
       now < first_reopen_us + SocketRetry::kPeriodUs; now += kLoopPeriodUs) {
    h.Tick(now, kAutoEnabled);
    EXPECT_EQ(h.endpoint.reopen_calls, 1) << now;
    EXPECT_EQ(h.gateway.snapshot().flags & hw::kEnabled, 0u) << now;
    EXPECT_TRUE(h.companion.DrainFrames().empty()) << now;
  }

  // 100 ms after the failed reopen: Reopen() succeeds, sending State and the
  // deferred auto-enabled DS packet with its original loss_us generation.
  h.endpoint.fail_reopen = false;
  const uint64_t recovered_us = first_reopen_us + SocketRetry::kPeriodUs;
  h.Tick(recovered_us, kAutoEnabled);
  EXPECT_TRUE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.reopen_calls, 2);
  EXPECT_EQ(h.endpoint.reopen_times_us,
            (std::vector<uint64_t>{first_reopen_us, recovered_us}));
  const auto frames = h.companion.DrainFrames();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].type, protocol::FrameType::kHardwareState);
  EXPECT_EQ(frames[1].type, protocol::FrameType::kDriverStation);
  ASSERT_TRUE(frames[1].ds.has_value());
  EXPECT_EQ(frames[1].ds->fms.flags, kAutoEnabled);
  EXPECT_EQ(frames[1].ds->mode_since_us, loss_us);
}

// Exercises Robot.cpp's actual TickSocketLink() Driver Station send &
// RevertSample() path on both transient (kWouldBlock) and permanent (EPIPE)
// DS send failures:
// - Transient kWouldBlock keeps the Endpoint open, reverts ds_sampler, and
//   retries the mode transition on the very next 5 ms tick (inside the 20 ms
//   kSamplePeriodUs window) with the original mode_since_us generation.
// - Permanent EPIPE closes the Endpoint, reverts ds_sampler, immediately
//   delivers the reverted mode transition upon 1-tick Reopen() with continuous
//   frame sequence, and a second EPIPE within 100 ms ticks Gateway disabled
//   until the 100 ms reopen cadence expires.
// REQ-RT-22
TEST(
    SocketRetry,
    RobotTickDriverStationSendFailureRevertsSampleClosesOnPermanentAndRetriesImmediately) {
  namespace ds = talos::driver_station;
  namespace hw = talos::hardware;
  namespace protocol = talos::protocol;
  constexpr uint32_t kTeleopEnabled =
      ds::kDsAttached | ds::kEnabled | ds::kTeleop;
  constexpr uint32_t kAutoEnabled =
      ds::kDsAttached | ds::kEnabled | ds::kAutonomous;

  SocketLinkHarness h;
  ASSERT_NE(h.companion.port(), 0);

  // Tick 0: initial teleop-enabled State + DS sent.
  h.Tick(kStartUs, kTeleopEnabled);
  ASSERT_TRUE(h.socket_retry.open());
  ASSERT_EQ(h.companion.DrainFrames().size(), 2u);

  // Part A: transient kWouldBlock on DS send during teleop -> auto switch.
  const uint64_t switch1_us = kStartUs + kLoopPeriodUs;
  h.endpoint.InjectNextDsSend(protocol::UdpStatus::kWouldBlock);
  h.Tick(switch1_us, kAutoEnabled);
  EXPECT_TRUE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.close_calls, 0);
  EXPECT_EQ(h.endpoint.reopen_calls, 0);
  auto frames = h.companion.DrainFrames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].type, protocol::FrameType::kHardwareState);

  // Next tick (+10 ms total, well inside 20 ms kSamplePeriodUs): RevertSample()
  // causes the auto-enabled DS packet to retry immediately with switch1_us gen.
  const uint64_t retry1_us = switch1_us + kLoopPeriodUs;
  h.Tick(retry1_us, kAutoEnabled);
  frames = h.companion.DrainFrames();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[1].type, protocol::FrameType::kDriverStation);
  ASSERT_TRUE(frames[1].ds.has_value());
  EXPECT_EQ(frames[1].ds->fms.flags, kAutoEnabled);
  EXPECT_EQ(frames[1].ds->mode_since_us, switch1_us);

  // Part B: advance > 100 ms so a 1-tick Reopen() is due, send a periodic DS
  // packet at t2 so last_sample_us_ == t2, then fail DS send permanently
  // (EPIPE) on an auto -> teleop switch 5 ms later.
  const uint64_t t2 = kStartUs + 200'000;
  h.Tick(t2, kAutoEnabled);
  frames = h.companion.DrainFrames();
  ASSERT_EQ(frames.size(), 2u);
  const uint64_t seq_after_t2 = frames.back().sequence;

  const uint64_t switch2_us = t2 + kLoopPeriodUs;
  h.endpoint.InjectNextDsSend(protocol::UdpStatus::kSocketError, EPIPE);
  h.Tick(switch2_us, kTeleopEnabled);
  EXPECT_FALSE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.close_calls, 1);
  EXPECT_TRUE(h.endpoint.closed);
  frames = h.companion.DrainFrames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].type, protocol::FrameType::kHardwareState);
  EXPECT_EQ(frames[0].sequence, seq_after_t2 + 1);

  // Next tick (t2 + 10 ms, still < 20 ms after t2): Reopen() runs at once and
  // the reverted teleop-enabled DS packet goes out immediately with switch2_us
  // generation and continuous UDP sequence.
  const uint64_t reopen1_us = switch2_us + kLoopPeriodUs;
  h.Tick(reopen1_us, kTeleopEnabled);
  EXPECT_TRUE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.reopen_calls, 1);
  frames = h.companion.DrainFrames();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].type, protocol::FrameType::kHardwareState);
  EXPECT_EQ(frames[0].sequence, seq_after_t2 + 2);
  EXPECT_EQ(frames[1].type, protocol::FrameType::kDriverStation);
  EXPECT_EQ(frames[1].sequence, seq_after_t2 + 3);
  ASSERT_TRUE(frames[1].ds.has_value());
  EXPECT_EQ(frames[1].ds->fms.flags, kTeleopEnabled);
  EXPECT_EQ(frames[1].ds->mode_since_us, switch2_us);

  // Part C: energize motor and inject a second EPIPE on the next periodic DS
  // tick (within 100 ms of reopen1_us): Endpoint closes, Gateway ticks
  // disabled (neutralizing outputs!) until reopen1_us + 100 ms, then Reopen()
  // succeeds and delivers the reverted DS packet.
  ASSERT_TRUE(h.EnergizeMotor(/*seq=*/1, reopen1_us));
  ASSERT_TRUE(h.backend.energized);
  const uint64_t periodic_ds_us = reopen1_us + ds::kSamplePeriodUs;
  h.endpoint.InjectNextDsSend(protocol::UdpStatus::kSocketError, EPIPE);
  h.Tick(periodic_ds_us, kTeleopEnabled);
  EXPECT_FALSE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.close_calls, 2);
  (void)h.companion.DrainFrames();

  for (uint64_t now = periodic_ds_us + kLoopPeriodUs;
       now < reopen1_us + SocketRetry::kPeriodUs; now += kLoopPeriodUs) {
    h.Tick(now, kTeleopEnabled);
    EXPECT_FALSE(h.socket_retry.open()) << now;
    EXPECT_EQ(h.endpoint.reopen_calls, 1) << now;
    EXPECT_FALSE(h.backend.energized) << now;
    EXPECT_EQ(h.gateway.snapshot().flags & hw::kEnabled, 0u) << now;
    EXPECT_TRUE(h.companion.DrainFrames().empty()) << now;
  }

  const uint64_t reopen2_us = reopen1_us + SocketRetry::kPeriodUs;
  h.Tick(reopen2_us, kTeleopEnabled);
  EXPECT_TRUE(h.socket_retry.open());
  EXPECT_EQ(h.endpoint.reopen_calls, 2);
  EXPECT_EQ(h.endpoint.reopen_times_us,
            (std::vector<uint64_t>{reopen1_us, reopen2_us}));
  frames = h.companion.DrainFrames();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].type, protocol::FrameType::kHardwareState);
  EXPECT_EQ(frames[1].type, protocol::FrameType::kDriverStation);
  ASSERT_TRUE(frames[1].ds.has_value());
  EXPECT_EQ(frames[1].ds->fms.flags, kTeleopEnabled);
  EXPECT_EQ(frames[1].ds->mode_since_us, switch2_us);
}
#endif

// Backward clock steps must not underflow now_us - last_attempt_us_ (bypassing
// the 100 ms reopen cadence) or now_us - spell_last_failure_us_ (ending a
// failing send spell prematurely).
// REQ-RT-22
TEST(SocketRetry, BackwardClockDoesNotBypassRetryOrSpellQuietPeriod) {
  SocketRetry retry;
  int attempts = 0;
  const auto fail_open = [&] {
    ++attempts;
    return false;
  };
  ASSERT_EQ(retry.Poll(10'000'000, fail_open), SocketRetry::Event::kFailed);
  ASSERT_EQ(attempts, 1);
  EXPECT_EQ(retry.Poll(5'000'000, fail_open), SocketRetry::Event::kNone);
  EXPECT_EQ(attempts, 1);

  ASSERT_EQ(
      retry.Poll(10'000'000 + SocketRetry::kPeriodUs, [] { return true; }),
      SocketRetry::Event::kOpened);
  ASSERT_EQ(retry.SendFailed(20'000'000, ECONNREFUSED),
            SocketRetry::SendEvent::kFailing);
  EXPECT_EQ(retry.SendSucceeded(19'000'000), SocketRetry::SendEvent::kNone)
      << "backward clock step prematurely ended failing send spell";
}

}  // namespace
