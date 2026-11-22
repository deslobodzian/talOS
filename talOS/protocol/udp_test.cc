#include "udp.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "gtest/gtest.h"
#include "udp_test_helpers.h"

namespace talos::protocol {
namespace {

// The port the OS bound `peer` to, when its Open() asked for port 0.
uint16_t BoundPort(const RuntimeUdpPeer& peer) {
  sockaddr_in address{};
  socklen_t size = sizeof(address);
  if (::getsockname(peer.NativeFd(), reinterpret_cast<sockaddr*>(&address),
                    &size) != 0) {
    return 0;
  }
  return ntohs(address.sin_port);
}

// Both send paths (copying and in-buffer) reach the peer intact and number
// their frames from one sequence. Both ends bind port 0: the receiving end
// stays unconnected, so it opens before it knows the sender's port, and the
// sender connects to the port the receiver was given.
TEST(RuntimeUdpTest, SendsFramesOverLoopback) {
  RuntimeUdpPeer mac;
  RuntimeUdpPeer rio;
  UdpSocketOptions options;
  options.non_blocking = false;
  UdpSocketOptions unconnected = options;
  unconnected.connect_peer = false;

  ASSERT_EQ(rio.Open("127.0.0.1", 0, "127.0.0.1", 0, unconnected),
            UdpStatus::kOk);
  const uint16_t rio_port = BoundPort(rio);
  ASSERT_NE(rio_port, 0);
  ASSERT_EQ(mac.Open("127.0.0.1", 0, "127.0.0.1", rio_port, options),
            UdpStatus::kOk);

  const std::array<uint8_t, 6> payload = {1, 2, 3, 4, 5, 6};
  ASSERT_EQ(mac.SendFrame(FrameType::kHardwareCommand, kFlagNone, 1000,
                          payload.data(), payload.size()),
            UdpStatus::kOk);

  DecodedFrame received;
  ASSERT_EQ(rio.TryReceive(&received), UdpStatus::kOk);
  EXPECT_EQ(received.header.type, FrameType::kHardwareCommand);
  EXPECT_EQ(received.header.sequence, 1u);
  EXPECT_EQ(received.header.monotonic_time_us, 1000u);
  ASSERT_EQ(received.payload_size, payload.size());
  EXPECT_EQ(std::memcmp(received.payload, payload.data(), payload.size()), 0);

  constexpr std::size_t kSize = 16;
  uint8_t* buffered = mac.MutablePayloadBuffer(kSize);
  ASSERT_NE(buffered, nullptr);
  for (std::size_t i = 0; i < kSize; ++i) {
    buffered[i] = static_cast<uint8_t>(99 + i);
  }
  ASSERT_EQ(
      mac.SendBufferedFrame(FrameType::kHardwareState, kFlagNone, 22, kSize),
      UdpStatus::kOk);

  ASSERT_EQ(rio.TryReceive(&received), UdpStatus::kOk);
  EXPECT_EQ(received.header.type, FrameType::kHardwareState);
  EXPECT_EQ(received.header.sequence, 2u);
  ASSERT_EQ(received.payload_size, kSize);
  for (std::size_t i = 0; i < kSize; ++i) {
    EXPECT_EQ(received.payload[i], static_cast<uint8_t>(99 + i));
  }
}

// A loopback UDP port the OS just handed out and released, so the test does
// not depend on a fixed port in the ephemeral range staying free.
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

TEST(RuntimeUdpTest, NonBlockingReceiveReportsWouldBlock) {
  const uint16_t remote = FreeLoopbackPort();
  ASSERT_NE(remote, 0);
  RuntimeUdpPeer peer;
  ASSERT_EQ(peer.Open("127.0.0.1", 0, "127.0.0.1", remote), UdpStatus::kOk);

  DecodedFrame received;
  EXPECT_EQ(peer.TryReceive(&received), UdpStatus::kWouldBlock);
}

// A second process opening a live peer's port -- a second hardware bridge
// on the RoboRIO port -- fails rather than sharing its datagrams, while a
// restarted peer gets the port back as soon as the old one closed.
TEST(RuntimeUdpTest, ALivePeersPortIsRefusedUntilItCloses) {
  const uint16_t port = FreeLoopbackPort();
  const uint16_t remote = FreeLoopbackPort();
  ASSERT_NE(port, 0);
  ASSERT_NE(remote, 0);
  RuntimeUdpPeer first;
  ASSERT_EQ(first.Open("127.0.0.1", port, "127.0.0.1", remote), UdpStatus::kOk);
  RuntimeUdpPeer second;
  EXPECT_EQ(second.Open("127.0.0.1", port, "127.0.0.1", remote),
            UdpStatus::kSocketError);
  first.Close();
  EXPECT_EQ(second.Open("127.0.0.1", port, "127.0.0.1", remote),
            UdpStatus::kOk);
}

// A socket that can no longer send (shut down for writing here; on macOS
// also one the kernel marked defunct, as around sleep and wake) fails both
// sends with kSocketError and EPIPE, and the process lives on. The sends
// run in a child with SIGPIPE at its default action, so a send that raises
// it kills the child, not the test. Only macOS raises SIGPIPE for a datagram
// socket (when SO_NOSIGPIPE is not set); Linux returns EPIPE without it, so
// there the child lives either way and the case checks the error a caller
// tells a lost socket by.
// REQ-RT-22
TEST(RuntimeUdpTest, ASendTheSocketCannotMakeFailsWithoutSigpipe) {
  const uint16_t remote = FreeLoopbackPort();
  ASSERT_NE(remote, 0);
  EXPECT_EXIT(
      {
        std::signal(SIGPIPE, SIG_DFL);
        RuntimeUdpPeer peer;
        if (peer.Open("127.0.0.1", 0, "127.0.0.1", remote) != UdpStatus::kOk ||
            ::shutdown(peer.NativeFd(), SHUT_WR) != 0) {
          std::_Exit(2);
        }
        const bool copied =
            peer.SendFrame(FrameType::kClockProbe, kFlagNone, 1, nullptr, 0) ==
                UdpStatus::kSocketError &&
            peer.stats().last_send_error == EPIPE;
        const bool buffered =
            peer.SendBufferedFrame(FrameType::kClockProbe, kFlagNone, 2, 0) ==
                UdpStatus::kSocketError &&
            peer.stats().last_send_error == EPIPE;
        std::_Exit(copied && buffered ? 0 : 3);
      },
      ::testing::ExitedWithCode(0), "");
}

// A peer whose socket was lost opens a new one on the same ports and numbers
// its frames on from the old sequence, so the gateway's replay gate
// (Gateway::Accept) takes the next command at once.
// REQ-RT-22
TEST(RuntimeUdpTest, ReopenKeepsThePortsAndTheSendSequence) {
  const uint16_t port = FreeLoopbackPort();
  const uint16_t remote = FreeLoopbackPort();
  ASSERT_NE(port, 0);
  ASSERT_NE(remote, 0);
  RuntimeUdpPeer mac;
  RuntimeUdpPeer rio;
  // The RoboRIO end is connected to the bridge's port: it receives only
  // from that port, so a frame it receives proves the reopened socket kept
  // it.
  ASSERT_EQ(rio.Open("127.0.0.1", remote, "127.0.0.1", port), UdpStatus::kOk);
  ASSERT_EQ(mac.Open("127.0.0.1", port, "127.0.0.1", remote), UdpStatus::kOk);

  DecodedFrame received;
  ASSERT_EQ(mac.SendFrame(FrameType::kClockProbe, kFlagNone, 1, nullptr, 0),
            UdpStatus::kOk);
  ASSERT_EQ(testing::ReceiveFrame(rio, &received), UdpStatus::kOk);
  EXPECT_EQ(received.header.sequence, 1u);

  ASSERT_EQ(::shutdown(mac.NativeFd(), SHUT_WR), 0);
  ASSERT_EQ(mac.SendFrame(FrameType::kClockProbe, kFlagNone, 2, nullptr, 0),
            UdpStatus::kSocketError);
  ASSERT_EQ(mac.Reopen(), UdpStatus::kOk);
  ASSERT_EQ(mac.SendFrame(FrameType::kClockProbe, kFlagNone, 3, nullptr, 0),
            UdpStatus::kOk);
  ASSERT_EQ(testing::ReceiveFrame(rio, &received), UdpStatus::kOk);
  EXPECT_EQ(received.header.monotonic_time_us, 3u);
  EXPECT_EQ(received.header.sequence, 2u);
}

// REQ-RT-22: sequence_gaps counts actual missed frames on in-order jumps,
// while out-of-order and duplicate frames increment out_of_order_frames
// without rolling last_received_sequence backwards (so the next in-order
// frame creates no phantom gap).
TEST(RuntimeUdpTest, CountsMissedSequenceGapsAndOutOfOrderWithoutRollback) {
  RuntimeUdpPeer receiver;
  UdpSocketOptions unconnected;
  unconnected.connect_peer = false;
  ASSERT_EQ(receiver.Open("127.0.0.1", 0, "127.0.0.1", 0, unconnected),
            UdpStatus::kOk);
  const uint16_t rx_port = BoundPort(receiver);
  ASSERT_NE(rx_port, 0);

  const int raw_sender = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(raw_sender, 0);
  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  dest.sin_port = htons(rx_port);

  const auto send_seq = [&](uint64_t seq) {
    std::array<uint8_t, kFrameHeaderSize> buf{};
    const std::size_t n =
        EncodeFrame(FrameType::kClockProbe, kFlagNone, seq, seq * 1000, nullptr,
                    0, buf.data(), buf.size());
    ASSERT_EQ(n, kFrameHeaderSize);
    ASSERT_EQ(::sendto(raw_sender, buf.data(), n, 0,
                       reinterpret_cast<const sockaddr*>(&dest), sizeof(dest)),
              static_cast<ssize_t>(n));
    DecodedFrame frame;
    ASSERT_EQ(testing::ReceiveFrame(receiver, &frame), UdpStatus::kOk);
    EXPECT_EQ(frame.header.sequence, seq);
  };

  send_seq(1);  // first frame
  EXPECT_EQ(receiver.stats().sequence_gaps, 0u);
  EXPECT_EQ(receiver.stats().last_received_sequence, 1u);

  send_seq(5);  // jumped 2, 3, 4 -> 3 missed frames
  EXPECT_EQ(receiver.stats().sequence_gaps, 3u);
  EXPECT_EQ(receiver.stats().out_of_order_frames, 0u);
  EXPECT_EQ(receiver.stats().last_received_sequence, 5u);

  send_seq(3);  // delayed out-of-order -> must not roll last_received back
  EXPECT_EQ(receiver.stats().sequence_gaps, 3u);
  EXPECT_EQ(receiver.stats().out_of_order_frames, 1u);
  EXPECT_EQ(receiver.stats().last_received_sequence, 5u);

  send_seq(5);  // duplicate -> out-of-order
  EXPECT_EQ(receiver.stats().sequence_gaps, 3u);
  EXPECT_EQ(receiver.stats().out_of_order_frames, 2u);
  EXPECT_EQ(receiver.stats().last_received_sequence, 5u);

  send_seq(6);  // contiguous after 5 -> 0 additional missed frames

  ::close(raw_sender);

  EXPECT_EQ(receiver.stats().sequence_gaps, 3u);
  EXPECT_EQ(receiver.stats().out_of_order_frames, 2u);
  EXPECT_EQ(receiver.stats().last_received_sequence, 6u);
  EXPECT_EQ(receiver.stats().received_frames, 5u);
  EXPECT_EQ(receiver.stats().send_would_block, 0u);
}

// REQ-RT-22: when a peer restarts and resets its send sequence to 1,
// RuntimeUdpPeer re-baselines last_received_sequence so later gaps are still
// counted (both after a long run and after a short run).
TEST(RuntimeUdpTest, PeerRestartRebaselinesSequenceGapCounting) {
  RuntimeUdpPeer receiver;
  UdpSocketOptions unconnected;
  unconnected.connect_peer = false;
  ASSERT_EQ(receiver.Open("127.0.0.1", 0, "127.0.0.1", 0, unconnected),
            UdpStatus::kOk);
  const uint16_t rx_port = BoundPort(receiver);
  ASSERT_NE(rx_port, 0);

  const int raw_sender = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(raw_sender, 0);
  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  dest.sin_port = htons(rx_port);

  const auto send_seq = [&](uint64_t seq) {
    std::array<uint8_t, kFrameHeaderSize> buf{};
    const std::size_t n =
        EncodeFrame(FrameType::kClockProbe, kFlagNone, seq, seq * 1000, nullptr,
                    0, buf.data(), buf.size());
    ASSERT_EQ(n, kFrameHeaderSize);
    ASSERT_EQ(::sendto(raw_sender, buf.data(), n, 0,
                       reinterpret_cast<const sockaddr*>(&dest), sizeof(dest)),
              static_cast<ssize_t>(n));
    DecodedFrame frame;
    ASSERT_EQ(testing::ReceiveFrame(receiver, &frame), UdpStatus::kOk);
  };

  // Long run restart: large backward jump re-baselines immediately.
  send_seq(120000);
  send_seq(1);
  EXPECT_EQ(receiver.stats().last_received_sequence, 1u);
  EXPECT_EQ(receiver.stats().sequence_gaps, 0u);
  send_seq(4);  // missed 2, 3 -> 2 missed frames
  EXPECT_EQ(receiver.stats().sequence_gaps, 2u);
  EXPECT_EQ(receiver.stats().last_received_sequence, 4u);

  // A 2-frame reorder inside the window must not re-baseline or add false gaps.
  send_seq(10);  // missed 5..9 -> +5 (total 7)
  EXPECT_EQ(receiver.stats().sequence_gaps, 7u);
  send_seq(8);
  send_seq(9);
  send_seq(11);
  EXPECT_EQ(receiver.stats().last_received_sequence, 11u);
  EXPECT_EQ(receiver.stats().sequence_gaps, 7u);

  // Short run restart (within reorder window): three ascending frames below
  // last re-baseline.
  send_seq(1);
  send_seq(2);
  send_seq(3);
  EXPECT_EQ(receiver.stats().last_received_sequence, 3u);
  send_seq(6);  // missed 4, 5 -> +2 (total 9)
  EXPECT_EQ(receiver.stats().sequence_gaps, 9u);

  ::close(raw_sender);
}

// REQ-RT-22: a non-EAGAIN recv() error records errno in last_recv_error.
TEST(RuntimeUdpTest, RecordsLastRecvErrorOnSocketError) {
  const uint16_t unbound = FreeLoopbackPort();
  ASSERT_NE(unbound, 0);
  RuntimeUdpPeer peer;
  ASSERT_EQ(peer.Open("127.0.0.1", 0, "127.0.0.1", unbound), UdpStatus::kOk);
  ASSERT_EQ(peer.SendFrame(FrameType::kClockProbe, kFlagNone, 1, nullptr, 0),
            UdpStatus::kOk);

  // Wait for ICMP port unreachable to surface on the connected UDP socket.
  pollfd pfd{peer.NativeFd(), POLLIN | POLLERR, 0};
  ASSERT_EQ(::poll(&pfd, 1, 5000), 1);

  DecodedFrame frame;
  EXPECT_EQ(peer.TryReceive(&frame), UdpStatus::kSocketError);
  EXPECT_EQ(peer.stats().last_recv_error, ECONNREFUSED);
}

// REQ-RT-22: permanent send errors trigger socket replacement, while transient
// errors such as ICMP port unreachable (ECONNREFUSED) keep it.
TEST(RuntimeUdpTest, SendErrorNeedsNewSocketClassifiesPermanentAndTransient) {
  for (const int permanent : {EPIPE, ENOTCONN, EDESTADDRREQ, EBADF, ENOTSOCK,
                              ENETUNREACH, EADDRNOTAVAIL}) {
    EXPECT_TRUE(SendErrorNeedsNewSocket(permanent)) << std::strerror(permanent);
  }
  for (const int transient : {0, ECONNREFUSED, EAGAIN, EWOULDBLOCK, EINTR,
                              ENOBUFS, ENETDOWN, EHOSTUNREACH}) {
    EXPECT_FALSE(SendErrorNeedsNewSocket(transient))
        << std::strerror(transient);
  }
}

// REQ-RT-22: permanent receive errors trigger socket replacement, while
// transient errors such as ICMP port unreachable (ECONNREFUSED) keep it.
TEST(RuntimeUdpTest,
     ReceiveErrorNeedsNewSocketClassifiesPermanentAndTransient) {
  for (const int permanent :
       {EBADF, ENOTCONN, ENOTSOCK, EPIPE, ENETUNREACH, EADDRNOTAVAIL}) {
    EXPECT_TRUE(ReceiveErrorNeedsNewSocket(permanent))
        << std::strerror(permanent);
  }
  for (const int transient : {0, ECONNREFUSED, EAGAIN, EWOULDBLOCK, EINTR,
                              ENOBUFS, ENETDOWN, EHOSTUNREACH}) {
    EXPECT_FALSE(ReceiveErrorNeedsNewSocket(transient))
        << std::strerror(transient);
  }
}

// REQ-RT-22: binding an IPv4 local_address that is not assigned to the host
// fails with EADDRNOTAVAIL rather than falling back to 0.0.0.0.
TEST(RuntimeUdpTest, OpenFailsCleanlyWhenLocalAddressIsAbsent) {
  const uint16_t remote = FreeLoopbackPort();
  ASSERT_NE(remote, 0);
  RuntimeUdpPeer peer;
  errno = 0;
  EXPECT_EQ(peer.Open("192.0.2.254", 0, "127.0.0.1", remote),
            UdpStatus::kSocketError);
  EXPECT_EQ(errno, EADDRNOTAVAIL);
  EXPECT_FALSE(peer.IsOpen());
  EXPECT_EQ(peer.NativeFd(), -1);
  EXPECT_EQ(peer.Reopen(), UdpStatus::kInvalidArgument);

  // A subsequent Open with an assigned address still succeeds cleanly.
  EXPECT_EQ(peer.Open("127.0.0.1", 0, "127.0.0.1", remote), UdpStatus::kOk);
  EXPECT_TRUE(peer.IsOpen());
}

// REQ-CONTRACT-01: a UDP datagram larger than kMaxFrameSize whose leading
// kMaxFrameSize bytes form a valid max-payload frame must be rejected as
// truncated/oversized rather than silently truncated to kMaxFrameSize and
// decoded as kOk.
TEST(RuntimeUdpTest, RejectsOversizedDatagramWithValidMaxPayloadPrefix) {
  RuntimeUdpPeer receiver;
  UdpSocketOptions unconnected;
  unconnected.connect_peer = false;
  ASSERT_EQ(receiver.Open("127.0.0.1", 0, "127.0.0.1", 0, unconnected),
            UdpStatus::kOk);
  const uint16_t rx_port = BoundPort(receiver);
  ASSERT_NE(rx_port, 0);

  const int raw_sender = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(raw_sender, 0);
  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  dest.sin_port = htons(rx_port);

  std::array<uint8_t, kMaxPayloadSize> payload{};
  payload.fill(0x5Au);
  std::array<uint8_t, kMaxFrameSize + 1> oversized{};
  ASSERT_EQ(
      EncodeFrame(FrameType::kHardwareState, kFlagNone, 1, 1000, payload.data(),
                  payload.size(), oversized.data(), kMaxFrameSize),
      kMaxFrameSize);
  oversized[kMaxFrameSize] = 0xAAu;

  ASSERT_EQ(::sendto(raw_sender, oversized.data(), oversized.size(), 0,
                     reinterpret_cast<const sockaddr*>(&dest), sizeof(dest)),
            static_cast<ssize_t>(oversized.size()));

  DecodedFrame frame;
  EXPECT_EQ(testing::ReceiveFrame(receiver, &frame), UdpStatus::kDecodeError);
  EXPECT_EQ(receiver.stats().last_decode_status, DecodeStatus::kTruncated);
  EXPECT_EQ(receiver.stats().decode_failures, 1u);
  EXPECT_EQ(receiver.stats().received_frames, 0u);

  // An exact kMaxFrameSize datagram still decodes cleanly.
  ASSERT_EQ(::sendto(raw_sender, oversized.data(), kMaxFrameSize, 0,
                     reinterpret_cast<const sockaddr*>(&dest), sizeof(dest)),
            static_cast<ssize_t>(kMaxFrameSize));
  ASSERT_EQ(testing::ReceiveFrame(receiver, &frame), UdpStatus::kOk);
  EXPECT_EQ(frame.header.sequence, 1u);
  EXPECT_EQ(frame.payload_size, kMaxPayloadSize);
  EXPECT_EQ(receiver.stats().received_frames, 1u);
  EXPECT_EQ(receiver.stats().received_bytes, kMaxFrameSize);

  ::close(raw_sender);
}

struct EintrTestState {
  volatile sig_atomic_t signal_count = 0;
  int sender_fd = -1;
  std::array<uint8_t, kFrameHeaderSize> frame{};
  std::size_t frame_size = 0;
};

EintrTestState g_eintr_state;

void OnSigAlrmForEintrTest(int /*sig*/) {
  const int saved_errno = errno;
  const sig_atomic_t count = g_eintr_state.signal_count + 1;
  g_eintr_state.signal_count = count;
  if (count >= 2 && g_eintr_state.sender_fd >= 0) {
    static_cast<void>(::send(g_eintr_state.sender_fd,
                             g_eintr_state.frame.data(),
                             g_eintr_state.frame_size, 0));
  }
  errno = saved_errno;
}

// REQ-RT-22: a signal interrupting a blocking POSIX recv() (EINTR) is retried
// rather than surfacing kSocketError with last_recv_error == EINTR.
TEST(RuntimeUdpTest, BlockingReceiveRetriesOnEintr) {
  RuntimeUdpPeer receiver;
  UdpSocketOptions blocking;
  blocking.non_blocking = false;
  blocking.connect_peer = false;
  ASSERT_EQ(receiver.Open("127.0.0.1", 0, "127.0.0.1", 0, blocking),
            UdpStatus::kOk);
  const uint16_t rx_port = BoundPort(receiver);
  ASSERT_NE(rx_port, 0);

  const int raw_sender = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(raw_sender, 0);
  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  dest.sin_port = htons(rx_port);
  ASSERT_EQ(::connect(raw_sender, reinterpret_cast<const sockaddr*>(&dest),
                      sizeof(dest)),
            0);

  g_eintr_state.signal_count = 0;
  g_eintr_state.sender_fd = raw_sender;
  g_eintr_state.frame_size =
      EncodeFrame(FrameType::kClockProbe, kFlagNone, 42, 9999, nullptr, 0,
                  g_eintr_state.frame.data(), g_eintr_state.frame.size());
  ASSERT_EQ(g_eintr_state.frame_size, kFrameHeaderSize);

  struct sigaction action {};
  action.sa_handler = OnSigAlrmForEintrTest;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;  // No SA_RESTART so blocking recv() returns EINTR.
  struct sigaction previous {};
  ASSERT_EQ(::sigaction(SIGALRM, &action, &previous), 0);

  struct ScopedCleanup {
    struct sigaction* previous_action;
    int fd;
    ~ScopedCleanup() {
      itimerval disarm{};
      static_cast<void>(::setitimer(ITIMER_REAL, &disarm, nullptr));
      static_cast<void>(::sigaction(SIGALRM, previous_action, nullptr));
      g_eintr_state.sender_fd = -1;
      ::close(fd);
    }
  } cleanup{&previous, raw_sender};

  itimerval timer{};
  timer.it_value.tv_usec = 20000;
  timer.it_interval.tv_usec = 20000;
  ASSERT_EQ(::setitimer(ITIMER_REAL, &timer, nullptr), 0);

  DecodedFrame frame;
  EXPECT_EQ(receiver.TryReceive(&frame), UdpStatus::kOk);
  EXPECT_EQ(frame.header.sequence, 42u);
  EXPECT_EQ(receiver.stats().last_recv_error, 0);
  EXPECT_GE(g_eintr_state.signal_count, 2);
}

}  // namespace
}  // namespace talos::protocol
