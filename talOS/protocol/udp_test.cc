#include "udp.h"

#include <arpa/inet.h>
#include <fcntl.h>
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
#include <utility>

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

// REQ-RT-22, REQ-OPS-20: three or more ascending reordered frames during a long
// run (seq > kReorderWindow) must not trigger short_run_restart, roll
// last_received_sequence backwards, or invent phantom sequence gaps.
TEST(RuntimeUdpTest, LongRunAscendingReorderDoesNotTriggerShortRunRestart) {
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

  send_seq(2000);
  send_seq(2005);  // jumped 2001..2004 -> 4 missed frames
  EXPECT_EQ(receiver.stats().last_received_sequence, 2005u);
  EXPECT_EQ(receiver.stats().sequence_gaps, 4u);
  EXPECT_EQ(receiver.stats().out_of_order_frames, 0u);

  // Delayed frames arriving in ascending order must stay out-of-order without
  // rolling last_received_sequence back from 2005.
  send_seq(2001);
  send_seq(2002);
  send_seq(2003);
  EXPECT_EQ(receiver.stats().last_received_sequence, 2005u);
  EXPECT_EQ(receiver.stats().sequence_gaps, 4u);
  EXPECT_EQ(receiver.stats().out_of_order_frames, 3u);

  send_seq(2004);
  EXPECT_EQ(receiver.stats().last_received_sequence, 2005u);
  EXPECT_EQ(receiver.stats().sequence_gaps, 4u);
  EXPECT_EQ(receiver.stats().out_of_order_frames, 4u);

  send_seq(2006);  // contiguous after 2005 -> no phantom gap
  EXPECT_EQ(receiver.stats().last_received_sequence, 2006u);
  EXPECT_EQ(receiver.stats().sequence_gaps, 4u);
  EXPECT_EQ(receiver.stats().out_of_order_frames, 4u);
  EXPECT_EQ(receiver.stats().received_frames, 7u);

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

// REQ-OPS-27: a connected RuntimeUdpPeer accepts frames only from the peer
// address/port it connected to; datagrams from a third, unconnected socket are
// dropped by the kernel (TryReceive reports kWouldBlock) while frames from its
// connected peer arrive intact.
TEST(RuntimeUdpTest, ConnectedPeerDropsFramesFromUnconnectedSender) {
  const int authorized = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(authorized, 0);
  struct ScopedFd {
    int fd;
    ~ScopedFd() {
      if (fd >= 0) ::close(fd);
    }
  } close_authorized{authorized};

  sockaddr_in auth_addr{};
  auth_addr.sin_family = AF_INET;
  auth_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  auth_addr.sin_port = 0;
  ASSERT_EQ(::bind(authorized, reinterpret_cast<const sockaddr*>(&auth_addr),
                   sizeof(auth_addr)),
            0);
  socklen_t auth_len = sizeof(auth_addr);
  ASSERT_EQ(::getsockname(authorized, reinterpret_cast<sockaddr*>(&auth_addr),
                          &auth_len),
            0);
  const uint16_t auth_port = ntohs(auth_addr.sin_port);
  ASSERT_NE(auth_port, 0);

  RuntimeUdpPeer receiver;
  ASSERT_EQ(receiver.Open("127.0.0.1", 0, "127.0.0.1", auth_port),
            UdpStatus::kOk);
  const uint16_t rx_port = BoundPort(receiver);
  ASSERT_NE(rx_port, 0);

  const int rogue = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(rogue, 0);
  ScopedFd close_rogue{rogue};
  sockaddr_in rogue_addr{};
  rogue_addr.sin_family = AF_INET;
  rogue_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  rogue_addr.sin_port = 0;
  ASSERT_EQ(::bind(rogue, reinterpret_cast<const sockaddr*>(&rogue_addr),
                   sizeof(rogue_addr)),
            0);
  socklen_t rogue_len = sizeof(rogue_addr);
  ASSERT_EQ(::getsockname(rogue, reinterpret_cast<sockaddr*>(&rogue_addr),
                          &rogue_len),
            0);
  ASSERT_NE(ntohs(rogue_addr.sin_port), auth_port);

  sockaddr_in rx_addr{};
  rx_addr.sin_family = AF_INET;
  rx_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  rx_addr.sin_port = htons(rx_port);

  const std::array<uint8_t, 4> rogue_payload = {0xDE, 0xAD, 0xBE, 0xEF};
  const auto send_rogue = [&](uint64_t seq, uint64_t time_us) {
    std::array<uint8_t, kFrameHeaderSize + rogue_payload.size()> wire{};
    const std::size_t n = EncodeFrame(
        FrameType::kHardwareCommand, kFlagNone, seq, time_us,
        rogue_payload.data(), rogue_payload.size(), wire.data(), wire.size());
    ASSERT_EQ(n, wire.size());
    ASSERT_EQ(
        ::sendto(rogue, wire.data(), n, 0,
                 reinterpret_cast<const sockaddr*>(&rx_addr), sizeof(rx_addr)),
        static_cast<ssize_t>(n));
  };
  send_rogue(1, 111);

  DecodedFrame received;
  EXPECT_EQ(receiver.TryReceive(&received), UdpStatus::kWouldBlock);
  EXPECT_EQ(receiver.stats().received_frames, 0u);

  // Send another rogue frame immediately followed by one from the connected
  // peer: only the connected peer's frame is delivered.
  send_rogue(2, 222);

  const std::array<uint8_t, 4> peer_payload = {1, 2, 3, 4};
  std::array<uint8_t, kFrameHeaderSize + peer_payload.size()> wire{};
  const std::size_t wire_size = EncodeFrame(
      FrameType::kHardwareCommand, kFlagNone, 7, 333, peer_payload.data(),
      peer_payload.size(), wire.data(), wire.size());
  ASSERT_EQ(wire_size, wire.size());
  ASSERT_EQ(
      ::sendto(authorized, wire.data(), wire_size, 0,
               reinterpret_cast<const sockaddr*>(&rx_addr), sizeof(rx_addr)),
      static_cast<ssize_t>(wire_size));

  ASSERT_EQ(testing::ReceiveFrame(receiver, &received), UdpStatus::kOk);
  EXPECT_EQ(received.header.sequence, 7u);
  EXPECT_EQ(received.header.monotonic_time_us, 333u);
  ASSERT_EQ(received.payload_size, peer_payload.size());
  EXPECT_EQ(
      std::memcmp(received.payload, peer_payload.data(), peer_payload.size()),
      0);
  EXPECT_EQ(receiver.TryReceive(&received), UdpStatus::kWouldBlock);
  EXPECT_EQ(receiver.stats().received_frames, 1u);
}

// REQ-RT-22: opened and reopened UDP sockets carry FD_CLOEXEC so exec'd child
// processes never inherit port 5802/5803 and block rebind across restart.
TEST(RuntimeUdpTest, OpenedSocketIsCloseOnExec) {
  const uint16_t remote = FreeLoopbackPort();
  ASSERT_NE(remote, 0);
  RuntimeUdpPeer peer;
  ASSERT_EQ(peer.Open("127.0.0.1", 0, "127.0.0.1", remote), UdpStatus::kOk);
  const int flags = ::fcntl(peer.NativeFd(), F_GETFD);
  ASSERT_GE(flags, 0);
  EXPECT_NE(flags & FD_CLOEXEC, 0);

  ASSERT_EQ(peer.Reopen(), UdpStatus::kOk);
  const int reopened_flags = ::fcntl(peer.NativeFd(), F_GETFD);
  ASSERT_GE(reopened_flags, 0);
  EXPECT_NE(reopened_flags & FD_CLOEXEC, 0);
}

// REQ-RT-22: moving a RuntimeUdpPeer clears opened_ on the moved-from instance
// so Reopen() on it cannot steal the moved-to peer's ports.
TEST(RuntimeUdpTest, MovedFromPeerCannotReopen) {
  const uint16_t port = FreeLoopbackPort();
  const uint16_t remote = FreeLoopbackPort();
  ASSERT_NE(port, 0);
  ASSERT_NE(remote, 0);

  RuntimeUdpPeer first;
  ASSERT_EQ(first.Open("127.0.0.1", port, "127.0.0.1", remote), UdpStatus::kOk);
  ASSERT_EQ(first.SendFrame(FrameType::kClockProbe, kFlagNone, 1, nullptr, 0),
            UdpStatus::kOk);
  ASSERT_EQ(first.next_send_sequence(), 2u);

  RuntimeUdpPeer second{std::move(first)};
  // NOLINTBEGIN(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
  EXPECT_FALSE(first.IsOpen());
  EXPECT_EQ(first.NativeFd(), -1);
  EXPECT_EQ(first.Reopen(), UdpStatus::kInvalidArgument);
  // NOLINTEND(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
  EXPECT_TRUE(second.IsOpen());
  EXPECT_EQ(second.next_send_sequence(), 2u);
  EXPECT_EQ(second.Reopen(), UdpStatus::kOk);
  EXPECT_EQ(second.next_send_sequence(), 2u);

  RuntimeUdpPeer third;
  third = std::move(second);
  // NOLINTBEGIN(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
  EXPECT_FALSE(second.IsOpen());
  EXPECT_EQ(second.NativeFd(), -1);
  EXPECT_EQ(second.Reopen(), UdpStatus::kInvalidArgument);
  // NOLINTEND(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
  EXPECT_TRUE(third.IsOpen());
  EXPECT_EQ(third.next_send_sequence(), 2u);
  EXPECT_EQ(third.Reopen(), UdpStatus::kOk);
  EXPECT_EQ(third.next_send_sequence(), 2u);
}

// REQ-CONTRACT-01: Open() rejects remote_port == 0 when connect_peer is true,
// while still allowing unconnected receivers to bind without a peer port.
TEST(RuntimeUdpTest, OpenRejectsZeroRemotePortWhenConnected) {
  RuntimeUdpPeer peer;
  EXPECT_EQ(peer.Open("127.0.0.1", 0, "127.0.0.1", 0),
            UdpStatus::kInvalidArgument);
  EXPECT_FALSE(peer.IsOpen());
  EXPECT_EQ(peer.Reopen(), UdpStatus::kInvalidArgument);

  UdpSocketOptions unconnected;
  unconnected.connect_peer = false;
  EXPECT_EQ(peer.Open("127.0.0.1", 0, "127.0.0.1", 0, unconnected),
            UdpStatus::kOk);
  EXPECT_TRUE(peer.IsOpen());
}

// REQ-CONTRACT-01: MutablePayloadBuffer and SendBufferedFrame reject oversize
// payloads and closed sockets without advancing next_send_sequence().
TEST(RuntimeUdpTest, BufferedSendRejectsInvalidAndOversizePayloads) {
  RuntimeUdpPeer peer;
  EXPECT_NE(peer.MutablePayloadBuffer(0), nullptr);
  EXPECT_NE(peer.MutablePayloadBuffer(kMaxPayloadSize), nullptr);
  EXPECT_EQ(peer.MutablePayloadBuffer(kMaxPayloadSize + 1), nullptr);
  EXPECT_EQ(peer.SendBufferedFrame(FrameType::kHardwareState, kFlagNone, 1, 0),
            UdpStatus::kSocketError);

  const uint16_t remote = FreeLoopbackPort();
  ASSERT_NE(remote, 0);
  ASSERT_EQ(peer.Open("127.0.0.1", 0, "127.0.0.1", remote), UdpStatus::kOk);
  EXPECT_EQ(peer.next_send_sequence(), 1u);
  EXPECT_EQ(peer.SendBufferedFrame(FrameType::kHardwareState, kFlagNone, 1,
                                   kMaxPayloadSize + 1),
            UdpStatus::kInvalidArgument);
  EXPECT_EQ(peer.next_send_sequence(), 1u);
  EXPECT_EQ(peer.stats().sent_frames, 0u);
}

// REQ-CONTRACT-01, REQ-RT-22: TryReceive records last_unsupported_version and
// decode_failures when a frame carries an unsupported protocol version.
TEST(RuntimeUdpTest, TryReceiveRecordsLastUnsupportedVersion) {
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

  uint64_t expected_failures = 0;
  for (const uint16_t bad_version :
       {static_cast<uint16_t>(kProtocolVersion + 1),
        static_cast<uint16_t>(kProtocolVersion - 1)}) {
    SCOPED_TRACE(bad_version);
    std::array<uint8_t, kFrameHeaderSize> wire{};
    ASSERT_EQ(EncodeFrame(FrameType::kClockProbe, kFlagNone, 1, 100, nullptr, 0,
                          wire.data(), wire.size()),
              kFrameHeaderSize);
    wire[8] = static_cast<uint8_t>(bad_version & 0xFF);
    wire[9] = static_cast<uint8_t>(bad_version >> 8);
    ASSERT_EQ(::sendto(raw_sender, wire.data(), wire.size(), 0,
                       reinterpret_cast<const sockaddr*>(&dest), sizeof(dest)),
              static_cast<ssize_t>(wire.size()));

    DecodedFrame decoded;
    EXPECT_EQ(testing::ReceiveFrame(receiver, &decoded),
              UdpStatus::kDecodeError);
    ++expected_failures;
    EXPECT_EQ(receiver.stats().last_decode_status,
              DecodeStatus::kUnsupportedVersion);
    EXPECT_EQ(receiver.stats().last_unsupported_version, bad_version);
    EXPECT_EQ(receiver.stats().decode_failures, expected_failures);
    EXPECT_EQ(receiver.stats().received_frames, 0u);
  }
  ::close(raw_sender);
}

// REQ-RT-22: a non-blocking send that hits EAGAIN/EWOULDBLOCK returns
// kWouldBlock, increments send_would_block, keeps next_send_sequence()
// unchanged, and leaves last_send_error at 0.
TEST(RuntimeUdpTest, NonBlockingSendWouldBlockPreservesSequenceAndSendError) {
  const uint16_t remote = FreeLoopbackPort();
  ASSERT_NE(remote, 0);
  RuntimeUdpPeer peer;
  ASSERT_EQ(peer.Open("127.0.0.1", 0, "127.0.0.1", remote), UdpStatus::kOk);

  // Swap a filled non-blocking AF_UNIX SOCK_STREAM socketpair onto peer's fd
  // so ::send() deterministically returns EAGAIN/EWOULDBLOCK on both Linux and
  // macOS without depending on UDP loopback queuing quirks.
  int sv[2] = {-1, -1};
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sv), 0);
  struct ClosePair {
    int* fds;
    ~ClosePair() {
      if (fds[0] >= 0) ::close(fds[0]);
      if (fds[1] >= 0) ::close(fds[1]);
    }
  } close_pair{sv};

  for (const int fd : sv) {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ASSERT_GE(flags, 0);
    ASSERT_EQ(::fcntl(fd, F_SETFL, flags | O_NONBLOCK), 0);
  }
  const int small_buf = 4096;
  static_cast<void>(
      ::setsockopt(sv[0], SOL_SOCKET, SO_SNDBUF, &small_buf, sizeof small_buf));
  static_cast<void>(
      ::setsockopt(sv[1], SOL_SOCKET, SO_RCVBUF, &small_buf, sizeof small_buf));

  std::array<uint8_t, 4096> filler{};
  bool filled = false;
  for (int i = 0; i < 4096; ++i) {
    const ssize_t n = ::send(sv[0], filler.data(), filler.size(), 0);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      filled = true;
      break;
    }
    ASSERT_GT(n, 0);
  }
  ASSERT_TRUE(filled);
  ASSERT_EQ(::dup2(sv[0], peer.NativeFd()), peer.NativeFd());

  EXPECT_EQ(peer.SendFrame(FrameType::kClockProbe, kFlagNone, 1, nullptr, 0),
            UdpStatus::kWouldBlock);
  EXPECT_EQ(peer.stats().send_would_block, 1u);
  EXPECT_EQ(peer.next_send_sequence(), 1u);
  EXPECT_EQ(peer.stats().last_send_error, 0);
  EXPECT_EQ(peer.stats().sent_frames, 0u);

  EXPECT_EQ(peer.SendBufferedFrame(FrameType::kClockProbe, kFlagNone, 2, 0),
            UdpStatus::kWouldBlock);
  EXPECT_EQ(peer.stats().send_would_block, 2u);
  EXPECT_EQ(peer.next_send_sequence(), 1u);
  EXPECT_EQ(peer.stats().last_send_error, 0);
  EXPECT_EQ(peer.stats().sent_frames, 0u);

  // Draining the peer end restores writability and sends sequence 1 cleanly.
  std::array<uint8_t, 4096> drain{};
  while (::recv(sv[1], drain.data(), drain.size(), 0) > 0) {
  }
  EXPECT_EQ(peer.SendFrame(FrameType::kClockProbe, kFlagNone, 3, nullptr, 0),
            UdpStatus::kOk);
  EXPECT_EQ(peer.next_send_sequence(), 2u);
  EXPECT_EQ(peer.stats().sent_frames, 1u);
  EXPECT_EQ(peer.stats().send_would_block, 2u);
  EXPECT_EQ(peer.stats().last_send_error, 0);
}

}  // namespace
}  // namespace talos::protocol
