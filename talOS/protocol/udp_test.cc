#include "udp.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
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

// Both send paths (copying and in-buffer) reach the peer intact and number
// their frames from one sequence.
TEST(RuntimeUdpTest, SendsFramesOverLoopback) {
  RuntimeUdpPeer mac;
  RuntimeUdpPeer rio;
  UdpSocketOptions options;
  options.non_blocking = false;

  ASSERT_EQ(rio.Open("127.0.0.1", 59101, "127.0.0.1", 59100, options),
            UdpStatus::kOk);
  ASSERT_EQ(mac.Open("127.0.0.1", 59100, "127.0.0.1", 59101, options),
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

TEST(RuntimeUdpTest, NonBlockingReceiveReportsWouldBlock) {
  RuntimeUdpPeer peer;
  ASSERT_EQ(peer.Open("127.0.0.1", 59102, "127.0.0.1", 59103), UdpStatus::kOk);

  DecodedFrame received;
  EXPECT_EQ(peer.TryReceive(&received), UdpStatus::kWouldBlock);
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

}  // namespace
}  // namespace talos::protocol
