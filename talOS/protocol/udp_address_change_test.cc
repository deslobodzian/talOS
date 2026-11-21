// A connected socket whose source address left the host, on the real Linux
// network stack: the error its sends fail with, and that a new socket on the
// same ports sends again.
#include <gtest/gtest.h>

#include "udp.h"

#if defined(__linux__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/if_tun.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#endif

namespace talos::protocol {
namespace {

#if defined(__linux__)
// The child's verdict, written to the parent through a pipe.
struct Verdict {
  int fd;
  [[noreturn]] void Say(const char* format, ...) const
      __attribute__((format(printf, 2, 3))) {
    std::array<char, 256> text{};
    va_list args;
    va_start(args, format);
    const int size = std::vsnprintf(text.data(), text.size(), format, args);
    va_end(args);
    if (size > 0) {
      (void)::write(fd, text.data(),
                    std::min<std::size_t>(static_cast<std::size_t>(size),
                                          text.size() - 1));
    }
    ::_exit(0);
  }
};

// Sets the IPv4 address of interface `name`, replacing the one it had, as a
// new DHCP lease does. Passing "0.0.0.0" removes the interface's IPv4 address.
bool SetAddress(int control, const char* name, const char* address) {
  ifreq request{};
  std::snprintf(request.ifr_name, sizeof(request.ifr_name), "%s", name);
  sockaddr_in in{};
  in.sin_family = AF_INET;
  if (::inet_pton(AF_INET, address, &in.sin_addr) != 1) return false;
  std::memcpy(&request.ifr_addr, &in, sizeof(in));
  return ::ioctl(control, SIOCSIFADDR, &request) == 0;
}

bool SetNetmask(int control, const char* name, const char* mask) {
  ifreq request{};
  std::snprintf(request.ifr_name, sizeof(request.ifr_name), "%s", name);
  sockaddr_in in{};
  in.sin_family = AF_INET;
  if (::inet_pton(AF_INET, mask, &in.sin_addr) != 1) return false;
  std::memcpy(&request.ifr_netmask, &in, sizeof(in));
  return ::ioctl(control, SIOCSIFNETMASK, &request) == 0;
}

bool SetUp(int control, const char* name, short flags) {
  ifreq request{};
  std::snprintf(request.ifr_name, sizeof(request.ifr_name), "%s", name);
  if (::ioctl(control, SIOCGIFFLAGS, &request) != 0) return false;
  request.ifr_flags = static_cast<short>(request.ifr_flags | flags);
  return ::ioctl(control, SIOCSIFFLAGS, &request) == 0;
}

int OpenTap(const char* name, const Verdict& verdict) {
  const int tap = ::open("/dev/net/tun", O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (tap < 0) verdict.Say("skip: cannot open /dev/net/tun: errno %d", errno);
  ifreq request{};
  std::snprintf(request.ifr_name, sizeof(request.ifr_name), "%s", name);
  request.ifr_flags = IFF_TAP | IFF_NO_PI;
  if (::ioctl(tap, TUNSETIFF, &request) != 0) {
    verdict.Say("skip: cannot create tap %s: errno %d", name, errno);
  }
  return tap;
}

uint32_t ExtractUdpSourceToPort(const uint8_t* packet, ssize_t size,
                                uint16_t port) {
  // Ethernet (14), IPv4 (ihl), then the UDP destination port.
  if (size < 14 + 20 + 8 || packet[12] != 0x08 || packet[13] != 0x00) {
    return 0;
  }
  const std::size_t ip = 14;
  const std::size_t header = std::size_t{4} * (packet[ip] & 0x0fU);
  if (packet[ip + 9] != IPPROTO_UDP ||
      static_cast<std::size_t>(size) < ip + header + 8) {
    return 0;
  }
  const uint16_t destination = static_cast<uint16_t>(
      (packet[ip + header + 2] << 8) | packet[ip + header + 3]);
  if (destination != port) return 0;
  uint32_t source = 0;
  std::memcpy(&source, &packet[ip + 12], sizeof(source));
  return source;
}

// The IPv4 source address of the next UDP datagram to `port` carried on any
// of `taps`; 0 if none arrives within 10 s (a bound on a hang, not the
// assertion: the frame is queued to the tap during the send).
uint32_t NextSourceToAny(const int* taps, std::size_t tap_count,
                         uint16_t port) {
  std::array<uint8_t, 2048> packet{};
  std::array<pollfd, 2> ready{};
  for (int polls = 0; polls < 1000; ++polls) {
    for (std::size_t i = 0; i < tap_count; ++i) {
      ready[i] = pollfd{.fd = taps[i], .events = POLLIN, .revents = 0};
    }
    const int n = ::poll(ready.data(), static_cast<nfds_t>(tap_count), 10);
    if (n < 0) return 0;
    if (n == 0) continue;
    for (std::size_t i = 0; i < tap_count; ++i) {
      if ((ready[i].revents & POLLIN) == 0) continue;
      for (;;) {
        const ssize_t size = ::read(taps[i], packet.data(), packet.size());
        if (size < 0) break;
        const uint32_t source =
            ExtractUdpSourceToPort(packet.data(), size, port);
        if (source != 0) return source;
      }
    }
  }
  return 0;
}

uint32_t NextSourceTo(int tap, uint16_t port) {
  return NextSourceToAny(&tap, 1, port);
}

uint32_t Address(const char* text) {
  in_addr address{};
  ::inet_pton(AF_INET, text, &address);
  return address.s_addr;
}

// In a private user and network namespace: an Ethernet tap device holds the
// host's address, 10.9.9.2/8, and the peer (the RoboRIO's gateway socket, or
// the bridge's) is bound to 0.0.0.0:5802 and connected to 10.9.9.7:5803, as
// on the robot. The address changes to 10.9.9.3 (a new DHCP lease after a
// radio swap, or a link flap that re-addresses). connect() fixed the
// socket's source at 10.9.9.2, which is no longer local, so on Linux every
// later send on it fails, though 10.9.9.7 is reachable from the new address.
void AddressChangeScenario(const Verdict& verdict) {
  if (::unshare(CLONE_NEWUSER | CLONE_NEWNET) != 0) {
    verdict.Say("skip: cannot create a network namespace: errno %d", errno);
  }
  const int tap = OpenTap("tap0", verdict);
  const int control = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  // No ARP: a frame to 10.9.9.7 leaves at once, for the tap to read.
  if (control < 0 || !SetAddress(control, "tap0", "10.9.9.2") ||
      !SetUp(control, "tap0", IFF_UP | IFF_NOARP)) {
    verdict.Say("cannot address the tap device: errno %d", errno);
  }

  // Fixed ports: the namespace is this child's alone.
  RuntimeUdpPeer peer;
  if (peer.Open("0.0.0.0", 5802, "10.9.9.7", 5803) != UdpStatus::kOk) {
    verdict.Say("cannot open the peer: errno %d", errno);
  }
  if (peer.SendFrame(FrameType::kClockProbe, kFlagNone, 1, nullptr, 0) !=
          UdpStatus::kOk ||
      NextSourceTo(tap, 5803) != Address("10.9.9.2")) {
    verdict.Say("the first frame did not leave from 10.9.9.2");
  }

  if (!SetAddress(control, "tap0", "10.9.9.3")) {
    verdict.Say("cannot change the address: errno %d", errno);
  }
  if (peer.SendFrame(FrameType::kClockProbe, kFlagNone, 2, nullptr, 0) !=
      UdpStatus::kSocketError) {
    verdict.Say("a send from the old source address did not fail");
  }
  const int error = peer.stats().last_send_error;
  if (!SendErrorNeedsNewSocket(error)) {
    verdict.Say(
        "errno %d (%s) after the address change is classed as clearing on "
        "the same socket",
        error, std::strerror(error));
  }
  // The same socket keeps failing: it never picks up the new address.
  if (peer.SendFrame(FrameType::kClockProbe, kFlagNone, 3, nullptr, 0) !=
      UdpStatus::kSocketError) {
    verdict.Say("the old socket sent again");
  }

  if (peer.Reopen() != UdpStatus::kOk) {
    verdict.Say("reopen failed: errno %d", errno);
  }
  if (peer.SendFrame(FrameType::kClockProbe, kFlagNone, 4, nullptr, 0) !=
      UdpStatus::kOk) {
    verdict.Say("the new socket cannot send: errno %d",
                peer.stats().last_send_error);
  }
  if (NextSourceTo(tap, 5803) != Address("10.9.9.3")) {
    verdict.Say("the new socket's frame did not leave from 10.9.9.3");
  }
  verdict.Say("ok");
}

// Two interfaces on the same /24 subnet: tap0 holds the robot-network
// address (10.9.9.2/24) and tap1 simulates a second interface routing the
// peer (10.9.9.99/24). When 10.9.9.2 leaves tap0 while tap1 still routes
// 10.9.9.7, a wildcard 0.0.0.0 peer binds and sends from 10.9.9.99 (and keeps
// sending from 10.9.9.99 even after 10.9.9.2 returns), whereas a peer bound
// to 10.9.9.2 fails Open()/Reopen() with EADDRNOTAVAIL while 10.9.9.2 is
// absent and, once 10.9.9.2 returns on tap0, succeeds and sends exclusively
// from 10.9.9.2.
void TwoInterfaceLocalAddressScenario(const Verdict& verdict) {
  if (::unshare(CLONE_NEWUSER | CLONE_NEWNET) != 0) {
    verdict.Say("skip: cannot create a network namespace: errno %d", errno);
  }
  const int tap0 = OpenTap("tap0", verdict);
  const int tap1 = OpenTap("tap1", verdict);
  const int taps[2] = {tap0, tap1};
  const int control = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (control < 0 || !SetAddress(control, "tap0", "10.9.9.2") ||
      !SetNetmask(control, "tap0", "255.255.255.0") ||
      !SetUp(control, "tap0", IFF_UP | IFF_NOARP) ||
      !SetAddress(control, "tap1", "10.9.9.99") ||
      !SetNetmask(control, "tap1", "255.255.255.0") ||
      !SetUp(control, "tap1", IFF_UP | IFF_NOARP)) {
    verdict.Say("cannot configure tap0/tap1: errno %d", errno);
  }

  // Bound peer starts while 10.9.9.2 is present and sends from 10.9.9.2.
  RuntimeUdpPeer bound;
  if (bound.Open("10.9.9.2", 5803, "10.9.9.7", 5802) != UdpStatus::kOk) {
    verdict.Say("cannot open bound peer: errno %d", errno);
  }
  if (bound.SendFrame(FrameType::kClockProbe, kFlagNone, 1, nullptr, 0) !=
          UdpStatus::kOk ||
      NextSourceToAny(taps, 2, 5802) != Address("10.9.9.2")) {
    verdict.Say("initial bound frame did not leave from 10.9.9.2");
  }

  // Remove 10.9.9.2 from tap0 while tap1 (10.9.9.99/24) still routes 10.9.9.7.
  if (!SetAddress(control, "tap0", "0.0.0.0")) {
    verdict.Say("cannot remove tap0 address: errno %d", errno);
  }

  // Wildcard peer opened while 10.9.9.2 is absent takes tap1's 10.9.9.99.
  RuntimeUdpPeer wildcard;
  if (wildcard.Open("0.0.0.0", 5804, "10.9.9.7", 5802) != UdpStatus::kOk) {
    verdict.Say("wildcard peer did not open over tap1: errno %d", errno);
  }
  if (wildcard.SendFrame(FrameType::kClockProbe, kFlagNone, 2, nullptr, 0) !=
          UdpStatus::kOk ||
      NextSourceToAny(taps, 2, 5802) != Address("10.9.9.99")) {
    verdict.Say("wildcard peer did not send from wrong source 10.9.9.99");
  }

  // Previously bound peer detects its lost source and fails Reopen() while
  // 10.9.9.2 is absent; a fresh bound Open() also fails with EADDRNOTAVAIL.
  if (bound.SendFrame(FrameType::kClockProbe, kFlagNone, 3, nullptr, 0) !=
      UdpStatus::kSocketError) {
    verdict.Say("bound peer did not fail send after 10.9.9.2 left");
  }
  if (!SendErrorNeedsNewSocket(bound.stats().last_send_error)) {
    verdict.Say("bound peer send error %d not classed as needing new socket",
                bound.stats().last_send_error);
  }
  errno = 0;
  if (bound.Reopen() != UdpStatus::kSocketError || errno != EADDRNOTAVAIL) {
    verdict.Say(
        "bound Reopen() while 10.9.9.2 absent did not fail "
        "EADDRNOTAVAIL (errno %d)",
        errno);
  }
  RuntimeUdpPeer cold_bound;
  errno = 0;
  if (cold_bound.Open("10.9.9.2", 5805, "10.9.9.7", 5802) !=
          UdpStatus::kSocketError ||
      errno != EADDRNOTAVAIL) {
    verdict.Say(
        "cold bound Open() while 10.9.9.2 absent did not fail "
        "EADDRNOTAVAIL (errno %d)",
        errno);
  }

  // Restore 10.9.9.2/24 on tap0 while tap1 remains up on the same subnet.
  if (!SetAddress(control, "tap0", "10.9.9.2") ||
      !SetNetmask(control, "tap0", "255.255.255.0") ||
      !SetUp(control, "tap0", IFF_UP | IFF_NOARP)) {
    verdict.Say("cannot restore tap0 address: errno %d", errno);
  }

  // Wildcard peer is stuck on 10.9.9.99 and never recovers on its own.
  if (wildcard.SendFrame(FrameType::kClockProbe, kFlagNone, 4, nullptr, 0) !=
          UdpStatus::kOk ||
      NextSourceToAny(taps, 2, 5802) != Address("10.9.9.99")) {
    verdict.Say("wildcard peer did not stay pinned to wrong source 10.9.9.99");
  }

  // Bound peers now succeed and send exclusively from 10.9.9.2, with Reopen()
  // preserving the send sequence.
  if (bound.Reopen() != UdpStatus::kOk) {
    verdict.Say("bound Reopen() failed after 10.9.9.2 returned: errno %d",
                errno);
  }
  if (bound.next_send_sequence() != 2u) {
    verdict.Say("bound Reopen() did not preserve send sequence");
  }
  if (bound.SendFrame(FrameType::kClockProbe, kFlagNone, 5, nullptr, 0) !=
          UdpStatus::kOk ||
      NextSourceToAny(taps, 2, 5802) != Address("10.9.9.2")) {
    verdict.Say("reopened bound peer did not send from 10.9.9.2");
  }
  if (cold_bound.Open("10.9.9.2", 5805, "10.9.9.7", 5802) != UdpStatus::kOk) {
    verdict.Say("cold bound Open() failed after 10.9.9.2 returned: errno %d",
                errno);
  }
  if (cold_bound.SendFrame(FrameType::kClockProbe, kFlagNone, 6, nullptr, 0) !=
          UdpStatus::kOk ||
      NextSourceToAny(taps, 2, 5802) != Address("10.9.9.2")) {
    verdict.Say("cold bound peer did not send from 10.9.9.2");
  }
  verdict.Say("ok");
}

void RunNetnsScenario(void (*scenario)(const Verdict&)) {
  std::array<int, 2> pipe{};
  ASSERT_EQ(::pipe2(pipe.data(), O_CLOEXEC), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(pipe[0]);
    scenario(Verdict{pipe[1]});
  }
  ::close(pipe[1]);
  std::string text;
  std::array<char, 256> buffer{};
  for (;;) {
    const ssize_t size = ::read(pipe[0], buffer.data(), buffer.size());
    if (size > 0) {
      text.append(buffer.data(), static_cast<std::size_t>(size));
    } else if (size == 0 || errno != EINTR) {
      break;
    }
  }
  ::close(pipe[0]);
  int status = 0;
  while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {
  }
  ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "the scenario's child died: status " << status;
  if (text.starts_with("skip: ")) {
    // CI's Linux job sets this after allowing unprivileged user namespaces,
    // so a host that still refuses them fails instead of passing unproven.
    const char* require = std::getenv("TALOS_REQUIRE_NETNS");
    if (require != nullptr && std::string_view{require} == "1") {
      FAIL() << "TALOS_REQUIRE_NETNS=1, but " << text;
    }
    GTEST_SKIP() << text;
  }
  EXPECT_EQ(text, "ok");
}
#endif

// REQ-RT-22
TEST(UdpAddressChangeTest, ASocketWhoseSourceAddressLeftIsReplaced) {
#if !defined(__linux__)
  GTEST_SKIP() << "network namespaces are Linux-only; macOS reports the same "
                  "case as EADDRNOTAVAIL";
#else
  RunNetnsScenario(AddressChangeScenario);
#endif
}

// REQ-RT-22
TEST(UdpAddressChangeTest, BoundLocalAddressSurvivesSecondInterfaceOutage) {
#if !defined(__linux__)
  GTEST_SKIP() << "network namespaces are Linux-only";
#else
  RunNetnsScenario(TwoInterfaceLocalAddressScenario);
#endif
}

}  // namespace
}  // namespace talos::protocol
