#include "talOS/rtms/wake.h"

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <random>
#include <string_view>
#include <system_error>
#include <utility>

namespace talos {
namespace {

// A receiver needs room for one pending datagram: further notifications
// while one is pending carry no information. The kernel rounds this up to
// its minimum.
constexpr int kReceiveBufferBytes = 1024;
// Draws of a new id when the address is taken: on macOS a crashed
// receiver's socket file stays in /tmp until the machine restarts.
constexpr int kBindAttempts = 16;

#if defined(MSG_NOSIGNAL)
constexpr int kSendFlags = MSG_DONTWAIT | MSG_NOSIGNAL;
#else
constexpr int kSendFlags = MSG_DONTWAIT;  // SO_NOSIGPIPE is set instead.
#endif

struct Address {
  sockaddr_un address{};
  socklen_t length{0};
};

Address AddressOf(std::uint64_t id) {
  constexpr std::string_view kPrefix = "talos-wake-";
  constexpr std::string_view kDigits = "0123456789abcdef";
  constexpr std::size_t kNameBytes = kPrefix.size() + 16;
  std::array<char, kNameBytes> name{};
  std::memcpy(name.data(), kPrefix.data(), kPrefix.size());
  for (std::size_t i = 0; i < 16; ++i) {
    name[kPrefix.size() + i] = kDigits[(id >> (60U - 4U * i)) & 0xFU];
  }

  Address result;
  result.address.sun_family = AF_UNIX;
#if defined(__linux__)
  // Abstract namespace: a leading NUL, no filesystem entry, released with
  // the socket. The length, not a terminator, ends the name.
  std::memcpy(result.address.sun_path + 1, name.data(), name.size());
  result.length =
      static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + name.size());
#else
  constexpr std::string_view kDirectory = "/tmp/";
  static_assert(kDirectory.size() + kNameBytes < sizeof(sockaddr_un::sun_path));
  std::memcpy(result.address.sun_path, kDirectory.data(), kDirectory.size());
  std::memcpy(result.address.sun_path + kDirectory.size(), name.data(),
              name.size());
  result.length = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                         kDirectory.size() + name.size() + 1);
#if defined(__APPLE__)
  result.address.sun_len = static_cast<std::uint8_t>(result.length);
#endif
#endif
  return result;
}

const sockaddr* AsSockaddr(const Address& address) {
  return reinterpret_cast<const sockaddr*>(&address.address);
}

// A non-blocking, close-on-exec datagram socket that never raises SIGPIPE,
// or -1 with errno set.
int OpenSocket() {
#if defined(__linux__)
  return ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
#else
  const int fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  if (fd < 0) return -1;
  const int flags = ::fcntl(fd, F_GETFL);
  bool ok = flags >= 0 && ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0 &&
            ::fcntl(fd, F_SETFD, FD_CLOEXEC) == 0;
#if defined(SO_NOSIGPIPE)
  const int one = 1;
  ok = ok && ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one)) == 0;
#endif
  if (!ok) {
    const int error = errno;
    ::close(fd);
    errno = error;
    return -1;
  }
  return fd;
#endif
}

// Errors that mean no socket is bound at the receiver's address any more.
bool ReceiverGone(int error) {
  return error == ECONNREFUSED || error == ENOENT || error == ENOTCONN ||
         error == ECONNRESET || error == EPIPE || error == EDESTADDRREQ;
}

// Errors that mean the receiver's queue is full: it has notifications
// pending already, so this one carries nothing new.
bool AlreadyPending(int error) {
#if EAGAIN != EWOULDBLOCK
  if (error == EWOULDBLOCK) return true;
#endif
  return error == EAGAIN || error == ENOBUFS;
}

std::uint64_t NewId(std::random_device& random) {
  for (;;) {
    const std::uint64_t id = (static_cast<std::uint64_t>(random()) << 32U) ^
                             static_cast<std::uint64_t>(random());
    if (id != 0) return id;
  }
}

}  // namespace

WakeReceiver::WakeReceiver() {
  fd_ = OpenSocket();
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(),
                            "talos wake receiver: socket");
  }
  const int size = kReceiveBufferBytes;
  // Best effort: the default only lets more redundant datagrams queue.
  (void)::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));

  std::random_device random;
  int error = 0;
  for (int attempt = 0; attempt < kBindAttempts; ++attempt) {
    const std::uint64_t id = NewId(random);
    const Address address = AddressOf(id);
    if (::bind(fd_, AsSockaddr(address), address.length) == 0) {
      id_ = id;
      return;
    }
    error = errno;
    if (error != EADDRINUSE) break;
  }
  ::close(fd_);
  fd_ = -1;
  throw std::system_error(error, std::generic_category(),
                          "talos wake receiver: bind");
}

WakeReceiver::~WakeReceiver() { close(); }

WakeReceiver::WakeReceiver(WakeReceiver&& other) noexcept
    : fd_{std::exchange(other.fd_, -1)}, id_{std::exchange(other.id_, 0)} {}

WakeReceiver& WakeReceiver::operator=(WakeReceiver&& other) noexcept {
  if (this != &other) {
    close();
    fd_ = std::exchange(other.fd_, -1);
    id_ = std::exchange(other.id_, 0);
  }
  return *this;
}

void WakeReceiver::close() noexcept {
  if (fd_ < 0) return;
#if !defined(__linux__)
  // Unlinked first, so a sender that finds no file knows the receiver is
  // gone rather than reaching a socket about to close.
  const Address address = AddressOf(id_);
  ::unlink(address.address.sun_path);
#endif
  ::close(fd_);
  fd_ = -1;
  id_ = 0;
}

void WakeReceiver::drain() noexcept {
  if (fd_ < 0) return;
  std::array<char, 16> buffer{};
  for (;;) {
    const ssize_t received =
        ::recv(fd_, buffer.data(), buffer.size(), MSG_DONTWAIT);
    if (received >= 0) continue;  // One datagram consumed; maybe more.
    if (errno == EINTR) continue;
    return;  // EAGAIN: nothing pending.
  }
}

WakeSender::~WakeSender() { reset(); }

WakeSender::WakeSender(WakeSender&& other) noexcept
    : fd_{std::exchange(other.fd_, -1)}, id_{std::exchange(other.id_, 0)} {}

WakeSender& WakeSender::operator=(WakeSender&& other) noexcept {
  if (this != &other) {
    reset();
    fd_ = std::exchange(other.fd_, -1);
    id_ = std::exchange(other.id_, 0);
  }
  return *this;
}

void WakeSender::reset() noexcept {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
  id_ = 0;
}

bool WakeSender::notify(std::uint64_t id) noexcept {
  if (id == 0) return false;
  if (fd_ < 0 || id_ != id) {
    reset();
    const int fd = OpenSocket();
    if (fd < 0) return true;  // Out of descriptors: the hint is lost.
    const Address address = AddressOf(id);
    if (::connect(fd, AsSockaddr(address), address.length) != 0) {
      const int error = errno;
      ::close(fd);
      return !ReceiverGone(error);
    }
    fd_ = fd;
    id_ = id;
  }
  const char byte = 1;
  for (;;) {
    if (::send(fd_, &byte, sizeof(byte), kSendFlags) >= 0) return true;
    const int error = errno;
    if (error == EINTR) continue;
    if (AlreadyPending(error)) return true;
    // Gone, or a socket in an unknown state: the next call reconnects.
    reset();
    return !ReceiverGone(error);
  }
}

}  // namespace talos
