#include "talOS/rtms/wake.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
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
// Draws of a new id when the address is taken (two receivers drew one id,
// or on macOS a killed receiver's file not yet reclaimed holds it).
constexpr int kBindAttempts = 16;
// Every receiver's name: this, then its id as 16 lowercase hex digits.
constexpr std::string_view kPrefix = "talos-wake-";
constexpr std::string_view kDigits = "0123456789abcdef";
constexpr std::size_t kNameBytes = kPrefix.size() + 16;

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

// Whether `name` is a receiver's: kPrefix and 16 lowercase hex digits.
bool IsReceiverName(std::string_view name) {
  if (name.size() != kNameBytes || !name.starts_with(kPrefix)) return false;
  name.remove_prefix(kPrefix.size());
  return name.find_first_not_of(kDigits) == std::string_view::npos;
}

std::uint64_t NewId(std::random_device& random) {
  for (;;) {
    const std::uint64_t id = (static_cast<std::uint64_t>(random()) << 32U) ^
                             static_cast<std::uint64_t>(random());
    if (id != 0) return id;
  }
}

}  // namespace

std::size_t ReclaimDeadWakeFiles(const char* directory,
                                 std::int64_t now_wall_s) noexcept {
  DIR* entries = ::opendir(directory);
  if (entries == nullptr) return 0;
  std::size_t removed = 0;
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  const std::size_t directory_bytes = std::strlen(directory);
  while (const dirent* entry = ::readdir(entries)) {
    const std::string_view name = entry->d_name;
    if (!IsReceiverName(name) ||
        directory_bytes + 1 + name.size() >= sizeof(address.sun_path)) {
      continue;
    }
    char* path = address.sun_path;
    std::memcpy(path, directory, directory_bytes);
    path[directory_bytes] = '/';
    std::memcpy(path + directory_bytes + 1, name.data(), name.size());
    path[directory_bytes + 1 + name.size()] = '\0';
    struct stat status {};
    if (::lstat(path, &status) != 0 || !S_ISSOCK(status.st_mode)) continue;
    // Young, or from the future after a clock step: a bind may still be
    // attaching its socket (kWakeFileReclaimAgeS), so a refusal proves
    // nothing yet.
    if (static_cast<std::int64_t>(status.st_ctime) >
        now_wall_s - kWakeFileReclaimAgeS) {
      continue;
    }
    const int fd = OpenSocket();
    if (fd < 0) break;  // No descriptor left: the next open tries again.
    int connected = 0;
    do {
      connected = ::connect(fd, reinterpret_cast<const sockaddr*>(&address),
                            sizeof(address));
    } while (connected != 0 && errno == EINTR);
    const int error = connected == 0 ? 0 : errno;
    ::close(fd);
    // Refused, and old enough that no bind is still attaching: nothing is
    // bound there. A bound receiver accepts the connect until its destructor
    // unlinks the file, and no new receiver can bind this path while it
    // exists.
    if (error == ECONNREFUSED && ::unlink(path) == 0) ++removed;
  }
  ::closedir(entries);
  return removed;
}

WakeReceiver::WakeReceiver() {
#if !defined(__linux__)
  // The files of receivers killed before their destructor ran, as RTMS
  // frees a dead reader's slot when the next reader claims one.
  (void)ReclaimDeadWakeFiles("/tmp",
                             static_cast<std::int64_t>(::time(nullptr)));
#endif
  std::random_device random;
  int error = 0;
  for (int attempt = 0; attempt < kBindAttempts; ++attempt) {
    if (fd_ < 0) {
      fd_ = OpenSocket();
      if (fd_ < 0) {
        throw std::system_error(errno, std::generic_category(),
                                "talos wake receiver: socket");
      }
      const int size = kReceiveBufferBytes;
      // Best effort: the default only lets more redundant datagrams queue.
      (void)::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));
    }
    const std::uint64_t id = NewId(random);
    const Address address = AddressOf(id);
    if (::bind(fd_, AsSockaddr(address), address.length) != 0) {
      error = errno;
      if (error != EADDRINUSE) break;
      continue;
    }
#if !defined(__linux__)
    // A reclaim elsewhere may have removed the file while this bind was
    // still attaching its socket (it refuses connects until then): only one
    // that passed the age guard, so against a binder stalled that long.
    // Bound without a file, no sender could reach this receiver, so it
    // binds afresh. Once bind() has returned the socket accepts every
    // connect, so no later reclaim can judge the file dead: one check
    // suffices, unless a reclaim's refused connect fell inside the bind and
    // its unlink lands after this check, which also needs the file to look
    // kWakeFileReclaimAgeS old within the bind (a wall clock step that size
    // at that instant).
    if (detail::after_wake_bind_for_testing != nullptr) {
      detail::after_wake_bind_for_testing(address.address.sun_path);
    }
    struct stat status {};
    if (::lstat(address.address.sun_path, &status) != 0 ||
        !S_ISSOCK(status.st_mode)) {
      error = ENOENT;
      ::close(fd_);
      fd_ = -1;
      continue;
    }
#endif
    id_ = id;
    return;
  }
  if (fd_ >= 0) ::close(fd_);
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

WakeStatus WakeSender::send(std::uint64_t id) noexcept {
  if (id == 0) return WakeStatus{WakeOutcome::RECEIVER_GONE, 0};
  const auto failed = [](int error) {
    const WakeOutcome outcome =
        ReceiverGone(error) ? WakeOutcome::RECEIVER_GONE : WakeOutcome::FAILED;
    return WakeStatus{outcome, error};
  };
  if (fd_ < 0 || id_ != id) {
    reset();
    const int fd = OpenSocket();
    // Out of descriptors, typically: the hint is lost.
    if (fd < 0) return WakeStatus{WakeOutcome::FAILED, errno};
    const Address address = AddressOf(id);
    int connected = 0;
    do {
      connected = ::connect(fd, AsSockaddr(address), address.length);
    } while (connected != 0 && errno == EINTR);
    if (connected != 0) {
      const int error = errno;
      ::close(fd);
      return failed(error);
    }
    fd_ = fd;
    id_ = id;
  }
  const char byte = 1;
  for (;;) {
    if (::send(fd_, &byte, sizeof(byte), kSendFlags) >= 0) return {};
    const int error = errno;
    if (error == EINTR) continue;
    if (AlreadyPending(error)) return {};
    // Gone, or a socket in an unknown state: the next call reconnects.
    reset();
    return failed(error);
  }
}

}  // namespace talos
