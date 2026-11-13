#pragma once

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string_view>

namespace talos::launcher {

// A nonblocking sd_notify transport: a failed or full socket must never stall
// supervision. Only the supervision thread sends watchdog notifications.
class ServiceNotifier {
 public:
  ServiceNotifier() {
    // systemd sets WATCHDOG_USEC with WatchdogSec=; without a socket to send
    // WATCHDOG=1 on it goes unused.
    const char* watchdog = std::getenv("WATCHDOG_USEC");
    if (watchdog != nullptr) {
      char* end = nullptr;
      const unsigned long long usec = std::strtoull(watchdog, &end, 10);
      if (end != watchdog && *end == '\0' && usec > 0)
        watchdog_ = std::chrono::microseconds{static_cast<int64_t>(usec)};
    }
    const char* path = std::getenv("NOTIFY_SOCKET");
    if (path == nullptr || path[0] == '\0') return;
    const std::size_t size = std::strlen(path);
    if (size >= sizeof(address_.sun_path) || (path[0] != '/' && path[0] != '@'))
      return;
    address_.sun_family = AF_UNIX;
    std::memcpy(address_.sun_path, path, size + 1);
    length_ =
        static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + size + 1);
    if (path[0] == '@') {
      address_.sun_path[0] = '\0';
      --length_;  // Abstract names have no terminating NUL.
    }
    fd_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd_ >= 0 && (::fcntl(fd_, F_SETFD, FD_CLOEXEC) < 0 ||
                     ::fcntl(fd_, F_SETFL, O_NONBLOCK) < 0)) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  ~ServiceNotifier() {
    if (fd_ >= 0) ::close(fd_);
  }
  ServiceNotifier(const ServiceNotifier&) = delete;
  ServiceNotifier& operator=(const ServiceNotifier&) = delete;

  // How often WATCHDOG=1 must go out at least: half the service's
  // WATCHDOG_USEC, as systemd advises (sd_watchdog_enabled), so a
  // notification sent late still lands inside it; nullopt without a
  // watchdog or a socket.
  std::optional<std::chrono::microseconds> watchdog_interval() const {
    if (fd_ < 0 || !watchdog_) return std::nullopt;
    return *watchdog_ / 2;
  }

  void Send(std::string_view message) const {
    if (fd_ < 0) return;
    (void)::sendto(fd_, message.data(), message.size(), 0,
                   reinterpret_cast<const sockaddr*>(&address_), length_);
  }

 private:
  int fd_{-1};
  std::optional<std::chrono::microseconds> watchdog_;
  sockaddr_un address_{};
  socklen_t length_{0};
};

}  // namespace talos::launcher
