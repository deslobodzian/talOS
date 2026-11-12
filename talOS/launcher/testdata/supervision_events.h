#pragma once

// What launcher tests wait on instead of a duration (docs/TESTING.md §6.3):
// the supervision loop's passes, and a supervision clock the test moves
// ahead past a deadline once the event it waits for has happened.

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

#include "talOS/launcher/supervision_policy.h"

namespace talos::launcher::testing {

// The supervision loop's passes, as its service watchdog sees them: the
// launcher sends WATCHDOG=1 to $NOTIFY_SOCKET at the start of every pass
// (Launcher::Supervise), so a second datagram received after a moment means
// a whole pass ran after it. Points NOTIFY_SOCKET at itself while it lives,
// for a launcher run in this process; one at a time.
class SupervisionPasses {
 public:
  SupervisionPasses() {
    static int serial = 0;
    path_ = "/tmp/tsp_" + std::to_string(::getpid()) + "_" +
            std::to_string(serial++);
    ::unlink(path_.c_str());
    fd_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path_.c_str(), path_.size() + 1);
    if (fd_ < 0 || ::fcntl(fd_, F_SETFL, O_NONBLOCK) < 0 ||
        ::bind(fd_, reinterpret_cast<const sockaddr*>(&address),
               static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                      path_.size() + 1)) != 0)
      throw std::runtime_error("cannot bind " + path_);
    ::setenv("NOTIFY_SOCKET", path_.c_str(), 1);
  }
  ~SupervisionPasses() {
    ::unsetenv("NOTIFY_SOCKET");
    ::close(fd_);
    ::unlink(path_.c_str());
  }
  SupervisionPasses(const SupervisionPasses&) = delete;
  SupervisionPasses& operator=(const SupervisionPasses&) = delete;

  // The passes started so far that were received; never waits. Datagrams
  // the socket had no room for are lost, which only makes a wait longer.
  uint64_t Started() {
    char buffer[64];
    for (ssize_t size; (size = ::recv(fd_, buffer, sizeof buffer, 0)) >= 0;) {
      if (std::string_view{buffer, static_cast<std::size_t>(size)} ==
          "WATCHDOG=1")
        ++started_;
    }
    return started_;
  }

 private:
  std::string path_;
  int fd_{-1};
  uint64_t started_{0};
};

// steady_clock plus however far the test has moved it ahead, and never past
// where the test holds it: for LauncherOptions::supervision_clock, so that a
// deadline the launcher measures on it (restart backoff, the Studio bridge's
// retry interval, the session duration, the stop's SIGKILL) passes when the
// test says, neither after a real wait nor, while held, before. Copies share
// one state.
class SteppedClock {
 public:
  SupervisionClock clock() const {
    return [state = state_] { return state->Now(); };
  }
  void Advance(std::chrono::nanoseconds by) {
    state_->ahead.fetch_add(by.count());
  }
  // From now on the clock reads at most what it reads now plus `after`,
  // until Release.
  void Hold(std::chrono::nanoseconds after) {
    state_->held.store((state_->Now() + after).time_since_epoch().count());
  }
  void Release() { state_->held.store(kFree); }

 private:
  static constexpr int64_t kFree = INT64_MAX;
  struct State {
    std::atomic<int64_t> ahead{0};
    std::atomic<int64_t> held{kFree};
    SupervisionTime Now() const {
      const SupervisionTime moved = std::chrono::steady_clock::now() +
                                    std::chrono::nanoseconds{ahead.load()};
      const SupervisionTime cap{std::chrono::nanoseconds{held.load()}};
      return std::min(moved, cap);
    }
  };
  std::shared_ptr<State> state_{std::make_shared<State>()};
};

}  // namespace talos::launcher::testing
