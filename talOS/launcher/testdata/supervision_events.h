#pragma once

// What launcher tests wait on instead of a duration (docs/TESTING.md §6.3):
// the supervision loop's passes, and a supervision clock the test moves
// ahead past a deadline once the event it waits for has happened.

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "talOS/launcher/spawn.h"
#include "talOS/launcher/supervision_policy.h"

namespace talos::launcher::testing {

// The supervision loop's passes, as its service watchdog sees them: the
// launcher sends WATCHDOG=1 to $NOTIFY_SOCKET at the start of every pass
// (Launcher::Supervise), so a second datagram received after a moment means
// a whole pass ran after it. A pass runs at the loop's next deadline, or at
// once on SupervisionWakeup::Wake(). Points NOTIFY_SOCKET at itself while it
// lives, for a launcher run in this process; one at a time.
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

// steady_clock plus however far the test has moved it ahead, or, while the
// test holds it, exactly where the test holds it: for
// LauncherOptions::supervision_clock, so that a deadline the launcher
// measures on it (restart backoff, the Studio bridge's settle interval, the
// session duration, the stop's SIGKILL) passes when the test says, neither
// after a real wait nor, while held, before. It never reads less than it
// has read. Every move wakes the supervision loop (SupervisionWakeup::Wake),
// which otherwise sleeps the real time its next deadline was away when it
// last looked. Copies share one state.
class SteppedClock {
 public:
  explicit SteppedClock(SupervisionClock base =
                            [] { return std::chrono::steady_clock::now(); })
      : state_{std::make_shared<State>(std::move(base))} {}

  SupervisionClock clock() const {
    return [state = state_] { return state->Now(); };
  }
  void Advance(std::chrono::nanoseconds by) {
    {
      std::lock_guard<std::mutex> lock{state_->mu};
      state_->ahead += by.count();
    }
    SupervisionWakeup::Wake();
  }
  // Holds the clock where it reads now, which it returns.
  SupervisionTime Freeze() {
    SupervisionTime now;
    {
      std::lock_guard<std::mutex> lock{state_->mu};
      now = state_->NowLocked();
      state_->held = now.time_since_epoch().count();
    }
    SupervisionWakeup::Wake();
    return now;
  }
  // The clock reads exactly `at`, no earlier than it has read, until
  // HoldAt again or Release.
  void HoldAt(SupervisionTime at) {
    {
      std::lock_guard<std::mutex> lock{state_->mu};
      const int64_t target =
          std::max(state_->last, at.time_since_epoch().count());
      state_->held = target;
      state_->last = target;
    }
    SupervisionWakeup::Wake();
  }
  // The clock runs on from where it is held.
  void Release() {
    {
      std::lock_guard<std::mutex> lock{state_->mu};
      if (state_->held == kFree) return;
      const int64_t real = state_->base().time_since_epoch().count();
      state_->ahead = state_->held - real;
      state_->held = kFree;
    }
    SupervisionWakeup::Wake();
  }

  // Releases the clock when it goes out of scope: a test that gives up
  // waiting must not leave the launcher's own deadlines frozen, or its hang
  // bound becomes a test-runner timeout with no message.
  class ReleaseOnExit {
   public:
    explicit ReleaseOnExit(SteppedClock& clock) : clock_{clock} {}
    ~ReleaseOnExit() { clock_.Release(); }
    ReleaseOnExit(const ReleaseOnExit&) = delete;
    ReleaseOnExit& operator=(const ReleaseOnExit&) = delete;

   private:
    SteppedClock& clock_;
  };

 private:
  static constexpr int64_t kFree = INT64_MAX;
  struct State {
    explicit State(SupervisionClock base_clock) : base{std::move(base_clock)} {}
    mutable std::mutex mu;
    SupervisionClock base;
    int64_t ahead{0};
    int64_t held{kFree};
    int64_t last{INT64_MIN};

    SupervisionTime NowLocked() {
      const int64_t raw =
          (held != kFree) ? held : base().time_since_epoch().count() + ahead;
      last = std::max(last, raw);
      return SupervisionTime{std::chrono::nanoseconds{last}};
    }

    SupervisionTime Now() {
      std::lock_guard<std::mutex> lock{mu};
      return NowLocked();
    }
  };
  std::shared_ptr<State> state_;
};

}  // namespace talos::launcher::testing
