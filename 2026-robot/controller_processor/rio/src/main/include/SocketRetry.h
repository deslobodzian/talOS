#pragma once

#include <cstdint>

// When the RoboRIO loop (Robot.cpp) tries to open the gateway's socket.
//
// The socket is connected to the companion, so at program start after a
// RoboRIO boot, before the network has an address and a route to it, the
// open's connect() fails (ENETUNREACH); until it opens the loop ticks the
// gateway disabled, every output neutral, and no State leaves. An attempt is a
// few non-blocking calls (socket, setsockopt, bind, connect, close; no name
// lookup, nothing waits), so retrying every kPeriodUs costs the 5 ms loop
// nothing it would notice, and State goes out within one period of the route
// appearing.
namespace talos::rio {

class SocketRetry {
 public:
  static constexpr uint64_t kPeriodUs = 100'000;

  enum class Event : uint8_t {
    kNone,    // open already, or no attempt due
    kFailed,  // an attempt failed
    kOpened,  // an attempt opened the socket
  };

  // The loop tick at `now_us` (steady clock): calls `open` (returning true
  // when the socket opened) if the socket is not open and an attempt is due,
  // at most once per kPeriodUs. Never allocates.
  template <typename Open>
  Event Poll(uint64_t now_us, const Open& open) {
    if (open_ || (attempted_ && now_us - last_attempt_us_ < kPeriodUs))
      return Event::kNone;
    attempted_ = true;
    last_attempt_us_ = now_us;
    if (open()) {
      open_ = true;
      return Event::kOpened;
    }
    if (failures_++ == 0) first_failure_us_ = now_us;
    return Event::kFailed;
  }

  bool open() const { return open_; }
  // Failed attempts so far, and the time of the first (0 before one).
  uint32_t failures() const { return failures_; }
  uint64_t first_failure_us() const { return first_failure_us_; }

 private:
  bool open_{false};
  bool attempted_{false};
  uint64_t last_attempt_us_{0};
  uint32_t failures_{0};
  uint64_t first_failure_us_{0};
};

}  // namespace talos::rio
