#pragma once

#include <cstdint>

#include "protocol/udp.h"

// When the RoboRIO loop (Robot.cpp) tries to open the gateway's socket, and
// to replace one that can no longer send.
//
// The socket is connected to the companion, so at program start after a
// RoboRIO boot, before the network has an address and a route to it, the
// open's connect() fails (ENETUNREACH); until it opens the loop ticks the
// gateway disabled, every output neutral, and no State leaves. An attempt is a
// few non-blocking calls (socket, setsockopt, bind, connect, close; no name
// lookup, nothing waits), so retrying every kPeriodUs costs the 5 ms loop
// nothing it would notice, and State goes out within one period of the route
// appearing.
//
// A send that fails with an error the socket never recovers from
// (protocol::SendErrorNeedsNewSocket: EPIPE, ENOTCONN, EDESTADDRREQ,
// EADDRNOTAVAIL, EBADF) loses the socket: the loop ticks the gateway disabled
// again, sends nothing, and the next attempt opens a new socket on the same
// ports (Endpoint::Reopen, keeping the frame sequence), on the same cadence:
// at once if no attempt was made in the last kPeriodUs, otherwise kPeriodUs
// after the last, so a condition that also breaks every new socket costs at
// most ten sockets a second. Any other send error (ENOBUFS, ECONNREFUSED,
// ENETUNREACH, ...) clears on the same socket and keeps it; EAGAIN is not
// an error here (kWouldBlock).
namespace talos::rio {

class SocketRetry {
 public:
  static constexpr uint64_t kPeriodUs = 100'000;

  enum class Event : uint8_t {
    kNone,      // open already, or no attempt due
    kFailed,    // an attempt failed
    kOpened,    // an attempt opened the socket
    kReplaced,  // an attempt opened a socket to replace one that was lost
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
      const bool replaced = replacing_;
      replacing_ = false;
      return replaced ? Event::kReplaced : Event::kOpened;
    }
    if (failures_++ == 0) first_failure_us_ = now_us;
    return Event::kFailed;
  }

  // A send on the open socket failed at `now_us` with OS error `error`
  // (UdpStats::last_send_error). When the socket can never send again it is
  // lost: open() is false from here, replacing() true until a Poll replaces
  // it, and failures() counts that replacement's failed attempts. Returns
  // whether it was lost; any other error keeps it.
  bool SendFailed(uint64_t now_us, int error) {
    if (!open_ || !protocol::SendErrorNeedsNewSocket(error)) return false;
    open_ = false;
    replacing_ = true;
    lost_us_ = now_us;
    failures_ = 0;
    first_failure_us_ = 0;
    return true;
  }

  bool open() const { return open_; }
  // Lost and not yet replaced: `open` must open the same ports again
  // (Endpoint::Reopen) rather than a first socket.
  bool replacing() const { return replacing_; }
  // Failed attempts since the program started or, after a loss, since the
  // socket was lost, and the time of the first of them (0 before one); the
  // time of the last loss (0 before one).
  uint32_t failures() const { return failures_; }
  uint64_t first_failure_us() const { return first_failure_us_; }
  uint64_t lost_us() const { return lost_us_; }

 private:
  bool open_{false};
  bool replacing_{false};
  bool attempted_{false};
  uint64_t last_attempt_us_{0};
  uint32_t failures_{0};
  uint64_t first_failure_us_{0};
  uint64_t lost_us_{0};
};

}  // namespace talos::rio
