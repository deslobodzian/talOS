#pragma once

#include <poll.h>

#include <chrono>

#include "udp.h"

namespace talos::protocol::testing {

// How long ReceiveFrame waits by default: a bound on a hang only
// (docs/TESTING.md §6.3), never a proof that nothing arrives.
inline constexpr std::chrono::seconds kReceiveHangBound{10};

// A successful UDP send need not make the receiving socket ready immediately,
// even on loopback. Wait only for delivery: never tick the system under test or
// resend a frame. Preserve decode/socket errors rather than retrying past them.
// Between non-blocking reads it waits on the socket itself (poll(2)), so a
// frame is returned as soon as it is readable. kWouldBlock means nothing
// arrived by `deadline`.
inline UdpStatus ReceiveFrame(RuntimeUdpPeer& peer, DecodedFrame* out,
                              std::chrono::steady_clock::time_point deadline =
                                  std::chrono::steady_clock::now() +
                                  kReceiveHangBound) {
  for (;;) {
    const auto status = peer.TryReceive(out);
    const auto now = std::chrono::steady_clock::now();
    if (status != UdpStatus::kWouldBlock || now >= deadline) return status;
    // Rounded up, so a wait shorter than 1 ms does not spin.
    const auto left =
        std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
    pollfd readable{peer.NativeFd(), POLLIN, 0};
    static_cast<void>(::poll(&readable, 1, static_cast<int>(left.count())));
  }
}

}  // namespace talos::protocol::testing
