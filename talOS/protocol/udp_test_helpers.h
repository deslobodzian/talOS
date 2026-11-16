#pragma once

#include <chrono>
#include <thread>

#include "udp.h"

namespace talos::protocol::testing {

// A successful UDP send need not make the receiving socket ready immediately,
// even on loopback. Wait only for delivery: never tick the system under test or
// resend a frame. Preserve decode/socket errors rather than retrying past them.
inline UdpStatus ReceiveFrame(RuntimeUdpPeer& peer, DecodedFrame* out,
                              std::chrono::steady_clock::time_point deadline =
                                  std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(200)) {
  for (;;) {
    const auto status = peer.TryReceive(out);
    if (status != UdpStatus::kWouldBlock ||
        std::chrono::steady_clock::now() >= deadline)
      return status;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace talos::protocol::testing
