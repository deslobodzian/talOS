#pragma once

#include <cstdint>

#include "RioLog.h"
#include "hardware/gateway.h"

// Console lines for what the RoboRIO loop (Robot.cpp) finds after each tick
// that no State field carries: each command expiry, with its time and its
// delay from the command's observation, so a link, bridge or launcher loss
// that leaves no State on the main processor still records when the RoboRIO
// went neutral.
//
// Called on the loop thread, it only compares counters and posts a
// fixed-size RioLog event when one rose; RioLog's logger thread writes the
// line (at most one a second per message, later ones coalesced with their
// count). It reads the gateway and never changes it.
namespace talos::rio {

class LoopConsole {
 public:
  // RioLog kMessage indices (rate-limit keys) of its lines; Robot.cpp uses 1
  // to 6, PhoenixBackend 0, 7, 8 and 16 to 31.
  static constexpr uint16_t kCommandExpiryMessage = 9;

  explicit LoopConsole(RioLog& log) : log_{log} {}

  // The loop thread, after each tick's gateway work: one line for the
  // gateway's last command expiry when one happened since the previous call.
  void ObserveGateway(const hardware::Gateway& gateway);

 private:
  RioLog& log_;
  uint64_t reported_expiries_{0};
};

}  // namespace talos::rio
