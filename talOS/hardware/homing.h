#pragma once

// Item 38: homing/zeroing semantics for position mechanisms.
//
// Two phases, deliberately split:
//
//   Seek (procedure, not code): with the robot disabled, move the mechanism
//     to its physical reference by hand or with a low open-loop demand, then
//     hold it there. Homing moves; nothing here moves for you.
//
//   Capture (Gateway::Zero): declare that the held position reads `home`.
//     The capture is synchronous and validated:
//
//     - Completion: the backend applies the offset and the gateway advances
//       its epoch, so success is observable as a new epoch in State.
//     - Timeout: the capture references the sample it was taken against
//       (`observed_time_us`); a capture older than command_timeout_us is
//       rejected, so a seek interrupted long ago cannot zero against a stale
//       frame. A backend that cannot apply reports false and latches a fault.
//     - Limits: when the motor enables soft limits, `home` must lie inside
//       [reverse_limit_rot, forward_limit_rot]; anything else is rejected
//       before touching hardware.
//     - Interruption: captures are refused while a command is active (zeroing
//       under load would jump the loop) and any enable transition bumps the
//       epoch, which invalidates the seek the capture was based on.
//
// Continuous-wrap (steering) mechanisms may be zeroed: the offset shifts the
// wrap frame, which is exactly what a steering alignment needs.

#include <cstdint>
#include <string>

#include "config.h"

namespace talos::hardware {

enum class ZeroRejectReason : uint8_t {
  kOk = 0,
  kNotConfigured,
  kUnknownMotor,
  kCommandActive,
  kStaleObservation,
  kOutsideSoftLimits,
  kBackendFailed,
};

inline const char* ZeroRejectName(ZeroRejectReason reason) {
  switch (reason) {
    case ZeroRejectReason::kOk:
      return "ok";
    case ZeroRejectReason::kNotConfigured:
      return "gateway not configured";
    case ZeroRejectReason::kUnknownMotor:
      return "unknown motor id";
    case ZeroRejectReason::kCommandActive:
      return "command active: stop driving before zeroing";
    case ZeroRejectReason::kStaleObservation:
      return "stale observation: re-seek and capture promptly";
    case ZeroRejectReason::kOutsideSoftLimits:
      return "home outside soft limits";
    case ZeroRejectReason::kBackendFailed:
      return "backend failed to apply the offset";
  }
  return "unknown";
}

// Pure validation half of Gateway::Zero, testable without a backend.
inline ZeroRejectReason ValidateZeroRequest(const Config& config, uint16_t motor_id,
                                            double home_position_rot) {
  const MotorConfig* found = nullptr;
  for (const auto& m : config.motors) {
    if (m.id == motor_id) {
      found = &m;
      break;
    }
  }
  if (found == nullptr) return ZeroRejectReason::kUnknownMotor;
  if (found->soft_limits &&
      (home_position_rot < found->reverse_limit_rot ||
       home_position_rot > found->forward_limit_rot))
    return ZeroRejectReason::kOutsideSoftLimits;
  return ZeroRejectReason::kOk;
}

}  // namespace talos::hardware
