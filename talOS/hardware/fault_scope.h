#pragma once

// Item 37: deliberate fault scope.
//
// One bad signal must not stop the robot when the rest of the bus is fine,
// but a lost feedback loop must never keep driving blind. The policy:
//
//   TelemetryOnly: stale motor voltage/current. The effort fields zero, the
//     sample stays valid, the backend stays healthy. Subsystems see current
//     telemetry degrade; motion continues.
//   Mechanism: an invalid RIO quadrature/duty-cycle encoder. That sample goes
//     invalid but the backend stays healthy: RIO encoders are auxiliary
//     feedback (TalonFX loops use rotor/RemoteCANcoder), so the owning
//     subsystem stops using that encoder while the robot keeps driving.
//     DIO/analog/PWM never sink the backend either (RIO reports them
//     unconditionally valid).
//   Robot: stale/invalid motor position/velocity, stale/invalid CAN
//     sensor (CANcoder/Pigeon2), any motor/sensor disconnect, frozen
//     feedback, failed Apply, failed Configure. The gateway neutralizes every
//     output, invalidates outstanding commands (epoch bump), and latches the
//     fault where applicable. Per-mechanism isolation for these is future
//     work: Gateway::Accept applies all motors atomically.

namespace talos::hardware {

enum class FaultScope : uint8_t { kTelemetryOnly = 0, kMechanism, kRobot };

enum class FailureKind : uint8_t {
  kStaleMotorCurrent = 0,
  kStaleMotorVoltage,
  kStaleMotorPosition,
  kStaleMotorVelocity,
  kMotorDisconnect,
  kStaleSensor,  // CANcoder or Pigeon2 position/velocity.
  kSensorDisconnect,
  kEncoderInvalid,  // RIO quadrature or duty-cycle encoder.
  kApplyFailed,
  kConfigFailed,
};

inline constexpr FaultScope ClassifyFailure(FailureKind kind) {
  switch (kind) {
    case FailureKind::kStaleMotorCurrent:
    case FailureKind::kStaleMotorVoltage:
      return FaultScope::kTelemetryOnly;
    case FailureKind::kEncoderInvalid:
      return FaultScope::kMechanism;
    default:
      return FaultScope::kRobot;
  }
}

inline constexpr const char* FaultScopeName(FaultScope scope) {
  switch (scope) {
    case FaultScope::kTelemetryOnly:
      return "telemetry-only";
    case FaultScope::kMechanism:
      return "mechanism";
    case FaultScope::kRobot:
      return "robot";
  }
  return "unknown";
}

}  // namespace talos::hardware
