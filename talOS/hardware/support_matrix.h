#pragma once

// Item 33: truthful per-device-type support matrix.
//
// "Accepted by config" (Validate() parses it) is not "supported end to end"
// (a PhoenixBackend device exists, a sim backend emulates it, and a test pins
// it). This table is the one place that distinction is stated, and
// hardware_ops_test.cc pins every row so the matrix cannot drift from the
// code. The same table is rendered in talOS/hardware/README.md; change both.

#include <array>
#include <cstddef>
#include <string_view>

namespace talos::hardware {

enum class DeviceKind : uint8_t {
  kMotorTalonFX = 0,
  kSensorCANcoder,
  kSensorPigeon2,
  kDigitalInput,
  kDigitalOutput,
  kAnalogInput,
  kEncoderQuadrature,
  kEncoderDutyCycle,
  kPwmOutput,
};

inline constexpr std::size_t kDeviceKindCount = 9;

struct DeviceSupport {
  DeviceKind kind;
  const char* name;
  // Parsed by Validate()/ConfigurationId()/wire codecs.
  bool parsed_by_config;
  // Owned by PhoenixBackend on RoboRIO (real CAN/HAL access).
  bool real_access_phoenix;
  // Emulated by SimBackend (ideal actuators, instant response).
  bool sim_ideal;
  // Emulated by sim::PhysicsSimBackend (voltage-driven believability).
  bool sim_physics;
  // Pinned by an automated test (bazel or GradleRIO).
  bool covered_by_tests;
  const char* notes;
};

inline constexpr std::array<DeviceSupport, kDeviceKindCount> SupportMatrix() {
  return {{
      {DeviceKind::kMotorTalonFX, "TalonFX motor", true, true, true, true, true,
       "Standard voltage controls only; TorqueCurrentFOC, followers, and "
       "FusedCANcoder are not implemented."},
      {DeviceKind::kSensorCANcoder, "CANcoder", true, true, true, true, true,
       "Mirrors its linked mechanism in sim; steering uses RemoteCANcoder."},
      {DeviceKind::kSensorPigeon2, "Pigeon2 IMU", true, true, true, true, true,
       "Native mounting frame only; yaw in rotations, CCW+."},
      {DeviceKind::kDigitalInput, "DIO input", true, true, true, true, true,
       "Beam breaks and limit switches via SetDigitalInput in sim."},
      {DeviceKind::kDigitalOutput, "DIO output", true, true, true, true, true,
       "De-energized (false) on every safe-state transition."},
      {DeviceKind::kAnalogInput, "Analog input", true, true, true, true, true,
       "Voltage readback; pressure-sensor style inputs in sim."},
      {DeviceKind::kEncoderQuadrature, "Quadrature encoder", true, true, true,
       true, true,
       "counts_per_rev scales edges to rotations; offset-corrected."},
      {DeviceKind::kEncoderDutyCycle, "Duty-cycle encoder", true, true, true,
       true, true,
       "Absolute position only; velocity is finite-difference across Reads."},
      {DeviceKind::kPwmOutput, "PWM output", true, true, true, true, true,
       "Zero on every safe-state transition."},
  }};
}

// Supported end to end: parsed, really accessible, simulated both ways, and
// tested. Anything less is "accepted by config" only.
inline constexpr bool IsSupportedEndToEnd(const DeviceSupport& s) {
  return s.parsed_by_config && s.real_access_phoenix && s.sim_ideal &&
         s.sim_physics && s.covered_by_tests;
}

inline constexpr std::string_view DeviceKindName(DeviceKind kind) {
  for (const auto& row : SupportMatrix()) {
    if (row.kind == kind) return row.name;
  }
  return "unknown";
}

}  // namespace talos::hardware
