#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "config.h"

namespace talos::hardware {
enum class Mode : uint8_t {
  kNeutral,
  kDutyCycle,
  kVoltage,
  kVelocity,
  kPosition,
  kMotionMagic
};
enum StateFlags : uint32_t {
  kConfigured = 1,
  kEnabled = 2,
  kCommandActive = 4,
  kHardwareFault = 8
};
struct MotorRequest {
  uint16_t id{};
  Mode mode{Mode::kNeutral};
  uint8_t slot{};
  double demand{};  // duty ratio, volts, mechanism rotations/s, or rotations.
  double feedforward_v{};
  bool operator==(const MotorRequest&) const = default;
};
struct DigitalOutputRequest {
  uint16_t id{};
  bool value{false};
  bool operator==(const DigitalOutputRequest&) const = default;
};
struct PwmRequest {
  uint16_t id{};
  double output{};
  bool operator==(const PwmRequest&) const = default;
};
struct Command {
  uint64_t config_id{}, boot_id{}, epoch{}, observed_time_us{};
  uint16_t count{};
  uint16_t digital_output_count{};
  uint16_t pwm_output_count{};
  std::array<MotorRequest, kMaxMotors> motors{};
  std::array<DigitalOutputRequest, kMaxDigitalOutputs> digital_outputs{};
  std::array<PwmRequest, kMaxPwmOutputs> pwm_outputs{};
  bool operator==(const Command&) const = default;
};
struct MotorSample {
  uint16_t id{};
  bool valid{};
  double position_rot{}, velocity_rps{}, voltage{}, stator_current_a{};
  uint32_t position_age_us{}, velocity_age_us{};
  uint32_t fault_flags{};
  bool operator==(const MotorSample&) const = default;
};
struct SensorSample {
  uint16_t id{};
  bool valid{};
  double position_rot{}, velocity_rps{};  // Pigeon yaw and yaw rate, CCW+.
  uint32_t position_age_us{}, velocity_age_us{};
  bool operator==(const SensorSample&) const = default;
};
struct DigitalInputSample {
  uint16_t id{};
  bool valid{};
  bool value{};
  bool operator==(const DigitalInputSample&) const = default;
};
struct DigitalOutputSample {
  uint16_t id{};
  bool valid{};
  bool value{};
  bool operator==(const DigitalOutputSample&) const = default;
};
struct AnalogInputSample {
  uint16_t id{};
  bool valid{};
  double voltage{};
  uint32_t age_us{};
  bool operator==(const AnalogInputSample&) const = default;
};
struct EncoderSample {
  uint16_t id{};
  bool valid{};
  double position_rot{};
  double velocity_rps{};
  uint32_t age_us{};
  bool operator==(const EncoderSample&) const = default;
};
struct PwmSample {
  uint16_t id{};
  bool valid{};
  double output{};
  bool operator==(const PwmSample&) const = default;
};
struct HealthSample {
  bool valid{};
  double battery_v{};
  bool brownout{};
  double can_utilization_frac{};
  bool operator==(const HealthSample&) const = default;
};
struct GatewayCounters {
  uint32_t rejected_command_count{};
  uint32_t neutralized_motor_count{};
  uint32_t apply_failure_count{};
  uint32_t device_reset_count{};
  bool operator==(const GatewayCounters&) const = default;
};
struct State {
  uint64_t config_id{}, boot_id{}, epoch{}, sample_time_us{},
      last_command_sequence{};
  uint32_t flags{};
  uint16_t motor_count{}, sensor_count{};
  uint16_t digital_input_count{}, digital_output_count{};
  uint16_t analog_input_count{}, encoder_count{}, pwm_output_count{};
  std::array<MotorSample, kMaxMotors> motors{};
  std::array<SensorSample, kMaxSensors> sensors{};
  std::array<DigitalInputSample, kMaxDigitalInputs> digital_inputs{};
  std::array<DigitalOutputSample, kMaxDigitalOutputs> digital_outputs{};
  std::array<AnalogInputSample, kMaxAnalogInputs> analog_inputs{};
  std::array<EncoderSample, kMaxEncoders> encoders{};
  std::array<PwmSample, kMaxPwmOutputs> pwm_outputs{};
  HealthSample health{};
  GatewayCounters counters{};
  bool operator==(const State&) const = default;
};

// Canonical encoded bytes, independent of native struct padding.
inline constexpr std::size_t kCommandBaseSize = 4 * 8 + 3 * 2;
inline constexpr std::size_t kMotorRequestSize = 2 + 1 + 1 + 2 * 8;
inline constexpr std::size_t kDigitalOutputRequestSize = 2 + 1;
inline constexpr std::size_t kPwmRequestSize = 2 + 8;
inline constexpr std::size_t kMaxCommandSize =
    kCommandBaseSize + kMaxMotors * kMotorRequestSize +
    kMaxDigitalOutputs * kDigitalOutputRequestSize +
    kMaxPwmOutputs * kPwmRequestSize;

inline constexpr std::size_t kHealthSampleSize = 1 + 8 + 1 + 8;
inline constexpr std::size_t kGatewayCountersSize = 4 * 4;
inline constexpr std::size_t kStateBaseSize =
    5 * 8 + 4 + 7 * 2 + kHealthSampleSize + kGatewayCountersSize;
inline constexpr std::size_t kMotorSampleSize = 2 + 1 + 4 * 8 + 3 * 4;
inline constexpr std::size_t kSensorSampleSize = 2 + 1 + 2 * 8 + 2 * 4;
inline constexpr std::size_t kDigitalInputSampleSize = 2 + 1 + 1;
inline constexpr std::size_t kDigitalOutputSampleSize = 2 + 1 + 1;
inline constexpr std::size_t kAnalogInputSampleSize = 2 + 1 + 8 + 4;
inline constexpr std::size_t kEncoderSampleSize = 2 + 1 + 2 * 8 + 4;
inline constexpr std::size_t kPwmSampleSize = 2 + 1 + 8;
inline constexpr std::size_t kMaxStateSize =
    kStateBaseSize + kMaxMotors * kMotorSampleSize +
    kMaxSensors * kSensorSampleSize +
    kMaxDigitalInputs * kDigitalInputSampleSize +
    kMaxDigitalOutputs * kDigitalOutputSampleSize +
    kMaxAnalogInputs * kAnalogInputSampleSize +
    kMaxEncoders * kEncoderSampleSize + kMaxPwmOutputs * kPwmSampleSize;

static_assert(kMaxCommandSize == 410, "Update docs/DATA_FLOW.md");
static_assert(kMaxStateSize == 1412, "Update docs/DATA_FLOW.md");

// Explicit little-endian codecs: no padding, native bool layout, or alignment
// assumptions cross the RoboRIO/companion boundary.
std::size_t Encode(const Command&, std::span<uint8_t>);
std::size_t Encode(const State&, std::span<uint8_t>);
bool Decode(std::span<const uint8_t>, Command&);
bool Decode(std::span<const uint8_t>, State&);
}  // namespace talos::hardware
