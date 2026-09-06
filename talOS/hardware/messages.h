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
  bool operator==(const State&) const = default;
};

// Explicit little-endian codecs: no padding, native bool layout, or alignment
// assumptions cross the RoboRIO/companion boundary.
std::size_t Encode(const Command&, std::span<uint8_t>);
std::size_t Encode(const State&, std::span<uint8_t>);
bool Decode(std::span<const uint8_t>, Command&);
bool Decode(std::span<const uint8_t>, State&);
}  // namespace talos::hardware
