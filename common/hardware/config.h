#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace talos::hardware {

inline constexpr std::size_t kMaxMotors = 16;
inline constexpr std::size_t kMaxSensors = 6;
inline constexpr std::size_t kMaxDigitalInputs = 8;
inline constexpr std::size_t kMaxDigitalOutputs = 4;
inline constexpr std::size_t kMaxAnalogInputs = 4;
inline constexpr std::size_t kMaxEncoders = 4;
inline constexpr std::size_t kMaxPwmOutputs = 4;
enum class SensorKind : uint8_t { kCANcoder = 1, kPigeon2 = 2 };
enum class Feedback : uint8_t { kRotor = 0, kRemoteCANcoder = 1 };

struct Gains {
  double p{}, i{}, d{}, s{}, v{}, a{}, g{};
  bool operator==(const Gains&) const = default;
};

struct MotorConfig {
  uint16_t id{};  // Logical ID, independent of CAN address and array index.
  std::string name;
  int can_id{};
  std::string bus{"rio"};
  bool inverted{false};
  bool brake{true};
  double supply_limit_a{40};
  double stator_limit_a{80};
  double max_voltage{12};
  Feedback feedback{Feedback::kRotor};
  uint16_t feedback_sensor_id{};
  double rotor_to_sensor_ratio{1};
  double sensor_to_mechanism_ratio{1};
  bool continuous_wrap{false};
  bool soft_limits{false};
  double reverse_limit_rot{-1}, forward_limit_rot{1};
  double max_velocity_rps{100};
  double cruise_velocity_rps{20}, acceleration_rps2{40}, jerk_rps3{0};
  std::array<Gains, 3> slots{};
  bool operator==(const MotorConfig&) const = default;
};

struct SensorConfig {
  uint16_t id{};
  std::string name;
  SensorKind kind{SensorKind::kCANcoder};
  int can_id{};
  std::string bus{"rio"};
  bool inverted{false};
  double offset_rot{0};  // CANcoder magnet offset in rotations.
  bool operator==(const SensorConfig&) const = default;
};

struct DigitalInputConfig {
  uint16_t id{};
  std::string name;
  int dio{};
  bool operator==(const DigitalInputConfig&) const = default;
};

struct DigitalOutputConfig {
  uint16_t id{};
  std::string name;
  int dio{};
  bool operator==(const DigitalOutputConfig&) const = default;
};

struct AnalogInputConfig {
  uint16_t id{};
  std::string name;
  int channel{};
  bool operator==(const AnalogInputConfig&) const = default;
};

enum class EncoderKind : uint8_t { kQuadrature = 1, kDutyCycle = 2 };

struct EncoderConfig {
  uint16_t id{};
  std::string name;
  EncoderKind kind{EncoderKind::kQuadrature};
  int dio_a{};
  int dio_b{-1};
  double counts_per_rev{2048};
  double offset_rot{0};
  bool operator==(const EncoderConfig&) const = default;
};

struct PwmConfig {
  uint16_t id{};
  std::string name;
  int channel{};
  bool operator==(const PwmConfig&) const = default;
};

struct Devices {
  std::string subsystem;
  std::vector<uint16_t> motors;
  std::vector<uint16_t> sensors;
  std::vector<uint16_t> digital_inputs;
  std::vector<uint16_t> digital_outputs;
  std::vector<uint16_t> analog_inputs;
  std::vector<uint16_t> encoders;
  std::vector<uint16_t> pwm_outputs;
  bool operator==(const Devices&) const = default;
};

struct Config {
  std::vector<MotorConfig> motors;
  std::vector<SensorConfig> sensors;
  std::vector<DigitalInputConfig> digital_inputs;
  std::vector<DigitalOutputConfig> digital_outputs;
  std::vector<AnalogInputConfig> analog_inputs;
  std::vector<EncoderConfig> encoders;
  std::vector<PwmConfig> pwm_outputs;
  uint32_t period_us{
      5000};  // 200 Hz setpoints/telemetry, PID stays on TalonFX.
  uint32_t command_timeout_us{100000};
  double status_hz{200};
  bool commissioned{false};
  bool operator==(const Config&) const = default;
};

// Throws before any device is opened. Unknown references and duplicate names,
// logical IDs, or CAN addresses on a bus are rejected.
void Validate(const Config& config);
uint64_t ConfigurationId(const Config& config);
}  // namespace talos::hardware
