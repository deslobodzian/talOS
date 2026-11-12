#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace talos::hardware {

inline constexpr std::size_t kMaxMotors = 16;
inline constexpr std::size_t kMaxSensors = 12;
inline constexpr std::size_t kMaxDigitalInputs = 8;
inline constexpr std::size_t kMaxDigitalOutputs = 4;
inline constexpr std::size_t kMaxAnalogInputs = 4;
inline constexpr std::size_t kMaxEncoders = 4;
inline constexpr std::size_t kMaxPwmOutputs = 4;
enum class SensorKind : uint8_t { kCANcoder = 1, kPigeon2 = 2 };
enum class Feedback : uint8_t { kRotor = 0, kRemoteCANcoder = 1 };
enum class GravityType : uint8_t { kElevatorStatic = 0, kArmCosine = 1 };
// A switch wired to the TalonFX's own forward or reverse limit input. While it
// is asserted (a normally open switch closed, a normally closed one open) the
// TalonFX itself holds output toward that direction neutral, in every control
// mode and whatever the position frame; output away from it still drives.
// kNormallyOpen, enabled, is Phoenix's default for both inputs.
enum class LimitSwitch : uint8_t {
  kNormallyOpen = 0,
  kNormallyClosed = 1,
  kDisabled = 2
};

struct Gains {
  double p{}, i{}, d{}, s{}, v{}, a{}, g{};
  GravityType gravity_type{GravityType::kElevatorStatic};
  bool operator==(const Gains&) const = default;
};

// A motor setting only the main processor reads. It is not on the config
// wire, so it compares equal whatever its value: Decode(Encode(c)) == c still
// holds, and the gateway's same-config shortcut (Reconfigure) is unaffected.
struct LocalMotorSetting {
  bool value{false};
  bool operator==(const LocalMotorSetting&) const { return true; }
};

// The motor a TalonFX drives. Only the physics simulator reads it, for its
// winding model (sim_physics.h MotorModelFor).
enum class MotorType : uint8_t { kKrakenX60 = 0, kKrakenX44, kFalcon500 };

// A motor type the RoboRIO never reads: like LocalMotorSetting, not on the
// config wire and equal whatever its value.
struct LocalMotorType {
  MotorType value{MotorType::kKrakenX60};
  bool operator==(const LocalMotorType&) const { return true; }
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
  LimitSwitch forward_limit_switch{LimitSwitch::kNormallyOpen};
  LimitSwitch reverse_limit_switch{LimitSwitch::kNormallyOpen};
  double max_velocity_rps{100};
  double cruise_velocity_rps{20}, acceleration_rps2{40}, jerk_rps3{0};
  std::array<Gains, 3> slots{};
  uint16_t master_id{0};  // Zero denotes an independent motor.
  bool oppose_master{false};
  // Main processor only (talOS/devices), not sent to the controller and not
  // part of ConfigurationId: [motors.*] position_frame = "absolute" (true)
  // or "rezero" (false, the default). A rezero-frame motor's position and
  // motion-magic demands go out neutral until the owning node's
  // Motor::rezero holds, and again once a device reset or gateway boot drops
  // it. An absolute-frame motor (remote CANcoder feedback only) commands
  // positions in the sensor's frame, which survives a reset, from its first
  // state, and keeps its rezero across resets.
  LocalMotorSetting absolute_position_frame{};
  // [motors.*] motor_type = "kraken_x60" (the default), "kraken_x44" or
  // "falcon_500": the physics simulator's stall current and free speed. Not
  // sent to the controller and not part of ConfigurationId; the physics
  // backend inside sim_gateway takes it from sim_gateway's own parse of the
  // robot config, since a pushed config carries none
  // (sim::PhysicsSimBackend::SetLocalMotorTypes).
  LocalMotorType motor_type{};
  bool operator==(const MotorConfig&) const = default;
};

struct SensorConfig {
  uint16_t id{};
  std::string name;
  SensorKind kind{SensorKind::kCANcoder};
  int can_id{};
  std::string bus{"rio"};
  bool inverted{false};
  // CANcoder magnet offset, or Pigeon mount yaw added before inversion.
  double offset_rot{0};  // rotations, [-0.5, 0.5)
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
  // Whether this subsystem drives an output (motor, digital output or PWM
  // output): the bridge merges and times out its requests, the configuration
  // bounds its period_us, and the launcher treats it as an actuator owner. A
  // new output kind joins here first.
  bool owns_actuators() const {
    return !motors.empty() || !digital_outputs.empty() || !pwm_outputs.empty();
  }
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
