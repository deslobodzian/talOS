#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
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
  // [motors.*] foc: FOC commutation (Phoenix EnableFOC) on every duty-cycle,
  // voltage, velocity, position and motion-magic request the RoboRIO sends
  // this motor. Needs a Phoenix Pro licence, which the RoboRIO checks at
  // configure by reading the device's IsProLicensed signal (whether a
  // CANivore's licence shows there is unconfirmed). On the config wire and in
  // ConfigurationId; refused on a follower, which copies its master's output.
  bool foc{false};
  // [motors.*] max_torque_current_a: the largest stator current a
  // torque-current demand may ask for, amperes; 0 (the default) refuses the
  // mode. Needs foc, at most stator_limit_a, refused on a follower and on a
  // motor with one (the mode drives a single motor). On the config wire and
  // in ConfigurationId.
  double max_torque_current_a{0};
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

// A simulated home switch on a digital input: the physics simulator reads it
// closed while `motor`'s reported position (mechanism rotations, hardware
// frame) is at or below `closed_at_rot` (at or above when `closed_above`),
// and reads `closed_reads` while closed. Like LocalMotorType, not on the
// config wire and equal whatever its value.
struct LocalHomeSwitch {
  std::string motor;  // Empty: no simulated switch.
  double closed_at_rot{};
  bool closed_above{false};
  bool closed_reads{false};
  bool operator==(const LocalHomeSwitch&) const { return true; }
};

struct DigitalInputConfig {
  uint16_t id{};
  std::string name;
  int dio{};
  // [digital_inputs.*] sim_motor, sim_closed_at_rot, sim_closed_above and
  // sim_closed_reads. Not sent to the controller and not part of
  // ConfigurationId; sim_gateway --physics hands them to the physics backend
  // from its own parse of the robot config
  // (sim::PhysicsSimBackend::SetHomeSwitches).
  LocalHomeSwitch sim_home_switch{};
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
// logical IDs, or CAN addresses on a bus are rejected, and so is a config
// naming more than kMaxCanBuses - 1 CAN buses besides "rio" (CanBuses).
void Validate(const Config& config);
uint64_t ConfigurationId(const Config& config);

// The CAN buses whose health State carries (HealthSample::can_bus) and
// /hw/status reports, one slot each.
// Slot 0 is "rio" always; then each other distinct bus a motor or sensor
// names, in byte order. config_id covers bus names, so slot i is the same
// bus on both ends; changing this order needs a protocol bump.
inline constexpr std::size_t kMaxCanBuses = 4;
struct CanBusList {
  std::array<std::string_view, kMaxCanBuses> names{};
  std::size_t count{};
};
// The names view `config`'s own strings ("rio" excepted), so they live as
// long as `config` does not change. Throws std::invalid_argument naming the
// buses when there are more than kMaxCanBuses - 1 besides "rio".
CanBusList CanBuses(const Config& config);
}  // namespace talos::hardware
