#include "gateway.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "config_wire.h"
#include "endpoint.h"
#include "sim_backend.h"
#include "sim_physics.h"

namespace talos::hardware {
namespace {

TEST(Messages, FullyCappedEncodingsHaveExactSizes) {
  State state;
  state.motor_count = kMaxMotors;
  state.sensor_count = kMaxSensors;
  state.digital_input_count = kMaxDigitalInputs;
  state.digital_output_count = kMaxDigitalOutputs;
  state.analog_input_count = kMaxAnalogInputs;
  state.encoder_count = kMaxEncoders;
  state.pwm_output_count = kMaxPwmOutputs;
  std::array<uint8_t, kMaxStateSize> state_bytes{};
  ASSERT_EQ(Encode(state, state_bytes), 1406u);
  State decoded_state;
  ASSERT_TRUE(Decode(state_bytes, decoded_state));
  EXPECT_EQ(decoded_state, state);
  EXPECT_EQ(Encode(state, std::span{state_bytes}.first(state_bytes.size() - 1)),
            0u);

  Command command;
  command.count = kMaxMotors;
  command.digital_output_count = kMaxDigitalOutputs;
  command.pwm_output_count = kMaxPwmOutputs;
  std::array<uint8_t, kMaxCommandSize> command_bytes{};
  ASSERT_EQ(Encode(command, command_bytes), 410u);
  Command decoded_command;
  ASSERT_TRUE(Decode(command_bytes, decoded_command));
  EXPECT_EQ(decoded_command, command);
  EXPECT_EQ(
      Encode(command, std::span{command_bytes}.first(command_bytes.size() - 1)),
      0u);
}

// The build identity sits after the seven u64 words of the State prefix. A
// program value this build does not know, or a dirty byte other than 0 or 1,
// is a malformed State, not a guess; and an unknown program never encodes.
// REQ-CONTRACT-30: unknown program or dirty byte other than 0/1 is rejected
TEST(Messages, GatewayBuildRejectsUnknownProgramsAndNonBooleanDirty) {
  State state;
  state.build = {0xfedcba9876543210ull, GatewayProgram::kSimGateway, false};
  std::array<uint8_t, kMaxStateSize> bytes{};
  const auto size = Encode(state, bytes);
  ASSERT_NE(size, 0u);
  const auto encoded = std::span{bytes}.first(size);
  EXPECT_EQ(bytes[56], 0x10);  // revision, little-endian
  EXPECT_EQ(bytes[63], 0xfe);
  EXPECT_EQ(bytes[64], static_cast<uint8_t>(GatewayProgram::kSimGateway));
  State decoded;
  bytes[64] = static_cast<uint8_t>(static_cast<int>(kLastGatewayProgram) + 1);
  EXPECT_FALSE(Decode(encoded, decoded));
  bytes[64] = static_cast<uint8_t>(GatewayProgram::kRoboRio);
  bytes[65] = 2;
  EXPECT_FALSE(Decode(encoded, decoded));

  state.build.program = static_cast<GatewayProgram>(
      static_cast<uint8_t>(kLastGatewayProgram) + 1);
  EXPECT_EQ(Encode(state, bytes), 0u);
}

TEST(Messages, MotorDiagnosticsUseSaturatingSignedTenths) {
  State state;
  state.motor_count = 1;
  auto& motor = state.motors[0];
  motor.valid = true;
  motor.temperature_c = 42.34;
  motor.supply_current_a = -12.36;
  motor.stator_current_a = 81.25;
  std::array<uint8_t, kMaxStateSize> bytes{};
  State decoded;
  auto size = Encode(state, bytes);
  ASSERT_NE(size, 0u);
  ASSERT_TRUE(Decode(std::span{bytes}.first(size), decoded));
  EXPECT_DOUBLE_EQ(decoded.motors[0].temperature_c, 42.3);
  EXPECT_DOUBLE_EQ(decoded.motors[0].supply_current_a, -12.4);
  EXPECT_DOUBLE_EQ(decoded.motors[0].stator_current_a, 81.3);

  motor.temperature_c = -1e100;
  motor.supply_current_a = 1e100;
  motor.stator_current_a = 3276.7;
  size = Encode(state, bytes);
  ASSERT_TRUE(Decode(std::span{bytes}.first(size), decoded));
  EXPECT_DOUBLE_EQ(decoded.motors[0].temperature_c, -3276.8);
  EXPECT_DOUBLE_EQ(decoded.motors[0].supply_current_a, 3276.7);
  EXPECT_DOUBLE_EQ(decoded.motors[0].stator_current_a, 3276.7);

  for (double invalid : {std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    motor.temperature_c = invalid;
    EXPECT_EQ(Encode(state, bytes), 0u);
    motor.temperature_c = 25;
    motor.supply_current_a = invalid;
    EXPECT_EQ(Encode(state, bytes), 0u);
    motor.supply_current_a = 0;
    motor.stator_current_a = invalid;
    EXPECT_EQ(Encode(state, bytes), 0u);
    motor.stator_current_a = 0;
  }
}

struct ModuleConfig {
  const char* name;
  uint16_t drive_id, steer_id, encoder_id;
  double x_m, y_m;
  double wheel_radius_m;
  int drive_can_id, steer_can_id, encoder_can_id;
  const char* bus{"rio"};
  double encoder_offset_rot{0};
  bool drive_inverted{false}, steer_inverted{false}, encoder_inverted{false};
};

inline constexpr std::array<ModuleConfig, 4> kTestSwerveModules{{
    {"back_left", 1, 3, 2, -0.30, 0.30, 0.0508, 5, 6, 22},
    {"back_right", 4, 6, 5, -0.30, -0.30, 0.0508, 7, 8, 23},
    {"front_left", 8, 10, 9, 0.30, 0.30, 0.0508, 1, 2, 20},
    {"front_right", 11, 13, 12, 0.30, -0.30, 0.0508, 3, 4, 21},
}};

inline Config SwerveConfig(bool simulation = false) {
  Config c;
  c.commissioned = simulation;
  for (std::size_t i = 0; i < kTestSwerveModules.size(); ++i) {
    const auto& module = kTestSwerveModules[i];
    SensorConfig encoder;
    encoder.id = module.encoder_id;
    encoder.name = std::string{module.name} + "_encoder";
    encoder.can_id = module.encoder_can_id;
    encoder.bus = module.bus;
    encoder.offset_rot = module.encoder_offset_rot;
    encoder.inverted = module.encoder_inverted;
    c.sensors.push_back(encoder);
    MotorConfig drive;
    drive.id = module.drive_id;
    drive.name = std::string{module.name} + "_drive";
    drive.can_id = module.drive_can_id;
    drive.bus = module.bus;
    drive.inverted = module.drive_inverted;
    drive.sensor_to_mechanism_ratio = 6.75;
    drive.max_velocity_rps = 20;
    drive.slots[0].p = 0;
    drive.slots[0].v = 0;
    c.motors.push_back(drive);
    MotorConfig steer;
    steer.id = module.steer_id;
    steer.name = std::string{module.name} + "_steer";
    steer.can_id = module.steer_can_id;
    steer.bus = module.bus;
    steer.inverted = module.steer_inverted;
    steer.feedback = Feedback::kRemoteCANcoder;
    steer.feedback_sensor_id = encoder.id;
    steer.rotor_to_sensor_ratio = 12.8;
    steer.continuous_wrap = true;
    steer.supply_limit_a = 20;
    steer.stator_limit_a = 40;
    steer.cruise_velocity_rps = 5;
    steer.acceleration_rps2 = 20;
    steer.slots[0].p = 0;
    c.motors.push_back(steer);
  }
  c.sensors.push_back({7, "drivetrain_imu", SensorKind::kPigeon2, 30, "rio"});
  std::sort(c.sensors.begin(), c.sensors.end(),
            [](const auto& a, const auto& b) { return a.name < b.name; });
  std::sort(c.motors.begin(), c.motors.end(),
            [](const auto& a, const auto& b) { return a.name < b.name; });
  return c;
}
class FakeBackend : public Backend {
 public:
  bool Configure(const Config&) override {
    ++configure_calls;
    events.push_back("configure");
    return configure_ok;
  }
  std::vector<uint16_t> UnavailableDevices() const override {
    return unavailable;
  }
  bool Read(State& s) override {
    for (auto& m : s.motors) {
      m.valid = healthy && m.id != invalid_motor;
      if (const auto it = positions.find(m.id); it != positions.end()) {
        m.position_rot = it->second;
      }
      m.velocity_rps = m.voltage = m.stator_current_a = m.supply_current_a =
          measurement;
      m.position_age_us = m.velocity_age_us = age_us;
      if (const auto it = position_ages.find(m.id); it != position_ages.end()) {
        m.position_age_us = it->second;
      }
      if (const auto it = velocity_ages.find(m.id); it != velocity_ages.end()) {
        m.velocity_age_us = it->second;
      }
    }
    for (auto& sensor : s.sensors) {
      sensor.valid = healthy && sensor.id != invalid_sensor;
      sensor.position_rot = sensor.velocity_rps = measurement;
      sensor.position_age_us = sensor.velocity_age_us = age_us;
      if (const auto it = position_ages.find(sensor.id);
          it != position_ages.end()) {
        sensor.position_age_us = it->second;
      }
    }
    for (auto& input : s.digital_inputs) {
      input.valid = true;
      input.value = true;
    }
    s.health = {.valid = true, .battery_v = 12};
    return healthy;
  }
  bool Apply(std::span<const MotorRequest> requests) override {
    last_requests.assign(requests.begin(), requests.end());
    calls.push_back({Call::kApply, last_requests});
    ++applied;
    // Device-local send failures. last_requests keeps what the gateway asked
    // for, so tests observe the gateway's own neutralization.
    apply_failed.clear();
    const bool fail_all =
        fail_all_after_neutral &&
        std::any_of(requests.begin(), requests.end(), [](const auto& request) {
          return request.mode == Mode::kNeutral;
        });
    for (const auto& request : requests) {
      if (fail_all || std::find(failing_motors.begin(), failing_motors.end(),
                                request.id) != failing_motors.end())
        apply_failed.push_back(request.id);
    }
    return apply_ok;
  }
  std::span<const uint16_t> ApplyFailedDevices() const override {
    return apply_failed;
  }
  bool ApplyOutputs(std::span<const DigitalOutputRequest> digital,
                    std::span<const PwmRequest> pwm) override {
    last_digital_outputs.assign(digital.begin(), digital.end());
    last_pwm_outputs.assign(pwm.begin(), pwm.end());
    return outputs_ok;
  }
  void Neutral() override {
    ++neutral;
    events.push_back("neutral");
    calls.push_back({Call::kNeutral, {}});
  }
  std::optional<ConfigError> last_config_error() const override {
    return config_error;
  }
  bool PollDeviceResets(std::vector<uint16_t>& ids) override {
    ids = resets;
    resets.clear();
    return !ids.empty();
  }
  void BeginRestoreAfterReset(std::span<const uint16_t> ids) override {
    restoring_ids.assign(ids.begin(), ids.end());
    events.push_back("restore");
    calls.push_back({Call::kBeginRestore, {}});
  }
  RestoreStatus PollRestore() override {
    if (pending_ticks-- > 0) return RestoreStatus::kPending;
    if (restore_ok) {
      std::erase_if(unavailable, [&](uint16_t id) {
        return std::find(restoring_ids.begin(), restoring_ids.end(), id) !=
               restoring_ids.end();
      });
    }
    calls.push_back({Call::kRestoreFinished, {}});
    return restore_ok ? RestoreStatus::kDone : RestoreStatus::kFailed;
  }
  // Every output and restore call in order, with the requests each Apply
  // carried, so a test can see what reached a device around its restore.
  struct Call {
    enum Kind { kApply, kNeutral, kBeginRestore, kRestoreFinished } kind;
    std::vector<MotorRequest> requests;
  };
  std::vector<Call> calls;
  int pending_ticks{};
  bool configure_ok{true}, healthy{true}, apply_ok{true}, restore_ok{true};
  bool outputs_ok{true};
  bool fail_all_after_neutral{false};
  std::vector<uint16_t> failing_motors, apply_failed;
  std::vector<DigitalOutputRequest> last_digital_outputs;
  std::vector<PwmRequest> last_pwm_outputs;
  int applied{}, neutral{}, configure_calls{};
  uint16_t invalid_motor{}, invalid_sensor{};
  std::vector<MotorRequest> last_requests{};
  std::vector<std::string> events{};
  std::vector<uint16_t> resets{};
  std::vector<uint16_t> restoring_ids;
  std::vector<uint16_t> unavailable;
  std::optional<ConfigError> config_error{};
  // Reported motor positions by id; others keep theirs. Every motor's and
  // sensor's ages are `age_us`, except a position age (motor or sensor) or a
  // motor velocity age named here. The fake never judges an age itself: the
  // gateway does, for every backend.
  std::map<uint16_t, double> positions;
  // Every motor's velocity, voltage, stator and supply current, and every
  // sensor's position and velocity, valid or not.
  double measurement{};
  uint32_t age_us{};
  std::map<uint16_t, uint32_t> position_ages, velocity_ages;
};
Command Request(const Gateway& g) {
  const auto s = g.snapshot();
  Command c{};
  c.config_id = s.config_id;
  c.boot_id = s.boot_id;
  c.epoch = s.epoch;
  c.observed_time_us = s.sample_time_us;
  c.count = s.motor_count;
  for (std::size_t i = 0; i < c.count; ++i) {
    c.motors[i] = {s.motors[i].id, Mode::kVelocity, 0, 1, 0};
  }
  return c;
}
std::size_t MotorIndex(const Config& config, Feedback feedback) {
  const auto motor = std::find_if(
      config.motors.begin(), config.motors.end(),
      [feedback](const auto& m) { return m.feedback == feedback; });
  return static_cast<std::size_t>(motor - config.motors.begin());
}
TEST(Config, ValidatesFollowersAndFingerprintsTheirSettings) {
  auto c = SwerveConfig(true);
  const auto independent_id = ConfigurationId(c);
  c.motors[1].master_id = c.motors[0].id;
  EXPECT_NO_THROW(Validate(c));
  EXPECT_NE(ConfigurationId(c), independent_id);
  const auto follower_id = ConfigurationId(c);
  c.motors[1].oppose_master = true;
  EXPECT_NE(ConfigurationId(c), follower_id);
  c.motors[1].slots[2].gravity_type = GravityType::kArmCosine;
  const auto gravity_id = ConfigurationId(c);
  c.motors[1].slots[2].gravity_type = GravityType::kElevatorStatic;
  EXPECT_NE(ConfigurationId(c), gravity_id);
  c.motors[1].master_id = c.motors[1].id;
  EXPECT_THROW(Validate(c), std::invalid_argument);
  c.motors[1].master_id = 65535;
  EXPECT_THROW(Validate(c), std::invalid_argument);
  c.motors[1].master_id = c.motors[0].id;
  c.motors[0].bus = "other";
  EXPECT_THROW(Validate(c), std::invalid_argument);
  c = SwerveConfig(true);
  c.motors[1].master_id = c.motors[0].id;
  c.motors[2].master_id = c.motors[1].id;
  EXPECT_THROW(Validate(c), std::invalid_argument);
  c.motors[2].master_id = 0;
  c.motors[0].master_id = c.motors[1].id;
  EXPECT_THROW(Validate(c), std::invalid_argument);
  c = SwerveConfig(true);
  // Intentionally cast 2 to GravityType to verify validation rejects it.
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  c.motors[0].slots[0].gravity_type = static_cast<GravityType>(2);
  EXPECT_THROW(Validate(c), std::invalid_argument);
}

// The TalonFX applies limit-switch settings itself, so both endpoints must
// agree on them: they travel on the config wire and change the config id.
TEST(Config, LimitSwitchesTravelOnTheWireAndFingerprint) {
  auto c = SwerveConfig(true);
  const auto default_id = ConfigurationId(c);
  c.motors[0].forward_limit_switch = LimitSwitch::kNormallyClosed;
  const auto forward_id = ConfigurationId(c);
  EXPECT_NE(forward_id, default_id);
  c.motors[0].reverse_limit_switch = LimitSwitch::kDisabled;
  EXPECT_NE(ConfigurationId(c), forward_id);
  Config decoded;
  ASSERT_TRUE(DeserializeConfig(SerializeConfig(c), decoded));
  EXPECT_EQ(decoded, c);

  // Out-of-range types are rejected by Validate and by the decoder.
  auto bad = c;
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  bad.motors[0].reverse_limit_switch = static_cast<LimitSwitch>(3);
  EXPECT_THROW(Validate(bad), std::invalid_argument);
  // The one byte that differs from a normally open reverse switch is its type.
  auto bytes = SerializeConfig(c);
  auto open = c;
  open.motors[0].reverse_limit_switch = LimitSwitch::kNormallyOpen;
  const auto open_bytes = SerializeConfig(open);
  ASSERT_EQ(bytes.size(), open_bytes.size());
  const auto at = std::mismatch(bytes.begin(), bytes.end(), open_bytes.begin());
  ASSERT_NE(at.first, bytes.end());
  *at.first = 3;
  EXPECT_FALSE(DeserializeConfig(bytes, decoded));
}

TEST(SimBackend, PigeonMountCalibrationReachesIdealBackend) {
  auto config = SwerveConfig(true);
  for (auto& sensor : config.sensors) {
    if (sensor.kind != SensorKind::kPigeon2) continue;
    sensor.offset_rot = 0.125;
    sensor.inverted = true;
  }
  SimBackend backend;
  Gateway gateway{config, backend, 1};
  gateway.Tick(1000, false);
  const auto state = gateway.snapshot();
  for (std::size_t i = 0; i < config.sensors.size(); ++i) {
    if (config.sensors[i].kind != SensorKind::kPigeon2) continue;
    EXPECT_TRUE(state.sensors[i].valid);
    EXPECT_DOUBLE_EQ(state.sensors[i].position_rot, -0.125);
    EXPECT_DOUBLE_EQ(state.sensors[i].velocity_rps, 0.0);
  }
}

TEST(Config, PigeonMountYawRetainsFiniteHalfTurnBounds) {
  auto config = SwerveConfig(true);
  for (auto& sensor : config.sensors) {
    if (sensor.kind != SensorKind::kPigeon2) continue;
    sensor.inverted = true;
    sensor.offset_rot = -0.5;
    EXPECT_NO_THROW(Validate(config));
    sensor.offset_rot = 0.5;
    EXPECT_THROW(Validate(config), std::invalid_argument);
    sensor.offset_rot = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(Validate(config), std::invalid_argument);
  }
}

TEST(Config, RejectsAmbiguousHardwareAndInvalidFeedback) {
  auto c = SwerveConfig(true);
  EXPECT_NO_THROW(Validate(c));
  c.motors[1].can_id = c.motors[0].can_id;
  EXPECT_THROW(Validate(c), std::invalid_argument);
  c = SwerveConfig(true);
  c.motors[0].id = c.motors[1].id;
  EXPECT_THROW(Validate(c), std::invalid_argument);
  c = SwerveConfig(true);
  c.motors[1].feedback_sensor_id = 999;
  EXPECT_THROW(Validate(c), std::invalid_argument);
  c = SwerveConfig(true);
  c.motors[1].bus = "different_bus";
  EXPECT_THROW(Validate(c), std::invalid_argument);
  c = SwerveConfig(true);
  c.motors[0].slots[0].p = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Validate(c), std::invalid_argument);
}
// Feedback command_timeout_us old is invalid, so a status rate with fewer than
// four frames in that time would read as a dropout at every frame: 40 Hz at
// the 100 ms timeout loads, anything slower is refused naming both keys.
// REQ-SAFE-26
TEST(Config, StatusRateBelowFourPerTimeoutIsRefused) {
  auto c = SwerveConfig(true);
  ASSERT_EQ(c.command_timeout_us, 100000u);
  c.status_hz = 40;
  EXPECT_NO_THROW(Validate(c));
  c.status_hz = std::nextafter(40.0, 0.0);
  try {
    Validate(c);
    ADD_FAILURE() << "status_hz just below 40 at 100000 us loaded";
  } catch (const std::invalid_argument& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("status_hz"), std::string::npos) << message;
    EXPECT_NE(message.find("command_timeout_us"), std::string::npos) << message;
  }
}
// One device of every kind, every motor setting class (follower, remote
// CANcoder, soft and switch limits, three gain slots) off its default, and
// the main-processor-only motor settings, which never reach the wire, set.
Config EveryDeviceKindConfig() {
  Config c;
  c.commissioned = true;
  MotorConfig arm;
  arm.id = 1;
  arm.name = "arm";
  arm.can_id = 1;
  arm.inverted = true;
  arm.brake = false;
  arm.feedback = Feedback::kRemoteCANcoder;
  arm.feedback_sensor_id = 10;
  arm.rotor_to_sensor_ratio = 12.8;
  arm.sensor_to_mechanism_ratio = 2;
  arm.soft_limits = true;
  arm.reverse_limit_rot = -0.25;
  arm.forward_limit_rot = 0.25;
  arm.forward_limit_switch = LimitSwitch::kNormallyClosed;
  arm.reverse_limit_switch = LimitSwitch::kDisabled;
  arm.jerk_rps3 = 400;
  for (std::size_t slot = 0; slot < arm.slots.size(); ++slot) {
    const double k = static_cast<double>(slot + 1);
    arm.slots[slot] = {k, k / 2, k / 4, k / 8, k / 16, k / 32, k / 64};
  }
  arm.slots[1].gravity_type = GravityType::kArmCosine;
  arm.absolute_position_frame.value = true;
  arm.motor_type.value = MotorType::kFalcon500;
  MotorConfig follower;
  follower.id = 2;
  follower.name = "arm_follower";
  follower.can_id = 2;
  follower.master_id = arm.id;
  follower.oppose_master = true;
  follower.motor_type.value = MotorType::kFalcon500;
  MotorConfig roller;
  roller.id = 3;
  roller.name = "roller";
  roller.can_id = 3;
  roller.continuous_wrap = true;
  roller.supply_limit_a = 30;
  roller.stator_limit_a = 60;
  roller.max_voltage = 10;
  roller.max_velocity_rps = 90;
  roller.cruise_velocity_rps = 30;
  roller.acceleration_rps2 = 60;
  roller.motor_type.value = MotorType::kKrakenX44;
  c.motors = {arm, follower, roller};
  c.sensors = {
      {10, "arm_encoder", SensorKind::kCANcoder, 20, "rio", true, 0.125},
      {11, "imu", SensorKind::kPigeon2, 30, "rio", false, -0.25}};
  c.digital_inputs = {{20, "beam_break", 0}};
  c.digital_outputs = {{21, "solenoid", 1}};
  c.analog_inputs = {{22, "pressure", 2}};
  c.encoders = {{23, "wheel", EncoderKind::kQuadrature, 2, 3, 2048, 0},
                {24, "turret", EncoderKind::kDutyCycle, 4, -1, 2048, 0.25}};
  c.pwm_outputs = {{25, "servo", 5}};
  return c;
}

// The id the gateway computes for `config` pushed over the config wire: the
// one its States carry and its Accept compares.
uint64_t GatewayConfigId(const Config& config) {
  ConfigAssembler assembler;
  std::string error;
  auto status = ConfigAssembler::Status::kIncomplete;
  for (const auto& chunk : CreateConfigChunks(config)) {
    status = assembler.AddChunk(chunk, error);
  }
  EXPECT_EQ(status, ConfigAssembler::Status::kComplete) << error;
  SimBackend backend;
  Gateway gateway{backend, 1};
  EXPECT_TRUE(gateway.Reconfigure(assembler.config()));
  return gateway.snapshot().config_id;
}

// The main processor (bridge, devices layer) computes the config id from the
// config it parsed, the gateway from the config the wire delivered; one
// function computes both, so they diverge only if it reads a setting the wire
// does not carry (a main-processor-only one included) or the wire carries one
// it does not read. The first: the gateway's id for a config pushed over the
// wire equals the main processor's, commissioned or not. The second: every
// single-bit change to the serialized config that still decodes to another
// valid config changes the id, the commissioned bit among them.
// REQ-CONTRACT-05, REQ-SAFE-20: both ends' id, over every wire setting
TEST(ConfigWire, BothEndsComputeOneIdCoveringEveryWireSetting) {
  const auto config = EveryDeviceKindConfig();
  ASSERT_NO_THROW(Validate(config));
  const uint64_t id = ConfigurationId(config);
  EXPECT_EQ(GatewayConfigId(config), id);
  auto uncommissioned = config;
  uncommissioned.commissioned = false;
  const uint64_t uncommissioned_id = ConfigurationId(uncommissioned);
  EXPECT_NE(uncommissioned_id, id);
  EXPECT_EQ(GatewayConfigId(uncommissioned), uncommissioned_id);

  const auto bytes = SerializeConfig(config);
  std::size_t changed_offsets = 0;
  for (std::size_t offset = 0; offset < bytes.size(); ++offset) {
    bool changed = false;
    for (int bit = 0; bit < 8; ++bit) {
      auto mutated = bytes;
      mutated[offset] ^= static_cast<uint8_t>(1u << bit);
      Config decoded;
      if (!DeserializeConfig(mutated, decoded) || decoded == config) continue;
      try {
        Validate(decoded);
      } catch (const std::invalid_argument&) {
        continue;
      }
      changed = true;
      EXPECT_NE(ConfigurationId(decoded), id)
          << "byte " << offset << " bit " << bit << " of " << bytes.size();
    }
    changed_offsets += changed ? 1 : 0;
  }
  // Counts, text lengths and values at a bound decode to nothing valid when
  // changed; every other byte must, or the sweep proves little.
  EXPECT_GT(changed_offsets, bytes.size() * 3 / 4)
      << changed_offsets << " of " << bytes.size() << " bytes";
}
// Consumers tell consecutive States from a gap by sequence + 1, so every
// tick counts, across a reconfigure too.
TEST(Gateway, SequenceCountsEveryTick) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  EXPECT_EQ(g.snapshot().sequence, 0u);
  g.Tick(1000, false);
  EXPECT_EQ(g.snapshot().sequence, 1u);
  ASSERT_TRUE(g.Reconfigure(SwerveConfig(false)));
  g.Tick(2000, true);
  g.Tick(3000, true);
  EXPECT_EQ(g.snapshot().sequence, 3u);
}

// REQ-SAFE-20, REQ-CONTRACT-05: refused while disabled, uncommissioned or
// mismatched
TEST(Gateway, RequiresEnableCommissioningAndMatchingManifest) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, false);
  EXPECT_FALSE(g.Accept(Request(g), 1, 1000));
  g.Tick(2000, true);
  auto c = Request(g);
  ++c.config_id;
  EXPECT_FALSE(g.Accept(c, 1, 2000));
  EXPECT_EQ(backend.applied, 0);
  EXPECT_TRUE(g.Accept(Request(g), 1, 2000));
  FakeBackend other;
  Gateway uncommissioned{SwerveConfig(false), other, 2};
  uncommissioned.Tick(2000, true);
  EXPECT_FALSE(uncommissioned.Accept(Request(uncommissioned), 1, 2000));
}
// While the robot cannot drive, every node flushes neutral and the bridge
// keeps forwarding the merge. Counting those refusals would bury every real
// rejection under a steady climb; a command that asks for output or carries
// another identity still counts.
TEST(Gateway, NeutralRefusedWhileUnableToDriveIsNotARejection) {
  for (const bool commissioned : {true, false}) {
    SCOPED_TRACE(commissioned ? "disabled" : "enabled, uncommissioned");
    FakeBackend backend;
    Gateway g{SwerveConfig(commissioned), backend, 1};
    g.Tick(1000, !commissioned);
    auto neutral = Request(g);
    for (std::size_t i = 0; i < neutral.count; ++i) {
      neutral.motors[i] = {neutral.motors[i].id};
    }
    for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
      EXPECT_FALSE(g.Accept(neutral, sequence, 1000));
    }
    EXPECT_EQ(g.snapshot().counters.rejected_command_count, 0u);
    auto misidentified = neutral;
    ++misidentified.config_id;
    EXPECT_FALSE(g.Accept(misidentified, 4, 1000));
    EXPECT_EQ(g.snapshot().counters.rejected_command_count, 1u);
    EXPECT_FALSE(g.Accept(Request(g), 5, 1000));
    EXPECT_EQ(g.snapshot().counters.rejected_command_count, 2u);
    EXPECT_EQ(backend.applied, 0);
  }
}
// REQ-SAFE-17: a command with a wrong boot, epoch, sequence or age is rejected
TEST(Gateway, RejectsOldBootEpochSequenceAndDelayedCommands) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  auto command = Request(g);
  EXPECT_TRUE(g.Accept(command, 7, 1000));
  EXPECT_FALSE(g.Accept(command, 7, 1001));
  EXPECT_FALSE(g.Accept(command, 6, 1001));
  auto bad = command;
  ++bad.boot_id;
  EXPECT_FALSE(g.Accept(bad, 8, 1001));
  bad = command;
  ++bad.epoch;
  EXPECT_FALSE(g.Accept(bad, 8, 1001));
  bad = command;
  bad.observed_time_us = 2000;
  EXPECT_FALSE(g.Accept(bad, 8, 1001));
  EXPECT_FALSE(g.Accept(command, 8, 101000));
  g.Tick(101000, true);
  EXPECT_FALSE(g.snapshot().flags & kCommandActive);
  EXPECT_NE(g.snapshot().epoch, command.epoch);
  EXPECT_FALSE(g.Accept(command, 100, 101000));
  EXPECT_TRUE(
      g.Accept(Request(g), 0, 101000));  // A restarted companion can reconnect.
}
// The RoboRIO's loop watchdog arms on command_expiry_us, so it must be the
// first tick time that invalidates the command, counted from the older of its
// acceptance and the State it was built from.
TEST(Gateway, CommandExpiryIsTheFirstTickThatInvalidatesTheCommand) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  EXPECT_EQ(g.command_expiry_us(), 0u);
  ASSERT_TRUE(g.Accept(Request(g), 1, 30000));
  const uint64_t expiry = g.command_expiry_us();
  EXPECT_EQ(expiry, 1000 + g.config().command_timeout_us);
  g.Tick(expiry - 1, true);
  EXPECT_TRUE(g.snapshot().flags & kCommandActive);
  g.Tick(expiry, true);
  EXPECT_FALSE(g.snapshot().flags & kCommandActive);
  EXPECT_EQ(g.command_expiry_us(), 0u);
}
TEST(Gateway, DisableAndTelemetryFailureInvalidateOutstandingCommands) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  auto command = Request(g);
  ASSERT_TRUE(g.Accept(command, 1, 1000));
  g.Tick(2000, false);
  g.Tick(3000, true);
  EXPECT_FALSE(g.Accept(command, 2, 3000));
  ASSERT_TRUE(g.Accept(Request(g), 0, 3000));
  command = Request(g);
  backend.healthy = false;
  g.Tick(4000, true);
  EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
  EXPECT_FALSE(g.snapshot().flags & kCommandActive);
  backend.healthy = true;
  g.Tick(5000, true);
  EXPECT_FALSE(g.Accept(command, 1, 5000));
  EXPECT_TRUE(g.Accept(Request(g), 0, 5000));
}
TEST(Gateway, InvalidMotorFeedbackIsolatesOnlyThatMotor) {
  FakeBackend backend;
  auto config = SwerveConfig(true);
  backend.invalid_motor = config.motors[0].id;
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  ASSERT_TRUE(g.Accept(Request(g), 1, 1000));
  ASSERT_EQ(backend.last_requests.size(), config.motors.size());
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests[1].mode, Mode::kVelocity);
  EXPECT_EQ(g.snapshot().counters.neutralized_motor_count, 1u);
  EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
}

TEST(Gateway, NeutralizedMasterAlsoNeutralizesFollowerRegardlessOfOrder) {
  for (bool master_first : {false, true}) {
    auto config = SwerveConfig(true);
    const std::size_t master = master_first ? 0 : 1;
    const std::size_t follower = master_first ? 1 : 0;
    config.motors[follower].master_id = config.motors[master].id;
    for (int failure = 0; failure < 4; ++failure) {
      FakeBackend backend;
      if (failure == 0) backend.invalid_motor = config.motors[master].id;
      if (failure == 1) {
        config.motors[master].feedback = Feedback::kRemoteCANcoder;
        config.motors[master].feedback_sensor_id = config.sensors[0].id;
        backend.invalid_sensor = config.sensors[0].id;
      }
      Gateway gateway{config, backend, 1};
      gateway.Tick(1000, true);
      auto command = Request(gateway);
      if (failure == 2) command.motors[master].mode = Mode::kNeutral;
      if (failure == 3) command.motors[master].demand = 1e9;
      ASSERT_TRUE(gateway.Accept(command, 1, 1000));
      EXPECT_EQ(backend.last_requests[master].mode, Mode::kNeutral);
      EXPECT_EQ(backend.last_requests[follower].mode, Mode::kNeutral);
    }
  }
}

TEST(Gateway, InvalidRemoteFeedbackIsolatesOnlyItsMotor) {
  FakeBackend backend;
  auto config = SwerveConfig(true);
  std::size_t steer_index = 0;
  while (config.motors[steer_index].feedback != Feedback::kRemoteCANcoder) {
    ++steer_index;
  }
  backend.invalid_sensor = config.motors[steer_index].feedback_sensor_id;
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  ASSERT_TRUE(g.Accept(Request(g), 1, 1000));
  for (std::size_t i = 0; i < config.motors.size(); ++i) {
    EXPECT_EQ(backend.last_requests[i].mode,
              i == steer_index ? Mode::kNeutral : Mode::kVelocity);
  }
  EXPECT_EQ(g.snapshot().counters.neutralized_motor_count, 1u);
  EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
}

// The bench's CAN silences froze the feedback of its one Talon FX for
// 64-66 ms (TODO.md RESET-3). Feedback younger than command_timeout_us still
// counts: through the gap the command in force keeps driving, with no
// neutral, no fault and no new epoch, and a command built during it drives.
// REQ-SAFE-22
TEST(Gateway, LateFeedbackUnderTheTimeoutKeepsDriving) {
  Config config;
  config.commissioned = true;
  MotorConfig motor;
  motor.id = 1;
  motor.name = "motor";
  motor.can_id = 1;
  motor.max_velocity_rps = 20;
  config.motors.push_back(motor);
  ASSERT_EQ(config.command_timeout_us, 100000u);
  FakeBackend backend;
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  ASSERT_TRUE(g.Accept(Request(g), 1, 1000));
  const auto epoch = g.snapshot().epoch;
  const int applied = backend.applied;
  const int neutrals = backend.neutral;
  for (uint64_t now = 6000; now <= 66000; now += 5000) {
    backend.age_us = static_cast<uint32_t>(now - 1000);
    g.Tick(now, true);
    const auto s = g.snapshot();
    ASSERT_TRUE(s.motors[0].valid) << now;
    EXPECT_TRUE(s.flags & kCommandActive) << now;
    EXPECT_FALSE(s.flags & kHardwareFault) << now;
    EXPECT_EQ(s.epoch, epoch) << now;
  }
  EXPECT_EQ(backend.applied, applied);
  EXPECT_EQ(backend.neutral, neutrals);
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kVelocity);
  ASSERT_TRUE(g.Accept(Request(g), 2, 66000));
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kVelocity);
  EXPECT_EQ(g.snapshot().counters.neutralized_motor_count, 0u);
}

// A position or velocity sample command_timeout_us old, of a motor or of the
// remote CANcoder a motor steers by, is invalid whatever the backend says:
// that motor goes neutral and is counted, the rest drive, and with other
// devices still valid it is no hardware fault.
// REQ-SAFE-22
TEST(Gateway, FeedbackAtTheTimeoutIsolatesItsMotor) {
  const auto config = SwerveConfig(true);
  const auto drive = MotorIndex(config, Feedback::kRotor);
  const auto steer = MotorIndex(config, Feedback::kRemoteCANcoder);
  const uint16_t drive_id = config.motors[drive].id;
  const uint16_t encoder_id = config.motors[steer].feedback_sensor_id;
  for (int row = 0; row < 3; ++row) {
    SCOPED_TRACE(row == 0   ? "motor position age"
                 : row == 1 ? "motor velocity age"
                            : "remote CANcoder position age");
    FakeBackend backend;
    Gateway g{config, backend, 1};
    g.Tick(1000, true);
    const uint32_t bound = config.command_timeout_us;
    if (row == 0) backend.position_ages[drive_id] = bound;
    if (row == 1) backend.velocity_ages[drive_id] = bound;
    if (row == 2) backend.position_ages[encoder_id] = bound;
    const std::size_t isolated = row == 2 ? steer : drive;
    g.Tick(6000, true);
    ASSERT_TRUE(g.Accept(Request(g), 1, 6000));
    for (std::size_t i = 0; i < config.motors.size(); ++i) {
      EXPECT_EQ(backend.last_requests[i].mode,
                i == isolated ? Mode::kNeutral : Mode::kVelocity)
          << config.motors[i].name;
    }
    const auto s = g.snapshot();
    EXPECT_EQ(s.counters.neutralized_motor_count, 1u);
    EXPECT_FALSE(s.flags & kHardwareFault);
  }
}

// The swerve config plus a follower of its first rotor-feedback motor.
struct FollowedSwerve {
  Config config = SwerveConfig(true);
  std::size_t drive = MotorIndex(config, Feedback::kRotor);
  std::size_t steer = MotorIndex(config, Feedback::kRemoteCANcoder);
  std::size_t follower = config.motors.size();
  FollowedSwerve() {
    auto motor = config.motors[drive];
    motor.id = 100;
    motor.name = "drive_follower";
    motor.can_id = 60;
    motor.master_id = config.motors[drive].id;
    config.motors.push_back(motor);
  }
};

// A motor whose feedback goes invalid while a command drives it is sent
// neutral in the tick that finds it invalid, with its follower, without
// waiting for the next command; the other motors keep their demands, no
// hardware fault is raised and the command stays in force. That neutral is
// not counted in neutralized_motor_count (which counts sanitized demands of
// accepted commands). FeedbackFresh makes an age one microsecond under
// command_timeout_us fresh and an age at it invalid, so the first stops
// nothing.
// REQ-SAFE-22
TEST(Gateway, InvalidFeedbackNeutralizesAtThatTick) {
  const FollowedSwerve swerve;
  const auto& config = swerve.config;
  const uint32_t bound = config.command_timeout_us;
  for (int row = 0; row < 3; ++row) {
    SCOPED_TRACE(row == 0   ? "motor position age"
                 : row == 1 ? "remote CANcoder position age"
                            : "backend reports it invalid");
    const uint16_t aged = row == 0
                              ? config.motors[swerve.drive].id
                              : config.motors[swerve.steer].feedback_sensor_id;
    FakeBackend backend;
    Gateway g{config, backend, 1};
    g.Tick(1000, true);
    ASSERT_TRUE(g.Accept(Request(g), 1, 1000));
    const auto epoch = g.snapshot().epoch;
    const int neutrals = backend.neutral;
    int applied = backend.applied;
    if (row < 2) {
      backend.position_ages[aged] = bound - 1;
      g.Tick(6000, true);
      EXPECT_EQ(backend.applied, applied);
      for (const auto& request : backend.last_requests) {
        EXPECT_EQ(request.mode, Mode::kVelocity) << request.id;
      }
      backend.position_ages[aged] = bound;
    } else {
      backend.invalid_motor = config.motors[swerve.drive].id;
    }
    g.Tick(7000, true);
    EXPECT_EQ(backend.applied, applied + 1);
    for (std::size_t i = 0; i < config.motors.size(); ++i) {
      const bool isolated = row == 1
                                ? i == swerve.steer
                                : i == swerve.drive || i == swerve.follower;
      EXPECT_EQ(backend.last_requests[i].mode,
                isolated ? Mode::kNeutral : Mode::kVelocity)
          << config.motors[i].name;
    }
    const auto s = g.snapshot();
    EXPECT_FALSE(s.flags & kHardwareFault);
    EXPECT_TRUE(s.flags & kCommandActive);
    EXPECT_EQ(s.epoch, epoch);
    EXPECT_EQ(s.counters.neutralized_motor_count, 0u);
    EXPECT_EQ(backend.neutral, neutrals);
    applied = backend.applied;
    g.Tick(8000, true);  // Already neutral: nothing more to send.
    EXPECT_EQ(backend.applied, applied);
  }
}

// With no command in force (expired, or disabled), a motor whose feedback
// goes invalid sends no Apply: the gateway still holds the last command's
// demands, and the tick's neutral pass must not resend them; Tick's
// Neutral() already stops every output.
// REQ-SAFE-22
TEST(Gateway, InvalidFeedbackWithNoCommandInForceAppliesNothing) {
  const FollowedSwerve swerve;
  const auto& config = swerve.config;
  for (const bool disable : {false, true}) {
    SCOPED_TRACE(disable ? "disabled" : "expired");
    FakeBackend backend;
    Gateway g{config, backend, 1};
    g.Tick(1000, true);
    ASSERT_TRUE(g.Accept(Request(g), 1, 1000));
    const int applied = backend.applied;
    const uint64_t idle = disable ? 6000 : 1000 + config.command_timeout_us;
    g.Tick(idle, !disable);
    ASSERT_FALSE(g.snapshot().flags & kCommandActive);
    backend.position_ages[config.motors[swerve.drive].id] =
        config.command_timeout_us;
    const int neutrals = backend.neutral;
    g.Tick(idle + 5000, !disable);
    ASSERT_FALSE(g.snapshot().motors[swerve.drive].valid);
    EXPECT_EQ(backend.applied, applied);
    EXPECT_GT(backend.neutral, neutrals);
  }
}

// A motor whose feedback was invalid drives again only on a command whose
// observed State shows it valid: neither the command in force when it went
// invalid nor one built on a State that showed it invalid moves it once its
// feedback returns, while the other motors drive on both; the first command
// observing a State with it valid drives it.
// REQ-SAFE-22
TEST(Gateway, RecoveredMotorNeedsALaterObservation) {
  const FollowedSwerve swerve;
  const auto& config = swerve.config;
  for (int row = 0; row < 2; ++row) {
    SCOPED_TRACE(row == 0 ? "motor position age"
                          : "remote CANcoder position age");
    const uint16_t aged = row == 0
                              ? config.motors[swerve.drive].id
                              : config.motors[swerve.steer].feedback_sensor_id;
    FakeBackend backend;
    Gateway g{config, backend, 1};
    const auto expect_isolated = [&](const char* phase, bool neutral) {
      SCOPED_TRACE(phase);
      for (std::size_t i = 0; i < config.motors.size(); ++i) {
        const bool isolated = row == 1
                                  ? i == swerve.steer
                                  : i == swerve.drive || i == swerve.follower;
        EXPECT_EQ(backend.last_requests[i].mode,
                  neutral && isolated ? Mode::kNeutral : Mode::kVelocity)
            << config.motors[i].name;
      }
    };
    g.Tick(1000, true);
    const auto before = Request(g);
    ASSERT_TRUE(g.Accept(before, 1, 1000));
    backend.position_ages[aged] = config.command_timeout_us;
    g.Tick(6000, true);
    const auto during = Request(g);
    expect_isolated("invalid", true);
    backend.position_ages.erase(aged);
    const int applied = backend.applied;
    g.Tick(11000, true);
    ASSERT_TRUE(g.snapshot().flags & kCommandActive);
    EXPECT_EQ(backend.applied, applied);
    expect_isolated("recovered, command in force", true);
    ASSERT_TRUE(g.Accept(before, 2, 11000));
    expect_isolated("command observed before it went invalid", true);
    ASSERT_TRUE(g.Accept(during, 3, 11000));
    expect_isolated("command observed while it was invalid", true);
    ASSERT_TRUE(g.Accept(Request(g), 4, 11000));
    expect_isolated("command observed after it recovered", false);
  }
}

// Every motor and CAN sensor command_timeout_us old is total loss, though the
// backend reports each sample valid: the State carries the hardware fault,
// the epoch advances and outputs go neutral, so a command built before the
// loss never drives after the feedback returns.
// REQ-SAFE-22, REQ-SAFE-18
TEST(Gateway, EveryDeviceAtTheTimeoutIsTotalLoss) {
  FakeBackend backend;
  const auto config = SwerveConfig(true);
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  const auto before = Request(g);
  ASSERT_TRUE(g.Accept(before, 1, 1000));
  const auto epoch = g.snapshot().epoch;
  const int neutrals = backend.neutral;
  backend.age_us = config.command_timeout_us;
  g.Tick(6000, true);
  const auto s = g.snapshot();
  EXPECT_TRUE(s.flags & kHardwareFault);
  EXPECT_FALSE(s.flags & kCommandActive);
  EXPECT_GT(s.epoch, epoch);
  EXPECT_GT(backend.neutral, neutrals);
  backend.age_us = 0;
  g.Tick(11000, true);
  EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
  EXPECT_FALSE(g.Accept(before, 2, 11000));
  EXPECT_TRUE(g.Accept(Request(g), 3, 11000));
}

// Total feedback loss advances the epoch whether or not a command is in
// force. Enabled with none in force (just enabled, or idle past the timeout),
// a request built from the State before the loss and arriving once the
// feedback returns, still inside command_timeout_us, must not drive on that
// pre-loss observation.
// REQ-SAFE-18
TEST(Gateway, TotalLossWhileInactiveAdvancesTheEpoch) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  ASSERT_FALSE(g.snapshot().flags & kCommandActive);
  const auto before = Request(g);
  backend.healthy = false;
  g.Tick(6000, true);
  ASSERT_TRUE(g.snapshot().flags & kHardwareFault);
  EXPECT_NE(g.snapshot().epoch, before.epoch);
  backend.healthy = true;
  g.Tick(11000, true);
  ASSERT_FALSE(g.snapshot().flags & kHardwareFault);
  EXPECT_FALSE(g.Accept(before, 1, 11000));
  EXPECT_EQ(backend.applied, 0);
  EXPECT_TRUE(g.Accept(Request(g), 2, 11000));
}

// Total loss is judged on the backend's samples, before the gateway masks a
// follower of an invalid master or the motor a reset CANcoder feeds. With
// such a pair as the only CAN devices, an invalid master, or a reset
// CANcoder, leaves the other sample valid at that judgment: the dependent is
// published invalid, but there is no hardware fault and no new epoch.
// SimHardwareTotalLoss.AMaskedDependentIsNotTotalLoss holds SimHardware to
// the same two scenarios.
// REQ-SAFE-22, REQ-DEV-07
TEST(Gateway, AMaskedDependentIsNotTotalLoss) {
  for (const bool remote : {false, true}) {
    SCOPED_TRACE(remote ? "remote CANcoder reset" : "master invalid");
    Config config;
    config.commissioned = true;
    MotorConfig first;
    first.id = 1;
    first.name = remote ? "remote" : "master";
    first.can_id = 1;
    first.max_velocity_rps = 20;
    if (remote) {
      config.sensors.push_back({2, "angle", SensorKind::kCANcoder, 2, "rio"});
      first.feedback = Feedback::kRemoteCANcoder;
      first.feedback_sensor_id = 2;
      config.motors.push_back(first);
    } else {
      MotorConfig follower = first;
      follower.id = 2;
      follower.name = "follower";
      follower.can_id = 2;
      follower.master_id = first.id;
      config.motors = {first, follower};
    }
    FakeBackend backend;
    Gateway g{config, backend, 1};
    g.Tick(1000, true);
    ASSERT_TRUE(g.Accept(Request(g), 1, 1000));
    const auto epoch = g.snapshot().epoch;
    if (remote) {
      backend.resets = {2};
    } else {
      backend.invalid_motor = 1;
    }
    g.Tick(6000, true);
    const auto s = g.snapshot();
    EXPECT_FALSE(s.motors[remote ? 0 : 1].valid);  // the masked dependent
    EXPECT_FALSE(s.flags & kHardwareFault);
    EXPECT_EQ(s.epoch, epoch);
  }
}

TEST(Gateway, RejectsWholeTransactionBeforeApplyingAnyMotor) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  for (int invalid = 0; invalid < 6; ++invalid) {
    auto c = Request(g);
    auto& m = c.motors[c.count - 1];
    if (invalid == 0) m.id = 65535;
    if (invalid == 1) m.id = c.motors[0].id;
    if (invalid == 2) m.slot = 3;
    if (invalid == 3) m.slot = 255;
    if (invalid == 4) m.mode = static_cast<Mode>(255);
    if (invalid == 5) --c.count;
    EXPECT_FALSE(g.Accept(c, 1, 1000));
  }
  EXPECT_EQ(backend.applied, 0);
}
// REQ-SAFE-21: outputs are neutralized before devices are reconfigured
TEST(Gateway, NeutralizesOutputsBeforeReconfiguringDevices) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, false);
  backend.events
      .clear();  // Tick neutralizes while disabled; isolate Reconfigure.
  auto cfg = SwerveConfig(true);
  cfg.motors[0].slots[0].p = 1.0;  // Genuine change, not an idempotent push.
  ASSERT_TRUE(g.Reconfigure(cfg));
  ASSERT_GE(backend.events.size(), 2u);
  EXPECT_EQ(backend.events[0], "neutral");
  EXPECT_EQ(backend.events[1], "configure");
  // A failed backend configuration neutralizes again, so no stale drive
  // persists after the failed device change.
  backend.configure_ok = false;
  backend.events.clear();
  auto bad = cfg;
  bad.motors[0].slots[0].p = 2.0;
  EXPECT_FALSE(g.Reconfigure(bad));
  ASSERT_GE(backend.events.size(), 3u);
  EXPECT_EQ(backend.events[0], "neutral");
  EXPECT_EQ(backend.events[1], "configure");
  EXPECT_EQ(backend.events[2], "neutral");
}
// REQ-SAFE-21: an identical push succeeds with no device traffic
TEST(Gateway, RepeatedConfigurationPushIsIdempotent) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, false);
  const int configures = backend.configure_calls;
  const int neutrals = backend.neutral;
  const auto epoch = g.snapshot().epoch;
  const auto config_id = g.snapshot().config_id;
  // Identical push: succeeds with no device traffic and no epoch change, so a
  // retried push never re-blocks on Phoenix configuration calls.
  EXPECT_TRUE(g.Reconfigure(SwerveConfig(true)));
  EXPECT_EQ(backend.configure_calls, configures);
  EXPECT_EQ(backend.neutral, neutrals);
  EXPECT_EQ(g.snapshot().epoch, epoch);
  EXPECT_EQ(g.snapshot().config_id, config_id);
  EXPECT_TRUE(g.configured());
  // A genuine change still applies exactly once and invalidates commands.
  auto changed = SwerveConfig(true);
  changed.motors[0].slots[0].p = 1.0;
  EXPECT_TRUE(g.Reconfigure(changed));
  EXPECT_EQ(backend.configure_calls, configures + 1);
  EXPECT_NE(g.snapshot().epoch, epoch);
}
// REQ-SAFE-21: a config change applies only while disabled
TEST(Gateway, ReconfigurationRequiresDisabledOperation) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);  // Enabled: hardware configuration is refused.
  auto changed = SwerveConfig(true);
  changed.motors[0].slots[0].p = 1.0;
  const int configures = backend.configure_calls;
  const int neutrals = backend.neutral;
  const std::size_t events = backend.events.size();
  const auto epoch = g.snapshot().epoch;
  EXPECT_FALSE(g.Reconfigure(changed));
  // The refused interlock touches nothing: no device traffic, no neutralize
  // glitch on live outputs, no epoch or fault/configured change.
  EXPECT_EQ(backend.configure_calls, configures);
  EXPECT_EQ(backend.neutral, neutrals);
  EXPECT_EQ(backend.events.size(), events);
  EXPECT_EQ(g.snapshot().epoch, epoch);
  EXPECT_TRUE(g.configured());
  EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
  // Disabled, the same push applies.
  g.Tick(2000, false);
  EXPECT_TRUE(g.Reconfigure(changed));
  EXPECT_EQ(backend.configure_calls, configures + 1);
}
// REQ-SAFE-23: repeated global apply failures latch a fault that re-enable
// clears
TEST(Gateway, ApplyFailureRetriesThenLatchesAndReenableClears) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  backend.apply_ok = false;
  EXPECT_FALSE(g.Accept(Request(g), 1, 1000));
  EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
  backend.apply_ok = true;
  EXPECT_TRUE(g.Accept(Request(g), 2, 1000));
  backend.apply_ok = false;
  for (uint32_t i = 0; i < Gateway::kConsecutiveApplyFailureLimit; ++i) {
    EXPECT_FALSE(g.Accept(Request(g), i + 3, 1000));
  }
  EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
  EXPECT_EQ(g.snapshot().counters.apply_failure_count, 4u);
  backend.apply_ok = true;
  EXPECT_FALSE(g.Accept(Request(g), 7, 1000));
  g.Tick(2000, false);
  g.Tick(3000, true);
  EXPECT_TRUE(g.Accept(Request(g), 8, 3000));
}

TEST(Gateway, IdenticalEnabledConfigPreservesLatchedApplyFault) {
  for (bool reconfigure_disabled : {false, true}) {
    SCOPED_TRACE(reconfigure_disabled);
    FakeBackend backend;
    const auto config = SwerveConfig(true);
    Gateway g{config, backend, 1};
    g.Tick(1000, true);
    backend.apply_ok = false;
    for (uint32_t i = 0; i < Gateway::kConsecutiveApplyFailureLimit; ++i) {
      EXPECT_FALSE(g.Accept(Request(g), i + 1, 1000));
    }
    ASSERT_TRUE(g.snapshot().flags & kHardwareFault);
    const int applied = backend.applied;
    const int configures = backend.configure_calls;
    const auto epoch = g.snapshot().epoch;
    EXPECT_FALSE(g.Reconfigure(config));
    EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
    EXPECT_EQ(g.snapshot().epoch, epoch);
    EXPECT_EQ(backend.configure_calls, configures);
    backend.apply_ok = true;
    EXPECT_FALSE(g.Accept(Request(g), 10, 1000));
    EXPECT_EQ(backend.applied, applied);

    g.Tick(2000, false);
    if (reconfigure_disabled) {
      EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
      EXPECT_TRUE(g.Reconfigure(config));
      EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
    }
    g.Tick(3000, true);
    EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
    EXPECT_TRUE(g.Accept(Request(g), 11, 3000));
  }
}
TEST(Wire, ExplicitEncodingRoundTripsAndRejectsMalformedFrames) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  auto c = Request(g);
  c.digital_output_count = 1;
  c.digital_outputs[0] = {4, true};
  c.pwm_output_count = 1;
  c.pwm_outputs[0] = {7, 0.85};
  std::array<uint8_t, protocol::kMaxPayloadSize> bytes{};
  const auto size = Encode(c, bytes);
  Command decoded;
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), size}, decoded));
  EXPECT_EQ(decoded, c);
  for (std::size_t i = 0; i < size; ++i) {
    EXPECT_FALSE(Decode(std::span<const uint8_t>{bytes.data(), i}, decoded));
  }
  EXPECT_FALSE(
      Decode(std::span<const uint8_t>{bytes.data(), size + 1}, decoded));
  bytes[32] = 255;
  EXPECT_FALSE(Decode(std::span<const uint8_t>{bytes.data(), size}, decoded));

  // Full State with every device kind
  State state = g.snapshot();
  state.motor_count = kMaxMotors;
  state.sensor_count = kMaxSensors;
  state.digital_input_count = kMaxDigitalInputs;
  state.digital_output_count = kMaxDigitalOutputs;
  state.analog_input_count = kMaxAnalogInputs;
  state.encoder_count = kMaxEncoders;
  state.pwm_output_count = kMaxPwmOutputs;
  state.health = {true, 9.2, true, 0.73};
  state.counters = {1, 2, 3, 4};
  state.sequence = 0x0102030405060708;
  state.build = {0x0123456789abcdefull, GatewayProgram::kRoboRio, true};

  for (std::size_t i = 0; i < kMaxMotors; ++i) {
    const auto x = static_cast<double>(i);
    state.motors[i] = {
        static_cast<uint16_t>(i + 1),   true, 1.0 + x,    2.0 + x, 11.5, 25.0,
        123 + static_cast<uint32_t>(i), 456,  0x80000001u};
  }
  for (std::size_t i = 0; i < kMaxSensors; ++i) {
    state.sensors[i] = {static_cast<uint16_t>(50 + i),
                        true,
                        0.25 * static_cast<double>(i),
                        1.5,
                        789,
                        1011};
  }
  for (std::size_t i = 0; i < kMaxDigitalInputs; ++i) {
    state.digital_inputs[i] = {static_cast<uint16_t>(100 + i), true,
                               (i % 2) == 0};
  }
  for (std::size_t i = 0; i < kMaxDigitalOutputs; ++i) {
    state.digital_outputs[i] = {static_cast<uint16_t>(150 + i), true,
                                (i % 2) == 1};
  }
  for (std::size_t i = 0; i < kMaxAnalogInputs; ++i) {
    state.analog_inputs[i] = {static_cast<uint16_t>(200 + i), true,
                              3.3 * static_cast<double>(i), 50};
  }
  for (std::size_t i = 0; i < kMaxEncoders; ++i) {
    state.encoders[i] = {static_cast<uint16_t>(250 + i), true,
                         10.0 * static_cast<double>(i), 5.0, 60};
  }
  for (std::size_t i = 0; i < kMaxPwmOutputs; ++i) {
    state.pwm_outputs[i] = {static_cast<uint16_t>(300 + i), true, 0.75};
  }

  const auto state_size = Encode(state, bytes);
  // The same per-device widths the payload-budget static_asserts in messages.cc
  // are built from, so raising a device cap moves both together or neither.
  constexpr std::size_t kWidestState =
      70 + 14 + 34 + kMaxMotors * 45 + kMaxSensors * 27 +
      kMaxDigitalInputs * 4 + kMaxDigitalOutputs * 4 + kMaxAnalogInputs * 15 +
      kMaxEncoders * 23 + kMaxPwmOutputs * 11;
  ASSERT_EQ(state_size, kWidestState);
  ASSERT_LE(state_size, protocol::kMaxPayloadSize);
  State copy;
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), state_size}, copy));
  EXPECT_EQ(copy, state);

  for (std::size_t i = 0; i < state_size; ++i) {
    EXPECT_FALSE(Decode(std::span<const uint8_t>{bytes.data(), i}, copy));
  }
}

TEST(ConfigWire, ChunksAndAssemblesConfigLosslessly) {
  auto cfg = SwerveConfig(false);
  cfg.motors[1].master_id = cfg.motors[0].id;
  cfg.motors[1].oppose_master = true;
  cfg.motors[1].slots[2].gravity_type = GravityType::kArmCosine;
  for (auto& sensor : cfg.sensors) {
    if (sensor.kind != SensorKind::kPigeon2) continue;
    sensor.offset_rot = 0.125;
    sensor.inverted = true;
  }
  auto chunks = CreateConfigChunks(cfg);
  EXPECT_GT(chunks.size(), 0u);
  for (const auto& ch : chunks) {
    EXPECT_LE(ch.size(), protocol::kMaxPayloadSize);
  }

  ConfigAssembler assembler;
  std::string err;
  ConfigAssembler::Status status = ConfigAssembler::Status::kIncomplete;
  for (const auto& ch : chunks) {
    status = assembler.AddChunk(ch, err);
  }
  EXPECT_EQ(status, ConfigAssembler::Status::kComplete);
  EXPECT_EQ(assembler.config(), cfg);
  EXPECT_EQ(assembler.config_id(), ConfigurationId(cfg));
}

TEST(ConfigWire, FullyCappedSerializationMatchesAssemblyLimit) {
  Config config;
  config.motors.resize(kMaxMotors);
  config.sensors.resize(kMaxSensors);
  config.digital_inputs.resize(kMaxDigitalInputs);
  config.digital_outputs.resize(kMaxDigitalOutputs);
  config.analog_inputs.resize(kMaxAnalogInputs);
  config.encoders.resize(kMaxEncoders);
  config.pwm_outputs.resize(kMaxPwmOutputs);
  uint16_t next_id = 1;
  const auto name_devices = [&](auto& devices) {
    for (auto& device : devices) {
      device.id = next_id++;
      const auto suffix = std::to_string(device.id);
      device.name = std::string(63 - suffix.size(), 'n') + suffix;
    }
  };
  name_devices(config.motors);
  name_devices(config.sensors);
  name_devices(config.digital_inputs);
  name_devices(config.digital_outputs);
  name_devices(config.analog_inputs);
  name_devices(config.encoders);
  name_devices(config.pwm_outputs);
  int next_can_id = 0;
  for (auto& motor : config.motors) {
    motor.bus.assign(63, 'b');
    motor.can_id = next_can_id++;
  }
  for (auto& sensor : config.sensors) {
    sensor.bus.assign(63, 'b');
    sensor.can_id = next_can_id++;
  }
  int next_dio = 0;
  for (auto& input : config.digital_inputs) input.dio = next_dio++;
  for (auto& output : config.digital_outputs) output.dio = next_dio++;
  for (auto& encoder : config.encoders) {
    encoder.dio_a = next_dio++;
    encoder.dio_b = next_dio++;
  }
  for (std::size_t i = 0; i < config.analog_inputs.size(); ++i) {
    config.analog_inputs[i].channel = static_cast<int>(i);
  }
  for (std::size_t i = 0; i < config.pwm_outputs.size(); ++i) {
    config.pwm_outputs[i].channel = static_cast<int>(i);
  }
  ASSERT_NO_THROW(Validate(config));
  EXPECT_EQ(SerializeConfig(config).size(), kMaxSerializedConfigSize);

  const auto chunks = CreateConfigChunks(config);
  ConfigAssembler assembler;
  std::string error;
  for (std::size_t i = chunks.size(); i > 0; --i) {
    EXPECT_EQ(assembler.AddChunk(chunks[i - 1], error),
              i == 1 ? ConfigAssembler::Status::kComplete
                     : ConfigAssembler::Status::kIncomplete);
  }
  EXPECT_EQ(assembler.config(), config);
}

// REQ-CONTRACT-04: oversized chunk metadata is rejected before allocation
TEST(ConfigWire, RejectsUnboundedChunkMetadataAndRecovers) {
  const auto config = SwerveConfig(false);
  const auto chunks = CreateConfigChunks(config, 200);
  ASSERT_GT(chunks.size(), 1u);
  ConfigChunkHeader original;
  std::memcpy(&original, chunks.front().data(), sizeof(original));
  std::vector<ConfigChunkHeader> invalid;
  auto header = original;
  header.total_bytes = std::numeric_limits<uint32_t>::max();
  invalid.push_back(header);
  header = original;
  header.chunk_count = std::numeric_limits<uint16_t>::max();
  invalid.push_back(header);
  header = original;
  header.chunk_index = header.chunk_count;
  invalid.push_back(header);
  header = original;
  header.total_bytes = header.chunk_bytes - 1;
  header.chunk_count = 1;
  header.chunk_index = 0;
  invalid.push_back(header);
  header = original;
  header.total_bytes = header.chunk_bytes - 1;
  header.chunk_count = 2;
  header.chunk_index = 1;
  invalid.push_back(header);
  header = original;
  header.total_bytes = header.chunk_bytes + 1;
  header.chunk_count = 3;
  header.chunk_index = 1;
  invalid.push_back(header);
  header = original;
  header.chunk_bytes = kMaxConfigChunkData + 1;
  invalid.push_back(header);
  header = original;
  header.chunk_bytes = 0;
  invalid.push_back(header);

  for (const auto& bad_header : invalid) {
    SCOPED_TRACE(::testing::Message() << "bytes=" << bad_header.total_bytes
                                      << " count=" << bad_header.chunk_count
                                      << " index=" << bad_header.chunk_index
                                      << " payload=" << bad_header.chunk_bytes);
    std::vector<uint8_t> bad_chunk(sizeof(ConfigChunkHeader) +
                                   bad_header.chunk_bytes);
    std::memcpy(bad_chunk.data(), &bad_header, sizeof(bad_header));
    ConfigAssembler assembler;
    std::string error;
    // These headers must be rejected before starting an allocation-backed
    // transfer, rather than eventually failing CRC or deserialization.
    static_assert(
        noexcept(assembler.AddChunk(std::span<const uint8_t>{}, error)));
    EXPECT_EQ(assembler.AddChunk(bad_chunk, error),
              ConfigAssembler::Status::kInvalidChunk);
    for (std::size_t i = 0; i < chunks.size(); ++i) {
      EXPECT_EQ(assembler.AddChunk(chunks[i], error),
                i + 1 == chunks.size() ? ConfigAssembler::Status::kComplete
                                       : ConfigAssembler::Status::kIncomplete);
    }
    EXPECT_EQ(assembler.config(), config);
  }
}

TEST(ConfigWire, RejectsChangedTransferTotalsAndRecovers) {
  const auto config = SwerveConfig(false);
  const auto chunks = CreateConfigChunks(config, 200);
  ASSERT_GT(chunks.size(), 2u);
  ConfigAssembler assembler;
  std::string error;
  ASSERT_EQ(assembler.AddChunk(chunks.front(), error),
            ConfigAssembler::Status::kIncomplete);
  auto changed = chunks[1];
  ConfigChunkHeader header;
  std::memcpy(&header, changed.data(), sizeof(header));
  ++header.total_bytes;
  std::memcpy(changed.data(), &header, sizeof(header));
  EXPECT_EQ(assembler.AddChunk(changed, error),
            ConfigAssembler::Status::kInvalidChunk);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    EXPECT_EQ(assembler.AddChunk(chunks[i], error),
              i + 1 == chunks.size() ? ConfigAssembler::Status::kComplete
                                     : ConfigAssembler::Status::kIncomplete);
  }
  EXPECT_EQ(assembler.config(), config);
}

TEST(ConfigWire, RejectsCeilingViolations) {
  auto cfg = SwerveConfig(false);
  cfg.motors[0].stator_limit_a = 150.0;
  auto res = CheckCeilings(cfg);
  EXPECT_FALSE(res.ok);

  auto chunks = CreateConfigChunks(cfg);
  ConfigAssembler assembler;
  std::string err;
  ConfigAssembler::Status status = ConfigAssembler::Status::kIncomplete;
  for (const auto& ch : chunks) {
    status = assembler.AddChunk(ch, err);
  }
  EXPECT_EQ(status, ConfigAssembler::Status::kCeilingExceeded);
}

// Chunk headers and ACKs are little-endian on the wire whatever the host's
// byte order, like every other payload (docs/DATA_FLOW.md).
TEST(ConfigWire, ChunkHeaderAndAckAreLittleEndian) {
  const auto config = SwerveConfig(true);
  const auto chunks = CreateConfigChunks(config);
  ASSERT_FALSE(chunks.empty());
  const auto id = ConfigurationId(config);
  const auto& first = chunks.front();
  ASSERT_GE(first.size(), sizeof(ConfigChunkHeader));
  for (int i = 0; i < 8; ++i)
    EXPECT_EQ(first[i], static_cast<uint8_t>(id >> (8 * i)));
  const auto total = static_cast<uint32_t>(SerializeConfig(config).size());
  for (int i = 0; i < 4; ++i)
    EXPECT_EQ(first[12 + i], static_cast<uint8_t>(total >> (8 * i)));
  EXPECT_EQ(first[16], 0);  // chunk_index 0
  EXPECT_EQ(first[17], 0);
  EXPECT_EQ(first[18], static_cast<uint8_t>(chunks.size()));
  EXPECT_EQ(first[19], 0);

  ConfigAckPayload ack{};
  ack.config_id = 0x0102030405060708ull;
  ack.status = ConfigAckStatus::kBackendFailed;
  std::array<uint8_t, sizeof(ConfigAckPayload)> bytes{};
  ASSERT_EQ(EncodeConfigAck(ack, bytes), bytes.size());
  const std::array<uint8_t, 9> expected{8, 7, 6, 5, 4, 3, 2, 1, 4};
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), bytes.begin()));
}

TEST(Gateway, StartsUnconfiguredAndRefusesEnableUntilReconfigured) {
  SimBackend backend;
  Gateway g{backend, 1001};
  EXPECT_FALSE(g.configured());
  EXPECT_EQ(g.snapshot().config_id, 0u);

  g.Tick(1000, true);
  // Refuses to enable or set kConfigured
  EXPECT_FALSE(g.snapshot().flags & kConfigured);
  EXPECT_FALSE(g.snapshot().flags & kEnabled);

  // Hardware configuration requires disabled: refused while enabled, with no
  // state change.
  auto valid_cfg = SwerveConfig(true);
  EXPECT_FALSE(g.Reconfigure(valid_cfg));
  EXPECT_FALSE(g.configured());

  g.Tick(1500, false);

  // Ceiling violation rejected
  auto invalid_cfg = SwerveConfig(true);
  invalid_cfg.motors[0].stator_limit_a = 150.0;
  EXPECT_FALSE(g.Reconfigure(invalid_cfg));
  EXPECT_FALSE(g.configured());

  g.Tick(2000, false);
  EXPECT_TRUE(g.snapshot().flags & kHardwareFault);

  // Valid config accepted
  EXPECT_TRUE(g.Reconfigure(valid_cfg));
  EXPECT_TRUE(g.configured());

  g.Tick(3000, true);
  EXPECT_TRUE(g.snapshot().flags & kConfigured);
  EXPECT_TRUE(g.snapshot().flags & kEnabled);
  EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
}

// While a configure runs, or after one fails, the gateway reads no device, so
// a State it publishes then has no measurement in it: every motor and CAN
// sensor sample invalid with zero measurements, every RIO device sample and
// the health sample invalid. Scenario: a gateway read at 30 rotations, then
// reconfigured over several ticks, or failing its configure; published as
// valid, those 30 rotations are a frozen position a node takes for a live one
// for as long as the configure lasts. An in-progress configure raises no fault
// (every boot runs one); the first State after a successful one is read
// again.
// REQ-CONTRACT-32
TEST(Gateway, UnconfiguredStatesCarryNoValidSample) {
  class PendingBackend : public FakeBackend {
   public:
    void BeginConfigure(const Config&) override {}
    ConfigureStatus PollConfigure() override { return status; }
    ConfigureStatus status{ConfigureStatus::kPending};
  } backend;
  backend.measurement = 3;
  auto config = SwerveConfig(true);
  config.digital_inputs.push_back({40, "beam", 1});
  for (const auto& motor : config.motors) backend.positions[motor.id] = 30;
  const auto read = [&config](const State& s) {
    EXPECT_EQ(s.motor_count, config.motors.size());
    for (std::size_t i = 0; i < s.motor_count; ++i) {
      SCOPED_TRACE(config.motors[i].name);
      EXPECT_TRUE(s.motors[i].valid);
      EXPECT_EQ(s.motors[i].position_rot, 30.0);
    }
    for (std::size_t i = 0; i < s.sensor_count; ++i) {
      EXPECT_TRUE(s.sensors[i].valid) << config.sensors[i].name;
    }
    EXPECT_TRUE(s.digital_inputs[0].valid);
    EXPECT_TRUE(s.health.valid);
  };
  const auto unread = [&config](const State& s) {
    for (std::size_t i = 0; i < s.motor_count; ++i) {
      SCOPED_TRACE(config.motors[i].name);
      EXPECT_FALSE(s.motors[i].valid);
      EXPECT_EQ(s.motors[i].position_rot, 0.0);
      EXPECT_EQ(s.motors[i].velocity_rps, 0.0);
      EXPECT_EQ(s.motors[i].voltage, 0.0);
      EXPECT_EQ(s.motors[i].stator_current_a, 0.0);
      EXPECT_EQ(s.motors[i].supply_current_a, 0.0);
    }
    for (std::size_t i = 0; i < s.sensor_count; ++i) {
      SCOPED_TRACE(config.sensors[i].name);
      EXPECT_FALSE(s.sensors[i].valid);
      EXPECT_EQ(s.sensors[i].position_rot, 0.0);
      EXPECT_EQ(s.sensors[i].velocity_rps, 0.0);
    }
    EXPECT_FALSE(s.digital_inputs[0].valid);
    EXPECT_FALSE(s.health.valid);
  };
  Gateway g{config, backend, 1};
  g.Tick(1000, false);
  read(g.snapshot());

  auto changed = config;
  changed.motors[0].slots[0].p = 1;
  ASSERT_TRUE(g.BeginReconfigure(changed));
  for (const uint64_t now : {uint64_t{2000}, uint64_t{3000}, uint64_t{4000}}) {
    SCOPED_TRACE("configuring at " + std::to_string(now));
    g.Tick(now, false);
    ASSERT_TRUE(g.configuring());
    unread(g.snapshot());
    EXPECT_FALSE(g.snapshot().flags & kConfigured);
    EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
  }
  backend.status = Backend::ConfigureStatus::kDone;
  g.Tick(5000, false);
  ASSERT_TRUE(g.configured());
  {
    SCOPED_TRACE("first State after the configure");
    read(g.snapshot());
    EXPECT_TRUE(g.snapshot().flags & kConfigured);
  }

  config.motors[0].slots[0].p = 2;
  backend.status = Backend::ConfigureStatus::kPending;
  ASSERT_TRUE(g.BeginReconfigure(config));
  backend.status = Backend::ConfigureStatus::kFailed;
  for (const uint64_t now : {uint64_t{6000}, uint64_t{7000}}) {
    SCOPED_TRACE("after a failed configure, at " + std::to_string(now));
    g.Tick(now, false);
    ASSERT_FALSE(g.configured());
    unread(g.snapshot());
    EXPECT_FALSE(g.snapshot().flags & kConfigured);
    EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
  }

  // A configure that completes within one call publishes nothing read under
  // the configuration before it either, until its own first Read.
  ASSERT_TRUE(g.Reconfigure(changed));
  {
    SCOPED_TRACE("configured, not yet read");
    unread(g.snapshot());
  }
  g.Tick(8000, false);
  read(g.snapshot());
}

// Polls `done` until it holds. The deadline only bounds a hang
// (docs/TESTING.md §6.3); no test asserts how long a wait took.
template <typename Done>
[[nodiscard]] bool Eventually(Done done) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{10};
  while (!done()) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  return true;
}

// The bridge's side of an Endpoint's link, on a raw loopback socket.
// Connect() binds an ephemeral port, opens the endpoint toward it from an
// ephemeral port of its own, and learns that port from a frame the endpoint
// sends, so no test binds a fixed port (docs/TESTING.md §6.4). A raw socket,
// since a RuntimeUdpPeer must know its peer's port before it binds, and sends
// only this build's protocol version.
class Companion {
 public:
  Companion() = default;
  ~Companion() {
    if (fd_ >= 0) ::close(fd_);
  }
  Companion(const Companion&) = delete;
  Companion& operator=(const Companion&) = delete;

  ::testing::AssertionResult Connect(Endpoint& endpoint) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return Failed("socket");
    // As a RuntimeUdpPeer's: room for the States a tick loop piles up.
    const int buffer_bytes = 256 * 1024;
    if (::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &buffer_bytes,
                     sizeof(buffer_bytes)) != 0 ||
        ::fcntl(fd_, F_SETFL, O_NONBLOCK) != 0)
      return Failed("configure socket");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t length = sizeof(address);
    if (::bind(fd_, reinterpret_cast<const sockaddr*>(&address),
               sizeof(address)) != 0 ||
        ::getsockname(fd_, reinterpret_cast<sockaddr*>(&address), &length) != 0)
      return Failed("bind port 0");
    const auto port = ntohs(address.sin_port);
    if (endpoint.Open("127.0.0.1", 0, "127.0.0.1", port) !=
        protocol::UdpStatus::kOk)
      return ::testing::AssertionFailure()
             << "Endpoint::Open toward port " << port;
    // The passthrough sends without ticking the gateway.
    const std::array<uint8_t, 1> hello{};
    if (endpoint.SendFrame(protocol::FrameType::kDriverStation, 0, hello.data(),
                           hello.size()) != protocol::UdpStatus::kOk)
      return ::testing::AssertionFailure() << "the endpoint did not send";
    sockaddr_in from{};
    socklen_t from_length = 0;
    const bool heard = Eventually([&] {
      from_length = sizeof(from);
      const auto got =
          ::recvfrom(fd_, rx_.data(), rx_.size(), 0,
                     reinterpret_cast<sockaddr*>(&from), &from_length);
      protocol::DecodedFrame frame;
      return got > 0 &&
             protocol::DecodeFrame(rx_.data(), static_cast<std::size_t>(got),
                                   &frame) == protocol::DecodeStatus::kOk &&
             frame.header.type == protocol::FrameType::kDriverStation;
    });
    if (!heard)
      return ::testing::AssertionFailure() << "no frame from the endpoint";
    if (::connect(fd_, reinterpret_cast<const sockaddr*>(&from), from_length) !=
        0)
      return Failed("connect");
    return ::testing::AssertionSuccess();
  }

  // Frames carry this companion's own sequence, from 1, like a
  // RuntimeUdpPeer's; `version` stamps another protocol version.
  protocol::UdpStatus SendFrame(protocol::FrameType type, uint16_t flags,
                                uint64_t monotonic_time_us, const void* payload,
                                std::size_t payload_size,
                                uint16_t version = protocol::kProtocolVersion) {
    std::array<uint8_t, protocol::kMaxFrameSize> frame{};
    const auto size = protocol::EncodeFrame(
        type, flags, next_send_sequence_, monotonic_time_us, payload,
        payload_size, frame.data(), frame.size());
    if (size == 0) return protocol::UdpStatus::kInvalidArgument;
    frame[8] = static_cast<uint8_t>(version);  // The header's version field.
    frame[9] = static_cast<uint8_t>(version >> 8);
    if (::send(fd_, frame.data(), size, 0) != static_cast<ssize_t>(size))
      return protocol::UdpStatus::kSocketError;
    ++next_send_sequence_;
    return protocol::UdpStatus::kOk;
  }

  protocol::UdpStatus TryReceive(protocol::DecodedFrame* out) {
    const auto got = ::recv(fd_, rx_.data(), rx_.size(), 0);
    if (got < 0)
      return errno == EAGAIN || errno == EWOULDBLOCK
                 ? protocol::UdpStatus::kWouldBlock
                 : protocol::UdpStatus::kSocketError;
    return protocol::DecodeFrame(rx_.data(), static_cast<std::size_t>(got),
                                 out) == protocol::DecodeStatus::kOk
               ? protocol::UdpStatus::kOk
               : protocol::UdpStatus::kDecodeError;
  }

  // Waits for the next frame: a sent datagram need not be readable at once,
  // even on loopback.
  protocol::UdpStatus Receive(protocol::DecodedFrame* out) {
    auto status = protocol::UdpStatus::kWouldBlock;
    (void)Eventually([&] {
      status = TryReceive(out);
      return status != protocol::UdpStatus::kWouldBlock;
    });
    return status;
  }

  // Waits for the next frame of `type`, skipping the others (the State every
  // tick sends).
  bool Receive(protocol::FrameType type, protocol::DecodedFrame* out) {
    while (Receive(out) == protocol::UdpStatus::kOk) {
      if (out->header.type == type) return true;
    }
    return false;
  }

  uint64_t next_send_sequence() const { return next_send_sequence_; }

 private:
  static ::testing::AssertionResult Failed(const char* step) {
    return ::testing::AssertionFailure()
           << step << ": " << std::strerror(errno);
  }

  int fd_{-1};
  uint64_t next_send_sequence_{1};
  std::array<uint8_t, protocol::kMaxFrameSize> rx_{};
};

// Ticks the endpoint and returns the State that tick sent, skipping the
// frames sent before it.
std::optional<State> TickForState(Endpoint& endpoint, Companion& companion,
                                  uint64_t now_us, bool enabled) {
  if (endpoint.Tick(now_us, enabled) != protocol::UdpStatus::kOk)
    return std::nullopt;
  protocol::DecodedFrame frame;
  while (companion.Receive(protocol::FrameType::kHardwareState, &frame)) {
    State state;
    if (!Decode({frame.payload, frame.payload_size}, state))
      return std::nullopt;
    if (state.sample_time_us == now_us) return state;
  }
  return std::nullopt;
}

TEST(Endpoint, ClockProbeEchoUsesLaterSampleWhileDisabledAndUnconfigured) {
  SimBackend backend;
  Gateway gateway{backend, 2003};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  ASSERT_EQ(companion.SendFrame(protocol::FrameType::kClockProbe, 0, 9000,
                                nullptr, 0),
            protocol::UdpStatus::kOk);
  // The State of the tick that receives the probe cannot echo it yet: that
  // tick's sample was taken before its receive.
  const auto expect_state = [&](uint64_t tick, uint64_t echo_us) {
    const auto state = TickForState(endpoint, companion, tick, false);
    ASSERT_TRUE(state.has_value()) << "tick " << tick;
    EXPECT_EQ(state->clock_probe_time_us, echo_us) << "tick " << tick;
    EXPECT_FALSE(state->flags & (kEnabled | kCommandActive));
    EXPECT_EQ(state->counters.rejected_command_count, 0u);
  };
  uint64_t tick = 1000;
  ASSERT_TRUE(Eventually([&] {
    expect_state(tick++, 0);
    return HasFailure() || endpoint.peer_contact().frame_us != 0;
  }));
  ASSERT_FALSE(HasFailure());
  expect_state(6000, 9000);
}

TEST(Endpoint, OutboundBlackoutPreservesReceiveAndSafetyTicks) {
  SimBackend backend;
  Gateway gateway{backend, 2002};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));

  endpoint.SetSendEnabled(false);
  const auto cfg = SwerveConfig(true);
  const auto chunks = CreateConfigChunks(cfg);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig, i + 1,
                                  1000, chunks[i].data(), chunks[i].size()),
              protocol::UdpStatus::kOk);
  }
  // Receives config, but drops its ACK.
  ASSERT_TRUE(Eventually([&] {
    EXPECT_EQ(endpoint.Tick(2000, false), protocol::UdpStatus::kOk);
    return gateway.configured();
  }));
  ASSERT_EQ(endpoint.Tick(3000, true), protocol::UdpStatus::kOk);
  EXPECT_TRUE(gateway.snapshot().flags & kEnabled);
  Command command = Request(gateway);
  command.motors[0].demand = 5.0;
  std::array<uint8_t, kMaxCommandSize> command_bytes{};
  const auto command_size = Encode(command, command_bytes);
  ASSERT_GT(command_size, 0u);
  ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareCommand, 100,
                                3000, command_bytes.data(), command_size),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(Eventually([&] {
    EXPECT_EQ(endpoint.Tick(4000, true), protocol::UdpStatus::kOk);
    return (gateway.snapshot().flags & kCommandActive) != 0;
  }));
  ASSERT_EQ(endpoint.Tick(5000, true), protocol::UdpStatus::kOk);
  EXPECT_TRUE(gateway.snapshot().flags & kCommandActive);
  EXPECT_DOUBLE_EQ(gateway.snapshot().motors[0].velocity_rps, 5.0);
  ASSERT_EQ(endpoint.Tick(5000 + cfg.command_timeout_us, true),
            protocol::UdpStatus::kOk);
  EXPECT_FALSE(gateway.snapshot().flags & kCommandActive);
  EXPECT_DOUBLE_EQ(gateway.snapshot().motors[0].velocity_rps, 0.0);
  const std::array<uint8_t, 1> ds_payload{0};
  ASSERT_EQ(endpoint.SendFrame(protocol::FrameType::kDriverStation, 3000,
                               ds_payload.data(), ds_payload.size()),
            protocol::UdpStatus::kOk);

  // One sender to one receiver on loopback keeps its order, so the first
  // frame after the blackout shows that the ACK, every State and the Driver
  // Station frame of the blackout were dropped, not queued.
  endpoint.SetSendEnabled(true);
  const uint64_t resumed_at = 6000 + cfg.command_timeout_us;
  ASSERT_EQ(endpoint.Tick(resumed_at, false), protocol::UdpStatus::kOk);
  ASSERT_EQ(endpoint.SendFrame(protocol::FrameType::kDriverStation, resumed_at,
                               ds_payload.data(), ds_payload.size()),
            protocol::UdpStatus::kOk);
  protocol::DecodedFrame frame;
  ASSERT_EQ(companion.Receive(&frame), protocol::UdpStatus::kOk);
  EXPECT_EQ(frame.header.type, protocol::FrameType::kHardwareState);
  State state;
  ASSERT_TRUE(Decode({frame.payload, frame.payload_size}, state));
  EXPECT_EQ(state.sample_time_us, resumed_at);
  EXPECT_FALSE(state.flags & kEnabled);
  ASSERT_EQ(companion.Receive(&frame), protocol::UdpStatus::kOk);
  EXPECT_EQ(frame.header.type, protocol::FrameType::kDriverStation);
  EXPECT_EQ(frame.header.monotonic_time_us, resumed_at);
}

// The RoboRIO program reports to the drive team whether its companion is
// talking, and in which protocol. A clock probe alone (all the bridge sends
// while nothing is commanded) is contact; a frame of another protocol version
// is recorded with that version, not dropped unseen.
TEST(Endpoint, PeerContactRecordsProbesCommandsAndForeignProtocolVersions) {
  SimBackend backend;
  Gateway gateway{backend, 2004};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  // Ticks at `now_us` until the endpoint has received what was sent.
  const auto tick_until = [&](uint64_t now_us, const uint64_t& heard_us) {
    return Eventually([&] {
      endpoint.Tick(now_us, false);
      return heard_us != 0;
    });
  };
  const auto& contact = endpoint.peer_contact();

  ASSERT_EQ(
      companion.SendFrame(protocol::FrameType::kClockProbe, 0, 0, nullptr, 0),
      protocol::UdpStatus::kOk);
  ASSERT_TRUE(tick_until(1000, contact.frame_us));
  EXPECT_EQ(contact.frame_us, 1000u);
  EXPECT_EQ(contact.command_us, 0u);

  std::array<uint8_t, kMaxCommandSize> command{};
  const auto command_size = Encode(Command{}, command);
  ASSERT_GT(command_size, 0u);
  ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareCommand, 0, 0,
                                command.data(), command_size),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(tick_until(2000, contact.command_us));
  EXPECT_EQ(contact.command_us, 2000u);
  EXPECT_EQ(contact.version_mismatch_us, 0u);

  const auto foreign = static_cast<uint16_t>(protocol::kProtocolVersion + 1);
  ASSERT_EQ(companion.SendFrame(protocol::FrameType::kClockProbe, 0, 0, nullptr,
                                0, foreign),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(tick_until(3000, contact.version_mismatch_us));
  EXPECT_EQ(contact.version_mismatch_us, 3000u);
  EXPECT_EQ(contact.mismatched_version, foreign);
  EXPECT_EQ(contact.frame_us, 2000u);
}

TEST(Endpoint, PushesConfigOverUdpAndAcknowledgesIt) {
  SimBackend backend;
  Gateway gateway{backend, 2002};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));

  // 1. Companion sees state with config_id = 0
  auto state = TickForState(endpoint, companion, 1000, true);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->config_id, 0u);
  EXPECT_EQ(state->boot_id, 2002u);

  // 2. Companion pushes config chunks
  auto cfg = SwerveConfig(true);
  auto chunks = CreateConfigChunks(cfg);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig, i + 1,
                                  1000, chunks[i].data(), chunks[i].size()),
              protocol::UdpStatus::kOk);
  }

  // 3. Endpoint ticks, processes chunks, sends ACK. Hardware configuration
  // requires disabled operation, so the config push is serviced disabled.
  ASSERT_TRUE(Eventually([&] {
    endpoint.Tick(2000, false);
    return gateway.configured();
  }));

  // 4. Companion receives ACK
  protocol::DecodedFrame ack_frame;
  ASSERT_TRUE(
      companion.Receive(protocol::FrameType::kHardwareConfigAck, &ack_frame));
  ConfigAckPayload ack{};
  ASSERT_TRUE(
      DecodeConfigAck({ack_frame.payload, ack_frame.payload_size}, ack));
  EXPECT_EQ(ack.status, ConfigAckStatus::kOk);
  EXPECT_EQ(ack.config_id, ConfigurationId(cfg));

  // 5. Subsequent state from endpoint has config_id set and kConfigured set
  state = TickForState(endpoint, companion, 3000, true);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->config_id, ConfigurationId(cfg));
  EXPECT_TRUE(state->flags & kConfigured);
}

TEST(SimBackend, NeutralTurnsOffEveryOutputOnDisableTimeoutAndFailure) {
  // SimBackend is final, so failure injection delegates to an owned
  // SimBackend; Neutral() under test is still SimBackend::Neutral().
  struct FailableSim : Backend {
    bool Configure(const Config& config) override {
      return sim.Configure(config);
    }
    bool Read(State& state) override {
      if (fail_read) return false;
      return sim.Read(state);
    }
    bool Apply(std::span<const MotorRequest> commands) override {
      return sim.Apply(commands);
    }
    std::span<const uint16_t> ApplyFailedDevices() const override {
      return sim.ApplyFailedDevices();
    }
    bool ApplyOutputs(std::span<const DigitalOutputRequest> digital,
                      std::span<const PwmRequest> pwm) override {
      return sim.ApplyOutputs(digital, pwm);
    }
    void Neutral() override { sim.Neutral(); }
    bool fail_read{false};
    SimBackend sim;
  };
  auto make_config = [] {
    Config config;
    config.commissioned = true;
    config.command_timeout_us = 100000;
    config.motors.push_back({1, "motor1", 1, "rio"});
    config.digital_outputs.push_back({4, "solenoid", 1});
    config.pwm_outputs.push_back({7, "servo", 0});
    return config;
  };
  auto drive_all = [](Gateway& gateway) {
    const State s = gateway.snapshot();
    Command cmd{};
    cmd.config_id = s.config_id;
    cmd.boot_id = s.boot_id;
    cmd.epoch = s.epoch;
    cmd.observed_time_us = s.sample_time_us;
    cmd.count = 1;
    cmd.motors[0] = {1, Mode::kVelocity, 0, 10.0, 0};
    cmd.digital_output_count = 1;
    cmd.digital_outputs[0] = {4, true};
    cmd.pwm_output_count = 1;
    cmd.pwm_outputs[0] = {7, 0.85};
    return cmd;
  };
  auto expect_driven = [](FailableSim& backend, const State& s) {
    EXPECT_TRUE(backend.sim.GetDigitalOutput(4));
    EXPECT_DOUBLE_EQ(backend.sim.GetPwm(7), 0.85);
    ASSERT_GT(s.motor_count, 0u);
    EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 10.0);
    EXPECT_EQ(s.digital_outputs[0].value, true);
    EXPECT_DOUBLE_EQ(s.pwm_outputs[0].output, 0.85);
  };
  auto expect_safe_backends = [](FailableSim& backend) {
    EXPECT_FALSE(backend.sim.GetDigitalOutput(4));
    EXPECT_DOUBLE_EQ(backend.sim.GetPwm(7), 0.0);
  };
  auto expect_safe_state = [](const State& s) {
    ASSERT_GT(s.motor_count, 0u);
    EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
    EXPECT_DOUBLE_EQ(s.motors[0].voltage, 0.0);
    EXPECT_EQ(s.digital_outputs[0].value, false);
    EXPECT_DOUBLE_EQ(s.pwm_outputs[0].output, 0.0);
  };

  FailableSim backend;
  Gateway gateway{make_config(), backend, 10};

  // Disable transition neutralizes every output type.
  gateway.Tick(1000, true);
  ASSERT_TRUE(gateway.Accept(drive_all(gateway), 1, 1000));
  gateway.Tick(6000, true);
  expect_driven(backend, gateway.snapshot());
  gateway.Tick(7000, false);
  expect_safe_backends(backend);
  expect_safe_state(gateway.snapshot());

  // Timeout transition neutralizes every output type.
  gateway.Tick(8000, true);
  ASSERT_TRUE(gateway.Accept(drive_all(gateway), 0, 8000));
  gateway.Tick(9000, true);
  expect_driven(backend, gateway.snapshot());
  gateway.Tick(9000 + 100000, true);
  expect_safe_backends(backend);
  expect_safe_state(gateway.snapshot());

  // Failure transition neutralizes backend outputs and latches the fault.
  // State keeps its last good samples while reads fail, so the motor safe
  // state is observed on the first successful read after recovery.
  gateway.Tick(200000, true);
  ASSERT_TRUE(gateway.Accept(drive_all(gateway), 0, 200000));
  gateway.Tick(201000, true);
  expect_driven(backend, gateway.snapshot());
  backend.fail_read = true;
  gateway.Tick(202000, true);
  expect_safe_backends(backend);
  EXPECT_TRUE(gateway.snapshot().flags & kHardwareFault);
  backend.fail_read = false;
  gateway.Tick(203000, true);
  expect_safe_backends(backend);
  expect_safe_state(gateway.snapshot());
}

// A motor (id 1, with a follower) resets while every motor drives at velocity.
// The backend keeps a motor's last request between commands, so the gateway
// must send its chain neutral in the tick that finds the reset, before
// BeginRestoreAfterReset reconfigures it, with no whole-robot Neutral() and
// no epoch change. Through a restore spanning three ticks, the command in
// force and one observing each State of the restore are accepted, but the
// chain receives only neutral; after the restore, a command observing the
// last restoring State still drives it nothing, and the first observing a
// State after the restore drives it again. The other motors drive throughout.
// REQ-SAFE-24
TEST(Gateway, ResetWhileEnabledNeutralizesBeforeRestore) {
  auto config = SwerveConfig(true);
  constexpr uint16_t kResetId = 1;
  auto follower = *std::find_if(
      config.motors.begin(), config.motors.end(),
      [](const MotorConfig& motor) { return motor.id == kResetId; });
  follower.id = 100;
  follower.name = "back_left_drive_follower";
  follower.can_id = 60;
  follower.master_id = kResetId;
  config.motors.push_back(follower);
  ASSERT_NO_THROW(Validate(config));
  const auto in_chain = [&config](std::size_t i) {
    return config.motors[i].id == kResetId ||
           config.motors[i].master_id == kResetId;
  };
  // Every chain motor neutral (`chain_neutral`) and every other motor still
  // at the velocity the commands ask for.
  const auto expect_apply = [&](const std::vector<MotorRequest>& requests,
                                bool chain_neutral, const std::string& where) {
    SCOPED_TRACE(where);
    ASSERT_EQ(requests.size(), config.motors.size());
    for (std::size_t i = 0; i < requests.size(); ++i) {
      const auto expected =
          in_chain(i) && chain_neutral ? Mode::kNeutral : Mode::kVelocity;
      EXPECT_EQ(requests[i].mode, expected) << config.motors[i].name;
    }
  };

  FakeBackend backend;
  backend.pending_ticks = 2;
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  const auto before_reset = Request(g);
  ASSERT_TRUE(g.Accept(before_reset, 1, 1000));
  expect_apply(backend.last_requests, false, "driving before the reset");
  const auto epoch = g.snapshot().epoch;
  backend.calls.clear();
  backend.resets = {kResetId};

  uint64_t now = 1000;
  uint64_t sequence = 1;
  uint64_t last_restoring_us = 0;
  const auto restore_finished = [&backend] {
    return std::any_of(backend.calls.begin(), backend.calls.end(),
                       [](const FakeBackend::Call& call) {
                         return call.kind ==
                                FakeBackend::Call::kRestoreFinished;
                       });
  };
  while (now < 20000) {
    now += 1000;
    g.Tick(now, true);
    if (restore_finished()) break;
    last_restoring_us = now;
    EXPECT_TRUE(g.Accept(before_reset, ++sequence, now)) << now;
    EXPECT_TRUE(g.Accept(Request(g), ++sequence, now)) << now;
  }
  ASSERT_TRUE(restore_finished());
  ASSERT_EQ(last_restoring_us, 4000u);  // Restoring through three ticks.
  EXPECT_EQ(backend.restoring_ids, std::vector<uint16_t>{kResetId});

  // Up to the end of the restore: the chain neutral before the restore
  // begins, and nothing but neutral for it until the restore finishes.
  bool neutral_sent = false, restore_begun = false;
  for (std::size_t k = 0; k < backend.calls.size(); ++k) {
    const auto& call = backend.calls[k];
    const std::string where = "call " + std::to_string(k);
    if (call.kind == FakeBackend::Call::kRestoreFinished) break;
    EXPECT_NE(call.kind, FakeBackend::Call::kNeutral) << where;
    if (call.kind == FakeBackend::Call::kBeginRestore) {
      EXPECT_TRUE(neutral_sent) << where << ": restore before neutral";
      restore_begun = true;
    }
    if (call.kind == FakeBackend::Call::kApply) {
      expect_apply(call.requests, true, where);
      neutral_sent = true;
    }
  }
  EXPECT_TRUE(restore_begun);
  EXPECT_EQ(g.snapshot().epoch, epoch);

  auto restoring = Request(g);
  restoring.observed_time_us = last_restoring_us;
  ASSERT_TRUE(g.Accept(restoring, ++sequence, now));
  expect_apply(backend.last_requests, true, "observed during the restore");
  ASSERT_TRUE(g.Accept(Request(g), ++sequence, now));
  expect_apply(backend.last_requests, false, "observed after the restore");
  EXPECT_EQ(g.snapshot().epoch, epoch);
}

// A dropped watchdog neutral must not leave the last demand in force: every
// Tick without an accepted command refreshes neutral until a fresh command
// supersedes it, matching the disabled and unhealthy paths.
TEST(Gateway, WatchdogNeutralRefreshesEveryTickUntilFreshCommand) {
  FakeBackend backend;
  const auto config = SwerveConfig(true);
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  ASSERT_TRUE(g.Accept(Request(g), 1, 1000));
  g.Tick(6000, true);
  int neutrals = backend.neutral;
  // Active command: no neutral refresh.
  g.Tick(11000, true);
  EXPECT_EQ(backend.neutral, neutrals);
  uint64_t now = 1000 + config.command_timeout_us;
  g.Tick(now, true);
  ASSERT_FALSE(g.snapshot().flags & kCommandActive);
  EXPECT_GT(backend.neutral, neutrals);
  for (int i = 0; i < 5; ++i) {
    neutrals = backend.neutral;
    now += 5000;
    g.Tick(now, true);
    EXPECT_EQ(backend.neutral, neutrals + 1) << "tick " << i;
  }
  // A fresh accepted command supersedes the refresh.
  ASSERT_TRUE(g.Accept(Request(g), 2, now));
  neutrals = backend.neutral;
  now += 5000;
  g.Tick(now, true);
  EXPECT_EQ(backend.neutral, neutrals);
  EXPECT_TRUE(g.snapshot().flags & kCommandActive);

  // Backend-unhealthy and disabled paths keep refreshing neutral too.
  backend.healthy = false;
  for (int i = 0; i < 3; ++i) {
    neutrals = backend.neutral;
    now += 5000;
    g.Tick(now, true);
    EXPECT_GE(backend.neutral, neutrals + 1);
  }
  backend.healthy = true;
  for (int i = 0; i < 3; ++i) {
    neutrals = backend.neutral;
    now += 5000;
    g.Tick(now, false);
    EXPECT_GE(backend.neutral, neutrals + 1);
    EXPECT_FALSE(g.Accept(Request(g), 10 + i, now));
  }
}

TEST(Gateway, PhysicsResetPreservesPositionAndRejectsOldDemand) {
  sim::PhysicsSimBackend backend;
  Gateway gateway{SwerveConfig(true), backend, 1};
  gateway.Tick(1000, true);
  for (uint64_t now = 6000; now <= 101000; now += 5000) {
    auto command = Request(gateway);
    for (std::size_t i = 0; i < command.count; ++i) {
      command.motors[i].mode = Mode::kDutyCycle;
      command.motors[i].demand = 0.2;
    }
    ASSERT_TRUE(gateway.Accept(command, now, now));
    gateway.Tick(now, true);
  }
  const auto stale = Request(gateway);
  const auto epoch = gateway.snapshot().epoch;
  const double before = gateway.snapshot().motors[0].position_rot;
  ASSERT_GT(before, 0.0);
  backend.SimulateDeviceReset(gateway.snapshot().motors[0].id);
  gateway.Tick(106000, true);
  // Masked, and so carrying no measurement, while it restores.
  ASSERT_FALSE(gateway.snapshot().motors[0].valid);
  EXPECT_EQ(gateway.snapshot().epoch, epoch);
  EXPECT_TRUE(gateway.Accept(stale, 200000, 106000));
  gateway.Tick(111000, true);  // Read while restoring, then restored.
  gateway.Tick(116000, true);
  // Restored where the reset tick left it, past `before` rather than
  // restarted at zero, and the stale demand drives nothing.
  const auto restored = gateway.snapshot().motors[0];
  ASSERT_TRUE(restored.valid);
  EXPECT_GT(restored.position_rot, before);
  EXPECT_DOUBLE_EQ(restored.velocity_rps, 0.0);
  EXPECT_DOUBLE_EQ(restored.voltage, 0.0);
}

// A config of one master and its follower, driving, whose master resets. On
// the robot PhoenixBackend::Read reports both invalid from the tick after the
// restore begins until PollRestore completes it, so that State has no valid
// feedback: total loss, which faults, ends the command in force and starts a
// new epoch. The next State is healthy again, and only a command of the new
// epoch drives the pair. A simulated backend must fault the same way.
template <typename SimulatedBackend>
void ExpectRestoringPairIsTotalLoss(SimulatedBackend& backend) {
  Config config;
  config.commissioned = true;
  MotorConfig master;
  master.id = 1;
  master.name = "shooter";
  master.can_id = 1;
  MotorConfig follower = master;
  follower.id = 2;
  follower.name = "shooter_follower";
  follower.can_id = 2;
  follower.master_id = master.id;
  config.motors = {master, follower};
  Gateway g{config, backend, 1};
  const auto drive = [&g](uint64_t sequence, uint64_t now) {
    auto command = Request(g);
    for (std::size_t i = 0; i < command.count; ++i) {
      command.motors[i] = {command.motors[i].id, Mode::kVoltage, 0, 2};
    }
    return g.Accept(command, sequence, now);
  };
  g.Tick(1000, true);
  ASSERT_TRUE(drive(1, 1000));
  const auto epoch = g.snapshot().epoch;
  backend.SimulateDeviceReset(master.id);
  g.Tick(2000, true);  // Reset found: the pair neutral, the restore begun.
  ASSERT_FALSE(g.snapshot().flags & kHardwareFault);
  const auto before_loss = Request(g);

  g.Tick(3000, true);  // Read while the pair restores.
  auto s = g.snapshot();
  EXPECT_FALSE(s.motors[0].valid);
  EXPECT_FALSE(s.motors[1].valid);
  EXPECT_TRUE(s.flags & kHardwareFault);
  EXPECT_FALSE(s.flags & kCommandActive);
  EXPECT_EQ(s.epoch, epoch + 1);
  EXPECT_FALSE(g.Accept(before_loss, 2, 3000));

  g.Tick(4000, true);  // Restored.
  s = g.snapshot();
  EXPECT_TRUE(s.motors[0].valid);
  EXPECT_TRUE(s.motors[1].valid);
  EXPECT_FALSE(s.flags & kHardwareFault);
  EXPECT_FALSE(g.Accept(before_loss, 3, 4000));
  ASSERT_TRUE(drive(4, 4000));
  g.Tick(5000, true);
  EXPECT_NE(g.snapshot().motors[0].voltage, 0.0);
  EXPECT_NE(g.snapshot().motors[1].voltage, 0.0);
}

TEST(SimBackend, RestoringChainIsInvalidAsOnTheRobot) {
  SimBackend backend;
  ExpectRestoringPairIsTotalLoss(backend);
}

TEST(PhysicsSimBackend, RestoringChainIsInvalidAsOnTheRobot) {
  sim::PhysicsSimBackend backend;
  ExpectRestoringPairIsTotalLoss(backend);
}

// A TalonFX reset restarts its rotor position at zero, and the restarted
// frames can reach the gateway before HasResetOccurred() does. A rotor-
// feedback motor whose position steps farther than its motor can turn is
// masked, counted and restored in that State, across a gap of stale frames,
// so a command observed before it never drives the restarted frame. The
// restored motor's first sample starts a new reference rather than reading
// as another jump; the late latch is counted as one more reset.
TEST(Gateway, RotorPositionJumpIsAResetBeforeItsLatchArrives) {
  const auto config = SwerveConfig(true);
  const auto index = MotorIndex(config, Feedback::kRotor);
  const auto other = MotorIndex(config, Feedback::kRemoteCANcoder);
  const uint16_t id = config.motors[index].id;
  FakeBackend backend;
  backend.pending_ticks = 2;
  Gateway g{config, backend, 1};
  backend.positions[id] = 30;
  g.Tick(1000, true);
  const auto before = Request(g);
  ASSERT_TRUE(g.Accept(before, 1, 1000));
  backend.invalid_motor = id;  // The TalonFX reboots: its frames go stale.
  g.Tick(2000, true);
  EXPECT_EQ(g.snapshot().counters.device_reset_count, 0u);
  backend.invalid_motor = 0;
  backend.positions[id] = 0;  // Restarted frames, no reset latch yet.
  g.Tick(3000, true);
  EXPECT_FALSE(g.snapshot().motors[index].valid);
  EXPECT_EQ(g.snapshot().counters.device_reset_count, 1u);
  EXPECT_EQ(backend.restoring_ids, std::vector<uint16_t>{id});
  EXPECT_EQ(backend.last_requests[index].mode, Mode::kNeutral);

  uint64_t now = 3000;
  while (!g.snapshot().motors[index].valid && now < 20000) {
    now += 1000;
    g.Tick(now, true);
  }
  ASSERT_TRUE(g.snapshot().motors[index].valid);
  ASSERT_TRUE(g.Accept(before, 2, now));
  EXPECT_EQ(backend.last_requests[index].mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests[other].mode, Mode::kVelocity);
  backend.positions[id] = 0.1;
  now += 1000;
  g.Tick(now, true);
  EXPECT_TRUE(g.snapshot().motors[index].valid);
  EXPECT_EQ(g.snapshot().counters.device_reset_count, 1u);
  ASSERT_TRUE(g.Accept(Request(g), 3, now));
  EXPECT_EQ(backend.last_requests[index].mode, Mode::kVelocity);

  backend.resets = {id};
  now += 1000;
  g.Tick(now, true);
  EXPECT_FALSE(g.snapshot().motors[index].valid);
  EXPECT_EQ(g.snapshot().counters.device_reset_count, 2u);
}

uint64_t late_read_us = 0;
uint64_t LateReadClock() { return late_read_us; }

// No reset for a step the motor can make: across a wrap of Phoenix's
// fixed-point position, after a held frame whose age widens the interval, or
// over a tick whose feedback was read late; nor for any step of a remote
// CANcoder motor, whose frame may be absolute and steps when its remote
// frames resume.
TEST(Gateway, PositionStepsWithinMotorSpeedOrOnRemoteFeedbackAreNoReset) {
  const auto config = SwerveConfig(true);
  const auto drive = MotorIndex(config, Feedback::kRotor);
  const auto steer = MotorIndex(config, Feedback::kRemoteCANcoder);
  ASSERT_EQ(config.motors[drive].sensor_to_mechanism_ratio, 6.75);
  const uint16_t drive_id = config.motors[drive].id;
  FakeBackend backend;
  Gateway g{config, backend, 1};
  backend.positions[drive_id] = 16383.8;
  g.Tick(1000000, true);
  // 1 ms plus 20 ms of slack at 250 / 6.75 rps allows 0.78 rotations; this
  // 0.75 wraps past 16384.
  backend.positions[drive_id] = -16383.45;
  backend.positions[config.motors[steer].id] = 5;
  g.Tick(1001000, true);
  // A frame held for 40 ms: the next one may come 61 ms of travel later.
  backend.position_ages[drive_id] = 40000;
  g.Tick(1002000, true);
  backend.position_ages[drive_id] = 0;
  backend.positions[drive_id] = -16381.45;
  g.Tick(1003000, true);
  // Read 100 ms after the tick began: 121 ms of travel.
  g.SetClock(LateReadClock);
  late_read_us = 1104000;
  backend.positions[drive_id] = -16377.95;
  g.Tick(1004000, true);
  const auto s = g.snapshot();
  EXPECT_EQ(s.counters.device_reset_count, 0u);
  EXPECT_TRUE(s.motors[drive].valid);
  EXPECT_TRUE(s.motors[steer].valid);
  EXPECT_TRUE(backend.restoring_ids.empty());
}

// A sample the gateway invalidates carries zero in every measured field, as
// one the backend invalidated does: a motor whose position is
// command_timeout_us old, a masked (unavailable) motor and a gyro whose yaw is
// that old all read zero position, velocity, voltage and currents, so a reader
// that skips connected() never takes the frozen values for a measurement.
// Valid samples in the same State keep theirs.
// REQ-CONTRACT-32
TEST(Gateway, InvalidSamplesCarryZeroMeasurements) {
  const auto config = SwerveConfig(true);
  const auto index_of = [&config](uint16_t id) {
    for (std::size_t i = 0; i < config.motors.size(); ++i) {
      if (config.motors[i].id == id) return i;
    }
    return config.motors.size();
  };
  const auto aged = index_of(1);    // back_left_drive
  const auto masked = index_of(4);  // back_right_drive
  const auto valid = index_of(8);   // front_left_drive
  std::size_t gyro = 0;
  while (config.sensors[gyro].kind != SensorKind::kPigeon2) ++gyro;
  FakeBackend backend;
  backend.measurement = 3;
  backend.unavailable = {config.motors[masked].id};
  for (const auto& motor : config.motors) backend.positions[motor.id] = 30;
  backend.position_ages[config.motors[aged].id] = config.command_timeout_us;
  backend.position_ages[config.sensors[gyro].id] = config.command_timeout_us;
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  const auto s = g.snapshot();
  for (const auto i : {aged, masked}) {
    SCOPED_TRACE(config.motors[i].name);
    const auto& m = s.motors[i];
    EXPECT_FALSE(m.valid);
    EXPECT_EQ(m.position_rot, 0.0);
    EXPECT_EQ(m.velocity_rps, 0.0);
    EXPECT_EQ(m.voltage, 0.0);
    EXPECT_EQ(m.stator_current_a, 0.0);
    EXPECT_EQ(m.supply_current_a, 0.0);
  }
  EXPECT_FALSE(s.sensors[gyro].valid);
  EXPECT_EQ(s.sensors[gyro].position_rot, 0.0);
  EXPECT_EQ(s.sensors[gyro].velocity_rps, 0.0);
  ASSERT_TRUE(s.motors[valid].valid);
  EXPECT_EQ(s.motors[valid].position_rot, 30.0);
  EXPECT_EQ(s.motors[valid].velocity_rps, 3.0);
  EXPECT_EQ(s.motors[valid].supply_current_a, 3.0);
}

// The zero an invalid sample carries is no position: a rotor-feedback motor at
// 30 rotations whose next sample is invalid (zeroed by the backend, or aged
// out at the gateway) and whose sample after that is valid at 30 again has not
// moved, so no reset is counted (a count change drops every Motor::rezero of
// a node) and nothing is restored; read as a position, the zero would be a
// step of 30 where 0.8 rotations is the bound.
// REQ-CONTRACT-32
TEST(Gateway, InvalidSampleBetweenEqualPositionsIsNoReset) {
  const auto config = SwerveConfig(true);
  const auto drive = MotorIndex(config, Feedback::kRotor);
  const uint16_t id = config.motors[drive].id;
  for (const bool aged : {false, true}) {
    SCOPED_TRACE(aged ? "aged out at the gateway" : "invalid from the backend");
    FakeBackend backend;
    Gateway g{config, backend, 1};
    backend.positions[id] = 30;
    g.Tick(1000, true);
    ASSERT_TRUE(g.snapshot().motors[drive].valid);
    if (aged) {
      backend.position_ages[id] = config.command_timeout_us;
    } else {
      backend.invalid_motor = id;
      backend.positions[id] = 0;
    }
    g.Tick(2000, true);
    ASSERT_FALSE(g.snapshot().motors[drive].valid);
    backend.position_ages.clear();
    backend.invalid_motor = 0;
    backend.positions[id] = 30;
    g.Tick(3000, true);
    const auto s = g.snapshot();
    EXPECT_TRUE(s.motors[drive].valid);
    EXPECT_EQ(s.counters.device_reset_count, 0u);
    EXPECT_TRUE(backend.restoring_ids.empty());
  }
}

TEST(Gateway, FailedDeviceConfigureAndRestoreLeaveOthersDriving) {
  FakeBackend backend;
  const auto config = SwerveConfig(true);
  const auto failed_id = config.motors[0].id;
  backend.unavailable = {failed_id};
  backend.restore_ok = false;
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  ASSERT_TRUE(g.configured());
  auto command = Request(g);
  for (std::size_t i = 0; i < command.count; ++i)
    command.motors[i] = {config.motors[i].id, Mode::kDutyCycle, 0, 0.2};
  ASSERT_TRUE(g.Accept(command, 1, 1000));
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests[1].mode, Mode::kDutyCycle);
  EXPECT_FALSE(g.snapshot().motors[0].valid);
  EXPECT_GT(g.snapshot().counters.neutralized_motor_count, 0u);
  g.Tick(250000, true);
  EXPECT_EQ(backend.restoring_ids, std::vector<uint16_t>{failed_id});
  g.Tick(251000, true);
  command.observed_time_us = 251000;
  command.epoch = g.snapshot().epoch;
  ASSERT_TRUE(g.Accept(command, 2, 251000));
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests[1].mode, Mode::kDutyCycle);
  backend.restore_ok = true;
  g.Tick(501000, true);
  g.Tick(502000, true);
  command.observed_time_us = 502000;
  command.epoch = g.snapshot().epoch;
  ASSERT_TRUE(g.Accept(command, 3, 502000));
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kDutyCycle);
  EXPECT_EQ(backend.last_requests[1].mode, Mode::kDutyCycle);
}

// A control-send failure on one motor quarantines only that motor and its
// follower; the rest of the command, DIO and PWM keep applying and no global
// fault latches however often it repeats. Restore brings the motor back.
// REQ-SAFE-23: a device-local apply failure neutralizes only its motor chain
TEST(Gateway, DeviceLocalApplyFailureIsolatesOnlyThatChain) {
  FakeBackend backend;
  auto config = SwerveConfig(true);
  config.motors[1].master_id = config.motors[0].id;
  config.digital_outputs.push_back({40, "solenoid", 1});
  config.pwm_outputs.push_back({41, "servo", 0});
  const auto failed_id = config.motors[0].id;
  Gateway g{config, backend, 1};
  const auto command_at = [&](uint64_t now) {
    auto c = Request(g);
    c.observed_time_us = now;
    for (std::size_t i = 0; i < c.count; ++i)
      c.motors[i] = {config.motors[i].id, Mode::kDutyCycle, 0, 0.2};
    c.digital_output_count = 1;
    c.digital_outputs[0] = {40, true};
    c.pwm_output_count = 1;
    c.pwm_outputs[0] = {41, 0.5};
    return c;
  };
  const auto expect_isolated = [&](const char* phase) {
    SCOPED_TRACE(phase);
    EXPECT_EQ(backend.last_requests[0].mode, Mode::kNeutral);
    EXPECT_EQ(backend.last_requests[1].mode, Mode::kNeutral);
    for (std::size_t m = 2; m < config.motors.size(); ++m)
      EXPECT_EQ(backend.last_requests[m].mode, Mode::kDutyCycle) << m;
    ASSERT_EQ(backend.last_digital_outputs.size(), 1u);
    EXPECT_TRUE(backend.last_digital_outputs[0].value);
    ASSERT_EQ(backend.last_pwm_outputs.size(), 1u);
    EXPECT_DOUBLE_EQ(backend.last_pwm_outputs[0].output, 0.5);
    const auto s = g.snapshot();
    EXPECT_FALSE(s.motors[0].valid);
    EXPECT_FALSE(s.motors[1].valid);
    EXPECT_FALSE(s.flags & kHardwareFault);
    EXPECT_TRUE(s.flags & kCommandActive);
    EXPECT_EQ(s.counters.apply_failure_count, 0u);
  };
  backend.failing_motors = {failed_id};
  uint64_t now = 1000;
  uint64_t sequence = 0;
  g.Tick(now, true);
  const auto epoch = g.snapshot().epoch;
  for (uint32_t i = 0; i < 2 * Gateway::kConsecutiveApplyFailureLimit; ++i) {
    const int neutrals = backend.neutral;
    ASSERT_TRUE(g.Accept(command_at(now), ++sequence, now)) << i;
    expect_isolated("repeated failure");
    EXPECT_EQ(backend.neutral, neutrals);  // No whole-robot neutral.
    now += 5000;
    g.Tick(now, true);
  }
  EXPECT_EQ(g.snapshot().epoch, epoch);

  // Restore is attempted while the motor still fails: the next fresh
  // observation retries it once and re-isolates it, still without a fault.
  now = 250000;
  g.Tick(now, true);
  EXPECT_EQ(backend.restoring_ids, std::vector<uint16_t>{failed_id});
  now += 5000;
  g.Tick(now, true);
  EXPECT_TRUE(g.snapshot().motors[0].valid);
  ASSERT_TRUE(g.Accept(command_at(now), ++sequence, now));
  expect_isolated("failure after restore");

  // Once sends succeed, the backed-off restore recovers the chain.
  backend.failing_motors.clear();
  const auto restores = g.restore_attempt_count();
  while (g.restore_attempt_count() == restores && now < 3000000) {
    EXPECT_FALSE(g.snapshot().motors[0].valid);  // Neutral until restored.
    now += 5000;
    g.Tick(now, true);
  }
  ASSERT_GT(g.restore_attempt_count(), restores);
  now += 5000;
  g.Tick(now, true);
  ASSERT_TRUE(g.snapshot().motors[0].valid);
  ASSERT_TRUE(g.Accept(command_at(now), ++sequence, now));
  for (std::size_t m = 0; m < config.motors.size(); ++m)
    EXPECT_EQ(backend.last_requests[m].mode, Mode::kDutyCycle) << m;
  EXPECT_FALSE(g.snapshot().flags & kHardwareFault);

  // Neither a stale observation nor disable can revive recovered outputs.
  const auto stale = command_at(now);
  now += config.command_timeout_us;
  const int before_timeout = backend.neutral;
  g.Tick(now, true);
  EXPECT_GT(backend.neutral, before_timeout);
  EXPECT_FALSE(g.Accept(stale, ++sequence, now));
  EXPECT_FALSE(g.snapshot().flags & kCommandActive);
  g.Tick(++now, false);
  const int applied = backend.applied;
  EXPECT_FALSE(g.Accept(command_at(now), ++sequence, now));
  EXPECT_EQ(backend.applied, applied);
  EXPECT_FALSE(g.snapshot().flags & kCommandActive);
}

// Only a genuinely global failure keeps invalidate + three-strike latch.
TEST(Gateway, EveryMotorOrOutputApplyFailureStillLatches) {
  for (bool outputs : {false, true}) {
    SCOPED_TRACE(outputs ? "outputs" : "every motor");
    FakeBackend backend;
    const auto config = SwerveConfig(true);
    Gateway g{config, backend, 1};
    g.Tick(1000, true);
    if (outputs) {
      backend.outputs_ok = false;
    } else {
      for (const auto& motor : config.motors)
        backend.failing_motors.push_back(motor.id);
    }
    const int neutrals = backend.neutral;
    for (uint32_t i = 0; i < Gateway::kConsecutiveApplyFailureLimit; ++i)
      EXPECT_FALSE(g.Accept(Request(g), i + 1, 1000));
    EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
    EXPECT_FALSE(g.snapshot().flags & kCommandActive);
    EXPECT_EQ(g.snapshot().counters.apply_failure_count,
              Gateway::kConsecutiveApplyFailureLimit);
    EXPECT_GE(
        backend.neutral,
        neutrals + static_cast<int>(Gateway::kConsecutiveApplyFailureLimit));
  }
}

// REQ-SAFE-23: a global apply failure latches even while a chain is isolated
TEST(Gateway, GlobalFailureDuringIsolationStillLatches) {
  FakeBackend backend;
  const auto config = SwerveConfig(true);
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  backend.failing_motors = {config.motors[0].id};
  backend.fail_all_after_neutral = true;
  for (uint32_t i = 0; i < Gateway::kConsecutiveApplyFailureLimit; ++i) {
    auto command = Request(g);
    for (std::size_t m = 0; m < command.count; ++m)
      command.motors[m] = {config.motors[m].id, Mode::kDutyCycle, 0, 0.2};
    EXPECT_FALSE(g.Accept(command, i + 1, 1000));
  }
  EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
  EXPECT_FALSE(g.snapshot().flags & kCommandActive);
  EXPECT_EQ(g.snapshot().counters.apply_failure_count,
            Gateway::kConsecutiveApplyFailureLimit);
}

TEST(Gateway, RemoteSensorResetIsolatesItsMotorAndFollower) {
  auto config = SwerveConfig(true);
  const auto steer = std::find_if(
      config.motors.begin(), config.motors.end(), [](const auto& motor) {
        return motor.feedback == Feedback::kRemoteCANcoder;
      });
  ASSERT_NE(steer, config.motors.end());
  const auto index = static_cast<std::size_t>(steer - config.motors.begin());
  auto follower = *steer;
  follower.id = 100;
  follower.name = "steer_follower";
  follower.can_id = 60;
  follower.master_id = steer->id;
  config.motors.push_back(follower);
  FakeBackend backend;
  backend.pending_ticks = 10;
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  auto command = Request(g);
  for (std::size_t i = 0; i < command.count; ++i)
    command.motors[i] = {config.motors[i].id, Mode::kDutyCycle, 0, 0.2};
  ASSERT_TRUE(g.Accept(command, 1, 1000));
  backend.resets = {config.motors[index].feedback_sensor_id};
  g.Tick(2000, true);
  EXPECT_EQ(backend.last_requests[index].mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests.back().mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kDutyCycle);
  EXPECT_TRUE(g.Accept(command, 2, 2000));
  EXPECT_EQ(backend.last_requests[index].mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests.back().mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kDutyCycle);
  g.Tick(3000, false);
  EXPECT_FALSE(g.Accept(command, 3, 3000));
  g.Tick(200000, true);
  EXPECT_FALSE(g.Accept(command, 4, 200000));
}

// Pushes `cfg` and ticks until the endpoint acknowledges it or begins a
// backend configure that has not finished (whose ACK comes later): nullopt.
std::optional<ConfigAckPayload> PushConfig(Endpoint& endpoint,
                                           const Gateway& gateway,
                                           Companion& companion,
                                           const Config& cfg,
                                           bool enabled = false) {
  for (const auto& chunk : CreateConfigChunks(cfg)) {
    EXPECT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig,
                                  protocol::kFlagNone, 1000, chunk.data(),
                                  chunk.size()),
              protocol::UdpStatus::kOk);
  }
  std::optional<ConfigAckPayload> ack;
  const bool answered = Eventually([&] {
    endpoint.Tick(2000, enabled);
    protocol::DecodedFrame frame;
    while (!ack && companion.TryReceive(&frame) == protocol::UdpStatus::kOk) {
      if (frame.header.type != protocol::FrameType::kHardwareConfigAck)
        continue;
      ack.emplace();
      EXPECT_TRUE(DecodeConfigAck({frame.payload, frame.payload_size}, *ack));
    }
    return ack.has_value() || gateway.configuring();
  });
  EXPECT_TRUE(answered) << "no ACK, and no configure begun";
  return ack;
}

TEST(Endpoint, FailedConfigurationAcknowledgementBoundsReason) {
  FakeBackend backend;
  backend.configure_ok = false;
  ConfigError err;
  err.device_name = "front_left_drive";
  err.bus = "rio";
  err.can_id = 3;
  err.operation = "apply_talonfx_config";
  err.vendor_status = -1;
  err.message = "timeout waiting for device response; retries exhausted";
  backend.config_error = err;
  Gateway gateway{backend, 3003};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  const auto ack = PushConfig(endpoint, gateway, companion, SwerveConfig(true));
  if (!ack.has_value()) {
    FAIL() << "no config ack";
  }
  EXPECT_EQ(ack->status, ConfigAckStatus::kBackendFailed);
  EXPECT_NE(std::find(std::begin(ack->reason), std::end(ack->reason), '\0'),
            std::end(ack->reason));
  EXPECT_EQ(ack->reason[sizeof(ack->reason) - 1], '\0');
}

// The ACK is a frame type, not a flag: its header flags stay zero no matter
// which incoming sequence it answers.
TEST(Endpoint, ConfigAckFlagsIgnoreIncomingSequence) {
  FakeBackend backend;
  Gateway gateway{backend, 3005};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  // Advance the companion's sequence so its low bits are nonzero flag bits if
  // the incoming sequence leaked into the ACK header.
  const std::array<uint8_t, 1> junk{};
  for (int i = 0; i < 0xff; ++i) {
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kDriverStation,
                                  protocol::kFlagNone, 1000, junk.data(),
                                  junk.size()),
              protocol::UdpStatus::kOk);
    if (i % 16 == 15) endpoint.Tick(1000, false);
  }
  for (const auto& chunk : CreateConfigChunks(SwerveConfig(true))) {
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig,
                                  protocol::kFlagNone, 1000, chunk.data(),
                                  chunk.size()),
              protocol::UdpStatus::kOk);
  }
  ASSERT_GT(companion.next_send_sequence(), 0xffu);
  std::optional<uint16_t> ack_flags;
  uint64_t tick = 2000;
  ASSERT_TRUE(Eventually([&] {
    endpoint.Tick(tick++, false);
    protocol::DecodedFrame frame;
    while (companion.TryReceive(&frame) == protocol::UdpStatus::kOk) {
      EXPECT_EQ(frame.header.flags, protocol::kFlagNone);
      if (frame.header.type == protocol::FrameType::kHardwareConfigAck)
        ack_flags = frame.header.flags;
    }
    return ack_flags.has_value();
  }));
  EXPECT_EQ(*ack_flags, protocol::kFlagNone);
}

TEST(Gateway, PendingRestoreKeepsPollingAndNeutral) {
  FakeBackend backend;
  backend.pending_ticks = 3;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  backend.resets = {1, 2};
  g.Tick(2000, true);
  for (uint64_t now = 3000; now <= 5000; now += 1000) {
    g.Tick(now, true);
    EXPECT_EQ(g.snapshot().sample_time_us, now);
    EXPECT_TRUE(g.Accept(Request(g), now, now));
    EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
    EXPECT_FALSE(g.snapshot().motors[0].valid);
  }
  g.Tick(6000, true);
  EXPECT_TRUE(g.Accept(Request(g), 6000, 6000));
  EXPECT_EQ(g.snapshot().counters.device_reset_count, 2u);
}

TEST(Gateway, RangeErrorsNeutralizeOnlyTheirMotor) {
  SimBackend backend;
  auto config = SwerveConfig(true);
  config.motors[0].soft_limits = true;
  config.motors[0].reverse_limit_rot = -2;
  config.motors[0].forward_limit_rot = 2;
  Gateway g{config, backend, 1};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::array<MotorRequest, 8> invalid{{
      {config.motors[0].id, Mode::kVelocity, 0, nan, 0},
      {config.motors[0].id, Mode::kVelocity, 0, 21, 0},
      {config.motors[0].id, Mode::kDutyCycle, 0, 1.1, 0},
      {config.motors[0].id, Mode::kVoltage, 0, 13, 0},
      {config.motors[0].id, Mode::kPosition, 0, -3, 0},
      {config.motors[0].id, Mode::kMotionMagic, 0, 3, 0},
      {config.motors[0].id, Mode::kVelocity, 0, 1, nan},
      {config.motors[0].id, Mode::kVelocity, 0, 1, 13},
  }};
  uint64_t now = 1000, sequence = 1;
  for (const auto& request : invalid) {
    g.Tick(now, true);
    auto command = Request(g);
    command.motors[0] = request;
    ASSERT_TRUE(g.Accept(command, sequence++, now));
    g.Tick(++now, true);
    EXPECT_DOUBLE_EQ(g.snapshot().motors[0].velocity_rps, 0);
    EXPECT_DOUBLE_EQ(g.snapshot().motors[1].velocity_rps, 1);
  }
  EXPECT_EQ(g.snapshot().counters.neutralized_motor_count, invalid.size());
  EXPECT_EQ(g.snapshot().counters.rejected_command_count, 0u);
}

TEST(ConfigWire, PersistenceValidatesCrcAndAssemblerResetsAfterCompletion) {
  const auto config = SwerveConfig(true);
  auto bytes = PersistedConfig(config);
  Config loaded;
  ASSERT_TRUE(LoadPersistedConfig(bytes, loaded));
  EXPECT_EQ(loaded, config);
  bytes[0] ^= 1;
  EXPECT_FALSE(LoadPersistedConfig(bytes, loaded));
  EXPECT_FALSE(LoadPersistedConfig({}, loaded));
  auto invalid = config;
  invalid.motors[0].max_voltage = -1;
  EXPECT_FALSE(LoadPersistedConfig(PersistedConfig(invalid), loaded));
  auto chunks = CreateConfigChunks(config, 200);
  ASSERT_GT(chunks.size(), 1u);
  ConfigAssembler assembler;
  std::string error;
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    EXPECT_EQ(assembler.AddChunk(chunks[i], error),
              i + 1 == chunks.size() ? ConfigAssembler::Status::kComplete
                                     : ConfigAssembler::Status::kIncomplete);
  }
  EXPECT_EQ(assembler.AddChunk(chunks[0], error),
            ConfigAssembler::Status::kIncomplete);
}

TEST(Endpoint, RestoredGatewayAcknowledgesIdenticalConfigWhileEnabled) {
  Config restored;
  ASSERT_TRUE(
      LoadPersistedConfig(PersistedConfig(SwerveConfig(true)), restored));
  FakeBackend backend;
  Gateway gateway{backend, 3010};
  ASSERT_TRUE(gateway.Reconfigure(restored));
  gateway.Tick(1000, true);
  const auto configure_calls = backend.configure_calls;
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  const auto ack = PushConfig(endpoint, gateway, companion, restored, true);
  if (!ack.has_value()) {
    FAIL() << "Expected configuration acknowledgement";
  }
  const auto& acknowledgement = *ack;
  EXPECT_EQ(acknowledgement.status, ConfigAckStatus::kOk);
  EXPECT_EQ(backend.configure_calls, configure_calls);
  EXPECT_TRUE(gateway.Accept(Request(gateway), 1, 2000));
}

TEST(Endpoint, PendingDegradedConfigureDoesNotBlockTicksAndRecoversDevice) {
  class PendingBackend : public FakeBackend {
   public:
    void BeginConfigure(const Config&) override { started = true; }
    ConfigureStatus PollConfigure() override {
      return complete ? ConfigureStatus::kDone : ConfigureStatus::kPending;
    }
    bool started{}, complete{};
  } backend;
  const auto config = SwerveConfig(true);
  backend.unavailable = {config.motors[0].id};
  Gateway gateway{backend, 3011};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  EXPECT_FALSE(PushConfig(endpoint, gateway, companion, config).has_value());
  EXPECT_TRUE(backend.started);
  EXPECT_EQ(backend.configure_calls, 0);
  EXPECT_TRUE(gateway.configuring());
  endpoint.Tick(7000, true);
  EXPECT_EQ(gateway.snapshot().sample_time_us, 7000u);
  EXPECT_FALSE(gateway.Accept(Request(gateway), 1, 7000));
  backend.complete = true;
  endpoint.Tick(12000, true);
  EXPECT_TRUE(gateway.configured());
  EXPECT_FALSE(gateway.snapshot().motors[0].valid);
  // The first ACK is the completed configure's: none went out while it was
  // pending.
  protocol::DecodedFrame frame;
  ASSERT_TRUE(
      companion.Receive(protocol::FrameType::kHardwareConfigAck, &frame));
  ConfigAckPayload ack{};
  ASSERT_TRUE(DecodeConfigAck({frame.payload, frame.payload_size}, ack));
  EXPECT_EQ(ack.status, ConfigAckStatus::kOk);
  auto command = Request(gateway);
  for (std::size_t i = 0; i < command.count; ++i)
    command.motors[i] = {config.motors[i].id, Mode::kDutyCycle, 0, 0.2};
  ASSERT_TRUE(gateway.Accept(command, 2, 12000));
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests[1].mode, Mode::kDutyCycle);
  endpoint.Tick(262000, true);
  endpoint.Tick(267000, true);
  command.observed_time_us = 267000;
  command.epoch = gateway.snapshot().epoch;
  ASSERT_TRUE(gateway.Accept(command, 3, 267000));
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kDutyCycle);
}

// After a stall the socket holds a command backlog; replaying it would put
// every superseded demand on the CAN bus back-to-back.
TEST(Endpoint, AppliesOnlyTheNewestQueuedCommand) {
  FakeBackend backend;
  Gateway gateway{backend, 3012};
  ASSERT_TRUE(gateway.Reconfigure(SwerveConfig(true)));
  gateway.Tick(1000, true);
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  auto command = Request(gateway);
  // Each command's demand is its sequence, so the applied demand names the
  // command it came from.
  const auto send_backlog = [&] {
    for (int i = 0; i < 5; ++i) {
      command.motors[0].demand =
          static_cast<double>(companion.next_send_sequence());
      std::array<uint8_t, kMaxCommandSize> bytes{};
      const auto size = Encode(command, bytes);
      ASSERT_GT(size, 0u);
      ASSERT_EQ(
          companion.SendFrame(protocol::FrameType::kHardwareCommand,
                              protocol::kFlagNone, 1000, bytes.data(), size),
          protocol::UdpStatus::kOk);
    }
  };
  send_backlog();
  // Tick until one tick drains at least two queued commands: datagrams may
  // become readable one at a time, and a tick that drains a single command
  // proves nothing here. Should they all trickle in, queue another backlog.
  ASSERT_TRUE(Eventually([&] {
    const auto sequence_before = gateway.snapshot().last_command_sequence;
    const int applied_before = backend.applied;
    EXPECT_EQ(endpoint.Tick(2000, true), protocol::UdpStatus::kOk);
    const auto sequence = gateway.snapshot().last_command_sequence;
    if (sequence - sequence_before >= 2) {
      EXPECT_EQ(backend.applied - applied_before, 1)
          << "commands " << sequence_before + 1 << " to " << sequence
          << " were drained by one tick";
      EXPECT_FALSE(backend.last_requests.empty());
      if (!backend.last_requests.empty()) {
        EXPECT_DOUBLE_EQ(backend.last_requests[0].demand,
                         static_cast<double>(sequence));
      }
      return true;
    }
    if (sequence == companion.next_send_sequence() - 1) send_backlog();
    return HasFailure();
  }));
  EXPECT_EQ(gateway.snapshot().counters.rejected_command_count, 0u);
}

}  // namespace
}  // namespace talos::hardware
