#include "gateway.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
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
  ASSERT_EQ(Encode(state, state_bytes), 1412u);
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
    for (auto& m : s.motors) m.valid = healthy && m.id != invalid_motor;
    for (auto& sensor : s.sensors) {
      sensor.valid = healthy && sensor.id != invalid_sensor;
    }
    return healthy;
  }
  bool Apply(std::span<const MotorRequest> requests) override {
    last_requests.assign(requests.begin(), requests.end());
    ++applied;
    return apply_ok;
  }
  void Neutral() override {
    ++neutral;
    events.push_back("neutral");
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
  }
  RestoreStatus PollRestore() override {
    if (pending_ticks-- > 0) return RestoreStatus::kPending;
    if (restore_ok) {
      std::erase_if(unavailable, [&](uint16_t id) {
        return std::find(restoring_ids.begin(), restoring_ids.end(), id) !=
               restoring_ids.end();
      });
    }
    return restore_ok ? RestoreStatus::kDone : RestoreStatus::kFailed;
  }
  int pending_ticks{};
  bool configure_ok{true}, healthy{true}, apply_ok{true}, restore_ok{true};
  int applied{}, neutral{}, configure_calls{};
  uint16_t invalid_motor{}, invalid_sensor{};
  std::vector<MotorRequest> last_requests{};
  std::vector<std::string> events{};
  std::vector<uint16_t> resets{};
  std::vector<uint16_t> restoring_ids;
  std::vector<uint16_t> unavailable;
  std::optional<ConfigError> config_error{};
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

TEST(ConfigWire, FollowersAndGravityRoundTrip) {
  auto config = SwerveConfig(true);
  config.motors[1].master_id = config.motors[0].id;
  config.motors[1].oppose_master = true;
  config.motors[1].slots[2].gravity_type = GravityType::kArmCosine;
  Config decoded;
  ASSERT_TRUE(DeserializeConfig(SerializeConfig(config), decoded));
  EXPECT_EQ(decoded, config);
  EXPECT_EQ(ConfigurationId(decoded), ConfigurationId(config));
}

TEST(ConfigWire, PigeonMountCalibrationRoundTripsAndReachesIdealBackend) {
  auto config = SwerveConfig(true);
  for (auto& sensor : config.sensors) {
    if (sensor.kind != SensorKind::kPigeon2) continue;
    sensor.offset_rot = 0.125;
    sensor.inverted = true;
  }
  EXPECT_NO_THROW(Validate(config));
  Config decoded;
  ASSERT_TRUE(DeserializeConfig(SerializeConfig(config), decoded));
  EXPECT_EQ(decoded, config);
  SimBackend backend;
  Gateway gateway{decoded, backend, 1};
  gateway.Tick(1000, false);
  const auto state = gateway.snapshot();
  for (std::size_t i = 0; i < decoded.sensors.size(); ++i) {
    if (decoded.sensors[i].kind != SensorKind::kPigeon2) continue;
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
TEST(Config, FingerprintIncludesCalibrationAndGains) {
  auto c = SwerveConfig(true);
  const auto id = ConfigurationId(c);
  c.sensors[0].offset_rot = 0.1;
  EXPECT_NE(id, ConfigurationId(c));
  c = SwerveConfig(true);
  c.motors[0].slots[1].p = 1;
  EXPECT_NE(id, ConfigurationId(c));
}
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
  const auto c = Request(g);
  std::array<uint8_t, protocol::kMaxPayloadSize> bytes{};
  const auto size = Encode(c, bytes);
  Command decoded;
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), size}, decoded));
  EXPECT_EQ(decoded.count, c.count);
  EXPECT_EQ(decoded.boot_id, c.boot_id);
  EXPECT_DOUBLE_EQ(decoded.motors[0].demand, 1);
  for (std::size_t i = 0; i < size; ++i) {
    EXPECT_FALSE(Decode(std::span<const uint8_t>{bytes.data(), i}, decoded));
  }
  EXPECT_FALSE(
      Decode(std::span<const uint8_t>{bytes.data(), size + 1}, decoded));
  bytes[32] = 255;
  EXPECT_FALSE(Decode(std::span<const uint8_t>{bytes.data(), size}, decoded));

  // Empty Command
  Command empty_cmd{};
  const auto empty_cmd_size = Encode(empty_cmd, bytes);
  ASSERT_GT(empty_cmd_size, 0u);
  Command decoded_empty_cmd;
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), empty_cmd_size},
                     decoded_empty_cmd));
  EXPECT_EQ(decoded_empty_cmd.count, 0u);
  EXPECT_EQ(decoded_empty_cmd.digital_output_count, 0u);
  EXPECT_EQ(decoded_empty_cmd.pwm_output_count, 0u);

  // Full Command with every actuator kind
  Command full_cmd{};
  full_cmd.config_id = 111;
  full_cmd.boot_id = 222;
  full_cmd.epoch = 333;
  full_cmd.observed_time_us = 444;
  full_cmd.count = kMaxMotors;
  full_cmd.digital_output_count = kMaxDigitalOutputs;
  full_cmd.pwm_output_count = kMaxPwmOutputs;
  for (std::size_t i = 0; i < kMaxMotors; ++i) {
    full_cmd.motors[i] = {static_cast<uint16_t>(i + 1), Mode::kVelocity, 1,
                          15.5 + static_cast<double>(i), 1.2};
  }
  for (std::size_t i = 0; i < kMaxDigitalOutputs; ++i) {
    full_cmd.digital_outputs[i] = {static_cast<uint16_t>(100 + i),
                                   (i % 2) == 1};
  }
  for (std::size_t i = 0; i < kMaxPwmOutputs; ++i) {
    full_cmd.pwm_outputs[i] = {static_cast<uint16_t>(200 + i),
                               0.5 - 0.1 * static_cast<double>(i)};
  }
  const auto full_cmd_size = Encode(full_cmd, bytes);
  ASSERT_GT(full_cmd_size, 0u);
  ASSERT_LE(full_cmd_size, protocol::kMaxPayloadSize);
  Command decoded_full_cmd;
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), full_cmd_size},
                     decoded_full_cmd));
  EXPECT_EQ(decoded_full_cmd.count, kMaxMotors);
  EXPECT_EQ(decoded_full_cmd.digital_output_count, kMaxDigitalOutputs);
  EXPECT_EQ(decoded_full_cmd.pwm_output_count, kMaxPwmOutputs);
  EXPECT_EQ(decoded_full_cmd.digital_outputs[1].value, true);
  EXPECT_DOUBLE_EQ(decoded_full_cmd.pwm_outputs[0].output, 0.5);

  // Empty State
  State empty_state{};
  const auto empty_state_size = Encode(empty_state, bytes);
  ASSERT_GT(empty_state_size, 0u);
  State decoded_empty_state;
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), empty_state_size},
                     decoded_empty_state));
  EXPECT_EQ(decoded_empty_state.motor_count, 0u);
  EXPECT_EQ(decoded_empty_state.sensor_count, 0u);
  EXPECT_EQ(decoded_empty_state.digital_input_count, 0u);
  EXPECT_EQ(decoded_empty_state.digital_output_count, 0u);
  EXPECT_EQ(decoded_empty_state.analog_input_count, 0u);
  EXPECT_EQ(decoded_empty_state.encoder_count, 0u);
  EXPECT_EQ(decoded_empty_state.pwm_output_count, 0u);

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
      44 + 14 + 34 + kMaxMotors * 47 + kMaxSensors * 27 +
      kMaxDigitalInputs * 4 + kMaxDigitalOutputs * 4 + kMaxAnalogInputs * 15 +
      kMaxEncoders * 23 + kMaxPwmOutputs * 11;
  ASSERT_EQ(state_size, kWidestState);
  ASSERT_LE(state_size, protocol::kMaxPayloadSize);
  State copy;
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), state_size}, copy));
  EXPECT_EQ(copy, state);
  EXPECT_EQ(copy.motors[0].position_age_us, 123u);
  EXPECT_EQ(copy.sensors[0].velocity_age_us, 1011u);
  EXPECT_EQ(copy.digital_inputs[0].value, true);
  EXPECT_EQ(copy.digital_outputs[1].value, true);
  EXPECT_DOUBLE_EQ(copy.analog_inputs[1].voltage, 3.3);
  EXPECT_DOUBLE_EQ(copy.encoders[2].position_rot, 20.0);
  EXPECT_DOUBLE_EQ(copy.pwm_outputs[3].output, 0.75);

  for (std::size_t i = 0; i < state_size; ++i) {
    EXPECT_FALSE(Decode(std::span<const uint8_t>{bytes.data(), i}, copy));
  }
}

TEST(SimBackend, ModelsAllDeviceKindsIdeally) {
  Config config;
  config.commissioned = true;
  config.motors.push_back(
      {1, "motor1", 1, "rio", false, true, 40, 80,  12, Feedback::kRotor,
       0, 1,        1, false, false, -1,   1,  100, 20, 40,
       0});
  config.sensors.push_back(
      {2, "sensor1", SensorKind::kCANcoder, 2, "rio", false, 0});
  config.digital_inputs.push_back({3, "beam_break", 0});
  config.digital_outputs.push_back({4, "solenoid", 1});
  config.analog_inputs.push_back({5, "pressure", 0});
  config.encoders.push_back(
      {6, "quad_enc", EncoderKind::kQuadrature, 2, 3, 2048, 0});
  config.pwm_outputs.push_back({7, "servo", 0});

  SimBackend backend;
  Gateway g{config, backend, 10};

  // Set inputs on sim backend
  backend.SetDigitalInput(3, true);
  backend.SetAnalogInput(5, 4.5);
  backend.SetEncoder(6, 12.34, 5.67);

  g.Tick(1000, true);
  State s = g.snapshot();
  EXPECT_EQ(s.digital_inputs[0].id, 3);
  EXPECT_EQ(s.digital_inputs[0].value, true);
  EXPECT_EQ(s.analog_inputs[0].id, 5);
  EXPECT_DOUBLE_EQ(s.analog_inputs[0].voltage, 4.5);
  EXPECT_EQ(s.encoders[0].id, 6);
  EXPECT_DOUBLE_EQ(s.encoders[0].position_rot, 12.34);
  EXPECT_DOUBLE_EQ(s.encoders[0].velocity_rps, 5.67);

  // Send outputs via Command
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

  EXPECT_TRUE(g.Accept(cmd, 1, 1000));
  EXPECT_TRUE(backend.GetDigitalOutput(4));
  EXPECT_DOUBLE_EQ(backend.GetPwm(7), 0.85);

  // Tick again and verify readback
  g.Tick(6000, true);
  s = g.snapshot();
  EXPECT_EQ(s.digital_outputs[0].value, true);
  EXPECT_DOUBLE_EQ(s.pwm_outputs[0].output, 0.85);
}

TEST(ConfigWire, ChunksAndAssemblesConfigLosslessly) {
  auto cfg = SwerveConfig(false);
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
    EXPECT_FALSE(error.empty());
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
  EXPECT_FALSE(error.empty());
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
  EXPECT_NE(res.reason.find("stator limit"), std::string::npos);

  auto chunks = CreateConfigChunks(cfg);
  ConfigAssembler assembler;
  std::string err;
  ConfigAssembler::Status status = ConfigAssembler::Status::kIncomplete;
  for (const auto& ch : chunks) {
    status = assembler.AddChunk(ch, err);
  }
  EXPECT_EQ(status, ConfigAssembler::Status::kCeilingExceeded);
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

TEST(Endpoint, OutboundBlackoutPreservesReceiveAndSafetyTicks) {
  SimBackend backend;
  Gateway gateway{backend, 2002};
  Endpoint endpoint{gateway};
  ASSERT_EQ(endpoint.Open("127.0.0.1", 15812, "127.0.0.1", 15813),
            protocol::UdpStatus::kOk);
  protocol::RuntimeUdpPeer companion;
  ASSERT_EQ(companion.Open("127.0.0.1", 15813, "127.0.0.1", 15812),
            protocol::UdpStatus::kOk);

  endpoint.SetSendEnabled(false);
  const auto cfg = SwerveConfig(true);
  const auto chunks = CreateConfigChunks(cfg);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig, i + 1,
                                  1000, chunks[i].data(), chunks[i].size()),
              protocol::UdpStatus::kOk);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  ASSERT_EQ(endpoint.Tick(2000, false), protocol::UdpStatus::kOk);
  EXPECT_TRUE(gateway.configured());  // Receives config, but drops its ACK.
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
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  ASSERT_EQ(endpoint.Tick(4000, true), protocol::UdpStatus::kOk);
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
  protocol::DecodedFrame frame;
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_EQ(companion.TryReceive(&frame), protocol::UdpStatus::kWouldBlock);

  endpoint.SetSendEnabled(true);
  const uint64_t resumed_at = 6000 + cfg.command_timeout_us;
  ASSERT_EQ(endpoint.Tick(resumed_at, false), protocol::UdpStatus::kOk);
  ASSERT_EQ(endpoint.SendFrame(protocol::FrameType::kDriverStation, resumed_at,
                               ds_payload.data(), ds_payload.size()),
            protocol::UdpStatus::kOk);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  ASSERT_EQ(companion.TryReceive(&frame), protocol::UdpStatus::kOk);
  EXPECT_EQ(frame.header.type, protocol::FrameType::kHardwareState);
  State state;
  ASSERT_TRUE(Decode({frame.payload, frame.payload_size}, state));
  EXPECT_EQ(state.sample_time_us, resumed_at);
  EXPECT_FALSE(state.flags & kEnabled);
  ASSERT_EQ(companion.TryReceive(&frame), protocol::UdpStatus::kOk);
  EXPECT_EQ(frame.header.type, protocol::FrameType::kDriverStation);
  EXPECT_EQ(companion.TryReceive(&frame), protocol::UdpStatus::kWouldBlock);
}

TEST(Endpoint, PushesConfigOverUdpAndHandlesEpochChangeOnRestart) {
  SimBackend backend;
  Gateway gateway{backend, 2002};
  Endpoint endpoint{gateway};
  ASSERT_EQ(endpoint.Open("127.0.0.1", 15802, "127.0.0.1", 15803),
            protocol::UdpStatus::kOk);

  protocol::RuntimeUdpPeer companion;
  ASSERT_EQ(companion.Open("127.0.0.1", 15803, "127.0.0.1", 15802),
            protocol::UdpStatus::kOk);

  auto poll_receive = [](protocol::RuntimeUdpPeer& peer,
                         protocol::DecodedFrame* out) {
    for (int i = 0; i < 50; ++i) {
      auto status = peer.TryReceive(out);
      if (status == protocol::UdpStatus::kOk) return status;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return peer.TryReceive(out);
  };

  // 1. Companion ticks and sees state with config_id = 0
  endpoint.Tick(1000, true);

  protocol::DecodedFrame frame;
  ASSERT_EQ(poll_receive(companion, &frame), protocol::UdpStatus::kOk);
  EXPECT_EQ(frame.header.type, protocol::FrameType::kHardwareState);
  State state;
  ASSERT_TRUE(Decode({frame.payload, frame.payload_size}, state));
  EXPECT_EQ(state.config_id, 0u);
  EXPECT_EQ(state.boot_id, 2002u);

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
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  endpoint.Tick(2000, false);
  EXPECT_TRUE(gateway.configured());

  // 4. Companion receives ACK
  protocol::DecodedFrame ack_frame;
  ASSERT_EQ(poll_receive(companion, &ack_frame), protocol::UdpStatus::kOk);
  EXPECT_EQ(ack_frame.header.type, protocol::FrameType::kHardwareConfigAck);
  ConfigAckPayload ack{};
  ASSERT_TRUE(
      DecodeConfigAck({ack_frame.payload, ack_frame.payload_size}, ack));
  EXPECT_EQ(ack.status, ConfigAckStatus::kOk);
  EXPECT_EQ(ack.config_id, ConfigurationId(cfg));

  // 5. Subsequent state from endpoint has config_id set and kConfigured set
  endpoint.Tick(3000, true);
  protocol::DecodedFrame state_frame;
  ASSERT_EQ(poll_receive(companion, &state_frame), protocol::UdpStatus::kOk);
  EXPECT_EQ(state_frame.header.type, protocol::FrameType::kHardwareState);
  ASSERT_TRUE(Decode({state_frame.payload, state_frame.payload_size}, state));
  EXPECT_EQ(state.config_id, ConfigurationId(cfg));
  EXPECT_TRUE(state.flags & kConfigured);
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

TEST(Gateway, ResetWhileEnabledNeutralizesBeforeRestore) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  auto stale = Request(g);
  ASSERT_TRUE(g.Accept(stale, 1, 1000));
  const auto epoch = g.snapshot().epoch;
  backend.events.clear();
  backend.resets.push_back(1);
  // Enabled: reset recovery still runs — the device is already at defaults —
  // but from the safe state, not around live demand.
  g.Tick(2000, true);
  ASSERT_EQ(backend.events.size(), 1u);
  EXPECT_EQ(backend.events[0], "restore");
  EXPECT_EQ(g.snapshot().epoch, epoch);
  EXPECT_TRUE(g.snapshot().flags & kCommandActive);
  EXPECT_TRUE(g.Accept(stale, 2, 2000));
  g.Tick(3000, true);
  EXPECT_TRUE(g.Accept(Request(g), 3, 3000));
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
  backend.SimulateDeviceReset(gateway.snapshot().motors[0].id);
  gateway.Tick(106000, true);
  const double position = gateway.snapshot().motors[0].position_rot;
  ASSERT_NE(position, 0.0);
  EXPECT_EQ(gateway.snapshot().epoch, epoch);
  EXPECT_TRUE(gateway.Accept(stale, 200000, 106000));
  gateway.Tick(111000, true);
  EXPECT_DOUBLE_EQ(gateway.snapshot().motors[0].position_rot, position);
  EXPECT_DOUBLE_EQ(gateway.snapshot().motors[0].velocity_rps, 0.0);
  EXPECT_DOUBLE_EQ(gateway.snapshot().motors[0].voltage, 0.0);
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

std::optional<ConfigAckPayload> PushConfigAndAwaitAck(
    Endpoint& endpoint, protocol::RuntimeUdpPeer& companion, const Config& cfg,
    bool enabled = false) {
  auto chunks = CreateConfigChunks(cfg);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    EXPECT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig, i + 1,
                                  1000, chunks[i].data(), chunks[i].size()),
              protocol::UdpStatus::kOk);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  endpoint.Tick(2000, enabled);
  for (int i = 0; i < 100; ++i) {
    protocol::DecodedFrame frame;
    if (companion.TryReceive(&frame) != protocol::UdpStatus::kOk) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (frame.header.type != protocol::FrameType::kHardwareConfigAck) continue;
    ConfigAckPayload ack{};
    EXPECT_TRUE(DecodeConfigAck({frame.payload, frame.payload_size}, ack));
    return ack;
  }
  return std::nullopt;
}

TEST(Endpoint, ConfigAckReasonCarriesFailingDeviceAndCause) {
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
  ASSERT_EQ(endpoint.Open("127.0.0.1", 15804, "127.0.0.1", 15805),
            protocol::UdpStatus::kOk);
  protocol::RuntimeUdpPeer companion;
  ASSERT_EQ(companion.Open("127.0.0.1", 15805, "127.0.0.1", 15804),
            protocol::UdpStatus::kOk);
  const auto ack =
      PushConfigAndAwaitAck(endpoint, companion, SwerveConfig(true));
  if (!ack.has_value()) {
    FAIL() << "no config ack";
  }
  EXPECT_EQ(ack->status, ConfigAckStatus::kBackendFailed);
  const std::string reason(ack->reason);
  EXPECT_NE(reason.find("front_left_drive"), std::string::npos);
  EXPECT_NE(reason.find("timeout"), std::string::npos);
  // Truncated to the 64-byte field, always NUL-terminated. The long cause
  // caps the reason at 63 chars.
  EXPECT_LT(reason.size(), sizeof(ack->reason));
  EXPECT_EQ(reason.size(), sizeof(ack->reason) - 1);
  EXPECT_EQ(ack->reason[sizeof(ack->reason) - 1], '\0');
}

TEST(Endpoint, ConfigAckReasonFallsBackWithoutDetailedError) {
  FakeBackend backend;
  backend.configure_ok = false;  // No config_error: backend said nothing more.
  Gateway gateway{backend, 3004};
  Endpoint endpoint{gateway};
  ASSERT_EQ(endpoint.Open("127.0.0.1", 15806, "127.0.0.1", 15807),
            protocol::UdpStatus::kOk);
  protocol::RuntimeUdpPeer companion;
  ASSERT_EQ(companion.Open("127.0.0.1", 15807, "127.0.0.1", 15806),
            protocol::UdpStatus::kOk);
  const auto ack =
      PushConfigAndAwaitAck(endpoint, companion, SwerveConfig(true));
  if (!ack.has_value()) {
    FAIL() << "no config ack";
  }
  EXPECT_EQ(ack->status, ConfigAckStatus::kBackendFailed);
  EXPECT_EQ(std::string(ack->reason), "Backend configuration failed");
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
  ASSERT_EQ(endpoint.Open("127.0.0.1", 15810, "127.0.0.1", 15811),
            protocol::UdpStatus::kOk);
  protocol::RuntimeUdpPeer companion;
  ASSERT_EQ(companion.Open("127.0.0.1", 15811, "127.0.0.1", 15810),
            protocol::UdpStatus::kOk);
  const auto ack = PushConfigAndAwaitAck(endpoint, companion, restored, true);
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
  ASSERT_EQ(endpoint.Open("127.0.0.1", 15812, "127.0.0.1", 15813),
            protocol::UdpStatus::kOk);
  protocol::RuntimeUdpPeer companion;
  ASSERT_EQ(companion.Open("127.0.0.1", 15813, "127.0.0.1", 15812),
            protocol::UdpStatus::kOk);
  EXPECT_FALSE(PushConfigAndAwaitAck(endpoint, companion, config).has_value());
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
  ConfigAckPayload ack{};
  bool received = false;
  protocol::DecodedFrame frame;
  while (companion.TryReceive(&frame) == protocol::UdpStatus::kOk) {
    if (frame.header.type != protocol::FrameType::kHardwareConfigAck) continue;
    ASSERT_TRUE(DecodeConfigAck({frame.payload, frame.payload_size}, ack));
    received = true;
  }
  ASSERT_TRUE(received);
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

}  // namespace
}  // namespace talos::hardware
