#include "gateway.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "config_wire.h"
#include "gateway_test_support.h"
#include "sim_backend.h"
#include "sim_physics.h"
#include "talOS/protocol/frame.h"

namespace talos::hardware {
namespace {

using namespace gateway_test;  // NOLINT(google-build-using-namespace)

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
  ASSERT_EQ(Encode(state, state_bytes), 1419u);
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
  ASSERT_EQ(Encode(command, command_bytes), 414u);
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

// Each State carries one CAN bus's health. Every field crosses the wire
// exactly; a bus count above kMaxCanBuses, an index the count has no slot
// for, a read state this build does not know or a utilization that is no
// number is a malformed State, and none of them encodes.
// REQ-OPS-20, REQ-CONTRACT-30
TEST(Messages, CanBusSlotRoundTripsAndRejectsWhatItCannotPlace) {
  State state;
  state.health = {.valid = true,
                  .battery_v = 12.5,
                  .can_bus_count = 3,
                  .can_bus_index = 2,
                  .can_bus = {CanBusReadState::kFailed, 0.375F, 70000, 9, 255,
                              128, 0x01020304}};
  std::array<uint8_t, kMaxStateSize> bytes{};
  const auto size = Encode(state, bytes);
  ASSERT_NE(size, 0u);
  State decoded;
  ASSERT_TRUE(Decode(std::span{bytes}.first(size), decoded));
  EXPECT_EQ(decoded.health, state.health);

  // A State with no device: the health sample follows the 84-byte prefix
  // (valid, battery f64, brownout, then the bus count and index, the read
  // state and the utilization f32).
  constexpr std::size_t kCount = 84 + 10;
  constexpr std::size_t kIndex = kCount + 1;
  constexpr std::size_t kReadState = kCount + 2;
  constexpr std::size_t kUtilization = kCount + 3;
  struct Corruption {
    const char* what;
    std::size_t offset;
    std::vector<uint8_t> value;
  };
  const Corruption corruptions[] = {
      {"count above kMaxCanBuses", kCount, {5}},
      {"index at the count", kIndex, {3}},
      {"index past a zero count's one slot", kCount, {0}},
      {"read state above kSimulated", kReadState, {6}},
      {"NaN utilization", kUtilization, {0x00, 0x00, 0xc0, 0x7f}},
      {"infinite utilization", kUtilization, {0x00, 0x00, 0x80, 0x7f}},
  };
  for (const auto& corruption : corruptions) {
    SCOPED_TRACE(corruption.what);
    auto copy = bytes;
    std::copy(corruption.value.begin(), corruption.value.end(),
              copy.begin() + static_cast<std::ptrdiff_t>(corruption.offset));
    EXPECT_FALSE(Decode(std::span{copy}.first(size), decoded));
  }

  // No bus at all (a State before any configuration): count 0, index 0.
  EXPECT_TRUE(Decode(std::span{bytes}.first(Encode(State{}, bytes)), decoded));

  for (const HealthSample& unplaceable :
       {HealthSample{.can_bus_count = 5},
        HealthSample{.can_bus_count = 3, .can_bus_index = 3},
        HealthSample{.can_bus_index = 1},
        HealthSample{
            .can_bus_count = 1,
            .can_bus = {.read_state = static_cast<CanBusReadState>(6)}}}) {
    state.health = unplaceable;
    EXPECT_EQ(Encode(state, bytes), 0u)
        << "count " << int{unplaceable.can_bus_count} << " index "
        << int{unplaceable.can_bus_index};
  }
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

// A Command the main processor and the RoboRIO agree on byte for byte,
// whatever either host's byte order: every integer and IEEE 754 double
// little-endian, no padding. Each device count differs from the others, so
// two counts written in each other's places fail.
// REQ-CONTRACT-03
TEST(Messages, CommandIsLittleEndianOnTheWire) {
  Command command;
  command.config_id = 0x0102030405060708ull;
  command.boot_id = 0x1112131415161718ull;
  command.epoch = 0x2122232425262728ull;
  command.observed_time_us = 0x3132333435363738ull;
  command.count = 1;
  command.motors[0] = {0x0102, Mode::kVoltage, 1, 1.0, -2.0};
  command.digital_output_count = 2;
  command.digital_outputs[0] = {0x0304, true};
  command.digital_outputs[1] = {0x0305, false};
  command.pwm_output_count = 3;
  command.pwm_outputs[0] = {0x0506, PwmMode::kPulse, 0.5};
  command.pwm_outputs[1] = {0x0507, PwmMode::kOff, 0.0};
  command.pwm_outputs[2] = {0x0508, PwmMode::kPulse, -1.0};
  const std::vector<uint8_t> expected{
      0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,   // config_id
      0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11,   // boot_id
      0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21,   // epoch
      0x38, 0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31,   // observed_time_us
      1,    0,    2,    0,    3,    0,                  // counts
      0x02, 0x01, 2,    1,                              // motor id, mode, slot
      0,    0,    0,    0,    0,    0,    0xF0, 0x3F,   // demand 1.0
      0,    0,    0,    0,    0,    0,    0,    0xC0,   // feedforward_v -2.0
      0x04, 0x03, 1,    0x05, 0x03, 0,                  // digital outputs
      0x06, 0x05, 1,                                    // pwm id, pulse
      0,    0,    0,    0,    0,    0,    0xE0, 0x3F,   // output 0.5
      0x07, 0x05, 0,                                    // pwm id, off
      0,    0,    0,    0,    0,    0,    0,    0,      // output +0.0
      0x08, 0x05, 1,                                    // pwm id, pulse
      0,    0,    0,    0,    0,    0,    0xF0, 0xBF};  // output -1.0
  std::array<uint8_t, kMaxCommandSize> bytes{};
  const auto size = Encode(command, bytes);
  ASSERT_EQ(size, expected.size());
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), bytes.begin()));
  Command decoded;
  ASSERT_TRUE(Decode(expected, decoded));
  EXPECT_EQ(decoded, command);
}

// A PWM request is off (no pulse) or a pulse, and off has one byte string:
// mode 0 and output +0.0. A pulse of 0 (the 1.5 ms centre, which centres a
// servo) and off both round-trip as sent, never one for the other. A mode
// byte above kLastPwmMode, or an off request whose output bytes are not all
// zero (-0.0 included), neither encodes nor decodes: a peer reading such
// bytes cannot know which of the two was meant.
// REQ-SAFE-03, REQ-CONTRACT-03
TEST(Messages, PwmRequestIsOffOrAPulseAndOffHasOneEncoding) {
  Command command;
  command.pwm_output_count = 2;
  command.pwm_outputs[0] = {0x0506, PwmMode::kOff, 0.0};
  command.pwm_outputs[1] = {0x0708, PwmMode::kPulse, 0.0};
  std::array<uint8_t, kMaxCommandSize> bytes{};
  const auto size = Encode(command, bytes);
  ASSERT_EQ(size, kCommandBaseSize + 2 * kPwmRequestSize);
  Command decoded;
  ASSERT_TRUE(Decode(std::span{bytes}.first(size), decoded));
  EXPECT_EQ(decoded.pwm_outputs[0], command.pwm_outputs[0]);
  EXPECT_EQ(decoded.pwm_outputs[1], command.pwm_outputs[1]);

  // Byte offsets of the first request's mode and output.
  const std::size_t mode_at = kCommandBaseSize + 2;
  const std::size_t output_at = mode_at + 1;
  const struct {
    const char* label;
    PwmRequest request;
    std::size_t byte;
    uint8_t value;
  } rows[] = {
      {"mode 2", {0x0506, static_cast<PwmMode>(2), 0.0}, mode_at, 2},
      {"mode 255", {0x0506, static_cast<PwmMode>(255), 0.0}, mode_at, 255},
      {"off with a nonzero output",
       {0x0506, PwmMode::kOff, 0.5},
       output_at + 7,
       0x3F},
      {"off with output -0.0",
       {0x0506, PwmMode::kOff, -0.0},
       output_at + 7,
       0x80},
  };
  for (const auto& row : rows) {
    SCOPED_TRACE(row.label);
    Command bad = command;
    bad.pwm_outputs[0] = row.request;
    std::array<uint8_t, kMaxCommandSize> refused{};
    EXPECT_EQ(Encode(bad, refused), 0u);
    auto corrupted = bytes;
    corrupted[row.byte] = row.value;
    EXPECT_FALSE(Decode(std::span{corrupted}.first(size), decoded));
  }
}

// The motor mode byte runs to kLastMode, torque current (6), and no further:
// a torque-current request round-trips as sent, and a mode byte past it is
// refused, so a peer of a later protocol is never misread.
// REQ-SAFE-43, REQ-CONTRACT-03
TEST(Messages, MotorModeRunsToTorqueCurrentAndNoFurther) {
  Command command;
  command.count = 1;
  command.motors[0] = {0x0102, Mode::kTorqueCurrent, 0, -2.5, 0.0};
  std::array<uint8_t, kMaxCommandSize> bytes{};
  const auto size = Encode(command, bytes);
  ASSERT_EQ(size, kCommandBaseSize + kMotorRequestSize);
  const std::size_t mode_at = kCommandBaseSize + 2;
  ASSERT_EQ(bytes[mode_at], 6);
  Command decoded;
  ASSERT_TRUE(Decode(std::span{bytes}.first(size), decoded));
  EXPECT_EQ(decoded, command);
  bytes[mode_at] = 7;
  EXPECT_FALSE(Decode(std::span{bytes}.first(size), decoded));
}

// The State the RoboRIO sends, byte for byte: integers, doubles, the float
// CAN utilization and the signed-tenths diagnostics all little-endian. One
// device of every kind leads its section, with the rest of its count zero
// samples; each device count differs from the others, so two counts or two
// sections written in each other's places fail.
// REQ-CONTRACT-03
TEST(Messages, StateIsLittleEndianOnTheWire) {
  State state;
  state.config_id = 0x0102030405060708ull;
  state.boot_id = 0x1112131415161718ull;
  state.epoch = 0x2122232425262728ull;
  state.sample_time_us = 0x3132333435363738ull;
  state.last_command_sequence = 0x4142434445464748ull;
  state.clock_probe_time_us = 0x5152535455565758ull;
  state.sequence = 0x6162636465666768ull;
  state.build = {0x7172737475767778ull, GatewayProgram::kRoboRio, true};
  state.flags = 0x0A0B0C0D;
  state.motor_count = 5;
  state.sensor_count = 6;
  state.digital_input_count = 7;
  state.digital_output_count = 4;
  state.analog_input_count = 2;
  state.encoder_count = 1;
  state.pwm_output_count = 3;
  auto& motor = state.motors[0];
  motor.id = 0x0708;
  motor.valid = true;
  motor.position_rot = 1.0;
  motor.velocity_rps = -2.0;
  motor.voltage = 0.5;
  motor.stator_current_a = 12.3;
  motor.position_age_us = 0x01020304;
  motor.velocity_age_us = 0x05060708;
  motor.fault_flags = 0x090A0B0C;
  motor.temperature_c = -4.5;
  motor.supply_current_a = 0.1;
  state.sensors[0] = {0x0910, true, 0.25, -0.5, 0x11121314, 0x15161718};
  state.digital_inputs[0] = {0x0A0B, true, true};
  state.digital_outputs[0] = {0x0C0D, true, false};
  state.analog_inputs[0] = {0x0E0F, true, 2.5, 0x21222324};
  state.encoders[0] = {0x1011, true, -2.0, 4.0, 0x25262728};
  state.pwm_outputs[0] = {0x1213, true, -0.25};
  state.health.valid = true;
  state.health.battery_v = 12.5;
  state.health.can_bus_count = 2;
  state.health.can_bus_index = 1;
  state.health.can_bus = {
      CanBusReadState::kOk, 0.25f, 0x11121314, 0x21222324, 5, 6, 0x31323334};
  state.counters = {0x41424344, 0x51525354, 0x61626364, 0x71727374};
  std::vector<uint8_t> expected{
      0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,  // config_id
      0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11,  // boot_id
      0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21,  // epoch
      0x38, 0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31,  // sample_time_us
      0x48, 0x47, 0x46, 0x45, 0x44, 0x43, 0x42, 0x41,  // last_command_sequence
      0x58, 0x57, 0x56, 0x55, 0x54, 0x53, 0x52, 0x51,  // clock_probe_time_us
      0x68, 0x67, 0x66, 0x65, 0x64, 0x63, 0x62, 0x61,  // sequence
      0x78, 0x77, 0x76, 0x75, 0x74, 0x73, 0x72, 0x71,  // build.revision
      1,    1,                                         // program, dirty
      0x0D, 0x0C, 0x0B, 0x0A,                          // flags
      5,    0,    6,    0,    7,    0,    4,    0,     // motor, sensor, DI, DO
      2,    0,    1,    0,    3,    0};                // analog, encoder, PWM
  // `first`, then `more` samples of its size, all zero.
  const auto section = [&expected](std::initializer_list<uint8_t> first,
                                   std::size_t more) {
    expected.insert(expected.end(), first);
    expected.insert(expected.end(), more * first.size(), uint8_t{0});
  };
  section({0x08, 0x07, 1,                                   // motor id, valid
           0,    0,    0,    0,    0,    0,    0xF0, 0x3F,  // position 1.0
           0,    0,    0,    0,    0,    0,    0,    0xC0,  // velocity -2.0
           0,    0,    0,    0,    0,    0,    0xE0, 0x3F,  // voltage 0.5
           123,  0,                                         // stator 12.3 A
           0x04, 0x03, 0x02, 0x01, 0x08, 0x07, 0x06, 0x05,  // ages
           0x0C, 0x0B, 0x0A, 0x09,                          // fault_flags
           0xD3, 0xFF, 1,    0},                            // -4.5 C, 0.1 A
          4);
  section({0x10, 0x09, 1,                                    // sensor id, valid
           0,    0,    0,    0,    0,    0,    0xD0, 0x3F,   // position 0.25
           0,    0,    0,    0,    0,    0,    0xE0, 0xBF,   // velocity -0.5
           0x14, 0x13, 0x12, 0x11, 0x18, 0x17, 0x16, 0x15},  // ages
          5);
  section({0x0B, 0x0A, 1, 1}, 6);         // digital input id, valid, value
  section({0x0D, 0x0C, 1, 0}, 3);         // digital output id, valid, value
  section({0x0F, 0x0E, 1,                 // analog id, valid
           0, 0, 0, 0, 0, 0, 0x04, 0x40,  // voltage 2.5
           0x24, 0x23, 0x22, 0x21},       // age
          1);
  section({0x11, 0x10, 1,                            // encoder id, valid
           0,    0,    0,    0,   0, 0, 0,    0xC0,  // position -2.0
           0,    0,    0,    0,   0, 0, 0x10, 0x40,  // velocity 4.0
           0x28, 0x27, 0x26, 0x25},                  // age
          0);
  section({0x13, 0x12, 1,                  // PWM id, valid
           0, 0, 0, 0, 0, 0, 0xD0, 0xBF},  // output -0.25
          2);
  expected.insert(
      expected.end(),
      {1,                                               // health valid
       0,    0,    0,    0,    0,    0,    0x29, 0x40,  // battery_v 12.5
       0,    2,    1,    1,                             // brownout..read_state
       0,    0,    0x80, 0x3E,                          // utilization 0.25f
       0x14, 0x13, 0x12, 0x11, 0x24, 0x23, 0x22, 0x21,  // bus-off, buffer-full
       5,    6,    0x34, 0x33, 0x32, 0x31,              // rx, tx, failures
       0x44, 0x43, 0x42, 0x41, 0x54, 0x53, 0x52, 0x51,  // counters
       0x64, 0x63, 0x62, 0x61, 0x74, 0x73, 0x72, 0x71});
  std::array<uint8_t, kMaxStateSize> bytes{};
  const auto size = Encode(state, bytes);
  ASSERT_EQ(size, expected.size());
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), bytes.begin()));
  State decoded;
  ASSERT_TRUE(Decode(expected, decoded));
  EXPECT_EQ(decoded, state);
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

// The RoboRIO sets EnableFOC from foc, so both ends must agree on it: it
// travels on the config wire as one byte, 0 or 1, and changes the config id.
// A follower copies its master's output, so it cannot set foc.
// REQ-CONTRACT-39
TEST(Config, FocTravelsOnTheWireAndFingerprintAndAFollowerCannotSetIt) {
  auto c = SwerveConfig(true);
  const auto default_id = ConfigurationId(c);
  c.motors[0].foc = true;
  EXPECT_NE(ConfigurationId(c), default_id);
  Config decoded;
  ASSERT_TRUE(DeserializeConfig(SerializeConfig(c), decoded));
  EXPECT_EQ(decoded, c);

  // The one byte that differs from foc = false is foc's; 2 is refused.
  auto bytes = SerializeConfig(c);
  auto off = c;
  off.motors[0].foc = false;
  const auto off_bytes = SerializeConfig(off);
  ASSERT_EQ(bytes.size(), off_bytes.size());
  const auto at = std::mismatch(bytes.begin(), bytes.end(), off_bytes.begin());
  ASSERT_NE(at.first, bytes.end());
  EXPECT_EQ(*at.first, 1);
  *at.first = 2;
  EXPECT_FALSE(DeserializeConfig(bytes, decoded));

  auto follower = c;
  follower.motors[1].master_id = follower.motors[0].id;
  EXPECT_NO_THROW(Validate(follower));
  follower.motors[1].foc = true;
  EXPECT_THROW(Validate(follower), std::invalid_argument);
}

// The gateway checks a torque-current demand against max_torque_current_a
// and the RoboRIO sets its peak torque current from it, so both ends must
// agree on it: it travels on the config wire and changes the config id.
// REQ-SAFE-43
TEST(Config, TorqueCurrentLimitTravelsOnTheWireAndFingerprint) {
  auto c = SwerveConfig(true);
  c.motors[0].foc = true;
  const auto no_limit_id = ConfigurationId(c);
  c.motors[0].max_torque_current_a = 12.5;
  EXPECT_NE(ConfigurationId(c), no_limit_id);
  Config decoded;
  ASSERT_TRUE(DeserializeConfig(SerializeConfig(c), decoded));
  EXPECT_EQ(decoded, c);
}

// A RoboRIO's saved configuration written before foc existed (each motor
// record one byte shorter) is refused at startup rather than misread, so the
// RoboRIO boots unconfigured, as with a corrupt file, until the bridge
// pushes its configuration.
// REQ-CONTRACT-39
TEST(ConfigWire, APersistedConfigWithoutTheFocByteIsRefused) {
  const auto c = SwerveConfig(true);
  ASSERT_GT(c.motors.size(), 1u);
  auto bytes = SerializeConfig(c);
  // Each motor's foc byte is the one that differs when only its foc does.
  std::vector<std::ptrdiff_t> foc_offsets;
  for (std::size_t i = 0; i < c.motors.size(); ++i) {
    auto on = c;
    on.motors[i].foc = true;
    const auto on_bytes = SerializeConfig(on);
    ASSERT_EQ(on_bytes.size(), bytes.size());
    const auto at = std::mismatch(bytes.begin(), bytes.end(), on_bytes.begin());
    ASSERT_NE(at.first, bytes.end());
    foc_offsets.push_back(at.first - bytes.begin());
  }
  for (auto it = foc_offsets.rbegin(); it != foc_offsets.rend(); ++it)
    bytes.erase(bytes.begin() + *it);
  const uint32_t crc = ComputeCrc32(bytes);
  for (int i = 0; i < 4; ++i)
    bytes.push_back(static_cast<uint8_t>(crc >> (8 * i)));
  Config loaded;
  EXPECT_FALSE(LoadPersistedConfig(bytes, loaded));
  ASSERT_TRUE(LoadPersistedConfig(PersistedConfig(c), loaded));
  EXPECT_EQ(loaded, c);
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
// One motor on each of `buses`, for the CAN bus tests.
Config BusConfig(const std::vector<std::string>& buses) {
  Config c;
  uint16_t id = 1;
  for (const auto& bus : buses) {
    MotorConfig m;
    m.id = id;
    m.name = "motor" + std::to_string(id);
    m.can_id = id;
    m.bus = bus;
    c.motors.push_back(m);
    ++id;
  }
  return c;
}

// /hw/status's slot i is the same bus on the RoboRIO and the bridge: "rio"
// first whether or not a device names it, then every other bus once, in byte
// order. More than three besides "rio" do not fit and are refused, naming
// each bus.
// REQ-OPS-20
TEST(Config, CanBusesListRioThenTheOthersInByteOrderAndRefuseAFifth) {
  const auto names = [](const Config& c) {
    const CanBusList list = CanBuses(c);
    return std::vector<std::string>{
        list.names.begin(),
        list.names.begin() + static_cast<std::ptrdiff_t>(list.count)};
  };
  EXPECT_EQ(names(Config{}), std::vector<std::string>{"rio"});
  auto c = BusConfig({"b", "rio", "C", "b"});
  SensorConfig sensor;
  sensor.id = 99;
  sensor.name = "encoder";
  sensor.can_id = 1;
  sensor.bus = "a";
  c.sensors.push_back(sensor);
  EXPECT_NO_THROW(Validate(c));
  EXPECT_EQ(names(c), (std::vector<std::string>{"rio", "C", "a", "b"}));

  c.motors[1].bus = "Shooter";
  try {
    Validate(c);
    FAIL() << "four buses besides rio were accepted";
  } catch (const std::invalid_argument& e) {
    for (const char* bus : {"C", "Shooter", "a", "b"}) {
      EXPECT_NE(std::string{e.what()}.find(bus), std::string::npos)
          << bus << " is not named: " << e.what();
    }
  }
}

// Bus names are compared byte for byte (the wire carries them so), but
// "Shooter" and "shooter" may be one CANivore to Phoenix: two status slots
// and two load estimates for it would split its load. Config load refuses
// two names that differ only by case, naming both and a device on each;
// "rio" counts as named even when no device names it.
// REQ-OPS-20
TEST(Config, RefusesCanBusNamesThatDifferOnlyByCase) {
  auto c = BusConfig({"Shooter", "rio", "shooter"});
  try {
    Validate(c);
    FAIL() << "Shooter and shooter were accepted as two buses";
  } catch (const std::invalid_argument& e) {
    for (const char* text : {"Shooter", "shooter", "motor1", "motor3"}) {
      EXPECT_NE(std::string{e.what()}.find(text), std::string::npos)
          << text << " is not named: " << e.what();
    }
  }
  c = BusConfig({"RIO"});
  EXPECT_THROW(Validate(c), std::invalid_argument);
}

// The gateway reports one bus per State, in turn (index = sequence % count),
// so each bus's slot comes every `count` States, and the slot is what the
// backend read for that bus: a simulated backend reads kSimulated, one that
// reads no bus leaves it kNone rather than an idle bus's zeros.
// REQ-OPS-20
TEST(Gateway, ReportsEachCanBusInTurn) {
  const Config config = BusConfig({"Shooter", "rio", "Arm"});
  SimBackend simulated;
  FakeBackend unread;
  const std::pair<Backend*, CanBusReadState> cases[] = {
      {&simulated, CanBusReadState::kSimulated},
      {&unread, CanBusReadState::kNone}};
  for (const auto& [backend, expected] : cases) {
    Gateway g{config, *backend, 1};
    std::vector<int> indices;
    for (uint64_t tick = 1; tick <= 6; ++tick) {
      g.Tick(tick * 5000, false);
      const State s = g.snapshot();
      SCOPED_TRACE(tick);
      EXPECT_EQ(s.health.can_bus_count, 3);
      indices.push_back(s.health.can_bus_index);
      EXPECT_EQ(s.health.can_bus.read_state, expected);
    }
    EXPECT_EQ(indices, (std::vector<int>{1, 2, 0, 1, 2, 0}));
  }
}

// The RoboRIO's backend configures asynchronously: PollConfigure reports
// kPending for some ticks, then kDone, and the gateway commits the new
// configuration inside that tick. Scenario (bench, a 60 s session): every
// State then carried can_bus_count 0, the count from before the commit
// written back over the committed one, so /hw/status showed no CAN bus at
// all. The committed configuration's buses must be in the commit tick's
// State and every later one, and a reconfiguration to fewer buses (or to no
// device) must never leave an index at or past the new count, which would
// not even encode.
// REQ-OPS-20
TEST(Gateway, AnAsynchronousConfigureReportsItsOwnCanBuses) {
  class AsyncBackend : public FakeBackend {
   public:
    void BeginConfigure(const Config&) override {
      status = ConfigureStatus::kPending;
    }
    ConfigureStatus PollConfigure() override { return status; }
    ConfigureStatus status{ConfigureStatus::kPending};
  } backend;
  Gateway g{backend, 1};
  uint64_t now = 0;
  const auto tick = [&] {
    now += 5000;
    g.Tick(now, false);
    const State s = g.snapshot();
    std::array<uint8_t, kMaxStateSize> bytes{};
    EXPECT_NE(Encode(s, bytes), 0u) << "count " << int{s.health.can_bus_count}
                                    << " index " << int{s.health.can_bus_index};
    return s;
  };
  const auto configure = [&](const Config& config, int expected) {
    SCOPED_TRACE(expected);
    ASSERT_TRUE(g.BeginReconfigure(config));
    tick();  // pending
    backend.status = Backend::ConfigureStatus::kDone;
    std::vector<int> indices;
    for (int i = 0; i < 4; ++i) {
      const State s = tick();  // the first is the commit tick
      EXPECT_EQ(int{s.health.can_bus_count}, expected) << "tick " << i;
      indices.push_back(s.health.can_bus_index);
    }
    for (const int index : indices) EXPECT_LT(index, expected);
    if (expected > 1) EXPECT_NE(indices[0], indices[1]) << "no rotation";
  };
  configure(BusConfig({"rio", "Shooter"}), 2);
  configure(BusConfig({"rio"}), 1);
  configure(Config{}, 1);  // "rio" has a slot even with no device
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

// A command timeout above half a second would let a stalled node or a lost
// link keep a motor driving that long: 500000 us loads, one more is refused
// naming the key.
// REQ-SAFE-26
TEST(Config, CommandTimeoutAboveHalfASecondIsRefused) {
  auto c = SwerveConfig(true);
  c.command_timeout_us = 500000;
  EXPECT_NO_THROW(Validate(c));
  c.command_timeout_us = 500001;
  try {
    Validate(c);
    ADD_FAILURE() << "command_timeout_us 500001 loaded";
  } catch (const std::invalid_argument& error) {
    const std::string message = error.what();
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
// `c` with every motor neutral at zero demand and feedforward.
Command AllMotorsNeutral(Command c) {
  for (std::size_t i = 0; i < c.count; ++i) c.motors[i] = {c.motors[i].id};
  return c;
}
// The bridge keeps forwarding the neutral merge it built from the last State
// it saw. The tick that first sees an enable or a disable starts a new epoch
// before it accepts the commands drained with it, so a neutral already sent
// is refused there for its epoch. That is the epoch rule working, not a
// rejection: counting it would raise the counter at about one clean enable
// in ten, where operators expect it flat.
// REQ-SAFE-17
TEST(Gateway, NeutralInFlightAcrossAnEpochChangeIsNotARejection) {
  for (const bool enabling : {true, false}) {
    SCOPED_TRACE(enabling ? "enable edge" : "disable edge");
    FakeBackend backend;
    Gateway g{SwerveConfig(true), backend, 1};
    g.Tick(1000, !enabling);
    if (!enabling) {
      ASSERT_TRUE(g.Accept(Request(g), 1, 1000));
    }
    const auto in_flight = AllMotorsNeutral(Request(g));
    g.Tick(2000, enabling);
    ASSERT_GT(g.snapshot().epoch, in_flight.epoch);
    const auto applied = backend.applied;
    EXPECT_FALSE(g.Accept(in_flight, 2, 2000));
    EXPECT_EQ(backend.applied, applied);
    EXPECT_EQ(g.snapshot().counters.rejected_command_count, 0u);
  }
}
// Only a neutral that can have been in flight across the epoch change is
// exempt. An older-epoch command that drives any output (the counter's
// safety case; a PWM pulse of 0 is the 1.5 ms centre pulse, so it drives),
// one arriving while a command of the new epoch is in force,
// one of a newer or never-issued epoch, and one that fails another check too
// are each refused and counted.
// REQ-SAFE-17
TEST(Gateway, RefusedCommandsAroundAnEpochChangeStillCount) {
  auto config = SwerveConfig(true);
  config.digital_outputs.push_back({40, "solenoid", 1});
  config.pwm_outputs.push_back({41, "servo", 0});
  // Neutral, its outputs off, observing `g`'s State.
  const auto neutral = [](const Gateway& g) {
    auto c = AllMotorsNeutral(Request(g));
    c.digital_output_count = 1;
    c.digital_outputs[0] = {40, false};
    c.pwm_output_count = 1;
    c.pwm_outputs[0] = {41, PwmMode::kOff, 0.0};
    return c;
  };
  enum class Epoch { kOlder, kNewer, kNeverIssued };
  struct Row {
    const char* name;
    Epoch epoch;
    bool command_in_force;
    bool expired;
    void (*change)(Command&);
  };
  const auto none = [](Command&) {};
  const Row rows[] = {
      {"older epoch, a motor driven", Epoch::kOlder, false, false,
       [](Command& c) {
         c.motors[0] = {c.motors[0].id, Mode::kVelocity, 0, 1};
       }},
      {"older epoch, digital output on", Epoch::kOlder, false, false,
       [](Command& c) { c.digital_outputs[0].value = true; }},
      {"older epoch, PWM pulsing at its centre", Epoch::kOlder, false, false,
       [](Command& c) { c.pwm_outputs[0].mode = PwmMode::kPulse; }},
      {"older epoch, a command in force", Epoch::kOlder, true, false, none},
      {"newer epoch", Epoch::kNewer, false, false, none},
      {"epoch never issued", Epoch::kNeverIssued, false, false, none},
      {"older epoch, another boot", Epoch::kOlder, false, false,
       [](Command& c) { ++c.boot_id; }},
      {"older epoch, expired observation", Epoch::kOlder, false, true, none},
  };
  for (const auto& row : rows) {
    SCOPED_TRACE(row.name);
    FakeBackend backend;
    Gateway g{config, backend, 1};
    g.Tick(1000, false);
    auto command = neutral(g);
    row.change(command);
    g.Tick(2000, true);
    if (row.epoch == Epoch::kNewer) command.epoch = g.snapshot().epoch + 1;
    if (row.epoch == Epoch::kNeverIssued) command.epoch = 0;
    if (row.command_in_force) {
      ASSERT_TRUE(g.Accept(neutral(g), 1, 2000));
    }
    const uint64_t now = row.expired ? 1000 + config.command_timeout_us : 2000;
    const auto applied = backend.applied;
    EXPECT_FALSE(g.Accept(command, 2, now));
    EXPECT_EQ(backend.applied, applied);
    EXPECT_EQ(g.snapshot().counters.rejected_command_count, 1u);
  }
}
// The gateway hands each PWM request to the backend as the command carries
// it: off as off (no pulse), a pulse of 0 as a pulse (the centre). A pulse
// outside [-1, 1] or not finite, or a mode the wire does not define, rejects
// the whole command and applies nothing.
// REQ-SAFE-03, REQ-SAFE-17
TEST(Gateway, PwmRequestsReachTheBackendAsSentAndABadOneRejectsTheCommand) {
  auto config = SwerveConfig(true);
  config.pwm_outputs.push_back({41, "servo", 0});
  const struct {
    const char* label;
    PwmRequest request;
    bool accepted;
  } rows[] = {
      {"off", {41, PwmMode::kOff, 0.0}, true},
      {"a pulse of 0", {41, PwmMode::kPulse, 0.0}, true},
      {"a pulse of -1", {41, PwmMode::kPulse, -1.0}, true},
      {"a pulse above 1", {41, PwmMode::kPulse, 1.5}, false},
      {"a pulse not finite", {41, PwmMode::kPulse, std::nan("")}, false},
      {"mode 2", {41, static_cast<PwmMode>(2), 0.0}, false},
  };
  for (const auto& row : rows) {
    SCOPED_TRACE(row.label);
    FakeBackend backend;
    Gateway g{config, backend, 1};
    g.Tick(1000, true);
    auto command = Request(g);
    command.pwm_output_count = 1;
    command.pwm_outputs[0] = row.request;
    const int applied = backend.applied;
    EXPECT_EQ(g.Accept(command, 1, 1000), row.accepted);
    EXPECT_EQ(backend.applied - applied, row.accepted ? 1 : 0);
    EXPECT_EQ(g.snapshot().counters.rejected_command_count,
              row.accepted ? 0u : 1u);
    if (row.accepted) {
      ASSERT_EQ(backend.last_pwm_outputs.size(), 1u);
      EXPECT_EQ(backend.last_pwm_outputs[0], row.request);
    } else {
      EXPECT_TRUE(backend.last_pwm_outputs.empty());
    }
  }
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
      if (failure == 3) {
        // A sanitized master takes its follower with it; both count.
        EXPECT_EQ(gateway.snapshot().counters.neutralized_motor_count, 2u);
      }
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

// During total loss every node flushes neutral (its State carries
// kHardwareFault, so robot_enabled is false) and the bridge keeps forwarding
// that merge, stamped with the loss's own epoch. Refusing it is the gate
// working, as while disabled, so it is not counted; a command of the same
// epoch that drives an output is still refused and counted.
// REQ-SAFE-17
TEST(Gateway, NeutralRefusedDuringTotalLossIsNotARejection) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  backend.healthy = false;
  g.Tick(6000, true);
  ASSERT_TRUE(g.snapshot().flags & kHardwareFault);
  const auto neutral = AllMotorsNeutral(Request(g));
  ASSERT_EQ(neutral.epoch, g.snapshot().epoch);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    EXPECT_FALSE(g.Accept(neutral, sequence, 6000));
  }
  EXPECT_EQ(g.snapshot().counters.rejected_command_count, 0u);
  EXPECT_FALSE(g.Accept(Request(g), 4, 6000));
  EXPECT_EQ(g.snapshot().counters.rejected_command_count, 1u);
  EXPECT_EQ(backend.applied, 0);
}

// Total loss is not latched and clears in its own epoch, so a command built
// on a State of the loss still matches once the feedback returns. It drives
// nothing: its motors go neutral (motor_drives_from_) and so do its digital
// and PWM outputs, sent false and off; a command observing the recovered
// State applies them as sent.
// REQ-SAFE-18, REQ-SAFE-22
TEST(Gateway, ACommandObservingTotalLossDrivesNoOutputAfterIt) {
  auto config = SwerveConfig(true);
  config.digital_outputs.push_back({40, "solenoid", 1});
  config.pwm_outputs.push_back({41, "servo", 0});
  const auto with_outputs = [](Command c) {
    c.digital_output_count = 1;
    c.digital_outputs[0] = {40, true};
    c.pwm_output_count = 1;
    c.pwm_outputs[0] = {41, PwmMode::kPulse, 0.5};
    return c;
  };
  FakeBackend backend;
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  backend.healthy = false;
  g.Tick(6000, true);
  ASSERT_TRUE(g.snapshot().flags & kHardwareFault);
  const auto during = with_outputs(Request(g));
  backend.healthy = true;
  g.Tick(11000, true);
  ASSERT_FALSE(g.snapshot().flags & kHardwareFault);
  ASSERT_EQ(g.snapshot().epoch, during.epoch);
  ASSERT_TRUE(g.Accept(during, 1, 11000));
  for (const auto& request : backend.last_requests) {
    EXPECT_EQ(request.mode, Mode::kNeutral);
  }
  ASSERT_EQ(backend.last_digital_outputs.size(), 1u);
  EXPECT_FALSE(backend.last_digital_outputs[0].value);
  ASSERT_EQ(backend.last_pwm_outputs.size(), 1u);
  EXPECT_EQ(backend.last_pwm_outputs[0].mode, PwmMode::kOff);
  EXPECT_EQ(backend.last_pwm_outputs[0].output, 0.0);

  ASSERT_TRUE(g.Accept(with_outputs(Request(g)), 2, 11000));
  EXPECT_TRUE(backend.last_digital_outputs[0].value);
  EXPECT_EQ(backend.last_pwm_outputs[0],
            (PwmRequest{41, PwmMode::kPulse, 0.5}));
}

// A failed read pass is total loss whatever the samples show, and the Backend
// contract lets Read return false with a sample still valid. A command
// observing such a State must hold that motor once the loss clears, as it
// holds the digital and PWM outputs: the hold may not depend on the backend
// also marking every sample invalid.
// REQ-SAFE-22
TEST(Gateway, ACommandObservingAFailedReadHoldsAValidMotor) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  backend.read_fails = true;
  g.Tick(6000, true);
  ASSERT_TRUE(g.snapshot().flags & kHardwareFault);
  ASSERT_TRUE(g.snapshot().motors[0].valid);
  const auto during = Request(g);
  backend.read_fails = false;
  g.Tick(11000, true);
  ASSERT_FALSE(g.snapshot().flags & kHardwareFault);
  ASSERT_EQ(g.snapshot().epoch, during.epoch);
  ASSERT_TRUE(g.Accept(during, 1, 11000));
  for (const auto& request : backend.last_requests) {
    EXPECT_EQ(request.mode, Mode::kNeutral) << "motor " << request.id;
  }

  ASSERT_TRUE(g.Accept(Request(g), 2, 11000));
  for (const auto& request : backend.last_requests) {
    EXPECT_EQ(request.mode, Mode::kVelocity) << "motor " << request.id;
    EXPECT_EQ(request.demand, 1) << "motor " << request.id;
  }
}

// Total loss is judged on the State as published, the gateway's masks
// included: a follower of an invalid master, or the motor a reset CANcoder
// feeds, counts as lost though the backend reads it valid. With such a pair as
// the only CAN devices, an invalid master, or a reset CANcoder, leaves no
// valid sample in the State: the hardware fault, a new epoch and the command
// in force ended. SimHardwareTotalLoss.EverySampleMaskedIsTotalLoss holds
// SimHardware to the same two scenarios.
// REQ-SAFE-22, REQ-DEV-07
TEST(Gateway, EverySampleMaskedIsTotalLoss) {
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
    EXPECT_TRUE(s.flags & kHardwareFault);
    EXPECT_FALSE(s.flags & kCommandActive);
    EXPECT_EQ(s.epoch, epoch + 1);
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
  c.pwm_outputs[0] = {7, PwmMode::kPulse, 0.85};
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
  state.health = {true, 9.2, true,
                  4,    3,   {CanBusReadState::kStale, 0.73F, 1, 2, 3, 4, 5}};
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
      70 + 14 + 47 + kMaxMotors * 45 + kMaxSensors * 27 +
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

// A robot brought up before its first mechanism has no device at all, and
// one with only a solenoid has no motor: both cross the wire like any other.
TEST(ConfigWire, ChunksAndAssemblesConfigLosslessly) {
  auto swerve = SwerveConfig(false);
  swerve.motors[1].master_id = swerve.motors[0].id;
  swerve.motors[1].oppose_master = true;
  swerve.motors[1].slots[2].gravity_type = GravityType::kArmCosine;
  for (auto& sensor : swerve.sensors) {
    if (sensor.kind != SensorKind::kPigeon2) continue;
    sensor.offset_rot = 0.125;
    sensor.inverted = true;
  }
  Config solenoid_only;
  solenoid_only.digital_outputs.push_back({40, "solenoid", 1});
  const std::pair<const char*, Config> rows[] = {
      {"swerve", swerve},
      {"no device", Config{}},
      {"a digital output and no motor", solenoid_only},
  };
  for (const auto& [label, cfg] : rows) {
    SCOPED_TRACE(label);
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
    EXPECT_EQ(status, ConfigAssembler::Status::kComplete) << err;
    EXPECT_EQ(assembler.config(), cfg);
    EXPECT_EQ(assembler.config_id(), ConfigurationId(cfg));
  }
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
// REQ-CONTRACT-03
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

// The serialized config body both ends agree on byte for byte, one device
// of every kind: each integer (a signed one as its two's-complement bits)
// and IEEE 754 double little-endian, each string a u16 length then its
// bytes, each device list a u16 count.
// REQ-CONTRACT-03
TEST(ConfigWire, SerializedConfigIsLittleEndian) {
  Config config;
  config.period_us = 5000;
  config.command_timeout_us = 100000;
  config.status_hz = 200;
  config.commissioned = true;
  MotorConfig motor;
  motor.id = 0x0102;
  motor.name = "m";
  motor.can_id = 0x01020304;
  motor.feedback = Feedback::kRemoteCANcoder;
  motor.feedback_sensor_id = 0x0506;
  motor.reverse_limit_switch = LimitSwitch::kDisabled;
  motor.slots[0].p = 1.5;
  motor.slots[0].gravity_type = GravityType::kArmCosine;
  motor.foc = true;
  motor.max_torque_current_a = 0.5;
  config.motors.push_back(motor);
  config.sensors.push_back(
      {0x0708, "s", SensorKind::kPigeon2, 0x11121314, "cv", true, -0.25});
  config.digital_inputs.push_back({0x0901, "a", 0x21222324});
  config.digital_outputs.push_back({0x0902, "b", 0x31323334});
  config.analog_inputs.push_back({0x0903, "c", 0x41424344});
  config.encoders.push_back(
      {0x0904, "e", EncoderKind::kDutyCycle, 0x51525354, -1, 2048, 0.5});
  config.pwm_outputs.push_back({0x0905, "p", 0x61626364});
  std::vector<uint8_t> expected{
      0x88, 0x13, 0,    0,    0xA0, 0x86, 0x01, 0,     // period, timeout
      0,    0,    0,    0,    0,    0,    0x69, 0x40,  // status_hz 200
      1,    1,    0,                                   // commissioned, motors
      0x02, 0x01, 1,    0,    'm',                     // id, name
      0x04, 0x03, 0x02, 0x01, 3,    0,    'r',  'i',  'o',  // can_id, bus
      0,    1,    0,    0,    0,                            // inverted..oppose
      0,    0,    0,    0,    0,    0,    0x44, 0x40,       // supply 40 A
      0,    0,    0,    0,    0,    0,    0x54, 0x40,       // stator 80 A
      0,    0,    0,    0,    0,    0,    0x28, 0x40,       // max_voltage 12 V
      1,    0x06, 0x05,                                 // feedback, its sensor
      0,    0,    0,    0,    0,    0,    0xF0, 0x3F,   // rotor_to_sensor 1
      0,    0,    0,    0,    0,    0,    0xF0, 0x3F,   // sensor_to_mechanism 1
      0,    0,                                          // wrap, soft limits
      0,    0,    0,    0,    0,    0,    0xF0, 0xBF,   // reverse limit -1
      0,    0,    0,    0,    0,    0,    0xF0, 0x3F,   // forward limit 1
      0,    2,                                          // limit switches
      0,    0,    0,    0,    0,    0,    0x59, 0x40,   // max velocity 100
      0,    0,    0,    0,    0,    0,    0x34, 0x40,   // cruise 20
      0,    0,    0,    0,    0,    0,    0x44, 0x40,   // acceleration 40
      0,    0,    0,    0,    0,    0,    0,    0,      // jerk 0
      0,    0,    0,    0,    0,    0,    0xF8, 0x3F};  // slot 0 p 1.5
  // Slot 0's i, d, s, v, a and g, then its gravity type; slots 1 and 2.
  expected.insert(expected.end(), size_t{6} * 8, uint8_t{0});
  expected.push_back(1);
  expected.insert(expected.end(), size_t{2} * (7 * 8 + 1), uint8_t{0});
  expected.insert(
      expected.end(),
      {1,                                                  // foc
       0,    0,    0,    0,    0,    0,    0xE0, 0x3F,     // torque current 0.5
       1,    0,    0x08, 0x07, 1,    0,    's',  2,        // sensor, Pigeon 2
       0x14, 0x13, 0x12, 0x11, 2,    0,    'c',  'v',  1,  // can_id, bus, inv
       0,    0,    0,    0,    0,    0,    0xD0, 0xBF,     // offset -0.25
       1,    0,    0x01, 0x09, 1,    0,    'a',            // digital input
       0x24, 0x23, 0x22, 0x21,                             //
       1,    0,    0x02, 0x09, 1,    0,    'b',            // digital output
       0x34, 0x33, 0x32, 0x31,                             //
       1,    0,    0x03, 0x09, 1,    0,    'c',            // analog input
       0x44, 0x43, 0x42, 0x41,                             //
       1,    0,    0x04, 0x09, 1,    0,    'e',  2,     // encoder, duty cycle
       0x54, 0x53, 0x52, 0x51, 0xFF, 0xFF, 0xFF, 0xFF,  // dio_a, dio_b -1
       0,    0,    0,    0,    0,    0,    0xA0, 0x40,  // counts_per_rev 2048
       0,    0,    0,    0,    0,    0,    0xE0, 0x3F,  // offset 0.5
       1,    0,    0x05, 0x09, 1,    0,    'p',         // PWM output
       0x64, 0x63, 0x62, 0x61});
  EXPECT_EQ(SerializeConfig(config), expected);
  Config decoded;
  ASSERT_TRUE(DeserializeConfig(expected, decoded));
  EXPECT_EQ(decoded, config);
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

// A robot is brought up (link, Driver Station, telemetry) before its first
// mechanism: a config with no device, or with a digital output and no motor,
// configures on either simulated backend, publishes States, enables, and
// raises no fault, since with no motor or CAN sensor there is no feedback to
// lose (AnyFeedbackValid). A command for the digital output applies it.
// REQ-CONTRACT-35
TEST(Gateway, ARobotWithNoMotorConfiguresAndEnables) {
  Config solenoid_only;
  solenoid_only.commissioned = true;
  solenoid_only.digital_outputs.push_back({40, "solenoid", 1});
  Config empty;
  empty.commissioned = true;
  for (const auto& [label, config] :
       {std::pair<const char*, Config>{"no device", empty},
        std::pair<const char*, Config>{"a digital output", solenoid_only}}) {
    for (const bool physics : {false, true}) {
      SCOPED_TRACE(std::string{label} + (physics ? ", physics" : ", ideal"));
      SimBackend ideal;
      sim::PhysicsSimBackend physical;
      Backend& backend = physics ? static_cast<Backend&>(physical) : ideal;
      Gateway g{backend, 1};
      g.Tick(1000, false);
      ASSERT_TRUE(g.Reconfigure(config));
      g.Tick(2000, true);
      const auto s = g.snapshot();
      EXPECT_EQ(s.config_id, ConfigurationId(config));
      EXPECT_EQ(s.motor_count, 0u);
      EXPECT_EQ(s.flags & (kConfigured | kEnabled | kHardwareFault),
                kConfigured | kEnabled);
      Command c = Request(g);
      c.digital_output_count = config.digital_outputs.size();
      if (c.digital_output_count > 0) c.digital_outputs[0] = {40, true};
      EXPECT_TRUE(g.Accept(c, 1, 2000));
      g.Tick(3000, true);
      const auto after = g.snapshot();
      EXPECT_EQ(after.flags & (kCommandActive | kHardwareFault),
                kCommandActive);
      if (config.digital_outputs.empty()) continue;
      ASSERT_TRUE(after.digital_outputs[0].valid);
      EXPECT_TRUE(after.digital_outputs[0].value);
    }
  }
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
    cmd.pwm_outputs[0] = {7, PwmMode::kPulse, 0.85};
    return cmd;
  };
  auto expect_driven = [](FailableSim& backend, const State& s) {
    EXPECT_TRUE(backend.sim.GetDigitalOutput(4));
    EXPECT_EQ(backend.sim.GetPwm(7), (PwmRequest{7, PwmMode::kPulse, 0.85}));
    ASSERT_GT(s.motor_count, 0u);
    EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 10.0);
    EXPECT_EQ(s.digital_outputs[0].value, true);
    EXPECT_DOUBLE_EQ(s.pwm_outputs[0].output, 0.85);
  };
  auto expect_safe_backends = [](FailableSim& backend) {
    EXPECT_FALSE(backend.sim.GetDigitalOutput(4));
    // No pulse: off, not a pulse of 0 (which centres a servo).
    EXPECT_EQ(backend.sim.GetPwm(7), (PwmRequest{.id = 7}));
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
// chain receives only neutral. This backend reads the chain valid throughout,
// as the Backend contract allows, so the gateway alone holds it: the State of
// the tick that completes the restore was read before the restore completed,
// so the chain is invalid in it and a command observing it still drives the
// chain nothing; the first command observing a State read after the restore
// drives it again. The other motors drive throughout.
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

  const auto chain_valid = [&config](const State& state) {
    for (std::size_t i = 0; i < config.motors.size(); ++i) {
      if ((config.motors[i].id == kResetId ||
           config.motors[i].master_id == kResetId) &&
          state.motors[i].valid) {
        return true;
      }
    }
    return false;
  };
  EXPECT_FALSE(chain_valid(g.snapshot())) << "read before the restore ended";
  ASSERT_TRUE(g.Accept(Request(g), ++sequence, now));
  expect_apply(backend.last_requests, true, "observed as the restore ended");
  now += 1000;
  g.Tick(now, true);
  EXPECT_TRUE(chain_valid(g.snapshot()));
  ASSERT_TRUE(g.Accept(Request(g), ++sequence, now));
  expect_apply(backend.last_requests, false, "observed after the restore");
  EXPECT_EQ(g.snapshot().epoch, epoch);
}

// PhoenixBackend leaves a restoring CANcoder's own sample valid (RestoredWith,
// feedback.h), so in the tick whose PollRestore ends its restore the State
// holds a value read while it was reconfigured. The gateway masks it there,
// as it masks the motor it feeds, so a node reading the CANcoder directly
// (absolute homing) never takes that value. The motor drives again only on a
// command observing the next State; the other motors drive throughout.
// REQ-SAFE-24
TEST(Gateway, RestoredCANcoderIsInvalidInTheTickItsRestoreEnds) {
  const auto config = SwerveConfig(true);
  const auto steer = MotorIndex(config, Feedback::kRemoteCANcoder);
  const auto drive = MotorIndex(config, Feedback::kRotor);
  const uint16_t encoder_id = config.motors[steer].feedback_sensor_id;
  const auto encoder = static_cast<std::size_t>(
      std::find_if(config.sensors.begin(), config.sensors.end(),
                   [encoder_id](const auto& s) { return s.id == encoder_id; }) -
      config.sensors.begin());
  FakeBackend backend;  // Reads the CANcoder valid throughout, as Phoenix does.
  Gateway g{config, backend, 1};
  g.Tick(1000, true);
  ASSERT_TRUE(g.Accept(Request(g), 1, 1000));
  backend.resets = {encoder_id};
  g.Tick(2000, true);  // Reset found: masked, the restore begun.
  ASSERT_EQ(backend.restoring_ids, std::vector<uint16_t>{encoder_id});
  EXPECT_FALSE(g.snapshot().sensors[encoder].valid);
  backend.calls.clear();
  g.Tick(3000, true);  // The restore ends, in a State read before it did.
  ASSERT_TRUE(std::any_of(
      backend.calls.begin(), backend.calls.end(), [](const auto& call) {
        return call.kind == FakeBackend::Call::kRestoreFinished;
      }));
  auto s = g.snapshot();
  EXPECT_FALSE(s.sensors[encoder].valid);
  EXPECT_FALSE(s.motors[steer].valid);
  ASSERT_TRUE(g.Accept(Request(g), 2, 3000));
  EXPECT_EQ(backend.last_requests[steer].mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests[drive].mode, Mode::kVelocity);

  g.Tick(4000, true);
  s = g.snapshot();
  EXPECT_TRUE(s.sensors[encoder].valid);
  EXPECT_TRUE(s.motors[steer].valid);
  ASSERT_TRUE(g.Accept(Request(g), 3, 4000));
  EXPECT_EQ(backend.last_requests[steer].mode, Mode::kVelocity);
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

// The Endpoint publishes the State taken after Accept, so Accept judges total
// loss on it too. Two motors and a solenoid: the first motor's reset masks
// it, then a command's send to the second fails, which isolates and masks
// that one as well. The State Accept leaves has no valid sample: the
// hardware fault, a new epoch, the command ended and every output neutral,
// the solenoid included.
// REQ-SAFE-22, REQ-SAFE-23
TEST(Gateway, ASendFailureThatMasksTheLastValidSampleIsTotalLoss) {
  Config config;
  config.commissioned = true;
  MotorConfig motor;
  motor.name = "left";
  motor.id = 1;
  motor.can_id = 1;
  config.motors.push_back(motor);
  motor.name = "right";
  motor.id = 2;
  motor.can_id = 2;
  config.motors.push_back(motor);
  config.digital_outputs = {{21, "solenoid", 1}};
  FakeBackend backend;
  Gateway g{config, backend, 1};
  const auto drive = [&g] {
    auto command = Request(g);
    command.motors[0] = {1, Mode::kVoltage, 0, 2};
    command.motors[1] = {2, Mode::kVoltage, 0, 2};
    command.digital_output_count = 1;
    command.digital_outputs[0] = {21, true};
    return command;
  };
  g.Tick(1000, true);
  ASSERT_TRUE(g.Accept(drive(), 1, 1000));
  backend.resets = {1};
  g.Tick(2000, true);
  ASSERT_FALSE(g.snapshot().motors[0].valid);
  ASSERT_TRUE(g.snapshot().motors[1].valid);
  ASSERT_FALSE(g.snapshot().flags & kHardwareFault);
  const auto epoch = g.snapshot().epoch;
  const int neutrals = backend.neutral;
  backend.failing_motors = {2};
  EXPECT_FALSE(g.Accept(drive(), 2, 2000));
  const auto s = g.snapshot();
  EXPECT_FALSE(s.motors[1].valid);
  EXPECT_TRUE(s.flags & kHardwareFault);
  EXPECT_FALSE(s.flags & kCommandActive);
  EXPECT_EQ(s.epoch, epoch + 1);
  EXPECT_GT(backend.neutral, neutrals);
}

// One master (id 1) and its follower (id 2), and nothing else.
Config MasterFollowerConfig() {
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
  return config;
}
// Accepts a command, observing `g`'s State, that drives every motor at 2 V.
bool DriveAtTwoVolts(Gateway& g, uint64_t sequence, uint64_t now) {
  auto command = Request(g);
  for (std::size_t i = 0; i < command.count; ++i) {
    command.motors[i] = {command.motors[i].id, Mode::kVoltage, 0, 2};
  }
  return g.Accept(command, sequence, now);
}

// A config of one master and its follower, driving, whose master resets. The
// gateway masks the pair from the tick that finds the reset until the tick
// whose PollRestore ends the restore (whatever the backend reads for it
// meanwhile; PhoenixBackend::Read reads it invalid), so those States have no
// valid feedback: total loss, which faults, ends the command in force and
// starts one new epoch. The first State read after the restore is healthy
// again, and only a command of the new epoch drives the pair. A simulated
// backend must fault the same way.
template <typename SimulatedBackend>
void ExpectRestoringPairIsTotalLoss(SimulatedBackend& backend) {
  Gateway g{MasterFollowerConfig(), backend, 1};
  const auto drive = [&g](uint64_t sequence, uint64_t now) {
    return DriveAtTwoVolts(g, sequence, now);
  };
  g.Tick(1000, true);
  ASSERT_TRUE(drive(1, 1000));
  const auto epoch = g.snapshot().epoch;
  const auto before_loss = Request(g);
  backend.SimulateDeviceReset(1);
  // Reset found: the pair masked, the restore begun; then read while the
  // pair restores, and the restore ended.
  for (const uint64_t now : {uint64_t{2000}, uint64_t{3000}}) {
    SCOPED_TRACE(now);
    g.Tick(now, true);
    const auto s = g.snapshot();
    EXPECT_FALSE(s.motors[0].valid);
    EXPECT_FALSE(s.motors[1].valid);
    EXPECT_TRUE(s.flags & kHardwareFault);
    EXPECT_FALSE(s.flags & kCommandActive);
    EXPECT_EQ(s.epoch, epoch + 1);
    EXPECT_FALSE(g.Accept(before_loss, now / 1000, now));
  }

  g.Tick(4000, true);  // Restored.
  const auto s = g.snapshot();
  EXPECT_TRUE(s.motors[0].valid);
  EXPECT_TRUE(s.motors[1].valid);
  EXPECT_FALSE(s.flags & kHardwareFault);
  EXPECT_FALSE(g.Accept(before_loss, 4, 4000));
  ASSERT_TRUE(drive(5, 4000));
  g.Tick(5000, true);
  EXPECT_NE(g.snapshot().motors[0].voltage, 0.0);
  EXPECT_NE(g.snapshot().motors[1].voltage, 0.0);
}

// REQ-DEV-24
TEST(SimBackend, RestoringChainIsInvalidAsOnTheRobot) {
  SimBackend backend;
  ExpectRestoringPairIsTotalLoss(backend);
}

// REQ-DEV-24
TEST(PhysicsSimBackend, RestoringChainIsInvalidAsOnTheRobot) {
  sim::PhysicsSimBackend backend;
  ExpectRestoringPairIsTotalLoss(backend);
}

// The same pair, whose master resets and whose first two restores fail. On
// the robot a device whose restore failed stays unavailable, and
// PhoenixBackend::Read reports it invalid, until a later restore completes
// it (retried at the gateway's backoff, 250 ms then 500 ms), so every State
// until then is total loss; the pair drives again only once a restore
// succeeds, on a command observing a State read after it. The gateway's mask
// of the unavailable pair decides this, whatever the backend reads for it.
template <typename SimulatedBackend>
void ExpectFailingRestoreKeepsPairInvalid(SimulatedBackend& backend) {
  Gateway g{MasterFollowerConfig(), backend, 1};
  g.Tick(1000, true);
  ASSERT_TRUE(DriveAtTwoVolts(g, 1, 1000));
  backend.SetRestoreFails(true);
  backend.SimulateDeviceReset(1);
  uint64_t now = 2000;
  g.Tick(now, true);  // Reset found: the pair neutral, restore 1 begun.
  ASSERT_EQ(g.restore_attempt_count(), 1u);
  // Restores 1 and 2 fail at their polls; the tick that begins restore 3 ends
  // the loop.
  while (g.restore_attempt_count() < 3 && now < 2000000) {
    now += 5000;
    g.Tick(now, true);
    const auto s = g.snapshot();
    ASSERT_FALSE(s.motors[0].valid) << "tick " << now;
    ASSERT_FALSE(s.motors[1].valid) << "tick " << now;
    ASSERT_TRUE(s.flags & kHardwareFault) << "tick " << now;
  }
  ASSERT_EQ(g.restore_attempt_count(), 3u) << "tick " << now;
  backend.SetRestoreFails(false);
  now += 5000;
  g.Tick(now, true);  // Restored, in a State read before it was.
  auto s = g.snapshot();
  EXPECT_FALSE(s.motors[0].valid);
  EXPECT_TRUE(s.flags & kHardwareFault);
  now += 5000;
  g.Tick(now, true);
  s = g.snapshot();
  EXPECT_TRUE(s.motors[0].valid);
  EXPECT_TRUE(s.motors[1].valid);
  EXPECT_FALSE(s.flags & kHardwareFault);
  ASSERT_TRUE(DriveAtTwoVolts(g, 2, now));
  g.Tick(now + 5000, true);
  EXPECT_NE(g.snapshot().motors[0].voltage, 0.0);
  EXPECT_NE(g.snapshot().motors[1].voltage, 0.0);
}

// REQ-DEV-24
TEST(SimBackend, FailingRestoreKeepsChainInvalidAsOnTheRobot) {
  SimBackend backend;
  ExpectFailingRestoreKeepsPairInvalid(backend);
}

// REQ-DEV-24
TEST(PhysicsSimBackend, FailingRestoreKeepsChainInvalidAsOnTheRobot) {
  sim::PhysicsSimBackend backend;
  ExpectFailingRestoreKeepsPairInvalid(backend);
}

// One motor, a Pigeon 2, a solenoid and a PWM output.
Config MotorBesideAPigeonConfig() {
  Config config;
  config.commissioned = true;
  MotorConfig motor;
  motor.id = 1;
  motor.name = "roller";
  motor.can_id = 1;
  config.motors = {motor};
  config.sensors = {{11, "imu", SensorKind::kPigeon2, 30, "rio"}};
  config.digital_outputs = {{21, "solenoid", 1}};
  config.pwm_outputs = {{31, "servo", 0}};
  return config;
}
// A command for that robot, observing `g`'s last State, that drives the motor
// at 2 V, the solenoid on and the PWM output at a half pulse.
Command DriveBesideAPigeon(const Gateway& g) {
  auto command = Request(g);
  command.motors[0] = {1, Mode::kVoltage, 0, 2};
  command.digital_output_count = 1;
  command.digital_outputs[0] = {21, true};
  command.pwm_output_count = 1;
  command.pwm_outputs[0] = {31, PwmMode::kPulse, 0.5};
  return command;
}

// A CAN brownout resets every CAN device at once. The gateway masks the
// motor and the Pigeon in every State from the tick that finds the reset to
// the tick whose PollRestore ends the restore, though the backends read the
// restoring Pigeon valid (as PhoenixBackend does). Those States carry no
// valid motion sample, so each is total loss, judged on the State as
// published: the hardware fault, one new epoch, every output neutral, the
// solenoid and PWM output included, and no command accepted. The first State
// read after the restore clears the fault in that epoch; a command observing
// it drives everything, one built before the brownout nothing.
template <typename SimulatedBackend>
void ExpectEveryDeviceResetIsTotalLoss(SimulatedBackend& backend) {
  Gateway g{MotorBesideAPigeonConfig(), backend, 1};
  uint64_t sequence = 0;
  g.Tick(1000, true);
  ASSERT_TRUE(g.Accept(DriveBesideAPigeon(g), ++sequence, 1000));
  const auto epoch = g.snapshot().epoch;
  const auto before = DriveBesideAPigeon(g);
  backend.SimulateDeviceReset(1);
  backend.SimulateDeviceReset(11);
  // The reset found, then read while restoring and restored.
  for (const uint64_t now : {uint64_t{6000}, uint64_t{11000}}) {
    SCOPED_TRACE(now);
    g.Tick(now, true);
    const auto s = g.snapshot();
    EXPECT_FALSE(s.motors[0].valid);
    EXPECT_FALSE(s.sensors[0].valid);
    EXPECT_TRUE(s.flags & kHardwareFault);
    EXPECT_FALSE(s.flags & kCommandActive);
    EXPECT_EQ(s.epoch, epoch + 1);
    EXPECT_FALSE(backend.GetDigitalOutput(21));
    EXPECT_EQ(backend.GetPwm(31).mode, PwmMode::kOff);
    EXPECT_FALSE(g.Accept(DriveBesideAPigeon(g), ++sequence, now));
  }
  g.Tick(16000, true);
  const auto s = g.snapshot();
  EXPECT_TRUE(s.motors[0].valid);
  EXPECT_TRUE(s.sensors[0].valid);
  EXPECT_FALSE(s.flags & kHardwareFault);
  EXPECT_EQ(s.epoch, epoch + 1);
  EXPECT_FALSE(g.Accept(before, ++sequence, 16000));
  const auto neutralized = s.counters.neutralized_motor_count;
  ASSERT_TRUE(g.Accept(DriveBesideAPigeon(g), ++sequence, 16000));
  EXPECT_EQ(g.snapshot().counters.neutralized_motor_count, neutralized);
  EXPECT_TRUE(backend.GetDigitalOutput(21));
  EXPECT_EQ(backend.GetPwm(31).mode, PwmMode::kPulse);
  g.Tick(21000, true);
  EXPECT_NE(g.snapshot().motors[0].voltage, 0.0);
}

// REQ-SAFE-22, REQ-SAFE-24, REQ-DEV-24
TEST(SimBackend, EveryDeviceResetIsTotalLoss) {
  SimBackend backend;
  ExpectEveryDeviceResetIsTotalLoss(backend);
}

// REQ-SAFE-22, REQ-SAFE-24, REQ-DEV-24
TEST(PhysicsSimBackend, EveryDeviceResetIsTotalLoss) {
  sim::PhysicsSimBackend backend;
  ExpectEveryDeviceResetIsTotalLoss(backend);
}

// The same robot, whose motor alone resets: the Pigeon stays valid, so no
// State is total loss: no fault and no new epoch. Only the motor is masked,
// neutral in every command until one observes a State read after its
// restore, while the solenoid and PWM output, outside the reset chain, keep
// applying.
template <typename SimulatedBackend>
void ExpectAMotorResetBesideAValidSensorIsLocal(SimulatedBackend& backend) {
  Gateway g{MotorBesideAPigeonConfig(), backend, 1};
  // Accepts `command`; returns how many motors it neutralized.
  uint64_t sequence = 0;
  const auto accept = [&g, &sequence](const Command& command, uint64_t now) {
    const auto neutralized = g.snapshot().counters.neutralized_motor_count;
    EXPECT_TRUE(g.Accept(command, ++sequence, now));
    return g.snapshot().counters.neutralized_motor_count - neutralized;
  };
  g.Tick(1000, true);
  EXPECT_EQ(accept(DriveBesideAPigeon(g), 1000), 0u);
  const auto epoch = g.snapshot().epoch;
  backend.SimulateDeviceReset(1);
  // The reset found, then read while restoring and restored.
  Command masked{};
  for (const uint64_t now : {uint64_t{6000}, uint64_t{11000}}) {
    SCOPED_TRACE(now);
    g.Tick(now, true);
    const auto s = g.snapshot();
    EXPECT_FALSE(s.motors[0].valid);
    EXPECT_TRUE(s.sensors[0].valid);
    EXPECT_FALSE(s.flags & kHardwareFault);
    EXPECT_EQ(s.epoch, epoch);
    masked = DriveBesideAPigeon(g);
    EXPECT_EQ(accept(masked, now), 1u);  // The motor neutral.
    EXPECT_TRUE(backend.GetDigitalOutput(21));
    EXPECT_EQ(backend.GetPwm(31).mode, PwmMode::kPulse);
  }
  // Read after the restore ended. A command observing the last masked State
  // still drives no motor; one observing this State does.
  g.Tick(16000, true);
  EXPECT_TRUE(g.snapshot().motors[0].valid);
  EXPECT_EQ(accept(masked, 16000), 1u);
  g.Tick(21000, true);
  EXPECT_DOUBLE_EQ(g.snapshot().motors[0].voltage, 0.0);
  EXPECT_EQ(accept(DriveBesideAPigeon(g), 21000), 0u);
  g.Tick(26000, true);
  EXPECT_NE(g.snapshot().motors[0].voltage, 0.0);
}

// REQ-SAFE-24, REQ-DEV-24
TEST(SimBackend, AMotorResetBesideAValidSensorIsLocal) {
  SimBackend backend;
  ExpectAMotorResetBesideAValidSensorIsLocal(backend);
}

// REQ-SAFE-24, REQ-DEV-24
TEST(PhysicsSimBackend, AMotorResetBesideAValidSensorIsLocal) {
  sim::PhysicsSimBackend backend;
  ExpectAMotorResetBesideAValidSensorIsLocal(backend);
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
  g.Tick(502000, true);  // Restored, in a State read before it was.
  g.Tick(503000, true);
  command.observed_time_us = 503000;
  command.epoch = g.snapshot().epoch;
  ASSERT_TRUE(g.Accept(command, 3, 503000));
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
    c.pwm_outputs[0] = {41, PwmMode::kPulse, 0.5};
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
    EXPECT_EQ(backend.last_pwm_outputs[0],
              (PwmRequest{41, PwmMode::kPulse, 0.5}));
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
  g.Tick(now, true);  // Restored, in a State read before it was.
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
  g.Tick(now, true);  // Restored, in a State read before it was.
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

// A torque-current request reaches the backend as sent only for a motor with
// foc and a positive max_torque_current_a, within that limit and with no
// feedforward; 0 A is a driving request, applied as sent, not neutral. Any
// other torque-current request neutralizes that motor alone (counted), while
// the rest of the command drives. (A torque-current motor has no follower:
// config load refuses one.)
// REQ-SAFE-43
TEST(Gateway, TorqueCurrentDrivesOnlyAFocMotorWithinItsLimit) {
  FakeBackend backend;
  auto config = SwerveConfig(true);
  config.motors[0].foc = true;
  config.motors[0].max_torque_current_a = 10;
  config.motors[2].foc = true;  // No limit: the mode stays refused.
  Gateway g{config, backend, 1};
  const struct {
    const char* label;
    std::size_t motor;
    double demand, feedforward_v;
    bool sent;
  } rows[] = {
      {"at the limit", 0, -10, 0, true},
      {"0 A", 0, 0, 0, true},
      {"over the limit", 0, 10.5, 0, false},
      {"with feedforward", 0, 5, 0.5, false},
      {"no foc, no limit", 1, 0, 0, false},
      {"foc, no limit", 2, 0, 0, false},
  };
  uint64_t now = 1000, sequence = 1;
  uint32_t neutralized = 0;
  for (const auto& row : rows) {
    SCOPED_TRACE(row.label);
    g.Tick(now, true);
    auto command = Request(g);
    const MotorRequest torque{config.motors[row.motor].id, Mode::kTorqueCurrent,
                              0, row.demand, row.feedforward_v};
    command.motors[row.motor] = torque;
    ASSERT_TRUE(g.Accept(command, sequence++, now++));
    ASSERT_EQ(backend.last_requests.size(), config.motors.size());
    const MotorRequest neutral{torque.id, Mode::kNeutral, 0, 0, 0};
    EXPECT_EQ(backend.last_requests[row.motor], row.sent ? torque : neutral);
    neutralized += row.sent ? 0 : 1;
    EXPECT_EQ(backend.last_requests[3], command.motors[3]);
  }
  EXPECT_EQ(g.snapshot().counters.neutralized_motor_count, neutralized);
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

}  // namespace
}  // namespace talos::hardware
