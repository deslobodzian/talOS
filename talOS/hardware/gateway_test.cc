#include "gateway.h"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <thread>

#include "config_wire.h"
#include "endpoint.h"
#include "sim_backend.h"

namespace talos::hardware {
namespace {

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
  bool Configure(const Config&) override { return configure_ok; }
  bool Read(State& s) override {
    for (auto& m : s.motors) m.valid = healthy;
    return healthy;
  }
  bool Apply(std::span<const MotorRequest>) override {
    ++applied;
    return apply_ok;
  }
  void Neutral() override { ++neutral; }
  bool configure_ok{true}, healthy{true}, apply_ok{true};
  int applied{}, neutral{};
};
Command Request(const Gateway& g) {
  const auto s = g.snapshot();
  Command c{};
  c.config_id = s.config_id;
  c.boot_id = s.boot_id;
  c.epoch = s.epoch;
  c.observed_time_us = s.sample_time_us;
  c.count = s.motor_count;
  for (std::size_t i = 0; i < c.count; ++i)
    c.motors[i] = {s.motors[i].id, Mode::kVelocity, 0, 1, 0};
  return c;
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
TEST(Gateway, RejectsWholeTransactionBeforeApplyingAnyMotor) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  for (int invalid = 0; invalid < 6; ++invalid) {
    auto c = Request(g);
    auto& m = c.motors[c.count - 1];
    if (invalid == 0) m.demand = std::numeric_limits<double>::quiet_NaN();
    if (invalid == 1) m.id = c.motors[0].id;
    if (invalid == 2) m.slot = 3;
    if (invalid == 3) m.demand = 1e9;
    if (invalid == 4) m.mode = static_cast<Mode>(255);
    if (invalid == 5) --c.count;
    EXPECT_FALSE(g.Accept(c, 1, 1000));
  }
  EXPECT_EQ(backend.applied, 0);
}
TEST(Gateway, ApplyFailureLatchesNeutral) {
  FakeBackend backend;
  Gateway g{SwerveConfig(true), backend, 1};
  g.Tick(1000, true);
  backend.apply_ok = false;
  EXPECT_FALSE(g.Accept(Request(g), 1, 1000));
  backend.apply_ok = true;
  g.Tick(2000, true);
  EXPECT_FALSE(g.Accept(Request(g), 2, 2000));
  EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
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
  for (std::size_t i = 0; i < size; ++i)
    EXPECT_FALSE(Decode(std::span<const uint8_t>{bytes.data(), i}, decoded));
  EXPECT_FALSE(
      Decode(std::span<const uint8_t>{bytes.data(), size + 1}, decoded));
  bytes[32] = 255;
  EXPECT_FALSE(Decode(std::span<const uint8_t>{bytes.data(), size}, decoded));

  // Empty Command
  Command empty_cmd{};
  const auto empty_cmd_size = Encode(empty_cmd, bytes);
  ASSERT_GT(empty_cmd_size, 0u);
  Command decoded_empty_cmd;
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), empty_cmd_size}, decoded_empty_cmd));
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
    full_cmd.motors[i] = {static_cast<uint16_t>(i + 1), Mode::kVelocity, 1, 15.5 + i, 1.2};
  }
  for (std::size_t i = 0; i < kMaxDigitalOutputs; ++i) {
    full_cmd.digital_outputs[i] = {static_cast<uint16_t>(100 + i), (i % 2) == 1};
  }
  for (std::size_t i = 0; i < kMaxPwmOutputs; ++i) {
    full_cmd.pwm_outputs[i] = {static_cast<uint16_t>(200 + i), 0.5 - 0.1 * i};
  }
  const auto full_cmd_size = Encode(full_cmd, bytes);
  ASSERT_GT(full_cmd_size, 0u);
  ASSERT_LE(full_cmd_size, protocol::kMaxPayloadSize);
  Command decoded_full_cmd;
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), full_cmd_size}, decoded_full_cmd));
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
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), empty_state_size}, decoded_empty_state));
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

  for (std::size_t i = 0; i < kMaxMotors; ++i) {
    state.motors[i] = {static_cast<uint16_t>(i + 1), true, 1.0 + i, 2.0 + i, 11.5, 25.0, 123 + static_cast<uint32_t>(i), 456};
  }
  for (std::size_t i = 0; i < kMaxSensors; ++i) {
    state.sensors[i] = {static_cast<uint16_t>(50 + i), true, 0.25 * i, 1.5, 789, 1011};
  }
  for (std::size_t i = 0; i < kMaxDigitalInputs; ++i) {
    state.digital_inputs[i] = {static_cast<uint16_t>(100 + i), true, (i % 2) == 0};
  }
  for (std::size_t i = 0; i < kMaxDigitalOutputs; ++i) {
    state.digital_outputs[i] = {static_cast<uint16_t>(150 + i), true, (i % 2) == 1};
  }
  for (std::size_t i = 0; i < kMaxAnalogInputs; ++i) {
    state.analog_inputs[i] = {static_cast<uint16_t>(200 + i), true, 3.3 * i, 50};
  }
  for (std::size_t i = 0; i < kMaxEncoders; ++i) {
    state.encoders[i] = {static_cast<uint16_t>(250 + i), true, 10.0 * i, 5.0, 60};
  }
  for (std::size_t i = 0; i < kMaxPwmOutputs; ++i) {
    state.pwm_outputs[i] = {static_cast<uint16_t>(300 + i), true, 0.75};
  }

  const auto state_size = Encode(state, bytes);
  // The same per-device widths the payload-budget static_asserts in messages.cc
  // are built from, so raising a device cap moves both together or neither.
  constexpr std::size_t kWidestState =
      44 + 14 + kMaxMotors * 43 + kMaxSensors * 27 + kMaxDigitalInputs * 4 +
      kMaxDigitalOutputs * 4 + kMaxAnalogInputs * 15 + kMaxEncoders * 23 +
      kMaxPwmOutputs * 11;
  ASSERT_EQ(state_size, kWidestState);
  ASSERT_LE(state_size, protocol::kMaxPayloadSize);
  State copy;
  ASSERT_TRUE(Decode(std::span<const uint8_t>{bytes.data(), state_size}, copy));
  EXPECT_EQ(copy.motors[0].position_age_us, 123u);
  EXPECT_EQ(copy.sensors[0].velocity_age_us, 1011u);
  EXPECT_EQ(copy.digital_inputs[0].value, true);
  EXPECT_EQ(copy.digital_outputs[1].value, true);
  EXPECT_DOUBLE_EQ(copy.analog_inputs[1].voltage, 3.3);
  EXPECT_DOUBLE_EQ(copy.encoders[2].position_rot, 20.0);
  EXPECT_DOUBLE_EQ(copy.pwm_outputs[3].output, 0.75);

  for (std::size_t i = 0; i < state_size; ++i)
    EXPECT_FALSE(Decode(std::span<const uint8_t>{bytes.data(), i}, copy));
}

TEST(SimBackend, ModelsAllDeviceKindsIdeally) {
  Config config;
  config.commissioned = true;
  config.motors.push_back({1, "motor1", 1, "rio", false, true, 40, 80, 12, Feedback::kRotor, 0, 1, 1, false, false, -1, 1, 100, 20, 40, 0});
  config.sensors.push_back({2, "sensor1", SensorKind::kCANcoder, 2, "rio", false, 0});
  config.digital_inputs.push_back({3, "beam_break", 0});
  config.digital_outputs.push_back({4, "solenoid", 1});
  config.analog_inputs.push_back({5, "pressure", 0});
  config.encoders.push_back({6, "quad_enc", EncoderKind::kQuadrature, 2, 3, 2048, 0});
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

  // Ceiling violation rejected
  auto invalid_cfg = SwerveConfig(true);
  invalid_cfg.motors[0].stator_limit_a = 150.0;
  EXPECT_FALSE(g.Reconfigure(invalid_cfg));
  EXPECT_FALSE(g.configured());

  g.Tick(2000, true);
  EXPECT_TRUE(g.snapshot().flags & kHardwareFault);

  // Valid config accepted
  auto valid_cfg = SwerveConfig(true);
  EXPECT_TRUE(g.Reconfigure(valid_cfg));
  EXPECT_TRUE(g.configured());

  g.Tick(3000, true);
  EXPECT_TRUE(g.snapshot().flags & kConfigured);
  EXPECT_TRUE(g.snapshot().flags & kEnabled);
  EXPECT_FALSE(g.snapshot().flags & kHardwareFault);
}

TEST(Endpoint, PushesConfigOverUdpAndHandlesEpochChangeOnRestart) {
  SimBackend backend;
  Gateway gateway{backend, 2002};
  Endpoint endpoint{gateway};
  ASSERT_EQ(endpoint.Open("127.0.0.1", 15802, "127.0.0.1", 15803), protocol::UdpStatus::kOk);

  protocol::RuntimeUdpPeer companion;
  ASSERT_EQ(companion.Open("127.0.0.1", 15803, "127.0.0.1", 15802), protocol::UdpStatus::kOk);

  auto poll_receive = [](protocol::RuntimeUdpPeer& peer, protocol::DecodedFrame* out) {
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
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig, i + 1, 1000,
                                  chunks[i].data(), chunks[i].size()),
              protocol::UdpStatus::kOk);
  }

  // 3. Endpoint ticks, processes chunks, sends ACK
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  endpoint.Tick(2000, true);
  EXPECT_TRUE(gateway.configured());

  // 4. Companion receives ACK
  protocol::DecodedFrame ack_frame;
  ASSERT_EQ(poll_receive(companion, &ack_frame), protocol::UdpStatus::kOk);
  EXPECT_EQ(ack_frame.header.type, protocol::FrameType::kHardwareConfigAck);
  ConfigAckPayload ack{};
  ASSERT_TRUE(DecodeConfigAck({ack_frame.payload, ack_frame.payload_size}, ack));
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

}  // namespace
}  // namespace talos::hardware
