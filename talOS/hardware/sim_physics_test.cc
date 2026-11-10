// The ideal SimBackend stays the fast logic fake, while PhysicsSimBackend
// (sim_physics.h) earns trust for PID tuning, gearing, direction, and limits.
// All tests step an explicit sim clock; no wall-clock, no sockets, no live
// sim.
#include "sim_physics.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <tuple>

#include "gateway.h"
#include "sim_backend.h"

namespace talos::hardware {
namespace {

using sim::PhysicsSimBackend;

MotorConfig TestMotor(uint16_t id, const char* name, int can_id) {
  MotorConfig m;
  m.id = id;
  m.name = name;
  m.can_id = can_id;
  m.bus = "rio";
  return m;  // Zero gains: honesty checks rely on the missing-gain default.
}

Config SingleMotorConfig() {
  Config c;
  c.commissioned = true;
  c.command_timeout_us = 100000;
  c.motors.push_back(TestMotor(1, "roller", 1));
  return c;
}

MotorRequest Duty(uint16_t id, double demand) {
  MotorRequest r;
  r.id = id;
  r.mode = Mode::kDutyCycle;
  r.demand = demand;
  return r;
}

// Deterministic stepping through the Backend boundary: each Read advances
// physics by exactly step_us from the explicit sample timestamp.
void StepBackend(Backend& backend, State& s, uint64_t& now_us, uint64_t step_us,
                 int steps) {
  for (int i = 0; i < steps; ++i) {
    now_us += step_us;
    s.sample_time_us = now_us;
    backend.Read(s);
  }
}

TEST(SimMotorDiagnostics, AmbientAndEffortPlaceholdersAcrossNeutral) {
  auto cfg = SingleMotorConfig();
  cfg.motors[0].supply_limit_a = 40.0;
  SimBackend ideal;
  PhysicsSimBackend physics;
  for (Backend* backend :
       {static_cast<Backend*>(&ideal), static_cast<Backend*>(&physics)}) {
    ASSERT_TRUE(backend->Configure(cfg));
    std::array<MotorRequest, 1> command{{{1, Mode::kVoltage, 0, 6.0, 0.0}}};
    ASSERT_TRUE(backend->Apply(command));
    State state;
    state.motor_count = 1;
    uint64_t now_us = 0;
    StepBackend(*backend, state, now_us, 5000, 2);
    EXPECT_DOUBLE_EQ(state.motors[0].temperature_c, 25.0);
    EXPECT_DOUBLE_EQ(state.motors[0].supply_current_a, 20.0);
    backend->Neutral();
    StepBackend(*backend, state, now_us, 5000, 1);
    EXPECT_DOUBLE_EQ(state.motors[0].temperature_c, 25.0);
    EXPECT_DOUBLE_EQ(state.motors[0].supply_current_a, 0.0);
  }
}

TEST(SimGravity, ClosedLoopUsesSelectedSlotAndMechanismAngle) {
  MotorConfig cfg = TestMotor(1, "arm", 1);
  cfg.slots[1].g = 3.0;
  for (const auto mode :
       {Mode::kVelocity, Mode::kPosition, Mode::kMotionMagic}) {
    MotorRequest req{1, mode, 1, 0.0, 0.5};
    for (const double position : {0.0, 0.25, 0.5, 0.75, 1.0}) {
      double volts = 0.0;
      cfg.slots[1].gravity_type = GravityType::kElevatorStatic;
      ASSERT_TRUE(sim::DriveVoltage(req, cfg, position, 0.0, 12.0, volts));
      EXPECT_DOUBLE_EQ(volts, 3.5);
      cfg.slots[1].gravity_type = GravityType::kArmCosine;
      ASSERT_TRUE(sim::DriveVoltage(req, cfg, position, 0.0, 12.0, volts));
      EXPECT_NEAR(volts, 0.5 + 3.0 * std::cos(position * 2.0 * std::acos(-1.0)),
                  1e-12);
    }
    req.slot = 0;
    double volts = 0.0;
    ASSERT_TRUE(sim::DriveVoltage(req, cfg, 0.0, 0.0, 12.0, volts));
    EXPECT_DOUBLE_EQ(volts, 0.5);
  }
}

TEST(SimGravity, ProfiledMotionMagicIncludesGravityAndClampsVoltage) {
  for (const auto type :
       {GravityType::kElevatorStatic, GravityType::kArmCosine}) {
    auto cfg = SingleMotorConfig();
    cfg.motors[0].slots[0].g = 20.0;
    cfg.motors[0].slots[0].gravity_type = type;
    cfg.motors[0].cruise_velocity_rps = 1.0;
    cfg.motors[0].acceleration_rps2 = 2.0;
    cfg.motors[0].max_voltage = 5.0;
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(cfg));
    std::array<MotorRequest, kMaxMotors> commands{};
    commands[0] = {1, Mode::kMotionMagic, 0, 1.0, 0.0};
    ASSERT_TRUE(backend.Apply(commands));
    backend.Step(0.001);
    EXPECT_DOUBLE_EQ(backend.applied_voltage(0), 5.0);
    backend.Neutral();
    backend.Step(0.001);
    EXPECT_DOUBLE_EQ(backend.applied_voltage(0), 0.0);
  }
}

TEST(SimFollowers, PhysicsResolvesOrderOppositionAndNeutral) {
  for (const bool follower_first : {false, true}) {
    for (const bool oppose : {false, true}) {
      auto cfg = SingleMotorConfig();
      auto follower = TestMotor(2, "follower", 2);
      follower.master_id = 1;
      follower.oppose_master = oppose;
      cfg.motors.push_back(follower);
      if (follower_first) std::swap(cfg.motors[0], cfg.motors[1]);
      const std::size_t master = follower_first ? 1 : 0;
      const std::size_t slave = 1 - master;
      PhysicsSimBackend backend;
      ASSERT_TRUE(backend.Configure(cfg));
      std::array<MotorRequest, kMaxMotors> commands{};
      commands[master] = Duty(1, 0.5);
      commands[slave] = Duty(2, -0.9);  // Followers ignore independent demands.
      ASSERT_TRUE(backend.Apply(commands));
      backend.Step(0.005);
      EXPECT_DOUBLE_EQ(backend.applied_voltage(master), 6.0);
      EXPECT_DOUBLE_EQ(backend.applied_voltage(slave), oppose ? -6.0 : 6.0);
      State state{};
      ASSERT_TRUE(backend.Read(state));
      EXPECT_DOUBLE_EQ(state.motors[slave].velocity_rps,
                       state.motors[master].velocity_rps * (oppose ? -1 : 1));
      backend.SetMotorConnected(1, false);
      backend.Step(0.005);
      EXPECT_DOUBLE_EQ(backend.applied_voltage(slave), 0.0);
      backend.SetMotorConnected(1, true);
      commands[master] = {};
      ASSERT_TRUE(backend.Apply(commands));
      backend.Step(0.005);
      EXPECT_DOUBLE_EQ(backend.applied_voltage(slave), 0.0);
      commands[master] = Duty(1, 0.5);
      ASSERT_TRUE(backend.Apply(commands));
      backend.Step(0.005);
      backend.Neutral();
      backend.Step(0.005);
      EXPECT_DOUBLE_EQ(backend.applied_voltage(slave), 0.0);
    }
  }
}

TEST(SimFollowers, InvalidRemoteMasterFeedbackNeutralizesFollower) {
  auto cfg = SingleMotorConfig();
  cfg.motors[0].feedback = Feedback::kRemoteCANcoder;
  cfg.motors[0].feedback_sensor_id = 99;  // Missing sensor is invalid.
  auto follower = TestMotor(2, "follower", 2);
  follower.master_id = 1;
  cfg.motors.push_back(follower);
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(cfg));
  std::array<MotorRequest, kMaxMotors> commands{};
  commands[0] = Duty(1, 0.5);
  ASSERT_TRUE(backend.Apply(commands));
  backend.Step(0.005);
  EXPECT_DOUBLE_EQ(backend.applied_voltage(0), 0.0);
  EXPECT_DOUBLE_EQ(backend.applied_voltage(1), 0.0);
}

TEST(SimFollowers, IdealMirrorsPositionVelocityAndNeutralInEitherOrder) {
  for (const bool follower_first : {false, true}) {
    for (const bool oppose : {false, true}) {
      auto cfg = SingleMotorConfig();
      auto follower = TestMotor(2, "follower", 2);
      follower.master_id = 1;
      follower.oppose_master = oppose;
      cfg.motors.push_back(follower);
      if (follower_first) std::swap(cfg.motors[0], cfg.motors[1]);
      const std::size_t master = follower_first ? 1 : 0;
      const std::size_t slave = 1 - master;
      const double sign = oppose ? -1.0 : 1.0;
      SimBackend backend;
      ASSERT_TRUE(backend.Configure(cfg));
      std::array<MotorRequest, kMaxMotors> commands{};
      commands[master] = {1, Mode::kPosition, 0, 2.0, 0.0};
      commands[slave] = Duty(2, 1.0);
      ASSERT_TRUE(backend.Apply(commands));
      State state{};
      state.sample_time_us = 1000;
      ASSERT_TRUE(backend.Read(state));
      EXPECT_DOUBLE_EQ(state.motors[slave].position_rot, 2.0 * sign);
      commands[master] = {1, Mode::kVelocity, 0, 3.0, 0.0};
      ASSERT_TRUE(backend.Apply(commands));
      state.sample_time_us += 10000;
      ASSERT_TRUE(backend.Read(state));
      EXPECT_DOUBLE_EQ(state.motors[slave].velocity_rps, 3.0 * sign);
      EXPECT_DOUBLE_EQ(state.motors[slave].position_rot,
                       state.motors[master].position_rot * sign);
      backend.Neutral();
      state.sample_time_us += 10000;
      ASSERT_TRUE(backend.Read(state));
      EXPECT_DOUBLE_EQ(state.motors[slave].velocity_rps, 0.0);
    }
  }
}

template <typename Sim>
void CheckPowerHealth() {
  Sim backend;
  Gateway gateway{SingleMotorConfig(), backend, 1};
  gateway.Tick(1000, false);
  EXPECT_EQ(gateway.snapshot().health, (HealthSample{true, 12.0, false, 0.0}));
  backend.SetPowerHealth(6.5, true);
  gateway.Tick(2000, true);
  EXPECT_EQ(gateway.snapshot().health, (HealthSample{true, 6.5, true, 0.0}));
  // Health remains observable without a live command, including
  // timeout/disable.
  gateway.Tick(200000, true);
  EXPECT_TRUE(gateway.snapshot().health.brownout);
  EXPECT_EQ(gateway.snapshot().flags & kCommandActive, 0u);
  gateway.Tick(201000, false);
  EXPECT_DOUBLE_EQ(gateway.snapshot().health.battery_v, 6.5);
}

TEST(SimHealth, PowerVisibleWhileDisabledAndStale) {
  CheckPowerHealth<SimBackend>();
  CheckPowerHealth<PhysicsSimBackend>();
}

// Voltage ramps the mechanism rather than snapping it to demand; position
// integrates velocity.
TEST(SimBelievability, VoltageRampsMechanismAndPositionIntegrates) {
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(SingleMotorConfig()));
  std::array<MotorRequest, kMaxMotors> cmd{};
  cmd[0] = Duty(1, 0.5);
  ASSERT_TRUE(backend.Apply(cmd));
  State s{};
  uint64_t now_us = 0;
  // Two Reads: the first establishes the timestamp (dt = 0), the second
  // advances one 5 ms step, which only starts the ramp.
  StepBackend(backend, s, now_us, 5000, 2);
  EXPECT_GT(s.motors[0].velocity_rps, 0.0);
  EXPECT_LT(s.motors[0].velocity_rps, 50.0);   // Far from the ~50 rps target.
  StepBackend(backend, s, now_us, 5000, 198);  // 1 s at ~6 V.
  const double pos_1s = s.motors[0].position_rot;
  const double vel_1s = s.motors[0].velocity_rps;
  EXPECT_GT(vel_1s, 5.0);
  EXPECT_GT(pos_1s, 0.0);
  StepBackend(backend, s, now_us, 5000, 200);  // Another second.
  // Near steady state, the second second adds ~vel * 1 s.
  EXPECT_NEAR(s.motors[0].position_rot - pos_1s, vel_1s, vel_1s * 0.25 + 1.0);
  EXPECT_NEAR(s.motors[0].voltage, 6.0, 1e-9);
}

// Honesty: zero gains yield ~0 V, so velocity/position demands stall.
TEST(SimBelievability, ZeroGainsStallInsteadOfTeleporting) {
  for (const auto mode : {Mode::kVelocity, Mode::kPosition}) {
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(SingleMotorConfig()));
    std::array<MotorRequest, kMaxMotors> cmd{};
    cmd[0] = {1, mode, 0, mode == Mode::kVelocity ? 10.0 : 5.0, 0};
    ASSERT_TRUE(backend.Apply(cmd));
    State s{};
    uint64_t now_us = 0;
    StepBackend(backend, s, now_us, 5000, 200);
    EXPECT_LT(std::abs(s.motors[0].velocity_rps), 0.5);
    EXPECT_LT(std::abs(s.motors[0].position_rot), 0.5);
    EXPECT_LT(std::abs(s.motors[0].voltage), 0.5);
  }
}

// With tuned gains the loop tracks: the physics backend is usable for
// PID checks, the ideal backend would pass regardless of gains.
TEST(SimBelievability, TunedVelocityLoopTracksDemand) {
  Config c = SingleMotorConfig();
  c.motors[0].slots[0].p = 2.0;  // V per rps of error.
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(c));
  std::array<MotorRequest, kMaxMotors> cmd{};
  cmd[0] = {1, Mode::kVelocity, 0, 10.0, 0};
  ASSERT_TRUE(backend.Apply(cmd));
  State s{};
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 600);  // 3 s to settle.
  EXPECT_GT(s.motors[0].velocity_rps, 5.0);
  EXPECT_LE(s.motors[0].velocity_rps, 10.0);
}

// Gearing divides rotor speed into mechanism speed; soft limits clamp and
// stop the axis.
TEST(SimBelievability, GearingAndLimits) {
  // Gearing: identical volts, gear 2 must run half mechanism speed of gear 1.
  {
    PhysicsSimBackend geared, direct;
    Config g = SingleMotorConfig(), d = SingleMotorConfig();
    g.motors[0].rotor_to_sensor_ratio = 2.0;
    ASSERT_TRUE(geared.Configure(g));
    ASSERT_TRUE(direct.Configure(d));
    std::array<MotorRequest, kMaxMotors> cmd{};
    cmd[0] = Duty(1, 0.5);
    ASSERT_TRUE(geared.Apply(cmd));
    ASSERT_TRUE(direct.Apply(cmd));
    State sg{}, sd{};
    uint64_t now_us = 0;
    // Step both in lockstep on the same timestamps.
    for (int i = 0; i < 200; ++i) {
      now_us += 5000;
      sg.sample_time_us = now_us;
      sd.sample_time_us = now_us;
      geared.Read(sg);
      direct.Read(sd);
    }
    EXPECT_NEAR(sg.motors[0].velocity_rps * 2.0, sd.motors[0].velocity_rps,
                std::abs(sd.motors[0].velocity_rps) * 0.05 + 1e-6);
  }
  // Limits: the axis clamps at the forward soft limit and stops.
  {
    Config c = SingleMotorConfig();
    c.motors[0].soft_limits = true;
    c.motors[0].reverse_limit_rot = -1.0;
    c.motors[0].forward_limit_rot = 1.0;
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(c));
    std::array<MotorRequest, kMaxMotors> cmd{};
    cmd[0] = Duty(1, 1.0);
    ASSERT_TRUE(backend.Apply(cmd));
    State s{};
    uint64_t now_us = 0;
    StepBackend(backend, s, now_us, 5000, 600);
    EXPECT_LE(s.motors[0].position_rot, 1.0);
    EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
  }
}

// Phoenix 6's MotorOutput.Inverted flips the rotor sensor with the output, so
// a TalonFX reports position and velocity in its configured positive
// direction: inverted or not, a closed loop converges.
TEST(SimBelievability, InvertedMotorReportsItsConfiguredDirection) {
  for (const bool inverted : {false, true}) {
    for (const auto mode : {Mode::kPosition, Mode::kMotionMagic}) {
      Config c = SingleMotorConfig();
      c.motors[0].inverted = inverted;
      c.motors[0].cruise_velocity_rps = 2.0;
      c.motors[0].acceleration_rps2 = 4.0;
      c.motors[0].slots[0].p = 1.0;
      c.motors[0].slots[0].d = 0.05;
      c.motors[0].slots[0].v = 0.12;
      PhysicsSimBackend backend;
      ASSERT_TRUE(backend.Configure(c));
      std::array<MotorRequest, 1> cmd{{{1, mode, 0, 2.0, 0.0}}};
      ASSERT_TRUE(backend.Apply(cmd));
      State s{};
      uint64_t now_us = 0;
      StepBackend(backend, s, now_us, 5000, 1000);
      EXPECT_NEAR(s.motors[0].position_rot, 2.0, 0.01)
          << "inverted=" << inverted << " mode=" << static_cast<int>(mode);
      EXPECT_NEAR(s.motors[0].velocity_rps, 0.0, 0.01);
    }
  }
}

// A remote CANcoder keeps its own direction, and a TalonFX fed back from it
// reports and closes its loops on that reading. A loop whose motor and
// CANcoder directions agree converges; a mismatch is positive feedback and
// runs away, as it would on a robot.
TEST(SimBelievability, RemoteCancoderFeedbackKeepsTheEncoderDirection) {
  for (const bool motor_inverted : {false, true}) {
    for (const bool sensor_inverted : {false, true}) {
      Config c = SingleMotorConfig();
      c.motors[0].inverted = motor_inverted;
      SensorConfig encoder{2, "encoder", SensorKind::kCANcoder, 20, "rio"};
      encoder.inverted = sensor_inverted;
      c.sensors.push_back(encoder);
      c.motors[0].feedback = Feedback::kRemoteCANcoder;
      c.motors[0].feedback_sensor_id = 2;
      c.motors[0].slots[0].p = 1.0;
      c.motors[0].slots[0].d = 0.05;
      PhysicsSimBackend backend;
      ASSERT_TRUE(backend.Configure(c));
      std::array<MotorRequest, 1> cmd{{{1, Mode::kPosition, 0, 2.0, 0.0}}};
      ASSERT_TRUE(backend.Apply(cmd));
      State s{};
      uint64_t now_us = 0;
      StepBackend(backend, s, now_us, 5000, 1000);
      EXPECT_DOUBLE_EQ(s.motors[0].position_rot, s.sensors[0].position_rot);
      if (motor_inverted == sensor_inverted) {
        EXPECT_NEAR(s.motors[0].position_rot, 2.0, 0.01)
            << "motor_inverted=" << motor_inverted;
      } else {
        EXPECT_LT(s.motors[0].position_rot, -100.0)
            << "motor_inverted=" << motor_inverted;
      }
    }
  }
}

// Under Phoenix 6's Follower request a follower drives in its master's
// configured direction, reversed when opposed; its own Inverted does not
// apply. CANcoders on each rotor show the physical direction.
TEST(SimFollowers, FollowerTakesItsMasterDirectionNotItsOwnInverted) {
  for (const bool master_inverted : {false, true}) {
    for (const bool follower_inverted : {false, true}) {
      for (const bool oppose : {false, true}) {
        Config c = SingleMotorConfig();
        c.motors[0].inverted = master_inverted;
        auto follower = TestMotor(2, "follower", 2);
        follower.master_id = 1;
        follower.oppose_master = oppose;
        follower.inverted = follower_inverted;
        c.motors.push_back(follower);
        for (std::size_t i = 0; i < 2; ++i) {
          const auto id = static_cast<uint16_t>(10 + i);
          c.sensors.push_back(
              {id, i == 0 ? "master_encoder" : "follower_encoder",
               SensorKind::kCANcoder, static_cast<int>(20 + i), "rio"});
          c.motors[i].feedback = Feedback::kRemoteCANcoder;
          c.motors[i].feedback_sensor_id = id;
        }
        PhysicsSimBackend backend;
        ASSERT_TRUE(backend.Configure(c));
        std::array<MotorRequest, kMaxMotors> cmd{};
        cmd[0] = Duty(1, 0.5);
        cmd[1] = Duty(2, 0.0);  // A follower runs only while not neutral.
        ASSERT_TRUE(backend.Apply(cmd));
        State s{};
        uint64_t now_us = 0;
        StepBackend(backend, s, now_us, 5000, 100);
        const double master_rps = s.sensors[0].velocity_rps;
        EXPECT_GT(master_rps * (master_inverted ? -1.0 : 1.0), 10.0);
        EXPECT_NEAR(s.sensors[1].velocity_rps,
                    master_rps * (oppose ? -1.0 : 1.0), 1e-9)
            << "master_inverted=" << master_inverted
            << " follower_inverted=" << follower_inverted
            << " oppose=" << oppose;
      }
    }
  }
}

// A pressed switch on the TalonFX's forward input holds forward output
// neutral in every mode (here duty cycle, which soft limits in the gateway do
// not check) while reverse output still drives off it; a disabled input does
// nothing. Forward is the configured positive direction, inverted or not.
TEST(SimLimitSwitches, PressedForwardSwitchStopsOnlyForwardOutput) {
  for (const bool disabled : {false, true}) {
    Config c = SingleMotorConfig();
    c.motors[0].inverted = true;
    if (disabled) c.motors[0].forward_limit_switch = LimitSwitch::kDisabled;
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(c));
    backend.SetForwardLimitPressed(1, true);
    std::array<MotorRequest, 1> cmd{Duty(1, 0.5)};
    ASSERT_TRUE(backend.Apply(cmd));
    State s{};
    uint64_t now_us = 0;
    StepBackend(backend, s, now_us, 5000, 100);
    if (disabled) {
      EXPECT_GT(backend.applied_voltage(0), 5.0);
      EXPECT_GT(std::abs(s.motors[0].velocity_rps), 10.0);
      continue;
    }
    EXPECT_DOUBLE_EQ(backend.applied_voltage(0), 0.0);
    EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
    cmd[0] = Duty(1, -0.5);
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, s, now_us, 5000, 100);
    EXPECT_LT(backend.applied_voltage(0), -5.0);
    EXPECT_GT(std::abs(s.motors[0].velocity_rps), 10.0);
  }
}

TEST(SimBelievability, PigeonMountOffsetPrecedesInversion) {
  for (const bool inverted : {false, true}) {
    Config config = SingleMotorConfig();
    SensorConfig imu{9, "imu", SensorKind::kPigeon2, 30, "rio"};
    imu.offset_rot = 0.125;
    imu.inverted = inverted;
    config.sensors.push_back(imu);
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(config));
    backend.SetChassisYawRate(0.25);
    State state{};
    uint64_t now_us = 0;
    StepBackend(backend, state, now_us, 5000, 201);
    const double sign = inverted ? -1.0 : 1.0;
    EXPECT_NEAR(state.sensors[0].position_rot, sign * 0.375, 1e-9);
    EXPECT_NEAR(state.sensors[0].velocity_rps, sign * 0.25, 1e-9);
  }
}

TEST(SimBelievability, ResetRestorePreservesPhysicalRotorPosition) {
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(SingleMotorConfig()));
  const std::array commands{Duty(1, 0.5)};
  ASSERT_TRUE(backend.Apply(commands));
  State state{};
  uint64_t now_us = 0;
  StepBackend(backend, state, now_us, 5000, 201);
  const double position = state.motors[0].position_rot;
  ASSERT_GT(position, 0.0);
  ASSERT_GT(state.motors[0].velocity_rps, 0.0);
  const std::array<uint16_t, 1> reset_ids{1};
  backend.BeginRestoreAfterReset(reset_ids);
  ASSERT_TRUE(backend.Read(state));
  EXPECT_DOUBLE_EQ(state.motors[0].position_rot, position);
  EXPECT_DOUBLE_EQ(state.motors[0].velocity_rps, 0.0);
  EXPECT_DOUBLE_EQ(state.motors[0].voltage, 0.0);
  StepBackend(backend, state, now_us, 5000, 20);
  EXPECT_DOUBLE_EQ(state.motors[0].position_rot, position);
}

// CANcoders mirror the linked mechanism through the boundary.
TEST(SimBelievability, CancoderMirrorsLinkedMechanism) {
  Config c = SingleMotorConfig();
  c.sensors.push_back({2, "encoder", SensorKind::kCANcoder, 20, "rio"});
  c.motors[0].feedback = Feedback::kRemoteCANcoder;
  c.motors[0].feedback_sensor_id = 2;
  c.motors[0].sensor_to_mechanism_ratio = 2.0;
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(c));
  std::array<MotorRequest, kMaxMotors> cmd{};
  cmd[0] = Duty(1, 0.5);
  ASSERT_TRUE(backend.Apply(cmd));
  State s{};
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 200);
  EXPECT_TRUE(s.sensors[0].valid);
  EXPECT_NEAR(s.sensors[0].position_rot, s.motors[0].position_rot, 1e-9);
  EXPECT_NEAR(s.sensors[0].velocity_rps, s.motors[0].velocity_rps, 1e-9);
}

// Commanded neutral coasts/brakes per model; it never teleports to a
// stop. A zero-velocity demand instead holds an active standstill.
TEST(SimBelievability, NeutralDecaysPerBrakeWhileZeroVelocityHolds) {
  auto spin_up = [](bool brake) {
    Config c = SingleMotorConfig();
    c.motors[0].brake = brake;
    auto backend = std::make_unique<PhysicsSimBackend>();
    EXPECT_TRUE(backend->Configure(c));
    std::array<MotorRequest, kMaxMotors> cmd{};
    cmd[0] = Duty(1, 1.0);
    EXPECT_TRUE(backend->Apply(cmd));
    State s{};
    uint64_t now_us = 0;
    StepBackend(*backend, s, now_us, 5000, 400);
    EXPECT_GT(s.motors[0].velocity_rps, 50.0);
    return std::make_tuple(std::move(backend), s, now_us);
  };
  // Brake: neutral stops quickly.
  {
    auto [backend, s, now_us] = spin_up(true);
    std::array<MotorRequest, kMaxMotors> cmd{};  // kNeutral default.
    EXPECT_TRUE(backend->Apply(cmd));
    StepBackend(*backend, s, now_us, 5000, 100);  // 0.5 s coasting down.
    EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
    EXPECT_DOUBLE_EQ(s.motors[0].voltage, 0.0);
  }
  // Coast: neutral keeps the roller moving well past the brake stop time.
  {
    auto [backend, s, now_us] = spin_up(false);
    std::array<MotorRequest, kMaxMotors> cmd{};
    EXPECT_TRUE(backend->Apply(cmd));
    StepBackend(*backend, s, now_us, 5000, 100);
    EXPECT_GT(s.motors[0].velocity_rps, 5.0);
    EXPECT_GT(s.motors[0].position_rot, 0.0);
  }
  // Standstill: a tuned zero-velocity demand holds at zero without rolling.
  {
    Config c = SingleMotorConfig();
    c.motors[0].slots[0].p = 2.0;
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(c));
    std::array<MotorRequest, kMaxMotors> cmd{};
    cmd[0] = Duty(1, 0.3);  // Gentle spin-up so the hold-settle travel fits.
    ASSERT_TRUE(backend.Apply(cmd));
    State s{};
    uint64_t now_us = 0;
    StepBackend(backend, s, now_us, 5000, 400);
    cmd[0] = {1, Mode::kVelocity, 0, 0.0, 0};
    ASSERT_TRUE(backend.Apply(cmd));
    const double pos_at_hold = s.motors[0].position_rot;
    StepBackend(backend, s, now_us, 5000, 200);
    EXPECT_LT(std::abs(s.motors[0].velocity_rps), 0.5);
    EXPECT_LT(std::abs(s.motors[0].position_rot - pos_at_hold), 1.0);
  }
}

// Through the Gateway: disable neutralizes drive but the rotor decays,
// it does not snap to zero.
TEST(SimBelievability, GatewayDisableDecaysRatherThanSnaps) {
  PhysicsSimBackend backend;
  Gateway g{SingleMotorConfig(), backend, 7};
  uint64_t seq = 1;
  // Production re-sends the latest command every tick; a one-shot Accept
  // would age out mid-test and invalidate before the disable edge.
  auto send = [&](uint64_t now) {
    Command c{};
    const State snap = g.snapshot();
    c.config_id = snap.config_id;
    c.boot_id = snap.boot_id;
    c.epoch = snap.epoch;
    c.observed_time_us = snap.sample_time_us;
    c.count = 1;
    c.motors[0] = Duty(1, 1.0);
    return g.Accept(c, seq++, now);
  };
  g.Tick(1000, true);
  ASSERT_TRUE(send(1000));
  for (uint64_t t = 6000; t <= 1000000; t += 5000) {
    g.Tick(t, true);
    ASSERT_TRUE(send(t));
  }
  const double spinning = g.snapshot().motors[0].velocity_rps;
  ASSERT_GT(spinning, 50.0);
  g.Tick(1005000, false);  // Disable: drive removed, rotor still turning.
  const double at_disable = g.snapshot().motors[0].velocity_rps;
  EXPECT_GT(at_disable, 0.0);
  EXPECT_LT(at_disable, spinning);  // One brake step has already decayed it.
  for (uint64_t t = 1010000; t <= 3005000; t += 5000) g.Tick(t, false);
  EXPECT_DOUBLE_EQ(g.snapshot().motors[0].velocity_rps, 0.0);
}

// Frozen feedback holds stale samples while physics keeps moving.
TEST(SimBelievability, FrozenFeedbackHoldsStaleSamples) {
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(SingleMotorConfig()));
  std::array<MotorRequest, kMaxMotors> cmd{};
  cmd[0] = Duty(1, 0.5);
  ASSERT_TRUE(backend.Apply(cmd));
  State s{};
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 100);
  backend.FreezeFeedback(1);
  const double held_pos = s.motors[0].position_rot;
  StepBackend(backend, s, now_us, 5000, 100);
  EXPECT_DOUBLE_EQ(s.motors[0].position_rot, held_pos);
  EXPECT_TRUE(s.motors[0].valid);  // Frozen is stale, not disconnected.
  backend.UnfreezeFeedback(1);
  StepBackend(backend, s, now_us, 5000, 100);
  EXPECT_GT(s.motors[0].position_rot, held_pos);
}

// Individual disconnects invalidate only that sample; total bus loss fails.
TEST(SimBelievability, DisconnectIsIsolatedUntilAllDevicesAreLost) {
  Config c = SingleMotorConfig();
  c.sensors.push_back({9, "imu", SensorKind::kPigeon2, 30, "rio"});
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(c));
  State s{};
  s.sample_time_us = 5000;
  ASSERT_TRUE(backend.Read(s));
  backend.SetMotorConnected(1, false);
  s.sample_time_us = 10000;
  EXPECT_TRUE(backend.Read(s));
  EXPECT_FALSE(s.motors[0].valid);
  backend.SetMotorConnected(1, true);
  s.sample_time_us = 15000;
  EXPECT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.motors[0].valid);
  backend.SetSensorConnected(9, false);
  s.sample_time_us = 20000;
  EXPECT_TRUE(backend.Read(s));
  EXPECT_FALSE(s.sensors[0].valid);
  backend.SetMotorConnected(1, false);
  EXPECT_FALSE(backend.Read(s));
  backend.SetSensorConnected(9, true);
  s.sample_time_us = 25000;
  EXPECT_TRUE(backend.Read(s));
}

TEST(SimBelievability, EmptyBusRemainsHealthy) {
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(Config{}));
  State state{};
  EXPECT_TRUE(backend.Read(state));
}

TEST(SimBelievability, MotionMagicProfilesCruiseAndSettlesInBothDirections) {
  for (double direction : {-1.0, 1.0}) {
    Config config = SingleMotorConfig();
    auto& motor = config.motors[0];
    motor.cruise_velocity_rps = 2.0;
    motor.acceleration_rps2 = 4.0;
    motor.slots[0].p = 1.0;
    motor.slots[0].v = 0.12;
    motor.slots[0].a = 0.006;  // 50 ms mechanism time constant times kV.
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(config));
    std::array<MotorRequest, kMaxMotors> commands{};
    commands[0] = {1, Mode::kMotionMagic, 0, direction * 6.0, 0};
    ASSERT_TRUE(backend.Apply(commands));
    State state{};
    uint64_t now = 0;
    double peak_velocity = 0.0;
    for (int i = 0; i < 1600; ++i) {
      StepBackend(backend, state, now, 5000, 1);
      peak_velocity =
          std::max(peak_velocity, std::abs(state.motors[0].velocity_rps));
    }
    EXPECT_GT(peak_velocity, 1.9);
    EXPECT_LT(peak_velocity, 2.1);
    EXPECT_NEAR(state.motors[0].position_rot, direction * 6.0, 0.01);
    EXPECT_NEAR(state.motors[0].velocity_rps, 0.0, 0.01);
  }
}

// Conventional module names infer swerve geometry, so wheel motion alone
// rotates the chassis that the Pigeon reports.
TEST(SimBelievability, SwerveRotationDrivesPigeonAndExplicitOverrideWins) {
  Config config;
  const std::array<const char*, 4> names{"front_left", "front_right",
                                         "back_left", "back_right"};
  const std::array<double, 4> angles{0.375, 0.125, -0.375, -0.125};
  std::array<MotorRequest, kMaxMotors> commands{};
  for (std::size_t i = 0; i < names.size(); ++i) {
    auto drive = TestMotor(2 * i + 1, "drive", static_cast<int>(2 * i + 1));
    drive.name = std::string(names[i]) + "_drive";
    auto steer = TestMotor(2 * i + 2, "steer", static_cast<int>(2 * i + 2));
    steer.name = std::string(names[i]) + "_steer";
    steer.slots[0].p = 1.0;
    config.motors.push_back(drive);
    config.motors.push_back(steer);
    commands[2 * i + 1] = {steer.id, Mode::kPosition, 0, angles[i], 0};
  }
  config.sensors.push_back({20, "imu", SensorKind::kPigeon2, 30, "rio"});
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(backend.Apply(commands));
  State state{};
  uint64_t now = 0;
  StepBackend(backend, state, now, 5000, 400);
  for (std::size_t i = 0; i < names.size(); ++i) {
    commands[2 * i] = Duty(2 * i + 1, 0.02);
  }
  ASSERT_TRUE(backend.Apply(commands));
  StepBackend(backend, state, now, 5000, 201);
  EXPECT_GT(state.sensors[0].position_rot, 0.15);
  EXPECT_GT(state.sensors[0].velocity_rps, 0.2);
  backend.SetGyro(2.0, -0.25);
  StepBackend(backend, state, now, 5000, 200);
  EXPECT_NEAR(state.sensors[0].position_rot, 1.75, 1e-9);
  EXPECT_DOUBLE_EQ(state.sensors[0].velocity_rps, -0.25);
}

// Supply-voltage change scales speed; restart clears motion safely.
TEST(SimBelievability, SupplyVoltageScalesSpeedAndRestartZeroesMotion) {
  PhysicsSimBackend low, high;
  Config c = SingleMotorConfig();
  ASSERT_TRUE(low.Configure(c));
  ASSERT_TRUE(high.Configure(c));
  low.SetSupplyVoltage(6.0);
  std::array<MotorRequest, kMaxMotors> cmd{};
  cmd[0] = Duty(1, 1.0);  // Full duty: 6 V vs 12 V rails.
  ASSERT_TRUE(low.Apply(cmd));
  ASSERT_TRUE(high.Apply(cmd));
  State sl{}, sh{};
  uint64_t now_us = 0;
  for (int i = 0; i < 400; ++i) {
    now_us += 5000;
    sl.sample_time_us = now_us;
    sh.sample_time_us = now_us;
    low.Read(sl);
    high.Read(sh);
  }
  EXPECT_NEAR(sl.motors[0].velocity_rps, sh.motors[0].velocity_rps / 2.0,
              std::abs(sh.motors[0].velocity_rps) * 0.1 + 1e-6);
  high.RestartController();
  sh.sample_time_us = now_us + 5000;
  ASSERT_TRUE(high.Read(sh));
  EXPECT_DOUBLE_EQ(sh.motors[0].position_rot, 0.0);
  EXPECT_DOUBLE_EQ(sh.motors[0].velocity_rps, 0.0);
}

// Deterministic stepping: the same timestamps always give the same
// state, and a lone Step() matches timestamp-driven Reads.
TEST(SimBelievability, SteppingIsDeterministic) {
  auto run = [] {
    PhysicsSimBackend backend;
    EXPECT_TRUE(backend.Configure(SingleMotorConfig()));
    std::array<MotorRequest, kMaxMotors> cmd{};
    cmd[0] = Duty(1, 0.7);
    EXPECT_TRUE(backend.Apply(cmd));
    State s{};
    uint64_t now_us = 0;
    StepBackend(backend, s, now_us, 5000, 123);
    return s.motors[0];
  };
  const MotorSample a = run();
  const MotorSample b = run();
  EXPECT_DOUBLE_EQ(a.position_rot, b.position_rot);
  EXPECT_DOUBLE_EQ(a.velocity_rps, b.velocity_rps);
  // Direct Step() agrees with timestamp-driven Reads for the same dt.
  PhysicsSimBackend stepped, stamped;
  ASSERT_TRUE(stepped.Configure(SingleMotorConfig()));
  ASSERT_TRUE(stamped.Configure(SingleMotorConfig()));
  std::array<MotorRequest, kMaxMotors> cmd{};
  cmd[0] = Duty(1, 0.7);
  ASSERT_TRUE(stepped.Apply(cmd));
  ASSERT_TRUE(stamped.Apply(cmd));
  for (int i = 0; i < 50; ++i) stepped.Step(0.005);
  State s{};
  uint64_t now_us = 0;
  // 51 Reads: the first establishes the timestamp (dt = 0), the next 50 step.
  StepBackend(stamped, s, now_us, 5000, 51);
  State q{};
  q.sample_time_us = 1;  // No previous timestamp: dt = 0, reports as-is.
  ASSERT_TRUE(stepped.Read(q));
  EXPECT_NEAR(s.motors[0].position_rot, q.motors[0].position_rot, 1e-6);
  EXPECT_NEAR(s.motors[0].velocity_rps, q.motors[0].velocity_rps, 1e-6);
}

// Four independently controlled modules; all observations cross Read() with
// an explicit sim clock. Non-centered layouts exercise translation elimination
// in the chassis least-squares solve rather than assuming symmetry.
struct WorldFixture {
  static constexpr double kTau = 6.28318530717958647692;
  static constexpr double kRadius = 0.0508;
  PhysicsSimBackend backend;
  State state{};
  uint64_t now_us{};
  std::array<MotorRequest, kMaxMotors> commands{};
  std::array<double, 4> x{0.3, 0.3, -0.3, -0.3};
  std::array<double, 4> y{0.3, -0.3, 0.3, -0.3};

  explicit WorldFixture(sim::PhysicsOptions options = {},
                        bool off_center = false)
      : backend{options} {
    Config config;
    for (std::size_t i = 0; i < 4; ++i) {
      auto drive = TestMotor(2 * i + 1, "drive", static_cast<int>(2 * i + 1));
      auto steer = TestMotor(2 * i + 2, "steer", static_cast<int>(2 * i + 2));
      drive.name += std::to_string(i);
      steer.name += std::to_string(i);
      steer.slots[0].p = 1.0;
      config.motors.push_back(drive);
      config.motors.push_back(steer);
    }
    config.sensors.push_back({20, "imu", SensorKind::kPigeon2, 30, "rio"});
    EXPECT_TRUE(backend.Configure(config));
    for (std::size_t i = 0; i < 4; ++i) {
      if (off_center) {
        x[i] += 0.15;
        y[i] += 0.07;
      }
      backend.SetSwerveModuleGeometry(2 * i + 1, 2 * i + 2, x[i], y[i]);
    }
    StepBackend(backend, state, now_us, 5000, 1);
  }

  void Drive(double vx, double vy, double omega) {
    for (std::size_t i = 0; i < 4; ++i) {
      const double wheel_vx = vx - omega * y[i];
      const double wheel_vy = vy + omega * x[i];
      commands[2 * i + 1] = {static_cast<uint16_t>(2 * i + 2), Mode::kPosition,
                             0, std::atan2(wheel_vy, wheel_vx) / kTau, 0.0};
    }
    EXPECT_TRUE(backend.Apply(commands));
    Advance(2000);  // Settle azimuths before applying drive power.
    for (std::size_t i = 0; i < 4; ++i) {
      const double speed = std::hypot(vx - omega * y[i], vy + omega * x[i]);
      commands[2 * i] = Duty(2 * i + 1, speed / (100.0 * kTau * kRadius));
    }
    EXPECT_TRUE(backend.Apply(commands));
  }

  void Advance(int steps) { StepBackend(backend, state, now_us, 5000, steps); }
};

TEST(SimWorld, ZeroErrorStraightRotationAndArcMatchCommandedChassisMotion) {
  for (const auto [vx, vy, omega] :
       {std::tuple{1.0, 0.0, 0.0}, std::tuple{0.0, 0.0, 0.8},
        std::tuple{0.8, 0.3, 0.6}}) {
    WorldFixture world{
        {.start_x_m = 2.0, .start_y_m = 3.0, .start_yaw_rad = 0.0}, true};
    world.Drive(vx, vy, omega);
    world.Advance(400);  // Reach steady commanded speed.
    const auto before = world.backend.truth();
    world.Advance(200);
    const auto& after = world.backend.truth();
    const double sinc = omega == 0.0 ? 1.0 : std::sin(omega) / omega;
    const double cosc = omega == 0.0 ? 0.0 : (1.0 - std::cos(omega)) / omega;
    const double dx = sinc * vx - cosc * vy;
    const double dy = cosc * vx + sinc * vy;
    EXPECT_NEAR(after.x_m,
                before.x_m + std::cos(before.yaw_rad) * dx -
                    std::sin(before.yaw_rad) * dy,
                1e-8);
    EXPECT_NEAR(after.y_m,
                before.y_m + std::sin(before.yaw_rad) * dx +
                    std::cos(before.yaw_rad) * dy,
                1e-8);
    EXPECT_NEAR(after.yaw_rad, before.yaw_rad + omega, 1e-8);
    EXPECT_NEAR(after.vx_mps,
                std::cos(after.yaw_rad) * vx - std::sin(after.yaw_rad) * vy,
                1e-8);
    EXPECT_NEAR(after.vy_mps,
                std::sin(after.yaw_rad) * vx + std::cos(after.yaw_rad) * vy,
                1e-8);
    EXPECT_NEAR(after.omega_radps, omega, 1e-8);
    EXPECT_NEAR(world.state.sensors[0].position_rot,
                after.yaw_rad / WorldFixture::kTau, 1e-12);
    EXPECT_NEAR(world.state.sensors[0].velocity_rps, omega / WorldFixture::kTau,
                1e-8);
  }
}

TEST(SimWorld, RadiusErrorChangesGroundTravelButNotWheelEncoders) {
  WorldFixture nominal;
  WorldFixture larger{{.wheel_radius_error_frac = 0.2}};
  nominal.Drive(1.0, 0.0, 0.0);
  larger.Drive(1.0, 0.0, 0.0);
  nominal.Advance(400);
  larger.Advance(400);
  const double odometric_travel = larger.state.motors[0].position_rot *
                                  WorldFixture::kTau * WorldFixture::kRadius;
  EXPECT_NEAR(larger.backend.truth().x_m, 1.2 * odometric_travel, 1e-10);
  EXPECT_DOUBLE_EQ(larger.state.motors[0].position_rot,
                   nominal.state.motors[0].position_rot);
  EXPECT_DOUBLE_EQ(larger.state.motors[0].velocity_rps,
                   nominal.state.motors[0].velocity_rps);
  EXPECT_NEAR(nominal.backend.truth().x_m, odometric_travel, 1e-10);
}

TEST(SimWorld, SlipReducesTravelOnlyDuringWheelAcceleration) {
  WorldFixture nominal;
  WorldFixture slipping{{.wheel_slip_gain = 0.3}};
  nominal.Drive(1.0, 0.0, 0.0);
  slipping.Drive(1.0, 0.0, 0.0);
  nominal.Advance(40);
  slipping.Advance(40);
  EXPECT_LT(slipping.backend.truth().x_m, nominal.backend.truth().x_m);
  EXPECT_DOUBLE_EQ(slipping.state.motors[0].position_rot,
                   nominal.state.motors[0].position_rot);
  nominal.Advance(360);
  slipping.Advance(360);
  const double nominal_before = nominal.backend.truth().x_m;
  const double slip_before = slipping.backend.truth().x_m;
  nominal.Advance(200);
  slipping.Advance(200);
  EXPECT_NEAR(slipping.backend.truth().x_m - slip_before,
              nominal.backend.truth().x_m - nominal_before, 1e-10);
  EXPECT_NEAR(slipping.backend.truth().vx_mps, 1.0, 1e-10);
}

TEST(SimWorld, GyroDriftIsLinearAndDoesNotRotatePhysicalTruth) {
  WorldFixture world{{.gyro_drift_radps = 0.03}};
  for (int second = 1; second <= 3; ++second) {
    world.Advance(200);
    const auto& truth = world.backend.truth();
    EXPECT_DOUBLE_EQ(truth.yaw_rad, 0.0);
    EXPECT_NEAR(world.state.sensors[0].position_rot * WorldFixture::kTau -
                    truth.yaw_rad,
                0.03 * second, 1e-12);
    EXPECT_NEAR(world.state.sensors[0].velocity_rps, 0.03 / WorldFixture::kTau,
                1e-12);
  }
}

TEST(SimWorld, StartPoseSurvivesControllerRestartWhileReportedGyroZeroes) {
  WorldFixture world{
      {.start_x_m = 2.0, .start_y_m = 4.0, .start_yaw_rad = 0.7}};
  EXPECT_DOUBLE_EQ(world.backend.truth().x_m, 2.0);
  EXPECT_DOUBLE_EQ(world.backend.truth().y_m, 4.0);
  EXPECT_DOUBLE_EQ(world.backend.truth().yaw_rad, 0.7);
  EXPECT_DOUBLE_EQ(world.state.sensors[0].position_rot, 0.0);
  world.Drive(1.0, 0.0, 0.3);
  world.Advance(200);
  const auto before = world.backend.truth();
  world.backend.RestartController();
  world.Advance(1);
  const auto& after = world.backend.truth();
  EXPECT_DOUBLE_EQ(after.x_m, before.x_m);
  EXPECT_DOUBLE_EQ(after.y_m, before.y_m);
  EXPECT_DOUBLE_EQ(after.yaw_rad, before.yaw_rad);
  EXPECT_DOUBLE_EQ(after.vx_mps, 0.0);
  EXPECT_DOUBLE_EQ(after.vy_mps, 0.0);
  EXPECT_DOUBLE_EQ(after.omega_radps, 0.0);
  EXPECT_DOUBLE_EQ(world.state.sensors[0].position_rot, 0.0);
  EXPECT_DOUBLE_EQ(world.state.sensors[0].velocity_rps, 0.0);
  EXPECT_DOUBLE_EQ(world.state.motors[0].position_rot, 0.0);
  EXPECT_DOUBLE_EQ(world.state.motors[0].velocity_rps, 0.0);
}

TEST(SimWorld, ExplicitGyroOverridesDoNotTeleportTruth) {
  WorldFixture world{{.start_yaw_rad = 0.7}};
  world.backend.SetGyro(2.0, -0.25);
  world.Advance(200);
  EXPECT_DOUBLE_EQ(world.backend.truth().yaw_rad, 0.7);
  EXPECT_NEAR(world.state.sensors[0].position_rot, 1.75, 1e-12);
  EXPECT_DOUBLE_EQ(world.state.sensors[0].velocity_rps, -0.25);
}

}  // namespace
}  // namespace talos::hardware
