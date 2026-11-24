// The ideal SimBackend stays the fast logic fake, while PhysicsSimBackend
// (sim_physics.h) earns trust for PID tuning, gearing, direction, and limits.
// All tests step an explicit sim clock; no wall-clock, no sockets, no live
// sim.
#include "sim_physics.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "config_wire.h"
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
    if (backend == &ideal) {
      EXPECT_DOUBLE_EQ(state.motors[0].supply_current_a, 20.0);
    } else {
      EXPECT_GT(state.motors[0].supply_current_a, 0.0);
      EXPECT_LE(state.motors[0].supply_current_a, 40.0);
      // At 6 V command on a 12.6 V bus, step-down power conservation keeps
      // battery draw strictly below winding (stator) current.
      EXPECT_LT(state.motors[0].supply_current_a,
                state.motors[0].stator_current_a);
    }
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

// A velocity request whose feedforward alone (slot v times demand) exceeds
// max_voltage applies max_voltage, as the Talon's peak voltage holds
// VelocityVoltage: 0.1 V/rps at ±30 rps asks ±3 V of a 2 V motor.
// REQ-DEV-10
TEST(SimVoltageCeiling, VelocityIsClampedToMaxVoltage) {
  MotorConfig cfg = TestMotor(1, "roller", 1);
  cfg.max_voltage = 2.0;
  cfg.slots[0].v = 0.1;
  for (const double sign : {1.0, -1.0}) {
    const MotorRequest req{1, Mode::kVelocity, 0, sign * 30.0, 0.0};
    double volts = 0.0;
    ASSERT_TRUE(sim::DriveVoltage(req, cfg, 0.0, 0.0, 12.0, volts));
    EXPECT_DOUBLE_EQ(volts, sign * 2.0);
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
    // Above stall so max_voltage, not current limiting, is the binding clamp.
    cfg.motors[0].stator_limit_a = 400.0;
    cfg.motors[0].supply_limit_a = 400.0;
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
      // Above stall so duty scaling to 0.5 * supply_v is not current-limited.
      cfg.motors[0].stator_limit_a = 400.0;
      cfg.motors[0].supply_limit_a = 400.0;
      auto follower = TestMotor(2, "follower", 2);
      follower.master_id = 1;
      follower.oppose_master = oppose;
      follower.stator_limit_a = 400.0;
      follower.supply_limit_a = 400.0;
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
      const double half = 0.5 * kSimRestingSupplyV;  // Duty scales the bus.
      EXPECT_DOUBLE_EQ(backend.applied_voltage(master), half);
      EXPECT_DOUBLE_EQ(backend.applied_voltage(slave), oppose ? -half : half);
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

// The RoboRIO reports a CANcoder's absolute position in [-0.5, 0.5)
// rotations (CANcoderAbsoluteRot); the ideal backend's CANcoder, which
// mirrors its motor's position, wraps the same way, so an absolute-homing
// node under sim_gateway sees the reading the robot gives.
// REQ-DEV-05
TEST(SimBackend, CancoderReadsItsAbsolutePositionWrapped) {
  auto cfg = SingleMotorConfig();
  cfg.sensors.push_back({2, "angle", SensorKind::kCANcoder, 2, "rio"});
  cfg.motors[0].feedback = Feedback::kRemoteCANcoder;
  cfg.motors[0].feedback_sensor_id = 2;
  SimBackend backend;
  ASSERT_TRUE(backend.Configure(cfg));
  for (const auto& [demand, reported] :
       {std::pair{0.25, 0.25}, std::pair{0.75, -0.25}, std::pair{0.5, -0.5},
        std::pair{-0.75, 0.25}, std::pair{2.25, 0.25}}) {
    SCOPED_TRACE(demand);
    std::array<MotorRequest, 1> command{{{1, Mode::kPosition, 0, demand, 0}}};
    ASSERT_TRUE(backend.Apply(command));
    State s{};
    ASSERT_TRUE(backend.Read(s));
    EXPECT_DOUBLE_EQ(s.motors[0].position_rot, demand);  // continuous
    EXPECT_DOUBLE_EQ(s.sensors[0].position_rot, reported);
  }
}

// What each simulated backend's own Read reports for the chain a restore
// reapplies, before the gateway's mask: a motor fed by the reset CANcoder and
// its follower read invalid, zero in every measured field, from
// BeginRestoreAfterReset until a PollRestore that succeeds (through failed
// ones), while an unrelated motor and the CANcoder itself stay valid. The
// gateway tests cannot see this, since the gateway masks the chain anyway.
template <typename Sim>
void CheckRestoringChainReadsInvalid() {
  auto cfg = SingleMotorConfig();
  cfg.sensors.push_back({10, "angle", SensorKind::kCANcoder, 10, "rio"});
  cfg.motors[0].feedback = Feedback::kRemoteCANcoder;
  cfg.motors[0].feedback_sensor_id = 10;
  cfg.motors.push_back(TestMotor(2, "follower", 2));
  cfg.motors[1].master_id = 1;
  cfg.motors.push_back(TestMotor(3, "unrelated", 3));
  Sim backend;
  ASSERT_TRUE(backend.Configure(cfg));
  std::array<MotorRequest, 3> command{{{1, Mode::kVoltage, 0, 6.0, 0.0},
                                       {2, Mode::kVoltage, 0, 6.0, 0.0},
                                       {3, Mode::kVoltage, 0, 6.0, 0.0}}};
  ASSERT_TRUE(backend.Apply(command));
  State s;
  s.motor_count = 3;
  s.sensor_count = 1;
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 4);
  for (std::size_t i = 0; i < 3; ++i) ASSERT_TRUE(s.motors[i].valid) << i;
  const auto expect_chain_invalid = [&s](const char* phase) {
    for (std::size_t i = 0; i < 2; ++i) {
      SCOPED_TRACE(testing::Message() << phase << ", motor " << i);
      EXPECT_FALSE(s.motors[i].valid);
      EXPECT_EQ(s.motors[i].position_rot, 0.0);
      EXPECT_EQ(s.motors[i].velocity_rps, 0.0);
      EXPECT_EQ(s.motors[i].voltage, 0.0);
      EXPECT_EQ(s.motors[i].stator_current_a, 0.0);
      EXPECT_EQ(s.motors[i].supply_current_a, 0.0);
    }
    EXPECT_TRUE(s.motors[2].valid) << phase;
    EXPECT_TRUE(s.sensors[0].valid) << phase;
  };
  const std::array<uint16_t, 1> reset{10};
  backend.BeginRestoreAfterReset(reset);
  StepBackend(backend, s, now_us, 5000, 1);
  expect_chain_invalid("restoring");
  backend.SetRestoreFails(true);
  EXPECT_EQ(backend.PollRestore(), Backend::RestoreStatus::kFailed);
  StepBackend(backend, s, now_us, 5000, 1);
  expect_chain_invalid("after a failed restore");
  backend.SetRestoreFails(false);
  EXPECT_EQ(backend.PollRestore(), Backend::RestoreStatus::kDone);
  StepBackend(backend, s, now_us, 5000, 1);
  for (std::size_t i = 0; i < 3; ++i) EXPECT_TRUE(s.motors[i].valid) << i;
}

// REQ-DEV-24
TEST(SimBackend, ReadReportsTheRestoringChainInvalid) {
  CheckRestoringChainReadsInvalid<SimBackend>();
}

// REQ-DEV-24
TEST(PhysicsSimBackend, ReadReportsTheRestoringChainInvalid) {
  CheckRestoringChainReadsInvalid<PhysicsSimBackend>();
}

template <typename Sim>
void CheckPowerHealth() {
  Sim backend;
  Gateway gateway{SingleMotorConfig(), backend, 1};
  gateway.Tick(1000, false);
  // One bus, "rio", simulated: its slot in every State.
  const CanBusSample simulated{.read_state = CanBusReadState::kSimulated};
  EXPECT_EQ(gateway.snapshot().health,
            (HealthSample{true, kSimRestingSupplyV, false, 1, 0, simulated}));
  backend.SetPowerHealth(6.5, true);
  gateway.Tick(2000, true);
  EXPECT_EQ(gateway.snapshot().health,
            (HealthSample{true, 6.5, true, 1, 0, simulated}));
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

// Stator current is the winding model, (applied volts - back-EMF) / R with
// R = 12 V / the motor type's stall current, with LimitWindingVoltage enforcing
// stator_limit_a within the voltage ceiling: a stalled rotor draws
// stall_current / 12 per volt (a 1 V homing push into a stop reads loaded), one
// at its free speed for the applied volts draws none, and 12 V at half the 12 V
// free speed draws half the stall current.
TEST(SimMotorDiagnostics, StatorCurrentIsTheWindingModel) {
  struct Case {
    MotorType type;
    double stall_a;
  };
  for (const Case c :
       {Case{MotorType::kKrakenX60, 366.0}, Case{MotorType::kKrakenX44, 275.0},
        Case{MotorType::kFalcon500, 257.0}}) {
    SCOPED_TRACE(c.stall_a);
    auto cfg = SingleMotorConfig();
    cfg.motors[0].stator_limit_a = 1000.0;  // above stall: unclamped
    cfg.motors[0].supply_limit_a = 1000.0;
    cfg.motors[0].motor_type.value = c.type;
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(cfg));
    std::array<MotorRequest, 1> command{{{1, Mode::kVoltage, 0, 1.0, 0.0}}};
    ASSERT_TRUE(backend.Apply(command));
    State s{};
    s.motor_count = 1;
    uint64_t now_us = 0;
    StepBackend(backend, s, now_us, 5000, 1);  // the first Read: at rest
    EXPECT_NEAR(s.motors[0].stator_current_a, c.stall_a / 12.0, 1e-9);
    command[0].demand = 6.0;
    ASSERT_TRUE(backend.Apply(command));
    StepBackend(backend, s, now_us, 5000, 400);  // 2 s: free speed for 6 V
    EXPECT_NEAR(s.motors[0].stator_current_a, 0.0, 0.01 * c.stall_a);
    command[0].demand = 12.0;
    ASSERT_TRUE(backend.Apply(command));
    StepBackend(backend, s, now_us, 1, 1);  // 1 us later: still that speed
    EXPECT_NEAR(s.motors[0].stator_current_a, c.stall_a / 2.0,
                0.01 * c.stall_a);
  }
  auto cfg = SingleMotorConfig();
  cfg.motors[0].stator_limit_a = 80.0;
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(cfg));
  std::array<MotorRequest, 1> command{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
  ASSERT_TRUE(backend.Apply(command));
  State s{};
  s.motor_count = 1;
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 1);
  EXPECT_NEAR(s.motors[0].stator_current_a, 80.0, 1e-9);
}

// A motor driven into its soft limit is held there with its rotor stopped,
// so it reads the stall current of its applied volts, as a mechanism pushed
// into a stop does: stall homing (PositionMechanism) relies on it. The
// current is taken after the limit stops the rotor, not before.
// REQ-DEV-10
TEST(SimMotorDiagnostics, MotorHeldAtASoftLimitReadsStalled) {
  auto cfg = SingleMotorConfig();
  cfg.motors[0].soft_limits = true;
  cfg.motors[0].reverse_limit_rot = -1.0;
  cfg.motors[0].forward_limit_rot = 1.0;
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(cfg));
  std::array<MotorRequest, 1> command{{{1, Mode::kVoltage, 0, 1.0, 0.0}}};
  ASSERT_TRUE(backend.Apply(command));
  State s{};
  s.motor_count = 1;
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 400);  // 2 s at ~8 rps: at the limit
  ASSERT_DOUBLE_EQ(s.motors[0].position_rot, 1.0);
  EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
  EXPECT_NEAR(s.motors[0].stator_current_a, 366.0 / 12.0, 1e-9);  // X60
}

// A torque-current demand passes that stator current through the winding at
// the rotor's speed, in the motor's configured direction, so a free rotor
// accelerates under it and levels at the speed of max_voltage, as
// TorqueCurrentFOC's duty cap (max_voltage over the battery) holds it on the
// robot; the current never exceeds max_torque_current_a. 0 A is zero torque,
// powered: the rotor spins on rather than braking as neutral does.
// REQ-SAFE-43
TEST(SimMotorDiagnostics, TorqueCurrentAcceleratesToTheVoltageCeiling) {
  for (const bool inverted : {false, true}) {
    SCOPED_TRACE(inverted);
    auto cfg = SingleMotorConfig();  // Kraken X60, brake
    cfg.motors[0].inverted = inverted;
    cfg.motors[0].foc = true;
    cfg.motors[0].max_torque_current_a = 5.0;
    cfg.motors[0].max_voltage = 4.0;
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(cfg));
    std::array<MotorRequest, 1> command{
        {{1, Mode::kTorqueCurrent, 0, 8.0, 0.0}}};
    ASSERT_TRUE(backend.Apply(command));
    State s{};
    s.motor_count = 1;
    uint64_t now_us = 0;
    StepBackend(backend, s, now_us, 5000, 1);  // At rest: clamped to 5 A.
    EXPECT_NEAR(s.motors[0].stator_current_a, 5.0, 1e-9);
    command[0].demand = 2.0;
    ASSERT_TRUE(backend.Apply(command));
    StepBackend(backend, s, now_us, 5000, 200);  // 1 s: still accelerating
    EXPECT_GT(s.motors[0].velocity_rps, 1.0);
    EXPECT_LT(s.motors[0].velocity_rps, 30.0);
    EXPECT_GT(s.motors[0].stator_current_a, 1.5);
    EXPECT_LE(s.motors[0].stator_current_a, 2.0 + 1e-9);
    StepBackend(backend, s, now_us, 5000, 800);  // 5 s: at the 4 V ceiling
    const double ceiling_rps = 4.0 / 12.0 * 100.0;
    const double vel_at_ceiling = s.motors[0].velocity_rps;
    EXPECT_NEAR(vel_at_ceiling, ceiling_rps, 0.01 * ceiling_rps);
    EXPECT_NEAR(s.motors[0].voltage, 4.0, 1e-9);
    EXPECT_LE(s.motors[0].stator_current_a, 2.0 + 1e-9);
    command[0].demand = 0.0;
    ASSERT_TRUE(backend.Apply(command));
    StepBackend(backend, s, now_us, 5000, 200);  // 1 s of zero stator current
    // At 0 A electromagnetic torque, only no-load drag slows the rotor slightly
    // (rather than braking to 0 as Neutral() does on a brake-mode motor).
    EXPECT_GT(s.motors[0].velocity_rps, 0.85 * vel_at_ceiling);
    EXPECT_LT(s.motors[0].velocity_rps, vel_at_ceiling);
    EXPECT_NEAR(s.motors[0].stator_current_a, 0.0, 1e-9);
  }
}

// The wire config carries no motor type, so a config the bridge pushes to
// sim_gateway that differs from the one it holds reconfigures the physics
// backend with every motor a Kraken X60. Each motor keeps the type of the
// motor of the same name in sim_gateway's own parse: after such a push, a
// Kraken X44 still stalls at the X44's current.
// REQ-DEV-10
TEST(SimMotorDiagnostics, PushedConfigKeepsTheLocalMotorType) {
  auto local = SingleMotorConfig();  // 80 A stator limit: 1 V stays under it
  local.motors[0].motor_type.value = MotorType::kKrakenX44;
  PhysicsSimBackend backend;
  backend.SetLocalMotorTypes(local);  // as sim_gateway does from its parse
  Gateway gateway{backend, 1};
  ASSERT_TRUE(gateway.Reconfigure(local));
  auto pushed = local;
  pushed.motors[0].max_voltage = 11.0;  // differs: Configure runs again
  Config wire;
  ASSERT_TRUE(DeserializeConfig(SerializeConfig(pushed), wire));
  ASSERT_EQ(wire.motors[0].motor_type.value, MotorType::kKrakenX60);
  ASSERT_TRUE(gateway.Reconfigure(wire));
  std::array<MotorRequest, 1> command{{{1, Mode::kVoltage, 0, 1.0, 0.0}}};
  ASSERT_TRUE(backend.Apply(command));
  State s{};
  s.motor_count = 1;
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 1);  // the first Read: at rest
  EXPECT_NEAR(s.motors[0].stator_current_a, 275.0 / 12.0, 1e-9);
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
  EXPECT_NEAR(s.motors[0].voltage, 0.5 * kSimRestingSupplyV, 1e-9);
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
      // The CANcoder's absolute position, wrapped; the motor's continuous.
      EXPECT_NEAR(
          std::remainder(s.motors[0].position_rot - s.sensors[0].position_rot,
                         1.0),
          0.0, 1e-9);
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
// The asserted switch is reported as the FaultField's ForwardHardLimit bit.
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
      EXPECT_EQ(s.motors[0].fault_flags, 0u);
      continue;
    }
    EXPECT_DOUBLE_EQ(backend.applied_voltage(0), 0.0);
    EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
    EXPECT_EQ(s.motors[0].fault_flags, kMotorForwardHardLimitFlag);
    EXPECT_FALSE(MotorFaulted(s.motors[0].fault_flags));
    cmd[0] = Duty(1, -0.5);
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, s, now_us, 5000, 100);
    EXPECT_LT(backend.applied_voltage(0), -5.0);
    EXPECT_GT(std::abs(s.motors[0].velocity_rps), 10.0);
    backend.SetForwardLimitPressed(1, false);
    StepBackend(backend, s, now_us, 5000, 1);
    EXPECT_EQ(s.motors[0].fault_flags, 0u);
  }
}

// A home switch reads closed exactly while the motor's reported position is
// at its point or beyond it, with no hysteresis either way: "bottom"
// (normally open, closed at or below -0.3) and "top" (closed reads true,
// closed at or above 0.25) as the mechanism sweeps down into its -0.3 soft
// limit and back up. At the limit the position is exactly -0.3: "bottom" is
// closed there, and "below" (closed one ulp further down) is open. The
// binding survives a reconfigure, as a config the bridge pushes to
// sim_gateway.
// REQ-DEV-10
TEST(SimLimitSwitches, HomeSwitchClosesAtItsPositionExactly) {
  Config c = SingleMotorConfig();
  c.motors[0].soft_limits = true;
  c.motors[0].reverse_limit_rot = -0.3;
  c.motors[0].forward_limit_rot = 0.3;
  for (const auto& [id, name] : {std::pair<uint16_t, const char*>{1, "bottom"},
                                 {2, "top"},
                                 {3, "below"}}) {
    c.digital_inputs.push_back({.id = id, .name = name, .dio = id});
  }
  const double below = std::nextafter(-0.3, -1.0);
  PhysicsSimBackend backend;
  backend.SetHomeSwitches(
      {{.input = "bottom", .motor = "roller", .closed_at_rot = -0.3},
       {.input = "top",
        .motor = "roller",
        .closed_at_rot = 0.25,
        .closed_above = true,
        .closed_reads = true},
       {.input = "below", .motor = "roller", .closed_at_rot = below}});
  ASSERT_TRUE(backend.Configure(c));
  State s{};
  s.motor_count = 1;
  uint64_t now_us = 0;
  int bottom_closed = 0;
  int top_closed = 0;
  const auto sweep = [&](double volts) {
    std::array<MotorRequest, 1> command{{{1, Mode::kVoltage, 0, volts, 0.0}}};
    ASSERT_TRUE(backend.Apply(command));
    for (int i = 0; i < 100; ++i) {  // 0.5 s at about 8 rps: limit to limit
      StepBackend(backend, s, now_us, 5000, 1);
      const double p = s.motors[0].position_rot;
      EXPECT_EQ(s.digital_inputs[0].value, !(p <= -0.3)) << p;
      EXPECT_EQ(s.digital_inputs[1].value, p >= 0.25) << p;
      EXPECT_EQ(s.digital_inputs[2].value, !(p <= below)) << p;
      bottom_closed += p <= -0.3 ? 1 : 0;
      top_closed += p >= 0.25 ? 1 : 0;
    }
  };
  sweep(-1.0);
  ASSERT_EQ(s.motors[0].position_rot, -0.3);
  EXPECT_FALSE(s.digital_inputs[0].value);  // closed at its point
  EXPECT_TRUE(s.digital_inputs[2].value);   // open an ulp short of its point
  sweep(1.0);
  EXPECT_EQ(s.motors[0].position_rot, 0.3);
  EXPECT_GT(bottom_closed, 0);
  EXPECT_GT(top_closed, 0);

  // Reconfigured, the axis restarts at 0, between the switches: open.
  c.motors[0].max_voltage = 11.0;
  ASSERT_TRUE(backend.Configure(c));
  StepBackend(backend, s, now_us, 5000, 1);
  EXPECT_TRUE(s.digital_inputs[0].value);
  EXPECT_FALSE(s.digital_inputs[1].value);
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
  ASSERT_EQ(backend.PollRestore(), Backend::RestoreStatus::kDone);
  ASSERT_TRUE(backend.Read(state));
  EXPECT_DOUBLE_EQ(state.motors[0].position_rot, position);
  EXPECT_DOUBLE_EQ(state.motors[0].velocity_rps, 0.0);
  EXPECT_DOUBLE_EQ(state.motors[0].voltage, 0.0);
  StepBackend(backend, state, now_us, 5000, 20);
  EXPECT_DOUBLE_EQ(state.motors[0].position_rot, position);
}

// CANcoders mirror the linked sensor shaft through the boundary, reporting its
// absolute position as the RoboRIO does (AbsoluteSensorDiscontinuityPoint =
// 0.5): wrapped to [-0.5, 0.5) rotations on the sensor shaft (mechanism *
// sensor_to_mechanism_ratio), while the remote-CANcoder motor's own mechanism
// position stays continuous.
TEST(SimBelievability, CancoderMirrorsLinkedMechanism) {
  constexpr double kSensorToMechanism = 2.0;
  Config c = SingleMotorConfig();
  c.sensors.push_back({2, "encoder", SensorKind::kCANcoder, 20, "rio"});
  c.motors[0].feedback = Feedback::kRemoteCANcoder;
  c.motors[0].feedback_sensor_id = 2;
  c.motors[0].sensor_to_mechanism_ratio = kSensorToMechanism;
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(c));
  std::array<MotorRequest, kMaxMotors> cmd{};
  cmd[0] = Duty(1, 0.5);
  ASSERT_TRUE(backend.Apply(cmd));
  State s{};
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 200);
  EXPECT_TRUE(s.sensors[0].valid);
  ASSERT_GT(s.motors[0].position_rot, 1.0);
  EXPECT_GE(s.sensors[0].position_rot, -0.5);
  EXPECT_LT(s.sensors[0].position_rot, 0.5);
  EXPECT_NEAR(std::remainder(s.sensors[0].position_rot -
                                 s.motors[0].position_rot * kSensorToMechanism,
                             1.0),
              0.0, 1e-9);
  EXPECT_NEAR(s.sensors[0].velocity_rps,
              s.motors[0].velocity_rps * kSensorToMechanism, 1e-9);
  // Backwards past -0.5 wraps the other way.
  cmd[0] = Duty(1, -0.5);
  ASSERT_TRUE(backend.Apply(cmd));
  StepBackend(backend, s, now_us, 5000, 400);
  ASSERT_LT(s.motors[0].position_rot, -1.0);
  EXPECT_GE(s.sensors[0].position_rot, -0.5);
  EXPECT_LT(s.sensors[0].position_rot, 0.5);
  EXPECT_NEAR(std::remainder(s.sensors[0].position_rot -
                                 s.motors[0].position_rot * kSensorToMechanism,
                             1.0),
              0.0, 1e-9);
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
    cmd[0] = Duty(1, 0.25);  // Gentle spin-up so the 80 A-limited stop fits.
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
  // Its age says so, so the gateway's reset check widens its bound by the
  // freeze instead of reading the unfreeze as a reset.
  EXPECT_EQ(s.motors[0].position_age_us, 500000u);
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

// REQ-CONTRACT-32, REQ-DEV-10
// A disconnected device reads as PhoenixBackend reports one whose frames
// stopped: invalid, its ages growing from its last sample, its limit bits
// held until the FaultField is kFaultFieldMaxAgeUs old (the simulator models
// no fault bits, only limit switches) and its temperature held until it is
// kTemperatureMaxAgeUs old, then 0. A node's age rule then sees in
// simulation what it would see on the robot.
TEST(SimBelievability, ADisconnectedDeviceAgesAsPhoenixReportsIt) {
  Config c = SingleMotorConfig();
  c.sensors.push_back({9, "imu", SensorKind::kPigeon2, 30, "rio"});
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(c));
  backend.SetForwardLimitPressed(1, true);
  State s{};
  constexpr uint64_t kLastReadUs = 5000;
  s.sample_time_us = kLastReadUs;
  ASSERT_TRUE(backend.Read(s));
  ASSERT_EQ(s.motors[0].fault_flags, kMotorForwardHardLimitFlag);
  ASSERT_DOUBLE_EQ(s.motors[0].temperature_c, 25.0);
  backend.SetMotorConnected(1, false);
  backend.SetSensorConnected(9, false);
  const auto read_at = [&](uint64_t age_us) {
    s.sample_time_us = kLastReadUs + age_us;
    backend.Read(s);
    EXPECT_FALSE(s.motors[0].valid);
    EXPECT_EQ(s.motors[0].position_age_us, age_us);
    EXPECT_EQ(s.motors[0].velocity_age_us, age_us);
    EXPECT_FALSE(s.sensors[0].valid);
    EXPECT_EQ(s.sensors[0].position_age_us, age_us);
    EXPECT_EQ(s.sensors[0].velocity_age_us, age_us);
  };
  read_at(kFaultFieldMaxAgeUs - 1);
  EXPECT_EQ(s.motors[0].fault_flags, kMotorForwardHardLimitFlag);
  read_at(kFaultFieldMaxAgeUs);
  EXPECT_EQ(s.motors[0].fault_flags, 0u);
  read_at(kTemperatureMaxAgeUs - 1);
  EXPECT_DOUBLE_EQ(s.motors[0].temperature_c, 25.0);
  read_at(kTemperatureMaxAgeUs);
  EXPECT_DOUBLE_EQ(s.motors[0].temperature_c, 0.0);
  // Reconnected, it reads current again.
  backend.SetMotorConnected(1, true);
  backend.SetSensorConnected(9, true);
  s.sample_time_us += 5000;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_EQ(s.motors[0].position_age_us, 0u);
  EXPECT_EQ(s.sensors[0].position_age_us, 0u);
  EXPECT_EQ(s.motors[0].fault_flags, kMotorForwardHardLimitFlag);
  EXPECT_DOUBLE_EQ(s.motors[0].temperature_c, 25.0);
}

// REQ-CONTRACT-32, REQ-DEV-10
// Frozen feedback is a device whose frames stopped while its sample stays
// valid: its held limit bits clear and its held temperature reads 0 by the
// same ages as a disconnected one's.
TEST(SimBelievability, FrozenFeedbackAgesItsTemperatureAndLimitBits) {
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(SingleMotorConfig()));
  backend.SetForwardLimitPressed(1, true);
  State s{};
  constexpr uint64_t kLastReadUs = 5000;
  s.sample_time_us = kLastReadUs;
  ASSERT_TRUE(backend.Read(s));
  backend.FreezeFeedback(1);
  s.sample_time_us = kLastReadUs + kFaultFieldMaxAgeUs - 1;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_EQ(s.motors[0].fault_flags, kMotorForwardHardLimitFlag);
  s.sample_time_us = kLastReadUs + kFaultFieldMaxAgeUs;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_EQ(s.motors[0].fault_flags, 0u);
  s.sample_time_us = kLastReadUs + kTemperatureMaxAgeUs - 1;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_DOUBLE_EQ(s.motors[0].temperature_c, 25.0);
  s.sample_time_us = kLastReadUs + kTemperatureMaxAgeUs;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.motors[0].valid);
  EXPECT_EQ(s.motors[0].position_age_us, kTemperatureMaxAgeUs);
  EXPECT_DOUBLE_EQ(s.motors[0].temperature_c, 0.0);
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
      // Duty scales the resting supply; free speed is 100 rps at 12 V.
      commands[2 * i] = Duty(2 * i + 1, speed / (100.0 * kTau * kRadius) *
                                            12.0 / kSimRestingSupplyV);
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

// REQ-DEV-05, REQ-DEV-23: physics uses configured wheel_radius_m and explicit
// swerve module coordinates instead of hard-coded 0.0508 m / ±0.30 m, overrides
// inferred conventional-name placement without duplicating modules, and keeps
// the explicit geometry across a later Configure() (pushed config).
TEST(SimWorld, SecondGeometryControlsLinearAndYawMotionAndSurvivesReconfigure) {
  static constexpr double kTau = WorldFixture::kTau;
  static constexpr double kWheelRadius = 0.06;
  const std::array<const char*, 4> names{"front_left", "front_right",
                                         "back_left", "back_right"};
  const std::array<double, 4> xs{0.40, 0.40, -0.40, -0.40};
  const std::array<double, 4> ys{0.25, -0.25, 0.25, -0.25};

  Config config;
  std::vector<sim::SwerveModuleSpec> specs;
  for (std::size_t i = 0; i < 4; ++i) {
    auto drive = TestMotor(2 * i + 1, "drive", static_cast<int>(2 * i + 1));
    drive.name = std::string(names[i]) + "_drive";
    auto steer = TestMotor(2 * i + 2, "steer", static_cast<int>(2 * i + 2));
    steer.name = std::string(names[i]) + "_steer";
    steer.slots[0].p = 1.0;
    config.motors.push_back(drive);
    config.motors.push_back(steer);
    specs.push_back({drive.name.str(), steer.name.str(), xs[i], ys[i]});
  }
  config.sensors.push_back({20, "imu", SensorKind::kPigeon2, 30, "rio"});

  PhysicsSimBackend backend{{.wheel_radius_m = kWheelRadius}};
  backend.SetSwerveModules(specs);
  ASSERT_TRUE(backend.Configure(config));
  // Simulate a later config pushed over the wire: explicit module geometry
  // must survive without falling back to ±0.30 m inference.
  ASSERT_TRUE(backend.Configure(config));

  State state{};
  uint64_t now_us = 0;
  std::array<MotorRequest, kMaxMotors> commands{};
  auto drive_chassis = [&](double vx, double vy, double omega) {
    for (std::size_t i = 0; i < 4; ++i) {
      const double wheel_vx = vx - omega * ys[i];
      const double wheel_vy = vy + omega * xs[i];
      commands[2 * i] = MotorRequest{static_cast<uint16_t>(2 * i + 1),
                                     Mode::kNeutral, 0, 0.0, 0.0};
      commands[2 * i + 1] = {static_cast<uint16_t>(2 * i + 2), Mode::kPosition,
                             0, std::atan2(wheel_vy, wheel_vx) / kTau, 0.0};
    }
    ASSERT_TRUE(backend.Apply(commands));
    StepBackend(backend, state, now_us, 5000, 2000);
    for (std::size_t i = 0; i < 4; ++i) {
      const double speed = std::hypot(vx - omega * ys[i], vy + omega * xs[i]);
      commands[2 * i] = Duty(2 * i + 1, speed / (100.0 * kTau * kWheelRadius) *
                                            12.0 / kSimRestingSupplyV);
    }
    ASSERT_TRUE(backend.Apply(commands));
    StepBackend(backend, state, now_us, 5000, 400);
  };

  // Pure linear translation: ground speed scales with 0.06 m, not 0.0508 m.
  drive_chassis(1.2, 0.0, 0.0);
  const auto before_linear = backend.truth();
  StepBackend(backend, state, now_us, 5000, 200);
  const auto after_linear = backend.truth();
  EXPECT_NEAR(after_linear.x_m - before_linear.x_m, 1.2, 1e-8);
  EXPECT_NEAR(after_linear.y_m - before_linear.y_m, 0.0, 1e-8);
  EXPECT_NEAR(after_linear.vx_mps, 1.2, 1e-8);
  EXPECT_NEAR(after_linear.omega_radps, 0.0, 1e-8);
  EXPECT_NEAR(state.motors[0].velocity_rps * kTau * kWheelRadius, 1.2, 1e-8);

  // Pure yaw rotation: angular velocity matches (±0.40, ±0.25) and 0.06 m,
  // not inferred ±0.30 m or duplicated modules.
  drive_chassis(0.0, 0.0, 0.9);
  const auto before_rot = backend.truth();
  StepBackend(backend, state, now_us, 5000, 200);
  const auto after_rot = backend.truth();
  EXPECT_NEAR(after_rot.yaw_rad - before_rot.yaw_rad, 0.9, 1e-8);
  EXPECT_NEAR(after_rot.omega_radps, 0.9, 1e-8);
  EXPECT_NEAR(state.sensors[0].velocity_rps, 0.9 / kTau, 1e-8);
  EXPECT_NEAR(after_rot.x_m - before_rot.x_m, 0.0, 1e-8);
  EXPECT_NEAR(after_rot.y_m - before_rot.y_m, 0.0, 1e-8);

  // Non-positive or non-finite wheel_radius_m falls back to 0.0508 m.
  for (const double bad_radius :
       {0.0, -0.06, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()}) {
    WorldFixture fallback{{.wheel_radius_m = bad_radius}};
    fallback.Drive(1.0, 0.0, 0.0);
    fallback.Advance(400);
    EXPECT_NEAR(fallback.backend.truth().vx_mps, 1.0, 1e-8);
  }
}

// REQ-DEV-10: At 12 V, each motor type draws its manufacturer stall current at
// rest and its free current at free speed, and supply current obeys power
// conservation |applied_volts * stator_current| / supply_v.
TEST(SimMotorDiagnostics, FreeCurrentAndSupplyCurrentMatchMotorSpecs) {
  struct Spec {
    MotorType type;
    double free_rps;
    double stall_a;
    double free_a;
  };
  for (const Spec s :
       {Spec{MotorType::kKrakenX60, 6000.0 / 60.0, 366.0, 2.0},
        Spec{MotorType::kKrakenX44, 7530.0 / 60.0, 275.0, 2.0},
        Spec{MotorType::kFalcon500, 6380.0 / 60.0, 257.0, 1.5}}) {
    SCOPED_TRACE(s.stall_a);
    auto cfg = SingleMotorConfig();
    cfg.motors[0].motor_type.value = s.type;
    cfg.motors[0].stator_limit_a = 1000.0;
    cfg.motors[0].supply_limit_a = 1000.0;
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(cfg));
    std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
    ASSERT_TRUE(backend.Apply(cmd));
    State st{};
    st.motor_count = 1;
    uint64_t now_us = 0;
    StepBackend(backend, st, now_us, 5000, 1);  // At rest: 12 V stall.
    EXPECT_NEAR(st.motors[0].stator_current_a, s.stall_a, 1e-9);
    EXPECT_NEAR(st.motors[0].supply_current_a,
                12.0 * s.stall_a / kSimRestingSupplyV, 1e-9);
    StepBackend(backend, st, now_us, 5000, 400);  // 2 s: 12 V free speed.
    EXPECT_NEAR(st.motors[0].velocity_rps, s.free_rps, 1e-4);
    EXPECT_NEAR(st.motors[0].stator_current_a, s.free_a, 1e-3);
    EXPECT_NEAR(st.motors[0].supply_current_a,
                12.0 * s.free_a / kSimRestingSupplyV, 1e-3);
  }
}

// REQ-DEV-10: stator_limit_a feeds back into rotor dynamics by clamping
// effective winding voltage around back-EMF before StepAxis, slowing spin-up
// and capping applied voltage and reported stator current.
TEST(SimMotorDiagnostics, StatorLimitClampsAppliedVoltageAndSlowsSpinUp) {
  auto make_cfg = [](double stator_limit_a) {
    auto cfg = SingleMotorConfig();
    cfg.motors[0].stator_limit_a = stator_limit_a;
    cfg.motors[0].supply_limit_a = 400.0;
    return cfg;
  };
  PhysicsSimBackend tight, loose;
  ASSERT_TRUE(tight.Configure(make_cfg(20.0)));
  ASSERT_TRUE(loose.Configure(make_cfg(200.0)));
  std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
  ASSERT_TRUE(tight.Apply(cmd));
  ASSERT_TRUE(loose.Apply(cmd));
  State st{}, sl{};
  st.motor_count = sl.motor_count = 1;
  uint64_t now_t = 0;
  uint64_t now_l = 0;
  const double r_ohm = 12.0 / 366.0;
  StepBackend(tight, st, now_t, 5000, 1);  // dt = 0, at rest.
  StepBackend(loose, sl, now_l, 5000, 1);
  EXPECT_NEAR(st.motors[0].voltage, 20.0 * r_ohm, 1e-9);
  EXPECT_NEAR(st.motors[0].stator_current_a, 20.0, 1e-9);
  EXPECT_NEAR(sl.motors[0].voltage, 200.0 * r_ohm, 1e-9);
  StepBackend(tight, st, now_t, 5000, 10);  // 50 ms of acceleration.
  StepBackend(loose, sl, now_l, 5000, 10);
  EXPECT_LT(st.motors[0].velocity_rps, sl.motors[0].velocity_rps * 0.25);
  EXPECT_LE(st.motors[0].stator_current_a, 20.0 + 1e-9);
  EXPECT_LT(st.motors[0].voltage, 12.0);
}

// REQ-DEV-10: supply_limit_a caps power drawn from the battery consistently
// with applied voltage and stator current, and supply current drops to 0 when
// unpowered or when supply_v <= 0.
TEST(SimMotorDiagnostics, SupplyLimitCapsPowerAndDropsToZeroWhenUnpowered) {
  auto cfg = SingleMotorConfig();
  cfg.motors[0].stator_limit_a = 1000.0;
  cfg.motors[0].supply_limit_a = 15.0;
  PhysicsSimBackend limited;
  ASSERT_TRUE(limited.Configure(cfg));
  auto unlimited_cfg = cfg;
  unlimited_cfg.motors[0].supply_limit_a = 400.0;
  PhysicsSimBackend unlimited;
  ASSERT_TRUE(unlimited.Configure(unlimited_cfg));
  std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
  ASSERT_TRUE(limited.Apply(cmd));
  ASSERT_TRUE(unlimited.Apply(cmd));
  State sl{}, su{};
  sl.motor_count = su.motor_count = 1;
  uint64_t now_l = 0;
  uint64_t now_u = 0;
  StepBackend(limited, sl, now_l, 5000, 1);
  StepBackend(unlimited, su, now_u, 5000, 1);
  EXPECT_NEAR(sl.motors[0].supply_current_a, 15.0, 1e-9);
  EXPECT_NEAR(std::abs(sl.motors[0].voltage * sl.motors[0].stator_current_a) /
                  kSimRestingSupplyV,
              15.0, 1e-9);
  EXPECT_LT(sl.motors[0].voltage, 12.0);
  StepBackend(limited, sl, now_l, 5000, 10);
  StepBackend(unlimited, su, now_u, 5000, 10);
  EXPECT_LE(sl.motors[0].supply_current_a, 15.0 + 1e-9);
  EXPECT_LT(sl.motors[0].velocity_rps, su.motors[0].velocity_rps);

  limited.SetSupplyVoltage(0.0);
  StepBackend(limited, sl, now_l, 5000, 1);
  EXPECT_DOUBLE_EQ(sl.motors[0].supply_current_a, 0.0);

  limited.SetSupplyVoltage(kSimRestingSupplyV);
  limited.Neutral();
  StepBackend(limited, sl, now_l, 5000, 1);
  EXPECT_DOUBLE_EQ(sl.motors[0].supply_current_a, 0.0);
  EXPECT_DOUBLE_EQ(sl.motors[0].stator_current_a, 0.0);
}

// REQ-DEV-10: End-of-travel hard stops clamp position, zero velocity when
// driven into the stop (raising stator current to stall), allow motion away
// from the stop, work even when soft_limits = false, and survive Configure().
TEST(SimMotorDiagnostics,
     HardStopsStallWithoutSoftLimitsAndSurviveReconfigure) {
  for (const bool inverted : {false, true}) {
    SCOPED_TRACE(inverted);
    auto cfg = SingleMotorConfig();
    cfg.motors[0].soft_limits = false;
    cfg.motors[0].inverted = inverted;
    cfg.motors[0].sensor_to_mechanism_ratio = 5.0;
    PhysicsSimBackend backend;
    backend.SetHardStops(
        {{.motor = "roller", .min_rot = -0.3, .max_rot = 0.5}});
    ASSERT_TRUE(backend.Configure(cfg));
    // Reconfigure (as a config pushed by the bridge): hard stops survive.
    cfg.motors[0].max_voltage = 11.0;
    ASSERT_TRUE(backend.Configure(cfg));

    State s{};
    s.motor_count = 1;
    uint64_t now_us = 0;
    std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, -1.5, 0.0}}};
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, s, now_us, 5000, 200);  // 1 s into min_rot stop.
    EXPECT_DOUBLE_EQ(s.motors[0].position_rot, -0.3);
    EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
    EXPECT_NEAR(s.motors[0].stator_current_a, 1.5 * 366.0 / 12.0, 1e-9);

    // Reversing drives freely off min_rot and stalls at max_rot.
    cmd[0].demand = 1.5;
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, s, now_us, 5000, 5);
    EXPECT_GT(s.motors[0].position_rot, -0.3);
    EXPECT_GT(s.motors[0].velocity_rps, 0.0);
    StepBackend(backend, s, now_us, 5000, 200);
    EXPECT_DOUBLE_EQ(s.motors[0].position_rot, 0.5);
    EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
    EXPECT_NEAR(s.motors[0].stator_current_a, 1.5 * 366.0 / 12.0, 1e-9);
  }

  // One-sided hard stop (min_rot only): clamps below, free above.
  auto cfg = SingleMotorConfig();
  cfg.motors[0].soft_limits = false;
  PhysicsSimBackend backend;
  backend.SetHardStops({{.motor = "roller", .min_rot = -0.2}});
  ASSERT_TRUE(backend.Configure(cfg));
  State s{};
  s.motor_count = 1;
  uint64_t now_us = 0;
  std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 2.0, 0.0}}};
  ASSERT_TRUE(backend.Apply(cmd));
  StepBackend(backend, s, now_us, 5000, 100);
  EXPECT_GT(s.motors[0].position_rot, 5.0);
}

// REQ-DEV-10: SupplyCurrent is signed (positive when drawing from the battery,
// negative during regenerative braking when applied voltage lies between 0 and
// back-EMF), caps only positive draw at supply_limit_a (leaving regen signed
// and unclamped), and StatorCurrent reports the true winding current from
// applied_volts_ even when brownout lowers the voltage ceiling and forces
// current above stator_limit_a.
TEST(SimMotorDiagnostics,
     RegenSupplyCurrentIsSignedAndBrownoutStatorCurrentIsNotHidden) {
  for (const bool inverted : {false, true}) {
    SCOPED_TRACE(inverted);
    auto cfg = SingleMotorConfig();
    cfg.motors[0].inverted = inverted;
    cfg.motors[0].stator_limit_a = 40.0;
    cfg.motors[0].supply_limit_a = 15.0;
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(cfg));
    State s{};
    s.motor_count = 1;
    uint64_t now_us = 0;

    // Motoring from rest: positive battery draw capped at supply_limit_a.
    std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, s, now_us, 5000, 1);
    EXPECT_GT(s.motors[0].supply_current_a, 0.0);
    EXPECT_LE(s.motors[0].supply_current_a, 15.0 + 1e-9);

    // Spin up to 12 V free speed (~100 rps, back-EMF ~11.93 V).
    StepBackend(backend, s, now_us, 5000, 400);
    ASSERT_GT(s.motors[0].velocity_rps, 95.0);
    EXPECT_GT(s.motors[0].supply_current_a, 0.0);

    // Regenerative braking: commanding 6 V below ~11.93 V back-EMF holds
    // applied voltage at back-EMF - 40 A * R (~10.62 V), returning ~33.7 A to
    // the battery (negative, and not clamped to -15 A by supply_limit_a).
    cmd[0].demand = 6.0;
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, s, now_us, 1, 1);  // 1 us later: still near 100 rps.
    EXPECT_NEAR(s.motors[0].stator_current_a, 40.0, 0.5);
    EXPECT_LT(s.motors[0].supply_current_a, -25.0);

    // Brownout while still near 100 rps: voltage ceiling drops to 6.0 V, which
    // is ~5.93 V below back-EMF (~181 A). StatorCurrent must report the actual
    // winding current rather than hiding it behind a second stator_limit_a cap.
    backend.SetSupplyVoltage(6.0);
    StepBackend(backend, s, now_us, 1, 1);
    EXPECT_GT(s.motors[0].stator_current_a, 100.0);
    EXPECT_LT(s.motors[0].supply_current_a, -100.0);
  }
}

// REQ-DEV-10: A hard stop on a master motor clamps every follower in its chain
// (aligned or opposed, regardless of master inversion or follower gear ratio),
// so followers stall with their master instead of spinning freely past the stop
// and reverse freely off it.
TEST(SimMotorDiagnostics, HardStopOnMasterClampsAndStallsFollowers) {
  for (const bool master_inverted : {false, true}) {
    for (const bool oppose : {false, true}) {
      for (const double follower_gear : {5.0, 1.0}) {
        SCOPED_TRACE(testing::Message() << "master_inverted=" << master_inverted
                                        << " oppose=" << oppose
                                        << " follower_gear=" << follower_gear);
        auto cfg = SingleMotorConfig();
        cfg.motors[0].soft_limits = false;
        cfg.motors[0].inverted = master_inverted;
        cfg.motors[0].sensor_to_mechanism_ratio = 5.0;
        auto follower = TestMotor(2, "follower", 2);
        follower.master_id = 1;
        follower.oppose_master = oppose;
        follower.soft_limits = false;
        follower.sensor_to_mechanism_ratio = follower_gear;
        cfg.motors.push_back(follower);

        PhysicsSimBackend backend;
        backend.SetHardStops(
            {{.motor = "roller", .min_rot = -0.3, .max_rot = 0.5}});
        ASSERT_TRUE(backend.Configure(cfg));

        State s{};
        s.motor_count = 2;
        uint64_t now_us = 0;
        const double sign = oppose ? -1.0 : 1.0;
        const double scale = 5.0 / follower_gear;
        std::array<MotorRequest, 2> cmd{
            {{1, Mode::kVoltage, 0, -1.5, 0.0}, Duty(2, 0.0)}};
        ASSERT_TRUE(backend.Apply(cmd));
        StepBackend(backend, s, now_us, 5000, 200);  // 1 s into min_rot stop.
        EXPECT_DOUBLE_EQ(s.motors[0].position_rot, -0.3);
        EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
        EXPECT_DOUBLE_EQ(s.motors[1].position_rot, -0.3 * sign * scale);
        EXPECT_DOUBLE_EQ(s.motors[1].velocity_rps, 0.0);
        EXPECT_NEAR(s.motors[1].stator_current_a, s.motors[0].stator_current_a,
                    1e-9);
        EXPECT_NEAR(s.motors[1].stator_current_a, 1.5 * 366.0 / 12.0, 1e-9);

        // Reversing drives both master and follower freely off min_rot to
        // max_rot.
        cmd[0].demand = 1.5;
        ASSERT_TRUE(backend.Apply(cmd));
        StepBackend(backend, s, now_us, 5000, 5);
        EXPECT_GT(s.motors[0].position_rot, -0.3);
        EXPECT_GT(s.motors[1].position_rot * sign, -0.3 * scale);
        EXPECT_GT(s.motors[1].velocity_rps * sign, 0.0);
        StepBackend(backend, s, now_us, 5000, 200);
        EXPECT_DOUBLE_EQ(s.motors[0].position_rot, 0.5);
        EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
        EXPECT_DOUBLE_EQ(s.motors[1].position_rot, 0.5 * sign * scale);
        EXPECT_DOUBLE_EQ(s.motors[1].velocity_rps, 0.0);
        EXPECT_NEAR(s.motors[1].stator_current_a, s.motors[0].stator_current_a,
                    1e-9);
      }
    }
  }
}

// REQ-DEV-10: Hard stops evaluate position and velocity in FeedbackDirection
// (the remote CANcoder's frame when configured), not the motor's output
// Direction. With motor inverted=false (Direction=+1) and remote CANcoder
// inverted=true (FeedbackDirection=-1), asymmetric stops [-0.3, 0.5] stop
// positive voltage at reported -0.3 and negative voltage at reported +0.5.
TEST(SimMotorDiagnostics, HardStopUsesRemoteCancoderFeedbackDirection) {
  auto cfg = SingleMotorConfig();
  cfg.motors[0].soft_limits = false;
  cfg.motors[0].inverted = false;
  cfg.motors[0].sensor_to_mechanism_ratio = 5.0;
  SensorConfig encoder{2, "encoder", SensorKind::kCANcoder, 20, "rio"};
  encoder.inverted = true;
  cfg.sensors.push_back(encoder);
  cfg.motors[0].feedback = Feedback::kRemoteCANcoder;
  cfg.motors[0].feedback_sensor_id = 2;

  PhysicsSimBackend backend;
  backend.SetHardStops({{.motor = "roller", .min_rot = -0.3, .max_rot = 0.5}});
  ASSERT_TRUE(backend.Configure(cfg));

  State s{};
  s.motor_count = 1;
  s.sensor_count = 1;
  uint64_t now_us = 0;

  // Positive motor voltage moves the inverted CANcoder reading negatively into
  // min_rot = -0.3 (not -0.5, which Direction would clamp it to).
  std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 1.5, 0.0}}};
  ASSERT_TRUE(backend.Apply(cmd));
  StepBackend(backend, s, now_us, 5000, 200);
  EXPECT_DOUBLE_EQ(s.motors[0].position_rot, -0.3);
  EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
  EXPECT_NEAR(s.motors[0].stator_current_a, 1.5 * 366.0 / 12.0, 1e-9);

  // Negative motor voltage moves the inverted CANcoder reading positively into
  // max_rot = +0.5 (not +0.3).
  cmd[0].demand = -1.5;
  ASSERT_TRUE(backend.Apply(cmd));
  StepBackend(backend, s, now_us, 5000, 200);
  EXPECT_DOUBLE_EQ(s.motors[0].position_rot, 0.5);
  EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
  EXPECT_NEAR(s.motors[0].stator_current_a, 1.5 * 366.0 / 12.0, 1e-9);
}

// REQ-DEV-10, REQ-DEV-05: StepChassis applies drive motor inversion, steer
// motor inversion, and remote CANcoder direction + offset_rot, matching
// PhoenixBackend's chassis kinematics.
TEST(SimWorld,
     StepChassisAppliesDriveInversionSteerInversionAndCancoderOffset) {
  // Remote-CANcoder swerve with alternating drive inversion, inverted steer,
  // and nonzero CANcoder offset_rot drives CCW yaw cleanly (matching
  // PhoenixBackendPigeon.SwerveRemoteCancoderWithMagnetOffsetAndInversionDrivesYawCcw).
  {
    Config config;
    const std::array<const char*, 4> names{"front_left", "front_right",
                                           "back_left", "back_right"};
    const std::array<double, 4> spin_angles{0.375, 0.125, -0.375, -0.125};
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto enc_id = static_cast<uint16_t>(10 + i);
      SensorConfig enc{enc_id, std::string(names[i]) + "_enc",
                       SensorKind::kCANcoder, static_cast<int>(10 + i), "rio"};
      enc.inverted = true;
      enc.offset_rot = 0.25;
      config.sensors.push_back(enc);

      auto drive = TestMotor(static_cast<uint16_t>(2 * i + 1), "drive",
                             static_cast<int>(2 * i + 1));
      drive.name = std::string(names[i]) + "_drive";
      drive.inverted = (i % 2 == 1);
      drive.sensor_to_mechanism_ratio = 6.75;

      auto steer = TestMotor(static_cast<uint16_t>(2 * i + 2), "steer",
                             static_cast<int>(2 * i + 2));
      steer.name = std::string(names[i]) + "_steer";
      steer.inverted = true;
      steer.feedback = Feedback::kRemoteCANcoder;
      steer.feedback_sensor_id = enc_id;
      steer.rotor_to_sensor_ratio = 12.8;
      steer.continuous_wrap = true;
      steer.slots[0].p = 12.0;

      config.motors.push_back(drive);
      config.motors.push_back(steer);
    }
    config.sensors.push_back({20, "imu", SensorKind::kPigeon2, 30, "rio"});

    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(config));
    std::array<MotorRequest, kMaxMotors> cmd{};
    for (std::size_t i = 0; i < 4; ++i) {
      cmd[2 * i + 1] = {static_cast<uint16_t>(2 * i + 2), Mode::kPosition, 0,
                        spin_angles[i], 0.0};
    }
    ASSERT_TRUE(backend.Apply(cmd));
    State state{};
    uint64_t now_us = 0;
    StepBackend(backend, state, now_us, 5000, 400);
    for (std::size_t i = 0; i < 4; ++i) {
      EXPECT_NEAR(
          std::remainder(state.sensors[i].position_rot - spin_angles[i], 1.0),
          0.0, 0.02)
          << "corner " << i;
    }
    for (std::size_t i = 0; i < 4; ++i) {
      cmd[2 * i] = {static_cast<uint16_t>(2 * i + 1), Mode::kVoltage, 0, 6.0,
                    0.0};
    }
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, state, now_us, 5000, 200);
    EXPECT_GT(backend.truth().omega_radps, 0.5);
    EXPECT_GT(backend.truth().yaw_rad, 0.2);
    EXPECT_GT(state.sensors[4].velocity_rps, 0.1);
    EXPECT_GT(state.sensors[4].position_rot, 0.05);
  }

  // Inverted rotor-feedback drive and steer motors translate in the commanded
  // robot-frame direction (+y left when steered to +0.25 rot).
  {
    Config config;
    const std::array<const char*, 4> names{"front_left", "front_right",
                                           "back_left", "back_right"};
    for (std::size_t i = 0; i < names.size(); ++i) {
      auto drive = TestMotor(static_cast<uint16_t>(2 * i + 1), "drive",
                             static_cast<int>(2 * i + 1));
      drive.name = std::string(names[i]) + "_drive";
      drive.inverted = true;
      drive.sensor_to_mechanism_ratio = 6.75;
      auto steer = TestMotor(static_cast<uint16_t>(2 * i + 2), "steer",
                             static_cast<int>(2 * i + 2));
      steer.name = std::string(names[i]) + "_steer";
      steer.inverted = true;
      steer.slots[0].p = 12.0;
      config.motors.push_back(drive);
      config.motors.push_back(steer);
    }
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(config));
    std::array<MotorRequest, kMaxMotors> cmd{};
    for (std::size_t i = 0; i < 4; ++i) {
      cmd[2 * i + 1] = {static_cast<uint16_t>(2 * i + 2), Mode::kPosition, 0,
                        0.25, 0.0};
    }
    ASSERT_TRUE(backend.Apply(cmd));
    State state{};
    uint64_t now_us = 0;
    StepBackend(backend, state, now_us, 5000, 400);
    for (std::size_t i = 0; i < 4; ++i) {
      cmd[2 * i] = {static_cast<uint16_t>(2 * i + 1), Mode::kVoltage, 0, 6.0,
                    0.0};
    }
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, state, now_us, 5000, 200);
    EXPECT_GT(backend.truth().vy_mps, 1.0);
    EXPECT_GT(backend.truth().y_m, 0.5);
    EXPECT_NEAR(backend.truth().vx_mps, 0.0, 1e-2);
    EXPECT_NEAR(backend.truth().omega_radps, 0.0, 1e-2);
  }
}

// REQ-DEV-10, REQ-DEV-05: Both PhysicsSimBackend and SimBackend apply CANcoder
// offset_rot (after sensor inversion, matching Phoenix 6 MagnetOffset) to
// standalone CANcoders, linked CANcoders, and remote-CANcoder motor feedback.
TEST(SimBelievability, CancoderOffsetAppliesInPhysicsAndIdealBackends) {
  for (const bool sensor_inverted : {false, true}) {
    SCOPED_TRACE(sensor_inverted);
    Config cfg = SingleMotorConfig();
    cfg.motors[0].inverted = sensor_inverted;
    cfg.motors[0].feedback = Feedback::kRemoteCANcoder;
    cfg.motors[0].feedback_sensor_id = 2;
    cfg.motors[0].slots[0].p = 10.0;
    cfg.motors[0].slots[0].d = 0.2;
    SensorConfig linked{2, "linked", SensorKind::kCANcoder, 20, "rio"};
    linked.inverted = sensor_inverted;
    linked.offset_rot = 0.25;
    SensorConfig unlinked{3, "unlinked", SensorKind::kCANcoder, 21, "rio"};
    unlinked.inverted = sensor_inverted;
    unlinked.offset_rot = -0.375;
    cfg.sensors.push_back(linked);
    cfg.sensors.push_back(unlinked);

    // PhysicsSimBackend: resting offset, closed-loop convergence, and soft
    // limit clamping all respect offset_rot without drifting.
    {
      PhysicsSimBackend physics;
      ASSERT_TRUE(physics.Configure(cfg));
      State s{};
      uint64_t now_us = 0;
      StepBackend(physics, s, now_us, 5000, 1);
      EXPECT_DOUBLE_EQ(s.motors[0].position_rot, 0.25);
      EXPECT_DOUBLE_EQ(s.sensors[0].position_rot, 0.25);
      EXPECT_DOUBLE_EQ(s.sensors[1].position_rot, -0.375);

      std::array<MotorRequest, 1> cmd{{{1, Mode::kPosition, 0, 0.6, 0.0}}};
      ASSERT_TRUE(physics.Apply(cmd));
      StepBackend(physics, s, now_us, 5000, 400);
      EXPECT_NEAR(s.motors[0].position_rot, 0.6, 0.01);
      EXPECT_NEAR(s.sensors[0].position_rot, -0.4, 0.01);
      EXPECT_NEAR(
          std::remainder(s.motors[0].position_rot - s.sensors[0].position_rot,
                         1.0),
          0.0, 1e-9);

      // Soft-limit clamping with nonzero offset_rot holds the limit without
      // shifting rotor position by offset_rot on subsequent ticks.
      Config limited = cfg;
      limited.motors[0].soft_limits = true;
      limited.motors[0].reverse_limit_rot = -0.4;
      limited.motors[0].forward_limit_rot = 0.45;
      ASSERT_TRUE(physics.Configure(limited));
      cmd[0] = {1, Mode::kVoltage, 0, 2.0, 0.0};
      ASSERT_TRUE(physics.Apply(cmd));
      StepBackend(physics, s, now_us, 5000, 100);
      EXPECT_DOUBLE_EQ(s.motors[0].position_rot, 0.45);
      EXPECT_NEAR(s.sensors[0].position_rot, 0.45, 1e-9);
    }

    // SimBackend: resting offset on linked and unlinked CANcoders, velocity
    // integration from offset, and position demand matching CANcoder wrap.
    {
      SimBackend ideal;
      ASSERT_TRUE(ideal.Configure(cfg));
      State s{};
      uint64_t now_us = 0;
      StepBackend(ideal, s, now_us, 5000, 1);
      EXPECT_DOUBLE_EQ(s.motors[0].position_rot, 0.25);
      EXPECT_DOUBLE_EQ(s.sensors[0].position_rot, 0.25);
      EXPECT_DOUBLE_EQ(s.sensors[1].position_rot, -0.375);

      std::array<MotorRequest, 1> vel_cmd{{{1, Mode::kVelocity, 0, 2.0, 0.0}}};
      ASSERT_TRUE(ideal.Apply(vel_cmd));
      StepBackend(ideal, s, now_us, 100000, 1);  // +0.2 rot -> 0.45
      EXPECT_NEAR(s.motors[0].position_rot, 0.45, 1e-9);
      EXPECT_NEAR(s.sensors[0].position_rot, 0.45, 1e-9);

      std::array<MotorRequest, 1> pos_cmd{{{1, Mode::kPosition, 0, 0.75, 0.0}}};
      ASSERT_TRUE(ideal.Apply(pos_cmd));
      StepBackend(ideal, s, now_us, 5000, 1);
      EXPECT_DOUBLE_EQ(s.motors[0].position_rot, 0.75);
      EXPECT_DOUBLE_EQ(s.sensors[0].position_rot, -0.25);
    }
  }
}

// REQ-DEV-10: continuous_wrap = true wraps closed-loop position and profiled
// Motion Magic errors across the +/-0.5 rot seam, taking the shortest angular
// path instead of rotating 345 degrees the long way around.
TEST(SimBelievability, ContinuousWrapTakesShortestPathAcrossHalfTurnSeam) {
  MotorConfig wrap_cfg = TestMotor(1, "steer", 1);
  wrap_cfg.slots[0].p = 10.0;
  wrap_cfg.continuous_wrap = true;
  double volts_wrap = 0.0;
  ASSERT_TRUE(sim::DriveVoltage({1, Mode::kPosition, 0, -0.48, 0.0}, wrap_cfg,
                                0.48, 0.0, 12.0, volts_wrap));
  EXPECT_NEAR(volts_wrap, 0.4, 1e-9);

  wrap_cfg.continuous_wrap = false;
  double volts_nowrap = 0.0;
  ASSERT_TRUE(sim::DriveVoltage({1, Mode::kPosition, 0, -0.48, 0.0}, wrap_cfg,
                                0.48, 0.0, 12.0, volts_nowrap));
  EXPECT_NEAR(volts_nowrap, -9.6, 1e-9);

  for (const auto mode : {Mode::kPosition, Mode::kMotionMagic}) {
    SCOPED_TRACE(static_cast<int>(mode));
    Config c = SingleMotorConfig();
    c.motors[0].continuous_wrap = true;
    c.motors[0].cruise_velocity_rps = 5.0;
    c.motors[0].acceleration_rps2 = 20.0;
    c.motors[0].slots[0].p = 12.0;
    c.motors[0].slots[0].d = 0.2;
    c.motors[0].slots[0].v = 0.12;
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(c));

    State s{};
    uint64_t now_us = 0;
    std::array<MotorRequest, 1> cmd{{{1, mode, 0, 0.48, 0.0}}};
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, s, now_us, 5000, 400);
    ASSERT_NEAR(s.motors[0].position_rot, 0.48, 0.01);

    // Cross the +0.5 seam: 0.48 -> -0.48 must travel +0.04 to +0.52.
    cmd[0].demand = -0.48;
    ASSERT_TRUE(backend.Apply(cmd));
    double min_pos = s.motors[0].position_rot;
    for (int i = 0; i < 400; ++i) {
      StepBackend(backend, s, now_us, 5000, 1);
      min_pos = std::min(min_pos, s.motors[0].position_rot);
    }
    EXPECT_GT(min_pos, 0.45);
    EXPECT_NEAR(s.motors[0].position_rot, 0.52, 0.01);

    // Cross back across the seam: 0.52 -> +0.48 must travel -0.04 to +0.48.
    cmd[0].demand = 0.48;
    ASSERT_TRUE(backend.Apply(cmd));
    double max_pos = s.motors[0].position_rot;
    for (int i = 0; i < 400; ++i) {
      StepBackend(backend, s, now_us, 5000, 1);
      max_pos = std::max(max_pos, s.motors[0].position_rot);
    }
    EXPECT_LT(max_pos, 0.55);
    EXPECT_NEAR(s.motors[0].position_rot, 0.48, 0.01);
  }
}

// REQ-DEV-10: Shortening a Motion Magic goal inside the current stopping
// distance (vel^2 / (2 * accel) > goal - pos) decelerates smoothly at most
// acceleration_rps2 without teleporting backwards or spiking acceleration,
// then turns around and settles at the new goal.
TEST(SimBelievability,
     MotionMagicGoalShorteningDeceleratesSmoothlyWithoutTeleport) {
  // Isolate AdvanceProfile's returned acceleration through kA feedforward:
  // with only kA = 1.0 (unclamped voltage), applied_voltage == profile accel.
  {
    Config c = SingleMotorConfig();
    auto& m = c.motors[0];
    m.cruise_velocity_rps = 10.0;
    m.acceleration_rps2 = 10.0;
    m.max_voltage = 12.0;
    m.stator_limit_a = 400.0;
    m.supply_limit_a = 400.0;
    m.slots[0].p = 10.0;
    m.slots[0].v = 0.12;
    m.slots[0].a = 0.05;
    m.slots[0].d = 0.1;

    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(c));
    State s{};
    uint64_t now_us = 0;
    std::array<MotorRequest, 1> cmd{{{1, Mode::kMotionMagic, 0, 20.0, 0.0}}};
    ASSERT_TRUE(backend.Apply(cmd));
    // Accelerate to cruise (~10 rps); stopping distance is ~5 rot.
    StepBackend(backend, s, now_us, 5000, 250);
    ASSERT_GT(s.motors[0].velocity_rps, 8.0);

    const double pos_at_change = s.motors[0].position_rot;
    const double vel_at_change = s.motors[0].velocity_rps;
    const double shortened_goal = pos_at_change + 0.5;
    cmd[0].demand = shortened_goal;
    ASSERT_TRUE(backend.Apply(cmd));

    // First step after shortening: must continue forward and not spike accel
    // (old code teleported profile_pos backwards and spiked voltage to -12 V).
    StepBackend(backend, s, now_us, 5000, 1);
    EXPECT_GT(s.motors[0].position_rot, pos_at_change);
    EXPECT_GT(s.motors[0].velocity_rps, 0.5 * vel_at_change);
    EXPECT_GT(backend.applied_voltage(0), -6.0);

    // Continue stepping: must settle cleanly at the shortened goal.
    StepBackend(backend, s, now_us, 5000, 1200);
    EXPECT_NEAR(s.motors[0].position_rot, shortened_goal, 0.02);
    EXPECT_NEAR(s.motors[0].velocity_rps, 0.0, 0.02);
  }

  // Direct acceleration bound check: kA = 0.5, kP = kV = kD = 0 so applied
  // voltage is strictly 0.5 * profile_accel; must never exceed 0.5 * accel
  // even when entering Motion Magic faster than cruise_velocity_rps or
  // reversing goal inside stopping distance.
  {
    Config c = SingleMotorConfig();
    auto& m = c.motors[0];
    m.cruise_velocity_rps = 5.0;
    m.acceleration_rps2 = 10.0;
    m.max_voltage = 12.0;
    m.stator_limit_a = 400.0;
    m.supply_limit_a = 400.0;
    m.slots[0].a = 0.5;  // Expected |applied_voltage| <= 5.0 V.

    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(c));
    State s{};
    uint64_t now_us = 0;
    // Spin rotor to > 20 rps (above cruise = 5 rps), then engage Motion Magic.
    std::array<MotorRequest, 1> cmd{Duty(1, 0.4)};
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, s, now_us, 5000, 100);
    ASSERT_GT(s.motors[0].velocity_rps, 20.0);

    cmd[0] = {1, Mode::kMotionMagic, 0, s.motors[0].position_rot + 0.5, 0.0};
    ASSERT_TRUE(backend.Apply(cmd));
    for (int i = 0; i < 400; ++i) {
      StepBackend(backend, s, now_us, 5000, 1);
      EXPECT_LE(std::abs(backend.applied_voltage(0)), 5.0 + 1e-6)
          << "step " << i;
    }
  }
}

// REQ-DEV-10: Explicitly setting supply voltage to 0 V clamps output to 0 V
// across every control mode rather than falling back to 12 V.
TEST(SimBelievability, ZeroSupplyVoltageAppliesZeroVoltsAcrossAllModes) {
  MotorConfig cfg = TestMotor(1, "roller", 1);
  cfg.slots[0].p = 10.0;
  cfg.slots[0].v = 0.5;
  for (const auto mode : {Mode::kDutyCycle, Mode::kVoltage, Mode::kVelocity,
                          Mode::kPosition, Mode::kMotionMagic}) {
    double volts = 99.0;
    ASSERT_TRUE(
        sim::DriveVoltage({1, mode, 0, 5.0, 2.0}, cfg, 0.0, 0.0, 0.0, volts));
    EXPECT_DOUBLE_EQ(volts, 0.0) << "mode=" << static_cast<int>(mode);
  }

  const sim::AxisState stepped =
      sim::StepAxis(sim::MotorModel{}, sim::MechanismModel{}, sim::AxisState{},
                    12.0, true, 0.0, 0.05);
  EXPECT_DOUBLE_EQ(stepped.rotor_velocity_rps, 0.0);
  EXPECT_DOUBLE_EQ(stepped.rotor_position_rot, 0.0);

  for (const auto mode :
       {Mode::kDutyCycle, Mode::kVoltage, Mode::kVelocity, Mode::kPosition,
        Mode::kMotionMagic, Mode::kTorqueCurrent}) {
    SCOPED_TRACE(static_cast<int>(mode));
    Config c = SingleMotorConfig();
    c.motors[0].foc = true;
    c.motors[0].max_torque_current_a = 40.0;
    c.motors[0].slots[0].p = 10.0;
    c.motors[0].slots[0].v = 0.5;
    PhysicsSimBackend backend;
    ASSERT_TRUE(backend.Configure(c));
    backend.SetSupplyVoltage(0.0);
    std::array<MotorRequest, 1> cmd{{{1, mode, 0, 5.0, 0.0}}};
    ASSERT_TRUE(backend.Apply(cmd));
    State s{};
    uint64_t now_us = 0;
    StepBackend(backend, s, now_us, 5000, 20);
    EXPECT_DOUBLE_EQ(backend.applied_voltage(0), 0.0);
    EXPECT_DOUBLE_EQ(s.motors[0].voltage, 0.0);
    EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 0.0);
    EXPECT_DOUBLE_EQ(s.motors[0].position_rot, 0.0);
    EXPECT_DOUBLE_EQ(s.motors[0].supply_current_a, 0.0);
  }
}

// REQ-DEV-10, REQ-SAFE-22: Disconnecting or freezing a remote-CANcoder motor
// does not teleport its independent, healthy CANcoder to 0.0 rot.
TEST(SimBelievability,
     DisconnectingOrFreezingMotorKeepsHealthyCancoderReading) {
  Config c = SingleMotorConfig();
  SensorConfig enc{2, "encoder", SensorKind::kCANcoder, 20, "rio"};
  enc.offset_rot = 0.1;
  c.sensors.push_back(enc);
  c.motors[0].feedback = Feedback::kRemoteCANcoder;
  c.motors[0].feedback_sensor_id = 2;
  c.motors[0].sensor_to_mechanism_ratio = 2.0;

  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(c));
  std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 2.0, 0.0}}};
  ASSERT_TRUE(backend.Apply(cmd));
  State s{};
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 10);
  // Stop motor away from 0.0 rot on the CANcoder.
  backend.Neutral();
  StepBackend(backend, s, now_us, 5000, 50);
  const double settled_sensor = s.sensors[0].position_rot;
  ASSERT_GT(std::abs(settled_sensor), 0.05);

  backend.SetMotorConnected(1, false);
  StepBackend(backend, s, now_us, 5000, 2);
  EXPECT_FALSE(s.motors[0].valid);
  EXPECT_TRUE(s.sensors[0].valid);
  EXPECT_NEAR(s.sensors[0].position_rot, settled_sensor, 1e-9);

  backend.SetMotorConnected(1, true);
  backend.FreezeFeedback(1);
  StepBackend(backend, s, now_us, 5000, 2);
  EXPECT_TRUE(s.motors[0].valid);
  EXPECT_TRUE(s.sensors[0].valid);
  EXPECT_NEAR(s.sensors[0].position_rot, settled_sensor, 1e-9);
}

// REQ-DEV-10: Configure() and RestartController() both clear brownout_.
TEST(SimHealth, ConfigureAndRestartControllerClearBrownout) {
  const Config cfg = SingleMotorConfig();
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(cfg));
  State s{};
  uint64_t now_us = 0;

  backend.SetPowerHealth(6.0, true);
  StepBackend(backend, s, now_us, 5000, 1);
  ASSERT_TRUE(s.health.brownout);
  ASSERT_DOUBLE_EQ(s.health.battery_v, 6.0);

  backend.RestartController();
  StepBackend(backend, s, now_us, 5000, 1);
  EXPECT_FALSE(s.health.brownout);

  backend.SetPowerHealth(6.0, true);
  StepBackend(backend, s, now_us, 5000, 1);
  ASSERT_TRUE(s.health.brownout);

  ASSERT_TRUE(backend.Configure(cfg));
  StepBackend(backend, s, now_us, 5000, 1);
  EXPECT_FALSE(s.health.brownout);
  EXPECT_DOUBLE_EQ(s.health.battery_v, kSimRestingSupplyV);
}

// REQ-DEV-05, REQ-DEV-06, REQ-DEV-10: With remote CANcoder feedback and
// sensor_to_mechanism_ratio != 1, the CANcoder sits on the sensor shaft
// (rotor / rotor_to_sensor_ratio = mechanism * sensor_to_mechanism_ratio) and
// the TalonFX divides the CANcoder reading (including offset_rot) by
// sensor_to_mechanism_ratio once, matching Phoenix 6 and PhoenixBackend.
TEST(SimBelievability,
     RemoteCancoderScalesBySensorToMechanismRatioInPhysicsAndIdealBackends) {
  constexpr double kRotorToSensor = 4.0;
  constexpr double kSensorToMechanism = 2.0;
  constexpr double kTotalRatio = kRotorToSensor * kSensorToMechanism;
  constexpr double kOffsetRot = 0.25;

  for (const bool sensor_inverted : {false, true}) {
    SCOPED_TRACE(sensor_inverted);
    Config cfg = SingleMotorConfig();
    cfg.motors[0].inverted = sensor_inverted;
    cfg.motors[0].feedback = Feedback::kRemoteCANcoder;
    cfg.motors[0].feedback_sensor_id = 2;
    cfg.motors[0].rotor_to_sensor_ratio = kRotorToSensor;
    cfg.motors[0].sensor_to_mechanism_ratio = kSensorToMechanism;
    cfg.motors[0].slots[0].p = 10.0;
    cfg.motors[0].slots[0].d = 0.2;
    // Reference 1:1 rotor-feedback motor driven identically in
    // PhysicsSimBackend to verify exact rotor-to-sensor and rotor-to-mechanism
    // ratios.
    auto ref = TestMotor(3, "rotor_ref", 3);
    ref.inverted = sensor_inverted;
    cfg.motors.push_back(ref);
    SensorConfig enc{2, "encoder", SensorKind::kCANcoder, 20, "rio"};
    enc.inverted = sensor_inverted;
    enc.offset_rot = kOffsetRot;
    cfg.sensors.push_back(enc);

    // PhysicsSimBackend: resting offset, open-loop rotor/sensor/mechanism
    // consistency, closed-loop position convergence, and soft limits.
    {
      PhysicsSimBackend physics;
      ASSERT_TRUE(physics.Configure(cfg));
      State s{};
      uint64_t now_us = 0;
      StepBackend(physics, s, now_us, 5000, 1);
      EXPECT_DOUBLE_EQ(s.motors[0].position_rot,
                       kOffsetRot / kSensorToMechanism);
      EXPECT_DOUBLE_EQ(s.sensors[0].position_rot, kOffsetRot);

      std::array<MotorRequest, 2> open_loop{
          {{1, Mode::kVoltage, 0, 6.0, 0.0}, {3, Mode::kVoltage, 0, 6.0, 0.0}}};
      ASSERT_TRUE(physics.Apply(open_loop));
      StepBackend(physics, s, now_us, 5000, 60);
      ASSERT_GT(s.motors[1].velocity_rps, 10.0);
      ASSERT_GT(s.motors[1].position_rot, 1.0);
      EXPECT_NEAR(s.motors[0].velocity_rps,
                  s.motors[1].velocity_rps / kTotalRatio, 1e-9);
      EXPECT_NEAR(s.sensors[0].velocity_rps,
                  s.motors[1].velocity_rps / kRotorToSensor, 1e-9);
      EXPECT_NEAR(s.sensors[0].velocity_rps,
                  s.motors[0].velocity_rps * kSensorToMechanism, 1e-9);
      EXPECT_NEAR(s.motors[0].position_rot,
                  s.motors[1].position_rot / kTotalRatio +
                      kOffsetRot / kSensorToMechanism,
                  1e-9);
      EXPECT_NEAR(
          std::remainder(
              s.sensors[0].position_rot -
                  (s.motors[1].position_rot / kRotorToSensor + kOffsetRot),
              1.0),
          0.0, 1e-9);
      EXPECT_NEAR(
          std::remainder(s.sensors[0].position_rot -
                             s.motors[0].position_rot * kSensorToMechanism,
                         1.0),
          0.0, 1e-9);

      // Closed-loop position convergence: 0.6 mech rot -> 1.2 sensor rot ->
      // +0.2 wrapped CANcoder reading.
      ASSERT_TRUE(physics.Configure(cfg));
      std::array<MotorRequest, 2> pos_cmd{{{1, Mode::kPosition, 0, 0.6, 0.0},
                                           {3, Mode::kNeutral, 0, 0.0, 0.0}}};
      ASSERT_TRUE(physics.Apply(pos_cmd));
      StepBackend(physics, s, now_us, 5000, 600);
      EXPECT_NEAR(s.motors[0].position_rot, 0.6, 0.01);
      EXPECT_NEAR(s.sensors[0].position_rot, 0.2, 0.02);
      EXPECT_NEAR(
          std::remainder(s.sensors[0].position_rot -
                             s.motors[0].position_rot * kSensorToMechanism,
                         1.0),
          0.0, 1e-9);

      // Soft limit clamping holds 0.45 mech rot -> 0.90 sensor rot -> -0.10.
      Config limited = cfg;
      limited.motors[0].soft_limits = true;
      limited.motors[0].reverse_limit_rot = -0.4;
      limited.motors[0].forward_limit_rot = 0.45;
      ASSERT_TRUE(physics.Configure(limited));
      std::array<MotorRequest, 2> drive_cmd{
          {{1, Mode::kVoltage, 0, 2.0, 0.0}, {3, Mode::kNeutral, 0, 0.0, 0.0}}};
      ASSERT_TRUE(physics.Apply(drive_cmd));
      StepBackend(physics, s, now_us, 5000, 400);
      EXPECT_DOUBLE_EQ(s.motors[0].position_rot, 0.45);
      EXPECT_NEAR(s.sensors[0].position_rot, -0.1, 1e-9);
    }

    // SimBackend: resting offset divided by S, velocity and position mirrored
    // onto sensor shaft (* S).
    {
      SimBackend ideal;
      ASSERT_TRUE(ideal.Configure(cfg));
      State s{};
      uint64_t now_us = 0;
      StepBackend(ideal, s, now_us, 5000, 1);
      EXPECT_DOUBLE_EQ(s.motors[0].position_rot,
                       kOffsetRot / kSensorToMechanism);
      EXPECT_DOUBLE_EQ(s.sensors[0].position_rot, kOffsetRot);

      std::array<MotorRequest, 2> vel_cmd{{{1, Mode::kVelocity, 0, 2.0, 0.0},
                                           {3, Mode::kNeutral, 0, 0.0, 0.0}}};
      ASSERT_TRUE(ideal.Apply(vel_cmd));
      StepBackend(ideal, s, now_us, 100000, 1);  // +0.2 mech rot -> 0.325
      EXPECT_NEAR(s.motors[0].position_rot, 0.325, 1e-9);
      EXPECT_DOUBLE_EQ(s.motors[0].velocity_rps, 2.0);
      EXPECT_NEAR(s.sensors[0].position_rot, -0.35, 1e-9);  // wrap(0.65)
      EXPECT_DOUBLE_EQ(s.sensors[0].velocity_rps, 4.0);

      std::array<MotorRequest, 2> pos_cmd{{{1, Mode::kPosition, 0, 0.6, 0.0},
                                           {3, Mode::kNeutral, 0, 0.0, 0.0}}};
      ASSERT_TRUE(ideal.Apply(pos_cmd));
      StepBackend(ideal, s, now_us, 5000, 1);
      EXPECT_DOUBLE_EQ(s.motors[0].position_rot, 0.6);
      EXPECT_NEAR(s.sensors[0].position_rot, 0.2, 1e-9);  // wrap(1.2)
      EXPECT_DOUBLE_EQ(s.sensors[0].velocity_rps, 0.0);
    }
  }
}

namespace {

constexpr double kTestTau = 6.28318530717958647692;

Config FourCornerSwerveConfig(double gear_ratio = 6.75,
                              double stator_limit_a = 80.0,
                              double supply_limit_a = 400.0) {
  Config config;
  config.commissioned = true;
  config.command_timeout_us = 100000;
  const std::array<const char*, 4> names{"front_left", "front_right",
                                         "back_left", "back_right"};
  for (std::size_t i = 0; i < names.size(); ++i) {
    auto drive = TestMotor(static_cast<uint16_t>(2 * i + 1), "drive",
                           static_cast<int>(2 * i + 1));
    drive.name = std::string(names[i]) + "_drive";
    drive.sensor_to_mechanism_ratio = gear_ratio;
    drive.stator_limit_a = stator_limit_a;
    drive.supply_limit_a = supply_limit_a;
    drive.max_voltage = 12.0;

    auto steer = TestMotor(static_cast<uint16_t>(2 * i + 2), "steer",
                           static_cast<int>(2 * i + 2));
    steer.name = std::string(names[i]) + "_steer";
    steer.continuous_wrap = true;
    steer.slots[0].p = 20.0;
    steer.slots[0].d = 0.2;
    steer.stator_limit_a = 80.0;
    steer.supply_limit_a = 80.0;

    config.motors.push_back(drive);
    config.motors.push_back(steer);
  }
  config.sensors.push_back({20, "imu", SensorKind::kPigeon2, 30, "rio"});
  return config;
}

}  // namespace

// REQ-DEV-10: Rigid-body mechanism ODE obeys analytical current-limited
// flywheel spin-up (delta_omega ~= (G * k_t * I_lim / J_mech) * dt), scales
// monotonically in inertia J and gear ratio G, honors static friction stiction,
// and survives Configure().
TEST(SimMechanismOde, FlywheelSpinUpInertiaGearingAndStaticFriction) {
  constexpr double kGear = 2.0;
  constexpr double kInertia = 0.02;
  constexpr double kStatorLimit = 40.0;
  constexpr double kDt = 0.05;  // 10 x 5 ms
  const sim::MotorModel x60 = sim::MotorModelFor(MotorType::kKrakenX60);
  const double expected_delta_rps =
      (kGear * x60.torque_constant_nm_per_a() * kStatorLimit / kInertia) * kDt /
      kTestTau;

  Config cfg = SingleMotorConfig();
  cfg.motors[0].sensor_to_mechanism_ratio = kGear;
  cfg.motors[0].stator_limit_a = kStatorLimit;
  cfg.motors[0].supply_limit_a = 400.0;

  // SetMechanisms before Configure() must survive Configure().
  PhysicsSimBackend backend;
  backend.SetMechanisms({{.motor = "roller",
                          .inertia_kg_m2 = kInertia,
                          .load = sim::LoadKind::kFlywheel}});
  ASSERT_TRUE(backend.Configure(cfg));
  std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
  ASSERT_TRUE(backend.Apply(cmd));
  State s{};
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 1);   // dt = 0
  StepBackend(backend, s, now_us, 5000, 10);  // 50 ms
  EXPECT_NEAR(s.motors[0].velocity_rps, expected_delta_rps,
              0.02 * expected_delta_rps);
  EXPECT_NEAR(s.motors[0].stator_current_a, kStatorLimit, 0.5);

  // Monotonicity in J: larger inertia yields strictly slower spin-up, scaling
  // inversely with J while current-limited.
  double prev_vel = std::numeric_limits<double>::infinity();
  std::array<double, 3> vels_by_j{};
  const std::array<double, 3> inertias{0.005, 0.02, 0.08};
  for (std::size_t idx = 0; idx < inertias.size(); ++idx) {
    PhysicsSimBackend b;
    b.SetMechanisms({{.motor = "roller",
                      .inertia_kg_m2 = inertias[idx],
                      .load = sim::LoadKind::kFlywheel}});
    ASSERT_TRUE(b.Configure(cfg));
    ASSERT_TRUE(b.Apply(cmd));
    State st{};
    uint64_t t = 0;
    StepBackend(b, st, t, 5000, 1);
    StepBackend(b, st, t, 5000, 10);
    vels_by_j[idx] = st.motors[0].velocity_rps;
    EXPECT_LT(vels_by_j[idx], prev_vel);
    prev_vel = vels_by_j[idx];
  }
  EXPECT_NEAR(vels_by_j[0] / vels_by_j[1], 4.0, 0.05);
  EXPECT_NEAR(vels_by_j[1] / vels_by_j[2], 4.0, 0.05);

  // Monotonicity in G: higher gear ratio multiplies mechanism torque
  // (G * k_t * I_lim) so current-limited initial acceleration increases with G.
  double prev_gear_vel = 0.0;
  for (const double gear : {1.0, 2.0, 4.0}) {
    Config gcfg = cfg;
    gcfg.motors[0].sensor_to_mechanism_ratio = gear;
    PhysicsSimBackend b;
    b.SetMechanisms({{.motor = "roller",
                      .inertia_kg_m2 = 0.05,
                      .load = sim::LoadKind::kFlywheel}});
    ASSERT_TRUE(b.Configure(gcfg));
    ASSERT_TRUE(b.Apply(cmd));
    State st{};
    uint64_t t = 0;
    StepBackend(b, st, t, 5000, 1);
    StepBackend(b, st, t, 5000, 5);  // 25 ms, still current-limited
    EXPECT_GT(st.motors[0].velocity_rps, prev_gear_vel);
    prev_gear_vel = st.motors[0].velocity_rps;
  }

  // Static friction holds the shaft at 0 below breakaway torque and lets it
  // spin above breakaway torque.
  {
    PhysicsSimBackend b;
    b.SetMechanisms({{.motor = "roller",
                      .inertia_kg_m2 = 0.01,
                      .load = sim::LoadKind::kRoller,
                      .viscous_damping_nm_per_radps = 0.0,
                      .static_friction_nm = 2.0}});
    ASSERT_TRUE(
        b.Configure(cfg));  // G = 2 -> 1.0 N*m rotor breakaway (~51.6 A)
    // Stator limit 40 A gives G * k_t * 40 = 1.55 N*m < 2.0 N*m static
    // friction.
    ASSERT_TRUE(b.Apply(cmd));
    State st{};
    uint64_t t = 0;
    StepBackend(b, st, t, 5000, 20);
    EXPECT_DOUBLE_EQ(st.motors[0].velocity_rps, 0.0);
    EXPECT_DOUBLE_EQ(st.motors[0].position_rot, 0.0);

    // Raise stator limit to 80 A -> 3.10 N*m > 2.0 N*m: breaks away cleanly.
    Config strong = cfg;
    strong.motors[0].stator_limit_a = 80.0;
    ASSERT_TRUE(b.Configure(strong));
    ASSERT_TRUE(b.Apply(cmd));
    StepBackend(b, st, t, 5000, 1);
    StepBackend(b, st, t, 5000, 20);
    EXPECT_GT(st.motors[0].velocity_rps, 1.0);
  }
}

// REQ-DEV-10: Active followers (aligned or opposed, in any config order) sum
// torque onto the root master's shared rigid-body shaft, doubling acceleration
// for two identical current-limited motors.
TEST(SimMechanismOde, MasterAndFollowerDoubleAccelerationOnSharedShaft) {
  auto make_single = []() {
    Config c = SingleMotorConfig();
    c.motors[0].sensor_to_mechanism_ratio = 2.0;
    c.motors[0].stator_limit_a = 40.0;
    c.motors[0].supply_limit_a = 400.0;
    return c;
  };

  PhysicsSimBackend single_backend;
  single_backend.SetMechanisms({{.motor = "roller",
                                 .inertia_kg_m2 = 0.05,
                                 .load = sim::LoadKind::kFlywheel}});
  ASSERT_TRUE(single_backend.Configure(make_single()));
  std::array<MotorRequest, 1> single_cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
  ASSERT_TRUE(single_backend.Apply(single_cmd));
  State single_state{};
  uint64_t t0 = 0;
  StepBackend(single_backend, single_state, t0, 5000, 1);
  StepBackend(single_backend, single_state, t0, 5000, 10);
  const double single_vel = single_state.motors[0].velocity_rps;
  ASSERT_GT(single_vel, 0.2);

  for (const bool oppose : {false, true}) {
    for (const bool follower_first : {false, true}) {
      SCOPED_TRACE(testing::Message() << "oppose=" << oppose
                                      << " follower_first=" << follower_first);
      Config dual = make_single();
      auto follower = TestMotor(2, "follower", 2);
      follower.master_id = 1;
      follower.oppose_master = oppose;
      follower.sensor_to_mechanism_ratio = 2.0;
      follower.stator_limit_a = 40.0;
      follower.supply_limit_a = 400.0;
      dual.motors.push_back(follower);
      if (follower_first) std::swap(dual.motors[0], dual.motors[1]);
      const std::size_t master_idx = follower_first ? 1 : 0;
      const std::size_t slave_idx = 1 - master_idx;

      PhysicsSimBackend dual_backend;
      dual_backend.SetMechanisms({{.motor = "roller",
                                   .inertia_kg_m2 = 0.05,
                                   .load = sim::LoadKind::kFlywheel}});
      ASSERT_TRUE(dual_backend.Configure(dual));
      std::array<MotorRequest, 2> cmd{};
      cmd[master_idx] = {1, Mode::kVoltage, 0, 12.0, 0.0};
      cmd[slave_idx] = Duty(2, 0.0);  // Non-neutral activates follower.
      ASSERT_TRUE(dual_backend.Apply(cmd));
      State st{};
      uint64_t t = 0;
      StepBackend(dual_backend, st, t, 5000, 1);
      StepBackend(dual_backend, st, t, 5000, 10);

      EXPECT_NEAR(st.motors[master_idx].velocity_rps, 2.0 * single_vel,
                  0.03 * single_vel);
      EXPECT_NEAR(st.motors[slave_idx].velocity_rps,
                  st.motors[master_idx].velocity_rps * (oppose ? -1.0 : 1.0),
                  1e-9);
    }
  }
}

// REQ-DEV-10: In the rigid-body ODE, brake neutral applies shorted-winding
// back-EMF braking (V = 0), stopping far faster than coast neutral (I = 0,
// decaying only via viscous damping and static friction).
TEST(SimMechanismOde, BrakeNeutralStopsFasterThanCoastNeutral) {
  auto run_neutral = [](bool brake, double viscous) {
    Config cfg = SingleMotorConfig();
    cfg.motors[0].brake = brake;
    cfg.motors[0].stator_limit_a = 400.0;
    cfg.motors[0].supply_limit_a = 400.0;
    PhysicsSimBackend backend;
    backend.SetMechanisms({{.motor = "roller",
                            .inertia_kg_m2 = 0.005,
                            .load = sim::LoadKind::kFlywheel,
                            .viscous_damping_nm_per_radps = viscous}});
    EXPECT_TRUE(backend.Configure(cfg));
    std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
    EXPECT_TRUE(backend.Apply(cmd));
    State s{};
    uint64_t t = 0;
    StepBackend(backend, s, t, 5000, 1);
    StepBackend(backend, s, t, 5000, 120);  // 0.6 s spin-up
    const double peak = s.motors[0].velocity_rps;
    backend.Neutral();
    StepBackend(backend, s, t, 5000, 100);  // 0.5 s unpowered decay
    return std::pair{peak, s.motors[0].velocity_rps};
  };

  const auto [brake_peak, brake_after] = run_neutral(true, 0.0);
  const auto [coast_peak, coast_after] = run_neutral(false, 0.0);
  const auto [damped_peak, damped_after] = run_neutral(false, 0.005);

  ASSERT_GT(brake_peak, 60.0);
  ASSERT_NEAR(brake_peak, coast_peak, 1e-9);
  EXPECT_LT(brake_after, 0.4 * brake_peak);
  EXPECT_GT(coast_after, 0.95 * coast_peak);
  EXPECT_LT(damped_after, coast_after);
  EXPECT_GT(damped_after, brake_after);
}

// REQ-DEV-10: No-load motor viscous damping preserves exact CTRE 12 V free
// speed and free current across Kraken X60, Kraken X44, and Falcon 500.
TEST(SimMechanismOde, TwelveVoltFreeSpeedAndFreeCurrentMatchDatasheet) {
  struct Case {
    MotorType type;
    double free_rps;
    double free_a;
  };
  for (const Case& c : {Case{MotorType::kKrakenX60, 6000.0 / 60.0, 2.0},
                        Case{MotorType::kKrakenX44, 7530.0 / 60.0, 2.0},
                        Case{MotorType::kFalcon500, 6380.0 / 60.0, 1.5}}) {
    SCOPED_TRACE(static_cast<int>(c.type));
    Config cfg = SingleMotorConfig();
    cfg.motors[0].motor_type.value = c.type;
    cfg.motors[0].stator_limit_a = 1000.0;
    cfg.motors[0].supply_limit_a = 1000.0;
    PhysicsSimBackend backend;
    backend.SetMechanisms({{.motor = "roller",
                            .inertia_kg_m2 = 0.001,
                            .load = sim::LoadKind::kRoller}});
    ASSERT_TRUE(backend.Configure(cfg));
    std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
    ASSERT_TRUE(backend.Apply(cmd));
    State s{};
    uint64_t t = 0;
    StepBackend(backend, s, t, 5000, 1);
    StepBackend(backend, s, t, 5000, 500);  // 2.5 s -> > 12 time constants
    EXPECT_NEAR(s.motors[0].velocity_rps, c.free_rps, 0.001 * c.free_rps);
    EXPECT_NEAR(s.motors[0].stator_current_a, c.free_a, 0.02 * c.free_a);
  }
}

// REQ-DEV-10: Property test — rigid-body ODE stays unconditionally stable
// (finite, bounded by free speed, non-oscillatory) down to tiny inertia J.
TEST(SimMechanismOde, SmallInertiaRemainsNumericallyStable) {
  for (const double j : {1e-7, 1e-6, 1e-5, 1e-4, 1e-3}) {
    for (const double gear : {1.0, 5.0}) {
      for (const uint64_t dt_us :
           {uint64_t{1000}, uint64_t{5000}, uint64_t{20000}}) {
        SCOPED_TRACE(testing::Message()
                     << "j=" << j << " gear=" << gear << " dt_us=" << dt_us);
        Config cfg = SingleMotorConfig();
        cfg.motors[0].sensor_to_mechanism_ratio = gear;
        cfg.motors[0].stator_limit_a = 80.0;
        cfg.motors[0].supply_limit_a = 40.0;
        PhysicsSimBackend backend;
        backend.SetMechanisms({{.motor = "roller",
                                .inertia_kg_m2 = j,
                                .load = sim::LoadKind::kRoller}});
        ASSERT_TRUE(backend.Configure(cfg));
        std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
        ASSERT_TRUE(backend.Apply(cmd));
        State s{};
        uint64_t t = 0;
        StepBackend(backend, s, t, dt_us, 1);
        const double free_mech_rps = 100.0 / gear;
        double prev_vel = 0.0;
        for (int step = 0; step < 30; ++step) {
          StepBackend(backend, s, t, dt_us, 1);
          ASSERT_TRUE(std::isfinite(s.motors[0].velocity_rps));
          ASSERT_TRUE(std::isfinite(s.motors[0].position_rot));
          EXPECT_GE(s.motors[0].velocity_rps, prev_vel - 1e-6);
          EXPECT_LE(s.motors[0].velocity_rps, free_mech_rps * 1.005);
          prev_vel = s.motors[0].velocity_rps;
        }
        // Brake neutral from speed must also decay monotonically to >= 0.
        backend.Neutral();
        for (int step = 0; step < 20; ++step) {
          StepBackend(backend, s, t, dt_us, 1);
          ASSERT_TRUE(std::isfinite(s.motors[0].velocity_rps));
          EXPECT_LE(s.motors[0].velocity_rps, prev_vel + 1e-6);
          EXPECT_GE(s.motors[0].velocity_rps, -1e-9);
          prev_vel = s.motors[0].velocity_rps;
        }
      }
    }
  }
}

// REQ-DEV-10: Two master motors sharing the same CAN id on different buses
// bind MechanismSpec by unique motor name, so attaching a spec to "flywheel"
// on "rio" drives "flywheel" with the rigid-body ODE while "fl_drive" on
// "canivore" keeps the legacy first-order lag (and survives Configure()).
TEST(SimMechanismOde, SharedCanIdAcrossBusesBindsByMotorName) {
  constexpr double kGear = 2.0;
  constexpr double kInertia = 0.08;
  constexpr double kStatorLimit = 40.0;
  constexpr double kDt = 0.05;  // 10 x 5 ms
  const sim::MotorModel x60 = sim::MotorModelFor(MotorType::kKrakenX60);
  const double expected_ode_rps =
      (kGear * x60.torque_constant_nm_per_a() * kStatorLimit / kInertia) * kDt /
      kTestTau;

  auto make_motor = [](uint16_t id, const char* name, const char* bus) {
    MotorConfig m = TestMotor(id, name, /*can_id=*/1);
    m.bus = bus;
    m.sensor_to_mechanism_ratio = kGear;
    m.stator_limit_a = kStatorLimit;
    m.supply_limit_a = 400.0;
    return m;
  };

  // Reference legacy lag velocity for an identical motor with no MechanismSpec.
  Config legacy_cfg;
  legacy_cfg.commissioned = true;
  legacy_cfg.command_timeout_us = 100000;
  legacy_cfg.motors.push_back(make_motor(1, "fl_drive", "canivore"));
  PhysicsSimBackend legacy_backend;
  ASSERT_TRUE(legacy_backend.Configure(legacy_cfg));
  std::array<MotorRequest, 1> legacy_cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
  ASSERT_TRUE(legacy_backend.Apply(legacy_cmd));
  State legacy_state{};
  uint64_t legacy_t = 0;
  StepBackend(legacy_backend, legacy_state, legacy_t, 5000, 1);
  StepBackend(legacy_backend, legacy_state, legacy_t, 5000, 10);
  const double legacy_rps = legacy_state.motors[0].velocity_rps;
  ASSERT_GT(legacy_rps, 5.0 * expected_ode_rps);

  // Two master motors sharing can_id = 1 across two buses; fl_drive appears
  // first in config_.motors so CAN-id matching would misbind fl_drive instead
  // of flywheel.
  Config cfg;
  cfg.commissioned = true;
  cfg.command_timeout_us = 100000;
  cfg.motors.push_back(make_motor(1, "fl_drive", "canivore"));
  cfg.motors.push_back(make_motor(2, "flywheel", "rio"));

  PhysicsSimBackend backend;
  backend.SetMechanisms({{.motor = "flywheel",
                          .inertia_kg_m2 = kInertia,
                          .load = sim::LoadKind::kFlywheel}});
  ASSERT_TRUE(backend.Configure(cfg));
  std::array<MotorRequest, 2> cmd{
      {{1, Mode::kVoltage, 0, 12.0, 0.0}, {2, Mode::kVoltage, 0, 12.0, 0.0}}};
  ASSERT_TRUE(backend.Apply(cmd));
  State s{};
  uint64_t t = 0;
  StepBackend(backend, s, t, 5000, 1);   // dt = 0
  StepBackend(backend, s, t, 5000, 10);  // 50 ms

  // "fl_drive" (index 0, canivore, can_id = 1) keeps the legacy lag.
  EXPECT_NEAR(s.motors[0].velocity_rps, legacy_rps, 1e-9);
  // "flywheel" (index 1, rio, can_id = 1) runs the rigid-body ODE.
  EXPECT_NEAR(s.motors[1].velocity_rps, expected_ode_rps,
              0.02 * expected_ode_rps);
}

// REQ-DEV-10: Battery sag solves V_batt = V_oc - R_int * (I_base + sum I_sup)
// deterministically, converges with zero oscillation even when R_eq < R_int,
// trips and recovers auto-brownout, and respects SetSupplyVoltage /
// ClearSupplyVoltageOverride and R_int = 0 backward compatibility.
TEST(SimBatteryModel, SagMultiMotorStallBrownoutOverrideAndBackwardCompat) {
  // 1. Single stalled motor + base current satisfies V_oc - I_total * R_int.
  {
    Config cfg = SingleMotorConfig();
    cfg.motors[0].stator_limit_a = 80.0;
    cfg.motors[0].supply_limit_a = 80.0;
    PhysicsSimBackend backend;
    const sim::BatteryModel batt{.nominal_voltage_v = 12.6,
                                 .internal_resistance_ohm = 0.025,
                                 .base_current_a = 4.0,
                                 .brownout_voltage_v = 6.75};
    backend.SetBatteryModel(batt);
    ASSERT_TRUE(backend.Configure(cfg));

    // Unloaded resting voltage reflects base_current_a.
    State s{};
    uint64_t t = 0;
    StepBackend(backend, s, t, 5000, 1);
    EXPECT_NEAR(s.health.battery_v, 12.6 - 0.025 * 4.0, 1e-6);
    EXPECT_FALSE(s.health.brownout);

    // Under stall (dt = 0 after fresh Configure -> rotor at rest), bus voltage
    // satisfies V_batt = V_oc - R_int * (base + supply_current_a).
    ASSERT_TRUE(backend.Configure(cfg));
    std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 12.0, 0.0}}};
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, s, t, 5000, 1);
    const double expected_v =
        batt.nominal_voltage_v -
        batt.internal_resistance_ohm *
            (batt.base_current_a + s.motors[0].supply_current_a);
    EXPECT_LT(s.health.battery_v, 12.5);
    EXPECT_NEAR(s.health.battery_v, expected_v, 1e-5);
    EXPECT_FALSE(s.health.brownout);
  }

  // 2. Multi-motor stall where R_eq < R_int: 4 Kraken X60s stalled against
  // hard stops (R_eq = 12 / (4 * 366) = 0.00820 ohm < R_int = 0.020 ohm).
  // Exact analytical solution: V = V_oc * R_eq / (R_eq + R_int), zero
  // oscillation across ticks, auto-brownout trips and recovers on Neutral().
  {
    Config cfg;
    cfg.commissioned = true;
    cfg.command_timeout_us = 100000;
    std::vector<sim::HardStop> stops;
    for (int i = 0; i < 4; ++i) {
      auto m = TestMotor(static_cast<uint16_t>(i + 1),
                         ("m" + std::to_string(i)).c_str(), i + 1);
      m.stator_limit_a = 1000.0;
      m.supply_limit_a = 1000.0;
      cfg.motors.push_back(m);
      stops.push_back(
          {.motor = m.name.str(), .min_rot = -0.01, .max_rot = 0.0});
    }
    PhysicsSimBackend backend;
    backend.SetHardStops(stops);
    backend.SetBatteryModel({.nominal_voltage_v = 12.8,
                             .internal_resistance_ohm = 0.02,
                             .base_current_a = 0.0,
                             .brownout_voltage_v = 6.75});
    ASSERT_TRUE(backend.Configure(cfg));

    const double r_eq = (12.0 / 366.0) / 4.0;
    const double expected_stall_v = 12.8 * r_eq / (r_eq + 0.02);
    ASSERT_LT(r_eq, 0.02);
    ASSERT_LT(expected_stall_v, 6.75);

    std::array<MotorRequest, 4> cmd{};
    for (std::size_t i = 0; i < 4; ++i) {
      cmd[i] = {static_cast<uint16_t>(i + 1), Mode::kVoltage, 0, 12.0, 0.0};
    }
    ASSERT_TRUE(backend.Apply(cmd));
    State s{};
    uint64_t t = 0;
    StepBackend(backend, s, t, 5000, 1);  // dt = 0
    for (int step = 0; step < 15; ++step) {
      StepBackend(backend, s, t, 5000, 1);
      EXPECT_NEAR(s.health.battery_v, expected_stall_v, 1e-4)
          << "step " << step;
      EXPECT_TRUE(s.health.brownout) << "step " << step;
    }

    // Explicit SetSupplyVoltage overrides BatteryModel and clears
    // auto-brownout.
    backend.SetSupplyVoltage(11.5);
    StepBackend(backend, s, t, 5000, 1);
    EXPECT_DOUBLE_EQ(s.health.battery_v, 11.5);
    EXPECT_FALSE(s.health.brownout);

    // ClearSupplyVoltageOverride resumes BatteryModel -> sags and browns out.
    backend.ClearSupplyVoltageOverride();
    StepBackend(backend, s, t, 5000, 1);
    EXPECT_NEAR(s.health.battery_v, expected_stall_v, 1e-4);
    EXPECT_TRUE(s.health.brownout);

    // Neutral removes load -> battery recovers and brownout clears.
    backend.Neutral();
    StepBackend(backend, s, t, 5000, 1);
    EXPECT_DOUBLE_EQ(s.health.battery_v, 12.8);
    EXPECT_FALSE(s.health.brownout);
  }

  // 3. Backward compatibility: R_int = 0 with nominal = 12.8 is bit-for-bit
  // identical to never calling SetBatteryModel.
  {
    Config cfg = SingleMotorConfig();
    PhysicsSimBackend unset_backend;
    PhysicsSimBackend default_batt_backend;
    default_batt_backend.SetBatteryModel(sim::BatteryModel{});
    ASSERT_TRUE(unset_backend.Configure(cfg));
    ASSERT_TRUE(default_batt_backend.Configure(cfg));
    std::array<MotorRequest, 1> cmd{{{1, Mode::kVoltage, 0, 8.0, 0.0}}};
    ASSERT_TRUE(unset_backend.Apply(cmd));
    ASSERT_TRUE(default_batt_backend.Apply(cmd));
    State sa{}, sb{};
    uint64_t ta = 0, tb = 0;
    StepBackend(unset_backend, sa, ta, 5000, 50);
    StepBackend(default_batt_backend, sb, tb, 5000, 50);
    EXPECT_DOUBLE_EQ(sa.health.battery_v, sb.health.battery_v);
    EXPECT_EQ(sa.health.brownout, sb.health.brownout);
    EXPECT_DOUBLE_EQ(sa.motors[0].position_rot, sb.motors[0].position_rot);
    EXPECT_DOUBLE_EQ(sa.motors[0].velocity_rps, sb.motors[0].velocity_rps);
    EXPECT_DOUBLE_EQ(sa.motors[0].voltage, sb.motors[0].voltage);
    EXPECT_DOUBLE_EQ(sa.motors[0].stator_current_a,
                     sb.motors[0].stator_current_a);
    EXPECT_DOUBLE_EQ(sa.motors[0].supply_current_a,
                     sb.motors[0].supply_current_a);
  }
}

// REQ-DEV-10: Coupled 2D chassis dynamics limits acceleration by mass, scales
// angular acceleration by moment_of_inertia_kg_m2 (with mass * mean(r^2)
// default), produces physical wheel slip (encoder_speed > ground_speed) under
// traction_coefficient, and stays bit-for-bit compatible when omitted.
TEST(SimChassisDynamics, MassMoiTractionSlipAndBackwardCompat) {
  constexpr double kGear = 6.75;
  constexpr double kWheelRadius = sim::PhysicsOptions::kDefaultWheelRadiusM;
  constexpr double kStatorLimit = 60.0;
  const sim::MotorModel x60 = sim::MotorModelFor(MotorType::kKrakenX60);

  // 1. Mass acceleration limit & non-slipping wheel coupling (infinite
  // traction).
  {
    const Config cfg = FourCornerSwerveConfig(kGear, kStatorLimit, 400.0);
    auto run_mass = [&](double mass_kg) {
      PhysicsSimBackend b;
      b.SetChassisDynamics({.mass_kg = mass_kg});  // Survives Configure().
      EXPECT_TRUE(b.Configure(cfg));
      std::array<MotorRequest, kMaxMotors> cmd{};
      for (std::size_t i = 0; i < 4; ++i) {
        cmd[2 * i] = {static_cast<uint16_t>(2 * i + 1), Mode::kVoltage, 0, 12.0,
                      0.0};
      }
      EXPECT_TRUE(b.Apply(cmd));
      State s{};
      uint64_t t = 0;
      StepBackend(b, s, t, 5000, 1);   // dt = 0
      StepBackend(b, s, t, 5000, 10);  // 50 ms
      const double enc_speed =
          s.motors[0].velocity_rps * kTestTau * kWheelRadius;
      return std::pair{b.truth().vx_mps, enc_speed};
    };

    const auto [vx_40, enc_40] = run_mass(40.0);
    const auto [vx_80, enc_80] = run_mass(80.0);
    const double wheel_force =
        kGear * x60.torque_constant_nm_per_a() * kStatorLimit / kWheelRadius;
    const double expected_vx_40 = (4.0 * wheel_force / 40.0) * 0.05;
    EXPECT_NEAR(vx_40, expected_vx_40, 0.03 * expected_vx_40);
    EXPECT_NEAR(vx_80, 0.5 * vx_40, 0.02 * vx_40);
    // Infinite traction: no slip, wheel encoder speed equals ground speed.
    EXPECT_NEAR(enc_40, vx_40, 1e-6);
    EXPECT_NEAR(enc_80, vx_80, 1e-6);
  }

  // 2. Rotational MOI scaling and default mass * mean(x_i^2 + y_i^2).
  {
    const Config cfg = FourCornerSwerveConfig(kGear, kStatorLimit, 400.0);
    const std::array<double, 4> spin_angles{0.375, 0.125, -0.375, -0.125};
    auto run_spin = [&](double moi) {
      PhysicsSimBackend b;
      b.SetChassisDynamics({.mass_kg = 50.0, .moment_of_inertia_kg_m2 = moi});
      EXPECT_TRUE(b.Configure(cfg));
      std::array<MotorRequest, kMaxMotors> cmd{};
      for (std::size_t i = 0; i < 4; ++i) {
        cmd[2 * i + 1] = {static_cast<uint16_t>(2 * i + 2), Mode::kPosition, 0,
                          spin_angles[i], 0.0};
      }
      EXPECT_TRUE(b.Apply(cmd));
      State s{};
      uint64_t t = 0;
      StepBackend(b, s, t, 5000, 100);  // Steer modules into spin tangent.
      for (std::size_t i = 0; i < 4; ++i) {
        cmd[2 * i] = {static_cast<uint16_t>(2 * i + 1), Mode::kVoltage, 0, 12.0,
                      0.0};
      }
      EXPECT_TRUE(b.Apply(cmd));
      StepBackend(b, s, t, 5000, 10);  // 50 ms spin acceleration
      EXPECT_NEAR(s.sensors[0].position_rot, b.truth().yaw_rad / kTestTau,
                  1e-9);
      return b.truth().omega_radps;
    };

    // Default MOI for (±0.3, ±0.3): 50 * (0.09 + 0.09) = 9.0 kg*m^2.
    const double omega_default = run_spin(0.0);
    const double omega_explicit_9 = run_spin(9.0);
    const double omega_18 = run_spin(18.0);
    EXPECT_DOUBLE_EQ(omega_default, omega_explicit_9);
    EXPECT_GT(omega_default, 0.5);
    EXPECT_NEAR(omega_18, 0.5 * omega_default, 0.03 * omega_default);
  }

  // 3. Finite traction_coefficient caps ground acceleration to mu * g and
  // spins slipping wheels up on excess torque (encoder_speed > ground_speed).
  {
    const Config cfg = FourCornerSwerveConfig(kGear, 80.0, 400.0);
    constexpr double kMu = 0.4;
    constexpr double kG = 9.80665;
    PhysicsSimBackend backend;
    backend.SetChassisDynamics({.mass_kg = 50.0, .traction_coefficient = kMu});
    ASSERT_TRUE(backend.Configure(cfg));
    std::array<MotorRequest, kMaxMotors> cmd{};
    for (std::size_t i = 0; i < 4; ++i) {
      cmd[2 * i] = {static_cast<uint16_t>(2 * i + 1), Mode::kVoltage, 0, 12.0,
                    0.0};
    }
    ASSERT_TRUE(backend.Apply(cmd));
    State s{};
    uint64_t t = 0;
    StepBackend(backend, s, t, 5000, 1);   // dt = 0
    StepBackend(backend, s, t, 5000, 10);  // 50 ms
    const double ground_vx = backend.truth().vx_mps;
    const double encoder_vx =
        s.motors[0].velocity_rps * kTestTau * kWheelRadius;
    EXPECT_NEAR(ground_vx, kMu * kG * 0.05, 0.01);
    EXPECT_GT(encoder_vx, ground_vx * 2.0);

    // Lateral scrub: steer wheels 90 deg while moving in +x and neutral drive;
    // lateral friction decelerates +x smoothly without instability.
    for (std::size_t i = 0; i < 4; ++i) {
      cmd[2 * i] = {static_cast<uint16_t>(2 * i + 1), Mode::kNeutral, 0, 0.0,
                    0.0};
      cmd[2 * i + 1] = {static_cast<uint16_t>(2 * i + 2), Mode::kPosition, 0,
                        0.25, 0.0};
    }
    ASSERT_TRUE(backend.Apply(cmd));
    StepBackend(backend, s, t, 5000, 60);
    EXPECT_TRUE(std::isfinite(backend.truth().vx_mps));
    EXPECT_LT(std::abs(backend.truth().vx_mps), ground_vx);
  }

  // 4. Backward compatibility: omitted or mass_kg <= 0 stays bit-for-bit
  // identical to legacy kinematics.
  {
    const Config cfg = FourCornerSwerveConfig(kGear, 80.0, 400.0);
    PhysicsSimBackend unset_backend;
    PhysicsSimBackend zero_mass_backend;
    zero_mass_backend.SetChassisDynamics(sim::ChassisDynamics{.mass_kg = 0.0});
    ASSERT_TRUE(unset_backend.Configure(cfg));
    ASSERT_TRUE(zero_mass_backend.Configure(cfg));
    std::array<MotorRequest, kMaxMotors> cmd{};
    for (std::size_t i = 0; i < 4; ++i) {
      cmd[2 * i] = {static_cast<uint16_t>(2 * i + 1), Mode::kVoltage, 0, 6.0,
                    0.0};
      cmd[2 * i + 1] = {static_cast<uint16_t>(2 * i + 2), Mode::kPosition, 0,
                        0.1, 0.0};
    }
    ASSERT_TRUE(unset_backend.Apply(cmd));
    ASSERT_TRUE(zero_mass_backend.Apply(cmd));
    State sa{}, sb{};
    uint64_t ta = 0, tb = 0;
    StepBackend(unset_backend, sa, ta, 5000, 40);
    StepBackend(zero_mass_backend, sb, tb, 5000, 40);
    EXPECT_DOUBLE_EQ(unset_backend.truth().x_m, zero_mass_backend.truth().x_m);
    EXPECT_DOUBLE_EQ(unset_backend.truth().y_m, zero_mass_backend.truth().y_m);
    EXPECT_DOUBLE_EQ(unset_backend.truth().yaw_rad,
                     zero_mass_backend.truth().yaw_rad);
    EXPECT_DOUBLE_EQ(unset_backend.truth().vx_mps,
                     zero_mass_backend.truth().vx_mps);
    EXPECT_DOUBLE_EQ(unset_backend.truth().vy_mps,
                     zero_mass_backend.truth().vy_mps);
    EXPECT_DOUBLE_EQ(unset_backend.truth().omega_radps,
                     zero_mass_backend.truth().omega_radps);
  }
}

// An external plant (sim_plant.h) that integrates nothing: each bound
// motor's rotor stays where the test puts it, so a Read's drive and its
// reports are at that tick-start speed. It records what the backend hands it.
class FakePlant final : public sim::ExternalPlant {
 public:
  uint32_t step_us() const override { return step; }
  bool Bind(std::span<const sim::PlantMotor> motors,
            std::span<const sim::PlantInput> inputs, std::span<bool> owned,
            ConfigError& error) override {
    CountCall();
    if (refusal) {
      error = *refusal;
      return false;
    }
    for (std::size_t i = 0; i < motors.size(); ++i) {
      owned[i] =
          std::find(binds.begin(), binds.end(), motors[i].name) != binds.end();
    }
    sensors.clear();
    for (const auto& input : inputs) sensors.emplace_back(input.sensor);
    return true;
  }
  bool Step(int steps, std::span<const sim::PlantDrive> drives) override {
    CountCall();
    step_log.push_back(steps);
    last_drives.assign(drives.begin(), drives.end());
    if (rps_after_step) {
      for (auto& rotor : rotors) rotor.velocity_rps = *rps_after_step;
    }
    if (chassis_after_step) chassis_state = chassis_after_step;
    if (fail_next_step) {
      fail_next_step = false;
      failed = true;
      return false;
    }
    return true;
  }
  sim::PlantRotor rotor(std::size_t motor) const override {
    CountCall();
    return rotors[motor];
  }
  std::optional<sim::PlantChassis> chassis() const override {
    CountCall();
    return chassis_state;
  }
  bool input_closed(std::size_t input) const override {
    CountCall();
    return closed[input];
  }
  std::string_view failure() const override { return "BADQACC at 1.2 s"; }
  void CountCall() const {
    if (failed) ++calls_after_failure;
  }

  uint32_t step{5000};
  std::vector<std::string_view> binds;
  // Bind returns false with this, when set.
  std::optional<ConfigError> refusal;
  bool fail_next_step{false};
  bool failed{false};
  mutable int calls_after_failure{0};
  // The chassis once a Step returns, when set.
  std::optional<sim::PlantChassis> chassis_after_step;
  // Every rotor's speed once a Step returns, when set.
  std::optional<double> rps_after_step;
  std::array<sim::PlantRotor, kMaxMotors> rotors{};
  std::optional<sim::PlantChassis> chassis_state;
  std::array<bool, 4> closed{};
  std::vector<std::string> sensors;
  std::vector<int> step_log;
  std::vector<sim::PlantDrive> last_drives;
};

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTauRad = 2.0 * std::numbers::pi;

// The torque the plant law (sim_plant.h) gives at rotor speed `rps`, less
// the no-load drag both models add on their own.
double DriveTorqueNm(const sim::PlantDrive& drive, double rps,
                     MotorType type = MotorType::kKrakenX60) {
  const sim::MotorModel motor = sim::MotorModelFor(type);
  const double emf =
      drive.back_emf ? motor.back_emf_v_per_radps() * rps * kTauRad : 0.0;
  return std::clamp(motor.torque_constant_nm_per_a() / motor.resistance_ohm() *
                        (drive.volts - emf),
                    drive.min_torque_nm, drive.max_torque_nm);
}

void ExpectBound(double actual, double expected, const char* what) {
  if (std::isinf(expected)) {
    EXPECT_EQ(actual, expected) << what;
  } else {
    EXPECT_NEAR(actual, expected, 1e-9) << what;
  }
}

void AddFollower(Config& c, bool oppose) {
  auto f = TestMotor(2, "roller_follower", 2);
  f.master_id = 1;
  f.oppose_master = oppose;
  c.motors.push_back(f);
  for (auto& m : c.motors) m.stator_limit_a = m.supply_limit_a = 0.0;
}

void AddSwerveModule(Config& c) {
  c.motors.push_back(TestMotor(2, "front_left_drive", 2));
  c.motors.push_back(TestMotor(3, "front_left_steer", 3));
}

// REQ-DEV-37: with a plant, each Read steps it the nearest whole number of
// its steps to the sample time not yet stepped, at most
// 2 * ceil(period_us / step_us); time past that cap is dropped and counted,
// never caught up. Everything the backend models advances by the same plant
// time (here the Pigeon's drift), and every motion sample is as old as the
// plant lags the sample time. Floor stepping would give 0 then 2 steps under
// 4900/5100 µs jitter.
TEST(SimPlant, StepsTheNearestWholeStepsAndDropsTimePastTheCap) {
  struct Case {
    const char* name;
    std::vector<uint64_t> ticks_us;
    std::vector<int> steps;
    std::vector<int32_t> lag_us;
    uint64_t dropped_us;
    uint64_t dropped_ticks;
  };
  std::vector<Case> cases{
      {"catch-up past the cap",
       {0, 4900, 10100, 15000, 33000},
       {0, 1, 1, 1, 2},
       {0, -100, 100, 0, -2000},
       10000,
       1},
      {"jitter", {0}, {0}, {0}, 0, 0},
  };
  for (int k = 0; k < 10; ++k) {
    Case& jitter = cases.back();
    jitter.ticks_us.push_back(jitter.ticks_us.back() +
                              (k % 2 != 0 ? 5100 : 4900));
    jitter.steps.push_back(1);
    jitter.lag_us.push_back(k % 2 != 0 ? 0 : -100);
  }
  constexpr double kDriftRadps = 0.1;
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    Config cfg = SingleMotorConfig();
    cfg.period_us = 5000;
    SensorConfig pigeon;
    pigeon.id = 20;
    pigeon.name = "gyro";
    pigeon.kind = SensorKind::kPigeon2;
    pigeon.can_id = 20;
    cfg.sensors.push_back(pigeon);
    FakePlant plant;
    plant.binds = {"roller"};
    PhysicsSimBackend backend({.gyro_drift_radps = kDriftRadps});
    backend.SetPlant(&plant);
    ASSERT_TRUE(backend.Configure(cfg));
    EXPECT_EQ(backend.plant_time().max_steps, 2u);
    State state{};
    for (std::size_t t = 0; t < c.ticks_us.size(); ++t) {
      SCOPED_TRACE(testing::Message() << "tick at " << c.ticks_us[t] << " us");
      state.sample_time_us = c.ticks_us[t];
      ASSERT_TRUE(backend.Read(state));
      ASSERT_EQ(plant.step_log.size(), t + 1);
      EXPECT_EQ(plant.step_log.back(), c.steps[t]);
      EXPECT_EQ(backend.plant_time().lag_us, c.lag_us[t]);
      const auto age_us =
          static_cast<uint32_t>(std::max<int32_t>(c.lag_us[t], 0));
      EXPECT_EQ(state.motors[0].position_age_us, age_us);
      EXPECT_EQ(state.motors[0].velocity_age_us, age_us);
      EXPECT_EQ(state.sensors[0].position_age_us, age_us);
    }
    const auto& time = backend.plant_time();
    EXPECT_EQ(time.dropped_us, c.dropped_us);
    EXPECT_EQ(time.dropped_ticks, c.dropped_ticks);
    EXPECT_EQ(
        static_cast<int64_t>(time.stepped_us + time.dropped_us) + time.lag_us,
        static_cast<int64_t>(c.ticks_us.back()));
    EXPECT_NEAR(
        state.sensors[0].position_rot,
        kDriftRadps * static_cast<double>(time.stepped_us) * 1e-6 / kTauRad,
        1e-12);
  }
  // A step longer than the period still allows two steps a tick: 32-bit
  // arithmetic would wrap period + step - 1 and allow none.
  FakePlant plant;
  plant.step = 4294967294u;
  PhysicsSimBackend backend;
  backend.SetPlant(&plant);
  ASSERT_TRUE(backend.Configure(SingleMotorConfig()));
  EXPECT_EQ(backend.plant_time().max_steps, 2u);
}

// REQ-DEV-37: the drive a bound motor's winding gets gives, at the
// tick-start rotor speed, the torque k_t times the stator current the
// analytic winding model gives there, in every control mode, and the State
// reports that current. Each expected current is worked from the motor's
// constants. Catches a sign or direction error, 0 A torque current taken as
// neutral, brake and coast swapped, and the supply limit judged before the
// stator band (the drive's held volts).
TEST(SimPlant, DriveTorqueIsTheWindingModelsInEveryMode) {
  const sim::MotorModel x60 = sim::MotorModelFor(MotorType::kKrakenX60);
  const double r_ohm = x60.resistance_ohm();
  const double kt = x60.torque_constant_nm_per_a();
  const double w = 30.0;  // rotor rps
  const double emf = x60.back_emf_v_per_rps() * w;
  const double bus = kSimRestingSupplyV;
  const double brake_nm = -kt * x60.back_emf_v_per_radps() / r_ohm *
                          (w * kTauRad);  // shorted winding
  const double supply_v =
      0.5 * (emf + std::sqrt(emf * emf + 4.0 * 30.0 * bus * r_ohm));
  const auto amps = [&](double rotor_volts, double rotor_rps) {
    return (rotor_volts - x60.back_emf_v_per_rps() * rotor_rps) / r_ohm;
  };
  struct Motor {
    double rps;
    sim::PlantDrive drive;
    double torque_nm;
    double stator_a;
  };
  // Motor 0's drive torque at another speed within the tick.
  struct Probe {
    double rps;
    double torque_nm;
    std::size_t motor{0};
  };
  struct Case {
    const char* name;
    void (*arrange)(Config&);
    std::vector<MotorRequest> commands;
    std::vector<Motor> motors;
    std::vector<Probe> probes{};
    // A motor's reported voltage at its tick-start speed.
    std::vector<std::pair<std::size_t, double>> reported_volts{};
  };
  const auto voltage = [](uint16_t id, double v) {
    return MotorRequest{id, Mode::kVoltage, 0, v, 0.0};
  };
  const auto torque_current = [](double a) {
    return MotorRequest{1, Mode::kTorqueCurrent, 0, a, 0.0};
  };
  const auto unlimited = [](Config& c) {
    c.motors[0].stator_limit_a = c.motors[0].supply_limit_a = 0.0;
  };
  const auto tc = [](Config& c) {
    c.motors[0].foc = true;
    c.motors[0].max_torque_current_a = 60.0;
  };
  const double i6 = amps(6.0, w);
  // Supply-limited winding volts at rotor speed `rps` for supply_limit_a.
  const auto supply_held = [&](double limit_a, double rps) {
    const double e = x60.back_emf_v_per_rps() * rps;
    return 0.5 * (e + std::sqrt(e * e + 4.0 * limit_a * bus * r_ohm));
  };
  const double held_tc = supply_held(10.0, 80.0);
  const double held_follower = supply_held(30.0, 10.0);
  // The X60 master's stator-banded volts at 30 rps, which an X44 follower
  // holds; and its current through the X44's own winding.
  const sim::MotorModel x44 = sim::MotorModelFor(MotorType::kKrakenX44);
  const double banded_30 = emf + 40.0 * r_ohm;
  const double x44_a =
      (banded_30 - x44.back_emf_v_per_rps() * w) / x44.resistance_ohm();
  const double i_opposed = amps(-40.0 * r_ohm, -1.0);
  const std::vector<Case> cases{
      {"duty cycle",
       unlimited,
       {Duty(1, 0.5)},
       {{w,
         {0.5 * bus, true, -kInf, kInf},
         kt * amps(0.5 * bus, w),
         std::abs(amps(0.5 * bus, w))}}},
      {"voltage, inverted",
       [](Config& c) {
         c.motors[0].inverted = true;
         c.motors[0].stator_limit_a = c.motors[0].supply_limit_a = 0.0;
       },
       {voltage(1, 6.0)},
       {{-w, {-6.0, true, -kInf, kInf}, -kt * i6, i6}}},
      {"velocity",
       [](Config& c) {
         c.motors[0].slots[0].p = 0.2;
         c.motors[0].slots[0].v = 0.1;
         c.motors[0].stator_limit_a = c.motors[0].supply_limit_a = 0.0;
       },
       {{1, Mode::kVelocity, 0, 40.0, 0.0}},  // 0.1 * 40 + 0.2 * (40 - 30)
       {{w, {6.0, true, -kInf, kInf}, kt * i6, i6}}},
      {"motion magic",
       unlimited,
       {{1, Mode::kMotionMagic, 0, 5.0, 2.0}},  // zero gains: feedforward
       {{w,
         {2.0, true, -kInf, kInf},
         kt * amps(2.0, w),
         std::abs(amps(2.0, w))}}},
      {"remote CANcoder, inverted",
       [](Config& c) {
         SensorConfig s;
         s.id = 10;
         s.name = "roller_encoder";
         s.can_id = 10;
         s.inverted = true;  // reads -30 rps: error +30
         c.sensors.push_back(s);
         c.motors[0].feedback = Feedback::kRemoteCANcoder;
         c.motors[0].feedback_sensor_id = 10;
         c.motors[0].slots[0].p = 0.2;
         c.motors[0].stator_limit_a = c.motors[0].supply_limit_a = 0.0;
       },
       {{1, Mode::kVelocity, 0, 0.0, 0.0}},
       {{w, {6.0, true, -kInf, kInf}, kt * i6, i6}}},
      {"torque current forward",
       tc,
       {torque_current(20.0)},
       {{w, {12.0, true, -kt * 80.0, kt * 20.0}, kt * 20.0, 20.0}}},
      {"torque current reverse",
       tc,
       {torque_current(-20.0)},
       {{w, {-12.0, true, -kt * 20.0, kt * 80.0}, -kt * 20.0, 20.0}}},
      {"torque current, inverted",
       [](Config& c) {
         c.motors[0].foc = true;
         c.motors[0].max_torque_current_a = 60.0;
         c.motors[0].inverted = true;
       },
       {torque_current(20.0)},
       {{-w, {-12.0, true, -kt * 20.0, kt * 80.0}, -kt * 20.0, 20.0}}},
      {"torque current 0 A turning",
       tc,
       {torque_current(0.0)},
       {{w, {12.0, true, -kt * 80.0, 0.0}, 0.0, 0.0}}},
      {"torque current, max_voltage below the supply",
       [](Config& c) {
         c.motors[0].foc = true;
         c.motors[0].max_torque_current_a = 60.0;
         c.motors[0].max_voltage = 8.0;
       },
       {torque_current(20.0)},
       {{w, {8.0, true, -kt * 80.0, kt * 20.0}, kt * 20.0, 20.0}}},
      // Held 9.97 V passes 12.8 A at 80 rps; slowing within the tick must
      // not take it past the 20 A demanded.
      {"torque current, supply limited",
       [](Config& c) {
         c.motors[0].foc = true;
         c.motors[0].max_torque_current_a = 60.0;
         c.motors[0].stator_limit_a = 60.0;
         c.motors[0].supply_limit_a = 10.0;
       },
       {torque_current(20.0)},
       {{80.0,
         {held_tc, true, -kt * 60.0, kt * 20.0},
         kt * amps(held_tc, 80.0),
         amps(held_tc, 80.0)}},
       {{70.0, kt * 20.0}, {0.0, kt * 20.0}}},
      // Back-driven past the opposite ceiling the controller saturates at
      // -12 V and carries more than the demand (34.4 A), up to the stator
      // limit; once the rotor slows out of it, the demand again.
      {"torque current, back-driven past the opposite ceiling",
       [](Config& c) {
         c.motors[0].foc = true;
         c.motors[0].max_torque_current_a = 60.0;
         c.motors[0].stator_limit_a = 40.0;
       },
       {torque_current(10.0)},
       {{-110.0,
         {-12.0, true, kt * 10.0, kt * 40.0},
         kt * amps(-12.0, -110.0),
         amps(-12.0, -110.0)}},
       {{-95.0, kt * 10.0}, {-140.0, kt * 40.0}}},
      {"torque current reverse, driven past the ceiling",
       [](Config& c) {
         c.motors[0].foc = true;
         c.motors[0].max_torque_current_a = 60.0;
         c.motors[0].stator_limit_a = 40.0;
       },
       {torque_current(-10.0)},
       {{110.0,
         {12.0, true, -kt * 40.0, -kt * 10.0},
         kt * amps(12.0, 110.0),
         -amps(12.0, 110.0)}},
       {{95.0, -kt * 10.0}, {140.0, -kt * 40.0}}},
      // The same two, inverted: +10 A asks for physical -10 A.
      {"torque current, inverted, back-driven past the opposite ceiling",
       [](Config& c) {
         c.motors[0].foc = true;
         c.motors[0].max_torque_current_a = 60.0;
         c.motors[0].stator_limit_a = 40.0;
         c.motors[0].inverted = true;
       },
       {torque_current(10.0)},
       {{110.0,
         {12.0, true, -kt * 40.0, -kt * 10.0},
         kt * amps(12.0, 110.0),
         -amps(12.0, 110.0)}},
       {{95.0, -kt * 10.0}, {140.0, -kt * 40.0}}},
      {"torque current reverse, inverted, driven past the ceiling",
       [](Config& c) {
         c.motors[0].foc = true;
         c.motors[0].max_torque_current_a = 60.0;
         c.motors[0].stator_limit_a = 40.0;
         c.motors[0].inverted = true;
       },
       {torque_current(-10.0)},
       {{-110.0,
         {-12.0, true, kt * 10.0, kt * 40.0},
         kt * amps(-12.0, -110.0),
         amps(-12.0, -110.0)}},
       {{-95.0, kt * 10.0}, {-140.0, kt * 40.0}}},
      {"torque current 0 A at rest",
       tc,
       {torque_current(0.0)},
       {{0.0, {0.0, true, 0.0, 0.0}, 0.0, 0.0}}},
      {"brake neutral",
       [](Config& c) { c.motors[0].brake = true; },
       {MotorRequest{.id = 1}},
       {{w, {0.0, true, -kInf, kInf}, brake_nm, 0.0}}},
      {"coast neutral",
       [](Config& c) { c.motors[0].brake = false; },
       {MotorRequest{.id = 1}},
       {{w, {0.0, false, -kInf, kInf}, 0.0, 0.0}}},
      {"stator limit",
       [](Config& c) {
         c.motors[0].stator_limit_a = 40.0;
         c.motors[0].supply_limit_a = 0.0;
       },
       {voltage(1, 12.0)},
       {{w, {12.0, true, -kt * 40.0, kt * 40.0}, kt * 40.0, 40.0}}},
      {"supply limit",
       [](Config& c) {
         c.motors[0].stator_limit_a = 0.0;
         c.motors[0].supply_limit_a = 30.0;
       },
       {voltage(1, 12.0)},
       {{w,
         {supply_v, true, -kInf, kInf},
         kt * amps(supply_v, w),
         amps(supply_v, w)}}},
      {"stator band before the supply limit",
       [](Config& c) {
         c.motors[0].stator_limit_a = 40.0;
         c.motors[0].supply_limit_a = 30.0;
       },
       {voltage(1, 12.0)},
       {{w, {12.0, true, -kt * 40.0, kt * 40.0}, kt * 40.0, 40.0}}},
      {"follower aligned",
       [](Config& c) { AddFollower(c, false); },
       {voltage(1, 6.0), voltage(2, 0.0)},
       {{w, {6.0, true, -kInf, kInf}, kt * i6, i6},
        {w, {6.0, true, -kInf, kInf}, kt * i6, i6}}},
      {"follower opposed",
       [](Config& c) { AddFollower(c, true); },
       {voltage(1, 6.0), voltage(2, 0.0)},
       {{w, {6.0, true, -kInf, kInf}, kt * i6, i6},
        {-w, {-6.0, true, -kInf, kInf}, -kt * i6, i6}}},
      // A follower holds its master's regime (here the stator limit), not
      // the master's tick-start banded volts: speeding up within the tick
      // keeps both at 40 A.
      {"follower of a stator-limited master",
       [](Config& c) {
         AddFollower(c, false);
         for (auto& m : c.motors) m.stator_limit_a = 40.0;
       },
       {voltage(1, 12.0), voltage(2, 0.0)},
       {{w, {12.0, true, -kt * 40.0, kt * 40.0}, kt * 40.0, 40.0},
        {w, {12.0, true, -kt * 40.0, kt * 40.0}, kt * 40.0, 40.0}},
       {{40.0, kt * 40.0, 0}, {40.0, kt * 40.0, 1}}},
      {"opposed follower of a stator-limited master",
       [](Config& c) {
         AddFollower(c, true);
         for (auto& m : c.motors) m.stator_limit_a = 40.0;
       },
       {voltage(1, 12.0), voltage(2, 0.0)},
       {{w, {12.0, true, -kt * 40.0, kt * 40.0}, kt * 40.0, 40.0},
        {-w, {-12.0, true, -kt * 40.0, kt * 40.0}, -kt * 40.0, 40.0}},
       {{-40.0, -kt * 40.0, 1}}},
      // On its own joint the follower takes what the master applies
      // (1.31 V, the master stalled against its stator limit) at its own
      // 30 rps: 2.27 V, braking at -40 A, as the analytic model gives; the
      // master's 12 V regime would push +40 A.
      {"follower on another shaft, master stalled",
       [](Config& c) {
         AddFollower(c, false);
         for (auto& m : c.motors) m.stator_limit_a = 40.0;
       },
       {voltage(1, 12.0), voltage(2, 0.0)},
       {{0.0, {12.0, true, -kt * 40.0, kt * 40.0}, kt * 40.0, 40.0},
        {w, {40.0 * r_ohm, true, -kt * 40.0, kt * 40.0}, -kt * 40.0, 40.0}},
       {{35.0, -kt * 40.0, 1}},
       {{1, -40.0 * r_ohm + emf}}},
      // An opposed follower ignores its own inverted; at -1 rps it carries
      // -36.4 A of the master's -1.31 V, inside its band.
      {"opposed, inverted follower at another speed",
       [](Config& c) {
         AddFollower(c, true);
         c.motors[1].inverted = true;
         for (auto& m : c.motors) m.stator_limit_a = 40.0;
       },
       {voltage(1, 12.0), voltage(2, 0.0)},
       {{0.0, {12.0, true, -kt * 40.0, kt * 40.0}, kt * 40.0, 40.0},
        {-1.0,
         {-40.0 * r_ohm, true, -kt * 40.0, kt * 40.0},
         kt * i_opposed,
         std::abs(i_opposed)}}},
      // An X44 follower with no stator limit of its own takes no bound from
      // the X60 master's: it holds the master's banded volts.
      {"follower of another motor type, no stator limit of its own",
       [](Config& c) {
         AddFollower(c, false);
         c.motors[0].stator_limit_a = 40.0;
         c.motors[1].motor_type.value = MotorType::kKrakenX44;
       },
       {voltage(1, 12.0), voltage(2, 0.0)},
       {{w, {12.0, true, -kt * 40.0, kt * 40.0}, kt * 40.0, 40.0},
        {w,
         {banded_30, true, -kInf, kInf},
         x44.torque_constant_nm_per_a() * x44_a,
         x44_a}}},
      // Its master's applied volts, limited at its own speed (10 rps).
      {"follower at its own speed, supply limited",
       [](Config& c) {
         AddFollower(c, false);
         c.motors[1].supply_limit_a = 30.0;
       },
       {voltage(1, 6.0), voltage(2, 0.0)},
       {{w, {6.0, true, -kInf, kInf}, kt * i6, i6},
        {10.0,
         {held_follower, true, -kInf, kInf},
         kt * amps(held_follower, 10.0),
         amps(held_follower, 10.0)}}},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    Config cfg = SingleMotorConfig();
    c.arrange(cfg);
    FakePlant plant;
    plant.binds = {"roller", "roller_follower"};
    for (std::size_t i = 0; i < c.motors.size(); ++i) {
      plant.rotors[i] = {0.0, c.motors[i].rps};
    }
    PhysicsSimBackend backend;
    backend.SetPlant(&plant);
    ASSERT_TRUE(backend.Configure(cfg));
    ASSERT_TRUE(backend.Apply(c.commands));
    State state{};
    ASSERT_TRUE(backend.Read(state));
    ASSERT_EQ(plant.last_drives.size(), c.motors.size());
    for (std::size_t i = 0; i < c.motors.size(); ++i) {
      SCOPED_TRACE(testing::Message() << "motor " << i);
      const Motor& want = c.motors[i];
      const sim::PlantDrive& drive = plant.last_drives[i];
      EXPECT_NEAR(drive.volts, want.drive.volts, 1e-9);
      EXPECT_EQ(drive.back_emf, want.drive.back_emf);
      ExpectBound(drive.min_torque_nm, want.drive.min_torque_nm, "min");
      ExpectBound(drive.max_torque_nm, want.drive.max_torque_nm, "max");
      EXPECT_NEAR(
          DriveTorqueNm(drive, want.rps, cfg.motors[i].motor_type.value),
          want.torque_nm, 1e-9);
      EXPECT_NEAR(state.motors[i].stator_current_a, want.stator_a, 1e-9);
    }
    for (const Probe& probe : c.probes) {
      EXPECT_NEAR(DriveTorqueNm(plant.last_drives[probe.motor], probe.rps,
                                cfg.motors[probe.motor].motor_type.value),
                  probe.torque_nm, 1e-9)
          << "motor " << probe.motor << " at " << probe.rps << " rps";
    }
    for (const auto& [motor, volts] : c.reported_volts) {
      EXPECT_NEAR(state.motors[motor].voltage, volts, 1e-9)
          << "motor " << motor;
    }
  }
}

// REQ-DEV-37: a bound motor reports the voltage and currents of the drive
// the plant integrated, at the speed the plant reached: a 12 V drive whose
// stator limit bound at 30 rps draws only (12 V - back-EMF) / R at 100 rps,
// at the full 12 V. Where the drive caps regeneration at the stator limit
// (back-EMF beyond the ceiling plus 40 A * R, at 150 rps), the controller
// is at its ceiling: the voltage reads the ceiling, the stator current the
// cap, the supply current their power over the bus. Catches reports left
// at the tick-start speed's banded voltage, and a voltage back-solved past
// the ceiling (16.6 V).
TEST(SimPlant, ReportsTheDriveAtThePostStepSpeed) {
  const sim::MotorModel x60 = sim::MotorModelFor(MotorType::kKrakenX60);
  const double bus = kSimRestingSupplyV;
  const double free_a =
      (12.0 - x60.back_emf_v_per_rps() * 100.0) / x60.resistance_ohm();
  struct Case {
    const char* name;
    MotorRequest command;
    double rps_before;
    double rps_after;
    double voltage;
    double stator_a;
    double supply_a;
    double supply_limit_a{0.0};
  };
  const std::vector<Case> cases{
      {"stator limit released",
       {1, Mode::kVoltage, 0, 12.0, 0.0},
       30.0,
       100.0,
       12.0,
       free_a,
       12.0 * free_a / bus},
      {"regeneration capped",
       {1, Mode::kVoltage, 0, 12.0, 0.0},
       150.0,
       150.0,
       12.0,
       40.0,
       -12.0 * 40.0 / bus},
      {"regeneration capped, reverse",
       {1, Mode::kVoltage, 0, -12.0, 0.0},
       -150.0,
       -150.0,
       -12.0,
       40.0,
       -12.0 * 40.0 / bus},
      // Regeneration is never clamped by supply_limit_a.
      {"regeneration capped, supply limit set",
       {1, Mode::kVoltage, 0, 12.0, 0.0},
       150.0,
       150.0,
       12.0,
       40.0,
       -12.0 * 40.0 / bus,
       30.0},
      {"torque current back-driven past the stator limit",
       {1, Mode::kTorqueCurrent, 0, 10.0, 0.0},
       -110.0,
       -150.0,
       -12.0,
       40.0,
       -12.0 * 40.0 / bus},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    Config cfg = SingleMotorConfig();
    cfg.motors[0].stator_limit_a = 40.0;
    cfg.motors[0].supply_limit_a = c.supply_limit_a;
    cfg.motors[0].foc = true;
    cfg.motors[0].max_torque_current_a = 40.0;
    FakePlant plant;
    plant.binds = {"roller"};
    plant.rotors[0] = {0.0, c.rps_before};
    plant.rps_after_step = c.rps_after;
    PhysicsSimBackend backend;
    backend.SetPlant(&plant);
    ASSERT_TRUE(backend.Configure(cfg));
    ASSERT_TRUE(backend.Apply(std::array{c.command}));
    State state{};
    ASSERT_TRUE(backend.Read(state));
    EXPECT_LE(std::abs(state.motors[0].voltage), 12.0);
    EXPECT_NEAR(state.motors[0].voltage, c.voltage, 1e-9);
    EXPECT_NEAR(state.motors[0].stator_current_a, c.stator_a, 1e-9);
    EXPECT_NEAR(state.motors[0].supply_current_a, c.supply_a, 1e-9);
  }
}

// REQ-DEV-37: bound motors draw from the battery as analytic ones do: two
// bound motors sag the solved supply exactly as two analytic motors at the
// same speeds do, stalled and turning, whatever speed the plant reaches in
// the step. Catches bound motors left out of the sag sum, or summed at a
// speed other than their tick-start speed (such as the post-step speed).
TEST(SimPlant, BoundMotorsSagTheBatteryAsAnalyticOnes) {
  Config cfg = SingleMotorConfig();
  cfg.motors.push_back(TestMotor(2, "intake", 2));
  const std::array commands{MotorRequest{1, Mode::kVoltage, 0, 12.0, 0.0},
                            MotorRequest{2, Mode::kVoltage, 0, 6.0, 0.0}};
  for (const int ticks : {0, 40}) {
    SCOPED_TRACE(testing::Message() << ticks << " ticks spun up");
    // Analytic: spun up for `ticks`, then the sag of the next tick, solved
    // at the speeds the last Read reported.
    PhysicsSimBackend analytic;
    analytic.SetBatteryModel({.internal_resistance_ohm = 0.05});
    ASSERT_TRUE(analytic.Configure(cfg));
    ASSERT_TRUE(analytic.Apply(commands));
    State state{};
    uint64_t now_us = 0;
    ASSERT_TRUE(analytic.Read(state));
    StepBackend(analytic, state, now_us, 5000, ticks);
    FakePlant plant;
    plant.binds = {"roller", "intake"};
    for (std::size_t i = 0; i < 2; ++i) {
      plant.rotors[i] = {0.0, state.motors[i].velocity_rps};  // gear 1
    }
    plant.rps_after_step = ticks == 0 ? 50.0 : 0.0;
    StepBackend(analytic, state, now_us, 5000, 1);
    ASSERT_LT(analytic.supply_voltage(), kSimRestingSupplyV - 0.5);
    PhysicsSimBackend bound;
    bound.SetBatteryModel({.internal_resistance_ohm = 0.05});
    bound.SetPlant(&plant);
    ASSERT_TRUE(bound.Configure(cfg));
    ASSERT_TRUE(bound.Apply(commands));
    State bound_state{};
    ASSERT_TRUE(bound.Read(bound_state));
    EXPECT_DOUBLE_EQ(bound.supply_voltage(), analytic.supply_voltage());
  }
  // Regeneration capped at 40 A at 150 rps, against a 100 A base load: the
  // ceiling is the sagged supply V itself, so the motor returns
  // V * 40 A / V = 40 A and V = 12.8 - 0.05 * (100 - 40) = 9.8 V. The
  // uncapped winding would return about 250 A.
  Config regen = SingleMotorConfig();
  regen.motors[0].stator_limit_a = 40.0;
  regen.motors[0].supply_limit_a = 0.0;
  FakePlant plant;
  plant.binds = {"roller"};
  plant.rotors[0] = {0.0, 150.0};
  PhysicsSimBackend backend;
  backend.SetBatteryModel(
      {.internal_resistance_ohm = 0.05, .base_current_a = 100.0});
  backend.SetPlant(&plant);
  ASSERT_TRUE(backend.Configure(regen));
  ASSERT_TRUE(
      backend.Apply(std::array{MotorRequest{1, Mode::kVoltage, 0, 12.0, 0.0}}));
  State state{};
  ASSERT_TRUE(backend.Read(state));
  EXPECT_NEAR(backend.supply_voltage(), 12.8 - 0.05 * (100.0 - 40.0), 1e-6);
}

// REQ-DEV-37: a motor no plant actuator binds behaves as under the analytic
// backend alone: with motor A bound and B not, B's samples equal those of a
// backend with no plant running the same config, commands and whole-step
// tick times, every tick. With an ideal battery the supply is 12.8 V
// whatever A draws, and B is equal bit for bit. With internal resistance A
// stalls on both sides (a hard stop in the analytic backend, the plant
// holding it at 0 rps) and draws its 80 A stator limit, so A's share of the
// sag reaches B; B is then equal within 1e-6, because the plant path forms
// A's current as R * (V / R), not bit-equal to V, and the 28-step sag
// bisection can settle one step (12.8 V / 2^28) apart.
TEST(SimPlant, AnUnboundMotorBehavesAsWithoutAPlant) {
  for (const double r_int : {0.0, 0.05}) {
    SCOPED_TRACE(testing::Message() << "battery " << r_int << " ohm");
    Config cfg = SingleMotorConfig();
    cfg.motors.push_back(TestMotor(2, "intake", 2));
    const std::array commands{MotorRequest{1, Mode::kVoltage, 0, 3.0, 0.0},
                              MotorRequest{2, Mode::kVoltage, 0, 6.0, 0.0}};
    FakePlant plant;
    plant.binds = {"roller"};
    plant.rotors[0] = {0.0, r_int > 0.0 ? 0.0 : 20.0};
    PhysicsSimBackend with_plant;
    with_plant.SetPlant(&plant);
    PhysicsSimBackend analytic;
    if (r_int > 0.0) analytic.SetHardStops({{"roller", std::nullopt, 0.0}});
    for (PhysicsSimBackend* backend : {&with_plant, &analytic}) {
      backend->SetBatteryModel({.internal_resistance_ohm = r_int});
      ASSERT_TRUE(backend->Configure(cfg));
      ASSERT_TRUE(backend->Apply(commands));
    }
    State a{}, b{};
    for (uint64_t now_us = 0; now_us <= 100000; now_us += 5000) {
      SCOPED_TRACE(testing::Message() << "tick at " << now_us << " us");
      a.sample_time_us = b.sample_time_us = now_us;
      ASSERT_TRUE(with_plant.Read(a));
      ASSERT_TRUE(analytic.Read(b));
      if (r_int == 0.0) {
        EXPECT_EQ(a.motors[1], b.motors[1]);
        continue;
      }
      ASSERT_EQ(b.motors[0].velocity_rps, 0.0);  // A stalled
      EXPECT_NEAR(a.motors[1].position_rot, b.motors[1].position_rot, 1e-6);
      EXPECT_NEAR(a.motors[1].velocity_rps, b.motors[1].velocity_rps, 1e-6);
      EXPECT_NEAR(a.motors[1].voltage, b.motors[1].voltage, 1e-6);
      EXPECT_NEAR(a.motors[1].stator_current_a, b.motors[1].stator_current_a,
                  1e-6);
      EXPECT_NEAR(a.motors[1].supply_current_a, b.motors[1].supply_current_a,
                  1e-6);
    }
    EXPECT_GT(b.motors[1].velocity_rps, 1.0);
    if (r_int > 0.0) {
      EXPECT_LT(analytic.supply_voltage(), kSimRestingSupplyV - 0.5);
    }
  }
}

// REQ-DEV-37: what a bound motor or a model-owned chassis cannot keep is
// refused at Configure, naming the config key and the motor, and the plant
// is then not stepped.
TEST(SimPlant, ConfigureRefusesWhatTheModelOwnsNamingTheKey) {
  struct Case {
    const char* name;
    void (*arrange)(Config&, PhysicsSimBackend&, FakePlant&);
    const char* device;
    const char* key;
  };
  const std::vector<Case> cases{
      {"partly bound follower chain",
       [](Config& c, PhysicsSimBackend&, FakePlant&) { AddFollower(c, false); },
       "roller_follower", "[motors.roller_follower] follow = 'roller'"},
      {"mechanism on a bound motor",
       [](Config&, PhysicsSimBackend& b, FakePlant&) {
         b.SetMechanisms({{.motor = "roller", .inertia_kg_m2 = 0.01}});
       },
       "roller", "[[simulation.mechanisms]]"},
      {"hard stop on a bound motor",
       [](Config&, PhysicsSimBackend& b, FakePlant&) {
         b.SetHardStops({{"roller", -1.0, 1.0}});
       },
       "roller", "[[simulation.hard_stops]]"},
      {"unbound module motor while the model owns the chassis",
       [](Config& c, PhysicsSimBackend&, FakePlant& p) {
         AddSwerveModule(c);
         p.binds.emplace_back("front_left_drive");
         p.chassis_state = sim::PlantChassis{};
       },
       "front_left_steer", "[motors.front_left_steer]"},
      {"swerve module on a welded root body",
       [](Config& c, PhysicsSimBackend&, FakePlant& p) {
         AddSwerveModule(c);
         p.binds.emplace_back("front_left_drive");
         p.binds.emplace_back("front_left_steer");
       },
       "front_left_drive", "[motors.front_left_drive]"},
      {"odd plant step",
       [](Config&, PhysicsSimBackend&, FakePlant& p) { p.step = 4999; },
       "plant", "step_us"},
      {"the plant's own refusal",
       [](Config&, PhysicsSimBackend&, FakePlant& p) {
         p.refusal = ConfigError{.device_name = "robot_model",
                                 .operation = "bind",
                                 .message = "robot.xml: actuator 'arm'"};
       },
       "robot_model", "robot.xml: actuator 'arm'"},
      {"a digital input bound twice",
       [](Config& c, PhysicsSimBackend& b, FakePlant&) {
         DigitalInputConfig beam;
         beam.id = 3;
         beam.name = "intake_beam";
         c.digital_inputs.push_back(beam);
         b.SetWorldInputs({{"intake_beam", "beam_rangefinder", false},
                           {"intake_beam", "beam_touch", false}});
       },
       "intake_beam", "[digital_inputs.intake_beam] sim_sensor"},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    Config cfg = SingleMotorConfig();
    FakePlant plant;
    plant.binds = {"roller"};
    PhysicsSimBackend backend;
    backend.SetPlant(&plant);
    c.arrange(cfg, backend, plant);
    EXPECT_FALSE(backend.Configure(cfg));
    const auto error = backend.last_config_error();
    ASSERT_TRUE(error.has_value());
    EXPECT_EQ(error->device_name, c.device);
    EXPECT_NE(error->message.find(c.key), std::string::npos) << error->message;
    // A refused configuration never steps the plant.
    State state{};
    backend.Read(state);
    state.sample_time_us = 5000;
    backend.Read(state);
    EXPECT_TRUE(plant.step_log.empty());
  }
}

// REQ-DEV-37: a device reset restarts a bound motor's controller but not its
// joint: the next tick's closed loop runs at the speed the plant has, and
// the position reads on without a jump.
TEST(SimPlant, DeviceResetLeavesTheJointMoving) {
  Config cfg = SingleMotorConfig();
  cfg.motors[0].slots[0].p = 0.2;
  cfg.motors[0].stator_limit_a = cfg.motors[0].supply_limit_a = 0.0;
  FakePlant plant;
  plant.binds = {"roller"};
  plant.rotors[0] = {3.0, 30.0};
  PhysicsSimBackend backend;
  backend.SetPlant(&plant);
  ASSERT_TRUE(backend.Configure(cfg));
  State state{};
  ASSERT_TRUE(backend.Read(state));
  const std::array<uint16_t, 1> reset_ids{1};
  backend.BeginRestoreAfterReset(reset_ids);
  ASSERT_EQ(backend.PollRestore(), Backend::RestoreStatus::kDone);
  const std::array commands{MotorRequest{1, Mode::kVelocity, 0, 40.0, 0.0}};
  ASSERT_TRUE(backend.Apply(commands));
  state.sample_time_us = 5000;
  ASSERT_TRUE(backend.Read(state));
  EXPECT_NEAR(plant.last_drives[0].volts, 0.2 * (40.0 - 30.0), 1e-12);
  EXPECT_DOUBLE_EQ(state.motors[0].position_rot, 3.0);
  EXPECT_DOUBLE_EQ(state.motors[0].velocity_rps, 30.0);
}

// REQ-DEV-37: a bound motor's soft limit stops its output toward the limit
// and leaves the joint where the plant has it (the analytic model clamps
// it); output away from the limit still drives. Torque current is judged by
// its demand: +10 A while back-driven at -30 rps asks for negative volts
// but forward torque.
TEST(SimPlant, SoftLimitGatesOutputAndLeavesTheJoint) {
  struct Case {
    MotorRequest command;
    double rps;
    bool gated;
  };
  const std::vector<Case> cases{
      {{1, Mode::kVoltage, 0, 6.0, 0.0}, 10.0, true},
      {{1, Mode::kVoltage, 0, -6.0, 0.0}, 10.0, false},
      {{1, Mode::kTorqueCurrent, 0, 10.0, 0.0}, -30.0, true},
      {{1, Mode::kTorqueCurrent, 0, -10.0, 0.0}, -30.0, false},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(testing::Message()
                 << "mode " << static_cast<int>(c.command.mode) << ", demand "
                 << c.command.demand);
    Config cfg = SingleMotorConfig();
    cfg.motors[0].soft_limits = true;
    cfg.motors[0].reverse_limit_rot = -1.0;
    cfg.motors[0].forward_limit_rot = 1.0;
    cfg.motors[0].supply_limit_a = 0.0;
    cfg.motors[0].foc = true;
    cfg.motors[0].max_torque_current_a = 60.0;
    FakePlant plant;
    plant.binds = {"roller"};
    plant.rotors[0] = {1.5, c.rps};  // past the forward limit
    PhysicsSimBackend backend;
    backend.SetPlant(&plant);
    ASSERT_TRUE(backend.Configure(cfg));
    ASSERT_TRUE(backend.Apply(std::array{c.command}));
    State state{};
    ASSERT_TRUE(backend.Read(state));
    const sim::PlantDrive& drive = plant.last_drives[0];
    if (c.gated) {  // the brake neutral drive
      EXPECT_EQ(drive.volts, 0.0);
      EXPECT_EQ(drive.min_torque_nm, -kInf);
      EXPECT_EQ(drive.max_torque_nm, kInf);
    } else {
      EXPECT_LT(drive.volts, 0.0);
    }
    EXPECT_DOUBLE_EQ(state.motors[0].position_rot, 1.5);
    EXPECT_DOUBLE_EQ(state.motors[0].velocity_rps, c.rps);
  }
}

// REQ-DEV-37: a plant that fails is total loss for good: that Read and every
// later one returns false with every motor and CAN sensor invalid, world
// inputs hold their last reading, the plant is never called again, and a
// Configure refuses naming its failure.
TEST(SimPlant, PlantFailureLatchesTotalLoss) {
  Config cfg = SingleMotorConfig();
  SensorConfig encoder;
  encoder.id = 10;
  encoder.name = "roller_encoder";
  encoder.can_id = 10;
  cfg.sensors.push_back(encoder);
  DigitalInputConfig beam;
  beam.id = 3;
  beam.name = "intake_beam";
  cfg.digital_inputs.push_back(beam);
  FakePlant plant;
  plant.binds = {"roller"};
  plant.rotors[0] = {0.25, 1.0};
  plant.closed[0] = true;
  PhysicsSimBackend backend;
  backend.SetPlant(&plant);
  backend.SetWorldInputs({{"intake_beam", "beam_rangefinder", false}});
  ASSERT_TRUE(backend.Configure(cfg));
  State state{};
  ASSERT_TRUE(backend.Read(state));
  plant.fail_next_step = true;
  for (const uint64_t now_us : {5000, 10000}) {
    SCOPED_TRACE(testing::Message() << "tick at " << now_us << " us");
    state.sample_time_us = now_us;
    EXPECT_FALSE(backend.Read(state));
    EXPECT_FALSE(state.motors[0].valid);
    EXPECT_FALSE(state.sensors[0].valid);
    EXPECT_FALSE(state.digital_inputs[0].value);  // closed reads false
    EXPECT_EQ(plant.step_log.size(), 2u);
    plant.closed[0] = false;
  }
  backend.RestartController();
  EXPECT_FALSE(backend.Configure(cfg));
  const auto error = backend.last_config_error();
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->message.find("BADQACC at 1.2 s"), std::string::npos)
      << error->message;
  EXPECT_EQ(plant.calls_after_failure, 0);
}

// REQ-DEV-37: a model that owns the chassis is the field truth and the
// Pigeon's source: truth() is the plant's chassis from the bind on, the gyro
// zeroes on its heading there, the Pigeon reads its yaw and rate after, and
// the analytic chassis is not stepped (a bound swerve module spinning would
// move it).
TEST(SimPlant, AModelOwnedChassisIsTheTruthAndThePigeons) {
  Config cfg = SingleMotorConfig();
  AddSwerveModule(cfg);
  SensorConfig pigeon;
  pigeon.id = 20;
  pigeon.name = "gyro";
  pigeon.kind = SensorKind::kPigeon2;
  pigeon.can_id = 20;
  cfg.sensors.push_back(pigeon);
  FakePlant plant;
  plant.binds = {"front_left_drive", "front_left_steer"};
  plant.rotors[1] = {0.0, 50.0};  // front_left_drive, spinning
  const sim::PlantChassis start{1.0, 2.0, 0.5, 0.0, 0.0, 0.0};
  const sim::PlantChassis moved{1.2, 2.1, 0.8, 0.4, 0.2, 0.6};
  plant.chassis_state = start;
  plant.chassis_after_step = moved;
  PhysicsSimBackend backend({.start_x_m = 5.0, .start_yaw_rad = -1.0});
  backend.SetPlant(&plant);
  ASSERT_TRUE(backend.Configure(cfg));
  EXPECT_DOUBLE_EQ(backend.truth().x_m, start.x_m);
  EXPECT_DOUBLE_EQ(backend.truth().yaw_rad, start.yaw_rad);
  State state{};
  state.sample_time_us = 0;
  ASSERT_TRUE(backend.Read(state));
  const sim::WorldState& truth = backend.truth();
  EXPECT_DOUBLE_EQ(truth.x_m, moved.x_m);
  EXPECT_DOUBLE_EQ(truth.y_m, moved.y_m);
  EXPECT_DOUBLE_EQ(truth.yaw_rad, moved.yaw_rad);
  EXPECT_DOUBLE_EQ(truth.vx_mps, moved.vx_mps);
  EXPECT_DOUBLE_EQ(truth.vy_mps, moved.vy_mps);
  EXPECT_DOUBLE_EQ(truth.omega_radps, moved.omega_radps);
  EXPECT_NEAR(state.sensors[0].position_rot, (0.8 - 0.5) / kTauRad, 1e-12);
  EXPECT_NEAR(state.sensors[0].velocity_rps, 0.6 / kTauRad, 1e-12);
  state.sample_time_us = 5000;
  ASSERT_TRUE(backend.Read(state));
  EXPECT_DOUBLE_EQ(backend.truth().x_m, moved.x_m);
  EXPECT_DOUBLE_EQ(backend.truth().yaw_rad, moved.yaw_rad);
}

// REQ-DEV-37: a world input reads the plant's sensor of that name through
// its closed_reads, over SetDigitalInput; one whose input the config does
// not name is not bound.
TEST(SimPlant, WorldInputReadsThePlantSensor) {
  Config cfg = SingleMotorConfig();
  DigitalInputConfig beam;
  beam.id = 3;
  beam.name = "intake_beam";
  cfg.digital_inputs.push_back(beam);
  FakePlant plant;
  PhysicsSimBackend backend;
  backend.SetPlant(&plant);
  backend.SetWorldInputs({{"intake_beam", "beam_rangefinder", false},
                          {"missing_input", "other_sensor", true}});
  ASSERT_TRUE(backend.Configure(cfg));
  EXPECT_EQ(plant.sensors, std::vector<std::string>{"beam_rangefinder"});
  backend.SetDigitalInput(3, true);
  for (const bool closed : {true, false}) {
    plant.closed[0] = closed;
    State state{};
    ASSERT_TRUE(backend.Read(state));
    EXPECT_EQ(state.digital_inputs[0].value, !closed) << "closed " << closed;
  }
}

}  // namespace
}  // namespace talos::hardware
