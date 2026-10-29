// Simulation believability track (items 13-19): the ideal SimBackend stays
// the fast logic fake, while PhysicsSimBackend (sim_physics.h) earns trust
// for PID tuning, gearing, direction, and limits. All tests step an explicit
// sim clock; no wall-clock, no sockets, no live sim.
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

// The ideal backend is for fast logic tests: demands apply instantly,
// even with zero gains. The physics backend must NOT behave this way.
TEST(SimBelievability, IdealTeleportsWhilePhysicsRamps) {
  State s{};
  // Ideal: one tick at demand.
  {
    SimBackend ideal;
    Gateway g{SingleMotorConfig(), ideal, 1};
    g.Tick(1000, true);
    Command c{};
    const State snap = g.snapshot();
    c.config_id = snap.config_id;
    c.boot_id = snap.boot_id;
    c.epoch = snap.epoch;
    c.observed_time_us = snap.sample_time_us;
    c.count = 1;
    c.motors[0] = {1, Mode::kVelocity, 0, 10.0, 0};
    ASSERT_TRUE(g.Accept(c, 1, 1000));
    g.Tick(6000, true);
    EXPECT_DOUBLE_EQ(g.snapshot().motors[0].velocity_rps, 10.0);
  }
  // Physics: one 5 ms step only starts the ramp; it converges, not snaps.
  {
    PhysicsSimBackend physics;
    ASSERT_TRUE(physics.Configure(SingleMotorConfig()));
    std::array<MotorRequest, kMaxMotors> cmd{};
    cmd[0] = Duty(1, 0.5);
    ASSERT_TRUE(physics.Apply(cmd));
    uint64_t now_us = 0;
    // Two Reads: the first establishes the timestamp (dt = 0), the second
    // advances one effective 5 ms step.
    StepBackend(physics, s, now_us, 5000, 2);
    EXPECT_GT(s.motors[0].velocity_rps, 0.0);
    EXPECT_LT(s.motors[0].velocity_rps, 50.0);   // Far from the ~50 rps target.
    StepBackend(physics, s, now_us, 5000, 400);  // 2 s more.
    EXPECT_GT(s.motors[0].velocity_rps, 40.0);
    EXPECT_GT(s.motors[0].position_rot, 0.0);
  }
}

// Voltage drives the mechanism; position integrates velocity.
TEST(SimBelievability, VoltageDrivesMechanismAndPositionIntegrates) {
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(SingleMotorConfig()));
  std::array<MotorRequest, kMaxMotors> cmd{};
  cmd[0] = Duty(1, 0.5);
  ASSERT_TRUE(backend.Apply(cmd));
  State s{};
  uint64_t now_us = 0;
  StepBackend(backend, s, now_us, 5000, 200);  // 1 s at ~6 V.
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

// (14/15) Gearing divides rotor speed into mechanism speed; inversion flips
// direction; soft limits clamp and stop the axis.
TEST(SimBelievability, GearingDirectionAndLimits) {
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
  // Direction: inverted spins the mechanism the opposite way.
  {
    PhysicsSimBackend fwd, rev;
    Config c = SingleMotorConfig(), ci = SingleMotorConfig();
    ci.motors[0].inverted = true;
    ASSERT_TRUE(fwd.Configure(c));
    ASSERT_TRUE(rev.Configure(ci));
    std::array<MotorRequest, kMaxMotors> cmd{};
    cmd[0] = Duty(1, 0.5);
    ASSERT_TRUE(fwd.Apply(cmd));
    ASSERT_TRUE(rev.Apply(cmd));
    State sf{}, sr{};
    uint64_t now_us = 0;
    for (int i = 0; i < 100; ++i) {
      now_us += 5000;
      sf.sample_time_us = now_us;
      sr.sample_time_us = now_us;
      fwd.Read(sf);
      rev.Read(sr);
    }
    EXPECT_NEAR(sr.motors[0].velocity_rps, -sf.motors[0].velocity_rps, 1e-6);
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

// Pigeon yaw comes from simulated chassis motion through the boundary.
TEST(SimBelievability, PigeonYawFollowsChassisMotion) {
  Config c = SingleMotorConfig();
  c.sensors.push_back({9, "imu", SensorKind::kPigeon2, 30, "rio"});
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(c));
  backend.SetChassisYawRate(0.25);  // Quarter turn per second, CCW+.
  State s{};
  uint64_t now_us = 0;
  // 201 Reads: the first establishes the timestamp, the next 200 step 1 s.
  StepBackend(backend, s, now_us, 5000, 201);  // 1 s.
  ASSERT_EQ(s.sensors[0].id, 9u);
  EXPECT_TRUE(s.sensors[0].valid);
  EXPECT_NEAR(s.sensors[0].position_rot, 0.25, 1e-9);
  EXPECT_NEAR(s.sensors[0].velocity_rps, 0.25, 1e-9);
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

// Controlled inputs: beam breaks read back through the boundary.
TEST(SimBelievability, BeamBreakInputReadsBack) {
  Config c = SingleMotorConfig();
  c.digital_inputs.push_back({3, "beam_break", 0});
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(c));
  backend.SetDigitalInput(3, true);
  State s{};
  s.sample_time_us = 5000;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.digital_inputs[0].value);
  backend.SetDigitalInput(3, false);
  s.sample_time_us = 10000;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_FALSE(s.digital_inputs[0].value);
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

// Disconnect invalidates the sample and fails the read (gateway fault
// path); reconnect recovers. Sensors behave the same way.
TEST(SimBelievability, DisconnectFailsReadsAndReconnectRecovers) {
  Config c = SingleMotorConfig();
  c.sensors.push_back({9, "imu", SensorKind::kPigeon2, 30, "rio"});
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(c));
  State s{};
  s.sample_time_us = 5000;
  ASSERT_TRUE(backend.Read(s));
  backend.SetMotorConnected(1, false);
  s.sample_time_us = 10000;
  EXPECT_FALSE(backend.Read(s));
  EXPECT_FALSE(s.motors[0].valid);
  backend.SetMotorConnected(1, true);
  s.sample_time_us = 15000;
  EXPECT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.motors[0].valid);
  backend.SetSensorConnected(9, false);
  s.sample_time_us = 20000;
  EXPECT_FALSE(backend.Read(s));
  EXPECT_FALSE(s.sensors[0].valid);
  backend.SetSensorConnected(9, true);
  s.sample_time_us = 25000;
  EXPECT_TRUE(backend.Read(s));
}

// Supply-voltage change scales the available speed; restart zeroes
// motion but keeps the configuration.
TEST(SimBelievability, SupplyVoltageScalesSpeedAndRestartZeroesMotion) {
  PhysicsSimBackend low, high;
  Config c = SingleMotorConfig();
  ASSERT_TRUE(low.Configure(c));
  ASSERT_TRUE(high.Configure(c));
  low.SetSupplyVoltage(6.0);
  EXPECT_DOUBLE_EQ(low.supply_voltage(), 6.0);
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
  // Restart zeroes motion state; the config (and output safe state) survives.
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

}  // namespace
}  // namespace talos::hardware
