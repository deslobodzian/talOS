// Regression tests for PhoenixBackend encoder handling and simulation
// honesty: quadrature scaling from counts_per_rev,
// duty-cycle device support with offsets and connection validity, and no
// demand teleport when controller voltage is zero (missing gains exposed).
#include "PhoenixBackend.h"

#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/DutyCycleEncoderSim.h>
#include <frc/simulation/EncoderSim.h>
#include <gtest/gtest.h>
#include <hal/HAL.h>

#include <array>
#include <chrono>
#include <cmath>
#include <span>
#include <thread>

#include "hardware/gateway.h"

namespace hw = talos::hardware;

// The GradleRIO google-test wiring links libgtest but not gtest_main.
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace {

void EnsureHal() {
  static const bool once = [] {
    HAL_Initialize(500, 0);
    return true;
  }();
  (void)once;
}

// CTRE zeroes outputs while the driver station reports disabled, so motor
// simulation tests must run enabled (as in a live sim session).
struct EnabledDs {
  EnabledDs() {
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
  }
  ~EnabledDs() {
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
  }
};

// Production re-sends the latest command every Gateway tick, so the pump does
// the same: a single one-shot Apply would otherwise age out mid-test.
void PumpReads(PhoenixBackend& backend, hw::State& s,
               const std::array<hw::MotorRequest, 1>& cmd, int iters = 20) {
  for (int i = 0; i < iters; ++i) {
    backend.Apply(cmd);
    backend.Read(s);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

hw::MotorConfig TestMotor(uint16_t id, const std::string& name, int can_id) {
  hw::MotorConfig m;
  m.id = id;
  m.name = name;
  m.can_id = can_id;
  m.bus = "rio";
  return m;  // All slots keep zero gains: missing-gain behavior is the point.
}

hw::EncoderConfig QuadratureEncoder(uint16_t id, const std::string& name,
                                    int dio_a, int dio_b, double cpr,
                                    double offset) {
  hw::EncoderConfig e;
  e.id = id;
  e.name = name;
  e.kind = hw::EncoderKind::kQuadrature;
  e.dio_a = dio_a;
  e.dio_b = dio_b;
  e.counts_per_rev = cpr;
  e.offset_rot = offset;
  return e;
}

hw::EncoderConfig DutyEncoder(uint16_t id, const std::string& name, int dio,
                              double offset) {
  hw::EncoderConfig e;
  e.id = id;
  e.name = name;
  e.kind = hw::EncoderKind::kDutyCycle;
  e.dio_a = dio;
  e.dio_b = -1;
  e.offset_rot = offset;
  return e;
}

hw::MotorRequest Request(uint16_t id, hw::Mode mode, double demand) {
  hw::MotorRequest r;
  r.id = id;
  r.mode = mode;
  r.slot = 0;
  r.demand = demand;
  r.feedforward_v = 0.0;
  return r;
}

}  // namespace

TEST(PhoenixBackendEncoders, QuadratureScalingAppliesCountsPerRevAndOffset) {
  EnsureHal();
  hw::Config c;
  c.motors.push_back(TestMotor(1, "m1", 1));
  c.encoders.push_back(QuadratureEncoder(10, "quad", 0, 1, 2048, 0.25));

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));

  // k4X decoding counts every edge, so counts_per_rev is edges per rev.
  auto sim = frc::sim::EncoderSim::CreateForChannel(0);
  sim.SetCount(4096);

  hw::State s;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.encoders[0].valid);
  // 4096 edges / 2048 counts-per-rev, minus the 0.25 rot offset.
  EXPECT_NEAR(s.encoders[0].position_rot, 1.75, 1e-9);
}

TEST(PhoenixBackendEncoders, QuadratureRatePassesThrough) {
  EnsureHal();
  hw::Config c;
  c.motors.push_back(TestMotor(1, "m1", 1));
  c.encoders.push_back(QuadratureEncoder(10, "quad", 0, 1, 2048, 0.0));

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));

  // Simulation injects the rate downstream of distance scaling (the position
  // test above proves counts_per_rev scaling); the backend must pass it
  // through unmodified.
  auto sim = frc::sim::EncoderSim::CreateForChannel(0);
  sim.SetRate(3.0);

  hw::State s;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.encoders[0].valid);
  EXPECT_NEAR(s.encoders[0].velocity_rps, 3.0, 1e-9);
}

TEST(PhoenixBackendEncoders, DutyCycleReportsOffsetCorrectedPosition) {
  EnsureHal();
  hw::Config c;
  c.motors.push_back(TestMotor(1, "m1", 1));
  c.encoders.push_back(DutyEncoder(11, "duty", 2, 0.125));

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));

  frc::sim::DutyCycleEncoderSim sim(2);
  sim.SetConnected(true);
  sim.Set(0.5);

  hw::State s;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.encoders[0].valid);
  EXPECT_NEAR(s.encoders[0].position_rot, 0.375, 1e-9);

  // No movement between Reads: finite-difference velocity is exactly zero.
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.encoders[0].valid);
  EXPECT_DOUBLE_EQ(s.encoders[0].velocity_rps, 0.0);
}

TEST(PhoenixBackendEncoders, DutyCycleDisconnectInvalidatesSample) {
  EnsureHal();
  // A bad RIO encoder is mechanism scope: the backend must stay healthy and
  // only that sample goes invalid.
  hw::Config c;
  c.motors.push_back(TestMotor(1, "m1", 1));
  c.encoders.push_back(DutyEncoder(11, "duty", 2, 0.0));

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));

  frc::sim::DutyCycleEncoderSim sim(2);
  sim.SetConnected(true);
  sim.Set(0.5);

  hw::State s;
  ASSERT_TRUE(backend.Read(s));
  ASSERT_TRUE(s.encoders[0].valid);

  // Disconnect: the owning mechanism observes an invalid, zeroed sample ...
  sim.SetConnected(false);

  EXPECT_TRUE(backend.Read(s));
  EXPECT_FALSE(s.encoders[0].valid);
  EXPECT_DOUBLE_EQ(s.encoders[0].position_rot, 0.0);
  EXPECT_DOUBLE_EQ(s.encoders[0].velocity_rps, 0.0);
  // ... while the rest of the robot keeps driving.
  EXPECT_TRUE(s.motors[0].valid);
}

TEST(PhoenixBackendNeutral, NeutralDeEnergizesDigitalOutputsAndZeroesPwm) {
  EnsureHal();
  hw::Config c;
  c.motors.push_back(TestMotor(1, "m1", 1));
  hw::DigitalOutputConfig d;
  d.id = 20;
  d.name = "solenoid";
  d.dio = 5;
  c.digital_outputs.push_back(d);
  hw::PwmConfig p;
  p.id = 30;
  p.name = "servo";
  p.channel = 0;
  c.pwm_outputs.push_back(p);

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));

  const std::array<hw::DigitalOutputRequest, 1> dos{
      hw::DigitalOutputRequest{20, true}};
  const std::array<hw::PwmRequest, 1> pwms{hw::PwmRequest{30, 0.75}};
  ASSERT_TRUE(backend.ApplyOutputs(dos, pwms));

  // Read back through the real HAL adapter path: outputs are driven.
  hw::State s;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.digital_outputs[0].valid);
  EXPECT_TRUE(s.digital_outputs[0].value);
  EXPECT_TRUE(s.pwm_outputs[0].valid);
  EXPECT_GT(s.pwm_outputs[0].output, 0.5);

  backend.Neutral();

  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.digital_outputs[0].valid);
  EXPECT_FALSE(s.digital_outputs[0].value);
  EXPECT_TRUE(s.pwm_outputs[0].valid);
  EXPECT_DOUBLE_EQ(s.pwm_outputs[0].output, 0.0);
}

TEST(PhoenixBackendSim, ZeroVoltageVelocityRequestDoesNotSupplyDemand) {
  EnsureHal();
  EnabledDs ds;
  hw::Config c;
  c.motors.push_back(TestMotor(1, "m1", 1));

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));

  const std::array<hw::MotorRequest, 1> cmd{
      Request(1, hw::Mode::kVelocity, 10.0)};
  ASSERT_TRUE(backend.Apply(cmd));

  hw::State s;
  PumpReads(backend, s, cmd);
  // Missing gains yield ~0 V, so the rotor must not spin at the 10 rps demand.
  EXPECT_LT(std::abs(s.motors[0].velocity_rps), 0.5);
  EXPECT_LT(std::abs(s.motors[0].voltage), 0.5);
}

TEST(PhoenixBackendSim, ZeroVoltagePositionRequestDoesNotTeleport) {
  EnsureHal();
  EnabledDs ds;
  hw::Config c;
  c.motors.push_back(TestMotor(1, "m1", 1));

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));

  const std::array<hw::MotorRequest, 1> cmd{
      Request(1, hw::Mode::kPosition, 5.0)};
  ASSERT_TRUE(backend.Apply(cmd));

  hw::State s;
  PumpReads(backend, s, cmd);
  // Missing gains yield ~0 V, so position must not jump to the 5 rot demand.
  EXPECT_LT(std::abs(s.motors[0].position_rot), 0.5);
  EXPECT_LT(std::abs(s.motors[0].velocity_rps), 0.5);
}

TEST(PhoenixBackendSim, VoltageStillDrivesMotor) {
  EnsureHal();
  EnabledDs ds;
  hw::Config c;
  c.motors.push_back(TestMotor(1, "m1", 1));

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));

  const std::array<hw::MotorRequest, 1> cmd{
      Request(1, hw::Mode::kDutyCycle, 0.5)};
  ASSERT_TRUE(backend.Apply(cmd));

  hw::State s;
  PumpReads(backend, s, cmd, 30);
  // 0.5 duty is ~6 V, targeting ~50 rps: physics must still move the rotor.
  EXPECT_GT(s.motors[0].velocity_rps, 5.0);
  EXPECT_GT(s.motors[0].position_rot, 0.0);
}
