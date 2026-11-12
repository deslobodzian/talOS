// Regression tests for PhoenixBackend device handling and simulation
// honesty: quadrature scaling from counts_per_rev, duty-cycle offsets, wrap
// and connection validity, no demand teleport when controller voltage is zero
// (missing gains exposed), neutral keeping rotor momentum with
// brake/coast-dependent decay, follower gating, late feedback left valid for
// the gateway to judge, per-device recovery and the loop watchdog's neutral.
#include "PhoenixBackend.h"

#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/DutyCycleEncoderSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/PWMSim.h>
#include <frc/simulation/RoboRioSim.h>
#include <gtest/gtest.h>
#include <hal/HAL.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <vector>

#include "hardware/gate.h"
#include "hardware/gateway.h"

namespace hw = talos::hardware;

// The GradleRIO google-test wiring links libgtest but not gtest_main.
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
#ifdef __linux__
  // Phoenix's simulator runs every simulated device on one thread, which also
  // rewrites each device's persisted state under ctre_sim/ in the working
  // directory on every config write. With the disk under write-back load (CI
  // builds the other variant meanwhile) one rewrite blocks that thread, and
  // every device, for hundreds of milliseconds. A fresh RAM-backed directory
  // per run avoids the disk and keeps an earlier run's device state out.
  char dir[] = "/dev/shm/talos_phoenix_test_XXXXXX";
  const bool in_ram = mkdtemp(dir) != nullptr;
  if (in_ram) std::filesystem::current_path(dir);
#endif
  const int result = RUN_ALL_TESTS();
#ifdef __linux__
  std::error_code ignored;
  if (in_ram) std::filesystem::remove_all(dir, ignored);
#endif
  return result;
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

// Because one simulator thread serves every device (see main()), anything that
// delays it (host scheduling, or disk I/O where no RAM directory exists) stops
// every device from answering requests (TxFailed, RxTimeout) and publishing
// status frames. kReadyTimeout bounds each wait for the simulator to catch up.
constexpr std::chrono::seconds kReadyTimeout{10};

// Reads every 10 ms, as PumpReads does, until `done()` holds; false once
// kReadyTimeout has passed.
template <typename Done>
bool ReadUntil(PhoenixBackend& backend, hw::State& state, Done done) {
  const auto deadline = std::chrono::steady_clock::now() + kReadyTimeout;
  while (true) {
    backend.Read(state);
    if (done()) return true;
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// Phoenix stamps feedback with the host clock, so a stall of its simulator
// thread or of this one (a loaded host) ages every device's feedback: at
// command_timeout_us the gateway rightly counts it invalid and the backend
// holds followers neutral, and the devices can sit at 0 V for tens of
// milliseconds after the stall. Feedback is fresh here while every motor and
// sensor reads younger than kFreshAgeUs (unused slots read 0), which leaves
// the rest of the kFeedbackBoundUs window for this thread's next call; a pump
// ends only after kSteadyReads fresh reads in a row, so the state it leaves
// follows 100 ms without a stall.
constexpr uint32_t kFreshAgeUs = 20'000;
constexpr int kSteadyReads = 10;
// The age at which the gateway, and Apply's follower check, count feedback
// invalid: every config here keeps the default command_timeout_us.
const uint32_t kFeedbackBoundUs = hw::Config{}.command_timeout_us;

bool FeedbackFresh(const hw::State& s) {
  const auto fresh = [](const auto& sample) {
    return sample.position_age_us < kFreshAgeUs &&
           sample.velocity_age_us < kFreshAgeUs;
  };
  return std::all_of(s.motors.begin(), s.motors.end(), fresh) &&
         std::all_of(s.sensors.begin(), s.sensors.end(), fresh);
}

// Until the returned instant, Apply still finds the motor feedback this read
// got fresh (younger than kFeedbackBoundUs); `read_start` when any was not.
std::chrono::steady_clock::time_point FreshUntil(
    const hw::State& s, std::chrono::steady_clock::time_point read_start) {
  uint32_t age_us = 0;
  for (const auto& m : s.motors) {
    if (!m.valid && m.id != 0) return read_start;
    age_us = std::max({age_us, m.position_age_us, m.velocity_age_us});
  }
  if (age_us >= kFeedbackBoundUs) return read_start;
  return read_start + std::chrono::microseconds(kFeedbackBoundUs - age_us);
}

// Reads every 10 ms and after each read calls `step(fresh_until)`, which
// applies the requests: they are one-shot, refreshed in production by the
// bridge heartbeat (at most 20 ms), not by Gateway ticks, so they are re-sent
// on stale reads too. An Apply finished before `fresh_until` saw every motor's
// feedback fresh. Stops at the first failed step, or once `iters` steps ran
// and the last kSteadyReads reads, `s` among them, were fresh; fails once
// kReadyTimeout has passed.
template <typename Step>
::testing::AssertionResult Pump(PhoenixBackend& backend, hw::State& s,
                                int iters, Step step) {
  const auto deadline = std::chrono::steady_clock::now() + kReadyTimeout;
  int fresh_reads = 0;
  for (int i = 0;; ++i) {
    const auto read_start = std::chrono::steady_clock::now();
    backend.Read(s);
    fresh_reads = FeedbackFresh(s) ? fresh_reads + 1 : 0;
    if (i >= iters && fresh_reads >= kSteadyReads)
      return ::testing::AssertionSuccess();
    if (std::chrono::steady_clock::now() >= deadline) break;
    if (auto result = step(FreshUntil(s, read_start)); !result) return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  auto failure = ::testing::AssertionFailure()
                 << "no " << kSteadyReads << " fresh reads in a row within "
                 << kReadyTimeout.count() << " s; last read:";
  for (std::size_t i = 0; i < s.motors.size(); ++i)
    failure << " motor slot " << i << " age "
            << std::max(s.motors[i].position_age_us,
                        s.motors[i].velocity_age_us)
            << " us;";
  return failure;
}

void PumpReads(PhoenixBackend& backend, hw::State& s,
               std::span<const hw::MotorRequest> cmd, int iters = 20) {
  const auto pumped = Pump(backend, s, iters, [&](auto) {
    backend.Apply(cmd);
    return ::testing::AssertionSuccess();
  });
  if (!pumped) ADD_FAILURE() << pumped.message();
}

bool IsTransportTimeout(int status) {
  using ctre::phoenix::StatusCode;
  return status == StatusCode::TxFailed || status == StatusCode::RxTimeout ||
         status == StatusCode::TxTimeout;
}

// Re-sends a config request that a stalled simulator dropped (a transport
// timeout; waiting longer for the same request does not help) until
// kReadyTimeout has passed. Any other status returns at once.
template <typename Request>
ctre::phoenix::StatusCode AwaitAnswer(Request request) {
  const auto deadline = std::chrono::steady_clock::now() + kReadyTimeout;
  auto status = request();
  while (IsTransportTimeout(status) &&
         std::chrono::steady_clock::now() < deadline)
    status = request();
  return status;
}

// Gateway holds outputs neutral after Configure() and restores devices that
// Configure() left unavailable when its bounded retries ran out. There is no
// Gateway here, so do both through the same backend calls, then wait until
// every CAN motor and sensor reports fresh feedback and every motor reports
// zero output: a simulated device outlives its backend and resumes the
// previous test's last request once the robot is enabled. A config error other
// than a transport timeout fails at once; a device not ready at the deadline
// fails with its state.
::testing::AssertionResult AwaitReady(PhoenixBackend& backend,
                                      const hw::Config& config) {
  if (const auto error = backend.last_config_error();
      error && !IsTransportTimeout(error->vendor_status))
    return ::testing::AssertionFailure() << hw::FormatConfigError(*error);
  const auto start = std::chrono::steady_clock::now();
  hw::State state{};
  bool restoring = false;
  bool restored = false;
  const bool ready = ReadUntil(backend, state, [&] {
    backend.Neutral();  // Refreshed on every pass, as Gateway does per tick.
    if (restoring)
      restoring = backend.PollRestore() == hw::Backend::RestoreStatus::kPending;
    if (!restoring && !backend.UnavailableDevices().empty()) {
      backend.BeginRestoreAfterReset(backend.UnavailableDevices());
      restoring = restored = true;
    }
    if (restoring) return false;
    // Valid as the gateway would judge it: the backend leaves age to it.
    const auto valid = [](const auto& sample) {
      return sample.valid &&
             hw::FeedbackFresh(sample.position_age_us, kFeedbackBoundUs) &&
             hw::FeedbackFresh(sample.velocity_age_us, kFeedbackBoundUs);
    };
    for (std::size_t i = 0; i < config.motors.size(); ++i)
      if (!valid(state.motors[i]) || state.motors[i].voltage != 0.0)
        return false;
    for (std::size_t i = 0; i < config.sensors.size(); ++i)
      if (!valid(state.sensors[i])) return false;
    return true;
  });
  const auto waited_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - start)
                             .count();
  if (!ready) {
    auto failure = ::testing::AssertionFailure()
                   << "Phoenix devices not ready after " << waited_ms << " ms";
    for (const auto id : backend.UnavailableDevices())
      failure << "; device " << id << " unavailable";
    if (const auto error = backend.last_config_error())
      failure << "; first Configure failure: " << hw::FormatConfigError(*error);
    for (std::size_t i = 0; i < config.motors.size(); ++i) {
      const auto& motor = state.motors[i];
      if (!motor.valid)
        failure << "; motor " << config.motors[i].name
                << " feedback invalid (position age " << motor.position_age_us
                << " us)";
      else if (motor.voltage != 0.0)
        failure << "; motor " << config.motors[i].name << " still at "
                << motor.voltage << " V";
    }
    for (std::size_t i = 0; i < config.sensors.size(); ++i)
      if (!state.sensors[i].valid)
        failure << "; sensor " << config.sensors[i].name
                << " feedback invalid (position age "
                << state.sensors[i].position_age_us << " us)";
    return failure;
  }
  // Keep the simulator's timing visible in CI logs when it was slow.
  if (restored || waited_ms >= 100)
    std::cerr << "[test] Phoenix devices ready after " << waited_ms << " ms"
              << (restored ? ", restoring devices Configure left unavailable"
                           : "")
              << "\n";
  return ::testing::AssertionSuccess();
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

TEST(PhoenixBackendPigeon, MountYawConfiguredAndInversionFlipsReadYaw) {
  EnsureHal();
  hw::Config config;
  config.motors.push_back(TestMotor(1, "m1", 1));
  hw::SensorConfig imu{9, "imu", hw::SensorKind::kPigeon2, 30, "rio"};
  imu.offset_rot = 0.125;
  imu.inverted = true;
  config.sensors.push_back(imu);
  PhoenixBackend backend(false);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  // A second handle observes the same simulated CAN device/configurator.
  ctre::phoenix6::hardware::Pigeon2 observer(30, ctre::phoenix6::CANBus{"rio"});
  ctre::phoenix6::configs::Pigeon2Configuration applied;
  ASSERT_TRUE(AwaitAnswer([&] {
                return observer.GetConfigurator().Refresh(applied);
              }).IsOK());
  EXPECT_NEAR(applied.MountPose.MountPoseYaw.value(), 45.0, 1e-3);
  observer.GetSimState().SetSupplyVoltage(units::volt_t{12.0});
  observer.GetSimState().SetRawYaw(units::degree_t{90.0});
  // The simulated yaw starts at zero, so a nonzero sample carries the new one.
  hw::State state{};
  ASSERT_TRUE(ReadUntil(backend, state, [&] {
    return state.sensors[0].valid &&
           std::abs(state.sensors[0].position_rot) > 1.0 / 360.0;
  })) << "no fresh yaw sample reflected the simulated raw yaw";
  auto& yaw = observer.GetYaw();
  ASSERT_TRUE(yaw.Refresh().GetStatus().IsOK());
  ASSERT_GT(std::abs(yaw.GetValue().value()), 1.0);
  EXPECT_NEAR(state.sensors[0].position_rot, -yaw.GetValue().value() / 360.0,
              1e-6);
}

TEST(PhoenixBackendEncoders, QuadratureScalingAppliesCountsPerRevAndOffset) {
  EnsureHal();
  hw::Config c;
  c.motors.push_back(TestMotor(1, "m1", 1));
  c.encoders.push_back(QuadratureEncoder(10, "quad", 0, 1, 2048, 0.25));

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));
  ASSERT_TRUE(AwaitReady(backend, c));

  // k4X decoding counts every edge, so counts_per_rev is edges per rev.
  auto sim = frc::sim::EncoderSim::CreateForChannel(0);
  sim.SetCount(4096);

  hw::State s;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.encoders[0].valid);
  // 4096 edges / 2048 counts-per-rev, minus the 0.25 rot offset.
  EXPECT_NEAR(s.encoders[0].position_rot, 1.75, 1e-9);
}

TEST(PhoenixBackendEncoders, DutyCycleVelocityWrapsAndReconnectStartsAtRest) {
  EnsureHal();
  hw::Config config;
  config.motors.push_back(TestMotor(1, "m1", 1));
  config.encoders.push_back(DutyEncoder(11, "duty", 2, 0.125));
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  frc::sim::DutyCycleEncoderSim sim(2);
  sim.SetConnected(true);
  sim.Set(0.99);
  // Wait after positioning the encoder, so readiness reads seed its velocity
  // history with this position rather than another one.
  ASSERT_TRUE(AwaitReady(backend, config));
  hw::State state{};
  const auto before_first = std::chrono::steady_clock::now();
  ASSERT_TRUE(backend.Read(state));
  const auto after_first = std::chrono::steady_clock::now();
  ASSERT_TRUE(state.encoders[0].valid);
  EXPECT_NEAR(state.encoders[0].position_rot, 0.865, 1e-9);
  EXPECT_DOUBLE_EQ(state.encoders[0].velocity_rps, 0.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  sim.Set(0.01);
  const auto before_second = std::chrono::steady_clock::now();
  ASSERT_TRUE(backend.Read(state));
  const auto after_second = std::chrono::steady_clock::now();
  ASSERT_TRUE(state.encoders[0].valid);
  // The read timestamps lie inside these intervals. Across the rollover the
  // displacement is +0.02 rotations, not -0.98.
  const double min_dt =
      std::chrono::duration<double>(before_second - after_first).count();
  const double max_dt =
      std::chrono::duration<double>(after_second - before_first).count();
  ASSERT_GT(min_dt, 0.0);
  EXPECT_GE(state.encoders[0].velocity_rps, 0.02 / max_dt - 1e-9);
  EXPECT_LE(state.encoders[0].velocity_rps, 0.02 / min_dt + 1e-9);

  // A bad RIO encoder is mechanism scope: Read stays healthy, only that
  // sample goes invalid and zeroed, and the rest of the robot keeps driving.
  sim.SetConnected(false);
  sim.Set(0.7);
  ASSERT_TRUE(backend.Read(state));
  EXPECT_FALSE(state.encoders[0].valid);
  EXPECT_DOUBLE_EQ(state.encoders[0].position_rot, 0.0);
  EXPECT_DOUBLE_EQ(state.encoders[0].velocity_rps, 0.0);
  EXPECT_TRUE(state.motors[0].valid);
  sim.SetConnected(true);
  sim.Set(0.4);
  ASSERT_TRUE(backend.Read(state));
  EXPECT_TRUE(state.encoders[0].valid);
  EXPECT_NEAR(state.encoders[0].position_rot, 0.275, 1e-9);
  EXPECT_DOUBLE_EQ(state.encoders[0].velocity_rps, 0.0);
}

TEST(PhoenixBackendNeutral, NeutralDeEnergizesDigitalOutputsAndDisablesPwm) {
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
  ASSERT_TRUE(AwaitReady(backend, c));

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
  frc::sim::PWMSim pwm_sim{0};
  EXPECT_GT(pwm_sim.GetPulseMicrosecond(), 0);

  backend.Neutral();

  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.digital_outputs[0].valid);
  EXPECT_FALSE(s.digital_outputs[0].value);
  EXPECT_TRUE(s.pwm_outputs[0].valid);
  EXPECT_DOUBLE_EQ(s.pwm_outputs[0].output, 0.0);
  EXPECT_EQ(pwm_sim.GetPulseMicrosecond(), 0);
}

TEST(PhoenixBackendHealth, ReportsBatteryVoltage) {
  EnsureHal();
  PhoenixBackend backend(true);
  hw::Config config;
  config.motors.push_back(TestMotor(1, "health_motor", 1));
  ASSERT_TRUE(backend.Configure(config));
  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{7.1});
  hw::State state;
  backend.Read(state);  // Power health does not depend on CAN sample freshness.
  EXPECT_TRUE(state.health.valid);
  EXPECT_DOUBLE_EQ(state.health.battery_v, 7.1);
  EXPECT_GE(state.health.can_utilization_frac, 0.0);
  EXPECT_LE(state.health.can_utilization_frac, 1.0);
  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{12.0});
}

// Missing gains yield ~0 V, so a closed-loop demand must neither spin the
// rotor nor teleport its position to the target.
class PhoenixBackendZeroGains
    : public ::testing::TestWithParam<std::tuple<hw::Mode, double>> {};

TEST_P(PhoenixBackendZeroGains, ClosedLoopRequestDoesNotSupplyDemand) {
  EnsureHal();
  EnabledDs ds;
  hw::Config c;
  c.motors.push_back(TestMotor(1, "m1", 1));

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));
  ASSERT_TRUE(AwaitReady(backend, c));

  const auto [mode, demand] = GetParam();
  const std::array<hw::MotorRequest, 1> cmd{Request(1, mode, demand)};
  ASSERT_TRUE(backend.Apply(cmd));

  hw::State s;
  PumpReads(backend, s, cmd);
  EXPECT_LT(std::abs(s.motors[0].voltage), 0.5);
  EXPECT_LT(std::abs(s.motors[0].velocity_rps), 0.5);
  EXPECT_LT(std::abs(s.motors[0].position_rot), 0.5);
}

INSTANTIATE_TEST_SUITE_P(
    Modes, PhoenixBackendZeroGains,
    ::testing::Values(std::make_tuple(hw::Mode::kVelocity, 10.0),
                      std::make_tuple(hw::Mode::kPosition, 5.0)));

TEST(PhoenixBackendSim, ReportsAmbientAndPhoenixSupplyCurrent) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  config.motors.push_back(TestMotor(1, "health_motor", 1));
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  const std::array<hw::MotorRequest, 1> commands{
      Request(1, hw::Mode::kVoltage, 6.0)};
  hw::State state;
  PumpReads(backend, state, commands, 30);
  ASSERT_TRUE(state.motors[0].valid);
  EXPECT_DOUBLE_EQ(state.motors[0].temperature_c, 25.0);
  EXPECT_TRUE(std::isfinite(state.motors[0].supply_current_a));
  // The SDK calculates electrical current from the simulated rotor, rather
  // than interpreting a motion target as current or aliasing stator current.
  ctre::phoenix6::hardware::TalonFX device{1, ctre::phoenix6::CANBus{"rio"}};
  EXPECT_NEAR(state.motors[0].supply_current_a,
              device.GetSimState().GetSupplyCurrent().value(), 0.5);

  backend.Neutral();
  const std::array<hw::MotorRequest, 1> neutral{
      Request(1, hw::Mode::kNeutral, 0.0)};
  PumpReads(backend, state, neutral, 30);
  EXPECT_DOUBLE_EQ(state.motors[0].temperature_c, 25.0);
  EXPECT_NEAR(state.motors[0].supply_current_a, 0.0, 0.5);
}

namespace {

struct NeutralSample {
  double spinning_rps;
  double after_100ms_rps;
  double after_100ms_rot;
  double after_300ms_rps;
  double after_300ms_rot;
};

// Spin one motor at 6 V, then go neutral through both entry points (the
// backend-wide Neutral() and per-motor kNeutral requests) and sample the
// rotor at ~100 ms and ~300 ms of wall-clock sim time.
NeutralSample SpinThenNeutral(bool brake) {
  hw::Config c;
  auto motor = TestMotor(1, "m1", 1);
  motor.brake = brake;
  c.motors.push_back(motor);
  PhoenixBackend backend(true);
  EXPECT_TRUE(backend.Configure(c));
  EXPECT_TRUE(AwaitReady(backend, c));

  std::array<hw::MotorRequest, 1> cmd{Request(1, hw::Mode::kDutyCycle, 0.5)};
  hw::State s;
  PumpReads(backend, s, cmd, 30);
  NeutralSample out{};
  out.spinning_rps = s.motors[0].velocity_rps;

  backend.Neutral();
  cmd[0] = Request(1, hw::Mode::kNeutral, 0.0);
  PumpReads(backend, s, cmd, 10);
  out.after_100ms_rps = s.motors[0].velocity_rps;
  out.after_100ms_rot = s.motors[0].position_rot;
  PumpReads(backend, s, cmd, 20);
  out.after_300ms_rps = s.motors[0].velocity_rps;
  out.after_300ms_rot = s.motors[0].position_rot;
  return out;
}

}  // namespace

TEST(PhoenixBackendSim, NeutralPreservesMomentumAndBrakeDecaysFasterThanCoast) {
  EnsureHal();
  EnabledDs ds;
  const NeutralSample brake = SpinThenNeutral(/*brake=*/true);
  const NeutralSample coast = SpinThenNeutral(/*brake=*/false);
  ASSERT_GT(brake.spinning_rps, 20.0);
  ASSERT_GT(coast.spinning_rps, 20.0);
  EXPECT_GT(coast.after_100ms_rps, 0.5 * coast.spinning_rps);
  EXPECT_LT(coast.after_100ms_rps, coast.spinning_rps);
  EXPECT_LT(coast.after_300ms_rps, coast.after_100ms_rps);
  EXPECT_GT(coast.after_300ms_rps, 0.0);
  EXPECT_GT(coast.after_300ms_rot, coast.after_100ms_rot);
  // ~50 ms brake time constant: most speed is gone after ~100 ms, but the
  // rotor did not stop instantly (no velocity snap on neutral).
  EXPECT_LT(brake.after_100ms_rps, 0.3 * brake.spinning_rps);
  EXPECT_GT(brake.after_100ms_rps, 0.0);
  EXPECT_LT(brake.after_100ms_rps / brake.spinning_rps,
            0.5 * coast.after_100ms_rps / coast.spinning_rps);
  EXPECT_LT(brake.after_300ms_rps, 0.5);
  EXPECT_GT(coast.after_300ms_rps, 10.0 * std::max(brake.after_300ms_rps, 0.1));
}

class PhoenixBackendFollower : public ::testing::TestWithParam<bool> {};

TEST_P(PhoenixBackendFollower, FollowsMasterAndNeutralizesWithMaster) {
  EnsureHal();
  EnabledDs ds;
  hw::Config c;
  // Nonmatching logical/CAN IDs and follower-first order exercise lookup.
  auto follower = TestMotor(20, "follower", 2);
  follower.master_id = 10;
  follower.oppose_master = GetParam();
  c.motors.push_back(follower);
  c.motors.push_back(TestMotor(10, "master", 1));
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));
  ASSERT_TRUE(AwaitReady(backend, c));
  std::array<hw::MotorRequest, 2> commands{
      Request(20, hw::Mode::kVoltage, 0.0),
      Request(10, hw::Mode::kVoltage, 6.0)};
  hw::State state;
  PumpReads(backend, state, commands, 30);
  ASSERT_TRUE(state.motors[0].valid);
  ASSERT_TRUE(state.motors[1].valid);
  EXPECT_GT(state.motors[1].voltage, 3.0);
  EXPECT_GT(state.motors[0].voltage * (GetParam() ? -1.0 : 1.0), 3.0);
  EXPECT_GT(state.motors[0].velocity_rps * (GetParam() ? -1.0 : 1.0), 1.0);
  // The physical simulation voltage includes opposition; the controller's
  // logical MotorVoltage signal does not. Platform feedback uses the former
  // direction in the configured mechanism frame.
  ctre::phoenix6::hardware::TalonFX follower_device{
      2, ctre::phoenix6::CANBus{"rio"}};
  EXPECT_GT(follower_device.GetSimState().GetMotorVoltage().value() *
                (GetParam() ? -1.0 : 1.0),
            3.0);

  commands[1] = Request(10, hw::Mode::kNeutral, 0.0);
  // A follower's independent demand must not override master neutral.
  commands[0] = Request(20, hw::Mode::kVoltage, 6.0);
  PumpReads(backend, state, commands);
  EXPECT_NEAR(state.motors[0].voltage, 0.0, 0.5);

  commands[1] = Request(10, hw::Mode::kVoltage, 6.0);
  PumpReads(backend, state, commands, 30);
  EXPECT_GT(std::abs(state.motors[0].voltage), 3.0);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
  PumpReads(backend, state, commands);
  EXPECT_NEAR(state.motors[0].voltage, 0.0, 0.5);
}

INSTANTIATE_TEST_SUITE_P(Direction, PhoenixBackendFollower, ::testing::Bool());

class PhoenixBackendFollowerFeedback
    : public ::testing::TestWithParam<std::tuple<bool, bool>> {
 protected:
  static ctre::phoenix6::BaseStatusSignal& StaleSignal(PhoenixBackend& backend,
                                                       bool remote,
                                                       bool position) {
    if (remote) {
      if (position)
        return backend.sensors_[0].encoder->GetAbsolutePosition(false);
      return backend.sensors_[0].encoder->GetVelocity(false);
    }
    if (position) return backend.motors_[0]->GetPosition(false);
    return backend.motors_[0]->GetVelocity(false);
  }

  // The refreshed feedback of motor `i`, as FeedbackFresh judges a read.
  static bool Fresh(PhoenixBackend& backend, std::size_t i) {
    return backend.motors_[i]->GetPosition(false).GetTimestamp().GetLatency() <
           units::microsecond_t{kFreshAgeUs};
  }

  static void RefreshExcept(PhoenixBackend& backend,
                            ctre::phoenix6::BaseStatusSignal& stale) {
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    for (auto* signal : backend.signals_) {
      if (signal != &stale) signals.push_back(signal);
    }
    ctre::phoenix6::BaseStatusSignal::RefreshAll(
        false, std::span<ctre::phoenix6::BaseStatusSignal* const>{signals});
  }
};

TEST_P(PhoenixBackendFollowerFeedback,
       StaleOwnFeedbackNeutralizesOnlyFollower) {
  EnsureHal();
  EnabledDs ds;
  const auto [remote, position] = GetParam();
  hw::Config config;
  auto follower = TestMotor(20, "follower", 2);
  follower.master_id = 10;
  if (remote) {
    hw::SensorConfig sensor;
    sensor.id = 30;
    sensor.name = "follower_encoder";
    sensor.kind = hw::SensorKind::kCANcoder;
    sensor.can_id = 3;
    sensor.bus = "rio";
    config.sensors.push_back(sensor);
    follower.feedback = hw::Feedback::kRemoteCANcoder;
    follower.feedback_sensor_id = sensor.id;
  }
  config.motors.push_back(follower);
  config.motors.push_back(TestMotor(10, "master", 1));
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  const std::array<hw::MotorRequest, 2> commands{
      Request(20, hw::Mode::kVoltage, 0.0),
      Request(10, hw::Mode::kVoltage, 6.0)};
  hw::State state;
  PumpReads(backend, state, commands, 30);
  ASSERT_TRUE(state.motors[0].valid);
  ASSERT_TRUE(state.motors[1].valid);
  if (remote) {
    ASSERT_TRUE(state.sensors[0].valid);
  }
  ASSERT_GT(state.motors[0].voltage, 3.0);
  ASSERT_GT(state.motors[1].voltage, 3.0);

  // Stop refreshing exactly one cached feedback signal until it is older than
  // kFeedbackBoundUs. Read would refresh it, so inspect physical sim voltage
  // directly instead of masked state fields.
  auto& stale = StaleSignal(backend, remote, position);
  // As Pump does, end only after kSteadyReads fresh refreshes of the master in
  // a row, so a simulator stall cannot leave it at 0 V when sampled.
  std::this_thread::sleep_for(std::chrono::microseconds(kFeedbackBoundUs) +
                              std::chrono::milliseconds(20));
  const auto deadline = std::chrono::steady_clock::now() + kReadyTimeout;
  for (int i = 0, fresh_reads = 0; i < 20 || fresh_reads < kSteadyReads; ++i) {
    ASSERT_LT(std::chrono::steady_clock::now(), deadline);
    RefreshExcept(backend, stale);
    fresh_reads = Fresh(backend, 1) ? fresh_reads + 1 : 0;
    ASSERT_TRUE(backend.Apply(commands));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GE(stale.GetTimestamp().GetLatency().value(), kFeedbackBoundUs * 1e-6);
  ctre::phoenix6::hardware::TalonFX follower_device{
      2, ctre::phoenix6::CANBus{"rio"}};
  ctre::phoenix6::hardware::TalonFX master_device{
      1, ctre::phoenix6::CANBus{"rio"}};
  EXPECT_NEAR(follower_device.GetSimState().GetMotorVoltage().value(), 0.0,
              0.5);
  EXPECT_GT(master_device.GetSimState().GetMotorVoltage().value(), 3.0);

  // Refreshing the failed feedback restores following without reconfiguration.
  PumpReads(backend, state, commands, 30);
  EXPECT_GT(state.motors[0].voltage, 3.0);
  EXPECT_GT(state.motors[1].voltage, 3.0);
}

INSTANTIATE_TEST_SUITE_P(SourceAndSignal, PhoenixBackendFollowerFeedback,
                         ::testing::Combine(::testing::Bool(),
                                            ::testing::Bool()));

class PhoenixBackendRecovery : public ::testing::Test {
 protected:
  static auto& Motor(PhoenixBackend& backend, std::size_t i) {
    return *backend.motors_[i];
  }
  static auto& MotorConfig(PhoenixBackend& backend, std::size_t i) {
    return backend.config_.motors[i];
  }
  static void StopRefreshing(PhoenixBackend& backend,
                             ctre::phoenix6::BaseStatusSignal& signal) {
    std::erase(backend.signals_, &signal);
  }
  static void ResumeRefreshing(PhoenixBackend& backend,
                               ctre::phoenix6::BaseStatusSignal& signal) {
    backend.signals_.push_back(&signal);
  }
  // Holds the rotor still under whatever voltage the TalonFX applies, so it
  // draws stall current, by taking the backend's rotor model out of the loop.
  static void StallRotor(PhoenixBackend& backend) {
    backend.is_simulation_ = false;
    Motor(backend, 0)
        .GetSimState()
        .SetRotorVelocity(units::turns_per_second_t{0});
  }
  static void MarkUnavailable(PhoenixBackend& backend, uint16_t id) {
    backend.unavailable_.push_back(id);
  }
  static void StopRefreshingHealth(PhoenixBackend& backend) {
    auto& motor = Motor(backend, 0);
    std::erase_if(backend.signals_, [&](auto* signal) {
      return signal == &motor.GetDeviceTemp(false) ||
             signal == &motor.GetSupplyCurrent(false);
    });
    // Exercise hardware telemetry handling, without the ambient sim override.
    backend.is_simulation_ = false;
  }
};

TEST_F(PhoenixBackendRecovery, DiagnosticRatesRespectPhoenixSharedFrames) {
  EnsureHal();
  hw::Config config;
  config.status_hz = 200;
  config.motors.push_back(TestMotor(1, "motor", 1));
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  auto& motor = Motor(backend, 0);
  EXPECT_DOUBLE_EQ(motor.GetPosition(false).GetAppliedUpdateFrequency().value(),
                   200);
  EXPECT_DOUBLE_EQ(motor.GetVelocity(false).GetAppliedUpdateFrequency().value(),
                   200);
  // Phoenix coalesces these with motion feedback at the highest requested
  // frame rate; requesting 50 Hz cannot slow the control frame.
  EXPECT_DOUBLE_EQ(
      motor.GetMotorVoltage(false).GetAppliedUpdateFrequency().value(), 200);
  EXPECT_DOUBLE_EQ(
      motor.GetStatorCurrent(false).GetAppliedUpdateFrequency().value(), 200);
  // SDK 26.3 SupplyAndTemp contains all three signals; requesting 4 Hz for
  // temperature and 50 Hz for supply current cannot slow their shared frame.
  EXPECT_DOUBLE_EQ(
      motor.GetSupplyCurrent(false).GetAppliedUpdateFrequency().value(), 200);
  EXPECT_DOUBLE_EQ(
      motor.GetDeviceTemp(false).GetAppliedUpdateFrequency().value(), 200);
  EXPECT_DOUBLE_EQ(
      motor.GetFaultField(false).GetAppliedUpdateFrequency().value(), 50);
}

// The TalonFX enforces its limit pins itself, so the motor config must reach
// the device: a normally closed forward switch, a disabled reverse one.
TEST_F(PhoenixBackendRecovery, AppliesConfiguredHardwareLimitSwitches) {
  EnsureHal();
  hw::Config config;
  auto lift = TestMotor(1, "lift", 1);
  lift.forward_limit_switch = hw::LimitSwitch::kNormallyClosed;
  lift.reverse_limit_switch = hw::LimitSwitch::kDisabled;
  config.motors.push_back(lift);
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  ctre::phoenix6::configs::TalonFXConfiguration applied;
  ASSERT_TRUE(AwaitAnswer([&] {
                return Motor(backend, 0).GetConfigurator().Refresh(applied);
              }).IsOK());
  using ctre::phoenix6::signals::ForwardLimitSourceValue;
  using ctre::phoenix6::signals::ForwardLimitTypeValue;
  const auto& limits = applied.HardwareLimitSwitch;
  EXPECT_TRUE(limits.ForwardLimitEnable);
  EXPECT_EQ(limits.ForwardLimitType.value,
            ForwardLimitTypeValue::NormallyClosed);
  EXPECT_EQ(limits.ForwardLimitSource.value,
            ForwardLimitSourceValue::LimitSwitchPin);
  EXPECT_FALSE(limits.ReverseLimitEnable);
}

TEST_F(PhoenixBackendRecovery, StaleHealthDoesNotInvalidateMotionFeedback) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  config.motors.push_back(TestMotor(1, "motor", 1));
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  const std::array<hw::MotorRequest, 1> commands{
      Request(1, hw::Mode::kVoltage, 6.0)};
  hw::State state;
  PumpReads(backend, state, commands, 30);
  ASSERT_TRUE(state.motors[0].valid);
  StopRefreshingHealth(backend);
  // Exceed both the 50 ms current and 500 ms temperature freshness windows.
  PumpReads(backend, state, commands, 65);
  EXPECT_TRUE(state.motors[0].valid);
  EXPECT_GT(state.motors[0].voltage, 3.0);
  EXPECT_DOUBLE_EQ(state.motors[0].temperature_c, 0.0);
  EXPECT_DOUBLE_EQ(state.motors[0].supply_current_a, 0.0);
}

// A stalled motor draws supply current. Once its motion feedback is invalid
// (here, an isolated device), the sample carries zero in every measured
// field, supply current included, although that signal is still fresh.
// REQ-CONTRACT-32
TEST_F(PhoenixBackendRecovery, InvalidMotionFeedbackZeroesSupplyCurrent) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  config.motors.push_back(TestMotor(1, "motor", 1));
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  const std::array<hw::MotorRequest, 1> commands{
      Request(1, hw::Mode::kVoltage, 6.0)};
  StallRotor(backend);
  hw::State state;
  PumpReads(backend, state, commands, 30);
  ASSERT_TRUE(state.motors[0].valid);
  ASSERT_GT(std::abs(state.motors[0].supply_current_a), 1.0);
  MarkUnavailable(backend, 1);
  auto& supply = Motor(backend, 0).GetSupplyCurrent(false);
  // A read whose supply current is fresh, so only validity can zero it.
  ASSERT_TRUE(ReadUntil(backend, state, [&] {
    return supply.GetTimestamp().GetLatency().value() < 0.02;
  }));
  EXPECT_FALSE(state.motors[0].valid);
  EXPECT_DOUBLE_EQ(state.motors[0].supply_current_a, 0.0)
      << "fresh signal reads " << supply.GetValue().value() << " A";
  EXPECT_DOUBLE_EQ(state.motors[0].stator_current_a, 0.0);
  EXPECT_DOUBLE_EQ(state.motors[0].voltage, 0.0);
  EXPECT_DOUBLE_EQ(state.motors[0].position_rot, 0.0);
  EXPECT_DOUBLE_EQ(state.motors[0].velocity_rps, 0.0);
}

// A CAN silence shorter than the gateway's bound (the bench's 64-66 ms gaps)
// leaves the backend's sample valid: it reports the age and the gateway
// judges it, the same for every backend. The console shows the spell instead:
// LATE once the feedback is four status periods old, then ON TIME with the
// spell's oldest age.
// REQ-SAFE-22
TEST_F(PhoenixBackendRecovery, LateFeedbackStaysValidAndIsReportedLate) {
  EnsureHal();
  std::vector<std::string> lines;
  talos::rio::RioLog log{
      [&](std::string_view line) { lines.emplace_back(line); }};
  hw::Config config;
  config.motors.push_back(TestMotor(1, "motor", 1));
  PhoenixBackend backend(true, &log);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  log.Flush(0);
  lines.clear();
  auto& position = Motor(backend, 0).GetPosition(false);
  StopRefreshing(backend, position);
  hw::State state;
  // About the bench's gaps: past the LATE threshold (20 ms at 200 Hz) and the
  // 50 ms diagnostics bound, short of the gateway's.
  constexpr uint32_t kGapUs = 70'000;
  ASSERT_TRUE(ReadUntil(backend, state, [&] {
    return state.motors[0].position_age_us >= kGapUs;
  }));
  EXPECT_TRUE(state.motors[0].valid)
      << "position age " << state.motors[0].position_age_us << " us";
  ResumeRefreshing(backend, position);
  ASSERT_TRUE(ReadUntil(backend, state, [&] {
    return state.motors[0].position_age_us < kFreshAgeUs;
  }));
  EXPECT_TRUE(state.motors[0].valid);
  log.Flush(0);
  bool late = false;
  uint64_t peak_age_us = 0;
  for (const auto& line : lines) {
    late = late || line.find("feedback LATE") != std::string::npos;
    constexpr std::string_view kOnTime = "feedback ON TIME (peak_age_us=";
    if (const auto at = line.find(kOnTime); at != std::string::npos)
      peak_age_us = std::stoull(line.substr(at + kOnTime.size()));
  }
  EXPECT_TRUE(late);
  EXPECT_GE(peak_age_us, kGapUs);
}

// FaultField is a separate signal from the motion feedback that keeps the
// sample valid. Once it is 50 ms old the backend reports no limit or limiter
// bit, since an old switch or limiter state is a wrong display (a motor
// driven into its pressed forward switch, or stalled at its stator limit),
// but keeps the fault bits it last had: losing the signal never un-faults a
// motor (BootDuringEnable here: that row's TalonFX, CAN id 57, which no other
// test uses, first boots inside it, while enabled). Each row first waits
// until a read shows its bit: the reads that PumpReads gates on fresh motion
// feedback say nothing of the separate 50 Hz FaultField, and one that finds
// it 50 ms old reports no limit by this same rule.
TEST_F(PhoenixBackendRecovery, StaleFaultFieldClearsLimitsAndKeepsFaults) {
  enum class Row { kForwardHardLimit, kStatorLimiter, kBootFault };
  // FaultField bit 4, BootDuringEnable: a fault (hw::MotorFaulted).
  constexpr uint32_t kBootDuringEnableFlag = uint32_t{1} << 4;
  for (const Row row :
       {Row::kForwardHardLimit, Row::kStatorLimiter, Row::kBootFault}) {
    const uint32_t bit =
        row == Row::kForwardHardLimit ? hw::kMotorForwardHardLimitFlag
        : row == Row::kStatorLimiter  ? hw::kMotorCurrentLimitFlags
                                      : kBootDuringEnableFlag;
    SCOPED_TRACE(row == Row::kForwardHardLimit ? "forward hard limit"
                 : row == Row::kStatorLimiter  ? "stator current limit"
                                               : "boot fault");
    EnsureHal();
    EnabledDs ds;
    hw::Config config;
    auto motor = TestMotor(1, "motor", row == Row::kBootFault ? 57 : 1);
    motor.stator_limit_a = 10;
    config.motors.push_back(motor);
    PhoenixBackend backend(true);
    ASSERT_TRUE(backend.Configure(config));
    ASSERT_TRUE(AwaitReady(backend, config));
    const std::array<hw::MotorRequest, 1> commands{
        Request(1, hw::Mode::kVoltage, row == Row::kStatorLimiter ? 6.0 : 2.0)};
    hw::State state;
    auto& sim = Motor(backend, 0).GetSimState();
    if (row == Row::kStatorLimiter) StallRotor(backend);
    if (row == Row::kForwardHardLimit) sim.SetForwardLimit(true);
    PumpReads(backend, state, commands, 30);
    auto& faults = Motor(backend, 0).GetFaultField(false);
    ASSERT_TRUE(ReadUntil(backend, state,
                          [&] {
                            backend.Apply(commands);
                            return state.motors[0].valid &&
                                   (state.motors[0].fault_flags & bit) != 0;
                          }))
        << "fault_flags " << state.motors[0].fault_flags << ", FaultField "
        << faults.GetTimestamp().GetLatency().value() * 1e3 << " ms old";
    StopRefreshing(backend, faults);
    PumpReads(backend, state, commands, 30);
    ASSERT_GE(faults.GetTimestamp().GetLatency().value(), 0.05);
    EXPECT_TRUE(state.motors[0].valid);
    if (row == Row::kBootFault) {
      EXPECT_NE(state.motors[0].fault_flags & bit, 0u)
          << "fault_flags " << state.motors[0].fault_flags;
    } else {
      EXPECT_EQ(state.motors[0].fault_flags & hw::kMotorNotFaultFlags, 0u)
          << "fault_flags " << state.motors[0].fault_flags;
    }
    sim.SetForwardLimit(false);
  }
}

TEST_F(PhoenixBackendRecovery, RestoreDoesNotReconfigureUnrelatedMotor) {
  EnsureHal();
  hw::Config config;
  auto first = TestMotor(1, "first", 1);
  auto second = TestMotor(2, "second", 2);
  first.slots[0].p = 3;
  second.slots[0].p = 5;
  config.motors = {first, second};
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  ctre::phoenix6::configs::TalonFXConfiguration defaults;
  ASSERT_TRUE(AwaitAnswer([&] {
                return Motor(backend, 0).GetConfigurator().Apply(defaults);
              }).IsOK());
  ASSERT_TRUE(AwaitAnswer([&] {
                return Motor(backend, 1).GetConfigurator().Apply(defaults);
              }).IsOK());
  const std::array<uint16_t, 1> ids{1};
  backend.BeginRestoreAfterReset(ids);
  auto result = hw::Backend::RestoreStatus::kPending;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{5};
  while (result == hw::Backend::RestoreStatus::kPending &&
         std::chrono::steady_clock::now() < deadline) {
    result = backend.PollRestore();
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  ASSERT_EQ(result, hw::Backend::RestoreStatus::kDone);
  EXPECT_TRUE(backend.UnavailableDevices().empty());
  ctre::phoenix6::configs::TalonFXConfiguration applied;
  ASSERT_TRUE(AwaitAnswer([&] {
                return Motor(backend, 0).GetConfigurator().Refresh(applied);
              }).IsOK());
  EXPECT_DOUBLE_EQ(applied.Slot0.kP, 3);
  ASSERT_TRUE(AwaitAnswer([&] {
                return Motor(backend, 1).GetConfigurator().Refresh(applied);
              }).IsOK());
  EXPECT_DOUBLE_EQ(applied.Slot0.kP, 0);
}

// A control send that fails on one motor neutralizes and reports only that
// motor; unrelated motors keep driving instead of a whole-robot neutral.
TEST_F(PhoenixBackendRecovery, SendFailureNeutralizesOnlyThatMotor) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  auto follower = TestMotor(20, "follower", 2);
  follower.master_id = 10;
  config.motors = {TestMotor(10, "master", 1), follower,
                   TestMotor(30, "independent", 3)};
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  const std::array<hw::MotorRequest, 3> commands{
      Request(10, hw::Mode::kVoltage, 6.0),
      Request(20, hw::Mode::kVoltage, 0.0),
      Request(30, hw::Mode::kVoltage, 6.0)};
  hw::State state;
  PumpReads(backend, state, commands, 30);
  for (std::size_t i = 0; i < config.motors.size(); ++i)
    ASSERT_TRUE(state.motors[i].valid) << i;
  // Test-only fault injection: an out-of-range master CAN id makes Phoenix
  // reject the follower's Follower request, a genuine SetControl failure.
  // Only a send on fresh feedback is held to it: on stale feedback the
  // backend rightly holds the follower neutral and sends no Follower request.
  MotorConfig(backend, 0).can_id = 99;
  int checked = 0;
  ASSERT_TRUE(Pump(backend, state, 30, [&](auto fresh_until) {
    if (!backend.Apply(commands))
      return ::testing::AssertionFailure() << "Apply refused";
    if (std::chrono::steady_clock::now() >= fresh_until)
      return ::testing::AssertionSuccess();
    ++checked;
    const auto failed = backend.ApplyFailedDevices();
    if (std::vector<uint16_t>(failed.begin(), failed.end()) ==
        std::vector<uint16_t>{20})
      return ::testing::AssertionSuccess();
    auto failure = ::testing::AssertionFailure() << "failed sends {";
    for (const auto id : failed) failure << " " << id;
    return failure << " }, want only the follower's { 20 }";
  }));
  ASSERT_GT(checked, 0);
  ctre::phoenix6::hardware::TalonFX master{1, ctre::phoenix6::CANBus{"rio"}};
  ctre::phoenix6::hardware::TalonFX follower_device{
      2, ctre::phoenix6::CANBus{"rio"}};
  ctre::phoenix6::hardware::TalonFX independent{3,
                                                ctre::phoenix6::CANBus{"rio"}};
  EXPECT_GT(independent.GetSimState().GetMotorVoltage().value(), 3.0);
  EXPECT_GT(master.GetSimState().GetMotorVoltage().value(), 3.0);
  EXPECT_NEAR(follower_device.GetSimState().GetMotorVoltage().value(), 0.0,
              0.5);

  // The next successful send clears the report and following resumes.
  MotorConfig(backend, 0).can_id = 1;
  ASSERT_TRUE(Pump(backend, state, 30, [&](auto) {
    if (!backend.Apply(commands))
      return ::testing::AssertionFailure() << "Apply refused";
    if (!backend.ApplyFailedDevices().empty())
      return ::testing::AssertionFailure() << "a send still failed";
    return ::testing::AssertionSuccess();
  }));
  EXPECT_GT(follower_device.GetSimState().GetMotorVoltage().value(), 3.0);
}

class PhoenixBackendWatchdog : public ::testing::Test {
 protected:
  static std::mutex& DevicesMutex(PhoenixBackend& backend) {
    return backend.devices_mutex_;
  }
  static std::string_view AppliedControl(PhoenixBackend& backend,
                                         std::size_t i) {
    return backend.motors_[i]->GetAppliedControl()->GetName();
  }
};

// The loop watchdog's neutral, from its own thread: while a configure holds
// the device lists it touches nothing and says so; otherwise every TalonFX
// takes NeutralOut, DIO goes false and PWM disabled.
TEST_F(PhoenixBackendWatchdog, NeutralizesEveryOutputUnlessAConfigureHoldsIt) {
  EnsureHal();
  EnabledDs ds;
  hw::Config c;
  c.motors = {TestMotor(10, "left", 1), TestMotor(20, "right", 2)};
  hw::DigitalOutputConfig d;
  d.id = 40;
  d.name = "solenoid";
  d.dio = 5;
  c.digital_outputs.push_back(d);
  hw::PwmConfig p;
  p.id = 50;
  p.name = "servo";
  p.channel = 0;
  c.pwm_outputs.push_back(p);
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));
  ASSERT_TRUE(AwaitReady(backend, c));

  const std::array<hw::MotorRequest, 2> commands{
      Request(10, hw::Mode::kVoltage, 6.0),
      Request(20, hw::Mode::kVoltage, 6.0)};
  ASSERT_TRUE(backend.ApplyOutputs(
      std::array<hw::DigitalOutputRequest, 1>{{{40, true}}},
      std::array<hw::PwmRequest, 1>{{{50, 0.75}}}));
  hw::State state;
  PumpReads(backend, state, commands, 5);
  frc::sim::PWMSim pwm_sim{0};
  ASSERT_GT(pwm_sim.GetPulseMicrosecond(), 0);

  const auto neutral_from_another_thread = [&backend] {
    bool sent = false;
    std::thread watchdog{[&] { sent = backend.WatchdogNeutral(); }};
    watchdog.join();
    return sent;
  };
  {
    const std::lock_guard configuring{DevicesMutex(backend)};
    EXPECT_FALSE(neutral_from_another_thread());
  }
  ASSERT_TRUE(backend.Read(state));
  EXPECT_TRUE(state.digital_outputs[0].value);
  EXPECT_GT(pwm_sim.GetPulseMicrosecond(), 0);
  EXPECT_EQ(AppliedControl(backend, 0), "VoltageOut");

  EXPECT_TRUE(neutral_from_another_thread());
  ASSERT_TRUE(backend.Read(state));
  EXPECT_FALSE(state.digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(state.pwm_outputs[0].output, 0.0);
  EXPECT_EQ(pwm_sim.GetPulseMicrosecond(), 0);
  for (std::size_t i = 0; i < c.motors.size(); ++i)
    EXPECT_EQ(AppliedControl(backend, i), "NeutralOut") << i;
}
