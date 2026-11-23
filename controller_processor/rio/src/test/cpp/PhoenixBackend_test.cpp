// Regression tests for PhoenixBackend device handling and simulation
// honesty: quadrature scaling from counts_per_rev, duty-cycle offsets, wrap
// and connection validity, no demand teleport when controller voltage is zero
// (missing gains exposed), neutral keeping rotor momentum with
// brake/coast-dependent decay, follower gating, late feedback left valid for
// the gateway to judge, per-device recovery, the loop watchdog's neutral, and
// a PWM output that is off sending no pulse rather than the centre pulse,
// and a motor's foc setting: EnableFOC on its requests, and a motor not read
// Phoenix Pro licensed held failed until the next configure; and a
// torque-current request: TorqueCurrentFOC within its peak and duty cap, and
// NeutralOut for one out of contract.
#include "PhoenixBackend.h"

#include <frc/DigitalInput.h>
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
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <filesystem>
#include <functional>
#include <future>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "hardware/gate.h"
#include "hardware/gateway.h"
#include "hardware/sim_backend.h"

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

// One restore pass of `ids`, as the gateway runs one after a reset.
::testing::AssertionResult RestorePass(PhoenixBackend& backend,
                                       std::span<const uint16_t> ids) {
  const auto deadline = std::chrono::steady_clock::now() + kReadyTimeout;
  backend.BeginRestoreAfterReset(ids);
  while (backend.PollRestore() == hw::Backend::RestoreStatus::kPending) {
    if (std::chrono::steady_clock::now() >= deadline)
      return ::testing::AssertionFailure() << "restore still pending";
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
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

// A non-finite duty-cycle reading while connected invalidates only that tick
// and must not poison last_position_rot so the next valid reading is also
// invalidated by a NaN velocity finite difference (a backend-internal gap
// Gateway's ReplaceNonFinite cannot repair).
// REQ-SAFE-22, REQ-CONTRACT-32
TEST(PhoenixBackendEncoders,
     NonFiniteDutyCycleReadingDoesNotPoisonNextValidTick) {
  EnsureHal();
  hw::Config config;
  config.motors.push_back(TestMotor(1, "m1", 1));
  config.encoders.push_back(DutyEncoder(11, "duty", 2, 0.125));
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  frc::sim::DutyCycleEncoderSim sim(2);
  sim.SetConnected(true);
  sim.Set(0.25);
  ASSERT_TRUE(AwaitReady(backend, config));

  hw::State state{};
  ASSERT_TRUE(backend.Read(state));
  ASSERT_TRUE(state.encoders[0].valid);

  // Inject NaN while connected: this tick's encoder sample is invalid and
  // zeroed, while the backend stays healthy.
  sim.Set(std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(backend.Read(state));
  EXPECT_FALSE(state.encoders[0].valid);
  EXPECT_DOUBLE_EQ(state.encoders[0].position_rot, 0.0);
  EXPECT_DOUBLE_EQ(state.encoders[0].velocity_rps, 0.0);

  // Next tick returns a valid finite reading: must recover immediately at rest
  // rather than computing remainder(pos - NaN, 1.0).
  sim.Set(0.4);
  ASSERT_TRUE(backend.Read(state));
  EXPECT_TRUE(state.encoders[0].valid)
      << "non-finite reading poisoned last_position_rot on the next valid tick";
  EXPECT_NEAR(state.encoders[0].position_rot, 0.275, 1e-9);
  EXPECT_DOUBLE_EQ(state.encoders[0].velocity_rps, 0.0);

  // Same for +Inf.
  sim.Set(std::numeric_limits<double>::infinity());
  ASSERT_TRUE(backend.Read(state));
  EXPECT_FALSE(state.encoders[0].valid);
  EXPECT_DOUBLE_EQ(state.encoders[0].position_rot, 0.0);
  EXPECT_DOUBLE_EQ(state.encoders[0].velocity_rps, 0.0);

  sim.Set(0.5);
  ASSERT_TRUE(backend.Read(state));
  EXPECT_TRUE(state.encoders[0].valid)
      << "+Inf reading poisoned last_position_rot on the next valid tick";
  EXPECT_NEAR(state.encoders[0].position_rot, 0.375, 1e-9);
  EXPECT_DOUBLE_EQ(state.encoders[0].velocity_rps, 0.0);
}

// The HAL's default PWM centre pulse, which frc::PWM::SetSpeed(0) gives: the
// backend sets no bounds of its own.
constexpr int32_t kCentrePulseUs = 1500;

// Neutral stops the pulse, the centre pulse included: a servo goes limp
// rather than to its centre.
// REQ-SAFE-03
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
  const std::array<hw::PwmRequest, 1> pwms{
      hw::PwmRequest{30, hw::PwmMode::kPulse, 0.0}};
  ASSERT_TRUE(backend.ApplyOutputs(dos, pwms));

  // Read back through the real HAL adapter path: outputs are driven.
  hw::State s;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.digital_outputs[0].valid);
  EXPECT_TRUE(s.digital_outputs[0].value);
  frc::sim::PWMSim pwm_sim{0};
  ASSERT_EQ(pwm_sim.GetPulseMicrosecond(), kCentrePulseUs);

  backend.Neutral();

  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.digital_outputs[0].valid);
  EXPECT_FALSE(s.digital_outputs[0].value);
  EXPECT_TRUE(s.pwm_outputs[0].valid);
  EXPECT_DOUBLE_EQ(s.pwm_outputs[0].output, 0.0);
  EXPECT_EQ(pwm_sim.GetPulseMicrosecond(), 0);
}

// A PWM request's mode decides whether its output pulses: off sends no pulse
// (frc::PWM::SetDisabled), even right after a pulse, and its State sample
// reads 0; a pulse of 0 is the centre pulse and 0.75 lies above it; a pulse
// after off pulses again; each output follows only its own request.
// REQ-SAFE-03
TEST(PhoenixBackendPwm, OffSendsNoPulseAndAPulseOfZeroIsTheCentre) {
  EnsureHal();
  hw::Config c;
  hw::PwmConfig servo;
  servo.id = 30;
  servo.name = "servo";
  servo.channel = 0;
  hw::PwmConfig blower;
  blower.id = 31;
  blower.name = "blower";
  blower.channel = 1;
  c.pwm_outputs = {servo, blower};
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(c));
  ASSERT_TRUE(AwaitReady(backend, c));
  const frc::sim::PWMSim servo_sim{0};
  const frc::sim::PWMSim blower_sim{1};
  hw::State s;
  const auto apply = [&](hw::PwmRequest a, hw::PwmRequest b) {
    ASSERT_TRUE(backend.ApplyOutputs({}, std::array{a, b}));
    ASSERT_TRUE(backend.Read(s));
  };

  {
    SCOPED_TRACE("servo pulse 0.0, blower pulse 0.75");
    apply({30, hw::PwmMode::kPulse, 0.0}, {31, hw::PwmMode::kPulse, 0.75});
    EXPECT_EQ(servo_sim.GetPulseMicrosecond(), kCentrePulseUs);
    EXPECT_GT(blower_sim.GetPulseMicrosecond(), kCentrePulseUs);
    EXPECT_DOUBLE_EQ(s.pwm_outputs[0].output, 0.0);
    EXPECT_NEAR(s.pwm_outputs[1].output, 0.75, 0.01);
  }
  {
    SCOPED_TRACE("both off after their pulses");
    apply({30}, {31});
    EXPECT_EQ(servo_sim.GetPulseMicrosecond(), 0);
    EXPECT_EQ(blower_sim.GetPulseMicrosecond(), 0);
    for (std::size_t i = 0; i < 2; ++i) {
      EXPECT_TRUE(s.pwm_outputs[i].valid) << i;
      EXPECT_DOUBLE_EQ(s.pwm_outputs[i].output, 0.0) << i;
    }
  }
  {
    SCOPED_TRACE("servo pulse 0.0 again, blower stays off");
    apply({30, hw::PwmMode::kPulse, 0.0}, {31});
    EXPECT_EQ(servo_sim.GetPulseMicrosecond(), kCentrePulseUs);
    EXPECT_EQ(blower_sim.GetPulseMicrosecond(), 0);
  }
}

// A configuration with no motor, or no device at all, is valid: the backend
// configures it, its Read has no device signal to refresh and still reports
// a healthy read with power health, and a digital output alone drives, reads
// back and goes de-energized on Neutral.
// REQ-CONTRACT-35
TEST(PhoenixBackendNoMotor, EmptyAndDigitalOutputOnlyConfigsConfigureAndRead) {
  EnsureHal();
  {
    const hw::Config empty;
    PhoenixBackend backend(true);
    ASSERT_TRUE(backend.Configure(empty));
    ASSERT_TRUE(AwaitReady(backend, empty));
    EXPECT_TRUE(backend.UnavailableDevices().empty());
    hw::State s;
    EXPECT_TRUE(backend.Read(s));
    EXPECT_TRUE(s.health.valid);
    EXPECT_TRUE(backend.Apply({}));
    backend.Neutral();
    EXPECT_TRUE(backend.Read(s));
  }

  hw::Config outputs;
  hw::DigitalOutputConfig d;
  d.id = 20;
  d.name = "solenoid";
  d.dio = 6;
  outputs.digital_outputs.push_back(d);
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(outputs));
  ASSERT_TRUE(AwaitReady(backend, outputs));
  const std::array<hw::DigitalOutputRequest, 1> on{
      hw::DigitalOutputRequest{20, true}};
  ASSERT_TRUE(backend.ApplyOutputs(on, {}));
  hw::State s;
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.digital_outputs[0].valid);
  EXPECT_TRUE(s.digital_outputs[0].value);
  backend.Neutral();
  ASSERT_TRUE(backend.Read(s));
  EXPECT_TRUE(s.digital_outputs[0].valid);
  EXPECT_FALSE(s.digital_outputs[0].value);
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
  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{12.0});
}

// WPILib's simulated battery reads 12 V, which fails a pre-match battery
// check (the 2026 dashboard's asks for 12.5 V). The desktop simulation's
// backend rests it at the charged supply sim_gateway reports, so both
// simulated gateways read the same battery at rest.
// REQ-DEV-06
TEST(PhoenixBackendHealth, SimulationRestsTheBatteryAtTheChargedSupply) {
  EnsureHal();
  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{12.0});  // WPILib's
  PhoenixBackend backend(true);
  hw::State state;
  backend.Read(state);
  EXPECT_DOUBLE_EQ(state.health.battery_v, hw::kSimRestingSupplyV);
}

// Under WPILib simulation no CAN bus is read: the slot the gateway asks for
// reads SIMULATED, never a failing bus or an idle one.
//
// Through the gateway, as the RoboRIO runs it: PhoenixBackend configures on
// a worker (BeginConfigure, PollConfigure), and the gateway commits the
// configuration in the tick that finds it done. Scenario (bench, a 60 s
// session): with that asynchronous commit every State carried
// can_bus_count 0, the pre-commit count written back over the committed
// one. Each State must carry the committed configuration's buses, in turn,
// after a second configuration with fewer buses too.
// REQ-OPS-20
TEST(PhoenixBackendHealth, AnAsynchronousConfigureReportsEachBusSimulated) {
  EnsureHal();
  PhoenixBackend backend(true);
  hw::Gateway gateway{backend, 1};
  uint64_t now = 0;
  const auto configure = [&](const hw::Config& config, int buses) {
    SCOPED_TRACE(buses);
    ASSERT_TRUE(gateway.BeginReconfigure(config));
    const auto deadline = std::chrono::steady_clock::now() + kReadyTimeout;
    while (!gateway.configured()) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline)
          << "the configure never completed";
      gateway.Tick(now += 5000, false);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::vector<int> indices;
    for (int i = 0; i < 2 * buses; ++i) {
      gateway.Tick(now += 5000, false);
      const hw::State s = gateway.snapshot();
      EXPECT_EQ(int{s.health.can_bus_count}, buses) << "tick " << i;
      EXPECT_EQ(s.health.can_bus.read_state, hw::CanBusReadState::kSimulated)
          << "tick " << i;
      indices.push_back(s.health.can_bus_index);
    }
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    EXPECT_EQ(static_cast<int>(indices.size()), buses)
        << "not every bus's slot came round";
  };
  hw::Config two;
  two.motors.push_back(TestMotor(1, "rio_motor", 1));
  two.motors.push_back(TestMotor(2, "shooter_motor", 2));
  two.motors.back().bus = "Shooter";
  configure(two, 2);
  hw::Config one;
  one.motors.push_back(TestMotor(1, "rio_motor", 1));
  configure(one, 1);
}

// A robot may put some devices on the RoboRIO's own bus and the rest on a
// CANivore (the remedy for the 2026 robot's 107 % "rio" bus: CAN-1 in its
// repository's TODO.md).
// Phoenix refreshes the signals of one CAN network per RefreshAll: one call
// over both returned InvalidNetwork and refreshed none, so every motor and
// CAN sensor on both buses read invalid and the gateway saw total feedback
// loss; that robot would never move. Each network's devices read valid,
// fresh feedback.
// REQ-SAFE-22
TEST(PhoenixBackendBuses, DevicesOnTwoCanNetworksReadValidFeedback) {
  EnsureHal();
  hw::Config config;
  const auto encoder = [](uint16_t id, const std::string& name,
                          const std::string& bus) {
    hw::SensorConfig s;
    s.id = id;
    s.name = name;
    s.kind = hw::SensorKind::kCANcoder;
    s.can_id = id;
    s.bus = bus;
    return s;
  };
  config.motors.push_back(TestMotor(41, "rio_motor", 41));
  config.motors.push_back(TestMotor(42, "shooter_motor", 42));
  config.motors.back().bus = "Shooter";
  config.sensors.push_back(encoder(43, "rio_encoder", "rio"));
  config.sensors.push_back(encoder(44, "shooter_encoder", "Shooter"));
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  EXPECT_TRUE(AwaitReady(backend, config));
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

// Desktop simulation drives each motor through WPILib's DCMotorSystem plant
// (motor type, FOC variant, and gear-reflected inertia) and forwards the
// simulated RoboRIO battery voltage to each device rather than assuming a
// fixed 100 rps / 12 V lag.
// REQ-DEV-06
TEST(PhoenixBackendSim, WpilibPlantReflectsMotorTypeFocGearingAndSupply) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  auto x60 = TestMotor(1, "x60", 1);
  auto x60_foc = TestMotor(2, "x60_foc", 2);
  x60_foc.foc = true;
  auto x44 = TestMotor(3, "x44", 3);
  x44.motor_type.value = hw::MotorType::kKrakenX44;
  auto geared_x60 = TestMotor(4, "geared_x60", 4);
  geared_x60.sensor_to_mechanism_ratio = 5.0;
  config.motors = {x60, x60_foc, x44, geared_x60};

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  const std::array<hw::MotorRequest, 4> drive_6v{
      Request(1, hw::Mode::kVoltage, 6.0), Request(2, hw::Mode::kVoltage, 6.0),
      Request(3, hw::Mode::kVoltage, 6.0), Request(4, hw::Mode::kVoltage, 6.0)};
  hw::State state;
  PumpReads(backend, state, drive_6v, 35);
  for (std::size_t i = 0; i < 4; ++i) ASSERT_TRUE(state.motors[i].valid) << i;

  // Kraken X44 (7758 rpm) spins noticeably faster than Kraken X60 (6000 rpm),
  // which spins slightly faster unloaded than Kraken X60 FOC (5800 rpm);
  // gearing divides reported mechanism speed while preserving rotor dynamics.
  EXPECT_GT(state.motors[0].velocity_rps, 45.0);
  EXPECT_GT(state.motors[2].velocity_rps, state.motors[0].velocity_rps * 1.20);
  EXPECT_GT(state.motors[0].velocity_rps, state.motors[1].velocity_rps + 1.0);
  EXPECT_NEAR(state.motors[3].velocity_rps, state.motors[0].velocity_rps / 5.0,
              0.5);

  // Forwarding RoboRIO battery voltage: duty cycle 1.0 at an injected 8 V
  // battery applies ~8 V instead of hardcoded 12 V, without overwriting the
  // injected RoboRioSim battery reading.
  const auto prev_v = frc::sim::RoboRioSim::GetVInVoltage();
  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{8.0});
  struct RestoreBattery {
    units::volt_t volts;
    ~RestoreBattery() { frc::sim::RoboRioSim::SetVInVoltage(volts); }
  } restore{prev_v};

  const std::array<hw::MotorRequest, 4> full_duty{
      Request(1, hw::Mode::kDutyCycle, 1.0),
      Request(2, hw::Mode::kNeutral, 0.0), Request(3, hw::Mode::kNeutral, 0.0),
      Request(4, hw::Mode::kNeutral, 0.0)};
  PumpReads(backend, state, full_duty, 30);
  EXPECT_DOUBLE_EQ(state.health.battery_v, 8.0);
  EXPECT_NEAR(state.motors[0].voltage, 8.0, 0.5);

  // Dropping the simulated battery below Phoenix's 4 V floor (12.8 V -> 3.0 V)
  // updates simulated device supply to 4.0 V rather than leaving it stuck at
  // 12.8 V, while keeping the raw 3.0 V RoboRIO battery reading untouched.
  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{hw::kSimRestingSupplyV});
  PumpReads(backend, state, full_duty, 20);
  EXPECT_DOUBLE_EQ(state.health.battery_v, hw::kSimRestingSupplyV);
  EXPECT_GT(state.motors[0].voltage, 11.5);

  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{3.0});
  PumpReads(backend, state, full_duty, 30);
  EXPECT_DOUBLE_EQ(state.health.battery_v, 3.0);
  EXPECT_NEAR(state.motors[0].voltage, 4.0, 0.5);
  backend.Neutral();
}

// Under desktop simulation with kRemoteCANcoder and sensor_to_mechanism_ratio
// != 1, the simulated CANcoder sits on the sensor shaft (rotor /
// rotor_to_sensor_ratio = mechanism * sensor_to_mechanism_ratio) so Phoenix 6's
// TalonFX divides by SensorToMechanismRatio once, reporting rotor / (R * S)
// rather than rotor / (R * S^2).
// REQ-DEV-06
TEST(PhoenixBackendSim, RemoteCancoderAppliesSensorToMechanismRatioOnce) {
  EnsureHal();
  EnabledDs ds;

  constexpr double kRotorToSensor = 4.0;
  constexpr double kSensorToMechanism = 3.0;
  constexpr double kTotalRatio = kRotorToSensor * kSensorToMechanism;

  hw::Config config;
  auto rotor_ref = TestMotor(1, "rotor_ref", 1);
  hw::SensorConfig enc{10, "remote_enc", hw::SensorKind::kCANcoder, 10, "rio"};
  config.sensors.push_back(enc);

  auto remote = TestMotor(2, "remote_motor", 2);
  remote.feedback = hw::Feedback::kRemoteCANcoder;
  remote.feedback_sensor_id = enc.id;
  remote.rotor_to_sensor_ratio = kRotorToSensor;
  remote.sensor_to_mechanism_ratio = kSensorToMechanism;
  config.motors = {rotor_ref, remote};

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  const std::array<hw::MotorRequest, 2> drive_6v{
      Request(1, hw::Mode::kVoltage, 6.0), Request(2, hw::Mode::kVoltage, 6.0)};
  hw::State state;
  // Wait for rotor_ref to approach its ~50 rps 6 V terminal speed with steady
  // fresh reads so velocity comparisons see near-zero acceleration even when
  // host scheduling delays individual status frames.
  int settled = 0;
  ASSERT_TRUE(ReadUntil(backend, state,
                        [&] {
                          const bool ready =
                              FeedbackFresh(state) && state.motors[0].valid &&
                              state.motors[1].valid && state.sensors[0].valid &&
                              state.motors[0].velocity_rps > 48.0 &&
                              state.motors[0].position_rot > 1.0;
                          settled = ready ? settled + 1 : 0;
                          if (settled >= kSteadyReads) return true;
                          backend.Apply(drive_6v);
                          return false;
                        }))
      << "motors did not reach steady 6 V speed; rotor_ref vel="
      << state.motors[0].velocity_rps
      << " pos=" << state.motors[0].position_rot;

  // CANcoder sits on the sensor shaft: rotor / rotor_to_sensor_ratio.
  EXPECT_NEAR(state.sensors[0].velocity_rps,
              state.motors[0].velocity_rps / kRotorToSensor, 0.5);
  // TalonFX divides CANcoder by sensor_to_mechanism_ratio once:
  // mechanism = rotor / (rotor_to_sensor_ratio * sensor_to_mechanism_ratio).
  EXPECT_NEAR(state.motors[1].velocity_rps,
              state.motors[0].velocity_rps / kTotalRatio, 0.25);

  // Brake both motors to standstill before comparing positions: Phoenix's
  // simulator runs on the wall clock and refreshes CANcoder and remote TalonFX
  // frames asynchronously, so a moving mechanism skews sensors[0].position_rot
  // against motors[1].position_rot * kSensorToMechanism on a loaded host.
  backend.Neutral();
  int stopped = 0;
  ASSERT_TRUE(ReadUntil(backend, state,
                        [&] {
                          const bool still =
                              FeedbackFresh(state) && state.motors[0].valid &&
                              state.motors[1].valid && state.sensors[0].valid &&
                              std::abs(state.motors[0].velocity_rps) < 1e-3 &&
                              std::abs(state.motors[1].velocity_rps) < 1e-3 &&
                              std::abs(state.sensors[0].velocity_rps) < 1e-3;
                          stopped = still ? stopped + 1 : 0;
                          if (stopped >= kSteadyReads) return true;
                          backend.Neutral();
                          return false;
                        }))
      << "motors did not stop under brake neutral; vel=("
      << state.motors[0].velocity_rps << ", " << state.motors[1].velocity_rps
      << ", " << state.sensors[0].velocity_rps << ")";
  ASSERT_GT(state.motors[0].position_rot, 1.0);
  EXPECT_NEAR(state.motors[1].position_rot,
              state.motors[0].position_rot / kTotalRatio, 0.15);
  EXPECT_NEAR(
      std::remainder(state.sensors[0].position_rot -
                         state.motors[1].position_rot * kSensorToMechanism,
                     1.0),
      0.0, 0.05);
}

// Under desktop simulation, inferred swerve or differential chassis kinematics
// integrate wheel motion into Pigeon2 yaw and yaw rate, while a non-chassis
// configuration leaves manually injected SetRawYaw untouched.
// REQ-DEV-06
TEST(PhoenixBackendPigeon,
     ChassisKinematicsDrivesYawAndNonChassisPreservesManualYaw) {
  EnsureHal();
  EnabledDs ds;

  // 1. Swerve chassis spinning in place CCW+, then reversing CW-.
  {
    hw::Config config;
    const std::array<const char*, 4> corners{"front_left", "front_right",
                                             "back_left", "back_right"};
    const std::array<double, 4> spin_angles{0.375, 0.125, -0.375, -0.125};
    for (std::size_t i = 0; i < corners.size(); ++i) {
      auto drive = TestMotor(static_cast<uint16_t>(2 * i + 1),
                             std::string(corners[i]) + "_drive",
                             static_cast<int>(2 * i + 1));
      drive.sensor_to_mechanism_ratio = 6.75;
      auto steer = TestMotor(static_cast<uint16_t>(2 * i + 2),
                             std::string(corners[i]) + "_steer",
                             static_cast<int>(2 * i + 2));
      steer.sensor_to_mechanism_ratio = 12.8;
      steer.slots[0].p = 6.0;
      config.motors.push_back(drive);
      config.motors.push_back(steer);
    }
    config.sensors.push_back(
        hw::SensorConfig{20, "imu", hw::SensorKind::kPigeon2, 30, "rio"});

    PhoenixBackend backend(true);
    ASSERT_TRUE(backend.Configure(config));
    ASSERT_TRUE(AwaitReady(backend, config));

    std::array<hw::MotorRequest, 8> cmd{};
    for (std::size_t i = 0; i < 4; ++i) {
      cmd[2 * i] =
          Request(static_cast<uint16_t>(2 * i + 1), hw::Mode::kVoltage, 6.0);
      cmd[2 * i + 1] = Request(static_cast<uint16_t>(2 * i + 2),
                               hw::Mode::kPosition, spin_angles[i]);
    }
    hw::State state;
    PumpReads(backend, state, cmd, 35);
    ASSERT_TRUE(state.sensors[0].valid);
    EXPECT_GT(state.sensors[0].velocity_rps, 0.1);
    EXPECT_GT(state.sensors[0].position_rot, 0.05);

    for (std::size_t i = 0; i < 4; ++i) {
      cmd[2 * i] =
          Request(static_cast<uint16_t>(2 * i + 1), hw::Mode::kVoltage, -6.0);
    }
    PumpReads(backend, state, cmd, 35);
    EXPECT_LT(state.sensors[0].velocity_rps, -0.1);
    backend.Neutral();
  }

  // 2. Differential chassis fallback (left/right drive pairs, 0.60 m track).
  {
    hw::Config config;
    auto left = TestMotor(1, "left_drive", 1);
    left.sensor_to_mechanism_ratio = 6.75;
    auto right = TestMotor(2, "right_drive", 2);
    right.sensor_to_mechanism_ratio = 6.75;
    config.motors = {left, right};
    config.sensors.push_back(
        hw::SensorConfig{20, "imu", hw::SensorKind::kPigeon2, 30, "rio"});

    PhoenixBackend backend(true);
    ASSERT_TRUE(backend.Configure(config));
    ASSERT_TRUE(AwaitReady(backend, config));

    const std::array<hw::MotorRequest, 2> spin_ccw{
        Request(1, hw::Mode::kVoltage, -6.0),
        Request(2, hw::Mode::kVoltage, 6.0)};
    hw::State state;
    PumpReads(backend, state, spin_ccw, 30);
    ASSERT_TRUE(state.sensors[0].valid);
    EXPECT_GT(state.sensors[0].velocity_rps, 0.1);
    EXPECT_GT(state.sensors[0].position_rot, 0.05);
    backend.Neutral();
  }

  // 3. Non-chassis config under desktop simulation keeps manual SetRawYaw.
  {
    hw::Config config;
    config.motors.push_back(TestMotor(1, "roller", 1));
    config.sensors.push_back(
        hw::SensorConfig{20, "imu", hw::SensorKind::kPigeon2, 30, "rio"});

    PhoenixBackend backend(true);
    ASSERT_TRUE(backend.Configure(config));
    ASSERT_TRUE(AwaitReady(backend, config));

    ctre::phoenix6::hardware::Pigeon2 observer(30,
                                               ctre::phoenix6::CANBus{"rio"});
    observer.GetSimState().SetRawYaw(units::degree_t{90.0});
    const std::array<hw::MotorRequest, 1> neutral{
        Request(1, hw::Mode::kNeutral, 0.0)};
    hw::State state;
    PumpReads(backend, state, neutral, 15);
    ASSERT_TRUE(state.sensors[0].valid);
    EXPECT_NEAR(state.sensors[0].position_rot, 0.25, 1e-3);
  }
}

// Remote-CANcoder swerve modules with nonzero magnet offset and inverted
// sensor/steer direction close their steer loops on the CANcoder's reported
// angle (wrap(steer_sign * raw + offset_rot)), so chassis kinematics must apply
// both steer_sign and offset_rot to integrate Pigeon2 yaw CCW+ rather than
// cancelling or reversing it.
// REQ-DEV-06
TEST(PhoenixBackendPigeon,
     SwerveRemoteCancoderWithMagnetOffsetAndInversionDrivesYawCcw) {
  EnsureHal();
  EnabledDs ds;

  hw::Config config;
  const std::array<const char*, 4> corners{"front_left", "front_right",
                                           "back_left", "back_right"};
  const std::array<double, 4> spin_angles{0.375, 0.125, -0.375, -0.125};
  for (std::size_t i = 0; i < corners.size(); ++i) {
    const auto enc_id = static_cast<uint16_t>(10 + i);
    const int enc_can_id = static_cast<int>(10 + i);
    hw::SensorConfig enc{enc_id, std::string(corners[i]) + "_enc",
                         hw::SensorKind::kCANcoder, enc_can_id, "rio"};
    enc.inverted = true;
    enc.offset_rot = 0.25;
    config.sensors.push_back(enc);

    auto drive = TestMotor(static_cast<uint16_t>(2 * i + 1),
                           std::string(corners[i]) + "_drive",
                           static_cast<int>(2 * i + 1));
    drive.sensor_to_mechanism_ratio = 6.75;

    auto steer = TestMotor(static_cast<uint16_t>(2 * i + 2),
                           std::string(corners[i]) + "_steer",
                           static_cast<int>(2 * i + 2));
    steer.inverted = true;
    steer.feedback = hw::Feedback::kRemoteCANcoder;
    steer.feedback_sensor_id = enc_id;
    steer.rotor_to_sensor_ratio = 12.8;
    steer.sensor_to_mechanism_ratio = 1.0;
    steer.continuous_wrap = true;
    steer.slots[0].p = 12.0;

    config.motors.push_back(drive);
    config.motors.push_back(steer);
  }
  config.sensors.push_back(
      hw::SensorConfig{20, "imu", hw::SensorKind::kPigeon2, 30, "rio"});

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  std::array<hw::MotorRequest, 8> cmd{};
  for (std::size_t i = 0; i < 4; ++i) {
    cmd[2 * i] =
        Request(static_cast<uint16_t>(2 * i + 1), hw::Mode::kVoltage, 6.0);
    cmd[2 * i + 1] = Request(static_cast<uint16_t>(2 * i + 2),
                             hw::Mode::kPosition, spin_angles[i]);
  }
  hw::State state;
  PumpReads(backend, state, cmd, 50);

  // Steer CANcoders should have converged to the requested spin azimuths.
  for (std::size_t i = 0; i < 4; ++i) {
    ASSERT_TRUE(state.sensors[i].valid) << i;
    const double err =
        std::remainder(state.sensors[i].position_rot - spin_angles[i], 1.0);
    EXPECT_NEAR(err, 0.0, 0.05) << "corner " << i;
  }
  ASSERT_TRUE(state.sensors[4].valid);
  EXPECT_GT(state.sensors[4].velocity_rps, 0.1);
  EXPECT_GT(state.sensors[4].position_rot, 0.05);
  backend.Neutral();
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
  // The signals Read refreshes: these tests' devices are all on "rio", one
  // network, so one span covers them.
  static void Refreshed(PhoenixBackend& backend) {
    ASSERT_EQ(backend.refresh_group_count_, 1u);
    backend.refresh_groups_[0] = {0, backend.refresh_signals_.size()};
  }
  static void StopRefreshing(PhoenixBackend& backend,
                             ctre::phoenix6::BaseStatusSignal& signal) {
    std::erase(backend.refresh_signals_, &signal);
    Refreshed(backend);
  }
  static void ResumeRefreshing(PhoenixBackend& backend,
                               ctre::phoenix6::BaseStatusSignal& signal) {
    backend.refresh_signals_.push_back(&signal);
    Refreshed(backend);
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
  static std::size_t UnavailableCapacity(const PhoenixBackend& backend) {
    return backend.unavailable_.capacity();
  }
  static std::size_t RestoringIdsCapacity(const PhoenixBackend& backend) {
    return backend.restoring_ids_.capacity();
  }
  static const uint16_t* UnavailableData(const PhoenixBackend& backend) {
    return backend.unavailable_.data();
  }
  static const uint16_t* RestoringIdsData(const PhoenixBackend& backend) {
    return backend.restoring_ids_.data();
  }
  static void PushRestoreOp(PhoenixBackend& backend, uint16_t id,
                            std::function<bool()> op) {
    backend.restore_operations_.push_back({id, std::move(op)});
  }
  static void PopRestoreOp(PhoenixBackend& backend) {
    backend.restore_operations_.pop_back();
  }
  static void SetRestoreException(PhoenixBackend& backend,
                                  std::exception_ptr ex) {
    using Result = decltype(backend.restore_.get());
    std::promise<Result> p;
    p.set_exception(std::move(ex));
    backend.restore_ = p.get_future();
  }
  using RestoreTask = PhoenixBackend::RestoreTask;
  static void SetRestoreLauncher(
      PhoenixBackend& backend,
      std::function<
          std::future<std::optional<std::vector<uint16_t>>>(RestoreTask)>
          launcher) {
    backend.launch_restore_ = std::move(launcher);
  }
  static ::testing::AssertionResult WaitRestoreReady(PhoenixBackend& backend) {
    if (!backend.restore_.valid())
      return ::testing::AssertionFailure() << "restore future is not valid";
    if (backend.restore_.wait_for(kReadyTimeout) != std::future_status::ready)
      return ::testing::AssertionFailure() << "restore stayed pending";
    return ::testing::AssertionSuccess();
  }
  static void SetConfigureFuture(PhoenixBackend& backend,
                                 std::future<bool> future) {
    backend.configure_ = std::move(future);
  }
  static ::testing::AssertionResult WaitConfigureReady(
      PhoenixBackend& backend) {
    if (!backend.configure_.valid())
      return ::testing::AssertionFailure() << "configure future is not valid";
    if (backend.configure_.wait_for(kReadyTimeout) != std::future_status::ready)
      return ::testing::AssertionFailure() << "configure stayed pending";
    return ::testing::AssertionSuccess();
  }
  static void StopRefreshingHealth(PhoenixBackend& backend) {
    auto& motor = Motor(backend, 0);
    std::erase_if(backend.refresh_signals_, [&](auto* signal) {
      return signal == &motor.GetDeviceTemp(false) ||
             signal == &motor.GetSupplyCurrent(false);
    });
    Refreshed(backend);
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

// A Phoenix Follower copies its leader's output status signal, whose rate is
// the follower's update rate and must be no slower than 20 Hz
// (controls/Follower.hpp). A master outputs DutyCycle in duty-cycle mode and
// MotorVoltage in the voltage-based ones; at the slowest status rate config
// load allows (status_hz 8 at the 500 ms timeout cap) both must still reach
// 20 Hz.
//
// Not faster than MotorVoltage either: on CAN 2.0 the two share the TalonFX
// MotorOutput status frame (SpnValue.hpp's PRO_MotorOutput_DutyCycle and
// PRO_MotorOutput_MotorVoltage, both 100 Hz by default where position and
// velocity are 50 Hz), and Phoenix runs a frame at its fastest request, so
// DutyCycle at status_hz (200 Hz on the bench) would undo MotorVoltage's
// 50 Hz cap. The simulator runs every signal but FaultField in one frame,
// so the master's DutyCycle request shows only with position and velocity
// requested no faster than the capped group.
// REQ-CONTRACT-40
TEST_F(PhoenixBackendRecovery, AMasterWithFollowersSendsItsOutputAt20Hz) {
  EnsureHal();
  for (const double status_hz : {200.0, 8.0}) {
    SCOPED_TRACE(status_hz);
    hw::Config config;
    config.status_hz = status_hz;
    config.command_timeout_us = 500'000;
    auto follower = TestMotor(20, "follower", 2);
    follower.master_id = 10;
    config.motors = {TestMotor(10, "lift", 1), follower};
    PhoenixBackend backend(true);
    ASSERT_TRUE(backend.Configure(config));
    ASSERT_TRUE(AwaitReady(backend, config));
    auto& master = Motor(backend, 0);
    EXPECT_GE(master.GetDutyCycle(false).GetAppliedUpdateFrequency().value(),
              20.0);
    EXPECT_GE(master.GetMotorVoltage(false).GetAppliedUpdateFrequency().value(),
              20.0);
    if (status_hz > 50.0) {
      for (auto* motion :
           std::initializer_list<ctre::phoenix6::BaseStatusSignal*>{
               &master.GetPosition(false), &master.GetVelocity(false)}) {
        ASSERT_TRUE(AwaitAnswer([&] {
                      return motion->SetUpdateFrequency(units::hertz_t{50});
                    }).IsOK());
      }
      EXPECT_DOUBLE_EQ(
          master.GetDutyCycle(false).GetAppliedUpdateFrequency().value(), 50.0)
          << "the master's DutyCycle is requested faster than MotorVoltage";
    }

    // Lower every master signal below 20 Hz, then verify RestorePass restores
    // DutyCycle and MotorVoltage to >= 20 Hz for both status_hz = 8 and 200.
    ASSERT_TRUE(
        AwaitAnswer([&] {
          return ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
              units::hertz_t{4}, master.GetPosition(false),
              master.GetVelocity(false), master.GetMotorVoltage(false),
              master.GetDutyCycle(false), master.GetStatorCurrent(false),
              master.GetSupplyCurrent(false), master.GetDeviceTemp(false));
        }).IsOK());
    ASSERT_LT(master.GetDutyCycle(false).GetAppliedUpdateFrequency().value(),
              20.0);
    ASSERT_LT(master.GetMotorVoltage(false).GetAppliedUpdateFrequency().value(),
              20.0);
    ASSERT_TRUE(RestorePass(backend, std::array<uint16_t, 1>{10}));
    EXPECT_GE(master.GetDutyCycle(false).GetAppliedUpdateFrequency().value(),
              20.0);
    EXPECT_GE(master.GetMotorVoltage(false).GetAppliedUpdateFrequency().value(),
              20.0);
    if (status_hz > 50.0) {
      ASSERT_TRUE(
          AwaitAnswer([&] {
            return ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
                units::hertz_t{4}, master.GetPosition(false),
                master.GetVelocity(false), master.GetStatorCurrent(false),
                master.GetSupplyCurrent(false));
          }).IsOK());
      EXPECT_DOUBLE_EQ(
          master.GetDutyCycle(false).GetAppliedUpdateFrequency().value(), 50.0);
      EXPECT_DOUBLE_EQ(
          master.GetMotorVoltage(false).GetAppliedUpdateFrequency().value(),
          50.0);
    }
  }
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
// motor (OverSupplyV here, raised deterministically through the simulator's
// supply-voltage input with no wall-clock window). Each row first waits until
// a read shows its bit: the reads that PumpReads gates on fresh motion
// feedback say nothing of the separate 50 Hz FaultField, and one that finds it
// 50 ms old reports no limit by this same rule.
// REQ-CONTRACT-32
TEST_F(PhoenixBackendRecovery, StaleFaultFieldClearsLimitsAndKeepsFaults) {
  enum class Row { kForwardHardLimit, kStatorLimiter, kOverSupplyFault };
  // FaultField bit 18, OverSupplyV: a fault (hw::MotorFaulted).
  constexpr uint32_t kOverSupplyVFlag = uint32_t{1} << 18;
  static_assert(hw::MotorFaulted(kOverSupplyVFlag));
  for (const Row row :
       {Row::kForwardHardLimit, Row::kStatorLimiter, Row::kOverSupplyFault}) {
    const uint32_t bit =
        row == Row::kForwardHardLimit ? hw::kMotorForwardHardLimitFlag
        : row == Row::kStatorLimiter  ? hw::kMotorCurrentLimitFlags
                                      : kOverSupplyVFlag;
    SCOPED_TRACE(row == Row::kForwardHardLimit ? "forward hard limit"
                 : row == Row::kStatorLimiter  ? "stator current limit"
                                               : "over-supply voltage fault");
    EnsureHal();
    EnabledDs ds;
    hw::Config config;
    auto motor = TestMotor(1, "motor", 1);
    motor.stator_limit_a = 10;
    config.motors.push_back(motor);
    PhoenixBackend backend(true);
    ASSERT_TRUE(backend.Configure(config));
    ASSERT_TRUE(AwaitReady(backend, config));
    const std::array<hw::MotorRequest, 1> commands{
        Request(1, hw::Mode::kVoltage, row == Row::kStatorLimiter ? 6.0 : 2.0)};
    hw::State state;
    auto& sim = Motor(backend, 0).GetSimState();
    struct SimReset {
      ctre::phoenix6::sim::TalonFXSimState& sim;
      ~SimReset() {
        sim.SetForwardLimit(false);
        sim.SetSupplyVoltage(units::volt_t{12.0});
      }
    } reset{sim};
    if (row == Row::kStatorLimiter) StallRotor(backend);
    if (row == Row::kForwardHardLimit) sim.SetForwardLimit(true);
    if (row == Row::kOverSupplyFault) sim.SetSupplyVoltage(units::volt_t{30.0});
    if (row != Row::kOverSupplyFault) PumpReads(backend, state, commands, 30);
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
    if (row == Row::kOverSupplyFault) {
      EXPECT_NE(state.motors[0].fault_flags & bit, 0u)
          << "fault_flags " << state.motors[0].fault_flags;
      EXPECT_TRUE(hw::MotorFaulted(state.motors[0].fault_flags))
          << "fault_flags " << state.motors[0].fault_flags;
      // Restore nominal supply voltage and wait for the simulated TalonFX to
      // clear OverSupplyV so later rows and tests on CAN id 1 start clean.
      sim.SetSupplyVoltage(units::volt_t{12.0});
      ResumeRefreshing(backend, faults);
      ASSERT_TRUE(ReadUntil(backend, state, [&] {
        return (state.motors[0].fault_flags & kOverSupplyVFlag) == 0;
      })) << "OverSupplyV did not clear after restoring 12 V";
    } else {
      EXPECT_EQ(state.motors[0].fault_flags & hw::kMotorNotFaultFlags, 0u)
          << "fault_flags " << state.motors[0].fault_flags;
    }
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
      std::array<hw::PwmRequest, 1>{{{50, hw::PwmMode::kPulse, 0.75}}}));
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

class PhoenixBackendFoc : public ::testing::Test {
 protected:
  static std::string AppliedControl(PhoenixBackend& backend, std::size_t i) {
    const auto control = backend.motors_[i]->GetAppliedControl();
    return std::string{control->GetName()};
  }
  // Motor `i`'s last control request, every field as Phoenix reports it.
  static std::map<std::string, std::string> AppliedFields(
      PhoenixBackend& backend, std::size_t i) {
    return backend.motors_[i]->GetAppliedControl()->GetControlInfo();
  }
  // The EnableFOC field of motor `i`'s last control request.
  static std::string AppliedEnableFoc(PhoenixBackend& backend, std::size_t i) {
    const auto info = AppliedFields(backend, i);
    const auto field = info.find("EnableFOC");
    return field == info.end() ? "(no EnableFOC field)" : field->second;
  }
  static ctre::phoenix6::hardware::TalonFX& Motor(PhoenixBackend& backend,
                                                  std::size_t i) {
    return *backend.motors_[i];
  }
  static std::string EnableFoc(bool enabled) {
    return ctre::phoenix6::controls::VoltageOut{units::volt_t{0}}
        .WithEnableFOC(enabled)
        .GetControlInfo()
        .at("EnableFOC");
  }
  // Stands in for each motor's IsProLicensed read: true or false as read,
  // nullopt when the read gets no answer.
  static void SetProLicensed(
      PhoenixBackend& backend,
      std::function<std::optional<bool>(std::size_t)> licensed) {
    backend.pro_licensed_ = std::move(licensed);
  }
  static void PushRestoreOp(PhoenixBackend& backend, uint16_t id,
                            std::function<bool()> op) {
    backend.restore_operations_.push_back({id, std::move(op)});
  }
  // Configures until no device's answer was lost to a stalled simulator
  // (AwaitAnswer's rule), so the first configure error is the one under
  // test, not a transport timeout recorded before it. A licence read the
  // seam leaves unanswered is the one under test.
  static ::testing::AssertionResult ConfigureAnswered(
      PhoenixBackend& backend, const hw::Config& config) {
    const auto deadline = std::chrono::steady_clock::now() + kReadyTimeout;
    for (;;) {
      if (!backend.Configure(config))
        return ::testing::AssertionFailure() << "Configure refused";
      const auto error = backend.last_config_error();
      if (!error || !IsTransportTimeout(error->vendor_status) ||
          error->operation == "check_pro_license")
        return ::testing::AssertionSuccess();
      if (std::chrono::steady_clock::now() >= deadline)
        return ::testing::AssertionFailure() << hw::FormatConfigError(*error);
    }
  }
  // Restores the devices a stalled simulator left unavailable, as AwaitReady
  // does; AwaitReady itself refuses a configure error that is not a
  // transport timeout, and check_pro_license is one.
  static ::testing::AssertionResult RestoreUnavailable(
      PhoenixBackend& backend) {
    const auto deadline = std::chrono::steady_clock::now() + kReadyTimeout;
    while (!backend.UnavailableDevices().empty()) {
      backend.BeginRestoreAfterReset(backend.UnavailableDevices());
      while (backend.PollRestore() == hw::Backend::RestoreStatus::kPending) {
        if (std::chrono::steady_clock::now() >= deadline)
          return ::testing::AssertionFailure() << "restore still pending";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (std::chrono::steady_clock::now() >= deadline)
        return ::testing::AssertionFailure() << "devices still unavailable";
    }
    return ::testing::AssertionSuccess();
  }
  // Applies `commands` until no send fails: a stalled simulator can fail
  // one (TxFailed), and the backend then sends that motor NeutralOut.
  static ::testing::AssertionResult ApplySent(
      PhoenixBackend& backend, std::span<const hw::MotorRequest> commands) {
    const auto deadline = std::chrono::steady_clock::now() + kReadyTimeout;
    for (;;) {
      if (!backend.Apply(commands))
        return ::testing::AssertionFailure() << "Apply refused";
      if (backend.ApplyFailedDevices().empty())
        return ::testing::AssertionSuccess();
      if (std::chrono::steady_clock::now() >= deadline)
        return ::testing::AssertionFailure() << "a send kept failing";
    }
  }
};

class PhoenixBackendFocModes : public PhoenixBackendFoc,
                               public ::testing::WithParamInterface<bool> {};

// Every request that has the flag carries EnableFOC exactly when the motor's
// config sets foc: duty cycle included, one rule for all five.
// REQ-CONTRACT-39
TEST_P(PhoenixBackendFocModes, EveryVoltageFamilyRequestCarriesTheMotorsFoc) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  auto motor = TestMotor(1, "motor", 1);
  motor.foc = GetParam();
  config.motors.push_back(motor);
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  ASSERT_NE(EnableFoc(true), EnableFoc(false));
  const std::array<std::pair<hw::Mode, std::string_view>, 5> modes{{
      {hw::Mode::kDutyCycle, "DutyCycleOut"},
      {hw::Mode::kVoltage, "VoltageOut"},
      {hw::Mode::kVelocity, "VelocityVoltage"},
      {hw::Mode::kPosition, "PositionVoltage"},
      {hw::Mode::kMotionMagic, "MotionMagicVoltage"},
  }};
  for (const auto& [mode, control] : modes) {
    const std::array<hw::MotorRequest, 1> commands{Request(1, mode, 0.0)};
    ASSERT_TRUE(ApplySent(backend, commands)) << control;
    EXPECT_EQ(AppliedControl(backend, 0), control);
    EXPECT_EQ(AppliedEnableFoc(backend, 0), EnableFoc(GetParam())) << control;
  }
}

INSTANTIATE_TEST_SUITE_P(Foc, PhoenixBackendFocModes, ::testing::Bool());

// The RoboRIO console of a backend: its RioLog, not started, so the test
// drains it on its own thread.
class Console {
 public:
  Console()
      : log_{[this](std::string_view line) { lines_.emplace_back(line); }} {}
  talos::rio::RioLog* log() { return &log_; }
  // Every line written so far that contains each of `words`.
  std::vector<std::string> Lines(
      std::initializer_list<std::string_view> words) {
    log_.Flush(static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count()));
    std::vector<std::string> found;
    for (const auto& line : lines_) {
      if (std::all_of(words.begin(), words.end(), [&](std::string_view word) {
            return line.find(word) != std::string::npos;
          }))
        found.push_back(line);
    }
    return found;
  }

 private:
  std::vector<std::string> lines_;
  talos::rio::RioLog log_;
};

// A foc motor read not Phoenix Pro licensed at configure is a failed device:
// invalid, zeroed samples, NeutralOut whatever it is asked, a console line
// naming it, and its follower with it, while a licensed foc motor drives.
// A follower of a licensed master is read for itself: read unlicensed, it
// fails alone and its master drives. Each failed motor gets its own line,
// though only the first is the configure's error. It is not left to the
// restore loop. The next configure that reads it licensed clears it. The
// intake asks for torque current, which needs the licence too: NeutralOut
// all the same.
// REQ-CONTRACT-39
// REQ-SAFE-43
TEST_F(PhoenixBackendFoc,
       AnUnlicensedMotorAndItsFollowerFailUntilReconfigured) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  auto lift = TestMotor(10, "lift", 1);
  lift.foc = true;
  auto follower = TestMotor(20, "follower", 2);
  follower.master_id = 10;
  auto roller = TestMotor(30, "roller", 3);
  roller.foc = true;
  auto intake = TestMotor(40, "intake", 4);
  intake.foc = true;
  intake.max_torque_current_a = 5.0;
  auto roller_follower = TestMotor(50, "roller_follower", 5);
  roller_follower.master_id = 30;
  config.motors = {lift, follower, roller, intake, roller_follower};
  Console console;
  PhoenixBackend backend(true, console.log());
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  std::vector<std::size_t> checked;
  SetProLicensed(backend, [&checked](std::size_t i) {
    checked.push_back(i);
    // The lift, the intake and the roller's follower are unlicensed.
    return i != 0 && i != 3 && i != 4;
  });
  ASSERT_TRUE(ConfigureAnswered(backend, config));
  // The last configure read the lift, the roller and the intake for their
  // own foc, and the roller's follower for its master's; the lift's
  // follower failed with the lift unread. (A stalled simulator can make
  // ConfigureAnswered configure more than once.)
  ASSERT_GE(checked.size(), 4u);
  EXPECT_EQ(std::vector<std::size_t>(checked.end() - 4, checked.end()),
            (std::vector<std::size_t>{0, 2, 3, 4}));
  const auto error = backend.last_config_error();
  ASSERT_TRUE(error);
  EXPECT_EQ(error->operation, "check_pro_license");
  EXPECT_EQ(error->device_name, "lift");
  EXPECT_EQ(error->can_id, 1);
  for (const auto* motor :
       {"'lift'", "'follower'", "'intake'", "'roller_follower'"}) {
    EXPECT_FALSE(
        console.Lines({"[phoenix]", "check_pro_license", motor}).empty())
        << motor;
  }
  EXPECT_TRUE(console.Lines({"check_pro_license", "'roller'"}).empty());
  ASSERT_TRUE(RestoreUnavailable(backend));

  const std::array<hw::MotorRequest, 5> commands{
      Request(10, hw::Mode::kVoltage, 6.0),
      Request(20, hw::Mode::kVoltage, 0.0),
      Request(30, hw::Mode::kVoltage, 6.0),
      Request(40, hw::Mode::kTorqueCurrent, 2.0),
      Request(50, hw::Mode::kVoltage, 0.0)};
  hw::State state;
  PumpReads(backend, state, commands, 30);
  for (const std::size_t i : {0U, 1U, 3U, 4U}) {
    EXPECT_FALSE(state.motors[i].valid) << i;
    EXPECT_EQ(state.motors[i].position_rot, 0.0) << i;
    EXPECT_EQ(state.motors[i].voltage, 0.0) << i;
  }
  EXPECT_TRUE(state.motors[2].valid);
  ASSERT_TRUE(ApplySent(backend, commands));
  EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut");
  EXPECT_EQ(AppliedControl(backend, 1), "NeutralOut");
  EXPECT_EQ(AppliedControl(backend, 2), "VoltageOut");
  EXPECT_EQ(AppliedControl(backend, 3), "NeutralOut");
  EXPECT_EQ(AppliedControl(backend, 4), "NeutralOut");
  EXPECT_TRUE(backend.UnavailableDevices().empty());
  ctre::phoenix6::hardware::TalonFX lift_device{1,
                                                ctre::phoenix6::CANBus{"rio"}};
  ctre::phoenix6::hardware::TalonFX roller_device{
      3, ctre::phoenix6::CANBus{"rio"}};
  EXPECT_NEAR(lift_device.GetSimState().GetMotorVoltage().value(), 0.0, 0.5);
  EXPECT_GT(roller_device.GetSimState().GetMotorVoltage().value(), 3.0);

  SetProLicensed(backend, nullptr);  // Simulation: licensed.
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  PumpReads(backend, state, commands, 30);
  ASSERT_TRUE(ApplySent(backend, commands));
  EXPECT_EQ(AppliedControl(backend, 0), "VoltageOut");
  EXPECT_EQ(AppliedEnableFoc(backend, 0), EnableFoc(true));
  EXPECT_GT(state.motors[0].voltage, 3.0);
  EXPECT_GT(state.motors[1].voltage, 3.0);
}

// A licence read that gets no answer (the device absent, or its frame late)
// is not a refusal: the motor is an unavailable device, invalid and
// neutral, and each restore reads the licence again with the rest of its
// configuration until one gets an answer. Read licensed, it drives, and a
// later restore (a device reset) reads it no more; read unlicensed, it
// fails with its follower until the next configure, named on the console.
// REQ-CONTRACT-39
TEST_F(PhoenixBackendFoc, ALicenceNotReadIsReadAgainByTheRestoreLoop) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  auto lift = TestMotor(10, "lift", 1);
  lift.foc = true;
  auto follower = TestMotor(20, "follower", 2);
  follower.master_id = 10;
  config.motors = {lift, follower};
  Console console;
  PhoenixBackend backend(true, console.log());
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  std::optional<bool> lift_licence;  // No answer.
  int reads = 0;
  SetProLicensed(backend, [&lift_licence, &reads](std::size_t i) {
    ++reads;
    return i == 0 ? lift_licence : std::optional<bool>{true};
  });
  ASSERT_TRUE(ConfigureAnswered(backend, config));
  const auto error = backend.last_config_error();
  ASSERT_TRUE(error);
  EXPECT_EQ(error->operation, "check_pro_license");
  EXPECT_EQ(error->device_name, "lift");
  const auto unavailable = backend.UnavailableDevices();
  EXPECT_NE(std::find(unavailable.begin(), unavailable.end(), 10),
            unavailable.end());
  const std::array<hw::MotorRequest, 2> commands{
      Request(10, hw::Mode::kVoltage, 6.0),
      Request(20, hw::Mode::kVoltage, 0.0)};
  hw::State state;
  // A restore pass that still gets no answer leaves it so, unrefused.
  ASSERT_TRUE(RestorePass(backend, backend.UnavailableDevices()));
  const auto still_unavailable = backend.UnavailableDevices();
  EXPECT_NE(std::find(still_unavailable.begin(), still_unavailable.end(), 10),
            still_unavailable.end());
  backend.Read(state);
  EXPECT_FALSE(state.motors[0].valid);
  ASSERT_TRUE(ApplySent(backend, commands));
  EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut");
  EXPECT_TRUE(console.Lines({"foc needs a Phoenix Pro licence"}).empty());

  // The restore pass reads it licensed: it drives, its follower with it.
  lift_licence = true;
  ASSERT_TRUE(AwaitReady(backend, config));
  PumpReads(backend, state, commands, 30);
  EXPECT_TRUE(state.motors[0].valid);
  EXPECT_GT(state.motors[0].voltage, 3.0);
  EXPECT_GT(state.motors[1].voltage, 3.0);

  // A reset of the licensed motor (and so of its follower): the restore
  // reads neither licence again, and both drive.
  const int reads_before_reset = reads;
  ASSERT_TRUE(RestorePass(backend, std::array<uint16_t, 1>{10}));
  EXPECT_EQ(reads, reads_before_reset);
  EXPECT_TRUE(backend.UnavailableDevices().empty());
  PumpReads(backend, state, commands, 30);
  EXPECT_GT(state.motors[0].voltage, 3.0);
  EXPECT_GT(state.motors[1].voltage, 3.0);

  // Unanswered again, then read unlicensed: a failed device with its
  // follower, no longer retried.
  lift_licence.reset();
  ASSERT_TRUE(ConfigureAnswered(backend, config));
  ASSERT_TRUE(console.Lines({"foc needs a Phoenix Pro licence"}).empty());
  lift_licence = false;
  ASSERT_TRUE(RestoreUnavailable(backend));
  EXPECT_EQ(console
                .Lines({"[phoenix]", "check_pro_license", "'lift'",
                        "foc needs a Phoenix Pro licence"})
                .size(),
            1u);
  EXPECT_FALSE(
      console.Lines({"[phoenix]", "check_pro_license", "'follower'"}).empty());
  PumpReads(backend, state, commands, 30);
  EXPECT_FALSE(state.motors[0].valid);
  EXPECT_FALSE(state.motors[1].valid);
  ASSERT_TRUE(ApplySent(backend, commands));
  EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut");
  EXPECT_EQ(AppliedControl(backend, 1), "NeutralOut");
  EXPECT_TRUE(backend.UnavailableDevices().empty());
}

// A restore pass that fails for one device ends (kDone, so the gateway reads
// UnavailableDevices() and retries with backoff) with that device still
// unavailable: invalid and sent NeutralOut. A device restored in the same
// pass leaves the set and drives. The failure here is the licence read the
// restore repeats (no answer); any failed restore operation takes the same
// path.
// REQ-SAFE-24
TEST_F(PhoenixBackendFoc, AFailedRestoreLeavesOnlyItsDeviceUnavailable) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  auto lift = TestMotor(10, "lift", 1);
  lift.foc = true;
  config.motors = {lift, TestMotor(30, "roller", 3)};
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  SetProLicensed(backend, [](std::size_t i) -> std::optional<bool> {
    return i == 0 ? std::nullopt : std::optional<bool>{true};
  });
  ASSERT_TRUE(ConfigureAnswered(backend, config));
  const auto contains = [](const std::vector<uint16_t>& ids, uint16_t id) {
    return std::find(ids.begin(), ids.end(), id) != ids.end();
  };
  ASSERT_TRUE(contains(backend.UnavailableDevices(), 10));

  // Both in one pass, as after a reset of the roller while the lift waits.
  // The roller's own restore can lose a transport answer to a stalled
  // simulator; such a pass is repeated, and each must leave the lift.
  const std::array<uint16_t, 2> ids{10, 30};
  const auto deadline = std::chrono::steady_clock::now() + kReadyTimeout;
  do {
    backend.BeginRestoreAfterReset(ids);
    auto status = backend.PollRestore();
    while (status == hw::Backend::RestoreStatus::kPending) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline)
          << "restore still pending";
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      status = backend.PollRestore();
    }
    ASSERT_EQ(status, hw::Backend::RestoreStatus::kDone);
    ASSERT_TRUE(contains(backend.UnavailableDevices(), 10));
    ASSERT_LT(std::chrono::steady_clock::now(), deadline)
        << "roller still unavailable";
  } while (contains(backend.UnavailableDevices(), 30));

  const std::array<hw::MotorRequest, 2> commands{
      Request(10, hw::Mode::kVoltage, 6.0),
      Request(30, hw::Mode::kVoltage, 6.0)};
  hw::State state;
  PumpReads(backend, state, commands, 30);
  EXPECT_FALSE(state.motors[0].valid);
  EXPECT_TRUE(state.motors[1].valid);
  ASSERT_TRUE(ApplySent(backend, commands));
  EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut");
  EXPECT_EQ(AppliedControl(backend, 1), "VoltageOut");
}

namespace {

// The simulated RoboRIO's battery reading for one test; 12 V, the
// simulator's default, again when it goes.
class SimBattery {
 public:
  explicit SimBattery(double volts) { Set(volts); }
  ~SimBattery() { Set(12.0); }
  SimBattery(const SimBattery&) = delete;
  SimBattery& operator=(const SimBattery&) = delete;
  static void Set(double volts) {
    frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{volts});
  }
};

}  // namespace

class PhoenixBackendTorque : public PhoenixBackendFoc {
 protected:
  // A motor that may run torque current: foc, up to 5 A, at most 4 V.
  static hw::MotorConfig TorqueMotor(uint16_t id, const std::string& name,
                                     int can_id) {
    auto m = TestMotor(id, name, can_id);
    m.foc = true;
    m.max_torque_current_a = 5.0;
    m.max_voltage = 4.0;
    return m;
  }
  // Every field of the request Apply should send for `amperes` under the
  // duty cap `duty`.
  static std::map<std::string, std::string> TorqueCurrentFoc(double amperes,
                                                             double duty) {
    return ctre::phoenix6::controls::TorqueCurrentFOC{units::ampere_t{amperes}}
        .WithMaxAbsDutyCycle(duty)
        .WithUpdateFreqHz(units::hertz_t{0})
        .GetControlInfo();
  }
  static ::testing::AssertionResult AppliedConfig(
      PhoenixBackend& backend, std::size_t i,
      ctre::phoenix6::configs::TalonFXConfiguration& applied) {
    const auto status = AwaitAnswer(
        [&] { return Motor(backend, i).GetConfigurator().Refresh(applied); });
    if (status.IsOK()) return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << status.GetName();
  }
};

// A torque-current request is TorqueCurrentFOC with the demand in amperes,
// peak torque current +-max_torque_current_a, and a duty cap of max_voltage
// over the last battery reading: none before the first reading (cap 0, no
// output), following the battery, and kept over a lost reading. A neutral
// after it is NeutralOut.
// REQ-SAFE-43
TEST_F(PhoenixBackendTorque, ARequestIsTorqueCurrentFocCappedAtMaxVoltage) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  config.motors.push_back(TorqueMotor(1, "gripper", 1));
  PhoenixBackend backend(true);
  // After the backend, which rests the simulated battery at 12.8 V.
  SimBattery battery{0.0};  // No reading yet.
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));
  ctre::phoenix6::configs::TalonFXConfiguration applied;
  ASSERT_TRUE(AppliedConfig(backend, 0, applied));
  EXPECT_DOUBLE_EQ(applied.TorqueCurrent.PeakForwardTorqueCurrent.value(), 5.0);
  EXPECT_DOUBLE_EQ(applied.TorqueCurrent.PeakReverseTorqueCurrent.value(),
                   -5.0);

  const std::array<hw::MotorRequest, 1> torque{
      Request(1, hw::Mode::kTorqueCurrent, -2.5)};
  ASSERT_TRUE(ApplySent(backend, torque));
  EXPECT_EQ(AppliedFields(backend, 0), TorqueCurrentFoc(-2.5, 0.0))
      << "before a battery reading";
  hw::State state;
  for (const double battery_v : {12.5, 8.0}) {
    battery.Set(battery_v);
    backend.Read(state);
    ASSERT_TRUE(ApplySent(backend, torque));
    EXPECT_EQ(AppliedFields(backend, 0),
              TorqueCurrentFoc(-2.5, 4.0 / battery_v))
        << battery_v << " V";
  }
  battery.Set(0.0);
  backend.Read(state);
  ASSERT_TRUE(ApplySent(backend, torque));
  EXPECT_EQ(AppliedFields(backend, 0), TorqueCurrentFoc(-2.5, 4.0 / 8.0))
      << "a lost reading keeps the last";

  const std::array<hw::MotorRequest, 1> neutral{
      Request(1, hw::Mode::kNeutral, 0.0)};
  ASSERT_TRUE(ApplySent(backend, neutral));
  EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut");
}

// A torque-current request the gateway would have neutralized never drives
// when it reaches Apply anyway: over the limit, not finite, with a
// feedforward, or for a motor whose config does not allow the mode (foc
// without max_torque_current_a; config load refuses the limit without foc).
// Each is that motor's NeutralOut, not an apply failure.
// REQ-SAFE-43
TEST_F(PhoenixBackendTorque, AnOutOfContractRequestIsNeutralOut) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  config.motors.push_back(TorqueMotor(1, "gripper", 1));
  auto no_limit = TorqueMotor(2, "roller", 2);
  no_limit.max_torque_current_a = 0;
  config.motors.push_back(no_limit);
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  auto with_feedforward = Request(1, hw::Mode::kTorqueCurrent, 2.0);
  with_feedforward.feedforward_v = 1.0;
  const std::array<std::pair<hw::MotorRequest, std::string_view>, 4> bad{{
      {Request(1, hw::Mode::kTorqueCurrent, 5.5), "over the limit"},
      {Request(1, hw::Mode::kTorqueCurrent, -5.5), "under the limit"},
      {Request(1, hw::Mode::kTorqueCurrent, std::nan("")), "not finite"},
      {with_feedforward, "with a feedforward"},
  }};
  for (const auto& [request, why] : bad) {
    const std::array<hw::MotorRequest, 2> good{
        Request(1, hw::Mode::kTorqueCurrent, 5.0),
        Request(2, hw::Mode::kTorqueCurrent, 1.0)};
    ASSERT_TRUE(ApplySent(backend, good)) << why;
    ASSERT_EQ(AppliedControl(backend, 0), "TorqueCurrentFOC") << why;
    EXPECT_EQ(AppliedControl(backend, 1), "NeutralOut") << "without a limit";
    const std::array<hw::MotorRequest, 2> commands{request, good[1]};
    ASSERT_TRUE(ApplySent(backend, commands)) << why;
    EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut") << why;
  }
}

// Every NeutralOut that Apply sends (direct kNeutral, a neutralized follower,
// and an out-of-contract torque-current request) must be one-shot (0 Hz),
// matching Neutral() and WatchdogNeutral(), so Phoenix's background control
// thread does not keep resending periodic 20 Hz frames. And the gateway's
// neutral, Neutral(), is that one-shot NeutralOut on every configured motor,
// whatever each was driving: a master, its follower and a torque-current
// motor.
// REQ-SAFE-43
TEST_F(PhoenixBackendTorque, EveryApplyNeutralOutIsOneShot) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  auto master = TestMotor(10, "master", 1);
  auto follower = TestMotor(20, "follower", 2);
  follower.master_id = 10;
  auto gripper = TorqueMotor(30, "gripper", 3);
  config.motors = {master, follower, gripper};
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  const auto expect_one_shot_neutral = [&](std::size_t i,
                                           std::string_view why) {
    const auto applied = Motor(backend, i).GetAppliedControl();
    ASSERT_NE(applied, nullptr) << why;
    const auto neutral =
        std::dynamic_pointer_cast<const ctre::phoenix6::controls::NeutralOut>(
            applied);
    ASSERT_NE(neutral, nullptr) << why << ": got " << applied->GetName();
    EXPECT_DOUBLE_EQ(neutral->UpdateFreqHz.value(), 0.0) << why;
  };

  // Drive every motor first so each NeutralOut replaces an active control.
  const std::array<hw::MotorRequest, 3> drive{
      Request(10, hw::Mode::kVoltage, 4.0),
      Request(20, hw::Mode::kVoltage, 0.0),
      Request(30, hw::Mode::kTorqueCurrent, 2.0)};
  ASSERT_TRUE(ApplySent(backend, drive));

  // Direct kNeutral on master, follower neutralized with master, and
  // out-of-contract kTorqueCurrent (over max_torque_current_a) on gripper.
  const std::array<hw::MotorRequest, 3> neutral_all{
      Request(10, hw::Mode::kNeutral, 0.0),
      Request(20, hw::Mode::kVoltage, 4.0),
      Request(30, hw::Mode::kTorqueCurrent, 10.0)};
  ASSERT_TRUE(ApplySent(backend, neutral_all));
  expect_one_shot_neutral(0, "direct kNeutral");
  expect_one_shot_neutral(1, "follower neutral");
  expect_one_shot_neutral(2, "out-of-contract torque current");

  ASSERT_TRUE(ApplySent(backend, drive));
  ASSERT_EQ(AppliedControl(backend, 2), "TorqueCurrentFOC");
  backend.Neutral();
  for (std::size_t i = 0; i < config.motors.size(); ++i)
    expect_one_shot_neutral(i, "Neutral() on " + config.motors[i].name);
}

class PhoenixBackendPeakVoltage : public PhoenixBackendTorque {};

// The TalonFX's peak voltage, which holds every voltage-based request
// (VoltageOut, VelocityVoltage, PositionVoltage, MotionMagicVoltage) to
// max_voltage on the device: configured at ±max_voltage on every motor
// (independent, master, and follower), and applied again by the restore
// after a reset returned each device to Phoenix's defaults.
// REQ-SAFE-44
TEST_F(PhoenixBackendPeakVoltage, IsMaxVoltageAfterConfigureAndRestore) {
  EnsureHal();
  hw::Config config;
  auto master = TestMotor(1, "roller", 1);
  master.max_voltage = 2.0;
  auto follower = TestMotor(2, "roller_follower", 2);
  follower.master_id = master.id;
  follower.max_voltage = 3.5;
  auto independent = TestMotor(3, "intake", 3);
  independent.max_voltage = 5.0;
  config.motors = {master, follower, independent};
  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  const auto expect_peak_voltage = [&](std::size_t i, double volts,
                                       std::string_view when) {
    ctre::phoenix6::configs::TalonFXConfiguration applied;
    ASSERT_TRUE(AppliedConfig(backend, i, applied))
        << config.motors[i].name << " " << when;
    EXPECT_DOUBLE_EQ(applied.Voltage.PeakForwardVoltage.value(), volts)
        << config.motors[i].name << " " << when;
    EXPECT_DOUBLE_EQ(applied.Voltage.PeakReverseVoltage.value(), -volts)
        << config.motors[i].name << " " << when;
  };

  expect_peak_voltage(0, 2.0, "after configure");
  expect_peak_voltage(1, 3.5, "after configure");
  expect_peak_voltage(2, 5.0, "after configure");

  const ctre::phoenix6::configs::TalonFXConfiguration defaults;
  for (std::size_t i = 0; i < config.motors.size(); ++i) {
    ASSERT_TRUE(AwaitAnswer([&] {
                  return Motor(backend, i).GetConfigurator().Apply(defaults);
                }).IsOK())
        << config.motors[i].name;
    ctre::phoenix6::configs::TalonFXConfiguration applied;
    ASSERT_TRUE(AppliedConfig(backend, i, applied)) << config.motors[i].name;
    ASSERT_NE(applied.Voltage.PeakForwardVoltage.value(),
              config.motors[i].max_voltage)
        << config.motors[i].name << " not reset";
    ASSERT_NE(applied.Voltage.PeakReverseVoltage.value(),
              -config.motors[i].max_voltage)
        << config.motors[i].name << " not reset";
  }

  // Restore the follower on its own first so a restore path that skips
  // followers fails even if restoring the master would re-apply its chain.
  ASSERT_TRUE(RestorePass(backend, std::array<uint16_t, 1>{follower.id}));
  expect_peak_voltage(1, 3.5, "after follower restore");
  ASSERT_TRUE(
      RestorePass(backend, std::array<uint16_t, 2>{master.id, independent.id}));
  expect_peak_voltage(0, 2.0, "after master restore");
  expect_peak_voltage(2, 5.0, "after independent restore");
}

// Asynchronous configuration runs on a worker thread and is polled on the
// SCHED_FIFO RoboRIO loop thread. Any exception thrown inside Configure()
// (validation, WPILib HAL resource allocation) or from configure_.get() must
// report ConfigureStatus::kFailed with a recorded error and never escape onto
// the loop thread.
// REQ-SAFE-21
TEST_F(PhoenixBackendRecovery, AsyncConfigureCatchesExceptionsAsFailure) {
  EnsureHal();
  Console console;
  PhoenixBackend backend(true, console.log());

  // 1. hw::Validate throws std::invalid_argument inside the async worker.
  hw::Config invalid;
  invalid.period_us = 0;
  ASSERT_NO_THROW(backend.BeginConfigure(invalid));
  ASSERT_TRUE(WaitConfigureReady(backend));
  hw::Backend::ConfigureStatus status = hw::Backend::ConfigureStatus::kPending;
  ASSERT_NO_THROW(status = backend.PollConfigure());
  EXPECT_EQ(status, hw::Backend::ConfigureStatus::kFailed);
  ASSERT_TRUE(backend.last_config_error().has_value());
  EXPECT_FALSE(backend.last_config_error()->message.empty());
  EXPECT_FALSE(console.Lines({"[phoenix]", "config failed"}).empty());

  // 2. WPILib HAL constructor throws frc::RuntimeError after Validate passes
  // (e.g. a DIO channel already held by another resource), exercised through
  // Gateway::BeginReconfigure/Tick as on the RoboRIO loop.
  frc::DigitalInput occupied_dio{7};
  hw::Config dio_conflict;
  hw::DigitalInputConfig di;
  di.id = 40;
  di.name = "conflict_di";
  di.dio = 7;
  dio_conflict.digital_inputs.push_back(di);
  hw::Gateway gateway{backend, 1};
  ASSERT_TRUE(gateway.BeginReconfigure(dio_conflict));
  ASSERT_TRUE(WaitConfigureReady(backend));
  ASSERT_NO_THROW(gateway.Tick(5000, false));
  EXPECT_FALSE(gateway.configuring());
  EXPECT_FALSE(gateway.configured());
  EXPECT_TRUE((gateway.snapshot().flags & hw::kHardwareFault) != 0);
  ASSERT_TRUE(gateway.config_error().has_value());
  EXPECT_FALSE(gateway.config_error()->message.empty());

  // 3. PollConfigure() itself catches both std::exception and unknown
  // exceptions stored in configure_.
  {
    std::promise<bool> p;
    p.set_exception(
        std::make_exception_ptr(std::runtime_error("injected future error")));
    SetConfigureFuture(backend, p.get_future());
    ASSERT_NO_THROW(status = backend.PollConfigure());
    EXPECT_EQ(status, hw::Backend::ConfigureStatus::kFailed);
    ASSERT_TRUE(backend.last_config_error().has_value());
    EXPECT_NE(
        backend.last_config_error()->message.find("injected future error"),
        std::string::npos);
  }
  {
    std::promise<bool> p;
    p.set_exception(std::make_exception_ptr(42));
    SetConfigureFuture(backend, p.get_future());
    ASSERT_NO_THROW(status = backend.PollConfigure());
    EXPECT_EQ(status, hw::Backend::ConfigureStatus::kFailed);
    ASSERT_TRUE(backend.last_config_error().has_value());
    EXPECT_FALSE(backend.last_config_error()->message.empty());
  }
}

// Device restoration runs on a worker thread and is started/polled on the
// SCHED_FIFO RoboRIO loop thread. Configure() must preallocate unavailable_
// and restoring_ids_ so BeginRestoreAfterReset never allocates for them on
// the loop thread, and any exception from a restore operation or restore_.get()
// must be caught, logged via RioLog, and reported as RestoreStatus::kFailed
// while keeping the affected IDs unavailable for backed-off retry.
// REQ-SAFE-24
// REQ-RT-07
TEST_F(PhoenixBackendRecovery,
       RestorePreallocatesAndCatchesExceptionsAsFailure) {
  EnsureHal();
  Console console;
  hw::Config config;
  auto master = TestMotor(10, "lift", 1);
  auto follower = TestMotor(20, "follower", 2);
  follower.master_id = 10;
  config.motors = {master, follower};
  PhoenixBackend backend(true, console.log());
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  // 1. Configure() reserves unavailable_ and restoring_ids_ for the full
  // device budget so BeginRestoreAfterReset does not reallocate them.
  constexpr std::size_t kBudget = hw::kMaxMotors + hw::kMaxSensors;
  EXPECT_GE(UnavailableCapacity(backend), kBudget);
  EXPECT_GE(RestoringIdsCapacity(backend), kBudget);
  const uint16_t* unavailable_before = UnavailableData(backend);
  const uint16_t* restoring_before = RestoringIdsData(backend);

  const auto contains = [](const std::vector<uint16_t>& ids, uint16_t id) {
    return std::find(ids.begin(), ids.end(), id) != ids.end();
  };

  // 2. A restore operation throwing std::exception inside the worker must not
  // escape PollRestore(), must return kFailed, log, and keep the chain
  // unavailable without reallocating unavailable_/restoring_ids_.
  PushRestoreOp(backend, 10, []() -> bool {
    throw std::runtime_error("injected restore op error");
  });
  const std::array<uint16_t, 1> ids{10};
  ASSERT_NO_THROW(backend.BeginRestoreAfterReset(ids));
  EXPECT_EQ(UnavailableData(backend), unavailable_before);
  EXPECT_EQ(RestoringIdsData(backend), restoring_before);
  ASSERT_TRUE(WaitRestoreReady(backend));
  hw::Backend::RestoreStatus status = hw::Backend::RestoreStatus::kPending;
  ASSERT_NO_THROW(status = backend.PollRestore());
  EXPECT_EQ(status, hw::Backend::RestoreStatus::kFailed);
  EXPECT_TRUE(contains(backend.UnavailableDevices(), 10));
  EXPECT_TRUE(contains(backend.UnavailableDevices(), 20));
  EXPECT_FALSE(
      console
          .Lines({"[phoenix]", "restore failed", "injected restore op error"})
          .empty());
  PopRestoreOp(backend);

  // 3. Same for a non-std::exception thrown by a restore operation.
  PushRestoreOp(backend, 10, []() -> bool { throw 42; });
  ASSERT_NO_THROW(backend.BeginRestoreAfterReset(ids));
  ASSERT_TRUE(WaitRestoreReady(backend));
  ASSERT_NO_THROW(status = backend.PollRestore());
  EXPECT_EQ(status, hw::Backend::RestoreStatus::kFailed);
  EXPECT_TRUE(contains(backend.UnavailableDevices(), 10));
  PopRestoreOp(backend);

  // 4. PollRestore() itself catches both std::exception and unknown exceptions
  // stored directly in restore_, keeping the devices unavailable and resetting
  // restore_ cleanly.
  {
    SetRestoreException(
        backend,
        std::make_exception_ptr(std::runtime_error("injected future error")));
    ASSERT_NO_THROW(status = backend.PollRestore());
    EXPECT_EQ(status, hw::Backend::RestoreStatus::kFailed);
    EXPECT_TRUE(contains(backend.UnavailableDevices(), 10));
    EXPECT_FALSE(
        console.Lines({"[phoenix]", "restore failed", "injected future error"})
            .empty());
  }
  {
    SetRestoreException(backend, std::make_exception_ptr(42));
    ASSERT_NO_THROW(status = backend.PollRestore());
    EXPECT_EQ(status, hw::Backend::RestoreStatus::kFailed);
    EXPECT_TRUE(contains(backend.UnavailableDevices(), 10));
  }

  // 5. Thread exhaustion (EAGAIN / std::system_error) when launching the
  // async restore worker is caught in BeginRestoreAfterReset, logged, leaves
  // the chain unavailable, and reports RestoreStatus::kFailed on PollRestore.
  SetRestoreLauncher(
      backend,
      [](RestoreTask) -> std::future<std::optional<std::vector<uint16_t>>> {
        throw std::system_error(
            std::make_error_code(std::errc::resource_unavailable_try_again),
            "injected EAGAIN");
      });
  ASSERT_NO_THROW(backend.BeginRestoreAfterReset(ids));
  ASSERT_NO_THROW(status = backend.PollRestore());
  EXPECT_EQ(status, hw::Backend::RestoreStatus::kFailed);
  EXPECT_TRUE(contains(backend.UnavailableDevices(), 10));
  EXPECT_TRUE(contains(backend.UnavailableDevices(), 20));
  EXPECT_FALSE(console.Lines({"[phoenix]", "restore failed", "injected EAGAIN"})
                   .empty());
  SetRestoreLauncher(backend, nullptr);

  // 6. After clean failure handling, a subsequent restore pass succeeds and
  // clears UnavailableDevices().
  ASSERT_TRUE(RestorePass(backend, ids));
  EXPECT_TRUE(backend.UnavailableDevices().empty());
}

// Config validation allows a follower motor to precede its master in
// config.motors. Configure() must check masters before followers so a follower
// of an unlicensed foc master is never read itself, and neither Configure() nor
// PollRestore() may leave a motor in both foc_unlicensed_ and unavailable_
// (which would churn BeginRestoreAfterReset forever).
// REQ-CONTRACT-39
TEST_F(PhoenixBackendFoc,
       FollowerBeforeUnlicensedMasterIsNotReadAndNeverChurnsUnavailable) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  auto follower = TestMotor(20, "follower", 2);
  follower.master_id = 10;
  auto lift = TestMotor(10, "lift", 1);
  lift.foc = true;
  // Follower intentionally precedes its foc master in config.motors.
  config.motors = {follower, lift};
  Console console;
  PhoenixBackend backend(true, console.log());
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  // 1. Configure(): follower would time out (nullopt) if read, while master
  // reads unlicensed (false). Master must be checked first, follower must not
  // be read at all, and UnavailableDevices() must be empty.
  std::vector<std::size_t> checked;
  SetProLicensed(backend, [&checked](std::size_t i) -> std::optional<bool> {
    checked.push_back(i);
    return i == 1 ? std::optional<bool>{false} : std::nullopt;
  });
  ASSERT_TRUE(ConfigureAnswered(backend, config));
  EXPECT_EQ(std::count(checked.begin(), checked.end(), std::size_t{0}), 0)
      << "follower was read despite its master being unlicensed";
  EXPECT_GE(std::count(checked.begin(), checked.end(), std::size_t{1}), 1);
  EXPECT_TRUE(backend.UnavailableDevices().empty())
      << "unlicensed follower/master left in UnavailableDevices()";
  EXPECT_FALSE(
      console.Lines({"[phoenix]", "check_pro_license", "'lift'"}).empty());
  EXPECT_FALSE(
      console.Lines({"[phoenix]", "check_pro_license", "'follower'"}).empty());
  const std::array<hw::MotorRequest, 2> commands{
      Request(20, hw::Mode::kVoltage, 0.0),
      Request(10, hw::Mode::kVoltage, 6.0)};
  hw::State state;
  PumpReads(backend, state, commands, 15);
  EXPECT_FALSE(state.motors[0].valid);
  EXPECT_FALSE(state.motors[1].valid);
  ASSERT_TRUE(ApplySent(backend, commands));
  EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut");
  EXPECT_EQ(AppliedControl(backend, 1), "NeutralOut");

  // 2. PollRestore(): both master and follower initially get no answer at
  // Configure() (both unavailable). During restore, the follower still fails
  // (timeout/disconnected op) while the master reads unlicensed. PollRestore()
  // must refuse both and erase both from UnavailableDevices() so the restore
  // loop never churns on them.
  std::optional<bool> master_licence;
  std::optional<bool> follower_licence;
  checked.clear();
  SetProLicensed(backend, [&](std::size_t i) -> std::optional<bool> {
    checked.push_back(i);
    return i == 1 ? master_licence : follower_licence;
  });
  ASSERT_TRUE(ConfigureAnswered(backend, config));
  ASSERT_FALSE(backend.UnavailableDevices().empty());

  checked.clear();
  master_licence = false;
  follower_licence = std::nullopt;
  // Also simulate a failing device restore op on the follower (e.g. absent
  // follower failing apply_talonfx_config) in the same restore pass.
  PushRestoreOp(backend, 20, []() { return false; });
  ASSERT_TRUE(RestorePass(backend, backend.UnavailableDevices()));
  EXPECT_EQ(std::count(checked.begin(), checked.end(), std::size_t{0}), 0)
      << "follower licence was read during restore after master read "
         "unlicensed";
  EXPECT_TRUE(backend.UnavailableDevices().empty())
      << "unlicensed follower remained in UnavailableDevices() after restore";
  PumpReads(backend, state, commands, 15);
  EXPECT_FALSE(state.motors[0].valid);
  EXPECT_FALSE(state.motors[1].valid);
  ASSERT_TRUE(ApplySent(backend, commands));
  EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut");
  EXPECT_EQ(AppliedControl(backend, 1), "NeutralOut");
}

// Defense-in-depth at the backend boundary: non-finite motor demands or
// feedforwards and out-of-range closed-loop slots (slot > 2) neutralize that
// motor instead of reaching Phoenix SetControl, and non-finite or out-of-range
// PWM pulse outputs disable the PWM channel instead of calling SetSpeed.
// REQ-SAFE-03, REQ-SAFE-04
TEST_F(PhoenixBackendFoc, ApplyNeutralizesNonFiniteInvalidSlotAndBadPwm) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  config.motors.push_back(TestMotor(10, "m1", 1));
  hw::PwmConfig pwm;
  pwm.id = 30;
  pwm.name = "servo";
  pwm.channel = 0;
  config.pwm_outputs.push_back(pwm);

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  // First drive motor and PWM so each bad request must actively neutralize it.
  const std::array<hw::MotorRequest, 1> valid_cmd{
      Request(10, hw::Mode::kVoltage, 4.0)};
  ASSERT_TRUE(ApplySent(backend, valid_cmd));
  ASSERT_EQ(AppliedControl(backend, 0), "VoltageOut");

  const std::array<hw::MotorRequest, 6> bad_motor_requests{
      hw::MotorRequest{10, hw::Mode::kDutyCycle, 0, nan, 0.0},
      hw::MotorRequest{10, hw::Mode::kVoltage, 0, inf, 0.0},
      hw::MotorRequest{10, hw::Mode::kVelocity, 0, 5.0, nan},
      hw::MotorRequest{10, hw::Mode::kPosition, /*slot=*/3, 1.0, 0.0},
      hw::MotorRequest{10, hw::Mode::kMotionMagic, 0, -inf, 0.0},
      hw::MotorRequest{10, hw::Mode::kMotionMagic, /*slot=*/4, 1.0, 0.0},
  };
  for (std::size_t k = 0; k < bad_motor_requests.size(); ++k) {
    ASSERT_TRUE(ApplySent(backend, valid_cmd)) << "setup " << k;
    ASSERT_EQ(AppliedControl(backend, 0), "VoltageOut") << "setup " << k;
    const std::array<hw::MotorRequest, 1> bad{bad_motor_requests[k]};
    ASSERT_TRUE(ApplySent(backend, bad)) << "case " << k;
    EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut") << "case " << k;
  }

  // Bad PWM pulses must disable the PWM channel (0 us pulse) rather than
  // calling SetSpeed with NaN/Inf/out-of-range.
  const frc::sim::PWMSim pwm_sim{0};
  for (const double bad_output : {nan, inf, -inf, 1.5, -1.5}) {
    ASSERT_TRUE(backend.ApplyOutputs(
        {}, std::array{hw::PwmRequest{30, hw::PwmMode::kPulse, 0.25}}));
    ASSERT_GT(pwm_sim.GetPulseMicrosecond(), 0);
    ASSERT_TRUE(backend.ApplyOutputs(
        {}, std::array{hw::PwmRequest{30, hw::PwmMode::kPulse, bad_output}}));
    EXPECT_EQ(pwm_sim.GetPulseMicrosecond(), 0)
        << "bad PWM pulse " << bad_output << " was not disabled";
  }
}

// Even when a follower precedes its master in config.motors, Apply() must
// neutralize the follower whenever the master's request is disallowed (e.g.
// non-finite demand or disallowed TorqueCurrent) so the follower never runs
// Follower{master} while the master is neutralized.
// REQ-SAFE-03, REQ-SAFE-04
TEST_F(PhoenixBackendFoc,
       FollowerNeutralizesWhenMasterRequestIsDisallowedAtBackend) {
  EnsureHal();
  EnabledDs ds;
  hw::Config config;
  auto follower = TestMotor(20, "follower", 2);
  follower.master_id = 10;
  auto master = TestMotor(10, "master", 1);
  // Follower intentionally precedes master in manifest order.
  config.motors = {follower, master};

  PhoenixBackend backend(true);
  ASSERT_TRUE(backend.Configure(config));
  ASSERT_TRUE(AwaitReady(backend, config));

  // Baseline: valid master demand drives both master and follower.
  const std::array<hw::MotorRequest, 2> good_cmds{
      Request(20, hw::Mode::kVoltage, 0.0),
      Request(10, hw::Mode::kVoltage, 4.0)};
  hw::State state;
  PumpReads(backend, state, good_cmds, 15);
  ASSERT_TRUE(ApplySent(backend, good_cmds));
  ASSERT_EQ(AppliedControl(backend, 0), "Follower");
  ASSERT_EQ(AppliedControl(backend, 1), "VoltageOut");

  // Disallowed master demand (NaN voltage): BOTH master and follower must go
  // NeutralOut.
  const std::array<hw::MotorRequest, 2> nan_master{
      Request(20, hw::Mode::kVoltage, 0.0),
      Request(10, hw::Mode::kVoltage,
              std::numeric_limits<double>::quiet_NaN())};
  ASSERT_TRUE(ApplySent(backend, nan_master));
  EXPECT_EQ(AppliedControl(backend, 1), "NeutralOut");
  EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut")
      << "follower stayed active when master demand was non-finite";

  // Disallowed TorqueCurrent on non-FOC master: BOTH master and follower must
  // go NeutralOut.
  ASSERT_TRUE(ApplySent(backend, good_cmds));
  ASSERT_EQ(AppliedControl(backend, 0), "Follower");
  const std::array<hw::MotorRequest, 2> bad_torque_master{
      Request(20, hw::Mode::kTorqueCurrent, 0.0),
      Request(10, hw::Mode::kTorqueCurrent, 10.0)};
  ASSERT_TRUE(ApplySent(backend, bad_torque_master));
  EXPECT_EQ(AppliedControl(backend, 1), "NeutralOut");
  EXPECT_EQ(AppliedControl(backend, 0), "NeutralOut")
      << "follower stayed active when master TorqueCurrent was disallowed";
}
