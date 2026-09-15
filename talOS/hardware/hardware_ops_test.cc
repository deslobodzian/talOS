// Items 33-40: hardware operations. One behavior per section, no loop, no
// transport, no live sim: Gateway ticks use explicit timestamps and the sim
// backends step on the explicit sample clock.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "characterization.h"
#include "commissioning.h"
#include "config_error.h"
#include "fault_scope.h"
#include "gateway.h"
#include "homing.h"
#include "sim_backend.h"
#include "sim_physics.h"
#include "support_matrix.h"

namespace talos::hardware {
namespace {

using sim::PhysicsSimBackend;

Config OneMotor(bool commissioned = true) {
  Config c;
  c.commissioned = commissioned;
  MotorConfig m;
  m.id = 1;
  m.name = "roller";
  m.can_id = 1;
  m.bus = "rio";
  c.motors.push_back(m);
  return c;
}

Command IdleCommand(const Gateway& g) {
  const State s = g.snapshot();
  Command c{};
  c.config_id = s.config_id;
  c.boot_id = s.boot_id;
  c.epoch = s.epoch;
  c.observed_time_us = s.sample_time_us;
  c.count = s.motor_count;
  for (std::size_t i = 0; i < c.count; ++i)
    c.motors[i] = {s.motors[i].id, Mode::kNeutral, 0, 0, 0};
  return c;
}

// (33) The matrix states parsing vs real access vs sim vs tests honestly.
TEST(HardwareOps, SupportMatrixIsEndToEndAndNamesEveryKind) {
  const auto matrix = SupportMatrix();
  EXPECT_EQ(matrix.size(), kDeviceKindCount);
  for (const auto& row : matrix) {
    EXPECT_TRUE(row.parsed_by_config) << row.name;
    EXPECT_TRUE(row.real_access_phoenix) << row.name;
    EXPECT_TRUE(row.sim_ideal) << row.name;
    EXPECT_TRUE(row.sim_physics) << row.name;
    EXPECT_TRUE(row.covered_by_tests) << row.name;
    EXPECT_TRUE(IsSupportedEndToEnd(row)) << row.name;
    EXPECT_EQ(DeviceKindName(row.kind), std::string_view(row.name));
    EXPECT_GT(std::string_view(row.notes).size(), 0u) << row.name;
  }
  // The documented control-mode limit lives with the motor row.
  EXPECT_NE(std::string_view(matrix[0].notes).find("TorqueCurrentFOC"),
            std::string_view::npos);
}

// (34) Commissioning cannot complete until every step is signed off.
TEST(HardwareOps, CommissioningBlocksUntilEveryStepSignedOff) {
  Config c = OneMotor(false);
  c.sensors.push_back({2, "encoder", SensorKind::kCANcoder, 20, "rio"});
  c.motors[0].feedback = Feedback::kRemoteCANcoder;
  c.motors[0].feedback_sensor_id = 2;

  CommissioningRecord record;
  auto report = record.ApplyTo(c);
  EXPECT_FALSE(report.ready);
  EXPECT_FALSE(c.commissioned);
  EXPECT_FALSE(report.issues.empty());

  record.motors[1] = {true, true, true, true, true, "spun +duty, moved +"};
  record.sensors[2] = {true, true, "magnet offset 0.0 measured"};
  report = record.ApplyTo(c);
  EXPECT_TRUE(report.ready) << report.issues.front().detail;
  EXPECT_TRUE(c.commissioned);

  // One missing step blocks again, naming the check.
  record.motors[1].direction_ok = false;
  c.commissioned = false;
  report = record.ApplyTo(c);
  EXPECT_FALSE(report.ready);
  EXPECT_FALSE(c.commissioned);
  bool names_direction = false;
  for (const auto& i : report.issues) names_direction |= i.check == "direction";
  EXPECT_TRUE(names_direction);
}

namespace {
class OpsFake : public Backend {
 public:
  bool Configure(const Config&) override {
    ++configures;
    if (!configure_ok) {
      error = ConfigError{};
      error->device_name = "front_left_drive";
      error->bus = "rio";
      error->can_id = 3;
      error->operation = "apply_talonfx_config";
      error->vendor_status = -1;
      error->message = "timeout";
      return false;
    }
    error.reset();
    return true;
  }
  bool Read(State&) override { return read_ok; }
  bool Apply(std::span<const MotorRequest>) override { return apply_ok; }
  void Neutral() override { ++neutrals; }
  std::optional<ConfigError> last_config_error() const override { return error; }
  bool PollDeviceResets(std::vector<uint16_t>& ids) override {
    ids = resets;
    resets.clear();
    return !ids.empty();
  }
  bool RestoreAfterReset() override { return restore_ok; }
  bool ZeroPosition(uint16_t id, double home) override {
    zeroed_id = id;
    zeroed_home = home;
    return zero_ok;
  }
  HealthSnapshot GetHealth() override {
    HealthSnapshot s;
    s.supply_voltage_v = 12.5;
    s.supply_valid = true;
    return s;
  }
  bool configure_ok{true}, read_ok{true}, apply_ok{true}, restore_ok{true},
      zero_ok{true};
  int configures{0}, neutrals{0};
  uint16_t zeroed_id{0};
  double zeroed_home{0.0};
  std::vector<uint16_t> resets;
  std::optional<ConfigError> error;
};
}  // namespace

// (35) A failed Configure names the device, bus, CAN ID, operation, and
// vendor status; success clears it.
TEST(HardwareOps, ConfigFailureNamesDeviceBusAddressAndVendorStatus) {
  OpsFake backend;
  backend.configure_ok = false;
  Gateway g{OneMotor(), backend, 7};
  EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
  ASSERT_TRUE(g.config_error().has_value());
  const ConfigError err = *g.config_error();
  EXPECT_EQ(err.device_name, "front_left_drive");
  EXPECT_EQ(err.bus, "rio");
  EXPECT_EQ(err.can_id, 3);
  EXPECT_EQ(err.operation, "apply_talonfx_config");
  EXPECT_EQ(err.vendor_status, -1);
  const std::string text = FormatConfigError(err);
  EXPECT_NE(text.find("front_left_drive"), std::string::npos);
  EXPECT_NE(text.find("rio"), std::string::npos);
  EXPECT_NE(text.find("apply_talonfx_config"), std::string::npos);

  backend.configure_ok = true;
  g.Tick(1000, false);
  ASSERT_TRUE(g.Reconfigure(OneMotor()));
  EXPECT_FALSE(g.config_error().has_value());
}

// (36) A device reset restores configuration without resuming obsolete
// demand; a failed restore latches a fault.
TEST(HardwareOps, DeviceResetRestoresWithoutResumingObsoleteDemand) {
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(OneMotor()));
  Gateway g{OneMotor(), backend, 11};
  g.Tick(1000, true);
  Command c = IdleCommand(g);
  c.motors[0] = {1, Mode::kDutyCycle, 0, 0.5, 0};
  ASSERT_TRUE(g.Accept(c, 1, 1000));
  const auto epoch = g.snapshot().epoch;

  backend.SimulateDeviceReset(1);
  g.Tick(6000, true);
  // Epoch advanced: outstanding commands are dead, nothing resumes.
  EXPECT_NE(g.snapshot().epoch, epoch);
  EXPECT_FALSE(g.snapshot().flags & kCommandActive);
  EXPECT_FALSE(g.Accept(c, 2, 6000));
  EXPECT_EQ(g.health().reset_count, 1u);
  // The restored device is stopped, not re-driven at the old demand.
  g.Tick(11000, true);
  EXPECT_DOUBLE_EQ(g.snapshot().motors[0].velocity_rps, 0.0);
  // Fresh commands drive again.
  EXPECT_TRUE(g.Accept(IdleCommand(g), 0, 11000));
}

TEST(HardwareOps, FailedRestoreLatchesFault) {
  OpsFake backend;
  Gateway g{OneMotor(), backend, 13};
  g.Tick(1000, true);
  backend.restore_ok = false;
  backend.resets.push_back(1);
  g.Tick(2000, true);
  EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
  EXPECT_EQ(g.health().reset_count, 1u);
}

// (37) Fault scope is deliberate: effort staleness degrades telemetry only,
// encoder loss stops one mechanism, motion/sensor/apply loss stops the robot.
TEST(HardwareOps, FaultScopeClassification) {
  EXPECT_EQ(ClassifyFailure(FailureKind::kStaleMotorCurrent),
            FaultScope::kTelemetryOnly);
  EXPECT_EQ(ClassifyFailure(FailureKind::kStaleMotorVoltage),
            FaultScope::kTelemetryOnly);
  EXPECT_EQ(ClassifyFailure(FailureKind::kEncoderInvalid),
            FaultScope::kMechanism);
  EXPECT_EQ(ClassifyFailure(FailureKind::kStaleMotorPosition),
            FaultScope::kRobot);
  EXPECT_EQ(ClassifyFailure(FailureKind::kStaleMotorVelocity),
            FaultScope::kRobot);
  EXPECT_EQ(ClassifyFailure(FailureKind::kMotorDisconnect), FaultScope::kRobot);
  EXPECT_EQ(ClassifyFailure(FailureKind::kStaleSensor), FaultScope::kRobot);
  EXPECT_EQ(ClassifyFailure(FailureKind::kSensorDisconnect), FaultScope::kRobot);
  EXPECT_EQ(ClassifyFailure(FailureKind::kApplyFailed), FaultScope::kRobot);
  EXPECT_EQ(ClassifyFailure(FailureKind::kConfigFailed), FaultScope::kRobot);
  EXPECT_STREQ(FaultScopeName(FaultScope::kRobot), "robot");
}

// (38) Zeroing completes with an epoch bump, and rejects unknown motors,
// out-of-limit homes, active commands, stale seeks, and backend failures.
TEST(HardwareOps, ZeroCompletesAndRejectsEveryBadCapture) {
  Config c = OneMotor();
  c.motors[0].soft_limits = true;
  c.motors[0].reverse_limit_rot = -1.0;
  c.motors[0].forward_limit_rot = 1.0;
  PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(c));
  Gateway g{c, backend, 17};
  g.Tick(1000, true);

  std::string reason;
  const auto epoch = g.snapshot().epoch;
  EXPECT_TRUE(g.Zero(1, 0.25, 1000, 1000, &reason));
  EXPECT_EQ(reason, "ok");
  EXPECT_NE(g.snapshot().epoch, epoch);
  g.Tick(6000, true);
  EXPECT_NEAR(g.snapshot().motors[0].position_rot, 0.25, 1e-9);

  EXPECT_FALSE(g.Zero(99, 0.0, 6000, 6000, &reason));  // Unknown motor.
  EXPECT_FALSE(g.Zero(1, 5.0, 6000, 6000, &reason));   // Outside soft limits.
  // Stale seek: the observation aged past the command timeout.
  EXPECT_FALSE(g.Zero(1, 0.0, 0, 6000 + 100000, &reason));

  // Interruption by driving: zeroing under load is refused.
  Command drive = IdleCommand(g);
  drive.motors[0] = {1, Mode::kDutyCycle, 0, 0.2, 0};
  ASSERT_TRUE(g.Accept(drive, 3, 6000));
  EXPECT_FALSE(g.Zero(1, 0.0, 6000, 6000, &reason));
}

TEST(HardwareOps, ZeroBackendFailureLatchesFault) {
  OpsFake backend;
  backend.zero_ok = false;
  Gateway g{OneMotor(), backend, 19};
  g.Tick(1000, false);
  std::string reason;
  EXPECT_FALSE(g.Zero(1, 0.0, 1000, 1000, &reason));
  EXPECT_TRUE(g.snapshot().flags & kHardwareFault);
}

// (39) Health reports the rail, the reset count, and the fault latch.
TEST(HardwareOps, HealthReportsSupplyResetsAndFaults) {
  SimBackend backend;
  backend.SetSupplyVoltage(11.2);
  Gateway g{OneMotor(), backend, 23};
  g.Tick(1000, true);
  HealthSnapshot h = g.health();
  EXPECT_TRUE(h.supply_valid);
  EXPECT_DOUBLE_EQ(h.supply_voltage_v, 11.2);
  EXPECT_EQ(h.reset_count, 0u);
  EXPECT_FALSE(h.fault_latched);
  EXPECT_TRUE(h.configured);
  EXPECT_TRUE(h.enabled);

  backend.SimulateDeviceReset(1);
  g.Tick(2000, true);
  EXPECT_EQ(g.health().reset_count, 1u);

  OpsFake failing;
  failing.apply_ok = false;
  Gateway f{OneMotor(), failing, 29};
  f.Tick(1000, true);
  EXPECT_FALSE(f.Accept(IdleCommand(f), 1, 1000));
  EXPECT_TRUE(f.health().fault_latched);
  EXPECT_DOUBLE_EQ(f.health().supply_voltage_v, 12.5);
}

// (40) Characterization records synchronized samples, fits kS/kV, exports
// CSV, and stamps provenance; bad datasets say so.
TEST(HardwareOps, CharacterizationFitsGainsWithProvenance) {
  CharRecorder recorder;
  EXPECT_TRUE(recorder.Start(1, 42, "roller ramp"));
  EXPECT_FALSE(recorder.Start(1, 42));  // Already recording.
  // Synthetic steady-state data: V = 1.0 * sgn(vel) + 0.5 * vel, as a
  // bidirectional quasistatic ramp produces (voltage changes sign with
  // direction; a always-positive V would be a one-sided dataset).
  int t = 0;
  for (const double vel : {1.0, 2.0, 4.0, -1.0, -2.0, -4.0, 3.0, -3.0}) {
    CharSample s{};
    s.t_s = ++t * 0.01;
    s.velocity_rps = vel;
    s.voltage_v = (vel >= 0.0 ? 1.0 : -1.0) + 0.5 * vel;
    s.position_rot = vel * s.t_s;
    EXPECT_TRUE(recorder.Add(1, 42, s));
  }
  EXPECT_FALSE(recorder.Add(2, 42, CharSample{}));  // Wrong motor.
  CharSample back{};
  back.t_s = 0.005;  // Non-monotonic time.
  EXPECT_FALSE(recorder.Add(1, 42, back));
  CharDataset dataset;
  ASSERT_TRUE(recorder.Finish(dataset));
  EXPECT_EQ(dataset.samples.size(), 8u);

  const CharFit fit = FitStaticGains(dataset);
  EXPECT_TRUE(fit.ok) << fit.detail;
  EXPECT_NEAR(fit.ks_v, 1.0, 1e-9);
  EXPECT_NEAR(fit.kv_vs_per_rps, 0.5, 1e-9);
  EXPECT_GT(fit.r_squared, 0.99);

  const std::string csv = ExportCharCsv(dataset);
  EXPECT_EQ(csv.find("t_s,voltage_v,position_rot,velocity_rps"), 0u);
  const std::string prov =
      FormatCharProvenance(dataset, fit, "op: des, 2026-09-14");
  EXPECT_NE(prov.find("motor_id=1"), std::string::npos);
  EXPECT_NE(prov.find("config_id=42"), std::string::npos);
  EXPECT_NE(prov.find("op: des"), std::string::npos);
}

TEST(HardwareOps, CharacterizationRejectsDegenerateDatasets) {
  CharDataset flat;
  flat.motor_id = 1;
  for (int i = 0; i < 5; ++i) {
    CharSample s{};
    s.t_s = (i + 1) * 0.01;
    s.velocity_rps = 2.0;  // No velocity spread: nothing to fit.
    s.voltage_v = 2.0;
    flat.samples.push_back(s);
  }
  EXPECT_FALSE(FitStaticGains(flat).ok);

  CharDataset crawl;
  crawl.motor_id = 1;
  for (int i = 0; i < 5; ++i) {
    CharSample s{};
    s.t_s = (i + 1) * 0.01;
    s.velocity_rps = 0.01;  // Below the crawl threshold: no usable samples.
    s.voltage_v = 0.1;
    crawl.samples.push_back(s);
  }
  EXPECT_FALSE(FitStaticGains(crawl).ok);

  CharRecorder recorder;
  EXPECT_TRUE(recorder.Start(1, 7));
  CharDataset empty;
  EXPECT_FALSE(recorder.Finish(empty));  // Nothing recorded.
}

}  // namespace
}  // namespace talos::hardware
