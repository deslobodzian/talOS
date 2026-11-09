// End-to-end scenarios over the real Gateway + SimBackend: failure
// stop-conditions and fresh-info resume rules, checked on what the simulated
// wheel actually does rather than on backend call logs.
//
// No processes, no ports, no shared memory: the gateway is driven directly
// through Tick/Accept/snapshot, exactly as the bridge drives it, with a
// deterministic ideal-actuator backend. Each scenario names the stop
// condition (what motion is forbidden) and the fresh information resume
// requires (epoch/boot/sequence/observation), so a resume on stale data is
// a test failure, not a silent replay.

#include <gtest/gtest.h>

#include <cstdint>
#include <span>
#include <vector>

#include "talOS/hardware/config_wire.h"
#include "talOS/hardware/gateway.h"
#include "talOS/hardware/sim_backend.h"

namespace talos::hardware {
namespace {

constexpr uint16_t kMotorId = 7;
constexpr uint64_t kBootId = 0xB0071DULL;
constexpr uint64_t kTimeoutUs = 100000;

Config RollerConfig() {
  Config c;
  c.commissioned = true;
  MotorConfig m;
  m.id = kMotorId;
  m.name = "roller";
  m.can_id = 5;
  m.bus = "rio";
  c.motors.push_back(m);
  return c;
}

Command FreshCommand(const Gateway& gw, uint64_t boot, Mode mode, double demand,
                     uint64_t now, uint64_t sequence) {
  Command c;
  c.config_id = gw.snapshot().config_id;
  c.boot_id = boot;
  c.epoch = gw.snapshot().epoch;
  c.observed_time_us = now;
  c.count = 1;
  c.motors[0] = MotorRequest{kMotorId, mode, 0, demand, 0.0};
  (void)sequence;
  return c;
}

bool HasFlag(const State& s, uint32_t flag) { return (s.flags & flag) != 0u; }

// Shorthand: accept a velocity demand shaped fresh off this gateway.
bool AcceptVel(Gateway& gw, uint64_t boot, double demand, uint64_t now,
               uint64_t sequence) {
  return gw.Accept(
      FreshCommand(gw, boot, Mode::kVelocity, demand, now, sequence), sequence,
      now);
}

// A Backend over the deterministic SimBackend that can fail each
// device-facing call independently, so the matrix can observe the
// gateway's stop condition without real hardware. (SimBackend is final,
// so this wraps it instead of deriving from it.)
class FaultyBackend : public Backend {
 public:
  bool fail_read = false;
  bool fail_apply = false;
  bool fail_restore = false;
  int restore_failures_remaining{};
  uint16_t disconnected_motor{};
  int restore_attempts{};
  bool Configure(const Config& config) override {
    return sim_.Configure(config);
  }
  bool Read(State& state) override {
    const bool read_ok = sim_.Read(state);
    bool any_valid = state.motor_count == 0 && state.sensor_count == 0;
    for (std::size_t i = 0; i < state.motor_count; ++i) {
      auto& motor = state.motors[i];
      motor.valid &= !fail_read && motor.id != disconnected_motor;
      any_valid |= motor.valid;
    }
    for (std::size_t i = 0; i < state.sensor_count; ++i) {
      state.sensors[i].valid &= !fail_read;
      any_valid |= state.sensors[i].valid;
    }
    return read_ok && any_valid;
  }
  bool Apply(std::span<const MotorRequest> commands) override {
    if (fail_apply) return false;
    return sim_.Apply(commands);
  }
  std::span<const uint16_t> ApplyFailedDevices() const override {
    return sim_.ApplyFailedDevices();
  }
  bool ApplyOutputs(std::span<const DigitalOutputRequest> digital_outputs,
                    std::span<const PwmRequest> pwm_outputs) override {
    return sim_.ApplyOutputs(digital_outputs, pwm_outputs);
  }
  void Neutral() override { sim_.Neutral(); }
  bool PollDeviceResets(std::vector<uint16_t>& reset_ids) override {
    return sim_.PollDeviceResets(reset_ids);
  }
  void BeginRestoreAfterReset(std::span<const uint16_t> ids) override {
    ++restore_attempts;
    sim_.BeginRestoreAfterReset(ids);
  }
  RestoreStatus PollRestore() override {
    if (restore_failures_remaining > 0) {
      --restore_failures_remaining;
      return RestoreStatus::kFailed;
    }
    return fail_restore ? RestoreStatus::kFailed : sim_.PollRestore();
  }
  void SimulateDeviceReset(uint16_t id) { sim_.SimulateDeviceReset(id); }

 private:
  SimBackend sim_;
};

TEST(GatewayScenario, StartupDisabledAcceptsNothing) {
  SimBackend backend;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(1000, /*driver_station_enabled=*/false);
  const State s = gw.snapshot();
  EXPECT_FALSE(HasFlag(s, kEnabled));
  EXPECT_FALSE(HasFlag(s, kCommandActive));
  EXPECT_FALSE(AcceptVel(gw, kBootId, 10.0, 1000, 1));
}

TEST(GatewayScenario, EnableThenNormalOpMirrorsDemand) {
  SimBackend backend;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, true);
  EXPECT_TRUE(HasFlag(gw.snapshot(), kEnabled));
  EXPECT_TRUE(AcceptVel(gw, kBootId, 10.0, 2000, 1));
  EXPECT_TRUE(HasFlag(gw.snapshot(), kCommandActive));
  // Production re-sends every tick; the test pumps the same way.
  for (uint64_t t = 7000, seq = 2; t <= 22000; t += 5000, ++seq) {
    gw.Tick(t, true);
    EXPECT_TRUE(AcceptVel(gw, kBootId, 10.0, t, seq));
  }
  gw.Tick(27000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 10.0);
  EXPECT_TRUE(gw.snapshot().motors[0].valid);
}

TEST(GatewayScenario, DisableWhileMovingStopsAndNeutralizes) {
  SimBackend backend;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, true);
  ASSERT_TRUE(AcceptVel(gw, kBootId, 10.0, 2000, 1));
  gw.Tick(7000, true);
  ASSERT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 10.0);
  gw.Tick(12000, false);
  const State s = gw.snapshot();
  EXPECT_FALSE(HasFlag(s, kEnabled));
  EXPECT_FALSE(HasFlag(s, kCommandActive));
  // Stop condition: the disable neutralized the backend, so the wheel reads
  // zero even though no new command arrived.
  gw.Tick(17000, false);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
  // A command shaped before the re-enable does not resume on it: the
  // enable edge bumped the epoch, so resume requires the fresh epoch.
  Command pre_enable =
      FreshCommand(gw, kBootId, Mode::kVelocity, 10.0, 17000, 1);
  gw.Tick(22000, true);
  EXPECT_FALSE(gw.Accept(pre_enable, 1, 22000));
  EXPECT_TRUE(AcceptVel(gw, kBootId, 10.0, 22000, 1));
}

TEST(GatewayScenario, TargetExpiryInvalidatesWithoutResend) {
  SimBackend backend;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, true);
  ASSERT_TRUE(AcceptVel(gw, kBootId, 10.0, 2000, 1));
  ASSERT_TRUE(HasFlag(gw.snapshot(), kCommandActive));
  gw.Tick(2000 + kTimeoutUs, true);  // no re-send: the target ages out
  EXPECT_FALSE(HasFlag(gw.snapshot(), kCommandActive));
  gw.Tick(2000 + kTimeoutUs + 5000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
}

TEST(GatewayScenario, MissingFeedbackFaultsAndBlocksCommands) {
  FaultyBackend backend;
  backend.fail_read = true;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, true);
  // Stop condition: unhealthy reads latch the fault flag and refuse demand.
  EXPECT_TRUE(HasFlag(gw.snapshot(), kHardwareFault));
  EXPECT_FALSE(AcceptVel(gw, kBootId, 10.0, 2000, 1));
  // Recovery still needs fresh information: an observation older than the
  // command timeout is rejected even after reads recover.
  backend.fail_read = false;
  gw.Tick(2000 + kTimeoutUs + 5000, true);
  EXPECT_FALSE(HasFlag(gw.snapshot(), kHardwareFault));
  const uint64_t recovered_now = 2000 + kTimeoutUs + 5000;
  Command stale_obs = FreshCommand(gw, kBootId, Mode::kVelocity, 10.0, 2000, 2);
  EXPECT_FALSE(gw.Accept(stale_obs, 2, recovered_now));
  EXPECT_TRUE(gw.Accept(
      FreshCommand(gw, kBootId, Mode::kVelocity, 10.0, recovered_now, 2), 2,
      recovered_now));
}
TEST(GatewayScenario, DisconnectedMotorLeavesHealthyMotorDriving) {
  FaultyBackend backend;
  auto config = RollerConfig();
  auto second = config.motors[0];
  second.id = 8;
  second.name = "second";
  second.can_id = 6;
  config.motors.push_back(second);
  Gateway gw(config, backend, kBootId);
  gw.Tick(2000, true);
  auto command = FreshCommand(gw, kBootId, Mode::kVelocity, 10.0, 2000, 1);
  command.count = 2;
  command.motors[1] = {second.id, Mode::kVelocity, 0, 12.0, 0.0};
  ASSERT_TRUE(gw.Accept(command, 1, 2000));
  const auto epoch = gw.snapshot().epoch;
  backend.disconnected_motor = kMotorId;
  gw.Tick(7000, true);
  EXPECT_EQ(gw.snapshot().epoch, epoch);
  command.observed_time_us = 7000;
  ASSERT_TRUE(gw.Accept(command, 2, 7000));
  gw.Tick(12000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[1].velocity_rps, 12.0);
  EXPECT_EQ(gw.snapshot().counters.neutralized_motor_count, 1u);
  EXPECT_FALSE(HasFlag(gw.snapshot(), kHardwareFault));

  backend.fail_read = true;
  gw.Tick(17000, true);
  EXPECT_TRUE(HasFlag(gw.snapshot(), kHardwareFault));
  EXPECT_FALSE(HasFlag(gw.snapshot(), kCommandActive));
  EXPECT_GT(gw.snapshot().epoch, epoch);
  EXPECT_FALSE(gw.Accept(command, 3, 17000));
  backend.fail_read = false;
  gw.Tick(22000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[1].velocity_rps, 0.0);
}

TEST(GatewayScenario, ControllerRestartRequiresFreshEpoch) {
  SimBackend backend;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, true);
  ASSERT_TRUE(AcceptVel(gw, kBootId, 10.0, 2000, 1));
  const uint64_t epoch_before = gw.snapshot().epoch;
  backend.SimulateDeviceReset(kMotorId);
  gw.Tick(7000, true);
  // A reset invalidates only this motor's observation, not the global epoch.
  EXPECT_EQ(gw.snapshot().epoch, epoch_before);
  EXPECT_TRUE(HasFlag(gw.snapshot(), kCommandActive));
  gw.Tick(12000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
  Command stale = FreshCommand(gw, kBootId, Mode::kVelocity, 10.0, 2000, 2);
  EXPECT_TRUE(gw.Accept(stale, 2, 12000));
  gw.Tick(13000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
  EXPECT_TRUE(AcceptVel(gw, kBootId, 10.0, 13000, 3));
}

TEST(GatewayScenario, TransientApplyFailureRecoversOnFreshCommand) {
  FaultyBackend backend;
  backend.fail_apply = true;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(3000, true);
  EXPECT_FALSE(AcceptVel(gw, kBootId, 10.0, 3000, 1));
  EXPECT_FALSE(HasFlag(gw.snapshot(), kHardwareFault));
  backend.fail_apply = false;
  gw.Tick(8000, true);
  ASSERT_TRUE(AcceptVel(gw, kBootId, 10.0, 8000, 2));
  gw.Tick(9000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 10);
}

TEST(GatewayScenario, PersistedRestartAcceptsIdenticalPushWhileEnabled) {
  const auto bytes = PersistedConfig(RollerConfig());
  Config restored;
  ASSERT_TRUE(LoadPersistedConfig(bytes, restored));
  SimBackend backend;
  Gateway gw(backend, kBootId);
  ASSERT_TRUE(gw.Reconfigure(restored));
  gw.Tick(3000, true);
  const auto epoch = gw.snapshot().epoch;
  ASSERT_TRUE(gw.Reconfigure(RollerConfig()));
  EXPECT_EQ(gw.snapshot().epoch, epoch);
  EXPECT_TRUE(AcceptVel(gw, kBootId, 10, 3000, 1));
}

TEST(GatewayScenario, FailedRestoreStaysNeutralAndDropsDemand) {
  FaultyBackend backend;
  backend.fail_restore = true;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, true);
  ASSERT_TRUE(AcceptVel(gw, kBootId, 10.0, 2000, 1));
  backend.SimulateDeviceReset(kMotorId);
  gw.Tick(7000, true);
  EXPECT_FALSE(HasFlag(gw.snapshot(), kHardwareFault));
  EXPECT_FALSE(gw.snapshot().motors[0].valid);
  // The reset tick read before the restore ran; the next read off the
  // restored device is zero, never the pre-reset demand.
  gw.Tick(12000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
}

TEST(GatewayScenario, RestoreRetriesTwiceThenResumesWhileEnabled) {
  FaultyBackend backend;
  backend.restore_failures_remaining = 2;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(1000, true);
  ASSERT_TRUE(AcceptVel(gw, kBootId, 10, 1000, 1));
  const auto stale = FreshCommand(gw, kBootId, Mode::kVelocity, 10, 1000, 2);
  backend.SimulateDeviceReset(kMotorId);
  gw.Tick(2000, true);
  gw.Tick(3000, true);
  EXPECT_TRUE(gw.configured());
  EXPECT_EQ(backend.restore_attempts, 1);
  gw.Tick(252999, true);
  EXPECT_EQ(backend.restore_attempts, 1);
  EXPECT_EQ(gw.snapshot().sample_time_us, 252999u);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0);
  EXPECT_TRUE(AcceptVel(gw, kBootId, 10, 252999, 3));
  gw.Tick(253000, true);
  EXPECT_EQ(backend.restore_attempts, 2);
  gw.Tick(254000, true);
  gw.Tick(753999, true);
  EXPECT_EQ(backend.restore_attempts, 2);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0);
  gw.Tick(754000, true);
  EXPECT_EQ(backend.restore_attempts, 3);
  gw.Tick(755000, true);
  EXPECT_TRUE(gw.configured());
  EXPECT_FALSE(HasFlag(gw.snapshot(), kHardwareFault));
  EXPECT_EQ(gw.restore_attempt_count(), 3u);
  EXPECT_EQ(gw.snapshot().counters.device_reset_count, 1u);
  EXPECT_TRUE(gw.snapshot().motors[0].valid);
  EXPECT_FALSE(gw.Accept(stale, 4, 755000));
  ASSERT_TRUE(AcceptVel(gw, kBootId, 10, 755000, 5));
  gw.Tick(756000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 10);
}

TEST(GatewayScenario, RestoreBackoffCapsAtTwoSeconds) {
  FaultyBackend backend;
  backend.fail_restore = true;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(1000, true);
  backend.SimulateDeviceReset(kMotorId);
  gw.Tick(2000, true);
  uint64_t now = 3000;
  int attempts = 1;
  for (uint64_t delay : {250000u, 500000u, 1000000u, 2000000u, 2000000u}) {
    gw.Tick(now, true);
    gw.Tick(now + delay - 1, true);
    EXPECT_EQ(backend.restore_attempts, attempts);
    EXPECT_EQ(gw.snapshot().sample_time_us, now + delay - 1);
    EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0);
    gw.Tick(now + delay, true);
    EXPECT_EQ(backend.restore_attempts, ++attempts);
    EXPECT_TRUE(AcceptVel(gw, kBootId, 10, now + delay, attempts));
    now += delay + 1;
  }
  EXPECT_EQ(gw.restore_attempt_count(), 6u);
}

}  // namespace
}  // namespace talos::hardware
