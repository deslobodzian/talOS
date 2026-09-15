// End-to-end scenario matrix over the real Gateway + SimBackend (item 52)
// with failure stop-conditions and fresh-info resume rules (item 53).
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

Command FreshCommand(const Gateway& gw, uint64_t boot, Mode mode,
                     double demand, uint64_t now, uint64_t sequence) {
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
      FreshCommand(gw, boot, Mode::kVelocity, demand, now, sequence),
      sequence, now);
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
  bool Configure(const Config& config) override {
    return sim_.Configure(config);
  }
  bool Read(State& state) override {
    if (fail_read) return false;
    return sim_.Read(state);
  }
  bool Apply(std::span<const MotorRequest> commands) override {
    if (fail_apply) return false;
    return sim_.Apply(commands);
  }
  void Neutral() override { sim_.Neutral(); }
  bool PollDeviceResets(std::vector<uint16_t>& reset_ids) override {
    return sim_.PollDeviceResets(reset_ids);
  }
  bool RestoreAfterReset() override {
    if (fail_restore) return false;
    return sim_.RestoreAfterReset();
  }
  bool ZeroPosition(uint16_t motor_id, double home_position_rot) override {
    return sim_.ZeroPosition(motor_id, home_position_rot);
  }
  HealthSnapshot GetHealth() override { return sim_.GetHealth(); }
  void SimulateDeviceReset(uint16_t id) { sim_.SimulateDeviceReset(id); }

 private:
  SimBackend sim_;
};

TEST(DeliveryMatrix, StartupDisabledAcceptsNothing) {
  SimBackend backend;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(1000, /*driver_station_enabled=*/false);
  const State s = gw.snapshot();
  EXPECT_FALSE(HasFlag(s, kEnabled));
  EXPECT_FALSE(HasFlag(s, kCommandActive));
  EXPECT_FALSE(AcceptVel(gw, kBootId, 10.0, 1000, 1));
}

TEST(DeliveryMatrix, EnableThenNormalOpMirrorsDemand) {
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

TEST(DeliveryMatrix, DisableWhileMovingStopsAndNeutralizes) {
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

TEST(DeliveryMatrix, TargetExpiryInvalidatesWithoutResend) {
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

TEST(DeliveryMatrix, MissingFeedbackFaultsAndBlocksCommands) {
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
  Command stale_obs =
      FreshCommand(gw, kBootId, Mode::kVelocity, 10.0, 2000, 2);
  EXPECT_FALSE(gw.Accept(stale_obs, 2, recovered_now));
  EXPECT_TRUE(gw.Accept(FreshCommand(gw, kBootId, Mode::kVelocity, 10.0,
                                     recovered_now, 2),
                        2, recovered_now));
}

TEST(DeliveryMatrix, CompanionRestartRejectsPreviousBootCommands) {
  SimBackend backend;
  Gateway old_gw(RollerConfig(), backend, 0xA11CEULL);
  old_gw.Tick(2000, true);
  ASSERT_TRUE(AcceptVel(old_gw, 0xA11CEULL, 10.0, 2000, 1));
  // The companion restarts: same config, new boot ID, epoch back at 1.
  Gateway new_gw(RollerConfig(), backend, kBootId);
  new_gw.Tick(3000, true);
  Command stale =
      FreshCommand(old_gw, 0xA11CEULL, Mode::kVelocity, 10.0, 3000, 2);
  EXPECT_FALSE(new_gw.Accept(stale, 2, 3000));
  EXPECT_TRUE(new_gw.Accept(
      FreshCommand(new_gw, kBootId, Mode::kVelocity, 10.0, 3000, 1), 1, 3000));
}

TEST(DeliveryMatrix, ControllerRestartRequiresFreshEpoch) {
  SimBackend backend;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, true);
  ASSERT_TRUE(AcceptVel(gw, kBootId, 10.0, 2000, 1));
  const uint64_t epoch_before = gw.snapshot().epoch;
  backend.SimulateDeviceReset(kMotorId);
  gw.Tick(7000, true);
  // Stop condition: the reset invalidated the demand (epoch bump). The next
  // read off the restored device is zero, never the pre-reset demand.
  EXPECT_GT(gw.snapshot().epoch, epoch_before);
  EXPECT_FALSE(HasFlag(gw.snapshot(), kCommandActive));
  gw.Tick(12000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
  // Resume requires the fresh epoch: the pre-reset command is stale forever.
  Command stale = FreshCommand(gw, kBootId, Mode::kVelocity, 10.0, 12000, 2);
  stale.epoch = epoch_before;
  EXPECT_FALSE(gw.Accept(stale, 2, 12000));
  EXPECT_TRUE(AcceptVel(gw, kBootId, 10.0, 12000, 2));
}

TEST(DeliveryMatrix, FailedApplyLatchesFaultUntilReconfigure) {
  FaultyBackend backend;
  backend.fail_apply = true;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, false);
  gw.Tick(3000, true);
  // The failed transaction latches the fault and neutralizes: nothing moves.
  EXPECT_FALSE(gw.Accept(
      FreshCommand(gw, kBootId, Mode::kVelocity, 10.0, 3000, 1), 1, 3000));
  EXPECT_TRUE(HasFlag(gw.snapshot(), kHardwareFault));
  backend.fail_apply = false;
  gw.Tick(8000, true);
  // Clearing the device fault is not enough: the latch holds until a
  // successful reconfiguration, so Accepts keep failing.
  EXPECT_FALSE(gw.Accept(
      FreshCommand(gw, kBootId, Mode::kVelocity, 10.0, 8000, 2), 2, 8000));
  // Recovery is a disabled-side reconfiguration (changed, so it is not the
  // idempotent no-op) followed by a fully fresh command.
  gw.Tick(9000, false);
  Config recovered = RollerConfig();
  recovered.motors[0].max_velocity_rps = 50.0;
  ASSERT_TRUE(gw.Reconfigure(recovered));
  EXPECT_FALSE(HasFlag(gw.snapshot(), kHardwareFault));
  gw.Tick(10000, true);
  EXPECT_TRUE(AcceptVel(gw, kBootId, 10.0, 10000, 3));
}

TEST(DeliveryMatrix, FailedRestoreLatchesFaultAndDropsDemand) {
  FaultyBackend backend;
  backend.fail_restore = true;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, true);
  ASSERT_TRUE(AcceptVel(gw, kBootId, 10.0, 2000, 1));
  backend.SimulateDeviceReset(kMotorId);
  gw.Tick(7000, true);
  EXPECT_TRUE(HasFlag(gw.snapshot(), kHardwareFault));
  EXPECT_FALSE(HasFlag(gw.snapshot(), kCommandActive));
  // The reset tick read before the restore ran; the next read off the
  // restored device is zero, never the pre-reset demand.
  gw.Tick(12000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
}

}  // namespace
}  // namespace talos::hardware
