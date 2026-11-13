// Device-reset restore supervision over the real Gateway + SimBackend: the
// retry schedule, its cap, and the fresh information a resume requires. Each
// is checked on the mode the gateway last applied to the reset motor (a
// neutral pass, Neutral(), or the motor's request in an accepted command)
// and, once it drives again, on what the simulated wheel does. While the
// gateway masks the motor its published velocity reads zero whatever the
// wheel does (an invalid sample carries no measurement), so the velocity
// says nothing then.

#include <gtest/gtest.h>

#include <cstdint>
#include <span>
#include <vector>

#include "talOS/hardware/gateway.h"
#include "talOS/hardware/sim_backend.h"

namespace talos::hardware {
namespace {

constexpr uint16_t kMotorId = 7;
constexpr uint16_t kOtherMotorId = 8;
constexpr uint64_t kBootId = 0xB0071DULL;

// The roller and a motor its reset leaves alone, whose valid feedback keeps
// the roller's restore from being total feedback loss: with the roller alone,
// the State read while it restores would fault and end every command
// (SimBackend.RestoringChainIsInvalidAsOnTheRobot).
Config RollerConfig() {
  Config c;
  c.commissioned = true;
  MotorConfig m;
  m.id = kMotorId;
  m.name = "roller";
  m.can_id = 5;
  m.bus = "rio";
  c.motors.push_back(m);
  m.id = kOtherMotorId;
  m.name = "feeder";
  m.can_id = 6;
  c.motors.push_back(m);
  return c;
}

Command FreshCommand(const Gateway& gw, double demand, uint64_t now) {
  Command c;
  c.config_id = gw.snapshot().config_id;
  c.boot_id = kBootId;
  c.epoch = gw.snapshot().epoch;
  c.observed_time_us = now;
  c.count = 2;
  c.motors[0] = MotorRequest{kMotorId, Mode::kVelocity, 0, demand, 0.0};
  c.motors[1] = MotorRequest{kOtherMotorId};
  return c;
}

bool AcceptVel(Gateway& gw, double demand, uint64_t now, uint64_t sequence) {
  return gw.Accept(FreshCommand(gw, demand, now), sequence, now);
}

// The deterministic SimBackend with restores that fail on demand, recording
// what the gateway applied to each motor. (SimBackend is final, so this wraps
// it instead of deriving from it.) A failed restore still ends the simulated
// one, so the chain reads valid from the backend afterwards: only the
// gateway's own mask of an unavailable device keeps it neutral, which is what
// these tests check.
class FaultyBackend : public Backend {
 public:
  bool fail_restore = false;
  int restore_failures_remaining{};
  int restore_attempts{};
  bool Configure(const Config& config) override {
    applied_.clear();
    return sim_.Configure(config);
  }
  bool Read(State& state) override { return sim_.Read(state); }
  bool Apply(std::span<const MotorRequest> commands) override {
    applied_.assign(commands.begin(), commands.end());
    return sim_.Apply(commands);
  }
  // The mode of the gateway's last Apply for motor `id`, neutral since a
  // Neutral() or before any Apply.
  Mode applied_mode(uint16_t id) const {
    for (const auto& request : applied_) {
      if (request.id == id) return request.mode;
    }
    return Mode::kNeutral;
  }
  std::span<const uint16_t> ApplyFailedDevices() const override {
    return sim_.ApplyFailedDevices();
  }
  bool ApplyOutputs(std::span<const DigitalOutputRequest> digital_outputs,
                    std::span<const PwmRequest> pwm_outputs) override {
    return sim_.ApplyOutputs(digital_outputs, pwm_outputs);
  }
  void Neutral() override {
    applied_.clear();
    sim_.Neutral();
  }
  bool PollDeviceResets(std::vector<uint16_t>& reset_ids) override {
    return sim_.PollDeviceResets(reset_ids);
  }
  void BeginRestoreAfterReset(std::span<const uint16_t> ids) override {
    ++restore_attempts;
    sim_.BeginRestoreAfterReset(ids);
  }
  RestoreStatus PollRestore() override {
    const auto status = sim_.PollRestore();
    if (restore_failures_remaining > 0) {
      --restore_failures_remaining;
      return RestoreStatus::kFailed;
    }
    return fail_restore ? RestoreStatus::kFailed : status;
  }
  void SimulateDeviceReset(uint16_t id) { sim_.SimulateDeviceReset(id); }

 private:
  SimBackend sim_;
  std::vector<MotorRequest> applied_;
};

// REQ-SAFE-24: a reset roller goes neutral in the tick that finds it, and a
// command observed before the reset never drives it again.
TEST(GatewayScenario, ControllerResetRequiresFreshMotorObservation) {
  FaultyBackend backend;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, true);
  ASSERT_TRUE(AcceptVel(gw, 10.0, 2000, 1));
  ASSERT_EQ(backend.applied_mode(kMotorId), Mode::kVelocity);
  const uint64_t epoch_before = gw.snapshot().epoch;
  backend.SimulateDeviceReset(kMotorId);
  gw.Tick(7000, true);  // Reset found: the roller neutral, its restore begun.
  EXPECT_EQ(gw.snapshot().epoch, epoch_before);
  EXPECT_TRUE(gw.snapshot().flags & kCommandActive);
  EXPECT_FALSE(gw.snapshot().motors[0].valid);
  EXPECT_EQ(backend.applied_mode(kMotorId), Mode::kNeutral);
  gw.Tick(12000, true);  // Read while restoring, then restored.
  EXPECT_FALSE(gw.snapshot().motors[0].valid);
  const Command stale = FreshCommand(gw, 10.0, 2000);
  EXPECT_TRUE(gw.Accept(stale, 2, 12000));
  EXPECT_EQ(backend.applied_mode(kMotorId), Mode::kNeutral);
  gw.Tick(13000, true);
  EXPECT_TRUE(gw.snapshot().motors[0].valid);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
  ASSERT_TRUE(AcceptVel(gw, 10.0, 13000, 3));
  gw.Tick(18000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 10.0);
}

// REQ-SAFE-24: through two failed restores the gateway keeps the roller
// neutral under fresh commands, though the backend reads it valid again, and
// drives it only on a command observing a State after the restore succeeds.
TEST(GatewayScenario, RestoreRetriesTwiceThenResumesWhileEnabled) {
  FaultyBackend backend;
  backend.restore_failures_remaining = 2;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(1000, true);
  ASSERT_TRUE(AcceptVel(gw, 10, 1000, 1));
  const auto stale = FreshCommand(gw, 10, 1000);
  backend.SimulateDeviceReset(kMotorId);
  gw.Tick(2000, true);
  gw.Tick(3000, true);
  EXPECT_TRUE(gw.configured());
  EXPECT_EQ(backend.restore_attempts, 1);
  gw.Tick(252999, true);
  EXPECT_EQ(backend.restore_attempts, 1);
  EXPECT_EQ(gw.snapshot().sample_time_us, 252999u);
  EXPECT_FALSE(gw.snapshot().motors[0].valid);
  EXPECT_TRUE(AcceptVel(gw, 10, 252999, 3));
  EXPECT_EQ(backend.applied_mode(kMotorId), Mode::kNeutral);
  gw.Tick(253000, true);
  EXPECT_EQ(backend.restore_attempts, 2);
  gw.Tick(254000, true);
  gw.Tick(753999, true);
  EXPECT_EQ(backend.restore_attempts, 2);
  EXPECT_FALSE(gw.snapshot().motors[0].valid);
  EXPECT_TRUE(AcceptVel(gw, 10, 753999, 4));
  EXPECT_EQ(backend.applied_mode(kMotorId), Mode::kNeutral);
  gw.Tick(754000, true);
  EXPECT_EQ(backend.restore_attempts, 3);
  gw.Tick(755000, true);  // Read while restoring, then restored.
  gw.Tick(756000, true);
  EXPECT_TRUE(gw.configured());
  EXPECT_FALSE(gw.snapshot().flags & kHardwareFault);
  EXPECT_EQ(gw.restore_attempt_count(), 3u);
  EXPECT_EQ(gw.snapshot().counters.device_reset_count, 1u);
  EXPECT_TRUE(gw.snapshot().motors[0].valid);
  EXPECT_FALSE(gw.Accept(stale, 5, 756000));
  ASSERT_TRUE(AcceptVel(gw, 10, 756000, 6));
  EXPECT_EQ(backend.applied_mode(kMotorId), Mode::kVelocity);
  gw.Tick(757000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 10);
}

// REQ-SAFE-24: every retry of a failing restore keeps its backoff, doubling
// to the cap, and the roller neutral under fresh commands between retries.
TEST(GatewayScenario, RestoreBackoffCapsAtTwoSeconds) {
  FaultyBackend backend;
  backend.fail_restore = true;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(1000, true);
  backend.SimulateDeviceReset(kMotorId);
  gw.Tick(2000, true);
  uint64_t now = 3000;
  uint64_t sequence = 0;
  int attempts = 1;
  for (uint64_t delay : {250000u, 500000u, 1000000u, 2000000u, 2000000u}) {
    gw.Tick(now, true);
    gw.Tick(now + delay - 1, true);
    EXPECT_EQ(backend.restore_attempts, attempts);
    EXPECT_EQ(gw.snapshot().sample_time_us, now + delay - 1);
    EXPECT_FALSE(gw.snapshot().motors[0].valid);
    EXPECT_TRUE(AcceptVel(gw, 10, now + delay - 1, ++sequence));
    EXPECT_EQ(backend.applied_mode(kMotorId), Mode::kNeutral);
    gw.Tick(now + delay, true);
    EXPECT_EQ(backend.restore_attempts, ++attempts);
    EXPECT_TRUE(AcceptVel(gw, 10, now + delay, ++sequence));
    now += delay + 1;
  }
  EXPECT_EQ(gw.restore_attempt_count(), 6u);
}

}  // namespace
}  // namespace talos::hardware
