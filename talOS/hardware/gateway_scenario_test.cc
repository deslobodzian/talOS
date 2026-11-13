// Device-reset restore supervision over the real Gateway + SimBackend: the
// retry schedule, its cap, and the fresh information a resume requires,
// checked on what the simulated wheel does rather than on backend call logs.

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

// The deterministic SimBackend with restores that fail on demand. (SimBackend
// is final, so this wraps it instead of deriving from it.)
class FaultyBackend : public Backend {
 public:
  bool fail_restore = false;
  int restore_failures_remaining{};
  int restore_attempts{};
  bool Configure(const Config& config) override {
    return sim_.Configure(config);
  }
  bool Read(State& state) override { return sim_.Read(state); }
  bool Apply(std::span<const MotorRequest> commands) override {
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

TEST(GatewayScenario, ControllerResetRequiresFreshMotorObservation) {
  SimBackend backend;
  Gateway gw(RollerConfig(), backend, kBootId);
  gw.Tick(2000, true);
  ASSERT_TRUE(AcceptVel(gw, 10.0, 2000, 1));
  const uint64_t epoch_before = gw.snapshot().epoch;
  backend.SimulateDeviceReset(kMotorId);
  gw.Tick(7000, true);
  EXPECT_EQ(gw.snapshot().epoch, epoch_before);
  EXPECT_TRUE(gw.snapshot().flags & kCommandActive);
  gw.Tick(12000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
  const Command stale = FreshCommand(gw, 10.0, 2000);
  EXPECT_TRUE(gw.Accept(stale, 2, 12000));
  gw.Tick(13000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0.0);
  ASSERT_TRUE(AcceptVel(gw, 10.0, 13000, 3));
  gw.Tick(18000, true);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 10.0);
}

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
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0);
  EXPECT_TRUE(AcceptVel(gw, 10, 252999, 3));
  gw.Tick(253000, true);
  EXPECT_EQ(backend.restore_attempts, 2);
  gw.Tick(254000, true);
  gw.Tick(753999, true);
  EXPECT_EQ(backend.restore_attempts, 2);
  EXPECT_DOUBLE_EQ(gw.snapshot().motors[0].velocity_rps, 0);
  gw.Tick(754000, true);
  EXPECT_EQ(backend.restore_attempts, 3);
  gw.Tick(755000, true);  // Read while restoring, then restored.
  gw.Tick(756000, true);
  EXPECT_TRUE(gw.configured());
  EXPECT_FALSE(gw.snapshot().flags & kHardwareFault);
  EXPECT_EQ(gw.restore_attempt_count(), 3u);
  EXPECT_EQ(gw.snapshot().counters.device_reset_count, 1u);
  EXPECT_TRUE(gw.snapshot().motors[0].valid);
  EXPECT_FALSE(gw.Accept(stale, 4, 756000));
  ASSERT_TRUE(AcceptVel(gw, 10, 756000, 5));
  gw.Tick(757000, true);
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
    EXPECT_TRUE(AcceptVel(gw, 10, now + delay, attempts));
    now += delay + 1;
  }
  EXPECT_EQ(gw.restore_attempt_count(), 6u);
}

}  // namespace
}  // namespace talos::hardware
