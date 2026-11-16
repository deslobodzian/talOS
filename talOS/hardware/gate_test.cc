#include "talOS/hardware/gate.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace talos::hardware {
namespace {

State TestState() {
  State state;
  state.config_id = 7;
  state.boot_id = 3;
  state.sample_time_us = 1000;
  state.flags = kConfigured | kEnabled;
  state.motor_count = 1;
  state.motors[0].id = 2;
  state.motors[0].valid = true;
  return state;
}

TEST(GateTest, AcceptStateRejectsOtherConfigAndStaleSamplesOfOneBoot) {
  const State prev = TestState();
  // The first sample is gated on the config id only; zero accepts any.
  EXPECT_TRUE(AcceptState(false, State{}, 7, prev));
  EXPECT_TRUE(AcceptState(false, State{}, 0, prev));
  EXPECT_FALSE(AcceptState(false, State{}, 99, prev));

  State other_config = TestState();
  other_config.config_id = 8;
  other_config.sample_time_us = 2000;
  EXPECT_FALSE(AcceptState(true, prev, 7, other_config));
  EXPECT_TRUE(AcceptState(true, prev, 0, other_config));

  State older = TestState();
  older.sample_time_us = 999;
  EXPECT_FALSE(AcceptState(true, prev, 7, older));
  // Equal timestamps are duplicates, not new data.
  EXPECT_FALSE(AcceptState(true, prev, 7, prev));
  State newer = TestState();
  newer.sample_time_us = 1001;
  EXPECT_TRUE(AcceptState(true, prev, 7, newer));
  // A new boot restarts the clock.
  older.boot_id = 4;
  EXPECT_TRUE(AcceptState(true, prev, 7, older));
}

// REQ-SAFE-22
TEST(GateTest, FeedbackFreshBoundary) {
  constexpr uint64_t kTimeoutUs = 100000;
  EXPECT_TRUE(FeedbackFresh(kTimeoutUs - 1, kTimeoutUs));
  EXPECT_FALSE(FeedbackFresh(kTimeoutUs, kTimeoutUs));
  // Unknown is stale under any bound, even one above every reportable age.
  constexpr uint32_t kUnknown = std::numeric_limits<uint32_t>::max();
  EXPECT_FALSE(FeedbackFresh(kUnknown, kTimeoutUs));
  EXPECT_FALSE(FeedbackFresh(kUnknown, uint64_t{kUnknown} + 1));
}

TEST(GateTest, GatewayReadyNeedsConfiguredAndEnabledWithoutFault) {
  EXPECT_TRUE(GatewayReady(TestState()));
  State missing = TestState();
  missing.flags = kConfigured;
  EXPECT_FALSE(GatewayReady(missing));
  State faulted = TestState();
  faulted.flags |= kHardwareFault;
  EXPECT_FALSE(GatewayReady(faulted));
  EXPECT_FALSE(GatewayReady(State{}));
}

TEST(GateTest, NeutralCommandEchoesStateAndCoastsAtMostMaxMotors) {
  const State state = TestState();
  const std::array<uint16_t, 2> ids{2, 5};
  const Command command = NeutralCommand(state, ids);
  EXPECT_EQ(command.config_id, state.config_id);
  EXPECT_EQ(command.boot_id, state.boot_id);
  EXPECT_EQ(command.epoch, state.epoch);
  EXPECT_EQ(command.observed_time_us, state.sample_time_us);
  EXPECT_EQ(command.count, 2u);
  EXPECT_EQ(command.motors[0].id, 2u);
  EXPECT_EQ(command.motors[1].id, 5u);
  for (std::size_t i = 0; i < command.count; ++i) {
    EXPECT_EQ(command.motors[i].mode, Mode::kNeutral);
    EXPECT_EQ(command.motors[i].demand, 0.0);
  }

  const std::vector<uint16_t> too_many(kMaxMotors + 4, 1);
  EXPECT_EQ(NeutralCommand(state, too_many).count, kMaxMotors);
}

}  // namespace
}  // namespace talos::hardware
