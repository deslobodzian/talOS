#include "talOS/hardware/gate.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
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

TEST(GateTest, FirstStateGatedOnConfigOnly) {
  const State next = TestState();
  EXPECT_TRUE(AcceptState(false, State{}, 7, next));
  EXPECT_TRUE(AcceptState(false, State{}, 0, next));
  EXPECT_FALSE(AcceptState(false, State{}, 99, next));
}

TEST(GateTest, ConfigIdMismatchRejected) {
  const State prev = TestState();
  State next = TestState();
  next.config_id = 8;
  next.sample_time_us = 2000;
  EXPECT_FALSE(AcceptState(true, prev, 7, next));
}

TEST(GateTest, ZeroConfigIdAcceptsAny) {
  const State prev = TestState();
  State next = TestState();
  next.config_id = 1234;
  next.sample_time_us = 2000;
  EXPECT_TRUE(AcceptState(false, State{}, 0, next));
  EXPECT_TRUE(AcceptState(true, prev, 0, next));
}

TEST(GateTest, StaleOrDuplicateSampleRejected) {
  const State prev = TestState();
  State older = TestState();
  older.sample_time_us = 999;
  EXPECT_FALSE(AcceptState(true, prev, 7, older));
  // Equal timestamps are duplicates, not new data.
  EXPECT_FALSE(AcceptState(true, prev, 7, prev));
  State newer = TestState();
  newer.sample_time_us = 1001;
  EXPECT_TRUE(AcceptState(true, prev, 7, newer));
}

TEST(GateTest, NewBootAcceptsOlderTimestamp) {
  const State prev = TestState();
  State rebooted = TestState();
  rebooted.boot_id = 4;
  rebooted.sample_time_us = 10;
  EXPECT_TRUE(AcceptState(true, prev, 7, rebooted));
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

TEST(GateTest, NeutralCommandEchoesStateAndCoasts) {
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
}

TEST(GateTest, NeutralCommandClampsToMaxMotors) {
  const State state = TestState();
  std::vector<uint16_t> ids(kMaxMotors + 4, 1);
  const Command command = NeutralCommand(state, ids);
  EXPECT_EQ(command.count, kMaxMotors);
}

}  // namespace
}  // namespace talos::hardware
