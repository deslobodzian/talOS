#include "talOS/hardware/gate.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
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

// REQ-SAFE-11
TEST(GateTest, AcceptStateRejectsOtherConfigAndStaleSamplesOfOneBoot) {
  const State prev = TestState();
  // The first sample is gated on the config id only; zero accepts any.
  EXPECT_TRUE(AcceptState(false, State{}, 7, prev, /*boot_retired=*/false));
  EXPECT_TRUE(AcceptState(false, State{}, 0, prev, /*boot_retired=*/false));
  EXPECT_FALSE(AcceptState(false, State{}, 99, prev, /*boot_retired=*/false));

  State other_config = TestState();
  other_config.config_id = 8;
  other_config.sample_time_us = 2000;
  EXPECT_FALSE(
      AcceptState(true, prev, 7, other_config, /*boot_retired=*/false));
  EXPECT_TRUE(AcceptState(true, prev, 0, other_config, /*boot_retired=*/false));

  State older = TestState();
  older.sample_time_us = 999;
  EXPECT_FALSE(AcceptState(true, prev, 7, older, /*boot_retired=*/false));
  // Equal timestamps are duplicates, not new data.
  EXPECT_FALSE(AcceptState(true, prev, 7, prev, /*boot_retired=*/false));
  State newer = TestState();
  newer.sample_time_us = 1001;
  EXPECT_TRUE(AcceptState(true, prev, 7, newer, /*boot_retired=*/false));
  // A new boot restarts the clock; a recently retired boot is ignored even
  // with a newer timestamp and matching (or wildcard) config id.
  older.boot_id = 4;
  EXPECT_TRUE(AcceptState(true, prev, 7, older, /*boot_retired=*/false));
  State retired = TestState();
  retired.boot_id = 2;
  retired.sample_time_us = 2000;
  EXPECT_FALSE(AcceptState(true, prev, 7, retired, /*boot_retired=*/true));
  EXPECT_FALSE(AcceptState(true, prev, 0, retired, /*boot_retired=*/true));
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

// REQ-SAFE-22
TEST(GateTest, MotorFeedbackValidRequiresPresentValidRemoteCANcoder) {
  MotorConfig motor;
  motor.id = 1;
  motor.feedback = Feedback::kRemoteCANcoder;
  motor.feedback_sensor_id = 10;
  const std::array<MotorConfig, 1> motors{motor};

  State state;
  state.motor_count = 1;
  state.motors[0].id = 1;
  state.motors[0].valid = true;

  // No sensors reported at all: remote feedback cannot be verified.
  state.sensor_count = 0;
  EXPECT_FALSE(MotorFeedbackValid(motors, state, 0));

  // Only an unrelated sensor ID is present and valid.
  state.sensor_count = 1;
  state.sensors[0] = {.id = 99, .valid = true};
  EXPECT_FALSE(MotorFeedbackValid(motors, state, 0));

  // Matching remote CANcoder present but invalid.
  state.sensors[0] = {.id = 10, .valid = false};
  EXPECT_FALSE(MotorFeedbackValid(motors, state, 0));

  // Matching remote CANcoder present and valid.
  state.sensors[0] = {.id = 10, .valid = true};
  EXPECT_TRUE(MotorFeedbackValid(motors, state, 0));

  // Motor's own sample invalid vetoes even with a valid remote CANcoder.
  state.motors[0].valid = false;
  EXPECT_FALSE(MotorFeedbackValid(motors, state, 0));
}

// REQ-SAFE-22
TEST(GateTest, MotorMayDriveChecksMasterRemoteCANcoderForRotorFollower) {
  for (const bool master_first : {true, false}) {
    SCOPED_TRACE(master_first ? "master_first" : "follower_first");
    MotorConfig master;
    master.id = 1;
    master.feedback = Feedback::kRemoteCANcoder;
    master.feedback_sensor_id = 10;

    MotorConfig follower;
    follower.id = 2;
    follower.feedback = Feedback::kRotor;
    follower.master_id = master.id;

    const std::size_t master_idx = master_first ? 0 : 1;
    const std::size_t follower_idx = master_first ? 1 : 0;
    std::array<MotorConfig, 2> motors{};
    motors[master_idx] = master;
    motors[follower_idx] = follower;

    State state;
    state.motor_count = 2;
    state.motors[master_idx] = {.id = master.id, .valid = true};
    state.motors[follower_idx] = {.id = follower.id, .valid = true};
    state.sensor_count = 1;
    state.sensors[0] = {.id = 10, .valid = true};

    EXPECT_TRUE(MotorMayDrive(motors, state, master_idx));
    EXPECT_TRUE(MotorMayDrive(motors, state, follower_idx));

    // Master motor sample stays valid, but its remote CANcoder is invalid:
    // neither the master nor its Rotor follower may drive.
    state.sensors[0].valid = false;
    EXPECT_FALSE(MotorMayDrive(motors, state, master_idx));
    EXPECT_FALSE(MotorMayDrive(motors, state, follower_idx));

    // Omitted remote CANcoder likewise blocks both.
    state.sensor_count = 0;
    EXPECT_FALSE(MotorMayDrive(motors, state, master_idx));
    EXPECT_FALSE(MotorMayDrive(motors, state, follower_idx));
  }
}

// The torque-current rule the gateway and the RoboRIO both hold: a foc motor
// with a positive limit, |demand| within it (the limit itself included), and
// a feedforward equal to 0 (-0.0 is 0); anything non-finite fails.
// REQ-SAFE-43
TEST(GateTest, TorqueCurrentAllowedBoundaries) {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const struct {
    const char* label;
    bool foc;
    double limit_a, demand_a, feedforward_v;
    bool allowed;
  } rows[] = {
      {"within", true, 5, 2, 0, true},
      {"0 A", true, 5, 0, 0, true},
      {"at the limit", true, 5, 5, 0, true},
      {"at the reverse limit", true, 5, -5, 0, true},
      {"-0.0 feedforward", true, 5, 2, -0.0, true},
      {"over the limit", true, 5, std::nextafter(5.0, 6.0), 0, false},
      {"under the reverse limit", true, 5, -5.5, 0, false},
      {"NaN demand", true, 5, kNan, 0, false},
      {"infinite demand", true, 5, kInf, 0, false},
      {"-infinite demand", true, 5, -kInf, 0, false},
      {"feedforward", true, 5, 2, 0.5, false},
      {"NaN feedforward", true, 5, 2, kNan, false},
      {"infinite feedforward", true, 5, 2, kInf, false},
      {"no foc", false, 5, 2, 0, false},
      {"no limit", true, 0, 0, 0, false},
      {"NaN limit", true, kNan, 0, 0, false},
  };
  for (const auto& row : rows) {
    SCOPED_TRACE(row.label);
    MotorConfig m;
    m.foc = row.foc;
    m.max_torque_current_a = row.limit_a;
    const MotorRequest r{2, Mode::kTorqueCurrent, 0, row.demand_a,
                         row.feedforward_v};
    EXPECT_EQ(TorqueCurrentAllowed(m, r), row.allowed);
  }
}

// REQ-SAFE-02: each of configured and enabled is needed on its own, and a
// latched fault vetoes both.
TEST(GateTest, GatewayReadyNeedsConfiguredAndEnabledWithoutFault) {
  EXPECT_TRUE(GatewayReady(TestState()));
  State missing = TestState();
  missing.flags = kConfigured;
  EXPECT_FALSE(GatewayReady(missing));
  State unconfigured = TestState();
  unconfigured.flags = kEnabled;
  EXPECT_FALSE(GatewayReady(unconfigured));
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
