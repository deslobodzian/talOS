// The bridge's request trust at component tier (docs/TESTING.md §2): which
// requests merge (identity, age, ownership, transitions), what a request
// leaves out and what an owner timeout neutralizes, and which gateway frames
// reach the merge. Each HardwareNode is driven through an in-process gateway
// (in_process_gateway.h) on the synthetic clock Tick() takes: no socket, no
// wall clock. hardware_node_test.cc covers the UDP path into the same
// handler.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "talOS/bridge/in_process_gateway.h"
#include "talOS/bridge/merge.h"
#include "talOS/bridge/node.h"
#include "talOS/bridge/node_test_support.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/config_wire.h"
#include "talOS/hardware/messages.h"
#include "talOS/ipc/publisher.h"
#include "talOS/ipc/subscriber.h"

namespace talos::hardware {
namespace {
using namespace bridge_test;  // NOLINT(google-build-using-namespace)

// A bridge as --sim launches it (so its config id forces commissioning,
// NodeConfigId), with the gateway end of its link in process.
struct Bridge {
  explicit Bridge(const config::RobotConfig& cfg)
      : node{cfg, "127.0.0.1", 5802, 5803, /*simulation=*/true},
        gateway{node} {}
  HardwareNode node;
  InProcessGateway gateway;
};

// The commands among `frames`, in the order sent.
std::vector<Command> Commands(
    const std::vector<InProcessGateway::Frame>& frames) {
  std::vector<Command> commands;
  for (const auto& frame : frames) {
    if (frame.type != protocol::FrameType::kHardwareCommand) continue;
    Command command{};
    EXPECT_TRUE(Decode(std::span<const uint8_t>{frame.payload}, command));
    commands.push_back(command);
  }
  return commands;
}

// How many configuration frames are among `frames`: one per push of the
// test configs, which fit one chunk.
unsigned ConfigPushes(const std::vector<InProcessGateway::Frame>& frames) {
  unsigned count = 0;
  for (const auto& frame : frames) {
    count += frame.type == protocol::FrameType::kHardwareConfig ? 1 : 0;
  }
  return count;
}

// REQ-SAFE-15
TEST(HardwareNodeTrustTest, LastSubsystemTimeoutSendsOneFreshNeutralFrame) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  InProcessGateway& gateway = bridge.gateway;
  const auto config_id = NodeConfigId(cfg);
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 1, 1000), 1000000));
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  Command request{};
  request.config_id = config_id;
  request.boot_id = 11;
  request.epoch = 1;
  request.observed_time_us = 1000;
  request.count = 1;
  request.motors[0] = {cfg.subsystems.at("intake").motors[0], Mode::kVoltage, 0,
                       4.0, 1.0};
  request.digital_output_count = 1;
  request.digital_outputs[0] = {cfg.hardware.digital_outputs[0].id, true};
  request.pwm_output_count = 1;
  request.pwm_outputs[0] = {cfg.hardware.pwm_outputs[0].id, 0.7};
  PublishCommand(pub, request);
  node.Tick(1001000);
  ASSERT_EQ(node.commands_sent(), 1u);
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 1, 12000), 1011000));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));

  // The last active subsystem expires after its 50 ms owner timeout. Its
  // neutral command must reference the new state, not the old request's
  // observation.
  node.Tick(1051001);
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  ASSERT_EQ(node.commands_sent(), 2u);

  const auto commands = Commands(gateway.TakeSent());
  ASSERT_EQ(commands.size(), 2u);
  EXPECT_EQ(commands[0].motors[1].mode, Mode::kVoltage);
  const auto& neutral = commands[1];
  EXPECT_EQ(neutral.config_id, config_id);
  EXPECT_EQ(neutral.boot_id, 11u);
  EXPECT_EQ(neutral.epoch, 1u);
  EXPECT_EQ(neutral.observed_time_us, 12000u);
  for (std::size_t i = 0; i < neutral.count; ++i) {
    EXPECT_EQ(neutral.motors[i].mode, Mode::kNeutral);
    EXPECT_DOUBLE_EQ(neutral.motors[i].demand, 0);
    EXPECT_DOUBLE_EQ(neutral.motors[i].feedforward_v, 0);
  }
  ASSERT_EQ(neutral.digital_output_count, 1u);
  EXPECT_FALSE(neutral.digital_outputs[0].value);
  ASSERT_EQ(neutral.pwm_output_count, 1u);
  EXPECT_DOUBLE_EQ(neutral.pwm_outputs[0].output, 0);

  for (uint64_t now_us = 1052000; now_us <= 1152000; now_us += 1000) {
    node.Tick(now_us);
  }
  EXPECT_EQ(node.commands_sent(), 2u);
  EXPECT_TRUE(Commands(gateway.TakeSent()).empty());
}

TEST(HardwareNodeTrustTest, LatestRequestNeutralizesOmittedOutputs) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  node.Tick(1000000);
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  Command request{};
  request.config_id = NodeConfigId(cfg);
  request.count = 1;
  request.motors[0] = {cfg.subsystems.at("intake").motors[0], Mode::kVoltage, 0,
                       4.0, 0.0};
  request.digital_output_count = 1;
  request.digital_outputs[0] = {cfg.hardware.digital_outputs[0].id, true};
  request.pwm_output_count = 1;
  request.pwm_outputs[0] = {cfg.hardware.pwm_outputs[0].id, 0.7};
  PublishCommand(pub, request);
  node.Tick(1001000);
  EXPECT_TRUE(node.current_command().digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(node.current_command().pwm_outputs[0].output, 0.7);
  request.count = 0;
  request.digital_output_count = 0;
  request.pwm_output_count = 0;
  PublishCommand(pub, request);
  node.Tick(1002000);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_FALSE(node.current_command().digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(node.current_command().pwm_outputs[0].output, 0.0);
  EXPECT_EQ(node.current_command().motors[1].mode, Mode::kNeutral);
}

// REQ-SAFE-19: a request for another subsystem's device is rejected whole
TEST(HardwareNodeTrustTest, RejectsRequestsForDevicesOwnedByAnotherSubsystem) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  ASSERT_EQ(cfg.hardware.motors.size(), 2u);
  ASSERT_EQ(cfg.hardware.digital_outputs.size(), 1u);
  ASSERT_EQ(cfg.hardware.pwm_outputs.size(), 1u);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint16_t beacon = cfg.hardware.digital_outputs[0].id;
  const uint16_t strip = cfg.hardware.pwm_outputs[0].id;
  ASSERT_TRUE(cfg.subsystems.at("drivetrain").motors.size() == 1u &&
              cfg.subsystems.at("drivetrain").motors[0] == drive);
  ASSERT_TRUE(cfg.subsystems.at("intake").motors.size() == 1u &&
              cfg.subsystems.at("intake").motors[0] == roller);

  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  const uint64_t config_id = NodeConfigId(cfg);
  ASSERT_TRUE(
      bridge.gateway.Deliver(TrustState(config_id, 11, 1, 1000), 1000000));

  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};

  // An intake request that accidentally addresses the drivetrain motor: the
  // whole request is rejected, so neither the stray drive demand nor the
  // intake's own roller demand merges.
  Command cross{};
  cross.config_id = config_id;
  cross.boot_id = 11;
  cross.epoch = 1;
  cross.observed_time_us = 1000;
  cross.count = 2;
  cross.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  cross.motors[1] = {drive, Mode::kVelocity, 0, 99.0, 0.0};
  PublishCommand(intake_pub, cross);
  node.Tick(1001000);

  EXPECT_EQ(node.ownership_rejected(), 1u);
  EXPECT_EQ(node.identity_rejected(), 0u);
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  const Command& cmd1 = node.current_command();
  EXPECT_EQ(cmd1.motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(cmd1.motors[1].mode, Mode::kNeutral);

  // Same for the other actuator kinds: drivetrain must not drive the intake's
  // digital output or PWM channel.
  Command cross_do{};
  cross_do.config_id = config_id;
  cross_do.boot_id = 11;
  cross_do.epoch = 1;
  cross_do.observed_time_us = 1000;
  cross_do.count = 1;
  cross_do.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  cross_do.digital_output_count = 1;
  cross_do.digital_outputs[0] = {beacon, true};
  PublishCommand(drive_pub, cross_do);
  node.Tick(1002000);

  EXPECT_EQ(node.ownership_rejected(), 2u);
  EXPECT_EQ(cmd1.motors[0].mode, Mode::kNeutral);
  EXPECT_FALSE(node.current_command().digital_outputs[0].value);

  Command cross_pwm{};
  cross_pwm.config_id = config_id;
  cross_pwm.boot_id = 11;
  cross_pwm.epoch = 1;
  cross_pwm.observed_time_us = 1000;
  cross_pwm.count = 1;
  cross_pwm.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  cross_pwm.pwm_output_count = 1;
  cross_pwm.pwm_outputs[0] = {strip, 0.9};
  PublishCommand(drive_pub, cross_pwm);
  node.Tick(1003000);

  EXPECT_EQ(node.ownership_rejected(), 3u);
  EXPECT_DOUBLE_EQ(node.current_command().pwm_outputs[0].output, 0.0);

  // Positive control: requests that stay inside their own ownership still
  // merge.
  Command intake_ok{};
  intake_ok.config_id = config_id;
  intake_ok.boot_id = 11;
  intake_ok.epoch = 1;
  intake_ok.observed_time_us = 1000;
  intake_ok.count = 1;
  intake_ok.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  intake_ok.digital_output_count = 1;
  intake_ok.digital_outputs[0] = {beacon, true};
  intake_ok.pwm_output_count = 1;
  intake_ok.pwm_outputs[0] = {strip, 0.5};
  PublishCommand(intake_pub, intake_ok);
  Command drive_ok{};
  drive_ok.config_id = config_id;
  drive_ok.boot_id = 11;
  drive_ok.epoch = 1;
  drive_ok.observed_time_us = 1000;
  drive_ok.count = 1;
  drive_ok.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  PublishCommand(drive_pub, drive_ok);
  node.Tick(1004000);

  EXPECT_EQ(node.ownership_rejected(), 3u);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  const Command& cmd2 = node.current_command();
  EXPECT_DOUBLE_EQ(cmd2.motors[0].demand, 15.0);
  EXPECT_DOUBLE_EQ(cmd2.motors[1].demand, 30.0);
  EXPECT_TRUE(cmd2.digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(cmd2.pwm_outputs[0].output, 0.5);
}

// REQ-SAFE-16
TEST(HardwareNodeTrustTest, FreshSubsystemCannotRefreshStaleSubsystem) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  InProcessGateway& gateway = bridge.gateway;
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 1, 5000), 1000000));

  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};

  Command d{};
  d.config_id = config_id;
  d.boot_id = 11;
  d.epoch = 1;
  d.observed_time_us = 5000;
  d.count = 1;
  d.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  Command r{};
  r.config_id = config_id;
  r.boot_id = 11;
  r.epoch = 1;
  r.observed_time_us = 5000;
  r.count = 1;
  r.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  PublishCommand(drive_pub, d);
  PublishCommand(intake_pub, r);
  node.Tick(1001000);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);

  // A newer sample arrives. Only drivetrain publishes from it; intake stays
  // quiet but within its bridge liveness timeout, so its cached demand still
  // drives the merge. The merged observation must stay at intake's 5000:
  // drivetrain's freshness must not refresh intake's stale demand.
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 1, 6000), 1002000));
  Command d_new = d;
  d_new.observed_time_us = 6000;
  d_new.motors[0].demand = 16.0;
  PublishCommand(drive_pub, d_new);
  node.Tick(1002500);

  EXPECT_EQ(node.identity_rejected(), 0u);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 16.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);

  // Intake catches up from the newer sample; now both agree and the merge
  // advances.
  Command r_new = r;
  r_new.observed_time_us = 6000;
  r_new.motors[0].demand = 31.0;
  PublishCommand(intake_pub, r_new);
  node.Tick(1003000);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  EXPECT_EQ(node.current_command().observed_time_us, 6000u);

  // Monotonicity is per subsystem: drivetrain replaying its own older demand
  // is rejected without disturbing intake's merged demand.
  Command d_replay = d;
  d_replay.motors[0].demand = 99.0;
  PublishCommand(drive_pub, d_replay);
  node.Tick(1003500);
  EXPECT_EQ(node.identity_rejected(), 1u);
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 16.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  EXPECT_EQ(node.current_command().observed_time_us, 6000u);
}

TEST(HardwareNodeTrustTest, RejectsWrongIdentityWordsEvenWithinBound) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  ASSERT_TRUE(
      bridge.gateway.Deliver(TrustState(config_id, 11, 1, 5000), 1000000));

  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};

  // Age zero (observed == last_sample): freshness passes, so each rejection
  // below proves the config/boot/epoch words are still exact-match, and a
  // future observation is still rejected.
  Command base{};
  base.config_id = config_id;
  base.boot_id = 11;
  base.epoch = 1;
  base.observed_time_us = 5000;
  base.count = 1;
  base.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};

  Command bad_config = base;
  bad_config.config_id = config_id + 1;
  PublishCommand(intake_pub, bad_config);
  node.Tick(1001000);

  Command bad_boot = base;
  bad_boot.boot_id = 12;
  PublishCommand(intake_pub, bad_boot);
  node.Tick(1002000);

  Command bad_epoch = base;
  bad_epoch.epoch = 2;
  PublishCommand(intake_pub, bad_epoch);
  node.Tick(1003000);

  Command bad_future = base;
  bad_future.observed_time_us = 6000;
  PublishCommand(intake_pub, bad_future);
  node.Tick(1004000);

  EXPECT_EQ(node.identity_rejected(), 4u);
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[1].mode, Mode::kNeutral);

  PublishCommand(intake_pub, base);
  node.Tick(1005000);
  EXPECT_EQ(node.identity_rejected(), 4u);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);
}

// A request merges only while the State it was built from is younger than
// command_timeout_us against the latest State (MergeOwner::Admits, given
// HardwareNode's bound): the age bound the gateway itself applies to a
// command's observation, except that the gateway counts from its own clock,
// later than the latest State's sample by up to a RoboRIO tick and the
// transit. So a request just inside the bound can still merge and be refused
// there (the cost ARCHITECTURE.md "The tick" accepts); a wider bound would
// merge requests the gateway always refuses, whose observation, the oldest
// merged, makes it refuse every owner's outputs; a narrower one, such as the
// 50 ms owner timeout, would drop a live owner whose request lags the latest
// State by less than the timeout. One request built from the State sampled at
// 1000: admitted while the latest State is command_timeout_us - 1 newer,
// refused once it is command_timeout_us newer, its demand left out.
// REQ-SAFE-16, REQ-SAFE-39
TEST(HardwareNodeTrustTest,
     AdmitsARequestOnlyWhileItsStateIsYoungerThanTheTimeout) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.subsystems.at("intake").motors[0];
  const uint64_t config_id = NodeConfigId(cfg);
  const uint64_t timeout_us = cfg.hardware.command_timeout_us;
  ASSERT_LT(OwnerTimeoutUs(5000, timeout_us), timeout_us)
      << "the owner timeout must differ from the bound for this to tell them "
         "apart";
  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  InProcessGateway& gateway = bridge.gateway;
  // IntakeRequest is built from boot 11, epoch 1, sample 1000.
  constexpr uint64_t kBuiltUs = 1000;
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 1, kBuiltUs), 1000000));
  ipc::Publisher<Packet> intake{RequestTopic("intake")};

  ASSERT_TRUE(gateway.Deliver(
      TrustState(config_id, 11, 1, kBuiltUs + timeout_us - 1), 1001000));
  PublishCommand(intake, IntakeRequest(cfg, 4.0));
  node.Tick(1002000);
  EXPECT_EQ(node.identity_rejected(), 0u)
      << "built " << timeout_us - 1 << " us before the latest State";
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(MotorOf(node.current_command(), roller).mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(MotorOf(node.current_command(), roller).demand, 4.0);

  ASSERT_TRUE(gateway.Deliver(
      TrustState(config_id, 11, 1, kBuiltUs + timeout_us), 1003000));
  PublishCommand(intake, IntakeRequest(cfg, 5.0));
  node.Tick(1004000);
  EXPECT_EQ(node.identity_rejected(), 1u)
      << "built " << timeout_us << " us before the latest State";
  EXPECT_DOUBLE_EQ(MotorOf(node.current_command(), roller).demand, 4.0);
}

TEST(HardwareNodeTrustTest, RequiresFreshRequestsAfterControllerTransitions) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  InProcessGateway& gateway = bridge.gateway;
  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};

  const auto publish_valid = [&](uint64_t boot, uint64_t epoch,
                                 uint64_t observed, double drive_demand,
                                 double roller_demand) {
    Command d{};
    d.config_id = config_id;
    d.boot_id = boot;
    d.epoch = epoch;
    d.observed_time_us = observed;
    d.count = 1;
    d.motors[0] = {drive, Mode::kVelocity, 0, drive_demand, 0.0};
    PublishCommand(drive_pub, d);
    Command r{};
    r.config_id = config_id;
    r.boot_id = boot;
    r.epoch = epoch;
    r.observed_time_us = observed;
    r.count = 1;
    r.motors[0] = {roller, Mode::kVelocity, 0, roller_demand, 0.0};
    PublishCommand(intake_pub, r);
  };

  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 1, 1000), 1000000));
  publish_valid(11, 1, 1000, 15.0, 30.0);
  node.Tick(1001000);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 15.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  // Single tick, single send. Later phases count from the sends after each
  // State's own tick.
  const uint64_t sent_after_first_merge = node.commands_sent();
  EXPECT_EQ(sent_after_first_merge, 1u);

  // Reboot: new boot id. Without fresh requests nothing merges and nothing is
  // sent; the cached demands are neutralized.
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 12, 1, 2000), 1002000));
  const uint64_t sent_after_reboot = node.commands_sent();
  node.Tick(1003000);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(node.current_command().motors[1].mode, Mode::kNeutral);
  EXPECT_EQ(node.commands_sent(), sent_after_reboot);

  // Fresh requests stamped with the new identity resume motion.
  publish_valid(12, 1, 2000, 16.0, 31.0);
  node.Tick(1004000);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 16.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  const uint64_t sent_after_resume = node.commands_sent();
  EXPECT_EQ(sent_after_resume, sent_after_reboot + 1);

  // Disable/re-enable surfaces as an epoch bump: same treatment.
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 12, 2, 3000), 1005000));
  const uint64_t sent_after_epoch = node.commands_sent();
  node.Tick(1006000);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(node.commands_sent(), sent_after_epoch);

  publish_valid(12, 2, 3000, 17.0, 32.0);
  node.Tick(1007000);
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 17.0);
  const uint64_t sent_after_second_resume = node.commands_sent();
  EXPECT_EQ(sent_after_second_resume, sent_after_epoch + 1);

  // A bare enabled->disabled edge invalidates even when the epoch has not
  // moved yet.
  State disabled = TrustState(config_id, 12, 2, 4000);
  disabled.flags = kConfigured;
  ASSERT_TRUE(gateway.Deliver(disabled, 1008000));
  const uint64_t sent_after_disable = node.commands_sent();
  node.Tick(1009000);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(node.commands_sent(), sent_after_disable);

  // Re-enable (enabled bit returns, epoch advances) needs fresh requests too.
  State recovered = TrustState(config_id, 12, 3, 5000);
  ASSERT_TRUE(gateway.Deliver(recovered, 1010000));
  const uint64_t sent_after_recovery = node.commands_sent();
  node.Tick(1011000);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_EQ(node.commands_sent(), sent_after_recovery);

  publish_valid(12, 3, 5000, 18.0, 33.0);
  node.Tick(1012000);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 18.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 33.0);
  EXPECT_EQ(node.commands_sent(), sent_after_recovery + 1);
  EXPECT_EQ(node.identity_rejected(), 0u);
  EXPECT_EQ(node.ownership_rejected(), 0u);
}

// A State is accepted only when it is newer than the last one accepted in its
// gateway boot, or opens a boot the bridge has not seen. A duplicate, a
// reordered sample (even one carrying an older epoch or enable edge) and a
// datagram delayed from a boot the gateway has since left are dropped before
// they reach identity, freshness, the cached requests, /hw/state or the config
// push. A genuine new boot still recovers.
TEST(HardwareNodeTrustTest, ReorderedAndRetiredBootStatesChangeNothing) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  InProcessGateway& gateway = bridge.gateway;
  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Subscriber<talos::hardware::Packet> state_sub{kStateTopic};
  const auto drain_configs = [&] { return ConfigPushes(gateway.TakeSent()); };
  const auto publish = [&](uint64_t boot, uint64_t epoch, uint64_t observed,
                           double demand) {
    Command d{};
    d.config_id = config_id;
    d.boot_id = boot;
    d.epoch = epoch;
    d.observed_time_us = observed;
    d.count = 1;
    d.motors[0] = {drive, Mode::kVelocity, 0, demand, 0.0};
    PublishCommand(drive_pub, d);
    Command r = d;
    r.motors[0] = {roller, Mode::kVelocity, 0, demand, 0.0};
    PublishCommand(intake_pub, r);
  };

  // Boot 10 ran for a long time; boot 11 restarts the gateway clock low.
  ASSERT_TRUE(
      gateway.Deliver(TrustState(config_id, 10, 1, 900'000'000), 1000000));
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 2, 1000), 1001000));
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 2, 2000), 1002000));
  // Configured, so nothing but a transition pushes the configuration again.
  ConfigAckPayload ack{};
  ack.config_id = config_id;
  ack.status = ConfigAckStatus::kOk;
  std::array<uint8_t, 128> ack_bytes{};
  const auto ack_size = EncodeConfigAck(ack, ack_bytes);
  ASSERT_GT(ack_size, 0u);
  gateway.Receive(protocol::FrameType::kHardwareConfigAck, 1002000,
                  std::span<const uint8_t>{ack_bytes.data(), ack_size});
  node.Tick(1002000);
  ASSERT_TRUE(node.is_configured());
  publish(11, 2, 2000, 15.0);
  node.Tick(1002500);
  ASSERT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  ASSERT_FALSE(node.is_subsystem_timed_out("intake"));
  ASSERT_DOUBLE_EQ(node.current_command().motors[0].demand, 15.0);
  while (state_sub.read()) {
  }
  drain_configs();
  const uint64_t ignored_before = node.Status(1002500).clock_ignored_count();

  State duplicate_new_epoch = TrustState(config_id, 11, 3, 2000);
  State older_disabled = TrustState(config_id, 11, 1, 1500);
  older_disabled.flags = kConfigured;
  State retired = TrustState(config_id, 10, 1, 900'005'000);
  uint64_t now_us = 1003000;
  for (const State& stale : {duplicate_new_epoch, older_disabled, retired}) {
    ASSERT_TRUE(gateway.Deliver(stale, now_us));
    EXPECT_FALSE(state_sub.read()) << "boot " << stale.boot_id << " sample "
                                   << stale.sample_time_us << " published";
    EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
    EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
    EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 15.0);
    EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 15.0);
    EXPECT_TRUE(node.is_configured());
    EXPECT_EQ(drain_configs(), 0u);
    now_us += 500;
  }
  const bridge::BridgeStatus status = node.Status(now_us);
  EXPECT_EQ(status.clock_ignored_count(), ignored_before + 3);
  EXPECT_EQ(status.clock_boot_id(), 11u);
  // Freshness was not refreshed by the dropped frames.
  EXPECT_EQ(status.state_age_us(), now_us - 1002000);
  // The identity still is boot 11, epoch 2, sample 2000: a request built from
  // it merges, and one built from a dropped frame's words does not.
  const uint64_t rejected_before = node.identity_rejected();
  publish(11, 2, 2000, 16.0);
  node.Tick(now_us);
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 16.0);
  EXPECT_EQ(node.identity_rejected(), rejected_before);
  publish(11, 3, 2000, 17.0);
  node.Tick(now_us + 100);
  EXPECT_EQ(node.identity_rejected(), rejected_before + 2);
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 16.0);

  // A genuine reboot: a boot never seen before is accepted, published, pushes
  // the configuration and invalidates every cached request.
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 12, 1, 500), 1010000));
  EXPECT_TRUE(state_sub.read());
  EXPECT_EQ(drain_configs(), 1u);
  node.Tick(1010100);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(node.Status(1010100).clock_boot_id(), 12u);
  publish(12, 1, 500, 18.0);
  node.Tick(1010200);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 18.0);
}

// Driver Station packets carry the gateway boot id. The bridge forwards only
// the accepted State's boot, so a low-uptime boot B's packets go through
// right after the reboot, and a boot-A packet delayed past it, or one from a
// boot whose State has not arrived, does not.
// REQ-SAFE-11: only Driver Station packets of the accepted boot are forwarded
TEST(HardwareNodeTrustTest,
     ForwardsDriverStationPacketsOnlyFromTheCurrentBoot) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint64_t config_id = NodeConfigId(cfg);
  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  InProcessGateway& gateway = bridge.gateway;
  ipc::Subscriber<talos::hardware::Packet> ds_sub{kDriverStationTopic};

  // Hands the node one packet, ticks, and returns the boot id of what it
  // forwarded (nullopt: nothing).
  const auto send = [&](uint64_t boot_id, uint64_t sample_time_us,
                        uint64_t now_us) -> std::optional<uint64_t> {
    driver_station::DriverStationData data{};
    data.boot_id = boot_id;
    data.sample_time_us = sample_time_us;
    data.mode_since_us = sample_time_us;
    data.fms.flags = driver_station::kDsAttached;
    std::array<uint8_t, driver_station::kMaxEncodedSize> payload{};
    const auto size = driver_station::Encode(data, payload);
    EXPECT_GT(size, 0u);
    gateway.Receive(protocol::FrameType::kDriverStation, now_us,
                    std::span<const uint8_t>{payload.data(), size});
    node.Tick(now_us);
    const auto pkt = ds_sub.read();
    if (!pkt) return std::nullopt;
    driver_station::DriverStationData forwarded{};
    EXPECT_TRUE(driver_station::Decode(pkt->bytes(), forwarded));
    return forwarded.boot_id;
  };
  constexpr uint64_t kBootA = 0xA1, kBootB = 0xB1, kBootC = 0xC1;

  ASSERT_TRUE(
      gateway.Deliver(TrustState(config_id, kBootA, 1, 900'000'000), 1000000));
  EXPECT_EQ(send(kBootA, 900'000'000, 1001000), kBootA);
  // Boot C has sent no State: its packet is not forwarded yet.
  EXPECT_EQ(send(kBootC, 500, 1002000), std::nullopt);

  // Boot B: its State is accepted, so its low-uptime packets go through.
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, kBootB, 1, 1000), 1003000));
  EXPECT_EQ(send(kBootB, 1000, 1004000), kBootB);
  EXPECT_EQ(send(kBootB, 21000, 1005000), kBootB);
  // A's packet, delayed past the reboot, and B's own reordered one: dropped.
  EXPECT_EQ(send(kBootA, 900'020'000, 1006000), std::nullopt);
  EXPECT_EQ(send(kBootB, 11000, 1007000), std::nullopt);
  EXPECT_EQ(send(kBootB, 41000, 1008000), kBootB);
  EXPECT_EQ(node.driver_station_packets_received(), 4u);
}

// A timed-out subsystem neutralizes exactly its own motors, digital outputs
// and PWM channels; another subsystem's merged demand stays in place.
// REQ-SAFE-15
TEST(HardwareNodeTrustTest,
     TimeoutNeutralizesOnlyTheSilentSubsystemsActuators) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint16_t beacon = cfg.hardware.digital_outputs[0].id;
  const uint16_t strip = cfg.hardware.pwm_outputs[0].id;
  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  const uint64_t config_id = NodeConfigId(cfg);
  ASSERT_TRUE(
      bridge.gateway.Deliver(TrustState(config_id, 11, 1, 1000), 1000000));

  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  Command drive_ok{};
  drive_ok.config_id = config_id;
  drive_ok.boot_id = 11;
  drive_ok.epoch = 1;
  drive_ok.observed_time_us = 1000;
  drive_ok.count = 1;
  drive_ok.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  Command intake_ok = drive_ok;
  intake_ok.motors[0] = {roller, Mode::kDutyCycle, 0, 0.4, 0.0};
  intake_ok.digital_output_count = 1;
  intake_ok.digital_outputs[0] = {beacon, true};
  intake_ok.pwm_output_count = 1;
  intake_ok.pwm_outputs[0] = {strip, 0.5};
  PublishCommand(drive_pub, drive_ok);
  PublishCommand(intake_pub, intake_ok);
  node.Tick(1001000);

  const Command& merged = node.current_command();
  ASSERT_EQ(merged.count, 2u);
  EXPECT_EQ(merged.motors[0].id, drive);
  EXPECT_EQ(merged.motors[0].mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(merged.motors[0].demand, 15.0);
  EXPECT_EQ(merged.motors[1].id, roller);
  EXPECT_EQ(merged.motors[1].mode, Mode::kDutyCycle);
  EXPECT_DOUBLE_EQ(merged.motors[1].demand, 0.4);
  EXPECT_TRUE(merged.digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(merged.pwm_outputs[0].output, 0.5);

  // The intake goes quiet past its 50 ms owner timeout; the drivetrain keeps
  // publishing every 5 ms.
  for (uint64_t now_us = 1006000; now_us <= 1051000; now_us += 5000) {
    PublishCommand(drive_pub, drive_ok);
    node.Tick(now_us);
  }
  PublishCommand(drive_pub, drive_ok);
  node.Tick(1051001);

  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  const Command& neutral = node.current_command();
  EXPECT_EQ(neutral.motors[0].id, drive);
  EXPECT_EQ(neutral.motors[0].mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(neutral.motors[0].demand, 15.0);
  EXPECT_EQ(neutral.motors[1].id, roller);
  EXPECT_EQ(neutral.motors[1].mode, Mode::kNeutral);
  EXPECT_DOUBLE_EQ(neutral.motors[1].demand, 0.0);
  EXPECT_EQ(neutral.digital_outputs[0].id, beacon);
  EXPECT_FALSE(neutral.digital_outputs[0].value);
  EXPECT_EQ(neutral.pwm_outputs[0].id, strip);
  EXPECT_DOUBLE_EQ(neutral.pwm_outputs[0].output, 0.0);
  EXPECT_EQ(node.Status(1051001).timed_out_mask(), 0b10u);
  PublishCommand(intake_pub, intake_ok);
  node.Tick(1052000);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[1].mode, Mode::kDutyCycle);
  EXPECT_TRUE(node.current_command().digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(node.current_command().pwm_outputs[0].output, 0.5);
}

}  // namespace
}  // namespace talos::hardware
