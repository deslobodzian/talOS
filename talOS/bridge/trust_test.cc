// The bridge's request trust at component tier (docs/TESTING.md §2): which
// requests merge (identity, age, ownership, transitions), what a request
// leaves out and what an owner timeout neutralizes, and which gateway frames
// reach the merge. Each HardwareNode is driven through an in-process gateway
// (in_process_gateway.h) on the synthetic clock Tick() takes: no socket, no
// wall clock. hardware_node_test.cc covers the UDP path into the same
// handler.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "talOS/bridge/in_process_gateway.h"
#include "talOS/bridge/merge.h"
#include "talOS/bridge/node.h"
#include "talOS/bridge/node_test_support.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/config_wire.h"
#include "talOS/hardware/gate.h"
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

// /hw/status's timed_out_mask as of `now_us`, checking that
// alert_warning_owner_timed_out, which reads the same rule, agrees.
uint32_t TimedOutMask(const HardwareNode& node, uint64_t now_us) {
  const bridge::BridgeStatus status = node.Status(now_us);
  EXPECT_EQ(status.alert_warning_owner_timed_out(),
            status.timed_out_mask() != 0)
      << "at " << now_us;
  return status.timed_out_mask();
}

// The gateway acknowledging configuration `config_id` at `now_us`, and the
// Tick that takes it in.
void Acknowledge(Bridge& rig, uint64_t config_id, uint64_t now_us) {
  ConfigAckPayload ack{};
  ack.config_id = config_id;
  ack.status = ConfigAckStatus::kOk;
  std::array<uint8_t, sizeof(ConfigAckPayload)> bytes{};
  ASSERT_EQ(EncodeConfigAck(ack, bytes), sizeof(ConfigAckPayload));
  rig.gateway.Receive(protocol::FrameType::kHardwareConfigAck, now_us, bytes);
  rig.node.Tick(now_us);
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

// REQ-SAFE-03, REQ-SAFE-19: omitted DIO, PWM and motors go neutral
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
  // Driven before it is omitted, so the neutral below is the omission's.
  EXPECT_EQ(node.current_command().motors[1].mode, Mode::kVoltage);
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
  Acknowledge(bridge, config_id, 999'500);
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

// The 2026 robot's mix of owner periods: one 5 ms and one 20 ms actuator
// owner.
constexpr std::string_view kMixedPeriodsToml = R"(
[robot]
name = "rate_robot"
period_us = 5000

[subsystems.drivetrain]
period_us = 5000

[subsystems.drivetrain.motors.drive]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.intake]
period_us = 20000

[subsystems.intake.motors.roller]
type = "TalonFX"
bus = "rio"
can_id = 2
)";

// Ticks `node` as Run() does while no input arrives before `until_us`: at
// each duty NextDutyUs() names before it. `now_us` ends at the last one.
void TickDutiesBefore(HardwareNode& node, uint64_t& now_us, uint64_t until_us) {
  for (uint64_t due_us = node.NextDutyUs(now_us); due_us < until_us;
       due_us = node.NextDutyUs(now_us)) {
    now_us = due_us;
    node.Tick(now_us);
  }
}

// A disabled robot: every State says the gateway cannot take a command, so
// each owner flushes neutral requests (NeutralCommand, as the devices layer
// does), stamped with the latest State each one observed. Between
// transitions only the merged observation changes. The bridge sends one
// command per heartbeat (plus the first at once), whatever the owners'
// phases against the State cadence. Sending every change of the
// observation made the rate follow the phases: 51 commands a second when
// both owners observed the same State, 101 when a State landed between the
// drivetrain's tick and the intake's.
// REQ-RT-13
TEST(HardwareNodeTrustTest, NotReadyCommandRateIsOnePerHeartbeat) {
  auto cfg = config::ParseRobotConfigString(kMixedPeriodsToml);
  const uint64_t config_id = NodeConfigId(cfg);
  const std::vector<uint16_t>& drive = cfg.subsystems.at("drivetrain").motors;
  const std::vector<uint16_t>& roller = cfg.subsystems.at("intake").motors;
  constexpr uint64_t kStartUs = 1'000'000;
  // The gateway clock runs this far behind the bridge's.
  constexpr uint64_t kBehindUs = 900'000;
  constexpr uint64_t kStatePeriodUs = 5'000;
  constexpr uint64_t kWindowUs = 1'000'000;
  // Phases are offset from the States' instants by 500 us, so an owner's
  // tick never coincides with a State's receipt.
  for (uint64_t drive_phase_us = 500; drive_phase_us < 5'000;
       drive_phase_us += 1'000) {
    for (uint64_t intake_phase_us = 500; intake_phase_us < 20'000;
         intake_phase_us += 1'000) {
      SCOPED_TRACE(::testing::Message()
                   << "drivetrain phase " << drive_phase_us
                   << " us, intake phase " << intake_phase_us << " us");
      Bridge bridge{cfg};
      HardwareNode& node = bridge.node;
      ASSERT_EQ(node.heartbeat_us(), 20'000u);
      ipc::Publisher<Packet> drive_pub{RequestTopic("drivetrain")};
      ipc::Publisher<Packet> intake_pub{RequestTopic("intake")};
      State state = TrustState(config_id, 11, 1, 0);
      state.flags = kConfigured;  // disabled
      uint64_t now_us = kStartUs;
      uint64_t next_state_us = kStartUs;
      uint64_t next_drive_us = kStartUs + drive_phase_us;
      uint64_t next_intake_us = kStartUs + intake_phase_us;
      const uint64_t end_us = kStartUs + 20'000 + kWindowUs;
      for (;;) {
        const uint64_t event_us =
            std::min({next_state_us, next_drive_us, next_intake_us});
        if (event_us > end_us) break;
        TickDutiesBefore(node, now_us, event_us);
        now_us = event_us;
        if (event_us == next_state_us) {
          state.sample_time_us = now_us - kBehindUs;
          ASSERT_TRUE(bridge.gateway.Deliver(state, now_us));
          next_state_us += kStatePeriodUs;
        }
        bool published = false;
        if (event_us == next_drive_us) {
          PublishCommand(drive_pub, NeutralCommand(state, drive));
          next_drive_us += 5'000;
          published = true;
        }
        if (event_us == next_intake_us) {
          PublishCommand(intake_pub, NeutralCommand(state, roller));
          next_intake_us += 20'000;
          published = true;
        }
        if (published) node.Tick(now_us);
      }
      ASSERT_EQ(node.identity_rejected(), 0u);

      std::vector<uint64_t> sent_us;
      for (const auto& frame : bridge.gateway.TakeSent()) {
        if (frame.type == protocol::FrameType::kHardwareCommand) {
          sent_us.push_back(frame.sent_us);
        }
      }
      ASSERT_FALSE(sent_us.empty());
      const uint64_t first_us = sent_us.front();
      const auto in_window = std::count_if(
          sent_us.begin(), sent_us.end(),
          [first_us](uint64_t us) { return us <= first_us + kWindowUs; });
      const auto expected =
          static_cast<std::ptrdiff_t>(1 + (kWindowUs / node.heartbeat_us()));
      EXPECT_GE(in_window, expected - 1) << "in the second from " << first_us;
      EXPECT_LE(in_window, expected + 1) << "in the second from " << first_us;
    }
  }
}

// NextDutyUsIsWhenTickNextHasWork (hardware_node_test.cc) on a disabled
// robot: a request that changes only the merged observation does not send,
// and NextDutyUs() names the heartbeat that carries it rather than now.
// Ticking just before each duty does nothing, and each duty it names has
// work (a frame to the gateway or a /hw/status), so the loop neither sends
// early nor wakes every millisecond for a send Tick will not make.
// REQ-RT-13
TEST(HardwareNodeTrustTest, NotReadyNextDutyUsIsTheHeartbeat) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const std::vector<uint16_t>& roller = cfg.subsystems.at("intake").motors;
  Bridge rig{cfg};
  HardwareNode& node = rig.node;
  InProcessGateway& gateway = rig.gateway;
  ipc::Subscriber<bridge::BridgeStatus> statuses{
      bridge::topics::kStatus,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  constexpr uint64_t kStartUs = 1'000'000;
  constexpr uint64_t kRequestUs = kStartUs + 7'000;
  const uint64_t heartbeat_us = node.heartbeat_us();
  ASSERT_EQ(heartbeat_us, 20'000u);

  State state = TrustState(NodeConfigId(cfg), 11, 1, 1000);
  state.flags = kConfigured;  // disabled
  ASSERT_TRUE(gateway.Deliver(state, kStartUs));
  PublishCommand(pub, NeutralCommand(state, roller));
  node.Tick(kRequestUs);
  ASSERT_EQ(node.commands_sent(), 1u);
  state.sample_time_us = 2000;
  ASSERT_TRUE(gateway.Deliver(state, kStartUs + 8'000));
  PublishCommand(pub, NeutralCommand(state, roller));
  uint64_t now_us = kStartUs + 9'000;
  node.Tick(now_us);
  EXPECT_EQ(node.commands_sent(), 1u) << "an observation alone sent at once";
  gateway.TakeSent();
  while (statuses.read()) {
  }

  // Every duty before the intake's owner timeout (at kStartUs + 59'001,
  // whose Tick changes only the bridge's own merge).
  std::vector<SentCommand> commands;
  for (uint64_t due_us = node.NextDutyUs(now_us); due_us < kStartUs + 50'000;
       due_us = node.NextDutyUs(now_us)) {
    ASSERT_GT(due_us, now_us) << "nothing is pending at " << now_us;
    if (due_us - 1 > now_us) {
      node.Tick(due_us - 1);
      EXPECT_TRUE(gateway.TakeSent().empty()) << "early, at " << due_us - 1;
      EXPECT_FALSE(statuses.read()) << "early, at " << due_us - 1;
    }
    now_us = due_us;
    node.Tick(now_us);
    const auto frames = gateway.TakeSent();
    const bool status = statuses.read().has_value();
    EXPECT_TRUE(!frames.empty() || status) << "no work at " << now_us;
    for (const auto& command : Commands(frames)) {
      commands.push_back({now_us, command});
    }
  }

  ASSERT_EQ(commands.size(), 2u);
  for (std::size_t i = 0; i < commands.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(commands[i].sent_us, kRequestUs + ((i + 1) * heartbeat_us));
    EXPECT_EQ(commands[i].command.observed_time_us, 2000u);
  }
}

// While the gateway cannot take a command, a change of a demand still sends
// at once; only a change of the observation alone waits for the heartbeat.
// The devices layer flushes neutral then, so the change comes from a request
// published as is, as a node outside the devices layer could: the bridge
// forwards a demand whatever the State says and the gateway refuses it while
// disabled, but the neutral that replaces it must not wait a heartbeat. The
// duties NextDutyUs() names afterwards count the heartbeat from that send.
// REQ-RT-13
TEST(HardwareNodeTrustTest, NotReadyDemandChangeSendsAtOnce) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const std::vector<uint16_t>& roller = cfg.subsystems.at("intake").motors;
  Bridge rig{cfg};
  HardwareNode& node = rig.node;
  InProcessGateway& gateway = rig.gateway;
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  constexpr uint64_t kStartUs = 1'000'000;
  const uint64_t heartbeat_us = node.heartbeat_us();
  ASSERT_EQ(heartbeat_us, 20'000u);

  State disabled = TrustState(NodeConfigId(cfg), 11, 1, 1000);
  disabled.flags = kConfigured;
  ASSERT_TRUE(gateway.Deliver(disabled, kStartUs));
  const Command neutral = NeutralCommand(disabled, roller);
  PublishCommand(pub, neutral);
  node.Tick(kStartUs + 7'000);
  ASSERT_EQ(node.commands_sent(), 1u);
  gateway.TakeSent();

  // The same observation, each time: only the demand changes.
  Command driven = neutral;
  driven.motors[0].mode = Mode::kVoltage;
  driven.motors[0].demand = 4.0;
  std::vector<SentCommand> commands;
  const auto tick = [&](uint64_t now_us) {
    node.Tick(now_us);
    for (const auto& command : Commands(gateway.TakeSent())) {
      commands.push_back({now_us, command});
    }
  };
  constexpr uint64_t kDrivenUs = kStartUs + 9'000;
  constexpr uint64_t kNeutralUs = kStartUs + 10'000;
  PublishCommand(pub, driven);
  tick(kDrivenUs);
  PublishCommand(pub, neutral);
  tick(kNeutralUs);

  // Up to just before the intake's owner timeout at kNeutralUs + 50'001.
  uint64_t now_us = kNeutralUs;
  for (uint64_t due_us = node.NextDutyUs(now_us); due_us < kStartUs + 55'000;
       due_us = node.NextDutyUs(now_us)) {
    ASSERT_GT(due_us, now_us) << "nothing is pending at " << now_us;
    now_us = due_us;
    tick(now_us);
  }

  const std::array<uint64_t, 4> expected_us = {kDrivenUs, kNeutralUs,
                                               kNeutralUs + heartbeat_us,
                                               kNeutralUs + (2 * heartbeat_us)};
  ASSERT_EQ(commands.size(), expected_us.size());
  for (std::size_t i = 0; i < commands.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(commands[i].sent_us, expected_us[i]);
    EXPECT_EQ(MotorOf(commands[i].command, roller[0]).mode,
              i == 0 ? Mode::kVoltage : Mode::kNeutral);
    EXPECT_EQ(commands[i].command.observed_time_us, 1000u);
  }
}

// While the gateway can take a command, a request that changes only the
// merged observation sends at once: the gateway ages every output from that
// observation, and holding it for a heartbeat would spend up to 20 ms of the
// silent-owner tolerance ARCHITECTURE.md "The tick" states.
// REQ-RT-13
TEST(HardwareNodeTrustTest, ReadyObservationChangeSendsAtOnce) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  InProcessGateway& gateway = bridge.gateway;
  const uint64_t config_id = NodeConfigId(cfg);
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 1, 1000), 1'000'000));
  PublishCommand(pub, IntakeRequest(cfg, 4.0));
  node.Tick(1'001'000);
  ASSERT_EQ(node.commands_sent(), 1u);
  gateway.TakeSent();

  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 1, 2000), 1'002'000));
  Command request = IntakeRequest(cfg, 4.0);
  request.observed_time_us = 2000;
  PublishCommand(pub, request);
  node.Tick(1'003'000);
  EXPECT_EQ(node.commands_sent(), 2u);
  const auto commands = Commands(gateway.TakeSent());
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].observed_time_us, 2000u);
}

// A controller transition resets every owner in the merge until its next
// request, which must keep dropping their demands; /hw/status must not
// report it as an owner timeout. Seen: a timed_out_mask bit for one status
// at the disabled-to-enabled edge in 3 to 5 of 10 sim runs, which Studio's
// link-timeout-mask detector warns on (TODO.md RT-12). An owner that does go
// silent sets its bit one microsecond past its owner timeout, clears it with
// its next accepted request, and counts one owner-timeout episode however
// many ticks pass in it.
// REQ-OPS-20
TEST(HardwareNodeTrustTest,
     EnableEdgeSetsNoTimedOutBitAndASilentOwnerCountsOneTimeout) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint64_t config_id = NodeConfigId(cfg);
  const std::vector<uint16_t>& drive = cfg.subsystems.at("drivetrain").motors;
  const std::vector<uint16_t>& roller = cfg.subsystems.at("intake").motors;
  const uint64_t owner_timeout_us =
      OwnerTimeoutUs(5000, cfg.hardware.command_timeout_us);
  Bridge rig{cfg};
  HardwareNode& node = rig.node;
  ipc::Publisher<Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<Packet> intake_pub{RequestTopic("intake")};
  constexpr uint64_t kStartUs = 1'000'000;

  // Disabled, both owners flushing neutral every 5 ms.
  State disabled = TrustState(config_id, 11, 1, 1000);
  disabled.flags = kConfigured;
  Acknowledge(rig, config_id, kStartUs - 500);
  ASSERT_TRUE(rig.gateway.Deliver(disabled, kStartUs));
  for (uint64_t now_us = kStartUs + 1000; now_us < kStartUs + 20'000;
       now_us += 5000) {
    PublishCommand(drive_pub, NeutralCommand(disabled, drive));
    PublishCommand(intake_pub, NeutralCommand(disabled, roller));
    node.Tick(now_us);
  }
  ASSERT_EQ(node.Status(kStartUs + 19'000).timed_out_mask(), 0u);

  // The enable edge: a new epoch, which times both owners out in the merge.
  constexpr uint64_t kEnableUs = kStartUs + 20'000;
  const State enabled = TrustState(config_id, 11, 2, 20'000);
  ASSERT_TRUE(rig.gateway.Deliver(enabled, kEnableUs));
  ASSERT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  ASSERT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(TimedOutMask(node, kEnableUs), 0u);

  // Both owners' first requests of the new epoch, then the intake falls
  // silent while the drivetrain keeps its period.
  constexpr uint64_t kLastIntakeUs = kEnableUs + 1000;
  PublishCommand(drive_pub, NeutralCommand(enabled, drive));
  PublishCommand(intake_pub, NeutralCommand(enabled, roller));
  node.Tick(kLastIntakeUs);
  EXPECT_EQ(node.Status(kLastIntakeUs).timed_out_mask(), 0u);
  uint64_t now_us = kLastIntakeUs;
  while (now_us < kLastIntakeUs + owner_timeout_us + 30'000) {
    now_us += 5000;
    PublishCommand(drive_pub, NeutralCommand(enabled, drive));
    node.Tick(now_us);
  }
  EXPECT_EQ(TimedOutMask(node, kLastIntakeUs + owner_timeout_us), 0u);
  EXPECT_EQ(TimedOutMask(node, kLastIntakeUs + owner_timeout_us + 1), 0b10u);
  EXPECT_EQ(node.owner_timeouts("intake"), 1u);
  EXPECT_EQ(node.owner_timeouts("drivetrain"), 0u);

  // A transition while the intake is silent keeps its bit: no transition
  // hides an owner already silent.
  const State next = TrustState(config_id, 11, 3, 30'000);
  now_us += 1000;
  ASSERT_TRUE(rig.gateway.Deliver(next, now_us));
  EXPECT_EQ(TimedOutMask(node, now_us), 0b10u);

  // The intake's next request clears its bit; the episode stays counted.
  now_us += 4000;
  PublishCommand(drive_pub, NeutralCommand(next, drive));
  PublishCommand(intake_pub, NeutralCommand(next, roller));
  node.Tick(now_us);
  EXPECT_EQ(node.Status(now_us).timed_out_mask(), 0u);
  EXPECT_EQ(node.owner_timeouts("intake"), 1u);
  EXPECT_EQ(node.identity_rejected(), 0u);
}

// Two actuator owners at the longest period config load allows, a quarter
// of the command timeout, so each owner timeout is exactly two periods.
constexpr std::string_view kQuarterTimeoutToml = R"(
[robot]
name = "quarter_robot"
period_us = 5000

[subsystems.drivetrain]
period_us = 25000

[subsystems.drivetrain.motors.drive]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.intake]
period_us = 25000

[subsystems.intake.motors.roller]
type = "TalonFX"
bus = "rio"
can_id = 2
)";

// A new-epoch State lands just after both owners drained for their next
// tick, so the requests they flush in it carry the old epoch and are
// refused, and each owner's next accepted request comes two periods (its
// whole owner timeout) after its last, plus any jitter. Measured from the
// last accepted request, that raised alert_warning_owner_timed_out (and a
// timed_out_mask bit) for one status at any jitter, and the dashboard
// latches an alert for the match. Silence counts from the transition
// instead; an owner that stays silent for its owner timeout after the
// transition still raises, at exactly that boundary.
// REQ-OPS-20
TEST(HardwareNodeTrustTest, TransitionAtTwoPeriodsOfSilenceRaisesNoTimeout) {
  auto cfg = config::ParseRobotConfigString(kQuarterTimeoutToml);
  const uint64_t config_id = NodeConfigId(cfg);
  constexpr uint64_t kPeriodUs = 25'000;
  ASSERT_EQ(kPeriodUs * 4, cfg.hardware.command_timeout_us);
  const uint64_t owner_timeout_us =
      OwnerTimeoutUs(kPeriodUs, cfg.hardware.command_timeout_us);
  ASSERT_EQ(owner_timeout_us, 2 * kPeriodUs);
  const std::vector<uint16_t>& drive = cfg.subsystems.at("drivetrain").motors;
  const std::vector<uint16_t>& roller = cfg.subsystems.at("intake").motors;
  Bridge rig{cfg};
  HardwareNode& node = rig.node;
  ipc::Publisher<Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<Packet> intake_pub{RequestTopic("intake")};
  constexpr uint64_t kStartUs = 1'000'000;

  const State first = TrustState(config_id, 11, 1, 1000);
  Acknowledge(rig, config_id, kStartUs - 500);
  ASSERT_TRUE(rig.gateway.Deliver(first, kStartUs));
  constexpr uint64_t kAcceptedUs = kStartUs + 1000;
  PublishCommand(drive_pub, NeutralCommand(first, drive));
  PublishCommand(intake_pub, NeutralCommand(first, roller));
  node.Tick(kAcceptedUs);
  ASSERT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  ASSERT_FALSE(node.is_subsystem_timed_out("intake"));

  // The new epoch, 1 us before both owners' next flush, which it refuses.
  constexpr uint64_t kTransitionUs = kAcceptedUs + kPeriodUs - 1;
  const State second = TrustState(config_id, 11, 2, 1000 + kPeriodUs);
  ASSERT_TRUE(rig.gateway.Deliver(second, kTransitionUs));
  PublishCommand(drive_pub, NeutralCommand(first, drive));
  PublishCommand(intake_pub, NeutralCommand(first, roller));
  node.Tick(kAcceptedUs + kPeriodUs);
  ASSERT_EQ(node.identity_rejected(), 2u);

  // Two periods and one microsecond since each owner's last accepted
  // request: no timeout.
  EXPECT_EQ(TimedOutMask(node, kAcceptedUs + owner_timeout_us + 1), 0u);
  // The intake's next tick, a millisecond late, merges; the drivetrain's
  // node has stopped.
  constexpr uint64_t kIntakeBackUs = kAcceptedUs + (2 * kPeriodUs) + 1000;
  PublishCommand(intake_pub, NeutralCommand(second, roller));
  node.Tick(kIntakeBackUs);
  ASSERT_FALSE(node.is_subsystem_timed_out("intake"));
  // Another transition restarts the silence of the intake, heard since the
  // first, but not of the drivetrain: one per accepted request.
  const State third = TrustState(config_id, 11, 3, 2000 + kPeriodUs);
  ASSERT_TRUE(rig.gateway.Deliver(third, kIntakeBackUs + 1000));
  EXPECT_EQ(TimedOutMask(node, kTransitionUs + owner_timeout_us), 0u);
  EXPECT_EQ(TimedOutMask(node, kTransitionUs + owner_timeout_us + 1), 0b01u);
  node.Tick(kTransitionUs + owner_timeout_us + 1);
  EXPECT_EQ(node.owner_timeouts("drivetrain"), 1u);
  EXPECT_EQ(node.owner_timeouts("intake"), 0u);
  EXPECT_EQ(TimedOutMask(node, kIntakeBackUs + 1000 + owner_timeout_us + 1),
            0b11u);
}

// The session's start is no owner timeout: until a State is accepted and the
// gateway has acknowledged the configuration (alert_error_link_lost and
// alert_error_not_configured speak meanwhile), no owner is reported silent,
// and an owner this bridge has never accepted a request from is never
// reported, however long the link has been ready: the launcher reports a
// node that does not start (REQ-OPS-05). An owner once heard stays heard
// across a gateway reboot: silent through it, it is reported one owner
// timeout after the new boot acknowledges, with no request since, and counts
// an episode.
// REQ-OPS-20
TEST(HardwareNodeTrustTest,
     NeverHeardOwnerIsNotReportedAHeardOneIsAfterReboot) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint64_t config_id = NodeConfigId(cfg);
  const std::vector<uint16_t>& drive = cfg.subsystems.at("drivetrain").motors;
  const uint64_t owner_timeout_us =
      OwnerTimeoutUs(5000, cfg.hardware.command_timeout_us);
  Bridge rig{cfg};
  HardwareNode& node = rig.node;
  ipc::Publisher<Packet> drive_pub{RequestTopic("drivetrain")};
  constexpr uint64_t kStartUs = 1'000'000;
  // The drivetrain flushing every 5 ms on `state` from now_us up to
  // `to_us`; the intake's node never started.
  uint64_t now_us = kStartUs;
  const auto drive_until = [&](const State& state, uint64_t to_us) {
    for (; now_us + 5000 <= to_us;) {
      now_us += 5000;
      PublishCommand(drive_pub, NeutralCommand(state, drive));
      node.Tick(now_us);
      EXPECT_EQ(TimedOutMask(node, now_us), 0u);
    }
  };

  // No State, then a State of a boot that has not acknowledged.
  node.Tick(kStartUs);
  EXPECT_TRUE(node.Status(kStartUs).alert_error_link_lost());
  EXPECT_EQ(TimedOutMask(node, kStartUs + owner_timeout_us + 1), 0u);
  const State boot11 = TrustState(config_id, 11, 1, 1000);
  ASSERT_TRUE(rig.gateway.Deliver(boot11, kStartUs + 1000));
  now_us = kStartUs + 1000;
  drive_until(boot11, kStartUs + 100'000);
  EXPECT_TRUE(node.Status(now_us).alert_error_not_configured());

  // Acknowledged: the link is ready. Neither the drivetrain, heard every
  // 5 ms, nor the intake, never heard, is reported, for many owner timeouts.
  const uint64_t ready_us = now_us + 500;
  Acknowledge(rig, config_id, ready_us);
  drive_until(boot11, ready_us + (5 * owner_timeout_us));
  EXPECT_EQ(node.owner_timeouts("intake"), 0u);

  // The drivetrain's node stops, then the gateway reboots: nothing is
  // reported until boot 12 acknowledges.
  const State boot12 = TrustState(config_id, 12, 1, 500);
  now_us += 1000;
  ASSERT_TRUE(rig.gateway.Deliver(boot12, now_us));
  EXPECT_TRUE(node.Status(now_us).alert_error_not_configured());
  EXPECT_EQ(TimedOutMask(node, now_us), 0u);
  const uint64_t reready_us = now_us + 30'000;
  Acknowledge(rig, config_id, reready_us);
  // Once ready again, the drivetrain (bit 0), heard before the reboot, is
  // reported one owner timeout later; the intake still is not.
  EXPECT_EQ(TimedOutMask(node, reready_us + owner_timeout_us), 0u);
  EXPECT_EQ(TimedOutMask(node, reready_us + owner_timeout_us + 1), 0b01u);
  node.Tick(reready_us + owner_timeout_us + 1);
  EXPECT_EQ(node.owner_timeouts("drivetrain"), 1u);
  EXPECT_EQ(node.owner_timeouts("intake"), 0u);
  EXPECT_EQ(TimedOutMask(node, reready_us + (10 * owner_timeout_us)), 0b01u);
}

// A Python node's first request reached the bridge about 157 ms into every
// simulated session, three owner timeouts after the link became ready, and
// set its timed_out_mask bit and alert_warning_owner_timed_out at every
// start-up (11 of 11 sessions, macOS): the dashboard latches that alert for
// the match. An owner is not reported before the bridge has accepted a
// request from it, and a refused request (an obsolete identity, or a device
// another subsystem owns) is not one.
// REQ-OPS-20
TEST(HardwareNodeTrustTest, AnOwnerFirstHeardLateRaisesNoTimeout) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint64_t config_id = NodeConfigId(cfg);
  const std::vector<uint16_t>& drive = cfg.subsystems.at("drivetrain").motors;
  const std::vector<uint16_t>& roller = cfg.subsystems.at("intake").motors;
  const uint64_t owner_timeout_us =
      OwnerTimeoutUs(5000, cfg.hardware.command_timeout_us);
  Bridge rig{cfg};
  HardwareNode& node = rig.node;
  ipc::Publisher<Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<Packet> intake_pub{RequestTopic("intake")};
  constexpr uint64_t kReadyUs = 1'000'000;
  constexpr uint64_t kFirstIntakeUs = kReadyUs + 155'000;
  ASSERT_GT(kFirstIntakeUs - kReadyUs, 3 * owner_timeout_us);

  Acknowledge(rig, config_id, kReadyUs - 500);
  State state = TrustState(config_id, 11, 1, 1000);
  ASSERT_TRUE(rig.gateway.Deliver(state, kReadyUs));
  // A State every 5 ms; the drivetrain flushes on each from the start, the
  // intake sends one request of an obsolete epoch and one addressing the
  // drivetrain's motor, then its first good one at kFirstIntakeUs.
  for (uint64_t now_us = kReadyUs + 5000; now_us <= kFirstIntakeUs;
       now_us += 5000) {
    PublishCommand(drive_pub, NeutralCommand(state, drive));
    if (now_us == kReadyUs + 10'000) {
      Command obsolete = NeutralCommand(state, roller);
      obsolete.epoch = 0;
      PublishCommand(intake_pub, obsolete);
    } else if (now_us == kReadyUs + 20'000) {
      PublishCommand(intake_pub, NeutralCommand(state, drive));
    } else if (now_us == kFirstIntakeUs) {
      PublishCommand(intake_pub, NeutralCommand(state, roller));
    }
    state.sample_time_us += 5000;
    ASSERT_TRUE(rig.gateway.Deliver(state, now_us));
    EXPECT_EQ(TimedOutMask(node, now_us), 0u) << "at " << now_us;
  }
  ASSERT_EQ(node.identity_rejected(), 1u);
  ASSERT_EQ(node.ownership_rejected(), 1u);
  ASSERT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.owner_timeouts("intake"), 0u);
  EXPECT_EQ(node.owner_timeouts("drivetrain"), 0u);
}

// The alerts `status` raises, by name, in field order.
std::vector<std::string> Alerts(const bridge::BridgeStatus& status) {
  std::vector<std::string> active;
  if (status.alert_error_link_lost()) active.emplace_back("link_lost");
  if (status.alert_error_not_configured()) {
    active.emplace_back("not_configured");
  }
  if (status.alert_error_uncommissioned()) {
    active.emplace_back("uncommissioned");
  }
  if (status.alert_error_motor_fault()) active.emplace_back("motor_fault");
  if (status.alert_warning_owner_timed_out()) {
    active.emplace_back("owner_timed_out");
  }
  if (status.alert_warning_brownout()) active.emplace_back("brownout");
  return active;
}

// Each /hw/status alert is raised while its condition holds and cleared in
// the first status after it stops, at the condition's own boundary; a
// controller transition, which times every owner out in the merge until its
// next request, raises none, and before the link is ready only the link and
// configuration alerts speak.
// REQ-OPS-20, REQ-OPS-37
TEST(HardwareNodeTrustTest, StatusAlertsFollowTheirConditions) {
  using Names = std::vector<std::string>;
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint64_t timeout_us = cfg.hardware.command_timeout_us;
  const uint64_t owner_timeout_us = OwnerTimeoutUs(5000, timeout_us);
  ASSERT_LT(owner_timeout_us, timeout_us);

  // Not --sim, so the config's own commissioned (false) stands.
  const HardwareNode uncommissioned{cfg};
  EXPECT_FALSE(uncommissioned.Status(1000000).commissioned());
  EXPECT_EQ(Alerts(uncommissioned.Status(1000000)),
            (Names{"link_lost", "not_configured", "uncommissioned"}));

  Bridge bridge{cfg};
  HardwareNode& node = bridge.node;
  InProcessGateway& gateway = bridge.gateway;
  const uint64_t config_id = NodeConfigId(cfg);
  ipc::Publisher<Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<Packet> intake_pub{RequestTopic("intake")};
  // A request from `pub`'s subsystem built on the State of boot 11 at
  // `epoch` sampled at `observed_us`.
  const auto request = [&](ipc::Publisher<Packet>& pub, uint64_t epoch,
                           uint64_t observed_us) {
    Command command{};
    command.config_id = config_id;
    command.boot_id = 11;
    command.epoch = epoch;
    command.observed_time_us = observed_us;
    PublishCommand(pub, command);
  };
  const auto acknowledge = [&](uint64_t now_us) {
    ConfigAckPayload ack{};
    ack.config_id = config_id;
    std::array<uint8_t, sizeof(ConfigAckPayload)> bytes{};
    ASSERT_EQ(EncodeConfigAck(ack, bytes), sizeof(ConfigAckPayload));
    gateway.Receive(protocol::FrameType::kHardwareConfigAck, now_us, bytes);
  };

  // Before the link: no State, no acknowledgement, no request.
  node.Tick(1000000);
  EXPECT_TRUE(node.Status(1000000).commissioned());
  EXPECT_EQ(Alerts(node.Status(1000000)),
            (Names{"link_lost", "not_configured"}));

  // The link up, the configuration acknowledged, both owners merged.
  acknowledge(1000500);
  request(drive_pub, 1, 1000);
  request(intake_pub, 1, 1000);
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 1, 1000), 1001000));
  EXPECT_EQ(Alerts(node.Status(1001000)), Names{});

  // Both owners silent: the alert at their owner timeout's boundary.
  EXPECT_EQ(Alerts(node.Status(1001000 + owner_timeout_us)), Names{});
  EXPECT_EQ(Alerts(node.Status(1001000 + owner_timeout_us + 1)),
            Names{"owner_timed_out"});
  // One silent owner is enough; its next request clears it.
  request(drive_pub, 1, 60000);
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 1, 60000), 1060000));
  EXPECT_EQ(Alerts(node.Status(1060000)), Names{"owner_timed_out"});
  request(intake_pub, 1, 60000);
  node.Tick(1061000);
  EXPECT_EQ(Alerts(node.Status(1061000)), Names{});

  // A new epoch resets the merge, not the alert or timed_out_mask.
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 2, 62000), 1062000));
  ASSERT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_EQ(Alerts(node.Status(1062000)), Names{});

  // No State for longer than the command timeout, while both owners keep
  // publishing on the last one: the link alert alone, from its boundary.
  uint64_t now_us = 1062000;
  for (; now_us + 5000 <= 1062000 + timeout_us; now_us += 5000) {
    request(drive_pub, 2, 62000);
    request(intake_pub, 2, 62000);
    node.Tick(now_us + 5000);
  }
  EXPECT_EQ(Alerts(node.Status(1062000 + timeout_us)), Names{});
  EXPECT_EQ(Alerts(node.Status(1062000 + timeout_us + 1)), Names{"link_lost"});
  now_us = 1062000 + timeout_us + 2000;
  request(drive_pub, 2, 200000);
  request(intake_pub, 2, 200000);
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 2, 200000), now_us));
  EXPECT_EQ(Alerts(node.Status(now_us)), Names{});

  // A State reporting an invalid motor sample and a brownout, then one
  // reporting neither, then a brownout in a health sample that is not valid.
  State state = TrustState(config_id, 11, 2, 201000);
  state.motor_count = 1;
  state.motors[0].id = cfg.hardware.motors[0].id;
  state.motors[0].valid = false;
  state.health = {true, 6.5, true, 0.2};
  now_us += 1000;
  request(drive_pub, 2, 201000);
  request(intake_pub, 2, 201000);
  ASSERT_TRUE(gateway.Deliver(state, now_us));
  EXPECT_EQ(Alerts(node.Status(now_us)), (Names{"motor_fault", "brownout"}));
  state.sample_time_us = 202000;
  state.motors[0].valid = true;
  state.health = {true, 12.5, false, 0.2};
  now_us += 1000;
  request(drive_pub, 2, 202000);
  request(intake_pub, 2, 202000);
  ASSERT_TRUE(gateway.Deliver(state, now_us));
  EXPECT_EQ(Alerts(node.Status(now_us)), Names{});
  state.sample_time_us = 203000;
  state.health = {false, 0.0, true, 0.0};
  now_us += 1000;
  request(drive_pub, 2, 203000);
  request(intake_pub, 2, 203000);
  ASSERT_TRUE(gateway.Deliver(state, now_us));
  EXPECT_EQ(Alerts(node.Status(now_us)), Names{});

  // The gateway names another configuration, then ours again, which only
  // its acknowledgement restores.
  now_us += 1000;
  ASSERT_TRUE(
      gateway.Deliver(TrustState(config_id + 1, 11, 3, 204000), now_us));
  EXPECT_EQ(Alerts(node.Status(now_us)), Names{"not_configured"});
  now_us += 1000;
  ASSERT_TRUE(gateway.Deliver(TrustState(config_id, 11, 4, 205000), now_us));
  EXPECT_EQ(Alerts(node.Status(now_us)), Names{"not_configured"});
  acknowledge(now_us);
  EXPECT_EQ(Alerts(node.Status(now_us)), Names{});
}

}  // namespace
}  // namespace talos::hardware
