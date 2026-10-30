#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "node.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"

namespace talos::hardware {
namespace {

constexpr std::string_view kTestToml = R"(
[robot]
name = "test_robot"
period_us = 5000

[subsystems.drivetrain]
period_us = 5000

[subsystems.drivetrain.motors.drive]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.shooter]
period_us = 10000

[subsystems.shooter.motors.flywheel]
type = "TalonFX"
bus = "rio"
can_id = 2

[subsystems.operator_interface]
period_us = 20000

[subsystems.beam_break]
period_us = 5000

[subsystems.beam_break.digital_inputs.gate]
dio = 3
)";

// The config id the node itself computes: simulation forces commissioned,
// which is part of the hash, so hashing the parsed config as-is mismatches.
uint64_t NodeConfigId(const talos::config::RobotConfig& cfg) {
  Config hw = cfg.hardware;
  hw.commissioned = true;
  return ConfigurationId(hw);
}

TEST(HardwareNodeTest, MergesSubsystemRequestsAndIsolatesTimeouts) {
  auto cfg = talos::config::ParseRobotConfigString(kTestToml);
  ASSERT_EQ(cfg.hardware.motors.size(), 2u);

  HardwareNode node{cfg, "127.0.0.1", 25802, 25803, true};
  ASSERT_TRUE(node.Open());

  // Named through RequestTopic() rather than spelled out: a test that hardcodes
  // the topic keeps passing after a rename while the robot stops working, which
  // is the exact failure this whole protocol exists to prevent.
  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<talos::hardware::Packet> shooter_pub{RequestTopic("shooter")};

  // Simulate gateway state arriving
  State fake_state{};
  fake_state.config_id = NodeConfigId(cfg);
  fake_state.boot_id = 999;
  fake_state.epoch = 1;
  fake_state.sample_time_us = 1000;
  fake_state.motor_count = 2;
  fake_state.motors[0].id = cfg.hardware.motors[0].id;
  fake_state.motors[1].id = cfg.hardware.motors[1].id;

  talos::hardware::Packet state_pkt{};
  state_pkt.size = static_cast<uint32_t>(Encode(fake_state, state_pkt.data));

  // Feed direct state packet
  ipc::Publisher<talos::hardware::Packet> hw_state_pub{kStateTopic};
  hw_state_pub.write(state_pkt);

  // 1. Both subsystems publish active commands. No controller state has arrived
  // over UDP yet, so the bridge's last-observed identity is (config, 0, 0, 0):
  // well-formed requests stamp exactly that.
  const uint64_t config_id = NodeConfigId(cfg);
  Command drive_cmd{};
  drive_cmd.config_id = config_id;
  drive_cmd.count = 1;
  drive_cmd.motors[0] = {cfg.hardware.motors[0].id, Mode::kVelocity, 0, 15.0,
                         1.2};
  talos::hardware::Packet drive_pkt{};
  drive_pkt.size = static_cast<uint32_t>(Encode(drive_cmd, drive_pkt.data));
  drive_pub.write(drive_pkt);

  Command shooter_cmd{};
  shooter_cmd.config_id = config_id;
  shooter_cmd.count = 1;
  shooter_cmd.motors[0] = {cfg.hardware.motors[1].id, Mode::kVelocity, 0, 50.0,
                           2.5};
  talos::hardware::Packet shooter_pkt{};
  shooter_pkt.size =
      static_cast<uint32_t>(Encode(shooter_cmd, shooter_pkt.data));
  shooter_pub.write(shooter_pkt);

  // Tick at now_us = 2000
  node.Tick(2000);

  // Both should be active and merged
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("shooter"));
  const auto& cmd1 = node.current_command();
  EXPECT_EQ(cmd1.count, 2u);
  EXPECT_EQ(cmd1.motors[0].id, cfg.hardware.motors[0].id);
  EXPECT_EQ(cmd1.motors[0].mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(cmd1.motors[0].demand, 15.0);

  EXPECT_EQ(cmd1.motors[1].id, cfg.hardware.motors[1].id);
  EXPECT_EQ(cmd1.motors[1].mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(cmd1.motors[1].demand, 50.0);

  // 2. Advance time past drivetrain timeout (2 * 5000us = 10000us)
  // Only shooter publishes
  shooter_pub.write(shooter_pkt);
  node.Tick(15000);  // 13000us since last drive command

  // Drivetrain is timed out, shooter is still active!
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("shooter"));
  const auto& cmd2 = node.current_command();
  // Drivetrain motor is neutralized!
  EXPECT_EQ(cmd2.motors[0].mode, Mode::kNeutral);
  EXPECT_DOUBLE_EQ(cmd2.motors[0].demand, 0.0);
  // Shooter motor remains active!
  EXPECT_EQ(cmd2.motors[1].mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(cmd2.motors[1].demand, 50.0);

  // 3. Drivetrain resumes publishing!
  drive_pub.write(drive_pkt);
  node.Tick(16000);

  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  const auto& cmd3 = node.current_command();
  EXPECT_EQ(cmd3.motors[0].mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(cmd3.motors[0].demand, 15.0);
}

TEST(HardwareNodeTest, ActuatorLessSubsystemsGetNoTracker) {
  auto cfg = talos::config::ParseRobotConfigString(kTestToml);
  ASSERT_TRUE(cfg.subsystems.contains("operator_interface"));
  ASSERT_TRUE(cfg.subsystems.contains("beam_break"));
  ASSERT_TRUE(cfg.subsystems.at("operator_interface").motors.empty());
  ASSERT_EQ(cfg.subsystems.at("beam_break").digital_inputs.size(), 1u);

  HardwareNode node{cfg, "127.0.0.1", 25806, 25807, true};
  ASSERT_TRUE(node.Open());

  // Nothing publishes any request, so every tracked subsystem times out.
  node.Tick(1000000);

  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("shooter"));
  EXPECT_FALSE(node.is_subsystem_timed_out("operator_interface"));
  EXPECT_FALSE(node.is_subsystem_timed_out("beam_break"));
}

TEST(HardwareNodeTest, ForwardsDriverStationPacketsToIpc) {
  auto cfg = talos::config::ParseRobotConfigString(kTestToml);
  HardwareNode node{cfg, "127.0.0.1", 25804, 25805, true};
  ASSERT_TRUE(node.Open());

  protocol::RuntimeUdpPeer gateway_peer;
  ASSERT_EQ(gateway_peer.Open("127.0.0.1", 25804, "127.0.0.1", 25805),
            protocol::UdpStatus::kOk);

  ipc::Subscriber<talos::hardware::Packet> ds_sub{kDriverStationTopic};

  std::vector<uint8_t> dummy_payload = {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc};
  ASSERT_EQ(gateway_peer.SendFrame(protocol::FrameType::kDriverStation, 1, 1000,
                                   dummy_payload.data(), dummy_payload.size()),
            protocol::UdpStatus::kOk);

  for (int i = 0; i < 20; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    node.Tick(2000 + i * 1000);
    if (node.driver_station_packets_received() > 0) break;
  }

  EXPECT_EQ(node.driver_station_packets_received(), 1u);
  auto pkt = ds_sub.read();
  ASSERT_TRUE(pkt.has_value());
  EXPECT_EQ(pkt->size, dummy_payload.size());
  EXPECT_EQ(std::memcmp(pkt->data.data(), dummy_payload.data(), pkt->size), 0);
}

constexpr std::string_view kTrustToml = R"(
[robot]
name = "trust_robot"
period_us = 5000

[subsystems.drivetrain]
period_us = 5000

[subsystems.drivetrain.motors.drive]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.intake]
period_us = 5000

[subsystems.intake.motors.roller]
type = "TalonFX"
bus = "rio"
can_id = 2

[subsystems.intake.digital_outputs.beacon]
dio = 1

[subsystems.intake.pwm_outputs.strip]
channel = 0
)";

void PublishCommand(ipc::Publisher<talos::hardware::Packet>& pub,
                    const Command& cmd) {
  talos::hardware::Packet pkt{};
  pkt.size = static_cast<uint32_t>(Encode(cmd, pkt.data));
  pub.write(pkt);
}

// Sends one controller state frame from a fake gateway, then ticks until the
// node ingests it. False means the state never arrived (a test failure, not a
// hang).
bool InjectState(HardwareNode& node, protocol::RuntimeUdpPeer& gateway,
                 const State& state, uint64_t now_us) {
  talos::hardware::Packet buf{};
  buf.size = static_cast<uint32_t>(Encode(state, buf.data));
  if (gateway.SendFrame(protocol::FrameType::kHardwareState, 1, now_us,
                        buf.data.data(),
                        buf.size) != protocol::UdpStatus::kOk) {
    return false;
  }
  const uint64_t before = node.states_received();
  for (int i = 0; i < 200; ++i) {
    node.Tick(now_us);
    if (node.states_received() > before) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

State TrustState(uint64_t config_id, uint64_t boot_id, uint64_t epoch,
                 uint64_t sample_time_us) {
  State state{};
  state.config_id = config_id;
  state.boot_id = boot_id;
  state.epoch = epoch;
  state.sample_time_us = sample_time_us;
  state.flags = kConfigured | kEnabled;
  return state;
}

TEST(HardwareNodeTest, RejectsRequestsForDevicesOwnedByAnotherSubsystem) {
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

  HardwareNode node{cfg, "127.0.0.1", 25812, 25813, true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);  // Drain any stale packet a previous test left behind.

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25812, "127.0.0.1", 25813),
            protocol::UdpStatus::kOk);
  const uint64_t config_id = NodeConfigId(cfg);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));

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

TEST(HardwareNodeTest, RejectsRequestsWithStaleIdentity) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", 25814, 25815, true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};

  // No controller state has arrived, so the current identity is
  // (config, 0, 0, 0). Each wrong word is rejected on its own.
  Command base{};
  base.config_id = config_id;
  base.count = 1;
  base.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};

  Command bad_config = base;
  bad_config.config_id = config_id + 1;
  PublishCommand(intake_pub, bad_config);
  node.Tick(1000000);

  Command bad_boot = base;
  bad_boot.boot_id = 7;
  PublishCommand(intake_pub, bad_boot);
  node.Tick(1001000);

  Command bad_epoch = base;
  bad_epoch.epoch = 2;
  PublishCommand(intake_pub, bad_epoch);
  node.Tick(1002000);

  Command bad_observed = base;
  bad_observed.observed_time_us = 5;
  PublishCommand(intake_pub, bad_observed);
  node.Tick(1003000);

  EXPECT_EQ(node.identity_rejected(), 4u);
  EXPECT_EQ(node.ownership_rejected(), 0u);
  // Rejections prove nothing: the subsystem is still timed out and its demand
  // never merged.
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[1].mode, Mode::kNeutral);

  PublishCommand(intake_pub, base);
  node.Tick(1004000);
  EXPECT_EQ(node.identity_rejected(), 4u);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
}

TEST(HardwareNodeTest, ObsoleteRequestsNeverLookCurrent) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);
  const uint64_t timeout_us = cfg.hardware.command_timeout_us;

  HardwareNode node{cfg, "127.0.0.1", 25816, 25817, true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25816, "127.0.0.1", 25817),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 5000), 1000000));

  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  Command fresh{};
  fresh.config_id = config_id;
  fresh.boot_id = 11;
  fresh.epoch = 1;
  fresh.observed_time_us = 5000;
  fresh.count = 1;
  fresh.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  PublishCommand(intake_pub, fresh);
  node.Tick(1001000);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);

  // A much newer sample arrives (same boot and epoch: not a transition, so
  // the cached merge survives). Replaying the request built from the older
  // sample is now beyond the age bound (last_sample - observed >=
  // command_timeout), so it is rejected, and the merge keeps the older
  // observation instead of being re-stamped: stale stays visibly stale.
  const uint64_t stale_sample = 5000 + timeout_us + 1000;
  ASSERT_TRUE(InjectState(node, gateway,
                          TrustState(config_id, 11, 1, stale_sample), 1002000));
  PublishCommand(intake_pub, fresh);
  node.Tick(1002500);

  EXPECT_EQ(node.identity_rejected(), 1u);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);
}

TEST(HardwareNodeTest, AcceptsRequestsWithinFreshnessBound) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", 25820, 25821, true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25820, "127.0.0.1", 25821),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 5000), 1000000));

  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  Command fresh{};
  fresh.config_id = config_id;
  fresh.boot_id = 11;
  fresh.epoch = 1;
  fresh.observed_time_us = 5000;
  fresh.count = 1;
  fresh.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  PublishCommand(intake_pub, fresh);
  node.Tick(1001000);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);

  // A newer sample arrives (same boot and epoch: not a transition). A request
  // computed from sample N is still acceptable after N+1 arrives while its
  // gateway-clock age (last_sample - observed) stays below the command
  // timeout. The merge keeps the older observation rather than re-stamping.
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 6000), 1002000));
  PublishCommand(intake_pub, fresh);
  node.Tick(1002500);

  EXPECT_EQ(node.identity_rejected(), 0u);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);

  // A request built from the newest sample advances the merge.
  Command newer = fresh;
  newer.observed_time_us = 6000;
  newer.motors[0].demand = 31.0;
  PublishCommand(intake_pub, newer);
  node.Tick(1003000);
  EXPECT_EQ(node.identity_rejected(), 0u);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  EXPECT_EQ(node.current_command().observed_time_us, 6000u);
}

TEST(HardwareNodeTest, RejectsReplayOlderThanLastAccepted) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", 25822, 25823, true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25822, "127.0.0.1", 25823),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 5000), 1000000));

  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  Command first{};
  first.config_id = config_id;
  first.boot_id = 11;
  first.epoch = 1;
  first.observed_time_us = 5000;
  first.count = 1;
  first.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  PublishCommand(intake_pub, first);
  node.Tick(1001000);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);

  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 6000), 1002000));
  Command second = first;
  second.observed_time_us = 6000;
  second.motors[0].demand = 31.0;
  PublishCommand(intake_pub, second);
  node.Tick(1002500);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  EXPECT_EQ(node.current_command().observed_time_us, 6000u);

  // Replaying the older demand is within the age bound (6000 - 5000) but
  // older than this subsystem's last accepted observation, so it is
  // rejected and the merge is untouched.
  PublishCommand(intake_pub, first);
  node.Tick(1003000);

  EXPECT_EQ(node.identity_rejected(), 1u);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  EXPECT_EQ(node.current_command().observed_time_us, 6000u);
}

TEST(HardwareNodeTest, FreshSubsystemCannotRefreshStaleSubsystem) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", 25824, 25825, true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25824, "127.0.0.1", 25825),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 5000), 1000000));

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
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 6000), 1002000));
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

TEST(HardwareNodeTest, RejectsWrongIdentityWordsEvenWithinBound) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", 25826, 25827, true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25826, "127.0.0.1", 25827),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 5000), 1000000));

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

TEST(HardwareNodeTest, RequiresFreshRequestsAfterControllerTransitions) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", 25818, 25819, true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25818, "127.0.0.1", 25819),
            protocol::UdpStatus::kOk);
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

  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));
  publish_valid(11, 1, 1000, 15.0, 30.0);
  node.Tick(1001000);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 15.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  // Single tick, single send. Later phases capture the counter instead: the
  // inject loop above may itself tick-and-send while polling for the frame.
  const uint64_t sent_after_first_merge = node.commands_sent();
  EXPECT_EQ(sent_after_first_merge, 1u);

  // Reboot: new boot id. Without fresh requests nothing merges and nothing is
  // sent; the cached demands are neutralized.
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 12, 1, 2000), 1002000));
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
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 12, 2, 3000), 1005000));
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
  ASSERT_TRUE(InjectState(node, gateway, disabled, 1008000));
  const uint64_t sent_after_disable = node.commands_sent();
  node.Tick(1009000);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(node.commands_sent(), sent_after_disable);

  // Fault recovery (fault bit clears, epoch advances) needs fresh requests too.
  State recovered = TrustState(config_id, 12, 3, 5000);
  ASSERT_TRUE(InjectState(node, gateway, recovered, 1010000));
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

// Describe() must answer without a socket, a shared-memory segment or a
// registry slot, because the launcher asks every binary in a config what it
// would connect to before it spawns any of them. Open() is never called here;
// if it becomes necessary, this test hangs on a port instead of passing.
TEST(HardwareNodeTest, DescribesItselfWithoutOpeningAnything) {
  auto cfg = talos::config::ParseRobotConfigString(kTestToml);
  HardwareNode node{cfg, "127.0.0.1", 25808, 25809, true};

  const auto description = node.Describe();
  EXPECT_EQ(description.name, kNodeName);
  EXPECT_EQ(description.target, kNodeTarget);

  // Every name the node declares has to satisfy the protocol, and the node has
  // to be findable from the target it claims. Checked here rather than only in
  // the launcher so a rename that breaks the grammar fails at this build.
  EXPECT_FALSE(introspect::naming::CheckNodeName(description.name).has_value());
  EXPECT_FALSE(
      introspect::naming::CheckNodeTarget(description.name, description.target)
          .has_value());

  std::map<std::string, introspect::naming::SourceShape> by_name;
  for (const auto& source : description.sources) {
    const auto problem = introspect::naming::CheckTopicName(source.name);
    EXPECT_FALSE(problem.has_value()) << (problem ? *problem : std::string{});
    EXPECT_EQ(source.message_bytes, source.name == bridge::topics::kStatus
                                        ? sizeof(bridge::BridgeStatus)
                                        : sizeof(Packet));
    by_name[source.name] = source;
  }

  // The four fixed ends, plus one request per actuator-owning subsystem. The
  // input-only subsystems in kTestToml contribute none.
  EXPECT_EQ(description.sources.size(), 6u);
  ASSERT_TRUE(by_name.contains(kStateTopic));
  ASSERT_TRUE(by_name.contains(kCommandTopic));
  ASSERT_TRUE(by_name.contains(kDriverStationTopic));
  ASSERT_TRUE(by_name.contains(bridge::topics::kStatus));
  ASSERT_TRUE(by_name.contains(RequestTopic("drivetrain")));
  ASSERT_TRUE(by_name.contains(RequestTopic("shooter")));
  EXPECT_FALSE(by_name.contains(RequestTopic("operator_interface")));
  EXPECT_FALSE(by_name.contains(RequestTopic("beam_break")));

  EXPECT_EQ(by_name[kStateTopic].kind, event::SourceKind::SENDER);
  EXPECT_EQ(by_name[kCommandTopic].kind, event::SourceKind::SENDER);
  EXPECT_EQ(by_name[kDriverStationTopic].kind, event::SourceKind::SENDER);
  EXPECT_EQ(by_name[bridge::topics::kStatus].kind, event::SourceKind::SENDER);
  EXPECT_FALSE(by_name[bridge::topics::kStatus].external());
  EXPECT_FALSE(by_name[bridge::topics::kStatus].optional());
  EXPECT_EQ(by_name[RequestTopic("drivetrain")].kind,
            event::SourceKind::FETCHER);

  // The attribute that keeps a working graph from reading as a broken one: the
  // command goes to the RoboRIO over UDP and will never have a shared-memory
  // subscriber.
  EXPECT_TRUE(by_name[kCommandTopic].external());
  EXPECT_FALSE(by_name[kCommandTopic].optional());
  EXPECT_FALSE(by_name[kStateTopic].external());
  EXPECT_FALSE(by_name[kStateTopic].optional());
  EXPECT_FALSE(by_name[RequestTopic("drivetrain")].external());

  // The JSON round trip, because the launcher reads what --describe printed
  // rather than the struct it came from.
  const std::string json = introspect::DescribeToJson(description);
  introspect::Description parsed;
  std::string error;
  ASSERT_TRUE(introspect::DescribeReader::Parse(json, parsed, error)) << error;
  EXPECT_EQ(parsed.name, description.name);
  ASSERT_EQ(parsed.sources.size(), description.sources.size());
}

// The alias is gone: the bridge reads the subsystem's own name and nothing
// else. A publisher on the old `/hw/req/drive` must not reach the merge, or the
// special case has merely moved.
TEST(HardwareNodeTest, IgnoresTheRetiredDriveAlias) {
  auto cfg = talos::config::ParseRobotConfigString(kTestToml);
  HardwareNode node{cfg, "127.0.0.1", 25810, 25811, true};
  ASSERT_TRUE(node.Open());

  ipc::Publisher<talos::hardware::Packet> alias_pub{"/hw/req/drive"};
  Command drive_cmd{};
  drive_cmd.count = 1;
  drive_cmd.motors[0] = {cfg.hardware.motors[0].id, Mode::kVelocity, 0, 15.0,
                         1.2};
  talos::hardware::Packet drive_pkt{};
  drive_pkt.size = static_cast<uint32_t>(Encode(drive_cmd, drive_pkt.data));
  alias_pub.write(drive_pkt);

  node.Tick(2000);

  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
}

// /hw/status carries the bridge's counters, the age of the last controller
// state on the bridge clock, which actuator subsystems have gone quiet and
// whether the RoboRIO accepted the configuration.
TEST(HardwareNodeTest, PublishesBridgeStatus) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  HardwareNode node{cfg, "127.0.0.1", 25828, 25829, true};
  ASSERT_TRUE(node.Open());
  ipc::Subscriber<bridge::BridgeStatus> status_sub{
      bridge::topics::kStatus,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};

  // The first tick publishes. Nothing has arrived from the controller, and
  // both actuator subsystems (drivetrain bit 0, intake bit 1: sorted name
  // order) are timed out.
  node.Tick(999000);
  const bridge::BridgeStatus initial = node.Status(999000);
  EXPECT_EQ(initial.states_received(), 0u);
  EXPECT_EQ(initial.state_age_us(), 0u);
  EXPECT_EQ(initial.subsystem_count(), 2u);
  EXPECT_EQ(initial.timed_out_mask(), 0b11u);
  EXPECT_FALSE(initial.configured());
  auto published = status_sub.read();
  ASSERT_TRUE(published.has_value());
  EXPECT_TRUE(*published == initial);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25828, "127.0.0.1", 25829),
            protocol::UdpStatus::kOk);
  const uint64_t config_id = NodeConfigId(cfg);

  // The RoboRIO acknowledges this configuration.
  ConfigAckPayload ack{};
  ack.config_id = config_id;
  std::array<uint8_t, sizeof(ConfigAckPayload)> ack_bytes{};
  ASSERT_EQ(EncodeConfigAck(ack, ack_bytes), sizeof(ConfigAckPayload));
  ASSERT_EQ(gateway.SendFrame(protocol::FrameType::kHardwareConfigAck, 1,
                              999500, ack_bytes.data(), ack_bytes.size()),
            protocol::UdpStatus::kOk);
  for (int i = 0; i < 200 && !node.is_configured(); ++i) {
    node.Tick(999500);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(node.is_configured());
  EXPECT_TRUE(node.Status(999500).configured());

  // State age runs on the bridge clock from the frame's receipt.
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));
  EXPECT_EQ(node.Status(1000000).states_received(), 1u);
  EXPECT_EQ(node.Status(1000000).state_age_us(), 0u);
  EXPECT_EQ(node.Status(1003500).state_age_us(), 3500u);

  // Only the drivetrain publishes: the silent intake keeps its bit.
  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  Command drive_ok{};
  drive_ok.config_id = config_id;
  drive_ok.boot_id = 11;
  drive_ok.epoch = 1;
  drive_ok.observed_time_us = 1000;
  drive_ok.count = 1;
  drive_ok.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  PublishCommand(drive_pub, drive_ok);
  node.Tick(1001000);
  ASSERT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  ASSERT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.Status(1001000).timed_out_mask(), 0b10u);
  EXPECT_EQ(node.Status(1001000).commands_sent(), 1u);

  // A request from the wrong boot and one that addresses another subsystem's
  // motor each count once.
  const uint64_t identity_before = node.identity_rejected();
  const uint64_t ownership_before = node.ownership_rejected();
  Command stale = drive_ok;
  stale.boot_id = 10;
  PublishCommand(drive_pub, stale);
  node.Tick(1002000);
  Command cross{};
  cross.config_id = config_id;
  cross.boot_id = 11;
  cross.epoch = 1;
  cross.observed_time_us = 1000;
  cross.count = 2;
  cross.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  cross.motors[1] = {drive, Mode::kVelocity, 0, 99.0, 0.0};
  PublishCommand(intake_pub, cross);
  node.Tick(1003000);
  const bridge::BridgeStatus counted = node.Status(1003000);
  EXPECT_EQ(counted.identity_rejected(), identity_before + 1);
  EXPECT_EQ(counted.ownership_rejected(), ownership_before + 1);
  EXPECT_EQ(counted.identity_rejected(), node.identity_rejected());
  EXPECT_EQ(counted.ownership_rejected(), node.ownership_rejected());
  EXPECT_EQ(counted.driver_station_packets(),
            node.driver_station_packets_received());
  EXPECT_EQ(counted.state_age_us(), 3000u);
  EXPECT_EQ(counted.timed_out_mask(), 0b10u);

  // Every 20 ms of bridge time, not every tick: the last publication was the
  // first tick at 999000.
  EXPECT_FALSE(status_sub.read().has_value());
  node.Tick(1018999);
  EXPECT_FALSE(status_sub.read().has_value());
  node.Tick(1019000);
  published = status_sub.read();
  ASSERT_TRUE(published.has_value());
  EXPECT_TRUE(*published == node.Status(1019000));
  // By now the drivetrain has been quiet past twice its period as well.
  EXPECT_EQ(published->timed_out_mask(), 0b11u);
  EXPECT_EQ(published->state_age_us(), 19000u);
  EXPECT_TRUE(published->configured());
}

// A timed-out subsystem neutralizes exactly its own motors, digital outputs
// and PWM channels; another subsystem's merged demand stays in place.
TEST(HardwareNodeTest, TimeoutNeutralizesOnlyTheSilentSubsystemsActuators) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint16_t beacon = cfg.hardware.digital_outputs[0].id;
  const uint16_t strip = cfg.hardware.pwm_outputs[0].id;
  HardwareNode node{cfg, "127.0.0.1", 25830, 25831, true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25830, "127.0.0.1", 25831),
            protocol::UdpStatus::kOk);
  const uint64_t config_id = NodeConfigId(cfg);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));

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

  // The intake goes quiet past twice its 5 ms period; the drivetrain keeps
  // publishing.
  PublishCommand(drive_pub, drive_ok);
  node.Tick(1006000);
  PublishCommand(drive_pub, drive_ok);
  node.Tick(1011001);

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
  EXPECT_EQ(node.Status(1011001).timed_out_mask(), 0b10u);
}

}  // namespace
}  // namespace talos::hardware
