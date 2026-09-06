#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

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

[subsystems.driver_station]
node = "//2026-robot/main_processor/driver_station:node"
period_us = 20000

[subsystems.beam_break]
period_us = 5000

[subsystems.beam_break.digital_inputs.gate]
dio = 3
)";

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
  fake_state.config_id = ConfigurationId(cfg.hardware);
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

  // 1. Both subsystems publish active commands
  Command drive_cmd{};
  drive_cmd.count = 1;
  drive_cmd.motors[0] = {cfg.hardware.motors[0].id, Mode::kVelocity, 0, 15.0,
                         1.2};
  talos::hardware::Packet drive_pkt{};
  drive_pkt.size = static_cast<uint32_t>(Encode(drive_cmd, drive_pkt.data));
  drive_pub.write(drive_pkt);

  Command shooter_cmd{};
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
  ASSERT_TRUE(cfg.subsystems.contains("driver_station"));
  ASSERT_TRUE(cfg.subsystems.contains("beam_break"));
  ASSERT_TRUE(cfg.subsystems.at("driver_station").motors.empty());
  ASSERT_EQ(cfg.subsystems.at("beam_break").digital_inputs.size(), 1u);

  HardwareNode node{cfg, "127.0.0.1", 25806, 25807, true};
  ASSERT_TRUE(node.Open());

  // Nothing publishes any request, so every tracked subsystem times out.
  node.Tick(1000000);

  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("shooter"));
  EXPECT_FALSE(node.is_subsystem_timed_out("driver_station"));
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
    EXPECT_EQ(source.message_bytes, sizeof(Packet));
    by_name[source.name] = source;
  }

  // The four fixed ends, plus one request per actuator-owning subsystem. The
  // input-only subsystems in kTestToml contribute none.
  EXPECT_EQ(description.sources.size(), 6u);
  ASSERT_TRUE(by_name.contains(kStateTopic));
  ASSERT_TRUE(by_name.contains(kCommandTopic));
  ASSERT_TRUE(by_name.contains(kDriverStationTopic));
  ASSERT_TRUE(by_name.contains(kCommandOverrideTopic));
  ASSERT_TRUE(by_name.contains(RequestTopic("drivetrain")));
  ASSERT_TRUE(by_name.contains(RequestTopic("shooter")));
  EXPECT_FALSE(by_name.contains(RequestTopic("driver_station")));
  EXPECT_FALSE(by_name.contains(RequestTopic("beam_break")));

  EXPECT_EQ(by_name[kStateTopic].kind, event::SourceKind::SENDER);
  EXPECT_EQ(by_name[kCommandTopic].kind, event::SourceKind::SENDER);
  EXPECT_EQ(by_name[kDriverStationTopic].kind, event::SourceKind::SENDER);
  EXPECT_EQ(by_name[kCommandOverrideTopic].kind, event::SourceKind::FETCHER);
  EXPECT_EQ(by_name[RequestTopic("drivetrain")].kind,
            event::SourceKind::FETCHER);

  // The two attributes that keep a working graph from reading as a broken one:
  // the command goes to the RoboRIO over UDP and will never have a
  // shared-memory subscriber, and nothing in the tree publishes the override.
  EXPECT_TRUE(by_name[kCommandTopic].external());
  EXPECT_TRUE(by_name[kCommandOverrideTopic].optional());
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

}  // namespace
}  // namespace talos::hardware
