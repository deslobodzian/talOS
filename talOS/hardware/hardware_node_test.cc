#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include "node.h"

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
node = "//talOS/driver_station:node"
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

  ipc::Publisher<talos::drive::Packet> drive_pub{"/hw/req/drivetrain"};
  ipc::Publisher<talos::drive::Packet> shooter_pub{"/hw/req/shooter"};

  // Simulate gateway state arriving
  State fake_state{};
  fake_state.config_id = ConfigurationId(cfg.hardware);
  fake_state.boot_id = 999;
  fake_state.epoch = 1;
  fake_state.sample_time_us = 1000;
  fake_state.motor_count = 2;
  fake_state.motors[0].id = cfg.hardware.motors[0].id;
  fake_state.motors[1].id = cfg.hardware.motors[1].id;

  talos::drive::Packet state_pkt{};
  state_pkt.size = static_cast<uint32_t>(Encode(fake_state, state_pkt.data));

  // Feed direct state packet
  ipc::Publisher<talos::drive::Packet> hw_state_pub{"/hw/state"};
  hw_state_pub.write(state_pkt);

  // 1. Both subsystems publish active commands
  Command drive_cmd{};
  drive_cmd.count = 1;
  drive_cmd.motors[0] = {cfg.hardware.motors[0].id, Mode::kVelocity, 0, 15.0, 1.2};
  talos::drive::Packet drive_pkt{};
  drive_pkt.size = static_cast<uint32_t>(Encode(drive_cmd, drive_pkt.data));
  drive_pub.write(drive_pkt);

  Command shooter_cmd{};
  shooter_cmd.count = 1;
  shooter_cmd.motors[0] = {cfg.hardware.motors[1].id, Mode::kVelocity, 0, 50.0, 2.5};
  talos::drive::Packet shooter_pkt{};
  shooter_pkt.size = static_cast<uint32_t>(Encode(shooter_cmd, shooter_pkt.data));
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
  node.Tick(15000); // 13000us since last drive command

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

  ipc::Subscriber<talos::drive::Packet> ds_sub{"/hw/ds"};

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

}  // namespace
}  // namespace talos::hardware
