#include <gtest/gtest.h>

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

}  // namespace
}  // namespace talos::hardware
