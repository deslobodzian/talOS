// Safety and boundary regression tests for talOS/node_api (RULES.md 2, 6, 7).

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>

#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/node_api/node_api.h"

namespace {

constexpr int64_t kMs = 1'000'000;

constexpr const char* kRobotToml = R"(
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.roller.motors.wheel]
can_id = 1
)";

struct ForgedWriter {
  TalosNode* node = nullptr;
  int64_t motor = -1;
  int32_t send_status = 0;
  std::string send_error;
  bool attempted = false;
};

int32_t ForgedWriterUpdate(void* user) {
  auto& w = *static_cast<ForgedWriter*>(user);
  if (w.attempted) return 0;
  w.attempted = true;
  // Build a non-neutral hardware::Command for motor CAN id 1 and attempt to
  // inject it directly into publisher slot 0 (the devices layer's /hw/request
  // publisher) via talos_publisher_send, bypassing robot_enabled and target
  // gating.
  talos::hardware::Command cmd{};
  cmd.count = 1;
  cmd.motors[0].id = 1;
  cmd.motors[0].mode = talos::hardware::Mode::kDutyCycle;
  cmd.motors[0].demand = 0.75;
  talos::hardware::Packet packet{};
  packet.size =
      static_cast<uint32_t>(talos::hardware::Encode(cmd, packet.data));
  w.send_status = talos_publisher_send(w.node, 0, &packet, sizeof(packet));
  w.send_error = talos_last_error();
  return 0;
}

int32_t BuildForgedWriter(TalosNode* node, void* user) {
  auto& w = *static_cast<ForgedWriter*>(user);
  w.node = node;
  // Opening the first output device registers /hw/request/<node> at slot 0.
  w.motor = talos_device(node, TALOS_DEVICE_MOTOR, "wheel");
  if (w.motor < 0) return 1;
  return talos_node_set_update(node, &ForgedWriterUpdate, &w);
}

class SafetyAbiTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_EQ(talos_abi_version(), TALOS_NODE_ABI_VERSION);
    sim_ = talos_sim_from_toml(kRobotToml);
    ASSERT_NE(sim_, nullptr) << talos_last_error();
    hw_ = talos_sim_hardware_create(sim_);
    ASSERT_NE(hw_, nullptr) << talos_last_error();
  }
  void TearDown() override {
    talos_sim_hardware_destroy(hw_);
    talos_sim_destroy(sim_);
  }
  TalosSim* sim_ = nullptr;
  TalosSimHardware* hw_ = nullptr;
};

// REQ-SAFE-02: talos_publisher_send refuses to write directly to the devices
// layer's /hw/request/<node> publisher slot (RULES.md 2).
TEST_F(SafetyAbiTest, PublisherSendRefusesHardwareRequestSlot) {
  ForgedWriter writer;
  ASSERT_NE(talos_sim_add(sim_, "roller", &BuildForgedWriter, &writer), nullptr)
      << talos_last_error();
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();

  EXPECT_TRUE(writer.attempted);
  EXPECT_EQ(writer.send_status, TALOS_ERR);
  EXPECT_NE(writer.send_error.find("hardware request"), std::string::npos)
      << writer.send_error;

  TalosMotorRequest req{};
  ASSERT_EQ(talos_sim_hardware_motor_request(hw_, "roller", "wheel", &req),
            TALOS_OK);
  EXPECT_EQ(req.mode, TALOS_MODE_NEUTRAL);
  EXPECT_DOUBLE_EQ(req.demand, 0.0);
}

// REQ-CONTRACT-13: talos_sim_hardware_set_driver_station rejects out-of-range
// alliance, station, and POV values instead of silently casting or truncating.
TEST_F(SafetyAbiTest, SetDriverStationRejectsOutOfRangeAllianceStationAndPov) {
  TalosDriverStationPacket bad_alliance{};
  bad_alliance.flags = TALOS_DS_ATTACHED;
  bad_alliance.alliance = 99;
  EXPECT_EQ(talos_sim_hardware_set_driver_station(hw_, &bad_alliance),
            TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("alliance"), std::string::npos)
      << talos_last_error();

  bad_alliance.alliance = -1;
  EXPECT_EQ(talos_sim_hardware_set_driver_station(hw_, &bad_alliance),
            TALOS_ERR);

  TalosDriverStationPacket bad_station{};
  bad_station.flags = TALOS_DS_ATTACHED;
  bad_station.station = -1;
  EXPECT_EQ(talos_sim_hardware_set_driver_station(hw_, &bad_station),
            TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("station"), std::string::npos)
      << talos_last_error();

  bad_station.station = 256;
  EXPECT_EQ(talos_sim_hardware_set_driver_station(hw_, &bad_station),
            TALOS_ERR);

  TalosDriverStationPacket bad_pov{};
  bad_pov.flags = TALOS_DS_ATTACHED;
  bad_pov.joysticks[0].connected = 1;
  bad_pov.joysticks[0].pov_count = 1;
  bad_pov.joysticks[0].povs[0] =
      static_cast<int32_t>(std::numeric_limits<int16_t>::max()) + 1;
  EXPECT_EQ(talos_sim_hardware_set_driver_station(hw_, &bad_pov), TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("pov"), std::string::npos)
      << talos_last_error();

  bad_pov.joysticks[0].povs[0] =
      static_cast<int32_t>(std::numeric_limits<int16_t>::min()) - 1;
  EXPECT_EQ(talos_sim_hardware_set_driver_station(hw_, &bad_pov), TALOS_ERR);

  TalosDriverStationPacket valid{};
  valid.flags = TALOS_DS_ATTACHED;
  valid.alliance = TALOS_ALLIANCE_BLUE;
  valid.station = 3;
  valid.joysticks[0].connected = 1;
  valid.joysticks[0].pov_count = 1;
  valid.joysticks[0].povs[0] = std::numeric_limits<int16_t>::max();
  EXPECT_EQ(talos_sim_hardware_set_driver_station(hw_, &valid), TALOS_OK)
      << talos_last_error();
}

}  // namespace
