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

// REQ-CONTRACT-13: talos_run rejects negative argc, null argv, and null
// argv[i] cleanly instead of dereferencing null.
TEST_F(SafetyAbiTest, RunRejectsNegativeArgcAndNullArgv) {
  auto noop = [](TalosNode*, void*) -> int32_t { return 0; };
  EXPECT_EQ(talos_run(-1, nullptr, "roller", "//a:roller", noop, nullptr), 1);
  EXPECT_NE(std::string{talos_last_error()}.size(), 0u);
  EXPECT_EQ(talos_run(2, nullptr, "roller", "//a:roller", noop, nullptr), 1);
  char prog[] = "node";
  char* with_null[] = {prog, nullptr};
  EXPECT_EQ(talos_run(2, with_null, "roller", "//a:roller", noop, nullptr), 1);
}

// REQ-CONTRACT-13: C ABI sim/target functions reject zero-byte raw topics and
// envelope sizes that overflow uint32_t.
TEST_F(SafetyAbiTest, SimRawAndTargetRejectZeroAndOverflowingSizes) {
  uint8_t buf[8] = {};
  EXPECT_EQ(talos_sim_publish_raw(sim_, "/roller/zero", buf, 0), TALOS_ERR);
  EXPECT_EQ(talos_sim_latest_raw(sim_, "/roller/zero", buf, 0), TALOS_ERR);

  constexpr uint32_t kOverflow = 0xFFFFFFF8u;  // +16 exceeds uint32_t::max()
  int32_t active = -1;
  EXPECT_EQ(talos_sim_publish_target(sim_, "roller", "teleop", buf, kOverflow,
                                     8, nullptr),
            TALOS_ERR);
  EXPECT_EQ(talos_sim_latest_target(sim_, "roller", "teleop", buf, kOverflow, 8,
                                    &active),
            TALOS_ERR);

  struct BadTarget {
    int64_t target = 0;
    int64_t lane = 0;
  } probe;
  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& p = *static_cast<BadTarget*>(user);
    constexpr TalosSchema kSchema{"safety.Big", 1u, nullptr, 0};
    p.target = talos_target_open(node, kOverflow, 8, &kSchema);
    p.lane = talos_target_lane_open(node, "roller", "auto", kOverflow, 8,
                                    &kSchema, -1);
    return 0;
  };
  ASSERT_NE(talos_sim_add(sim_, "roller", build, &probe), nullptr)
      << talos_last_error();
  EXPECT_EQ(probe.target, -1);
  EXPECT_EQ(probe.lane, -1);
}

// REQ-SAFE-04: talos_motor_set with TALOS_MODE_NEUTRAL clears nonzero demand
// and slot instead of sending non-zero fields in a neutral request.
TEST_F(SafetyAbiTest, MotorSetNeutralNormalizesDemandAndSlot) {
  struct NeutralSender {
    TalosNode* node = nullptr;
    int64_t wheel = -1;
  } sender;
  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& s = *static_cast<NeutralSender*>(user);
    s.node = node;
    s.wheel = talos_device(node, TALOS_DEVICE_MOTOR, "wheel");
    if (s.wheel < 0) return 1;
    if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
    return talos_node_set_update(
        node,
        [](void* ctx) -> int32_t {
          auto& self = *static_cast<NeutralSender*>(ctx);
          return talos_motor_set(self.node, self.wheel, TALOS_MODE_NEUTRAL, 5.0,
                                 0.0, 2);
        },
        &s);
  };
  ASSERT_NE(talos_sim_add(sim_, "roller", build, &sender), nullptr)
      << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_enable(hw_, 1), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();
  TalosMotorRequest req{};
  ASSERT_EQ(talos_sim_hardware_motor_request(hw_, "roller", "wheel", &req),
            TALOS_OK);
  EXPECT_EQ(req.mode, TALOS_MODE_NEUTRAL);
  EXPECT_DOUBLE_EQ(req.demand, 0.0);
  EXPECT_EQ(req.slot, 0);
}

// REQ-RT-05, REQ-CONTRACT-13: C ABI sim step, pause, gateway clock, and
// lose_states reject signed overflow and re-entrant pause/resume.
TEST_F(SafetyAbiTest, SimAndHardwareRejectOverflowAndReentrantPauseResume) {
  struct Reenter {
    TalosSim* sim = nullptr;
    int32_t pause_res = TALOS_OK;
    int32_t resume_res = TALOS_OK;
  } r{sim_};
  auto build = [](TalosNode* node, void* user) -> int32_t {
    return talos_node_set_update(
        node,
        [](void* ctx) -> int32_t {
          auto& self = *static_cast<Reenter*>(ctx);
          self.pause_res = talos_sim_pause(self.sim, "roller", 5 * kMs);
          self.resume_res = talos_sim_resume(self.sim, "roller");
          return 0;
        },
        user);
  };
  ASSERT_NE(talos_sim_add(sim_, "roller", build, &r), nullptr)
      << talos_last_error();
  ASSERT_EQ(talos_sim_step(sim_, 10 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(r.pause_res, TALOS_ERR);
  EXPECT_EQ(r.resume_res, TALOS_ERR);

  EXPECT_EQ(talos_sim_step(sim_, std::numeric_limits<int64_t>::max()),
            TALOS_ERR);
  EXPECT_EQ(
      talos_sim_pause(sim_, "roller", std::numeric_limits<int64_t>::max()),
      TALOS_ERR);
  EXPECT_EQ(talos_sim_hardware_set_gateway_clock(
                hw_, std::numeric_limits<int64_t>::min(), 1),
            TALOS_ERR);
  ASSERT_EQ(
      talos_sim_hardware_lose_states(hw_, std::numeric_limits<int32_t>::max()),
      TALOS_OK);
  EXPECT_EQ(talos_sim_hardware_lose_states(hw_, 1), TALOS_ERR);
}

}  // namespace
