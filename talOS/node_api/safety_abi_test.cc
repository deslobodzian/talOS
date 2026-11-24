// Safety and boundary regression tests for talOS/node_api (RULES.md 2, 6, 7).

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/node/node.h"
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

// REQ-SAFE-14, REQ-SAFE-09: talos_publisher_send refuses to write directly to
// a TargetLane publisher slot, preventing a caller from forging an active
// TargetEnvelope while OperatorArming is disarmed or forging mode_since_us.
TEST_F(SafetyAbiTest, PublisherSendRefusesTargetLaneSlot) {
  static constexpr TalosSchema kFloatSchema{"safety_abi_test.Speed", 0x5eedULL,
                                            nullptr, 0};
  struct Consumer {
    TalosNode* node = nullptr;
    int64_t target = -1;
    int32_t got_target = 0;
  } consumer;
  auto build_consumer = [](TalosNode* node, void* user) -> int32_t {
    auto& c = *static_cast<Consumer*>(user);
    c.node = node;
    c.target =
        talos_target_open(node, sizeof(float), alignof(float), &kFloatSchema);
    if (c.target < 0) return 1;
    return talos_node_set_update(
        node,
        [](void* ctx) -> int32_t {
          auto& self = *static_cast<Consumer*>(ctx);
          float value = 0.0f;
          if (talos_target_get(self.node, self.target, &value, nullptr) == 1) {
            self.got_target = 1;
          }
          return 0;
        },
        &c);
  };

  struct ForgedLaneWriter {
    TalosSim* sim = nullptr;
    TalosNode* node = nullptr;
    int64_t arming = -1;
    int64_t lane = -1;
    int32_t send_status = 0;
    std::string send_error;
    bool attempted = false;
  } writer{sim_};
  auto build_writer = [](TalosNode* node, void* user) -> int32_t {
    auto& w = *static_cast<ForgedLaneWriter*>(user);
    w.node = node;
    if (talos_driver_station(node, nullptr, nullptr) != TALOS_OK) return 1;
    w.arming = talos_operator_arming_open(node);
    if (w.arming < 0) return 1;
    // Opening the first publisher on this node places /roller/target/teleop at
    // publisher slot 0, armed by w.arming (which remains disarmed).
    w.lane = talos_target_lane_open(node, "roller", "teleop", sizeof(float),
                                    alignof(float), &kFloatSchema, w.arming);
    if (w.lane < 0) return 1;
    return talos_node_set_update(
        node,
        [](void* ctx) -> int32_t {
          auto& self = *static_cast<ForgedLaneWriter*>(ctx);
          const uint64_t gen = talos_sim_driver_station_generation(self.sim);
          if (gen == 0 || self.attempted) return 0;
          self.attempted = true;
          // 16-byte TargetEnvelope header + 4-byte float = 20 bytes.
          uint8_t forged[20] = {};
          std::memcpy(forged, &gen, sizeof(gen));
          forged[8] = 1;  // active = true
          const float speed = 9.0f;
          std::memcpy(forged + 16, &speed, sizeof(speed));
          self.send_status =
              talos_publisher_send(self.node, 0, forged, sizeof(forged));
          self.send_error = talos_last_error();
          return 0;
        },
        &w);
  };

  ASSERT_NE(talos_sim_add(sim_, "roller", build_consumer, &consumer), nullptr)
      << talos_last_error();
  ASSERT_NE(talos_sim_add(sim_, "driver", build_writer, &writer), nullptr)
      << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_enable(hw_, 1), TALOS_OK);
  TalosDriverStationPacket ds{};
  ds.flags = TALOS_DS_ATTACHED | TALOS_DS_ENABLED | TALOS_DS_TELEOP;
  ASSERT_EQ(talos_sim_hardware_set_driver_station(hw_, &ds), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 40 * kMs), TALOS_OK) << talos_last_error();

  EXPECT_TRUE(writer.attempted);
  EXPECT_EQ(writer.send_status, TALOS_ERR);
  EXPECT_NE(writer.send_error.find("target lane"), std::string::npos)
      << writer.send_error;
  EXPECT_EQ(consumer.got_target, 0);
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

// REQ-DEV-09: HostCppNode<T> checks for duplicate extension before
// constructing/installing update() (preventing UAF if add_extension throws)
// and propagates e.what() into talos_last_error().
TEST_F(SafetyAbiTest, HostCppNodeGuardsDuplicateUafAndPropagatesLastError) {
  struct HostedCounter {
    explicit HostedCounter(talos::Node&) : canary{0x12345678u} {}
    ~HostedCounter() { canary = 0; }
    void update() {
      EXPECT_EQ(canary, 0x12345678u);
      ++ticks;
    }
    uint32_t canary;
    int ticks = 0;
  };
  struct ThrowingHosted {
    explicit ThrowingHosted(talos::Node&) {
      throw std::invalid_argument("boom from C++ constructor");
    }
  };

  struct State {
    int32_t first_rc = -1;
    int32_t second_rc = 0;
    std::string second_err;
    int32_t throw_rc = 0;
    std::string throw_err;
  } state;

  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& s = *static_cast<State*>(user);
    s.first_rc = talos::detail::HostCppNode<HostedCounter>(node);
    s.second_rc = talos::detail::HostCppNode<HostedCounter>(node);
    s.second_err = talos_last_error();
    s.throw_rc = talos::detail::HostCppNode<ThrowingHosted>(node);
    s.throw_err = talos_last_error();
    return s.first_rc == 0 ? 0 : 1;
  };

  ASSERT_NE(talos_sim_add(sim_, "roller", build, &state), nullptr)
      << talos_last_error();
  EXPECT_EQ(state.first_rc, 0);
  EXPECT_EQ(state.second_rc, -1);
  EXPECT_NE(state.second_err.find("extension added twice"), std::string::npos)
      << state.second_err;
  EXPECT_EQ(state.throw_rc, -1);
  EXPECT_NE(state.throw_err.find("boom from C++ constructor"),
            std::string::npos)
      << state.throw_err;

  // Stepping runs HostedCounter::update(); under ASan this catches UAF if the
  // failed second HostCppNode overwrote update() with a freed instance.
  ASSERT_EQ(talos_sim_step(sim_, 30 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(talos_node_tick(talos_sim_node(sim_, "roller")), 3u);
}

// REQ-RT-05: talos_sim_resume guards overdue callbacks with Stepping so
// re-entrant remove/add/step/pause/resume return TALOS_ERR instead of causing
// a use-after-free while run_until() is on the stack.
TEST_F(SafetyAbiTest, SimResumeRejectsReentrantMutations) {
  struct ResumeProbe {
    TalosSim* sim = nullptr;
    bool armed = false;
    bool ran = false;
    int32_t remove_rc = TALOS_OK;
    TalosNode* add_rc = nullptr;
    int32_t step_rc = TALOS_OK;
    int32_t pause_rc = TALOS_OK;
    int32_t resume_rc = TALOS_OK;
  } probe{sim_};

  auto build = [](TalosNode* node, void* user) -> int32_t {
    return talos_node_set_update(
        node,
        [](void* ctx) -> int32_t {
          auto& p = *static_cast<ResumeProbe*>(ctx);
          if (!p.armed || p.ran) return 0;
          p.ran = true;
          p.remove_rc = talos_sim_remove(p.sim, "roller");
          p.add_rc = talos_sim_add(
              p.sim, "extra", [](TalosNode*, void*) { return 0; }, nullptr);
          p.step_rc = talos_sim_step(p.sim, 1 * kMs);
          p.pause_rc = talos_sim_pause(p.sim, "roller", 1 * kMs);
          p.resume_rc = talos_sim_resume(p.sim, "roller");
          return 0;
        },
        user);
  };

  ASSERT_NE(talos_sim_add(sim_, "roller", build, &probe), nullptr)
      << talos_last_error();
  ASSERT_EQ(talos_sim_step(sim_, 10 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_sim_pause(sim_, "roller", 100 * kMs), TALOS_OK)
      << talos_last_error();
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();

  probe.armed = true;
  ASSERT_EQ(talos_sim_resume(sim_, "roller"), TALOS_OK) << talos_last_error();
  EXPECT_TRUE(probe.ran);
  EXPECT_EQ(probe.remove_rc, TALOS_ERR);
  EXPECT_EQ(probe.add_rc, nullptr);
  EXPECT_EQ(probe.step_rc, TALOS_ERR);
  EXPECT_EQ(probe.pause_rc, TALOS_ERR);
  EXPECT_EQ(probe.resume_rc, TALOS_ERR);
  EXPECT_NE(talos_sim_node(sim_, "roller"), nullptr);
}

}  // namespace
