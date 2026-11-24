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

// REQ-SAFE-04, REQ-SAFE-36, REQ-SAFE-37: non-finite (NaN, +inf, -inf) demands,
// feedforwards, rezeros, PWM outputs, and mechanism goals never command
// non-neutral actuation or corrupt zero state across the C ABI.
TEST(SafetyAbiNonFinite, NonFiniteActuationAndRezeroNeutralizeSafely) {
  constexpr const char* kToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.rig.motors.wheel]
can_id = 1
foc = true
max_torque_current_a = 40.0
[subsystems.rig.sensors.gyro]
type = "Pigeon2"
can_id = 2
[subsystems.rig.pwm_outputs.servo]
channel = 0

[subsystems.arm.motors.arm]
can_id = 3
feedback = "RemoteCANcoder"
feedback_sensor = "arm_cc"
position_frame = "absolute"
[subsystems.arm.sensors.arm_cc]
type = "CANcoder"
can_id = 4
[subsystems.arm.arm]
tolerance_rot = 0.01
units_per_rot = 360.0

[subsystems.shooter.motors.flywheel]
can_id = 5
max_velocity_rps = 80.0
[subsystems.shooter.flywheel]
at_speed_entry_tick_count = 2
)";

  TalosSim* sim = talos_sim_from_toml(kToml);
  ASSERT_NE(sim, nullptr) << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim);
  ASSERT_NE(hw, nullptr) << talos_last_error();

  struct RigCtx {
    TalosNode* node = nullptr;
    int64_t wheel = -1;
    int64_t gyro = -1;
    int64_t servo = -1;
    int32_t mode = TALOS_MODE_NEUTRAL;
    double demand = 0.0;
    double ff = 0.0;
    double pwm_out = 0.0;
    bool set_pwm = false;
    int32_t motor_rc = TALOS_OK;
    int32_t pwm_rc = TALOS_OK;
  } rig;
  auto build_rig = [](TalosNode* node, void* user) -> int32_t {
    auto& r = *static_cast<RigCtx*>(user);
    r.node = node;
    r.wheel = talos_device(node, TALOS_DEVICE_MOTOR, "wheel");
    r.gyro = talos_device(node, TALOS_DEVICE_GYRO, "gyro");
    r.servo = talos_device(node, TALOS_DEVICE_PWM, "servo");
    if (r.wheel < 0 || r.gyro < 0 || r.servo < 0) return 1;
    if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
    return talos_node_set_update(
        node,
        [](void* ctx) -> int32_t {
          auto& self = *static_cast<RigCtx*>(ctx);
          self.motor_rc = talos_motor_set(self.node, self.wheel, self.mode,
                                          self.demand, self.ff, 0);
          if (self.set_pwm) {
            self.pwm_rc = talos_pwm_set(self.node, self.servo, self.pwm_out);
          }
          return 0;
        },
        &r);
  };

  struct ArmCtx {
    TalosNode* node = nullptr;
    int64_t mech = -1;
    int op = 0;  // 0=goal, 1=goal_units, 2=hold
    double arg = 0.0;
    double ff = 0.0;
  } arm_ctx;
  auto build_arm = [](TalosNode* node, void* user) -> int32_t {
    auto& a = *static_cast<ArmCtx*>(user);
    a.node = node;
    a.mech = talos_position_mechanism_open(node, "arm");
    if (a.mech < 0) return 1;
    if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
    return talos_node_set_update(
        node,
        [](void* ctx) -> int32_t {
          auto& self = *static_cast<ArmCtx*>(ctx);
          int32_t rc = TALOS_OK;
          if (self.op == 0) {
            rc = talos_position_mechanism_set_goal(self.node, self.mech,
                                                   self.arg, self.ff);
          } else if (self.op == 1) {
            rc = talos_position_mechanism_set_goal_units(self.node, self.mech,
                                                         self.arg, self.ff);
          } else {
            rc = talos_position_mechanism_hold(self.node, self.mech, self.ff);
          }
          if (rc != TALOS_OK) return 1;
          return talos_position_mechanism_update(self.node, self.mech);
        },
        &a);
  };

  struct ShooterCtx {
    TalosNode* node = nullptr;
    int64_t mech = -1;
    double vel = 0.0;
    double ff = 0.0;
  } shooter_ctx;
  auto build_shooter = [](TalosNode* node, void* user) -> int32_t {
    auto& s = *static_cast<ShooterCtx*>(user);
    s.node = node;
    s.mech = talos_velocity_mechanism_open(node, "flywheel");
    if (s.mech < 0) return 1;
    if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
    return talos_node_set_update(
        node,
        [](void* ctx) -> int32_t {
          auto& self = *static_cast<ShooterCtx*>(ctx);
          if (talos_velocity_mechanism_set_goal(self.node, self.mech, self.vel,
                                                self.ff) != TALOS_OK) {
            return 1;
          }
          return talos_velocity_mechanism_update(self.node, self.mech);
        },
        &s);
  };

  TalosNode* rig_node = talos_sim_add(sim, "rig", build_rig, &rig);
  ASSERT_NE(rig_node, nullptr) << talos_last_error();
  ASSERT_NE(talos_sim_add(sim, "arm", build_arm, &arm_ctx), nullptr)
      << talos_last_error();
  ASSERT_NE(talos_sim_add(sim, "shooter", build_shooter, &shooter_ctx), nullptr)
      << talos_last_error();

  ASSERT_EQ(talos_sim_hardware_enable(hw, 1), TALOS_OK);
  ASSERT_EQ(
      talos_sim_hardware_set_motor(hw, "rig", "wheel", 1.0, 0.0, 0, 0, 0, 0, 0),
      TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_gyro(hw, "rig", "gyro", 0.5, 0.0, 0, 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim, 15 * kMs), TALOS_OK) << talos_last_error();

  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  const double kInf = std::numeric_limits<double>::infinity();
  const double kNonFinite[] = {kNaN, kInf, -kInf};

  // Rezero ignores non-finite values: zeroed remains 0.
  for (const double bad : kNonFinite) {
    ASSERT_EQ(talos_motor_rezero(rig_node, rig.wheel, bad), TALOS_OK);
    ASSERT_EQ(talos_gyro_rezero(rig_node, rig.gyro, bad), TALOS_OK);
    EXPECT_EQ(talos_motor_zeroed(rig_node, rig.wheel), 0);
    EXPECT_EQ(talos_gyro_zeroed(rig_node, rig.gyro), 0);
  }
  // Establish a valid motor zero so position/motion_magic modes are active.
  ASSERT_EQ(talos_motor_rezero(rig_node, rig.wheel, 0.0), TALOS_OK);
  ASSERT_EQ(talos_motor_zeroed(rig_node, rig.wheel), 1);

  // Motor non-finite demands across every control mode neutralize and count.
  uint64_t prev_clamped = 0;
  ASSERT_EQ(talos_motor_clamped_count(rig_node, rig.wheel, &prev_clamped),
            TALOS_OK);
  for (const int32_t mode :
       {TALOS_MODE_DUTY_CYCLE, TALOS_MODE_VOLTAGE, TALOS_MODE_VELOCITY,
        TALOS_MODE_POSITION, TALOS_MODE_MOTION_MAGIC,
        TALOS_MODE_TORQUE_CURRENT}) {
    for (const double bad : kNonFinite) {
      rig.mode = mode;
      rig.demand = bad;
      rig.ff = 0.0;
      rig.set_pwm = true;
      rig.pwm_out = bad;
      ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();
      EXPECT_EQ(rig.motor_rc, TALOS_OK);
      EXPECT_EQ(rig.pwm_rc, TALOS_OK);
      TalosMotorRequest req{};
      ASSERT_EQ(talos_sim_hardware_applied_motor(hw, "rig", "wheel", &req),
                TALOS_OK);
      EXPECT_EQ(req.mode, TALOS_MODE_NEUTRAL) << "mode " << mode;
      double pwm = 1.0;
      EXPECT_EQ(talos_sim_hardware_applied_pwm(hw, "rig", "servo", &pwm), 0);
      uint64_t clamped = 0;
      ASSERT_EQ(talos_motor_clamped_count(rig_node, rig.wheel, &clamped),
                TALOS_OK);
      EXPECT_GT(clamped, prev_clamped) << "mode " << mode;
      prev_clamped = clamped;
    }
  }

  // Non-finite ff_volts on FF modes neutralizes; on non-FF modes returns
  // TALOS_ERR and leaves motor neutral.
  for (const int32_t mode :
       {TALOS_MODE_VELOCITY, TALOS_MODE_POSITION, TALOS_MODE_MOTION_MAGIC}) {
    for (const double bad : kNonFinite) {
      rig.mode = mode;
      rig.demand = 1.0;
      rig.ff = bad;
      rig.set_pwm = false;
      ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();
      EXPECT_EQ(rig.motor_rc, TALOS_OK);
      TalosMotorRequest req{};
      ASSERT_EQ(talos_sim_hardware_applied_motor(hw, "rig", "wheel", &req),
                TALOS_OK);
      EXPECT_EQ(req.mode, TALOS_MODE_NEUTRAL);
    }
  }
  for (const int32_t mode : {TALOS_MODE_NEUTRAL, TALOS_MODE_DUTY_CYCLE,
                             TALOS_MODE_VOLTAGE, TALOS_MODE_TORQUE_CURRENT}) {
    for (const double bad : kNonFinite) {
      rig.mode = mode;
      rig.demand = 0.5;
      rig.ff = bad;
      ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();
      EXPECT_EQ(rig.motor_rc, TALOS_ERR) << "mode " << mode;
      TalosMotorRequest req{};
      ASSERT_EQ(talos_sim_hardware_applied_motor(hw, "rig", "wheel", &req),
                TALOS_OK);
      EXPECT_EQ(req.mode, TALOS_MODE_NEUTRAL);
    }
  }

  // PositionMechanism & VelocityMechanism non-finite goals/ff neutralize.
  for (const double bad : kNonFinite) {
    for (const int op : {0, 1, 2}) {
      arm_ctx.op = op;
      arm_ctx.arg = (op == 2) ? 0.0 : bad;
      arm_ctx.ff = (op == 2) ? bad : 0.0;
      shooter_ctx.vel = bad;
      shooter_ctx.ff = 0.0;
      ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();
      TalosMotorRequest arm_req{};
      TalosMotorRequest vel_req{};
      ASSERT_EQ(talos_sim_hardware_applied_motor(hw, "arm", "arm", &arm_req),
                TALOS_OK);
      ASSERT_EQ(
          talos_sim_hardware_applied_motor(hw, "shooter", "flywheel", &vel_req),
          TALOS_OK);
      EXPECT_EQ(arm_req.mode, TALOS_MODE_NEUTRAL);
      EXPECT_EQ(vel_req.mode, TALOS_MODE_NEUTRAL);
    }
    arm_ctx.op = 0;
    arm_ctx.arg = 0.2;
    arm_ctx.ff = bad;
    shooter_ctx.vel = 20.0;
    shooter_ctx.ff = bad;
    ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();
    TalosMotorRequest arm_req{};
    TalosMotorRequest vel_req{};
    ASSERT_EQ(talos_sim_hardware_applied_motor(hw, "arm", "arm", &arm_req),
              TALOS_OK);
    ASSERT_EQ(
        talos_sim_hardware_applied_motor(hw, "shooter", "flywheel", &vel_req),
        TALOS_OK);
    EXPECT_EQ(arm_req.mode, TALOS_MODE_NEUTRAL);
    EXPECT_EQ(vel_req.mode, TALOS_MODE_NEUTRAL);
  }

  talos_sim_hardware_destroy(hw);
  talos_sim_destroy(sim);
}

}  // namespace
