// The C ABI drives the same Node, devices and Sim a C++ node does: these
// tests build nodes purely through it, the way the Python binding does.

#include "talOS/node_api/node_api.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>

namespace {

constexpr int64_t kMs = 1'000'000;

constexpr const char* kRobotToml = R"(
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.relay]
gain = 2.5
label = "left"
cameras = ["front", "back"]
offsets_m = [0.5, 1]
none = []
mixed = [1.0, "back"]

[subsystems.roller.motors.wheel]
can_id = 1
[subsystems.roller.digital_inputs.beam]
dio = 0
[subsystems.roller.sensors.gyro]
type = "Pigeon2"
can_id = 2
)";

struct Value {
  double x;
  int64_t stamp;
};

// Value's schema identity, as a binding passes its message class's.
constexpr TalosSchema kValueSchema{"node_api_test.Value", 0x0123456789abcdefULL,
                                   nullptr, 0};
constexpr TalosSchema kFloatSchema{"node_api_test.Speed", 0x5eedULL, nullptr,
                                   0};

// Relays /relay/in to /relay/out, scaled by its `gain` parameter.
struct Relay {
  TalosNode* node = nullptr;
  int64_t in = -1;
  int64_t out = -1;
  double gain = 0;
  int callbacks = 0;
  bool fail_update = false;
};

int32_t RelayUpdate(void* user) {
  auto& r = *static_cast<Relay*>(user);
  if (r.fail_update) return 1;
  if (talos_subscription_has_value(r.node, r.in) != 1) return 0;
  const auto* in =
      static_cast<const Value*>(talos_subscription_value(r.node, r.in));
  const Value out{in->x * r.gain, talos_node_now_ns(r.node)};
  return talos_publisher_send(r.node, r.out, &out, sizeof out) == 1 ? 0 : 1;
}

int32_t RelayMessage(const void* data, uint32_t size, void* user) {
  EXPECT_EQ(size, sizeof(Value));
  EXPECT_NE(data, nullptr);
  ++static_cast<Relay*>(user)->callbacks;
  return 0;
}

int32_t BuildRelay(TalosNode* node, void* user) {
  auto& r = *static_cast<Relay*>(user);
  r.node = node;
  TalosParam gain{};
  if (talos_node_param(node, "gain", TALOS_PARAM_DOUBLE, 1, &gain) != 1) {
    return 1;
  }
  r.gain = gain.number;
  r.in = talos_node_subscribe(node, "/relay/in", sizeof(Value), &kValueSchema,
                              0, &RelayMessage, &r);
  r.out = talos_node_publish(node, "/relay/out", sizeof(Value), &kValueSchema);
  if (r.in < 0 || r.out < 0) return 1;
  return talos_node_set_update(node, &RelayUpdate, &r);
}

class NodeApi : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_EQ(talos_abi_version(), TALOS_NODE_ABI_VERSION);
    sim_ = talos_sim_from_toml(kRobotToml);
    ASSERT_NE(sim_, nullptr) << talos_last_error();
  }
  void TearDown() override { talos_sim_destroy(sim_); }
  TalosSim* sim_ = nullptr;
};

TEST_F(NodeApi, NodeSubscribesPublishesAndTicks) {
  Relay relay;
  TalosNode* node = talos_sim_add(sim_, "relay", &BuildRelay, &relay);
  ASSERT_NE(node, nullptr) << talos_last_error();
  EXPECT_STREQ(talos_node_name(node), "relay");
  EXPECT_EQ(talos_node_simulation(node), 1);
  EXPECT_EQ(talos_node_period_ns(node), 10 * kMs);
  EXPECT_EQ(talos_subscription_has_value(node, relay.in), 0);
  EXPECT_EQ(talos_subscription_fresh(node, relay.in, -1), 0);

  const Value in{4.0, 0};
  ASSERT_EQ(talos_sim_publish_raw(sim_, "/relay/in", &in, sizeof in), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 25 * kMs), TALOS_OK) << talos_last_error();

  EXPECT_EQ(talos_node_tick(node), 2u);
  EXPECT_EQ(relay.callbacks, 1);
  EXPECT_EQ(talos_subscription_has_value(node, relay.in), 1);
  EXPECT_EQ(talos_subscription_fresh(node, relay.in, -1), 1);
  EXPECT_EQ(talos_subscription_fresh(node, relay.in, 1 * kMs), 0);
  EXPECT_GT(talos_subscription_age_ns(node, relay.in), 20 * kMs);
  EXPECT_EQ(talos_subscription_updated(node, relay.in), 0);  // seen at tick 1

  Value out{};
  ASSERT_EQ(talos_sim_latest_raw(sim_, "/relay/out", &out, sizeof out), 1);
  EXPECT_DOUBLE_EQ(out.x, 10.0);
  EXPECT_EQ(out.stamp, 20 * kMs);
  EXPECT_EQ(talos_sim_now_ns(sim_), 25 * kMs);
}

TEST_F(NodeApi, SessionDirIsNullWithoutLog) {
  Relay relay;
  TalosNode* node = talos_sim_add(sim_, "relay", &BuildRelay, &relay);
  ASSERT_NE(node, nullptr) << talos_last_error();
  EXPECT_EQ(talos_node_session_dir(node), nullptr);
}

TEST_F(NodeApi, SessionDirSetBeforeAddIsVisibleToNodesAddedAfter) {
  ASSERT_EQ(talos_sim_set_session_dir(sim_, "/tmp/talos_session"), TALOS_OK);
  Relay relay;
  TalosNode* node = talos_sim_add(sim_, "relay", &BuildRelay, &relay);
  ASSERT_NE(node, nullptr) << talos_last_error();
  EXPECT_STREQ(talos_node_session_dir(node), "/tmp/talos_session");
}

TEST_F(NodeApi, SessionDirClearedBeforeAddIsAbsent) {
  ASSERT_EQ(talos_sim_set_session_dir(sim_, "/tmp/talos_session"), TALOS_OK);
  ASSERT_EQ(talos_sim_set_session_dir(sim_, ""), TALOS_OK);
  Relay relay;
  TalosNode* node = talos_sim_add(sim_, "relay", &BuildRelay, &relay);
  ASSERT_NE(node, nullptr) << talos_last_error();
  EXPECT_EQ(talos_node_session_dir(node), nullptr);
}

// A second node on /relay/out with a same-size type of another schema.
struct Impostor {
  int64_t in = -1;
};

int32_t BuildImpostor(TalosNode* node, void* user) {
  constexpr TalosSchema kOther{"node_api_test.Other", 0xbadULL, nullptr, 0};
  auto& i = *static_cast<Impostor*>(user);
  i.in = talos_node_subscribe(node, "/relay/out", sizeof(Value), &kOther, 0,
                              nullptr, nullptr);
  return i.in < 0 ? 1 : 0;
}

TEST_F(NodeApi, ATopicRefusesAnEndpointOfAnotherSchemaNamingBoth) {
  Relay relay;
  ASSERT_NE(talos_sim_add(sim_, "relay", &BuildRelay, &relay), nullptr);
  Impostor impostor;
  EXPECT_EQ(talos_sim_add(sim_, "impostor", &BuildImpostor, &impostor),
            nullptr);
  const std::string error = talos_last_error();
  EXPECT_NE(error.find("node_api_test.Value"), std::string::npos) << error;
  EXPECT_NE(error.find("node_api_test.Other"), std::string::npos) << error;
  EXPECT_EQ(talos_node_publish(nullptr, "/x/state", 8, nullptr), -1);
}

TEST_F(NodeApi, ParamsFallBackOrReportTheMissingKey) {
  Relay relay;
  TalosNode* node = talos_sim_add(sim_, "relay", &BuildRelay, &relay);
  ASSERT_NE(node, nullptr) << talos_last_error();

  TalosParam p{};
  ASSERT_EQ(talos_node_param(node, "label", TALOS_PARAM_ANY, 1, &p), 1);
  EXPECT_EQ(p.kind, TALOS_PARAM_STRING);
  EXPECT_STREQ(p.string, "left");
  EXPECT_EQ(talos_node_param(node, "absent", TALOS_PARAM_INT, 0, &p), 0);
  EXPECT_EQ(talos_node_param(node, "absent", TALOS_PARAM_ANY, 1, &p),
            TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("'absent'"),
            std::string::npos);
  EXPECT_EQ(talos_node_param(node, "label", TALOS_PARAM_BOOL, 0, &p),
            TALOS_ERR);
}

TEST_F(NodeApi, ListParamsComeBackAsCountedArrays) {
  Relay relay;
  TalosNode* node = talos_sim_add(sim_, "relay", &BuildRelay, &relay);
  ASSERT_NE(node, nullptr) << talos_last_error();

  TalosParam p{};
  for (int32_t kind :
       {TALOS_PARAM_STRING_LIST, TALOS_PARAM_LIST, TALOS_PARAM_ANY}) {
    ASSERT_EQ(talos_node_param(node, "cameras", kind, 1, &p), 1)
        << talos_last_error();
    EXPECT_EQ(p.kind, TALOS_PARAM_STRING_LIST);
    ASSERT_EQ(p.count, 2u);
    EXPECT_STREQ(p.strings[0], "front");
    EXPECT_STREQ(p.strings[1], "back");
  }
  for (int32_t kind :
       {TALOS_PARAM_DOUBLE_LIST, TALOS_PARAM_LIST, TALOS_PARAM_ANY}) {
    ASSERT_EQ(talos_node_param(node, "offsets_m", kind, 1, &p), 1)
        << talos_last_error();
    EXPECT_EQ(p.kind, TALOS_PARAM_DOUBLE_LIST);
    ASSERT_EQ(p.count, 2u);
    EXPECT_DOUBLE_EQ(p.numbers[0], 0.5);
    EXPECT_DOUBLE_EQ(p.numbers[1], 1.0);  // integer converted
  }
  ASSERT_EQ(talos_node_param(node, "none", TALOS_PARAM_STRING_LIST, 1, &p), 1);
  EXPECT_EQ(p.kind, TALOS_PARAM_STRING_LIST);
  EXPECT_EQ(p.count, 0u);
  ASSERT_EQ(talos_node_param(node, "none", TALOS_PARAM_LIST, 1, &p), 1);
  EXPECT_EQ(p.kind, TALOS_PARAM_DOUBLE_LIST);
  EXPECT_EQ(p.count, 0u);
  EXPECT_EQ(talos_node_param(node, "absent", TALOS_PARAM_LIST, 0, &p), 0);
  EXPECT_EQ(p.count, 0u);

  EXPECT_EQ(talos_node_param(node, "mixed", TALOS_PARAM_ANY, 1, &p), TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find(
                "'mixed' must be an array of strings or of numbers; element 1 "
                "is a string"),
            std::string::npos)
      << talos_last_error();
  EXPECT_EQ(talos_node_param(node, "cameras", TALOS_PARAM_DOUBLE_LIST, 0, &p),
            TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("element 0 is a string"),
            std::string::npos);
  EXPECT_EQ(talos_node_param(node, "label", TALOS_PARAM_LIST, 0, &p),
            TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("'label' must be an array"),
            std::string::npos);
}

TEST_F(NodeApi, FailingCallbackStopsTheStep) {
  Relay relay;
  ASSERT_NE(talos_sim_add(sim_, "relay", &BuildRelay, &relay), nullptr);
  relay.fail_update = true;
  EXPECT_EQ(talos_sim_step(sim_, 15 * kMs), TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("update"), std::string::npos);
}

TEST_F(NodeApi, RegistrationFailureFailsTheAdd) {
  Relay relay;
  auto build = [](TalosNode*, void*) -> int32_t { return 1; };
  EXPECT_EQ(talos_sim_add(sim_, "relay", build, &relay), nullptr);
  EXPECT_NE(std::string{talos_last_error()}.find("factory"), std::string::npos);
}

struct ByteSender {
  TalosNode* node = nullptr;
  int64_t out = -1;
  int32_t last_sent = -1;
};

int32_t BuildByteSender(TalosNode* node, void* user) {
  auto& b = *static_cast<ByteSender*>(user);
  b.node = node;
  b.out = talos_node_publish_bytes(node, "frames", 4, 16, 1, 1);
  if (b.out < 0) return 1;
  return talos_node_set_update(
      node,
      [](void* u) -> int32_t {
        auto& sender = *static_cast<ByteSender*>(u);
        const uint8_t payload[3] = {
            1, 2, static_cast<uint8_t>(talos_node_tick(sender.node))};
        sender.last_sent = talos_byte_publisher_send(sender.node, sender.out,
                                                     payload, sizeof payload);
        return sender.last_sent < 0 ? 1 : 0;
      },
      &b);
}

TEST_F(NodeApi, PublishesVariableLengthBytesAndQueriesLatest) {
  ByteSender sender;
  TalosNode* node = talos_sim_add(sim_, "relay", &BuildByteSender, &sender);
  ASSERT_NE(node, nullptr) << talos_last_error();

  uint32_t size = 99;
  EXPECT_EQ(talos_sim_latest_bytes(sim_, "/relay/frames", nullptr, 0, &size),
            0);
  EXPECT_EQ(size, 0u);

  ASSERT_EQ(talos_sim_step(sim_, 25 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(sender.last_sent, 1);

  ASSERT_EQ(talos_sim_latest_bytes(sim_, "/relay/frames", nullptr, 0, &size),
            1);
  EXPECT_EQ(size, 3u);
  uint8_t out[4] = {};
  EXPECT_EQ(talos_sim_latest_bytes(sim_, "/relay/frames", out, 2, &size),
            TALOS_ERR);
  ASSERT_EQ(
      talos_sim_latest_bytes(sim_, "/relay/frames", out, sizeof out, &size), 1);
  EXPECT_EQ(size, 3u);
  EXPECT_EQ(out[0], 1u);
  EXPECT_EQ(out[1], 2u);
  EXPECT_EQ(out[2], 1u);  // tick 1 at 20 ms

  // Oversize returns 0 (dropped), empty payload is allowed.
  const uint8_t oversize[5] = {9, 9, 9, 9, 9};
  EXPECT_EQ(talos_byte_publisher_send(node, sender.out, oversize, 5), 0);
  EXPECT_EQ(talos_byte_publisher_send(node, sender.out, nullptr, 0), 1);
  ASSERT_EQ(
      talos_sim_latest_bytes(sim_, "/relay/frames", out, sizeof out, &size), 1);
  EXPECT_EQ(size, 0u);
  EXPECT_EQ(talos_byte_publisher_send(node, sender.out, nullptr, 1), TALOS_ERR);
}

struct Stepper {
  TalosNode* node = nullptr;
  int64_t sm = -1;
  int32_t just_entered_at_0 = -1;
  int32_t just_entered_at_1 = -1;
  int32_t just_entered_at_3 = -1;
};

int32_t BuildStepper(TalosNode* node, void* user) {
  auto& s = *static_cast<Stepper*>(user);
  s.node = node;
  s.sm = talos_state_machine_create(node, 10);
  if (s.sm < 0) return 1;
  return talos_node_set_update(
      node,
      [](void* u) -> int32_t {
        auto& st = *static_cast<Stepper*>(u);
        const uint64_t tick = talos_node_tick(st.node);
        const int32_t je = talos_state_machine_just_entered(st.node, st.sm);
        if (tick == 0) st.just_entered_at_0 = je;
        if (tick == 1) st.just_entered_at_1 = je;
        if (tick == 2) {
          if (talos_state_machine_go(st.node, st.sm, 20) != TALOS_OK) return 1;
        }
        if (tick == 3) st.just_entered_at_3 = je;
        return 0;
      },
      &s);
}

TEST_F(NodeApi, StateMachineTracksTransitionsAndTiming) {
  Stepper stepper;
  TalosNode* node = talos_sim_add(sim_, "relay", &BuildStepper, &stepper);
  ASSERT_NE(node, nullptr) << talos_last_error();
  EXPECT_EQ(talos_state_machine_state(node, stepper.sm), 10);
  EXPECT_EQ(talos_state_machine_previous(node, stepper.sm), 10);

  ASSERT_EQ(talos_sim_step(sim_, 45 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(stepper.just_entered_at_0, 1);
  EXPECT_EQ(stepper.just_entered_at_1, 0);
  EXPECT_EQ(stepper.just_entered_at_3, 1);
  EXPECT_EQ(talos_state_machine_state(node, stepper.sm), 20);
  EXPECT_EQ(talos_state_machine_previous(node, stepper.sm), 10);
  EXPECT_EQ(talos_state_machine_time_in_state_ns(node, stepper.sm), 15 * kMs);
}

// --- devices -------------------------------------------------------------

struct Roller {
  TalosNode* node = nullptr;
  int64_t wheel = -1;
  int64_t beam = -1;
  int32_t enabled = -1;
};

int32_t BuildRoller(TalosNode* node, void* user) {
  auto& r = *static_cast<Roller*>(user);
  r.node = node;
  r.wheel = talos_device(node, TALOS_DEVICE_MOTOR, "wheel");
  r.beam = talos_device(node, TALOS_DEVICE_DIGITAL_INPUT, "beam");
  if (r.wheel < 0 || r.beam < 0) return 1;
  return talos_node_set_update(
      node,
      [](void* u) -> int32_t {
        auto& roller = *static_cast<Roller*>(u);
        roller.enabled = talos_robot_enabled(roller.node);
        TalosSample beam{};
        if (talos_digital_input_sample(roller.node, roller.beam, &beam) != 0) {
          return 1;
        }
        if (beam.value) return 0;  // unset: neutral
        return talos_motor_set(roller.node, roller.wheel, TALOS_MODE_VELOCITY,
                               12.0, 0.5, 1);
      },
      &r);
}

TEST_F(NodeApi, MotorClampCountUsesSharedDevicesImplementation) {
  Roller roller;
  TalosNode* node = talos_sim_add(sim_, "roller", &BuildRoller, &roller);
  ASSERT_NE(node, nullptr);
  uint64_t count = 99;
  ASSERT_EQ(talos_motor_clamped_count(node, roller.wheel, &count), TALOS_OK);
  EXPECT_EQ(count, 0u);
  ASSERT_EQ(
      talos_motor_set(node, roller.wheel, TALOS_MODE_VELOCITY, 120.0, 0, 0),
      TALOS_OK);
  ASSERT_EQ(talos_motor_clamped_count(node, roller.wheel, &count), TALOS_OK);
  EXPECT_EQ(count, 1u);
  ASSERT_EQ(talos_motor_set(node, roller.wheel, TALOS_MODE_VOLTAGE, 1.0, 0, 0),
            TALOS_OK);
  ASSERT_EQ(talos_motor_clamped_count(node, roller.wheel, &count), TALOS_OK);
  EXPECT_EQ(count, 1u);
  EXPECT_EQ(talos_motor_clamped_count(node, roller.wheel, nullptr), TALOS_ERR);
}

TEST_F(NodeApi, MotorHealthFeedbackRoundTripsWhileDisabled) {
  Roller roller;
  TalosNode* node = talos_sim_add(sim_, "roller", &BuildRoller, &roller);
  ASSERT_NE(node, nullptr);
  TalosSample sample{};
  ASSERT_EQ(talos_motor_sample(node, roller.wheel, &sample), TALOS_OK);
  EXPECT_EQ(sample.valid, 0);
  EXPECT_DOUBLE_EQ(sample.temperature_c, 0);
  EXPECT_DOUBLE_EQ(sample.supply_current_a, 0);
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr);
  ASSERT_EQ(talos_sim_hardware_set_motor(hw, "roller", "wheel", 3, 2, 0, 0,
                                         62.5, 18.25),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 30 * kMs), TALOS_OK);
  ASSERT_EQ(talos_motor_sample(node, roller.wheel, &sample), TALOS_OK);
  EXPECT_EQ(sample.valid, 1);
  EXPECT_EQ(roller.enabled, 0);
  EXPECT_DOUBLE_EQ(sample.temperature_c, 62.5);
  EXPECT_DOUBLE_EQ(sample.supply_current_a, 18.3);
  EXPECT_DOUBLE_EQ(sample.current, 0);
  ASSERT_EQ(talos_sim_hardware_set_connected(hw, "roller", "wheel", 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 30 * kMs), TALOS_OK);
  ASSERT_EQ(talos_motor_sample(node, roller.wheel, &sample), TALOS_OK);
  EXPECT_EQ(sample.valid, 0);
  EXPECT_DOUBLE_EQ(sample.temperature_c, 0);
  EXPECT_DOUBLE_EQ(sample.supply_current_a, 0);
  talos_sim_hardware_destroy(hw);
}

TEST_F(NodeApi, MotorRezeroUsesSharedSoftwareFrame) {
  Roller roller;
  TalosNode* node = talos_sim_add(sim_, "roller", &BuildRoller, &roller);
  ASSERT_NE(node, nullptr);
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  const auto gyro = talos_device(node, TALOS_DEVICE_GYRO, "gyro");
  ASSERT_GE(gyro, 0);
  ASSERT_NE(hw, nullptr);
  ASSERT_EQ(
      talos_sim_hardware_set_motor(hw, "roller", "wheel", 3, 2, 0, 0, 0, 0),
      TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_gyro(hw, "roller", "gyro", 0.75, -0.5, 0, 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 30 * kMs), TALOS_OK);
  ASSERT_EQ(talos_motor_rezero(node, roller.wheel, 1), TALOS_OK);
  TalosSample sample{};
  ASSERT_EQ(talos_gyro_rezero(node, gyro, 0.25), TALOS_OK);
  ASSERT_EQ(talos_motor_sample(node, roller.wheel, &sample), TALOS_OK);
  EXPECT_DOUBLE_EQ(sample.position, 1);
  EXPECT_DOUBLE_EQ(sample.velocity, 2);
  EXPECT_EQ(talos_gyro_rezero(node, roller.wheel, 0), TALOS_ERR);
  ASSERT_EQ(talos_sensor_sample(node, gyro, &sample), TALOS_OK);
  EXPECT_DOUBLE_EQ(sample.position, 0.25);
  EXPECT_DOUBLE_EQ(sample.velocity, -0.5);
  EXPECT_EQ(talos_motor_rezero(node, -1, 0), TALOS_ERR);
  talos_sim_hardware_destroy(hw);
}

TEST_F(NodeApi, DevicesCommandOnlyWhenEnabled) {
  Roller roller;
  TalosNode* node = talos_sim_add(sim_, "roller", &BuildRoller, &roller);
  ASSERT_NE(node, nullptr) << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(
      talos_sim_hardware_set_motor(hw, "roller", "wheel", 3.0, 1.5, 0, 0, 0, 0),
      TALOS_OK);

  ASSERT_EQ(talos_sim_step(sim_, 30 * kMs), TALOS_OK) << talos_last_error();
  TalosMotorRequest request{};
  ASSERT_EQ(talos_sim_hardware_motor_request(hw, "roller", "wheel", &request),
            TALOS_OK);
  EXPECT_EQ(roller.enabled, 0);
  EXPECT_EQ(request.mode, TALOS_MODE_NEUTRAL);

  TalosSample wheel{};
  ASSERT_EQ(talos_motor_sample(node, roller.wheel, &wheel), TALOS_OK);
  EXPECT_EQ(wheel.valid, 1);
  EXPECT_DOUBLE_EQ(wheel.position, 3.0);
  EXPECT_DOUBLE_EQ(wheel.velocity, 1.5);

  ASSERT_EQ(talos_sim_hardware_enable(hw, 1), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 30 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_motor_request(hw, "roller", "wheel", &request),
            TALOS_OK);
  EXPECT_EQ(roller.enabled, 1);
  EXPECT_EQ(request.mode, TALOS_MODE_VELOCITY);
  EXPECT_DOUBLE_EQ(request.demand, 12.0);
  EXPECT_DOUBLE_EQ(request.ff_volts, 0.5);
  EXPECT_EQ(request.slot, 1);

  ASSERT_EQ(talos_sim_hardware_set_digital_input(hw, "roller", "beam", 1),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 30 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_motor_request(hw, "roller", "wheel", &request),
            TALOS_OK);
  EXPECT_EQ(request.mode, TALOS_MODE_NEUTRAL);
  talos_sim_hardware_destroy(hw);
}

TEST_F(NodeApi, UnknownDeviceIsNamed) {
  auto build = [](TalosNode* node, void*) -> int32_t {
    return talos_device(node, TALOS_DEVICE_MOTOR, "flywheel") < 0 ? 0 : 1;
  };
  ASSERT_NE(talos_sim_add(sim_, "roller", build, nullptr), nullptr);
  TalosNode* node = talos_sim_node(sim_, "roller");
  ASSERT_NE(node, nullptr);
  EXPECT_LT(talos_device(node, TALOS_DEVICE_MOTOR, "flywheel"), 0);
  EXPECT_NE(std::string{talos_last_error()}.find("'flywheel'"),
            std::string::npos);
}

TEST_F(NodeApi, DisconnectedDeviceSampleIsInvalid) {
  Roller roller;
  TalosNode* node = talos_sim_add(sim_, "roller", &BuildRoller, &roller);
  ASSERT_NE(node, nullptr) << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(
      talos_sim_hardware_set_motor(hw, "roller", "wheel", 3.0, 1.5, 0, 0, 0, 0),
      TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_connected(hw, "roller", "wheel", 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 15 * kMs), TALOS_OK) << talos_last_error();

  TalosSample wheel{};
  ASSERT_EQ(talos_motor_sample(node, roller.wheel, &wheel), TALOS_OK);
  EXPECT_EQ(wheel.valid, 0);
  EXPECT_EQ(wheel.position, 0.0);
  TalosSample beam{};
  ASSERT_EQ(talos_digital_input_sample(node, roller.beam, &beam), TALOS_OK);
  EXPECT_EQ(beam.valid, 1);

  EXPECT_EQ(talos_sim_hardware_set_connected(hw, "roller", "flywheel", 0),
            TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("'flywheel'"),
            std::string::npos);
  talos_sim_hardware_destroy(hw);
}

TEST_F(NodeApi, HardwareSampleTimeOnceTheClockMappingConverges) {
  Roller roller;
  TalosNode* node = talos_sim_add(sim_, "roller", &BuildRoller, &roller);
  ASSERT_NE(node, nullptr) << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  const auto gyro = talos_device(node, TALOS_DEVICE_GYRO, "gyro");
  ASSERT_GE(gyro, 0);
  ASSERT_EQ(talos_sim_hardware_set_motor(hw, "roller", "wheel", 3, 2, 1000,
                                         3000, 0, 0),
            TALOS_OK);
  ASSERT_EQ(
      talos_sim_hardware_set_gyro(hw, "roller", "gyro", 0.75, -0.5, 5000, 6000),
      TALOS_OK);
  // The gateway clock is 3 s ahead of sim time.
  ASSERT_EQ(talos_sim_hardware_set_gateway_clock(hw, 3000 * kMs, 0), TALOS_OK);
  EXPECT_EQ(talos_sim_hardware_set_gateway_clock(hw, 0, -1), TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("latency"), std::string::npos);

  // States every 10 ms from 0: at most 6 by 50 ms, short of the 8 the
  // bridge's filter needs.
  ASSERT_EQ(talos_sim_step(sim_, 50 * kMs), TALOS_OK) << talos_last_error();
  int64_t sampled = -1;
  EXPECT_EQ(talos_hardware_sample_time(node, &sampled), 0);
  EXPECT_EQ(sampled, -1);
  int64_t acquired = -1;
  EXPECT_EQ(talos_motor_time(node, roller.wheel, 0, &acquired), 0);
  EXPECT_EQ(acquired, -1);

  ASSERT_EQ(talos_sim_step(sim_, 50 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_hardware_sample_time(node, &sampled), 1);
  // A publication time in sim time, not the gateway clock 3 s off it.
  EXPECT_EQ(sampled % (10 * kMs), 0);
  EXPECT_LE(sampled, talos_sim_now_ns(sim_));
  EXPECT_GE(sampled, talos_sim_now_ns(sim_) - 20 * kMs);
  EXPECT_EQ(roller.enabled, 0);  // independent of enable
  EXPECT_EQ(talos_hardware_sample_time(node, nullptr), TALOS_ERR);
  ASSERT_EQ(talos_motor_time(node, roller.wheel, 0, &acquired), 1);
  EXPECT_EQ(acquired, sampled - kMs);
  ASSERT_EQ(talos_motor_time(node, roller.wheel, 1, &acquired), 1);
  EXPECT_EQ(acquired, sampled - 3 * kMs);
  ASSERT_EQ(talos_sensor_time(node, gyro, 0, &acquired), 1);
  EXPECT_EQ(acquired, sampled - 5 * kMs);
  ASSERT_EQ(talos_sensor_time(node, gyro, 1, &acquired), 1);
  EXPECT_EQ(acquired, sampled - 6 * kMs);
  EXPECT_EQ(talos_motor_time(node, roller.wheel, 0, nullptr), TALOS_ERR);
  EXPECT_EQ(talos_sensor_time(node, gyro, 0, nullptr), TALOS_ERR);
  EXPECT_EQ(talos_motor_time(node, -1, 0, &acquired), TALOS_ERR);

  talos_sim_hardware_destroy(hw);  // the bridge goes silent
  ASSERT_EQ(talos_sim_step(sim_, 150 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(talos_hardware_sample_time(node, &sampled), 0);
  EXPECT_EQ(talos_motor_time(node, roller.wheel, 0, &acquired), 0);
  EXPECT_EQ(talos_sensor_time(node, gyro, 0, &acquired), 0);
}

TEST_F(NodeApi, HardwareFreshnessAndAge) {
  Roller roller;
  TalosNode* node = talos_sim_add(sim_, "roller", &BuildRoller, &roller);
  ASSERT_NE(node, nullptr) << talos_last_error();
  int64_t age = -1;
  EXPECT_EQ(talos_hardware_fresh(node, -1), 0);
  EXPECT_EQ(talos_hardware_fresh(node, INT64_MAX), 0);
  EXPECT_EQ(talos_hardware_age(node, &age), 0);
  EXPECT_EQ(age, -1);
  EXPECT_EQ(talos_hardware_age(node, nullptr), TALOS_ERR);
  EXPECT_EQ(talos_hardware_age(nullptr, &age), TALOS_ERR);
  EXPECT_EQ(talos_hardware_fresh(nullptr, -1), TALOS_ERR);

  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_step(sim_, 30 * kMs), TALOS_OK);
  EXPECT_EQ(talos_robot_enabled(node), 0);
  EXPECT_EQ(talos_hardware_fresh(node, -1), 1);
  ASSERT_EQ(talos_hardware_age(node, &age), 1);
  EXPECT_GE(age, 0);
  EXPECT_LT(age, 100 * kMs);
  EXPECT_EQ(talos_hardware_fresh(node, 0), 0);
  EXPECT_EQ(talos_hardware_fresh(node, age), 0);
  EXPECT_EQ(talos_hardware_fresh(node, age + 1), 1);

  talos_sim_hardware_destroy(hw);
  ASSERT_EQ(talos_sim_step(sim_, 150 * kMs), TALOS_OK);
  EXPECT_EQ(talos_hardware_fresh(node, -1), 0);
  EXPECT_EQ(talos_hardware_fresh(node, 1000 * kMs), 1);
  ASSERT_EQ(talos_hardware_age(node, &age), 1);
  // A queued final state may drain after the hardware publisher is gone.
  EXPECT_GE(age, 100 * kMs);
}

// --- Driver Station ------------------------------------------------------

int32_t BuildPilot(TalosNode* node, void* user) {
  auto count = [](void* packets) -> int32_t {
    ++*static_cast<int*>(packets);
    return 0;
  };
  return talos_driver_station(node, count, user);
}

TEST_F(NodeApi, DriverStationReadsWhatSimHardwareForwards) {
  int packets = 0;
  TalosNode* node = talos_sim_add(sim_, "relay", &BuildPilot, &packets);
  ASSERT_NE(node, nullptr) << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();

  TalosDriverStationPacket packet{};
  packet.flags = TALOS_DS_ATTACHED | TALOS_DS_ENABLED | TALOS_DS_AUTONOMOUS;
  packet.alliance = 1;  // red
  packet.station = 2;
  packet.match_time_s = 7.5f;
  TalosJoystick& stick = packet.joysticks[0];
  stick.connected = 1;
  stick.axis_count = 2;
  stick.axes[0] = 0.25f;
  stick.axes[1] = -0.5f;
  stick.axes[2] = 0.75f;  // past axis_count
  stick.button_count = 3;
  stick.buttons = 0b101;
  stick.pov_count = 1;
  stick.povs[0] = 180;
  std::strncpy(stick.name, "pad", sizeof stick.name);
  ASSERT_EQ(talos_sim_hardware_set_driver_station(hw, &packet), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 25 * kMs), TALOS_OK) << talos_last_error();

  EXPECT_GT(packets, 0);
  TalosDriverStation ds{};
  ASSERT_EQ(talos_driver_station_read(node, &ds), TALOS_OK);
  EXPECT_EQ(ds.connected, 1);
  EXPECT_EQ(ds.enabled, 1);
  EXPECT_EQ(ds.autonomous, 1);
  EXPECT_EQ(ds.teleop, 0);
  EXPECT_EQ(ds.alliance, 1);
  EXPECT_EQ(ds.station, 2);
  EXPECT_DOUBLE_EQ(ds.match_time_s, 7.5);
  const TalosJoystick& read = ds.joysticks[0];
  EXPECT_EQ(read.connected, 1);
  EXPECT_EQ(read.axis_count, 2);
  EXPECT_FLOAT_EQ(read.axes[1], -0.5f);
  EXPECT_EQ(read.axes[2], 0.0f);  // as Joystick::axis reads it
  EXPECT_EQ(read.buttons, 0b101u);
  EXPECT_EQ(read.povs[0], 180);
  EXPECT_EQ(read.povs[1], -1);
  EXPECT_STREQ(read.name, "pad");
  EXPECT_EQ(ds.joysticks[1].connected, 0);

  EXPECT_EQ(talos_driver_station_read(node, nullptr), TALOS_ERR);
  talos_sim_hardware_destroy(hw);
}

struct Lanes {
  int64_t target = -1;
  int64_t lane = -1;
};

int32_t BuildLanes(TalosNode* node, void* user) {
  auto& l = *static_cast<Lanes*>(user);
  l.target =
      talos_target_open(node, sizeof(float), alignof(float), &kFloatSchema);
  // The roller writes its own subsystem's lane only in this test; the lint,
  // not the API, refuses that in a session.
  l.lane = talos_target_lane_open(node, "roller", "teleop", sizeof(float),
                                  alignof(float), &kFloatSchema);
  return l.target < 0 || l.lane < 0 ? 1 : 0;
}

TEST_F(NodeApi, TargetSelectsTheTeleopLaneAndLanesRelease) {
  Lanes lanes;
  TalosNode* node = talos_sim_add(sim_, "roller", &BuildLanes, &lanes);
  ASSERT_NE(node, nullptr) << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_enable(hw, 1), TALOS_OK);
  TalosDriverStationPacket packet{};
  packet.flags = TALOS_DS_ATTACHED | TALOS_DS_ENABLED | TALOS_DS_TELEOP;
  ASSERT_EQ(talos_sim_hardware_set_driver_station(hw, &packet), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();

  float out = 0;
  int32_t source = 7;
  EXPECT_EQ(talos_target_get(node, lanes.target, &out, &source), 0);
  EXPECT_EQ(source, -1);

  const float speed = 12.5f;
  ASSERT_EQ(talos_target_lane_send(node, lanes.lane, &speed, sizeof speed), 1);
  ASSERT_EQ(talos_sim_step(sim_, 10 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_target_get(node, lanes.target, &out, &source), 1);
  EXPECT_FLOAT_EQ(out, 12.5f);
  EXPECT_STREQ(talos_target_source_name(node, lanes.target, source), "teleop");
  EXPECT_EQ(talos_target_get(node, lanes.target, nullptr, nullptr), 1);

  float latest = 0;
  int32_t active = -1;
  ASSERT_EQ(talos_sim_latest_target(sim_, "roller", "teleop", &latest,
                                    sizeof latest, alignof(float), &active),
            1);
  EXPECT_EQ(active, 1);
  EXPECT_FLOAT_EQ(latest, 12.5f);

  ASSERT_EQ(talos_target_lane_release(node, lanes.lane), 1);
  ASSERT_EQ(talos_sim_step(sim_, 10 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(talos_target_get(node, lanes.target, &out, &source), 0);

  // The harness writes the same envelope, stamped with the generation of the
  // Driver Station packet it forwarded last...
  const uint64_t generation = talos_sim_driver_station_generation(sim_);
  ASSERT_NE(generation, 0u);
  const float other = 3.0f;
  ASSERT_EQ(talos_sim_publish_target(sim_, "roller", "teleop", &other,
                                     sizeof other, alignof(float), nullptr),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 10 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_target_get(node, lanes.target, &out, &source), 1);
  EXPECT_FLOAT_EQ(out, 3.0f);
  // ... or with an explicit one: another generation never drives.
  const uint64_t earlier = generation - 1;
  ASSERT_EQ(talos_sim_publish_target(sim_, "roller", "teleop", &other,
                                     sizeof other, alignof(float), &earlier),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 10 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(talos_target_get(node, lanes.target, &out, &source), 0);

  // Wrong sizes and alignments are refused, naming the problem.
  EXPECT_EQ(talos_target_lane_send(node, lanes.lane, &speed, 8), TALOS_ERR);
  EXPECT_EQ(talos_target_open(node, 12, 8, &kFloatSchema), -1);
  EXPECT_NE(std::string{talos_last_error()}.find("alignment"),
            std::string::npos);
  EXPECT_EQ(talos_target_source_name(node, lanes.target, 5), nullptr);

  // A silent Driver Station selects nothing.
  ASSERT_EQ(talos_sim_hardware_set_driver_station(hw, nullptr), TALOS_OK);
  ASSERT_EQ(talos_target_lane_send(node, lanes.lane, &speed, sizeof speed), 1);
  ASSERT_EQ(talos_sim_step(sim_, 150 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(talos_target_get(node, lanes.target, &out, &source), 0);
  talos_sim_hardware_destroy(hw);
}

}  // namespace
