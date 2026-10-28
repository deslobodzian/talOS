// The C ABI drives the same Node, devices and Sim a C++ node does: these
// tests build nodes purely through it, the way the Python binding does.

#include "talOS/node_api/node_api.h"

#include <gtest/gtest.h>

#include <cstdint>
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

[subsystems.roller.motors.wheel]
can_id = 1
[subsystems.roller.digital_inputs.beam]
dio = 0
)";

struct Value {
  double x;
  int64_t stamp;
};

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
  r.in = talos_node_subscribe(node, "/relay/in", sizeof(Value), 0,
                              &RelayMessage, &r);
  r.out = talos_node_publish(node, "/relay/out", sizeof(Value));
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

TEST_F(NodeApi, DevicesCommandOnlyWhenEnabled) {
  Roller roller;
  TalosNode* node = talos_sim_add(sim_, "roller", &BuildRoller, &roller);
  ASSERT_NE(node, nullptr) << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_set_motor(hw, "roller", "wheel", 3.0, 1.5),
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

}  // namespace
