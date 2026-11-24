// The C ABI drives the same Node, devices and Sim a C++ node does: these
// tests build nodes purely through it, the way the Python binding does.

#include "talOS/node_api/node_api.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "control/control_generated.h"

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
[subsystems.roller.pwm_outputs.servo]
channel = 0
[subsystems.roller.sensors.gyro]
type = "Pigeon2"
can_id = 2
[subsystems.roller.sensors.cc]
type = "CANcoder"
can_id = 3
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
                              0, TALOS_WAKE_ON_PUBLISH, &RelayMessage, &r);
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

// A second node on /relay/out with a same-size type of another schema.
struct Impostor {
  int64_t in = -1;
};

int32_t BuildImpostor(TalosNode* node, void* user) {
  constexpr TalosSchema kOther{"node_api_test.Other", 0xbadULL, nullptr, 0};
  auto& i = *static_cast<Impostor*>(user);
  i.in = talos_node_subscribe(node, "/relay/out", sizeof(Value), &kOther, 0,
                              TALOS_WAKE_ON_PUBLISH, nullptr, nullptr);
  return i.in < 0 ? 1 : 0;
}

// Records the node time of each /waiter/in message its at-drain callback
// hears, and what a subscription with an undefined wake returned.
struct Waiter {
  TalosNode* node = nullptr;
  std::vector<int64_t> heard_ns;
  int64_t refused = 0;
  std::string refusal;
};

int32_t WaiterMessage(const void* /*data*/, uint32_t /*size*/, void* user) {
  auto& w = *static_cast<Waiter*>(user);
  w.heard_ns.push_back(talos_node_now_ns(w.node));
  return 0;
}

int32_t WaiterUpdate(void* /*user*/) { return 0; }

int32_t BuildWaiter(TalosNode* node, void* user) {
  auto& w = *static_cast<Waiter*>(user);
  w.node = node;
  w.refused = talos_node_subscribe(node, "/waiter/other", sizeof(Value),
                                   &kValueSchema, 0, 2, &WaiterMessage, &w);
  w.refusal = talos_last_error();
  if (talos_node_subscribe(node, "/waiter/in", sizeof(Value), &kValueSchema, 0,
                           TALOS_WAKE_AT_DRAIN, &WaiterMessage, &w) < 0) {
    return 1;
  }
  return talos_node_set_update(node, &WaiterUpdate, &w);
}

// TALOS_WAKE_AT_DRAIN keeps the callback but wakes nothing: a message
// published between ticks reaches it at the next tick's drain (10 ms), not
// at its publication (2 ms). A wake the enum lacks is refused, naming it.
// REQ-RT-19
TEST_F(NodeApi, AnAtDrainCallbackHearsAMessageAtTheNextTick) {
  Waiter waiter;
  ASSERT_NE(talos_sim_add(sim_, "waiter", &BuildWaiter, &waiter), nullptr)
      << talos_last_error();
  EXPECT_EQ(waiter.refused, -1);
  EXPECT_NE(waiter.refusal.find("wake"), std::string::npos) << waiter.refusal;

  ASSERT_EQ(talos_sim_step(sim_, 2 * kMs), TALOS_OK) << talos_last_error();
  const Value in{1.0, 0};
  ASSERT_EQ(talos_sim_publish_raw(sim_, "/waiter/in", &in, sizeof in),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 7 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_TRUE(waiter.heard_ns.empty()) << "the publication woke the node";
  ASSERT_EQ(talos_sim_step(sim_, 2 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(waiter.heard_ns, std::vector<int64_t>{10 * kMs});
}

// REQ-RT-19: TALOS_WAKE_ON_PUBLISH on a timed node (10 ms update()) wakes the
// node at publication time (2 ms), before the 10 ms tick, without running
// update() early.
TEST_F(NodeApi, AnOnPublishCallbackWakesATimedNodeBeforeItsTick) {
  struct Prompt {
    TalosNode* node = nullptr;
    std::vector<int64_t> heard_ns;
    int updates = 0;
  };
  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& p = *static_cast<Prompt*>(user);
    p.node = node;
    const auto on_msg = [](const void* /*data*/, uint32_t /*size*/,
                           void* ctx) -> int32_t {
      auto& self = *static_cast<Prompt*>(ctx);
      self.heard_ns.push_back(talos_node_now_ns(self.node));
      return 0;
    };
    const auto on_update = [](void* ctx) -> int32_t {
      ++static_cast<Prompt*>(ctx)->updates;
      return 0;
    };
    if (talos_node_subscribe(node, "/prompt/in", sizeof(Value), &kValueSchema,
                             0, TALOS_WAKE_ON_PUBLISH, on_msg, &p) < 0) {
      return 1;
    }
    return talos_node_set_update(node, on_update, &p);
  };

  Prompt prompt;
  ASSERT_NE(talos_sim_add(sim_, "prompt", build, &prompt), nullptr)
      << talos_last_error();
  ASSERT_EQ(talos_sim_step(sim_, 2 * kMs), TALOS_OK) << talos_last_error();
  const Value in{1.0, 0};
  ASSERT_EQ(talos_sim_publish_raw(sim_, "/prompt/in", &in, sizeof in),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 1 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(prompt.heard_ns, std::vector<int64_t>{2 * kMs});
  EXPECT_EQ(prompt.updates, 0);

  ASSERT_EQ(talos_sim_step(sim_, 8 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(prompt.updates, 1);
  EXPECT_EQ(prompt.heard_ns, std::vector<int64_t>{2 * kMs});
}

TEST_F(NodeApi, ATopicRefusesAnEndpointOfAnotherSchema) {
  Relay relay;
  ASSERT_NE(talos_sim_add(sim_, "relay", &BuildRelay, &relay), nullptr);
  Impostor impostor;
  EXPECT_EQ(talos_sim_add(sim_, "impostor", &BuildImpostor, &impostor),
            nullptr);
  EXPECT_EQ(talos_node_publish(nullptr, "/x/state", 8, nullptr), -1);
}

// REQ-CONTRACT-13: Schema accepts bfbs == nullptr with bfbs_size == 0 without
// constructing std::string(nullptr, 0) (UB), accepts non-empty bfbs, and
// refuses null schema, null name, or null bfbs with nonzero size.
TEST_F(NodeApi, SchemaValidatesNullFieldsAndEmptyBfbs) {
  struct Probe {
    int64_t null_schema = 0;
    int64_t null_name = 0;
    int64_t null_bfbs_nonzero = 0;
    std::string null_bfbs_error;
    int64_t empty_bfbs = -1;
    int64_t non_empty_bfbs = -1;
  };
  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& p = *static_cast<Probe*>(user);
    p.null_schema =
        talos_node_publish(node, "/probe/a", sizeof(Value), nullptr);
    constexpr TalosSchema kNullName{nullptr, 1u, nullptr, 0};
    p.null_name =
        talos_node_publish(node, "/probe/b", sizeof(Value), &kNullName);
    constexpr TalosSchema kNullBfbsNonZero{"node_api_test.Bad", 1u, nullptr, 4};
    p.null_bfbs_nonzero =
        talos_node_publish(node, "/probe/c", sizeof(Value), &kNullBfbsNonZero);
    p.null_bfbs_error = talos_last_error();
    constexpr TalosSchema kEmptyBfbs{"node_api_test.Empty", 2u, nullptr, 0};
    p.empty_bfbs =
        talos_node_publish(node, "/probe/d", sizeof(Value), &kEmptyBfbs);
    static constexpr uint8_t kBfbsBytes[4] = {'B', 'F', 'B', 'S'};
    constexpr TalosSchema kWithBfbs{"node_api_test.WithBfbs", 3u, kBfbsBytes,
                                    sizeof kBfbsBytes};
    p.non_empty_bfbs =
        talos_node_publish(node, "/probe/e", sizeof(Value), &kWithBfbs);
    return 0;
  };
  Probe probe;
  ASSERT_NE(talos_sim_add(sim_, "probe", build, &probe), nullptr)
      << talos_last_error();
  EXPECT_EQ(probe.null_schema, -1);
  EXPECT_EQ(probe.null_name, -1);
  EXPECT_EQ(probe.null_bfbs_nonzero, -1);
  EXPECT_NE(probe.null_bfbs_error.find("bfbs"), std::string::npos)
      << probe.null_bfbs_error;
  EXPECT_GE(probe.empty_bfbs, 0);
  EXPECT_GE(probe.non_empty_bfbs, 0);
}

TEST_F(NodeApi, ParamsRejectMissingAndMistypedValues) {
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
  EXPECT_EQ(talos_node_param(node, "label", TALOS_PARAM_BOOL, 0, &p),
            TALOS_ERR);
}

// A path that does not fit with its NUL is refused, never truncated, so a
// binding cannot open a prefix of the configured file.
// REQ-CONTRACT-14 REQ-CONTRACT-31
TEST(NodeApiTrajectory, PathCopiesWholeOrNotAtAll) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  ASSERT_NE(tmp, nullptr);
  const std::string traj = std::string{tmp} + "/two_piece.traj";
  std::ofstream{traj} << "{}\n";
  const std::string toml = std::string{kRobotToml} +
                           "[subsystems.relay.trajectories]\ntwo_piece = \"" +
                           traj + "\"\n";
  TalosSim* sim = talos_sim_from_toml(toml.c_str());
  ASSERT_NE(sim, nullptr) << talos_last_error();
  Relay relay;
  TalosNode* node = talos_sim_add(sim, "relay", &BuildRelay, &relay);
  ASSERT_NE(node, nullptr) << talos_last_error();

  std::vector<char> out(traj.size() + 1, 'x');
  EXPECT_EQ(talos_node_trajectory_path(node, "two_piece", out.data(),
                                       static_cast<uint32_t>(traj.size())),
            TALOS_ERR);
  EXPECT_EQ(std::string(out.begin(), out.end()), std::string(out.size(), 'x'));
  ASSERT_EQ(talos_node_trajectory_path(node, "two_piece", out.data(),
                                       static_cast<uint32_t>(out.size())),
            TALOS_OK)
      << talos_last_error();
  EXPECT_STREQ(out.data(), traj.c_str());
  talos_sim_destroy(sim);
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
  EXPECT_EQ(talos_node_param(node, "cameras", TALOS_PARAM_DOUBLE_LIST, 0, &p),
            TALOS_ERR);
  EXPECT_EQ(talos_node_param(node, "label", TALOS_PARAM_LIST, 0, &p),
            TALOS_ERR);
}

TEST_F(NodeApi, FailingCallbackStopsTheStep) {
  Relay relay;
  ASSERT_NE(talos_sim_add(sim_, "relay", &BuildRelay, &relay), nullptr);
  relay.fail_update = true;
  EXPECT_EQ(talos_sim_step(sim_, 15 * kMs), TALOS_ERR);
}

TEST_F(NodeApi, RegistrationFailureFailsTheAdd) {
  Relay relay;
  auto build = [](TalosNode*, void*) -> int32_t { return 1; };
  EXPECT_EQ(talos_sim_add(sim_, "relay", build, &relay), nullptr);
}

// REQ-SAFE-27: talos_node_publish and talos_node_publish_bytes refuse any
// /hw/request topic at construction, direct or remapped.
TEST(NodeApiHardwareRequest, PublishAndPublishBytesRefuseHardwareRequestTopic) {
  TalosSim* sim = talos_sim_from_toml(R"(
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.plain]
[subsystems.bytes]
[subsystems.remapped.remap]
state = "/hw/request/remapped"
[subsystems.remapped_bytes.remap]
frames = "/hw/request/remapped_bytes"

[subsystems.roller.motors.wheel]
can_id = 1
)");
  ASSERT_NE(sim, nullptr) << talos_last_error();

  auto build_plain = [](TalosNode* node, void*) -> int32_t {
    return talos_node_publish(node, "/hw/request/plain", sizeof(Value),
                              &kValueSchema) < 0
               ? 1
               : 0;
  };
  auto build_bytes = [](TalosNode* node, void*) -> int32_t {
    return talos_node_publish_bytes(node, "/hw/request/bytes", 64, 16, 1, 0) < 0
               ? 1
               : 0;
  };
  auto build_remapped = [](TalosNode* node, void*) -> int32_t {
    return talos_node_publish(node, "state", sizeof(Value), &kValueSchema) < 0
               ? 1
               : 0;
  };
  auto build_remapped_bytes = [](TalosNode* node, void*) -> int32_t {
    return talos_node_publish_bytes(node, "frames", 64, 16, 1, 0) < 0 ? 1 : 0;
  };

  for (const auto& [name, factory] :
       {std::pair<const char*, TalosFactory>{"plain", build_plain},
        std::pair<const char*, TalosFactory>{"bytes", build_bytes},
        std::pair<const char*, TalosFactory>{"remapped", build_remapped},
        std::pair<const char*, TalosFactory>{"remapped_bytes",
                                             build_remapped_bytes}}) {
    SCOPED_TRACE(name);
    EXPECT_EQ(talos_sim_add(sim, name, factory, nullptr), nullptr);
    const std::string err = talos_last_error();
    EXPECT_NE(err.find("/hw/request"), std::string::npos) << err;
  }
  talos_sim_destroy(sim);
}

TEST_F(NodeApi, RemoveAndMidRunAddRestartNodeAtCurrentSimTime) {
  Relay first;
  TalosNode* node = talos_sim_add(sim_, "relay", &BuildRelay, &first);
  ASSERT_NE(node, nullptr) << talos_last_error();
  const Value in1{4.0, 0};
  ASSERT_EQ(talos_sim_publish_raw(sim_, "/relay/in", &in1, sizeof in1),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 25 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_node_tick(node), 2u);
  ASSERT_EQ(first.callbacks, 1);

  EXPECT_EQ(talos_sim_remove(nullptr, "relay"), TALOS_ERR);
  EXPECT_EQ(talos_sim_remove(sim_, nullptr), TALOS_ERR);
  EXPECT_EQ(talos_sim_remove(sim_, "missing"), TALOS_ERR);
  ASSERT_EQ(talos_sim_remove(sim_, "relay"), TALOS_OK) << talos_last_error();
  EXPECT_EQ(talos_sim_remove(sim_, "relay"), TALOS_ERR);
  EXPECT_EQ(talos_sim_node(sim_, "relay"), nullptr);

  // Messages published while removed must not reach the old or new instance.
  const Value while_stopped{6.0, 0};
  ASSERT_EQ(talos_sim_publish_raw(sim_, "/relay/in", &while_stopped,
                                  sizeof while_stopped),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 10 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(first.callbacks, 1);
  ASSERT_EQ(talos_sim_now_ns(sim_), 35 * kMs);

  // Re-adding mid-run starts at sim now (35 ms) with no catch-up ticks and no
  // pre-attach messages.
  Relay second;
  TalosNode* restarted = talos_sim_add(sim_, "relay", &BuildRelay, &second);
  ASSERT_NE(restarted, nullptr) << talos_last_error();
  EXPECT_EQ(talos_node_now_ns(restarted), 35 * kMs);
  EXPECT_EQ(talos_node_tick(restarted), 0u);
  ASSERT_EQ(talos_sim_step(sim_, 10 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(talos_node_tick(restarted), 1u);
  EXPECT_EQ(second.callbacks, 0);
  EXPECT_EQ(talos_subscription_has_value(restarted, second.in), 0);

  // A new message published after attach drives the restarted node normally.
  const Value in2{8.0, 0};
  ASSERT_EQ(talos_sim_publish_raw(sim_, "/relay/in", &in2, sizeof in2),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 10 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(second.callbacks, 1);
  Value out{};
  ASSERT_EQ(talos_sim_latest_raw(sim_, "/relay/out", &out, sizeof out), 1);
  EXPECT_DOUBLE_EQ(out.x, 20.0);
  EXPECT_EQ(out.stamp, 55 * kMs);

  // Re-entrant remove/add from inside talos_sim_step returns TALOS_ERR.
  struct Reenter {
    TalosSim* sim = nullptr;
    int32_t remove_result = TALOS_OK;
    TalosNode* add_result = nullptr;
  } reenter{sim_};
  auto build_reenter = [](TalosNode* n, void* u) -> int32_t {
    return talos_node_set_update(
        n,
        [](void* ctx) -> int32_t {
          auto& r = *static_cast<Reenter*>(ctx);
          r.remove_result = talos_sim_remove(r.sim, "relay");
          r.add_result = talos_sim_add(
              r.sim, "extra", [](TalosNode*, void*) { return 0; }, nullptr);
          return 0;
        },
        u);
  };
  ASSERT_NE(talos_sim_add(sim_, "reenter", build_reenter, &reenter), nullptr)
      << talos_last_error();
  ASSERT_EQ(talos_sim_step(sim_, 10 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(reenter.remove_result, TALOS_ERR);
  EXPECT_EQ(reenter.add_result, nullptr);
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
  if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
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
  // REQ-SAFE-42: the opt-out is the factory's alone.
  EXPECT_EQ(talos_allow_untargeted_outputs(node), TALOS_ERR);
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_set_motor(hw, "roller", "wheel", 3.0, 1.5, 0, 0,
                                         0, 0, 0),
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
  EXPECT_EQ(wheel.forward_limit, 0);
  EXPECT_EQ(wheel.reverse_limit, 0);

  ASSERT_EQ(talos_sim_hardware_set_limit_switches(hw, "roller", "wheel", 1, 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_motor_sample(node, roller.wheel, &wheel), TALOS_OK);
  EXPECT_EQ(wheel.forward_limit, 1);
  EXPECT_EQ(wheel.reverse_limit, 0);

  ASSERT_EQ(talos_sim_hardware_set_limit_switches(hw, "roller", "wheel", 0, 1),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_motor_sample(node, roller.wheel, &wheel), TALOS_OK);
  EXPECT_EQ(wheel.forward_limit, 0);
  EXPECT_EQ(wheel.reverse_limit, 1);

  ASSERT_EQ(talos_sim_hardware_set_connected(hw, "roller", "wheel", 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_motor_sample(node, roller.wheel, &wheel), TALOS_OK);
  EXPECT_EQ(wheel.valid, 0);
  EXPECT_EQ(wheel.forward_limit, 0);
  EXPECT_EQ(wheel.reverse_limit, 0);

  ASSERT_EQ(talos_sim_hardware_set_connected(hw, "roller", "wheel", 1),
            TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_limit_switches(hw, "roller", "wheel", 0, 0),
            TALOS_OK);

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

// REQ-CONTRACT-34: before any valid sample, Motor, CANcoder, Encoder and Gyro
// read 0.0; across disconnect and stale link, each holds its last valid
// position/yaw through any rezero offset while rates and efforts read 0.0.
TEST(NodeApiDevices, InvalidAndStaleSamplesHoldPositionAndZeroRatesAndEfforts) {
  constexpr const char* kDevicesToml = R"(
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.mech.motors.wheel]
can_id = 1
[subsystems.mech.sensors.cc]
type = "CANcoder"
can_id = 2
[subsystems.mech.sensors.gyro]
type = "Pigeon2"
can_id = 3
[subsystems.mech.encoders.enc]
type = "DutyCycleEncoder"
dio = 2
)";
  struct Handles {
    int64_t wheel = -1;
    int64_t cc = -1;
    int64_t gyro = -1;
    int64_t enc = -1;
  };
  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& h = *static_cast<Handles*>(user);
    h.wheel = talos_device(node, TALOS_DEVICE_MOTOR, "wheel");
    h.cc = talos_device(node, TALOS_DEVICE_CANCODER, "cc");
    h.gyro = talos_device(node, TALOS_DEVICE_GYRO, "gyro");
    h.enc = talos_device(node, TALOS_DEVICE_ENCODER, "enc");
    if (h.wheel < 0 || h.cc < 0 || h.gyro < 0 || h.enc < 0) return 1;
    return 0;
  };

  TalosSim* sim = talos_sim_from_toml(kDevicesToml);
  ASSERT_NE(sim, nullptr) << talos_last_error();
  Handles h;
  TalosNode* node = talos_sim_add(sim, "mech", build, &h);
  ASSERT_NE(node, nullptr) << talos_last_error();

  // Before the first valid sample: valid == 0 and all reads are 0.0.
  ASSERT_EQ(talos_sim_step(sim, 20 * kMs), TALOS_OK) << talos_last_error();
  TalosSample wheel{}, cc{}, gyro{}, enc{};
  ASSERT_EQ(talos_motor_sample(node, h.wheel, &wheel), TALOS_OK);
  ASSERT_EQ(talos_sensor_sample(node, h.cc, &cc), TALOS_OK);
  ASSERT_EQ(talos_sensor_sample(node, h.gyro, &gyro), TALOS_OK);
  ASSERT_EQ(talos_encoder_sample(node, h.enc, &enc), TALOS_OK);
  for (const TalosSample* s : {&wheel, &cc, &gyro, &enc}) {
    EXPECT_EQ(s->valid, 0);
    EXPECT_EQ(s->position, 0.0);
    EXPECT_EQ(s->velocity, 0.0);
  }
  EXPECT_EQ(wheel.voltage, 0.0);
  EXPECT_EQ(wheel.current, 0.0);
  EXPECT_EQ(wheel.supply_current_a, 0.0);

  // Establish live samples and rezero Motor and Gyro.
  TalosSimHardware* hw = talos_sim_hardware_create(sim);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_set_motor(hw, "mech", "wheel", 3.0, 2.0, 0, 0,
                                         40.0, 18.0, 30.0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_cancoder(hw, "mech", "cc", 0.2, 0.3, 0, 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_gyro(hw, "mech", "gyro", 0.25, -0.5, 0, 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_encoder(hw, "mech", "enc", 0.4, 0.6),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim, 20 * kMs), TALOS_OK) << talos_last_error();

  // SimHardware::set_motor leaves MotorSample::voltage at 0.0; patch the
  // latest encoded /hw/state packet so the live sample carries a non-zero
  // voltage (11.5 V) before disconnect or staleness, proving voltage -> 0.0
  // is not tautological.
  auto publish_live_motor_voltage = [&](double expected_pos,
                                        double expected_vel, double voltage) {
    constexpr uint32_t kPacketSize = 1436;
    constexpr size_t kWire = 4;  // uint32_t size prefix in hardware::Packet
    std::vector<uint8_t> packet(kPacketSize);
    ASSERT_EQ(
        talos_sim_latest_raw(sim, "/hw/state", packet.data(), kPacketSize), 1);
    uint32_t wire_size = 0;
    std::memcpy(&wire_size, packet.data(), sizeof wire_size);
    ASSERT_GE(wire_size, 84u + 45u);
    ASSERT_LE(wire_size, kPacketSize - kWire);
    uint16_t motor_count = 0;
    std::memcpy(&motor_count, packet.data() + kWire + 70, sizeof motor_count);
    ASSERT_EQ(motor_count, 1u);
    ASSERT_EQ(packet[kWire + 84 + 2], 1u);  // motors[0].valid
    double pos = 0;
    double vel = 0;
    std::memcpy(&pos, packet.data() + kWire + 84 + 3, sizeof pos);
    std::memcpy(&vel, packet.data() + kWire + 84 + 11, sizeof vel);
    ASSERT_DOUBLE_EQ(pos, expected_pos);
    ASSERT_DOUBLE_EQ(vel, expected_vel);
    uint64_t sample_time_us = 0;
    uint64_t sequence = 0;
    std::memcpy(&sample_time_us, packet.data() + kWire + 24,
                sizeof sample_time_us);
    std::memcpy(&sequence, packet.data() + kWire + 48, sizeof sequence);
    sample_time_us += 1000;
    ++sequence;
    std::memcpy(packet.data() + kWire + 24, &sample_time_us,
                sizeof sample_time_us);
    std::memcpy(packet.data() + kWire + 48, &sequence, sizeof sequence);
    std::memcpy(packet.data() + kWire + 84 + 19, &voltage, sizeof voltage);
    ASSERT_EQ(talos_sim_hardware_lose_states(hw, 1), TALOS_OK)
        << talos_last_error();
    ASSERT_EQ(
        talos_sim_publish_raw(sim, "/hw/state", packet.data(), kPacketSize),
        TALOS_OK)
        << talos_last_error();
    ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();
  };

  ASSERT_EQ(talos_motor_rezero(node, h.wheel, 1.0), TALOS_OK);
  ASSERT_EQ(talos_gyro_rezero(node, h.gyro, 0.0), TALOS_OK);
  ASSERT_EQ(talos_motor_zeroed(node, h.wheel), 1);
  ASSERT_EQ(talos_gyro_zeroed(node, h.gyro), 1);
  publish_live_motor_voltage(3.0, 2.0, 11.5);

  ASSERT_EQ(talos_motor_sample(node, h.wheel, &wheel), TALOS_OK);
  ASSERT_EQ(talos_sensor_sample(node, h.cc, &cc), TALOS_OK);
  ASSERT_EQ(talos_sensor_sample(node, h.gyro, &gyro), TALOS_OK);
  ASSERT_EQ(talos_encoder_sample(node, h.enc, &enc), TALOS_OK);
  EXPECT_EQ(wheel.valid, 1);
  EXPECT_DOUBLE_EQ(wheel.position, 1.0);
  EXPECT_DOUBLE_EQ(wheel.velocity, 2.0);
  EXPECT_DOUBLE_EQ(wheel.voltage, 11.5);
  EXPECT_DOUBLE_EQ(wheel.current, 30.0);
  EXPECT_DOUBLE_EQ(wheel.supply_current_a, 18.0);
  EXPECT_EQ(cc.valid, 1);
  EXPECT_DOUBLE_EQ(cc.position, 0.2);
  EXPECT_DOUBLE_EQ(cc.velocity, 0.3);
  EXPECT_EQ(gyro.valid, 1);
  EXPECT_DOUBLE_EQ(gyro.position, 0.0);
  EXPECT_DOUBLE_EQ(gyro.velocity, -0.5);
  EXPECT_EQ(enc.valid, 1);
  EXPECT_DOUBLE_EQ(enc.position, 0.4);
  EXPECT_DOUBLE_EQ(enc.velocity, 0.6);

  // Disconnect every device: positions hold through rezero offsets; rates and
  // efforts read 0.0.
  for (const char* dev : {"wheel", "cc", "gyro", "enc"}) {
    ASSERT_EQ(talos_sim_hardware_set_connected(hw, "mech", dev, 0), TALOS_OK);
  }
  ASSERT_EQ(talos_sim_step(sim, 20 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_motor_sample(node, h.wheel, &wheel), TALOS_OK);
  ASSERT_EQ(talos_sensor_sample(node, h.cc, &cc), TALOS_OK);
  ASSERT_EQ(talos_sensor_sample(node, h.gyro, &gyro), TALOS_OK);
  ASSERT_EQ(talos_encoder_sample(node, h.enc, &enc), TALOS_OK);
  EXPECT_EQ(wheel.valid, 0);
  EXPECT_EQ(cc.valid, 0);
  EXPECT_EQ(gyro.valid, 0);
  EXPECT_EQ(enc.valid, 0);
  EXPECT_DOUBLE_EQ(wheel.position, 1.0);
  EXPECT_DOUBLE_EQ(gyro.position, 0.0);
  EXPECT_DOUBLE_EQ(cc.position, 0.2);
  EXPECT_DOUBLE_EQ(enc.position, 0.4);
  EXPECT_EQ(wheel.velocity, 0.0);
  EXPECT_EQ(wheel.voltage, 0.0);
  EXPECT_EQ(wheel.current, 0.0);
  EXPECT_EQ(wheel.supply_current_a, 0.0);
  EXPECT_EQ(cc.velocity, 0.0);
  EXPECT_EQ(gyro.velocity, 0.0);
  EXPECT_EQ(enc.velocity, 0.0);

  // Reconnect with updated live readings, then drop the hardware link past
  // command_timeout_us (stale link).
  ASSERT_EQ(talos_sim_hardware_set_motor(hw, "mech", "wheel", 5.0, 2.0, 0, 0,
                                         45.0, 18.0, 30.0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_cancoder(hw, "mech", "cc", 0.35, 0.3, 0, 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_gyro(hw, "mech", "gyro", 0.5, -0.5, 0, 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_encoder(hw, "mech", "enc", 0.55, 0.6),
            TALOS_OK);
  for (const char* dev : {"wheel", "cc", "gyro", "enc"}) {
    ASSERT_EQ(talos_sim_hardware_set_connected(hw, "mech", dev, 1), TALOS_OK);
  }
  ASSERT_EQ(talos_sim_step(sim, 20 * kMs), TALOS_OK) << talos_last_error();
  publish_live_motor_voltage(5.0, 2.0, 11.5);
  ASSERT_EQ(talos_motor_sample(node, h.wheel, &wheel), TALOS_OK);
  ASSERT_EQ(talos_sensor_sample(node, h.gyro, &gyro), TALOS_OK);
  ASSERT_DOUBLE_EQ(wheel.position, 3.0);
  ASSERT_DOUBLE_EQ(wheel.voltage, 11.5);
  ASSERT_DOUBLE_EQ(gyro.position, 0.25);

  talos_sim_hardware_destroy(hw);
  ASSERT_EQ(talos_sim_step(sim, 150 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_motor_sample(node, h.wheel, &wheel), TALOS_OK);
  ASSERT_EQ(talos_sensor_sample(node, h.cc, &cc), TALOS_OK);
  ASSERT_EQ(talos_sensor_sample(node, h.gyro, &gyro), TALOS_OK);
  ASSERT_EQ(talos_encoder_sample(node, h.enc, &enc), TALOS_OK);
  EXPECT_EQ(wheel.valid, 0);
  EXPECT_EQ(cc.valid, 0);
  EXPECT_EQ(gyro.valid, 0);
  EXPECT_EQ(enc.valid, 0);
  EXPECT_DOUBLE_EQ(wheel.position, 3.0);
  EXPECT_DOUBLE_EQ(gyro.position, 0.25);
  EXPECT_DOUBLE_EQ(cc.position, 0.35);
  EXPECT_DOUBLE_EQ(enc.position, 0.55);
  EXPECT_EQ(wheel.velocity, 0.0);
  EXPECT_EQ(wheel.voltage, 0.0);
  EXPECT_EQ(wheel.current, 0.0);
  EXPECT_EQ(wheel.supply_current_a, 0.0);
  EXPECT_EQ(cc.velocity, 0.0);
  EXPECT_EQ(gyro.velocity, 0.0);
  EXPECT_EQ(enc.velocity, 0.0);

  talos_sim_destroy(sim);
}

// A PWM output set to 0 pulses at the centre; after talos_pwm_stop it sends
// no pulse, and the inspection calls tell the two apart. Like every setter,
// talos_pwm_stop fails outside update().
// REQ-SAFE-03
TEST_F(NodeApi, PwmStopSendsNoPulseAndSetZeroIsTheCentrePulse) {
  struct Servo {
    TalosNode* node = nullptr;
    int64_t servo = -1;
    bool stop = false;
  };
  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& s = *static_cast<Servo*>(user);
    s.node = node;
    s.servo = talos_device(node, TALOS_DEVICE_PWM, "servo");
    if (s.servo < 0) return 1;
    if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
    return talos_node_set_update(
        node,
        [](void* u) -> int32_t {
          auto& servo = *static_cast<Servo*>(u);
          return servo.stop ? talos_pwm_stop(servo.node, servo.servo)
                            : talos_pwm_set(servo.node, servo.servo, 0.0);
        },
        &s);
  };
  Servo servo;
  TalosNode* node = talos_sim_add(sim_, "roller", build, &servo);
  ASSERT_NE(node, nullptr) << talos_last_error();
  EXPECT_EQ(talos_pwm_stop(node, servo.servo), TALOS_ERR);
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_enable(hw, 1), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 30 * kMs), TALOS_OK) << talos_last_error();
  double out = -1;
  EXPECT_EQ(talos_sim_hardware_pwm(hw, "roller", "servo", &out), 1);
  EXPECT_EQ(out, 0.0);
  out = -1;
  EXPECT_EQ(talos_sim_hardware_applied_pwm(hw, "roller", "servo", &out), 1);
  EXPECT_EQ(out, 0.0);

  servo.stop = true;
  ASSERT_EQ(talos_sim_step(sim_, 30 * kMs), TALOS_OK) << talos_last_error();
  out = -1;
  EXPECT_EQ(talos_sim_hardware_pwm(hw, "roller", "servo", &out), 0);
  EXPECT_EQ(out, 0.0);
  out = -1;
  EXPECT_EQ(talos_sim_hardware_applied_pwm(hw, "roller", "servo", &out), 0);
  EXPECT_EQ(out, 0.0);
  talos_sim_hardware_destroy(hw);
}

// TALOS_MODE_TORQUE_CURRENT, the last mode, reaches the request as stator
// amperes; the value past it is refused.
// REQ-SAFE-43
TEST(NodeApiTorque, TorqueCurrentModeReachesTheRequestAndTheNextIsRefused) {
  struct Torquer {
    TalosNode* node = nullptr;
    int64_t wheel = -1;
    int32_t mode = TALOS_MODE_TORQUE_CURRENT;
    int32_t result = TALOS_OK;
  };
  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& t = *static_cast<Torquer*>(user);
    t.node = node;
    t.wheel = talos_device(node, TALOS_DEVICE_MOTOR, "wheel");
    if (t.wheel < 0) return 1;
    if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
    return talos_node_set_update(
        node,
        [](void* u) -> int32_t {
          auto& torquer = *static_cast<Torquer*>(u);
          torquer.result = talos_motor_set(torquer.node, torquer.wheel,
                                           torquer.mode, 2.5, 0, 0);
          return 0;
        },
        &t);
  };
  std::string toml{kRobotToml};
  toml.insert(toml.find("can_id = 1\n") + 11,
              "foc = true\nmax_torque_current_a = 5\n");
  TalosSim* sim = talos_sim_from_toml(toml.c_str());
  ASSERT_NE(sim, nullptr) << talos_last_error();
  Torquer torquer;
  ASSERT_NE(talos_sim_add(sim, "roller", build, &torquer), nullptr)
      << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_enable(hw, 1), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim, 30 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(torquer.result, TALOS_OK) << talos_last_error();
  TalosMotorRequest request{};
  ASSERT_EQ(talos_sim_hardware_motor_request(hw, "roller", "wheel", &request),
            TALOS_OK);
  EXPECT_EQ(request.mode, TALOS_MODE_TORQUE_CURRENT);
  EXPECT_EQ(request.demand, 2.5);
  EXPECT_EQ(request.ff_volts, 0.0);
  EXPECT_EQ(request.slot, 0);

  torquer.mode = TALOS_MODE_TORQUE_CURRENT + 1;
  ASSERT_EQ(talos_sim_step(sim, 30 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(torquer.result, TALOS_ERR);
  talos_sim_hardware_destroy(hw);
  talos_sim_destroy(sim);
}

// Only velocity, position and motion magic carry a feedforward. In any other
// mode a nonzero ff_volts is refused, naming the motor, and the motor goes
// neutral: the gateway would neutralize that request on the robot while
// SimHardware applied it as sent, so a binding's Sim test would pass where
// the motor never moves. Scenario: a FOC motor allowed torque current asks
// each mode for 0.5 with 0.5 V of feedforward.
// REQ-SAFE-04
TEST(NodeApiFeedforward, AModeWithoutFeedforwardRefusesOne) {
  struct Asker {
    TalosNode* node = nullptr;
    int64_t wheel = -1;
    int32_t mode = TALOS_MODE_VELOCITY;
    int32_t result = TALOS_OK;
    std::string error;
  };
  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& a = *static_cast<Asker*>(user);
    a.node = node;
    a.wheel = talos_device(node, TALOS_DEVICE_MOTOR, "wheel");
    if (a.wheel < 0) return 1;
    if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
    return talos_node_set_update(
        node,
        [](void* u) -> int32_t {
          auto& asker = *static_cast<Asker*>(u);
          asker.result =
              talos_motor_set(asker.node, asker.wheel, asker.mode, 0.5, 0.5, 0);
          asker.error =
              asker.result == TALOS_OK ? "" : std::string{talos_last_error()};
          return 0;
        },
        &a);
  };
  std::string toml{kRobotToml};
  toml.insert(toml.find("can_id = 1\n") + 11,
              "foc = true\nmax_torque_current_a = 5\n");
  TalosSim* sim = talos_sim_from_toml(toml.c_str());
  ASSERT_NE(sim, nullptr) << talos_last_error();
  Asker asker;
  ASSERT_NE(talos_sim_add(sim, "roller", build, &asker), nullptr)
      << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_enable(hw, 1), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim, 30 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(asker.result, TALOS_OK) << asker.error;
  TalosMotorRequest request{};
  ASSERT_EQ(talos_sim_hardware_applied_motor(hw, "roller", "wheel", &request),
            TALOS_OK);
  ASSERT_EQ(request.mode, TALOS_MODE_VELOCITY);
  ASSERT_EQ(request.ff_volts, 0.5);

  for (const int32_t mode : {TALOS_MODE_NEUTRAL, TALOS_MODE_DUTY_CYCLE,
                             TALOS_MODE_VOLTAGE, TALOS_MODE_TORQUE_CURRENT}) {
    asker.mode = mode;
    ASSERT_EQ(talos_sim_step(sim, 30 * kMs), TALOS_OK) << talos_last_error();
    EXPECT_EQ(asker.result, TALOS_ERR) << "mode " << mode;
    EXPECT_NE(asker.error.find("'wheel'"), std::string::npos) << asker.error;
    ASSERT_EQ(talos_sim_hardware_applied_motor(hw, "roller", "wheel", &request),
              TALOS_OK);
    EXPECT_EQ(request.mode, TALOS_MODE_NEUTRAL) << "mode " << mode;
    EXPECT_EQ(request.ff_volts, 0.0) << "mode " << mode;
  }
  talos_sim_hardware_destroy(hw);
  talos_sim_destroy(sim);
}

// REQ-SAFE-04: non-finite ff_volts (NaN, +Inf, -Inf) in modes that carry no
// feedforward (neutral, duty cycle, voltage, torque current) fails closed:
// talos_motor_set returns TALOS_ERR naming the motor, and the motor goes
// neutral.
TEST(NodeApiFeedforward, NonFiniteFeedforwardFailsClosedInNonFeedforwardModes) {
  struct Asker {
    TalosNode* node = nullptr;
    int64_t wheel = -1;
    int32_t mode = TALOS_MODE_VOLTAGE;
    double demand = 3.0;
    double ff_volts = 0.0;
    int32_t result = TALOS_OK;
    std::string error;
  };
  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& a = *static_cast<Asker*>(user);
    a.node = node;
    a.wheel = talos_device(node, TALOS_DEVICE_MOTOR, "wheel");
    if (a.wheel < 0) return 1;
    if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
    return talos_node_set_update(
        node,
        [](void* u) -> int32_t {
          auto& asker = *static_cast<Asker*>(u);
          asker.result = talos_motor_set(asker.node, asker.wheel, asker.mode,
                                         asker.demand, asker.ff_volts, 0);
          asker.error =
              asker.result == TALOS_OK ? "" : std::string{talos_last_error()};
          return 0;
        },
        &a);
  };
  std::string toml{kRobotToml};
  toml.insert(toml.find("can_id = 1\n") + 11,
              "foc = true\nmax_torque_current_a = 5\n");
  TalosSim* sim = talos_sim_from_toml(toml.c_str());
  ASSERT_NE(sim, nullptr) << talos_last_error();
  Asker asker;
  ASSERT_NE(talos_sim_add(sim, "roller", build, &asker), nullptr)
      << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_enable(hw, 1), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim, 30 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(asker.result, TALOS_OK) << asker.error;
  TalosMotorRequest request{};
  ASSERT_EQ(talos_sim_hardware_applied_motor(hw, "roller", "wheel", &request),
            TALOS_OK);
  ASSERT_EQ(request.mode, TALOS_MODE_VOLTAGE);
  ASSERT_DOUBLE_EQ(request.demand, 3.0);

  constexpr double kInf = std::numeric_limits<double>::infinity();
  for (const int32_t mode : {TALOS_MODE_NEUTRAL, TALOS_MODE_DUTY_CYCLE,
                             TALOS_MODE_VOLTAGE, TALOS_MODE_TORQUE_CURRENT}) {
    for (const double ff :
         {std::numeric_limits<double>::quiet_NaN(), kInf, -kInf}) {
      asker.mode = mode;
      asker.demand = 0.5;
      asker.ff_volts = ff;
      ASSERT_EQ(talos_sim_step(sim, 20 * kMs), TALOS_OK) << talos_last_error();
      EXPECT_EQ(asker.result, TALOS_ERR) << "mode " << mode << " ff " << ff;
      EXPECT_NE(asker.error.find("'wheel'"), std::string::npos) << asker.error;
      ASSERT_EQ(
          talos_sim_hardware_motor_request(hw, "roller", "wheel", &request),
          TALOS_OK);
      EXPECT_EQ(request.mode, TALOS_MODE_NEUTRAL)
          << "mode " << mode << " ff " << ff;
      EXPECT_EQ(request.demand, 0.0);
      EXPECT_EQ(request.ff_volts, 0.0);
      ASSERT_EQ(
          talos_sim_hardware_applied_motor(hw, "roller", "wheel", &request),
          TALOS_OK);
      EXPECT_EQ(request.mode, TALOS_MODE_NEUTRAL)
          << "mode " << mode << " ff " << ff;
      EXPECT_EQ(request.demand, 0.0);
      EXPECT_EQ(request.ff_volts, 0.0);
    }
  }

  asker.mode = TALOS_MODE_DUTY_CYCLE;
  asker.demand = 0.5;
  asker.ff_volts = 0.0;
  ASSERT_EQ(talos_sim_step(sim, 20 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(asker.result, TALOS_OK) << asker.error;
  ASSERT_EQ(talos_sim_hardware_applied_motor(hw, "roller", "wheel", &request),
            TALOS_OK);
  EXPECT_EQ(request.mode, TALOS_MODE_DUTY_CYCLE);
  EXPECT_DOUBLE_EQ(request.demand, 0.5);

  talos_sim_hardware_destroy(hw);
  talos_sim_destroy(sim);
}

// The ABI takes raw handles, so it refuses an unknown device name and a
// handle of another kind or out of range.
TEST_F(NodeApi, DeviceHandlesAreCheckedByNameAndKind) {
  auto build = [](TalosNode* node, void* user) -> int32_t {
    if (talos_device(node, TALOS_DEVICE_MOTOR, "flywheel") >= 0) return 1;
    return BuildRoller(node, user);
  };
  Roller roller;
  TalosNode* node = talos_sim_add(sim_, "roller", build, &roller);
  ASSERT_NE(node, nullptr) << talos_last_error();
  TalosSample sample{};
  EXPECT_EQ(talos_motor_sample(node, roller.wheel, &sample), TALOS_OK);
  EXPECT_EQ(talos_motor_sample(node, roller.beam, &sample), TALOS_ERR);
  EXPECT_EQ(talos_gyro_rezero(node, roller.wheel, 0), TALOS_ERR);
  EXPECT_EQ(talos_motor_rezero(node, -1, 0), TALOS_ERR);
}

// REQ-CONTRACT-13: indexed device getters/setters look up HardwareLink without
// attaching it to a non-hardware node, and check Index(index) before lookup.
TEST_F(NodeApi, IndexedDeviceLookupsDoNotAttachHardwareLink) {
  struct Probe {
    int32_t neg_rc = TALOS_OK;
    std::string neg_err;
    int32_t zero_rc = TALOS_OK;
    std::string zero_err;
  } probe;
  auto build = [](TalosNode* node, void* user) -> int32_t {
    auto& p = *static_cast<Probe*>(user);
    TalosSample sample{};
    p.neg_rc = talos_motor_sample(node, -1, &sample);
    p.neg_err = talos_last_error();
    p.zero_rc = talos_motor_sample(node, 0, &sample);
    p.zero_err = talos_last_error();
    return 0;
  };
  TalosNode* node = talos_sim_add(sim_, "relay", build, &probe);
  ASSERT_NE(node, nullptr) << talos_last_error();
  EXPECT_EQ(probe.neg_rc, TALOS_ERR);
  EXPECT_NE(probe.neg_err.find("negative index"), std::string::npos)
      << probe.neg_err;
  EXPECT_EQ(probe.zero_rc, TALOS_ERR);
  EXPECT_NE(probe.zero_err.find("no device handle at index 0"),
            std::string::npos)
      << probe.zero_err;

  // Step with SimHardware running; if HardwareLink had been silently attached
  // during build, talos_hardware_fresh would succeed and report 1. Instead,
  // post-start lazy HardwareLink attachment must fail (registration closed),
  // while indexed device lookups still report out-of-range cleanly.
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();

  TalosSample sample{};
  int64_t ts = 0;
  uint64_t clamped = 0;
  EXPECT_EQ(talos_motor_sample(node, 0, &sample), TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("no device handle at index 0"),
            std::string::npos)
      << talos_last_error();
  EXPECT_EQ(talos_sensor_sample(node, 0, &sample), TALOS_ERR);
  EXPECT_EQ(talos_digital_input_sample(node, 0, &sample), TALOS_ERR);
  EXPECT_EQ(talos_analog_input_sample(node, 0, &sample), TALOS_ERR);
  EXPECT_EQ(talos_encoder_sample(node, 0, &sample), TALOS_ERR);
  EXPECT_EQ(talos_motor_time(node, 0, 0, &ts), TALOS_ERR);
  EXPECT_EQ(talos_sensor_time(node, 0, 0, &ts), TALOS_ERR);
  EXPECT_EQ(talos_motor_age(node, 0, 0, &ts), TALOS_ERR);
  EXPECT_EQ(talos_sensor_age(node, 0, 0, &ts), TALOS_ERR);
  EXPECT_EQ(talos_motor_zeroed(node, 0), TALOS_ERR);
  EXPECT_EQ(talos_gyro_zeroed(node, 0), TALOS_ERR);
  EXPECT_EQ(talos_motor_clamped_count(node, 0, &clamped), TALOS_ERR);
  EXPECT_EQ(talos_motor_hardware_index(node, 0), TALOS_ERR);
  EXPECT_EQ(talos_motor_set(node, 0, TALOS_MODE_NEUTRAL, 0.0, 0.0, 0),
            TALOS_ERR);
  EXPECT_EQ(talos_motor_rezero(node, 0, 0.0), TALOS_ERR);
  EXPECT_EQ(talos_gyro_rezero(node, 0, 0.0), TALOS_ERR);
  EXPECT_EQ(talos_digital_output_set(node, 0, 1), TALOS_ERR);
  EXPECT_EQ(talos_pwm_set(node, 0, 0.0), TALOS_ERR);
  EXPECT_EQ(talos_pwm_stop(node, 0), TALOS_ERR);

  // HardwareLink was never attached during build, so EnsureLink fails after
  // start (loop registration is closed).
  EXPECT_EQ(talos_hardware_fresh(node, -1), TALOS_ERR);
  talos_sim_hardware_destroy(hw);
}

// REQ-CONTRACT-14, REQ-RT-04, REQ-RT-06: invalid-index target, target-lane,
// and operator-arming lookups return errors without attaching Targets, and
// off-thread talos_target_source_name returns nullptr with the loop-thread
// error.
TEST_F(NodeApi, IndexedTargetLookupsValidateIndexAndLoopThread) {
  struct Probe {
    int32_t target_neg = TALOS_OK;
    int32_t target_zero = TALOS_OK;
    std::string target_zero_err;
    const char* source_neg = "";
    const char* source_zero = "";
    std::string source_zero_err;
    int32_t lane_neg = TALOS_OK;
    int32_t lane_zero = TALOS_OK;
    std::string lane_zero_err;
    int32_t arming_neg = TALOS_OK;
    int32_t arming_zero = TALOS_OK;
    std::string arming_zero_err;
  } probe;
  auto build_plain = [](TalosNode* node, void* user) -> int32_t {
    auto& p = *static_cast<Probe*>(user);
    float value = 0.0f;
    int32_t source = 99;
    p.target_neg = talos_target_get(node, -1, &value, &source);
    p.target_zero = talos_target_get(node, 0, &value, &source);
    p.target_zero_err = talos_last_error();
    p.source_neg = talos_target_source_name(node, -1, 0);
    p.source_zero = talos_target_source_name(node, 0, 0);
    p.source_zero_err = talos_last_error();
    p.lane_neg = talos_target_lane_send(node, -1, &value, sizeof value);
    p.lane_zero = talos_target_lane_send(node, 0, &value, sizeof value);
    p.lane_zero_err = talos_last_error();
    p.arming_neg = talos_operator_arming_armed(node, -1);
    p.arming_zero = talos_operator_arming_armed(node, 0);
    p.arming_zero_err = talos_last_error();
    return 0;
  };
  TalosNode* plain = talos_sim_add(sim_, "relay", build_plain, &probe);
  ASSERT_NE(plain, nullptr) << talos_last_error();

  EXPECT_EQ(probe.target_neg, TALOS_ERR);
  EXPECT_EQ(probe.target_zero, TALOS_ERR);
  EXPECT_NE(probe.target_zero_err.find("no target at index 0"),
            std::string::npos)
      << probe.target_zero_err;
  EXPECT_EQ(probe.source_neg, nullptr);
  EXPECT_EQ(probe.source_zero, nullptr);
  EXPECT_NE(probe.source_zero_err.find("no target at index 0"),
            std::string::npos)
      << probe.source_zero_err;
  EXPECT_EQ(probe.lane_neg, TALOS_ERR);
  EXPECT_EQ(probe.lane_zero, TALOS_ERR);
  EXPECT_NE(probe.lane_zero_err.find("no target lane at index 0"),
            std::string::npos)
      << probe.lane_zero_err;
  EXPECT_EQ(probe.arming_neg, TALOS_ERR);
  EXPECT_EQ(probe.arming_zero, TALOS_ERR);
  EXPECT_NE(probe.arming_zero_err.find("no operator arming at index 0"),
            std::string::npos)
      << probe.arming_zero_err;

  // Post-start invalid-index lookups on a node without Targets still report
  // out-of-range cleanly.
  float value = 0.0f;
  int32_t source = 99;
  EXPECT_EQ(talos_target_get(plain, 0, &value, &source), TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("no target at index 0"),
            std::string::npos)
      << talos_last_error();
  EXPECT_EQ(talos_target_source_name(plain, 0, 0), nullptr);
  EXPECT_NE(std::string{talos_last_error()}.find("no target at index 0"),
            std::string::npos)
      << talos_last_error();
  EXPECT_EQ(talos_target_lane_send(plain, 0, &value, sizeof value), TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("no target lane at index 0"),
            std::string::npos)
      << talos_last_error();
  EXPECT_EQ(talos_operator_arming_armed(plain, 0), TALOS_ERR);
  EXPECT_NE(
      std::string{talos_last_error()}.find("no operator arming at index 0"),
      std::string::npos)
      << talos_last_error();

  // Open a valid target on another node so off-thread rejection is exercised
  // both on an open target and on a node with no Targets attached.
  int64_t target = -1;
  auto build_target = [](TalosNode* node, void* user) -> int32_t {
    auto& idx = *static_cast<int64_t*>(user);
    idx = talos_target_open(node, sizeof(float), alignof(float), &kFloatSchema);
    return idx < 0 ? 1 : 0;
  };
  TalosNode* with_target = talos_sim_add(sim_, "roller", build_target, &target);
  ASSERT_NE(with_target, nullptr) << talos_last_error();
  ASSERT_NE(talos_target_source_name(with_target, target, 0), nullptr)
      << talos_last_error();

  const char* off_plain = "";
  std::string off_plain_err;
  const char* off_valid = "";
  std::string off_valid_err;
  std::thread worker{[&] {
    off_plain = talos_target_source_name(plain, 0, 0);
    off_plain_err = talos_last_error();
    off_valid = talos_target_source_name(with_target, target, 0);
    off_valid_err = talos_last_error();
  }};
  worker.join();
  EXPECT_EQ(off_plain, nullptr);
  EXPECT_NE(off_plain_err.find("loop thread"), std::string::npos)
      << off_plain_err;
  EXPECT_EQ(off_valid, nullptr);
  EXPECT_NE(off_valid_err.find("loop thread"), std::string::npos)
      << off_valid_err;
}

TEST_F(NodeApi, HardwareSampleTimeOnceTheClockMappingConverges) {
  struct ClockRoller {
    Roller roller;
    int64_t gyro = -1;
    int64_t cc = -1;
  } state;
  auto build = [](TalosNode* n, void* user) -> int32_t {
    auto& s = *static_cast<ClockRoller*>(user);
    if (BuildRoller(n, &s.roller) != TALOS_OK) return TALOS_ERR;
    s.gyro = talos_device(n, TALOS_DEVICE_GYRO, "gyro");
    s.cc = talos_device(n, TALOS_DEVICE_CANCODER, "cc");
    return (s.gyro >= 0 && s.cc >= 0) ? TALOS_OK : TALOS_ERR;
  };
  TalosNode* node = talos_sim_add(sim_, "roller", build, &state);
  ASSERT_NE(node, nullptr) << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  const auto gyro = state.gyro;
  const auto cc = state.cc;
  const auto& roller = state.roller;
  (void)roller;
  ASSERT_EQ(talos_sim_hardware_set_motor(hw, "roller", "wheel", 3, 2, 1000,
                                         3000, 0, 0, 0),
            TALOS_OK);
  ASSERT_EQ(
      talos_sim_hardware_set_cancoder(hw, "roller", "cc", 0.2, 0.3, 2000, 4000),
      TALOS_OK);
  ASSERT_EQ(
      talos_sim_hardware_set_gyro(hw, "roller", "gyro", 0.75, -0.5, 5000, 6000),
      TALOS_OK);
  // The gateway clock is 3 s ahead of sim time.
  ASSERT_EQ(talos_sim_hardware_set_gateway_clock(hw, 3000 * kMs, 0), TALOS_OK);
  EXPECT_EQ(talos_sim_hardware_set_gateway_clock(hw, 0, -1), TALOS_ERR);

  // States every 10 ms from 0: at most 6 by 50 ms, short of the 8 the
  // bridge's filter needs.
  ASSERT_EQ(talos_sim_step(sim_, 50 * kMs), TALOS_OK) << talos_last_error();
  int64_t sampled = -1;
  EXPECT_EQ(talos_hardware_sample_time(node, &sampled), 0);
  EXPECT_EQ(sampled, -1);
  int64_t acquired = -1;
  EXPECT_EQ(talos_motor_time(node, roller.wheel, 0, &acquired), 0);
  EXPECT_EQ(acquired, -1);
  EXPECT_EQ(talos_sensor_time(node, cc, 0, &acquired), 0);
  EXPECT_EQ(acquired, -1);
  // Signal ages come from the State itself: known before the mapping.
  int64_t age = -1;
  ASSERT_EQ(talos_motor_age(node, roller.wheel, 0, &age), 1);
  EXPECT_EQ(age, 1 * kMs);
  ASSERT_EQ(talos_motor_age(node, roller.wheel, 1, &age), 1);
  EXPECT_EQ(age, 3 * kMs);
  ASSERT_EQ(talos_sensor_age(node, cc, 0, &age), 1);
  EXPECT_EQ(age, 2 * kMs);
  ASSERT_EQ(talos_sensor_age(node, cc, 1, &age), 1);
  EXPECT_EQ(age, 4 * kMs);

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
  ASSERT_EQ(talos_sensor_time(node, cc, 0, &acquired), 1);
  EXPECT_EQ(acquired, sampled - 2 * kMs);
  ASSERT_EQ(talos_sensor_time(node, cc, 1, &acquired), 1);
  EXPECT_EQ(acquired, sampled - 4 * kMs);
  ASSERT_EQ(talos_sensor_time(node, gyro, 0, &acquired), 1);
  EXPECT_EQ(acquired, sampled - 5 * kMs);
  ASSERT_EQ(talos_sensor_time(node, gyro, 1, &acquired), 1);
  EXPECT_EQ(acquired, sampled - 6 * kMs);
  EXPECT_EQ(talos_motor_time(node, roller.wheel, 0, nullptr), TALOS_ERR);
  EXPECT_EQ(talos_sensor_time(node, gyro, 0, nullptr), TALOS_ERR);
  EXPECT_EQ(talos_motor_time(node, -1, 0, &acquired), TALOS_ERR);
  EXPECT_EQ(talos_sensor_age(node, cc, 0, nullptr), TALOS_ERR);
  EXPECT_EQ(talos_sensor_age(node, -1, 0, &age), TALOS_ERR);
  EXPECT_EQ(talos_sensor_age(node, roller.wheel, 0, &age), TALOS_ERR);

  // An unknown age invalidates the sample and returns 0.
  ASSERT_EQ(talos_sim_hardware_set_cancoder(hw, "roller", "cc", 0.2, 0.3,
                                            UINT32_MAX, 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(talos_sensor_age(node, cc, 0, &age), 0);
  EXPECT_EQ(talos_sensor_age(node, cc, 1, &age), 0);
  ASSERT_EQ(
      talos_sim_hardware_set_cancoder(hw, "roller", "cc", 0.2, 0.3, 2000, 4000),
      TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();
  ASSERT_EQ(talos_sensor_age(node, cc, 0, &age), 1);
  EXPECT_EQ(age, 2 * kMs);

  talos_sim_hardware_destroy(hw);  // the bridge goes silent
  ASSERT_EQ(talos_sim_step(sim_, 150 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(talos_hardware_sample_time(node, &sampled), 0);
  EXPECT_EQ(talos_motor_time(node, roller.wheel, 0, &acquired), 0);
  EXPECT_EQ(talos_sensor_time(node, gyro, 0, &acquired), 0);
  EXPECT_EQ(talos_sensor_age(node, cc, 0, &age), 0);
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
                                  alignof(float), &kFloatSchema, -1);
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
  EXPECT_EQ(talos_target_source_name(node, lanes.target, 5), nullptr);

  // A silent Driver Station selects nothing.
  ASSERT_EQ(talos_sim_hardware_set_driver_station(hw, nullptr), TALOS_OK);
  ASSERT_EQ(talos_target_lane_send(node, lanes.lane, &speed, sizeof speed), 1);
  ASSERT_EQ(talos_sim_step(sim_, 150 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_EQ(talos_target_get(node, lanes.target, &out, &source), 0);
  talos_sim_hardware_destroy(hw);
}

// REQ-OPS-55: OperatorArming blocker C ABI round-trips kind/port/index, clears
// on arm, preserves 1-arg evaluate, and refuses unknown kind or bad handle.
TEST_F(NodeApi, OperatorArmingBlockerRoundTripsAndValidatesKind) {
  int64_t arming = -1;
  auto build = [](TalosNode* node, void* user) -> int32_t {
    *static_cast<int64_t*>(user) = talos_operator_arming_open(node);
    return *static_cast<int64_t*>(user) < 0 ? 1 : 0;
  };
  TalosNode* node = talos_sim_add(sim_, "roller", build, &arming);
  ASSERT_NE(node, nullptr) << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim_);
  ASSERT_NE(hw, nullptr) << talos_last_error();
  ASSERT_EQ(talos_sim_hardware_enable(hw, 1), TALOS_OK);
  TalosDriverStationPacket packet{};
  packet.flags = TALOS_DS_ATTACHED | TALOS_DS_ENABLED | TALOS_DS_TELEOP;
  ASSERT_EQ(talos_sim_hardware_set_driver_station(hw, &packet), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim_, 20 * kMs), TALOS_OK) << talos_last_error();

  uint8_t kind = 99;
  int8_t port = 99;
  int16_t index = 99;
  ASSERT_EQ(talos_operator_arming_blocker(node, arming, &kind, &port, &index),
            TALOS_OK);
  EXPECT_EQ(kind, TALOS_ARMING_INPUT_NONE);
  EXPECT_EQ(port, -1);
  EXPECT_EQ(index, -1);

  // Legacy 1-arg evaluate(0) records OTHER/-1/-1.
  EXPECT_EQ(talos_operator_arming_evaluate(node, arming, 0), 0);
  ASSERT_EQ(talos_operator_arming_blocker(node, arming, &kind, &port, &index),
            TALOS_OK);
  EXPECT_EQ(kind, TALOS_ARMING_INPUT_OTHER);
  EXPECT_EQ(port, -1);
  EXPECT_EQ(index, -1);

  // Blocker overload records exact kind/port/index while disarmed.
  int32_t may_publish = -1;
  ASSERT_EQ(talos_operator_arming_evaluate_blocker(
                node, arming, 0, TALOS_ARMING_INPUT_AXIS, 1, 2, &may_publish),
            TALOS_OK);
  EXPECT_EQ(may_publish, 0);
  EXPECT_EQ(talos_operator_arming_armed(node, arming), 0);
  ASSERT_EQ(talos_operator_arming_blocker(node, arming, &kind, &port, &index),
            TALOS_OK);
  EXPECT_EQ(kind, TALOS_ARMING_INPUT_AXIS);
  EXPECT_EQ(port, 1);
  EXPECT_EQ(index, 2);

  // Neutral evaluation arms and clears blocker.
  ASSERT_EQ(talos_operator_arming_evaluate_blocker(
                node, arming, 1, TALOS_ARMING_INPUT_NONE, -1, -1, &may_publish),
            TALOS_OK);
  EXPECT_EQ(may_publish, 0);
  EXPECT_EQ(talos_operator_arming_armed(node, arming), 1);
  ASSERT_EQ(talos_operator_arming_blocker(node, arming, &kind, &port, &index),
            TALOS_OK);
  EXPECT_EQ(kind, TALOS_ARMING_INPUT_NONE);
  EXPECT_EQ(port, -1);
  EXPECT_EQ(index, -1);

  // While armed, non-neutral returns 1 and keeps blocker cleared.
  ASSERT_EQ(talos_operator_arming_evaluate_blocker(
                node, arming, 0, TALOS_ARMING_INPUT_BUTTON, 0, 4, &may_publish),
            TALOS_OK);
  EXPECT_EQ(may_publish, 1);
  ASSERT_EQ(talos_operator_arming_blocker(node, arming, &kind, &port, &index),
            TALOS_OK);
  EXPECT_EQ(kind, TALOS_ARMING_INPUT_NONE);
  EXPECT_EQ(port, -1);
  EXPECT_EQ(index, -1);

  // Invalid kind and invalid handle are refused.
  EXPECT_EQ(talos_operator_arming_evaluate_blocker(node, arming, 0, 99, 0, 0,
                                                   &may_publish),
            TALOS_ERR);
  EXPECT_EQ(talos_operator_arming_blocker(node, -1, &kind, &port, &index),
            TALOS_ERR);
  talos_sim_hardware_destroy(hw);
}

// REQ-SAFE-14, REQ-CONTRACT-14: C ABI enforces OperatorArming when a node
// reading DriverStation opens a teleop-policy target lane, both at startup
// (talos_sim_add) and on lazy talos_driver_station_read in update().
TEST_F(NodeApi, OperatorProducerTeleopLaneRequiresOperatorArming) {
  {
    SCOPED_TRACE("talos_driver_station + unarmed teleop lane fails sim_add");
    auto build = [](TalosNode* node, void*) -> int32_t {
      if (talos_driver_station(node, nullptr, nullptr) != TALOS_OK) return 1;
      if (talos_target_lane_open(node, "roller", "teleop", sizeof(float),
                                 alignof(float), &kFloatSchema, -1) < 0) {
        return 1;
      }
      return 0;
    };
    EXPECT_EQ(talos_sim_add(sim_, "driver", build, nullptr), nullptr);
    const std::string err = talos_last_error();
    EXPECT_NE(err.find("OperatorArming"), std::string::npos) << err;
    EXPECT_NE(err.find("REQ-SAFE-14"), std::string::npos) << err;
    EXPECT_NE(err.find("/roller/target/teleop"), std::string::npos) << err;
  }

  {
    SCOPED_TRACE("lazy talos_driver_station_read in update returns TALOS_ERR");
    struct LazyState {
      TalosNode* node = nullptr;
      int attempts = 0;
      int failures = 0;
      std::string last_err;
    } state;
    auto update = [](void* user) -> int32_t {
      auto& s = *static_cast<LazyState*>(user);
      for (int i = 0; i < 2; ++i) {
        ++s.attempts;
        TalosDriverStation ds{};
        if (talos_driver_station_read(s.node, &ds) == TALOS_ERR) {
          ++s.failures;
          s.last_err = talos_last_error();
        }
      }
      return 0;
    };
    TalosSim* sim2 = talos_sim_from_toml(kRobotToml);
    ASSERT_NE(sim2, nullptr) << talos_last_error();
    struct LazyBundle {
      LazyState* state;
      TalosCallback update_fn;
    } bundle{&state, update};
    auto build_with_update = [](TalosNode* n, void* user) -> int32_t {
      auto& b = *static_cast<LazyBundle*>(user);
      b.state->node = n;
      if (talos_target_lane_open(n, "roller", "teleop", sizeof(float),
                                 alignof(float), &kFloatSchema, -1) < 0) {
        return 1;
      }
      return talos_node_set_update(n, b.update_fn, b.state);
    };
    ASSERT_NE(talos_sim_add(sim2, "lazy_driver", build_with_update, &bundle),
              nullptr)
        << talos_last_error();
    ASSERT_EQ(talos_sim_step(sim2, 20 * kMs), TALOS_OK) << talos_last_error();
    EXPECT_GE(state.attempts, 2);
    EXPECT_EQ(state.failures, state.attempts);
    EXPECT_NE(state.last_err.find("OperatorArming"), std::string::npos)
        << state.last_err;
    talos_sim_destroy(sim2);
  }

  {
    SCOPED_TRACE("armed teleop lane and autonomous lane succeed");
    auto build_armed = [](TalosNode* node, void*) -> int32_t {
      if (talos_driver_station(node, nullptr, nullptr) != TALOS_OK) return 1;
      const int64_t arming = talos_operator_arming_open(node);
      if (arming < 0) return 1;
      if (talos_target_lane_open(node, "roller", "teleop", sizeof(float),
                                 alignof(float), &kFloatSchema, arming) < 0) {
        return 1;
      }
      if (talos_target_lane_open(node, "roller", "auto", sizeof(float),
                                 alignof(float), &kFloatSchema, -1) < 0) {
        return 1;
      }
      return 0;
    };
    EXPECT_NE(talos_sim_add(sim_, "armed_driver", build_armed, nullptr),
              nullptr)
        << talos_last_error();
  }
}

// REQ-CONTRACT-13, REQ-SAFE-36, REQ-SAFE-37, REQ-SAFE-38: C++ ABI coverage of
// all mechanisms_api.cc entry points (including error/null/buffer-size/invalid-
// index boundaries and live talos_position_mechanism_homed after reset) plus
// StateMachine, param_of, every, arming_disarm, and sim hardware reboot/reset
// (exercised under ASan/TSan where Python ctypes tests cannot run).
TEST(NodeApiMechanisms, LifecycleStateAndBoundaries) {
  constexpr const char* kMechToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.shared]
gain = 3.5

[subsystems.arm.motors.arm]
can_id = 1
[subsystems.arm.digital_inputs.home_sw]
dio = 0
[subsystems.arm.arm]
min_rot = 0.0
max_rot = 1.0
homing = "limit_switch"
limit_switch = "home_sw"
limit_switch_closed_reads = true
homing_v = -1.0
home_rot = 0.0
home_samples = 2
home_margin_rot = 0.2
homing_timeout_us = 1000000
homing_stall_rps = 0.05
homing_stall_us = 200000
tolerance_rot = 0.01
units_per_rot = 360.0

[subsystems.shooter.motors.flywheel]
can_id = 2
max_velocity_rps = 80.0
[subsystems.shooter.flywheel]
at_speed_entry_tick_count = 2

[subsystems.intake.motors.roller]
can_id = 3
[subsystems.intake.digital_inputs.beam]
dio = 1
[subsystems.intake.roller]
sensor = "beam"
intake_v = 6.0
hold_v = 1.0
eject_v = -6.0
eject_us = 100000
)";

  TalosSim* sim = talos_sim_from_toml(kMechToml);
  ASSERT_NE(sim, nullptr) << talos_last_error();
  TalosSimHardware* hw = talos_sim_hardware_create(sim);
  ASSERT_NE(hw, nullptr) << talos_last_error();

  struct ArmCtx {
    TalosNode* node = nullptr;
    int64_t mech = -1;
    int64_t sm = -1;
    int64_t arming = -1;
    double shared_gain = 0.0;
    int every_calls = 0;
    enum class Action {
      kGoal,
      kGoalUnits,
      kHold,
      kStop,
      kClear
    } action = Action::kGoal;
    std::vector<std::pair<int32_t, int32_t>> homed_before_after;
  } arm_ctx;

  auto build_arm = [](TalosNode* node, void* user) -> int32_t {
    auto& a = *static_cast<ArmCtx*>(user);
    a.node = node;
    TalosParam p{};
    if (talos_node_param_of(node, "shared", "gain", TALOS_PARAM_DOUBLE, 1,
                            &p) != 1) {
      return 1;
    }
    a.shared_gain = p.number;
    a.sm = talos_state_machine_create(node, 10);
    a.arming = talos_operator_arming_open(node);
    if (a.sm < 0 || a.arming < 0) return 1;
    if (talos_node_every(
            node, 10 * kMs,
            [](void* ctx) -> int32_t {
              ++static_cast<ArmCtx*>(ctx)->every_calls;
              return 0;
            },
            &a) != TALOS_OK) {
      return 1;
    }
    a.mech = talos_position_mechanism_open(node, "arm");
    if (a.mech < 0) return 1;
    if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
    return talos_node_set_update(
        node,
        [](void* ctx) -> int32_t {
          auto& self = *static_cast<ArmCtx*>(ctx);
          const int32_t before =
              talos_position_mechanism_homed(self.node, self.mech);
          switch (self.action) {
            case ArmCtx::Action::kGoal:
              if (talos_position_mechanism_set_goal(self.node, self.mech, 0.25,
                                                    0.0) != TALOS_OK) {
                return 1;
              }
              break;
            case ArmCtx::Action::kGoalUnits:
              if (talos_position_mechanism_set_goal_units(
                      self.node, self.mech, 180.0, 0.0) != TALOS_OK) {
                return 1;
              }
              break;
            case ArmCtx::Action::kHold:
              if (talos_position_mechanism_hold(self.node, self.mech, 0.0) !=
                  TALOS_OK) {
                return 1;
              }
              break;
            case ArmCtx::Action::kStop:
              if (talos_position_mechanism_stop(self.node, self.mech) !=
                  TALOS_OK) {
                return 1;
              }
              break;
            case ArmCtx::Action::kClear:
              if (talos_position_mechanism_clear_fault(self.node, self.mech) !=
                  TALOS_OK) {
                return 1;
              }
              break;
          }
          if (talos_position_mechanism_update(self.node, self.mech) !=
              TALOS_OK) {
            return 1;
          }
          const int32_t after =
              talos_position_mechanism_homed(self.node, self.mech);
          self.homed_before_after.emplace_back(before, after);
          return 0;
        },
        &a);
  };

  struct ShooterCtx {
    TalosNode* node = nullptr;
    int64_t mech = -1;
    bool stop = false;
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
          const int32_t rc =
              self.stop ? talos_velocity_mechanism_stop(self.node, self.mech)
                        : talos_velocity_mechanism_set_goal(
                              self.node, self.mech, 40.0, 0.5);
          if (rc != TALOS_OK) return 1;
          return talos_velocity_mechanism_update(self.node, self.mech);
        },
        &s);
  };

  struct IntakeCtx {
    TalosNode* node = nullptr;
    int64_t mech = -1;
    int32_t cmd = TALOS_ROLLER_COMMAND_INTAKE;
    bool stop = false;
  } intake_ctx;
  auto build_intake = [](TalosNode* node, void* user) -> int32_t {
    auto& i = *static_cast<IntakeCtx*>(user);
    i.node = node;
    i.mech = talos_roller_with_sensor_open(node, "roller");
    if (i.mech < 0) return 1;
    if (talos_allow_untargeted_outputs(node) != TALOS_OK) return 1;
    return talos_node_set_update(
        node,
        [](void* ctx) -> int32_t {
          auto& self = *static_cast<IntakeCtx*>(ctx);
          const int32_t rc =
              self.stop ? talos_roller_with_sensor_stop(self.node, self.mech)
                        : talos_roller_with_sensor_set_goal(
                              self.node, self.mech, self.cmd);
          if (rc != TALOS_OK) return 1;
          return talos_roller_with_sensor_update(self.node, self.mech);
        },
        &i);
  };

  TalosNode* arm = talos_sim_add(sim, "arm", build_arm, &arm_ctx);
  TalosNode* shooter =
      talos_sim_add(sim, "shooter", build_shooter, &shooter_ctx);
  TalosNode* intake = talos_sim_add(sim, "intake", build_intake, &intake_ctx);
  TalosNode* plain = talos_sim_add(
      sim, "shared", [](TalosNode*, void*) { return 0; }, nullptr);
  ASSERT_NE(arm, nullptr) << talos_last_error();
  ASSERT_NE(shooter, nullptr) << talos_last_error();
  ASSERT_NE(intake, nullptr) << talos_last_error();
  ASSERT_NE(plain, nullptr) << talos_last_error();
  EXPECT_DOUBLE_EQ(arm_ctx.shared_gain, 3.5);

  // StateMachine + OperatorArming::disarm C ABI coverage.
  EXPECT_EQ(talos_state_machine_state(arm, arm_ctx.sm), 10);
  EXPECT_EQ(talos_state_machine_previous(arm, arm_ctx.sm), 10);
  EXPECT_EQ(talos_state_machine_just_entered(arm, arm_ctx.sm), 1);
  ASSERT_EQ(talos_state_machine_go(arm, arm_ctx.sm, 20), TALOS_OK);
  EXPECT_EQ(talos_state_machine_state(arm, arm_ctx.sm), 20);
  EXPECT_EQ(talos_state_machine_previous(arm, arm_ctx.sm), 10);
  EXPECT_EQ(talos_state_machine_time_in_state_ns(arm, arm_ctx.sm), 0);
  EXPECT_EQ(talos_state_machine_go(arm, -1, 20), TALOS_ERR);

  // Null / bad name / invalid index / no-mechanism-on-node boundaries.
  EXPECT_EQ(talos_position_mechanism_open(nullptr, "arm"), -1);
  EXPECT_EQ(talos_position_mechanism_open(arm, nullptr), -1);
  EXPECT_EQ(talos_velocity_mechanism_open(nullptr, "flywheel"), -1);
  EXPECT_EQ(talos_velocity_mechanism_open(shooter, nullptr), -1);
  EXPECT_EQ(talos_roller_with_sensor_open(nullptr, "roller"), -1);
  EXPECT_EQ(talos_roller_with_sensor_open(intake, nullptr), -1);

  TalosPositionMechanismState pos_state{};
  TalosVelocityMechanismState vel_state{};
  TalosRollerWithSensorState rol_state{};
  control::PositionState pos_msg{};
  control::VelocityState vel_msg{};
  EXPECT_EQ(talos_position_mechanism_state(plain, 0, &pos_state, nullptr, 0),
            TALOS_ERR);
  EXPECT_EQ(talos_position_mechanism_homed(plain, 0), TALOS_ERR);
  EXPECT_EQ(talos_position_mechanism_homed(arm, -1), TALOS_ERR);
  EXPECT_EQ(talos_position_mechanism_homed(arm, 99), TALOS_ERR);
  EXPECT_EQ(
      talos_position_mechanism_state(arm, arm_ctx.mech, nullptr, nullptr, 0),
      TALOS_ERR);
  // CopyMessage buffer-size mismatch is refused; nullptr message is allowed.
  EXPECT_EQ(talos_position_mechanism_state(arm, arm_ctx.mech, &pos_state,
                                           &pos_msg, 1),
            TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}.find("bytes"), std::string::npos);
  EXPECT_EQ(talos_position_mechanism_state(arm, arm_ctx.mech, &pos_state,
                                           nullptr, 999),
            TALOS_OK);
  EXPECT_EQ(talos_position_mechanism_state(arm, arm_ctx.mech, &pos_state,
                                           &pos_msg, sizeof(pos_msg)),
            TALOS_OK);

  EXPECT_EQ(talos_velocity_mechanism_state(shooter, shooter_ctx.mech, nullptr,
                                           nullptr, 0),
            TALOS_ERR);
  EXPECT_EQ(talos_velocity_mechanism_state(shooter, shooter_ctx.mech,
                                           &vel_state, &vel_msg, 1),
            TALOS_ERR);
  EXPECT_EQ(
      talos_velocity_mechanism_state(shooter, shooter_ctx.mech, &vel_state,
                                     &vel_msg, sizeof(vel_msg)),
      TALOS_OK);
  EXPECT_EQ(talos_velocity_mechanism_state(plain, 0, &vel_state, nullptr, 0),
            TALOS_ERR);

  EXPECT_EQ(talos_roller_with_sensor_state(intake, intake_ctx.mech, nullptr),
            TALOS_ERR);
  EXPECT_EQ(talos_roller_with_sensor_state(plain, 0, &rol_state), TALOS_ERR);
  EXPECT_EQ(talos_roller_with_sensor_set_goal(intake, intake_ctx.mech, -1),
            TALOS_ERR);
  EXPECT_EQ(talos_roller_with_sensor_set_goal(intake, intake_ctx.mech, 99),
            TALOS_ERR);

  // Requests / update outside update() fail naming the mechanism.
  EXPECT_EQ(talos_position_mechanism_set_goal(arm, arm_ctx.mech, 0.1, 0.0),
            TALOS_ERR);
  EXPECT_EQ(
      talos_position_mechanism_set_goal_units(arm, arm_ctx.mech, 10.0, 0.0),
      TALOS_ERR);
  EXPECT_EQ(talos_position_mechanism_hold(arm, arm_ctx.mech, 0.0), TALOS_ERR);
  EXPECT_EQ(talos_position_mechanism_stop(arm, arm_ctx.mech), TALOS_ERR);
  EXPECT_EQ(talos_position_mechanism_clear_fault(arm, arm_ctx.mech), TALOS_ERR);
  EXPECT_EQ(talos_position_mechanism_update(arm, arm_ctx.mech), TALOS_ERR);
  EXPECT_EQ(
      talos_velocity_mechanism_set_goal(shooter, shooter_ctx.mech, 10.0, 0.0),
      TALOS_ERR);
  EXPECT_EQ(talos_velocity_mechanism_stop(shooter, shooter_ctx.mech),
            TALOS_ERR);
  EXPECT_EQ(talos_velocity_mechanism_update(shooter, shooter_ctx.mech),
            TALOS_ERR);
  EXPECT_EQ(talos_roller_with_sensor_stop(intake, intake_ctx.mech), TALOS_ERR);
  EXPECT_EQ(talos_roller_with_sensor_update(intake, intake_ctx.mech),
            TALOS_ERR);

  // Drive mechanisms through homing, goals, hold, stop, clear_fault, reset, and
  // reboot.
  ASSERT_EQ(talos_sim_hardware_enable(hw, 1), TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_ideal_motors(hw, 1), TALOS_OK);
  TalosDriverStationPacket ds{};
  ds.flags = TALOS_DS_ATTACHED | TALOS_DS_ENABLED | TALOS_DS_TELEOP;
  ASSERT_EQ(talos_sim_hardware_set_driver_station(hw, &ds), TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_set_digital_input(hw, "arm", "home_sw", 1),
            TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim, 40 * kMs), TALOS_OK) << talos_last_error();
  EXPECT_GT(arm_ctx.every_calls, 0);
  EXPECT_EQ(talos_position_mechanism_homed(arm, arm_ctx.mech), 1);

  // Exercise arming disarm + reboot + reset_device -> live homed == 0 before
  // update().
  ASSERT_EQ(talos_operator_arming_evaluate(arm, arm_ctx.arming, 1), 0);
  EXPECT_EQ(talos_operator_arming_armed(arm, arm_ctx.arming), 1);
  ASSERT_EQ(talos_operator_arming_disarm(arm, arm_ctx.arming), TALOS_OK);
  EXPECT_EQ(talos_operator_arming_armed(arm, arm_ctx.arming), 0);

  arm_ctx.action = ArmCtx::Action::kGoalUnits;
  ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();
  arm_ctx.action = ArmCtx::Action::kHold;
  ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();
  arm_ctx.action = ArmCtx::Action::kClear;
  ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();
  arm_ctx.action = ArmCtx::Action::kStop;
  shooter_ctx.stop = true;
  intake_ctx.stop = true;
  ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();

  ASSERT_EQ(talos_sim_hardware_set_digital_input(hw, "arm", "home_sw", 0),
            TALOS_OK);
  ASSERT_EQ(talos_sim_hardware_reset_device(hw, "arm", "arm"), TALOS_OK);
  arm_ctx.homed_before_after.clear();
  ASSERT_EQ(talos_sim_step(sim, 25 * kMs), TALOS_OK) << talos_last_error();
  bool saw_live_drop = false;
  for (const auto& [before, after] : arm_ctx.homed_before_after) {
    if (after == 0) {
      EXPECT_EQ(before, 0) << "homed must drop live before update()";
      saw_live_drop = true;
      break;
    }
  }
  EXPECT_TRUE(saw_live_drop);

  ASSERT_EQ(talos_sim_hardware_reboot(hw), TALOS_OK);
  ASSERT_EQ(talos_sim_step(sim, 10 * kMs), TALOS_OK) << talos_last_error();

  talos_sim_hardware_destroy(hw);
  talos_sim_destroy(sim);
}

// REQ-CONTRACT-13, REQ-OPS-20: talos_motor_hardware_index exposes the
// configured motor slot index and rejects invalid nodes or non-motor indices.
TEST(NodeApiStandalone, MotorHardwareIndexReturnsSlotAndRejectsInvalid) {
  constexpr const char* kMultiToml = R"(
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.alpha.motors.first]
can_id = 1

[subsystems.beta.digital_inputs.beam]
dio = 0
[subsystems.beta.motors.second]
can_id = 2
[subsystems.beta.motors.third]
can_id = 3
)";
  TalosSim* sim = talos_sim_from_toml(kMultiToml);
  ASSERT_NE(sim, nullptr) << talos_last_error();

  struct Alpha {
    int64_t first = -1;
  };
  struct Beta {
    int64_t beam = -1;
    int64_t third = -1;
    int64_t second = -1;
  };
  auto build_alpha = [](TalosNode* node, void* user) -> int32_t {
    auto& a = *static_cast<Alpha*>(user);
    a.first = talos_device(node, TALOS_DEVICE_MOTOR, "first");
    return a.first >= 0 ? 0 : 1;
  };
  auto build_beta = [](TalosNode* node, void* user) -> int32_t {
    auto& b = *static_cast<Beta*>(user);
    // Open a non-motor first and motors in reverse order so C ABI device
    // indices differ from hardware motor indices.
    b.beam = talos_device(node, TALOS_DEVICE_DIGITAL_INPUT, "beam");
    b.third = talos_device(node, TALOS_DEVICE_MOTOR, "third");
    b.second = talos_device(node, TALOS_DEVICE_MOTOR, "second");
    return b.beam >= 0 && b.third >= 0 && b.second >= 0 ? 0 : 1;
  };

  Alpha alpha;
  Beta beta;
  TalosNode* alpha_node = talos_sim_add(sim, "alpha", build_alpha, &alpha);
  ASSERT_NE(alpha_node, nullptr) << talos_last_error();
  TalosNode* beta_node = talos_sim_add(sim, "beta", build_beta, &beta);
  ASSERT_NE(beta_node, nullptr) << talos_last_error();

  EXPECT_EQ(talos_motor_hardware_index(nullptr, 0), TALOS_ERR);
  EXPECT_NE(std::string{talos_last_error()}, "");
  EXPECT_EQ(talos_motor_hardware_index(beta_node, -1), TALOS_ERR);
  EXPECT_EQ(talos_motor_hardware_index(beta_node, beta.beam), TALOS_ERR);
  EXPECT_EQ(talos_motor_hardware_index(beta_node, 99), TALOS_ERR);

  const int32_t first_idx = talos_motor_hardware_index(alpha_node, alpha.first);
  EXPECT_STREQ(talos_last_error(), "");
  const int32_t second_idx = talos_motor_hardware_index(beta_node, beta.second);
  const int32_t third_idx = talos_motor_hardware_index(beta_node, beta.third);
  EXPECT_GE(first_idx, 0);
  EXPECT_LT(first_idx, 16);
  EXPECT_GE(second_idx, 0);
  EXPECT_LT(second_idx, 16);
  EXPECT_GE(third_idx, 0);
  EXPECT_LT(third_idx, 16);
  EXPECT_NE(first_idx, second_idx);
  EXPECT_NE(second_idx, third_idx);
  EXPECT_NE(first_idx, third_idx);
  // Every motor slot in this 3-motor config is in {0, 1, 2}.
  EXPECT_EQ((1u << first_idx) | (1u << second_idx) | (1u << third_idx), 0x7u);

  talos_sim_destroy(sim);
}

}  // namespace
