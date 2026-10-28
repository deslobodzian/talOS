#include "talOS/node/node.h"

#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "node_test/node_test_message_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/node/testing.h"

namespace talos {
namespace {

using namespace std::chrono_literals;
using node_test::Count;
using testing::Sim;

constexpr const char* kRobotToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.talker]
period_us = 10000

[subsystems.listener]
gain = 2.5
whole = 3
big = 300
tag = "left"
inverted = true
[subsystems.listener.limits]
max = 4.0

[subsystems.mechanism.motors.unused]
can_id = 1
)";

// --- params and identity -------------------------------------------------

struct Empty {
  explicit Empty(Node&) {}
};

TEST(Node, ReadsParamsWithFallbacksAndRejectsMissingOrMistyped) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<Empty>("listener");
  const Node& node = sim.node("listener");

  EXPECT_EQ(node.name(), "listener");
  EXPECT_TRUE(node.simulation());
  EXPECT_DOUBLE_EQ(node.param<double>("gain"), 2.5);
  EXPECT_DOUBLE_EQ(node.param<double>("whole"), 3.0);  // integer as double
  EXPECT_EQ(node.param<int>("whole"), 3);
  EXPECT_EQ(node.param<std::string>("tag"), "left");
  EXPECT_TRUE(node.param<bool>("inverted"));
  EXPECT_DOUBLE_EQ(node.param<double>("limits.max"), 4.0);

  EXPECT_DOUBLE_EQ(node.param<double>("absent", 9.5), 9.5);
  EXPECT_EQ(node.param<std::string>("absent", "x"), "x");
  EXPECT_DOUBLE_EQ(node.param<double>("gain", 0.0), 2.5);

  try {
    (void)node.param<double>("absent");
    FAIL() << "a missing parameter must throw";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string{e.what()}.find("listener"), std::string::npos);
    EXPECT_NE(std::string{e.what()}.find("'absent'"), std::string::npos);
  }
  EXPECT_THROW((void)node.param<bool>("tag"), std::invalid_argument);
  EXPECT_THROW((void)node.param<int>("gain", 0), std::invalid_argument);
  EXPECT_EQ(node.param<std::uint8_t>("whole"), 3);
  EXPECT_THROW((void)node.param<std::uint8_t>("big"), std::invalid_argument);
}

TEST(Node, PeriodComesFromTheSubsystemElseTheRobot) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<Empty>("talker");
  sim.add<Empty>("listener");
  EXPECT_EQ(sim.node("talker").period(), 10ms);
  EXPECT_EQ(sim.node("listener").period(), 5ms);
  EXPECT_EQ(detail::NodeAccess::freshness_timeout(sim.node("listener")), 100ms);
}

// --- topics ----------------------------------------------------------------

struct Talker {
  explicit Talker(Node& n) : node{n}, out{n.publish<Count>("/talker/state")} {}
  void update() {
    out.send(Count{static_cast<std::int64_t>(node.tick()), node.now().nanos()});
  }
  Node& node;
  Publisher<Count> out;
};

struct Listener {
  explicit Listener(Node& n)
      : in{n.subscribe<Count>("/talker/state", [this](const Count& c) {
          heard.push_back(c.value());
        })} {}
  void update() { latest_in_update = in.has_value() ? in->value() : -1; }
  Subscription<Count> in;
  std::vector<std::int64_t> heard;
  std::int64_t latest_in_update = -1;
};

TEST(Node, TwoNodesTalkOverATopicInOneSim) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<Talker>("talker");
  auto& listener = sim.add<Listener>("listener");

  sim.step(52ms);  // talker ticks at 10..50 ms; the last lands by 51 ms

  EXPECT_EQ(listener.heard, (std::vector<std::int64_t>{0, 1, 2, 3, 4}));
  EXPECT_EQ(listener.in->value(), 4);
  EXPECT_GE(listener.latest_in_update, 3);
  const auto latest = sim.latest<Count>("/talker/state");
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->value(), 4);
}

struct Watcher {
  explicit Watcher(Node& n)
      : node{n}, in{n.subscribe<Count>("/watch/target")} {}
  void update() {
    updated.push_back(in.updated());
    fresh.push_back(in.fresh());
    ages.push_back(in.age());
  }
  Node& node;
  Subscription<Count> in;
  std::vector<bool> updated, fresh;
  std::vector<Duration> ages;
};

TEST(Node, SubscriptionTracksFreshnessUpdatesAndAge) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& watcher = sim.add<Watcher>("listener");  // 5 ms period

  EXPECT_FALSE(watcher.in.has_value());
  EXPECT_EQ(watcher.in->value(), 0);  // zero-initialized before any message
  EXPECT_EQ(watcher.in.age(), Duration::max());

  sim.step(5ms);  // tick 0: nothing yet
  ASSERT_EQ(watcher.updated.size(), 1u);
  EXPECT_FALSE(watcher.updated[0]);
  EXPECT_FALSE(watcher.fresh[0]);

  sim.publish("/watch/target", Count{7, 0});  // delivered at 6 ms
  sim.step(5ms);                              // tick 1 at 10 ms
  sim.step(5ms);  // tick 2 at 15 ms: nothing new since tick 1 began
  ASSERT_EQ(watcher.updated.size(), 3u);
  EXPECT_TRUE(watcher.updated[1]);
  EXPECT_FALSE(watcher.updated[2]);
  EXPECT_TRUE(watcher.fresh[1]);
  EXPECT_EQ(watcher.ages[1], 4ms);
  EXPECT_EQ(watcher.ages[2], 9ms);
  EXPECT_EQ(watcher.in->value(), 7);

  sim.step(100ms);  // past [robot] command_timeout_us
  EXPECT_FALSE(watcher.fresh.back());
  EXPECT_FALSE(watcher.in.fresh());
  EXPECT_TRUE(watcher.in.fresh(1s));
  EXPECT_TRUE(watcher.in.has_value());
}

// --- state machines --------------------------------------------------------

enum class Phase { kA, kB, kC };

struct Machine {
  struct Seen {
    Phase state;
    bool just_entered;
    Duration in_state;
  };
  explicit Machine(Node& n) : node{n}, sm{n, Phase::kA} {}
  void update() {
    seen.push_back({sm.state(), sm.just_entered(), sm.time_in_state()});
    if (node.tick() == 1) {
      sm.go(Phase::kB);
      sm.go(Phase::kB);  // no-op
    }
    if (node.tick() == 3) sm.go(Phase::kC);
  }
  Node& node;
  StateMachine<Phase> sm;
  std::vector<Seen> seen;
};

TEST(StateMachine, JustEnteredAndTimeInStateAcrossTransitions) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& machine = sim.add<Machine>("listener");  // ticks every 5 ms
  sim.step(25ms);

  ASSERT_EQ(machine.seen.size(), 5u);
  const auto expect = [&](std::size_t i, Phase state, bool entered,
                          Duration in_state) {
    EXPECT_EQ(machine.seen[i].state, state) << i;
    EXPECT_EQ(machine.seen[i].just_entered, entered) << i;
    EXPECT_EQ(machine.seen[i].in_state, in_state) << i;
  };
  expect(0, Phase::kA, true, 5ms);  // the initial state's first tick
  expect(1, Phase::kA, false, 10ms);
  expect(2, Phase::kB, true, 5ms);  // entered during tick 1
  expect(3, Phase::kB, false, 10ms);
  expect(4, Phase::kC, true, 5ms);
  EXPECT_TRUE(machine.sm.is(Phase::kC));
  EXPECT_EQ(machine.sm.previous(), Phase::kB);
}

// --- extension points ------------------------------------------------------

struct Layer {
  explicit Layer(Node& n) {
    ++alive;
    n.after_update([this] { ++flushes; });
  }
  ~Layer() { --alive; }
  int flushes = 0;
  static inline int alive = 0;
};

struct Layered {
  explicit Layered(Node& n)
      : first{&n.extension<Layer>()}, second{&n.extension<Layer>()} {
    n.after_update([this] { order.push_back("hook"); });
  }
  void update() { order.push_back("update"); }
  Layer* first;
  Layer* second;
  std::vector<std::string> order;
};

TEST(Node, ExtensionIsAPerNodeSingletonAndAfterUpdateFollowsUpdate) {
  {
    auto sim = Sim::FromToml(kRobotToml);
    auto& a = sim.add<Layered>("listener");
    auto& b = sim.add<Layered>("talker");
    EXPECT_EQ(a.first, a.second);
    EXPECT_NE(a.first, b.first);
    EXPECT_EQ(Layer::alive, 2);

    sim.step(10ms);  // listener: 2 ticks
    EXPECT_EQ(a.order,
              (std::vector<std::string>{"update", "hook", "update", "hook"}));
    EXPECT_EQ(a.first->flushes, 2);
    EXPECT_EQ(b.first->flushes, 1);
  }
  EXPECT_EQ(Layer::alive, 0);
}

struct HookOnly {
  explicit HookOnly(Node& n) {
    n.after_update([this] { ++ticks; });
  }
  int ticks = 0;
};

TEST(Node, AfterUpdateAloneStillTicks) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& node = sim.add<HookOnly>("listener");
  sim.step(20ms);
  EXPECT_EQ(node.ticks, 4);
}

struct Beats {
  explicit Beats(Node& n) {
    n.every(3ms, [this] { ++beats; });
  }
  int beats = 0;
};

TEST(Node, EveryRunsAnExtraTimer) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& node = sim.add<Beats>("listener");
  sim.step(30ms);
  EXPECT_EQ(node.beats, 10);
}

struct LateSubscriber {
  explicit LateSubscriber(Node& n) : node{n} {}
  void update() { (void)node.subscribe<Count>("/late/target"); }
  Node& node;
};

TEST(Node, RegistrationOutsideTheConstructorThrows) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<LateSubscriber>("listener");
  EXPECT_THROW(sim.step(10ms), std::logic_error);
  EXPECT_THROW(sim.node("listener").after_update([] {}), std::logic_error);
}

// --- the binary
// ----------------------------------------------------------------

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

struct Result {
  int status;
  std::string output;
};

Result RunBinary(const std::string& args) {
  const std::string command =
      "'" + Runfile("TALOS_TEST_NODE") + "' " + args + " 2>&1";
  FILE* pipe = ::popen(command.c_str(), "r");
  if (pipe == nullptr) throw std::runtime_error("popen failed");
  Result result{0, {}};
  char buffer[4096];
  while (const std::size_t n = std::fread(buffer, 1, sizeof buffer, pipe)) {
    result.output.append(buffer, n);
  }
  const int status = ::pclose(pipe);
  result.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return result;
}

std::string ConfigFlag() {
  return "--config '" + Runfile("TALOS_TEST_CONFIG") + "'";
}

TEST(Binary, DescribeListsEveryEndpointIncludingOptionalOnes) {
  const Result result = RunBinary("--describe " + ConfigFlag());
  ASSERT_EQ(result.status, 0) << result.output;
  EXPECT_NE(result.output.find(R"("name": "test_node")"), std::string::npos)
      << result.output;
  EXPECT_NE(result.output.find(R"("target": "//talOS/node:test_node")"),
            std::string::npos)
      << result.output;
  EXPECT_NE(
      result.output.find(
          R"({"kind": "WATCHER", "name": "/test_node/target", "message_bytes": 16, "external": false, "optional": true})"),
      std::string::npos)
      << result.output;
  EXPECT_NE(
      result.output.find(
          R"({"kind": "SENDER", "name": "/test_node/state", "message_bytes": 16, "external": false, "optional": false})"),
      std::string::npos)
      << result.output;
  EXPECT_NE(result.output.find(R"({"kind": "TIMER", "name": "test_node")"),
            std::string::npos)
      << result.output;
  // A node with no hardware never touches /hw/*.
  EXPECT_EQ(result.output.find("/hw/"), std::string::npos) << result.output;
}

TEST(Binary, LoggedSimRunReplaysWithoutDivergence) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string log =
      std::string{tmp != nullptr ? tmp : "/tmp"} + "/talos_node_test.tlog";
  const Result run =
      RunBinary("--sim --duration-s 1 --log '" + log + "' " + ConfigFlag());
  ASSERT_EQ(run.status, 0) << run.output;
  const Result replay = RunBinary("--replay '" + log + "' " + ConfigFlag());
  EXPECT_EQ(replay.status, 0) << replay.output;
  EXPECT_NE(replay.output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay.output;
}

}  // namespace
}  // namespace talos
