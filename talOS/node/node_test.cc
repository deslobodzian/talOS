#include "talOS/node/node.h"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "node_test/node_test_message_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/introspection/describe.h"
#include "talOS/memory/shared_memory_ptr.h"
#include "talOS/node/internal.h"
#include "talOS/node/testing.h"
#include "talOS/process/process.h"
#include "talOS/rtms/rtms.h"

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
cameras = ["front", "back"]
offsets_m = [0.5, 1, -2.25]
none = []
mixed = [1.0, "back"]
modules = [{ x_m = 0.3 }, { x_m = -0.25 }]
[subsystems.listener.limits]
max = 4.0

[subsystems.mechanism.motors.unused]
can_id = 1
)";

// --- params and identity -------------------------------------------------

struct Empty {
  explicit Empty(Node&) {}
};

template <class Fn>
void ExpectInvalidArgumentNaming(Fn&& fn,
                                 const std::vector<std::string_view>& parts) {
  try {
    fn();
    ADD_FAILURE() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    const std::string what = e.what();
    for (const std::string_view part : parts) {
      EXPECT_NE(what.find(part), std::string::npos) << part << " in " << what;
    }
  }
}

// REQ-CONTRACT-18
TEST(Node, ReadsParamsWithFallbacksAndRejectsMissingOrMistyped) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<Empty>("listener");
  const Node& node = sim.node("listener");

  EXPECT_DOUBLE_EQ(node.param<double>("gain"), 2.5);
  EXPECT_DOUBLE_EQ(node.param<double>("whole"), 3.0);  // integer as double
  EXPECT_EQ(node.param<int>("whole"), 3);
  EXPECT_EQ(node.param<std::string>("tag"), "left");
  EXPECT_TRUE(node.param<bool>("inverted"));
  EXPECT_DOUBLE_EQ(node.param<double>("limits.max"), 4.0);

  EXPECT_DOUBLE_EQ(node.param<double>("absent", 9.5), 9.5);
  EXPECT_EQ(node.param<std::string>("absent", "x"), "x");
  EXPECT_DOUBLE_EQ(node.param<double>("gain", 0.0), 2.5);

  ExpectInvalidArgumentNaming([&] { (void)node.param<double>("absent"); },
                              {"'absent'"});
  EXPECT_THROW((void)node.param<bool>("tag"), std::invalid_argument);
  EXPECT_THROW((void)node.param<int>("gain", 0), std::invalid_argument);
  EXPECT_EQ(node.param<std::uint8_t>("whole"), 3);
  EXPECT_THROW((void)node.param<std::uint8_t>("big"), std::invalid_argument);
}

// The actuation path picks a loop's scheduling tier: a node whose own
// subsystem owns an actuator (motor, digital output, or PWM output), or that
// writes a target lane, runs above the observers (sensors only, or publishing
// only its own topics), wherever the other subsystems' actuators are.
struct LanePublisher {
  explicit LanePublisher(Node& n)
      : lane{n.publish<Count>("/arm/target/auto")} {}
  Publisher<Count> lane;
};
struct StatePublisher {
  explicit StatePublisher(Node& n) : state{n.publish<Count>("state")} {}
  Publisher<Count> state;
};

// REQ-RT-08
TEST(Node, LoopTierFollowsTheActuationPath) {
  auto sim = Sim::FromToml(R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.arm.motors.lift]
can_id = 1

[subsystems.solenoid.digital_outputs.latch]
dio = 4

[subsystems.servo.pwm_outputs.hood]
channel = 0

[subsystems.gate.digital_inputs.beam]
dio = 3

[subsystems.autonomous]
[subsystems.estimator]
)");
  sim.add<Empty>("arm");
  sim.add<Empty>("solenoid");
  sim.add<Empty>("servo");
  sim.add<LanePublisher>("autonomous");
  sim.add<Empty>("gate");
  sim.add<StatePublisher>("estimator");

  EXPECT_EQ(detail::NodeAccess::loop_tier(sim.node("arm")),
            process::LoopTier::kControl);
  EXPECT_EQ(detail::NodeAccess::loop_tier(sim.node("solenoid")),
            process::LoopTier::kControl);
  EXPECT_EQ(detail::NodeAccess::loop_tier(sim.node("servo")),
            process::LoopTier::kControl);
  EXPECT_EQ(detail::NodeAccess::loop_tier(sim.node("autonomous")),
            process::LoopTier::kControl);
  EXPECT_EQ(detail::NodeAccess::loop_tier(sim.node("gate")),
            process::LoopTier::kObserver);
  EXPECT_EQ(detail::NodeAccess::loop_tier(sim.node("estimator")),
            process::LoopTier::kObserver);
}

// REQ-CONTRACT-18
TEST(Node, ArrayParamsRejectMissingAndMistypedValues) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<Empty>("listener");
  const Node& node = sim.node("listener");
  using Strings = std::vector<std::string>;
  using Doubles = std::vector<double>;

  EXPECT_EQ(node.param<Strings>("cameras"), (Strings{"front", "back"}));
  EXPECT_EQ(node.param<Doubles>("offsets_m"), (Doubles{0.5, 1.0, -2.25}));
  EXPECT_EQ(node.param<Strings>("none"), Strings{});
  EXPECT_EQ(node.param<Doubles>("none"), Doubles{});
  // A present array wins over the fallback, even an empty one.
  EXPECT_EQ(node.param<Strings>("cameras", {"x"}), (Strings{"front", "back"}));
  EXPECT_EQ(node.param<Doubles>("none", {9.0}), Doubles{});
  EXPECT_EQ(node.param<Strings>("absent", {"x"}), Strings{"x"});
  EXPECT_EQ(node.param<Doubles>("absent", {}), Doubles{});

  ExpectInvalidArgumentNaming([&] { (void)node.param<Strings>("absent"); },
                              {"'absent'"});
  EXPECT_THROW((void)node.param<Doubles>("cameras"), std::invalid_argument);
  EXPECT_THROW((void)node.param<Strings>("offsets_m", {}),
               std::invalid_argument);
  ExpectInvalidArgumentNaming([&] { (void)node.param<Doubles>("mixed"); },
                              {"'mixed'", "1"});
  EXPECT_THROW((void)node.param<Strings>("tag"), std::invalid_argument);
  EXPECT_THROW((void)node.param<Doubles>("limits", {}), std::invalid_argument);
  EXPECT_THROW((void)node.param<std::string>("cameras"), std::invalid_argument);
}

// One subsystem reads a value another owns, so it is configured once; a
// wrong subsystem or key fails construction, not a later tick.
// REQ-CONTRACT-18
TEST(Node, ParamOfReadsAnotherSubsystemOnlyInTheConstructor) {
  struct Reader {
    explicit Reader(Node& n)
        : gain{n.param_of<double>("listener", "gain")},
          second_x_m{n.param_of<double>("listener", "modules[1].x_m")},
          past_end{n.param_of<double>("listener", "modules[2].x_m", 9.5)},
          cameras{n.param_of<std::vector<std::string>>("listener", "cameras")} {
    }
    double gain;
    double second_x_m;
    double past_end;
    std::vector<std::string> cameras;
  };
  auto sim = Sim::FromToml(kRobotToml);
  const auto& reader = sim.add<Reader>("talker");
  EXPECT_DOUBLE_EQ(reader.gain, 2.5);
  EXPECT_DOUBLE_EQ(reader.second_x_m, -0.25);
  EXPECT_DOUBLE_EQ(reader.past_end, 9.5);
  EXPECT_EQ(reader.cameras, (std::vector<std::string>{"front", "back"}));
  // After construction the dependency is fixed.
  EXPECT_THROW((void)sim.node("talker").param_of<double>("listener", "gain"),
               std::logic_error);

  struct MissingKey {
    explicit MissingKey(Node& n) { (void)n.param_of<double>("listener", "x"); }
  };
  struct MissingSubsystem {
    explicit MissingSubsystem(Node& n) {
      (void)n.param_of<double>("absent", "gain", 1.0);
    }
  };
  struct Mistyped {
    explicit Mistyped(Node& n) { (void)n.param_of<bool>("listener", "tag"); }
  };
  ExpectInvalidArgumentNaming([&] { sim.add<MissingKey>("missing_key"); },
                              {"'x'", "in [subsystems.listener]"});
  EXPECT_THROW(sim.add<MissingSubsystem>("missing_subsystem"),
               std::invalid_argument);
  EXPECT_THROW(sim.add<Mistyped>("mistyped"), std::invalid_argument);
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

// A callback subscription.
struct Listener {
  explicit Listener(Node& n)
      : in{n.subscribe<Count>("/talker/state", [](const Count&) {})} {}
  void update() {}
  Subscription<Count> in;
};

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

// REQ-RT-01
TEST(Node, SubscriptionTracksFreshnessUpdatesAndAge) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& watcher = sim.add<Watcher>("listener");  // 5 ms period

  EXPECT_FALSE(watcher.in.has_value());
  // Zero-initialized before any message.
  EXPECT_EQ(watcher.in->tick_count(), 0);
  EXPECT_EQ(watcher.in.age(), Duration::max());

  sim.step(5ms);  // tick 0: nothing yet
  ASSERT_EQ(watcher.updated.size(), 1u);
  EXPECT_FALSE(watcher.updated[0]);
  EXPECT_FALSE(watcher.fresh[0]);

  // No callback, so the node drains right before its tick. Publication time
  // still counts: the 5 ms wait before the first drain must not disappear.
  sim.publish("/watch/target", Count{7, 0});
  sim.step(5ms);  // tick 1 at 10 ms
  sim.step(5ms);  // tick 2 at 15 ms: nothing new since tick 1 began
  ASSERT_EQ(watcher.updated.size(), 3u);
  EXPECT_TRUE(watcher.updated[1]);
  EXPECT_FALSE(watcher.updated[2]);
  EXPECT_TRUE(watcher.fresh[1]);
  EXPECT_EQ(watcher.ages[1], 5ms);
  EXPECT_EQ(watcher.ages[2], 10ms);
  EXPECT_EQ(watcher.in->tick_count(), 7);

  sim.step(100ms);  // past [robot] command_timeout_us
  EXPECT_FALSE(watcher.fresh.back());
  EXPECT_FALSE(watcher.in.fresh());
  EXPECT_TRUE(watcher.in.fresh(1s));
  EXPECT_TRUE(watcher.in.has_value());
}

// A callback-free node drains immediately before every timer (`update()` and
// `every()`): a message published between `update()` and `every()` is not
// drained early, and `every()` sees it at its deadline with its publication
// age, ahead of the next `update()`.
// REQ-RT-01
TEST(Node, CallbackFreeNodeDrainsBeforeEveryTimerAtItsDeadline) {
  struct DualTimerWatcher {
    struct Sample {
      Duration at;
      bool has_value;
      std::int64_t value;
      Duration age;
    };
    explicit DualTimerWatcher(Node& n)
        : node{n}, in{n.subscribe<Count>("/watch/target")} {
      n.every(3ms, [this] {
        every_seen.push_back({Duration{node.now().nanos()}, in.has_value(),
                              in->tick_count(), in.age()});
      });
    }
    void update() {
      update_seen.push_back({Duration{node.now().nanos()}, in.has_value(),
                             in->tick_count(), in.age()});
    }
    Node& node;
    Subscription<Count> in;
    std::vector<Sample> update_seen;
    std::vector<Sample> every_seen;
  };

  auto sim = Sim::FromToml(kRobotToml);
  auto& watcher = sim.add<DualTimerWatcher>("listener");  // update every 5 ms

  // First every() at 3 ms and first update() at 5 ms see nothing yet.
  sim.step(5ms);
  ASSERT_EQ(watcher.every_seen.size(), 1u);
  EXPECT_EQ(watcher.every_seen[0].at, 3ms);
  EXPECT_FALSE(watcher.every_seen[0].has_value);
  ASSERT_EQ(watcher.update_seen.size(), 1u);
  EXPECT_EQ(watcher.update_seen[0].at, 5ms);
  EXPECT_FALSE(watcher.update_seen[0].has_value);

  // Publish at 5.5 ms, between update() (5 ms) and every() (6 ms).
  sim.step(500us);
  sim.publish("/watch/target", Count{42, 0});
  // Before every()'s deadline (5.9 ms), the callback-free node has not drained.
  sim.step(400us);
  EXPECT_FALSE(watcher.in.has_value());

  // At every()'s deadline (6 ms), it drains immediately before running and sees
  // the publication at its exact age (500 us), ahead of the 10 ms update().
  sim.step(100us);
  ASSERT_EQ(watcher.every_seen.size(), 2u);
  EXPECT_EQ(watcher.every_seen[1].at, 6ms);
  EXPECT_TRUE(watcher.every_seen[1].has_value);
  EXPECT_EQ(watcher.every_seen[1].value, 42);
  EXPECT_EQ(watcher.every_seen[1].age, 500us);
  EXPECT_EQ(watcher.update_seen.size(), 1u);

  // A second publication at 7 ms is seen by every() at 9 ms (age 2 ms),
  // still ahead of the next update() at 10 ms.
  sim.step(1ms);
  sim.publish("/watch/target", Count{99, 0});
  sim.step(2ms);
  ASSERT_EQ(watcher.every_seen.size(), 3u);
  EXPECT_EQ(watcher.every_seen[2].at, 9ms);
  EXPECT_TRUE(watcher.every_seen[2].has_value);
  EXPECT_EQ(watcher.every_seen[2].value, 99);
  EXPECT_EQ(watcher.every_seen[2].age, 2ms);
  EXPECT_EQ(watcher.update_seen.size(), 1u);
}

// REQ-SAFE-08
TEST(Node, PausingOneNodeStopsItsTimersAndDeliveriesButNotItsClock) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<Talker>("talker");
  auto& watcher = sim.add<Watcher>("listener");
  sim.step(5ms);
  sim.pause("listener", 200ms);
  sim.publish("/watch/target", Count{7, 0});
  sim.step(125ms);
  EXPECT_EQ(watcher.updated.size(), 1u);
  EXPECT_FALSE(watcher.in.has_value());
  EXPECT_EQ(sim.node("listener").now(), sim.now());
  ASSERT_TRUE(sim.latest<Count>("/talker/state"));
  EXPECT_EQ(sim.latest<Count>("/talker/state")->tick_count(), 12);
  sim.resume("listener");
  ASSERT_EQ(watcher.updated.size(), 2u);
  EXPECT_TRUE(watcher.updated.back());
  EXPECT_FALSE(watcher.fresh.back());
  EXPECT_EQ(watcher.ages.back(), 125ms);
  sim.step(5ms);
  EXPECT_EQ(watcher.updated.size(), 3u);
}

TEST(Node, TimedPauseResumesExactlyAtItsDeadlineAndRejectsInvalidNodes) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& watcher = sim.add<Watcher>("listener");
  sim.pause("listener", 12ms);
  sim.step(11ms);
  EXPECT_TRUE(watcher.updated.empty());
  sim.step(1ms);
  ASSERT_EQ(watcher.updated.size(), 1u);
  sim.step(3ms);
  EXPECT_EQ(watcher.updated.size(), 2u);
  EXPECT_THROW(sim.pause("missing", 1ms), std::invalid_argument);
  EXPECT_THROW(sim.resume("missing"), std::invalid_argument);
  EXPECT_THROW(sim.pause("listener", -1ms), std::invalid_argument);
}

struct Restartable {
  enum class Mode { kIdle, kActive };
  static inline int alive = 0;
  explicit Restartable(Node& n)
      : node{n},
        sm{n, Mode::kIdle},
        plain{n.subscribe<Count>("/watch/target")},
        waking{n.subscribe<Count>("/watch/wake", [this](const Count& c) {
          woken.emplace_back(c.tick_count(), Duration{node.now().nanos()});
        })} {
    ++alive;
    n.every(3ms, [this] { beats.push_back(Duration{node.now().nanos()}); });
  }
  ~Restartable() { --alive; }
  void update() {
    ticks.push_back(Duration{node.now().nanos()});
    time_in_state.push_back(sm.time_in_state());
    just_entered.push_back(sm.just_entered());
  }
  Node& node;
  StateMachine<Mode> sm;
  Subscription<Count> plain;
  Subscription<Count> waking;
  std::vector<Duration> ticks;
  std::vector<Duration> beats;
  std::vector<Duration> time_in_state;
  std::vector<bool> just_entered;
  std::vector<std::pair<std::int64_t, Duration>> woken;
};

// REQ-SAFE-31: removing a node destroys it and stops its timers and callbacks;
// re-adding it mid-run starts its clock at sim.now() without catch-up timers
// from t = 0 and delivers only messages published after it attached.
TEST(Sim, RemoveDestroysNodeAndMidRunAddStartsAtNowWithoutCatchUp) {
  Restartable::alive = 0;

  auto sim = Sim::FromToml(kRobotToml);
  auto& first = sim.add<Restartable>("listener");  // 5 ms period
  ASSERT_EQ(Restartable::alive, 1);
  sim.publish("/watch/target", Count{1, 0});
  sim.step(20ms);
  ASSERT_EQ(first.ticks.size(), 4u);
  ASSERT_TRUE(first.plain.has_value());

  sim.remove("listener");
  EXPECT_EQ(Restartable::alive, 0);
  EXPECT_THROW((void)sim.node("listener"), std::invalid_argument);

  // Publications while removed and at the exact restart instant (before add)
  // must not be delivered to the restarted instance.
  sim.publish("/watch/target", Count{2, 0});
  sim.publish("/watch/wake", Count{20, 0});
  sim.step(5ms);  // now = 25 ms
  sim.publish("/watch/target", Count{3, 0});
  sim.publish("/watch/wake", Count{30, 0});

  auto& restarted = sim.add<Restartable>("listener");
  ASSERT_EQ(Restartable::alive, 1);
  EXPECT_EQ(sim.now(), Time{} + 25ms);
  EXPECT_EQ(sim.node("listener").now(), sim.now());
  EXPECT_EQ(sim.node("listener").tick(), 0u);

  // Stepping 0 ms starts the loop at 25 ms with no catch-up firings from 0.
  sim.step(0ms);
  EXPECT_TRUE(restarted.ticks.empty());
  EXPECT_TRUE(restarted.beats.empty());
  EXPECT_TRUE(restarted.woken.empty());
  EXPECT_FALSE(restarted.plain.has_value());

  // First every(3ms) fires at 28 ms and first update(5ms) at 30 ms.
  sim.step(5ms);
  EXPECT_EQ(restarted.beats, (std::vector<Duration>{28ms}));
  EXPECT_EQ(restarted.ticks, (std::vector<Duration>{30ms}));
  EXPECT_EQ(restarted.time_in_state, (std::vector<Duration>{5ms}));
  EXPECT_EQ(restarted.just_entered, (std::vector<bool>{true}));
  EXPECT_FALSE(restarted.plain.has_value());
  EXPECT_TRUE(restarted.woken.empty());

  // Messages published after attach reach both subscriptions normally.
  sim.publish("/watch/target", Count{4, 0});
  sim.publish("/watch/wake", Count{40, 0});
  sim.step(5ms);  // now = 35 ms
  EXPECT_EQ(restarted.woken,
            (std::vector<std::pair<std::int64_t, Duration>>{{40, 30ms}}));
  ASSERT_TRUE(restarted.plain.has_value());
  EXPECT_EQ(restarted.plain->tick_count(), 4);
  EXPECT_EQ(restarted.plain.age(), 5ms);
}

TEST(Sim, RemoveAndMidRunAddRejectMissingDuplicateAndReentrantCalls) {
  struct Reentrant {
    explicit Reentrant(Node& n)
        : in{n.subscribe<Count>("/reenter/wake", [this](const Count&) {
            if (on_wake) on_wake();
          })} {}
    void update() {
      if (on_update) on_update();
    }
    Subscription<Count> in;
    std::function<void()> on_update;
    std::function<void()> on_wake;
  };

  auto sim = Sim::FromToml(kRobotToml);
  EXPECT_THROW(sim.remove("missing"), std::invalid_argument);
  auto& node = sim.add<Reentrant>("listener");
  EXPECT_THROW(sim.add<Empty>("listener"), std::invalid_argument);

  // Calling remove() or add() inside step() throws std::logic_error and leaves
  // the Sim usable once step() unwinds.
  node.on_update = [&] { sim.remove("listener"); };
  EXPECT_THROW(sim.step(5ms), std::logic_error);
  // Throwing inside Round(5ms) left sim.now() at 0 while the node's periodic
  // timer already advanced to 10 ms, so step 10 ms to reach the next tick.
  node.on_update = [&] { sim.add<Empty>("talker"); };
  EXPECT_THROW(sim.step(10ms), std::logic_error);
  node.on_update = nullptr;

  node.on_wake = [&] { sim.remove("listener"); };
  sim.publish("/reenter/wake", Count{1, 0});
  EXPECT_THROW(sim.step(1ms), std::logic_error);
  node.on_wake = nullptr;

  // After stepping, duplicate name is still refused while hosted; remove works
  // once and refuses a second remove.
  EXPECT_THROW(sim.add<Empty>("listener"), std::invalid_argument);
  sim.remove("listener");
  EXPECT_THROW(sim.remove("listener"), std::invalid_argument);
  sim.add<Empty>("talker");
  sim.step(5ms);
}

// A node of type T named "listener" on a simulated loop the test holds, so
// that it can read what testing::Sim keeps inside its loops.
template <class T>
struct OnOwnLoop {
  OnOwnLoop() {
    const std::string& config_path = detail::SimAccess::config_path(sim);
    node = detail::NodeAccess::create(detail::NodeSetup{
        .name = "listener",
        .config_path = config_path,
        .config = std::make_shared<const config::RobotConfig>(
            config::ParseRobotConfig(config_path)),
        .simulation = true,
        .loop = &port,
    });
    user = detail::MakeInstance<T>()(*node);
    detail::NodeAccess::start(*node);
  }
  T& get() { return *static_cast<T*>(user.get()); }

  Sim sim = Sim::FromToml(kRobotToml);  // for its config file
  event::SimulationEnvironment environment;
  event::SimulatedEventLoop<> loop{environment};
  detail::LoopPortFor<event::SimulatedEventLoop<>> port{loop};
  std::unique_ptr<Node> node;
  detail::Instance user;
};

struct Backlog {
  explicit Backlog(Node& n)
      : latest{n.subscribe<Count>("/backlog")},
        ordered{n.subscribe<Count>("/backlog", [this](const Count& c) {
          received.push_back(c.tick_count());
          if (loop != nullptr) drain_of.push_back(loop->drains());
        })} {}
  void update() {}
  Subscription<Count> latest;
  Subscription<Count> ordered;
  std::vector<std::int64_t> received;
  // The loop's drain count at each callback, once the test gives the loop.
  const event::SimulatedEventLoop<>* loop = nullptr;
  std::vector<std::uint64_t> drain_of;
};

// A plain subscription is a latest value: one drain jumps a backlog longer
// than the per-drain cap to its newest message, aged from publication. A
// callback subscription still sees every message in order, 64 per drain:
// its publications wake the node, so the drains run back to back at the
// publication instant.
// REQ-RT-02
TEST(Node, PlainSubscriptionSkipsBacklogWhileCallbacksKeepCappedSequence) {
  OnOwnLoop<Backlog> hosted;
  Backlog& backlog = hosted.get();
  backlog.loop = &hosted.loop;
  const event::MonotonicTime at = event::MonotonicTime{} + 10ms;
  hosted.loop.run_until(at);
  for (std::int64_t i = 0; i < 130; ++i) {
    hosted.loop.inject("/backlog", Count{i, 0});
  }

  hosted.loop.run_until(at + 1ms);
  EXPECT_EQ(backlog.latest->tick_count(), 129);
  EXPECT_EQ(backlog.latest.age(), 1ms);
  ASSERT_EQ(backlog.received.size(), 130u);
  ASSERT_EQ(backlog.drain_of.size(), 130u);
  for (std::size_t i = 0; i < 130; ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(backlog.received[i], static_cast<std::int64_t>(i));
    EXPECT_EQ(backlog.drain_of[i] - backlog.drain_of[0], i / 64);
  }
}

// --- chronological causality ---------------------------------------------

constexpr const char* kCausalToml = R"(
[robot]
period_us = 1000
command_timeout_us = 100000

[subsystems.producer]
period_us = 1000

[subsystems.consumer]
period_us = 500

[subsystems.hearer]
period_us = 500

[subsystems.mechanism.motors.unused]
can_id = 1
)";

struct Producer {
  explicit Producer(Node& n)
      : node{n}, out{n.publish<Count>("/producer/state")} {}
  void update() {
    out.send(Count{static_cast<std::int64_t>(node.tick()), node.now().nanos()});
  }
  Node& node;
  Publisher<Count> out;
};

// What one consumer update() saw: its time, and the newest message's
// publication time (stamped by the producer) and age as delivered.
struct Seen {
  Duration now;
  Duration stamp;
  Duration age;
  bool fresh;
};

// A plain subscription: the node drains right before each update().
struct Consumer {
  explicit Consumer(Node& n)
      : node{n}, in{n.subscribe<Count>("/producer/state")} {}
  void update() {
    if (!in.has_value()) return;
    seen.push_back({Duration{node.now().nanos()}, Duration{in->stamp_ns()},
                    in.age(), in.fresh()});
  }
  Node& node;
  Subscription<Count> in;
  std::vector<Seen> seen;
};

// A callback subscription: each publication wakes the node, which the Sim
// models as a drain at the publication's instant, in sequence.
struct Hearer {
  explicit Hearer(Node& n)
      : node{n},
        in{n.subscribe<Count>("/producer/state", [this](const Count& c) {
          heard.emplace_back(Duration{node.now().nanos()},
                             Duration{c.stamp_ns()});
        })} {}
  Node& node;
  Subscription<Count> in;
  std::vector<std::pair<Duration, Duration>> heard;  // (delivered, stamp)
};

// A 1 ms producer and 500 us consumers in either insertion order: no
// consumer ever sees a publication from its future, so no age is negative.
// The only difference between the orders is the documented equal-time rule:
// at an instant several nodes run, the earlier-added one runs first, so a
// consumer added after the producer sees that instant's message at once, and
// one added before sees it at its next drain. A callback subscriber is woken
// by the publication instead, so it hears each one at its instant in either
// order: one added before the producer runs again at that instant.
// REQ-RT-05
TEST(Sim, LoopsAdvanceOnOneChronologicalFrontierInEitherInsertionOrder) {
  using std::chrono::milliseconds;
  for (const bool producer_first : {true, false}) {
    SCOPED_TRACE(producer_first ? "producer first" : "consumer first");
    auto sim = Sim::FromToml(kCausalToml);
    if (producer_first) sim.add<Producer>("producer");
    auto& consumer = sim.add<Consumer>("consumer");
    auto& hearer = sim.add<Hearer>("hearer");
    if (!producer_first) sim.add<Producer>("producer");
    sim.step(20ms);
    sim.pause("consumer", 2500us);
    sim.step(3ms);
    sim.step(17ms);

    // Every update from 1 ms (or 1.5 ms) on saw a message; the pause
    // coalesced the five missed ticks into one at its end, 22.5 ms.
    EXPECT_EQ(consumer.seen.size(), producer_first ? 75u : 74u);
    for (const Seen& s : consumer.seen) {
      SCOPED_TRACE(s.now.count());
      EXPECT_GE(s.age, Duration::zero());
      EXPECT_EQ(s.age, s.now - s.stamp);
      // The newest publication by then: this instant's when the producer
      // ran first, otherwise the previous millisecond's.
      EXPECT_EQ(s.stamp, producer_first
                             ? std::chrono::floor<milliseconds>(s.now)
                             : std::chrono::ceil<milliseconds>(s.now) - 1ms);
      EXPECT_TRUE(s.fresh);
    }

    // Every publication reaches the ordered subscriber once, at its instant,
    // the last one (at 40 ms) included.
    const auto latest = sim.latest<Count>("/producer/state");
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->tick_count(), 39);
    ASSERT_EQ(hearer.heard.size(), 40u);
    for (std::size_t i = 0; i < hearer.heard.size(); ++i) {
      const auto [delivered, stamp] = hearer.heard[i];
      EXPECT_EQ(stamp, milliseconds{i + 1});
      EXPECT_EQ(delivered, stamp);
    }
  }
}

// Hears /producer/state and passes each message on, as it arrives, on
// /relay/state.
struct Relay {
  explicit Relay(Node& n)
      : out{n.publish<Count>("/relay/state")},
        in{n.subscribe<Count>("/producer/state",
                              [this](const Count& c) { out.send(c); })} {}
  Publisher<Count> out;
  Subscription<Count> in;
};

// Hears /relay/state.
struct RelayHearer {
  explicit RelayHearer(Node& n)
      : node{n}, in{n.subscribe<Count>("/relay/state", [this](const Count& c) {
          heard.emplace_back(Duration{node.now().nanos()},
                             Duration{c.stamp_ns()});
        })} {}
  Node& node;
  Subscription<Count> in;
  std::vector<std::pair<Duration, Duration>> heard;  // (delivered, stamp)
};

// A chain of callbacks crosses nodes added in reverse order within the
// instant of the first publication: each hop runs another round there.
// REQ-RT-05
TEST(Sim, CallbackChainAcrossNodesSettlesAtThePublicationInstant) {
  auto sim = Sim::FromToml(kCausalToml);
  auto& hearer = sim.add<RelayHearer>("hearer");
  sim.add<Relay>("relay");
  sim.add<Producer>("producer");
  sim.step(5ms);
  ASSERT_EQ(hearer.heard.size(), 5u);
  for (const auto& [delivered, stamp] : hearer.heard) {
    EXPECT_EQ(delivered, stamp);
  }
}

// The end of a chain: hears /relay/state, and records at each update()
// whether it had heard anything.
struct ChainEnd {
  explicit ChainEnd(Node& n)
      : node{n}, in{n.subscribe<Count>("/relay/state", [this](const Count&) {
          heard_at.push_back(Duration{node.now().nanos()});
        })} {}
  void update() { had_value.push_back(in.has_value()); }
  Node& node;
  Subscription<Count> in;
  std::vector<Duration> heard_at;
  std::vector<bool> had_value;
};

// The start of a chain: passes what the test publishes on /test/start on to
// /producer/state as it arrives.
struct ChainStart {
  explicit ChainStart(Node& n)
      : out{n.publish<Count>("state")},
        in{n.subscribe<Count>("/test/start",
                              [this](const Count& c) { out.send(c); })} {}
  Publisher<Count> out;
  Subscription<Count> in;
};

// On the first step every node's loop starts at time 0, and the last one
// added can then run a callback that wakes one started before it. A
// publication made before the first step travels the whole chain at 0, so
// the end's first update(), at 1 ms, has it.
// REQ-RT-05
TEST(Sim, TheFirstStepSettlesAChainStartedBeforeIt) {
  auto sim = Sim::FromToml(R"(
[robot]
period_us = 1000
command_timeout_us = 100000

[subsystems.mechanism.motors.unused]
can_id = 1
)");
  auto& end = sim.add<ChainEnd>("hearer");
  sim.add<Relay>("relay");
  sim.add<ChainStart>("producer");
  sim.publish("/test/start", Count{7, 0});
  sim.step(5ms);
  EXPECT_EQ(end.heard_at, std::vector<Duration>{Duration::zero()});
  ASSERT_FALSE(end.had_value.empty());
  EXPECT_TRUE(end.had_value.front())
      << "the 1 ms update() ran without the message published at 0";
}

// A paused node hears nothing; when the pause ends it hears every
// publication it missed at once, in sequence, and one made at that instant
// by a node added after it in the round that follows, not at its next
// event.
// REQ-RT-05
TEST(Sim, APausedNodeHearsWhatItMissedWhenThePauseEnds) {
  using std::chrono::milliseconds;
  auto sim = Sim::FromToml(kCausalToml);
  auto& hearer = sim.add<Hearer>("hearer");
  sim.add<Producer>("producer");
  sim.step(5ms);
  sim.pause("hearer", 10ms);
  sim.step(9ms);
  EXPECT_EQ(hearer.heard.size(), 5u);
  sim.step(6ms);
  ASSERT_EQ(hearer.heard.size(), 20u);
  for (std::size_t i = 0; i < hearer.heard.size(); ++i) {
    SCOPED_TRACE(i);
    const auto [delivered, stamp] = hearer.heard[i];
    EXPECT_EQ(stamp, milliseconds{i + 1});
    const bool missed = stamp > 5ms && stamp <= 15ms;
    EXPECT_EQ(delivered, missed ? Duration{15ms} : stamp);
  }
}

// Two nodes whose callbacks answer each other at once never let time pass.
// The step fails after exactly 1000 rounds at one instant, naming every node
// still due and every due prompt topic, instead of hanging the test.
struct Echo {
  explicit Echo(Node& n)
      : out{n.publish<Count>("state")},
        in{n.subscribe<Count>(
            n.name() == "ping" ? "/pong/state" : "/ping/state",
            [this](const Count& c) {
              ++callbacks;
              out.send(c);
            })} {}
  Publisher<Count> out;
  Subscription<Count> in;
  int callbacks = 0;
};

// REQ-RT-05
TEST(Sim, AZeroTimePublishCycleFailsTheStepNamingWhatIsDue) {
  auto sim = Sim::FromToml(kCausalToml);
  auto& ping = sim.add<Echo>("ping");
  auto& pong = sim.add<Echo>("pong");
  // Start both loops before injecting so Settle() accounts for every round.
  sim.step(0ms);
  sim.publish("/pong/state", Count{0, 0});
  try {
    sim.step(1ms);
    FAIL() << "a zero-time cycle returned";
  } catch (const std::runtime_error& error) {
    EXPECT_EQ(ping.callbacks, 1000);
    EXPECT_EQ(pong.callbacks, 1000);
    const std::string what = error.what();
    EXPECT_NE(what.find("1000 rounds"), std::string::npos) << what;
    EXPECT_NE(what.find("'ping'"), std::string::npos) << what;
    EXPECT_NE(what.find("/pong/state"), std::string::npos) << what;
  }
}

// When a zero-time cycle leaves multiple nodes simultaneously due and a node
// has multiple due prompt topics, the failure after 1000 rounds names each due
// node and each of its due prompt topics.
// REQ-RT-05
TEST(Sim, AZeroTimeCycleNamesMultipleDueNodesAndMultipleDuePromptTopics) {
  struct MultiPrompt {
    explicit MultiPrompt(Node& n)
        : out{n.publish<Count>("state")},
          in_state{n.subscribe<Count>("/gamma/state",
                                      [this](const Count& c) {
                                        ++state_callbacks;
                                        out.send(c);
                                      })},
          in_status{n.subscribe<Count>(
              "/gamma/status", [this](const Count&) { ++status_callbacks; })} {}
    Publisher<Count> out;
    Subscription<Count> in_state;
    Subscription<Count> in_status;
    int state_callbacks = 0;
    int status_callbacks = 0;
  };
  struct PeerWatcher {
    explicit PeerWatcher(Node& n)
        : in{n.subscribe<Count>("/gamma/state",
                                [this](const Count&) { ++callbacks; })} {}
    Subscription<Count> in;
    int callbacks = 0;
  };
  struct Broadcaster {
    explicit Broadcaster(Node& n)
        : out_state{n.publish<Count>("state")},
          out_status{n.publish<Count>("status")},
          in{n.subscribe<Count>("/alpha/state", [this](const Count& c) {
            ++callbacks;
            out_state.send(c);
            out_status.send(c);
          })} {}
    Publisher<Count> out_state;
    Publisher<Count> out_status;
    Subscription<Count> in;
    int callbacks = 0;
  };

  auto sim = Sim::FromToml(kCausalToml);
  auto& alpha = sim.add<MultiPrompt>("alpha");
  auto& beta = sim.add<PeerWatcher>("beta");
  auto& gamma = sim.add<Broadcaster>("gamma");
  sim.step(0ms);
  sim.publish("/gamma/state", Count{0, 0});
  sim.publish("/gamma/status", Count{0, 0});
  try {
    sim.step(1ms);
    FAIL() << "a zero-time cycle returned";
  } catch (const std::runtime_error& error) {
    EXPECT_EQ(alpha.state_callbacks, 1000);
    EXPECT_EQ(alpha.status_callbacks, 1000);
    EXPECT_EQ(beta.callbacks, 1000);
    EXPECT_EQ(gamma.callbacks, 1000);
    const std::string what = error.what();
    EXPECT_NE(what.find("1000 rounds"), std::string::npos) << what;
    EXPECT_NE(what.find("node 'alpha' (/gamma/state, /gamma/status)"),
              std::string::npos)
        << what;
    EXPECT_NE(what.find("node 'beta' (/gamma/state)"), std::string::npos)
        << what;
    EXPECT_EQ(what.find("node 'gamma'"), std::string::npos) << what;
  }
}

struct RelativeWrist {
  explicit RelativeWrist(Node& n)
      : target{n.subscribe<Count>("target/teleop")},
        external{n.subscribe<Count>("/arm/state", {.optional = true})},
        state{n.publish<Count>("state")},
        frames{n.publish_bytes("frames", 8)} {}
  void update() {
    if (target.has_value()) {
      state.send(Count{target->tick_count() + external->tick_count(), 0});
      const std::byte b{static_cast<unsigned char>(target->tick_count())};
      frames.send(std::span<const std::byte>{&b, 1});
    }
  }
  Subscription<Count> target;
  Subscription<Count> external;
  Publisher<Count> state;
  BytePublisher frames;
};

TEST(Node, RelativeTopicsExpandAndRemapRedirectsRelativeAndAbsoluteTopics) {
  auto sim = Sim::FromToml(R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.wrist.motors.m]
can_id = 1

[subsystems.left_wrist.remap]
"target/teleop" = "/teleop/left_wrist"
"/left_wrist/state" = "/wrist/left_state"
"/arm/state" = "/left_arm/state"
"frames" = "/left_wrist/raw_frames"
)");
  sim.add<RelativeWrist>("wrist");
  sim.add<RelativeWrist>("left_wrist");

  sim.publish("/wrist/target/teleop", Count{3, 0});
  sim.publish("/arm/state", Count{10, 0});
  sim.publish("/teleop/left_wrist", Count{7, 0});
  sim.publish("/left_arm/state", Count{20, 0});
  sim.step(10ms);

  const auto wrist_state = sim.latest<Count>("/wrist/state");
  ASSERT_TRUE(wrist_state.has_value());
  const Count wrist = wrist_state.value_or(Count{});
  EXPECT_EQ(wrist.tick_count(), 13);
  const auto wrist_frames = sim.latest_bytes("/wrist/frames");
  ASSERT_TRUE(wrist_frames.has_value());
  const std::vector<std::byte> wframes =
      wrist_frames.value_or(std::vector<std::byte>{});
  EXPECT_EQ(wframes, std::vector<std::byte>{std::byte{3}});

  const auto left_state = sim.latest<Count>("/wrist/left_state");
  ASSERT_TRUE(left_state.has_value());
  const Count left = left_state.value_or(Count{});
  EXPECT_EQ(left.tick_count(), 27);
  const auto left_frames = sim.latest_bytes("/left_wrist/raw_frames");
  ASSERT_TRUE(left_frames.has_value());
  const std::vector<std::byte> lframes =
      left_frames.value_or(std::vector<std::byte>{});
  EXPECT_EQ(lframes, std::vector<std::byte>{std::byte{7}});
}

TEST(Node, NonStringRemapEntryThrows) {
  auto sim = Sim::FromToml(R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.wrist.motors.m]
can_id = 1
[subsystems.wrist.remap]
state = 42
)");
  EXPECT_THROW(sim.add<RelativeWrist>("wrist"), std::invalid_argument);
}

// RULES.md 2: a hardware request leaves only through the devices layer's
// flush, which gates it on robot_enabled and update() and applies the motor
// limits. A node publishing its own /hw/request with a plain publisher, typed
// or bytes, spelled out or reached by remap, is refused at construction, which
// --allow-graph-errors cannot override; the C ABI (Python) shares this path.
struct PlainRequest {
  explicit PlainRequest(Node& n)
      : request{n.publish<Count>("/hw/request/plain")} {}
  Publisher<Count> request;
};
struct ByteRequest {
  explicit ByteRequest(Node& n)
      : request{n.publish_bytes("/hw/request/bytes", 8)} {}
  BytePublisher request;
};
struct RemappedRequest {
  explicit RemappedRequest(Node& n) : request{n.publish<Count>("state")} {}
  Publisher<Count> request;
};
struct DoubleSlashRequest {
  explicit DoubleSlashRequest(Node& n)
      : request{n.publish<Count>("//hw/request/double")} {}
  Publisher<Count> request;
};
struct TripleSlashRequest {
  explicit TripleSlashRequest(Node& n)
      : request{n.publish<Count>("///hw/request/triple")} {}
  Publisher<Count> request;
};
struct DoubleSlashByteRequest {
  explicit DoubleSlashByteRequest(Node& n)
      : request{n.publish_bytes("//hw/request/bytes_double", 8)} {}
  BytePublisher request;
};
struct DotCollisionRequest {
  explicit DotCollisionRequest(Node& n)
      : request{n.publish<Count>("/hw.request.dot")} {}
  Publisher<Count> request;
};

// REQ-SAFE-27
TEST(Node, APlainPublisherOnAHardwareRequestIsRefused) {
  auto sim = Sim::FromToml(R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.plain]
[subsystems.bytes]
[subsystems.double_slash]
[subsystems.triple_slash]
[subsystems.bytes_double]
[subsystems.dot]
[subsystems.hashed_shm.remap]
state = "/~deadbeef12345678"
[subsystems.remapped.remap]
state = "/hw/request/remapped"
[subsystems.remapped_double.remap]
state = "//hw/request/remapped"

[subsystems.mechanism.motors.unused]
can_id = 1
)");
  EXPECT_THROW(sim.add<PlainRequest>("plain"), std::invalid_argument);
  EXPECT_THROW(sim.add<ByteRequest>("bytes"), std::invalid_argument);
  EXPECT_THROW(sim.add<DoubleSlashRequest>("double_slash"),
               std::invalid_argument);
  EXPECT_THROW(sim.add<TripleSlashRequest>("triple_slash"),
               std::invalid_argument);
  EXPECT_THROW(sim.add<DoubleSlashByteRequest>("bytes_double"),
               std::invalid_argument);
  EXPECT_THROW(sim.add<DotCollisionRequest>("dot"), std::invalid_argument);
  EXPECT_THROW(sim.add<RemappedRequest>("hashed_shm"), std::invalid_argument);
  EXPECT_THROW(sim.add<RemappedRequest>("remapped"), std::invalid_argument);
  EXPECT_THROW(sim.add<RemappedRequest>("remapped_double"),
               std::invalid_argument);
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

struct Silent {
  explicit Silent(Node& n) : in{n.subscribe<Count>("/watch/target")} {}
  Subscription<Count> in;
};

// Two raw callback subscriptions, as the devices layer opens /hw/state and
// the Driver Station link; only the second has a handler that must run as
// each message arrives (a Driver Station listener), when `listen`.
template <bool listen>
struct RawCallbacks {
  explicit RawCallbacks(Node& n) {
    const auto open = [&n](std::string_view topic) {
      return detail::NodeAccess::add_subscription(
          n, topic, sizeof(Count), kMessageAlignment, schema::Of<Count>(),
          /*optional=*/false, [](const void*) {});
    };
    open("/feed/state");
    [[maybe_unused]] const std::size_t link = open("/link/state");
    if constexpr (listen) {
      detail::NodeAccess::require_immediate_delivery(n, link);
    }
  }
  void update() {}
};

struct Wakes {
  event::Delivery policy;
  std::vector<bool> prompt;  // per watcher, in registration order
};

// The loop's delivery as NodeAccess::start picks it for T on node
// "listener", and which of its watchers wake it.
template <class T>
Wakes DeliveryFor() {
  const OnOwnLoop<T> hosted;
  Wakes wakes{.policy = hosted.loop.delivery(), .prompt = {}};
  for (const event::Registration& source : hosted.loop.manifest()) {
    if (source.kind == event::SourceKind::WATCHER) {
      wakes.prompt.push_back(hosted.loop.prompt(source.id));
    }
  }
  return wakes;
}

// A Wake::kAtDrain callback subscription, with a timer (AtDrain) or without
// one (SilentAtDrain).
struct AtDrain {
  explicit AtDrain(Node& n)
      : in{n.subscribe<Count>(
            "/feed/state", [](const Count&) {}, Wake::kAtDrain)} {}
  void update() {}
  Subscription<Count> in;
};

struct SilentAtDrain {
  explicit SilentAtDrain(Node& n)
      : in{n.subscribe<Count>(
            "/feed/state", [](const Count&) {}, Wake::kAtDrain)} {}
  Subscription<Count> in;
};

// Every message its callbacks heard, in order, with the instant of the
// drain that delivered it: /feed/state's with Wake::kAtDrain, then
// /poke/state's (heard as -1) with the default Wake::kOnPublish; and the
// feed subscription's age() inside each of its callbacks.
struct Collector {
  explicit Collector(Node& n)
      : node{n},
        feed{n.subscribe<Count>(
            "/feed/state",
            [this](const Count& c) {
              feed_ages.push_back(feed.age());
              Hear(c.tick_count());
            },
            Wake::kAtDrain)},
        poke{n.subscribe<Count>("/poke/state",
                                [this](const Count&) { Hear(-1); })} {}
  void update() {}
  void Hear(std::int64_t value) {
    heard.emplace_back(value, Duration{node.now().nanos()});
  }
  Node& node;
  Subscription<Count> feed;
  Subscription<Count> poke;
  std::vector<std::pair<std::int64_t, Duration>> heard;  // (value, delivered)
  std::vector<Duration> feed_ages;
};

// A node that only reads latched values at its own timers drains
// right before them and is never woken by a publication. A subscribe
// callback, or a listener on one raw subscription, makes that one
// subscription wake the node, not the node's other callback subscriptions,
// unless it is opened with Wake::kAtDrain; a node with no timer to drain
// before is woken by every subscription.
// REQ-RT-19
TEST(Node, OnlyASubscriptionWhoseHandlerMustRunAtOnceWakesTheNode) {
  constexpr auto BEFORE_TIMERS = event::Delivery::BEFORE_TIMERS;
  constexpr auto ON_PUBLISH = event::Delivery::ON_PUBLISH;
  const auto expect = [](const char* node, const Wakes& got,
                         event::Delivery policy,
                         const std::vector<bool>& prompt) {
    SCOPED_TRACE(node);
    EXPECT_EQ(got.policy, policy);
    EXPECT_EQ(got.prompt, prompt);
  };
  expect("timer reads a latched value", DeliveryFor<Watcher>(), BEFORE_TIMERS,
         {false});
  expect("timers only", DeliveryFor<Beats>(), BEFORE_TIMERS, {});
  expect("callbacks without a listener", DeliveryFor<RawCallbacks<false>>(),
         BEFORE_TIMERS, {false, false});
  expect("a listener on the second", DeliveryFor<RawCallbacks<true>>(),
         ON_PUBLISH, {false, true});
  expect("subscribe callback", DeliveryFor<Listener>(), ON_PUBLISH, {true});
  expect("at-drain callback", DeliveryFor<AtDrain>(), BEFORE_TIMERS, {false});
  expect("at-drain beside a waking callback", DeliveryFor<Collector>(),
         ON_PUBLISH, {false, true});
  expect("no timer", DeliveryFor<Silent>(), ON_PUBLISH, {true});
  expect("no timer, at-drain callback", DeliveryFor<SilentAtDrain>(),
         ON_PUBLISH, {true});
}

// A Wake::kAtDrain callback hears every message, in order, but only at the
// node's next drain, here its 5 ms update(): publications between ticks wake
// nothing. A drain delivers at most 64 of them, and one that stops there is
// followed at once by another, so a backlog over the cap is heard whole at
// that instant, as a waking callback's is, not one batch per tick. Inside
// the callback age() is that message's, counted from its publication.
// REQ-RT-02
// REQ-RT-19
TEST(Node, AnAtDrainCallbackHearsEveryMessageInOrderAtTheNextDrain) {
  using Heard = std::vector<std::pair<std::int64_t, Duration>>;
  auto sim = Sim::FromToml(kRobotToml);
  auto& collector = sim.add<Collector>("listener");  // update() every 5 ms
  sim.step(6ms);
  for (std::int64_t i = 0; i < 3; ++i) {
    sim.publish("/feed/state", Count{i, 0});
    sim.step(1ms);
  }
  EXPECT_EQ(collector.heard, Heard{})
      << "a publication at 6, 7 or 8 ms woke the node";
  sim.step(1ms);
  EXPECT_EQ(collector.heard, (Heard{{0, 10ms}, {1, 10ms}, {2, 10ms}}));
  EXPECT_EQ(collector.feed_ages, (std::vector<Duration>{4ms, 3ms, 2ms}));

  collector.heard.clear();
  for (std::int64_t i = 0; i < 70; ++i) {
    sim.publish("/feed/state", Count{i, 0});
  }
  sim.step(4ms);
  EXPECT_EQ(collector.heard, Heard{}) << "published at 10 ms, after its drain";
  sim.step(1ms);
  ASSERT_EQ(collector.heard.size(), 70u) << "the rest waited past 15 ms";
  for (std::size_t i = 0; i < collector.heard.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(collector.heard[i].first, static_cast<std::int64_t>(i));
    EXPECT_EQ(collector.heard[i].second, 15ms);
  }
}

// A wake from another topic drains every subscription, so the Wake::kAtDrain
// messages waiting then are heard at that wake, first (registration order),
// not at the next tick.
// REQ-RT-02
TEST(Node, AnotherTopicsWakeDeliversWaitingAtDrainMessages) {
  using Heard = std::vector<std::pair<std::int64_t, Duration>>;
  auto sim = Sim::FromToml(kRobotToml);
  auto& collector = sim.add<Collector>("listener");
  sim.step(6ms);
  sim.publish("/feed/state", Count{7, 0});
  sim.step(1ms);
  EXPECT_EQ(collector.heard, Heard{});
  sim.publish("/poke/state", Count{0, 0});
  sim.step(1ms);
  EXPECT_EQ(collector.heard, (Heard{{7, 7ms}, {-1, 7ms}}));
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

// --- byte topics -----------------------------------------------------------

struct Framer {
  explicit Framer(Node& n)
      : node{n}, out{n.publish_bytes("/framer/frames", 8)} {}
  // Tick k sends k + 1 bytes of value k: the length varies per message.
  void update() {
    const std::vector<std::byte> frame(node.tick() + 1,
                                       static_cast<std::byte>(node.tick()));
    sent.push_back(out.send(frame));
  }
  Node& node;
  BytePublisher out;
  std::vector<bool> sent;
};

TEST(BytePublisher, SimKeepsTheNewestMessageAsSentAndRefusesOversize) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& framer = sim.add<Framer>("talker");  // 10 ms period
  EXPECT_FALSE(sim.latest_bytes("/framer/frames").has_value());

  sim.step(35ms);  // ticks 0..2
  auto latest = sim.latest_bytes("/framer/frames");
  if (!latest.has_value()) {
    FAIL() << "nothing published on /framer/frames";
  }
  EXPECT_EQ(*latest, std::vector<std::byte>(3, std::byte{2}));

  sim.step(100ms);  // ticks 3..12; from tick 8 the frame exceeds 8 bytes
  latest = sim.latest_bytes("/framer/frames");
  if (!latest.has_value()) {
    FAIL() << "nothing published on /framer/frames";
  }
  EXPECT_EQ(*latest, std::vector<std::byte>(8, std::byte{7}));
  ASSERT_EQ(framer.sent.size(), 13u);
  for (std::size_t k = 0; k < framer.sent.size(); ++k) {
    EXPECT_EQ(framer.sent[k], k < 8) << k;
  }
  const Node& node = sim.node("talker");
  EXPECT_EQ(detail::NodeAccess::byte_topic_published(node, 0), 8u);
  EXPECT_EQ(detail::NodeAccess::byte_topic_dropped(node, 0), 5u);
}

struct BadFramer {
  explicit BadFramer(Node& n) {
    EXPECT_THROW((void)n.publish_bytes("/bad/frames", 0),
                 std::invalid_argument);
    EXPECT_THROW((void)n.publish_bytes("/bad/frames", 8, {.slots = 3}),
                 std::invalid_argument);
    EXPECT_THROW((void)n.publish_bytes("/bad/frames", 8, {.slots = 2048}),
                 std::invalid_argument);
  }
};

TEST(BytePublisher, RejectsBadLayouts) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<BadFramer>("listener");
}

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

bool ShmSegmentExists(const std::string& topic) {
  const int fd = ::shm_open(ShmObjectName(topic).c_str(), O_RDWR, 0666);
  if (fd == -1) return false;
  ::close(fd);
  return true;
}

// In a Sim a graph reader sees every node added before it, schemas included,
// plus Pass-1 static non-telemetry senders of earlier graph readers (never
// itself).
struct GraphReader {
  explicit GraphReader(Node& node)
      : graph{*detail::NodeAccess::declared_graph(node)} {}
  detail::DeclaredGraph graph;
};

struct GraphReaderWithStaticSender {
  explicit GraphReaderWithStaticSender(Node& node)
      : status{node.publish<Count>("status")},
        telemetry{node.publish<Count>("telemetry")},
        graph{*detail::NodeAccess::declared_graph(node)},
        dynamic{node.publish<Count>("/talker/target/teleop")} {}
  Publisher<Count> status;
  Publisher<Count> telemetry;
  detail::DeclaredGraph graph;
  Publisher<Count> dynamic;
};

struct CountPublisher {
  explicit CountPublisher(Node& node) : out{node.publish<Count>("state")} {}
  Publisher<Count> out;
};

TEST(Node, SimHandsAGraphReaderTheNodesAddedBeforeIt) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<CountPublisher>("talker");
  const auto& first = sim.add<GraphReader>("listener");
  const auto& second = sim.add<GraphReader>("mechanism");
  ASSERT_EQ(first.graph.size(), 1u);
  EXPECT_EQ(first.graph[0].name, "talker");
  const auto& state = first.graph[0].sources.back();
  EXPECT_EQ(state.name, "/talker/state");
  EXPECT_EQ(state.schema, "node_test.Count");
  EXPECT_EQ(state.bfbs, schema::Of<Count>().bfbs);
  // The first reader declared no static senders, so it is not in the second's
  // graph.
  ASSERT_EQ(second.graph.size(), 1u);
  EXPECT_EQ(second.graph[0].name, "talker");
  EXPECT_TRUE(detail::NodeAccess::reads_graph(sim.node("listener")));
  EXPECT_FALSE(detail::NodeAccess::reads_graph(sim.node("talker")));
}

TEST(Node, SimExportsEarlierGraphReadersPass1StaticSendersExcludingSelf) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<CountPublisher>("talker");
  const auto& first = sim.add<GraphReaderWithStaticSender>("listener");
  const auto& second = sim.add<GraphReader>("mechanism");

  // The first graph reader sees only talker, never itself.
  ASSERT_EQ(first.graph.size(), 1u);
  EXPECT_EQ(first.graph[0].name, "talker");

  // The second graph reader sees talker and the first reader's Pass-1 static
  // non-telemetry sender (/listener/status), not its telemetry-role sender
  // (/listener/telemetry) and not its post-graph dynamic sender.
  ASSERT_EQ(second.graph.size(), 2u);
  EXPECT_EQ(second.graph[0].name, "talker");
  EXPECT_EQ(second.graph[1].name, "listener");
  ASSERT_EQ(second.graph[1].sources.size(), 1u);
  EXPECT_EQ(second.graph[1].sources[0].kind, event::SourceKind::SENDER);
  EXPECT_EQ(second.graph[1].sources[0].name, "/listener/status");
}

// A node's sends belong to the thread that runs its loop: the loop's dispatch
// context, its log and the transport are single-threaded. A send from another
// thread (a node's worker; a Python thread, whose C calls release the GIL)
// throws instead of racing them, and publishes nothing.
TEST(Node, SendsOnlyFromTheThreadThatBuiltIt) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& talker = sim.add<CountPublisher>("talker");
  bool refused = false;
  std::thread worker{[&] {
    try {
      (void)talker.out.send(Count{1, 0});
    } catch (const std::logic_error&) {
      refused = true;
    }
  }};
  worker.join();
  EXPECT_TRUE(refused);
  EXPECT_FALSE(sim.latest<Count>("/talker/state").has_value());
  EXPECT_TRUE(talker.out.send(Count{2, 0}));
  ASSERT_TRUE(sim.latest<Count>("/talker/state").has_value());
  EXPECT_EQ(sim.latest<Count>("/talker/state")->tick_count(), 2);
}

// Outside the Sim a byte topic is an RTMS queue, opened as the node registers
// it, like the realtime loop's typed topics: a second writer is refused while
// it is being built, not at its first send in the middle of a run.
TEST(BytePublisher, ClaimsItsRtmsTopicWhenRegisteredAndDropsNewestWhenFull) {
  const std::string topic = "/test/node_bytes/p" + std::to_string(::getpid());
  const std::string config_path = Runfile("TALOS_TEST_CONFIG");
  const auto robot_config = std::make_shared<const config::RobotConfig>(
      config::ParseRobotConfig(config_path));
  event::RealtimeEventLoop<> loop;
  detail::LoopPortFor<event::RealtimeEventLoop<>> port{loop};
  const auto build = [&] {
    return detail::NodeAccess::create(detail::NodeSetup{
        .name = "test_node",
        .config_path = config_path,
        .config = robot_config,
        .loop = &port,
    });
  };
  const BytePublishOptions options{
      .slots = 4, .drop_newest = true, .external = true};
  const auto node = build();
  BytePublisher out = node->publish_bytes(topic, 64, options);
  EXPECT_TRUE(ShmSegmentExists(topic));
  const auto rival = build();
  EXPECT_THROW((void)rival->publish_bytes(topic, 64, options),
               std::runtime_error);
  detail::NodeAccess::start(*node);

  const std::vector<std::byte> first(5, std::byte{1});
  EXPECT_TRUE(out.send(first));

  RTMSQueue reader_queue{
      topic,
      64,
      kMessageAlignment,
      schema::Bytes(),
      4,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE}};
  const auto reader = reader_queue.register_reader();
  if (!reader.has_value()) {
    FAIL() << "RTMS reader slots exhausted";
  }

  const std::vector<std::byte> second(9, std::byte{2});
  EXPECT_TRUE(out.send(second));
  std::vector<std::byte> slot(64);
  MessageInfo info{};
  ASSERT_EQ(reader_queue.read_next(*reader, slot, info), ReadResult::OK);
  EXPECT_EQ(info.sequence, 1u);
  EXPECT_EQ(std::vector<std::byte>(slot.begin(), slot.begin() + 9), second);

  // Four slots and one reader caught up: four more fit, the fifth is
  // refused rather than written over unread bytes.
  for (int i = 0; i < 4; ++i) EXPECT_TRUE(out.send(first)) << i;
  EXPECT_FALSE(out.send(first));
  EXPECT_FALSE(out.send(std::vector<std::byte>(65)));  // longer than max_bytes
  EXPECT_EQ(detail::NodeAccess::byte_topic_published(*node, 0), 6u);
  EXPECT_EQ(detail::NodeAccess::byte_topic_dropped(*node, 0), 2u);

  reader_queue.release_reader(*reader);
  ::shm_unlink(ShmObjectName(topic).c_str());
}

// REQ-SAFE-27: NodeDetail::publisher_send (the public-API thunk backing
// Publisher<M>::send) refuses to write into a hardware request slot registered
// via NodeAccess::add_hardware_request; only NodeAccess::publisher_send may.
TEST(NodeDetail, RefusesPublisherSendOnHardwareRequestSlot) {
  const std::string config_path = Runfile("TALOS_TEST_CONFIG");
  const auto robot_config = std::make_shared<const config::RobotConfig>(
      config::ParseRobotConfig(config_path));
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  detail::LoopPortFor<event::SimulatedEventLoop<>> port{loop};
  auto node = detail::NodeAccess::create(detail::NodeSetup{
      .name = "test_node",
      .config_path = config_path,
      .config = robot_config,
      .loop = &port,
  });
  const std::size_t hw_slot = detail::NodeAccess::add_hardware_request(
      *node, sizeof(Count), kMessageAlignment, schema::Of<Count>());
  const std::size_t normal_slot = detail::NodeDetail::add_publisher(
      *node, "/test_node/state", sizeof(Count), kMessageAlignment,
      schema::Of<Count>());
  detail::NodeAccess::start(*node);

  const Count msg{42, 0};
  EXPECT_THROW(
      detail::NodeDetail::publisher_send(*node, hw_slot, &msg, sizeof(msg)),
      std::logic_error);
  EXPECT_TRUE(
      detail::NodeAccess::publisher_send(*node, hw_slot, &msg, sizeof(msg)));
  EXPECT_TRUE(detail::NodeDetail::publisher_send(*node, normal_slot, &msg,
                                                 sizeof(msg)));
}

}  // namespace
}  // namespace talos
