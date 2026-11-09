#include "talOS/node/node.h"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "node_test/node_test_message_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/introspection/describe.h"
#include "talOS/memory/shared_memory_ptr.h"
#include "talOS/node/internal.h"
#include "talOS/node/testing.h"
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

  EXPECT_DOUBLE_EQ(node.param<double>("gain"), 2.5);
  EXPECT_DOUBLE_EQ(node.param<double>("whole"), 3.0);  // integer as double
  EXPECT_EQ(node.param<int>("whole"), 3);
  EXPECT_EQ(node.param<std::string>("tag"), "left");
  EXPECT_TRUE(node.param<bool>("inverted"));
  EXPECT_DOUBLE_EQ(node.param<double>("limits.max"), 4.0);

  EXPECT_DOUBLE_EQ(node.param<double>("absent", 9.5), 9.5);
  EXPECT_EQ(node.param<std::string>("absent", "x"), "x");
  EXPECT_DOUBLE_EQ(node.param<double>("gain", 0.0), 2.5);

  EXPECT_THROW((void)node.param<double>("absent"), std::invalid_argument);
  EXPECT_THROW((void)node.param<bool>("tag"), std::invalid_argument);
  EXPECT_THROW((void)node.param<int>("gain", 0), std::invalid_argument);
  EXPECT_EQ(node.param<std::uint8_t>("whole"), 3);
  EXPECT_THROW((void)node.param<std::uint8_t>("big"), std::invalid_argument);
}

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

  EXPECT_THROW((void)node.param<Strings>("absent"), std::invalid_argument);
  EXPECT_THROW((void)node.param<Doubles>("cameras"), std::invalid_argument);
  EXPECT_THROW((void)node.param<Strings>("offsets_m", {}),
               std::invalid_argument);
  EXPECT_THROW((void)node.param<Doubles>("mixed"), std::invalid_argument);
  EXPECT_THROW((void)node.param<Strings>("tag"), std::invalid_argument);
  EXPECT_THROW((void)node.param<Doubles>("limits", {}), std::invalid_argument);
  EXPECT_THROW((void)node.param<std::string>("cameras"), std::invalid_argument);
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
          heard.push_back(c.tick_count());
        })} {}
  void update() { latest_in_update = in.has_value() ? in->tick_count() : -1; }
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
  EXPECT_EQ(listener.in->tick_count(), 4);
  EXPECT_GE(listener.latest_in_update, 3);
  const auto latest = sim.latest<Count>("/talker/state");
  if (!latest.has_value()) {
    FAIL() << "nothing published on /talker/state";
  }
  EXPECT_EQ(latest->tick_count(), 4);
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

struct Backlog {
  explicit Backlog(Node& n)
      : latest{n.subscribe<Count>("/backlog")},
        ordered{n.subscribe<Count>("/backlog", [this](const Count& c) {
          received.push_back(c.tick_count());
        })} {}
  void update() {}
  Subscription<Count> latest;
  Subscription<Count> ordered;
  std::vector<std::int64_t> received;
};

// A plain subscription is a latest value: one drain jumps a backlog longer
// than the per-drain cap to its newest message, aged from publication. A
// callback subscription still sees every message in order, 64 per drain.
TEST(Node, PlainSubscriptionSkipsBacklogWhileCallbacksKeepCappedSequence) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& backlog = sim.add<Backlog>("listener");
  sim.step(10ms);
  for (std::int64_t i = 0; i < 130; ++i) {
    sim.publish("/backlog", Count{i, 0});
  }

  sim.step(1ms);
  EXPECT_EQ(backlog.latest->tick_count(), 129);
  EXPECT_EQ(backlog.latest.age(), 1ms);
  ASSERT_EQ(backlog.received.size(), 64u);
  sim.step(1ms);
  ASSERT_EQ(backlog.received.size(), 128u);
  sim.step(1ms);
  ASSERT_EQ(backlog.received.size(), 130u);
  for (std::int64_t i = 0; i < 130; ++i) {
    EXPECT_EQ(backlog.received[static_cast<std::size_t>(i)], i);
  }
  EXPECT_EQ(backlog.latest->tick_count(), 129);
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

// A callback subscription: the node drains every 1 ms tick, in sequence.
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
// one added before sees it at its next drain.
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

    // Every publication reaches the ordered subscriber once, at the first
    // 1 ms drain that runs after it.
    const auto latest = sim.latest<Count>("/producer/state");
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->tick_count(), 39);
    ASSERT_EQ(hearer.heard.size(), producer_first ? 40u : 39u);
    for (std::size_t i = 0; i < hearer.heard.size(); ++i) {
      const auto [delivered, stamp] = hearer.heard[i];
      EXPECT_EQ(stamp, milliseconds{i + 1});
      EXPECT_EQ(delivered, producer_first ? stamp : stamp + 1ms);
    }
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

// The loop's delivery as NodeAccess::start picks it for T on node
// "listener".
template <class T>
event::Delivery DeliveryFor() {
  auto sim = Sim::FromToml(kRobotToml);
  const std::string& config_path = detail::SimAccess::config_path(sim);
  event::SimulationEnvironment environment;
  event::SimulatedEventLoop<> loop{environment};
  detail::LoopPortFor<event::SimulatedEventLoop<>> port{loop};
  const auto node = detail::NodeAccess::create(detail::NodeSetup{
      .name = "listener",
      .config_path = config_path,
      .config = std::make_shared<const config::RobotConfig>(
          config::ParseRobotConfig(config_path)),
      .simulation = true,
      .loop = &port,
  });
  const detail::Instance user = detail::MakeInstance<T>()(*node);
  detail::NodeAccess::start(*node);
  return loop.delivery();
}

// A node that only reads latched values at its own timers drains right
// before them; a message callback, or having no timer to drain before, keeps
// the 1 ms drain.
TEST(Node, DeliveryWaitsForTimersUnlessAHandlerNeedsEveryMessage) {
  EXPECT_EQ(DeliveryFor<Watcher>(), event::Delivery::BEFORE_TIMERS);
  EXPECT_EQ(DeliveryFor<Beats>(), event::Delivery::BEFORE_TIMERS);
  EXPECT_EQ(DeliveryFor<Listener>(), event::Delivery::POLL);
  EXPECT_EQ(DeliveryFor<Silent>(), event::Delivery::POLL);
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

TEST(BytePublisher, RejectsBadLayoutsAndLateRegistration) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<BadFramer>("listener");
  EXPECT_THROW((void)sim.node("listener").publish_bytes("/late/frames", 8),
               std::logic_error);
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

Result RunBinary(const std::string& args,
                 const char* binary = "TALOS_TEST_NODE") {
  const std::string command = "'" + Runfile(binary) + "' " + args + " 2>&1";
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
  // Every topic endpoint names its schema, with its reflection bytes; a
  // timer has none.
  const std::string count_schema =
      R"("schema": "node_test.Count", "schema_hash": ")" +
      introspect::HexHash(Descriptor<Count>::kHash) +
      R"(", "alignment": 8, "slots": 1024, "bfbs": ")" +
      introspect::Base64(schema::Of<Count>().bfbs) + R"("})";
  EXPECT_NE(
      result.output.find(
          R"({"kind": "WATCHER", "name": "/test_node/target", "message_bytes": 16, "external": false, "optional": true, "observer": false, )" +
          count_schema),
      std::string::npos)
      << result.output;
  EXPECT_NE(
      result.output.find(
          R"({"kind": "SENDER", "name": "/test_node/state", "message_bytes": 16, "external": false, "optional": false, "observer": false, )" +
          count_schema),
      std::string::npos)
      << result.output;
  EXPECT_NE(
      result.output.find(
          R"({"kind": "SENDER", "name": "/test_node/frames", "message_bytes": 64, "external": true, "optional": false, "observer": false, "schema": "talos.Bytes", "schema_hash": ")" +
          introspect::HexHash(schema::BytesHash()) +
          R"(", "alignment": 8, "slots": 16, "bfbs": ""})"),
      std::string::npos)
      << result.output;
  EXPECT_NE(result.output.find(R"("reads_graph": false)"), std::string::npos)
      << result.output;
  EXPECT_NE(result.output.find(R"({"kind": "TIMER", "name": "test_node")"),
            std::string::npos)
      << result.output;
  // A node with no hardware never touches /hw/*.
  EXPECT_EQ(result.output.find("/hw/"), std::string::npos) << result.output;
}

std::string TempFile(const std::string& name) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  return std::string{tmp != nullptr ? tmp : "/tmp"} + "/" + name;
}

// The launcher's two describe passes and its run, against a node that reads
// the declared graph.
TEST(Binary, AGraphReaderDescribesWhatTheGraphItWasGivenHolds) {
  // First pass: no graph yet; it says it reads one and registers nothing.
  const Result first =
      RunBinary("--describe " + ConfigFlag(), "TALOS_GRAPH_NODE");
  ASSERT_EQ(first.status, 0) << first.output;
  introspect::Description described;
  std::string error;
  ASSERT_TRUE(introspect::DescribeReader::Parse(first.output, described, error))
      << error << "\n"
      << first.output;
  EXPECT_TRUE(described.reads_graph);
  EXPECT_EQ(described.sources.size(), 1u);  // its tick timer

  // Second pass: test_node's description is the graph.
  const Result test_node = RunBinary("--describe " + ConfigFlag());
  ASSERT_EQ(test_node.status, 0) << test_node.output;
  introspect::Description publisher;
  ASSERT_TRUE(
      introspect::DescribeReader::Parse(test_node.output, publisher, error))
      << error;
  const std::string graph = TempFile("talos_declared_graph.json");
  {
    std::ofstream out{graph};
    out << introspect::DeclaredGraphToJson({publisher});
  }
  const Result second = RunBinary(
      "--describe --graph '" + graph + "' " + ConfigFlag(), "TALOS_GRAPH_NODE");
  ASSERT_EQ(second.status, 0) << second.output;
  ASSERT_TRUE(
      introspect::DescribeReader::Parse(second.output, described, error))
      << error;
  const auto observed = std::find_if(
      described.sources.begin(), described.sources.end(),
      [](const auto& source) { return source.name == "/test_node/state"; });
  ASSERT_NE(observed, described.sources.end()) << second.output;
  EXPECT_TRUE(observed->observer());
  EXPECT_EQ(observed->kind, event::SourceKind::WATCHER);
  EXPECT_EQ(observed->schema, "node_test.Count");
  EXPECT_EQ(observed->schema_hash, Descriptor<Count>::kHash);
  EXPECT_EQ(observed->bfbs, schema::Of<Count>().bfbs);

  // Run for real without a graph: it cannot be built, and says how to fix it.
  const Result run =
      RunBinary("--sim --duration-s 1 --log '" + TempFile("graph_node.tlog") +
                    "' " + ConfigFlag(),
                "TALOS_GRAPH_NODE");
  EXPECT_NE(run.status, 0);
}

// Exact replay is offline: a robot (here a sentinel publisher) may own the
// node's byte topic on this machine, and replay neither attaches to it nor
// fails because of it. Byte topics are not logged, so replay does not verify
// them; typed sends still are, and a changed parameter diverges.
TEST(Binary, LoggedSimRunReplaysOfflineWithoutDivergence) {
  const std::string log = TempFile("talos_node_test.tlog");
  // Isolate the sentinel and child replay from live publishers, restoring
  // the caller's namespace afterwards.
  struct Namespace {
    std::optional<std::string> previous;
    Namespace() {
      if (const char* value = std::getenv("TALOS_SHM_NAMESPACE"))
        previous = value;
      const std::string name = "replay_offline_" + std::to_string(::getpid());
      ::setenv("TALOS_SHM_NAMESPACE", name.c_str(), 1);
    }
    ~Namespace() {
      if (previous) {
        ::setenv("TALOS_SHM_NAMESPACE", previous->c_str(), 1);
      } else {
        ::unsetenv("TALOS_SHM_NAMESPACE");
      }
    }
  } isolated;
  const Result run =
      RunBinary("--sim --duration-s 1 --log '" + log + "' " + ConfigFlag());
  ASSERT_EQ(run.status, 0) << run.output;
  {
    RTMSQueue sentinel{
        "/test_node/frames",
        64,
        kMessageAlignment,
        schema::Bytes(),
        16,
        RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                    .read_mode = ReadMode::SEQUENCE,
                    .publisher = true}};
    const std::uint64_t written = sentinel.writer_sequence();
    const Result replay = RunBinary("--replay '" + log + "' " + ConfigFlag());
    EXPECT_EQ(replay.status, 0) << replay.output;
    EXPECT_EQ(sentinel.writer_sequence(), written);
  }

  std::string config;
  {
    std::ifstream in{Runfile("TALOS_TEST_CONFIG")};
    config.assign(std::istreambuf_iterator<char>{in}, {});
  }
  const std::size_t step = config.find("step = 3");
  ASSERT_NE(step, std::string::npos);
  config.replace(step, 8, "step = 4");
  const std::string changed = TempFile("talos_node_test_step4.toml");
  std::ofstream{changed} << config;
  const Result diverged =
      RunBinary("--replay '" + log + "' --config '" + changed + "'");
  EXPECT_NE(diverged.status, 0) << diverged.output;
  // Only the --sim run wrote the node's byte topic to shared memory.
  ::shm_unlink(ShmObjectName("/test_node/frames").c_str());
}

bool ShmSegmentExists(const std::string& topic) {
  const int fd = ::shm_open(ShmObjectName(topic).c_str(), O_RDWR, 0666);
  if (fd == -1) return false;
  ::close(fd);
  return true;
}

// In a Sim a graph reader sees every node added before it, schemas
// included, except other graph readers.
struct GraphReader {
  explicit GraphReader(Node& node)
      : graph{*detail::NodeAccess::declared_graph(node)} {}
  detail::DeclaredGraph graph;
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
  // The first reader is not in the second's graph.
  ASSERT_EQ(second.graph.size(), 1u);
  EXPECT_EQ(second.graph[0].name, "talker");
  EXPECT_TRUE(detail::NodeAccess::reads_graph(sim.node("listener")));
  EXPECT_FALSE(detail::NodeAccess::reads_graph(sim.node("talker")));
}

// Outside the Sim a byte topic is an RTMS queue, opened by the first send:
// building the node (as --describe does) leaves no segment behind.
TEST(BytePublisher, OpensRtmsOnFirstSendAndDropsNewestWhenFull) {
  const std::string topic = "/test/node_bytes/p" + std::to_string(::getpid());
  const std::string config_path = Runfile("TALOS_TEST_CONFIG");
  event::RealtimeEventLoop<> loop;
  detail::LoopPortFor<event::RealtimeEventLoop<>> port{loop};
  const auto node = detail::NodeAccess::create(detail::NodeSetup{
      .name = "test_node",
      .config_path = config_path,
      .config = std::make_shared<const config::RobotConfig>(
          config::ParseRobotConfig(config_path)),
      .loop = &port,
  });
  BytePublisher out = node->publish_bytes(
      topic, 64, {.slots = 4, .drop_newest = true, .external = true});
  detail::NodeAccess::start(*node);
  EXPECT_FALSE(ShmSegmentExists(topic));

  const std::vector<std::byte> first(5, std::byte{1});
  EXPECT_TRUE(out.send(first));
  ASSERT_TRUE(ShmSegmentExists(topic));

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

}  // namespace
}  // namespace talos
