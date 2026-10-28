#pragma once

// The talOS node API: the one way to write a node.
//
//   #include "talOS/node/node.h"
//
//   class Counter {
//    public:
//     explicit Counter(talos::Node& node)
//         : goal_{node.subscribe<demo::Goal>("/demo/goal")},
//           status_{node.publish<demo::Status>("/demo/status")} {}
//     void update() {
//       if (goal_.updated()) status_.send(demo::Status{goal_->value()});
//     }
//
//    private:
//     talos::Subscription<demo::Goal> goal_;
//     talos::Publisher<demo::Status> status_;
//   };
//   TALOS_NODE(Counter)
//
// A Node is generic: topics, timers, parameters and the update() tick. It
// knows nothing about robots or hardware. Layers attach per-node state with
// extension<T>() and run work after each tick with after_update(); the
// hardware layer (talOS/devices) is built exactly that way. Build a node with
// talos_cc_node in //talOS/build:contract.bzl, which supplies its name and
// target.

#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <utility>

#include "talOS/events/time.h"
#include "talOS/ipc/concepts.h"
#include "talOS/node/access.h"

namespace talos {

using Time = event::MonotonicTime;
using Duration = std::chrono::nanoseconds;

// Every user topic's RTMS slot is aligned to 8 bytes, whatever the message's
// own alignof. The slot layout is part of the shared-memory segment, and both
// ends of a topic must agree on it, but a Python endpoint cannot know the C++
// alignof of a flatc struct: its generated class exposes the size and nothing
// else. Fixing the alignment at the largest a flatbuffer struct can need
// (8, for a double or a 64-bit integer) makes the layout a function of the
// size alone, so a C++ publisher and a Python subscriber always open the same
// segment. Topics whose layout someone else owns (the bridge's /hw/*) are
// registered through the framework plumbing with that owner's alignment.
inline constexpr std::size_t kMessageAlignment = 8;

// A message that can cross a topic: a fixed-size flatbuffer struct (tables
// have no size) that fits the fixed slot alignment.
template <class M>
concept Message =
    std::is_trivially_copyable_v<M> && ipc::NotDerivedFromFlatbufferTable<M> &&
    alignof(M) <= kMessageAlignment;

// Node::subscribe options.
struct SubscribeOptions {
  bool optional = false;  // the graph lint tolerates no publisher
};

template <Message M>
class Subscription;
template <Message M>
class Publisher;

class Node {
 public:
  ~Node();
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  std::string_view name() const;  // subsystem name (== node_name)
  bool simulation() const;        // --sim
  Time now() const;
  std::uint64_t tick() const;  // update() calls completed so far
  Duration period() const;     // [subsystem] period_us, else [robot] period_us
  const std::string& config_path() const;  // the robot config (--config)

  // Custom keys of this subsystem's TOML table. Dotted keys reach nested
  // tables ("geometry.wheelbase_m"). T: bool, int64_t/int, double,
  // std::string. A present key of the wrong type throws.
  template <class T>
  T param(std::string_view key, T fallback) const;
  template <class T>
  T param(std::string_view key) const;  // throws if missing

  // Topics. Call in the constructor: registration order is what replay
  // checks a log against.
  using SubscribeOptions = talos::SubscribeOptions;
  template <Message M>
  Subscription<M> subscribe(std::string_view topic,
                            SubscribeOptions options = {});
  template <Message M, std::invocable<const M&> F>
  Subscription<M> subscribe(std::string_view topic, F on_message,
                            SubscribeOptions options = {});
  template <Message M>
  Publisher<M> publish(std::string_view topic);

  void every(Duration period, std::function<void()> fn);  // extra timer

  // --- extension points, for layers -----------------------------------------

  // This node's one T, constructed as T(Node&) on first use and destroyed
  // with the node (most recent first). First use belongs in the constructor
  // when T registers anything.
  template <class T>
  T& extension();
  // Runs after every update() tick, in registration order. Call in the
  // constructor.
  void after_update(std::function<void()> fn);

 private:
  friend struct detail::NodeAccess;  // the framework's side: access.h
  explicit Node(detail::NodeSetup setup);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Latest value on a topic, kept by the framework. A cheap copyable value.
template <Message M>
class Subscription {
 public:
  Subscription() = default;

  bool has_value() const;
  const M& value() const;  // zero-initialized M until the first message
  const M* operator->() const { return &value(); }
  const M& operator*() const { return value(); }
  bool fresh() const;  // has_value() && age() < [robot] command_timeout_us
  bool fresh(Duration max_age) const;
  Duration age() const;  // now - receive time (max if none)
  bool updated() const;  // a message arrived since the previous update() began

 private:
  friend class Node;
  Subscription(Node* node, std::size_t index) : node_{node}, index_{index} {}
  Node* node_{nullptr};
  std::size_t index_{0};
};

template <Message M>
class Publisher {
 public:
  Publisher() = default;

  bool send(const M& message);

 private:
  friend class Node;
  Publisher(Node* node, std::size_t index) : node_{node}, index_{index} {}
  Node* node_{nullptr};
  std::size_t index_{0};
};

// A state plus when it was entered, ticking with the node.
template <class S>
  requires std::is_enum_v<S>
class StateMachine {
 public:
  StateMachine(Node& node, S initial);

  S state() const { return state_; }
  bool is(S s) const { return state_ == s; }
  S previous() const { return previous_; }
  void go(S next);  // no-op when next == state()
  Duration time_in_state() const { return node_->now() - entered_at_; }
  // True during the first update() tick that starts in this state.
  bool just_entered() const { return node_->tick() == entered_update_; }

 private:
  const Node* node_;
  S state_;
  S previous_;
  std::uint64_t entered_update_;
  Time entered_at_;
};

// Runs T as this binary's node; TALOS_NODE(T) calls it.
template <class T>
int Run(int argc, char** argv);

// --- template definitions ------------------------------------------------

namespace detail {

template <class T>
std::optional<T> FindParam(const Node& node, std::string_view key) {
  if constexpr (std::is_same_v<T, bool>) {
    return NodeAccess::find_bool(node, key);
  } else if constexpr (std::is_integral_v<T>) {
    const std::optional<std::int64_t> value = NodeAccess::find_int(node, key);
    if (!value) return std::nullopt;
    if (!std::in_range<T>(*value)) {
      NodeAccess::param_error(node, key, "is out of range");
    }
    return static_cast<T>(*value);
  } else if constexpr (std::is_floating_point_v<T>) {
    const std::optional<double> value = NodeAccess::find_double(node, key);
    if (!value) return std::nullopt;
    return static_cast<T>(*value);
  } else if constexpr (std::is_same_v<T, std::string>) {
    return NodeAccess::find_string(node, key);
  } else {
    static_assert(
        sizeof(T) == 0,
        "param<T>: T must be bool, an integer, double or std::string");
  }
}

template <class T>
InstanceFactory MakeInstance() {
  static_assert(std::is_constructible_v<T, Node&>,
                "a talos node class must be constructible from talos::Node&");
  return [](Node& node) -> Instance {
    auto object = std::make_shared<T>(node);
    if constexpr (requires(T& t) { t.update(); }) {
      NodeAccess::set_update(node, [raw = object.get()] { raw->update(); });
    }
    return object;
  };
}

}  // namespace detail

template <class T>
T Node::param(std::string_view key, T fallback) const {
  std::optional<T> value = detail::FindParam<T>(*this, key);
  return value ? *std::move(value) : std::move(fallback);
}

template <class T>
T Node::param(std::string_view key) const {
  std::optional<T> value = detail::FindParam<T>(*this, key);
  if (!value) detail::NodeAccess::param_error(*this, key, "is missing");
  return *std::move(value);
}

template <Message M>
Subscription<M> Node::subscribe(std::string_view topic,
                                SubscribeOptions options) {
  return Subscription<M>{
      this, detail::NodeAccess::add_subscription(
                *this, topic, sizeof(M), kMessageAlignment, options.optional,
                {})};
}

template <Message M, std::invocable<const M&> F>
Subscription<M> Node::subscribe(std::string_view topic, F on_message,
                                SubscribeOptions options) {
  detail::MessageCallback callback = [fn = std::move(on_message)](
                                         const void* data) {
    fn(*static_cast<const M*>(data));
  };
  return Subscription<M>{
      this, detail::NodeAccess::add_subscription(
                *this, topic, sizeof(M), kMessageAlignment, options.optional,
                std::move(callback))};
}

template <Message M>
Publisher<M> Node::publish(std::string_view topic) {
  return Publisher<M>{this, detail::NodeAccess::add_publisher(
                                *this, topic, sizeof(M), kMessageAlignment)};
}

template <class T>
T& Node::extension() {
  const std::type_index type{typeid(T)};
  if (void* existing = detail::NodeAccess::find_extension(*this, type)) {
    return *static_cast<T*>(existing);
  }
  auto object = std::make_shared<T>(*this);
  T& result = *object;
  detail::NodeAccess::add_extension(*this, type, std::move(object));
  return result;
}

template <Message M>
bool Subscription<M>::has_value() const {
  return detail::NodeAccess::subscription_has_value(*node_, index_);
}
template <Message M>
const M& Subscription<M>::value() const {
  return *static_cast<const M*>(
      detail::NodeAccess::subscription_value(*node_, index_));
}
template <Message M>
bool Subscription<M>::fresh() const {
  return fresh(detail::NodeAccess::freshness_timeout(*node_));
}
template <Message M>
bool Subscription<M>::fresh(Duration max_age) const {
  return detail::NodeAccess::subscription_fresh(*node_, index_, max_age);
}
template <Message M>
Duration Subscription<M>::age() const {
  return detail::NodeAccess::subscription_age(*node_, index_);
}
template <Message M>
bool Subscription<M>::updated() const {
  return detail::NodeAccess::subscription_updated(*node_, index_);
}

template <Message M>
bool Publisher<M>::send(const M& message) {
  return detail::NodeAccess::publisher_send(*node_, index_, &message,
                                            sizeof(M));
}

template <class S>
  requires std::is_enum_v<S>
StateMachine<S>::StateMachine(Node& node, S initial)
    : node_{&node},
      state_{initial},
      previous_{initial},
      entered_update_{detail::NodeAccess::next_update(node)},
      entered_at_{node.now()} {}

template <class S>
  requires std::is_enum_v<S>
void StateMachine<S>::go(S next) {
  if (next == state_) return;
  previous_ = state_;
  state_ = next;
  entered_at_ = node_->now();
  // Entered during update() k: the first update to start in it is k + 1.
  entered_update_ = detail::NodeAccess::next_update(*node_);
}

// The name and target come from the talos_cc_node macro (TALOS_NODE_NAME /
// TALOS_NODE_TARGET local_defines).
template <class T>
int Run(int argc, char** argv) {
#if defined(TALOS_NODE_NAME) && defined(TALOS_NODE_TARGET)
  return detail::RunMain(argc, argv, TALOS_NODE_NAME, TALOS_NODE_TARGET,
                         detail::MakeInstance<T>());
#else
  static_assert(sizeof(T) == 0,
                "talos::Run needs TALOS_NODE_NAME and TALOS_NODE_TARGET; "
                "build the node with talos_cc_node");
  return 1;
#endif
}

}  // namespace talos

// Defines main() for a node class.
#define TALOS_NODE(Class) \
  int main(int argc, char** argv) { return ::talos::Run<Class>(argc, argv); }
