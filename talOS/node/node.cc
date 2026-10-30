#include "talOS/node/node.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/reporter.h"
#include "talOS/node/internal.h"
#include "talOS/process/process.h"
#include "talOS/rtms/rtms.h"

namespace talos {

static_assert(BytePublishOptions{}.slots == MAX_SLOTS,
              "byte topics default to the loop topics' ring size");

struct Node::Impl {
  struct SubscriptionSlot {
    std::string topic;
    std::size_t bytes;
    std::unique_ptr<std::byte[]> data;  // operator new[]: aligned for M
    bool optional;
    detail::MessageCallback on_message;
    bool has_value = false;
    Time received{};
    bool pending = false;  // arrived since the last update() began
    bool updated = false;  // what update() sees: pending at its start
  };
  struct PublisherSlot {
    std::uint16_t id;
    std::size_t bytes;
  };
  struct BytePublisherSlot {
    std::string topic;
    std::size_t max_bytes;
    bool external;
    std::unique_ptr<detail::ByteTopic> sink;
    // Read by the registry's thread.
    std::atomic<std::uint64_t> published{0};
    std::atomic<std::uint64_t> dropped{0};
  };
  struct TimerSlot {
    std::function<void()> fn;
  };
  struct StateMachineSlot {
    std::int64_t state;
    std::int64_t previous;
    std::uint64_t entered_update;
    Time entered_at;
  };

  std::string name;
  std::string config_path;
  std::shared_ptr<const config::RobotConfig> config;
  bool simulation = false;
  detail::LoopPort* loop = nullptr;
  Duration period{};
  Duration timeout{};

  std::vector<std::unique_ptr<SubscriptionSlot>> subscriptions;
  std::vector<PublisherSlot> publishers;
  std::vector<std::unique_ptr<BytePublisherSlot>> byte_publishers;
  std::vector<std::unique_ptr<TimerSlot>> timers;
  std::vector<StateMachineSlot> state_machines;
  std::vector<std::pair<std::type_index, std::shared_ptr<void>>> extensions;

  std::function<void()> update;
  std::vector<std::function<void()>> after_update;
  bool started = false;
  bool immediate_delivery = false;
  bool in_update = false;
  std::uint64_t ticks = 0;

  const SubscriptionSlot& SubscriptionAt(std::size_t index) const {
    if (index >= subscriptions.size()) {
      throw std::out_of_range(name + ": no subscription at index " +
                              std::to_string(index));
    }
    return *subscriptions[index];
  }

  BytePublisherSlot& ByteAt(std::size_t index) const {
    if (index >= byte_publishers.size()) {
      throw std::out_of_range(name + ": no byte publisher at index " +
                              std::to_string(index));
    }
    return *byte_publishers[index];
  }

  StateMachineSlot& StateMachineAt(std::size_t index) {
    if (index >= state_machines.size()) {
      throw std::out_of_range(name + ": no state machine at index " +
                              std::to_string(index));
    }
    return state_machines[index];
  }
  const StateMachineSlot& StateMachineAt(std::size_t index) const {
    if (index >= state_machines.size()) {
      throw std::out_of_range(name + ": no state machine at index " +
                              std::to_string(index));
    }
    return state_machines[index];
  }

  void CheckOpen(const char* what) const {
    if (started) {
      throw std::logic_error(name + ": " + what +
                             " must be called in the constructor");
    }
  }

  const toml::table* Table() const { return config->GetSubsystemTable(name); }
  toml::node_view<const toml::node> Param(std::string_view key) const {
    const toml::table* table = Table();
    if (table == nullptr) return {};
    return table->at_path(key);
  }

  std::string ResolveTopic(std::string_view topic) const {
    const toml::table* table = Table();
    const toml::table* remap =
        table != nullptr ? (*table)["remap"].as_table() : nullptr;
    auto lookup = [&](std::string_view key) -> std::optional<std::string> {
      if (remap == nullptr) return std::nullopt;
      const auto entry = (*remap)[key];
      if (!entry) return std::nullopt;
      if (auto s = entry.value_exact<std::string>()) return *s;
      throw std::invalid_argument(name + ": remap entry for '" +
                                  std::string{topic} + "' must be a string");
    };
    const auto direct = lookup(topic);
    std::string resolved = direct ? *direct : std::string{topic};
    const bool expanded = !resolved.empty() && resolved.front() != '/';
    if (expanded) {
      resolved = "/" + name + "/" + resolved;
    }
    if (!direct && expanded) {
      if (auto remapped = lookup(resolved)) {
        resolved = *std::move(remapped);
        if (!resolved.empty() && resolved.front() != '/') {
          resolved = "/" + name + "/" + resolved;
        }
      }
    }
    return resolved;
  }

  static void OnMessage(void* object, const event::Context& context,
                        std::span<const std::byte> payload) {
    auto* slot = static_cast<SubscriptionSlot*>(object);
    const std::size_t copied = std::min(payload.size(), slot->bytes);
    std::memcpy(slot->data.get(), payload.data(), copied);
    std::memset(slot->data.get() + copied, 0, slot->bytes - copied);
    slot->has_value = true;
    slot->received = context.now;
    slot->pending = true;
    if (slot->on_message) slot->on_message(slot->data.get());
  }

  static void OnTimer(void* object, const event::Context&,
                      std::span<const std::byte>) {
    static_cast<TimerSlot*>(object)->fn();
  }

  static void OnTick(void* object, const event::Context&,
                     std::span<const std::byte>) {
    auto* impl = static_cast<Impl*>(object);
    for (auto& slot : impl->subscriptions) {
      slot->updated = slot->pending;
      slot->pending = false;
    }
    if (impl->update) {
      struct InUpdate {
        bool& flag;
        explicit InUpdate(bool& f) : flag{f} { flag = true; }
        ~InUpdate() { flag = false; }
      } in_update{impl->in_update};
      impl->update();
    }
    ++impl->ticks;
    for (const auto& hook : impl->after_update) hook();
  }
};

Node::Node(detail::NodeSetup setup) : impl_{std::make_unique<Impl>()} {
  if (setup.loop == nullptr || setup.config == nullptr) {
    throw std::invalid_argument("talos::Node needs a loop and a config");
  }
  Impl& impl = *impl_;
  impl.name = std::move(setup.name);
  impl.config_path = std::move(setup.config_path);
  impl.config = std::move(setup.config);
  impl.simulation = setup.simulation;
  impl.loop = setup.loop;
  impl.timeout =
      std::chrono::microseconds{impl.config->hardware.command_timeout_us};

  std::int64_t period_us = impl.config->hardware.period_us;
  if (const toml::table* table = impl.Table()) {
    if (const auto node = (*table)["period_us"]) {
      const auto value = node.value_exact<std::int64_t>();
      if (!value) {
        throw std::invalid_argument(impl.name +
                                    ": period_us must be an integer");
      }
      period_us = *value;
    }
  }
  if (period_us <= 0) {
    throw std::invalid_argument(impl.name + ": period_us must be positive");
  }
  impl.period = std::chrono::microseconds{period_us};
}

// Extensions hold references into this node, so they go first, newest first.
Node::~Node() {
  while (!impl_->extensions.empty()) impl_->extensions.pop_back();
}

std::string_view Node::name() const { return impl_->name; }
bool Node::simulation() const { return impl_->simulation; }
Time Node::now() const { return impl_->loop->now(); }
std::uint64_t Node::tick() const { return impl_->ticks; }
Duration Node::period() const { return impl_->period; }
const std::string& Node::config_path() const { return impl_->config_path; }

void Node::after_update(std::function<void()> fn) {
  impl_->CheckOpen("after_update");
  if (!fn) throw std::invalid_argument(impl_->name + ": empty after_update");
  impl_->after_update.push_back(std::move(fn));
}

void Node::every(Duration period, std::function<void()> fn) {
  if (period <= Duration::zero()) {
    throw std::invalid_argument(impl_->name +
                                ": every() needs a positive period");
  }
  if (!fn) {
    throw std::invalid_argument(impl_->name + ": every() needs a function");
  }
  auto slot = std::make_unique<Impl::TimerSlot>(Impl::TimerSlot{std::move(fn)});
  const std::string label =
      impl_->name + "/every/" + std::to_string(impl_->timers.size());
  const std::uint16_t id = impl_->loop->register_timer(
      label, event::Thunk{slot.get(), &Impl::OnTimer});
  impl_->timers.push_back(std::move(slot));
  impl_->loop->arm_timer(id, now() + period, period);
}

BytePublisher Node::publish_bytes(std::string_view topic, std::size_t max_bytes,
                                  BytePublishOptions options) {
  return BytePublisher{this, detail::NodeAccess::add_byte_publisher(
                                 *this, topic, max_bytes, options)};
}

// --- framework plumbing ------------------------------------------------------

namespace detail {

std::unique_ptr<Node> NodeAccess::create(NodeSetup setup) {
  return std::unique_ptr<Node>{new Node{std::move(setup)}};
}

void NodeAccess::set_update(Node& node, std::function<void()> update) {
  node.impl_->CheckOpen("set_update");
  node.impl_->update = std::move(update);
}

void NodeAccess::start(Node& node) {
  Node::Impl& impl = *node.impl_;
  if (impl.started) throw std::logic_error(impl.name + ": started twice");
  impl.started = true;
  // Draining before timers needs a timer: a node with none (it only lets
  // its extensions latch values for someone else to read) keeps the tick.
  const bool has_tick = impl.update || !impl.after_update.empty();
  const bool timed = has_tick || !impl.timers.empty();
  impl.loop->set_delivery(impl.immediate_delivery || !timed
                              ? event::Delivery::POLL
                              : event::Delivery::BEFORE_TIMERS);
  if (!has_tick) return;
  const std::uint16_t id = impl.loop->register_timer(
      impl.name, event::Thunk{&impl, &Node::Impl::OnTick});
  impl.loop->arm_timer(id, node.now() + impl.period, impl.period);
}

void NodeAccess::require_immediate_delivery(Node& node) {
  node.impl_->CheckOpen("subscribe with a callback");
  node.impl_->immediate_delivery = true;
}

bool NodeAccess::immediate_delivery(const Node& node) {
  return node.impl_->immediate_delivery;
}

std::uint64_t NodeAccess::next_update(const Node& node) {
  return node.impl_->in_update ? node.impl_->ticks + 1 : node.impl_->ticks;
}

Duration NodeAccess::freshness_timeout(const Node& node) {
  return node.impl_->timeout;
}

std::vector<std::string_view> NodeAccess::optional_topics(const Node& node) {
  std::vector<std::string_view> topics;
  for (const auto& slot : node.impl_->subscriptions) {
    if (slot->optional) topics.push_back(slot->topic);
  }
  return topics;
}

std::size_t NodeAccess::add_state_machine(Node& node, std::int64_t initial) {
  node.impl_->state_machines.push_back(Node::Impl::StateMachineSlot{
      .state = initial,
      .previous = initial,
      .entered_update = next_update(node),
      .entered_at = node.now(),
  });
  return node.impl_->state_machines.size() - 1;
}

std::int64_t NodeAccess::state_machine_state(const Node& node,
                                             std::size_t index) {
  return node.impl_->StateMachineAt(index).state;
}

std::int64_t NodeAccess::state_machine_previous(const Node& node,
                                                std::size_t index) {
  return node.impl_->StateMachineAt(index).previous;
}

void NodeAccess::state_machine_go(Node& node, std::size_t index,
                                  std::int64_t next) {
  auto& slot = node.impl_->StateMachineAt(index);
  if (next == slot.state) return;
  slot.previous = slot.state;
  slot.state = next;
  slot.entered_at = node.now();
  // Entered during update() k: the first update to start in it is k + 1.
  slot.entered_update = next_update(node);
}

Duration NodeAccess::state_machine_time_in_state(const Node& node,
                                                 std::size_t index) {
  return node.now() - node.impl_->StateMachineAt(index).entered_at;
}

bool NodeAccess::state_machine_just_entered(const Node& node,
                                            std::size_t index) {
  return node.tick() == node.impl_->StateMachineAt(index).entered_update;
}

void* NodeAccess::find_extension(const Node& node, std::type_index type) {
  for (const auto& [key, object] : node.impl_->extensions) {
    if (key == type) return object.get();
  }
  return nullptr;
}

void NodeAccess::add_extension(Node& node, std::type_index type,
                               std::shared_ptr<void> object) {
  if (find_extension(node, type) != nullptr) {
    throw std::logic_error(node.impl_->name + ": extension added twice");
  }
  node.impl_->extensions.emplace_back(type, std::move(object));
}

// --- params ------------------------------------------------------------------

std::optional<bool> NodeAccess::find_bool(const Node& node,
                                          std::string_view key) {
  const auto value = node.impl_->Param(key);
  if (!value) return std::nullopt;
  if (auto b = value.value_exact<bool>()) return b;
  param_error(node, key, "must be a boolean");
}

std::optional<std::int64_t> NodeAccess::find_int(const Node& node,
                                                 std::string_view key) {
  const auto value = node.impl_->Param(key);
  if (!value) return std::nullopt;
  if (auto i = value.value_exact<std::int64_t>()) return i;
  param_error(node, key, "must be an integer");
}

std::optional<double> NodeAccess::find_double(const Node& node,
                                              std::string_view key) {
  const auto value = node.impl_->Param(key);
  if (!value) return std::nullopt;
  if (value.is_floating_point() || value.is_integer()) {
    if (auto d = value.value<double>()) return d;
  }
  param_error(node, key, "must be a number");
}

std::optional<std::string> NodeAccess::find_string(const Node& node,
                                                   std::string_view key) {
  const auto value = node.impl_->Param(key);
  if (!value) return std::nullopt;
  if (auto s = value.value_exact<std::string>()) return s;
  param_error(node, key, "must be a string");
}

std::optional<ParamValue> NodeAccess::find_param(const Node& node,
                                                 std::string_view key) {
  const auto value = node.impl_->Param(key);
  if (!value) return std::nullopt;
  if (auto b = value.value_exact<bool>()) return ParamValue{*b};
  if (auto i = value.value_exact<std::int64_t>()) return ParamValue{*i};
  if (value.is_floating_point()) {
    if (auto d = value.value<double>()) return ParamValue{*d};
  }
  if (auto s = value.value_exact<std::string>()) {
    return ParamValue{*std::move(s)};
  }
  param_error(node, key, "must be a boolean, number or string");
}

void NodeAccess::param_error(const Node& node, std::string_view key,
                             std::string_view problem) {
  const std::string& name = node.impl_->name;
  throw std::invalid_argument(name + ": parameter '" + std::string{key} + "' " +
                              std::string{problem} + " in [subsystems." + name +
                              "]");
}

// --- topics ------------------------------------------------------------------

namespace {

void CheckLayout(const std::string& node, std::string_view topic,
                 std::size_t bytes, std::size_t alignment) {
  if (bytes == 0 || bytes > std::numeric_limits<std::uint32_t>::max() ||
      alignment == 0 || alignment > kMessageAlignment ||
      (alignment & (alignment - 1)) != 0) {
    throw std::invalid_argument(node + ": bad message layout for '" +
                                std::string{topic} + "'");
  }
}

}  // namespace

std::size_t NodeAccess::add_subscription(Node& node, std::string_view topic,
                                         std::size_t bytes,
                                         std::size_t alignment, bool optional,
                                         MessageCallback on_message) {
  Node::Impl& impl = *node.impl_;
  std::string resolved = impl.ResolveTopic(topic);
  CheckLayout(impl.name, resolved, bytes, alignment);
  auto slot = std::make_unique<Node::Impl::SubscriptionSlot>();
  slot->topic = std::move(resolved);
  slot->bytes = bytes;
  slot->data = std::make_unique<std::byte[]>(bytes);
  slot->optional = optional;
  slot->on_message = std::move(on_message);
  impl.loop->register_watcher(slot->topic, static_cast<std::uint32_t>(bytes),
                              static_cast<std::uint32_t>(alignment),
                              event::Thunk{slot.get(), &Node::Impl::OnMessage});
  impl.subscriptions.push_back(std::move(slot));
  return impl.subscriptions.size() - 1;
}

const void* NodeAccess::subscription_value(const Node& node,
                                           std::size_t index) {
  return node.impl_->SubscriptionAt(index).data.get();
}

bool NodeAccess::subscription_has_value(const Node& node, std::size_t index) {
  return node.impl_->SubscriptionAt(index).has_value;
}

Duration NodeAccess::subscription_age(const Node& node, std::size_t index) {
  const auto& slot = node.impl_->SubscriptionAt(index);
  if (!slot.has_value) return Duration::max();
  return node.now() - slot.received;
}

bool NodeAccess::subscription_fresh(const Node& node, std::size_t index,
                                    Duration max_age) {
  return subscription_has_value(node, index) &&
         subscription_age(node, index) < max_age;
}

bool NodeAccess::subscription_updated(const Node& node, std::size_t index) {
  return node.impl_->SubscriptionAt(index).updated;
}

std::size_t NodeAccess::add_publisher(Node& node, std::string_view topic,
                                      std::size_t bytes,
                                      std::size_t alignment) {
  Node::Impl& impl = *node.impl_;
  const std::string resolved = impl.ResolveTopic(topic);
  CheckLayout(impl.name, resolved, bytes, alignment);
  const std::uint16_t id =
      impl.loop->register_sender(resolved, static_cast<std::uint32_t>(bytes),
                                 static_cast<std::uint32_t>(alignment));
  impl.publishers.push_back({id, bytes});
  return impl.publishers.size() - 1;
}

bool NodeAccess::publisher_send(Node& node, std::size_t index, const void* data,
                                std::size_t bytes) {
  Node::Impl& impl = *node.impl_;
  if (index >= impl.publishers.size()) {
    throw std::out_of_range(impl.name + ": no publisher at index " +
                            std::to_string(index));
  }
  const auto& publisher = impl.publishers[index];
  if (bytes != publisher.bytes) {
    throw std::invalid_argument(
        impl.name + ": sent " + std::to_string(bytes) + " bytes on a " +
        std::to_string(publisher.bytes) + "-byte topic");
  }
  return impl.loop->send(
      publisher.id,
      std::span<const std::byte>{static_cast<const std::byte*>(data), bytes});
}

// --- byte topics -------------------------------------------------------------

namespace {

// A byte topic on RTMS. This node is its only producer and owns its layout,
// so it reclaims a stale segment left by an older build.
class SharedByteTopic final : public ByteTopic {
 public:
  SharedByteTopic(std::string_view topic, std::size_t max_bytes,
                  std::size_t slots, bool drop_newest)
      : topic_{topic},
        max_bytes_{max_bytes},
        slots_{slots},
        policy_{drop_newest ? OverflowPolicy::DROP_NEWEST
                            : OverflowPolicy::OVERWRITE_OLDEST} {}

  bool write(std::span<const std::byte> message) override {
    if (!queue_) {
      queue_.emplace(topic_, max_bytes_, kMessageAlignment, slots_,
                     RTMSOptions{.overflow_policy = policy_,
                                 .read_mode = ReadMode::SEQUENCE,
                                 .reclaim_mismatched_segment = true});
    }
    return queue_->write(message) == WriteResult::SUCCESS;
  }

 private:
  std::string topic_;
  std::size_t max_bytes_;
  std::size_t slots_;
  OverflowPolicy policy_;
  std::optional<RTMSQueue> queue_;
};

}  // namespace

std::unique_ptr<ByteTopic> LoopPort::open_byte_topic(
    std::string_view topic, std::size_t max_bytes,
    const BytePublishOptions& options) {
  return std::make_unique<SharedByteTopic>(topic, max_bytes, options.slots,
                                           options.drop_newest);
}

std::size_t NodeAccess::add_byte_publisher(Node& node, std::string_view topic,
                                           std::size_t max_bytes,
                                           const BytePublishOptions& options) {
  Node::Impl& impl = *node.impl_;
  impl.CheckOpen("publish_bytes");
  std::string resolved = impl.ResolveTopic(topic);
  CheckLayout(impl.name, resolved, max_bytes, kMessageAlignment);
  if (!is_pow_2(options.slots) || options.slots > MAX_SLOTS) {
    throw std::invalid_argument(impl.name + ": byte topic '" + resolved +
                                "' needs a power-of-two slot count up to " +
                                std::to_string(MAX_SLOTS));
  }
  auto slot = std::make_unique<Node::Impl::BytePublisherSlot>();
  slot->topic = std::move(resolved);
  slot->max_bytes = max_bytes;
  slot->external = options.external;
  slot->sink = impl.loop->open_byte_topic(slot->topic, max_bytes, options);
  impl.byte_publishers.push_back(std::move(slot));
  return impl.byte_publishers.size() - 1;
}

bool NodeAccess::byte_publisher_send(Node& node, std::size_t index,
                                     std::span<const std::byte> message) {
  Node::Impl::BytePublisherSlot& slot = node.impl_->ByteAt(index);
  if (message.size() > slot.max_bytes || !slot.sink->write(message)) {
    slot.dropped.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  slot.published.fetch_add(1, std::memory_order_relaxed);
  return true;
}

std::vector<NodeAccess::ByteTopicShape> NodeAccess::byte_topics(
    const Node& node) {
  std::vector<ByteTopicShape> topics;
  for (const auto& slot : node.impl_->byte_publishers) {
    topics.push_back({slot->topic, slot->max_bytes, slot->external});
  }
  return topics;
}

std::uint64_t NodeAccess::byte_topic_published(const Node& node,
                                               std::size_t index) {
  return node.impl_->ByteAt(index).published.load(std::memory_order_relaxed);
}

std::uint64_t NodeAccess::byte_topic_dropped(const Node& node,
                                             std::size_t index) {
  return node.impl_->ByteAt(index).dropped.load(std::memory_order_relaxed);
}

}  // namespace detail

// --- the binary --------------------------------------------------------------

namespace detail {
namespace {

// Flags every node binary accepts.
struct Flags {
  std::string log_path;
  std::string replay_path;
  std::string config_path;
  int duration_s = 0;
  std::uint64_t session_id = 0;
  bool simulation = false;
  bool describe = false;
};

Flags ParseFlags(int argc, char** argv, const char* node_name) {
  Flags flags;
  flags.log_path = std::string{"/tmp/"} + node_name + ".tlog";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--sim") {
      flags.simulation = true;
    } else if (arg == "--describe") {
      flags.describe = true;
    } else if (arg == "--log" && i + 1 < argc) {
      flags.log_path = argv[++i];
    } else if (arg == "--session-id" && i + 1 < argc) {
      flags.session_id = std::stoull(argv[++i]);
    } else if (arg == "--config" && i + 1 < argc) {
      flags.config_path = argv[++i];
    } else if (arg == "--replay" && i + 1 < argc) {
      flags.replay_path = argv[++i];
    } else if (arg == "--duration-s" && i + 1 < argc) {
      flags.duration_s = std::stoi(argv[++i]);
    } else {
      throw std::invalid_argument(
          "usage: node [--sim] [--describe] [--duration-s N] [--log PATH] "
          "[--session-id ID] [--config PATH] [--replay PATH]");
    }
  }
  if (flags.duration_s < 0) {
    throw std::invalid_argument("duration must be nonnegative");
  }
  if (flags.config_path.empty()) {
    throw std::invalid_argument("missing --config PATH");
  }
  return flags;
}

// One node as the runner holds it. Members destroy in reverse: the user's
// object, then the Node its handles point into, then the loop port.
struct Hosted {
  std::unique_ptr<LoopPort> port;
  std::unique_ptr<Node> node;
  Instance user;
};

// Built the same way on all three loops, so --describe reports, and replay
// checks, exactly what the live constructor registers.
template <class Loop>
Hosted Host(Loop& loop, const char* node_name, const Flags& flags,
            const InstanceFactory& make) {
  Hosted hosted;
  hosted.port = std::make_unique<LoopPortFor<Loop>>(loop);
  hosted.node = NodeAccess::create(NodeSetup{
      .name = node_name,
      .config_path = flags.config_path,
      .config = std::make_shared<const config::RobotConfig>(
          config::ParseRobotConfig(flags.config_path)),
      .simulation = flags.simulation,
      .loop = hosted.port.get(),
  });
  hosted.user = make(*hosted.node);
  NodeAccess::start(*hosted.node);
  return hosted;
}

// Endpoint attributes of the loop's sources: the optional subscriptions.
std::vector<introspect::EndpointAttribute> Endpoints(const Node& node) {
  std::vector<introspect::EndpointAttribute> attributes;
  for (const std::string_view topic : NodeAccess::optional_topics(node)) {
    attributes.push_back({topic, introspect::naming::kSourceFlagOptional});
  }
  return attributes;
}

std::uint32_t ByteTopicFlags(const NodeAccess::ByteTopicShape& topic) {
  return topic.external ? introspect::naming::kSourceFlagExternal : 0u;
}

// Byte topics bypass the loop, so its manifest misses them; they are
// declared to the registry here, with their own counters.
std::vector<introspect::Reporter::ExtraSource> ByteTopicSources(
    const Node& node) {
  std::vector<introspect::Reporter::ExtraSource> sources;
  const auto topics = NodeAccess::byte_topics(node);
  for (std::size_t i = 0; i < topics.size(); ++i) {
    sources.push_back({
        .kind = event::SourceKind::SENDER,
        .name = std::string{topics[i].topic},
        .message_bytes = static_cast<std::uint32_t>(topics[i].max_bytes),
        .alignment = static_cast<std::uint32_t>(kMessageAlignment),
        .flags = ByteTopicFlags(topics[i]),
        .events = [&node,
                   i] { return NodeAccess::byte_topic_published(node, i); },
        .dropped = [&node,
                    i] { return NodeAccess::byte_topic_dropped(node, i); },
    });
  }
  return sources;
}

}  // namespace

int RunMain(int argc, char** argv, const char* node_name,
            const char* node_target, const InstanceFactory& make) {
  try {
    const Flags flags = ParseFlags(argc, argv, node_name);
    // --describe builds the node for real and prints what its constructor
    // registered. The loop is the only difference: a simulated loop's
    // channels live in this process, so no shared memory, no hardware and no
    // network is touched, and the launcher can ask what a node is without
    // starting a robot.
    if (flags.describe) {
      event::SimulationEnvironment environment;
      event::SimulatedEventLoop<> loop{environment};
      const Hosted hosted = Host(loop, node_name, flags, make);
      auto description = introspect::DescribeManifest(
          node_name, node_target, loop.manifest(), Endpoints(*hosted.node));
      for (const auto& topic : NodeAccess::byte_topics(*hosted.node)) {
        description.sources.push_back(
            {event::SourceKind::SENDER, std::string{topic.topic},
             static_cast<std::uint32_t>(topic.max_bytes),
             ByteTopicFlags(topic)});
      }
      std::printf("%s", introspect::DescribeToJson(description).c_str());
      return 0;
    }
    if (!flags.replay_path.empty()) {
      event::log::LogReader reader{flags.replay_path};
      event::ReplayEventLoop<> loop{reader};
      const Hosted hosted = Host(loop, node_name, flags, make);
      loop.run();
      std::printf("replay: diverged=%d clean_exit=%d\n", loop.diverged(),
                  loop.reached_exit());
      return loop.diverged() || !loop.reached_exit() ? 1 : 0;
    }
    event::RealtimeEventLoop<event::log::LogWriter> loop{event::log::LogWriter{
        flags.log_path, node_name, event::log::LogWriterOptions{},
        flags.session_id}};
    const Hosted hosted = Host(loop, node_name, flags, make);
    process::InstallStopHandlers();
    // Publishes this node's topics and activity into the live registry, so
    // Studio and any agent can see what is running without being told.
    // Declared after the loop and the node: reverse destruction order joins
    // its thread before the counters it samples are destroyed.
    introspect::Reporter reporter{loop,
                                  {.name = node_name,
                                   .target = node_target,
                                   .session_id = flags.session_id,
                                   .simulation = flags.simulation,
                                   .extra = ByteTopicSources(*hosted.node),
                                   .endpoints = Endpoints(*hosted.node)}};

    std::jthread stopper{[&](const std::stop_token& stop) {
      while (!stop.stop_requested()) {
        if (process::stop_requested.load() && loop.running()) {
          loop.exit();
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
      }
    }};
    // Last, from this thread: the reporter and the stopper above are
    // bookkeeping and must not inherit the loop's realtime policy.
    if (!flags.simulation) {
      process::ConfigureRealtime(
          {.period = hosted.node->period(), .priority = 50});
    }
    if (flags.duration_s) {
      loop.run_for(std::chrono::seconds{flags.duration_s});
    } else {
      loop.run();
    }
    stopper.request_stop();
    std::printf("%s: dispatches=%llu recording_failed=%d\n", node_name,
                static_cast<unsigned long long>(loop.dispatch_count()),
                loop.recorder().failed());
    if (loop.recorder().failed()) {
      std::fprintf(stderr, "%s\n", loop.recorder().error().c_str());
      return 1;
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}

}  // namespace detail
}  // namespace talos
