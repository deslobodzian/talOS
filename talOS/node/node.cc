#include "talOS/node/node.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/context.h"
#include "talOS/events/input_replay.h"
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
    bool observer = false;
    // Its publications wake the node (require_immediate_delivery).
    bool prompt = false;
    detail::MessageCallback on_message;
    std::uint16_t watcher = 0;  // the loop's source id
    bool has_value = false;
    Time published{};
    std::uint64_t sequence = 0;  // RTMS sequence of the latched message
    bool pending = false;        // arrived since the last update() began
    bool updated = false;        // what update() sees: pending at its start
  };
  struct PublisherSlot {
    std::uint16_t id;
    std::size_t bytes;
  };
  struct BytePublisherSlot {
    std::string topic;
    std::size_t max_bytes;
    std::size_t slots;
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
  struct SelectionSlot {
    std::vector<std::string> sources;
    // Written by the loop's thread on every selection, read by the
    // registry's.
    std::atomic<std::int32_t> selected{-1};
  };

  std::string name;
  std::string config_path;
  std::shared_ptr<const config::RobotConfig> config;
  bool simulation = false;
  bool describing = false;
  detail::LoopPort* loop = nullptr;
  detail::GraphSource graph;
  std::optional<std::filesystem::path> session_dir;
  bool reads_graph = false;
  // A publisher on a target lane (`/<sub>/target/<source>`) is registered.
  bool writes_target_lane = false;
  // The topic add_hardware_request registered, or empty.
  std::string hardware_request;
  Duration period{};
  Duration timeout{};

  std::vector<std::unique_ptr<SubscriptionSlot>> subscriptions;
  std::vector<PublisherSlot> publishers;
  std::vector<std::unique_ptr<BytePublisherSlot>> byte_publishers;
  std::vector<std::unique_ptr<TimerSlot>> timers;
  std::vector<StateMachineSlot> state_machines;
  std::vector<std::unique_ptr<SelectionSlot>> selections;
  std::vector<std::pair<std::type_index, std::shared_ptr<void>>> extensions;

  std::function<void()> update;
  std::vector<std::function<void()>> after_update;
  bool started = false;
  bool in_update = false;
  std::uint64_t ticks = 0;
  // The thread that built the node, which also runs its loop: talos::Run,
  // testing::Sim and the C ABI do both on one thread. require_loop_thread
  // accepts sends, and the devices layer's commands, from it alone.
  std::thread::id loop_thread = std::this_thread::get_id();

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

  SelectionSlot& SelectionAt(std::size_t index) const {
    if (index >= selections.size()) {
      throw std::out_of_range(name + ": no target selection at index " +
                              std::to_string(index));
    }
    return *selections[index];
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

  // Every key the constructor read, present or not: what --describe reports
  // the node reads (introspect::Description::params). Reads after the loop
  // starts are not recorded, so update() allocates nothing for them.
  mutable std::vector<introspect::ParamRead> params_read;

  void RecordParam(std::string_view subsystem, std::string_view key) const {
    if (started) return;
    introspect::ParamRead read{std::string{subsystem}, std::string{key}};
    if (std::find(params_read.begin(), params_read.end(), read) ==
        params_read.end()) {
      params_read.push_back(std::move(read));
    }
  }

  // `key` in this node's table, or in `subsystem`'s (param_of: read once,
  // in the constructor, from a subsystem the config must have).
  toml::node_view<const toml::node> Param(
      std::string_view key, std::optional<std::string_view> subsystem) const {
    if (!subsystem) {
      RecordParam(name, key);
      const toml::table* table = Table();
      if (table == nullptr) return {};
      return table->at_path(key);
    }
    CheckOpen("param_of");
    const toml::table* table =
        config->GetSubsystemTable(std::string{*subsystem});
    if (table == nullptr) {
      throw std::invalid_argument(name + ": param_of('" +
                                  std::string{*subsystem} + "', '" +
                                  std::string{key} + "'): no [subsystems." +
                                  std::string{*subsystem} + "] in the config");
    }
    RecordParam(*subsystem, key);
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
      if (auto s = entry.value_exact<std::string>()) return s;
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
    slot->published = context.publish_time;
    slot->sequence = context.sequence;
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
  impl.describing = setup.describing;
  impl.loop = setup.loop;
  impl.graph = std::move(setup.graph);
  impl.session_dir = std::move(setup.session_dir);
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
bool Node::describing() const { return impl_->describing; }
Time Node::now() const { return impl_->loop->now(); }
std::uint64_t Node::tick() const { return impl_->ticks; }
Duration Node::period() const { return impl_->period; }
const std::string& Node::config_path() const { return impl_->config_path; }

std::optional<std::filesystem::path> Node::session_dir() const {
  return impl_->session_dir;
}

std::filesystem::path Node::trajectory_path(std::string_view name) const {
  try {
    return impl_->config->TrajectoryPath(impl_->name, name);
  } catch (const std::invalid_argument& error) {
    throw std::invalid_argument(impl_->name + ": " + error.what());
  }
}

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
  // Draining before timers needs a timer: in a node with none (it only lets
  // its extensions latch values for someone else to read) nothing else would
  // ever drain, so every subscription wakes it.
  const bool has_tick = impl.update || !impl.after_update.empty();
  const bool timed = has_tick || !impl.timers.empty();
  bool prompt = false;
  for (const auto& slot : impl.subscriptions) {
    if (slot->prompt || !timed) {
      impl.loop->set_prompt(slot->watcher);
      prompt = true;
    }
  }
  impl.loop->set_delivery(prompt || !timed ? event::Delivery::ON_PUBLISH
                                           : event::Delivery::BEFORE_TIMERS);
  if (!has_tick) return;
  const std::uint16_t id = impl.loop->register_timer(
      impl.name, event::Thunk{&impl, &Node::Impl::OnTick});
  impl.loop->arm_timer(id, node.now() + impl.period, impl.period);
}

void NodeAccess::require_loop_thread(const Node& node, std::string_view what) {
  const Node::Impl& impl = *node.impl_;
  if (std::this_thread::get_id() == impl.loop_thread) return;
  throw std::logic_error(impl.name + ": " + std::string{what} +
                         " must run on the node's loop thread");
}

bool NodeAccess::in_update(const Node& node) { return node.impl_->in_update; }

void NodeAccess::require_immediate_delivery(Node& node,
                                            std::size_t subscription) {
  Node::Impl& impl = *node.impl_;
  impl.CheckOpen("subscribe with a callback");
  if (subscription >= impl.subscriptions.size()) {
    throw std::out_of_range(impl.name + ": no subscription at index " +
                            std::to_string(subscription));
  }
  impl.subscriptions[subscription]->prompt = true;
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

std::vector<std::string_view> NodeAccess::observer_topics(const Node& node) {
  std::vector<std::string_view> topics;
  for (const auto& slot : node.impl_->subscriptions) {
    if (slot->observer) topics.push_back(slot->topic);
  }
  return topics;
}

const DeclaredGraph* NodeAccess::declared_graph(Node& node) {
  Node::Impl& impl = *node.impl_;
  impl.CheckOpen("declared_graph");
  impl.reads_graph = true;
  return impl.graph ? impl.graph() : nullptr;
}

bool NodeAccess::reads_graph(const Node& node) {
  return node.impl_->reads_graph;
}

const std::vector<introspect::ParamRead>& NodeAccess::params_read(
    const Node& node) {
  return node.impl_->params_read;
}

const config::RobotConfig& NodeAccess::robot_config(const Node& node) {
  return *node.impl_->config;
}

process::LoopTier NodeAccess::loop_tier(const Node& node) {
  const Node::Impl& impl = *node.impl_;
  const hardware::Devices* devices = impl.config->GetDevices(impl.name);
  const bool owns_actuators = devices != nullptr && devices->owns_actuators();
  return owns_actuators || impl.writes_target_lane
             ? process::LoopTier::kControl
             : process::LoopTier::kObserver;
}

std::optional<std::filesystem::path> NodeAccess::session_dir(const Node& node) {
  return node.impl_->session_dir;
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
                                          std::string_view key,
                                          ParamTable subsystem) {
  const auto value = node.impl_->Param(key, subsystem);
  if (!value) return std::nullopt;
  if (auto b = value.value_exact<bool>()) return b;
  param_error(node, key, "must be a boolean", subsystem);
}

std::optional<std::int64_t> NodeAccess::find_int(const Node& node,
                                                 std::string_view key,
                                                 ParamTable subsystem) {
  const auto value = node.impl_->Param(key, subsystem);
  if (!value) return std::nullopt;
  if (auto i = value.value_exact<std::int64_t>()) return i;
  param_error(node, key, "must be an integer", subsystem);
}

std::optional<double> NodeAccess::find_double(const Node& node,
                                              std::string_view key,
                                              ParamTable subsystem) {
  const auto value = node.impl_->Param(key, subsystem);
  if (!value) return std::nullopt;
  if (value.is_floating_point() || value.is_integer()) {
    if (auto d = value.value<double>()) return d;
  }
  param_error(node, key, "must be a number", subsystem);
}

std::optional<std::string> NodeAccess::find_string(const Node& node,
                                                   std::string_view key,
                                                   ParamTable subsystem) {
  const auto value = node.impl_->Param(key, subsystem);
  if (!value) return std::nullopt;
  if (auto s = value.value_exact<std::string>()) return s;
  param_error(node, key, "must be a string", subsystem);
}

namespace {

std::string_view TypeName(const toml::node& value) {
  switch (value.type()) {
    case toml::node_type::string:
      return "a string";
    case toml::node_type::integer:
      return "an integer";
    case toml::node_type::floating_point:
      return "a float";
    case toml::node_type::boolean:
      return "a boolean";
    case toml::node_type::array:
      return "an array";
    case toml::node_type::table:
      return "a table";
    default:
      return "a date or time";
  }
}

[[noreturn]] void ElementError(const Node& node, std::string_view key,
                               NodeAccess::ParamTable subsystem,
                               std::string_view expected, std::size_t index,
                               const toml::node& element) {
  NodeAccess::param_error(node, key,
                          std::string{expected} + "; element " +
                              std::to_string(index) + " is " +
                              std::string{TypeName(element)},
                          subsystem);
}

std::vector<std::string> StringList(const Node& node, std::string_view key,
                                    NodeAccess::ParamTable subsystem,
                                    const toml::array& array,
                                    std::string_view expected) {
  std::vector<std::string> out;
  out.reserve(array.size());
  for (std::size_t i = 0; i < array.size(); ++i) {
    const toml::value<std::string>* s = array[i].as_string();
    if (s == nullptr) ElementError(node, key, subsystem, expected, i, array[i]);
    out.push_back(s->get());
  }
  return out;
}

std::vector<double> DoubleList(const Node& node, std::string_view key,
                               NodeAccess::ParamTable subsystem,
                               const toml::array& array,
                               std::string_view expected) {
  std::vector<double> out;
  out.reserve(array.size());
  for (std::size_t i = 0; i < array.size(); ++i) {
    const toml::node& element = array[i];
    std::optional<double> d;
    if (element.is_floating_point() || element.is_integer()) {
      d = element.value<double>();
    }
    if (!d) ElementError(node, key, subsystem, expected, i, element);
    out.push_back(*d);
  }
  return out;
}

}  // namespace

std::optional<std::vector<std::string>> NodeAccess::find_string_list(
    const Node& node, std::string_view key, ParamTable subsystem) {
  constexpr std::string_view kExpected = "must be an array of strings";
  const auto value = node.impl_->Param(key, subsystem);
  if (!value) return std::nullopt;
  const toml::array* array = value.as_array();
  if (array == nullptr) param_error(node, key, kExpected, subsystem);
  return StringList(node, key, subsystem, *array, kExpected);
}

std::optional<std::vector<double>> NodeAccess::find_double_list(
    const Node& node, std::string_view key, ParamTable subsystem) {
  constexpr std::string_view kExpected = "must be an array of numbers";
  const auto value = node.impl_->Param(key, subsystem);
  if (!value) return std::nullopt;
  const toml::array* array = value.as_array();
  if (array == nullptr) param_error(node, key, kExpected, subsystem);
  return DoubleList(node, key, subsystem, *array, kExpected);
}

std::optional<ParamValue> NodeAccess::find_param(const Node& node,
                                                 std::string_view key,
                                                 ParamTable subsystem) {
  const auto value = node.impl_->Param(key, subsystem);
  if (!value) return std::nullopt;
  if (auto b = value.value_exact<bool>()) return ParamValue{*b};
  if (auto i = value.value_exact<std::int64_t>()) return ParamValue{*i};
  if (value.is_floating_point()) {
    if (auto d = value.value<double>()) return ParamValue{*d};
  }
  if (auto s = value.value_exact<std::string>()) {
    return ParamValue{*std::move(s)};
  }
  if (const toml::array* array = value.as_array()) {
    constexpr std::string_view kExpected =
        "must be an array of strings or of numbers";
    if (!array->empty() && array->front().is_string()) {
      return ParamValue{StringList(node, key, subsystem, *array, kExpected)};
    }
    return ParamValue{DoubleList(node, key, subsystem, *array, kExpected)};
  }
  param_error(node, key, "must be a boolean, number, string or array",
              subsystem);
}

void NodeAccess::param_error(const Node& node, std::string_view key,
                             std::string_view problem, ParamTable subsystem) {
  const std::string& name = node.impl_->name;
  const std::string table{subsystem.value_or(name)};
  throw std::invalid_argument(name + ": parameter '" + std::string{key} + "' " +
                              std::string{problem} + " in [subsystems." +
                              table + "]");
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

// RULES.md 2: a hardware request leaves only through the devices layer's flush
// after update() (add_hardware_request), which gates it on robot_enabled and
// update() and applies the motor limits and position frames. A plain
// publisher there would skip all of that, so none is opened, however the
// topic was spelled or remapped, whatever --allow-graph-errors says.
void RefuseHardwareRequest(const std::string& node, std::string_view topic) {
  if (introspect::naming::IsHardwareRequest(topic)) {
    throw std::invalid_argument(
        node + ": cannot publish '" + std::string{topic} +
        "': hardware requests are sent only by the devices layer's flush "
        "after update(); command the node's devices (Motor, DigitalOutput, "
        "PwmOutput) instead");
  }
}

}  // namespace

std::size_t NodeAccess::add_subscription(Node& node, std::string_view topic,
                                         std::size_t bytes,
                                         std::size_t alignment,
                                         const schema::Identity& schema,
                                         bool optional,
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
  slot->watcher = impl.loop->register_watcher(
      slot->topic, static_cast<std::uint32_t>(bytes),
      static_cast<std::uint32_t>(alignment), schema,
      event::Thunk{slot.get(), &Node::Impl::OnMessage}, !slot->on_message);
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
  return node.now() - slot.published;
}

void NodeAccess::record_read(Node& node, std::size_t index) {
  const auto& slot = node.impl_->SubscriptionAt(index);
  if (!slot.has_value) return;
  node.impl_->loop->record_read(slot.watcher, node.now(), slot.published,
                                slot.sequence);
}

bool NodeAccess::subscription_fresh(const Node& node, std::size_t index,
                                    Duration max_age) {
  return subscription_has_value(node, index) &&
         subscription_age(node, index) < max_age;
}

bool NodeAccess::subscription_updated(const Node& node, std::size_t index) {
  return node.impl_->SubscriptionAt(index).updated;
}

std::size_t NodeAccess::add_observer(Node& node, std::string_view topic,
                                     std::size_t bytes, std::size_t alignment,
                                     const schema::Identity& schema) {
  const std::size_t index = add_subscription(node, topic, bytes, alignment,
                                             schema, /*optional=*/false, {});
  node.impl_->subscriptions[index]->observer = true;
  return index;
}

std::size_t NodeAccess::add_publisher(Node& node, std::string_view topic,
                                      std::size_t bytes, std::size_t alignment,
                                      const schema::Identity& schema) {
  Node::Impl& impl = *node.impl_;
  const std::string resolved = impl.ResolveTopic(topic);
  RefuseHardwareRequest(impl.name, resolved);
  CheckLayout(impl.name, resolved, bytes, alignment);
  const std::uint16_t id =
      impl.loop->register_sender(resolved, static_cast<std::uint32_t>(bytes),
                                 static_cast<std::uint32_t>(alignment), schema);
  impl.publishers.push_back({id, bytes});
  if (introspect::naming::IsTargetLane(resolved)) {
    impl.writes_target_lane = true;
  }
  return impl.publishers.size() - 1;
}

std::size_t NodeAccess::add_hardware_request(Node& node, std::size_t bytes,
                                             std::size_t alignment,
                                             const schema::Identity& schema) {
  Node::Impl& impl = *node.impl_;
  if (!impl.hardware_request.empty()) {
    throw std::logic_error(impl.name + ": hardware request '" +
                           impl.hardware_request + "' is already registered");
  }
  std::string topic = introspect::naming::HardwareRequestTopic(impl.name);
  CheckLayout(impl.name, topic, bytes, alignment);
  const std::uint16_t id =
      impl.loop->register_sender(topic, static_cast<std::uint32_t>(bytes),
                                 static_cast<std::uint32_t>(alignment), schema);
  impl.publishers.push_back({id, bytes});
  impl.hardware_request = std::move(topic);
  return impl.publishers.size() - 1;
}

std::string_view NodeAccess::hardware_request(const Node& node) {
  return node.impl_->hardware_request;
}

bool NodeAccess::publisher_send(Node& node, std::size_t index, const void* data,
                                std::size_t bytes) {
  require_loop_thread(node, "Publisher::send");
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

// A byte topic on RTMS (schema talos.Bytes), opened, and its writer claimed,
// as the node registers it, the way the realtime loop opens its typed topics:
// a topic another live process writes fails the node's construction, not a
// send in the middle of a run.
class SharedByteTopic final : public ByteTopic {
 public:
  SharedByteTopic(std::string_view topic, std::size_t max_bytes,
                  std::size_t slots, bool drop_newest)
      : queue_{topic,
               max_bytes,
               kMessageAlignment,
               schema::Bytes(),
               slots,
               RTMSOptions{.overflow_policy =
                               drop_newest ? OverflowPolicy::DROP_NEWEST
                                           : OverflowPolicy::OVERWRITE_OLDEST,
                           .read_mode = ReadMode::SEQUENCE,
                           .publisher = true}} {}

  bool write(std::span<const std::byte> message) override {
    return queue_.write(message) == WriteResult::SUCCESS;
  }

 private:
  RTMSQueue queue_;
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
  RefuseHardwareRequest(impl.name, resolved);
  CheckLayout(impl.name, resolved, max_bytes, kMessageAlignment);
  if (!is_pow_2(options.slots) || options.slots > MAX_SLOTS) {
    throw std::invalid_argument(impl.name + ": byte topic '" + resolved +
                                "' needs a power-of-two slot count up to " +
                                std::to_string(MAX_SLOTS));
  }
  auto slot = std::make_unique<Node::Impl::BytePublisherSlot>();
  slot->topic = std::move(resolved);
  slot->max_bytes = max_bytes;
  slot->slots = options.slots;
  slot->external = options.external;
  slot->sink = impl.loop->open_byte_topic(slot->topic, max_bytes, options);
  impl.byte_publishers.push_back(std::move(slot));
  return impl.byte_publishers.size() - 1;
}

bool NodeAccess::byte_publisher_send(Node& node, std::size_t index,
                                     std::span<const std::byte> message) {
  require_loop_thread(node, "BytePublisher::send");
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
    topics.push_back(
        {slot->topic, slot->max_bytes, slot->slots, slot->external});
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

std::size_t NodeAccess::add_selection(Node& node,
                                      std::vector<std::string> sources) {
  node.impl_->CheckOpen("add_selection");
  auto slot = std::make_unique<Node::Impl::SelectionSlot>();
  slot->sources = std::move(sources);
  node.impl_->selections.push_back(std::move(slot));
  return node.impl_->selections.size() - 1;
}

void NodeAccess::set_selected(Node& node, std::size_t index,
                              std::int32_t source) {
  std::atomic<std::int32_t>& selected = node.impl_->SelectionAt(index).selected;
  // The loop's thread is the only writer; skipping an unchanged store keeps
  // the registry thread's cache line clean on every tick a lane holds.
  if (selected.load(std::memory_order_relaxed) != source) {
    selected.store(source, std::memory_order_relaxed);
  }
}

std::size_t NodeAccess::selection_count(const Node& node) {
  return node.impl_->selections.size();
}

std::span<const std::string> NodeAccess::selection_sources(const Node& node,
                                                           std::size_t index) {
  return node.impl_->SelectionAt(index).sources;
}

std::int32_t NodeAccess::selected(const Node& node, std::size_t index) {
  return node.impl_->SelectionAt(index).selected.load(
      std::memory_order_relaxed);
}

}  // namespace detail

// --- the binary --------------------------------------------------------------

namespace detail {
namespace {

// Flags every node binary accepts.
struct Flags {
  std::string log_path;
  std::string replay_path;
  std::string replay_inputs;
  std::string config_path;
  std::string graph_path;
  int duration_s = 0;
  std::uint64_t session_id = 0;
  std::uint64_t log_max_bytes = 0;  // --log-max-bytes; 0 is unbounded
  // --log-window-us: LogWriterOptions::window; 0 ends segments on bytes only.
  std::chrono::nanoseconds log_window{0};
  bool simulation = false;
  bool describe = false;
  bool log_set = false;  // --log was passed (vs. the /tmp default)
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
      flags.log_set = true;
    } else if (arg == "--log-max-bytes" && i + 1 < argc) {
      flags.log_max_bytes = std::stoull(argv[++i]);
    } else if (arg == "--log-window-us" && i + 1 < argc) {
      flags.log_window = event::log::ParseLogWindowUs(argv[++i]);
    } else if (arg == "--session-id" && i + 1 < argc) {
      flags.session_id = std::stoull(argv[++i]);
    } else if (arg == "--config" && i + 1 < argc) {
      flags.config_path = argv[++i];
    } else if (arg == "--graph" && i + 1 < argc) {
      flags.graph_path = argv[++i];
    } else if (arg == "--replay" && i + 1 < argc) {
      flags.replay_path = argv[++i];
    } else if (arg == "--replay-inputs" && i + 1 < argc) {
      flags.replay_inputs = argv[++i];
    } else if (arg == "--duration-s" && i + 1 < argc) {
      flags.duration_s = std::stoi(argv[++i]);
    } else {
      throw std::invalid_argument(
          "usage: node [--sim] [--describe] [--duration-s N] [--log PATH] "
          "[--log-max-bytes N] [--log-window-us N] [--session-id ID] "
          "[--config PATH] "
          "[--graph PATH] [--replay PATH] [--replay-inputs SESSION_DIR]");
    }
  }
  if (!flags.replay_path.empty() && !flags.replay_inputs.empty()) {
    throw std::invalid_argument(
        "--replay and --replay-inputs exclude each other");
  }
  if (flags.duration_s < 0) {
    throw std::invalid_argument("duration must be nonnegative");
  }
  if (flags.config_path.empty()) {
    throw std::invalid_argument("missing --config PATH");
  }
  return flags;
}

// The declared graph from --graph, read on first use. Without the flag:
// under --replay-inputs, the one the replayed session was launched with
// (<session dir>/declared_graph.json, which the launcher writes into a
// session that runs a graph reader); none yet under --describe (the
// launcher's first pass); and an error otherwise, since a node that reads the
// graph cannot be built without one.
GraphSource GraphFromFlags(const Flags& flags, std::string_view node_name) {
  std::string path = flags.graph_path;
  std::string unreadable = "cannot read --graph " + path;
  if (path.empty() && !flags.replay_inputs.empty()) {
    path = (std::filesystem::path{flags.replay_inputs} / "declared_graph.json")
               .string();
    unreadable = std::string{node_name} +
                 " reads the declared graph, and the replayed session has "
                 "none: cannot read " +
                 path +
                 " (a session without a graph reader has none); pass --graph";
  }
  if (path.empty()) {
    if (flags.describe) return [] { return nullptr; };
    return [name = std::string{node_name}]() -> const DeclaredGraph* {
      throw std::runtime_error(
          name +
          " reads the declared graph: run it under the launcher, which "
          "passes --graph, or pass --graph <session dir>/declared_graph.json");
    };
  }
  auto graph = std::make_shared<std::optional<DeclaredGraph>>();
  return [graph, path = std::move(path),
          unreadable = std::move(unreadable)]() -> const DeclaredGraph* {
    if (!graph->has_value()) {
      std::ifstream file{path};
      if (!file) throw std::runtime_error(unreadable);
      const std::string text{std::istreambuf_iterator<char>{file}, {}};
      DeclaredGraph parsed;
      std::string error;
      if (!introspect::DescribeReader::ParseGraph(text, parsed, error)) {
        throw std::runtime_error("--graph " + path + ": " + error);
      }
      graph->emplace(std::move(parsed));
    }
    return &**graph;
  };
}

// One node as the runner holds it. Members destroy in reverse: the user's
// object, then the Node its handles point into, then the loop port.
struct Hosted {
  std::unique_ptr<LoopPort> port;
  std::unique_ptr<Node> node;
  Instance user;
};

// Built the same way on every loop, so --describe reports, and replay
// checks, exactly what the live constructor registers.
Hosted Host(std::unique_ptr<LoopPort> port, const char* node_name,
            const Flags& flags, const InstanceFactory& make) {
  Hosted hosted;
  hosted.port = std::move(port);
  // An offline run has no session, even with --log (--replay-inputs always
  // has one): state a node keeps per session or across restarts (the 2026
  // autonomous node's selection and progress) must not be read or written by
  // a replay or a probe.
  const bool offline = flags.describe || !flags.replay_path.empty() ||
                       !flags.replay_inputs.empty();
  std::optional<std::filesystem::path> session_dir;
  if (flags.log_set && !offline) {
    session_dir.emplace(std::filesystem::path{flags.log_path}.parent_path());
  }
  NodeSetup setup;
  setup.name = node_name;
  setup.config_path = flags.config_path;
  setup.config = std::make_shared<const config::RobotConfig>(
      config::ParseRobotConfig(flags.config_path));
  setup.simulation = flags.simulation;
  setup.describing = flags.describe;
  setup.loop = hosted.port.get();
  setup.graph = GraphFromFlags(flags, node_name);
  setup.session_dir = std::move(session_dir);
  hosted.node = NodeAccess::create(std::move(setup));
  hosted.user = make(*hosted.node);
  NodeAccess::start(*hosted.node);
  return hosted;
}

template <class Loop>
Hosted Host(Loop& loop, const char* node_name, const Flags& flags,
            const InstanceFactory& make) {
  return Host(std::make_unique<LoopPortFor<Loop>>(loop), node_name, flags,
              make);
}

// A byte topic that goes nowhere, for runs that must leave a robot that may
// be running on this machine alone: --describe (the launcher's probe) must
// not claim its writers, and a node replayed offline must not attach to or
// write into its shared memory. The loop never logged byte topics, so replay
// cannot verify them anyway.
class DiscardedByteTopic final : public ByteTopic {
 public:
  bool write(std::span<const std::byte>) override { return true; }
};

// The loop port under --describe, --replay and --replay-inputs: the loop,
// with byte topics discarded.
template <class Loop>
class OfflinePort final : public LoopPortFor<Loop> {
 public:
  using LoopPortFor<Loop>::LoopPortFor;
  std::unique_ptr<ByteTopic> open_byte_topic(
      std::string_view, std::size_t, const BytePublishOptions&) override {
    return std::make_unique<DiscardedByteTopic>();
  }
};

// --replay-inputs: this node, on a simulated loop on the recorded clock, fed
// every message the session logged on a topic it watches at its recorded
// time (talOS/events/input_replay.h). A node the session logged runs on its
// own recorded timeline (origin and run start, so its timers fire at the
// recorded deadlines) and is handed what it received live, at the drains it
// received it in. What it sends is logged to --log. A topic recorded with
// another schema fails before anything runs; topics nobody recorded are
// listed.
int RunOnRecordedInputs(const char* node_name, const Flags& flags,
                        const InstanceFactory& make) {
  using Loop = event::SimulatedEventLoop<event::log::LogWriter>;
  const event::RecordedSession session{flags.replay_inputs, node_name};
  event::SimulationEnvironment environment;
  Loop loop{
      environment,
      event::log::LogWriter{flags.log_path, node_name,
                            event::log::LogWriterOptions{}, flags.session_id},
      Loop::Options{.start = session.loop_start(),
                    .run_start = session.run_start()}};
  const Hosted hosted =
      Host(std::make_unique<OfflinePort<Loop>>(loop), node_name, flags, make);
  const event::InputReplayReport report =
      event::ReplayInputs(loop, environment, session);

  std::printf(
      "replay-inputs: %zu logs of session %llu, %.3f s\n",
      session.log_paths().size(),
      static_cast<unsigned long long>(session.session_id()),
      static_cast<double>((session.end() - session.start()).count()) / 1e9);
  for (const auto& fed : report.fed) {
    std::printf("  fed %s: %zu messages (recorded by %s)\n", fed.topic.c_str(),
                fed.messages, fed.recorded_by.c_str());
  }
  for (const auto& topic : report.unrecorded) {
    std::printf("  not recorded: %s\n", topic.c_str());
  }
  std::printf("%s: dispatches=%llu recording_failed=%d log=%s\n", node_name,
              static_cast<unsigned long long>(loop.dispatch_count()),
              loop.recorder().failed(), flags.log_path.c_str());
  if (loop.recorder().failed()) {
    std::fprintf(stderr, "%s\n", loop.recorder().error().c_str());
    return 1;
  }
  return 0;
}

// Endpoint attributes of the loop's sources: the optional subscriptions, the
// observers and the devices layer's hardware request.
std::vector<introspect::EndpointAttribute> Endpoints(const Node& node) {
  std::vector<introspect::EndpointAttribute> attributes;
  for (const std::string_view topic : NodeAccess::optional_topics(node)) {
    attributes.push_back({topic, introspect::naming::kSourceFlagOptional});
  }
  for (const std::string_view topic : NodeAccess::observer_topics(node)) {
    attributes.push_back({topic, introspect::naming::kSourceFlagObserver});
  }
  if (const std::string_view topic = NodeAccess::hardware_request(node);
      !topic.empty()) {
    attributes.push_back(
        {topic, introspect::naming::kSourceFlagHardwareRequest});
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
  sources.reserve(topics.size());
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

// Who drives the subsystem: each Target's policy and its latest selection.
std::vector<introspect::Reporter::Selection> Selections(const Node& node) {
  std::vector<introspect::Reporter::Selection> selections;
  const std::size_t count = NodeAccess::selection_count(node);
  selections.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto sources = NodeAccess::selection_sources(node, i);
    selections.push_back({
        .sources = {sources.begin(), sources.end()},
        .selected = [&node, i] { return NodeAccess::selected(node, i); },
    });
  }
  return selections;
}

}  // namespace

introspect::Description DescribeNode(
    const Node& node, std::string_view target, const event::Manifest& manifest,
    const std::vector<std::uint32_t>& source_flags) {
  auto description = introspect::DescribeManifest(
      std::string{node.name()}, std::string{target}, manifest,
      static_cast<std::uint32_t>(event::RealtimeEventLoop<>::Options{}.slots),
      Endpoints(node), source_flags);
  const schema::Identity bytes = schema::Bytes();
  for (const auto& topic : NodeAccess::byte_topics(node)) {
    introspect::naming::SourceShape source{
        event::SourceKind::SENDER, std::string{topic.topic},
        static_cast<std::uint32_t>(topic.max_bytes), ByteTopicFlags(topic)};
    source.schema = bytes.name;
    source.schema_hash = bytes.hash;
    source.alignment = static_cast<std::uint32_t>(kMessageAlignment);
    source.slots = static_cast<std::uint32_t>(topic.slots);
    description.sources.push_back(std::move(source));
  }
  description.reads_graph = NodeAccess::reads_graph(node);
  description.params = NodeAccess::params_read(node);
  return description;
}

int RunMain(int argc, char** argv, const char* node_name,
            const char* node_target, const InstanceFactory& make) {
  try {
    const Flags flags = ParseFlags(argc, argv, node_name);
    // --describe builds the node for real and prints what its constructor
    // registered. The loop and Node::describing() are the only differences:
    // a simulated loop's channels live in this process and its byte topics
    // are discarded, so no shared memory, no hardware and no network is
    // touched, and the launcher can ask what a node is without starting a
    // robot, or while one runs. The loop never runs, so no update() or
    // callback sees a node built without its backends.
    if (flags.describe) {
      event::SimulationEnvironment environment;
      event::SimulatedEventLoop<> loop{environment};
      const Hosted hosted =
          Host(std::make_unique<OfflinePort<event::SimulatedEventLoop<>>>(loop),
               node_name, flags, make);
      std::printf("%s",
                  introspect::DescribeToJson(
                      DescribeNode(*hosted.node, node_target, loop.manifest(),
                                   introspect::LoopSourceFlags(loop)))
                      .c_str());
      return 0;
    }
    if (!flags.replay_path.empty()) {
      event::log::LogReader reader{flags.replay_path};
      event::ReplayEventLoop<> loop{reader};
      const Hosted hosted =
          Host(std::make_unique<OfflinePort<event::ReplayEventLoop<>>>(loop),
               node_name, flags, make);
      loop.run();
      std::printf("replay: diverged=%d clean_exit=%d\n", loop.diverged(),
                  loop.reached_exit());
      return loop.diverged() || !loop.reached_exit() ? 1 : 0;
    }
    if (!flags.replay_inputs.empty()) {
      return RunOnRecordedInputs(node_name, flags, make);
    }
    // Before anything slow: a stop (SIGINT/SIGTERM) during construction is
    // recorded instead of killing the process, and honoured once the
    // constructor returns. A constructor blocked in a restartable system call
    // stays blocked (the handlers restart it); the launcher's SIGKILL after its
    // grace bounds that.
    process::InstallStopHandlers();
    const int stop_fd = process::StopFd();
    if (stop_fd < 0) {
      throw std::runtime_error(
          std::string{node_name} +
          ": cannot create the stop pipe: " + std::strerror(errno));
    }
    // The launcher bounds each node's share of the session's log budget; at
    // the cap the log drops its oldest segments to keep the newest records
    // (LogWriterOptions::max_bytes) and `<log>.capped` marks it incomplete.
    // It also passes the session's log window, so every capped member keeps
    // about the same record time (LogWriterOptions::window).
    event::log::LogWriterOptions log_options;
    log_options.max_bytes = flags.log_max_bytes;
    log_options.window = flags.log_window;
    // The loop reads the stop flag between dispatches, so a stop takes effect
    // after the current dispatch even when overdue timers never let it wait
    // (an update() that overruns its period).
    event::RealtimeEventLoop<event::log::LogWriter>::Options loop_options;
    loop_options.stop = &process::stop_requested;
    event::RealtimeEventLoop<event::log::LogWriter> loop{
        event::log::LogWriter{flags.log_path, node_name, log_options,
                              flags.session_id},
        loop_options};
    // Keep the hosted node alive until the reporting thread has joined, while
    // publishing identity before any user constructor or backend
    // initialization.
    std::optional<Hosted> hosted;
    introspect::Reporter reporter{{.name = node_name,
                                   .target = node_target,
                                   .session_id = flags.session_id,
                                   .simulation = flags.simulation}};
    hosted.emplace(Host(loop, node_name, flags, make));
    if (process::stop_requested.load()) {
      // Stopped while constructing: a clean stop, without running the loop.
      std::printf("%s: stopped during construction\n", node_name);
      return 0;
    }
    // A stop from here on also makes the stop pipe readable, which ends the
    // loop's wait wherever it waits, and the loop exits from its own thread:
    // no thread polls for a stop, so an idle node is woken only by its own
    // timers and publications. A stop that lands before the run starts is
    // seen by the run's first flag check.
    loop.watch_fd(stop_fd, [&loop] {
      if (process::ConsumeStopWake()) loop.exit();
    });
    reporter.start(loop, ByteTopicSources(*hosted->node),
                   Endpoints(*hosted->node), Selections(*hosted->node));

    // Last, from this thread: the reporter is bookkeeping and must not
    // inherit the loop's realtime policy. The row carries the outcome, where
    // the launcher and session_report look for a loop that runs without its
    // grant.
    process::RealtimeOutcome realtime{.tier =
                                          NodeAccess::loop_tier(*hosted->node)};
    if (process::RealtimeRequested(flags.simulation)) {
      realtime = process::ConfigureRealtime(
          {.period = hosted->node->period(), .tier = realtime.tier});
    }
    reporter.set_realtime(realtime);
    if (flags.duration_s) {
      loop.run_for(std::chrono::seconds{flags.duration_s});
    } else {
      loop.run();
    }
    std::printf("%s: dispatches=%llu recording_failed=%d recording_capped=%d\n",
                node_name,
                static_cast<unsigned long long>(loop.dispatch_count()),
                loop.recorder().failed(), loop.recorder().capped());
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
