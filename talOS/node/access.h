#pragma once

// Framework plumbing behind talos::Node: what talos::Run, testing::Sim, the
// devices layer and the C ABI (talOS/node_api) drive. Node code never uses
// it. Every operation is non-template and addresses a subscription or
// publisher by the index it was registered under, so the C ABI wraps each one
// directly.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <typeindex>
#include <variant>
#include <vector>

#include "talOS/schema/identity.h"

namespace talos {
class Node;
struct BytePublishOptions;
using Duration = std::chrono::nanoseconds;
namespace config {
struct RobotConfig;
}  // namespace config
namespace introspect {
struct Description;
}  // namespace introspect
}  // namespace talos

namespace talos::detail {

class LoopPort;  // talOS/node/internal.h

// The declared graph a framework node may read: introspect::DeclaredGraph
// (talOS/introspection/describe.h).
using DeclaredGraph = std::vector<introspect::Description>;

// Where a Node gets the declared graph: the graph, or null while there is
// none yet (the launcher's first --describe pass). May throw: a node run
// without a graph cannot build a node that needs one.
using GraphSource = std::function<const DeclaredGraph*()>;

// What a Node is built from: talos::Run, testing::Sim and the C ABI make one.
struct NodeSetup {
  std::string name;
  std::string config_path;
  std::shared_ptr<const config::RobotConfig> config;  // parsed config_path
  bool simulation = false;
  LoopPort* loop = nullptr;
  GraphSource graph;  // unset: no graph (null)
  // The session output directory the launcher started this node under (the
  // parent of --log), or nullopt without --log (tests, --describe, replay).
  std::optional<std::filesystem::path> session_dir;
};

// The user's object, type-erased so the runner and the harness need not be
// templates. The factory builds it against a Node and installs its update().
using Instance = std::shared_ptr<void>;
using InstanceFactory = std::function<Instance(Node&)>;

template <class T>
InstanceFactory MakeInstance();  // node.h

// The binary's main: flags, describe/replay/realtime, logging, the live
// registry.
int RunMain(int argc, char** argv, const char* node_name,
            const char* node_target, const InstanceFactory& make);

using MessageCallback = std::function<void(const void*)>;
using ParamValue = std::variant<bool, std::int64_t, double, std::string,
                                std::vector<std::string>, std::vector<double>>;

struct NodeAccess {
  static std::unique_ptr<Node> create(NodeSetup setup);

  // Called on every tick before the after_update hooks. Set before start().
  static void set_update(Node& node, std::function<void()> update);
  // Registers the tick timer (when there is an update() or an after_update
  // hook), arms it one period after now, and picks the loop's delivery: see
  // require_immediate_delivery. Called once, after the user's object is
  // built.
  static void start(Node& node);
  // Throws std::logic_error, naming the node and `what`, unless called on
  // the thread that built `node`, which runs its loop. A send or a device
  // command touches state only that thread may (the loop's dispatch context
  // and log, the transport, a device's pending request): publisher_send,
  // byte_publisher_send and the devices layer's commands call it first.
  static void require_loop_thread(const Node& node, std::string_view what);
  // Some handler must see each message as it arrives (a subscribe callback,
  // a Driver Station listener), so the loop drains every tick. Without it,
  // a node with a timer (update(), an after_update hook or every()) drains
  // only right before its own timers, which is all a node that reads latched
  // values there needs; a node with no timer drains every tick regardless.
  // Call before start().
  static void require_immediate_delivery(Node& node);
  static bool immediate_delivery(const Node& node);
  // Index of the update() currently running, or of the next one between
  // ticks. State machines stamp transitions with it.
  static std::uint64_t next_update(const Node& node);
  // [robot] command_timeout_us: how old a message may be and still be fresh.
  static std::chrono::nanoseconds freshness_timeout(const Node& node);
  // Topics subscribed with optional = true, for describe and the registry.
  static std::vector<std::string_view> optional_topics(const Node& node);
  // Topics subscribed with add_observer, for describe and the registry.
  static std::vector<std::string_view> observer_topics(const Node& node);

  // The declared graph (describe.h DeclaredGraph): every other node of the
  // session that does not read it, with each end's schema bytes. Null while
  // the launcher is still collecting it (its first --describe pass), so a
  // node registers only what it can without one; the launcher then
  // describes it again, and starts it, with the graph. Throws when the node
  // runs without one (a direct run without --graph). Call in the
  // constructor: asking marks the node as one whose topics depend on the
  // graph.
  static const DeclaredGraph* declared_graph(Node& node);
  static bool reads_graph(const Node& node);
  static const config::RobotConfig& robot_config(const Node& node);
  // The session output directory (parent of --log), or nullopt without --log.
  static std::optional<std::filesystem::path> session_dir(const Node& node);

  // A parameter of this node's subsystem table, or nullopt when absent. A
  // present key of another type throws, naming it.
  static std::optional<bool> find_bool(const Node& node, std::string_view key);
  static std::optional<std::int64_t> find_int(const Node& node,
                                              std::string_view key);
  static std::optional<double> find_double(const Node& node,
                                           std::string_view key);
  static std::optional<std::string> find_string(const Node& node,
                                                std::string_view key);
  // A TOML array: of strings, or of numbers (integers convert). An empty
  // array is valid. A non-array, or an element of another type, throws
  // naming the element's index and type.
  static std::optional<std::vector<std::string>> find_string_list(
      const Node& node, std::string_view key);
  static std::optional<std::vector<double>> find_double_list(
      const Node& node, std::string_view key);
  // Whatever type the key has (bool, integer, float, string, or an array of
  // strings or of numbers; an empty array is an empty double list).
  static std::optional<ParamValue> find_param(const Node& node,
                                              std::string_view key);
  // Throws std::invalid_argument naming the node, the key and the problem.
  [[noreturn]] static void param_error(const Node& node, std::string_view key,
                                       std::string_view problem);

  // `schema` is the topic's identity (talOS/schema/identity.h): what the
  // segment checks at attach, describe reports and the log records.
  // Without `on_message` each drain keeps only the newest message (skipped
  // ones count as dropped); with it every message arrives in sequence, at
  // most 64 per drain. subscription_age() is now minus the writer's
  // publication time for both kinds.
  static std::size_t add_subscription(Node& node, std::string_view topic,
                                      std::size_t bytes, std::size_t alignment,
                                      const schema::Identity& schema,
                                      bool optional,
                                      MessageCallback on_message);
  static const void* subscription_value(const Node& node, std::size_t index);
  static bool subscription_has_value(const Node& node, std::size_t index);
  static std::chrono::nanoseconds subscription_age(const Node& node,
                                                   std::size_t index);
  static bool subscription_fresh(const Node& node, std::size_t index,
                                 std::chrono::nanoseconds max_age);
  static bool subscription_updated(const Node& node, std::size_t index);
  // A subscription that observes a topic rather than consuming it: what a
  // framework node that reads every state topic in the declared graph opens.
  // The graph lint does not count it as a reader (names.h
  // kSourceFlagObserver). No callback: it is read at the node's tick.
  static std::size_t add_observer(Node& node, std::string_view topic,
                                  std::size_t bytes, std::size_t alignment,
                                  const schema::Identity& schema);

  static std::size_t add_publisher(Node& node, std::string_view topic,
                                   std::size_t bytes, std::size_t alignment,
                                   const schema::Identity& schema);
  static bool publisher_send(Node& node, std::size_t index, const void* data,
                             std::size_t bytes);

  static std::size_t add_byte_publisher(Node& node, std::string_view topic,
                                        std::size_t max_bytes,
                                        const BytePublishOptions& options);
  static bool byte_publisher_send(Node& node, std::size_t index,
                                  std::span<const std::byte> message);
  // Byte topics in registration order, for describe and the registry: they
  // bypass the loop, so its manifest does not list them. Their schema is
  // schema::Bytes().
  struct ByteTopicShape {
    std::string_view topic;
    std::size_t max_bytes;
    std::size_t slots;
    bool external;
  };
  static std::vector<ByteTopicShape> byte_topics(const Node& node);
  // Messages send() published, and refused, on byte topic `index`. Safe to
  // call from any thread.
  static std::uint64_t byte_topic_published(const Node& node,
                                            std::size_t index);
  static std::uint64_t byte_topic_dropped(const Node& node, std::size_t index);

  // Who drives this node's subsystem, for the registry: one entry per
  // talos::Target (talOS/devices/target.h), in the order they were opened.
  // `sources` is the Target's policy, in TargetPolicy::Sources() order.
  // Call in the constructor.
  static std::size_t add_selection(Node& node,
                                   std::vector<std::string> sources);
  // Records the lane selection `index` resolved to: an index into its
  // sources, or -1 for none. Called on every Target::get(), so it is one
  // relaxed atomic load, and a store only when the answer changed.
  static void set_selected(Node& node, std::size_t index, std::int32_t source);
  static std::size_t selection_count(const Node& node);
  static std::span<const std::string> selection_sources(const Node& node,
                                                        std::size_t index);
  // The last set_selected() value of `index`. Safe to call from any thread.
  static std::int32_t selected(const Node& node, std::size_t index);

  static std::size_t add_state_machine(Node& node, std::int64_t initial);
  static std::int64_t state_machine_state(const Node& node, std::size_t index);
  static std::int64_t state_machine_previous(const Node& node,
                                             std::size_t index);
  static void state_machine_go(Node& node, std::size_t index,
                               std::int64_t next);
  static Duration state_machine_time_in_state(const Node& node,
                                              std::size_t index);
  static bool state_machine_just_entered(const Node& node, std::size_t index);
  static void* find_extension(const Node& node, std::type_index type);
  static void add_extension(Node& node, std::type_index type,
                            std::shared_ptr<void> object);
};

}  // namespace talos::detail
