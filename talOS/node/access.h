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

#include "talOS/node/node_detail.h"
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
struct ParamRead;
}  // namespace introspect
namespace process {
enum class LoopTier : std::uint8_t;  // talOS/process/realtime_outcome.h
}  // namespace process
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
  bool describing = false;  // --describe: Node::describing()
  // --targeted-outputs: set by the launcher when --describe declared a
  // hardware_request endpoint without untargeted_outputs (REQ-SAFE-42), so
  // mark_untargeted_outputs is refused at construction.
  bool require_targeted_outputs = false;
  LoopPort* loop = nullptr;
  GraphSource graph;  // unset: no graph (null)
  // The session output directory the launcher started this node under (the
  // parent of --log), or nullopt without --log (tests) and in an offline run
  // (--describe, --replay, --replay-inputs): Node::session_dir().
  std::optional<std::filesystem::path> session_dir;
};

using ParamValue = std::variant<bool, std::int64_t, double, std::string,
                                std::vector<std::string>, std::vector<double>>;

// NodeDetail's members (node_detail.h: params, subscriptions, publishers,
// state machines, extensions) are NodeAccess's too; it adds the privileged
// hooks below. Its publisher_send is the devices flush's: unlike
// NodeDetail::publisher_send it accepts the hardware request slot.
struct NodeAccess : NodeDetail {
  static std::unique_ptr<Node> create(NodeSetup setup);

  // node_api's talos_node_set_update: the update() just set is a callback
  // through the C ABI, a Python node's, so its ticks run the interpreter.
  // --describe reports it (introspect::Description::abi_update).
  static void mark_abi_update(Node& node);
  static bool abi_update(const Node& node);
  // Registers the tick timer (when there is an update() or an after_update
  // hook), arms it one period after now, and picks the loop's delivery: see
  // require_immediate_delivery. When `!node.describing()`, also checks that
  // an explicit Driver Station reader publishes no teleop-policy target lane
  // without OperatorArming (REQ-SAFE-14). Called once, after the user's object
  // is built.
  static void start(Node& node);
  // Throws std::logic_error, naming the node and `what`, unless called on
  // the thread that built `node`, which runs its loop. A send or a device
  // command touches state only that thread may (the loop's dispatch context
  // and log, the transport, a device's pending request): publisher_send,
  // byte_publisher_send and the devices layer's commands call it first.
  static void require_loop_thread(const Node& node, std::string_view what);
  // Whether the node's update() is running now: false in the constructor,
  // in every() timers, message callbacks and after_update hooks. The devices
  // layer refuses an output command outside it, because its flush after
  // each update() sends neutral for every output that update() did not set.
  // Read it only on the loop thread (after require_loop_thread).
  static bool in_update(const Node& node);
  // Index of the update() currently running (1 for the first), or between
  // ticks of the last one to run, which the after_update hooks read; an
  // update() that threw keeps its index, so the next one never shares it.
  // State machines stamp transitions with it, a Target its READ log entry
  // and its live target, and the devices layer each output it sets.
  static std::uint64_t next_update(const Node& node);
  // Topics subscribed with optional = true, for describe and the registry.
  static std::vector<std::string_view> optional_topics(const Node& node);
  // Topics subscribed with add_observer, for describe and the registry.
  static std::vector<std::string_view> observer_topics(const Node& node);
  // Target-lane topics published with operator_arming = true
  // (add_target_lane_publisher), for describe and the registry (REQ-SAFE-14).
  static std::vector<std::string_view> armed_target_lane_topics(
      const Node& node);
  // Driver Station topics subscribed solely by internal Target/TargetLane/
  // OperatorArming plumbing (empty whenever the node explicitly reads the
  // Driver Station), for describe and the registry (REQ-SAFE-14).
  static std::vector<std::string_view> internal_driver_station_topics(
      const Node& node);

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
  // Every (subsystem, key) the constructor read with param or param_of, in
  // first-read order: what --describe reports as the node's parameters.
  static const std::vector<introspect::ParamRead>& params_read(
      const Node& node);
  static const config::RobotConfig& robot_config(const Node& node);
  // The scheduling tier talos::Run gives the node's loop thread: control when
  // its own subsystem's devices own an actuator
  // (hardware::Devices::owns_actuators) or it has registered a publisher on a
  // target lane, otherwise observer. Read after construction.
  static process::LoopTier loop_tier(const Node& node);
  // The session output directory (parent of --log), or nullopt without --log.
  static std::optional<std::filesystem::path> session_dir(const Node& node);

  using ParamTable = detail::ParamTable;  // node_detail.h
  // A parameter, as NodeDetail::find_bool and the others read one, of
  // whatever type the key has (bool, integer, float, string, or an array of
  // strings or of numbers; an empty array is an empty double list).
  static std::optional<ParamValue> find_param(const Node& node,
                                              std::string_view key,
                                              ParamTable subsystem = {});

  // Logs, in the dispatch log of the handler running now, that it acted on
  // subscription `index`'s latched message: an EventKind::READ record with
  // the watcher, now, that message's publish time and its RTMS sequence.
  // Nothing without a value or outside a dispatch. Target::get() calls it for
  // the lane an update() selects.
  static void record_read(Node& node, std::size_t index);
  // A subscription that observes a topic rather than consuming it: what a
  // framework node that reads every state topic in the declared graph opens.
  // The graph lint does not count it as a reader (names.h
  // kSourceFlagObserver). No callback: it is read at the node's tick.
  static std::size_t add_observer(Node& node, std::string_view topic,
                                  std::size_t bytes, std::size_t alignment,
                                  const schema::Identity& schema);
  // Internal Driver Station subscription opened by DriverStationLink for
  // Target/TargetLane/OperatorArming plumbing: does not mark the node as an
  // explicit Driver Station reader, and reports
  // naming::kSourceFlagInternalDriverStation unless mark_driver_station_reader
  // or a direct /hw/state/driver_station subscription marks it explicit.
  static std::size_t add_internal_driver_station_subscription(
      Node& node, std::string_view topic, std::size_t bytes,
      std::size_t alignment, const schema::Identity& schema,
      MessageCallback on_message);
  // Marks this node as an explicit Driver Station reader (public
  // talos::DriverStation construction). After start() when not describing,
  // refuses if the node already publishes any unarmed teleop-policy target
  // lane before mutating state (REQ-SAFE-14).
  static void mark_driver_station_reader(Node& node);
  // Target-lane publisher opened by TargetAccess::open_lane: marks the topic
  // with naming::kSourceFlagOperatorArming when `operator_arming` is true, and
  // enforces REQ-SAFE-14 if opened after start() on an explicit DS reader.
  static std::size_t add_target_lane_publisher(Node& node,
                                               std::string_view topic,
                                               std::size_t bytes,
                                               std::size_t alignment,
                                               const schema::Identity& schema,
                                               bool operator_arming);

  // The devices layer's flush: this node's hardware request,
  // naming::HardwareRequestTopic(name), never remapped, declared to describe
  // and the registry with naming::kSourceFlagHardwareRequest (the graph lint
  // refuses a request writer without it). Sent with publisher_send. Once per
  // node; a second call throws std::logic_error.
  static std::size_t add_hardware_request(Node& node, std::size_t bytes,
                                          std::size_t alignment,
                                          const schema::Identity& schema);
  // The topic add_hardware_request registered, or empty: for describe and
  // the registry.
  static std::string_view hardware_request(const Node& node);
  // talos::AllowUntargetedOutputs (talOS/devices/devices.h): the devices
  // flush gates this node's outputs on robot_enabled alone, and describe and
  // the registry mark its hardware request with
  // naming::kSourceFlagUntargetedOutputs. Constructor only, and refused when
  // the node was started with --targeted-outputs (its --describe probe
  // declared a hardware request without untargeted_outputs): throws
  // std::logic_error naming the node.
  static void mark_untargeted_outputs(Node& node);
  static bool untargeted_outputs(const Node& node);
  // Whether the devices flush is holding back outputs this node set: its
  // last flush ran enabled, not opted out, with no Target returning a target
  // in that update(), and that update() set an output non-neutral
  // (REQ-SAFE-41). The flush records it on every tick, so it is one relaxed
  // load, and a store only when the answer changed; the registry's row
  // carries it as introspect::kFlagOutputsHeld. outputs_held() is safe to
  // call from any thread.
  static void set_outputs_held(Node& node, bool held);
  static bool outputs_held(const Node& node);
  static bool publisher_send(Node& node, std::size_t index, const void* data,
                             std::size_t bytes);

  static std::size_t add_byte_publisher(Node& node, std::string_view topic,
                                        std::size_t max_bytes,
                                        const BytePublishOptions& options);
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
  // Wake notifications byte topic `index`'s writer could not send for a
  // reason other than a gone receiver (ByteTopic::wake_failures). Safe to
  // call from any thread.
  static std::uint64_t byte_topic_wake_failures(const Node& node,
                                                std::size_t index);

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

  // Hook for node_api to forward HostCppNode<T> errors into talos_last_error().
  using HostErrorCallback = void (*)(const char* message) noexcept;
  static void set_host_error_callback(HostErrorCallback callback) noexcept;
};

}  // namespace talos::detail
