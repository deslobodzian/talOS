#pragma once

// Framework plumbing behind talos::Node: what talos::Run, testing::Sim, the
// devices layer and the C ABI (talOS/node_api) drive. Node code never uses
// it. Every operation is non-template and addresses a subscription or
// publisher by the index it was registered under, so the C ABI wraps each one
// directly.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <typeindex>
#include <variant>
#include <vector>

namespace talos {
class Node;
namespace config {
struct RobotConfig;
}  // namespace config
}  // namespace talos

namespace talos::detail {

class LoopPort;  // talOS/node/internal.h

// What a Node is built from: talos::Run, testing::Sim and the C ABI make one.
struct NodeSetup {
  std::string name;
  std::string config_path;
  std::shared_ptr<const config::RobotConfig> config;  // parsed config_path
  bool simulation = false;
  LoopPort* loop = nullptr;
};

// The user's object, type-erased so the runner and the harness need not be
// templates. The factory builds it against a Node and installs its update().
using Instance = std::shared_ptr<void>;
using InstanceFactory = std::function<Instance(Node&)>;

template <class T>
InstanceFactory MakeInstance();  // node.h

// The binary's main: flags, describe/replay/realtime, logging, the live
// registry. Built on talos::process::RunNode.
int RunMain(int argc, char** argv, const char* node_name,
            const char* node_target, const InstanceFactory& make);

using MessageCallback = std::function<void(const void*)>;
using ParamValue = std::variant<bool, std::int64_t, double, std::string>;

struct NodeAccess {
  static std::unique_ptr<Node> create(NodeSetup setup);

  // Called on every tick before the after_update hooks. Set before start().
  static void set_update(Node& node, std::function<void()> update);
  // Registers the tick timer (when there is an update() or an after_update
  // hook) and arms it one period after now. Called once, after the user's
  // object is built.
  static void start(Node& node);
  // Index of the update() currently running, or of the next one between
  // ticks. State machines stamp transitions with it.
  static std::uint64_t next_update(const Node& node);
  // [robot] command_timeout_us: how old a message may be and still be fresh.
  static std::chrono::nanoseconds freshness_timeout(const Node& node);
  // Topics subscribed with optional = true, for describe and the registry.
  static std::vector<std::string_view> optional_topics(const Node& node);

  // A parameter of this node's subsystem table, or nullopt when absent. A
  // present key of another type throws, naming it.
  static std::optional<bool> find_bool(const Node& node, std::string_view key);
  static std::optional<std::int64_t> find_int(const Node& node,
                                              std::string_view key);
  static std::optional<double> find_double(const Node& node,
                                           std::string_view key);
  static std::optional<std::string> find_string(const Node& node,
                                                std::string_view key);
  // Whatever type the key has (bool, integer, float or string).
  static std::optional<ParamValue> find_param(const Node& node,
                                              std::string_view key);
  // Throws std::invalid_argument naming the node, the key and the problem.
  [[noreturn]] static void param_error(const Node& node, std::string_view key,
                                       std::string_view problem);

  static std::size_t add_subscription(Node& node, std::string_view topic,
                                      std::size_t bytes, std::size_t alignment,
                                      bool optional,
                                      MessageCallback on_message);
  static const void* subscription_value(const Node& node, std::size_t index);
  static bool subscription_has_value(const Node& node, std::size_t index);
  static std::chrono::nanoseconds subscription_age(const Node& node,
                                                   std::size_t index);
  static bool subscription_fresh(const Node& node, std::size_t index,
                                 std::chrono::nanoseconds max_age);
  static bool subscription_updated(const Node& node, std::size_t index);

  static std::size_t add_publisher(Node& node, std::string_view topic,
                                   std::size_t bytes, std::size_t alignment);
  static bool publisher_send(Node& node, std::size_t index, const void* data,
                             std::size_t bytes);

  static void* find_extension(const Node& node, std::type_index type);
  static void add_extension(Node& node, std::type_index type,
                            std::shared_ptr<void> object);
};

}  // namespace talos::detail
