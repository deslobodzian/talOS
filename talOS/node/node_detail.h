#pragma once

// Safe, non-privileged thunks behind the templates and inlines in
// talOS/node/node.h. Privileged framework hooks (NodeAccess, including
// add_hardware_request, mark_untargeted_outputs, create/start, declared_graph
// and add_observer) live in talOS/node/access.h behind the restricted
// //talOS/node:access target so robot node code cannot include or link them.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <typeindex>
#include <vector>

#include "talOS/schema/identity.h"

namespace talos {
class Node;
using Duration = std::chrono::nanoseconds;
}  // namespace talos

namespace talos::detail {

struct NodeAccess;  // talOS/node/access.h (restricted: //talOS/node:access)
struct NodeSetup;   // talOS/node/access.h (restricted: //talOS/node:access)

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
using ParamTable = std::optional<std::string_view>;

// Non-privileged thunks backing talos::Node, Subscription, Publisher,
// BytePublisher and StateMachine templates in node.h. Never opens or writes
// hardware requests (add_publisher refuses /hw/request and publisher_send
// refuses any slot opened by NodeAccess::add_hardware_request).
struct NodeDetail {
  static void set_update(Node& node, std::function<void()> update);
  static void require_immediate_delivery(Node& node, std::size_t subscription);
  static Duration freshness_timeout(const Node& node);

  static std::optional<bool> find_bool(const Node& node, std::string_view key,
                                       ParamTable subsystem = {});
  static std::optional<std::int64_t> find_int(const Node& node,
                                              std::string_view key,
                                              ParamTable subsystem = {});
  static std::optional<double> find_double(const Node& node,
                                           std::string_view key,
                                           ParamTable subsystem = {});
  static std::optional<std::string> find_string(const Node& node,
                                                std::string_view key,
                                                ParamTable subsystem = {});
  static std::optional<std::vector<std::string>> find_string_list(
      const Node& node, std::string_view key, ParamTable subsystem = {});
  static std::optional<std::vector<double>> find_double_list(
      const Node& node, std::string_view key, ParamTable subsystem = {});
  [[noreturn]] static void param_error(const Node& node, std::string_view key,
                                       std::string_view problem,
                                       ParamTable subsystem = {});

  static std::size_t add_subscription(Node& node, std::string_view topic,
                                      std::size_t bytes, std::size_t alignment,
                                      const schema::Identity& schema,
                                      bool optional,
                                      MessageCallback on_message);
  static const void* subscription_value(const Node& node, std::size_t index);
  static bool subscription_has_value(const Node& node, std::size_t index);
  static Duration subscription_age(const Node& node, std::size_t index);
  static bool subscription_fresh(const Node& node, std::size_t index,
                                 Duration max_age);
  static bool subscription_updated(const Node& node, std::size_t index);

  static std::size_t add_publisher(Node& node, std::string_view topic,
                                   std::size_t bytes, std::size_t alignment,
                                   const schema::Identity& schema);
  static bool publisher_send(Node& node, std::size_t index, const void* data,
                             std::size_t bytes);
  static bool byte_publisher_send(Node& node, std::size_t index,
                                  std::span<const std::byte> message);

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
