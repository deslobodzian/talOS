#pragma once

// Harness plumbing behind talos::testing::Sim: what layers that play the far
// end of a topic (talOS/devices/testing.h) and the C ABI drive. Tests never
// use it.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>

#include "talOS/node/access.h"

namespace talos::testing {
class Sim;
}  // namespace talos::testing

namespace talos::detail {

struct SimAccess {
  // Constructs a Node named `node_name` and the user's object on it; returns
  // that object.
  static void* add_instance(testing::Sim& sim, std::string_view node_name,
                            const InstanceFactory& make);
  static const std::string& config_path(const testing::Sim& sim);
  // Runs `fn` now and then every `period` of virtual time, before the nodes
  // run at that instant. Stop by letting `fn` return false.
  static void every(testing::Sim& sim, std::chrono::nanoseconds period,
                    std::function<bool()> fn);
  static void publish_raw(testing::Sim& sim, std::string_view topic,
                          const void* data, std::size_t bytes);
  // Messages ever published on `topic` (sequence numbers 0..count-1).
  static std::uint64_t message_count(const testing::Sim& sim,
                                     std::string_view topic, std::size_t bytes);
  static void read_raw(const testing::Sim& sim, std::string_view topic,
                       std::size_t bytes, std::uint64_t sequence, void* out);
};

}  // namespace talos::detail
