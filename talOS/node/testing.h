#pragma once

// talos::testing::Sim: one simulated clock hosting several nodes, for unit
// and multi-node integration tests.
//
//   auto sim = talos::testing::Sim::FromToml(R"(... robot.toml ...)");
//   auto& elevator = sim.add<Elevator>("elevator");
//   auto& superstructure = sim.add<Superstructure>("superstructure");
//   sim.publish("/superstructure/goal", superstructure::Goal{...});
//   sim.step(100ms);
//   EXPECT_TRUE(sim.latest<elevator::ElevatorStatus>("/elevator/status"));
//
// Each node runs on its own simulated loop, exactly as it would in its own
// process; the topics between them are in memory. Time is virtual and
// advances in 1 ms slices, the loops' message tick, so a message published at
// one slice reaches every subscriber by the next. Hardware is a layer on top:
// talOS/devices/testing.h plays the bridge.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/node/node.h"
#include "talOS/node/sim_access.h"

namespace talos::testing {

class Sim {
 public:
  explicit Sim(std::string config_path);  // robot.toml (manifest or single)
  static Sim FromToml(std::string_view robot_toml);  // inline single-file
  ~Sim();
  Sim(Sim&&) noexcept;
  Sim& operator=(Sim&&) noexcept;

  // Constructs a Node named `node_name` and T on it; returns the user
  // object. Add every node before the first step().
  template <class T>
  T& add(std::string_view node_name) {
    return *static_cast<T*>(detail::SimAccess::add_instance(
        *this, node_name, detail::MakeInstance<T>()));
  }

  // Advances virtual time, running every node's timers and deliveries.
  void step(Duration d);
  Time now() const;

  template <Message M>
  void publish(std::string_view topic, const M& message) {
    detail::SimAccess::publish_raw(*this, topic, &message, sizeof(M));
  }
  template <Message M>
  std::optional<M> latest(std::string_view topic) const {
    const std::uint64_t count =
        detail::SimAccess::message_count(*this, topic, sizeof(M));
    if (count == 0) return std::nullopt;
    M message{};
    detail::SimAccess::read_raw(*this, topic, sizeof(M), count - 1, &message);
    return message;
  }
  // The newest message a node sent with publish_bytes on `topic`, as sent.
  std::optional<std::vector<std::byte>> latest_bytes(
      std::string_view topic) const;

  Node& node(std::string_view node_name);

 private:
  friend struct detail::SimAccess;  // the harness's side: sim_access.h
  Sim();
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace talos::testing
