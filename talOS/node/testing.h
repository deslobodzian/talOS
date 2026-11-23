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
// Each node runs on its own simulated loop, with the process's delivery
// policy (see Node::subscribe) and bounded RTMS topic/drop semantics: a node
// drains right before each of its timers, and a publication on a
// subscription that wakes it (a callback with Wake::kOnPublish, a Driver
// Station listener, any subscription of a node with no timer) drains it at
// the publication's instant, as the robot's wake does; a Wake::kAtDrain
// callback's messages wait for the next drain. Time is virtual and advances
// on one chronological frontier: step() repeatedly takes the earliest pending
// event of any node's loop (a timer or such a publication), periodic hook or
// pause end, and runs every loop exactly up to it, so no node sees a
// publication from its future. At one instant the nodes run in rounds: each
// round runs them in the order they were added, then the Sim's own periodic
// hooks (SimHardware), so an earlier-added node's publications reach a
// later-added one's timers and wakes in the same round. A node woken by a
// publication made after it ran (by a later-added node, a hook or a callback)
// runs again at that instant in another round, until no node is due there; more
// than 1000 rounds at one instant throw std::runtime_error naming the nodes and
// topics still due (a zero-time publish cycle). A message the test publishes
// between steps is stamped now() and wakes its readers at the start of the
// next step(), at that instant. pause(name, duration) stops only that node's
// timers and deliveries; at its end, or at resume(name), the node runs its
// overdue timers and pending wakes at once. remove(name) stops and destroys
// a hosted node, and add() may be called after step() (including to re-add a
// removed name): a node added mid-run starts its clock at now(), schedules
// its first timer at now() + period, and receives only messages published
// after it attached (REQ-SAFE-31). Serial stepping proves causality
// and ordering, not concurrent RTMS behaviour or scheduler jitter. Hardware
// is a layer on top: talOS/devices/testing.h plays the bridge.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/time.h"
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
  // object. May be called before or after step(): a node added mid-run starts
  // its clock at now(), schedules its first periodic timer at now() + period
  // (no catch-up from 0), and receives only messages published after it
  // attached (REQ-SAFE-31). Throws std::invalid_argument if `node_name` is
  // already hosted, and std::logic_error if called inside step(). The thread
  // that adds a node is its loop thread (node.h): step the Sim from it, too.
  template <class T>
  T& add(std::string_view node_name) {
    return *static_cast<T*>(detail::SimAccess::add_instance(
        *this, node_name, detail::MakeInstance<T>()));
  }

  // Stops and removes the hosted node named `node_name`, destroying its user
  // object, Node and simulated loop; the name may be added again later.
  // Throws std::invalid_argument if `node_name` is not hosted, and
  // std::logic_error if called inside step().
  void remove(std::string_view node_name);

  // Advances virtual time, running every node's timers and deliveries.
  void step(Duration d);
  Time now() const;

  // Stop one node's deliveries and timers while every clock keeps advancing.
  // Missed periodic deadlines coalesce on resume, as after SIGSTOP/SIGCONT.
  void pause(std::string_view node_name, Duration duration);
  void resume(std::string_view node_name);

  // The session output directory the nodes this Sim hosts report through
  // Node::session_dir(). nullopt (the default) means none, as under tests and
  // --describe. Set it before adding nodes.
  void set_session_dir(std::optional<std::filesystem::path> dir);

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

// Node class T, with the wall time of each tick recorded for an update()
// timing benchmark: from just before T::update() to an after_update hook
// registered after T's constructor, so it also covers the hooks T's
// constructor registered (the devices layer's hardware-request flush).
// Add it as sim.add<TimedNode<T>>(name), then bind() where samples go: each
// tick appends one, in nanoseconds, while *recording is true.
template <class T>
class TimedNode {
 public:
  explicit TimedNode(Node& node) : inner_{node} {
    node.after_update([this] {
      const auto elapsed_ns = (event::steady_now() - tick_start_).count();
      if (recording_ != nullptr && *recording_ && samples_ns_ != nullptr) {
        samples_ns_->push_back(elapsed_ns);
      }
    });
  }

  void bind(std::vector<std::int64_t>* samples_ns, const bool* recording) {
    samples_ns_ = samples_ns;
    recording_ = recording;
  }

  void update() {
    tick_start_ = event::steady_now();
    inner_.update();
  }

 private:
  T inner_;
  event::MonotonicTime tick_start_{};
  std::vector<std::int64_t>* samples_ns_ = nullptr;
  const bool* recording_ = nullptr;
};

}  // namespace talos::testing
