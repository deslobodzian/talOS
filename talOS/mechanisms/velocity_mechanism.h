#pragma once

// talos::mechanisms::VelocityMechanism: one motor held at a velocity on its
// controller's velocity loop. A flywheel, roller or conveyor, configured
// rather than written.
//
//   explicit Shooter(talos::Node& node)
//       : flywheel_{node, "flywheel"}, target_{node},
//         state_{node.publish<control::VelocityState>("state")} {}
//   void update() {
//     if (const control::VelocityTarget* t = target_.get()) {
//       flywheel_.set_goal(t->velocity_rps(), t->feedforward_v());
//     } else {
//       flywheel_.stop();
//     }
//     flywheel_.update();
//     state_.send(flywheel_.state());  // flywheel_.at_speed() for a feeder
//   }
//
// The motor is [motors.<name>]; the parameters are the [<name>] table
// (talOS/mechanisms/README.md). Velocities are mechanism rotations per
// second; goals clamp to the motor's own max_velocity_rps.
//
// set_goal() and stop() are this tick's request, accepted only inside the
// node's update() (std::logic_error otherwise) and consumed by this tick's
// update(); a tick without set_goal() drives nothing and the devices layer
// sends neutral (the motor coasts or brakes as its `brake` config says), and
// a request whose tick ended without update() is dropped. Call set_goal() only
// with a live talos::Target. spin_down_rps2 limits how fast a *live* goal
// winds down; losing the goal is neutral at once.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "control/control_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/mechanisms/params.h"
#include "talOS/node/node.h"

namespace talos::mechanisms {

enum class VelocityPhase : std::uint8_t {
  kStopped,     // no goal this tick (neutral)
  kSpinningUp,  // driving, not at speed
  kAtSpeed,     // see at_speed()
};

class VelocityMechanism {
 public:
  // A goal this small is "stop": never at speed.
  static constexpr double kTargetEpsilonRps = 1e-3;

  // Opens [motors.<name>] and reads [<name>]. Throws std::invalid_argument,
  // naming the key, on an invalid parameter.
  VelocityMechanism(Node& node, std::string_view name);

  // This tick's request (the last call before update() wins). A non-finite
  // value drives nothing.
  void set_goal(double velocity_rps, double feedforward_v = 0);
  void stop();

  // Applies this tick's request. Call once from the owning node's update().
  void update();

  VelocityPhase phase() const { return phase_; }
  // Entered after at_speed_entry_tick_count consecutive ticks within
  // at_speed_entry_rps of the clamped goal; kept while within the exit band,
  // max(at_speed_entry_rps, at_speed_exit_frac * |goal|); lost when the
  // error leaves it, the asked goal changes, or the goal stops. A tick
  // counts only on a recent velocity sample: Motor::velocity_age(), read
  // from its State and so known without the bridge's clock mapping, at most
  // ReadinessAgeBound (four status periods). Any other sample, an unknown age
  // included, loses at_speed and restarts the entry count, so a velocity
  // frozen by a CAN gap, which the gateway keeps valid up to
  // command_timeout_us, does not hold it.
  bool at_speed() const { return phase_ == VelocityPhase::kAtSpeed; }
  bool driving() const { return driving_; }
  bool connected() const { return connected_; }
  double velocity_rps() const { return velocity_rps_; }
  // The goal this tick after the max_velocity_rps clamp (0 when stopped),
  // and what went to the motor after the spin-down limit.
  double goal_rps() const { return goal_rps_; }
  double commanded_rps() const { return commanded_rps_; }

  // For /<sub>/state: velocity, requested = goal_rps(), current, enabled =
  // driving(), connected().
  control::VelocityState state() const;

 private:
  void readiness(double requested_rps);
  bool velocity_recent() const;

  Node* node_;
  std::string label_;  // "VelocityMechanism '<name>'", for errors
  Motor motor_;
  Duration velocity_age_bound_;  // ReadinessAgeBound of [robot] status_hz

  double max_rps_{0};
  double entry_rps_{0};
  double exit_frac_{0};
  int entry_tick_count_{1};
  double spin_down_rps2_{0};
  int slot_{0};

  bool requested_{false};
  double request_rps_{0};
  double request_ff_v_{0};
  std::uint64_t request_tick_{0};  // node.tick() of the update() it came from

  VelocityPhase phase_{VelocityPhase::kStopped};
  bool driving_{false};
  bool connected_{false};
  bool velocity_recent_{false};
  double velocity_rps_{0};
  double current_a_{0};
  double goal_rps_{0};
  double commanded_rps_{0};
  double readiness_goal_rps_{0};  // the asked goal readiness counts toward
  int entry_ticks_{0};
  double last_commanded_rps_{0};
  std::optional<Time> last_drive_;  // when the last driving tick ran
};

}  // namespace talos::mechanisms
