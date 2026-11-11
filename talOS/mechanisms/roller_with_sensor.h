#pragma once

// talos::mechanisms::RollerWithSensor: a roller on voltage plus a beam break
// (or any digital piece sensor). An intake or indexer, configured rather than
// written.
//
//   explicit Intake(talos::Node& node) : roller_{node, "roller"}, goal_{node}
//   {} void update() {
//     if (const IntakeGoal* g = goal_.get()) {
//       roller_.set_goal(ToRollerCommand(g->command()));
//     } else {
//       roller_.stop();
//     }
//     roller_.update();
//     // roller_.phase(), roller_.has_piece() -> /intake/state
//   }
//
//   phase      roller    leaves on
//   kIdle      neutral   kIntake -> kIntaking; with idle_runs, a newly asked
//                        kEject or kFeed -> kEjecting / kFeeding: one that
//                        differs from the last live request, so holding the
//                        command, or asking it again after a stale spell,
//                        release or disable, runs it once
//   kIntaking  intake_v  piece -> kHolding; kIdle -> kIdle; kEject, kFeed
//   kHolding   hold_v    kEject, kFeed; piece gone -> kIdle
//   kEjecting  eject_v   after eject_us -> kIdle
//   kFeeding   feed_v    after feed_us -> kIdle
//
// The motor is [motors.<name>]; the parameters, including the sensor's
// device name, are the [<name>] table (talOS/mechanisms/README.md).
//
// set_goal() and stop() are this tick's request, accepted only inside the
// node's update() (std::logic_error otherwise) and consumed by this tick's
// update(); a request whose tick ended without update() is dropped. With no
// live request (stop(), a stale or released lane) the roller is neutral and
// the state machine runs as if asked for kIdle: it still tracks the sensor
// and finishes a timed run, without driving. Losing enable (robot_enabled
// false: a Driver Station disable, a stale link) also cancels intaking and
// any timed run, back to kIdle; a held piece stays kHolding. The roller
// reads no motor feedback: a motor whose feedback is invalid is
// neutralized by the gateway, not here.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "talOS/devices/devices.h"
#include "talOS/mechanisms/params.h"
#include "talOS/node/node.h"

namespace talos::mechanisms {

enum class RollerCommand : std::uint8_t { kIdle, kIntake, kEject, kFeed };

enum class RollerPhase : std::uint8_t {
  kIdle,      // neutral
  kIntaking,  // intake_v until the sensor sees a piece
  kHolding,   // hold_v (neutral without one) while the piece stays
  kEjecting,  // eject_v for eject_us
  kFeeding,   // feed_v for feed_us
};

class RollerWithSensor {
 public:
  // Opens [motors.<name>] and the digital input named by <name>.sensor, and
  // reads [<name>]. Throws std::invalid_argument on a missing or invalid
  // parameter.
  RollerWithSensor(Node& node, std::string_view name);

  // This tick's request (the last call before update() wins).
  void set_goal(RollerCommand command);
  void stop();

  // Applies this tick's request. Call once from the owning node's update().
  void update();

  RollerPhase phase() const { return phase_; }
  // The sensor (after sensor_inverted and sensor_debounce_us), as last
  // latched: the devices layer keeps the last reading while the link is
  // stale.
  bool has_piece() const { return has_piece_; }
  bool sensor_connected() const { return sensor_connected_; }
  bool driving() const { return driving_; }
  // The voltage sent this tick (0 when not driving).
  double output_v() const { return output_v_; }

 private:
  Node* node_;
  std::string label_;  // "RollerWithSensor '<name>'", for errors
  Motor motor_;
  DigitalInput sensor_;

  bool sensor_inverted_{false};
  bool idle_runs_{false};
  Debounce piece_debounce_;
  double intake_v_{0};
  std::optional<double> hold_v_;
  double eject_v_{0};
  double feed_v_{0};
  Duration eject_time_{};
  Duration feed_time_{};

  bool requested_{false};
  RollerCommand request_{RollerCommand::kIdle};
  std::uint64_t request_tick_{0};  // node.tick() of the update() it came from
  RollerCommand last_command_{RollerCommand::kIdle};

  RollerPhase phase_{RollerPhase::kIdle};
  Time phase_since_{};
  bool has_piece_{false};
  bool sensor_connected_{false};
  bool driving_{false};
  double output_v_{0};
};

}  // namespace talos::mechanisms
