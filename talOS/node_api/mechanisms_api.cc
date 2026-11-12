// The mechanisms section of node_api.h: talOS/mechanisms' classes, held per
// node and addressed by index.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "control/control_generated.h"
#include "talOS/mechanisms/position_mechanism.h"
#include "talOS/mechanisms/roller_with_sensor.h"
#include "talOS/mechanisms/velocity_mechanism.h"
#include "talOS/node/node.h"
#include "talOS/node_api/internal.h"
#include "talOS/node_api/node_api.h"

namespace talos::node_api {
namespace {

using mechanisms::PositionFault;
using mechanisms::PositionMechanism;
using mechanisms::PositionPhase;
using mechanisms::RollerCommand;
using mechanisms::RollerPhase;
using mechanisms::RollerWithSensor;
using mechanisms::VelocityMechanism;
using mechanisms::VelocityPhase;

// The enum values cross the ABI as these integers; Python takes them from
// _abi.py, which api_parity_test compares with node_api.h.
template <class E>
constexpr bool Same(std::int32_t abi, E value) {
  return abi == static_cast<std::int32_t>(value);
}
static_assert(Same(TALOS_POSITION_PHASE_UNHOMED, PositionPhase::kUnhomed) &&
              Same(TALOS_POSITION_PHASE_HOMING, PositionPhase::kHoming) &&
              Same(TALOS_POSITION_PHASE_READY, PositionPhase::kReady) &&
              Same(TALOS_POSITION_PHASE_MOVING, PositionPhase::kMoving) &&
              Same(TALOS_POSITION_PHASE_AT_GOAL, PositionPhase::kAtGoal) &&
              Same(TALOS_POSITION_PHASE_FAULTED, PositionPhase::kFaulted));
static_assert(
    Same(TALOS_POSITION_FAULT_NONE, PositionFault::kNone) &&
    Same(TALOS_POSITION_FAULT_HOMING_TIMEOUT, PositionFault::kHomingTimeout) &&
    Same(TALOS_POSITION_FAULT_HOMING_STALL, PositionFault::kHomingStall) &&
    Same(TALOS_POSITION_FAULT_SWITCH_STUCK, PositionFault::kSwitchStuck) &&
    Same(TALOS_POSITION_FAULT_STALL, PositionFault::kStall) &&
    Same(TALOS_POSITION_FAULT_MOVE_TIMEOUT, PositionFault::kMoveTimeout));
static_assert(Same(TALOS_VELOCITY_PHASE_STOPPED, VelocityPhase::kStopped) &&
              Same(TALOS_VELOCITY_PHASE_SPINNING_UP,
                   VelocityPhase::kSpinningUp) &&
              Same(TALOS_VELOCITY_PHASE_AT_SPEED, VelocityPhase::kAtSpeed));
static_assert(Same(TALOS_ROLLER_COMMAND_IDLE, RollerCommand::kIdle) &&
              Same(TALOS_ROLLER_COMMAND_INTAKE, RollerCommand::kIntake) &&
              Same(TALOS_ROLLER_COMMAND_EJECT, RollerCommand::kEject) &&
              Same(TALOS_ROLLER_COMMAND_FEED, RollerCommand::kFeed));
static_assert(Same(TALOS_ROLLER_PHASE_IDLE, RollerPhase::kIdle) &&
              Same(TALOS_ROLLER_PHASE_INTAKING, RollerPhase::kIntaking) &&
              Same(TALOS_ROLLER_PHASE_HOLDING, RollerPhase::kHolding) &&
              Same(TALOS_ROLLER_PHASE_EJECTING, RollerPhase::kEjecting) &&
              Same(TALOS_ROLLER_PHASE_FEEDING, RollerPhase::kFeeding));

// A node's mechanisms, destroyed with it (Node::extension).
struct Mechanisms {
  explicit Mechanisms(Node& /*node*/) {}
  std::vector<std::unique_ptr<PositionMechanism>> position;
  std::vector<std::unique_ptr<VelocityMechanism>> velocity;
  std::vector<std::unique_ptr<RollerWithSensor>> roller;
};

template <class M>
std::vector<std::unique_ptr<M>>& All(Mechanisms& all);
template <>
std::vector<std::unique_ptr<PositionMechanism>>& All(Mechanisms& all) {
  return all.position;
}
template <>
std::vector<std::unique_ptr<VelocityMechanism>>& All(Mechanisms& all) {
  return all.velocity;
}
template <>
std::vector<std::unique_ptr<RollerWithSensor>>& All(Mechanisms& all) {
  return all.roller;
}

template <class M>
std::int64_t Open(TalosNode* node, const char* name) {
  Node& n = AsNode(node);
  // Built before the store is attached, so the devices layer it opens is
  // older and outlives it.
  auto mechanism = std::make_unique<M>(n, NonNull(name, "mechanism name"));
  auto& list = All<M>(n.extension<Mechanisms>());
  list.push_back(std::move(mechanism));
  return static_cast<std::int64_t>(list.size() - 1);
}

template <class M>
M& Get(TalosNode* node, std::int64_t index) {
  auto& list = All<M>(AsNode(node).extension<Mechanisms>());
  if (index < 0 || static_cast<std::size_t>(index) >= list.size()) {
    throw std::out_of_range("no mechanism " + std::to_string(index) +
                            " of that kind on this node");
  }
  return *list[static_cast<std::size_t>(index)];
}

template <class M, class F>
std::int32_t Call(TalosNode* node, std::int64_t index, F&& body) {
  return Guard<std::int32_t>(TALOS_ERR, [&] {
    std::forward<F>(body)(Get<M>(node, index));
    return TALOS_OK;
  });
}

template <class Out>
Out& Clear(Out* out) {
  if (out == nullptr) throw std::invalid_argument("null state");
  *out = Out{};
  return *out;
}

}  // namespace
}  // namespace talos::node_api

using namespace talos::node_api;

extern "C" {

// --- PositionMechanism -------------------------------------------------------

int64_t talos_position_mechanism_open(TalosNode* node, const char* name) {
  return Guard<int64_t>(-1, [&] { return Open<PositionMechanism>(node, name); });
}

int32_t talos_position_mechanism_set_goal(TalosNode* node, int64_t index,
                                          double position_rot,
                                          double feedforward_v) {
  return Call<PositionMechanism>(node, index, [&](PositionMechanism& m) {
    m.set_goal(position_rot, feedforward_v);
  });
}

int32_t talos_position_mechanism_set_goal_units(TalosNode* node, int64_t index,
                                                double position_units,
                                                double feedforward_v) {
  return Call<PositionMechanism>(node, index, [&](PositionMechanism& m) {
    m.set_goal_units(position_units, feedforward_v);
  });
}

int32_t talos_position_mechanism_hold(TalosNode* node, int64_t index,
                                      double feedforward_v) {
  return Call<PositionMechanism>(
      node, index, [&](PositionMechanism& m) { m.hold(feedforward_v); });
}

int32_t talos_position_mechanism_stop(TalosNode* node, int64_t index) {
  return Call<PositionMechanism>(node, index,
                                 [](PositionMechanism& m) { m.stop(); });
}

int32_t talos_position_mechanism_update(TalosNode* node, int64_t index) {
  return Call<PositionMechanism>(node, index,
                                 [](PositionMechanism& m) { m.update(); });
}

int32_t talos_position_mechanism_clear_fault(TalosNode* node, int64_t index) {
  return Call<PositionMechanism>(
      node, index, [](PositionMechanism& m) { m.clear_fault(); });
}

int32_t talos_position_mechanism_state(TalosNode* node, int64_t index,
                                       TalosPositionMechanismState* out) {
  return Call<PositionMechanism>(node, index, [&](PositionMechanism& m) {
    TalosPositionMechanismState& o = Clear(out);
    o.phase = static_cast<int32_t>(m.phase());
    o.fault = static_cast<int32_t>(m.fault());
    o.homed = m.homed();
    o.at_goal = m.at_goal();
    o.driving = m.driving();
    o.connected = m.connected();
    o.position_rot = m.position_rot();
    o.position_units = m.position_units();
    o.velocity_rps = m.velocity_rps();
    o.goal_rot = m.goal_rot();
    o.gravity_v = m.gravity_v();
    o.tolerance_rot = m.tolerance_rot();
  });
}

// --- VelocityMechanism -------------------------------------------------------

int64_t talos_velocity_mechanism_open(TalosNode* node, const char* name) {
  return Guard<int64_t>(-1, [&] { return Open<VelocityMechanism>(node, name); });
}

int32_t talos_velocity_mechanism_set_goal(TalosNode* node, int64_t index,
                                          double velocity_rps,
                                          double feedforward_v) {
  return Call<VelocityMechanism>(node, index, [&](VelocityMechanism& m) {
    m.set_goal(velocity_rps, feedforward_v);
  });
}

int32_t talos_velocity_mechanism_stop(TalosNode* node, int64_t index) {
  return Call<VelocityMechanism>(node, index,
                                 [](VelocityMechanism& m) { m.stop(); });
}

int32_t talos_velocity_mechanism_update(TalosNode* node, int64_t index) {
  return Call<VelocityMechanism>(node, index,
                                 [](VelocityMechanism& m) { m.update(); });
}

int32_t talos_velocity_mechanism_state(TalosNode* node, int64_t index,
                                       TalosVelocityMechanismState* out) {
  return Call<VelocityMechanism>(node, index, [&](VelocityMechanism& m) {
    TalosVelocityMechanismState& o = Clear(out);
    o.phase = static_cast<int32_t>(m.phase());
    o.at_speed = m.at_speed();
    o.driving = m.driving();
    o.connected = m.connected();
    o.velocity_rps = m.velocity_rps();
    o.goal_rps = m.goal_rps();
    o.commanded_rps = m.commanded_rps();
    o.current_a = m.state().current_a();
  });
}

// --- RollerWithSensor --------------------------------------------------------

int64_t talos_roller_with_sensor_open(TalosNode* node, const char* name) {
  return Guard<int64_t>(-1, [&] { return Open<RollerWithSensor>(node, name); });
}

int32_t talos_roller_with_sensor_set_goal(TalosNode* node, int64_t index,
                                          int32_t command) {
  return Call<RollerWithSensor>(node, index, [&](RollerWithSensor& m) {
    if (command < TALOS_ROLLER_COMMAND_IDLE ||
        command > TALOS_ROLLER_COMMAND_FEED) {
      throw std::invalid_argument("unknown roller command " +
                                  std::to_string(command));
    }
    m.set_goal(static_cast<RollerCommand>(command));
  });
}

int32_t talos_roller_with_sensor_stop(TalosNode* node, int64_t index) {
  return Call<RollerWithSensor>(node, index,
                                [](RollerWithSensor& m) { m.stop(); });
}

int32_t talos_roller_with_sensor_update(TalosNode* node, int64_t index) {
  return Call<RollerWithSensor>(node, index,
                                [](RollerWithSensor& m) { m.update(); });
}

int32_t talos_roller_with_sensor_state(TalosNode* node, int64_t index,
                                       TalosRollerWithSensorState* out) {
  return Call<RollerWithSensor>(node, index, [&](RollerWithSensor& m) {
    TalosRollerWithSensorState& o = Clear(out);
    o.phase = static_cast<int32_t>(m.phase());
    o.has_piece = m.has_piece();
    o.sensor_connected = m.sensor_connected();
    o.driving = m.driving();
    o.output_v = m.output_v();
  });
}

}  // extern "C"
