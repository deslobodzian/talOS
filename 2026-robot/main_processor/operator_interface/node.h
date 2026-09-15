#pragma once

#include <cstdint>

#include "talos_nodes/driver_station/driver_station_message_generated.h"
#include "talos_nodes/driver_station/packet.h"
#include "talos_nodes/intake/intake_message_generated.h"
#include "talos_nodes/intake/packet.h"
#include "talos_nodes/operator_interface/node.h"
#include "talos_nodes/shooter/packet.h"
#include "talos_nodes/shooter/shooter_message_generated.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/handles.h"
#include "talOS/process/node_main.h"

// Year-local teleop mechanism bindings for the 2026 game. Composes the
// reusable platform chassis core (talos-operator_interface) for stick mapping
// and adds shooter/intake senders, button masks, and setpoints that change
// each season. The platform core owns the chassis enable/teleop/connected gate
// and its own falling-edge Release; this wrapper owns a parallel gate for the
// mechanism targets so a disabled/teleop-exit still tells the shooter and
// intake to stop.
namespace talos::oi::year2026 {

struct YearOperatorInterfaceConfig {
  OperatorInterfaceConfig chassis{};
  double shooter_target_rps{60.0};
  double intake_target_rps{30.0};
  // Bit 0 is button 1, bit 1 is button 2, matching Driver Station numbering.
  uint32_t shoot_button_mask{1u << 0};
  uint32_t intake_button_mask{1u << 1};
};

template <typename Loop>
class YearOperatorInterfaceNode {
 public:
  explicit YearOperatorInterfaceNode(Loop& loop,
                                     YearOperatorInterfaceConfig config = {})
      : config_{config},
        chassis_{loop, config_.chassis},
        shooter_target_{event::make_sender<talos::shooter::ShooterTarget>(
            loop, talos::shooter::kTeleopShooterTargetTopic)},
        intake_target_{event::make_sender<talos::intake::IntakeTarget>(
            loop, talos::intake::kTeleopIntakeTargetTopic)} {
    event::watch<driver_station::DriverStationState,
                 &YearOperatorInterfaceNode::OnDriverStation>(
        loop, driver_station::kDsStateTopic, this);
  }

  void Start(event::MonotonicTime = {}) {}

  bool commanding() const { return commanding_; }

 private:
  void Release(const event::Context& ctx) {
    if (!commanding_) return;
    commanding_ = false;
    shooter_target_.send(
        talos::shooter::ShooterTarget{0.0, ctx.now.nanos(), false});
    intake_target_.send(talos::intake::IntakeTarget{
        static_cast<uint64_t>(ctx.now.nanos()), false, 0.0f});
  }

  void OnDriverStation(const event::Context& ctx,
                       const driver_station::DriverStationState& state) {
    const uint32_t flags = state.fms().flags();
    const bool enabled = (flags & driver_station::kEnabled) != 0;
    const bool teleop = (flags & driver_station::kTeleop) != 0;
    const auto& stick = state.stick0();
    if (!enabled || !teleop || !stick.connected()) {
      Release(ctx);
      return;
    }

    const bool shoot = (stick.buttons() & config_.shoot_button_mask) != 0;
    shooter_target_.send(talos::shooter::ShooterTarget{
        shoot ? config_.shooter_target_rps : 0.0, ctx.now.nanos(), shoot});

    const bool intake =
        (stick.buttons() & config_.intake_button_mask) != 0;
    intake_target_.send(talos::intake::IntakeTarget{
        static_cast<uint64_t>(ctx.now.nanos()), intake,
        intake ? static_cast<float>(config_.intake_target_rps) : 0.0f});
    commanding_ = true;
  }

  YearOperatorInterfaceConfig config_;
  OperatorInterfaceNode<Loop> chassis_;
  event::Sender<Loop, talos::shooter::ShooterTarget> shooter_target_;
  event::Sender<Loop, talos::intake::IntakeTarget> intake_target_;
  bool commanding_{false};
};

// Factory for talos_cc_node: chassis tuning plus year mechanism bindings come
// from the operator_interface subsystem table, with compiled defaults.
inline auto CreateNode(auto& loop, const talos::process::NodeFlags& flags) {
  const auto robot_config = talos::config::ParseRobotConfig(flags.config_path);
  YearOperatorInterfaceConfig cfg{};
  if (const auto* tbl = robot_config.GetSubsystemTable("operator_interface")) {
    cfg.chassis.max_linear_mps =
        (*tbl)["max_linear_mps"].value_or(cfg.chassis.max_linear_mps);
    cfg.chassis.max_angular_radps = (*tbl)["max_angular_radps"].value_or(
        cfg.chassis.max_angular_radps);
    cfg.chassis.deadband =
        (*tbl)["deadband"].value_or(cfg.chassis.deadband);
    cfg.chassis.field_oriented =
        (*tbl)["field_oriented"].value_or(cfg.chassis.field_oriented);
    cfg.shooter_target_rps =
        (*tbl)["shooter_target_rps"].value_or(cfg.shooter_target_rps);
    cfg.shoot_button_mask =
        (*tbl)["shoot_button_mask"].value_or(cfg.shoot_button_mask);
    cfg.intake_target_rps =
        (*tbl)["intake_target_rps"].value_or(cfg.intake_target_rps);
    cfg.intake_button_mask =
        (*tbl)["intake_button_mask"].value_or(cfg.intake_button_mask);
  }
  return YearOperatorInterfaceNode{loop, cfg};
}

}  // namespace talos::oi::year2026
