#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/handles.h"
#include "talOS/process/node_main.h"
#include "talos_nodes/intake/intake_message_generated.h"
#include "talos_nodes/intake/packet.h"
#include "talos_nodes/shooter/packet.h"
#include "talos_nodes/shooter/shooter_message_generated.h"
#include "talos_nodes/telemetry/node.h"

// Year-local telemetry mechanism bindings for the 2026 game. Composes the
// reusable platform core (talos-telemetry) for pose, chassis, and driver
// station channels and adds the year's mechanism channels, which change each
// season. The platform core owns the frame, the timer, and the Studio feed;
// this wrapper owns one watcher per year mechanism topic and forwards display
// values through the core's SetMechanismChannels, so a disabled robot or a
// missing mechanism publisher simply leaves those channels out of the frame.
namespace talos::telemetry::year2026 {

template <typename Loop>
class YearTelemetryNode {
 public:
  explicit YearTelemetryNode(Loop& loop, TelemetryConfig config = {})
      : stale_timeout_us_{config.stale_timeout_us}, core_{loop, config} {
    event::watch<talos::shooter::ShooterState, &YearTelemetryNode::OnShooter>(
        loop, talos::shooter::kShooterStateTopic, this);
    event::watch<talos::intake::IntakeState, &YearTelemetryNode::OnIntake>(
        loop, talos::intake::kIntakeStateTopic, this);
  }

  void Start(event::MonotonicTime first) { core_.Start(first); }

  uint64_t frames_published() const { return core_.frames_published(); }
  uint64_t frames_dropped() const { return core_.frames_dropped(); }
  uint64_t frames_rejected() const { return core_.frames_rejected(); }

 private:
  void OnShooter(const event::Context& context,
                 const talos::shooter::ShooterState& state) {
    shooter_ = state;
    have_shooter_ = true;
    last_shooter_ = context.now;
    RefreshChannels(context.now);
  }

  void OnIntake(const event::Context& context,
                const talos::intake::IntakeState& state) {
    intake_ = state;
    have_intake_ = true;
    last_intake_ = context.now;
    RefreshChannels(context.now);
  }

  // Human-readable intake inhibit reason for the `intake/inhibit` unit.
  // The numeric value stays the wire code (talos::intake::kInhibit* in
  // talos_nodes/intake/packet.h, append-only); the unit names it so Studio
  // -- which renders each channel's unit next to its value -- reads
  // "TARGET_STALE" instead of a bare 3.
  static const char* InhibitReason(uint8_t inhibit) {
    switch (inhibit) {
      case talos::intake::kInhibitDrive:
        return "DRIVE";
      case talos::intake::kInhibitGateway:
        return "GATEWAY_NOT_READY";
      case talos::intake::kInhibitTargetDisabled:
        return "TARGET_DISABLED";
      case talos::intake::kInhibitTargetStale:
        return "TARGET_STALE";
      case talos::intake::kInhibitFeedbackInvalid:
        return "FEEDBACK_INVALID";
      case talos::intake::kInhibitDemandNonfinite:
        return "DEMAND_NONFINITE";
      default:
        return "UNKNOWN";
    }
  }

  // One combined channel list: each mechanism contributes only after its
  // first state arrives, so a missing publisher leaves its rows out rather
  // than holding the whole frame. Each mechanism's rows are also dropped
  // once that mechanism goes quiet past the staleness horizon, with a
  // `<mechanism>/stale` marker left behind: re-evaluated on every mechanism
  // message, so a dead shooter beside a live intake shows stale shooter
  // rows instead of frozen shooter numbers, and a fully quiet wrapper ages
  // out as one group in the core. Intake rows are the node's Studio view:
  // requested speed (the demand acted on), measured speed (wheel feedback),
  // and the inhibit code naming why the wheel is not driving.
  void RefreshChannels(event::MonotonicTime now) {
    const event::Duration timeout{std::chrono::microseconds{stale_timeout_us_}};
    const auto fresh = [&](event::MonotonicTime last) {
      return stale_timeout_us_ == 0 || (now - last) <= timeout;
    };
    std::vector<MechanismChannel> channels;
    if (have_shooter_) {
      const bool ok = fresh(last_shooter_);
      if (ok) {
        channels.push_back(
            {"shooter/flywheel", shooter_.flywheel_velocity_rps(), "rev/s"});
        channels.push_back(
            {"shooter/beam_broken", shooter_.beam_broken() ? 1.0 : 0.0, ""});
      }
      channels.push_back({"shooter/stale", ok ? 0.0 : 1.0, ""});
    }
    if (have_intake_) {
      const bool ok = fresh(last_intake_);
      if (ok) {
        channels.push_back(
            {"intake/requested", intake_.requested_velocity_rps(), "rev/s"});
        channels.push_back(
            {"intake/measured", intake_.roller_velocity_rps(), "rev/s"});
        channels.push_back({"intake/inhibit",
                            static_cast<double>(intake_.inhibit()),
                            InhibitReason(intake_.inhibit())});
      }
      channels.push_back({"intake/stale", ok ? 0.0 : 1.0, ""});
    }
    core_.SetMechanismChannels(std::move(channels));
  }

  talos::shooter::ShooterState shooter_{};
  talos::intake::IntakeState intake_{};
  bool have_shooter_ = false;
  bool have_intake_ = false;
  event::MonotonicTime last_shooter_{}, last_intake_{};
  uint64_t stale_timeout_us_;

  TelemetryNode<Loop> core_;
};

// Factory for the year node: period and topic come from the telemetry
// subsystem table, with the core's compiled defaults as fallback. Shooter
// channels need no tuning.
inline auto CreateNode(auto& loop, const talos::process::NodeFlags& flags) {
  const auto robot_config = talos::config::ParseRobotConfig(flags.config_path);
  TelemetryConfig config{};
  if (const auto* tbl = robot_config.GetSubsystemTable("telemetry")) {
    config.period_us = (*tbl)["period_us"].value_or(config.period_us);
    config.topic = (*tbl)["topic"].value_or(config.topic);
    config.stale_timeout_us =
        (*tbl)["stale_timeout_us"].value_or(config.stale_timeout_us);
  }
  return YearTelemetryNode{loop, config};
}

}  // namespace talos::telemetry::year2026
