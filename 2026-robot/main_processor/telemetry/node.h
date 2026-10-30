#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "drivetrain/drive_message_generated.h"
#include "drivetrain/topics.h"
#include "intake/intake_message_generated.h"
#include "intake/topics.h"
#include "odometry/odometry_message_generated.h"
#include "odometry/topics.h"
#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"
#include "talOS/telemetry/telemetry_generated.h"
#include "talOS/telemetry/wire.h"

// Publishes the robot's state as Studio telemetry v2 frames: size-prefixed
// FlatBuffers on the byte topic the Studio bridge reads. This is the topic's
// only producer, and it is DROP_NEWEST, so a full queue drops the newest frame
// instead of overwriting bytes the bridge is still sending.
//
// Wire v2 (talOS/telemetry/README.md): the channel dictionary is built once
// here, from the fixed channels below plus one value/connected/stale triple
// per subsystem whose TOML sets telemetry = true. Every frame carries
// values[] aligned with that dictionary; NaN marks a source absent or stale
// this tick, never a frozen last value. The dictionary itself rides only the
// first frame and every 100th frame after it, so value frames stay under
// telemetry_wire::kMaxValueFrameBytes -- enforced at construction below.
//
// update() does not allocate once warm: the FlatBufferBuilder and the values
// vector are members reused every tick.
namespace talos::telemetry {

// Wire-compatible with the State struct of any single-demand mechanism
// scaffolded by tools/new_subsystem.py (enabled:bool + 7 pad + value:double +
// connected:bool + 7 pad).
struct alignas(8) GenericState {
  std::uint8_t enabled{0};
  std::uint8_t pad[7]{};
  double value{0.0};
  std::uint8_t connected{0};
  std::uint8_t pad2[7]{};
};
static_assert(sizeof(GenericState) == 24);
static_assert(talos::Message<GenericState>);

// One dictionary entry, in wire order.
struct ChannelDef {
  std::string name;
  std::string unit;
  std::vector<std::string> labels;  // non-empty: enum channel
};

// FNV-1a 32 over names, units and labels in dictionary order, with NUL
// separators between fields and 0xFF between channels.
inline std::uint32_t DictionaryId(const std::vector<ChannelDef>& dictionary) {
  std::uint32_t hash = 2166136261u;
  auto mix = [&hash](std::string_view text) {
    for (const char c : text) {
      hash ^= static_cast<std::uint8_t>(c);
      hash *= 16777619u;
    }
  };
  auto separator = [&hash](std::uint8_t byte) {
    hash ^= byte;
    hash *= 16777619u;
  };
  for (const ChannelDef& channel : dictionary) {
    mix(channel.name);
    separator(0);
    mix(channel.unit);
    separator(0);
    for (const std::string& label : channel.labels) {
      mix(label);
      separator(0);
    }
    separator(0xff);
  }
  return hash;
}

// Size in bytes (prefix included) of a value-only frame with every channel
// present. FlatBuffers vector size does not depend on the values, so this
// is the worst case the construction-time budget check measures.
inline std::size_t MeasureValueFrameBytes(std::size_t channel_count) {
  flatbuffers::FlatBufferBuilder builder{1024};
  const std::vector<double> values(channel_count, 0.0);
  const auto values_offset = builder.CreateVector(values.data(), values.size());
  const Talos::Telemetry::Pose chassis{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Talos::Telemetry::FinishSizePrefixedFrameBuffer(
      builder, Talos::Telemetry::CreateFrame(builder, /*stream_id=*/0,
                                             /*sequence_id=*/0,
                                             /*timestamp_ns=*/0,
                                             /*dictionary_id=*/0, &chassis,
                                             values_offset,
                                             /*dictionary=*/0));
  return builder.GetSize();
}

class Telemetry {
 public:
  explicit Telemetry(Node& node)
      : node_{node},
        stale_after_{std::chrono::microseconds{
            node.param<std::int64_t>("stale_timeout_us", 100'000)}},
        odometry_{node.subscribe<::odometry::OdometryState>(
            ::odometry::topics::kState)},
        drivetrain_{node.subscribe<::drivetrain::DrivetrainState>(
            ::drivetrain::topics::kState)},
        shooter_{
            node.subscribe<::shooter::ShooterState>(::shooter::topics::kState)},
        intake_{
            node.subscribe<::intake::IntakeState>(::intake::topics::kState)},
        bridge_{
            node.subscribe<::bridge::BridgeStatus>(::bridge::topics::kStatus)},
        ds_{node},
        studio_{node.publish_bytes(node.param<std::string>("topic"),
                                   telemetry_wire::slot_bytes,
                                   {.slots = telemetry_wire::slot_count,
                                    .drop_newest = true,
                                    .external = true})} {
    BuildDictionary(node);
    dictionary_id_ = DictionaryId(dictionary_);
    stream_id_ = RandomStreamId();
    values_.assign(dictionary_.size(),
                   std::numeric_limits<double>::quiet_NaN());
    // The radio budget is a construction-time guarantee, not a hope: a
    // dictionary grown past it fails here, loudly, before the robot moves.
    const std::size_t worst_case = MeasureValueFrameBytes(dictionary_.size());
    if (worst_case > telemetry_wire::kMaxValueFrameBytes) {
      throw std::runtime_error(
          "telemetry: worst-case value frame is " + std::to_string(worst_case) +
          " bytes, over the " +
          std::to_string(telemetry_wire::kMaxValueFrameBytes) +
          "-byte budget (see talOS/telemetry/README.md)");
    }
  }

  std::uint64_t stream_id() const { return stream_id_; }
  std::uint32_t dictionary_id() const { return dictionary_id_; }
  const std::vector<ChannelDef>& dictionary() const { return dictionary_; }

  void update() {
    const double yaw_rad = odometry_->yaw_rot() * 2.0 * std::numbers::pi;
    const bool odom_ok = Fresh(odometry_) && std::isfinite(odometry_->x_m()) &&
                         std::isfinite(odometry_->y_m()) &&
                         std::isfinite(yaw_rad);
    const bool shooter_ok = Fresh(shooter_);
    const bool intake_ok = Fresh(intake_);
    bool extra_ok = false;
    bool extra_has_value = false;
    for (const auto& mech : extra_mechanisms_) {
      if (mech.state.has_value()) extra_has_value = true;
      if (Fresh(mech.state)) extra_ok = true;
    }
    const bool mech_ok = shooter_ok || intake_ok || extra_ok;
    // A live mechanism is worth showing before (and without) any pose; the
    // drivetrain and driver station alone carry neither, so they never
    // trigger a frame on their own.
    if (!odom_ok && !mech_ok) return;

    std::fill(values_.begin(), values_.end(),
              std::numeric_limits<double>::quiet_NaN());
    auto set = [this](std::size_t index, double value) {
      // A non-finite sample is an absent sample: NaN is the wire's absence
      // marker, and shipping inf would break JSON consumers downstream.
      if (std::isfinite(value)) values_[index] = value;
    };

    // Chassis is zeros until a fresh finite pose arrives, so the field view
    // must gate drawing on this flag.
    set(idx_pose_valid_, odom_ok ? 1.0 : 0.0);
    if (odometry_.has_value())
      set(idx_odometry_stale_, Fresh(odometry_) ? 0.0 : 1.0);
    if (drivetrain_.has_value())
      set(idx_drivetrain_stale_, Fresh(drivetrain_) ? 0.0 : 1.0);
    set(idx_ds_stale_, ds_.connected() ? 0.0 : 1.0);
    if (shooter_.has_value() || intake_.has_value() || extra_has_value)
      set(idx_mechanism_stale_, mech_ok ? 0.0 : 1.0);

    if (odom_ok) {
      set(idx_odometry_vx_, odometry_->vx_mps());
      set(idx_odometry_vy_, odometry_->vy_mps());
      set(idx_odometry_omega_, odometry_->omega_radps());
    }
    if (mech_ok && shooter_.has_value()) {
      if (shooter_ok) {
        set(idx_shooter_flywheel_, shooter_->flywheel_velocity_rps());
        set(idx_shooter_beam_broken_, shooter_->beam_broken() ? 1.0 : 0.0);
      }
      set(idx_shooter_stale_, shooter_ok ? 0.0 : 1.0);
    }
    if (mech_ok && intake_.has_value()) {
      if (intake_ok) {
        set(idx_intake_requested_, intake_->requested_velocity_rps());
        set(idx_intake_measured_, intake_->roller_velocity_rps());
        set(idx_intake_inhibit_, static_cast<double>(intake_->inhibit()));
      }
      set(idx_intake_stale_, intake_ok ? 0.0 : 1.0);
    }
    for (const auto& mech : extra_mechanisms_) {
      if (mech.state.has_value()) {
        const bool ok = Fresh(mech.state);
        if (ok) {
          set(mech.value_index, mech.state->value);
          set(mech.connected_index, mech.state->connected != 0 ? 1.0 : 0.0);
        }
        set(mech.stale_index, ok ? 0.0 : 1.0);
      }
    }
    if (Fresh(drivetrain_)) {
      set(idx_drivetrain_vx_, drivetrain_->vx_mps());
      set(idx_drivetrain_vy_, drivetrain_->vy_mps());
      set(idx_drivetrain_omega_, drivetrain_->omega_radps());
      set(idx_drivetrain_yaw_rate_, drivetrain_->yaw_rate_rps());
      set(idx_drivetrain_enabled_, drivetrain_->enabled() ? 1.0 : 0.0);
    }
    if (ds_.connected()) {
      set(idx_ds_enabled_, ds_.enabled() ? 1.0 : 0.0);
      set(idx_ds_teleop_, ds_.teleop() ? 1.0 : 0.0);
      set(idx_ds_autonomous_, ds_.autonomous() ? 1.0 : 0.0);
      set(idx_ds_match_time_, ds_.match_time());
    }
    // The bridge's own health: how old the hardware state it last received
    // is, what it refused, and which actuator subsystems it timed out.
    if (bridge_.has_value()) {
      const bool bridge_ok = Fresh(bridge_);
      if (bridge_ok) {
        set(idx_bridge_state_age_us_,
            static_cast<double>(bridge_->state_age_us()));
        set(idx_bridge_identity_rejected_,
            static_cast<double>(bridge_->identity_rejected()));
        set(idx_bridge_ownership_rejected_,
            static_cast<double>(bridge_->ownership_rejected()));
        set(idx_bridge_timed_out_mask_,
            static_cast<double>(bridge_->timed_out_mask()));
        set(idx_bridge_configured_, bridge_->configured() ? 1.0 : 0.0);
      }
      set(idx_bridge_stale_, bridge_ok ? 0.0 : 1.0);
    }

    flatbuffers::FlatBufferBuilder& builder = builder_;
    builder.Clear();
    const auto values_offset =
        builder.CreateVector(values_.data(), values_.size());
    flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<Talos::Telemetry::ChannelInfo>>>
        dictionary_offset = 0;
    // First frame and every 100th frame; never otherwise.
    if (sequence_ % 100 == 0) {
      std::vector<flatbuffers::Offset<Talos::Telemetry::ChannelInfo>> entries;
      entries.reserve(dictionary_.size());
      for (const ChannelDef& channel : dictionary_) {
        std::vector<flatbuffers::Offset<flatbuffers::String>> labels;
        labels.reserve(channel.labels.size());
        for (const std::string& label : channel.labels) {
          labels.push_back(builder.CreateString(label));
        }
        entries.push_back(Talos::Telemetry::CreateChannelInfo(
            builder, builder.CreateString(channel.name),
            channel.unit.empty() ? 0 : builder.CreateString(channel.unit),
            channel.labels.empty() ? 0 : builder.CreateVector(labels)));
      }
      dictionary_offset = builder.CreateVector(entries);
    }
    const Talos::Telemetry::Pose chassis{odom_ok ? odometry_->x_m() : 0.0,
                                         odom_ok ? odometry_->y_m() : 0.0,
                                         0.0,
                                         0.0,
                                         0.0,
                                         odom_ok ? yaw_rad : 0.0};
    Talos::Telemetry::FinishSizePrefixedFrameBuffer(
        builder,
        Talos::Telemetry::CreateFrame(
            builder, stream_id_, sequence_,
            static_cast<std::uint64_t>(node_.now().nanos()), dictionary_id_,
            &chassis, values_offset, dictionary_offset));
    // A dropped frame still spends its sequence id, so the gap shows
    // downstream.
    studio_.send(std::as_bytes(
        std::span{builder.GetBufferPointer(), builder.GetSize()}));
    ++sequence_;
  }

 private:
  static std::uint64_t RandomStreamId() {
    std::random_device device;
    const std::uint64_t id =
        (static_cast<std::uint64_t>(device()) << 32) | device();
    return id != 0 ? id : 1;
  }

  std::size_t AddChannel(std::string name, std::string unit = "",
                         std::vector<std::string> labels = {}) {
    dictionary_.push_back(
        ChannelDef{std::move(name), std::move(unit), std::move(labels)});
    return dictionary_.size() - 1;
  }

  void BuildDictionary(Node& node) {
    idx_pose_valid_ = AddChannel("pose/valid");
    idx_odometry_stale_ = AddChannel("odometry/stale");
    idx_drivetrain_stale_ = AddChannel("drivetrain/stale");
    idx_ds_stale_ = AddChannel("ds/stale");
    idx_mechanism_stale_ = AddChannel("mechanism/stale");
    idx_odometry_vx_ = AddChannel("odometry/vx", "m/s");
    idx_odometry_vy_ = AddChannel("odometry/vy", "m/s");
    idx_odometry_omega_ = AddChannel("odometry/omega", "rad/s");
    idx_shooter_flywheel_ = AddChannel("shooter/flywheel", "rev/s");
    idx_shooter_beam_broken_ = AddChannel("shooter/beam_broken");
    idx_shooter_stale_ = AddChannel("shooter/stale");
    idx_intake_requested_ = AddChannel("intake/requested", "rev/s");
    idx_intake_measured_ = AddChannel("intake/measured", "rev/s");
    std::vector<std::string> inhibit_labels;
    for (const char* const* names = ::intake::EnumNamesInhibit();
         *names != nullptr; ++names) {
      inhibit_labels.emplace_back(*names);
    }
    idx_intake_inhibit_ =
        AddChannel("intake/inhibit", "", std::move(inhibit_labels));
    idx_intake_stale_ = AddChannel("intake/stale");

    // TOML-configured extra mechanisms (telemetry = true subsystems).
    if (!node.config_path().empty()) {
      const auto cfg = config::ParseRobotConfig(node.config_path());
      if (const auto* subs = cfg.toml_data["subsystems"].as_table()) {
        for (const auto& [key, val] : *subs) {
          const std::string_view sub_name = key.str();
          if (sub_name == "odometry" || sub_name == "drivetrain" ||
              sub_name == "driver_station" || sub_name == "shooter" ||
              sub_name == "intake" || sub_name == "telemetry") {
            continue;
          }
          const auto* tbl = val.as_table();
          if (tbl == nullptr) continue;
          const bool enabled = (*tbl)["telemetry"].value_or(false);
          const auto unit_opt =
              (*tbl)["telemetry_unit"].value<std::string_view>();
          if (!enabled && !unit_opt.has_value()) continue;
          const std::string name{sub_name};
          const std::string unit{unit_opt.value_or("rev/s")};
          ExtraMechanism mech;
          mech.state = node.subscribe<GenericState>("/" + name + "/state",
                                                    {.optional = true});
          mech.value_index = AddChannel(name + "/value", unit);
          mech.connected_index = AddChannel(name + "/connected");
          mech.stale_index = AddChannel(name + "/stale");
          extra_mechanisms_.push_back(std::move(mech));
        }
      }
    }

    idx_drivetrain_vx_ = AddChannel("drivetrain/vx", "m/s");
    idx_drivetrain_vy_ = AddChannel("drivetrain/vy", "m/s");
    idx_drivetrain_omega_ = AddChannel("drivetrain/omega", "rad/s");
    idx_drivetrain_yaw_rate_ = AddChannel("drivetrain/yaw_rate", "rev/s");
    idx_drivetrain_enabled_ = AddChannel("drivetrain/enabled");
    idx_ds_enabled_ = AddChannel("ds/enabled");
    idx_ds_teleop_ = AddChannel("ds/teleop");
    idx_ds_autonomous_ = AddChannel("ds/autonomous");
    idx_ds_match_time_ = AddChannel("ds/match_time", "s");
    idx_bridge_state_age_us_ = AddChannel("bridge/state_age_us", "us");
    idx_bridge_identity_rejected_ = AddChannel("bridge/identity_rejected");
    idx_bridge_ownership_rejected_ = AddChannel("bridge/ownership_rejected");
    idx_bridge_timed_out_mask_ = AddChannel("bridge/timed_out_mask");
    idx_bridge_configured_ = AddChannel("bridge/configured");
    idx_bridge_stale_ = AddChannel("bridge/stale");
  }

  template <class M>
  bool Fresh(const Subscription<M>& source) const {
    return source.has_value() &&
           (stale_after_ == Duration::zero() || source.age() <= stale_after_);
  }

  Node& node_;
  Duration stale_after_;
  Subscription<::odometry::OdometryState> odometry_;
  Subscription<::drivetrain::DrivetrainState> drivetrain_;
  Subscription<::shooter::ShooterState> shooter_;
  Subscription<::intake::IntakeState> intake_;
  Subscription<::bridge::BridgeStatus> bridge_;
  talos::DriverStation ds_;
  struct ExtraMechanism {
    Subscription<GenericState> state;
    std::size_t value_index{0};
    std::size_t connected_index{0};
    std::size_t stale_index{0};
  };
  std::vector<ExtraMechanism> extra_mechanisms_;
  BytePublisher studio_;
  std::vector<ChannelDef> dictionary_;
  std::uint32_t dictionary_id_{0};
  std::uint64_t stream_id_{0};
  std::vector<double> values_;
  flatbuffers::FlatBufferBuilder builder_{1024};
  std::uint64_t sequence_ = 0;
  std::size_t idx_pose_valid_{0}, idx_odometry_stale_{0},
      idx_drivetrain_stale_{0}, idx_ds_stale_{0}, idx_mechanism_stale_{0},
      idx_odometry_vx_{0}, idx_odometry_vy_{0}, idx_odometry_omega_{0},
      idx_shooter_flywheel_{0}, idx_shooter_beam_broken_{0},
      idx_shooter_stale_{0}, idx_intake_requested_{0}, idx_intake_measured_{0},
      idx_intake_inhibit_{0}, idx_intake_stale_{0}, idx_drivetrain_vx_{0},
      idx_drivetrain_vy_{0}, idx_drivetrain_omega_{0},
      idx_drivetrain_yaw_rate_{0}, idx_drivetrain_enabled_{0},
      idx_ds_enabled_{0}, idx_ds_teleop_{0}, idx_ds_autonomous_{0},
      idx_ds_match_time_{0}, idx_bridge_state_age_us_{0},
      idx_bridge_identity_rejected_{0}, idx_bridge_ownership_rejected_{0},
      idx_bridge_timed_out_mask_{0}, idx_bridge_configured_{0},
      idx_bridge_stale_{0};
};

}  // namespace talos::telemetry
