#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "driver_station/driver_station_message_generated.h"
#include "driver_station/topics.h"
#include "drivetrain/drive_message_generated.h"
#include "drivetrain/topics.h"
#include "intake/intake_message_generated.h"
#include "intake/topics.h"
#include "odometry/odometry_message_generated.h"
#include "odometry/topics.h"
#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "studio/schema/telemetry_generated.h"
#include "studio/schema/wire.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/handles.h"
#include "talOS/rtms/rtms.h"

// Publishes the robot's state as Studio telemetry frames. This is the only
// producer on the Studio topic, which the bridge requires: that topic carries
// DROP_NEWEST, so a full queue drops the newest frame instead of letting a
// producer overwrite bytes the bridge is still sending. Never point the bridge
// at a control-loop topic, which uses OVERWRITE_OLDEST.
//
// Channels: pose, chassis, and driver station, plus the 2026 mechanisms
// (shooter state and the intake Studio view). A missing mechanism publisher
// simply leaves its rows out of the frame.
//
// The one node not written on talos::Node, by design. Its output is a
// variable-length flatbuffer frame written straight to a DROP_NEWEST RTMS
// queue sized for Studio, not a fixed-size struct on a loop topic, so Node's
// publish<M>() cannot carry it: Node's topics are OVERWRITE_OLDEST slots of
// one message type. Being off the loop, the feed is invisible to the
// manifest, so main.cc runs this class on talos::process::RunNode with the
// hooks that declare it (an external source in --describe and in the live
// Reporter, with its frame counters) and print its own exit summary.
namespace talos::telemetry {

// The reserved `talos` owner because this feed belongs to no subsystem, and
// role `telemetry` because it is an observation feed: recording and display
// only, never an input to a control path.
inline constexpr const char* kStudioTopic = "/talos/telemetry";

struct TelemetryConfig {
  uint32_t period_us{10000};  // 100 Hz. Telemetry, not a control loop.
  std::string topic{kStudioTopic};
  // Silence horizon per data source. A source older than this at tick time
  // is stale: its value channels are omitted and its `<source>/stale`
  // marker reads 1, so a stopped publisher cannot look healthy. Zero
  // disables staleness marking (every seen source counts as fresh).
  uint64_t stale_timeout_us{100000};  // 100 ms, like command timeouts.
};

template <typename Loop>
class TelemetryNode {
 public:
  // The Studio shared-memory segment is NOT opened here. Construction only
  // registers loop watchers and a timer, all of which stay in-process on a
  // simulated loop -- so building the node for --describe (or merely
  // constructing it) has no socket or shared-memory side effects. The queue
  // opens on the first publishable frame instead (see EnsureQueue); --describe
  // never starts the timer and never ticks, so it never touches shm.
  explicit TelemetryNode(Loop& loop, TelemetryConfig config = {})
      : period_us_{config.period_us},
        stale_timeout_us_{config.stale_timeout_us},
        topic_{std::move(config.topic)} {
    event::watch<::odometry::OdometryState, &TelemetryNode::OnOdometry>(
        loop, ::odometry::kStateTopic, this);
    event::watch<::drivetrain::DrivetrainState, &TelemetryNode::OnDrivetrain>(
        loop, ::drivetrain::kStateTopic, this);
    event::watch<::driver_station::DriverStationState,
                 &TelemetryNode::OnDriverStation>(
        loop, ::driver_station::kStateTopic, this);
    event::watch<shooter::ShooterState, &TelemetryNode::OnShooter>(
        loop, shooter::kStateTopic, this);
    event::watch<::intake::IntakeState, &TelemetryNode::OnIntake>(
        loop, ::intake::kStateTopic, this);
    timer_ = event::make_timer<&TelemetryNode::Tick>(loop, "telemetry", this);
  }

  void Start(event::MonotonicTime first) {
    timer_.setup_periodic(first, std::chrono::microseconds{period_us_});
  }

  uint64_t frames_published() const { return published_; }
  uint64_t frames_dropped() const { return dropped_; }
  uint64_t frames_rejected() const { return rejected_; }

 private:
  void OnOdometry(const event::Context& context,
                  const ::odometry::OdometryState& state) {
    odometry_ = state;
    have_odometry_ = true;
    last_odometry_ = context.now;
  }
  void OnDrivetrain(const event::Context& context,
                    const ::drivetrain::DrivetrainState& state) {
    drivetrain_ = state;
    have_drivetrain_ = true;
    last_drivetrain_ = context.now;
  }
  void OnDriverStation(const event::Context& context,
                       const ::driver_station::DriverStationState& state) {
    driver_station_ = state;
    have_driver_station_ = true;
    last_driver_station_ = context.now;
  }
  void OnShooter(const event::Context& context,
                 const shooter::ShooterState& state) {
    shooter_ = state;
    have_shooter_ = true;
    last_shooter_ = context.now;
  }
  void OnIntake(const event::Context& context,
                const ::intake::IntakeState& state) {
    intake_ = state;
    have_intake_ = true;
    last_intake_ = context.now;
  }

  // Human-readable intake inhibit reason for the `intake/inhibit` unit.
  // The numeric value stays the wire code (intake::kInhibit* in
  // intake/topics.h, append-only); the unit names it so Studio
  // -- which renders each channel's unit next to its value -- reads
  // "TARGET_STALE" instead of a bare 3.
  static const char* InhibitReason(uint8_t inhibit) {
    switch (inhibit) {
      case ::intake::kInhibitDrive:
        return "DRIVE";
      case ::intake::kInhibitGateway:
        return "GATEWAY_NOT_READY";
      case ::intake::kInhibitTargetDisabled:
        return "TARGET_DISABLED";
      case ::intake::kInhibitTargetStale:
        return "TARGET_STALE";
      case ::intake::kInhibitFeedbackInvalid:
        return "FEEDBACK_INVALID";
      case ::intake::kInhibitDemandNonfinite:
        return "DEMAND_NONFINITE";
      default:
        return "UNKNOWN";
    }
  }

  void Tick(const event::Context& context) {
    const event::MonotonicTime now = context.now;
    const event::Duration timeout{std::chrono::microseconds{stale_timeout_us_}};
    const auto fresh = [&](event::MonotonicTime last) {
      return stale_timeout_us_ == 0 || (now - last) <= timeout;
    };

    double yaw_rad = 0.0;
    bool pose_finite = false;
    if (have_odometry_) {
      yaw_rad = odometry_.yaw_rot() * 2.0 * std::numbers::pi;
      pose_finite = Finite({odometry_.x_m(), odometry_.y_m(), yaw_rad});
    }
    const bool odom_ok = have_odometry_ && pose_finite && fresh(last_odometry_);
    // The mechanisms age out as one group (`mechanism/stale`), measured from
    // the newest mechanism message; within a live group each mechanism's
    // rows still drop on their own staleness, behind a `<mechanism>/stale`
    // marker, so a dead shooter beside a live intake never shows frozen
    // flywheel numbers.
    const bool have_mechanism = have_shooter_ || have_intake_;
    event::MonotonicTime mechanism_at{};
    if (have_shooter_) mechanism_at = last_shooter_;
    if (have_intake_ && last_intake_ > mechanism_at) mechanism_at = last_intake_;
    const bool mech_ok = have_mechanism && fresh(mechanism_at);
    const bool drive_ok = have_drivetrain_ && fresh(last_drivetrain_);
    const bool ds_ok = have_driver_station_ && fresh(last_driver_station_);

    // Mechanism channels publish independently: a live shooter or intake is
    // worth showing before (and without) any pose. Only the field/pose view
    // -- chassis plus the odometry/* channels, gated downstream on
    // pose/valid -- waits for a fresh finite pose. Drivetrain and driver
    // station alone carry neither field position nor mechanism rows, so they
    // ride along but never trigger a frame on their own.
    if (!odom_ok && !mech_ok) {
      // A non-finite pose is corrupt data worth counting; mere silence
      // (nothing seen yet, or every source aged out) is not.
      if (have_odometry_ && !pose_finite) ++rejected_;
      return;
    }

    flatbuffers::FlatBufferBuilder builder;
    std::vector<flatbuffers::Offset<Talos::Telemetry::Channel>> channels;
    // Studio discards a frame with a duplicate channel name, so the first
    // row with a given name wins rather than sinking the whole frame.
    std::vector<std::string> emitted;
    auto channel = [&](const char* name, double value, const char* unit) {
      if (!std::isfinite(value)) return;
      if (std::find(emitted.begin(), emitted.end(), name) != emitted.end())
        return;
      emitted.emplace_back(name);
      channels.push_back(
          Talos::Telemetry::CreateChannel(builder, builder.CreateString(name),
                                          value, builder.CreateString(unit)));
    };

    // Always present: the field/pose view must gate drawing on this, since
    // chassis below is zeros until a fresh finite pose arrives (chassis is a
    // required struct and Studio rejects a frame with a non-finite pose, so
    // there is no encodable "no pose" other than zeros plus this flag).
    channel("pose/valid", odom_ok ? 1.0 : 0.0, "");
    // Stale markers ride every published frame once their source was seen,
    // so a frozen-then-recovered source shows the gap in-stream. Additive
    // channel rows only: the flatbuffers schema is untouched, so old Studio
    // builds parse these frames and simply ignore names they do not know.
    if (have_odometry_)
      channel("odometry/stale", fresh(last_odometry_) ? 0.0 : 1.0, "");
    if (have_drivetrain_)
      channel("drivetrain/stale", fresh(last_drivetrain_) ? 0.0 : 1.0, "");
    if (have_driver_station_) {
      channel("ds/stale", fresh(last_driver_station_) ? 0.0 : 1.0, "");
    }
    if (have_mechanism)
      channel("mechanism/stale", fresh(mechanism_at) ? 0.0 : 1.0, "");

    // Stale value rows are omitted, never frozen: a stopped publisher's last
    // numbers must not keep displaying as live data next to a stale marker.
    if (odom_ok) {
      channel("odometry/vx", odometry_.vx_mps(), "m/s");
      channel("odometry/vy", odometry_.vy_mps(), "m/s");
      channel("odometry/omega", odometry_.omega_radps(), "rad/s");
    }
    // Intake rows are the node's Studio view: requested speed (the demand
    // acted on), measured speed (wheel feedback), and the inhibit code naming
    // why the wheel is not driving.
    if (mech_ok) {
      if (have_shooter_) {
        const bool ok = fresh(last_shooter_);
        if (ok) {
          channel("shooter/flywheel", shooter_.flywheel_velocity_rps(),
                  "rev/s");
          channel("shooter/beam_broken", shooter_.beam_broken() ? 1.0 : 0.0,
                  "");
        }
        channel("shooter/stale", ok ? 0.0 : 1.0, "");
      }
      if (have_intake_) {
        const bool ok = fresh(last_intake_);
        if (ok) {
          channel("intake/requested", intake_.requested_velocity_rps(),
                  "rev/s");
          channel("intake/measured", intake_.roller_velocity_rps(), "rev/s");
          channel("intake/inhibit", static_cast<double>(intake_.inhibit()),
                  InhibitReason(intake_.inhibit()));
        }
        channel("intake/stale", ok ? 0.0 : 1.0, "");
      }
    }
    if (drive_ok) {
      channel("drivetrain/vx", drivetrain_.vx_mps(), "m/s");
      channel("drivetrain/vy", drivetrain_.vy_mps(), "m/s");
      channel("drivetrain/omega", drivetrain_.omega_radps(), "rad/s");
      channel("drivetrain/yaw_rate", drivetrain_.yaw_rate_rps(), "rev/s");
      channel("drivetrain/enabled", drivetrain_.enabled() ? 1.0 : 0.0, "");
    }
    if (ds_ok) {
      const uint32_t flags = driver_station_.fms().flags();
      channel("ds/enabled", (flags & driver_station::kEnabled) ? 1.0 : 0.0, "");
      channel("ds/teleop", (flags & driver_station::kTeleop) ? 1.0 : 0.0, "");
      channel("ds/autonomous",
              (flags & driver_station::kAutonomous) ? 1.0 : 0.0, "");
      channel("ds/match_time",
              static_cast<double>(driver_station_.fms().match_time_s()), "s");
    }

    const auto channel_vector = builder.CreateVector(channels);
    const Talos::Telemetry::Pose chassis{odom_ok ? odometry_.x_m() : 0.0,
                                         odom_ok ? odometry_.y_m() : 0.0,
                                         0.0,
                                         0.0,
                                         0.0,
                                         odom_ok ? yaw_rad : 0.0};
    const auto frame = Talos::Telemetry::CreateFrame(
        builder, static_cast<uint64_t>(context.now.nanos()), sequence_,
        &chassis, /*ghosts=*/0, channel_vector, /*targets=*/0,
        /*elevator_m=*/0.0, /*arm_rad=*/0.0, /*camera=*/nullptr);
    Talos::Telemetry::FinishFrameBuffer(builder, frame);

    const auto size = builder.GetSize();
    if (size > slot_.size() - 4) {
      ++rejected_;
      return;
    }
    for (unsigned i = 0; i < 4; ++i)
      slot_[i] = std::byte((size >> (8 * i)) & 255);
    std::memcpy(slot_.data() + 4, builder.GetBufferPointer(), size);

    // First publishable frame opens the Studio segment; everything above
    // (no data yet, unusable pose, oversize frame) returns before any
    // shared-memory call, and construction/Start never open it at all.
    EnsureQueue();
    // A full queue means the bridge is behind. Telemetry is droppable by
    // design; the sequence id still advances so the gap is visible downstream.
    if (WriteResult(queue_->write({slot_.size(), slot_.data()})) ==
        WriteResult::SUCCESS) {
      ++published_;
    } else {
      ++dropped_;
    }
    ++sequence_;
  }

  static bool Finite(std::initializer_list<double> values) {
    for (double value : values)
      if (!std::isfinite(value)) return false;
    return true;
  }

  // Opens the Studio segment on first use. The node is this topic's only
  // producer, so it reclaims a stale segment left by an older build; readers
  // attach with the same geometry either before or after this runs.
  void EnsureQueue() {
    if (!queue_) {
      queue_.emplace(
          topic_, studio::slot_bytes, 8, studio::slot_count,
          RTMSOptions{OverflowPolicy::DROP_NEWEST, ReadMode::SEQUENCE,
                      /*reclaim_mismatched_segment=*/true});
    }
  }

  uint32_t period_us_;
  uint64_t stale_timeout_us_;
  std::string topic_;
  std::optional<RTMSQueue> queue_;
  std::array<std::byte, studio::slot_bytes> slot_{};

  ::odometry::OdometryState odometry_{};
  ::drivetrain::DrivetrainState drivetrain_{};
  ::driver_station::DriverStationState driver_station_{};
  shooter::ShooterState shooter_{};
  ::intake::IntakeState intake_{};
  bool have_odometry_{false}, have_drivetrain_{false},
      have_driver_station_{false}, have_shooter_{false}, have_intake_{false};
  // Last loop time per source, for staleness marking.
  event::MonotonicTime last_odometry_{}, last_drivetrain_{},
      last_driver_station_{}, last_shooter_{}, last_intake_{};

  uint64_t sequence_{0}, published_{0}, dropped_{0}, rejected_{0};
  event::Timer<Loop> timer_;
};

}  // namespace talos::telemetry
