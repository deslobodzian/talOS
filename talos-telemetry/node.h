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

#include "studio/schema/telemetry_generated.h"
#include "studio/schema/wire.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/handles.h"
#include "talOS/rtms/rtms.h"
#include "talos_nodes/driver_station/driver_station_message_generated.h"
#include "talos_nodes/driver_station/packet.h"
#include "talos_nodes/drivetrain/drive_message_generated.h"
#include "talos_nodes/odometry/odometry_message_generated.h"
#include "talos_nodes/odometry/packet.h"

// Publishes the robot's state as Studio telemetry frames. This is the only
// producer on the Studio topic, which the bridge requires: that topic carries
// DROP_NEWEST, so a full queue drops the newest frame instead of letting a
// producer overwrite bytes the bridge is still sending. Never point the bridge
// at a control-loop topic, which uses OVERWRITE_OLDEST.
//
// This core is game-agnostic and reusable across seasons: pose, chassis, and
// driver-station channels only. Year-specific mechanism channels (shooter,
// intake) live in the year wrapper
// (2026-robot/main_processor/telemetry/), which watches its year's mechanism
// state topics and forwards display values through SetMechanismChannels.
// Keeping them out of here means this core never changes with game logic.
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

// One extra display channel fed by the year wrapper. Names and units are the
// wrapper's year-local vocabulary (e.g. "shooter/flywheel"); the core passes
// them through the same finite-value filter as its own channels.
struct MechanismChannel {
  std::string name;
  double value{0.0};
  std::string unit;
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
    event::watch<odometry::OdometryState, &TelemetryNode::OnOdometry>(
        loop, odometry::kOdometryTopic, this);
    event::watch<talos::drive::DrivetrainState, &TelemetryNode::OnDrivetrain>(
        loop, talos::drive::kDrivetrainStateTopic, this);
    event::watch<driver_station::DriverStationState,
                 &TelemetryNode::OnDriverStation>(
        loop, driver_station::kDsStateTopic, this);
    timer_ = event::make_timer<&TelemetryNode::Tick>(loop, "telemetry", this);
  }

  void Start(event::MonotonicTime first) {
    // Seed the loop clock so a mechanism push that beats the first tick
    // still gets a sane freshness stamp (see SetMechanismChannels).
    last_loop_now_ = first - std::chrono::microseconds{period_us_};
    timer_.setup_periodic(first, std::chrono::microseconds{period_us_});
  }

  // Year mechanism display channels, set by the year wrapper from its own
  // mechanism watchers. Same single-threaded loop on both ends, so no
  // synchronization is needed; the latest set wins at the next tick.
  void SetMechanismChannels(std::vector<MechanismChannel> channels) {
    mechanism_ = std::move(channels);
    have_mechanism_ = true;
    // No loop timestamp rides along on this call, so stamp the latest loop
    // time this core observed. The wrapper calls this synchronously from
    // its own message handlers on the same loop, so the stamp lands within
    // one tick period of the mechanism message -- far below the staleness
    // horizon -- and every tick refreshes the clock in Tick below.
    mechanism_at_ = last_loop_now_;
  }

  uint64_t frames_published() const { return published_; }
  uint64_t frames_dropped() const { return dropped_; }
  uint64_t frames_rejected() const { return rejected_; }

 private:
  void OnOdometry(const event::Context& context,
                  const odometry::OdometryState& state) {
    odometry_ = state;
    have_odometry_ = true;
    last_odometry_ = context.now;
    last_loop_now_ = context.now;
  }
  void OnDrivetrain(const event::Context& context,
                    const talos::drive::DrivetrainState& state) {
    drivetrain_ = state;
    have_drivetrain_ = true;
    last_drivetrain_ = context.now;
    last_loop_now_ = context.now;
  }
  void OnDriverStation(const event::Context& context,
                       const driver_station::DriverStationState& state) {
    driver_station_ = state;
    have_driver_station_ = true;
    last_driver_station_ = context.now;
    last_loop_now_ = context.now;
  }

  void Tick(const event::Context& context) {
    const event::MonotonicTime now = context.now;
    last_loop_now_ = now;
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
    const bool mech_ok = have_mechanism_ && fresh(mechanism_at_);
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
    // Studio discards a frame with a duplicate channel name, so a mechanism
    // row colliding with a core status channel loses to the core row rather
    // than sinking the whole frame. The status rows below are the reserved
    // `pose/*`, `odometry/stale`, `drivetrain/stale`, `ds/stale`, and
    // `mechanism/stale` names.
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
    if (have_mechanism_)
      channel("mechanism/stale", fresh(mechanism_at_) ? 0.0 : 1.0, "");

    // Stale value rows are omitted, never frozen: a stopped publisher's last
    // numbers must not keep displaying as live data next to a stale marker.
    if (odom_ok) {
      channel("odometry/vx", odometry_.vx_mps(), "m/s");
      channel("odometry/vy", odometry_.vy_mps(), "m/s");
      channel("odometry/omega", odometry_.omega_radps(), "rad/s");
    }
    // Year-local mechanism channels keep their names, values, and units, so
    // a wrapper feeding the same values produces the same rows as the
    // pre-split node (frames additionally carry the status rows above).
    if (mech_ok) {
      for (const auto& extra : mechanism_) {
        channel(extra.name.c_str(), extra.value, extra.unit.c_str());
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

  odometry::OdometryState odometry_{};
  talos::drive::DrivetrainState drivetrain_{};
  driver_station::DriverStationState driver_station_{};
  std::vector<MechanismChannel> mechanism_;
  bool have_odometry_{false}, have_drivetrain_{false},
      have_driver_station_{false}, have_mechanism_{false};
  // Last loop time per source, for staleness marking. MonotonicTime{} is the
  // loop epoch; Start seeds last_loop_now_ so pre-first-tick mechanism
  // pushes stamp sanely (see SetMechanismChannels).
  event::MonotonicTime last_odometry_{}, last_drivetrain_{},
      last_driver_station_{}, mechanism_at_{}, last_loop_now_{};

  uint64_t sequence_{0}, published_{0}, dropped_{0}, rejected_{0};
  event::Timer<Loop> timer_;
};

}  // namespace talos::telemetry
