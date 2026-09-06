#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <string>
#include <vector>

#include "2026-robot/main_processor/driver_station/driver_station_message_generated.h"
#include "2026-robot/main_processor/driver_station/packet.h"
#include "2026-robot/main_processor/drivetrain/drive_message_generated.h"
#include "2026-robot/main_processor/odometry/odometry_message_generated.h"
#include "2026-robot/main_processor/odometry/packet.h"
#include "studio/schema/wire.h"
#include "studio/schema/telemetry_generated.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/handles.h"
#include "talOS/rtms/rtms.h"

// Publishes the robot's state as Studio telemetry frames. This is the only
// producer on the Studio topic, which the bridge requires: that topic carries
// DROP_NEWEST, so a full queue drops the newest frame instead of letting a
// producer overwrite bytes the bridge is still sending. Never point the bridge
// at a control-loop topic, which uses OVERWRITE_OLDEST.
namespace talos::telemetry {

// The reserved `talos` owner because this feed belongs to no subsystem, and
// role `telemetry` because it is an observation feed: recording and display
// only, never an input to a control path.
inline constexpr const char* kStudioTopic = "/talos/telemetry";

struct TelemetryConfig {
  uint32_t period_us{10000};  // 100 Hz. Telemetry, not a control loop.
  std::string topic{kStudioTopic};
};

template <typename Loop>
class TelemetryNode {
 public:
  explicit TelemetryNode(Loop& loop, TelemetryConfig config = {})
      : period_us_{config.period_us},
        queue_{config.topic, studio::slot_bytes, 8, studio::slot_count,
               RTMSOptions{OverflowPolicy::DROP_NEWEST, ReadMode::SEQUENCE,
                           /*reclaim_mismatched_segment=*/true}} {
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
    timer_.setup_periodic(first, std::chrono::microseconds{period_us_});
  }

  uint64_t frames_published() const { return published_; }
  uint64_t frames_dropped() const { return dropped_; }
  uint64_t frames_rejected() const { return rejected_; }

 private:
  void OnOdometry(const event::Context&, const odometry::OdometryState& state) {
    odometry_ = state;
    have_odometry_ = true;
  }
  void OnDrivetrain(const event::Context&,
                    const talos::drive::DrivetrainState& state) {
    drivetrain_ = state;
    have_drivetrain_ = true;
  }
  void OnDriverStation(const event::Context&,
                       const driver_station::DriverStationState& state) {
    driver_station_ = state;
    have_driver_station_ = true;
  }

  void Tick(const event::Context& context) {
    // chassis is a required field, and Studio rejects a frame without it. No
    // pose yet means nothing worth drawing, so publish nothing rather than
    // put the origin on the field as though the robot were sitting there.
    if (!have_odometry_) return;

    const double yaw_rad = odometry_.yaw_rot() * 2.0 * std::numbers::pi;
    if (!Finite({odometry_.x_m(), odometry_.y_m(), yaw_rad})) {
      ++rejected_;
      return;
    }

    flatbuffers::FlatBufferBuilder builder;
    std::vector<flatbuffers::Offset<Talos::Telemetry::Channel>> channels;
    auto channel = [&](const char* name, double value, const char* unit) {
      if (!std::isfinite(value)) return;
      channels.push_back(
          Talos::Telemetry::CreateChannel(builder, builder.CreateString(name),
                                          value, builder.CreateString(unit)));
    };

    channel("odometry/vx", odometry_.vx_mps(), "m/s");
    channel("odometry/vy", odometry_.vy_mps(), "m/s");
    channel("odometry/omega", odometry_.omega_radps(), "rad/s");
    channel("shooter/flywheel", odometry_.flywheel_rps(), "rev/s");
    channel("shooter/beam_broken", odometry_.beam_broken() ? 1.0 : 0.0, "");
    if (have_drivetrain_) {
      channel("drivetrain/vx", drivetrain_.vx_mps(), "m/s");
      channel("drivetrain/vy", drivetrain_.vy_mps(), "m/s");
      channel("drivetrain/omega", drivetrain_.omega_radps(), "rad/s");
      channel("drivetrain/yaw_rate", drivetrain_.yaw_rate_rps(), "rev/s");
      channel("drivetrain/enabled", drivetrain_.enabled() ? 1.0 : 0.0, "");
    }
    if (have_driver_station_) {
      const uint32_t flags = driver_station_.fms().flags();
      channel("ds/enabled", (flags & driver_station::kEnabled) ? 1.0 : 0.0, "");
      channel("ds/teleop", (flags & driver_station::kTeleop) ? 1.0 : 0.0, "");
      channel("ds/autonomous",
              (flags & driver_station::kAutonomous) ? 1.0 : 0.0, "");
      channel("ds/match_time",
              static_cast<double>(driver_station_.fms().match_time_s()), "s");
    }

    const auto channel_vector = builder.CreateVector(channels);
    const Talos::Telemetry::Pose chassis{
        odometry_.x_m(), odometry_.y_m(), 0.0, 0.0, 0.0, yaw_rad};
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

    // A full queue means the bridge is behind. Telemetry is droppable by
    // design; the sequence id still advances so the gap is visible downstream.
    if (WriteResult(queue_.write({slot_.size(), slot_.data()})) ==
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

  uint32_t period_us_;
  RTMSQueue queue_;
  std::array<std::byte, studio::slot_bytes> slot_{};

  odometry::OdometryState odometry_{};
  talos::drive::DrivetrainState drivetrain_{};
  driver_station::DriverStationState driver_station_{};
  bool have_odometry_{false}, have_drivetrain_{false},
      have_driver_station_{false};

  uint64_t sequence_{0}, published_{0}, dropped_{0}, rejected_{0};
  event::Timer<Loop> timer_;
};

}  // namespace talos::telemetry
