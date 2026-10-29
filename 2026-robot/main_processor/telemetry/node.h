#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>
#include <string>
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
#include "talOS/node/node.h"

// Publishes the robot's state as Studio telemetry frames: size-prefixed
// flatbuffers on the byte topic the Studio bridge reads. This is the topic's
// only producer, and it is DROP_NEWEST, so a full queue drops the newest frame
// instead of overwriting bytes the bridge is still sending.
//
// Channels: pose, chassis and driver station, plus the 2026 mechanisms
// (shooter state and the intake Studio view). A source silent for longer than
// stale_timeout_us is stale: its value rows are omitted, never frozen, and its
// `<source>/stale` marker reads 1. Zero disables staleness.
//
// [subsystems.telemetry] keys: topic (required; the bridge reads the same
// key), period_us, stale_timeout_us.
namespace talos::telemetry {

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
        driver_station_{node.subscribe<::driver_station::DriverStationState>(
            ::driver_station::topics::kState)},
        shooter_{
            node.subscribe<::shooter::ShooterState>(::shooter::topics::kState)},
        intake_{
            node.subscribe<::intake::IntakeState>(::intake::topics::kState)},
        studio_{node.publish_bytes(node.param<std::string>("topic"),
                                   studio::slot_bytes,
                                   {.slots = studio::slot_count,
                                    .drop_newest = true,
                                    .external = true})} {}

  void update() {
    const double yaw_rad = odometry_->yaw_rot() * 2.0 * std::numbers::pi;
    const bool odom_ok = Fresh(odometry_) && std::isfinite(odometry_->x_m()) &&
                         std::isfinite(odometry_->y_m()) &&
                         std::isfinite(yaw_rad);
    const bool shooter_ok = Fresh(shooter_);
    const bool intake_ok = Fresh(intake_);
    const bool mech_ok = shooter_ok || intake_ok;
    // A live mechanism is worth showing before (and without) any pose; the
    // drivetrain and driver station alone carry neither, so they never
    // trigger a frame on their own.
    if (!odom_ok && !mech_ok) return;

    flatbuffers::FlatBufferBuilder builder;
    std::vector<flatbuffers::Offset<Talos::Telemetry::Channel>> channels;
    // Studio throws on a non-finite number and discards the whole frame.
    auto channel = [&](const char* name, double value, const char* unit = "") {
      if (!std::isfinite(value)) return;
      channels.push_back(
          Talos::Telemetry::CreateChannel(builder, builder.CreateString(name),
                                          value, builder.CreateString(unit)));
    };

    // Chassis is zeros until a fresh finite pose arrives, so the field view
    // must gate drawing on this flag.
    channel("pose/valid", odom_ok);
    if (odometry_.has_value()) channel("odometry/stale", !Fresh(odometry_));
    if (drivetrain_.has_value())
      channel("drivetrain/stale", !Fresh(drivetrain_));
    if (driver_station_.has_value())
      channel("ds/stale", !Fresh(driver_station_));
    if (shooter_.has_value() || intake_.has_value())
      channel("mechanism/stale", !mech_ok);

    if (odom_ok) {
      channel("odometry/vx", odometry_->vx_mps(), "m/s");
      channel("odometry/vy", odometry_->vy_mps(), "m/s");
      channel("odometry/omega", odometry_->omega_radps(), "rad/s");
    }
    if (mech_ok && shooter_.has_value()) {
      if (shooter_ok) {
        channel("shooter/flywheel", shooter_->flywheel_velocity_rps(), "rev/s");
        channel("shooter/beam_broken", shooter_->beam_broken());
      }
      channel("shooter/stale", !shooter_ok);
    }
    // The intake's Studio view: the demand acted on, the measured speed, and
    // why the wheel is not driving, named in the unit.
    if (mech_ok && intake_.has_value()) {
      if (intake_ok) {
        const char* reason = ::intake::EnumNameInhibit(intake_->inhibit());
        channel("intake/requested", intake_->requested_velocity_rps(), "rev/s");
        channel("intake/measured", intake_->roller_velocity_rps(), "rev/s");
        channel("intake/inhibit", static_cast<double>(intake_->inhibit()),
                *reason != '\0' ? reason : "UNKNOWN");
      }
      channel("intake/stale", !intake_ok);
    }
    if (Fresh(drivetrain_)) {
      channel("drivetrain/vx", drivetrain_->vx_mps(), "m/s");
      channel("drivetrain/vy", drivetrain_->vy_mps(), "m/s");
      channel("drivetrain/omega", drivetrain_->omega_radps(), "rad/s");
      channel("drivetrain/yaw_rate", drivetrain_->yaw_rate_rps(), "rev/s");
      channel("drivetrain/enabled", drivetrain_->enabled());
    }
    if (Fresh(driver_station_)) {
      const std::uint32_t flags = driver_station_->fms().flags();
      channel("ds/enabled", (flags & driver_station::kEnabled) != 0);
      channel("ds/teleop", (flags & driver_station::kTeleop) != 0);
      channel("ds/autonomous", (flags & driver_station::kAutonomous) != 0);
      channel("ds/match_time", driver_station_->fms().match_time_s(), "s");
    }

    const auto channel_vector = builder.CreateVector(channels);
    const Talos::Telemetry::Pose chassis{odom_ok ? odometry_->x_m() : 0.0,
                                         odom_ok ? odometry_->y_m() : 0.0,
                                         0.0,
                                         0.0,
                                         0.0,
                                         odom_ok ? yaw_rad : 0.0};
    Talos::Telemetry::FinishSizePrefixedFrameBuffer(
        builder,
        Talos::Telemetry::CreateFrame(
            builder, static_cast<std::uint64_t>(node_.now().nanos()), sequence_,
            &chassis, /*ghosts=*/0, channel_vector, /*targets=*/0,
            /*elevator_m=*/0.0, /*arm_rad=*/0.0, /*camera=*/nullptr));
    // A dropped frame still spends its sequence id, so the gap shows
    // downstream.
    studio_.send(std::as_bytes(
        std::span{builder.GetBufferPointer(), builder.GetSize()}));
    ++sequence_;
  }

 private:
  template <class M>
  bool Fresh(const Subscription<M>& source) const {
    return source.has_value() &&
           (stale_after_ == Duration::zero() || source.age() <= stale_after_);
  }

  Node& node_;
  Duration stale_after_;
  Subscription<::odometry::OdometryState> odometry_;
  Subscription<::drivetrain::DrivetrainState> drivetrain_;
  Subscription<::driver_station::DriverStationState> driver_station_;
  Subscription<::shooter::ShooterState> shooter_;
  Subscription<::intake::IntakeState> intake_;
  BytePublisher studio_;
  std::uint64_t sequence_ = 0;
};

}  // namespace talos::telemetry
