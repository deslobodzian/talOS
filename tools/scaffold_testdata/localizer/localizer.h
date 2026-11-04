#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "geometry/geometry_generated.h"
#include "localizer/topics.h"
#include "talOS/node/node.h"

namespace localizer {

// A pose estimator.
// Reads geometry.PoseObservation from each camera node listed in `cameras`
// (/<camera>/state) and publishes geometry.PoseEstimate on its own state
// topic every tick. As generated it follows the newest usable observation
// (the most certain one when several arrive in one tick) and is valid while
// one arrived within observation_timeout_us; fuse odometry here (see
// 2026-robot/main_processor/pose_estimator, which runs wpimath's
// SwerveDrive4PoseEstimator or a GTSAM smoother). It drives nothing, so it
// runs whether the robot is enabled or not.
class Localizer {
 public:
  using State = geometry::PoseEstimate;
  using Observation = geometry::PoseObservation;

  explicit Localizer(talos::Node& node)
      : node_{node},
        timeout_{std::chrono::microseconds{
            node.param<std::int64_t>("observation_timeout_us", 500'000)}},
        state_{node.publish<State>(topics::kState)} {
    for (const std::string& camera :
         node.param<std::vector<std::string>>("cameras")) {
      cameras_.push_back(node.subscribe<Observation>("/" + camera + "/state",
                                                     {.optional = true}));
    }
  }

  void update() {
    const Observation* best = nullptr;
    for (const auto& camera : cameras_) {
      if (!camera.updated()) continue;  // each observation counts once
      const Observation& o = camera.value();
      if (!Usable(o)) {
        ++rejected_;
        continue;
      }
      ++accepted_;
      if (best == nullptr ||
          o.x_std_m() + o.y_std_m() < best->x_std_m() + best->y_std_m()) {
        best = &o;
      }
    }
    if (best != nullptr) {
      pose_ = best->pose();
      sample_time_ns_ = best->sample_time_ns();
      last_accepted_ = node_.now();
    }
    const bool valid =
        last_accepted_.has_value() && node_.now() - *last_accepted_ <= timeout_;
    state_.send(State{sample_time_ns_, pose_, geometry::Twist2d{}, valid,
                      accepted_, rejected_});
  }

 private:
  static bool Usable(const Observation& o) {
    for (const double v : {o.pose().x_m(), o.pose().y_m(), o.pose().yaw_rad(),
                           o.x_std_m(), o.y_std_m(), o.yaw_std_rad()}) {
      if (!std::isfinite(v)) return false;
    }
    return o.valid() && o.sample_time_ns() > 0 && o.x_std_m() > 0 &&
           o.y_std_m() > 0 && o.yaw_std_rad() > 0;
  }

  talos::Node& node_;
  talos::Duration timeout_;
  std::vector<talos::Subscription<Observation>> cameras_;
  talos::Publisher<State> state_;
  geometry::Pose2d pose_{};  // held while no usable observation arrives
  std::uint64_t sample_time_ns_{0};
  std::optional<talos::Time> last_accepted_;
  std::uint32_t accepted_{0};
  std::uint32_t rejected_{0};
};

}  // namespace localizer
