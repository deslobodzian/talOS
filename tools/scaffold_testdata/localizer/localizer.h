#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry/geometry_generated.h"
#include "localizer/topics.h"
#include "talOS/node/node.h"

namespace localizer {

// A pose estimator.
// Reads geometry.PoseObservation from each camera node listed in `cameras`
// (/<camera>/state) and publishes geometry.PoseEstimate on its own state
// topic every tick. Each camera is read through a callback subscription,
// which delivers every observation in sequence (a latest-value read would lose
// all but the newest per tick); the callback queues at most 64 per camera (the
// oldest is dropped and counted as rejected) and update() checks every queued
// one. As generated it follows the most certain usable observation of the tick
// and is valid while one arrived within observation_timeout_us; fuse odometry
// here (see
// 2026-robot/main_processor/pose_estimator, which runs wpimath's
// SwerveDrive4PoseEstimator or a GTSAM smoother). It drives nothing, so it
// runs whether the robot is enabled or not.
class Localizer {
 public:
  using State = geometry::PoseEstimate;
  using Observation = geometry::PoseObservation;
  // Observations queued per camera between two update()s: the framework's
  // per-topic, per-drain delivery cap, over a second of a 60 fps camera.
  static constexpr std::size_t kMaxPendingObservations = 64;

  // The camera callbacks capture `this`: the node stays where it was built.
  Localizer(const Localizer&) = delete;
  Localizer& operator=(const Localizer&) = delete;
  Localizer(Localizer&&) = delete;
  Localizer& operator=(Localizer&&) = delete;

  explicit Localizer(talos::Node& node)
      : node_{node},
        timeout_{std::chrono::microseconds{
            node.param<std::int64_t>("observation_timeout_us", 500'000)}},
        state_{node.publish<State>(topics::kState)} {
    if (timeout_ <= talos::Duration::zero()) {
      throw std::invalid_argument(
          "localizer: observation_timeout_us must be positive");
    }
    const auto names = node.param<std::vector<std::string>>("cameras");
    pending_.resize(names.size());
    for (std::size_t i = 0; i < names.size(); ++i) {
      // A callback sees every observation in sequence; update() reads them.
      cameras_.push_back(node.subscribe<Observation>(
          "/" + names[i] + "/state",
          [this, i](const Observation& o) { pending_[i].Push(o, rejected_); },
          {.optional = true}));
    }
  }

  void update() {
    std::optional<Observation> best;
    for (Pending& queue : pending_) {
      for (std::size_t k = 0; k < queue.count; ++k) {  // each counts once
        const Observation& o = queue.At(k);
        if (!Usable(o)) {
          ++rejected_;
          continue;
        }
        ++accepted_;
        if (!best.has_value() ||
            o.x_std_m() + o.y_std_m() < best->x_std_m() + best->y_std_m()) {
          best = o;
        }
      }
      queue.count = 0;
    }
    if (best.has_value()) {
      pose_ = best->pose();
      sample_time_ns_ = best->sample_time_ns();
      last_accepted_ = node_.now();
    }
    const bool valid =
        last_accepted_.has_value() && node_.now() - *last_accepted_ <= timeout_;
    state_.send(State{sample_time_ns_, pose_, geometry::Twist2d{}, valid, false,
                      accepted_, rejected_});
  }

 private:
  // A fixed ring of the newest observations, oldest first: no allocation
  // after construction.
  struct Pending {
    std::array<Observation, kMaxPendingObservations> slots{};
    std::size_t head = 0;  // index of the oldest
    std::size_t count = 0;

    const Observation& At(std::size_t k) const {
      return slots[(head + k) % kMaxPendingObservations];
    }
    void Push(const Observation& o, std::uint32_t& rejected) {
      if (count == kMaxPendingObservations) {  // drop the oldest
        head = (head + 1) % kMaxPendingObservations;
        --count;
        ++rejected;
      }
      slots[(head + count) % kMaxPendingObservations] = o;
      ++count;
    }
  };

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
  std::vector<Pending> pending_;  // per camera, in sequence
  talos::Publisher<State> state_;
  geometry::Pose2d pose_{};  // held while no usable observation arrives
  std::uint64_t sample_time_ns_{0};
  std::optional<talos::Time> last_accepted_;
  std::uint32_t accepted_{0};
  std::uint32_t rejected_{0};
};

}  // namespace localizer
