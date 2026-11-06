#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "drivetrain/drive_message_generated.h"
#include "drivetrain/topics.h"
#include "geometry/geometry_generated.h"
#include "talOS/node/node.h"
#include "talOS/vision/capture/avfoundation_source.h"
#include "talOS/vision/field_sim/field_layout_loader.h"
#include "talOS/vision/field_sim/tag_patterns.h"
#include "talOS/vision/field_sim/tag_rasterizer.h"
#include "talOS/vision/pipeline.h"
#include "vision/data/field_layout_2026.h"
#include "vision/sim_truth.h"
#include "vision/vision_generated.h"

namespace vision {

// One calibrated camera, one observation topic and one bounded live worker.
// Construction only registers/configures: describing never opens hardware.
class Vision {
 public:
  explicit Vision(talos::Node& node)
      : node_(node),
        simulated_(node.simulation()),
        rasterize_(node.param("rasterize", true)),
        tag_size_(node.param("tag_size_m", 0.1651)),
        base_xy_(node.param("base_xy_std_m", 0.08)),
        base_yaw_(node.param("base_yaw_std_rad", 0.06)),
        camera_id_(node.param<int>("camera.id")),
        device_(node.param<std::string>("camera.device", "")),
        truth_(node.param("initial_x_m", 2.0), node.param("initial_y_m", 2.0),
               node.param("initial_yaw_rad", 0.0)),
        drivetrain_(node.subscribe<drivetrain::DrivetrainState>(
            drivetrain::topics::kState,
            [this](const drivetrain::DrivetrainState& state) {
              if (simulated_) truth_.Observe(state);
            },
            {.optional = true})),
        state_(node.publish<geometry::PoseObservation>("state")),
        status_(node.publish<VisionStatus>("status")) {
    std::string error;
    if (!talos::vision::field_sim::LoadFieldLayoutFromJson(
            kFieldLayoutJson, tag_size_, &layout_, &error)) {
      throw std::runtime_error("vision layout: " + error);
    }
    LoadCamera();
    rng_.seed(camera_.rng_seed);
    talos::vision::PipelineConfig config;
    config.backend_name = node.param<std::string>("backend", "auto");
    config.intrinsics = camera_.intrinsics;
    config.mounting = camera_.mounting;
    config.layout = layout_.layout;
    if (!simulated_ || rasterize_) {
      pipeline_ = std::make_unique<talos::vision::AprilTagPipeline>(config);
    }
    if (simulated_ && rasterize_) {
      if (!talos::vision::field_sim::LoadTag36h11Patterns(&patterns_, &error)) {
        throw std::runtime_error("vision tag patterns: " + error);
      }
      pixels_ = std::make_shared<std::vector<std::uint8_t>>(
          static_cast<std::size_t>(camera_.intrinsics.image_width) *
          camera_.intrinsics.image_height);
    }
    sim_tags_.reserve(256);
    detections_.reserve(256);
    if (!simulated_) worker_ = std::thread([this] { Work(); });
  }

  ~Vision() {
    {
      std::lock_guard lock(mutex_);
      stopping_.store(true, std::memory_order_release);
    }
    wake_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

  bool AcceptEstimate(const talos::vision::RobotPoseEstimate& estimate) const {
    const auto& t = estimate.T_field_robot.translation();
    return estimate.validity.valid && t.allFinite() &&
           estimate.T_field_robot.rotation().rotationMatrix().allFinite() &&
           std::isfinite(estimate.reprojection_error_px) &&
           estimate.reprojection_error_px <= 3.5 && t.x() >= 0 && t.y() >= 0 &&
           t.x() <= layout_.dims.length_m && t.y() <= layout_.dims.width_m;
  }

  // Compact in place without releasing reserved storage. Invalid detections
  // count as neither visible tags nor range rejects.
  static std::uint32_t FilterDetections(
      std::vector<talos::vision::TagDetection3D>& detections,
      double max_range_m) {
    std::uint32_t rejected = 0;
    std::size_t accepted = 0;
    for (std::size_t i = 0; i < detections.size(); ++i) {
      const auto& detection = detections[i];
      if (!detection.valid) continue;
      const double range = detection.T_camera_tag.translation().norm();
      if (!std::isfinite(range) || range > max_range_m) {
        ++rejected;
        continue;
      }
      if (accepted != i) detections[accepted] = std::move(detections[i]);
      ++accepted;
    }
    detections.resize(accepted);
    return rejected;
  }

  void update() {
    const auto now = node_.now();
    std::optional<Completed> completed;
    if (simulated_) {
      completed = Simulate(now);
      ++captured_;
      RecordProcessed(completed->capture_ns);
    } else {
      worker_now_ns_.store(now.nanos(), std::memory_order_release);
      wake_.notify_one();
      std::unique_lock lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        completed = std::move(completed_);
        completed_.reset();
        captured_ = received_;
        processed_ = worker_processed_;
        dropped_ = worker_dropped_;
        rejected_range_ = worker_rejected_range_;
        frame_period_us_ = worker_period_us_;
      }
    }
    if (completed) {
      last_latency_us_ =
          static_cast<std::uint64_t>(std::max<std::int64_t>(
              0,
              now.nanos() - static_cast<std::int64_t>(completed->capture_ns))) /
          1000;
      visible_ = completed->visible;
      healthy_ = completed->success;
      PublishObservation(*completed);
    }
    const bool healthy =
        healthy_ && (simulated_ || connected_.load(std::memory_order_acquire));
    status_.send(VisionStatus{captured_, processed_, dropped_, frame_period_us_,
                              last_latency_us_, rejected_range_, visible_,
                              healthy});
  }

 private:
  struct Completed {
    explicit Completed(std::uint64_t stamp = 0)
        : observation(stamp, geometry::Pose2d{}, 0, 0, 0, 0, false),
          capture_ns(stamp) {}
    geometry::PoseObservation observation;
    std::uint64_t capture_ns = 0;
    std::uint32_t visible = 0;
    std::uint32_t rejected_range = 0;
    bool success = false;
  };
  static_assert(std::is_trivially_copyable_v<Completed>);

  void LoadCamera() {
    auto number = [&](const char* key, double fallback) {
      return node_.param(std::string("camera.") + key, fallback);
    };
    camera_.mounting = talos::vision::CameraMounting::FromExtrinsics(
        number("x_m", 0), number("y_m", 0), number("z_m", 0),
        number("roll_rad", 0), number("pitch_rad", 0), number("yaw_rad", 0));
    auto& intr = camera_.intrinsics;
    intr.fx = number("fx", 0);
    intr.fy = number("fy", 0);
    intr.cx = number("cx", 0);
    intr.cy = number("cy", 0);
    intr.k1 = number("k1", 0);
    intr.k2 = number("k2", 0);
    intr.k3 = number("k3", 0);
    intr.p1 = number("p1", 0);
    intr.p2 = number("p2", 0);
    const int width = node_.param<int>("camera.width", 1280);
    const int height = node_.param<int>("camera.height", 720);
    intr.image_width = width;
    intr.image_height = height;
    camera_.max_range_m = number("max_range_m", 8);
    camera_.min_tag_px = number("min_tag_px", 12);
    camera_.facing_min = number("facing_min", 0.2);
    camera_.pixel_noise_std = number("pixel_noise_std", 0);
    camera_.sensor_noise_std_gray = number("sensor_noise_std_gray", 0);
    camera_.exposure_gain_std = number("exposure_gain_std", 0);
    camera_.exposure_ms = number("exposure_ms", 0);
    const auto seed = node_.param<std::int64_t>("camera.rng_seed", 0);
    camera_.rng_seed = seed;
    if (camera_id_ < 0 || camera_id_ > 31 || width < 160 || width > 4096 ||
        height < 120 || height > 2160 || !intr.IsValid() ||
        !std::isfinite(camera_.max_range_m) || !(camera_.max_range_m > 0) ||
        !std::isfinite(camera_.min_tag_px) || !(camera_.min_tag_px >= 0) ||
        !(camera_.facing_min >= 0) || !(camera_.facing_min <= 1) ||
        !std::isfinite(camera_.pixel_noise_std) ||
        !(camera_.pixel_noise_std >= 0) ||
        !std::isfinite(camera_.sensor_noise_std_gray) ||
        !(camera_.sensor_noise_std_gray >= 0) ||
        !std::isfinite(camera_.exposure_gain_std) ||
        !(camera_.exposure_gain_std >= 0) ||
        !std::isfinite(camera_.exposure_ms) || !(camera_.exposure_ms >= 0) ||
        seed < 0 || !camera_.mounting.T_robot_camera.matrix().allFinite()) {
      throw std::runtime_error("invalid vision camera: " +
                               std::string(node_.name()));
    }
  }

  Completed Localize(std::vector<talos::vision::TagDetection3D>& detections,
                     std::uint64_t capture_ns, bool success) {
    Completed result{capture_ns};
    result.success = success;
    result.rejected_range = FilterDetections(detections, camera_.max_range_m);
    result.visible = static_cast<std::uint32_t>(detections.size());
    const auto estimate = talos::vision::SolveMultiTagRobotPose(
        detections, camera_id_, camera_.mounting, layout_.layout,
        camera_.intrinsics);
    result.observation = MakeObservation(estimate, capture_ns, success);
    return result;
  }

  Completed Process(const talos::vision::Frame& frame) {
    auto result = pipeline_->ProcessFrame(frame);
    return Localize(result.detections, frame.capture_timestamp_ns(),
                    result.success);
  }

  Completed Simulate(talos::Time now) {
    namespace fs = talos::vision::field_sim;
    Completed result;
    if (rasterize_) {
      fs::RenderFieldView(layout_.layout, patterns_, camera_, truth_.pose(),
                          tag_size_, {}, pixels_.get(),
                          truth_.velocity_field_mps(), truth_.yaw_rate_radps(),
                          &rng_, &render_scratch_);
      const auto& intr = camera_.intrinsics;
      const talos::vision::Frame frame(
          camera_id_, 1, captured_, now, intr.image_width, intr.image_height,
          intr.image_width, talos::vision::PixelFormat::kGray8,
          std::shared_ptr<const std::vector<std::uint8_t>>(pixels_));
      result = Process(frame);
    } else {
      detections_.clear();
      fs::SimulateVisibleTags(layout_.layout, camera_, truth_.pose(), tag_size_,
                              &sim_tags_, false, &rng_);
      for (const auto& tag : sim_tags_) {
        talos::vision::TagDetection3D detection;
        detection.valid = true;
        detection.tag_id = tag.tag_id;
        detection.T_camera_tag = tag.T_camera_tag;
        detection.ambiguity = tag.ambiguity;
        for (int c = 0; c < 4; ++c) {
          detection.corners[2 * c] = tag.corners[c].x();
          detection.corners[2 * c + 1] = tag.corners[c].y();
        }
        detections_.push_back(detection);
      }
      result = Localize(detections_, now.nanos(), true);
    }
    rejected_range_ += result.rejected_range;
    return result;
  }

  void RecordProcessed(std::uint64_t capture_ns) {
    if (processed_ == 0) first_capture_ns_ = capture_ns;
    ++processed_;
    frame_period_us_ =
        processed_ > 1 && capture_ns >= first_capture_ns_
            ? (capture_ns - first_capture_ns_) / (processed_ - 1) / 1000
            : 0;
  }

  geometry::PoseObservation MakeObservation(
      const talos::vision::RobotPoseEstimate& estimate,
      std::uint64_t capture_ns, bool success) const {
    geometry::PoseObservation observation{
        capture_ns, geometry::Pose2d{}, 0, 0, 0, 0, false};
    if (success && AcceptEstimate(estimate)) {
      const auto& t = estimate.T_field_robot.translation();
      const auto r = estimate.T_field_robot.rotation().rotationMatrix();
      const auto stds = talos::vision::ComputeObservationStdDevs(
          estimate.average_distance_m, estimate.tag_count, estimate.ambiguity,
          base_xy_, base_yaw_);
      observation = geometry::PoseObservation{
          capture_ns,
          geometry::Pose2d{t.x(), t.y(), std::atan2(r(1, 0), r(0, 0))},
          stds.x_std_m,
          stds.y_std_m,
          stds.yaw_std_rad,
          static_cast<std::uint32_t>(estimate.tag_count),
          true};
    }
    return observation;
  }

  void PublishObservation(const Completed& completed) {
    state_.send(completed.observation);
  }

  // Only the worker owns camera lifecycle calls. Retry timing comes from the
  // node clock supplied by update(), never a wall-clock timer.
  bool OpenCamera() {
    talos::vision::CameraSourceConfig config;
    config.camera_id = camera_id_;
    config.device_path = device_;
    config.width = camera_.intrinsics.image_width;
    config.height = camera_.intrinsics.image_height;
    source_ = talos::vision::CreatePlatformCameraSource(config);
    if (!source_) return false;
    source_->SetFrameCallback([this](const talos::vision::Frame& frame) {
      {
        std::lock_guard lock(mutex_);
        if (stopping_.load(std::memory_order_acquire)) return;
        ++received_;
        if (pending_) ++worker_dropped_;
        pending_ = frame;
      }
      wake_.notify_one();
    });
    return source_->Start() && source_->IsRunning() && source_->IsConnected();
  }

  void Work() {
    std::int64_t serviced_ns = -1;
    std::int64_t retry_at_ns = 0;
    std::int64_t retry_delay_ns = 1000000000;
    while (!stopping_.load(std::memory_order_acquire)) {
      std::optional<talos::vision::Frame> frame;
      {
        std::unique_lock lock(mutex_);
        wake_.wait(lock, [this, serviced_ns] {
          return stopping_.load(std::memory_order_acquire) || pending_ ||
                 worker_now_ns_.load(std::memory_order_acquire) != serviced_ns;
        });
        if (stopping_.load(std::memory_order_acquire)) break;
        serviced_ns = worker_now_ns_.load(std::memory_order_acquire);
        if (serviced_ns < 0) continue;
        frame = std::move(pending_);
        pending_.reset();
      }
      if (!source_ || !source_->IsRunning() || !source_->IsConnected()) {
        const bool was_connected =
            connected_.exchange(false, std::memory_order_acq_rel);
        if (source_) {
          source_->Stop();
          source_.reset();
        }
        // Frames from the retired capture session cannot survive a reconnect.
        {
          std::lock_guard lock(mutex_);
          if (pending_) {
            ++worker_dropped_;
            pending_.reset();
          }
          if (frame) {
            ++worker_dropped_;
            frame.reset();
          }
          if (was_connected) {
            completed_ = Completed{static_cast<std::uint64_t>(serviced_ns)};
          }
        }
        if (serviced_ns < retry_at_ns) continue;
        bool opened = false;
        try {
          opened = OpenCamera();
        } catch (const std::exception&) {
          opened = false;
        }
        connected_.store(opened, std::memory_order_release);
        retry_at_ns = serviced_ns + retry_delay_ns;
        retry_delay_ns = std::min<std::int64_t>(retry_delay_ns * 2, 8000000000);
        if (!opened) {
          if (source_) {
            source_->Stop();
            source_.reset();
          }
          std::lock_guard lock(mutex_);
          completed_ = Completed{static_cast<std::uint64_t>(serviced_ns)};
        }
        continue;
      }
      if (!frame) continue;
      Completed result;
      try {
        result = Process(*frame);
      } catch (const std::exception&) {
        result = Completed{
            static_cast<std::uint64_t>(frame->capture_timestamp_ns())};
      }
      retry_delay_ns = 1000000000;
      retry_at_ns = serviced_ns + retry_delay_ns;
      {
        std::lock_guard lock(mutex_);
        if (completed_) ++worker_dropped_;
        if (worker_processed_ == 0)
          worker_first_capture_ns_ = result.capture_ns;
        ++worker_processed_;
        worker_period_us_ =
            worker_processed_ > 1 &&
                    result.capture_ns >= worker_first_capture_ns_
                ? (result.capture_ns - worker_first_capture_ns_) /
                      (worker_processed_ - 1) / 1000
                : 0;
        worker_rejected_range_ += result.rejected_range;
        completed_ = std::move(result);
      }
    }
    connected_.store(false, std::memory_order_release);
    if (source_) {
      source_->Stop();
      source_.reset();
    }
  }

  talos::Node& node_;
  bool simulated_, rasterize_;
  double tag_size_, base_xy_, base_yaw_;
  int camera_id_;
  std::string device_;
  SimTruth truth_;
  talos::Subscription<drivetrain::DrivetrainState> drivetrain_;
  talos::Publisher<geometry::PoseObservation> state_;
  talos::Publisher<VisionStatus> status_;
  talos::vision::field_sim::FieldLayoutData layout_;
  talos::vision::field_sim::SimCameraView camera_;
  std::mt19937_64 rng_;
  std::map<int, talos::vision::field_sim::TagPattern> patterns_;
  std::shared_ptr<std::vector<std::uint8_t>> pixels_;
  talos::vision::field_sim::RenderScratch render_scratch_;
  std::vector<talos::vision::field_sim::SimTagDetection> sim_tags_;
  std::vector<talos::vision::TagDetection3D> detections_;
  std::unique_ptr<talos::vision::AprilTagPipeline> pipeline_;
  std::unique_ptr<talos::vision::CameraSource> source_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<talos::vision::Frame> pending_;
  std::optional<Completed> completed_;
  bool healthy_ = false;
  std::atomic<bool> stopping_{false}, connected_{false};
  std::atomic<std::int64_t> worker_now_ns_{-1};
  std::uint64_t received_ = 0, worker_processed_ = 0, worker_dropped_ = 0;
  std::uint64_t worker_first_capture_ns_ = 0, worker_period_us_ = 0,
                worker_rejected_range_ = 0;
  std::uint64_t captured_ = 0, processed_ = 0, dropped_ = 0,
                rejected_range_ = 0;
  std::uint64_t first_capture_ns_ = 0, frame_period_us_ = 0,
                last_latency_us_ = 0;
  std::uint32_t visible_ = 0;
};

}  // namespace vision
