#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "geometry/geometry_generated.h"
#include "sim/topics.h"
#include "talOS/node/node.h"
#include "talOS/vision/capture/avfoundation_source.h"
#include "talOS/vision/field_sim/field_layout_loader.h"
#include "talOS/vision/field_sim/tag_patterns.h"
#include "talOS/vision/field_sim/tag_rasterizer.h"
#include "talOS/vision/pipeline.h"
#include "talOS/vision/recording/recorder.h"
#include "talOS/vision/recording/recording.h"
#include "vision/data/field_layout_2026.h"
#include "vision/vision_generated.h"

namespace vision {

// One calibrated camera, one observation topic and one bounded live worker.
// Construction only registers/configures: describing never opens hardware.
class Vision {
 public:
  using SourceFactory =
      std::function<std::unique_ptr<talos::vision::CameraSource>(
          const talos::vision::CameraSourceConfig&)>;
  explicit Vision(talos::Node& node, SourceFactory factory = {})
      : node_(node),
        simulated_(node.simulation() && !factory),
        rasterize_(node.param("rasterize", true)),
        tag_size_(node.param("tag_size_m", 0.1651)),
        camera_id_(node.param<int>("camera.id")),
        device_(node.param<std::string>("camera.device", "")),
        state_(node.publish<geometry::PoseObservation>("state")),
        status_(node.publish<VisionStatus>("status")) {
    source_factory_ = factory ? std::move(factory)
                              : talos::vision::CreatePlatformCameraSource;
    if (simulated_) {
      truth_.emplace(node.subscribe<geometry::OdometrySample>(
          sim::topics::kState, {.optional = true}));
    }
    if (!simulated_ && device_.empty()) {
      std::fprintf(stderr,
                   "%.*s: camera.device must be set to a stable device id "
                   "(AVFoundation uniqueID on macOS, /dev/v4l/by-id/... on "
                   "Linux); live capture disabled\n",
                   static_cast<int>(node_.name().size()), node_.name().data());
    }
    std::string error;
    if (!talos::vision::field_sim::LoadFieldLayoutFromJson(
            kFieldLayoutJson, tag_size_, &layout_, &error)) {
      throw std::runtime_error("vision layout: " + error);
    }
    LoadCamera();
    if (simulated_ && sim_timestamp_lag_ns_ > 0) {
      // Reserve the capture-rate lag window plus boundary/scheduling margin.
      // The node/status tick may be much slower or faster than camera frames.
      constexpr std::size_t kMargin = 2;
      const auto frames =
          std::ceil(static_cast<long double>(sim_timestamp_lag_ns_) *
                    capture_config_.fps / 1.0e9L);
      if (frames > sim_pending_.max_size() - kMargin) {
        throw std::invalid_argument(
            std::string(node_.name()) +
            ": camera.fps requires too many delayed frames");
      }
      sim_pending_.resize(static_cast<std::size_t>(frames) + kMargin);
    }
    solver_options_.pixel_std_px =
        node.param("pixel_std_px", solver_options_.pixel_std_px);
    solver_options_.floor_z_std_m =
        node.param("floor_z_std_m", solver_options_.floor_z_std_m);
    solver_options_.floor_tilt_std_rad =
        node.param("floor_tilt_std_rad", solver_options_.floor_tilt_std_rad);
    solver_options_.huber_delta_px =
        node.param("huber_delta_px", solver_options_.huber_delta_px);
    solver_options_.min_xy_std_m =
        node.param("min_xy_std_m", solver_options_.min_xy_std_m);
    solver_options_.min_yaw_std_rad =
        node.param("min_yaw_std_rad", solver_options_.min_yaw_std_rad);
    rng_.seed(camera_.rng_seed);
    talos::vision::PipelineConfig config;
    config.backend_name = node.param<std::string>("backend", "auto");
    config.detector_options.quad_decimate =
        node.param<float>("detector.decimate", 1.0f);
    config.detector_options.threads = node.param<int>("detector.threads", 1);
    config.detector_options.quad_sigma =
        node.param<float>("detector.sigma", 0.0f);
    config.detector_options.refine_edges =
        node.param<bool>("detector.refine_edges", true);
    config.detector_options.max_hamming =
        node.param<int>("detector.hamming_threshold", 1);
    config.min_decision_margin =
        node.param<double>("detector.min_decision_margin", 30.0);
    config.intrinsics = camera_.intrinsics;
    config.mounting = camera_.mounting;
    config.layout = layout_.layout;
    config.validation_options = validation_options_;
    try {
      pipeline_ = std::make_unique<talos::vision::AprilTagPipeline>(config);
    } catch (const std::invalid_argument& error) {
      throw std::invalid_argument(std::string(node.name()) + ": " +
                                  error.what());
    }
    recording_enabled_ = node.param("recording.enabled", true);
    recording_sample_hz_ = node.param("recording.sample_hz", 1.0);
    recording_rejected_hz_ = node.param("recording.rejected_hz", 1.0);
    const auto recording_max_bytes =
        node.param<std::int64_t>("recording.max_bytes", 268435456);
    if (!std::isfinite(recording_sample_hz_) || recording_sample_hz_ < 0 ||
        !std::isfinite(recording_rejected_hz_) || recording_rejected_hz_ < 0) {
      throw std::invalid_argument(std::string(node.name()) +
                                  ": recording.sample_hz and "
                                  "recording.rejected_hz must be finite and "
                                  "nonnegative");
    }
    if (recording_max_bytes < 0) {
      throw std::invalid_argument(std::string(node.name()) +
                                  ": recording.max_bytes must be nonnegative");
    }
    recording_max_bytes_ = static_cast<std::uint64_t>(recording_max_bytes);
    if (recording_enabled_) {
      const auto session = node.session_dir();
      if (session.has_value()) {
        talos::vision::recording::FrameRecorder::Options recorder_options;
        recorder_options.output_dir =
            *session / "frames" / std::string{node.name()};
        recorder_options.sample_hz = recording_sample_hz_;
        recorder_options.rejected_hz = recording_rejected_hz_;
        recorder_options.max_bytes = recording_max_bytes_;
        recorder_options.context.camera = std::string{node.name()};
        recorder_options.context.width =
            static_cast<int>(camera_.intrinsics.image_width);
        recorder_options.context.height =
            static_cast<int>(camera_.intrinsics.image_height);
        recorder_options.context.intrinsics = camera_.intrinsics;
        recorder_options.context.mounting = extrinsics_;
        recorder_options.context.detector = {
            config.backend_name,
            static_cast<double>(config.detector_options.quad_decimate),
            config.detector_options.threads,
            static_cast<double>(config.detector_options.quad_sigma),
            config.detector_options.refine_edges,
            config.detector_options.max_hamming,
            config.min_decision_margin};
        try {
          recorder_ = std::make_unique<talos::vision::recording::FrameRecorder>(
              std::move(recorder_options));
        } catch (const std::exception& error) {
          std::fprintf(stderr, "%s: recording disabled: %s\n",
                       std::string{node.name()}.c_str(), error.what());
        }
      }
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
    if (CanCapture(simulated_, device_)) {
      worker_ = std::thread([this] { Work(); });
    }
  }

  ~Vision() {
    {
      std::lock_guard lock(mutex_);
      stopping_.store(true, std::memory_order_release);
    }
    wake_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

  static bool CanCapture(bool simulated, std::string_view device) {
    return !simulated && !device.empty();
  }

  // Shared by live capture and simulated truth stamps. Check before unsigned
  // arithmetic: a negative corrected time must not wrap into a future sample.
  static std::optional<std::uint64_t> CorrectCaptureTimestamp(
      std::uint64_t capture_ns, std::uint64_t offset_ns,
      std::uint64_t lag_ns = 0) {
    if (capture_ns > std::numeric_limits<std::uint64_t>::max() - lag_ns)
      return std::nullopt;
    const auto stamped_ns = capture_ns + lag_ns;
    if (stamped_ns < offset_ns) return std::nullopt;
    return stamped_ns - offset_ns;
  }

  const talos::vision::CameraSourceConfig& capture_config() const {
    return capture_config_;
  }

  bool AcceptEstimate(const talos::vision::RobotPoseEstimate& estimate) const {
    const auto& t = estimate.T_field_robot.translation();
    return estimate.validity.valid && t.allFinite() &&
           estimate.T_field_robot.rotation().rotationMatrix().allFinite() &&
           std::isfinite(estimate.reprojection_error_px) &&
           estimate.reprojection_error_px <=
               validation_options_.max_reprojection_error_px &&
           t.x() >= 0 && t.y() >= 0 && t.x() <= layout_.dims.length_m &&
           t.y() <= layout_.dims.width_m;
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
    const auto now_ns = static_cast<std::uint64_t>(now.nanos());
    std::optional<Completed> completed;
    // truth_ exists exactly when simulated_.
    if (truth_.has_value()) {
      if (truth_->fresh()) {
        truth_outage_ = false;
        // Deliver ready frames before rendering or admitting a new capture.
        DrainSimPending(now_ns);
        const auto& truth = truth_->value();
        const auto stamp = truth.sample_time_ns();
        if (SimCaptureDue(stamp)) {
          auto capture = Simulate(truth);
          ++captured_;
          RecordProcessed(capture.capture_ns);
          if (sim_pending_.empty()) {
            completed = capture;
          } else {
            if (sim_pending_count_ == sim_pending_.size()) {
              sim_pending_head_ = (sim_pending_head_ + 1) % sim_pending_.size();
              --sim_pending_count_;
              ++dropped_;
            }
            sim_pending_[(sim_pending_head_ + sim_pending_count_) %
                         sim_pending_.size()] = capture;
            ++sim_pending_count_;
          }
        }
        DrainSimPending(now_ns);
      } else {
        sim_pending_count_ = 0;
        completed = Completed{
            truth_->has_value() ? truth_->value().sample_time_ns() : 0};
        if (!truth_outage_) {
          std::fprintf(
              stderr,
              "%.*s: no fresh /sim/state; simulated cameras need "
              "sim_gateway --physics (WPILib sim publishes no truth)\n",
              static_cast<int>(node_.name().size()), node_.name().data());
          truth_outage_ = true;
        }
      }
    } else if (!CanCapture(simulated_, device_)) {
      completed = Completed{static_cast<std::uint64_t>(now.nanos())};
    } else {
      worker_now_ns_.store(now.nanos(), std::memory_order_release);
      wake_.notify_one();
      std::unique_lock lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        completed = completed_;
        completed_.reset();
        captured_ = received_;
        processed_ = worker_processed_;
        dropped_ = worker_dropped_;
        rejected_range_ = worker_rejected_range_;
        frame_period_us_ = worker_period_us_;
      }
    }
    if (completed) PublishObservation(*completed, now_ns);
    const bool healthy =
        healthy_ && (simulated_ || connected_.load(std::memory_order_acquire));
    status_.send(VisionStatus{
        captured_, processed_, dropped_, frame_period_us_, last_latency_us_,
        rejected_range_, visible_, healthy, reconnects_.load(), stalls_.load(),
        control_failures_.load(), controls_applied_.load()});
  }

  static_assert(sizeof(VisionStatus) == 88);

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
    talos::vision::TimestampKind timestamp_kind =
        talos::vision::TimestampKind::kExposure;
  };
  static_assert(std::is_trivially_copyable_v<Completed>);

  // What the recorder needs per frame beyond the detections already produced:
  // the gray frame (nullptr when there is none, e.g. the non-raster sim path)
  // and, under --sim, the /sim/state pose the frame was rendered from.
  struct RecordingFeed {
    RecordingFeed() : gray{nullptr} {}
    RecordingFeed(const talos::vision::Frame* g,
                  std::optional<talos::vision::recording::TruthRecord> t)
        : gray{g}, truth{t} {}
    const talos::vision::Frame* gray;
    std::optional<talos::vision::recording::TruthRecord> truth;
  };

  bool SimCaptureDue(std::uint64_t stamp) {
    if (!first_sim_capture_ns_) {
      first_sim_capture_ns_ = stamp;
      last_sim_capture_ns_ = stamp;
      return true;
    }
    if (stamp <= last_sim_capture_ns_) return false;
    // Keep frame deadlines anchored to the first truth sample, rather than
    // restarting a full frame period after each quantized node tick. Compute
    // the phase directly to avoid accumulating fractional-period rounding.
    const auto frame =
        std::floor(static_cast<long double>(stamp - *first_sim_capture_ns_) *
                   capture_config_.fps / 1.0e9L);
    if (frame < next_sim_capture_frame_) return false;
    const auto skipped = frame - next_sim_capture_frame_;
    // A slow tick captures only its latest truth, never synthesized frames.
    // Saturate only when the count cannot be represented by the wire counter.
    dropped_ += static_cast<std::uint64_t>(std::min(
        skipped, static_cast<long double>(
                     std::numeric_limits<std::uint64_t>::max() - dropped_)));
    next_sim_capture_frame_ = frame + 1;
    last_sim_capture_ns_ = stamp;
    return true;
  }

  void LoadCamera() {
    auto number = [&](const char* key, double fallback) {
      return node_.param(std::string("camera.") + key, fallback);
    };
    extrinsics_ = {number("x_m", 0),       number("y_m", 0),
                   number("z_m", 0),       number("roll_rad", 0),
                   number("pitch_rad", 0), number("yaw_rad", 0)};
    camera_.mounting = talos::vision::CameraMounting::FromExtrinsics(
        extrinsics_.x_m, extrinsics_.y_m, extrinsics_.z_m, extrinsics_.roll_rad,
        extrinsics_.pitch_rad, extrinsics_.yaw_rad);
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
    solver_options_.mount_angle_std_rad =
        number("mount_angle_std_rad", solver_options_.mount_angle_std_rad);
    solver_options_.mount_translation_std_m = number(
        "mount_translation_std_m", solver_options_.mount_translation_std_m);
    const auto seed = node_.param<std::int64_t>("camera.rng_seed", 0);
    camera_.rng_seed = seed;
    auto require = [&](bool valid, const char* key) {
      if (!valid) {
        throw std::runtime_error(std::string(node_.name()) + ": invalid " +
                                 key);
      }
    };
    auto duration_ns = [&](const char* key) {
      const auto value = number(key, 0);
      const auto qualified_key = std::string("camera.") + key;
      require(std::isfinite(value) && value >= 0 && value < 1.0e6,
              qualified_key.c_str());
      return static_cast<std::uint64_t>(value * 1000);
    };
    latency_offset_ns_ = duration_ns("latency_offset_us");
    sim_timestamp_lag_ns_ = duration_ns("sim_timestamp_lag_us");
    readout_ns_ = duration_ns("readout_us");
    const auto timeout_us = number("frame_timeout_us", 500000);
    require(
        std::isfinite(timeout_us) && timeout_us > 0 && timeout_us <= 10000000,
        "camera.frame_timeout_us");
    frame_timeout_ns_ = static_cast<std::int64_t>(timeout_us * 1000);
    require(camera_id_ >= 0 && camera_id_ <= 31, "camera.id");
    require(width >= 160 && width <= 4096, "camera.width");
    require(height >= 120 && height <= 2160, "camera.height");
    require(std::isfinite(solver_options_.mount_angle_std_rad) &&
                solver_options_.mount_angle_std_rad >= 0,
            "camera.mount_angle_std_rad");
    require(std::isfinite(solver_options_.mount_translation_std_m) &&
                solver_options_.mount_translation_std_m >= 0,
            "camera.mount_translation_std_m");
    if (!intr.IsValid() || !std::isfinite(camera_.max_range_m) ||
        !(camera_.max_range_m > 0) || !std::isfinite(camera_.min_tag_px) ||
        !(camera_.min_tag_px >= 0) || !(camera_.facing_min >= 0) ||
        !(camera_.facing_min <= 1) || !std::isfinite(camera_.pixel_noise_std) ||
        !(camera_.pixel_noise_std >= 0) ||
        !std::isfinite(camera_.sensor_noise_std_gray) ||
        !(camera_.sensor_noise_std_gray >= 0) ||
        !std::isfinite(camera_.exposure_gain_std) ||
        !(camera_.exposure_gain_std >= 0) ||
        !std::isfinite(camera_.exposure_ms) || !(camera_.exposure_ms >= 0) ||
        !(camera_.exposure_ms < 214748364) || seed < 0 ||
        !camera_.mounting.T_robot_camera.matrix().allFinite()) {
      throw std::runtime_error("invalid vision camera: " +
                               std::string(node_.name()));
    }
    capture_config_.camera_id = camera_id_;
    capture_config_.device_path = device_;
    capture_config_.width = width;
    capture_config_.height = height;
    capture_config_.exposure_ms = camera_.exposure_ms;
    capture_config_.gain = number("gain", 0);
    require(std::isfinite(capture_config_.gain) && capture_config_.gain >= 0 &&
                capture_config_.gain < 2147483647,
            "camera.gain");
    capture_config_.auto_exposure =
        node_.param("camera.auto_exposure", camera_.exposure_ms == 0);
    capture_config_.auto_gain =
        node_.param("camera.auto_gain", capture_config_.gain == 0);
    capture_config_.fps = node_.param<double>("camera.fps", 60.0);
    if (!std::isfinite(capture_config_.fps) || capture_config_.fps <= 0) {
      throw std::invalid_argument(std::string(node_.name()) +
                                  ": camera.fps must be finite and positive");
    }
    sim_camera_ = camera_;
    if (simulated_) {
      constexpr std::array<const char*, 6> keys = {
          "sim_mount_error_x_m",       "sim_mount_error_y_m",
          "sim_mount_error_z_m",       "sim_mount_error_roll_rad",
          "sim_mount_error_pitch_rad", "sim_mount_error_yaw_rad"};
      std::array<double, 6> error{};
      for (size_t i = 0; i < error.size(); ++i) {
        error[i] = number(keys[i], 0);
        require(std::isfinite(error[i]), keys[i]);
      }
      const auto delta = talos::vision::CameraMounting::FromExtrinsics(
          error[0], error[1], error[2], error[3], error[4], error[5]);
      const talos::geometry::SE3<double> optical_to_mount(
          talos::vision::CameraMounting::OpticalToRobotRotation().inverse(),
          Eigen::Vector3d::Zero());
      // Compose in the mount's NWU frame, before the optical-axis mapping.
      sim_camera_.mounting.T_robot_camera = camera_.mounting.T_robot_camera *
                                            optical_to_mount *
                                            delta.T_robot_camera;
    }
  }

  Completed Localize(std::vector<talos::vision::TagDetection3D>& detections,
                     std::uint64_t capture_ns, bool success,
                     const RecordingFeed& feed = {}) {
    Completed result{capture_ns};
    result.success = success;
    const bool record = recorder_ && feed.gray != nullptr;
    if (record) {
      talos::vision::AnnotateDetectionReasons(
          detections, layout_.layout, camera_.max_range_m, validation_options_);
      SnapshotDetections(detections);
    }
    result.rejected_range = FilterDetections(detections, camera_.max_range_m);
    result.visible = static_cast<std::uint32_t>(detections.size());
    const auto estimate = talos::vision::SolveMultiTagRobotPose(
        detections, camera_id_, camera_.mounting, layout_.layout,
        camera_.intrinsics, validation_options_, solver_options_);
    result.observation = MakeObservation(estimate, capture_ns, success);
    if (record) {
      // The ambiguity gate applies only to single-tag observations; mark that
      // lone detection so the sidecar attributes the rejection truthfully
      // instead of blaming every in-range tag.
      if (talos::vision::IsSingleTagAmbiguityRejection(estimate)) {
        for (auto& dr : recording_detections_) {
          if (dr.tag_id == estimate.tag_id) {
            dr.valid = false;
            dr.reject_reason =
                std::string{talos::vision::recording::kReasonAmbiguity};
          }
        }
      }
      RecordFeed(*feed.gray, capture_ns, result.observation, feed.truth);
    }
    return result;
  }

  void SnapshotDetections(
      const std::vector<talos::vision::TagDetection3D>& detections) {
    recording_detections_.clear();
    recording_detections_.reserve(detections.size());
    for (const auto& d : detections) {
      talos::vision::recording::DetectionRecord dr;
      dr.tag_id = d.tag_id;
      dr.hamming = d.hamming;
      dr.decision_margin = d.decision_margin;
      dr.corners_px = d.corners;
      dr.valid = d.valid && d.reject_reason.empty();
      dr.reprojection_error_px = d.reprojection_error;
      dr.ambiguity = d.ambiguity;
      dr.reject_reason = d.reject_reason.empty()
                             ? std::nullopt
                             : std::optional<std::string>{d.reject_reason};
      recording_detections_.push_back(std::move(dr));
    }
  }

  void RecordFeed(
      const talos::vision::Frame& gray, std::uint64_t capture_ns,
      const geometry::PoseObservation& observation,
      const std::optional<talos::vision::recording::TruthRecord>& truth) {
    recording_frame_.capture_ns = capture_ns;
    recording_frame_.detections.assign(recording_detections_.begin(),
                                       recording_detections_.end());
    recording_frame_.observation.valid = observation.valid();
    recording_frame_.observation.x_m = observation.pose().x_m();
    recording_frame_.observation.y_m = observation.pose().y_m();
    recording_frame_.observation.yaw_rad = observation.pose().yaw_rad();
    recording_frame_.observation.x_std_m = observation.x_std_m();
    recording_frame_.observation.y_std_m = observation.y_std_m();
    recording_frame_.observation.yaw_std_rad = observation.yaw_std_rad();
    recording_frame_.observation.tag_count =
        static_cast<int>(observation.tag_count());
    recording_frame_.truth = truth;
    recording_frame_.reason =
        std::string{talos::vision::recording::FrameReason(recording_frame_)};
    recorder_->Record(gray, recording_frame_);
  }

  Completed Process(const talos::vision::Frame& frame,
                    const std::optional<talos::vision::recording::TruthRecord>&
                        truth = std::nullopt) {
    auto result = pipeline_->ProcessFrame(frame);
    auto completed = Localize(result.detections, frame.capture_timestamp_ns(),
                              result.success, RecordingFeed{&frame, truth});
    completed.timestamp_kind = frame.timestamp_kind();
    return completed;
  }

  Completed Simulate(const geometry::OdometrySample& truth) {
    namespace fs = talos::vision::field_sim;
    const auto& pose = truth.pose();
    const auto field_pose =
        fs::RobotPoseOnField(pose.x_m(), pose.y_m(), pose.yaw_rad());
    const auto capture_time = talos::Time::from_nanos(
        static_cast<std::int64_t>(truth.sample_time_ns()));
    Completed result;
    if (rasterize_) {
      const auto& velocity = truth.velocity();
      fs::RenderFieldView(
          layout_.layout, patterns_, sim_camera_, field_pose, tag_size_, {},
          pixels_.get(),
          Eigen::Vector3d{velocity.vx_mps(), velocity.vy_mps(), 0},
          velocity.omega_radps(), &rng_, &render_scratch_);
      const auto& intr = camera_.intrinsics;
      const talos::vision::Frame frame(
          camera_id_, 1, captured_, capture_time, intr.image_width,
          intr.image_height, intr.image_width,
          talos::vision::PixelFormat::kGray8,
          std::shared_ptr<const std::vector<std::uint8_t>>(pixels_));
      result = Process(frame, talos::vision::recording::TruthRecord{
                                  pose.x_m(), pose.y_m(), pose.yaw_rad()});
    } else {
      detections_.clear();
      fs::SimulateVisibleTags(layout_.layout, sim_camera_, field_pose,
                              tag_size_, &sim_tags_, false, &rng_);
      for (const auto& tag : sim_tags_) {
        talos::vision::TagDetection3D detection;
        detection.valid = true;
        detection.tag_id = tag.tag_id;
        detection.T_camera_tag = tag.T_camera_tag;
        detection.ambiguity = tag.ambiguity;
        for (std::size_t c = 0; c < 4; ++c) {
          detection.corners[2 * c] = static_cast<float>(tag.corners[c].x());
          detection.corners[2 * c + 1] = static_cast<float>(tag.corners[c].y());
        }
        detections_.push_back(detection);
      }
      result = Localize(detections_, truth.sample_time_ns(), true);
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
      observation = geometry::PoseObservation{
          capture_ns,
          geometry::Pose2d{t.x(), t.y(), std::atan2(r(1, 0), r(0, 0))},
          estimate.std_devs.x_std_m,
          estimate.std_devs.y_std_m,
          estimate.std_devs.yaw_std_rad,
          static_cast<std::uint32_t>(estimate.tag_count),
          true};
    }
    return observation;
  }

  void DrainSimPending(std::uint64_t now_ns) {
    // A late timestamp delays physical delivery even when offset cancels its
    // bias. Publish every due frame in order, including catch-up batches.
    while (sim_pending_count_ > 0) {
      const auto& capture = sim_pending_[sim_pending_head_];
      const auto delivery_ns =
          CorrectCaptureTimestamp(capture.capture_ns, 0, sim_timestamp_lag_ns_);
      if (delivery_ns && *delivery_ns > now_ns) break;
      PublishObservation(capture, now_ns);
      sim_pending_head_ = (sim_pending_head_ + 1) % sim_pending_.size();
      --sim_pending_count_;
    }
  }

  void PublishObservation(const Completed& completed, std::uint64_t now_ns) {
    const auto stamp =
        simulated_
            ? CorrectCaptureTimestamp(completed.capture_ns, latency_offset_ns_,
                                      sim_timestamp_lag_ns_)
            : talos::vision::CaptureMidpoint(
                  completed.capture_ns, completed.timestamp_kind,
                  controls_applied_.load() && !capture_config_.auto_exposure
                      ? static_cast<std::uint64_t>(capture_config_.exposure_ms *
                                                   1e6)
                      : 0,
                  readout_ns_, latency_offset_ns_);
    const bool timestamp_valid = stamp && *stamp <= now_ns;
    const std::uint64_t age_ns = timestamp_valid ? now_ns - *stamp : 0;
    // A live stamp older than the frame timeout describes an instant the
    // camera would already count as silent: publish it invalid, still
    // reporting its age as latency, with one diagnostic per transition.
    // Only a valid stamp has an age, so an invalid one (future, underflow)
    // neither ends nor starts a stale run.
    const bool stale =
        !simulated_ && age_ns > static_cast<std::uint64_t>(frame_timeout_ns_);
    if (timestamp_valid && stale != stale_stamps_) {
      stale_stamps_ = stale;
      if (stale) {
        std::fprintf(stderr,
                     "%.*s: capture stamp %.3f s old exceeds "
                     "camera.frame_timeout_us = %lld; observations invalid\n",
                     static_cast<int>(node_.name().size()), node_.name().data(),
                     static_cast<double>(age_ns) / 1e9,
                     static_cast<long long>(frame_timeout_ns_ / 1000));
      } else {
        std::fprintf(stderr,
                     "%.*s: capture stamps within camera.frame_timeout_us "
                     "again\n",
                     static_cast<int>(node_.name().size()),
                     node_.name().data());
      }
    }
    const auto& observation = completed.observation;
    last_latency_us_ = age_ns / 1000;
    visible_ = completed.visible;
    healthy_ = completed.success;
    state_.send(geometry::PoseObservation{
        stamp.value_or(0), observation.pose(), observation.x_std_m(),
        observation.y_std_m(), observation.yaw_std_rad(),
        observation.tag_count(),
        observation.valid() && timestamp_valid && !stale});
  }

  // Only the worker owns camera lifecycle calls. Retry timing comes from the
  // node clock supplied by update(), never a wall-clock timer.
  bool OpenCamera() {
    if (!CanCapture(simulated_, device_)) return false;
    source_ = source_factory_(capture_config_);
    if (!source_) return false;
    source_->SetFrameCallback([this](const talos::vision::Frame& frame) {
      {
        std::lock_guard lock(mutex_);
        if (stopping_.load(std::memory_order_acquire)) return;
        ++received_;
        last_received_ns_.store(worker_now_ns_.load(std::memory_order_acquire),
                                std::memory_order_release);
        if (pending_) ++worker_dropped_;
        pending_ = frame;
      }
      wake_.notify_one();
    });
    const bool opened =
        source_->Start() && source_->IsRunning() && source_->IsConnected();
    controls_applied_.store(opened && source_->controls_applied());
    if (opened && !source_->controls_applied()) ++control_failures_;
    return opened;
  }

  void Work() {
    std::int64_t serviced_ns = -1;
    std::int64_t retry_at_ns = 0;
    std::int64_t retry_delay_ns = 100000000;
    bool opened_once = false;
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
      const auto last_received =
          last_received_ns_.load(std::memory_order_acquire);
      const bool stalled = source_ && connected_.load() &&
                           serviced_ns >= last_received &&
                           serviced_ns - last_received >= frame_timeout_ns_;
      if (stalled) {
        ++stalls_;
        source_->Stop();
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
        if (opened) {
          if (opened_once) ++reconnects_;
          opened_once = true;
          last_received_ns_.store(serviced_ns, std::memory_order_release);
        }
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
      retry_delay_ns = 100000000;
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
        completed_ = result;
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
  double tag_size_;
  int camera_id_;
  std::string device_;
  std::optional<talos::Subscription<geometry::OdometrySample>> truth_;
  bool truth_outage_ = false;
  std::optional<std::uint64_t> first_sim_capture_ns_;
  std::uint64_t last_sim_capture_ns_ = 0;
  long double next_sim_capture_frame_ = 1;
  std::uint64_t latency_offset_ns_ = 0, sim_timestamp_lag_ns_ = 0;
  std::vector<Completed> sim_pending_;
  std::size_t sim_pending_head_ = 0, sim_pending_count_ = 0;
  talos::Publisher<geometry::PoseObservation> state_;
  talos::Publisher<VisionStatus> status_;
  talos::vision::field_sim::FieldLayoutData layout_;
  talos::vision::field_sim::SimCameraView camera_;
  talos::vision::field_sim::SimCameraView sim_camera_;
  talos::vision::CameraSourceConfig capture_config_;
  talos::vision::PoseValidationOptions validation_options_;
  talos::vision::PoseSolverOptions solver_options_;
  std::mt19937_64 rng_;
  std::map<int, talos::vision::field_sim::TagPattern> patterns_;
  std::shared_ptr<std::vector<std::uint8_t>> pixels_;
  talos::vision::field_sim::RenderScratch render_scratch_;
  std::vector<talos::vision::field_sim::SimTagDetection> sim_tags_;
  std::vector<talos::vision::TagDetection3D> detections_;
  std::unique_ptr<talos::vision::AprilTagPipeline> pipeline_;
  std::unique_ptr<talos::vision::recording::FrameRecorder> recorder_;
  talos::vision::recording::Extrinsics extrinsics_;
  std::vector<talos::vision::recording::DetectionRecord> recording_detections_;
  talos::vision::recording::FrameRecord recording_frame_;
  bool recording_enabled_ = true;
  double recording_sample_hz_ = 1.0, recording_rejected_hz_ = 1.0;
  std::uint64_t recording_max_bytes_ = 268435456;
  std::unique_ptr<talos::vision::CameraSource> source_;
  SourceFactory source_factory_;
  std::uint64_t readout_ns_ = 0;
  std::int64_t frame_timeout_ns_ = 500000000;
  std::atomic<std::int64_t> last_received_ns_{0};
  std::atomic<std::uint64_t> reconnects_{0}, stalls_{0}, control_failures_{0};
  std::atomic<bool> controls_applied_{true};
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<talos::vision::Frame> pending_;
  std::optional<Completed> completed_;
  bool healthy_ = false, stale_stamps_ = false;
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
