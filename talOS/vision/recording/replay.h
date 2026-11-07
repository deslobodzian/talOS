#pragma once

// The replay analysis behind //talOS/vision/recording:frame_replay: re-runs
// AprilTag detection and the camera node's localization over every recorded
// sidecar and compares the outcome with what was recorded. Extracted from the
// binary so tests can exercise the same path.

#include <cstdint>
#include <map>
#include <optional>
#include <string>

#include "talOS/vision/field_sim/field_layout_loader.h"

namespace talos::vision::recording {

struct DetectorOverrides {
  std::optional<double> decimate;
  std::optional<int> threads;
  std::optional<double> sigma;
  std::optional<bool> refine_edges;
  std::optional<int> hamming;
  std::optional<double> min_decision_margin;
  std::string backend{"auto"};
  double max_range_m{8.0};  // matches vision.toml's camera.max_range_m default
};

struct ReplayStats {
  std::uint64_t frames{0};
  std::uint64_t valid_before{0};
  std::uint64_t valid_after{0};
  std::uint64_t truth_frames{0};
  // Per-reason rejection counts; key "(valid)" holds clean detections.
  std::map<std::string, std::uint64_t> rejections_before;
  std::map<std::string, std::uint64_t> rejections_after;
  // Position/yaw error RMS over truth frames with valid observations.
  double rms_x_before{0.0};
  double rms_y_before{0.0};
  double rms_yaw_before{0.0};
  double rms_x_after{0.0};
  double rms_y_after{0.0};
  double rms_yaw_after{0.0};
};

// Re-runs detection + localization over every recorded sidecar in `frames_dir`
// using `layout` and the sidecar detector settings overridden by `overrides`.
// Returns false with `error` on a malformed sidecar or unreadable image.
bool ReplayFrames(const std::string& frames_dir,
                  const field_sim::FieldLayoutData& layout,
                  const DetectorOverrides& overrides, ReplayStats* stats,
                  std::string* error);

}  // namespace talos::vision::recording
