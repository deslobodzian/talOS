#pragma once

// The on-disk frame recording contract, shared by the recorder (writer), the
// camera node (producer) and the replay tool (reader). See
// 2026-robot/main_processor/vision/README.md for the keys and the
// `<session>/frames/<camera>/<capture_ns>_<reason>.{png,json}` layout.
//
// The JSON is deliberately produced by a tiny hand-rolled serializer in
// recording.cc: the vision stack has no JSON writer dependency (the field-sim
// json_parser.h only reads), and these files are a few kilobytes each.

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/vision/calibration.h"

namespace talos::vision::recording {

inline constexpr int kFormatVersion = 1;

// Reject-reason vocabulary (an empty string means the detection is valid).
inline constexpr std::string_view kReasonHamming = "hamming";
inline constexpr std::string_view kReasonDecisionMargin = "decision_margin";
inline constexpr std::string_view kReasonReprojection = "reprojection";
inline constexpr std::string_view kReasonAmbiguity = "ambiguity";
inline constexpr std::string_view kReasonRange = "range";
inline constexpr std::string_view kReasonNotInLayout = "not_in_layout";

// Mounting extrinsics as the robot config names them (the camera node reads
// these directly; the recorder serializes them rather than decomposing an
// SE(3), so the round trip through the sidecar is lossless).
struct Extrinsics {
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double yaw_rad{0.0};
};

// The detector settings captured beside each frame, so replay defaults to
// what recorded the frame.
struct DetectorParams {
  std::string backend{"auto"};
  double decimate{1.0};
  int threads{1};
  double sigma{0.0};
  bool refine_edges{true};
  int hamming_threshold{1};
  double min_decision_margin{30.0};
};

struct DetectionRecord {
  int tag_id{0};
  int hamming{0};
  double decision_margin{0.0};
  std::array<float, 8> corners_px{};
  bool valid{true};
  double reprojection_error_px{0.0};
  double ambiguity{0.0};
  std::optional<std::string> reject_reason;  // nullopt when valid
};

struct ObservationRecord {
  bool valid{false};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double x_std_m{0.0};
  double y_std_m{0.0};
  double yaw_std_rad{0.0};
  int tag_count{0};
};

struct TruthRecord {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

// Per-frame data the camera node hands the recorder (the camera constants
// live in CameraContext, captured once).
struct FrameRecord {
  std::uint64_t capture_ns{0};
  std::string reason;  // "sampled" or "rejected"
  std::vector<DetectionRecord> detections;
  ObservationRecord observation;
  std::optional<TruthRecord> truth;
};

// Constants of one camera, captured once and stamped into every sidecar.
struct CameraContext {
  std::string camera;
  int width{0};
  int height{0};
  CameraIntrinsics intrinsics;
  Extrinsics mounting;
  DetectorParams detector;
};

// Serializes one sidecar JSON document (without a trailing newline).
std::string SerializeRecord(const CameraContext& context,
                            const FrameRecord& record);

// Parses one sidecar JSON document back into `context` and `record`.
// Returns false with `error` naming the problem.
bool ParseRecord(std::string_view json, CameraContext* context,
                 FrameRecord* record, std::string* error);

// The `<capture_ns>_<reason>` stem, capture_ns zero-padded to 20 digits so
// names sort by time.
std::string RecordStem(std::uint64_t capture_ns, std::string_view reason);

// The reason a frame should carry: "rejected" when it decoded at least one
// detection but produced no valid observation, or any detection is invalid
// (decode or pose-level); "sampled" otherwise.
inline std::string_view FrameReason(const FrameRecord& record) {
  if (!record.detections.empty() && !record.observation.valid) {
    return "rejected";
  }
  for (const auto& d : record.detections) {
    if (!d.valid || d.reject_reason.has_value()) return "rejected";
  }
  return "sampled";
}

}  // namespace talos::vision::recording
