#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "talOS/vision/apriltag/cpu_apriltag_backend.h"
#include "talOS/vision/frame.h"

namespace talos::vision {

// Breakdown of processing time across pipeline stages in microseconds.
struct StageTimings {
  // Color-conversion only. Backends must not fold threshold/CCL time in
  // here: the GPU preprocessing gate compares conversion vs conversion.
  uint64_t grayscale_us{0};
  // Adaptive threshold + connected-components (GPU backends). CPU backends
  // cannot separate thresholding from detection, so they leave this 0 and
  // fold threshold time into quad_us instead.
  uint64_t threshold_us{0};
  uint64_t quad_us{0};
  uint64_t decode_us{0};
  uint64_t pose_us{0};
  uint64_t total_us{0};

  constexpr auto operator<=>(const StageTimings&) const = default;
  constexpr bool operator==(const StageTimings&) const = default;
};

// Raw tag detection extracted directly by a compute backend.
struct RawTagDetection {
  int id{-1};
  int hamming{0};
  float center_x{0.0f};
  float center_y{0.0f};
  std::array<float, 8> corners{};  // [x0, y0, x1, y1, x2, y2, x3, y3]
  float decision_margin{0.0f};
  std::array<float, 9> homography{};  // 3x3 homography matrix, row-major
  bool valid{true};
};

// Outcome of executing a compute backend on a single frame.
struct BackendExecutionResult {
  bool success{false};
  std::string error_message;
  std::vector<RawTagDetection> detections;
  StageTimings timings;
  std::string effective_backend;
  std::string fallback_reason;
};

// Abstract compute backend for AprilTag / vision processing (CPU, Metal, CUDA).
class ComputeBackend {
 public:
  virtual ~ComputeBackend() = default;

  // Capability discovery
  virtual std::string name() const = 0;
  virtual std::string device_info() const = 0;
  virtual bool supports_gpu() const = 0;

  // Synchronous execution interface
  virtual BackendExecutionResult Execute(const Frame& frame) = 0;
};

// CPU reference backend using upstream AprilTag detector.
class CpuAprilTagBackend : public ComputeBackend {
 public:
  explicit CpuAprilTagBackend(int num_threads = 1, double quad_decimate = 1.0,
                              double quad_sigma = 0.0,
                              bool refine_edges = true);
  explicit CpuAprilTagBackend(const apriltag::DetectorOptions& options);
  ~CpuAprilTagBackend() override;

  std::string name() const override { return "cpu"; }
  std::string device_info() const override;
  bool supports_gpu() const override { return false; }

  BackendExecutionResult Execute(const Frame& frame) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace talos::vision
