#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "talOS/vision/apriltag/cpu_apriltag_backend.h"
#include "talOS/vision/compute_backend.h"
#include "talOS/vision/frame.h"

namespace MTL {
class CommandBuffer;
class Buffer;
class Device;
class CommandQueue;
class ComputeCommandEncoder;
}  // namespace MTL

namespace talos::vision::metal {

class MetalAprilTagBackend : public talos::vision::ComputeBackend {
 public:
  explicit MetalAprilTagBackend(
      const talos::vision::apriltag::DetectorOptions& options = {});
  ~MetalAprilTagBackend() override;

  MetalAprilTagBackend(const MetalAprilTagBackend&) = delete;
  MetalAprilTagBackend& operator=(const MetalAprilTagBackend&) = delete;
  MetalAprilTagBackend(MetalAprilTagBackend&&) noexcept;
  MetalAprilTagBackend& operator=(MetalAprilTagBackend&&) noexcept;

  std::string name() const override { return "metal"; }
  std::string device_info() const override;
  bool supports_gpu() const override;

  // ComputeBackend interface
  talos::vision::BackendExecutionResult Execute(
      const talos::vision::Frame& frame) override;

  // Granular detection interface directly matching CpuAprilTagBackend
  std::vector<talos::vision::apriltag::Detection> Detect(
      const uint8_t* data, int width, int height, int stride = 0,
      PixelFormat format = PixelFormat::kGray8);

  std::vector<talos::vision::apriltag::Detection> Detect(
      const uint8_t* data, int width, int height, int stride,
      PixelFormat format,
      const talos::vision::apriltag::CameraIntrinsics& intrinsics,
      double tag_size_meters);

  std::vector<talos::vision::apriltag::Detection> DetectImageFile(
      const std::string& filepath);

  std::vector<talos::vision::apriltag::Detection> DetectImageFile(
      const std::string& filepath,
      const talos::vision::apriltag::CameraIntrinsics& intrinsics,
      double tag_size_meters);

  void SetOptions(const talos::vision::apriltag::DetectorOptions& options);
  const talos::vision::apriltag::DetectorOptions& options() const;

  // Stage timings (grayscale_us, threshold_us, quad_us, decode_us,
  // pose_us, total_us)
  const talos::vision::StageTimings& last_timings() const;
  bool is_fallback() const;

  // GPU-accelerated Tile-Based Adaptive Thresholding
  void RunGpuThreshold(MTL::CommandBuffer* cmd_buf, MTL::Buffer* gray_buf,
                       MTL::Buffer* thresh_buf, uint32_t width, uint32_t height,
                       uint32_t stride, uint32_t min_diff);

  MTL::Device* device() const;
  MTL::CommandQueue* queue() const;
  MTL::Buffer* thresh_buffer() const;
  MTL::Buffer* min_buffer() const;
  MTL::Buffer* max_buffer() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace talos::vision::metal
