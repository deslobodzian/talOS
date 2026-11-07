#pragma once
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "talOS/vision/camera_source.h"
namespace talos::vision {
// Single pending frame: publishing replaces old work rather than building
// latency.
class LatestFrameSlot {
 public:
  void Publish(Frame frame) {
    std::lock_guard lock(mutex_);
    latest_ = std::move(frame);
  }
  std::optional<Frame> TakeLatest() {
    std::lock_guard lock(mutex_);
    auto frame = std::move(latest_);
    latest_.reset();
    return frame;
  }

 private:
  std::mutex mutex_;
  std::optional<Frame> latest_;
};
struct CaptureDevice {
  std::string id, name;
  std::vector<CameraMode> modes;
};
struct CameraSourceConfig {
  uint32_t camera_id{0};
  std::string device_path;
  uint32_t width{1280};
  uint32_t height{720};
  double fps{60.0};
  PixelFormat format{PixelFormat::kGray8};
  double exposure_ms{0.0};
  double gain{0.0};
  bool auto_exposure{false};
  bool auto_gain{false};
};
std::unique_ptr<CameraSource> CreatePlatformCameraSource(
    const CameraSourceConfig& config);
// No discovery, permissions, or camera access until explicitly called by live
// Start.
std::vector<CaptureDevice> DiscoverCaptureDevices();
std::unique_ptr<CameraSource> CreateCaptureSource(uint32_t camera_id,
                                                  const std::string& device_id);
}  // namespace talos::vision
