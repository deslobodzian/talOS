#pragma once
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "talOS/vision/camera_source.h"
namespace talos::vision {
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
