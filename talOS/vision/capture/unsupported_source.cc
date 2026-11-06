#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "talOS/vision/capture/avfoundation_source.h"

namespace talos::vision {
std::vector<CaptureDevice> DiscoverCaptureDevices() { return {}; }
std::unique_ptr<CameraSource> CreateCaptureSource(uint32_t,
                                                  const std::string&) {
  return nullptr;
}
std::unique_ptr<CameraSource> CreatePlatformCameraSource(
    const CameraSourceConfig&) {
  return nullptr;
}
}  // namespace talos::vision
