#pragma once
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
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
// A frame duration as AVFoundation states one: value / timescale seconds.
struct FrameDuration {
  int64_t value{0};
  int32_t timescale{1};
  constexpr bool operator==(const FrameDuration&) const = default;
  constexpr double seconds() const {
    return static_cast<double>(value) / timescale;
  }
};
// The frame duration to set for `fps` in a format's frame-rate range
// [min_fps, max_fps], whose shortest and longest durations are `shortest` and
// `longest`, or nullopt when the range does not hold `fps`. UVC frame
// intervals are integers of 100 ns, so a camera's 30, 60 and 120 fps reach
// AVFoundation as single-rate ranges at 30.00003, 60.00024 and 120.00048 fps;
// a relative 1e-4 tolerance accepts them and stays far below the gap between
// two rates a camera offers. AVFoundation throws on a duration outside the
// range, so a rate at an end gets that end's own duration and any other gets
// 1 / fps clamped into the range.
inline std::optional<FrameDuration> FrameDurationFor(double fps, double min_fps,
                                                     double max_fps,
                                                     FrameDuration shortest,
                                                     FrameDuration longest) {
  constexpr double kTolerance = 1e-4;
  const auto near = [&](double rate) {
    return std::abs(fps - rate) <= rate * kTolerance;
  };
  if (near(max_fps)) return shortest;
  if (near(min_fps)) return longest;
  if (!(fps > min_fps && fps < max_fps)) return std::nullopt;
  const FrameDuration duration{1000,
                               static_cast<int32_t>(std::lround(fps * 1000.0))};
  if (duration.seconds() < shortest.seconds()) return shortest;
  if (duration.seconds() > longest.seconds()) return longest;
  return duration;
}
std::unique_ptr<CameraSource> CreatePlatformCameraSource(
    const CameraSourceConfig& config);
// No discovery, permissions, or camera access until explicitly called by live
// Start.
std::vector<CaptureDevice> DiscoverCaptureDevices();
std::unique_ptr<CameraSource> CreateCaptureSource(uint32_t camera_id,
                                                  const std::string& device_id);
}  // namespace talos::vision
