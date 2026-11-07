#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/vision/frame.h"

namespace talos::vision {

// Negotiated or requested video capture mode.
struct CameraMode {
  uint32_t width{0};
  uint32_t height{0};
  double fps{0.0};
  PixelFormat format{PixelFormat::kGray8};

  constexpr auto operator<=>(const CameraMode&) const = default;
  constexpr bool operator==(const CameraMode&) const = default;
};

// Camera runtime settings and controls.
struct CameraControls {
  double exposure_ms{10.0};
  double gain{1.0};
  bool auto_exposure{false};
  bool auto_gain{false};

  constexpr auto operator<=>(const CameraControls&) const = default;
  constexpr bool operator==(const CameraControls&) const = default;
};

// Connection state reported to listeners.
enum class ConnectionState {
  kConnected,
  kDisconnected,
  kError,
};

constexpr std::string_view ConnectionStateToString(ConnectionState state) {
  switch (state) {
    case ConnectionState::kConnected:
      return "Connected";
    case ConnectionState::kDisconnected:
      return "Disconnected";
    case ConnectionState::kError:
      return "Error";
  }
  return "Unknown";
}

// Callback types for frame and lifecycle notifications.
using FrameCallback = std::function<void(const Frame& frame)>;
using ConnectionCallback =
    std::function<void(ConnectionState state, std::string_view message)>;

// Abstract interface for camera acquisition hardware or synthetic generators.
class CameraSource {
 public:
  virtual ~CameraSource() = default;

  // Stable device identification and logical camera slot (0..3).
  virtual uint32_t camera_id() const = 0;
  virtual std::string device_id() const = 0;
  virtual std::string name() const = 0;

  // Capture mode enumeration and selection.
  virtual std::vector<CameraMode> SupportedModes() const = 0;
  virtual CameraMode CurrentMode() const = 0;
  virtual bool SetMode(const CameraMode& mode) = 0;

  // Hardware controls.
  virtual CameraControls GetControls() const = 0;
  virtual bool SetControls(const CameraControls& controls) = 0;

  // Callbacks for frame arrival and connection changes.
  virtual void SetFrameCallback(FrameCallback callback) = 0;
  virtual void SetConnectionCallback(ConnectionCallback callback) = 0;

  // Lifecycle control.
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  virtual bool IsRunning() const = 0;
  virtual bool IsConnected() const = 0;
  // Cumulative source-side drops, including exhausted reusable buffers.
  virtual uint64_t dropped_frames() const { return 0; }
  // False when requested controls could not be applied; capture may continue.
  virtual bool controls_applied() const { return true; }
};

}  // namespace talos::vision
