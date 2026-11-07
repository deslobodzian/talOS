#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "talOS/vision/camera_source.h"
#include "talOS/vision/frame.h"

namespace talos::vision {

// Describes a synthetic tag placed in the rendered scene.
struct SyntheticTagDescriptor {
  int tag_id{0};
  double center_x{320.0};
  double center_y{240.0};
  double size_pixels{120.0};
  double rotation_rad{0.0};
};

// Generates deterministic scenes with tags or static fixtures for unit tests,
// replay, and headless CI. Supports simulating disconnects and timed delivery.
class SyntheticCameraSource : public CameraSource {
 public:
  explicit SyntheticCameraSource(uint32_t camera_id = 0,
                                 std::string device_id = "",
                                 CameraMode default_mode = {
                                     640, 480, 30.0, PixelFormat::kGray8});

  ~SyntheticCameraSource() override;

  // CameraSource implementation
  uint32_t camera_id() const override;
  std::string device_id() const override;
  std::string name() const override;

  std::vector<CameraMode> SupportedModes() const override;
  CameraMode CurrentMode() const override;
  bool SetMode(const CameraMode& mode) override;

  CameraControls GetControls() const override;
  bool SetControls(const CameraControls& controls) override;

  void SetFrameCallback(FrameCallback callback) override;
  void SetConnectionCallback(ConnectionCallback callback) override;

  bool Start() override;
  void Stop() override;
  bool IsRunning() const override;
  bool IsConnected() const override;

  // Synthetic scene configuration
  void AddTag(const SyntheticTagDescriptor& tag);
  void ClearTags();

  void SetFixtureFrame(const Frame& frame);
  void ClearFixtureFrame();

  // Generates the next deterministic frame based on current scene and mode.
  Frame GenerateFrame();

  // Triggers delivery of a single frame synchronously to the frame callback.
  // Returns true if delivered, false if disconnected or no callback.
  bool TriggerFrame();

  // Simulates camera disconnect and reconnect events.
  void SimulateDisconnect();
  void SimulateReconnect();

  uint64_t session_id() const;
  uint64_t sequence_number() const;

 private:
  void WorkerLoop();
  void RenderScene(Frame& frame);

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread delivery_thread_;

  std::atomic<bool> is_running_{false};
  std::atomic<bool> is_connected_{true};

  uint32_t camera_id_{0};
  std::string device_id_;
  std::string name_;

  CameraMode mode_;
  CameraControls controls_;

  FrameCallback frame_callback_;
  ConnectionCallback connection_callback_;

  std::vector<SyntheticTagDescriptor> tags_;
  std::optional<Frame> fixture_frame_;

  uint64_t session_id_{1};
  uint64_t sequence_number_{0};
};

}  // namespace talos::vision
