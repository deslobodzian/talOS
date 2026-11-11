#pragma once

// A CameraSource that plays a recording (an ImageSequence) instead of a
// camera, so the camera node and its unmodified pipeline run on recorded
// footage. Frames are stamped on the node clock: the instant playback starts
// plus the frame's presentation time on the recording's own timeline.

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "talOS/vision/camera_source.h"
#include "talOS/vision/capture/avfoundation_source.h"
#include "talOS/vision/playback/image_sequence.h"

namespace talos::vision::playback {

// `camera.device` prefix that selects a recording instead of a camera.
inline constexpr std::string_view kRecordingDevicePrefix = "recording:";

class RecordedCameraSource final : public CameraSource {
 public:
  enum class Pacing {
    // A playback thread delivers each frame at its presentation time on the
    // host monotonic clock (the realtime node clock), decoding ahead of it.
    // A frame whose time has passed by more than one mean frame period before
    // its decode starts is skipped and counted in dropped_frames(), as a
    // camera does not wait for its consumer.
    kRealtime,
    // No thread: DeliverUntil() delivers, on the caller's thread, every frame
    // due by the node-clock instant it is given. Deterministic, for tests
    // under testing::Sim.
    kManual,
  };

  // `config.device_path` is the recording directory (any
  // kRecordingDevicePrefix removed); `config.fps` is its frame rate when it
  // has no timestamps.csv. A nonzero `config.width`/`height` must match the
  // recording's frames. The recording is indexed and checked in Start(),
  // never in the constructor, so a bad recording is reported as a failed
  // start with its reason.
  RecordedCameraSource(const CameraSourceConfig& config, Pacing pacing);
  ~RecordedCameraSource() override;

  uint32_t camera_id() const override { return config_.camera_id; }
  std::string device_id() const override { return config_.device_path; }
  std::string name() const override { return "recording"; }
  std::vector<CameraMode> SupportedModes() const override;
  CameraMode CurrentMode() const override;
  bool SetMode(const CameraMode& mode) override;
  // A recording has no controls: they are reported applied, and ignored.
  CameraControls GetControls() const override { return {}; }
  bool SetControls(const CameraControls&) override { return true; }
  void SetFrameCallback(FrameCallback callback) override;
  void SetConnectionCallback(ConnectionCallback callback) override;
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override { return running_.load(); }
  // False once the recording has ended, one mean frame period after its
  // last frame (not with it, so the consumer takes that frame first): the
  // consumer sees a camera that went away and reopens it, which plays the
  // recording again.
  bool IsConnected() const override { return connected_.load(); }
  uint64_t dropped_frames() const override { return dropped_.load(); }

  // kManual only: delivers every frame due at node-clock `now_ns`, in order.
  // The first call after Start() anchors playback: the first frame is due at
  // that instant. Returns the number of frames delivered.
  std::size_t DeliverUntil(std::int64_t now_ns);

  std::size_t frame_count() const;

 private:
  // Decodes frame `index` into pixels_; false (counted dropped) when it
  // cannot be decoded or its size differs from the first frame's.
  bool Decode(std::size_t index);
  // Hands the decoded frame to the frame callback, stamped `capture_ns`.
  void Emit(std::size_t index, std::int64_t capture_ns);
  void Play(std::int64_t start_ns);
  // Sleeps until host-clock `due_ns`; false when Stop() interrupts it.
  bool WaitUntil(std::int64_t due_ns);
  void Report(ConnectionState state, std::string_view reason);
  void Finish();

  CameraSourceConfig config_;
  Pacing pacing_;
  std::optional<ImageSequence> sequence_;
  std::vector<std::uint8_t> pixels_;
  std::mutex callback_mutex_;
  FrameCallback frame_callback_;
  ConnectionCallback connection_callback_;
  std::atomic<bool> running_{false}, connected_{false};
  std::atomic<std::uint64_t> dropped_{0};
  std::uint64_t session_ = 0;
  // kManual playback position.
  std::optional<std::int64_t> manual_start_ns_;
  std::size_t next_ = 0;
  // When the recording ends, after its first frame: one frame period after
  // its last frame.
  std::int64_t end_ns_ = 0;
  // kRealtime playback thread.
  std::thread thread_;
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;
  bool stop_ = false;
};

// The camera node's source factory: a `camera.device` starting with
// kRecordingDevicePrefix plays that recording in realtime, any other device
// opens the platform camera (CreatePlatformCameraSource).
std::unique_ptr<CameraSource> CreateCameraSource(
    const CameraSourceConfig& config);

}  // namespace talos::vision::playback
