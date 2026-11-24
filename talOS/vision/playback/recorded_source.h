#pragma once

// A CameraSource that plays a recording (an ImageSequence) instead of a
// camera, so the camera node and its unmodified pipeline run on recorded
// footage. Frames are stamped on the node clock: the instant playback starts
// plus the frame's presentation time on the recording's own timeline.

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
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

// Maximum duration of a single condition_variable::wait_for slice so a large
// due timestamp (e.g. INT64_MAX) never overflows steady_clock::now() + rel.
inline constexpr std::int64_t kMaxWaitSliceNs = 3'600'000'000'000LL;

inline std::int64_t WaitSliceNs(std::int64_t now_ns, std::int64_t due_ns) {
  if (now_ns >= due_ns) return 0;
  const std::uint64_t remaining =
      static_cast<std::uint64_t>(due_ns) - static_cast<std::uint64_t>(now_ns);
  return static_cast<std::int64_t>(
      std::min<std::uint64_t>(remaining, kMaxWaitSliceNs));
}

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
  bool IsRunning() const override {
    return running_.load(std::memory_order_acquire);
  }
  // False once the recording has ended, one mean frame period after its
  // last frame (not with it, so the consumer takes that frame first): the
  // consumer sees a camera that went away and reopens it, which plays the
  // recording again.
  bool IsConnected() const override {
    return connected_.load(std::memory_order_acquire);
  }
  uint64_t dropped_frames() const override {
    return dropped_.load(std::memory_order_relaxed);
  }

  // kManual only: delivers every frame due at node-clock `now_ns`, in order.
  // The first call after Start() anchors playback: the first frame is due at
  // that instant. Returns the number of frames delivered.
  std::size_t DeliverUntil(std::int64_t now_ns);

  std::size_t frame_count() const;

 private:
  static thread_local const RecordedCameraSource* t_play_source;
  static thread_local bool t_in_frame_callback;

  bool IsSelfPlayThread() const { return t_play_source == this; }
  bool IsInFrameCallback() const {
    return t_play_source == this && t_in_frame_callback;
  }
  bool IsCancelled(std::uint64_t generation) const;
  bool HasJoinableThreadsLocked() const;
  void StopAndDetachThreadsLocked(std::vector<std::thread>& to_join);
  static void JoinThreads(std::vector<std::thread>& threads);

  // Decodes frame `index` of `sequence` into `pixels`; false (counted dropped)
  // when it cannot be decoded or its size differs from `width` x `height`.
  bool Decode(const ImageSequence& sequence, std::size_t index,
              std::uint32_t width, std::uint32_t height,
              std::vector<std::uint8_t>* pixels);
  // Hands the decoded frame to the frame callback, stamped `capture_ns`.
  void Emit(std::size_t index, std::int64_t capture_ns, std::uint64_t session,
            std::uint32_t width, std::uint32_t height,
            const std::vector<std::uint8_t>& pixels);
  void Play(const std::shared_ptr<const ImageSequence>& sequence,
            std::uint64_t generation, std::uint64_t session,
            std::uint32_t width, std::uint32_t height, std::int64_t start_ns,
            std::int64_t end_ns);
  // Sleeps until host-clock `due_ns`; false when Stop() or a newer Start()
  // interrupts it.
  bool WaitUntil(std::int64_t due_ns, std::uint64_t generation);
  void Report(ConnectionState state, std::string_view reason);
  void Finish(std::uint64_t generation);

  CameraSourceConfig config_;
  Pacing pacing_;
  mutable std::mutex lifecycle_mutex_;
  std::condition_variable lifecycle_cv_;
  std::size_t active_teardowns_ = 0;
  mutable std::mutex state_mutex_;
  std::shared_ptr<const ImageSequence> sequence_;
  std::mutex callback_mutex_;
  FrameCallback frame_callback_;
  ConnectionCallback connection_callback_;
  std::atomic<bool> running_{false}, connected_{false};
  std::atomic<std::uint64_t> dropped_{0};
  std::uint64_t session_ = 0;
  // kManual playback position (guarded by state_mutex_).
  std::optional<std::int64_t> manual_start_ns_;
  std::size_t next_ = 0;
  // When the recording ends, after its first frame: one frame period after
  // its last frame (guarded by state_mutex_).
  std::int64_t end_ns_ = 0;
  // kRealtime playback thread and cancellation state (guarded by stop_mutex_).
  std::thread thread_;
  std::vector<std::thread> retired_threads_;
  mutable std::mutex stop_mutex_;
  std::condition_variable stop_cv_;
  bool stop_ = false;
  std::uint64_t generation_ = 0;
};

// The camera node's source factory: a `camera.device` starting with
// kRecordingDevicePrefix plays that recording in realtime, any other device
// opens the platform camera (CreatePlatformCameraSource).
std::unique_ptr<CameraSource> CreateCameraSource(
    const CameraSourceConfig& config);

}  // namespace talos::vision::playback
