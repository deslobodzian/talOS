#include "talOS/vision/playback/recorded_source.h"

#include <time.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/events/time.h"
#include "talOS/vision/frame.h"

namespace talos::vision::playback {
namespace {

std::string RecordingPath(std::string_view device) {
  if (device.starts_with(kRecordingDevicePrefix)) {
    device.remove_prefix(kRecordingDevicePrefix.size());
  }
  return std::string(device);
}

// The realtime node clock (talOS/events pollers' now()): CLOCK_MONOTONIC on
// Linux, CLOCK_MONOTONIC_RAW (mach continuous time) on macOS. Not
// std::chrono::steady_clock, which on macOS stops during sleep.
std::int64_t HostNowNs() {
  struct timespec ts {};
#if defined(__APPLE__)
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
#else
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
  return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

}  // namespace

RecordedCameraSource::RecordedCameraSource(const CameraSourceConfig& config,
                                           Pacing pacing)
    : config_(config), pacing_(pacing) {
  config_.device_path = RecordingPath(config.device_path);
}

RecordedCameraSource::~RecordedCameraSource() { Stop(); }

std::vector<CameraMode> RecordedCameraSource::SupportedModes() const {
  return {CurrentMode()};
}

CameraMode RecordedCameraSource::CurrentMode() const {
  return {config_.width, config_.height, config_.fps, PixelFormat::kGray8};
}

bool RecordedCameraSource::SetMode(const CameraMode& mode) {
  return mode == CurrentMode();
}

void RecordedCameraSource::SetFrameCallback(FrameCallback callback) {
  std::lock_guard lock(callback_mutex_);
  frame_callback_ = std::move(callback);
}

void RecordedCameraSource::SetConnectionCallback(ConnectionCallback callback) {
  std::lock_guard lock(callback_mutex_);
  connection_callback_ = std::move(callback);
}

void RecordedCameraSource::Report(ConnectionState state,
                                  std::string_view reason) {
  ConnectionCallback callback;
  {
    std::lock_guard lock(callback_mutex_);
    callback = connection_callback_;
  }
  if (callback) callback(state, reason);
}

std::size_t RecordedCameraSource::frame_count() const {
  return sequence_ ? sequence_->size() : 0;
}

bool RecordedCameraSource::Start() {
  Stop();
  std::string error;
  sequence_ = ImageSequence::Open(config_.device_path, config_.fps, &error);
  if (!sequence_) {
    Report(ConnectionState::kError, "recording: " + error);
    return false;
  }
  int width = 0, height = 0;
  if (!sequence_->LoadGray(0, &pixels_, &width, &height, &error)) {
    Report(ConnectionState::kError, "recording: " + error);
    return false;
  }
  if ((config_.width != 0 && static_cast<uint32_t>(width) != config_.width) ||
      (config_.height != 0 &&
       static_cast<uint32_t>(height) != config_.height)) {
    Report(ConnectionState::kError,
           "recording frames are " + std::to_string(width) + "x" +
               std::to_string(height) + ", the camera is configured " +
               std::to_string(config_.width) + "x" +
               std::to_string(config_.height));
    return false;
  }
  config_.width = static_cast<uint32_t>(width);
  config_.height = static_cast<uint32_t>(height);
  // The recording ends one frame period after its last frame, so the
  // consumer has that long to take the last frame before the camera goes.
  const auto period =
      sequence_->mean_period_ns() > 0 ? sequence_->mean_period_ns()
      : config_.fps > 0 ? static_cast<std::int64_t>(1e9 / config_.fps)
                        : std::int64_t{100'000'000};
  end_ns_ = (*sequence_)[sequence_->size() - 1].pts_ns + period;
  ++session_;
  next_ = 0;
  manual_start_ns_.reset();
  running_.store(true);
  connected_.store(true);
  if (pacing_ == Pacing::kRealtime) {
    {
      std::lock_guard lock(stop_mutex_);
      stop_ = false;
    }
    const auto start_ns = HostNowNs();
    thread_ = std::thread([this, start_ns] { Play(start_ns); });
  }
  return true;
}

void RecordedCameraSource::Stop() {
  {
    std::lock_guard lock(stop_mutex_);
    stop_ = true;
  }
  stop_cv_.notify_all();
  if (thread_.joinable()) thread_.join();
  running_.store(false);
  connected_.store(false);
}

void RecordedCameraSource::Finish() {
  connected_.store(false);
  Report(ConnectionState::kDisconnected, "end of recording");
}

bool RecordedCameraSource::Decode(const ImageSequence& sequence,
                                  std::size_t index) {
  int width = 0, height = 0;
  std::string error;
  if (!sequence.LoadGray(index, &pixels_, &width, &height, &error) ||
      static_cast<uint32_t>(width) != config_.width ||
      static_cast<uint32_t>(height) != config_.height) {
    dropped_.fetch_add(1);
    return false;
  }
  return true;
}

void RecordedCameraSource::Emit(std::size_t index, std::int64_t capture_ns) {
  Frame frame(config_.camera_id, session_, index,
              talos::event::MonotonicTime::from_nanos(capture_ns),
              config_.width, config_.height, config_.width, PixelFormat::kGray8,
              pixels_.data(), pixels_.size());
  // The recording's own timeline, not asserted to be the exposure midpoint.
  frame.set_timestamp_kind(TimestampKind::kPresentation);
  frame.set_clock_domain(ClockDomain::kHostMonotonic);
  std::lock_guard lock(callback_mutex_);
  if (frame_callback_) frame_callback_(frame);
}

std::size_t RecordedCameraSource::DeliverUntil(std::int64_t now_ns) {
  if (pacing_ != Pacing::kManual || !connected_.load() || !sequence_) return 0;
  if (!manual_start_ns_) manual_start_ns_ = now_ns;
  std::size_t delivered = 0;
  while (next_ < sequence_->size() &&
         *manual_start_ns_ + (*sequence_)[next_].pts_ns <= now_ns) {
    if (Decode(*sequence_, next_)) {
      Emit(next_, *manual_start_ns_ + (*sequence_)[next_].pts_ns);
      ++delivered;
    }
    ++next_;
  }
  if (next_ == sequence_->size() && now_ns >= *manual_start_ns_ + end_ns_) {
    Finish();
  }
  return delivered;
}

bool RecordedCameraSource::WaitUntil(std::int64_t due_ns) {
  std::unique_lock lock(stop_mutex_);
  for (auto now_ns = HostNowNs(); !stop_ && now_ns < due_ns;
       now_ns = HostNowNs()) {
    stop_cv_.wait_for(lock, std::chrono::nanoseconds(due_ns - now_ns));
  }
  return !stop_;
}

void RecordedCameraSource::Play(std::int64_t start_ns) {
  // Play starts only once Open() has loaded the sequence.
  if (!sequence_) return;
  const ImageSequence& sequence = *sequence_;
  const auto period = sequence.mean_period_ns();
  for (std::size_t i = 0; i < sequence.size(); ++i) {
    const auto due_ns = start_ns + sequence[i].pts_ns;
    if (i > 0 && HostNowNs() > due_ns + period) {
      dropped_.fetch_add(1);
      continue;
    }
    // Decode ahead of the frame's time, then deliver it at that time.
    const bool decoded = Decode(sequence, i);
    if (!WaitUntil(due_ns)) return;
    if (decoded) Emit(i, due_ns);
  }
  if (WaitUntil(start_ns + end_ns_)) Finish();
}

std::unique_ptr<CameraSource> CreateCameraSource(
    const CameraSourceConfig& config) {
  if (std::string_view(config.device_path)
          .starts_with(kRecordingDevicePrefix)) {
    return std::make_unique<RecordedCameraSource>(
        config, RecordedCameraSource::Pacing::kRealtime);
  }
  return CreatePlatformCameraSource(config);
}

}  // namespace talos::vision::playback
