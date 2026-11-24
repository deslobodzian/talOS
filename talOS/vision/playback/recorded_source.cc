#include "talOS/vision/playback/recorded_source.h"

#include <time.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/events/time.h"
#include "talOS/process/process.h"
#include "talOS/vision/frame.h"

namespace talos::vision::playback {
namespace {

std::string RecordingPath(std::string_view device) {
  if (device.starts_with(kRecordingDevicePrefix)) {
    device.remove_prefix(kRecordingDevicePrefix.size());
  }
  return std::string(device);
}

std::int64_t SaturatingAddNs(std::int64_t a, std::int64_t b) {
  if (b >= 0 && a > std::numeric_limits<std::int64_t>::max() - b) {
    return std::numeric_limits<std::int64_t>::max();
  }
  if (b < 0 && a < std::numeric_limits<std::int64_t>::min() - b) {
    return std::numeric_limits<std::int64_t>::min();
  }
  return a + b;
}

bool ReachedOffsetNs(std::int64_t now_ns, std::int64_t start_ns,
                     std::int64_t offset_ns) {
  if (offset_ns <= 0) return now_ns >= start_ns;
  if (now_ns < start_ns) return false;
  return static_cast<std::uint64_t>(now_ns) -
             static_cast<std::uint64_t>(start_ns) >=
         static_cast<std::uint64_t>(offset_ns);
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

thread_local const RecordedCameraSource* RecordedCameraSource::t_play_source =
    nullptr;
thread_local bool RecordedCameraSource::t_in_frame_callback = false;

RecordedCameraSource::RecordedCameraSource(const CameraSourceConfig& config,
                                           Pacing pacing)
    : config_(config), pacing_(pacing) {
  config_.device_path = RecordingPath(config.device_path);
}

RecordedCameraSource::~RecordedCameraSource() {
  Stop();
  std::vector<std::thread> to_join;
  {
    std::lock_guard lock(stop_mutex_);
    const auto self = std::this_thread::get_id();
    for (auto& t : retired_threads_) {
      if (!t.joinable()) continue;
      if (t.get_id() == self) {
        t.detach();
      } else {
        to_join.push_back(std::move(t));
      }
    }
    retired_threads_.clear();
  }
  JoinThreads(to_join);
}

std::vector<CameraMode> RecordedCameraSource::SupportedModes() const {
  return {CurrentMode()};
}

CameraMode RecordedCameraSource::CurrentMode() const {
  std::lock_guard lock(state_mutex_);
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
  std::lock_guard lock(state_mutex_);
  return sequence_ ? sequence_->size() : 0;
}

bool RecordedCameraSource::IsCancelled(std::uint64_t generation) const {
  std::lock_guard lock(stop_mutex_);
  return stop_ || generation_ != generation;
}

bool RecordedCameraSource::HasJoinableThreadsLocked() const {
  std::lock_guard lock(stop_mutex_);
  return thread_.joinable() || !retired_threads_.empty();
}

void RecordedCameraSource::StopAndDetachThreadsLocked(
    std::vector<std::thread>& to_join) {
  std::lock_guard lock(stop_mutex_);
  stop_ = true;
  ++generation_;
  running_.store(false, std::memory_order_release);
  connected_.store(false, std::memory_order_release);
  const auto self = std::this_thread::get_id();
  if (thread_.joinable()) {
    if (thread_.get_id() == self) {
      retired_threads_.push_back(std::move(thread_));
    } else {
      to_join.push_back(std::move(thread_));
    }
  }
  std::erase_if(retired_threads_, [&](std::thread& t) {
    if (!t.joinable()) return true;
    if (t.get_id() == self) return false;
    to_join.push_back(std::move(t));
    return true;
  });
  stop_cv_.notify_all();
}

void RecordedCameraSource::JoinThreads(std::vector<std::thread>& threads) {
  for (auto& t : threads) {
    if (t.joinable()) t.join();
  }
}

bool RecordedCameraSource::Start() {
  if (IsInFrameCallback()) {
    Report(ConnectionState::kError,
           "recording: cannot restart inside frame callback");
    return false;
  }
  std::unique_lock lifecycle(lifecycle_mutex_);
  if (IsSelfPlayThread()) {
    if (active_teardowns_ > 0) {
      return false;
    }
    std::vector<std::thread> other_threads;
    StopAndDetachThreadsLocked(other_threads);
    if (!other_threads.empty()) {
      ++active_teardowns_;
      lifecycle.unlock();
      JoinThreads(other_threads);
      other_threads.clear();
      lifecycle.lock();
      --active_teardowns_;
      lifecycle_cv_.notify_all();
      if (active_teardowns_ > 0) {
        return false;
      }
    }
  } else {
    for (;;) {
      std::vector<std::thread> to_join;
      StopAndDetachThreadsLocked(to_join);
      if (!to_join.empty()) {
        ++active_teardowns_;
        lifecycle.unlock();
        JoinThreads(to_join);
        to_join.clear();
        lifecycle.lock();
        --active_teardowns_;
        lifecycle_cv_.notify_all();
        continue;
      }
      if (active_teardowns_ > 0) {
        lifecycle_cv_.wait(lifecycle, [&] {
          return active_teardowns_ == 0 || HasJoinableThreadsLocked();
        });
        continue;
      }
      break;
    }
  }

  auto fail = [&](std::string_view message) {
    running_.store(false, std::memory_order_release);
    connected_.store(false, std::memory_order_release);
    lifecycle.unlock();
    Report(ConnectionState::kError, message);
    return false;
  };

  uint32_t configured_width = 0;
  uint32_t configured_height = 0;
  {
    std::lock_guard lock(state_mutex_);
    sequence_.reset();
    configured_width = config_.width;
    configured_height = config_.height;
  }

  std::string error;
  auto sequence = ImageSequence::Open(config_.device_path, config_.fps, &error);
  if (!sequence) {
    return fail("recording: " + error);
  }
  int width = 0, height = 0;
  std::vector<std::uint8_t> pixels;
  if (!sequence->LoadGray(0, &pixels, &width, &height, &error)) {
    return fail("recording: " + error);
  }
  if (width <= 0 || height <= 0) {
    return fail("recording: invalid frame dimensions");
  }
  if ((configured_width != 0 &&
       static_cast<uint32_t>(width) != configured_width) ||
      (configured_height != 0 &&
       static_cast<uint32_t>(height) != configured_height)) {
    return fail("recording frames are " + std::to_string(width) + "x" +
                std::to_string(height) + ", the camera is configured " +
                std::to_string(configured_width) + "x" +
                std::to_string(configured_height));
  }

  auto shared_sequence =
      std::make_shared<const ImageSequence>(std::move(*sequence));
  // The recording ends one frame period after its last frame, so the
  // consumer has that long to take the last frame before the camera goes.
  constexpr double kMaxPtsNs =
      static_cast<double>(std::numeric_limits<std::int64_t>::max());
  std::int64_t period = 100'000'000;
  if (shared_sequence->mean_period_ns() > 0) {
    period = shared_sequence->mean_period_ns();
  } else if (std::isfinite(config_.fps) && config_.fps > 0.0) {
    const double period_d = 1e9 / config_.fps;
    if (std::isfinite(period_d) && period_d > 0.0 && period_d < kMaxPtsNs) {
      period = static_cast<std::int64_t>(period_d);
    }
  }
  const std::int64_t last_pts =
      (*shared_sequence)[shared_sequence->size() - 1].pts_ns;
  const std::int64_t end_ns =
      last_pts <= std::numeric_limits<std::int64_t>::max() - period
          ? last_pts + period
          : std::numeric_limits<std::int64_t>::max();
  const auto frame_width = static_cast<std::uint32_t>(width);
  const auto frame_height = static_cast<std::uint32_t>(height);
  {
    std::lock_guard stop_lock(stop_mutex_);
    std::lock_guard state_lock(state_mutex_);
    sequence_ = shared_sequence;
    config_.width = frame_width;
    config_.height = frame_height;
    end_ns_ = end_ns;
    ++session_;
    const std::uint64_t session = session_;
    next_ = 0;
    manual_start_ns_.reset();
    stop_ = false;
    const std::uint64_t generation = ++generation_;
    if (pacing_ == Pacing::kRealtime) {
      const auto start_ns = HostNowNs();
      thread_ =
          std::thread([this, seq = std::move(shared_sequence), generation,
                       session, frame_width, frame_height, start_ns, end_ns] {
            Play(seq, generation, session, frame_width, frame_height, start_ns,
                 end_ns);
          });
    }
    running_.store(true, std::memory_order_release);
    connected_.store(true, std::memory_order_release);
  }
  return true;
}

void RecordedCameraSource::Stop() {
  std::unique_lock lifecycle(lifecycle_mutex_);
  if (IsSelfPlayThread()) {
    std::vector<std::thread> other_threads;
    StopAndDetachThreadsLocked(other_threads);
    if (!other_threads.empty()) {
      ++active_teardowns_;
      lifecycle.unlock();
      JoinThreads(other_threads);
      other_threads.clear();
      lifecycle.lock();
      --active_teardowns_;
      lifecycle_cv_.notify_all();
    }
    return;
  }
  for (;;) {
    std::vector<std::thread> to_join;
    StopAndDetachThreadsLocked(to_join);
    if (!to_join.empty()) {
      ++active_teardowns_;
      lifecycle.unlock();
      JoinThreads(to_join);
      to_join.clear();
      lifecycle.lock();
      --active_teardowns_;
      lifecycle_cv_.notify_all();
      continue;
    }
    if (active_teardowns_ > 0) {
      lifecycle_cv_.wait(lifecycle, [&] {
        return active_teardowns_ == 0 || HasJoinableThreadsLocked();
      });
      continue;
    }
    return;
  }
}

void RecordedCameraSource::Finish(std::uint64_t generation) {
  {
    std::lock_guard lock(stop_mutex_);
    if (stop_ || generation_ != generation) return;
    running_.store(false, std::memory_order_release);
    connected_.store(false, std::memory_order_release);
  }
  Report(ConnectionState::kDisconnected, "end of recording");
}

bool RecordedCameraSource::Decode(const ImageSequence& sequence,
                                  std::size_t index, std::uint32_t width,
                                  std::uint32_t height,
                                  std::vector<std::uint8_t>* pixels) {
  int decoded_width = 0, decoded_height = 0;
  std::string error;
  if (!sequence.LoadGray(index, pixels, &decoded_width, &decoded_height,
                         &error) ||
      static_cast<std::uint32_t>(decoded_width) != width ||
      static_cast<std::uint32_t>(decoded_height) != height) {
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

void RecordedCameraSource::Emit(std::size_t index, std::int64_t capture_ns,
                                std::uint64_t session, std::uint32_t width,
                                std::uint32_t height,
                                const std::vector<std::uint8_t>& pixels) {
  Frame frame(config_.camera_id, session, index,
              talos::event::MonotonicTime::from_nanos(capture_ns), width,
              height, width, PixelFormat::kGray8, pixels.data(), pixels.size());
  // The recording's own timeline, not asserted to be the exposure midpoint.
  frame.set_timestamp_kind(TimestampKind::kPresentation);
  frame.set_clock_domain(ClockDomain::kHostMonotonic);
  FrameCallback callback;
  {
    std::lock_guard lock(callback_mutex_);
    callback = frame_callback_;
  }
  if (!callback) return;
  struct FrameCallbackScope {
    const RecordedCameraSource* prev_source;
    bool prev_in_frame_callback;
    explicit FrameCallbackScope(const RecordedCameraSource* self)
        : prev_source(t_play_source),
          prev_in_frame_callback(t_in_frame_callback) {
      t_play_source = self;
      t_in_frame_callback = true;
    }
    ~FrameCallbackScope() {
      t_in_frame_callback = prev_in_frame_callback;
      t_play_source = prev_source;
    }
  } scope{this};
  callback(frame);
}

std::size_t RecordedCameraSource::DeliverUntil(std::int64_t now_ns) {
  if (pacing_ != Pacing::kManual ||
      !connected_.load(std::memory_order_acquire)) {
    return 0;
  }
  std::shared_ptr<const ImageSequence> sequence;
  std::uint64_t generation = 0;
  std::uint64_t session = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::int64_t start_ns = 0;
  std::int64_t end_ns = 0;
  {
    std::lock_guard stop_lock(stop_mutex_);
    if (stop_) return 0;
    generation = generation_;
    std::lock_guard state_lock(state_mutex_);
    if (!sequence_) return 0;
    sequence = sequence_;
    if (!manual_start_ns_.has_value()) {
      manual_start_ns_ = now_ns;
      start_ns = now_ns;
    } else {
      start_ns = *manual_start_ns_;
    }
    end_ns = end_ns_;
    session = session_;
    width = config_.width;
    height = config_.height;
  }
  std::vector<std::uint8_t> pixels;
  pixels.reserve(static_cast<std::size_t>(width) * height);
  std::size_t delivered = 0;
  for (;;) {
    std::size_t index = 0;
    std::int64_t pts_ns = 0;
    {
      std::lock_guard stop_lock(stop_mutex_);
      if (stop_ || generation_ != generation) return delivered;
      std::lock_guard state_lock(state_mutex_);
      if (next_ >= sequence->size() ||
          !ReachedOffsetNs(now_ns, start_ns, (*sequence)[next_].pts_ns)) {
        break;
      }
      index = next_;
      pts_ns = (*sequence)[index].pts_ns;
      ++next_;
    }
    const bool decoded = Decode(*sequence, index, width, height, &pixels);
    if (IsCancelled(generation)) return delivered;
    if (decoded) {
      Emit(index, SaturatingAddNs(start_ns, pts_ns), session, width, height,
           pixels);
      ++delivered;
    }
  }
  bool should_finish = false;
  {
    std::lock_guard stop_lock(stop_mutex_);
    if (!stop_ && generation_ == generation) {
      std::lock_guard state_lock(state_mutex_);
      if (next_ == sequence->size() &&
          ReachedOffsetNs(now_ns, start_ns, end_ns)) {
        should_finish = true;
      }
    }
  }
  if (should_finish) {
    Finish(generation);
  }
  return delivered;
}

bool RecordedCameraSource::WaitUntil(std::int64_t due_ns,
                                     std::uint64_t generation) {
  std::unique_lock lock(stop_mutex_);
  for (auto now_ns = HostNowNs();
       !stop_ && generation_ == generation && now_ns < due_ns;
       now_ns = HostNowNs()) {
    stop_cv_.wait_for(lock,
                      std::chrono::nanoseconds(WaitSliceNs(now_ns, due_ns)));
  }
  return !stop_ && generation_ == generation;
}

void RecordedCameraSource::Play(
    const std::shared_ptr<const ImageSequence>& sequence,
    std::uint64_t generation, std::uint64_t session, std::uint32_t width,
    std::uint32_t height, std::int64_t start_ns, std::int64_t end_ns) {
  struct PlayScope {
    const RecordedCameraSource* prev_source;
    bool prev_in_frame_callback;
    explicit PlayScope(const RecordedCameraSource* self)
        : prev_source(t_play_source),
          prev_in_frame_callback(t_in_frame_callback) {
      t_play_source = self;
      t_in_frame_callback = false;
    }
    ~PlayScope() {
      t_in_frame_callback = prev_in_frame_callback;
      t_play_source = prev_source;
    }
  } scope{this};

  talos::process::ConfigureBackgroundThread();
  if (!sequence) return;
  const auto period = sequence->mean_period_ns();
  std::vector<std::uint8_t> pixels;
  pixels.reserve(static_cast<std::size_t>(width) * height);
  for (std::size_t i = 0; i < sequence->size(); ++i) {
    if (IsCancelled(generation)) return;
    const auto due_ns = SaturatingAddNs(start_ns, (*sequence)[i].pts_ns);
    if (i > 0 && HostNowNs() > SaturatingAddNs(due_ns, period)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    // Decode ahead of the frame's time, then deliver it at that time.
    const bool decoded = Decode(*sequence, i, width, height, &pixels);
    if (!WaitUntil(due_ns, generation)) return;
    if (decoded) Emit(i, due_ns, session, width, height, pixels);
  }
  if (WaitUntil(SaturatingAddNs(start_ns, end_ns), generation)) {
    Finish(generation);
  }
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
