#include "talOS/vision/capture/avfoundation_source.h"
#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include "talOS/events/os/poller.h"

@interface TalosCaptureDelegate
    : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate> {
 @public
  std::function<void(CMSampleBufferRef)> callback;
}
@end
@implementation TalosCaptureDelegate
- (void)captureOutput:(AVCaptureOutput*)output
    didOutputSampleBuffer:(CMSampleBufferRef)sample
           fromConnection:(AVCaptureConnection*)connection {
  if (callback) callback(sample);
}
@end

namespace talos::vision {
namespace {
// The duration to set for `fps` in `range`, or nullopt (FrameDurationFor).
std::optional<CMTime> Duration(double fps, AVFrameRateRange* range) {
  const auto duration = FrameDurationFor(
      fps, range.minFrameRate, range.maxFrameRate,
      {range.minFrameDuration.value, range.minFrameDuration.timescale},
      {range.maxFrameDuration.value, range.maxFrameDuration.timescale});
  if (!duration) return std::nullopt;
  return CMTimeMake(duration->value, duration->timescale);
}
std::vector<CameraMode> Modes(AVCaptureDevice* device) {
  std::vector<CameraMode> modes;
  for (AVCaptureDeviceFormat* format in device.formats) {
    auto d = CMVideoFormatDescriptionGetDimensions(format.formatDescription);
    for (AVFrameRateRange* range in format.videoSupportedFrameRateRanges) {
      for (double fps : {15., 30., 60., 90., 100., 120.,
                         std::round(range.minFrameRate),
                         std::round(range.maxFrameRate)}) {
        if (!Duration(fps, range)) continue;
        CameraMode mode{static_cast<uint32_t>(d.width),
                        static_cast<uint32_t>(d.height), fps,
                        PixelFormat::kBgra8};
        if (std::find(modes.begin(), modes.end(), mode) == modes.end())
          modes.push_back(mode);
      }
    }
  }
  return modes;
}
struct DetachedSession {
  std::mutex op_mutex;
  bool stopped = false;
  AVCaptureSession* session = nil;
  AVCaptureVideoDataOutput* output = nil;
  TalosCaptureDelegate* delegate = nil;
  dispatch_queue_t dispatch = nullptr;
};
void StopSessionOnce(DetachedSession& detached) {
  std::lock_guard<std::mutex> lock(detached.op_mutex);
  if (detached.stopped) return;
  detached.stopped = true;
  @autoreleasepool {
    [detached.session stopRunning];
    [detached.output setSampleBufferDelegate:nil queue:nullptr];
  }
}
class Source final : public CameraSource {
 public:
  Source(uint32_t id, std::string device)
      : id_(id), device_id_(std::move(device)) {}
  ~Source() override { Stop(); }
  uint32_t camera_id() const override { return id_; }
  std::string device_id() const override {
    std::lock_guard<std::mutex> lock(device_mutex_);
    return device_id_;
  }
  std::string name() const override { return device_id(); }
  std::vector<CameraMode> SupportedModes() const override {
    const std::string id = device_id();
    @autoreleasepool {
      return Modes([AVCaptureDevice deviceWithUniqueID:@(id.c_str())]);
    }
  }
  CameraMode CurrentMode() const override {
    std::lock_guard<std::mutex> lock(device_mutex_);
    return mode_;
  }
  bool SetMode(const CameraMode& mode) override {
    if (!mode.width || !mode.height || !std::isfinite(mode.fps) ||
        mode.fps <= 0 || mode.fps > 2147483.0)
      return false;
    std::unique_lock<std::mutex> lifecycle(lifecycle_mutex_);
    if (!PrepareQuiescentForConfigLocked(lifecycle)) return false;
    std::lock_guard<std::mutex> lock(device_mutex_);
    mode_ = mode;
    return true;
  }
  CameraControls GetControls() const override {
    std::lock_guard<std::mutex> lock(device_mutex_);
    return controls_;
  }
  bool SetControls(const CameraControls& c) override {
    std::unique_lock<std::mutex> lifecycle(lifecycle_mutex_);
    if (!PrepareQuiescentForConfigLocked(lifecycle)) return false;
    std::lock_guard<std::mutex> lock(device_mutex_);
    controls_ = c;
    return true;
  }
  void SetFrameCallback(FrameCallback cb) override {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(cb);
  }
  void SetConnectionCallback(ConnectionCallback cb) override {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    connection_ = std::move(cb);
  }
  bool IsRunning() const override {
    return running_.load(std::memory_order_acquire);
  }
  bool IsConnected() const override {
    if (!running_.load(std::memory_order_acquire) ||
        !connected_.load(std::memory_order_acquire)) {
      return false;
    }
    @autoreleasepool {
      AVCaptureDevice* device = nil;
      {
        std::lock_guard<std::mutex> lock(device_mutex_);
        device = device_;
      }
      return device != nil && device.connected;
    }
  }
  bool controls_applied() const override {
    return controls_applied_.load(std::memory_order_acquire);
  }
  bool Start() override {
    if (IsSelfReceive()) return false;
    std::unique_lock<std::mutex> lifecycle(lifecycle_mutex_);
    for (;;) {
      if (running_.load(std::memory_order_acquire)) return true;
      std::vector<std::shared_ptr<DetachedSession>> to_drain;
      CollectRetiredLocked(to_drain);
      if (!to_drain.empty()) {
        ++active_teardowns_;
        lifecycle.unlock();
        for (const auto& d : to_drain) TeardownDetached(d);
        to_drain.clear();
        lifecycle.lock();
        --active_teardowns_;
        lifecycle_cv_.notify_all();
        continue;
      }
      if (active_teardowns_ > 0) {
        lifecycle_cv_.wait(lifecycle, [&] {
          return active_teardowns_ == 0 ||
                 running_.load(std::memory_order_acquire) ||
                 !retired_sessions_.empty();
        });
        continue;
      }
      break;
    }
    auto fail = [&](ConnectionState state, const std::string& message) {
      std::shared_ptr<DetachedSession> detached = DetachSessionLocked();
      if (detached) ++active_teardowns_;
      const std::string full = device_id() + ": " + message;
      lifecycle.unlock();
      if (detached) {
        TeardownDetached(detached);
        lifecycle.lock();
        --active_teardowns_;
        lifecycle_cv_.notify_all();
        lifecycle.unlock();
      }
      NotifyConnection(state, full);
      return false;
    };
    @autoreleasepool {
      std::string device_id;
      CameraMode mode;
      CameraControls controls;
      {
        std::lock_guard<std::mutex> lock(device_mutex_);
        device_id = device_id_;
        mode = mode_;
        controls = controls_;
      }
      if (device_id.empty()) {
        const auto devices = DiscoverCaptureDevices();
        if (id_ >= devices.size())
          return fail(ConnectionState::kDisconnected,
                      "no camera at index " + std::to_string(id_));
        device_id = devices[id_].id;
        {
          std::lock_guard<std::mutex> lock(device_mutex_);
          device_id_ = device_id;
        }
      }
      AVCaptureDevice* device =
          [AVCaptureDevice deviceWithUniqueID:@(device_id.c_str())];
      {
        std::lock_guard<std::mutex> lock(device_mutex_);
        device_ = device;
      }
      if (!device_)
        return fail(ConnectionState::kDisconnected,
                    "no camera with uniqueID " + device_id);
      if (!device_.connected)
        return fail(ConnectionState::kDisconnected,
                    "camera " + device_id + " is not connected");
      const auto permission =
          [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
      if (permission == AVAuthorizationStatusNotDetermined) {
        // Request once without blocking the detector worker. Its normal device
        // retry starts capture after the user responds to the macOS prompt.
        static std::atomic<bool> requested{false};
        if (!requested.exchange(true)) {
          [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo
                                   completionHandler:^(BOOL granted){
                                   }];
        }
        return fail(ConnectionState::kDisconnected,
                    "Waiting for macOS camera permission");
      }
      if (permission != AVAuthorizationStatusAuthorized) {
        return fail(ConnectionState::kError,
                    "Camera access is not authorized for this process; "
                    "enable camera access in macOS Privacy settings");
      }
      NSError* error = nil;
      session_ = [[AVCaptureSession alloc] init];
      [session_ beginConfiguration];
      AVCaptureDeviceInput* input =
          [AVCaptureDeviceInput deviceInputWithDevice:device_ error:&error];
      if (!input) {
        [session_ commitConfiguration];
        return fail(ConnectionState::kError,
                    "cannot open camera input: " + Describe(error));
      }
      if (![session_ canAddInput:input]) {
        [session_ commitConfiguration];
        return fail(ConnectionState::kError,
                    "capture session refused the camera input (in use?)");
      }
      [session_ addInput:input];
      if (![device_ lockForConfiguration:&error]) {
        [session_ commitConfiguration];
        return fail(ConnectionState::kError,
                    "cannot lock camera configuration: " + Describe(error));
      }
      bool selected = false;
      for (AVCaptureDeviceFormat* format in device_.formats) {
        auto d =
            CMVideoFormatDescriptionGetDimensions(format.formatDescription);
        if (d.width != mode.width || d.height != mode.height) continue;
        for (AVFrameRateRange* range in format.videoSupportedFrameRateRanges) {
          const auto duration = Duration(mode.fps, range);
          if (!duration) continue;
          device_.activeFormat = format;
          device_.activeVideoMinFrameDuration = *duration;
          device_.activeVideoMaxFrameDuration = *duration;
          selected = true;
          break;
        }
        if (selected) break;
      }
      // macOS exposes exposure-mode selection, but not custom duration/ISO.
      // Keep streaming when unsupported and surface the failed request.
      const bool controls_applied =
          controls.auto_exposure && controls.auto_gain &&
          [device_ isExposureModeSupported:
                       AVCaptureExposureModeContinuousAutoExposure];
      controls_applied_.store(controls_applied, std::memory_order_release);
      if (controls_applied)
        device_.exposureMode = AVCaptureExposureModeContinuousAutoExposure;
      [device_ unlockForConfiguration];
      if (!selected) {
        [session_ commitConfiguration];
        return fail(ConnectionState::kError,
                    "camera has no " + std::to_string(mode.width) + "x" +
                        std::to_string(mode.height) + " format at " +
                        std::to_string(mode.fps) + " fps");
      }
      output_ = [[AVCaptureVideoDataOutput alloc] init];
      output_.alwaysDiscardsLateVideoFrames = YES;
      OSType requested = mode.format == PixelFormat::kGray8
                             ? kCVPixelFormatType_OneComponent8
                         : mode.format == PixelFormat::kYuyv
                             ? kCVPixelFormatType_422YpCbCr8_yuvs
                             : kCVPixelFormatType_32BGRA;
      if (![output_.availableVideoCVPixelFormatTypes
              containsObject:@(requested)])
        requested = kCVPixelFormatType_32BGRA;
      const PixelFormat effective_format =
          requested == kCVPixelFormatType_OneComponent8
              ? PixelFormat::kGray8
          : requested == kCVPixelFormatType_422YpCbCr8_yuvs
              ? PixelFormat::kYuyv
              : PixelFormat::kBgra8;
      {
        std::lock_guard<std::mutex> lock(device_mutex_);
        mode_.format = effective_format;
      }
      output_.videoSettings =
          @{(id)kCVPixelBufferPixelFormatTypeKey : @(requested)};
      delegate_ = [[TalosCaptureDelegate alloc] init];
      AVCaptureSession* session = session_;
      delegate_->callback = [this, session](CMSampleBufferRef sample) {
        Receive(sample, session);
      };
      // Frame delivery runs at the QoS that
      // talos::process::ConfigureBackgroundThread gives camera workers,
      // below every node loop.
      dispatch_ = dispatch_queue_create(
          "talos.camera", dispatch_queue_attr_make_with_qos_class(
                              DISPATCH_QUEUE_SERIAL, QOS_CLASS_UTILITY, 0));
      dispatch_queue_set_specific(dispatch_, &kDispatchQueueKey,
                                  (__bridge void*)dispatch_, nullptr);
      [output_ setSampleBufferDelegate:delegate_ queue:dispatch_];
      if (![session_ canAddOutput:output_]) {
        [session_ commitConfiguration];
        return fail(ConnectionState::kError,
                    "capture session refused the video output");
      }
      [session_ addOutput:output_];
      [session_ commitConfiguration];
      session_id_ = event::steady_now().nanos();
      sequence_ = 0;
      running_.store(true, std::memory_order_release);
      [session_ startRunning];
      if (!session_.running) {
        return fail(ConnectionState::kError, "Capture session did not start");
      }
      connected_.store(true, std::memory_order_release);
    }
    lifecycle.unlock();
    NotifyConnection(ConnectionState::kConnected, "");
    return true;
  }
  void Stop() override {
    std::unique_lock<std::mutex> lifecycle(lifecycle_mutex_);
    running_.store(false, std::memory_order_release);
    connected_.store(false, std::memory_order_release);
    if (IsSelfReceive()) {
      std::shared_ptr<DetachedSession> detached = DetachSessionLocked();
      if (detached) {
        retired_sessions_.push_back(detached);
      }
      lifecycle.unlock();
      if (detached) {
        dispatch_async(dispatch_get_global_queue(QOS_CLASS_UTILITY, 0), ^{
          StopSessionOnce(*detached);
        });
      }
      return;
    }
    for (;;) {
      running_.store(false, std::memory_order_release);
      connected_.store(false, std::memory_order_release);
      std::vector<std::shared_ptr<DetachedSession>> to_drain;
      if (auto active = DetachSessionLocked()) {
        to_drain.push_back(std::move(active));
      }
      CollectRetiredLocked(to_drain);
      if (!to_drain.empty()) {
        ++active_teardowns_;
        lifecycle.unlock();
        for (const auto& d : to_drain) TeardownDetached(d);
        to_drain.clear();
        lifecycle.lock();
        --active_teardowns_;
        lifecycle_cv_.notify_all();
        continue;
      }
      if (active_teardowns_ > 0) {
        lifecycle_cv_.wait(lifecycle, [&] {
          return active_teardowns_ == 0 || session_ != nil ||
                 !retired_sessions_.empty();
        });
        continue;
      }
      return;
    }
  }

  uint64_t dropped_frames() const override { return dropped_frames_.load(); }

 private:
  static inline thread_local const Source* t_current_receive_source = nullptr;

  bool IsSelfReceive() const { return t_current_receive_source == this; }

  void CollectRetiredLocked(
      std::vector<std::shared_ptr<DetachedSession>>& out) {
    for (auto& d : retired_sessions_) {
      if (d) out.push_back(std::move(d));
    }
    retired_sessions_.clear();
  }
  bool PrepareQuiescentForConfigLocked(
      std::unique_lock<std::mutex>& lifecycle) {
    if (IsSelfReceive()) return false;
    for (;;) {
      if (running_.load(std::memory_order_acquire)) return false;
      std::vector<std::shared_ptr<DetachedSession>> to_drain;
      CollectRetiredLocked(to_drain);
      if (!to_drain.empty()) {
        ++active_teardowns_;
        lifecycle.unlock();
        for (const auto& d : to_drain) TeardownDetached(d);
        to_drain.clear();
        lifecycle.lock();
        --active_teardowns_;
        lifecycle_cv_.notify_all();
        continue;
      }
      if (active_teardowns_ > 0) {
        lifecycle_cv_.wait(lifecycle, [&] {
          return active_teardowns_ == 0 ||
                 running_.load(std::memory_order_acquire) ||
                 !retired_sessions_.empty();
        });
        continue;
      }
      return !running_.load(std::memory_order_acquire);
    }
  }
  std::shared_ptr<DetachedSession> DetachSessionLocked() {
    running_.store(false, std::memory_order_release);
    connected_.store(false, std::memory_order_release);
    std::shared_ptr<DetachedSession> detached;
    if (session_ != nil || output_ != nil || delegate_ != nil ||
        dispatch_ != nullptr) {
      detached = std::make_shared<DetachedSession>();
      detached->session = session_;
      detached->output = output_;
      detached->delegate = delegate_;
      detached->dispatch = dispatch_;
    }
    session_ = nil;
    output_ = nil;
    delegate_ = nil;
    dispatch_ = nullptr;
    {
      std::lock_guard<std::mutex> lock(device_mutex_);
      device_ = nil;
    }
    return detached;
  }
  void TeardownDetached(const std::shared_ptr<DetachedSession>& detached) {
    if (!detached) return;
    @autoreleasepool {
      StopSessionOnce(*detached);
      const bool on_this_queue =
          detached->dispatch &&
          dispatch_get_specific(&kDispatchQueueKey) ==
              (__bridge void*)detached->dispatch;
      if (detached->dispatch && !on_this_queue) {
        dispatch_sync(detached->dispatch, ^{
                      });
      }
      if (detached->delegate) detached->delegate->callback = {};
      std::lock_guard<std::mutex> lock(detached->op_mutex);
      detached->delegate = nil;
      detached->output = nil;
      detached->session = nil;
      detached->dispatch = nullptr;
    }
  }
  void NotifyConnection(ConnectionState state, const std::string& message) {
    ConnectionCallback cb;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      cb = connection_;
    }
    if (cb) cb(state, message);
  }
  static std::string Describe(NSError* error) {
    if (!error) return "unknown error";
    const char* desc = error.localizedDescription.UTF8String;
    return desc ? std::string(desc) : std::string("unknown error");
  }
  void Receive(CMSampleBufferRef sample, AVCaptureSession* session) {
    struct ReceiveScope {
      const Source* prev;
      explicit ReceiveScope(const Source* self)
          : prev(t_current_receive_source) {
        t_current_receive_source = self;
      }
      ~ReceiveScope() { t_current_receive_source = prev; }
    } scope{this};
    if (!running_.load(std::memory_order_acquire)) return;
    CVPixelBufferRef pixels = CMSampleBufferGetImageBuffer(sample);
    if (!pixels || CVPixelBufferIsPlanar(pixels) ||
        CVPixelBufferLockBaseAddress(pixels, kCVPixelBufferLock_ReadOnly) !=
            kCVReturnSuccess) {
      // Counted by the consumer through dropped_frames().
      ++dropped_frames_;
      return;
    }
    struct PixelBufferUnlockGuard {
      CVPixelBufferRef buffer;
      ~PixelBufferUnlockGuard() {
        CVPixelBufferUnlockBaseAddress(buffer, kCVPixelBufferLock_ReadOnly);
      }
    } unlock_guard{pixels};
    PixelFormat format{};
    switch (CVPixelBufferGetPixelFormatType(pixels)) {
      case kCVPixelFormatType_OneComponent8:
        format = PixelFormat::kGray8;
        break;
      case kCVPixelFormatType_422YpCbCr8_yuvs:
        format = PixelFormat::kYuyv;
        break;
      case kCVPixelFormatType_32BGRA:
        format = PixelFormat::kBgra8;
        break;
      default:
        ++dropped_frames_;
        return;
    }
    const auto* base =
        static_cast<const uint8_t*>(CVPixelBufferGetBaseAddress(pixels));
    const size_t width = CVPixelBufferGetWidth(pixels);
    const size_t height = CVPixelBufferGetHeight(pixels);
    const size_t bytes_per_row = CVPixelBufferGetBytesPerRow(pixels);
    const size_t bpp = BytesPerPixel(format);
    constexpr size_t kMaxUint32 = std::numeric_limits<uint32_t>::max();
    if (base == nullptr || width == 0 || height == 0 || width > kMaxUint32 ||
        height > kMaxUint32 || bytes_per_row > kMaxUint32 ||
        width > std::numeric_limits<size_t>::max() / bpp ||
        bytes_per_row < width * bpp ||
        bytes_per_row > std::numeric_limits<size_t>::max() / height) {
      ++dropped_frames_;
      return;
    }
    const size_t size = bytes_per_row * height;
    // The frame borrows the locked pixel buffer until the callback returns;
    // the consumer copies what it keeps (FrameMailbox).
    Frame frame(id_, session_id_, sequence_++, event::Poller::now(),
                static_cast<uint32_t>(width), static_cast<uint32_t>(height),
                static_cast<uint32_t>(bytes_per_row), format, base, size);
    const CMTime pts = CMSampleBufferGetPresentationTimeStamp(sample);
    int64_t raw_timestamp_ns = 0;
    if (CMTIME_IS_NUMERIC(pts)) {
      const double pts_ns = CMTimeGetSeconds(pts) * 1e9;
      constexpr double kMinInt64 =
          static_cast<double>(std::numeric_limits<int64_t>::min());
      constexpr double kMaxInt64Exclusive =
          -static_cast<double>(std::numeric_limits<int64_t>::min());
      if (std::isfinite(pts_ns) && pts_ns >= kMinInt64 &&
          pts_ns < kMaxInt64Exclusive) {
        raw_timestamp_ns = static_cast<int64_t>(pts_ns);
      }
    }
    frame.set_raw_timestamp_ns(raw_timestamp_ns);
    frame.set_timestamp_kind(TimestampKind::kDelivery);
    frame.set_mapping_quality(TimestampMappingQuality::kEstimated);
    if (session.synchronizationClock && CMTIME_IS_NUMERIC(pts)) {
      const CMTime hostPTS = CMSyncConvertTime(
          pts, session.synchronizationClock, CMClockGetHostTimeClock());
      const double age = CMTimeGetSeconds(
          CMTimeSubtract(CMClockGetTime(CMClockGetHostTimeClock()), hostPTS));
      if (std::isfinite(age) && age >= 0 && age < 5) {
        frame.set_capture_time(event::MonotonicTime::from_nanos(
            event::Poller::now().nanos() - static_cast<int64_t>(age * 1e9)));
        frame.set_timestamp_kind(TimestampKind::kPresentation);
        frame.set_mapping_quality(TimestampMappingQuality::kInterpolated);
      }
    }
    FrameCallback cb;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      cb = callback_;
    }
    if (cb) cb(frame);
  }
  uint32_t id_;
  std::string device_id_;
  CameraMode mode_{1280, 720, 30, PixelFormat::kBgra8};
  CameraControls controls_{.auto_exposure = true, .auto_gain = true};
  mutable std::mutex callback_mutex_;
  FrameCallback callback_;
  ConnectionCallback connection_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<uint64_t> dropped_frames_{0};
  std::atomic<bool> controls_applied_{true};
  uint64_t session_id_ = 0, sequence_ = 0;
  mutable std::mutex lifecycle_mutex_;
  std::condition_variable lifecycle_cv_;
  size_t active_teardowns_{0};
  std::vector<std::shared_ptr<DetachedSession>> retired_sessions_;
  mutable std::mutex device_mutex_;
  static constexpr char kDispatchQueueKey = 0;
  AVCaptureDevice* device_ = nil;
  AVCaptureSession* session_ = nil;
  AVCaptureVideoDataOutput* output_ = nil;
  TalosCaptureDelegate* delegate_ = nil;
  dispatch_queue_t dispatch_ = nullptr;
};
}  // namespace
std::vector<CaptureDevice> DiscoverCaptureDevices() {
  @autoreleasepool {
    std::vector<CaptureDevice> result;
    AVCaptureDeviceDiscoverySession* discovery =
        [AVCaptureDeviceDiscoverySession
            discoverySessionWithDeviceTypes:@[
              AVCaptureDeviceTypeBuiltInWideAngleCamera,
              AVCaptureDeviceTypeExternalUnknown
            ]
                                  mediaType:AVMediaTypeVideo
                                   position:AVCaptureDevicePositionUnspecified];
    for (AVCaptureDevice* device in discovery.devices) {
      const char* uid = device.uniqueID.UTF8String;
      if (!uid) continue;
      const char* name = device.localizedName.UTF8String;
      result.push_back({uid, name ? name : "", Modes(device)});
    }
    return result;
  }
}
std::unique_ptr<CameraSource> CreateCaptureSource(uint32_t id,
                                                  const std::string& device) {
  return std::make_unique<Source>(id, device);
}
std::unique_ptr<CameraSource> CreatePlatformCameraSource(
    const CameraSourceConfig& config) {
  auto source = std::make_unique<Source>(config.camera_id, config.device_path);
  if (!source->SetControls({config.exposure_ms, config.gain,
                            config.auto_exposure, config.auto_gain})) {
    return nullptr;
  }
  if (!source->SetMode(
          {config.width, config.height, config.fps, config.format})) {
    return nullptr;
  }
  return source;
}
}  // namespace talos::vision
