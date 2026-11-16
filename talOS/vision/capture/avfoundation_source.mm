#include "talOS/vision/capture/avfoundation_source.h"
#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <functional>
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
class Source final : public CameraSource {
 public:
  Source(uint32_t id, std::string device)
      : id_(id), device_id_(std::move(device)) {}
  ~Source() override { Stop(); }
  uint32_t camera_id() const override { return id_; }
  std::string device_id() const override { return device_id_; }
  std::string name() const override { return device_id_; }
  std::vector<CameraMode> SupportedModes() const override {
    @autoreleasepool {
      return Modes([AVCaptureDevice deviceWithUniqueID:@(device_id_.c_str())]);
    }
  }
  CameraMode CurrentMode() const override { return mode_; }
  bool SetMode(const CameraMode& mode) override {
    if (running_) return false;
    if (!mode.width || !mode.height || !std::isfinite(mode.fps) ||
        mode.fps <= 0 || mode.fps > 2147483.0)
      return false;
    mode_ = mode;
    return true;
  }
  CameraControls GetControls() const override { return controls_; }
  bool SetControls(const CameraControls& c) override {
    if (running_) return false;
    controls_ = c;
    return true;
  }
  void SetFrameCallback(FrameCallback cb) override {
    callback_ = std::move(cb);
  }
  void SetConnectionCallback(ConnectionCallback cb) override {
    connection_ = std::move(cb);
  }
  bool IsRunning() const override { return running_; }
  bool IsConnected() const override { return running_ && device_.connected; }
  bool controls_applied() const override { return controls_applied_; }
  bool Start() override {
    if (running_) return true;
    @autoreleasepool {
      if (device_id_.empty()) {
        const auto devices = DiscoverCaptureDevices();
        if (id_ >= devices.size())
          return Fail(ConnectionState::kDisconnected,
                      "no camera at index " + std::to_string(id_));
        device_id_ = devices[id_].id;
      }
      device_ = [AVCaptureDevice deviceWithUniqueID:@(device_id_.c_str())];
      if (!device_)
        return Fail(ConnectionState::kDisconnected,
                    "no camera with uniqueID " + device_id_);
      if (!device_.connected)
        return Fail(ConnectionState::kDisconnected,
                    "camera " + device_id_ + " is not connected");
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
        return Fail(ConnectionState::kDisconnected,
                    "Waiting for macOS camera permission");
      }
      if (permission != AVAuthorizationStatusAuthorized) {
        return Fail(ConnectionState::kError,
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
        return Fail(ConnectionState::kError,
                    "cannot open camera input: " + Describe(error));
      }
      if (![session_ canAddInput:input]) {
        [session_ commitConfiguration];
        return Fail(ConnectionState::kError,
                    "capture session refused the camera input (in use?)");
      }
      [session_ addInput:input];
      if (![device_ lockForConfiguration:&error]) {
        [session_ commitConfiguration];
        return Fail(ConnectionState::kError,
                    "cannot lock camera configuration: " + Describe(error));
      }
      bool selected = false;
      for (AVCaptureDeviceFormat* format in device_.formats) {
        auto d =
            CMVideoFormatDescriptionGetDimensions(format.formatDescription);
        if (d.width != mode_.width || d.height != mode_.height) continue;
        for (AVFrameRateRange* range in format.videoSupportedFrameRateRanges) {
          const auto duration = Duration(mode_.fps, range);
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
      controls_applied_ =
          controls_.auto_exposure && controls_.auto_gain &&
          [device_ isExposureModeSupported:
                       AVCaptureExposureModeContinuousAutoExposure];
      if (controls_applied_)
        device_.exposureMode = AVCaptureExposureModeContinuousAutoExposure;
      [device_ unlockForConfiguration];
      if (!selected) {
        [session_ commitConfiguration];
        return Fail(ConnectionState::kError,
                    "camera has no " + std::to_string(mode_.width) + "x" +
                        std::to_string(mode_.height) + " format at " +
                        std::to_string(mode_.fps) + " fps");
      }
      output_ = [[AVCaptureVideoDataOutput alloc] init];
      output_.alwaysDiscardsLateVideoFrames = YES;
      OSType requested = mode_.format == PixelFormat::kGray8
                             ? kCVPixelFormatType_OneComponent8
                         : mode_.format == PixelFormat::kYuyv
                             ? kCVPixelFormatType_422YpCbCr8_yuvs
                             : kCVPixelFormatType_32BGRA;
      if (![output_.availableVideoCVPixelFormatTypes
              containsObject:@(requested)])
        requested = kCVPixelFormatType_32BGRA;
      output_.videoSettings =
          @{(id)kCVPixelBufferPixelFormatTypeKey : @(requested)};
      delegate_ = [[TalosCaptureDelegate alloc] init];
      delegate_->callback = [this](CMSampleBufferRef sample) {
        Receive(sample);
      };
      // Frame delivery runs at the QoS that
      // talos::process::ConfigureBackgroundThread gives camera workers,
      // below every node loop.
      dispatch_ = dispatch_queue_create(
          "talos.camera", dispatch_queue_attr_make_with_qos_class(
                              DISPATCH_QUEUE_SERIAL, QOS_CLASS_UTILITY, 0));
      [output_ setSampleBufferDelegate:delegate_ queue:dispatch_];
      if (![session_ canAddOutput:output_]) {
        [session_ commitConfiguration];
        return Fail(ConnectionState::kError,
                    "capture session refused the video output");
      }
      [session_ addOutput:output_];
      [session_ commitConfiguration];
      session_id_ = event::steady_now().nanos();
      sequence_ = 0;
      running_ = true;
      [session_ startRunning];
      if (!session_.running) {
        running_ = false;
        return Fail(ConnectionState::kError, "Capture session did not start");
      }
      if (connection_) connection_(ConnectionState::kConnected, "");
      return true;
    }
  }
  void Stop() override {
    running_ = false;
    @autoreleasepool {
      [session_ stopRunning];
      [output_ setSampleBufferDelegate:nil queue:nullptr];
      if (dispatch_)
        dispatch_sync(dispatch_, ^{
                      });
      if (delegate_) delegate_->callback = {};
      delegate_ = nil;
      output_ = nil;
      session_ = nil;
      device_ = nil;
      dispatch_ = nullptr;
    }
  }

  uint64_t dropped_frames() const override { return dropped_frames_.load(); }

 private:
  // Reports why Start failed and returns false.
  bool Fail(ConnectionState state, const std::string& message) {
    if (connection_) connection_(state, device_id_ + ": " + message);
    return false;
  }
  static std::string Describe(NSError* error) {
    return error ? std::string(error.localizedDescription.UTF8String)
                 : std::string("unknown error");
  }
  void Receive(CMSampleBufferRef sample) {
    if (!running_) return;
    CVPixelBufferRef pixels = CMSampleBufferGetImageBuffer(sample);
    if (!pixels ||
        CVPixelBufferLockBaseAddress(pixels, kCVPixelBufferLock_ReadOnly) !=
            kCVReturnSuccess) {
      // Counted by the consumer through dropped_frames().
      ++dropped_frames_;
      return;
    }
    const size_t size =
        CVPixelBufferGetBytesPerRow(pixels) * CVPixelBufferGetHeight(pixels);
    const auto pixel_type = CVPixelBufferGetPixelFormatType(pixels);
    const auto format = pixel_type == kCVPixelFormatType_OneComponent8
                            ? PixelFormat::kGray8
                        : pixel_type == kCVPixelFormatType_422YpCbCr8_yuvs
                            ? PixelFormat::kYuyv
                            : PixelFormat::kBgra8;
    // The frame borrows the locked pixel buffer until the callback returns;
    // the consumer copies what it keeps (FrameMailbox).
    Frame frame(
        id_, session_id_, sequence_++, event::Poller::now(),
        static_cast<uint32_t>(CVPixelBufferGetWidth(pixels)),
        static_cast<uint32_t>(CVPixelBufferGetHeight(pixels)),
        static_cast<uint32_t>(CVPixelBufferGetBytesPerRow(pixels)), format,
        static_cast<const uint8_t*>(CVPixelBufferGetBaseAddress(pixels)), size);
    const CMTime pts = CMSampleBufferGetPresentationTimeStamp(sample);
    frame.set_raw_timestamp_ns(
        CMTIME_IS_NUMERIC(pts) ? CMTimeGetSeconds(pts) * 1e9 : 0);
    frame.set_timestamp_kind(TimestampKind::kDelivery);
    frame.set_mapping_quality(TimestampMappingQuality::kEstimated);
    if (session_.synchronizationClock && CMTIME_IS_NUMERIC(pts)) {
      const CMTime hostPTS = CMSyncConvertTime(
          pts, session_.synchronizationClock, CMClockGetHostTimeClock());
      const double age = CMTimeGetSeconds(
          CMTimeSubtract(CMClockGetTime(CMClockGetHostTimeClock()), hostPTS));
      if (std::isfinite(age) && age >= 0 && age < 5) {
        frame.set_capture_time(event::MonotonicTime::from_nanos(
            event::Poller::now().nanos() - static_cast<int64_t>(age * 1e9)));
        frame.set_timestamp_kind(TimestampKind::kPresentation);
        frame.set_mapping_quality(TimestampMappingQuality::kInterpolated);
      }
    }
    if (callback_) callback_(frame);
    CVPixelBufferUnlockBaseAddress(pixels, kCVPixelBufferLock_ReadOnly);
  }
  uint32_t id_;
  std::string device_id_;
  CameraMode mode_{1280, 720, 30, PixelFormat::kBgra8};
  CameraControls controls_{.auto_exposure = true, .auto_gain = true};
  FrameCallback callback_;
  ConnectionCallback connection_;
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> dropped_frames_{0};
  bool controls_applied_{true};
  uint64_t session_id_ = 0, sequence_ = 0;
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
    for (AVCaptureDevice* device in discovery.devices)
      result.push_back({device.uniqueID.UTF8String,
                        device.localizedName.UTF8String, Modes(device)});
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
  source->SetControls({config.exposure_ms, config.gain, config.auto_exposure,
                       config.auto_gain});
  source->SetMode({config.width, config.height, config.fps, config.format});
  return source;
}
}  // namespace talos::vision
