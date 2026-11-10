#include "talOS/vision/capture/v4l2_source.h"

#include "talOS/events/os/poller.h"
#if defined(__linux__)
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace talos::vision {
namespace {
// NOLINTNEXTLINE(google-runtime-int): glibc ioctl request type
int Ioctl(int fd, unsigned long request, void* arg) {
  int result;
  do {
    result = ioctl(fd, request, arg);
  } while (result < 0 && errno == EINTR);
  return result;
}
class Source final : public CameraSource {
 public:
  explicit Source(CameraSourceConfig config) : config_(std::move(config)) {
    mode_ = {config_.width, config_.height, config_.fps, config_.format};
    controls_.exposure_ms = config_.exposure_ms;
    controls_.gain = config_.gain;
    controls_.auto_exposure = config_.auto_exposure;
    controls_.auto_gain = config_.auto_gain;
  }
  ~Source() override { Stop(); }
  uint32_t camera_id() const override { return config_.camera_id; }
  std::string device_id() const override {
    return config_.device_path.empty()
               ? "/dev/video" + std::to_string(config_.camera_id)
               : config_.device_path;
  }
  std::string name() const override { return device_id(); }
  std::vector<CameraMode> SupportedModes() const override { return {}; }
  CameraMode CurrentMode() const override { return mode_; }
  bool SetMode(const CameraMode& mode) override {
    if (worker_.joinable() || !mode.width || !mode.height ||
        !std::isfinite(mode.fps) || mode.fps <= 0 || mode.fps > 1000000) {
      return false;
    }
    mode_ = mode;
    return true;
  }
  CameraControls GetControls() const override { return controls_; }
  bool SetControls(const CameraControls& controls) override {
    if (worker_.joinable()) {
      return false;
    }
    controls_ = controls;
    return true;
  }
  void SetFrameCallback(FrameCallback cb) override {
    callback_ = std::move(cb);
  }
  void SetConnectionCallback(ConnectionCallback cb) override {
    connection_ = std::move(cb);
  }
  bool IsRunning() const override { return running_; }
  bool IsConnected() const override { return running_; }
  uint64_t dropped_frames() const override { return dropped_frames_.load(); }
  bool controls_applied() const override { return controls_applied_; }
  bool Start() override {
    if (running_) {
      return true;
    }
    Stop();
    if (!mode_.width || !mode_.height || !std::isfinite(mode_.fps) ||
        mode_.fps <= 0 || mode_.fps > 1000000) {
      return Fail("invalid capture mode", 0);
    }
    fd_ = open(device_id().c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      return Fail("open", errno);
    }
    v4l2_capability caps{};
    if (Ioctl(fd_, VIDIOC_QUERYCAP, &caps) < 0) {
      return Fail("VIDIOC_QUERYCAP", errno);
    }
    const uint32_t capabilities = caps.capabilities & V4L2_CAP_DEVICE_CAPS
                                      ? caps.device_caps
                                      : caps.capabilities;
    if (!(capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(capabilities & V4L2_CAP_STREAMING)) {
      return Fail("not a streaming video capture device", 0);
    }
    driver_.assign(reinterpret_cast<const char*>(caps.driver),
                   strnlen(reinterpret_cast<const char*>(caps.driver),
                           sizeof(caps.driver)));
    v4l2_format format{};
    bool selected = false;
    const uint32_t preferred = mode_.format == PixelFormat::kGray8
                                   ? V4L2_PIX_FMT_GREY
                                   : V4L2_PIX_FMT_YUYV;
    for (uint32_t pixel :
         {preferred, preferred == V4L2_PIX_FMT_GREY ? V4L2_PIX_FMT_YUYV
                                                    : V4L2_PIX_FMT_GREY}) {
      format = {};
      format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      format.fmt.pix.width = mode_.width;
      format.fmt.pix.height = mode_.height;
      format.fmt.pix.pixelformat = pixel;
      format.fmt.pix.field = V4L2_FIELD_ANY;
      if (Ioctl(fd_, VIDIOC_S_FMT, &format) == 0 &&
          (format.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY ||
           format.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)) {
        selected = true;
        break;
      }
    }
    if (!selected) {
      return Fail("neither GREY nor YUYV is available", 0);
    }
    if (mode_.width > 0 && mode_.height > 0 &&
        (format.fmt.pix.width != mode_.width ||
         format.fmt.pix.height != mode_.height)) {
      return Fail("requested width x height is not supported", 0);
    }
    mode_.width = format.fmt.pix.width;
    mode_.height = format.fmt.pix.height;
    mode_.format = format.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY
                       ? PixelFormat::kGray8
                       : PixelFormat::kYuyv;
    stride_ = format.fmt.pix.bytesperline;
    if (!mode_.height || stride_ < mode_.width * BytesPerPixel(mode_.format)) {
      return Fail("driver reported an invalid stride", 0);
    }
    const size_t size = static_cast<size_t>(stride_) * mode_.height;
    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe = {
        1000, static_cast<uint32_t>(std::lround(mode_.fps * 1000))};
    Ioctl(fd_, VIDIOC_S_PARM, &parm);
    controls_applied_ =
        ApplyV4L2Controls(controls_, [this](uint32_t id, int32_t value) {
          v4l2_control control{};
          control.id = id;
          control.value = value;
          return Ioctl(fd_, VIDIOC_S_CTRL, &control) == 0;
        });
    v4l2_requestbuffers request{};
    request.count = 4;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (Ioctl(fd_, VIDIOC_REQBUFS, &request) < 0) {
      return Fail("VIDIOC_REQBUFS", errno);
    }
    if (request.count < 2) {
      return Fail("driver granted fewer than 2 buffers", 0);
    }
    mappings_.resize(request.count);
    for (uint32_t i = 0; i < request.count; ++i) {
      v4l2_buffer buffer{};
      buffer.type = request.type;
      buffer.memory = request.memory;
      buffer.index = i;
      if (Ioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
        return Fail("VIDIOC_QUERYBUF", errno);
      }
      if (buffer.length < size) {
        return Fail("driver buffer smaller than one frame", 0);
      }
      mappings_[i].size = buffer.length;
      mappings_[i].data = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd_, buffer.m.offset);
      if (mappings_[i].data == MAP_FAILED ||
          Ioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
        return Fail("mmap/VIDIOC_QBUF", errno);
      }
    }
    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (Ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      return Fail("VIDIOC_STREAMON", errno);
    }
    streaming_ = true;
    session_ = event::steady_now().nanos();
    sequence_ = 0;
    running_ = true;
    worker_ =
        std::jthread([this](const std::stop_token& stop) { Capture(stop); });
    if (connection_) {
      connection_(ConnectionState::kConnected, "");
    }
    return true;
  }
  void Stop() override {
    if (worker_.joinable()) {
      worker_.request_stop();
      worker_.join();
    }
    running_ = false;
    if (streaming_) {
      auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      Ioctl(fd_, VIDIOC_STREAMOFF, &type);
      streaming_ = false;
    }
    for (auto& m : mappings_) {
      if (m.data != MAP_FAILED) {
        munmap(m.data, m.size);
      }
    }
    mappings_.clear();
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

 private:
  // Reports why Start failed; `error` is the errno of the failed call, or 0.
  bool Fail(const char* step, int error) {
    Stop();
    if (connection_) {
      std::string message = device_id() + ": " + step;
      if (error != 0) message += std::string(": ") + std::strerror(error);
      connection_(ConnectionState::kError, message);
    }
    return false;
  }
  void Capture(const std::stop_token& stop) {
    std::string reason = "capture stopped";
    while (!stop.stop_requested()) {
      pollfd poll_fd{fd_, POLLIN, 0};
      const int ready = poll(&poll_fd, 1, 100);
      if (ready < 0 && errno == EINTR) {
        continue;
      }
      if (ready < 0) {
        reason = std::string("poll: ") + std::strerror(errno);
        break;
      }
      if (ready > 0 && (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
        reason = "device error or hangup (unplugged?)";
        break;
      }
      if (!ready) {
        continue;
      }
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      if (Ioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
        if (errno == EAGAIN) {
          continue;
        }
        reason = std::string("VIDIOC_DQBUF: ") + std::strerror(errno);
        break;
      }
      timespec now{};
      clock_gettime(CLOCK_MONOTONIC, &now);
      const int64_t delivery =
          static_cast<int64_t>(now.tv_sec) * 1000000000 + now.tv_nsec;
      const int64_t node_delivery = event::Poller::now().nanos();
      const bool driver_monotonic =
          (buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
          V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
      const int64_t driver_time =
          static_cast<int64_t>(buffer.timestamp.tv_sec) * 1000000000 +
          static_cast<int64_t>(buffer.timestamp.tv_usec) * 1000;
      const int64_t capture_end = driver_monotonic
                                      ? node_delivery - (delivery - driver_time)
                                      : node_delivery;
      const size_t size = static_cast<size_t>(stride_) * mode_.height;
      if (buffer.index < mappings_.size() &&
          !(buffer.flags & V4L2_BUF_FLAG_ERROR) && buffer.bytesused >= size) {
        // The frame borrows the dequeued driver buffer until it is queued
        // again below; the consumer copies what it keeps (FrameMailbox).
        // Preserve source meaning; the camera publication boundary converts
        // it to a midpoint once (CaptureMidpoint).
        Frame frame(config_.camera_id, session_, sequence_++,
                    event::MonotonicTime::from_nanos(capture_end), mode_.width,
                    mode_.height, stride_, mode_.format,
                    static_cast<const uint8_t*>(mappings_[buffer.index].data),
                    size);
        frame.set_raw_timestamp_ns(driver_monotonic ? driver_time : delivery);
        frame.set_timestamp_kind(V4L2TimestampKind(buffer.flags, driver_));
        frame.set_mapping_quality(TimestampMappingQuality::kEstimated);
        if (callback_) {
          callback_(frame);
        }
      } else {
        // An errored or short driver buffer; counted by the consumer through
        // dropped_frames().
        ++dropped_frames_;
      }
      if (Ioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
        reason = std::string("VIDIOC_QBUF: ") + std::strerror(errno);
        break;
      }
    }
    running_ = false;
    if (!stop.stop_requested() && connection_) {
      connection_(ConnectionState::kDisconnected, device_id() + ": " + reason);
    }
  }
  struct Mapping {
    void* data{MAP_FAILED};
    size_t size{0};
  };
  CameraSourceConfig config_;
  CameraMode mode_;
  CameraControls controls_;
  FrameCallback callback_;
  ConnectionCallback connection_;
  int fd_{-1};
  bool streaming_{false};
  bool controls_applied_{true};
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> dropped_frames_{0};
  std::jthread worker_;
  std::vector<Mapping> mappings_;
  uint32_t stride_{0};
  uint64_t session_{0}, sequence_{0};
  std::string driver_;  // VIDIOC_QUERYCAP driver name; set before streaming.
};
}  // namespace
std::unique_ptr<CameraSource> CreatePlatformCameraSource(
    const CameraSourceConfig& config) {
  return std::make_unique<Source>(config);
}
std::unique_ptr<CameraSource> CreateCaptureSource(uint32_t id,
                                                  const std::string& device) {
  CameraSourceConfig config;
  config.camera_id = id;
  config.device_path = device;
  return CreatePlatformCameraSource(config);
}
std::vector<CaptureDevice> DiscoverCaptureDevices() { return {}; }
}  // namespace talos::vision
#endif
