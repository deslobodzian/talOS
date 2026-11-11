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
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
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
// The pixel format a frame is delivered in for a V4L2 pixel format.
PixelFormat DeliveredFormat(uint32_t pixel_format) {
  return pixel_format == V4L2_PIX_FMT_YUYV ? PixelFormat::kYuyv
                                           : PixelFormat::kGray8;
}
}  // namespace

bool DecodeMjpegGray(const uint8_t* data, size_t size, uint32_t width,
                     uint32_t height, std::vector<uint8_t>& gray) {
  // uvcvideo pads a frame with zeros after its end-of-image marker; a frame
  // without one was cut short on the bus.
  while (size > 0 && data[size - 1] == 0) --size;
  if (size < 4 || data[0] != 0xff || data[1] != 0xd8 ||
      data[size - 2] != 0xff || data[size - 1] != 0xd9 ||
      gray.size() != static_cast<size_t>(width) * height ||
      size > static_cast<size_t>(std::numeric_limits<int>::max())) {
    return false;
  }
  // Wrapping `gray` makes imdecode write into it when the JPEG has the
  // expected size and reallocate (detected below) when it does not. A colour
  // JPEG decodes its luma only: libjpeg skips the chroma IDCT and colour
  // conversion for a grayscale output.
  cv::Mat out(static_cast<int>(height), static_cast<int>(width), CV_8UC1,
              gray.data());
  try {
    cv::imdecode(
        cv::Mat(1, static_cast<int>(size), CV_8UC1, const_cast<uint8_t*>(data)),
        cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION, &out);
  } catch (const cv::Exception&) {
    return false;
  }
  return out.data == gray.data() && out.cols == static_cast<int>(width) &&
         out.rows == static_cast<int>(height) && out.type() == CV_8UC1;
}

namespace {

// Every discrete size and interval `fd` offers in the formats the source
// opens. A stepwise or continuous range contributes its two ends.
std::vector<V4L2Mode> EnumerateModes(int fd) {
  std::vector<V4L2Mode> modes;
  for (const uint32_t pixel : kV4L2PixelFormats) {
    bool listed = false;
    v4l2_fmtdesc description{};
    description.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (; Ioctl(fd, VIDIOC_ENUM_FMT, &description) == 0; ++description.index) {
      listed = listed || description.pixelformat == pixel;
    }
    if (!listed) continue;
    v4l2_frmsizeenum size{};
    size.pixel_format = pixel;
    for (; Ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0; ++size.index) {
      std::vector<std::pair<uint32_t, uint32_t>> sizes;
      if (size.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        sizes.emplace_back(size.discrete.width, size.discrete.height);
      } else {
        sizes.emplace_back(size.stepwise.min_width, size.stepwise.min_height);
        sizes.emplace_back(size.stepwise.max_width, size.stepwise.max_height);
      }
      for (const auto& [width, height] : sizes) {
        v4l2_frmivalenum interval{};
        interval.pixel_format = pixel;
        interval.width = width;
        interval.height = height;
        for (; Ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &interval) == 0;
             ++interval.index) {
          if (interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            modes.push_back({pixel, width, height, interval.discrete.numerator,
                             interval.discrete.denominator});
          } else {
            for (const auto& fraction :
                 {interval.stepwise.min, interval.stepwise.max}) {
              modes.push_back({pixel, width, height, fraction.numerator,
                               fraction.denominator});
            }
            break;
          }
        }
      }
      if (size.type != V4L2_FRMSIZE_TYPE_DISCRETE) break;
    }
  }
  return modes;
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
  std::vector<CameraMode> SupportedModes() const override {
    const int fd = open(device_id().c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return {};
    std::vector<CameraMode> modes;
    for (const auto& mode : EnumerateModes(fd)) {
      modes.push_back({mode.width, mode.height, mode.fps(),
                       DeliveredFormat(mode.pixel_format)});
    }
    close(fd);
    return modes;
  }
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
    const auto offered = EnumerateModes(fd_);
    const auto selected =
        SelectV4L2Mode(mode_.width, mode_.height, mode_.fps, offered);
    if (!selected) {
      char wanted[64];
      std::snprintf(wanted, sizeof(wanted), "%ux%u@%g", mode_.width,
                    mode_.height, mode_.fps);
      std::string message = std::string("mode ") + wanted +
                            " is not offered by the device; it offers:";
      for (const auto& mode : offered) {
        message += ' ' + DescribeV4L2Mode(mode) + ',';
      }
      if (!offered.empty()) message.pop_back();
      return Fail(message.c_str(), 0);
    }
    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = selected->width;
    format.fmt.pix.height = selected->height;
    format.fmt.pix.pixelformat = selected->pixel_format;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    if (Ioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
      return Fail("VIDIOC_S_FMT", errno);
    }
    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe = {selected->interval_numerator,
                                      selected->interval_denominator};
    if (Ioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
      return Fail("VIDIOC_S_PARM", errno);
    }
    if (!(parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
      return Fail("device cannot set its frame interval", 0);
    }
    const V4L2Mode granted{format.fmt.pix.pixelformat, format.fmt.pix.width,
                           format.fmt.pix.height,
                           parm.parm.capture.timeperframe.numerator,
                           parm.parm.capture.timeperframe.denominator};
    if (!V4L2Granted(*selected, granted)) {
      const std::string message = "driver granted " +
                                  DescribeV4L2Mode(granted) + " for " +
                                  DescribeV4L2Mode(*selected);
      return Fail(message.c_str(), 0);
    }
    mode_.width = granted.width;
    mode_.height = granted.height;
    mode_.fps = granted.fps();
    mode_.format = DeliveredFormat(granted.pixel_format);
    mjpeg_ = granted.pixel_format == V4L2_PIX_FMT_MJPEG;
    // A compressed frame's size varies; it is decoded into `decoded_`.
    stride_ = mjpeg_ ? mode_.width : format.fmt.pix.bytesperline;
    if (!mode_.height || stride_ < mode_.width * BytesPerPixel(mode_.format)) {
      return Fail("driver reported an invalid stride", 0);
    }
    const size_t size =
        mjpeg_ ? 1 : static_cast<size_t>(stride_) * mode_.height;
    decoded_.assign(mjpeg_ ? static_cast<size_t>(stride_) * mode_.height : 0,
                    0);
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
      const bool delivered =
          buffer.index < mappings_.size() &&
          !(buffer.flags & V4L2_BUF_FLAG_ERROR) &&
          (mjpeg_
               ? DecodeMjpegGray(
                     static_cast<const uint8_t*>(mappings_[buffer.index].data),
                     buffer.bytesused, mode_.width, mode_.height, decoded_)
               : buffer.bytesused >= size);
      if (delivered) {
        // The frame borrows the dequeued driver buffer until it is queued
        // again below; the consumer copies what it keeps (FrameMailbox).
        // Preserve source meaning; the camera publication boundary converts
        // it to a midpoint once (CaptureMidpoint).
        Frame frame(
            config_.camera_id, session_, sequence_++,
            event::MonotonicTime::from_nanos(capture_end), mode_.width,
            mode_.height, stride_, mode_.format,
            mjpeg_ ? decoded_.data()
                   : static_cast<const uint8_t*>(mappings_[buffer.index].data),
            size);
        frame.set_raw_timestamp_ns(driver_monotonic ? driver_time : delivery);
        frame.set_timestamp_kind(V4L2TimestampKind(buffer.flags, driver_));
        frame.set_mapping_quality(TimestampMappingQuality::kEstimated);
        if (callback_) {
          callback_(frame);
        }
      } else {
        // An errored or short driver buffer, or an MJPG frame that is
        // truncated or does not decode to the mode's size; counted by the
        // consumer through dropped_frames().
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
  bool mjpeg_{false};
  std::vector<uint8_t> decoded_;  // MJPG frames decode here, gray, stride_.
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
