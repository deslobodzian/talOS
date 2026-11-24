#include "talOS/vision/capture/v4l2_source.h"

#include "talOS/events/os/poller.h"
#include "talOS/process/process.h"
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
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace talos::vision {
namespace {
V4L2Syscalls DefaultSyscalls() {
  return {
      .open_fn = [](const char* path,
                    int flags) { return ::open(path, flags); },
      .close_fn = [](int fd) { return ::close(fd); },
      // NOLINTNEXTLINE(google-runtime-int): glibc ioctl request type
      .ioctl_fn = [](int fd, unsigned long request,
                     void* arg) { return ::ioctl(fd, request, arg); },
      .mmap_fn =
          [](void* addr, size_t len, int prot, int flags, int fd,
             off_t offset) {
            return ::mmap(addr, len, prot, flags, fd, offset);
          },
      .munmap_fn = [](void* addr, size_t len) { return ::munmap(addr, len); },
      .poll_fn = [](pollfd* fds, nfds_t nfds,
                    int timeout) { return ::poll(fds, nfds, timeout); },
  };
}

V4L2Syscalls WithDefaults(V4L2Syscalls sys) {
  const auto defaults = DefaultSyscalls();
  if (!sys.open_fn) sys.open_fn = defaults.open_fn;
  if (!sys.close_fn) sys.close_fn = defaults.close_fn;
  if (!sys.ioctl_fn) sys.ioctl_fn = defaults.ioctl_fn;
  if (!sys.mmap_fn) sys.mmap_fn = defaults.mmap_fn;
  if (!sys.munmap_fn) sys.munmap_fn = defaults.munmap_fn;
  if (!sys.poll_fn) sys.poll_fn = defaults.poll_fn;
  return sys;
}

// NOLINTNEXTLINE(google-runtime-int): glibc ioctl request type
int Ioctl(const V4L2Syscalls& sys, int fd, unsigned long request, void* arg) {
  int result;
  do {
    result = sys.ioctl_fn(fd, request, arg);
  } while (result < 0 && errno == EINTR);
  return result;
}
// V4L2ControlDevice over an open device.
class DeviceControls final : public V4L2ControlDevice {
 public:
  DeviceControls(int fd, const V4L2Syscalls& sys) : fd_(fd), sys_(sys) {}
  std::optional<Range> Query(uint32_t id) override {
    v4l2_queryctrl query{};
    query.id = id;
    if (Ioctl(sys_, fd_, VIDIOC_QUERYCTRL, &query) < 0 ||
        (query.flags & V4L2_CTRL_FLAG_DISABLED)) {
      return std::nullopt;
    }
    return Range{query.minimum, query.maximum};
  }
  bool MenuHas(uint32_t id, int32_t value) override {
    v4l2_querymenu menu{};
    menu.id = id;
    menu.index = static_cast<uint32_t>(value);
    return value >= 0 && Ioctl(sys_, fd_, VIDIOC_QUERYMENU, &menu) == 0;
  }
  bool Set(uint32_t id, int32_t value) override {
    v4l2_control control{};
    control.id = id;
    control.value = value;
    return Ioctl(sys_, fd_, VIDIOC_S_CTRL, &control) == 0;
  }
  std::optional<int32_t> Get(uint32_t id) override {
    v4l2_control control{};
    control.id = id;
    if (Ioctl(sys_, fd_, VIDIOC_G_CTRL, &control) < 0) return std::nullopt;
    return control.value;
  }

 private:
  int fd_;
  const V4L2Syscalls& sys_;
};

// The pixel format a frame is delivered in for a V4L2 pixel format.
PixelFormat DeliveredFormat(uint32_t pixel_format) {
  return pixel_format == V4L2_PIX_FMT_YUYV ? PixelFormat::kYuyv
                                           : PixelFormat::kGray8;
}
}  // namespace

bool DecodeMjpegGray(const uint8_t* data, size_t size, uint32_t width,
                     uint32_t height, std::vector<uint8_t>& gray) {
  constexpr auto kMaxInt =
      static_cast<uint32_t>(std::numeric_limits<int>::max());
  if (data == nullptr || width == 0 || height == 0 || width > kMaxInt ||
      height > kMaxInt) {
    return false;
  }
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
  try {
    cv::Mat out(static_cast<int>(height), static_cast<int>(width), CV_8UC1,
                gray.data());
    cv::imdecode(
        cv::Mat(1, static_cast<int>(size), CV_8UC1, const_cast<uint8_t*>(data)),
        cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION, &out);
    return out.data == gray.data() && out.cols == static_cast<int>(width) &&
           out.rows == static_cast<int>(height) && out.type() == CV_8UC1;
  } catch (const cv::Exception&) {
    return false;
  }
}

namespace {

// Every discrete size and interval `fd` offers in the formats the source
// opens. A stepwise or continuous range contributes its two ends.
std::vector<V4L2Mode> EnumerateModes(int fd, const V4L2Syscalls& sys) {
  std::vector<V4L2Mode> modes;
  for (const uint32_t pixel : kV4L2PixelFormats) {
    bool listed = false;
    v4l2_fmtdesc description{};
    description.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (; Ioctl(sys, fd, VIDIOC_ENUM_FMT, &description) == 0;
         ++description.index) {
      listed = listed || description.pixelformat == pixel;
    }
    if (!listed) continue;
    v4l2_frmsizeenum size{};
    size.pixel_format = pixel;
    for (; Ioctl(sys, fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0; ++size.index) {
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
        for (; Ioctl(sys, fd, VIDIOC_ENUM_FRAMEINTERVALS, &interval) == 0;
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
  explicit Source(CameraSourceConfig config,
                  V4L2Syscalls syscalls = DefaultSyscalls())
      : config_(std::move(config)), sys_(WithDefaults(std::move(syscalls))) {
    mode_ = {config_.width, config_.height, config_.fps, config_.format};
    controls_.exposure_ms = config_.exposure_ms;
    controls_.gain = config_.gain;
    controls_.auto_exposure = config_.auto_exposure;
    controls_.auto_gain = config_.auto_gain;
  }
  ~Source() override {
    Stop();
    std::lock_guard lock(state_mutex_);
    const auto self = std::this_thread::get_id();
    for (auto& t : retired_workers_) {
      if (!t.joinable()) continue;
      if (t.get_id() == self) {
        t.detach();
      } else {
        t.request_stop();
        t.join();
      }
    }
    retired_workers_.clear();
  }
  uint32_t camera_id() const override { return config_.camera_id; }
  std::string device_id() const override {
    return config_.device_path.empty()
               ? "/dev/video" + std::to_string(config_.camera_id)
               : config_.device_path;
  }
  std::string name() const override { return device_id(); }
  std::vector<CameraMode> SupportedModes() const override {
    const int fd =
        sys_.open_fn(device_id().c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return {};
    std::vector<CameraMode> modes;
    for (const auto& mode : EnumerateModes(fd, sys_)) {
      modes.push_back({mode.width, mode.height, mode.fps(),
                       DeliveredFormat(mode.pixel_format)});
    }
    sys_.close_fn(fd);
    return modes;
  }
  CameraMode CurrentMode() const override {
    std::lock_guard lock(state_mutex_);
    return mode_;
  }
  bool SetMode(const CameraMode& mode) override {
    if (!mode.width || !mode.height || !std::isfinite(mode.fps) ||
        mode.fps <= 0 || mode.fps > 1000000) {
      return false;
    }
    if (!JoinExitedWorker()) {
      return false;
    }
    std::lock_guard lock(state_mutex_);
    if (running_.load(std::memory_order_acquire) || worker_.joinable()) {
      return false;
    }
    mode_ = mode;
    return true;
  }
  CameraControls GetControls() const override {
    std::lock_guard lock(state_mutex_);
    return controls_;
  }
  bool SetControls(const CameraControls& controls) override {
    if (!JoinExitedWorker()) {
      return false;
    }
    std::lock_guard lock(state_mutex_);
    if (running_.load(std::memory_order_acquire) || worker_.joinable()) {
      return false;
    }
    controls_ = controls;
    return true;
  }
  void SetFrameCallback(FrameCallback cb) override {
    std::lock_guard lock(callback_mutex_);
    callback_ = std::move(cb);
  }
  void SetConnectionCallback(ConnectionCallback cb) override {
    std::lock_guard lock(callback_mutex_);
    connection_ = std::move(cb);
  }
  bool IsRunning() const override {
    return running_.load(std::memory_order_acquire);
  }
  bool IsConnected() const override {
    return running_.load(std::memory_order_acquire);
  }
  uint64_t dropped_frames() const override { return dropped_frames_.load(); }
  bool controls_applied() const override { return controls_applied_; }
  bool Start() override {
    if (running_.load(std::memory_order_acquire)) {
      return true;
    }
    Stop();
    CameraMode mode;
    CameraControls controls;
    {
      std::lock_guard lock(state_mutex_);
      mode = mode_;
      controls = controls_;
    }
    if (!mode.width || !mode.height || !std::isfinite(mode.fps) ||
        mode.fps <= 0 || mode.fps > 1000000) {
      return Fail("invalid capture mode", 0);
    }
    fd_ = sys_.open_fn(device_id().c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      return Fail("open", errno);
    }
    v4l2_capability caps{};
    if (Ioctl(sys_, fd_, VIDIOC_QUERYCAP, &caps) < 0) {
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
    const auto offered = EnumerateModes(fd_, sys_);
    const auto selected =
        SelectV4L2Mode(mode.width, mode.height, mode.fps, offered);
    if (!selected) {
      char wanted[64];
      std::snprintf(wanted, sizeof(wanted), "%ux%u@%g", mode.width, mode.height,
                    mode.fps);
      std::string message = std::string("mode ") + wanted +
                            " is not offered by the device; it offers:";
      for (const auto& offered_mode : offered) {
        message += ' ' + DescribeV4L2Mode(offered_mode) + ',';
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
    if (Ioctl(sys_, fd_, VIDIOC_S_FMT, &format) < 0) {
      return Fail("VIDIOC_S_FMT", errno);
    }
    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe = {selected->interval_numerator,
                                      selected->interval_denominator};
    if (Ioctl(sys_, fd_, VIDIOC_S_PARM, &parm) < 0) {
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
    mode.width = granted.width;
    mode.height = granted.height;
    mode.fps = granted.fps();
    mode.format = DeliveredFormat(granted.pixel_format);
    {
      std::lock_guard lock(state_mutex_);
      mode_ = mode;
    }
    mjpeg_ = granted.pixel_format == V4L2_PIX_FMT_MJPEG;
    // A compressed frame's size varies; it is decoded into `decoded_`.
    stride_ = mjpeg_ ? mode.width : format.fmt.pix.bytesperline;
    if (!mode.height || stride_ < mode.width * BytesPerPixel(mode.format)) {
      return Fail("driver reported an invalid stride", 0);
    }
    const size_t size = mjpeg_ ? 1 : static_cast<size_t>(stride_) * mode.height;
    decoded_.assign(mjpeg_ ? static_cast<size_t>(stride_) * mode.height : 0, 0);
    DeviceControls device(fd_, sys_);
    if (const auto error = ApplyV4L2Controls(controls, device);
        !error.empty()) {
      return Fail(("control " + error).c_str(), 0);
    }
    v4l2_requestbuffers request{};
    request.count = 4;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (Ioctl(sys_, fd_, VIDIOC_REQBUFS, &request) < 0) {
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
      if (Ioctl(sys_, fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
        return Fail("VIDIOC_QUERYBUF", errno);
      }
      if (buffer.length < size) {
        return Fail("driver buffer smaller than one frame", 0);
      }
      mappings_[i].size = buffer.length;
      mappings_[i].data =
          sys_.mmap_fn(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                       MAP_SHARED, fd_, buffer.m.offset);
      if (mappings_[i].data == MAP_FAILED ||
          Ioctl(sys_, fd_, VIDIOC_QBUF, &buffer) < 0) {
        return Fail("mmap/VIDIOC_QBUF", errno);
      }
    }
    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (Ioctl(sys_, fd_, VIDIOC_STREAMON, &type) < 0) {
      return Fail("VIDIOC_STREAMON", errno);
    }
    streaming_ = true;
    session_ = event::steady_now().nanos();
    sequence_ = 0;
    {
      std::lock_guard lock(state_mutex_);
      running_.store(true, std::memory_order_release);
      worker_ =
          std::jthread([this](const std::stop_token& stop) { Capture(stop); });
    }
    ReportConnection(ConnectionState::kConnected, "");
    return true;
  }
  void Stop() override {
    std::vector<std::jthread> to_join;
    {
      std::lock_guard lock(state_mutex_);
      const auto self = std::this_thread::get_id();
      if (worker_.joinable()) {
        worker_.request_stop();
        if (worker_.get_id() == self) {
          retired_workers_.push_back(std::move(worker_));
        } else {
          to_join.push_back(std::move(worker_));
        }
      }
      ReapRetiredLocked(to_join);
    }
    JoinThreads(to_join);
    running_.store(false, std::memory_order_release);
    ReleaseDevice();
  }

 private:
  void ReapRetiredLocked(std::vector<std::jthread>& to_join) {
    const auto self = std::this_thread::get_id();
    std::erase_if(retired_workers_, [&](std::jthread& t) {
      if (!t.joinable()) return true;
      if (t.get_id() == self) return false;
      to_join.push_back(std::move(t));
      return true;
    });
  }
  static void JoinThreads(std::vector<std::jthread>& threads) {
    for (auto& t : threads) {
      if (t.joinable()) {
        t.join();
      }
    }
  }
  bool JoinExitedWorker() {
    std::vector<std::jthread> to_join;
    {
      std::lock_guard lock(state_mutex_);
      if (running_.load(std::memory_order_acquire)) {
        return false;
      }
      const auto self = std::this_thread::get_id();
      if (worker_.joinable()) {
        if (worker_.get_id() == self) {
          retired_workers_.push_back(std::move(worker_));
        } else {
          to_join.push_back(std::move(worker_));
        }
      }
      ReapRetiredLocked(to_join);
    }
    JoinThreads(to_join);
    return !running_.load(std::memory_order_acquire);
  }
  void ReleaseDevice() {
    std::lock_guard lock(device_mutex_);
    if (streaming_) {
      auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      Ioctl(sys_, fd_, VIDIOC_STREAMOFF, &type);
      streaming_ = false;
    }
    for (auto& m : mappings_) {
      if (m.data != MAP_FAILED) {
        sys_.munmap_fn(m.data, m.size);
      }
    }
    mappings_.clear();
    if (fd_ >= 0) {
      sys_.close_fn(fd_);
      fd_ = -1;
    }
  }
  void ReportConnection(ConnectionState state, std::string_view message) {
    ConnectionCallback connection;
    {
      std::lock_guard lock(callback_mutex_);
      connection = connection_;
    }
    if (connection) {
      connection(state, message);
    }
  }
  // Reports why Start failed; `error` is the errno of the failed call, or 0.
  bool Fail(const char* step, int error) {
    Stop();
    std::string message = device_id() + ": " + step;
    if (error != 0) message += std::string(": ") + std::strerror(error);
    ReportConnection(ConnectionState::kError, message);
    return false;
  }
  void Capture(const std::stop_token& stop) {
    talos::process::ConfigureBackgroundThread();
    std::string reason = "capture stopped";
    const CameraMode mode = CurrentMode();
    while (!stop.stop_requested()) {
      pollfd poll_fd{fd_, POLLIN, 0};
      const int ready = sys_.poll_fn(&poll_fd, 1, 100);
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
      if (Ioctl(sys_, fd_, VIDIOC_DQBUF, &buffer) < 0) {
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
      const auto resolved = ResolveV4L2Timestamp(
          buffer.flags, buffer.timestamp, delivery, node_delivery, driver_);
      const size_t size = static_cast<size_t>(stride_) * mode.height;
      const bool delivered =
          buffer.index < mappings_.size() &&
          !(buffer.flags & V4L2_BUF_FLAG_ERROR) &&
          (mjpeg_
               ? (buffer.bytesused <= mappings_[buffer.index].size &&
                  DecodeMjpegGray(
                      static_cast<const uint8_t*>(mappings_[buffer.index].data),
                      buffer.bytesused, mode.width, mode.height, decoded_))
               : (size <= mappings_[buffer.index].size &&
                  buffer.bytesused >= size));
      if (delivered) {
        // The frame borrows the dequeued driver buffer until it is queued
        // again below; the consumer copies what it keeps (FrameMailbox).
        // Preserve source meaning; the camera publication boundary converts
        // it to a midpoint once (CaptureMidpoint).
        Frame frame(
            config_.camera_id, session_, sequence_++,
            event::MonotonicTime::from_nanos(resolved.capture_time_ns),
            mode.width, mode.height, stride_, mode.format,
            mjpeg_ ? decoded_.data()
                   : static_cast<const uint8_t*>(mappings_[buffer.index].data),
            size);
        frame.set_raw_timestamp_ns(resolved.raw_timestamp_ns);
        frame.set_timestamp_kind(resolved.kind);
        frame.set_mapping_quality(TimestampMappingQuality::kEstimated);
        FrameCallback callback;
        {
          std::lock_guard lock(callback_mutex_);
          callback = callback_;
        }
        if (callback) {
          callback(frame);
        }
      } else {
        // An errored or short driver buffer, or an MJPG frame that is
        // truncated or does not decode to the mode's size; counted by the
        // consumer through dropped_frames().
        ++dropped_frames_;
      }
      if (Ioctl(sys_, fd_, VIDIOC_QBUF, &buffer) < 0) {
        reason = std::string("VIDIOC_QBUF: ") + std::strerror(errno);
        break;
      }
    }
    running_.store(false, std::memory_order_release);
    ReleaseDevice();
    if (!stop.stop_requested()) {
      ReportConnection(ConnectionState::kDisconnected,
                       device_id() + ": " + reason);
    }
  }
  struct Mapping {
    void* data{MAP_FAILED};
    size_t size{0};
  };
  CameraSourceConfig config_;
  V4L2Syscalls sys_;
  mutable std::mutex state_mutex_;
  CameraMode mode_;
  CameraControls controls_;
  std::mutex callback_mutex_;
  FrameCallback callback_;
  ConnectionCallback connection_;
  std::mutex device_mutex_;
  int fd_{-1};
  bool streaming_{false};
  bool controls_applied_{true};
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> dropped_frames_{0};
  std::jthread worker_;
  std::vector<std::jthread> retired_workers_;
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
std::unique_ptr<CameraSource> CreateV4L2CameraSourceForTest(
    const CameraSourceConfig& config, V4L2Syscalls syscalls) {
  return std::make_unique<Source>(config, std::move(syscalls));
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
