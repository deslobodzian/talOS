#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cfenv>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "talOS/vision/capture/avfoundation_source.h"
#include "talOS/vision/capture/v4l2_source.h"

namespace talos::vision {
namespace {
TEST(Capture, TimestampKindsConvertAtOneBoundary) {
  // exposure 200, readout 50, latency offset 300: each kind's geometry, then
  // the camera's measured offset, which applies to every kind.
  EXPECT_EQ(
      CaptureMidpoint(1000, TimestampKind::kStartOfExposure, 200, 50, 300),
      800);
  EXPECT_EQ(CaptureMidpoint(1000, TimestampKind::kEndOfFrame, 200, 50, 300),
            550);
  for (auto kind : {TimestampKind::kExposure, TimestampKind::kDelivery,
                    TimestampKind::kPresentation}) {
    EXPECT_EQ(CaptureMidpoint(1000, kind, 200, 50, 300), 700);
    EXPECT_FALSE(CaptureMidpoint(299, kind, 200, 50, 300));
  }
  EXPECT_FALSE(
      CaptureMidpoint(199, TimestampKind::kStartOfExposure, 200, 50, 300));
  EXPECT_FALSE(
      CaptureMidpoint(UINT64_MAX, TimestampKind::kStartOfExposure, 200, 0, 0));
  EXPECT_FALSE(CaptureMidpoint(449, TimestampKind::kEndOfFrame, 200, 50, 300));
}

#if defined(__linux__)
TEST(Capture, LinuxTimestampFlagsRetainSourceMeaning) {
  EXPECT_EQ(V4L2TimestampKind(V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
                              V4L2_BUF_FLAG_TSTAMP_SRC_SOE),
            TimestampKind::kStartOfExposure);
  EXPECT_EQ(V4L2TimestampKind(V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
                              V4L2_BUF_FLAG_TSTAMP_SRC_EOF),
            TimestampKind::kEndOfFrame);
  EXPECT_EQ(V4L2TimestampKind(V4L2_BUF_FLAG_TSTAMP_SRC_SOE),
            TimestampKind::kDelivery);
  EXPECT_EQ(V4L2TimestampKind(V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
                                  V4L2_BUF_FLAG_TSTAMP_SRC_SOE,
                              "uvcvideo"),
            TimestampKind::kDelivery);
}

TEST(Capture, LinuxDriverTimestampSanityChecksZeroFutureAndStale) {
  constexpr uint32_t kMonoSoe =
      V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC | V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
  constexpr int64_t kDeliveryNs = 100'000'000'000LL;      // 100.0 s
  constexpr int64_t kNodeDeliveryNs = 200'000'000'000LL;  // 200.0 s

  // Valid driver timestamp 12 ms before delivery: mapped into node clock.
  {
    const timeval tv{99, 988'000};
    const auto r = ResolveV4L2Timestamp(kMonoSoe, tv, kDeliveryNs,
                                        kNodeDeliveryNs, "rkisp1");
    EXPECT_EQ(r.capture_time_ns, kNodeDeliveryNs - 12'000'000LL);
    EXPECT_EQ(r.raw_timestamp_ns, 99'988'000'000LL);
    EXPECT_EQ(r.kind, TimestampKind::kStartOfExposure);
  }
  // Exact boundary age == 0: accepted.
  {
    const timeval tv{100, 0};
    const auto r = ResolveV4L2Timestamp(kMonoSoe, tv, kDeliveryNs,
                                        kNodeDeliveryNs, "rkisp1");
    EXPECT_EQ(r.capture_time_ns, kNodeDeliveryNs);
    EXPECT_EQ(r.raw_timestamp_ns, kDeliveryNs);
    EXPECT_EQ(r.kind, TimestampKind::kStartOfExposure);
  }
  // Zero driver timestamp (even when delivery_ns < 5 s): falls back.
  {
    const timeval zero{0, 0};
    const auto r =
        ResolveV4L2Timestamp(kMonoSoe, zero, 2'000'000'000LL, 2'000'000'000LL);
    EXPECT_EQ(r.capture_time_ns, 2'000'000'000LL);
    EXPECT_EQ(r.raw_timestamp_ns, 2'000'000'000LL);
    EXPECT_EQ(r.kind, TimestampKind::kDelivery);
  }
  // Future driver timestamp (1 ms ahead of delivery): falls back.
  {
    const timeval future{100, 1'000};
    const auto r = ResolveV4L2Timestamp(kMonoSoe, future, kDeliveryNs,
                                        kNodeDeliveryNs, "rkisp1");
    EXPECT_EQ(r.capture_time_ns, kNodeDeliveryNs);
    EXPECT_EQ(r.raw_timestamp_ns, kDeliveryNs);
    EXPECT_EQ(r.kind, TimestampKind::kDelivery);
  }
  // Stale driver timestamp (exactly 5 s old or 10 s old): falls back.
  for (const timeval stale : {timeval{95, 0}, timeval{90, 0}}) {
    const auto r = ResolveV4L2Timestamp(kMonoSoe, stale, kDeliveryNs,
                                        kNodeDeliveryNs, "rkisp1");
    EXPECT_EQ(r.capture_time_ns, kNodeDeliveryNs);
    EXPECT_EQ(r.raw_timestamp_ns, kDeliveryNs);
    EXPECT_EQ(r.kind, TimestampKind::kDelivery);
  }
  // Negative or malformed tv_usec: falls back.
  for (const timeval bad :
       {timeval{-1, 0}, timeval{99, -1}, timeval{99, 1'000'000}}) {
    const auto r = ResolveV4L2Timestamp(kMonoSoe, bad, kDeliveryNs,
                                        kNodeDeliveryNs, "rkisp1");
    EXPECT_EQ(r.capture_time_ns, kNodeDeliveryNs);
    EXPECT_EQ(r.raw_timestamp_ns, kDeliveryNs);
    EXPECT_EQ(r.kind, TimestampKind::kDelivery);
  }
  // Non-monotonic buffer flags: always falls back.
  {
    const timeval tv{99, 990'000};
    const auto r = ResolveV4L2Timestamp(V4L2_BUF_FLAG_TSTAMP_SRC_SOE, tv,
                                        kDeliveryNs, kNodeDeliveryNs, "rkisp1");
    EXPECT_EQ(r.capture_time_ns, kNodeDeliveryNs);
    EXPECT_EQ(r.raw_timestamp_ns, kDeliveryNs);
    EXPECT_EQ(r.kind, TimestampKind::kDelivery);
  }
}

struct FakeV4L2State {
  static constexpr uint32_t kWidth = 64;
  static constexpr uint32_t kHeight = 48;
  static constexpr uint32_t kFps = 30;
  static constexpr size_t kBufSize = kWidth * kHeight;

  std::mutex mu;
  int open_fds{0};
  int active_mmaps{0};
  int streamon_count{0};
  int streamoff_count{0};
  int next_fd{100};
  std::vector<uint8_t> backing[2]{std::vector<uint8_t>(kBufSize, 0x55),
                                  std::vector<uint8_t>(kBufSize, 0x55)};

  V4L2Syscalls Syscalls(std::function<int(pollfd*, nfds_t, int)> poll_fn) {
    V4L2Syscalls sys;
    sys.open_fn = [this](const char*, int) {
      std::lock_guard lock(mu);
      ++open_fds;
      return next_fd++;
    };
    sys.close_fn = [this](int) {
      std::lock_guard lock(mu);
      --open_fds;
      return 0;
    };
    // NOLINTNEXTLINE(google-runtime-int): glibc ioctl request type
    sys.ioctl_fn = [this](int, unsigned long req, void* arg) -> int {
      switch (req) {
        case VIDIOC_QUERYCAP: {
          auto* c = static_cast<v4l2_capability*>(arg);
          c->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
          std::strncpy(reinterpret_cast<char*>(c->driver), "fakev4l2",
                       sizeof(c->driver) - 1);
          return 0;
        }
        case VIDIOC_ENUM_FMT: {
          auto* f = static_cast<v4l2_fmtdesc*>(arg);
          if (f->index > 0) {
            errno = EINVAL;
            return -1;
          }
          f->pixelformat = V4L2_PIX_FMT_GREY;
          return 0;
        }
        case VIDIOC_ENUM_FRAMESIZES: {
          auto* s = static_cast<v4l2_frmsizeenum*>(arg);
          if (s->index > 0) {
            errno = EINVAL;
            return -1;
          }
          s->type = V4L2_FRMSIZE_TYPE_DISCRETE;
          s->discrete.width = kWidth;
          s->discrete.height = kHeight;
          return 0;
        }
        case VIDIOC_ENUM_FRAMEINTERVALS: {
          auto* iv = static_cast<v4l2_frmivalenum*>(arg);
          if (iv->index > 0) {
            errno = EINVAL;
            return -1;
          }
          iv->type = V4L2_FRMIVAL_TYPE_DISCRETE;
          iv->discrete.numerator = 1;
          iv->discrete.denominator = kFps;
          return 0;
        }
        case VIDIOC_S_FMT: {
          auto* fmt = static_cast<v4l2_format*>(arg);
          fmt->fmt.pix.bytesperline = kWidth;
          fmt->fmt.pix.sizeimage = kBufSize;
          return 0;
        }
        case VIDIOC_S_PARM: {
          auto* p = static_cast<v4l2_streamparm*>(arg);
          p->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
          p->parm.capture.timeperframe = {1, kFps};
          return 0;
        }
        case VIDIOC_QUERYCTRL: {
          errno = EINVAL;
          return -1;
        }
        case VIDIOC_REQBUFS: {
          auto* r = static_cast<v4l2_requestbuffers*>(arg);
          r->count = 2;
          return 0;
        }
        case VIDIOC_QUERYBUF: {
          auto* b = static_cast<v4l2_buffer*>(arg);
          b->length = kBufSize;
          b->m.offset = b->index * 4096;
          return 0;
        }
        case VIDIOC_QBUF:
          return 0;
        case VIDIOC_STREAMON: {
          std::lock_guard lock(mu);
          ++streamon_count;
          return 0;
        }
        case VIDIOC_STREAMOFF: {
          std::lock_guard lock(mu);
          ++streamoff_count;
          return 0;
        }
        default:
          errno = EINVAL;
          return -1;
      }
    };
    sys.mmap_fn = [this](void*, size_t, int, int, int, off_t offset) -> void* {
      std::lock_guard lock(mu);
      ++active_mmaps;
      const size_t idx = static_cast<size_t>(offset / 4096) % 2;
      return backing[idx].data();
    };
    sys.munmap_fn = [this](void*, size_t) {
      std::lock_guard lock(mu);
      --active_mmaps;
      return 0;
    };
    sys.poll_fn = std::move(poll_fn);
    return sys;
  }
};

TEST(Capture, LinuxDisconnectReleasesDeviceAllowsSelfStopAndSetMode) {
  CameraSourceConfig config;
  config.device_path = "/dev/fakevideo0";
  config.width = FakeV4L2State::kWidth;
  config.height = FakeV4L2State::kHeight;
  config.fps = FakeV4L2State::kFps;
  config.format = PixelFormat::kGray8;

  // Case 1: POLLHUP unplug releases fd and mmaps BEFORE invoking the
  // disconnect callback, and calling Stop() inside the disconnect callback on
  // the worker thread does not self-join.
  {
    FakeV4L2State state;
    auto source = CreateV4L2CameraSourceForTest(
        config, state.Syscalls([](pollfd* fds, nfds_t, int) {
          fds[0].revents = POLLHUP;
          return 1;
        }));
    std::mutex mu;
    std::condition_variable cv;
    bool disconnected = false;
    int open_fds_in_cb = -1;
    int active_mmaps_in_cb = -1;
    bool running_in_cb = true;
    source->SetConnectionCallback([&](ConnectionState conn, std::string_view) {
      if (conn != ConnectionState::kDisconnected) return;
      {
        std::lock_guard lock(state.mu);
        open_fds_in_cb = state.open_fds;
        active_mmaps_in_cb = state.active_mmaps;
      }
      running_in_cb = source->IsRunning();
      source->Stop();
      {
        std::lock_guard lock(mu);
        disconnected = true;
      }
      cv.notify_one();
    });
    ASSERT_TRUE(source->Start());
    {
      std::unique_lock lock(mu);
      ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds{10},
                              [&] { return disconnected; }));
    }
    EXPECT_EQ(open_fds_in_cb, 0);
    EXPECT_EQ(active_mmaps_in_cb, 0);
    EXPECT_FALSE(running_in_cb);
    EXPECT_FALSE(source->IsRunning());
    EXPECT_FALSE(source->IsConnected());
    source->Stop();
  }

  // Case 2: After worker exits on disconnect WITHOUT calling Stop(), SetMode()
  // and SetControls() join the exited worker and succeed.
  {
    FakeV4L2State state;
    auto source = CreateV4L2CameraSourceForTest(
        config, state.Syscalls([](pollfd* fds, nfds_t, int) {
          fds[0].revents = POLLHUP;
          return 1;
        }));
    std::mutex mu;
    std::condition_variable cv;
    bool disconnected = false;
    source->SetConnectionCallback([&](ConnectionState conn, std::string_view) {
      if (conn != ConnectionState::kDisconnected) return;
      {
        std::lock_guard lock(mu);
        disconnected = true;
      }
      cv.notify_one();
    });
    ASSERT_TRUE(source->Start());
    {
      std::unique_lock lock(mu);
      ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds{10},
                              [&] { return disconnected; }));
    }
    EXPECT_FALSE(source->IsRunning());
    EXPECT_TRUE(source->SetMode({128, 96, 60.0, PixelFormat::kGray8}));
    EXPECT_TRUE(source->SetControls({.exposure_ms = 2.0, .gain = 5.0}));
    EXPECT_EQ(source->CurrentMode().width, 128u);
    EXPECT_DOUBLE_EQ(source->GetControls().exposure_ms, 2.0);
    {
      std::lock_guard lock(state.mu);
      EXPECT_EQ(state.open_fds, 0);
      EXPECT_EQ(state.active_mmaps, 0);
    }
  }
}

// FIELD-4 (no REQ row covers live capture modes yet): a live camera opens
// only the configured size and rate. Scenario: an Arducam OV9281 (uvcvideo)
// offers YUYV 1280x800 at 20/15 fps and 640x480 at 30/20 fps; asked for
// 640x480 at 25 fps the source opened, reported 25 fps and delivered 30, and
// asked for 1280x800 at 120 fps it reported 120 and delivered nothing.
TEST(Capture, LinuxOpensOnlyAnOfferedSizeAndRate) {
  // As uvcvideo enumerates them: intervals in 100 ns units.
  const auto mode = [](uint32_t format, uint32_t w, uint32_t h,
                       uint32_t interval_100ns) {
    return V4L2Mode{format, w, h, interval_100ns, 10000000};
  };
  const V4L2Mode offered[] = {
      mode(V4L2_PIX_FMT_YUYV, 1280, 800, 500000),
      mode(V4L2_PIX_FMT_YUYV, 1280, 800, 666666),
      mode(V4L2_PIX_FMT_YUYV, 640, 480, 333333),
      mode(V4L2_PIX_FMT_YUYV, 640, 480, 500000),
      mode(V4L2_PIX_FMT_GREY, 640, 480, 500000),
  };
  const struct {
    uint32_t width, height;
    double fps;
    std::optional<V4L2Mode> expected;
  } cases[] = {
      {640, 480, 30, offered[2]},
      // GREY needs no conversion: preferred over YUYV at the same mode.
      {640, 480, 20, offered[4]},
      {1280, 800, 20, offered[0]},
      {1280, 800, 15, offered[1]},
      {640, 480, 25, std::nullopt},
      {1280, 800, 30, std::nullopt},
      {1280, 720, 20, std::nullopt},
      {640, 480, 30.01, std::nullopt},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(testing::Message()
                 << c.width << "x" << c.height << "@" << c.fps);
    EXPECT_EQ(SelectV4L2Mode(c.width, c.height, c.fps, offered), c.expected);
  }
  // What the driver grants after S_FMT/S_PARM must be what was selected.
  const V4L2Mode selected = offered[2];
  EXPECT_TRUE(V4L2Granted(selected, selected));
  EXPECT_TRUE(V4L2Granted(selected, {V4L2_PIX_FMT_YUYV, 640, 480, 1, 30}));
  EXPECT_FALSE(V4L2Granted(selected, offered[3]));
  EXPECT_FALSE(V4L2Granted(selected, {V4L2_PIX_FMT_GREY, 640, 480, 1, 30}));
  EXPECT_FALSE(V4L2Granted(selected, {V4L2_PIX_FMT_YUYV, 640, 400, 1, 30}));
}

// FIELD-4: an OV9281 offers 1280x800 above 20 fps only as MJPG, a colour
// (4:2:2 YCbCr) JPEG with zero padding after its end marker. A frame decodes
// to the mode's gray image; one cut short on the bus, or of another size, is
// refused rather than delivered as a partial or wrong image.
TEST(Capture, LinuxMjpegFramesDecodeToGrayOrAreRefused) {
  cv::Mat bgr(80, 128, CV_8UC3);
  for (int y = 0; y < bgr.rows; ++y) {
    for (int x = 0; x < bgr.cols; ++x) {
      const auto v = static_cast<uint8_t>(x + y);
      bgr.at<cv::Vec3b>(y, x) = {v, v, v};
    }
  }
  std::vector<uint8_t> jpeg;
  ASSERT_TRUE(cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, 95}));
  cv::Mat expected;
  cv::cvtColor(bgr, expected, cv::COLOR_BGR2GRAY);
  std::vector<uint8_t> padded = jpeg;
  padded.resize(jpeg.size() + 64, 0);
  std::vector<uint8_t> gray(size_t{128} * 80, 0);
  ASSERT_TRUE(DecodeMjpegGray(padded.data(), padded.size(), 128, 80, gray));
  const cv::Mat decoded(80, 128, CV_8UC1, gray.data());
  EXPECT_LE(cv::norm(decoded, expected, cv::NORM_INF), 4.0);

  std::vector<uint8_t> truncated(jpeg.begin(), jpeg.end() - 200);
  EXPECT_FALSE(
      DecodeMjpegGray(truncated.data(), truncated.size(), 128, 80, gray));
  std::vector<uint8_t> other_size(size_t{64} * 80, 0);
  EXPECT_FALSE(DecodeMjpegGray(jpeg.data(), jpeg.size(), 64, 80, other_size));

  // Null input, zero dimensions, or dimensions exceeding INT_MAX must be
  // rejected before indexing `data` or constructing cv::Mat.
  EXPECT_FALSE(DecodeMjpegGray(nullptr, 16, 128, 80, gray));
  EXPECT_FALSE(DecodeMjpegGray(nullptr, 0, 128, 80, gray));
  std::vector<uint8_t> empty;
  EXPECT_FALSE(DecodeMjpegGray(jpeg.data(), jpeg.size(), 0, 80, empty));
  EXPECT_FALSE(DecodeMjpegGray(jpeg.data(), jpeg.size(), 128, 0, empty));
  constexpr uint32_t kOverIntMax =
      static_cast<uint32_t>(std::numeric_limits<int>::max()) + 1u;
  EXPECT_FALSE(
      DecodeMjpegGray(jpeg.data(), jpeg.size(), kOverIntMax, 1, empty));
  EXPECT_FALSE(
      DecodeMjpegGray(jpeg.data(), jpeg.size(), 1, kOverIntMax, empty));
}

// An Arducam OV9281's controls as uvcvideo reports them: auto_exposure menu
// {1 manual, 3 aperture priority}, exposure_time_absolute 1..5000 (100 us),
// exposure_dynamic_framerate, gain 0..100, no gain_automatic.
class FakeControls final : public V4L2ControlDevice {
 public:
  struct Control {
    Range range;
    std::vector<int32_t> menu;
    int32_t value{0};
    bool refuse{false};
    int32_t holds_instead{-1};  // A device that keeps another value.
  };
  std::map<uint32_t, Control> controls{
      {V4L2_CID_EXPOSURE_AUTO, {{0, 3}, {1, 3}, 3}},
      {V4L2_CID_EXPOSURE_ABSOLUTE, {{1, 5000}, {}, 157}},
      {V4L2_CID_EXPOSURE_AUTO_PRIORITY, {{0, 1}, {}, 1}},
      {V4L2_CID_GAIN, {{0, 100}, {}, 0}},
  };
  std::optional<Range> Query(uint32_t id) override {
    const auto it = controls.find(id);
    if (it == controls.end()) return std::nullopt;
    return it->second.range;
  }
  bool MenuHas(uint32_t id, int32_t value) override {
    const auto& menu = controls.at(id).menu;
    return std::find(menu.begin(), menu.end(), value) != menu.end();
  }
  bool Set(uint32_t id, int32_t value) override {
    auto& control = controls.at(id);
    if (control.refuse) return false;
    control.value = control.holds_instead >= 0 ? control.holds_instead : value;
    return true;
  }
  std::optional<int32_t> Get(uint32_t id) override {
    return controls.at(id).value;
  }
};

// FIELD-4 / PLATFORM-1: the configured exposure and gain reach the device in
// its units and are read back, or the open fails naming the control.
// Scenario: on an OV9281 the old writer requested auto exposure as mode 0
// (the camera offers only 1 and 3) and auto gain through a control it does
// not have, and a camera holding another value than the one set went
// unnoticed; capture continued with controls_applied false.
TEST(Capture, LinuxControlsReachTheDeviceOrNameTheOneRefused) {
  {
    FakeControls device;
    CameraControls manual{.exposure_ms = 4.5, .gain = 12};
    EXPECT_EQ(ApplyV4L2Controls(manual, device), "");
    EXPECT_EQ(device.controls[V4L2_CID_EXPOSURE_AUTO].value,
              V4L2_EXPOSURE_MANUAL);
    EXPECT_EQ(device.controls[V4L2_CID_EXPOSURE_ABSOLUTE].value, 45);
    EXPECT_EQ(device.controls[V4L2_CID_EXPOSURE_AUTO_PRIORITY].value, 0);
    EXPECT_EQ(device.controls[V4L2_CID_GAIN].value, 12);
  }
  {
    FakeControls device;
    CameraControls automatic{.exposure_ms = 4.5, .auto_exposure = true};
    EXPECT_EQ(ApplyV4L2Controls(automatic, device), "");
    EXPECT_EQ(device.controls[V4L2_CID_EXPOSURE_AUTO].value,
              V4L2_EXPOSURE_APERTURE_PRIORITY);
    EXPECT_EQ(device.controls[V4L2_CID_EXPOSURE_ABSOLUTE].value, 157);
    EXPECT_EQ(device.controls[V4L2_CID_EXPOSURE_AUTO_PRIORITY].value, 0);
  }
  const struct {
    const char* name;
    CameraControls controls;
    void (*setup)(FakeControls&);
    const char* control;
  } failures[] = {
      {"auto gain without gain_automatic",
       {.exposure_ms = 4.5, .gain = 0, .auto_gain = true},
       [](FakeControls&) {},
       "gain_automatic"},
      {"exposure beyond the range",
       {.exposure_ms = 600, .gain = 0},
       [](FakeControls&) {},
       "exposure_time_absolute"},
      {"gain beyond the range",
       {.exposure_ms = 4.5, .gain = 150},
       [](FakeControls&) {},
       "gain"},
      {"mode refused",
       {.exposure_ms = 4.5, .gain = 0},
       [](FakeControls& d) {
         d.controls[V4L2_CID_EXPOSURE_AUTO].refuse = true;
       },
       "auto_exposure"},
      {"exposure held at another value",
       {.exposure_ms = 4.5, .gain = 0},
       [](FakeControls& d) {
         d.controls[V4L2_CID_EXPOSURE_ABSOLUTE].holds_instead = 50;
       },
       "exposure_time_absolute"},
      {"no automatic exposure mode",
       {.exposure_ms = 0, .gain = 0, .auto_exposure = true},
       [](FakeControls& d) {
         d.controls[V4L2_CID_EXPOSURE_AUTO].menu = {V4L2_EXPOSURE_MANUAL};
       },
       "auto_exposure"},
  };
  for (const auto& failure : failures) {
    SCOPED_TRACE(failure.name);
    FakeControls device;
    failure.setup(device);
    const auto error = ApplyV4L2Controls(failure.controls, device);
    EXPECT_EQ(error.rfind(failure.control, 0), 0u) << error;
  }
}

TEST(Capture, LinuxControlsRejectNonFiniteAndHugeFiniteValues) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const struct {
    const char* name;
    CameraControls controls;
    const char* control;
  } cases[] = {
      {"NaN exposure_ms",
       {.exposure_ms = nan, .gain = 0},
       "exposure_time_absolute"},
      {"-inf exposure_ms",
       {.exposure_ms = -inf, .gain = 0},
       "exposure_time_absolute"},
      {"+inf exposure_ms",
       {.exposure_ms = inf, .gain = 0},
       "exposure_time_absolute"},
      {"huge finite exposure_ms",
       {.exposure_ms = 1e30, .gain = 0},
       "exposure_time_absolute"},
      {"NaN gain", {.exposure_ms = 0, .gain = nan}, "gain"},
      {"-inf gain", {.exposure_ms = 0, .gain = -inf}, "gain"},
      {"+inf gain", {.exposure_ms = 0, .gain = inf}, "gain"},
      {"huge finite gain", {.exposure_ms = 0, .gain = 1e30}, "gain"},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    FakeControls device;
    std::feclearexcept(FE_ALL_EXCEPT);
    const auto error = ApplyV4L2Controls(c.controls, device);
    EXPECT_EQ(std::fetestexcept(FE_INVALID), 0);
    EXPECT_EQ(error.rfind(c.control, 0), 0u) << error;
  }
}
#endif
TEST(Capture, FrameDurationIsOneTheRangeAccepts) {
  // An Arducam OV9281 (UVC) reports single-rate ranges through AVFoundation,
  // measured: 1 / fps itself (1000 / 60000) is refused with an exception.
  struct Range {
    double min_fps, max_fps;
    FrameDuration shortest, longest;
  };
  constexpr FrameDuration k30{1000000, 30000030}, k60{1000000, 60000240},
      k120{1000000, 120000480}, k100{1000000, 100000000};
  const Range uvc30{30.000030000030002, 30.000030000030002, k30, k30};
  const Range uvc60{60.000240000960005, 60.000240000960005, k60, k60};
  const Range uvc120{120.00048000192001, 120.00048000192001, k120, k120};
  const Range uvc100{100, 100, k100, k100};
  // A continuous range; its slow end is 3.75 fps as a UVC interval.
  const Range wide{3.7500009375002343, 30, {1, 30}, {2666666, 10000000}};
  const struct {
    double fps;
    Range range;
    std::optional<FrameDuration> expected;
  } cases[] = {
      {30, uvc30, k30},
      {60, uvc60, k60},
      {120, uvc120, k120},
      {100, uvc100, k100},
      {60, uvc30, std::nullopt},
      {59.9, uvc60, std::nullopt},
      {120.1, uvc120, std::nullopt},
      {15, wide, FrameDuration{1000, 15000}},
      {30, wide, FrameDuration{1, 30}},
      {3.75, wide, FrameDuration{2666666, 10000000}},
      // 1 / fps would be 1000 / 3750, longer than the range allows.
      {3.7504, wide, FrameDuration{2666666, 10000000}},
      {3.7, wide, std::nullopt},
      {31, wide, std::nullopt},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(testing::Message() << c.fps << " fps in [" << c.range.min_fps
                                    << ", " << c.range.max_fps << "]");
    EXPECT_EQ(FrameDurationFor(c.fps, c.range.min_fps, c.range.max_fps,
                               c.range.shortest, c.range.longest),
              c.expected);
  }

  // Non-positive or non-finite rates and non-positive timescales are rejected,
  // and tiny positive interior rates never yield timescale == 0.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(FrameDurationFor(1e-5, 0.0, 30.0, {1, 30}, {1, 1}), std::nullopt);
  EXPECT_EQ(FrameDurationFor(0.0, 0.0, 30.0, {1, 30}, {1, 1}), std::nullopt);
  EXPECT_EQ(FrameDurationFor(-1.0, 1.0, 30.0, {1, 30}, {1, 1}), std::nullopt);
  EXPECT_EQ(FrameDurationFor(15.0, -1.0, 30.0, {1, 30}, {1, 1}), std::nullopt);
  EXPECT_EQ(FrameDurationFor(15.0, 1.0, 0.0, {1, 30}, {1, 1}), std::nullopt);
  EXPECT_EQ(FrameDurationFor(nan, 1.0, 30.0, {1, 30}, {1, 1}), std::nullopt);
  EXPECT_EQ(FrameDurationFor(inf, 1.0, 30.0, {1, 30}, {1, 1}), std::nullopt);
  EXPECT_EQ(FrameDurationFor(30.0, 1.0, 30.0, {1, 0}, {1, 1}), std::nullopt);
  EXPECT_EQ(FrameDurationFor(1.0, 1.0, 30.0, {1, 30}, {1, 0}), std::nullopt);

  const auto slow = FrameDurationFor(1e-4, 1e-5, 30.0, {1, 30}, {100000, 1});
  ASSERT_TRUE(slow.has_value());
  EXPECT_GE(slow->timescale, 1);
  EXPECT_GT(slow->value, 0);
  EXPECT_GE(slow->seconds(), (FrameDuration{1, 30}.seconds()));
  EXPECT_LE(slow->seconds(), (FrameDuration{100000, 1}.seconds()));
}
TEST(Capture, MissingDeviceFailsAndStopIsIdempotent) {
  CameraSourceConfig config;
  config.device_path = "/talos-nonexistent-camera-device";
  auto source = CreatePlatformCameraSource(config);
  if (!source) return;  // Unsupported platforms have no capture implementation.
  EXPECT_FALSE(source->IsRunning());
  source->Stop();
  EXPECT_FALSE(source->Start());
  EXPECT_FALSE(source->IsRunning());
  source->Stop();
  source->Stop();
  EXPECT_FALSE(source->Start());
  source->Stop();
}

TEST(Capture, ConnectionCallbackCanReplaceItselfWhileInvoked) {
  CameraSourceConfig config;
  config.device_path = "/talos-nonexistent-camera-device";
  auto source = CreatePlatformCameraSource(config);
  if (!source) return;  // Unsupported platforms have no capture implementation.
  auto token = std::make_shared<int>(42);
  std::weak_ptr<int> weak = token;
  bool invoked = false;
  bool expired_during_callback = true;
  source->SetConnectionCallback(
      [&source, &invoked, &expired_during_callback, weak,
       owned = std::move(token)](ConnectionState, std::string_view) {
        (void)owned;
        invoked = true;
        source->SetConnectionCallback({});
        expired_during_callback = weak.expired();
      });
  EXPECT_FALSE(source->Start());
  EXPECT_TRUE(invoked);
  EXPECT_FALSE(expired_during_callback);
}
}  // namespace
}  // namespace talos::vision
