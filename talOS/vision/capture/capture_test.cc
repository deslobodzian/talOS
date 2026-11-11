#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <map>
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
  std::vector<uint8_t> gray(128 * 80, 0);
  ASSERT_TRUE(DecodeMjpegGray(padded.data(), padded.size(), 128, 80, gray));
  const cv::Mat decoded(80, 128, CV_8UC1, gray.data());
  EXPECT_LE(cv::norm(decoded, expected, cv::NORM_INF), 4.0);

  std::vector<uint8_t> truncated(jpeg.begin(), jpeg.end() - 200);
  EXPECT_FALSE(
      DecodeMjpegGray(truncated.data(), truncated.size(), 128, 80, gray));
  std::vector<uint8_t> other_size(64 * 80, 0);
  EXPECT_FALSE(DecodeMjpegGray(jpeg.data(), jpeg.size(), 64, 80, other_size));
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
}  // namespace
}  // namespace talos::vision
