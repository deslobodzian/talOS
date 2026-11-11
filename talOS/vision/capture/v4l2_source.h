#pragma once

#include "talOS/vision/capture/avfoundation_source.h"

#if defined(__linux__)
#include <linux/videodev2.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace talos::vision {
// One frame size and frame interval a V4L2 device offers in one pixel format
// (VIDIOC_ENUM_FMT, _ENUM_FRAMESIZES, _ENUM_FRAMEINTERVALS), or the one
// VIDIOC_S_FMT and VIDIOC_S_PARM granted.
struct V4L2Mode {
  uint32_t pixel_format{0};  // V4L2_PIX_FMT_*
  uint32_t width{0};
  uint32_t height{0};
  // Seconds per frame, numerator / denominator.
  uint32_t interval_numerator{0};
  uint32_t interval_denominator{0};
  constexpr double fps() const {
    return interval_numerator == 0
               ? 0.0
               : static_cast<double>(interval_denominator) / interval_numerator;
  }
  constexpr bool operator==(const V4L2Mode&) const = default;
};

// The pixel formats the source opens, in preference order.
// GREY and YUYV arrive uncompressed (exact pixels); MJPG, often the only
// format at a high rate (an OV9281's YUYV tops out at 20 fps at 1280x800),
// is decoded to gray on the capture thread.
inline constexpr uint32_t kV4L2PixelFormats[] = {
    V4L2_PIX_FMT_GREY, V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_MJPEG};

// Decodes one MJPG frame's luma into `gray` (`width` x `height`, sized by the
// caller, stride `width`). False, with `gray` unspecified, when the frame has
// no end-of-image marker (zero padding after it is allowed), does not decode,
// or is another size.
bool DecodeMjpegGray(const uint8_t* data, size_t size, uint32_t width,
                     uint32_t height, std::vector<uint8_t>& gray);

// Whether `offered` runs at `fps`. UVC states intervals in 100 ns units, so a
// camera's 120 fps is 83333 / 10000000 s (120.0005 fps); the same relative
// 1e-4 tolerance as FrameDurationFor accepts it and stays far below the gap
// between two rates a camera offers.
inline bool V4L2RateMatches(double fps, const V4L2Mode& offered) {
  const double rate = offered.fps();
  return rate > 0 && std::abs(fps - rate) <= rate * 1e-4;
}

// The offered mode to open for `width` x `height` at `fps`: the first pixel
// format in kV4L2PixelFormats that offers that size at that rate, or nullopt.
// The source never opens a different size or rate than the one configured.
inline std::optional<V4L2Mode> SelectV4L2Mode(
    uint32_t width, uint32_t height, double fps,
    std::span<const V4L2Mode> offered) {
  for (const uint32_t format : kV4L2PixelFormats) {
    for (const auto& mode : offered) {
      if (mode.pixel_format == format && mode.width == width &&
          mode.height == height && V4L2RateMatches(fps, mode)) {
        return mode;
      }
    }
  }
  return std::nullopt;
}

// Whether the driver granted the selected mode: the same pixel format and
// size, and an interval at the same rate.
inline bool V4L2Granted(const V4L2Mode& selected, const V4L2Mode& granted) {
  return granted.pixel_format == selected.pixel_format &&
         granted.width == selected.width && granted.height == selected.height &&
         V4L2RateMatches(selected.fps(), granted);
}

// "MJPG 1280x800@120.000" for a failure message.
inline std::string DescribeV4L2Mode(const V4L2Mode& mode) {
  std::string text;
  for (int shift = 0; shift < 32; shift += 8) {
    const char c = static_cast<char>((mode.pixel_format >> shift) & 0xff);
    if (c != ' ' && c != '\0') text += c;
  }
  char size[64];
  std::snprintf(size, sizeof(size), " %ux%u@%.3f", mode.width, mode.height,
                mode.fps());
  return text + size;
}

// A device's controls as the source reaches them (VIDIOC_QUERYCTRL,
// _QUERYMENU, _S_CTRL, _G_CTRL); tests fake it.
class V4L2ControlDevice {
 public:
  struct Range {
    int32_t minimum{0};
    int32_t maximum{0};
  };
  virtual ~V4L2ControlDevice() = default;
  // The control's range, or nullopt when the device has no such control (or
  // it is disabled).
  virtual std::optional<Range> Query(uint32_t id) = 0;
  // Whether menu control `id` offers item `value`.
  virtual bool MenuHas(uint32_t id, int32_t value) = 0;
  virtual bool Set(uint32_t id, int32_t value) = 0;
  virtual std::optional<int32_t> Get(uint32_t id) = 0;
};

// Applies `controls` and reads every written control back. Returns "" when
// the device holds them all, else why not, naming the control (its V4L2
// name, as v4l2-ctl prints it). Nothing falls back:
// - auto exposure sets auto_exposure to auto, else to aperture priority (the
//   automatic mode UVC cameras offer); a device with neither fails;
// - manual exposure sets auto_exposure to manual (when the device has the
//   control) and, for exposure_ms > 0, exposure_time_absolute in 100 us
//   units, which must lie in the device's range;
// - exposure_dynamic_framerate, which lets the device lengthen the frame
//   period for exposure, is turned off whenever the device has it;
// - auto gain needs gain_automatic; gain > 0 turns gain_automatic off (when
//   present) and sets gain, which must exist and lie in the device's range.
// Zero exposure or gain without the automatic flag leaves the device's value.
inline std::string ApplyV4L2Controls(const CameraControls& controls,
                                     V4L2ControlDevice& device) {
  const auto set = [&](uint32_t id, const char* name,
                       int32_t value) -> std::string {
    if (!device.Set(id, value)) {
      return std::string(name) + ": device refused " + std::to_string(value);
    }
    const auto held = device.Get(id);
    if (held != value) {
      return std::string(name) + ": set " + std::to_string(value) +
             ", device holds " + (held ? std::to_string(*held) : "nothing");
    }
    return {};
  };
  const auto in_range = [&](uint32_t id, const char* name, double value,
                            int32_t& device_value) -> std::string {
    const auto range = device.Query(id);
    if (!range) return std::string(name) + ": device has no such control";
    if (!std::isfinite(value) || std::lround(value) < range->minimum ||
        std::lround(value) > range->maximum) {
      return std::string(name) + ": " + std::to_string(value) +
             " outside the device's range " + std::to_string(range->minimum) +
             ".." + std::to_string(range->maximum);
    }
    device_value = static_cast<int32_t>(std::lround(value));
    return {};
  };
  std::string error;
  const bool has_mode = device.Query(V4L2_CID_EXPOSURE_AUTO).has_value();
  if (controls.auto_exposure) {
    int32_t mode = -1;
    for (const int32_t candidate :
         {static_cast<int32_t>(V4L2_EXPOSURE_AUTO),
          static_cast<int32_t>(V4L2_EXPOSURE_APERTURE_PRIORITY)}) {
      if (has_mode && device.MenuHas(V4L2_CID_EXPOSURE_AUTO, candidate)) {
        mode = candidate;
        break;
      }
    }
    if (mode < 0) return "auto_exposure: device offers no automatic mode";
    if (!(error = set(V4L2_CID_EXPOSURE_AUTO, "auto_exposure", mode)).empty()) {
      return error;
    }
  } else if (has_mode) {
    if (!(error = set(V4L2_CID_EXPOSURE_AUTO, "auto_exposure",
                      V4L2_EXPOSURE_MANUAL))
             .empty()) {
      return error;
    }
  }
  if (device.Query(V4L2_CID_EXPOSURE_AUTO_PRIORITY) &&
      !(error = set(V4L2_CID_EXPOSURE_AUTO_PRIORITY,
                    "exposure_dynamic_framerate", 0))
           .empty()) {
    return error;
  }
  if (!controls.auto_exposure && controls.exposure_ms > 0) {
    int32_t units = 0;
    if (!(error = in_range(V4L2_CID_EXPOSURE_ABSOLUTE, "exposure_time_absolute",
                           controls.exposure_ms * 10, units))
             .empty() ||
        !(error =
              set(V4L2_CID_EXPOSURE_ABSOLUTE, "exposure_time_absolute", units))
             .empty()) {
      return error;
    }
  }
  const bool has_auto_gain = device.Query(V4L2_CID_AUTOGAIN).has_value();
  if (controls.auto_gain) {
    if (!has_auto_gain) {
      return "gain_automatic: device has no automatic gain; set camera.gain "
             "or camera.auto_gain = false";
    }
    return set(V4L2_CID_AUTOGAIN, "gain_automatic", 1);
  }
  if (controls.gain > 0) {
    if (has_auto_gain &&
        !(error = set(V4L2_CID_AUTOGAIN, "gain_automatic", 0)).empty()) {
      return error;
    }
    int32_t value = 0;
    if (!(error = in_range(V4L2_CID_GAIN, "gain", controls.gain, value))
             .empty()) {
      return error;
    }
    return set(V4L2_CID_GAIN, "gain", value);
  }
  return {};
}

// uvcvideo labels its buffers SOE, but unless the module's hwtimestamps
// parameter is set (default off) the stamp is the host arrival time of the
// frame's first USB packet, after exposure has started; adding half an
// exposure would move it the wrong way. It is a delivery stamp, corrected by
// the camera's measured latency_offset_us only.
inline TimestampKind V4L2TimestampKind(uint32_t flags,
                                       std::string_view driver = {}) {
  if ((flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) !=
      V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
    return TimestampKind::kDelivery;
  if ((flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) != V4L2_BUF_FLAG_TSTAMP_SRC_SOE)
    return TimestampKind::kEndOfFrame;
  return driver == "uvcvideo" ? TimestampKind::kDelivery
                              : TimestampKind::kStartOfExposure;
}
}  // namespace talos::vision
#endif
