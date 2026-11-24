#pragma once

#include "talOS/vision/capture/avfoundation_source.h"

#if defined(__linux__)
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/time.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
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
    const double rounded = std::round(value);
    if (!std::isfinite(rounded) ||
        rounded < static_cast<double>(range->minimum) ||
        rounded > static_cast<double>(range->maximum)) {
      return std::string(name) + ": " + std::to_string(value) +
             " outside the device's range " + std::to_string(range->minimum) +
             ".." + std::to_string(range->maximum);
    }
    const double clamped = std::clamp(
        rounded, static_cast<double>(std::numeric_limits<int32_t>::min()),
        static_cast<double>(std::numeric_limits<int32_t>::max()));
    device_value = static_cast<int32_t>(std::lround(clamped));
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
    error = set(V4L2_CID_EXPOSURE_AUTO, "auto_exposure", mode);
    if (!error.empty()) return error;
  } else if (has_mode) {
    error = set(V4L2_CID_EXPOSURE_AUTO, "auto_exposure", V4L2_EXPOSURE_MANUAL);
    if (!error.empty()) return error;
  }
  if (device.Query(V4L2_CID_EXPOSURE_AUTO_PRIORITY)) {
    error =
        set(V4L2_CID_EXPOSURE_AUTO_PRIORITY, "exposure_dynamic_framerate", 0);
    if (!error.empty()) return error;
  }
  if (!controls.auto_exposure) {
    if (!std::isfinite(controls.exposure_ms)) {
      return "exposure_time_absolute: non-finite exposure_ms";
    }
    if (controls.exposure_ms > 0) {
      int32_t units = 0;
      error = in_range(V4L2_CID_EXPOSURE_ABSOLUTE, "exposure_time_absolute",
                       controls.exposure_ms * 10, units);
      if (!error.empty()) return error;
      error = set(V4L2_CID_EXPOSURE_ABSOLUTE, "exposure_time_absolute", units);
      if (!error.empty()) return error;
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
  if (!std::isfinite(controls.gain)) {
    return "gain: non-finite gain";
  }
  if (controls.gain > 0) {
    if (has_auto_gain) {
      error = set(V4L2_CID_AUTOGAIN, "gain_automatic", 0);
      if (!error.empty()) return error;
    }
    int32_t value = 0;
    error = in_range(V4L2_CID_GAIN, "gain", controls.gain, value);
    if (!error.empty()) return error;
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

inline constexpr int64_t kMaxV4L2DriverTimestampAgeNs = 5'000'000'000LL;

struct V4L2ResolvedTimestamp {
  int64_t capture_time_ns{0};
  int64_t raw_timestamp_ns{0};
  TimestampKind kind{TimestampKind::kDelivery};
  constexpr bool operator==(const V4L2ResolvedTimestamp&) const = default;
};

// Maps a V4L2 buffer timestamp into the node clock domain. A monotonic driver
// timestamp is trusted only when it is nonzero and lies in
// [delivery - 5 s, delivery] (matching AVFoundation's sanity window); zero,
// future, stale, or malformed timestamps fall back to `node_delivery_ns` with
// TimestampKind::kDelivery.
inline V4L2ResolvedTimestamp ResolveV4L2Timestamp(
    uint32_t flags, const timeval& driver_timestamp, int64_t delivery_ns,
    int64_t node_delivery_ns, std::string_view driver = {}) {
  const V4L2ResolvedTimestamp fallback{node_delivery_ns, delivery_ns,
                                       TimestampKind::kDelivery};
  if ((flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) !=
      V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
    return fallback;
  }
  if (driver_timestamp.tv_sec < 0 || driver_timestamp.tv_usec < 0 ||
      driver_timestamp.tv_usec >= 1'000'000) {
    return fallback;
  }
  constexpr int64_t kMaxSec =
      (std::numeric_limits<int64_t>::max() - 999'999'000LL) / 1'000'000'000LL;
  if (driver_timestamp.tv_sec > kMaxSec) {
    return fallback;
  }
  const int64_t driver_ns =
      static_cast<int64_t>(driver_timestamp.tv_sec) * 1'000'000'000LL +
      static_cast<int64_t>(driver_timestamp.tv_usec) * 1'000LL;
  if (driver_ns <= 0 || driver_ns > delivery_ns) {
    return fallback;
  }
  const int64_t age_ns = delivery_ns - driver_ns;
  if (age_ns < 0 || age_ns >= kMaxV4L2DriverTimestampAgeNs ||
      node_delivery_ns < age_ns) {
    return fallback;
  }
  return {node_delivery_ns - age_ns, driver_ns,
          V4L2TimestampKind(flags, driver)};
}

// OS syscalls used by the Linux V4L2 CameraSource; tests can substitute a fake
// device without touching /dev/videoN.
struct V4L2Syscalls {
  std::function<int(const char*, int)> open_fn;
  std::function<int(int)> close_fn;
  // NOLINTNEXTLINE(google-runtime-int): glibc ioctl request type
  std::function<int(int, unsigned long, void*)> ioctl_fn;
  std::function<void*(void*, size_t, int, int, int, off_t)> mmap_fn;
  std::function<int(void*, size_t)> munmap_fn;
  std::function<int(pollfd*, nfds_t, int)> poll_fn;
};

std::unique_ptr<CameraSource> CreateV4L2CameraSourceForTest(
    const CameraSourceConfig& config, V4L2Syscalls syscalls);
}  // namespace talos::vision
#endif
