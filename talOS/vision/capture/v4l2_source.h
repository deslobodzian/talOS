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
inline constexpr uint32_t kV4L2PixelFormats[] = {V4L2_PIX_FMT_GREY,
                                                 V4L2_PIX_FMT_YUYV};

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

// Injectable control writer keeps device units and failure handling testable.
template <class WriteControl>
bool ApplyV4L2Controls(const CameraControls& controls, WriteControl write) {
  bool applied =
      write(V4L2_CID_EXPOSURE_AUTO,
            controls.auto_exposure ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL);
  if (!controls.auto_exposure && controls.exposure_ms > 0) {
    const bool valid =
        std::isfinite(controls.exposure_ms) && controls.exposure_ms < 214748364;
    applied =
        (valid &&
         write(V4L2_CID_EXPOSURE_ABSOLUTE,
               static_cast<int32_t>(std::lround(controls.exposure_ms * 10)))) &&
        applied;
  }
  if (controls.auto_gain) {
    applied = write(V4L2_CID_AUTOGAIN, 1) && applied;
  } else if (controls.gain > 0) {
    applied = write(V4L2_CID_AUTOGAIN, 0) && applied;
    applied = (std::isfinite(controls.gain) && controls.gain < 2147483647 &&
               write(V4L2_CID_GAIN,
                     static_cast<int32_t>(std::lround(controls.gain)))) &&
              applied;
  }
  return applied;
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
