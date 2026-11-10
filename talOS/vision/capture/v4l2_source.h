#pragma once

#include "talOS/vision/capture/avfoundation_source.h"

#if defined(__linux__)
#include <linux/videodev2.h>

#include <cmath>
#include <string_view>

namespace talos::vision {
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
