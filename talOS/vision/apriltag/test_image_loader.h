#pragma once

// Test-only grayscale fixture loader built on OpenCV.
//
// Replaces direct uses of the upstream `image_u8_create_from_pnm` helper
// (and one hand-rolled PNM header parse in cpu_apriltag_backend_test) now
// that fixtures are PNG: the OpenCV build in MODULE.bazel cannot decode
// PNM, and PNG twins are pixel-identical to the removed .pnm files.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <opencv2/imgcodecs.hpp>
#include <string>

extern "C" {
#include "common/image_u8.h"
}

namespace talos::vision::apriltag {

// Loads a grayscale image file (PNG/JPEG) into an apriltag image_u8_t.
// Returns nullptr when the file cannot be decoded. The caller owns the
// result and must release it with image_u8_destroy.
inline image_u8_t* LoadGrayImageU8(const std::string& path) {
  const cv::Mat gray = cv::imread(path, cv::IMREAD_GRAYSCALE);
  if (gray.empty() || gray.cols <= 0 || gray.rows <= 0) {
    return nullptr;
  }
  image_u8_t* im = image_u8_create(static_cast<uint32_t>(gray.cols),
                                   static_cast<uint32_t>(gray.rows));
  for (int y = 0; y < gray.rows; ++y) {
    std::memcpy(im->buf + y * im->stride, gray.ptr<uint8_t>(y),
                static_cast<size_t>(gray.cols));
  }
  return im;
}

}  // namespace talos::vision::apriltag
