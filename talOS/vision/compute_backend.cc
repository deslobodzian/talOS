#include "talOS/vision/compute_backend.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

namespace talos::vision {

struct CpuAprilTagBackend::Impl {
  apriltag::DetectorOptions options;
  apriltag::CpuAprilTagBackend detector;
  std::mutex detector_mutex;
  explicit Impl(const apriltag::DetectorOptions& opts)
      : options(opts), detector(opts) {}
};

CpuAprilTagBackend::CpuAprilTagBackend(int threads, double decimate,
                                       double sigma, bool refine)
    : CpuAprilTagBackend(apriltag::DetectorOptions{
          .threads = threads,
          .quad_decimate = static_cast<float>(decimate),
          .quad_sigma = static_cast<float>(sigma),
          .refine_edges = refine}) {}
CpuAprilTagBackend::CpuAprilTagBackend(const apriltag::DetectorOptions& opts)
    : impl_(std::make_unique<Impl>(opts)) {}

CpuAprilTagBackend::~CpuAprilTagBackend() = default;

std::string CpuAprilTagBackend::device_info() const {
  return std::format("CPU AprilTag (threads={}, decimate={:.1f}, sigma={:.1f})",
                     impl_->options.threads, impl_->options.quad_decimate,
                     impl_->options.quad_sigma);
}

BackendExecutionResult CpuAprilTagBackend::Execute(const Frame& frame) {
  BackendExecutionResult result;
  if (!frame.holds_image()) {
    result.success = false;
    result.error_message =
        "Frame has invalid dimensions or stores fewer than stride * height "
        "bytes";
    return result;
  }

  const auto t_start = std::chrono::steady_clock::now();
  const uint32_t w = frame.width();
  const uint32_t h = frame.height();
  const uint32_t stride = frame.stride();
  constexpr uint32_t kMaxInt =
      static_cast<uint32_t>(std::numeric_limits<int>::max());
  if (w > kMaxInt || h > kMaxInt || stride > kMaxInt) {
    result.success = false;
    result.error_message = "Frame dimensions or stride exceed INT_MAX";
    return result;
  }

  std::vector<uint8_t> gray_storage;
  const uint8_t* gray_ptr = nullptr;
  uint32_t gray_stride = stride;

  auto t_gray_start = std::chrono::steady_clock::now();

  if (frame.format() == PixelFormat::kGray8) {
    gray_ptr = frame.data();
  } else if (frame.format() == PixelFormat::kYuyv && (w & 1u) != 0u) {
    // OpenCV's COLOR_YUV2GRAY_YUYV rejects odd widths; extract each pixel's Y
    // byte (offset 2 * x) directly.
    gray_storage.resize(static_cast<size_t>(w) * h);
    gray_stride = w;
    const uint8_t* src = frame.data();
    for (uint32_t y = 0; y < h; ++y) {
      const uint8_t* src_row = src + static_cast<size_t>(y) * stride;
      uint8_t* dst_row = gray_storage.data() + static_cast<size_t>(y) * w;
      for (uint32_t x = 0; x < w; ++x) {
        dst_row[x] = src_row[static_cast<size_t>(x) * 2];
      }
    }
    gray_ptr = gray_storage.data();
  } else {
    int src_type = 0;
    int code = 0;
    switch (frame.format()) {
      case PixelFormat::kRgb8:
        src_type = CV_8UC3;
        code = cv::COLOR_RGB2GRAY;
        break;
      case PixelFormat::kBgr8:
        src_type = CV_8UC3;
        code = cv::COLOR_BGR2GRAY;
        break;
      case PixelFormat::kBgra8:
        src_type = CV_8UC4;
        code = cv::COLOR_BGRA2GRAY;
        break;
      case PixelFormat::kYuyv:
        src_type = CV_8UC2;
        code = cv::COLOR_YUV2GRAY_YUYV;
        break;
      case PixelFormat::kGray8:
        break;  // Unreachable: handled above.
      default:
        result.success = false;
        result.error_message = "Unsupported pixel format";
        return result;
    }
    gray_storage.resize(static_cast<size_t>(w) * h);
    gray_stride = w;
    try {
      cv::Mat src_mat(static_cast<int>(h), static_cast<int>(w), src_type,
                      const_cast<uint8_t*>(frame.data()), stride);
      cv::Mat dst_mat(static_cast<int>(h), static_cast<int>(w), CV_8UC1,
                      gray_storage.data(), w);
      cv::cvtColor(src_mat, dst_mat, code);
    } catch (const cv::Exception& e) {
      result.success = false;
      result.error_message = e.what();
      return result;
    }
    gray_ptr = gray_storage.data();
  }

  auto t_gray_end = std::chrono::steady_clock::now();
  result.timings.grayscale_us =
      std::chrono::duration_cast<std::chrono::microseconds>(t_gray_end -
                                                            t_gray_start)
          .count();

  const auto t_detect_start = std::chrono::steady_clock::now();
  std::vector<apriltag::Detection> detections;
  {
    std::lock_guard lock(impl_->detector_mutex);
    detections = impl_->detector.Detect(gray_ptr, static_cast<int>(w),
                                        static_cast<int>(h),
                                        static_cast<int>(gray_stride));
  }
  // Upstream's public API does not separate threshold/quad/decode timing.
  result.timings.quad_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - t_detect_start)
          .count();
  result.detections.reserve(detections.size());
  for (const auto& det : detections) {
    result.detections.push_back(ToRawTagDetection(det));
  }
  result.effective_backend = "cpu";
  auto t_end = std::chrono::steady_clock::now();
  result.timings.total_us =
      std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start)
          .count();
  result.success = true;
  return result;
}

}  // namespace talos::vision
