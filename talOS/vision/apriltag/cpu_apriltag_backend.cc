#include "talOS/vision/apriltag/cpu_apriltag_backend.h"

#include <array>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "talOS/process/process.h"
#include "talOS/vision/localization.h"

extern "C" {
#include <apriltag.h>
#include <common/image_u8.h>
#include <common/matd.h>
#include <common/pjpeg.h>
#include <common/workerpool.h>
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>
}

namespace talos::vision::apriltag {

namespace {

// One task per pool thread: each lowers its own thread, then waits until
// every other thread has taken its task, so no thread takes two.
struct PoolStart {
  std::mutex mutex;
  std::condition_variable arrived_all;
  int arrived = 0;
  int threads = 0;
};

void EnterBackground(void* argument) {
  auto& start = *static_cast<PoolStart*>(argument);
  talos::process::ConfigureBackgroundThread();
  std::unique_lock lock{start.mutex};
  if (++start.arrived == start.threads) {
    start.arrived_all.notify_all();
    return;
  }
  start.arrived_all.wait(lock,
                         [&start] { return start.arrived == start.threads; });
}

}  // namespace

void EnsureBackgroundWorkerPool(apriltag_detector_t* td) {
  if (td == nullptr || td->nthreads < 1) {
    return;
  }
  if (td->wp != nullptr && workerpool_get_nthreads(td->wp) == td->nthreads) {
    return;
  }
  workerpool_destroy(td->wp);
  // On failure the detection that follows retries and reports it.
  td->wp = workerpool_create(td->nthreads);
  // With one thread the pool runs its tasks on the caller.
  if (td->wp == nullptr || td->nthreads <= 1) return;
  PoolStart start;
  start.threads = td->nthreads;
  for (int i = 0; i < td->nthreads; ++i) {
    workerpool_add_task(td->wp, EnterBackground, &start);
  }
  workerpool_run(td->wp);
}

apriltag_family* CreateTagFamily(const std::string& name) {
  if (name == "tag36h11") return tag36h11_create();
  if (name == "tag16h5") return tag16h5_create();
  if (name == "tag25h9") return tag25h9_create();
  if (name == "tagCircle21h7") return tagCircle21h7_create();
  if (name == "tagCircle49h12") return tagCircle49h12_create();
  if (name == "tagCustom48h12") return tagCustom48h12_create();
  if (name == "tagStandard41h12") return tagStandard41h12_create();
  if (name == "tagStandard52h13") return tagStandard52h13_create();
  return nullptr;
}

void DestroyTagFamily(const std::string& name, apriltag_family* family) {
  if (family == nullptr) return;
  if (name == "tag36h11") {
    tag36h11_destroy(family);
  } else if (name == "tag16h5") {
    tag16h5_destroy(family);
  } else if (name == "tag25h9") {
    tag25h9_destroy(family);
  } else if (name == "tagCircle21h7") {
    tagCircle21h7_destroy(family);
  } else if (name == "tagCircle49h12") {
    tagCircle49h12_destroy(family);
  } else if (name == "tagCustom48h12") {
    tagCustom48h12_destroy(family);
  } else if (name == "tagStandard41h12") {
    tagStandard41h12_destroy(family);
  } else if (name == "tagStandard52h13") {
    tagStandard52h13_destroy(family);
  }
}

image_u8* LoadGrayscaleImageFile(const std::string& filepath) {
  image_u8_t* im = nullptr;

  if (filepath.ends_with(".jpg") || filepath.ends_with(".jpeg") ||
      filepath.ends_with(".JPG") || filepath.ends_with(".JPEG")) {
    pjpeg_t* pjpeg = pjpeg_create_from_file(filepath.c_str(), 0, nullptr);
    if (pjpeg != nullptr) {
      im = pjpeg_to_u8_baseline(pjpeg);
      pjpeg_destroy(pjpeg);
    }
  } else {
    // PNG and anything else: OpenCV grayscale decode (replaces the
    // upstream PNM loader; fixtures are PNG).
    const cv::Mat gray = cv::imread(filepath, cv::IMREAD_GRAYSCALE);
    if (!gray.empty() && gray.cols > 0 && gray.rows > 0) {
      im = image_u8_create(static_cast<uint32_t>(gray.cols),
                           static_cast<uint32_t>(gray.rows));
      if (im == nullptr) {
        return nullptr;
      }
      if (im->buf == nullptr) {
        image_u8_destroy(im);
        return nullptr;
      }
      for (int y = 0; y < gray.rows; ++y) {
        std::memcpy(im->buf + static_cast<size_t>(y) * im->stride,
                    gray.ptr<uint8_t>(y), static_cast<size_t>(gray.cols));
      }
    }
  }

  return im;
}

std::vector<Detection> ExtractDetections(zarray* raw_detections,
                                         const CpuAprilTagBackend& pose_backend,
                                         const CameraIntrinsics* intrinsics,
                                         double tag_size_meters) {
  std::vector<Detection> results;
  if (raw_detections == nullptr) {
    return results;
  }
  const int count = zarray_size(raw_detections);
  results.reserve(count);

  for (int i = 0; i < count; ++i) {
    apriltag_detection_t* raw_det = nullptr;
    zarray_get(raw_detections, i, reinterpret_cast<void*>(&raw_det));
    if (raw_det == nullptr) continue;

    Detection det;
    det.id = raw_det->id;
    det.hamming = raw_det->hamming;
    det.decision_margin = raw_det->decision_margin;
    det.center = Point2D{raw_det->c[0], raw_det->c[1]};

    for (int j = 0; j < 4; ++j) {
      det.corners[j] = Point2D{raw_det->p[j][0], raw_det->p[j][1]};
    }

    if (raw_det->H != nullptr) {
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          det.homography(r, c) = MATD_EL(raw_det->H, r, c);
        }
      }
    }

    if (intrinsics != nullptr && tag_size_meters > 0.0) {
      det.pose = pose_backend.EstimatePose(det, *intrinsics, tag_size_meters);
    }

    results.push_back(std::move(det));
  }

  apriltag_detections_destroy(raw_detections);
  return results;
}

struct CpuAprilTagBackend::Impl {
  apriltag_detector_t* td = nullptr;
  apriltag_family_t* tf = nullptr;
  std::string active_family;

  ~Impl() { Destroy(); }

  void Destroy() {
    if (td != nullptr) {
      apriltag_detector_destroy(td);
      td = nullptr;
    }
    if (tf != nullptr) {
      DestroyTagFamily(active_family, tf);
      tf = nullptr;
    }
    active_family.clear();
  }

  void Initialize(const DetectorOptions& options) {
    if (options.threads < 1) {
      throw std::invalid_argument("AprilTag threads must be >= 1");
    }
    if (!std::isfinite(options.quad_decimate)) {
      throw std::invalid_argument("AprilTag quad_decimate must be finite");
    }
    std::string new_family = options.family_name;
    apriltag_family_t* new_tf = CreateTagFamily(new_family);
    if (new_tf == nullptr) {
      throw std::invalid_argument("Unsupported AprilTag family: " +
                                  options.family_name);
    }
    apriltag_detector_t* new_td = apriltag_detector_create();
    if (new_td == nullptr) {
      DestroyTagFamily(new_family, new_tf);
      throw std::runtime_error("Failed to create AprilTag detector");
    }
    apriltag_detector_add_family_bits(new_td, new_tf, options.max_hamming);

    new_td->nthreads = options.threads;
    new_td->quad_decimate = options.quad_decimate;
    new_td->quad_sigma = options.quad_sigma;
    new_td->refine_edges = options.refine_edges ? 1 : 0;
    new_td->decode_sharpening = options.decode_sharpening;

    // Commit atomically so a failure never leaves td/tf half-torn-down.
    Destroy();
    td = new_td;
    tf = new_tf;
    active_family = std::move(new_family);
  }
};

CpuAprilTagBackend::CpuAprilTagBackend()
    : impl_(std::make_unique<Impl>()), options_() {
  impl_->Initialize(options_);
}

CpuAprilTagBackend::CpuAprilTagBackend(const DetectorOptions& options)
    : impl_(std::make_unique<Impl>()), options_(options) {
  impl_->Initialize(options_);
}

CpuAprilTagBackend::~CpuAprilTagBackend() = default;

CpuAprilTagBackend::CpuAprilTagBackend(CpuAprilTagBackend&&) noexcept = default;
CpuAprilTagBackend& CpuAprilTagBackend::operator=(
    CpuAprilTagBackend&&) noexcept = default;

void CpuAprilTagBackend::SetOptions(const DetectorOptions& options) {
  DetectorOptions next = options;
  impl_->Initialize(next);
  options_ = std::move(next);
}

static bool ImageDimensionsSupported(const apriltag_detector_t* td, int width,
                                     int height) {
  if (td == nullptr || width < 4 || height < 4 || width >= 32768 ||
      height >= 32768) {
    return false;
  }
  if (!std::isfinite(td->quad_decimate)) {
    return false;
  }
  int quad_w = width;
  int quad_h = height;
  if (td->quad_decimate > 1.0f) {
    if (td->quad_decimate == 1.5f) {
      quad_w = (width / 3) * 2;
      quad_h = (height / 3) * 2;
    } else {
      if (td->quad_decimate >= 32768.0f) {
        return false;
      }
      const int factor = static_cast<int>(td->quad_decimate);
      if (factor > 1) {
        quad_w = 1 + (width - 1) / factor;
        quad_h = 1 + (height - 1) / factor;
      }
    }
  }
  // apriltag_quad_thresh's threshold() divides dimensions by tilesz=4 and
  // indexes im_max[(th - 1) * tw + ...], overflowing heap buffers if < 4.
  return quad_w >= 4 && quad_h >= 4 && quad_w < 32768 && quad_h < 32768;
}

static std::vector<Detection> ProcessDetections(
    const CpuAprilTagBackend& backend, apriltag_detector_t* td, image_u8_t* im,
    const CameraIntrinsics* intrinsics = nullptr,
    double tag_size_meters = 0.0) {
  if (td == nullptr || im == nullptr ||
      !ImageDimensionsSupported(td, im->width, im->height)) {
    return {};
  }
  EnsureBackgroundWorkerPool(td);
  zarray_t* raw_detections = apriltag_detector_detect(td, im);
  return ExtractDetections(raw_detections, backend, intrinsics,
                           tag_size_meters);
}

std::vector<Detection> CpuAprilTagBackend::Detect(const uint8_t* gray_data,
                                                  int width, int height,
                                                  int stride) {
  return Detect(gray_data, width, height, stride, CameraIntrinsics{}, 0.0);
}

std::vector<Detection> CpuAprilTagBackend::Detect(
    const uint8_t* gray_data, int width, int height, int stride,
    const CameraIntrinsics& intrinsics, double tag_size_meters) {
  if (gray_data == nullptr || width <= 0 || height <= 0) {
    return {};
  }
  if (stride < 0 || (stride > 0 && stride < width)) {
    return {};
  }
  if (impl_ == nullptr || impl_->td == nullptr ||
      !ImageDimensionsSupported(impl_->td, width, height)) {
    return {};
  }
  const int actual_stride = (stride > 0) ? stride : width;

  image_u8_t* im = nullptr;
  bool owns_image = false;
  image_u8_t stack_im = {
      .width = width,
      .height = height,
      .stride = actual_stride,
      .buf = const_cast<uint8_t*>(gray_data),
  };

  // AprilTag's internal thresholding requires stride to match its aligned
  // layout. Furthermore, when decimation <= 1 and quad_sigma != 0, apriltag
  // blurs (> 0) or sharpens (< 0) in-place. If stride != width or in-place
  // filtering would occur, copy into a clean image_u8 buffer to guarantee
  // safety and buffer immutability.
  if (actual_stride != width ||
      (options_.quad_sigma != 0.0f && options_.quad_decimate <= 1.0f)) {
    im = image_u8_create(width, height);
    if (im == nullptr) {
      return {};
    }
    if (im->buf == nullptr) {
      image_u8_destroy(im);
      return {};
    }
    for (int y = 0; y < height; ++y) {
      std::memcpy(&im->buf[static_cast<size_t>(y) * im->stride],
                  &gray_data[static_cast<size_t>(y) * actual_stride], width);
    }
    owns_image = true;
  } else {
    // Zero-copy wrapper over contiguous user memory
    im = &stack_im;
  }

  const CameraIntrinsics* p_intrinsics =
      (tag_size_meters > 0.0) ? &intrinsics : nullptr;
  std::vector<Detection> results =
      ProcessDetections(*this, impl_->td, im, p_intrinsics, tag_size_meters);

  if (owns_image) {
    image_u8_destroy(im);
  }

  return results;
}

std::vector<Detection> CpuAprilTagBackend::DetectImageFile(
    const std::string& filepath) {
  return DetectImageFile(filepath, CameraIntrinsics{}, 0.0);
}

std::vector<Detection> CpuAprilTagBackend::DetectImageFile(
    const std::string& filepath, const CameraIntrinsics& intrinsics,
    double tag_size_meters) {
  if (impl_ == nullptr || impl_->td == nullptr) {
    return {};
  }
  image_u8_t* im = LoadGrayscaleImageFile(filepath);
  if (im == nullptr) {
    return {};
  }

  const CameraIntrinsics* p_intrinsics =
      (tag_size_meters > 0.0) ? &intrinsics : nullptr;
  std::vector<Detection> results =
      ProcessDetections(*this, impl_->td, im, p_intrinsics, tag_size_meters);
  image_u8_destroy(im);
  return results;
}

std::optional<PoseEstimation> CpuAprilTagBackend::EstimatePose(
    const Detection& detection, const CameraIntrinsics& intrinsics,
    double tag_size_meters) const {
  std::array<float, 9> homography;
  std::array<float, 8> corners;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      homography[3 * static_cast<size_t>(r) + c] =
          static_cast<float>(detection.homography(r, c));
    }
  }
  for (int i = 0; i < 4; ++i) {
    corners[2 * static_cast<size_t>(i)] =
        static_cast<float>(detection.corners[i].x);
    corners[2 * static_cast<size_t>(i) + 1] =
        static_cast<float>(detection.corners[i].y);
  }
  PoseEstimation result;
  double ambiguity = 0.0;
  if (!talos::vision::EstimateTagPoseFromHomography(
          homography, intrinsics, tag_size_meters, &result.pose,
          &result.alternative_pose, &result.error, &result.alternative_error,
          &ambiguity, &corners)) {
    return std::nullopt;
  }
  result.rotation = result.pose.rotation().rotationMatrix();
  result.translation = result.pose.translation();
  result.has_alternative = std::isfinite(result.alternative_error);
  if (result.has_alternative) {
    result.alternative_rotation =
        result.alternative_pose.rotation().rotationMatrix();
    result.alternative_translation = result.alternative_pose.translation();
  }
  return result;
}

apriltag_detector* CpuAprilTagBackend::detector_for_testing() const {
  return impl_ != nullptr ? impl_->td : nullptr;
}

}  // namespace talos::vision::apriltag
