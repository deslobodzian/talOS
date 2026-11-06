#include "talOS/vision/apriltag/cpu_apriltag_backend.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "talOS/vision/localization.h"

extern "C" {
#include <apriltag.h>
#include <apriltag_pose.h>
#include <common/image_u8.h>
#include <common/matd.h>
#include <common/pjpeg.h>
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

// The C solver reports poses in its own tag frame; everything downstream
// uses the WPILib/layout frame (+X out of the face, +Z up). Keep the
// struct's redundant members consistent with the converted pose.
void ConvertToLayoutTagFrame(PoseEstimation* estimation) {
  if (estimation == nullptr) return;
  estimation->pose = SolverTagFrameToLayoutFrame(estimation->pose);
  estimation->rotation = estimation->pose.rotation().rotationMatrix();
  estimation->translation = estimation->pose.translation();
  if (estimation->has_alternative) {
    estimation->alternative_pose =
        SolverTagFrameToLayoutFrame(estimation->alternative_pose);
    estimation->alternative_rotation =
        estimation->alternative_pose.rotation().rotationMatrix();
    estimation->alternative_translation =
        estimation->alternative_pose.translation();
  }
}

}  // namespace

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
      DestroyFamily(active_family, tf);
      tf = nullptr;
    }
    active_family.clear();
  }

  static apriltag_family_t* CreateFamily(const std::string& name) {
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

  static void DestroyFamily(const std::string& name, apriltag_family_t* fam) {
    if (fam == nullptr) return;
    if (name == "tag36h11") {
      tag36h11_destroy(fam);
    } else if (name == "tag16h5") {
      tag16h5_destroy(fam);
    } else if (name == "tag25h9") {
      tag25h9_destroy(fam);
    } else if (name == "tagCircle21h7") {
      tagCircle21h7_destroy(fam);
    } else if (name == "tagCircle49h12") {
      tagCircle49h12_destroy(fam);
    } else if (name == "tagCustom48h12") {
      tagCustom48h12_destroy(fam);
    } else if (name == "tagStandard41h12") {
      tagStandard41h12_destroy(fam);
    } else if (name == "tagStandard52h13") {
      tagStandard52h13_destroy(fam);
    }
  }

  void Initialize(const DetectorOptions& options) {
    Destroy();
    td = apriltag_detector_create();
    tf = CreateFamily(options.family_name);
    if (tf == nullptr) {
      throw std::invalid_argument("Unsupported AprilTag family: " +
                                  options.family_name);
    }
    active_family = options.family_name;
    apriltag_detector_add_family_bits(td, tf, options.max_hamming);

    td->nthreads = options.threads;
    td->quad_decimate = options.quad_decimate;
    td->quad_sigma = options.quad_sigma;
    td->refine_edges = options.refine_edges ? 1 : 0;
    td->decode_sharpening = options.decode_sharpening;
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
  options_ = options;
  impl_->Initialize(options_);
}

static std::vector<Detection> ProcessDetections(
    const CpuAprilTagBackend& backend, apriltag_detector_t* td, image_u8_t* im,
    const CameraIntrinsics* intrinsics = nullptr,
    double tag_size_meters = 0.0) {
  zarray_t* raw_detections = apriltag_detector_detect(td, im);
  std::vector<Detection> results;
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
      det.pose = backend.EstimatePose(det, *intrinsics, tag_size_meters);
    }

    results.push_back(det);
  }

  apriltag_detections_destroy(raw_detections);
  return results;
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
  // layout. Furthermore, when decimation <= 1 and quad_sigma > 0, apriltag
  // blurs in-place. If stride != width or in-place blur would occur, copy into
  // a clean image_u8 buffer to guarantee safety and buffer immutability.
  if (actual_stride != width ||
      (options_.quad_sigma > 0.0f && options_.quad_decimate <= 1.0f)) {
    im = image_u8_create(width, height);
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
      for (int y = 0; y < gray.rows; ++y) {
        std::memcpy(im->buf + static_cast<size_t>(y) * im->stride,
                    gray.ptr<uint8_t>(y), static_cast<size_t>(gray.cols));
      }
    }
  }

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

PoseEstimation CpuAprilTagBackend::EstimatePose(
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
    return result;
  }
  result.rotation = result.pose.rotation().rotationMatrix();
  result.translation = result.pose.translation();
  result.has_alternative = ambiguity > 0.0;
  result.alternative_rotation =
      result.alternative_pose.rotation().rotationMatrix();
  result.alternative_translation = result.alternative_pose.translation();
  return result;
}

PoseEstimation CpuAprilTagBackend::EstimatePoseHomography(
    const Detection& detection, const CameraIntrinsics& intrinsics,
    double tag_size_meters) const {
  apriltag_detection_t det;
  det.H = matd_create(3, 3);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      MATD_EL(det.H, r, c) = detection.homography(r, c);
    }
  }
  for (int j = 0; j < 4; ++j) {
    det.p[j][0] = detection.corners[j].x;
    det.p[j][1] = detection.corners[j].y;
  }

  apriltag_detection_info_t info;
  info.det = &det;
  info.tagsize = tag_size_meters;
  info.fx = intrinsics.fx;
  info.fy = intrinsics.fy;
  info.cx = intrinsics.cx;
  info.cy = intrinsics.cy;

  apriltag_pose_t pose;
  estimate_pose_for_tag_homography(&info, &pose);

  PoseEstimation result;
  if (pose.R != nullptr && pose.t != nullptr) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        result.rotation(r, c) = MATD_EL(pose.R, r, c);
      }
    }
    result.translation = Eigen::Vector3d(
        MATD_EL(pose.t, 0, 0), MATD_EL(pose.t, 1, 0), MATD_EL(pose.t, 2, 0));
    result.pose = talos::geometry::SE3<double>(
        talos::geometry::SO3<double>(result.rotation), result.translation);
    matd_destroy(pose.R);
    matd_destroy(pose.t);
  }

  matd_destroy(det.H);
  ConvertToLayoutTagFrame(&result);
  return result;
}

PoseEstimation CpuAprilTagBackend::EstimatePoseOrthogonalIteration(
    const Detection& detection, const CameraIntrinsics& intrinsics,
    double tag_size_meters, int n_iters) const {
  apriltag_detection_t det;
  det.H = matd_create(3, 3);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      MATD_EL(det.H, r, c) = detection.homography(r, c);
    }
  }
  for (int j = 0; j < 4; ++j) {
    det.p[j][0] = detection.corners[j].x;
    det.p[j][1] = detection.corners[j].y;
  }

  apriltag_detection_info_t info;
  info.det = &det;
  info.tagsize = tag_size_meters;
  info.fx = intrinsics.fx;
  info.fy = intrinsics.fy;
  info.cx = intrinsics.cx;
  info.cy = intrinsics.cy;

  double err1 = 0.0;
  double err2 = 0.0;
  apriltag_pose_t pose1;
  apriltag_pose_t pose2;
  estimate_tag_pose_orthogonal_iteration(&info, &err1, &pose1, &err2, &pose2,
                                         n_iters);

  PoseEstimation result;
  result.error = err1;
  if (pose1.R != nullptr && pose1.t != nullptr) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        result.rotation(r, c) = MATD_EL(pose1.R, r, c);
      }
    }
    result.translation = Eigen::Vector3d(
        MATD_EL(pose1.t, 0, 0), MATD_EL(pose1.t, 1, 0), MATD_EL(pose1.t, 2, 0));
    result.pose = talos::geometry::SE3<double>(
        talos::geometry::SO3<double>(result.rotation), result.translation);
    matd_destroy(pose1.R);
    matd_destroy(pose1.t);
  }

  if (pose2.R != nullptr && pose2.t != nullptr) {
    result.has_alternative = true;
    result.alternative_error = err2;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        result.alternative_rotation(r, c) = MATD_EL(pose2.R, r, c);
      }
    }
    result.alternative_translation = Eigen::Vector3d(
        MATD_EL(pose2.t, 0, 0), MATD_EL(pose2.t, 1, 0), MATD_EL(pose2.t, 2, 0));
    result.alternative_pose = talos::geometry::SE3<double>(
        talos::geometry::SO3<double>(result.alternative_rotation),
        result.alternative_translation);
    matd_destroy(pose2.R);
    matd_destroy(pose2.t);
  }

  matd_destroy(det.H);
  ConvertToLayoutTagFrame(&result);
  return result;
}

}  // namespace talos::vision::apriltag
