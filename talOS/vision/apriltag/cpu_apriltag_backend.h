#pragma once

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "talOS/geometry/se3.h"
#include "talOS/geometry/so3.h"
#include "talOS/vision/calibration.h"

namespace talos::vision::apriltag {

using CameraIntrinsics = talos::vision::CameraIntrinsics;

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

struct PoseEstimation {
  // Rigid transform mapping coordinates from the WPILib/layout tag frame
  // (+X out of the tag face, +Z up) to the camera optical frame:
  // p_camera = pose.act(p_tag) = rotation * p_tag + translation
  talos::geometry::SE3<double> pose;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  double error = 0.0;

  // Secondary local minimum candidate from Orthogonal Iteration for ambiguity
  // resolution
  bool has_alternative = false;
  talos::geometry::SE3<double> alternative_pose;
  Eigen::Matrix3d alternative_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d alternative_translation = Eigen::Vector3d::Zero();
  double alternative_error = 0.0;
};

struct Detection {
  int id = -1;
  int hamming = 0;
  float decision_margin = 0.0f;
  Point2D center;
  // Corners in image pixel coordinates, ordered counter-clockwise:
  // p[0]: bottom-left
  // p[1]: bottom-right
  // p[2]: top-right
  // p[3]: top-left
  std::array<Point2D, 4> corners;
  // 3x3 homography matrix mapping canonical tag space to image pixel space
  Eigen::Matrix3d homography = Eigen::Matrix3d::Identity();

  // 3D pose in camera frame (populated when intrinsics and tag size are
  // supplied)
  std::optional<PoseEstimation> pose;
};

struct DetectorOptions {
  std::string family_name = "tag36h11";
  int threads = 1;
  float quad_decimate = 2.0f;
  float quad_sigma = 0.0f;
  bool refine_edges = true;
  double decode_sharpening = 0.25;
  int max_hamming = 1;
};

class CpuAprilTagBackend {
 public:
  CpuAprilTagBackend();
  explicit CpuAprilTagBackend(const DetectorOptions& options);
  ~CpuAprilTagBackend();

  CpuAprilTagBackend(const CpuAprilTagBackend&) = delete;
  CpuAprilTagBackend& operator=(const CpuAprilTagBackend&) = delete;

  CpuAprilTagBackend(CpuAprilTagBackend&&) noexcept;
  CpuAprilTagBackend& operator=(CpuAprilTagBackend&&) noexcept;

  void SetOptions(const DetectorOptions& options);
  const DetectorOptions& options() const { return options_; }

  // Detect tags in raw grayscale image buffer (uint8_t, 1 byte per pixel).
  // If stride is 0 or negative, stride is assumed to equal width.
  std::vector<Detection> Detect(const uint8_t* gray_data, int width, int height,
                                int stride = 0);

  // Detect tags and estimate 3D pose using provided camera intrinsics and tag
  // size.
  std::vector<Detection> Detect(const uint8_t* gray_data, int width, int height,
                                int stride, const CameraIntrinsics& intrinsics,
                                double tag_size_meters);

  // Detect tags from an image file (supports JPEG and PNM/PGM/PPM).
  std::vector<Detection> DetectImageFile(const std::string& filepath);
  std::vector<Detection> DetectImageFile(const std::string& filepath,
                                         const CameraIntrinsics& intrinsics,
                                         double tag_size_meters);

  // 3D pose estimation from an existing detection.
  PoseEstimation EstimatePose(const Detection& detection,
                              const CameraIntrinsics& intrinsics,
                              double tag_size_meters) const;

  PoseEstimation EstimatePoseHomography(const Detection& detection,
                                        const CameraIntrinsics& intrinsics,
                                        double tag_size_meters) const;

  PoseEstimation EstimatePoseOrthogonalIteration(
      const Detection& detection, const CameraIntrinsics& intrinsics,
      double tag_size_meters, int n_iters = 50) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  DetectorOptions options_;
};

}  // namespace talos::vision::apriltag
