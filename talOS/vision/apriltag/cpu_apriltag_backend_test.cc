#include "talOS/vision/apriltag/cpu_apriltag_backend.h"

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "talOS/geometry/se3.h"
#include "talOS/process/process.h"
#include "talOS/vision/apriltag/test_image_loader.h"

extern "C" {
#include <apriltag.h>
#include <common/workerpool.h>
}

namespace talos::vision::apriltag {
namespace {

std::string GetTestdataPath(const std::string& filename) {
  const std::vector<std::string> candidates = {
      "talOS/vision/testdata/" + filename,
      "_main/talOS/vision/testdata/" + filename,
  };
  const char* srcdir = std::getenv("TEST_SRCDIR");
  if (srcdir != nullptr) {
    const char* workspace = std::getenv("TEST_WORKSPACE");
    const std::string ws = (workspace != nullptr) ? workspace : "_main";
    std::string p =
        std::string(srcdir) + "/" + ws + "/talOS/vision/testdata/" + filename;
    if (std::filesystem::exists(p)) return p;
    std::string p2 = std::string(srcdir) + "/talOS/vision/testdata/" + filename;
    if (std::filesystem::exists(p2)) return p2;
  }
  for (const auto& path : candidates) {
    if (std::filesystem::exists(path)) return path;
  }
  return "talOS/vision/testdata/" + filename;
}

struct ExpectedDetection {
  int id = 0;
  std::array<Point2D, 4> corners;
};

std::vector<ExpectedDetection> LoadExpectedFile(const std::string& filepath) {
  std::vector<ExpectedDetection> expected;
  std::ifstream infile(filepath);
  if (!infile.is_open()) return expected;

  std::string line;
  while (std::getline(infile, line)) {
    if (line.empty()) continue;
    ExpectedDetection det;
    const int parsed = std::sscanf(
        line.c_str(), "%i, (%lf %lf), (%lf %lf), (%lf %lf), (%lf %lf)", &det.id,
        &det.corners[0].x, &det.corners[0].y, &det.corners[1].x,
        &det.corners[1].y, &det.corners[2].x, &det.corners[2].y,
        &det.corners[3].x, &det.corners[3].y);
    if (parsed == 9) {
      expected.push_back(det);
    }
  }
  return expected;
}

bool DetectionComparator(const Detection& a, const Detection& b) {
  if (a.id != b.id) return a.id < b.id;
  for (int e = 0; e < 4; ++e) {
    if (std::abs(a.corners[e].x - b.corners[e].x) > 1e-1) {
      return a.corners[e].x < b.corners[e].x;
    }
    if (std::abs(a.corners[e].y - b.corners[e].y) > 1e-1) {
      return a.corners[e].y < b.corners[e].y;
    }
  }
  return false;
}

bool ExpectedComparator(const ExpectedDetection& a,
                        const ExpectedDetection& b) {
  if (a.id != b.id) return a.id < b.id;
  for (int e = 0; e < 4; ++e) {
    if (std::abs(a.corners[e].x - b.corners[e].x) > 1e-1) {
      return a.corners[e].x < b.corners[e].x;
    }
    if (std::abs(a.corners[e].y - b.corners[e].y) > 1e-1) {
      return a.corners[e].y < b.corners[e].y;
    }
  }
  return false;
}

void VerifyPhotographicFixture(const std::string& img_filename,
                               const std::string& txt_filename,
                               int expected_count) {
  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;
  options.refine_edges = false;
  options.threads = 1;

  CpuAprilTagBackend backend(options);

  const std::string img_path = GetTestdataPath(img_filename);
  const std::string txt_path = GetTestdataPath(txt_filename);

  ASSERT_TRUE(std::filesystem::exists(img_path)) << "Missing: " << img_path;
  ASSERT_TRUE(std::filesystem::exists(txt_path)) << "Missing: " << txt_path;

  const std::vector<ExpectedDetection> expected = LoadExpectedFile(txt_path);
  ASSERT_EQ(static_cast<int>(expected.size()), expected_count);

  const CameraIntrinsics intrinsics{
      .fx = 600.0, .fy = 600.0, .cx = 400.0, .cy = 300.0};
  constexpr double kTagSizeMeters = 0.16;

  std::vector<Detection> detections =
      backend.DetectImageFile(img_path, intrinsics, kTagSizeMeters);
  ASSERT_EQ(static_cast<int>(detections.size()), expected_count);

  std::vector<ExpectedDetection> sorted_expected = expected;
  std::sort(sorted_expected.begin(), sorted_expected.end(), ExpectedComparator);
  std::sort(detections.begin(), detections.end(), DetectionComparator);

  for (size_t i = 0; i < detections.size(); ++i) {
    const auto& det = detections[i];
    const auto& exp = sorted_expected[i];

    EXPECT_EQ(det.id, exp.id);
    EXPECT_LE(det.hamming, 2);
    EXPECT_GT(det.decision_margin, 0.0f);

    // Assert corner Euclidean distance <= 0.1 pixel (per PLAN.md acceptance
    // requirement)
    for (int c = 0; c < 4; ++c) {
      const double dx = det.corners[c].x - exp.corners[c].x;
      const double dy = det.corners[c].y - exp.corners[c].y;
      const double dist = std::hypot(dx, dy);
      EXPECT_LE(dist, 0.1) << "Corner " << c << " mismatch on tag " << det.id
                           << ": got (" << det.corners[c].x << ", "
                           << det.corners[c].y << ") vs expected ("
                           << exp.corners[c].x << ", " << exp.corners[c].y
                           << ")";
    }

    // Verify center lies within bounding quad
    double avg_x = 0.0;
    double avg_y = 0.0;
    for (int c = 0; c < 4; ++c) {
      avg_x += det.corners[c].x;
      avg_y += det.corners[c].y;
    }
    avg_x /= 4.0;
    avg_y /= 4.0;
    EXPECT_NEAR(det.center.x, avg_x, 5.0);
    EXPECT_NEAR(det.center.y, avg_y, 5.0);

    // Verify homography is non-singular
    EXPECT_NE(det.homography.determinant(), 0.0);

    // Verify 3D pose estimation
    if (!det.pose.has_value()) {
      FAIL() << "Missing detected tag pose";
    }
    const auto& pose_est = *det.pose;
    EXPECT_GT(pose_est.translation.z(), 0.0)
        << "Tag must be in front of camera";

    // Rotation matrix must be orthonormal: R * R^T == I
    const Eigen::Matrix3d r_rt =
        pose_est.rotation * pose_est.rotation.transpose();
    EXPECT_TRUE(r_rt.isApprox(Eigen::Matrix3d::Identity(), 1e-4));
  }
}

}  // namespace

TEST(CpuAprilTagBackendTest, PhotographicFixtures) {
  struct Fixture {
    const char* stem;
    int count;
  };
  for (const auto& fixture : {Fixture{"33369213973_9d9bb4cc96_c", 12},
                              Fixture{"34085369442_304b6bafd9_c", 25},
                              Fixture{"34139872896_defdb2f8d9_c", 10}}) {
    SCOPED_TRACE(fixture.stem);
    VerifyPhotographicFixture(std::string(fixture.stem) + ".jpg",
                              std::string(fixture.stem) + ".txt",
                              fixture.count);
  }
}

TEST(CpuAprilTagBackendTest, SyntheticSingleTagScene) {
  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;
  options.refine_edges = true;

  CpuAprilTagBackend backend(options);
  const std::string path = GetTestdataPath("scene_single_tag0.png");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Missing: " << path;

  const CameraIntrinsics intrinsics{
      .fx = 500.0, .fy = 500.0, .cx = 320.0, .cy = 240.0};
  constexpr double kTagSizeMeters = 0.16;

  std::vector<Detection> detections =
      backend.DetectImageFile(path, intrinsics, kTagSizeMeters);
  ASSERT_EQ(detections.size(), 1u);

  const auto& det = detections[0];
  EXPECT_EQ(det.id, 0);
  EXPECT_EQ(det.hamming, 0);
  EXPECT_GT(det.decision_margin, 50.0f);

  // In scene_single_tag0, tag 0 is 160x160 centered in 640x480 at (240, 160)
  EXPECT_NEAR(det.center.x, 320.0, 2.0);
  EXPECT_NEAR(det.center.y, 240.0, 2.0);

  if (!det.pose.has_value()) {
    FAIL() << "Missing detected tag pose";
  }
  const auto& pose = *det.pose;
  EXPECT_GT(pose.translation.z(), 0.3);
  // Centered tag means translation in x, y is close to 0
  EXPECT_NEAR(pose.translation.x(), 0.0, 0.05);
  EXPECT_NEAR(pose.translation.y(), 0.0, 0.05);
}

TEST(CpuAprilTagBackendTest, SyntheticMultiTagScene) {
  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;
  options.refine_edges = true;

  CpuAprilTagBackend backend(options);
  const std::string path = GetTestdataPath("scene_multi_tag.png");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Missing: " << path;

  std::vector<Detection> detections = backend.DetectImageFile(path);
  ASSERT_EQ(detections.size(), 4u);

  std::vector<int> detected_ids;
  for (const auto& det : detections) {
    detected_ids.push_back(det.id);
    EXPECT_EQ(det.hamming, 0);
    EXPECT_GT(det.decision_margin, 30.0f);
  }
  std::sort(detected_ids.begin(), detected_ids.end());
  EXPECT_EQ(detected_ids, (std::vector<int>{0, 1, 2, 3}));
}

TEST(CpuAprilTagBackendTest, SyntheticRotatedTagScene) {
  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;
  options.refine_edges = true;

  CpuAprilTagBackend backend(options);
  const std::string path = GetTestdataPath("scene_rotated_tag.png");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Missing: " << path;

  std::vector<Detection> detections = backend.DetectImageFile(path);
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].id, 4);
  EXPECT_EQ(detections[0].hamming, 0);
  EXPECT_GT(detections[0].decision_margin, 20.0f);
}

TEST(CpuAprilTagBackendTest, SyntheticNonAlignedDimensionsScene) {
  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;
  options.refine_edges = true;

  CpuAprilTagBackend backend(options);
  const std::string path = GetTestdataPath("scene_non_aligned.png");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Missing: " << path;

  std::vector<Detection> detections = backend.DetectImageFile(path);
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].id, 0);
  EXPECT_EQ(detections[0].hamming, 0);
}

TEST(CpuAprilTagBackendTest, SyntheticEmptyScene) {
  DetectorOptions options;
  options.family_name = "tag36h11";

  CpuAprilTagBackend backend(options);
  const std::string path = GetTestdataPath("scene_empty.png");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Missing: " << path;

  std::vector<Detection> detections = backend.DetectImageFile(path);
  EXPECT_TRUE(detections.empty())
      << "Empty scene must produce zero false positives";
}

TEST(CpuAprilTagBackendTest, RawGrayscaleBufferWithPaddedStride) {
  const std::string src_path = GetTestdataPath("scene_single_tag0.png");
  CpuAprilTagBackend file_backend;
  std::vector<Detection> file_detections =
      file_backend.DetectImageFile(src_path);
  ASSERT_EQ(file_detections.size(), 1u);

  // Load the fixture into memory to create a padded buffer.
  image_u8_t* src_im = LoadGrayImageU8(src_path);
  ASSERT_NE(src_im, nullptr);
  const int w = src_im->width;
  const int h = src_im->height;
  std::vector<uint8_t> raw(static_cast<size_t>(w) * h);
  for (int y = 0; y < h; ++y) {
    std::memcpy(&raw[static_cast<size_t>(y) * w],
                src_im->buf + static_cast<size_t>(y) * src_im->stride,
                static_cast<size_t>(w));
  }
  image_u8_destroy(src_im);

  // Create padded buffer with stride = w + 32
  const int padded_stride = w + 32;
  std::vector<uint8_t> padded(static_cast<size_t>(padded_stride) * h, 0);
  for (int y = 0; y < h; ++y) {
    std::memcpy(&padded[static_cast<size_t>(y) * padded_stride],
                &raw[static_cast<size_t>(y) * w], w);
  }

  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;
  CpuAprilTagBackend padded_backend(options);

  std::vector<Detection> detections =
      padded_backend.Detect(padded.data(), w, h, padded_stride);
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].id, 0);
  EXPECT_EQ(detections[0].hamming, 0);
  EXPECT_NEAR(detections[0].center.x, file_detections[0].center.x, 0.01);
  EXPECT_NEAR(detections[0].center.y, file_detections[0].center.y, 0.01);
}

TEST(CpuAprilTagBackendTest, ProductionPoseReprojectsDetectedCorners) {
  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;

  CpuAprilTagBackend backend(options);
  const std::string path = GetTestdataPath("scene_single_tag0.png");
  std::vector<Detection> detections = backend.DetectImageFile(path);
  ASSERT_EQ(detections.size(), 1u);
  const auto& det = detections[0];

  const CameraIntrinsics intrinsics{
      .fx = 500.0, .fy = 500.0, .cx = 320.0, .cy = 240.0};
  constexpr double kTagSizeMeters = 0.16;

  const std::optional<PoseEstimation> pose_opt =
      backend.EstimatePose(det, intrinsics, kTagSizeMeters);
  ASSERT_TRUE(pose_opt.has_value());
  const PoseEstimation& pose_std = *pose_opt;
  EXPECT_GT(pose_std.translation.z(), 0.3);

  // Verify Reprojection of 3D tag corners back to image plane
  const double s = kTagSizeMeters / 2.0;
  // Canonical tag corners in the WPILib/layout tag frame (+X out of the
  // face, +Z up), detector index order.
  const std::vector<Eigen::Vector3d> tag_corners = {
      {0.0, -s, -s}, {0.0, s, -s}, {0.0, s, s}, {0.0, -s, s}};

  for (size_t i = 0; i < 4; ++i) {
    const Eigen::Vector3d p_cam = pose_std.pose.act(tag_corners[i]);
    const double u = intrinsics.fx * (p_cam.x() / p_cam.z()) + intrinsics.cx;
    const double v = intrinsics.fy * (p_cam.y() / p_cam.z()) + intrinsics.cy;

    // Reprojected corner must match one of the detected corners within 2.0
    // pixels
    double min_dist = 1e9;
    for (int j = 0; j < 4; ++j) {
      const double dist =
          std::hypot(u - det.corners[j].x, v - det.corners[j].y);
      if (dist < min_dist) min_dist = dist;
    }
    EXPECT_LT(min_dist, 2.0)
        << "Reprojected corner " << i << " failed to match detected corners";
  }
}

TEST(CpuAprilTagBackendTest, PoseUsesUndistortedCorners) {
  CameraIntrinsics intrinsics{.fx = 600.0,
                              .fy = 610.0,
                              .cx = 320.0,
                              .cy = 240.0,
                              .k1 = -0.25,
                              .k2 = 0.04,
                              .p1 = 0.002,
                              .p2 = -0.003};
  constexpr double size = 0.2;
  const auto rotation =
      talos::geometry::SO3<double>::Exp(Eigen::Vector3d(0.2, -0.3, 0.1));
  const Eigen::Vector3d translation(0.3, 0.1, 1.2);
  Detection detection;
  constexpr double xs[4] = {-1, 1, 1, -1};
  constexpr double ys[4] = {-1, -1, 1, 1};
  for (int i = 0; i < 4; ++i) {
    Eigen::Vector2d pixel;
    ASSERT_TRUE(intrinsics.ProjectPoint(
        rotation.act(Eigen::Vector3d(xs[i] * size / 2, ys[i] * size / 2, 0)) +
            translation,
        &pixel));
    detection.corners[i] = {pixel.x(), pixel.y()};
  }
  // Deliberately stale H: the calibrated solver must reconstruct it from
  // undistorted observed corners rather than trust a distorted homography.
  detection.homography.setIdentity();
  CpuAprilTagBackend backend;
  const auto estimate = backend.EstimatePose(detection, intrinsics, size);
  ASSERT_TRUE(estimate.has_value());
  EXPECT_LT((estimate->translation - translation).norm(), 1e-4);
  EXPECT_NEAR(estimate->rotation.determinant(), 1.0, 1e-10);
  EXPECT_LT((estimate->rotation.transpose() * estimate->rotation -
             Eigen::Matrix3d::Identity())
                .norm(),
            1e-10);
}

namespace {

// What each pool thread reports, one task per thread: a task waits until
// every thread holds one, so no thread runs two.
struct PoolProbe {
  std::mutex mutex;
  std::condition_variable arrived_all;
  int threads = 0;
  int arrived = 0;
  int background = 0;
};

void ProbeThread(void* argument) {
  auto& probe = *static_cast<PoolProbe*>(argument);
  const bool background = talos::process::ThreadIsBackground();
  std::unique_lock lock{probe.mutex};
  probe.background += background ? 1 : 0;
  if (++probe.arrived == probe.threads) {
    probe.arrived_all.notify_all();
    return;
  }
  probe.arrived_all.wait(lock,
                         [&probe] { return probe.arrived == probe.threads; });
}

}  // namespace

// The detector's pool threads (created by the library with default
// attributes, from a thread at ordinary priority) must run below the node
// loops, and the pool must be the size detection keeps, not one it replaces.
TEST(CpuAprilTagBackendTest, DetectorPoolThreadsRunAtBackgroundPriority) {
  apriltag_detector_t* td = apriltag_detector_create();
  td->nthreads = 3;
  EnsureBackgroundWorkerPool(td);
  ASSERT_NE(td->wp, nullptr);
  EXPECT_EQ(workerpool_get_nthreads(td->wp), td->nthreads);

  PoolProbe probe;
  probe.threads = td->nthreads;
  for (int i = 0; i < td->nthreads; ++i) {
    workerpool_add_task(td->wp, ProbeThread, &probe);
  }
  workerpool_run(td->wp);
  apriltag_detector_destroy(td);

  EXPECT_EQ(probe.arrived, 3);
  EXPECT_EQ(probe.background, 3);
}

TEST(CpuAprilTagBackendTest, RejectsPositiveStrideSmallerThanWidth) {
  const long raw_page = ::sysconf(_SC_PAGESIZE);
  ASSERT_GT(raw_page, 0);
  const size_t page_size = static_cast<size_t>(raw_page);
  void* mapping = ::mmap(nullptr, 2 * page_size, PROT_READ | PROT_WRITE,
                         MAP_PRIVATE | MAP_ANON, -1, 0);
  ASSERT_NE(mapping, MAP_FAILED);
  auto* base = static_cast<uint8_t*>(mapping);
  ASSERT_EQ(::mprotect(base + page_size, page_size, PROT_NONE), 0);

  constexpr int kWidth = 64;
  constexpr int kHeight = 4;
  constexpr int kStride = 32;
  const size_t valid_bytes = static_cast<size_t>(kStride) * kHeight;
  uint8_t* ptr = base + page_size - valid_bytes;
  std::memset(ptr, 128, valid_bytes);

  CpuAprilTagBackend backend;
  const std::vector<Detection> detections =
      backend.Detect(ptr, kWidth, kHeight, kStride);
  EXPECT_TRUE(detections.empty());

  ::munmap(mapping, 2 * page_size);
}

TEST(CpuAprilTagBackendTest, NegativeQuadSigmaDoesNotMutateCallerBuffer) {
  const std::string src_path = GetTestdataPath("scene_single_tag0.png");
  image_u8_t* src_im = LoadGrayImageU8(src_path);
  ASSERT_NE(src_im, nullptr);
  const int w = src_im->width;
  const int h = src_im->height;
  std::vector<uint8_t> raw(static_cast<size_t>(w) * h);
  for (int y = 0; y < h; ++y) {
    std::memcpy(&raw[static_cast<size_t>(y) * w],
                src_im->buf + static_cast<size_t>(y) * src_im->stride,
                static_cast<size_t>(w));
  }
  image_u8_destroy(src_im);
  const std::vector<uint8_t> original = raw;

  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;
  options.quad_sigma = -0.8f;
  options.threads = 1;
  CpuAprilTagBackend backend(options);

  const std::vector<Detection> detections = backend.Detect(raw.data(), w, h, w);
  EXPECT_EQ(raw, original);
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].id, 0);
}

TEST(CpuAprilTagBackendTest,
     RejectsImagesWhoseDecimatedDimensionsAreUnderFour) {
  const std::vector<uint8_t> small(std::size_t{64} * 8, 128);
  for (const float decimate : {1.0f, 1.5f, 2.0f}) {
    DetectorOptions options;
    options.family_name = "tag36h11";
    options.quad_decimate = decimate;
    options.threads = 1;
    CpuAprilTagBackend backend(options);
    EXPECT_TRUE(backend.Detect(small.data(), 64, 1, 64).empty());
    EXPECT_TRUE(backend.Detect(small.data(), 1, 64, 1).empty());
    EXPECT_TRUE(backend.Detect(small.data(), 3, 3, 3).empty());
    if (decimate > 1.0f) {
      EXPECT_TRUE(backend.Detect(small.data(), 5, 5, 5).empty());
    }
    if (decimate >= 2.0f) {
      EXPECT_TRUE(backend.Detect(small.data(), 6, 6, 6).empty());
    }
  }
}

TEST(CpuAprilTagBackendTest, RejectsOversizedDimensionsBeforeAllocating) {
  const std::vector<uint8_t> small(64, 128);
  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;
  options.threads = 1;
  CpuAprilTagBackend backend(options);

  // stride != width previously called image_u8_create(w, h) before
  // ImageDimensionsSupported, attempting a multi-GB allocation or 32-bit
  // overflow and reading past `small`.
  EXPECT_TRUE(backend.Detect(small.data(), 32768, 32768, 32769).empty());
  EXPECT_TRUE(backend.Detect(small.data(), 65536, 65536, 65537).empty());

  // Huge finite quad_decimate must not invoke float-to-int UB.
  options.quad_decimate = 1e20f;
  backend.SetOptions(options);
  EXPECT_TRUE(backend.Detect(small.data(), 8, 8, 8).empty());
}

TEST(CpuAprilTagBackendTest, ValidatesOptionsAndCommitsSetOptionsAtomically) {
  for (const int bad_threads : {0, -1}) {
    DetectorOptions opts;
    opts.threads = bad_threads;
    EXPECT_THROW(CpuAprilTagBackend{opts}, std::invalid_argument);
  }
  for (const float bad_decimate : {std::numeric_limits<float>::quiet_NaN(),
                                   std::numeric_limits<float>::infinity(),
                                   -std::numeric_limits<float>::infinity()}) {
    DetectorOptions opts;
    opts.quad_decimate = bad_decimate;
    EXPECT_THROW(CpuAprilTagBackend{opts}, std::invalid_argument);
  }

  DetectorOptions valid;
  valid.family_name = "tag36h11";
  valid.quad_decimate = 1.0f;
  valid.threads = 1;
  CpuAprilTagBackend backend(valid);

  // Failed SetOptions must leave options_ and the active detector intact.
  DetectorOptions bad_family = valid;
  bad_family.family_name = "no_such_family";
  bad_family.threads = 2;
  EXPECT_THROW(backend.SetOptions(bad_family), std::invalid_argument);
  EXPECT_EQ(backend.options().family_name, "tag36h11");
  EXPECT_EQ(backend.options().threads, 1);

  DetectorOptions bad_threads = valid;
  bad_threads.threads = 0;
  EXPECT_THROW(backend.SetOptions(bad_threads), std::invalid_argument);
  EXPECT_EQ(backend.options().threads, 1);

  const std::string path = GetTestdataPath("scene_single_tag0.png");
  const auto detections = backend.DetectImageFile(path);
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].id, 0);
}

TEST(CpuAprilTagBackendTest,
     PoseEstimationReturnsNulloptOnFailureAndSetsHasAlternativeFromError) {
  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;
  CpuAprilTagBackend backend(options);

  // Invalid intrinsics must leave det.pose unset instead of an identity pose.
  const std::string path = GetTestdataPath("scene_single_tag0.png");
  const CameraIntrinsics invalid_intrinsics{
      .fx = 0.0, .fy = 0.0, .cx = 320.0, .cy = 240.0};
  const auto dets = backend.DetectImageFile(path, invalid_intrinsics, 0.16);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_FALSE(dets[0].pose.has_value());

  // Direct EstimatePose rejects invalid intrinsics, non-positive tag size, and
  // degenerate collinear corners with std::nullopt.
  const CameraIntrinsics valid_intrinsics{
      .fx = 700.0, .fy = 700.0, .cx = 640.0, .cy = 360.0};
  EXPECT_FALSE(
      backend.EstimatePose(dets[0], invalid_intrinsics, 0.16).has_value());
  EXPECT_FALSE(
      backend.EstimatePose(dets[0], valid_intrinsics, 0.0).has_value());
  Detection collinear;
  collinear.corners = {Point2D{100.0, 100.0}, Point2D{200.0, 100.0},
                       Point2D{300.0, 100.0}, Point2D{400.0, 100.0}};
  EXPECT_FALSE(
      backend.EstimatePose(collinear, valid_intrinsics, 0.16).has_value());

  // Pinned failed-secondary-search corners (new_minima > 1 -> ambiguity == 1.0
  // while alternative_error == +Inf): has_alternative must be false.
  Detection missing_secondary;
  missing_secondary.corners = {
      Point2D{488.490112, 242.771851}, Point2D{497.506744, 250.814423},
      Point2D{504.358826, 243.886002}, Point2D{498.08548, 236.256561}};
  const auto missing_alt =
      backend.EstimatePose(missing_secondary, valid_intrinsics, 0.1651);
  ASSERT_TRUE(missing_alt.has_value());
  EXPECT_TRUE(std::isinf(missing_alt->alternative_error));
  EXPECT_FALSE(missing_alt->has_alternative);

  // Clean tilted synthetic tag with finite dual minimum must report
  // has_alternative == true and an orthonormal alternative rotation.
  const CameraIntrinsics clean_intr{
      .fx = 600.0, .fy = 600.0, .cx = 320.0, .cy = 240.0};
  constexpr double kTagSize = 0.2;
  const auto rot =
      talos::geometry::SO3<double>::Exp(Eigen::Vector3d(0.25, -0.35, 0.1));
  const Eigen::Vector3d trans(0.2, -0.1, 2.5);
  Detection tilted;
  constexpr double xs[4] = {-1.0, 1.0, 1.0, -1.0};
  constexpr double ys[4] = {1.0, 1.0, -1.0, -1.0};
  for (int i = 0; i < 4; ++i) {
    Eigen::Vector2d px;
    ASSERT_TRUE(clean_intr.ProjectPoint(
        rot.act(Eigen::Vector3d(xs[i] * kTagSize / 2.0, ys[i] * kTagSize / 2.0,
                                0.0)) +
            trans,
        &px));
    tilted.corners[i] = Point2D{px.x(), px.y()};
  }
  const auto dual = backend.EstimatePose(tilted, clean_intr, kTagSize);
  ASSERT_TRUE(dual.has_value());
  EXPECT_TRUE(std::isfinite(dual->alternative_error));
  EXPECT_TRUE(dual->has_alternative);
  EXPECT_NEAR(dual->alternative_rotation.determinant(), 1.0, 1e-6);
}

TEST(CpuAprilTagBackendTest, RejectsNegativeStride) {
  DetectorOptions options;
  options.family_name = "tag36h11";
  options.quad_decimate = 1.0f;
  CpuAprilTagBackend backend(options);

  const std::string path = GetTestdataPath("scene_single_tag0.png");
  image_u8_t* im = LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);
  ASSERT_EQ(backend.Detect(im->buf, im->width, im->height, im->stride).size(),
            1u);
  EXPECT_TRUE(
      backend.Detect(im->buf, im->width, im->height, -im->width).empty());
  EXPECT_TRUE(backend.Detect(im->buf, im->width, im->height, -1).empty());
  image_u8_destroy(im);
}

}  // namespace talos::vision::apriltag
