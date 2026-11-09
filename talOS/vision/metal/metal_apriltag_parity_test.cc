#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "talOS/geometry/se3.h"
#include "talOS/vision/apriltag/cpu_apriltag_backend.h"
#include "talOS/vision/apriltag/test_image_loader.h"
#include "talOS/vision/compute_backend.h"
#include "talOS/vision/frame.h"
#include "talOS/vision/metal/metal_apriltag_backend.h"

#if defined(__APPLE__)
#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#endif

extern "C" {
#include "apriltag.h"
#include "common/image_u8.h"
#include "common/pjpeg.h"
#include "common/workerpool.h"
#include "tag36h11.h"

image_u8_t* threshold(apriltag_detector_t* td, image_u8_t* im);
}

namespace talos::vision::metal {
namespace {

std::string ResolveTestdataPath(const std::string& filename) {
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
    std::string p3 =
        std::string(srcdir) + "/+_repo_rules+apriltag/test/data/" + filename;
    if (std::filesystem::exists(p3)) return p3;
  }
  for (const auto& path : candidates) {
    if (std::filesystem::exists(path)) return path;
  }
  return "talOS/vision/testdata/" + filename;
}

// Asserts the exact acceptance contract in PLAN.md:
// - Detection set: exact count, family, ID, and Hamming results; no missing or
// extra tags.
// - Matching: 1-to-1 spatial matching within family/ID groups.
// - Corners and center: at most 0.1 pixel Euclidean error per point.
// - Decision margin: absolute difference at most 1e-4 + 1e-5 * abs(reference).
// - Homography: compare projected canonical tag points, accounting for
// homogeneous scale;
//   at most 0.1 pixel projection error.
// - Pose: for fixed, well-conditioned calibrated fixtures: at most 1 mm
// translation
//   and 0.1 degree rotation difference.
void AssertAcceptanceContract(
    const std::vector<talos::vision::apriltag::Detection>& cpu_detections,
    const std::vector<talos::vision::apriltag::Detection>& metal_detections,
    const std::string& scene_name) {
  // 1. Detection set: exact count, family, ID, and Hamming results; no missing
  // or extra tags.
  ASSERT_EQ(cpu_detections.size(), metal_detections.size())
      << "Detection count mismatch in " << scene_name << ": CPU found "
      << cpu_detections.size() << ", Metal found " << metal_detections.size();

  if (cpu_detections.empty()) {
    return;
  }

  // 2. Matching: 1-to-1 spatial matching within family/ID groups.
  std::vector<bool> metal_matched(metal_detections.size(), false);

  for (size_t i = 0; i < cpu_detections.size(); ++i) {
    const auto& c_det = cpu_detections[i];
    int best_match_idx = -1;
    double best_dist = std::numeric_limits<double>::max();

    for (size_t j = 0; j < metal_detections.size(); ++j) {
      if (metal_matched[j]) continue;
      const auto& m_det = metal_detections[j];

      // Match by ID
      if (c_det.id == m_det.id) {
        double dist = std::hypot(c_det.center.x - m_det.center.x,
                                 c_det.center.y - m_det.center.y);
        if (dist < best_dist) {
          best_dist = dist;
          best_match_idx = static_cast<int>(j);
        }
      }
    }

    ASSERT_GE(best_match_idx, 0)
        << "Could not find 1-to-1 spatial match for CPU tag ID " << c_det.id
        << " in scene " << scene_name;
    metal_matched[best_match_idx] = true;

    const auto& m_det = metal_detections[best_match_idx];

    // Detection set checks: ID and Hamming
    EXPECT_EQ(c_det.id, m_det.id) << "Tag ID mismatch in scene " << scene_name;
    EXPECT_EQ(c_det.hamming, m_det.hamming)
        << "Hamming distance mismatch for tag ID " << c_det.id << " in scene "
        << scene_name;

    // 3. Corners and center: at most 0.1 pixel Euclidean error per point.
    double center_err = std::hypot(c_det.center.x - m_det.center.x,
                                   c_det.center.y - m_det.center.y);
    EXPECT_LE(center_err, 0.1) << "Center Euclidean error (" << center_err
                               << " px) exceeds 0.1 px for tag ID " << c_det.id
                               << " in scene " << scene_name;

    for (size_t k = 0; k < 4; ++k) {
      double corner_err = std::hypot(c_det.corners[k].x - m_det.corners[k].x,
                                     c_det.corners[k].y - m_det.corners[k].y);
      EXPECT_LE(corner_err, 0.1)
          << "Corner " << k << " Euclidean error (" << corner_err
          << " px) exceeds 0.1 px for tag ID " << c_det.id << " in scene "
          << scene_name;
    }

    // 4. Decision margin: absolute difference at most 1e-4 + 1e-5 *
    // abs(reference).
    double margin_diff =
        std::abs(m_det.decision_margin - c_det.decision_margin);
    double margin_tol = 1e-4 + 1e-5 * std::abs(c_det.decision_margin);
    EXPECT_LE(margin_diff, margin_tol)
        << "Decision margin difference (" << margin_diff
        << ") exceeds tolerance (" << margin_tol << ") for tag ID " << c_det.id
        << " in scene " << scene_name;

    // 5. Homography: compare projected canonical tag points, accounting for
    // homogeneous scale; at most 0.1 pixel projection error.
    const std::array<std::pair<double, double>, 4> canonical_pts = {
        {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}}};

    for (size_t p = 0; p < canonical_pts.size(); ++p) {
      double u = canonical_pts[p].first;
      double v = canonical_pts[p].second;

      // Project using CPU homography
      double wc = c_det.homography(2, 0) * u + c_det.homography(2, 1) * v +
                  c_det.homography(2, 2);
      ASSERT_GT(std::abs(wc), 1e-9)
          << "Degenerate CPU homography scale in " << scene_name;
      double xc = (c_det.homography(0, 0) * u + c_det.homography(0, 1) * v +
                   c_det.homography(0, 2)) /
                  wc;
      double yc = (c_det.homography(1, 0) * u + c_det.homography(1, 1) * v +
                   c_det.homography(1, 2)) /
                  wc;

      // Project using Metal homography
      double wm = m_det.homography(2, 0) * u + m_det.homography(2, 1) * v +
                  m_det.homography(2, 2);
      ASSERT_GT(std::abs(wm), 1e-9)
          << "Degenerate Metal homography scale in " << scene_name;
      double xm = (m_det.homography(0, 0) * u + m_det.homography(0, 1) * v +
                   m_det.homography(0, 2)) /
                  wm;
      double ym = (m_det.homography(1, 0) * u + m_det.homography(1, 1) * v +
                   m_det.homography(1, 2)) /
                  wm;

      double proj_err = std::hypot(xm - xc, ym - yc);
      EXPECT_LE(proj_err, 0.1) << "Homography projection error (" << proj_err
                               << " px) exceeds 0.1 px for tag ID " << c_det.id
                               << " in scene " << scene_name;
    }

    // 6. Pose: for fixed, well-conditioned calibrated fixtures:
    // at most 1 mm translation and 0.1 degree rotation difference.
    if (c_det.pose.has_value() && m_det.pose.has_value()) {
      const auto& c_pose = *c_det.pose;
      const auto& m_pose = *m_det.pose;

      double trans_err = (m_pose.translation - c_pose.translation).norm();
      EXPECT_LE(trans_err, 0.001)
          << "Pose translation error (" << (trans_err * 1000.0)
          << " mm) exceeds 1 mm for tag ID " << c_det.id << " in scene "
          << scene_name;

      Eigen::Matrix3d R_rel = m_pose.rotation * c_pose.rotation.transpose();
      double tr = R_rel.trace();
      double cos_theta = std::clamp((tr - 1.0) * 0.5, -1.0, 1.0);
      double angle_deg = std::acos(cos_theta) * (180.0 / M_PI);
      EXPECT_LE(angle_deg, 0.1) << "Pose rotation angle error (" << angle_deg
                                << " deg) exceeds 0.1 deg for tag ID "
                                << c_det.id << " in scene " << scene_name;
    }
  }

  // Ensure 1-to-1 matching (no extra Metal tags)
  for (size_t j = 0; j < metal_matched.size(); ++j) {
    EXPECT_TRUE(metal_matched[j])
        << "Extra unmatched Metal detection tag ID " << metal_detections[j].id
        << " in scene " << scene_name;
  }
}

class MetalAprilTagParityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup identical options for both backends
    options_.family_name = "tag36h11";
    options_.threads = 1;
    options_.quad_decimate = 1.0f;
    options_.quad_sigma = 0.0f;
    options_.refine_edges = false;
    options_.decode_sharpening = 0.25;
    options_.max_hamming = 2;

    intrinsics_.fx = 600.0;
    intrinsics_.fy = 600.0;
    intrinsics_.cx = 400.0;
    intrinsics_.cy = 300.0;
    tag_size_m_ = 0.1651;

    cpu_backend_ =
        std::make_unique<talos::vision::apriltag::CpuAprilTagBackend>(options_);
    metal_backend_ = std::make_unique<MetalAprilTagBackend>(options_);
  }

  talos::vision::apriltag::DetectorOptions options_;
  talos::vision::apriltag::CameraIntrinsics intrinsics_;
  double tag_size_m_{0.1651};
  std::unique_ptr<talos::vision::apriltag::CpuAprilTagBackend> cpu_backend_;
  std::unique_ptr<MetalAprilTagBackend> metal_backend_;
};

TEST_F(MetalAprilTagParityTest, FixtureDetectionAndPoseParity) {
  struct Fixture {
    const char* file;
    size_t count;
  };
  for (const auto& fixture : {
           Fixture{"33369213973_9d9bb4cc96_c.jpg", 12},
           Fixture{"34085369442_304b6bafd9_c.jpg", 25},
           Fixture{"34139872896_defdb2f8d9_c.jpg", 10},
           Fixture{"scene_single_tag0.png", 1},
           Fixture{"scene_rotated_tag.png", 1},
           Fixture{"scene_multi_tag.png", 4},
           Fixture{"scene_non_aligned.png", 1},
           Fixture{"scene_empty.png", 0},
       }) {
    SCOPED_TRACE(fixture.file);
    const auto path = ResolveTestdataPath(fixture.file);
    const auto cpu =
        cpu_backend_->DetectImageFile(path, intrinsics_, tag_size_m_);
    const auto metal =
        metal_backend_->DetectImageFile(path, intrinsics_, tag_size_m_);
    ASSERT_EQ(cpu.size(), fixture.count);
    AssertAcceptanceContract(cpu, metal, fixture.file);
  }
}

TEST_F(MetalAprilTagParityTest, InGridSyntheticRgbAndDecimationParity) {
  // Test in-memory RGB synthetic frame with 2x decimation
  talos::vision::apriltag::DetectorOptions dec_opts = options_;
  dec_opts.quad_decimate = 2.0f;
  dec_opts.refine_edges = true;
  cpu_backend_->SetOptions(dec_opts);
  metal_backend_->SetOptions(dec_opts);

  apriltag_family_t* tf = tag36h11_create();
  image_u8_t* tag_im = apriltag_to_image(tf, 3);

  const int width = 640;
  const int height = 480;
  std::vector<uint8_t> rgb_buf(static_cast<size_t>(width) * height * 3,
                               200);  // Light gray background

  // Stamp tag into center of RGB image
  const int start_x = 260;
  const int start_y = 180;
  const int scale = 10;
  for (int ty = 0; ty < tag_im->height; ++ty) {
    for (int tx = 0; tx < tag_im->width; ++tx) {
      uint8_t v = tag_im->buf[ty * tag_im->stride + tx];
      for (int sy = 0; sy < scale; ++sy) {
        for (int sx = 0; sx < scale; ++sx) {
          int px = start_x + tx * scale + sx;
          int py = start_y + ty * scale + sy;
          if (px < width && py < height) {
            int idx = (py * width + px) * 3;
            rgb_buf[idx + 0] = v;
            rgb_buf[idx + 1] = v;
            rgb_buf[idx + 2] = v;
          }
        }
      }
    }
  }

  // CPU reference: convert RGB to grayscale using Rec. 601 formula matching
  // Metal shader
  std::vector<uint8_t> cpu_gray_buf(static_cast<size_t>(width) * height);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx = (y * width + x) * 3;
      uint32_t r = rgb_buf[idx + 0];
      uint32_t g = rgb_buf[idx + 1];
      uint32_t b = rgb_buf[idx + 2];
      cpu_gray_buf[y * width + x] =
          static_cast<uint8_t>((299 * r + 587 * g + 114 * b + 500) / 1000);
    }
  }

  auto cpu_dets = cpu_backend_->Detect(cpu_gray_buf.data(), width, height,
                                       width, intrinsics_, tag_size_m_);
  auto metal_dets =
      metal_backend_->Detect(rgb_buf.data(), width, height, width * 3,
                             PixelFormat::kRgb8, intrinsics_, tag_size_m_);

  ASSERT_EQ(cpu_dets.size(), 1u);
  AssertAcceptanceContract(cpu_dets, metal_dets,
                           "InGridSyntheticRgbAndDecimation");

  const auto& timings = metal_backend_->last_timings();
  EXPECT_GT(timings.total_us, 0u);
  if (metal_backend_->supports_gpu()) {
    EXPECT_GT(timings.grayscale_us + timings.threshold_us, 0u);
  }

  // The contiguous-gray input path must retain full-resolution pixels too:
  // refinement at 2x decimation must not operate on the half-size image.
  auto gray_metal_dets =
      metal_backend_->Detect(cpu_gray_buf.data(), width, height, width,
                             PixelFormat::kGray8, intrinsics_, tag_size_m_);
  AssertAcceptanceContract(cpu_dets, gray_metal_dets,
                           "InGridSyntheticGrayAndDecimation");

  image_u8_destroy(tag_im);
  tag36h11_destroy(tf);

  // Restore options
  cpu_backend_->SetOptions(options_);
  metal_backend_->SetOptions(options_);
}

TEST_F(MetalAprilTagParityTest, RepeatedIdSpatialMatchingParity) {
  // Test repeated IDs in different quadrants: ensures 1-to-1 spatial matching
  // within same ID
  apriltag_family_t* tf = tag36h11_create();
  image_u8_t* tag_im = apriltag_to_image(tf, 7);

  const int width = 800;
  const int height = 600;
  std::vector<uint8_t> gray_buf(static_cast<size_t>(width) * height, 220);

  auto stamp_tag = [&](int start_x, int start_y) {
    const int scale = 8;
    for (int ty = 0; ty < tag_im->height; ++ty) {
      for (int tx = 0; tx < tag_im->width; ++tx) {
        uint8_t v = tag_im->buf[ty * tag_im->stride + tx];
        for (int sy = 0; sy < scale; ++sy) {
          for (int sx = 0; sx < scale; ++sx) {
            int px = start_x + tx * scale + sx;
            int py = start_y + ty * scale + sy;
            if (px < width && py < height) {
              gray_buf[py * width + px] = v;
            }
          }
        }
      }
    }
  };

  // Stamp tag ID 7 twice in different quadrants
  stamp_tag(100, 100);
  stamp_tag(500, 350);

  auto cpu_dets = cpu_backend_->Detect(gray_buf.data(), width, height, width,
                                       intrinsics_, tag_size_m_);
  auto metal_dets =
      metal_backend_->Detect(gray_buf.data(), width, height, width,
                             PixelFormat::kGray8, intrinsics_, tag_size_m_);

  ASSERT_EQ(cpu_dets.size(), 2u);
  EXPECT_EQ(cpu_dets[0].id, 7);
  EXPECT_EQ(cpu_dets[1].id, 7);
  AssertAcceptanceContract(cpu_dets, metal_dets, "RepeatedIdSpatialMatching");

  image_u8_destroy(tag_im);
  tag36h11_destroy(tf);
}

#if defined(__APPLE__)
void AssertThresholdParity(MetalAprilTagBackend& backend, image_u8_t* im,
                           uint32_t min_diff = 5,
                           const std::string& test_name = "") {
  ASSERT_NE(im, nullptr);
  ASSERT_TRUE(backend.supports_gpu());

  apriltag_detector_t* td = apriltag_detector_create();
  td->nthreads = 1;
  td->wp = workerpool_create(td->nthreads);
  td->qtp.min_white_black_diff = static_cast<int>(min_diff);
  image_u8_t* cpu_thresh = threshold(td, im);
  ASSERT_NE(cpu_thresh, nullptr);

  MTL::Device* device = backend.device();
  MTL::CommandQueue* queue = backend.queue();
  ASSERT_NE(device, nullptr);
  ASSERT_NE(queue, nullptr);

  const size_t buf_bytes = static_cast<size_t>(im->stride) * im->height;
  MTL::Buffer* gray_buf =
      device->newBuffer(im->buf, buf_bytes, MTL::ResourceStorageModeShared);
  MTL::Buffer* thresh_buf =
      device->newBuffer(buf_bytes, MTL::ResourceStorageModeShared);
  ASSERT_NE(gray_buf, nullptr);
  ASSERT_NE(thresh_buf, nullptr);

  MTL::CommandBuffer* cmd_buf = queue->commandBuffer();
  backend.RunGpuThreshold(cmd_buf, gray_buf, thresh_buf, im->width, im->height,
                          im->stride, min_diff);
  cmd_buf->commit();
  cmd_buf->waitUntilCompleted();

  const uint8_t* gpu_data = static_cast<const uint8_t*>(thresh_buf->contents());

  int mismatches = 0;
  for (int y = 0; y < im->height; ++y) {
    for (int x = 0; x < im->width; ++x) {
      uint8_t c = cpu_thresh->buf[y * cpu_thresh->stride + x];
      uint8_t g = gpu_data[y * im->stride + x];

      // Tri-state binarization contract: values must be 0, 127, or 255
      EXPECT_TRUE(g == 0 || g == 127 || g == 255)
          << "Invalid pixel value in GPU threshold at (" << x << ", " << y
          << "): " << static_cast<int>(g) << " in " << test_name;

      if (c != g) {
        if (mismatches < 10) {
          ADD_FAILURE() << "Threshold mismatch at (" << x << ", " << y
                        << "): CPU=" << static_cast<int>(c)
                        << " GPU=" << static_cast<int>(g) << " in "
                        << test_name;
        }
        mismatches++;
      }
    }
  }

  EXPECT_EQ(mismatches, 0) << test_name << ": Total " << mismatches
                           << " mismatches out of " << (im->width * im->height)
                           << " pixels ("
                           << (100.0 * mismatches / (im->width * im->height))
                           << "% mismatch rate)";

  gray_buf->release();
  thresh_buf->release();
  image_u8_destroy(cpu_thresh);
  apriltag_detector_destroy(td);
}

TEST_F(MetalAprilTagParityTest, GpuThresholdParityOnPhotographicFixture1) {
  const std::string path = ResolveTestdataPath("33369213973_9d9bb4cc96_c.jpg");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Fixture not found: " << path;

  pjpeg_t* pjpeg = pjpeg_create_from_file(path.c_str(), 0, nullptr);
  ASSERT_NE(pjpeg, nullptr);
  image_u8_t* im = pjpeg_to_u8_baseline(pjpeg);
  pjpeg_destroy(pjpeg);
  ASSERT_NE(im, nullptr);

  AssertThresholdParity(*metal_backend_, im, 5, "PhotographicFixture1");
  image_u8_destroy(im);
}

TEST_F(MetalAprilTagParityTest, GpuThresholdParityOnPhotographicFixture2) {
  const std::string path = ResolveTestdataPath("34085369442_304b6bafd9_c.jpg");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Fixture not found: " << path;

  pjpeg_t* pjpeg = pjpeg_create_from_file(path.c_str(), 0, nullptr);
  ASSERT_NE(pjpeg, nullptr);
  image_u8_t* im = pjpeg_to_u8_baseline(pjpeg);
  pjpeg_destroy(pjpeg);
  ASSERT_NE(im, nullptr);

  AssertThresholdParity(*metal_backend_, im, 5, "PhotographicFixture2");
  image_u8_destroy(im);
}

TEST_F(MetalAprilTagParityTest, GpuThresholdParityOnSyntheticScene) {
  const std::string path = ResolveTestdataPath("scene_single_tag0.png");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Fixture not found: " << path;

  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);

  AssertThresholdParity(*metal_backend_, im, 5, "SyntheticScene");
  image_u8_destroy(im);
}

TEST_F(MetalAprilTagParityTest, GpuThresholdParityOnNonAlignedRemainderPixels) {
  // Test image with dimensions that do not divide evenly by 4 (e.g. 643 x 482)
  // This explicitly validates the border and remainder pixel clamping logic.
  const int w = 643;
  const int h = 482;
  image_u8_t* im = image_u8_create(w, h);
  ASSERT_NE(im, nullptr);

  // Fill with diverse pattern: step edges, ramps, constant areas
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (x < 200 && y < 200) {
        im->buf[y * im->stride + x] = static_cast<uint8_t>((x + y) % 256);
      } else if (x >= 400 && y >= 300) {
        im->buf[y * im->stride + x] = ((x / 16) % 2 == (y / 16) % 2) ? 240 : 20;
      } else if (y >= 478 || x >= 640) {
        // Remainder pixel border region
        im->buf[y * im->stride + x] =
            static_cast<uint8_t>((x * 37 + y * 17) % 256);
      } else {
        // Low contrast region
        im->buf[y * im->stride + x] = static_cast<uint8_t>(100 + (x % 3));
      }
    }
  }

  AssertThresholdParity(*metal_backend_, im, 5, "NonAlignedRemainderPixels");
  image_u8_destroy(im);
}

TEST_F(MetalAprilTagParityTest, GpuThresholdParityOnLowAndHighContrastTiles) {
  // 320x240 image with distinct tile regions:
  // - Top half: low contrast tiles (< min_diff=5) -> all pixels must binarize
  // to 127
  // - Bottom half: high contrast tiles (difference > 200) -> 0 or 255
  const int w = 320;
  const int h = 240;
  image_u8_t* im = image_u8_create(w, h);
  ASSERT_NE(im, nullptr);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (y < 120) {
        // Low contrast: variation of at most 2 across entire tile
        im->buf[y * im->stride + x] = static_cast<uint8_t>(128 + ((x + y) % 2));
      } else {
        // High contrast: checkerboard with black=20, white=230
        bool white = ((x / 4) % 2 == (y / 4) % 2);
        im->buf[y * im->stride + x] = white ? 230 : 20;
      }
    }
  }

  AssertThresholdParity(*metal_backend_, im, 5, "LowAndHighContrastTiles");
  image_u8_destroy(im);
}

TEST_F(MetalAprilTagParityTest, GpuThresholdInternalBufferCaching) {
  // Test passing thresh_buf = nullptr to verify internal caching
  const int w = 400;
  const int h = 300;
  image_u8_t* im = image_u8_create(w, h);
  ASSERT_NE(im, nullptr);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      im->buf[y * im->stride + x] = ((x / 8) % 2 == (y / 8) % 2) ? 220 : 30;
    }
  }

  apriltag_detector_t* td = apriltag_detector_create();
  td->nthreads = 1;
  td->wp = workerpool_create(td->nthreads);
  td->qtp.min_white_black_diff = 5;
  image_u8_t* cpu_thresh = threshold(td, im);
  ASSERT_NE(cpu_thresh, nullptr);

  MTL::Device* device = metal_backend_->device();
  MTL::CommandQueue* queue = metal_backend_->queue();
  ASSERT_NE(device, nullptr);
  ASSERT_NE(queue, nullptr);

  const size_t buf_bytes = static_cast<size_t>(im->stride) * im->height;
  MTL::Buffer* gray_buf =
      device->newBuffer(im->buf, buf_bytes, MTL::ResourceStorageModeShared);
  ASSERT_NE(gray_buf, nullptr);

  // Pass nullptr for thresh_buf so RunGpuThreshold allocates and caches it
  // internally
  MTL::CommandBuffer* cmd_buf = queue->commandBuffer();
  metal_backend_->RunGpuThreshold(cmd_buf, gray_buf, nullptr, im->width,
                                  im->height, im->stride, 5);
  cmd_buf->commit();
  cmd_buf->waitUntilCompleted();

  MTL::Buffer* thresh_buf = metal_backend_->thresh_buffer();
  MTL::Buffer* min_buf = metal_backend_->min_buffer();
  MTL::Buffer* max_buf = metal_backend_->max_buffer();

  ASSERT_NE(thresh_buf, nullptr);
  ASSERT_NE(min_buf, nullptr);
  ASSERT_NE(max_buf, nullptr);

  const uint8_t* gpu_data = static_cast<const uint8_t*>(thresh_buf->contents());
  for (int y = 0; y < im->height; ++y) {
    for (int x = 0; x < im->width; ++x) {
      uint8_t c = cpu_thresh->buf[y * cpu_thresh->stride + x];
      uint8_t g = gpu_data[y * im->stride + x];
      EXPECT_EQ(c, g);
    }
  }

  gray_buf->release();
  image_u8_destroy(cpu_thresh);
  image_u8_destroy(im);
  apriltag_detector_destroy(td);
}

#endif  // defined(__APPLE__)

}  // namespace
}  // namespace talos::vision::metal
