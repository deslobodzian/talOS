#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
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
#include <stdexcept>
#include <string>
#include <thread>
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

TEST_F(MetalAprilTagParityTest, OddDimensionsDecimate2xParity) {
  talos::vision::apriltag::DetectorOptions dec_opts = options_;
  dec_opts.quad_decimate = 2.0f;
  dec_opts.refine_edges = true;
  cpu_backend_->SetOptions(dec_opts);
  metal_backend_->SetOptions(dec_opts);

  apriltag_family_t* tf = tag36h11_create();
  image_u8_t* tag_im = apriltag_to_image(tf, 5);
  ASSERT_NE(tag_im, nullptr);

  // Odd dimensions (643x487) decimate to ceil dims 322x244 in upstream
  // image_u8_decimate(..., 2.0f); Metal must match ceil dims rather than floor.
  const int width = 643;
  const int height = 487;
  std::vector<uint8_t> rgb_buf(static_cast<size_t>(width) * height * 3, 200);
  std::vector<uint8_t> gray_buf(static_cast<size_t>(width) * height, 200);

  const int start_x = 261;
  const int start_y = 183;
  const int scale = 10;
  for (int ty = 0; ty < tag_im->height; ++ty) {
    for (int tx = 0; tx < tag_im->width; ++tx) {
      const uint8_t v = tag_im->buf[ty * tag_im->stride + tx];
      for (int sy = 0; sy < scale; ++sy) {
        for (int sx = 0; sx < scale; ++sx) {
          const int px = start_x + tx * scale + sx;
          const int py = start_y + ty * scale + sy;
          if (px < width && py < height) {
            gray_buf[py * width + px] = v;
            const int idx = (py * width + px) * 3;
            rgb_buf[idx + 0] = v;
            rgb_buf[idx + 1] = v;
            rgb_buf[idx + 2] = v;
          }
        }
      }
    }
  }

  const auto cpu_dets = cpu_backend_->Detect(gray_buf.data(), width, height,
                                             width, intrinsics_, tag_size_m_);
  const auto metal_rgb_dets =
      metal_backend_->Detect(rgb_buf.data(), width, height, width * 3,
                             PixelFormat::kRgb8, intrinsics_, tag_size_m_);
  const auto metal_gray_dets =
      metal_backend_->Detect(gray_buf.data(), width, height, width,
                             PixelFormat::kGray8, intrinsics_, tag_size_m_);

  ASSERT_EQ(cpu_dets.size(), 1u);
  EXPECT_EQ(cpu_dets[0].id, 5);
  AssertAcceptanceContract(cpu_dets, metal_rgb_dets,
                           "OddDimensionsDecimate2xRgb");
  AssertAcceptanceContract(cpu_dets, metal_gray_dets,
                           "OddDimensionsDecimate2xGray");

  Frame odd_frame(0, 0, 0, event::steady_now(), width, height,
                  PixelFormat::kGray8, width);
  std::memcpy(odd_frame.mutable_data(), gray_buf.data(), gray_buf.size());
  const auto exec = metal_backend_->Execute(odd_frame);
  ASSERT_TRUE(exec.success);
  ASSERT_EQ(exec.detections.size(), 1u);
  EXPECT_EQ(exec.detections[0].id, 5);
  EXPECT_TRUE(exec.detections[0].valid);
  if (metal_backend_->supports_gpu()) {
    EXPECT_EQ(exec.effective_backend, "metal");
  }

  // Also exercise a tiny odd image (65x49 -> 33x25) with 2x decimation.
  const int small_w = 65;
  const int small_h = 49;
  std::vector<uint8_t> small_gray(static_cast<size_t>(small_w) * small_h, 180);
  const auto small_cpu =
      cpu_backend_->Detect(small_gray.data(), small_w, small_h, small_w);
  const auto small_metal = metal_backend_->Detect(
      small_gray.data(), small_w, small_h, small_w, PixelFormat::kGray8);
  AssertAcceptanceContract(small_cpu, small_metal,
                           "TinyOddDimensionsDecimate2x");

  image_u8_destroy(tag_im);
  tag36h11_destroy(tf);

  cpu_backend_->SetOptions(options_);
  metal_backend_->SetOptions(options_);
}

TEST_F(MetalAprilTagParityTest, HammingOneRawValidMatchesCpuBackend) {
  apriltag_family_t* tf = tag36h11_create();
  image_u8_t* tag_im = apriltag_to_image(tf, 0);
  ASSERT_NE(tag_im, nullptr);

  // Flip exactly one interior payload bit so apriltag decodes ID 0 with
  // hamming == 1 (accepted under max_hamming = 2).
  tag_im->buf[3 * tag_im->stride + 3] =
      static_cast<uint8_t>(255 - tag_im->buf[3 * tag_im->stride + 3]);

  const int width = 320;
  const int height = 240;
  Frame frame(0, 0, 0, event::steady_now(), width, height, PixelFormat::kGray8,
              width);
  std::memset(frame.mutable_data(), 220, static_cast<size_t>(width) * height);

  const int start_x = 110;
  const int start_y = 70;
  const int scale = 10;
  for (int ty = 0; ty < tag_im->height; ++ty) {
    for (int tx = 0; tx < tag_im->width; ++tx) {
      const uint8_t v = tag_im->buf[ty * tag_im->stride + tx];
      for (int sy = 0; sy < scale; ++sy) {
        for (int sx = 0; sx < scale; ++sx) {
          frame.mutable_data()[(start_y + ty * scale + sy) * width +
                               (start_x + tx * scale + sx)] = v;
        }
      }
    }
  }
  image_u8_destroy(tag_im);
  tag36h11_destroy(tf);

  talos::vision::CpuAprilTagBackend cpu_compute(options_);
  const auto cpu_res = cpu_compute.Execute(frame);
  const auto metal_res = metal_backend_->Execute(frame);

  ASSERT_TRUE(cpu_res.success);
  ASSERT_TRUE(metal_res.success);
  ASSERT_EQ(cpu_res.detections.size(), 1u);
  ASSERT_EQ(metal_res.detections.size(), 1u);
  EXPECT_EQ(cpu_res.detections[0].id, 0);
  EXPECT_EQ(metal_res.detections[0].id, 0);
  EXPECT_EQ(cpu_res.detections[0].hamming, 1);
  EXPECT_EQ(metal_res.detections[0].hamming, 1);
  EXPECT_TRUE(cpu_res.detections[0].valid);
  EXPECT_EQ(metal_res.detections[0].valid, cpu_res.detections[0].valid);
}

TEST_F(MetalAprilTagParityTest, Stage2CandidateOverflowAfterLargerFrame) {
  // First run a large frame (800x600) so Metal's grow-only cached_cand_capacity
  // is 240,000, much larger than the next frame's maxc.
  const int big_w = 800;
  const int big_h = 600;
  std::vector<uint8_t> big_gray(static_cast<size_t>(big_w) * big_h, 128);
  for (int y = 0; y < big_h; ++y) {
    for (int x = 0; x < big_w; ++x) {
      big_gray[y * big_w + x] = ((x + y) & 1) ? 240 : 15;
    }
  }
  (void)metal_backend_->Detect(big_gray.data(), big_w, big_h, big_w,
                               PixelFormat::kGray8);

  // Now run a smaller 256x256 frame (maxc = 32,768) where most pixels are a
  // 1px checkerboard (> 32,768 edge candidates, yet <= 240,000 cached capacity)
  // around a clean tag36h11 ID 2. Stage 2 must detect overflow against this
  // frame's maxc and fall back cleanly to Stage 2a without reading stale slots.
  const int w = 256;
  const int h = 256;
  std::vector<uint8_t> small_gray(static_cast<size_t>(w) * h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      small_gray[y * w + x] = ((x + y) & 1) ? 240 : 15;
    }
  }

  // Clear a quiet margin around the stamped tag so it decodes cleanly.
  const int start_x = 88;
  const int start_y = 88;
  const int scale = 8;
  for (int y = start_y - 8; y < start_y + 10 * scale + 8; ++y) {
    for (int x = start_x - 8; x < start_x + 10 * scale + 8; ++x) {
      small_gray[y * w + x] = 220;
    }
  }
  apriltag_family_t* tf = tag36h11_create();
  image_u8_t* tag_im = apriltag_to_image(tf, 2);
  ASSERT_NE(tag_im, nullptr);
  for (int ty = 0; ty < tag_im->height; ++ty) {
    for (int tx = 0; tx < tag_im->width; ++tx) {
      const uint8_t v = tag_im->buf[ty * tag_im->stride + tx];
      for (int sy = 0; sy < scale; ++sy) {
        for (int sx = 0; sx < scale; ++sx) {
          small_gray[(start_y + ty * scale + sy) * w +
                     (start_x + tx * scale + sx)] = v;
        }
      }
    }
  }
  image_u8_destroy(tag_im);
  tag36h11_destroy(tf);

  const auto cpu_dets = cpu_backend_->Detect(small_gray.data(), w, h, w,
                                             intrinsics_, tag_size_m_);
  const auto metal_dets =
      metal_backend_->Detect(small_gray.data(), w, h, w, PixelFormat::kGray8,
                             intrinsics_, tag_size_m_);
  ASSERT_EQ(cpu_dets.size(), 1u);
  EXPECT_EQ(cpu_dets[0].id, 2);
  AssertAcceptanceContract(cpu_dets, metal_dets,
                           "Stage2CandidateOverflowAfterLargerFrame");
}

TEST_F(MetalAprilTagParityTest, RejectsUnsupportedDimensionsAndInvalidStride) {
  // 1. Sub-4x4 at decimate=1 and 6x6 at decimate=2 must return empty without
  // crashing in upstream threshold().
  std::vector<uint8_t> small_gray(size_t{64} * 64, 128);
  EXPECT_TRUE(
      metal_backend_->Detect(small_gray.data(), 3, 3, 3, PixelFormat::kGray8)
          .empty());

  Frame frame_3x3(0, 0, 0, event::steady_now(), 3, 3, PixelFormat::kGray8, 3);
  std::memset(frame_3x3.mutable_data(), 128, 9);
  const auto res_3x3 = metal_backend_->Execute(frame_3x3);
  EXPECT_TRUE(res_3x3.success);
  EXPECT_TRUE(res_3x3.detections.empty());

  auto dec2_opts = options_;
  dec2_opts.quad_decimate = 2.0f;
  metal_backend_->SetOptions(dec2_opts);
  EXPECT_TRUE(
      metal_backend_->Detect(small_gray.data(), 6, 6, 6, PixelFormat::kGray8)
          .empty());

  Frame frame_6x6(0, 0, 0, event::steady_now(), 6, 6, PixelFormat::kGray8, 6);
  std::memset(frame_6x6.mutable_data(), 128, 36);
  const auto res_6x6 = metal_backend_->Execute(frame_6x6);
  EXPECT_TRUE(res_6x6.success);
  EXPECT_TRUE(res_6x6.detections.empty());
  metal_backend_->SetOptions(options_);

  // 2. Dimensions >= 32768 must return empty before allocating buffers.
  uint8_t dummy = 0;
  EXPECT_TRUE(
      metal_backend_->Detect(&dummy, 32768, 4, 32768, PixelFormat::kGray8)
          .empty());
  EXPECT_TRUE(
      metal_backend_->Detect(&dummy, 4, 32768, 4, PixelFormat::kGray8).empty());

  // 3. Stride < width * bpp (e.g. stride == width for kBgra8), negative
  // stride, and width * bpp > INT_MAX must be rejected before allocation/copy.
  std::vector<uint8_t> bgra_small(size_t{64} * 64, 128);
  EXPECT_TRUE(metal_backend_
                  ->Detect(bgra_small.data(), 64, 64, /*stride=*/64,
                           PixelFormat::kBgra8)
                  .empty());
  EXPECT_TRUE(metal_backend_
                  ->Detect(small_gray.data(), 64, 64, /*stride=*/-1,
                           PixelFormat::kGray8)
                  .empty());
  EXPECT_TRUE(
      metal_backend_
          ->Detect(&dummy, 0x20000001, 4, /*stride=*/0, PixelFormat::kBgra8)
          .empty());
}

TEST_F(MetalAprilTagParityTest, ConcurrentExecuteAndSetOptionsAreThreadSafe) {
  const int width = 64;
  const int height = 64;
  Frame frame(0, 0, 0, event::steady_now(), width, height, PixelFormat::kGray8,
              width);
  std::memset(frame.mutable_data(), 180, static_cast<size_t>(width) * height);

  std::atomic<bool> stop{false};
  std::thread mutator([&]() {
    auto opts_a = options_;
    opts_a.quad_decimate = 1.0f;
    auto opts_b = options_;
    opts_b.quad_decimate = 2.0f;
    for (int i = 0; i < 12; ++i) {
      metal_backend_->SetOptions((i % 2 == 0) ? opts_a : opts_b);
    }
    stop.store(true, std::memory_order_release);
  });

  std::vector<std::thread> workers;
  workers.reserve(2);
  for (int t = 0; t < 2; ++t) {
    workers.emplace_back([&]() {
      while (!stop.load(std::memory_order_acquire)) {
        const auto res = metal_backend_->Execute(frame);
        EXPECT_TRUE(res.success);
        const auto info = metal_backend_->device_info();
        EXPECT_FALSE(info.empty());
        const auto opts = metal_backend_->options();
        EXPECT_EQ(opts.family_name, "tag36h11");
        (void)metal_backend_->last_timings();
      }
    });
  }

  mutator.join();
  for (auto& w : workers) w.join();
  metal_backend_->SetOptions(options_);
}

TEST_F(MetalAprilTagParityTest, SetOptionsIsAtomicOnInvalidOptions) {
  const std::string path = ResolveTestdataPath("scene_single_tag0.png");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Fixture not found: " << path;
  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);

  talos::vision::apriltag::DetectorOptions good_opts = options_;
  good_opts.quad_decimate = 1.0f;
  good_opts.quad_sigma = 0.0f;
  metal_backend_->SetOptions(good_opts);

  // Invalid threads with quad_sigma != 0 must throw without mutating
  // impl_->options or tearing down the working detector.
  talos::vision::apriltag::DetectorOptions bad_threads = good_opts;
  bad_threads.quad_sigma = 1.0f;
  bad_threads.threads = 0;
  EXPECT_THROW(metal_backend_->SetOptions(bad_threads), std::invalid_argument);

  talos::vision::apriltag::DetectorOptions bad_family = good_opts;
  bad_family.quad_decimate = 3.0f;
  bad_family.family_name = "not_a_real_family";
  EXPECT_THROW(metal_backend_->SetOptions(bad_family), std::invalid_argument);

  const auto expect_invalid_option =
      [&](const char* label,
          const talos::vision::apriltag::DetectorOptions& bad_opts) {
        SCOPED_TRACE(label);
        EXPECT_THROW((void)MetalAprilTagBackend(bad_opts),
                     std::invalid_argument);
        EXPECT_THROW(metal_backend_->SetOptions(bad_opts),
                     std::invalid_argument);
      };

  for (const float bad_decimate : {std::numeric_limits<float>::quiet_NaN(),
                                   std::numeric_limits<float>::infinity(),
                                   -std::numeric_limits<float>::infinity(),
                                   0.0f, -1.0f, -1e20f, 32768.0f}) {
    auto opts = good_opts;
    opts.quad_decimate = bad_decimate;
    expect_invalid_option("quad_decimate", opts);
  }
  for (const float bad_sigma :
       {std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(), 64.5f, -64.5f}) {
    auto opts = good_opts;
    opts.quad_sigma = bad_sigma;
    expect_invalid_option("quad_sigma", opts);
  }
  for (const double bad_sharpening :
       {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    auto opts = good_opts;
    opts.decode_sharpening = bad_sharpening;
    expect_invalid_option("decode_sharpening", opts);
  }
  for (const int bad_hamming : {-1, 4}) {
    auto opts = good_opts;
    opts.max_hamming = bad_hamming;
    expect_invalid_option("max_hamming", opts);
  }

  talos::vision::Frame frame(
      0, 0, 0, talos::event::steady_now(), static_cast<uint32_t>(im->width),
      static_cast<uint32_t>(im->height), static_cast<uint32_t>(im->stride),
      PixelFormat::kGray8, im->buf,
      static_cast<size_t>(im->stride) * im->height);
  const auto res = metal_backend_->Execute(frame);
  EXPECT_TRUE(res.success);
  ASSERT_EQ(res.detections.size(), 1u);
  EXPECT_EQ(res.detections[0].id, 0);
  if (metal_backend_->supports_gpu()) {
    EXPECT_EQ(res.effective_backend, "metal");
  }
  image_u8_destroy(im);
}

TEST_F(MetalAprilTagParityTest,
       DetectHandles4096AlignedAndNonPageMultipleInputs) {
  apriltag_family_t* tf = tag36h11_create();
  image_u8_t* tag_im = apriltag_to_image(tf, 4);
  ASSERT_NE(tag_im, nullptr);

  auto stamp_centered_tag = [&](uint8_t* dst, int width, int height,
                                int stride) {
    for (int y = 0; y < height; ++y) {
      std::memset(dst + static_cast<size_t>(y) * static_cast<size_t>(stride),
                  210, static_cast<size_t>(width));
    }
    const int scale = 8;
    const int start_x = (width - tag_im->width * scale) / 2;
    const int start_y = (height - tag_im->height * scale) / 2;
    for (int ty = 0; ty < tag_im->height; ++ty) {
      for (int tx = 0; tx < tag_im->width; ++tx) {
        const uint8_t v = tag_im->buf[static_cast<size_t>(ty) *
                                          static_cast<size_t>(tag_im->stride) +
                                      static_cast<size_t>(tx)];
        for (int sy = 0; sy < scale; ++sy) {
          for (int sx = 0; sx < scale; ++sx) {
            dst[static_cast<size_t>(start_y + ty * scale + sy) *
                    static_cast<size_t>(stride) +
                static_cast<size_t>(start_x + tx * scale + sx)] = v;
          }
        }
      }
    }
  };

  // 1. 4096-aligned (explicitly non-16384-aligned) pointer with 65536-byte
  // (16 KiB multiple) buffer: on Apple Silicon, newBufferWithBytesNoCopy
  // requires 16 KiB alignment so Metal must copy instead of aliasing.
  {
    constexpr int kWidth = 256;
    constexpr int kHeight = 256;
    constexpr size_t kBytes = static_cast<size_t>(kWidth) * kHeight;
    static_assert(kBytes % 16384u == 0u);

    std::vector<uint8_t> storage(kBytes + 32768u, 0);
    const uintptr_t base = reinterpret_cast<uintptr_t>(storage.data());
    const uintptr_t aligned_16k = (base + 16383u) & ~uintptr_t{16383u};
    uint8_t* ptr_16k = storage.data() + static_cast<size_t>(aligned_16k - base);
    uint8_t* ptr_4k = ptr_16k + 4096u;
    ASSERT_EQ(reinterpret_cast<uintptr_t>(ptr_4k) % 4096u, 0u);
    ASSERT_NE(reinterpret_cast<uintptr_t>(ptr_4k) % 16384u, 0u);

    stamp_centered_tag(ptr_4k, kWidth, kHeight, kWidth);
    const auto cpu_dets = cpu_backend_->Detect(ptr_4k, kWidth, kHeight, kWidth,
                                               intrinsics_, tag_size_m_);
    const auto metal_dets =
        metal_backend_->Detect(ptr_4k, kWidth, kHeight, kWidth,
                               PixelFormat::kGray8, intrinsics_, tag_size_m_);
    ASSERT_EQ(cpu_dets.size(), 1u);
    EXPECT_EQ(cpu_dets[0].id, 4);
    AssertAcceptanceContract(cpu_dets, metal_dets, "Aligned4096Not16384");

    // Also verify page-aligned + page-multiple buffer parity (zero-copy path).
    ASSERT_EQ(reinterpret_cast<uintptr_t>(ptr_16k) % 16384u, 0u);
    std::memmove(ptr_16k, ptr_4k, kBytes);
    const auto metal_16k_dets =
        metal_backend_->Detect(ptr_16k, kWidth, kHeight, kWidth,
                               PixelFormat::kGray8, intrinsics_, tag_size_m_);
    AssertAcceptanceContract(cpu_dets, metal_16k_dets, "Aligned16384PageMult");
  }

  // 2. 16384-aligned pointer with non-page-multiple byte size (321 * 241 =
  // 77361 bytes, odd): must not pass non-page-multiple length to
  // newBufferWithBytesNoCopy.
  {
    constexpr int kWidth = 321;
    constexpr int kHeight = 241;
    constexpr size_t kBytes = static_cast<size_t>(kWidth) * kHeight;
    static_assert(kBytes % 4096u != 0u);
    static_assert(kBytes % 16384u != 0u);

    std::vector<uint8_t> storage(kBytes + 16384u, 0);
    const uintptr_t base = reinterpret_cast<uintptr_t>(storage.data());
    const uintptr_t aligned_16k = (base + 16383u) & ~uintptr_t{16383u};
    uint8_t* ptr_16k = storage.data() + static_cast<size_t>(aligned_16k - base);
    ASSERT_EQ(reinterpret_cast<uintptr_t>(ptr_16k) % 16384u, 0u);

    stamp_centered_tag(ptr_16k, kWidth, kHeight, kWidth);
    const auto cpu_dets = cpu_backend_->Detect(ptr_16k, kWidth, kHeight, kWidth,
                                               intrinsics_, tag_size_m_);
    const auto metal_dets =
        metal_backend_->Detect(ptr_16k, kWidth, kHeight, kWidth,
                               PixelFormat::kGray8, intrinsics_, tag_size_m_);
    ASSERT_EQ(cpu_dets.size(), 1u);
    EXPECT_EQ(cpu_dets[0].id, 4);
    AssertAcceptanceContract(cpu_dets, metal_dets, "NonPageMultipleBytes");
  }

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

TEST_F(MetalAprilTagParityTest, GpuThresholdParityWithMinDiff256) {
  // min_diff == 256 must NOT truncate to 0 via uchar cast: all non-remainder
  // pixels must binarize to 127, matching CPU threshold().
  const int w = 64;
  const int h = 64;
  image_u8_t* im = image_u8_create(w, h);
  ASSERT_NE(im, nullptr);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      im->buf[y * im->stride + x] = (((x / 4) + (y / 4)) % 2 == 0) ? 255 : 0;
    }
  }
  AssertThresholdParity(*metal_backend_, im, 256, "MinDiff256");
  image_u8_destroy(im);
}

TEST_F(MetalAprilTagParityTest,
       RunGpuThresholdRejectsShortStrideAndUndersizedBuffers) {
  if (!metal_backend_->supports_gpu()) {
    GTEST_SKIP() << "No Metal GPU available";
  }
  MTL::Device* device = metal_backend_->device();
  MTL::CommandQueue* queue = metal_backend_->queue();
  ASSERT_NE(device, nullptr);
  ASSERT_NE(queue, nullptr);

  NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();

  constexpr uint32_t kWidth = 32;
  constexpr uint32_t kHeight = 32;
  constexpr size_t kFullBytes = static_cast<size_t>(kWidth) * kHeight;
  constexpr uint8_t kSentinel = 0x5Au;

  auto run_and_expect_untouched = [&](MTL::Buffer* gray_buf,
                                      MTL::Buffer* thresh_buf, uint32_t w,
                                      uint32_t h, uint32_t stride) {
    std::memset(thresh_buf->contents(), kSentinel, thresh_buf->length());
    MTL::CommandBuffer* cmd_buf = queue->commandBuffer();
    ASSERT_NE(cmd_buf, nullptr);
    metal_backend_->RunGpuThreshold(cmd_buf, gray_buf, thresh_buf, w, h, stride,
                                    /*min_diff=*/5);
    cmd_buf->commit();
    cmd_buf->waitUntilCompleted();
    EXPECT_EQ(cmd_buf->status(), MTL::CommandBufferStatusCompleted);
    const auto* out = static_cast<const uint8_t*>(thresh_buf->contents());
    for (size_t i = 0; i < thresh_buf->length(); ++i) {
      ASSERT_EQ(out[i], kSentinel) << "Output buffer mutated at byte " << i;
    }
  };

  MTL::Buffer* full_gray =
      device->newBuffer(kFullBytes, MTL::ResourceStorageModeShared);
  MTL::Buffer* small_gray =
      device->newBuffer(kFullBytes - 1, MTL::ResourceStorageModeShared);
  MTL::Buffer* full_thresh =
      device->newBuffer(kFullBytes, MTL::ResourceStorageModeShared);
  MTL::Buffer* small_thresh =
      device->newBuffer(kFullBytes - 1, MTL::ResourceStorageModeShared);
  ASSERT_NE(full_gray, nullptr);
  ASSERT_NE(small_gray, nullptr);
  ASSERT_NE(full_thresh, nullptr);
  ASSERT_NE(small_thresh, nullptr);
  std::memset(full_gray->contents(), 128, kFullBytes);
  std::memset(small_gray->contents(), 128, kFullBytes - 1);

  // 1. Non-zero stride < width must be rejected.
  run_and_expect_untouched(full_gray, full_thresh, kWidth, kHeight, kWidth - 1);
  // 2. Undersized gray buffer must be rejected.
  run_and_expect_untouched(small_gray, full_thresh, kWidth, kHeight, kWidth);
  // 3. Undersized threshold output buffer must be rejected.
  run_and_expect_untouched(full_gray, small_thresh, kWidth, kHeight, kWidth);
  // 4. Stride * height exceeding buffer length must be rejected.
  run_and_expect_untouched(full_gray, full_thresh, kWidth, kHeight, kWidth + 4);

  full_gray->release();
  small_gray->release();
  full_thresh->release();
  small_thresh->release();
  pool->release();
}

#endif  // defined(__APPLE__)

}  // namespace
}  // namespace talos::vision::metal
