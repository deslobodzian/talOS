// Software rasterizer for synthetic AprilTag camera images.
//
// Renders field-layout tags as perspective-correct 10x10 tag patterns into
// a grayscale buffer, so the REAL detector pipeline can run on synthetic
// frames. This is the image-level sim: what the detector sees is rendered,
// not projected, which means decode, pose estimation, ambiguity, and timing
// are all exercised for real — the properties estimator testing depends on.
//
// Pattern convention: the 10x10 grid spans tag units [-1.25, +1.25] (the
// fitted quad the detector finds is the outer edge of the black
// border, i.e. tag units ±1). Ring 0 is white, ring 1 black, and the inner
// 6x6 carries the code bits (row 0 = TOP as seen facing the tag face,
// true = white). The row orientation is pinned by the render-to-detect
// round-trip test: a flipped mapping fails to decode, so this cannot drift
// silently.
//
// Honest limitations, documented so sim results are read correctly:
//   - flat shading only: no lighting falloff, lens blur, or occlusion by
//     field structures/robots;
//   - the background defaults to white, not black: the detector's white
//     border model samples OUTSIDE the fitted quad, so a black void would
//     starve it. Matches the repo's proven synthetic renderer;
//   - image-edge clipping is supported, but a tag crossing the camera's near
//     plane is culled; tag overlap uses far-to-near painter ordering.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <opencv2/imgproc.hpp>
#include <random>
#include <vector>

#include "talOS/vision/field_sim/sim_camera.h"

namespace talos::vision::field_sim {

// Tag code bits: 6x6 data cells, row 0 = TOP as seen facing the tag face,
// true = WHITE cell. Only code bits are stored; both border rings are fixed
// by the family spec and painted by the rasterizer.
struct TagPattern {
  int id = -1;
  std::array<std::array<bool, 6>, 6> bits{};
};

struct SimRenderOptions {
  // White void by default: the detector's white border model samples
  // OUTSIDE the fitted quad, so the surround must read bright — the same
  // reason the repo's proven SyntheticCameraSource renders on white. Think
  // bright lab, not dark arena.
  uint8_t background_gray = 255;
  uint8_t black_gray = 0;
  uint8_t white_gray = 255;
};

// Caller-owned storage retains capacity across frames and exposure samples.
struct RenderScratch {
  std::vector<SimTagDetection> detections;
  std::vector<SimTagDetection> start;
  std::vector<SimTagDetection> end;
  std::vector<uint8_t> instant;
  cv::Mat accumulated;
  cv::Mat sensor_noise;
  cv::Mat exposed;
};

// Renders every projectable layout tag with a known pattern into gray_out
// (resized to w*h, row-major, stride w). Tags without a pattern entry are
// skipped. Returns the number of tags rasterized.
inline int RenderInstantaneousFieldView(
    const FieldLayout& layout, const std::map<int, TagPattern>& patterns,
    const SimCameraView& cam, const talos::geometry::SE3<double>& T_field_robot,
    double tag_size_m, const SimRenderOptions& options,
    std::vector<uint8_t>* gray_out, std::vector<SimTagDetection>* detections) {
  if (gray_out == nullptr) return 0;
  const int w = static_cast<int>(cam.intrinsics.image_width);
  const int h = static_cast<int>(cam.intrinsics.image_height);
  if (w <= 0 || h <= 0 || !(tag_size_m > 0.0)) return 0;
  gray_out->assign(static_cast<size_t>(w) * h, options.background_gray);
  // Header over the caller's buffer: fills write in place, no copy.
  cv::Mat img(h, w, CV_8UC1, gray_out->data());

  SimulateVisibleTags(layout, cam, T_field_robot, tag_size_m, detections, true);
  auto& dets = *detections;
  std::sort(dets.begin(), dets.end(), [](const auto& a, const auto& b) {
    return a.T_camera_tag.translation().z() > b.T_camera_tag.translation().z();
  });
  const double half_m = tag_size_m * 0.5;
  int painted = 0;
  for (const SimTagDetection& det : dets) {
    const auto it = patterns.find(det.tag_id);
    if (it == patterns.end()) continue;
    const TagPattern& pattern = it->second;
    for (int gr = 0; gr < 10; ++gr) {
      for (int gc = 0; gc < 10; ++gc) {
        uint8_t color;
        if (gc == 0 || gc == 9 || gr == 0 || gr == 9) {
          // White quiet-zone margin OUTSIDE the quad: the detector's white
          // border model samples there, so it must read bright.
          color = options.white_gray;
        } else if (gc == 1 || gc == 8 || gr == 1 || gr == 8) {
          // Black border ring; the fitted quad is its OUTER edge.
          color = options.black_gray;
        } else if (pattern.bits[gr - 2][gc - 2]) {
          color = options.white_gray;  // Set data bit.
        } else {
          // Unset data bit. (Every interior cell of a real family code is
          // a data bit, so this also covers any non-bit interior cell.)
          color = options.black_gray;
        }
        // Cell corners in tag-frame meters (grid spans ±1.25 tag units in
        // the y/z plane, x=0). Row 0 at the TOP (+Z); column 0 at the LEFT
        // as seen facing the tag face (-Y side: a viewer on the +X side
        // looks back along -X, so screen-right is +Y and the left edge
        // starts at -1.25). Matches the detector's bit sampling through
        // the fitted quad — pinned by the render-to-detect round trip).
        const double y_lo = -1.25 + gc * 0.25;
        const double z_top = 1.25 - gr * 0.25;
        const Eigen::Vector3d corners_m[4] = {
            {0.0, y_lo * half_m, z_top * half_m},
            {0.0, (y_lo + 0.25) * half_m, z_top * half_m},
            {0.0, (y_lo + 0.25) * half_m, (z_top - 0.25) * half_m},
            {0.0, y_lo * half_m, (z_top - 0.25) * half_m}};
        Eigen::Vector2d px[4];
        bool ok = true;
        for (int i = 0; i < 4; ++i) {
          if (!cam.intrinsics.ProjectPoint(det.T_camera_tag.act(corners_m[i]),
                                           &px[i])) {
            ok = false;
            break;
          }
        }
        if (!ok) continue;
        // fillConvexPoly takes integer points; rounding matches the old
        // scanline filler's pixel coverage to the pixel.
        const std::array<cv::Point, 4> poly = {
            {{cvRound(px[0].x()), cvRound(px[0].y())},
             {cvRound(px[1].x()), cvRound(px[1].y())},
             {cvRound(px[2].x()), cvRound(px[2].y())},
             {cvRound(px[3].x()), cvRound(px[3].y())}}};
        cv::fillConvexPoly(img, poly.data(), 4, cv::Scalar(color));
      }
    }
    ++painted;
  }
  return painted;
}

// Integrate a global-shutter exposure centered on T_field_robot. Translation
// is constant in field coordinates; yaw rotates about the robot origin, not
// the camera. Composing the mounting at each instant therefore includes the
// camera's tangential velocity from its mounting lever arm.
inline int RenderFieldView(
    const FieldLayout& layout, const std::map<int, TagPattern>& patterns,
    const SimCameraView& cam, const talos::geometry::SE3<double>& T_field_robot,
    double tag_size_m, const SimRenderOptions& options,
    std::vector<uint8_t>* gray_out,
    const Eigen::Vector3d& robot_velocity_field_mps = Eigen::Vector3d::Zero(),
    double yaw_rate_radps = 0.0, std::mt19937_64* rng = nullptr,
    RenderScratch* scratch = nullptr) {
  if (gray_out == nullptr) return 0;
  if (cam.intrinsics.image_width <= 0 || cam.intrinsics.image_height <= 0 ||
      !(tag_size_m > 0.0)) {
    return 0;
  }
  const double exposure_s = std::max(0.0, cam.exposure_ms) * 0.001;
  const bool moving =
      exposure_s > 0.0 &&
      (robot_velocity_field_mps.squaredNorm() > 0.0 || yaw_rate_radps != 0.0);
  // Static rendering remains usable without scratch; motion integration
  // requires caller-owned storage rather than allocating frame temporaries.
  if (moving && scratch == nullptr) return 0;
  RenderScratch local_scratch;
  if (scratch == nullptr) scratch = &local_scratch;
  int painted = 0;
  if (!moving) {
    painted = RenderInstantaneousFieldView(layout, patterns, cam, T_field_robot,
                                           tag_size_m, options, gray_out,
                                           &scratch->detections);
  } else {
    auto pose_at = [&](double time) {
      return talos::geometry::SE3<double>(
          talos::geometry::SO3<double>::Exp(
              Eigen::Vector3d(0.0, 0.0, yaw_rate_radps * time)) *
              T_field_robot.rotation(),
          T_field_robot.translation() + robot_velocity_field_mps * time);
    };
    // Resolve the exposure at roughly half-pixel intervals. Use projected
    // motion, so depth, focal length and mounting offset all affect sampling.
    SimulateVisibleTags(layout, cam, pose_at(-exposure_s * 0.5), tag_size_m,
                        &scratch->start, true);
    SimulateVisibleTags(layout, cam, pose_at(exposure_s * 0.5), tag_size_m,
                        &scratch->end, true);
    double displacement_px = 0.0;
    for (const auto& a : scratch->start) {
      for (const auto& b : scratch->end) {
        if (a.tag_id != b.tag_id) continue;
        for (int i = 0; i < 4; ++i) {
          displacement_px =
              std::max(displacement_px, (a.corners[i] - b.corners[i]).norm());
        }
      }
    }
    if (displacement_px < 0.5) {
      painted = RenderInstantaneousFieldView(layout, patterns, cam,
                                             T_field_robot, tag_size_m, options,
                                             gray_out, &scratch->detections);
    } else {
      const int samples = static_cast<int>(
          std::clamp(std::ceil(displacement_px * 2.0), 2.0, 64.0));
      auto& instant = scratch->instant;
      auto& accumulated = scratch->accumulated;
      accumulated.create(cam.intrinsics.image_height,
                         cam.intrinsics.image_width, CV_32FC1);
      accumulated.setTo(0);
      for (int sample = 0; sample < samples; ++sample) {
        const double time = ((sample + 0.5) / samples - 0.5) * exposure_s;
        painted = std::max(painted,
                           RenderInstantaneousFieldView(
                               layout, patterns, cam, pose_at(time), tag_size_m,
                               options, &instant, &scratch->detections));
        const cv::Mat sample_image(cam.intrinsics.image_height,
                                   cam.intrinsics.image_width, CV_8UC1,
                                   instant.data());
        cv::add(accumulated, sample_image, accumulated, cv::noArray(), CV_32F);
      }
      gray_out->resize(static_cast<size_t>(accumulated.rows) *
                       accumulated.cols);
      cv::Mat output(accumulated.rows, accumulated.cols, CV_8UC1,
                     gray_out->data());
      accumulated.convertTo(output, CV_8U, 1.0 / samples);
    }
  }
  // Exposure varies coherently across the frame; read noise is independent
  // for every sensor pixel and added AFTER exposure integration and gain.
  double gain = 1.0;
  if (rng != nullptr && cam.exposure_gain_std > 0.0) {
    std::normal_distribution<double> distribution(1.0, cam.exposure_gain_std);
    gain = std::max(0.0, distribution(*rng));
  }
  cv::Mat output(cam.intrinsics.image_height, cam.intrinsics.image_width,
                 CV_8UC1, gray_out->data());
  if (rng != nullptr && cam.sensor_noise_std_gray > 0.0) {
    scratch->sensor_noise.create(output.size(), CV_32FC1);
    // Local OpenCV state avoids its thread-global RNG and derives each frame's
    // deterministic sensor stream from the camera's explicit generator.
    cv::RNG sensor_rng((*rng)());
    sensor_rng.fill(scratch->sensor_noise, cv::RNG::NORMAL, 0.0,
                    cam.sensor_noise_std_gray);
    output.convertTo(scratch->exposed, CV_32F, gain);
    cv::add(scratch->exposed, scratch->sensor_noise, scratch->exposed);
    scratch->exposed.convertTo(output, CV_8U);
  } else if (gain != 1.0) {
    output.convertTo(output, CV_8U, gain);
  }
  return painted;
}

}  // namespace talos::vision::field_sim
