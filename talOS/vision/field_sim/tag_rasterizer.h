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
  std::vector<cv::Rect> regions;
  cv::Mat accumulated;
  cv::Mat sensor_noise;
  cv::Mat exposed;
};

// Paint projected tags without initializing the background. Exposure
// integration clears only its active regions; the static renderer clears the
// whole frame.
inline int RasterizeProjectedTags(const std::map<int, TagPattern>& patterns,
                                  const SimCameraView& cam, double tag_size_m,
                                  const SimRenderOptions& options, cv::Mat& img,
                                  std::vector<SimTagDetection>* detections) {
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
    // Adjacent cells share vertices: project each lattice point once instead
    // of repeating four SE(3)/distortion evaluations for every one of 100
    // cells.
    std::array<cv::Point, 121> vertices;
    std::array<bool, 121> valid;
    for (int row = 0; row <= 10; ++row) {
      for (int col = 0; col <= 10; ++col) {
        const auto index = row * 11 + col;
        const Eigen::Vector3d corner{0, (-1.25 + col * 0.25) * half_m,
                                     (1.25 - row * 0.25) * half_m};
        Eigen::Vector2d pixel;
        valid[index] =
            cam.intrinsics.ProjectPoint(det.T_camera_tag.act(corner), &pixel);
        if (valid[index])
          vertices[index] = {cvRound(pixel.x()), cvRound(pixel.y())};
      }
    }
    for (int gr = 0; gr < 10; ++gr) {
      for (int gc = 0; gc < 10; ++gc) {
        // White quiet-zone margin OUTSIDE the quad: the detector's white
        // border model samples there, so it must read bright.
        const bool quiet_zone = gc == 0 || gc == 9 || gr == 0 || gr == 9;
        // Black border ring; the fitted quad is its OUTER edge.
        const bool border = gc == 1 || gc == 8 || gr == 1 || gr == 8;
        // Every interior cell of a real family code is a data bit: set bits
        // are white, unset bits (and any non-bit interior cell) are black.
        const bool white =
            quiet_zone || (!border && pattern.bits[gr - 2][gc - 2]);
        const uint8_t color = white ? options.white_gray : options.black_gray;
        // The lattice spans ±1.25 tag units in the tag's y/z plane:
        // row zero is top (+Z), column zero is left (-Y as seen by the lens).
        const int index = gr * 11 + gc;
        if (!valid[index] || !valid[index + 1] || !valid[index + 12] ||
            !valid[index + 11])
          continue;
        const std::array<cv::Point, 4> poly = {
            vertices[index], vertices[index + 1], vertices[index + 12],
            vertices[index + 11]};
        cv::fillConvexPoly(img, poly.data(), 4, cv::Scalar(color));
      }
    }
    ++painted;
  }
  return painted;
}

// Renders every projectable layout tag with a known pattern into gray_out
// (resized to w*h, row-major, stride w). Returns the number of tags rasterized.
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
  cv::Mat image(h, w, CV_8UC1, gray_out->data());
  SimulateVisibleTags(layout, cam, T_field_robot, tag_size_m, detections, true);
  return RasterizeProjectedTags(patterns, cam, tag_size_m, options, image,
                                detections);
}

// Maintain disjoint rectangles covering the union. Overlapping rectangles
// merge, so a pixel shared by two tags is accumulated exactly once per sample.
inline void IncludeExposureRegion(cv::Rect region,
                                  std::vector<cv::Rect>* regions) {
  if (region.empty()) return;
  for (std::size_t i = 0; i < regions->size();) {
    if ((region & (*regions)[i]).empty()) {
      ++i;
    } else {
      region |= (*regions)[i];
      (*regions)[i] = regions->back();
      regions->pop_back();
      i = 0;
    }
  }
  regions->push_back(region);
}

inline void IncludeExposureTags(const std::vector<SimTagDetection>& detections,
                                const std::map<int, TagPattern>& patterns,
                                const SimCameraView& cam, double tag_size_m,
                                std::vector<cv::Rect>* regions) {
  // The fitted quad excludes the white quiet zone, which extends to 1.25
  // times its size. Project that margin rather than scaling a perspective bbox.
  const auto corners = GetTagModelCorners(tag_size_m * 1.25);
  for (const auto& detection : detections) {
    if (!patterns.contains(detection.tag_id)) continue;
    double xmin = cam.intrinsics.image_width,
           ymin = cam.intrinsics.image_height;
    double xmax = 0, ymax = 0;
    bool projected = true;
    for (const auto& corner : corners) {
      Eigen::Vector2d pixel;
      if (!cam.intrinsics.ProjectPoint(detection.T_camera_tag.act(corner),
                                       &pixel)) {
        projected = false;
        break;
      }
      xmin = std::min(xmin, pixel.x());
      xmax = std::max(xmax, pixel.x());
      ymin = std::min(ymin, pixel.y());
      ymax = std::max(ymax, pixel.y());
    }
    // A quiet-zone corner can cross the near plane while inner cells still
    // project. In that degenerate case the tag may cover the whole image.
    if (!projected) {
      IncludeExposureRegion(
          cv::Rect(0, 0, static_cast<int>(cam.intrinsics.image_width),
                   static_cast<int>(cam.intrinsics.image_height)),
          regions);
      continue;
    }
    // Cover integer polygon rounding and the rasterizer's boundary pixels.
    const int left = static_cast<int>(
        std::clamp(std::floor(xmin) - 2, 0.0,
                   static_cast<double>(cam.intrinsics.image_width)));
    const int top = static_cast<int>(
        std::clamp(std::floor(ymin) - 2, 0.0,
                   static_cast<double>(cam.intrinsics.image_height)));
    const int right = static_cast<int>(
        std::clamp(std::ceil(xmax) + 3, 0.0,
                   static_cast<double>(cam.intrinsics.image_width)));
    const int bottom = static_cast<int>(
        std::clamp(std::ceil(ymax) + 3, 0.0,
                   static_cast<double>(cam.intrinsics.image_height)));
    IncludeExposureRegion(cv::Rect(left, top, right - left, bottom - top),
                          regions);
  }
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
  const int width = static_cast<int>(cam.intrinsics.image_width);
  const int height = static_cast<int>(cam.intrinsics.image_height);
  if (width <= 0 || height <= 0 || !(tag_size_m > 0.0)) {
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
      auto& regions = scratch->regions;
      regions.clear();
      IncludeExposureTags(scratch->start, patterns, cam, tag_size_m, &regions);
      IncludeExposureTags(scratch->end, patterns, cam, tag_size_m, &regions);
      auto& instant = scratch->instant;
      instant.resize(static_cast<size_t>(height) * width);
      cv::Mat sample_image(height, width, CV_8UC1, instant.data());
      auto& accumulated = scratch->accumulated;
      accumulated.create(sample_image.size(), CV_32FC1);
      accumulated.setTo(0);
      for (int sample = 0; sample < samples; ++sample) {
        const double time = ((sample + 0.5) / samples - 0.5) * exposure_s;
        SimulateVisibleTags(layout, cam, pose_at(time), tag_size_m,
                            &scratch->detections, true);
        // Endpoints bound ordinary short exposures. Extend for tags entering
        // view or curved projected trajectories, preserving exact coverage.
        IncludeExposureTags(scratch->detections, patterns, cam, tag_size_m,
                            &regions);
        for (const auto& region : regions)
          sample_image(region).setTo(options.background_gray);
        painted = std::max(painted, RasterizeProjectedTags(
                                        patterns, cam, tag_size_m, options,
                                        sample_image, &scratch->detections));
        for (const auto& region : regions) {
          cv::Mat sum = accumulated(region);
          cv::addWeighted(sample_image(region), 1, sum, 1,
                          -static_cast<double>(options.background_gray), sum,
                          CV_32F);
        }
      }
      gray_out->resize(instant.size());
      cv::Mat output(sample_image.size(), CV_8UC1, gray_out->data());
      // The static background contributes once; only tag-region deviations
      // were integrated. Untouched pixels therefore remain the static frame.
      accumulated.convertTo(output, CV_8U, 1.0 / samples,
                            options.background_gray);
    }
  }
  // Exposure varies coherently across the frame; read noise is independent
  // for every sensor pixel and added AFTER exposure integration and gain.
  double gain = 1.0;
  if (rng != nullptr && cam.exposure_gain_std > 0.0) {
    std::normal_distribution<double> distribution(1.0, cam.exposure_gain_std);
    gain = std::max(0.0, distribution(*rng));
  }
  cv::Mat output(height, width, CV_8UC1, gray_out->data());
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
