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
//   - flat shading only: no lighting falloff, lens blur, motion blur, sensor
//     noise, or occlusion by field structures/robots;
//   - the background defaults to white, not black: the detector's white
//     border model samples OUTSIDE the fitted quad, so a black void would
//     starve it. Matches the repo's proven synthetic renderer;
//   - image-edge clipping is supported, but a tag crossing the camera's near
//     plane is culled; tag overlap uses far-to-near painter ordering.

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <opencv2/imgproc.hpp>
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

// Renders every projectable layout tag with a known pattern into gray_out
// (resized to w*h, row-major, stride w). Tags without a pattern entry are
// skipped. Returns the number of tags rasterized.
inline int RenderFieldView(const FieldLayout& layout,
                           const std::map<int, TagPattern>& patterns,
                           const SimCameraView& cam,
                           const talos::geometry::SE3<double>& T_field_robot,
                           double tag_size_m, const SimRenderOptions& options,
                           std::vector<uint8_t>* gray_out) {
  if (gray_out == nullptr) return 0;
  const int w = static_cast<int>(cam.intrinsics.image_width);
  const int h = static_cast<int>(cam.intrinsics.image_height);
  if (w <= 0 || h <= 0 || !(tag_size_m > 0.0)) return 0;
  gray_out->assign(static_cast<size_t>(w) * h, options.background_gray);
  // Header over the caller's buffer: fills write in place, no copy.
  cv::Mat img(h, w, CV_8UC1, gray_out->data());

  std::vector<SimTagDetection> dets =
      SimulateVisibleTags(layout, cam, T_field_robot, tag_size_m, true);
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
        const std::vector<cv::Point> poly = {
            {cvRound(px[0].x()), cvRound(px[0].y())},
            {cvRound(px[1].x()), cvRound(px[1].y())},
            {cvRound(px[2].x()), cvRound(px[2].y())},
            {cvRound(px[3].x()), cvRound(px[3].y())}};
        cv::fillConvexPoly(img, poly, cv::Scalar(color));
      }
    }
    ++painted;
  }
  return painted;
}

}  // namespace talos::vision::field_sim
