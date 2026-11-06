#pragma once

// C++ and C compatible (declaration uses C linkage and C types only).

#include <stdint.h>

#include "common/image_u8.h"
#include "common/zarray.h"

#ifdef __cplusplus
extern "C" {
#endif

// Point layout identical to upstream `struct pt` in apriltag_quad_thresh.c:
// 2x-scale coords, gradient, and slope written later by fit_quads. Must stay
// 12 bytes; fit_quads reads elements as its own struct pt.
typedef struct gpu_cluster_point {
  uint16_t x, y;
  int16_t gx, gy;
  float slope;
} gpu_cluster_point_t;

// Builds AprilTag gradient clusters from GPU connected-components labels and
// GPU edge candidates WITHOUT scanning all w*h pixels on CPU: only the
// candidate_coords entries are evaluated (typically <5% of pixels).
//
// Mirrors do_gradient_clusters semantics: transitions with v0+v1 == 255,
// set sizes >= 25, connected-last 8-connectivity gating, (rep0, rep1) hashing.
//
// @param gpu_parents Flattened canonical roots per pixel, 0xFFFFFFFF for
//   ignored pixels (ccl_flatten output).
// @param gpu_sizes Per-pixel set sizes counting members INCLUDING the root
//   (ccl_compute_sizes output). Indexed by root id.
// @param candidate_coords Packed (flags, y, x) entries (extract_edge_candidates
//   output). Entries outside [0, w) x [0, h) are ignored.
// @return zarray_t* of zarray_t* of gpu_cluster_point_t. Caller owns the
//   array; it can be transferred to apriltag_quad_thresh_from_gpu_clusters,
//   which takes ownership.
zarray_t* BuildSparseClustersFromGpuLabels(const image_u8_t* threshim, int w,
                                           int h, int ts,
                                           const uint32_t* gpu_parents,
                                           const uint32_t* gpu_sizes,
                                           const uint32_t* candidate_coords,
                                           uint32_t candidate_count);

#ifdef __cplusplus
}
#endif
