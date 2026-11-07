#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "apriltag.h"
#include "common/image_u8.h"
#include "common/zarray.h"

/**
 * Executes steps 2..4 of apriltag_quad_thresh (connected_components,
 * gradient_clusters, fit_quads) using an externally provided threshold image
 * (threshim).
 *
 * This bypasses the expensive CPU threshold step (~1,650 us) by taking a
 * GPU-generated or precomputed threshold image and continuing the pipeline
 * directly into connected components, gradient clustering, and quad extraction.
 *
 * @param td AprilTag detector configuration and state
 * @param im Grayscale image used for quad edge fitting (width and height must
 * match threshim)
 * @param threshim Binary edge/threshold image (width and height must match im)
 *
 * @return zarray_t* of struct quad elements. Caller owns the returned array and
 * its quads.
 *
 * Note: Caller retains ownership of im and threshim (neither is deallocated by
 * this function). Appropriate timeprofile stamps ("threshold", "unionfind",
 * "make clusters", "fit quads to clusters") are recorded into td->tp.
 */
zarray_t* apriltag_quad_thresh_from_threshim(apriltag_detector_t* td,
                                             image_u8_t* im,
                                             image_u8_t* threshim);

/**
 * Same as apriltag_quad_thresh_from_threshim, but reuses GPU-produced
 * connected-components labels instead of running CPU connected_components.
 *
 * @param gpu_parents Flattened label for each of the w*h pixels of threshim
 *   (canonical roots, 0xFFFFFFFF for ignored pixels), e.g. from the ccl_flatten
 *   Metal kernel.
 * @param gpu_sizes Per-pixel set sizes counting members INCLUDING the root,
 *   e.g. from the ccl_compute_sizes Metal kernel.
 *
 * If either label buffer is NULL, falls back to CPU connected_components.
 * Caller retains ownership of all inputs.
 */
zarray_t* apriltag_quad_thresh_from_gpu_labels(apriltag_detector_t* td,
                                               image_u8_t* im,
                                               image_u8_t* threshim,
                                               const uint32_t* gpu_parents,
                                               const uint32_t* gpu_sizes);

/**
 * Executes the AprilTag detection flow, using an externally provided threshim
 * to bypass CPU thresholding.
 *
 * If threshim is NULL, falls back to standard apriltag_detector_detect(td,
 * im_orig). If threshim is provided:
 *   - Runs apriltag_quad_thresh_from_threshim
 *   - Scales quad coordinates if decimation was applied
 *   - Decodes tags from each quad using im_orig
 *   - Performs duplicate suppression (reconcile)
 *
 * @param td AprilTag detector configuration and state
 * @param im_orig Original input grayscale image (used for decodes and full-res
 * homographies)
 * @param threshim Intermediate thresholded binary image (either matching
 * im_orig or decimated resolution)
 *
 * @return zarray_t* of apriltag_detection_t* (same contract as
 * apriltag_detector_detect).
 */
zarray_t* apriltag_detector_detect_with_threshim(apriltag_detector_t* td,
                                                 image_u8_t* im_orig,
                                                 image_u8_t* threshim);

/**
 * Same as apriltag_detector_detect_with_threshim, but reuses GPU-produced
 * connected-components labels (see apriltag_quad_thresh_from_gpu_labels for
 * the buffer contract) instead of running CPU connected_components.
 *
 * If threshim is NULL, falls back to standard apriltag_detector_detect.
 * If the label buffers are NULL (or dims mismatch), falls back to the
 * threshim path with CPU connected_components.
 */
zarray_t* apriltag_detector_detect_with_gpu_labels(apriltag_detector_t* td,
                                                   image_u8_t* im_orig,
                                                   image_u8_t* threshim,
                                                   const uint32_t* gpu_parents,
                                                   const uint32_t* gpu_sizes);

/**
 * Runs only Step 4 (fit quads) on externally provided clusters, e.g. from
 * GPU candidate extraction + sparse clustering.
 *
 * @param clusters zarray_t* of zarray_t* of struct pt (upstream 12-byte
 *   layout {x, y, gx, gy, slope} with 2x coords). Ownership transfers to this
 *   function, which destroys the array (even on graceful failure paths).
 */
zarray_t* apriltag_quad_thresh_from_gpu_clusters(apriltag_detector_t* td,
                                                 image_u8_t* im,
                                                 zarray_t* clusters);

/**
 * Same as apriltag_detector_detect_with_threshim, but skips thresholding,
 * connected components, AND gradient clustering, fitting quads directly to
 * externally provided clusters (see apriltag_quad_thresh_from_gpu_clusters
 * for the layout/ownership contract). Falls back to the threshim path when
 * clusters is NULL or dims mismatch (taking ownership either way).
 */
zarray_t* apriltag_detector_detect_with_gpu_clusters(apriltag_detector_t* td,
                                                     image_u8_t* im_orig,
                                                     image_u8_t* threshim,
                                                     zarray_t* gpu_clusters);

/**
 * Helper to compute the CPU threshold image using upstream threshold()
 * function. Used for testing and validation of parity between CPU and GPU
 * thresholding paths. Caller owns the returned image_u8_t* and must free it
 * with image_u8_destroy().
 */
image_u8_t* apriltag_threshold_cpu(apriltag_detector_t* td, image_u8_t* im);

#ifdef __cplusplus
}
#endif
