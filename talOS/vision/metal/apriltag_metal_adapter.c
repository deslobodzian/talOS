// NOLINTBEGIN
// Derived from upstream AprilTag C; retain upstream style for diffing.
#include "talOS/vision/metal/apriltag_metal_adapter.h"

#define _USE_MATH_DEFINES
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "apriltag.h"
#include "apriltag_math.h"
#include "common/debug_print.h"
#include "common/g2d.h"
#include "common/homography.h"
#include "common/image_u8.h"
#include "common/image_u8_parallel.h"
#include "common/image_u8x3.h"
#include "common/matd.h"
#include "common/math_util.h"
#include "common/postscript_utils.h"
#include "common/timeprofile.h"
#include "common/unionfind.h"
#include "common/workerpool.h"
#include "common/zarray.h"

#ifdef _WIN32
static inline void srandom(unsigned int seed) { srand(seed); }
static inline long int random(void) { return rand(); }
#endif

#define APRILTAG_U64_ONE ((uint64_t)1)

// External functions with external C linkage from apriltag_quad_thresh.c
extern unionfind_t* connected_components(apriltag_detector_t* td,
                                         image_u8_t* threshim, int w, int h,
                                         int ts);
extern zarray_t* gradient_clusters(apriltag_detector_t* td,
                                   image_u8_t* threshim, int w, int h, int ts,
                                   unionfind_t* uf);
extern zarray_t* fit_quads(apriltag_detector_t* td, int w, int h,
                           zarray_t* clusters, image_u8_t* im);
extern image_u8_t* threshold(apriltag_detector_t* td, image_u8_t* im);

// Internal point structure matching apriltag_quad_thresh.c for debug visualizer
struct pt {
  uint16_t x, y;
  int16_t gx, gy;
  float slope;
};

// Internal quick_decode structures matching apriltag.c for decoding
struct quick_decode_entry {
  uint64_t rcode;
  uint16_t id;
  uint8_t hamming;
  uint8_t rotation;
};

struct quick_decode {
  int nentries;
  struct quick_decode_entry* entries;
};

struct quad_decode_task {
  int i0, i1;
  zarray_t* quads;
  apriltag_detector_t* td;
  image_u8_t* im;
  zarray_t* detections;
  image_u8_t* im_samples;
};

struct graymodel {
  double A[3][3];
  double B[3];
  double C[3];
};

static void graymodel_init(struct graymodel* gm) {
  memset(gm, 0, sizeof(struct graymodel));
}

static void graymodel_add(struct graymodel* gm, double x, double y,
                          double gray) {
  gm->A[0][0] += x * x;
  gm->A[0][1] += x * y;
  gm->A[0][2] += x;
  gm->A[1][1] += y * y;
  gm->A[1][2] += y;
  gm->A[2][2] += 1;

  gm->B[0] += x * gray;
  gm->B[1] += y * gray;
  gm->B[2] += gray;
}

static void graymodel_solve(struct graymodel* gm) {
  mat33_sym_solve((double*)gm->A, gm->B, gm->C);
}

static double graymodel_interpolate(struct graymodel* gm, double x, double y) {
  return gm->C[0] * x + gm->C[1] * y + gm->C[2];
}

static uint64_t rotate90(uint64_t w, int numBits) {
  int p = numBits;
  uint64_t l = 0;
  if (numBits % 4 == 1) {
    p = numBits - 1;
    l = 1;
  }
  w = ((w >> l) << (p / 4 + l)) | (w >> (3 * p / 4 + l) << l) | (w & l);
  w &= ((APRILTAG_U64_ONE << numBits) - 1);
  return w;
}

static void quad_destroy(struct quad* quad) {
  if (!quad) return;
  matd_destroy(quad->H);
  matd_destroy(quad->Hinv);
  free(quad);
}

static struct quad* quad_copy(struct quad* quad) {
  struct quad* q = calloc(1, sizeof(struct quad));
  memcpy(q, quad, sizeof(struct quad));
  if (quad->H) q->H = matd_copy(quad->H);
  if (quad->Hinv) q->Hinv = matd_copy(quad->Hinv);
  return q;
}

static void quick_decode_codeword(apriltag_family_t* tf, uint64_t rcode,
                                  struct quick_decode_entry* entry) {
  struct quick_decode* qd = (struct quick_decode*)tf->impl;

  for (int ridx = 0; qd != NULL && ridx < 4; ridx++) {
    for (int bucket = rcode % qd->nentries;
         qd->entries[bucket].rcode != UINT64_MAX;
         bucket = (bucket + 1) % qd->nentries) {
      if (qd->entries[bucket].rcode == rcode) {
        *entry = qd->entries[bucket];
        entry->rotation = ridx;
        return;
      }
    }
    rcode = rotate90(rcode, tf->nbits);
  }

  entry->rcode = 0;
  entry->id = 65535;
  entry->hamming = 255;
  entry->rotation = 0;
}

static matd_t* homography_compute2(double c[4][4]) {
  double A[] = {
      c[0][0],
      c[0][1],
      1,
      0,
      0,
      0,
      -c[0][0] * c[0][2],
      -c[0][1] * c[0][2],
      c[0][2],
      0,
      0,
      0,
      c[0][0],
      c[0][1],
      1,
      -c[0][0] * c[0][3],
      -c[0][1] * c[0][3],
      c[0][3],
      c[1][0],
      c[1][1],
      1,
      0,
      0,
      0,
      -c[1][0] * c[1][2],
      -c[1][1] * c[1][2],
      c[1][2],
      0,
      0,
      0,
      c[1][0],
      c[1][1],
      1,
      -c[1][0] * c[1][3],
      -c[1][1] * c[1][3],
      c[1][3],
      c[2][0],
      c[2][1],
      1,
      0,
      0,
      0,
      -c[2][0] * c[2][2],
      -c[2][1] * c[2][2],
      c[2][2],
      0,
      0,
      0,
      c[2][0],
      c[2][1],
      1,
      -c[2][0] * c[2][3],
      -c[2][1] * c[2][3],
      c[2][3],
      c[3][0],
      c[3][1],
      1,
      0,
      0,
      0,
      -c[3][0] * c[3][2],
      -c[3][1] * c[3][2],
      c[3][2],
      0,
      0,
      0,
      c[3][0],
      c[3][1],
      1,
      -c[3][0] * c[3][3],
      -c[3][1] * c[3][3],
      c[3][3],
  };

  double epsilon = 1e-10;

  for (int col = 0; col < 8; col++) {
    double max_val = 0;
    int max_val_idx = -1;
    for (int row = col; row < 8; row++) {
      double val = fabs(A[row * 9 + col]);
      if (val > max_val) {
        max_val = val;
        max_val_idx = row;
      }
    }

    if (max_val_idx < 0) return NULL;
    if (max_val < epsilon) return NULL;

    if (max_val_idx != col) {
      for (int i = col; i < 9; i++) {
        double tmp = A[col * 9 + i];
        A[col * 9 + i] = A[max_val_idx * 9 + i];
        A[max_val_idx * 9 + i] = tmp;
      }
    }

    for (int i = col + 1; i < 8; i++) {
      double f = A[i * 9 + col] / A[col * 9 + col];
      A[i * 9 + col] = 0;
      for (int j = col + 1; j < 9; j++) {
        A[i * 9 + j] -= f * A[col * 9 + j];
      }
    }
  }

  for (int col = 7; col >= 0; col--) {
    double sum = 0;
    for (int i = col + 1; i < 8; i++) {
      sum += A[col * 9 + i] * A[i * 9 + 8];
    }
    A[col * 9 + 8] = (A[col * 9 + 8] - sum) / A[col * 9 + col];
  }
  return matd_create_data(
      3, 3,
      (double[]){A[8], A[17], A[26], A[35], A[44], A[53], A[62], A[71], 1});
}

static int quad_update_homographies(struct quad* quad) {
  double corr_arr[4][4];
  for (int i = 0; i < 4; i++) {
    corr_arr[i][0] = (i == 0 || i == 3) ? -1 : 1;
    corr_arr[i][1] = (i == 0 || i == 1) ? -1 : 1;
    corr_arr[i][2] = quad->p[i][0];
    corr_arr[i][3] = quad->p[i][1];
  }

  if (quad->H) matd_destroy(quad->H);
  if (quad->Hinv) matd_destroy(quad->Hinv);
  quad->Hinv = NULL;

  quad->H = homography_compute2(corr_arr);
  if (quad->H != NULL) {
    quad->Hinv = matd_inverse(quad->H);
    if (quad->Hinv != NULL) {
      return 0;
    }
    matd_destroy(quad->H);
    quad->H = NULL;
  }
  return -1;
}

static double value_for_pixel(image_u8_t* im, double px, double py) {
  int x1 = floor(px - 0.5);
  int x2 = ceil(px - 0.5);
  double x = px - 0.5 - x1;
  int y1 = floor(py - 0.5);
  int y2 = ceil(py - 0.5);
  double y = py - 0.5 - y1;
  if (x1 < 0 || x2 >= im->width || y1 < 0 || y2 >= im->height) {
    return -1;
  }
  return im->buf[y1 * im->stride + x1] * (1 - x) * (1 - y) +
         im->buf[y1 * im->stride + x2] * x * (1 - y) +
         im->buf[y2 * im->stride + x1] * (1 - x) * y +
         im->buf[y2 * im->stride + x2] * x * y;
}

static void sharpen(apriltag_detector_t* td, double* values, int size) {
  double* sharpened = malloc(sizeof(double) * size * size);
  double kernel[9] = {0, -1, 0, -1, 4, -1, 0, -1, 0};

  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      sharpened[y * size + x] = 0;
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          if ((y + i - 1) < 0 || (y + i - 1) > size - 1 || (x + j - 1) < 0 ||
              (x + j - 1) > size - 1) {
            continue;
          }
          sharpened[y * size + x] +=
              values[(y + i - 1) * size + (x + j - 1)] * kernel[i * 3 + j];
        }
      }
    }
  }

  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      values[y * size + x] = values[y * size + x] +
                             td->decode_sharpening * sharpened[y * size + x];
    }
  }

  free(sharpened);
}

static float quad_decode(apriltag_detector_t* td, apriltag_family_t* family,
                         image_u8_t* im, struct quad* quad,
                         struct quick_decode_entry* entry,
                         image_u8_t* im_samples) {
  float patterns[] = {// left white column
                      -0.5, 0.5, 0, 1, 1,
                      // left black column
                      0.5, 0.5, 0, 1, 0,
                      // right white column
                      family->width_at_border + 0.5, 0.5, 0, 1, 1,
                      // right black column
                      family->width_at_border - 0.5, 0.5, 0, 1, 0,
                      // top white row
                      0.5, -0.5, 1, 0, 1,
                      // top black row
                      0.5, 0.5, 1, 0, 0,
                      // bottom white row
                      0.5, family->width_at_border + 0.5, 1, 0, 1,
                      // bottom black row
                      0.5, family->width_at_border - 0.5, 1, 0, 0};

  struct graymodel whitemodel, blackmodel;
  graymodel_init(&whitemodel);
  graymodel_init(&blackmodel);

  for (size_t pattern_idx = 0;
       pattern_idx < sizeof(patterns) / (5 * sizeof(float)); pattern_idx++) {
    float* pattern = &patterns[pattern_idx * 5];
    int is_white = pattern[4];

    for (int i = 0; i < family->width_at_border; i++) {
      double tagx01 = (pattern[0] + i * pattern[2]) / (family->width_at_border);
      double tagy01 = (pattern[1] + i * pattern[3]) / (family->width_at_border);
      double tagx = 2 * (tagx01 - 0.5);
      double tagy = 2 * (tagy01 - 0.5);

      double px, py;
      homography_project(quad->H, tagx, tagy, &px, &py);

      int ix = px;
      int iy = py;
      if (ix < 0 || iy < 0 || ix >= im->width || iy >= im->height) continue;

      int v = im->buf[iy * im->stride + ix];
      if (im_samples) {
        im_samples->buf[iy * im_samples->stride + ix] = (1 - is_white) * 255;
      }

      if (is_white)
        graymodel_add(&whitemodel, tagx, tagy, v);
      else
        graymodel_add(&blackmodel, tagx, tagy, v);
    }
  }

  if (family->width_at_border > 1) {
    graymodel_solve(&whitemodel);
    graymodel_solve(&blackmodel);
  } else {
    graymodel_solve(&whitemodel);
    blackmodel.C[0] = 0;
    blackmodel.C[1] = 0;
    blackmodel.C[2] = blackmodel.B[2] / 4;
  }

  if ((graymodel_interpolate(&whitemodel, 0, 0) -
           graymodel_interpolate(&blackmodel, 0, 0) <
       0) != family->reversed_border) {
    return -1;
  }

  float black_score = 0, white_score = 0;
  float black_score_count = 1, white_score_count = 1;

  double* values =
      calloc(family->total_width * family->total_width, sizeof(double));
  int min_coord = (family->width_at_border - family->total_width) / 2;

  for (uint32_t i = 0; i < family->nbits; i++) {
    int bity = family->bit_y[i];
    int bitx = family->bit_x[i];

    double tagx01 = (bitx + 0.5) / (family->width_at_border);
    double tagy01 = (bity + 0.5) / (family->width_at_border);
    double tagx = 2 * (tagx01 - 0.5);
    double tagy = 2 * (tagy01 - 0.5);

    double px, py;
    homography_project(quad->H, tagx, tagy, &px, &py);

    double v = value_for_pixel(im, px, py);
    if (v == -1) continue;

    double thresh = (graymodel_interpolate(&blackmodel, tagx, tagy) +
                     graymodel_interpolate(&whitemodel, tagx, tagy)) /
                    2.0;
    values[family->total_width * (bity - min_coord) + bitx - min_coord] =
        v - thresh;

    if (im_samples) {
      int ix = px;
      int iy = py;
      im_samples->buf[iy * im_samples->stride + ix] = (v < thresh) * 255;
    }
  }

  sharpen(td, values, family->total_width);

  uint64_t rcode = 0;
  for (uint32_t i = 0; i < family->nbits; i++) {
    int bity = family->bit_y[i];
    int bitx = family->bit_x[i];
    rcode = (rcode << 1);
    double v =
        values[(bity - min_coord) * family->total_width + bitx - min_coord];

    if (v > 0) {
      white_score += v;
      white_score_count++;
      rcode |= 1;
    } else {
      black_score -= v;
      black_score_count++;
    }
  }

  quick_decode_codeword(family, rcode, entry);
  free(values);
  return fmin(white_score / white_score_count, black_score / black_score_count);
}

static void refine_edges(apriltag_detector_t* td, image_u8_t* im_orig,
                         struct quad* quad) {
  double lines[4][4];

  for (int edge = 0; edge < 4; edge++) {
    int a = edge, b = (edge + 1) & 3;

    double nx = quad->p[b][1] - quad->p[a][1];
    double ny = -quad->p[b][0] + quad->p[a][0];
    double mag = sqrt(nx * nx + ny * ny);
    nx /= mag;
    ny /= mag;

    if (quad->reversed_border) {
      nx = -nx;
      ny = -ny;
    }

    int nsamples = imax(16, mag / 8);
    double Mx = 0, My = 0, Mxx = 0, Mxy = 0, Myy = 0, N = 0;

    for (int s = 0; s < nsamples; s++) {
      double alpha = (1.0 + s) / (nsamples + 1);
      double x0 = alpha * quad->p[a][0] + (1 - alpha) * quad->p[b][0];
      double y0 = alpha * quad->p[a][1] + (1 - alpha) * quad->p[b][1];

      double Mn = 0;
      double Mcount = 0;
      int range = td->quad_decimate + 1;
      int steps_per_unit = 4;
      double step_length = 1.0 / steps_per_unit;
      int max_steps = 2 * steps_per_unit * range + 1;
      double delta = 0.5;

      for (int step = 0; step < max_steps; ++step) {
        double n = -range + step_length * step;
        double grange = 1;

        double x1 = x0 + (n + grange) * nx - delta;
        double y1 = y0 + (n + grange) * ny - delta;
        double x1i_d, y1i_d, a1, b1;
        a1 = modf(x1, &x1i_d);
        b1 = modf(y1, &y1i_d);
        int x1i = x1i_d, y1i = y1i_d;

        if (x1i < 0 || x1i + 1 >= im_orig->width || y1i < 0 ||
            y1i + 1 >= im_orig->height)
          continue;

        double x2 = x0 + (n - grange) * nx - delta;
        double y2 = y0 + (n - grange) * ny - delta;
        double x2i_d, y2i_d, a2, b2;
        a2 = modf(x2, &x2i_d);
        b2 = modf(y2, &y2i_d);
        int x2i = x2i_d, y2i = y2i_d;

        if (x2i < 0 || x2i + 1 >= im_orig->width || y2i < 0 ||
            y2i + 1 >= im_orig->height)
          continue;

        double g1 =
            (1 - a1) * (1 - b1) * im_orig->buf[y1i * im_orig->stride + x1i] +
            a1 * (1 - b1) * im_orig->buf[y1i * im_orig->stride + x1i + 1] +
            (1 - a1) * b1 * im_orig->buf[(y1i + 1) * im_orig->stride + x1i] +
            a1 * b1 * im_orig->buf[(y1i + 1) * im_orig->stride + x1i + 1];
        double g2 =
            (1 - a2) * (1 - b2) * im_orig->buf[y2i * im_orig->stride + x2i] +
            a2 * (1 - b2) * im_orig->buf[y2i * im_orig->stride + x2i + 1] +
            (1 - a2) * b2 * im_orig->buf[(y2i + 1) * im_orig->stride + x2i] +
            a2 * b2 * im_orig->buf[(y2i + 1) * im_orig->stride + x2i + 1];

        if (g1 < g2) continue;

        double weight = (g2 - g1) * (g2 - g1);
        Mn += weight * n;
        Mcount += weight;
      }

      if (Mcount == 0) continue;

      double n0 = Mn / Mcount;
      double bestx = x0 + n0 * nx;
      double besty = y0 + n0 * ny;

      Mx += bestx;
      My += besty;
      Mxx += bestx * bestx;
      Mxy += bestx * besty;
      Myy += besty * besty;
      N++;
    }

    double Ex = Mx / N, Ey = My / N;
    double Cxx = Mxx / N - Ex * Ex;
    double Cxy = Mxy / N - Ex * Ey;
    double Cyy = Myy / N - Ey * Ey;

    double normal_theta = 0.5 * atan2f(-2 * Cxy, (Cyy - Cxx));
    nx = cosf(normal_theta);
    ny = sinf(normal_theta);
    lines[edge][0] = Ex;
    lines[edge][1] = Ey;
    lines[edge][2] = nx;
    lines[edge][3] = ny;
  }

  for (int i = 0; i < 4; i++) {
    double A00 = lines[i][3], A01 = -lines[(i + 1) & 3][3];
    double A10 = -lines[i][2], A11 = lines[(i + 1) & 3][2];
    double B0 = -lines[i][0] + lines[(i + 1) & 3][0];
    double B1 = -lines[i][1] + lines[(i + 1) & 3][1];

    double det = A00 * A11 - A10 * A01;
    if (fabs(det) > 0.001) {
      double W00 = A11 / det, W01 = -A01 / det;
      double L0 = W00 * B0 + W01 * B1;
      quad->p[(i + 1) & 3][0] = lines[i][0] + L0 * A00;
      quad->p[(i + 1) & 3][1] = lines[i][1] + L0 * A10;
    }
  }
}

static void quad_decode_task(void* _u) {
  struct quad_decode_task* task = (struct quad_decode_task*)_u;
  apriltag_detector_t* td = task->td;
  image_u8_t* im = task->im;

  for (int quadidx = task->i0; quadidx < task->i1; quadidx++) {
    struct quad* quad_original;
    zarray_get_volatile(task->quads, quadidx, &quad_original);

    if (td->refine_edges) {
      refine_edges(td, im, quad_original);
    }

    if (quad_update_homographies(quad_original) != 0) continue;

    for (int famidx = 0; famidx < zarray_size(td->tag_families); famidx++) {
      apriltag_family_t* family;
      zarray_get(td->tag_families, famidx, &family);

      if (family->reversed_border != quad_original->reversed_border) continue;

      struct quad* quad = quad_copy(quad_original);
      struct quick_decode_entry entry;

      float decision_margin =
          quad_decode(td, family, im, quad, &entry, task->im_samples);

      if (decision_margin >= 0 && entry.hamming < 255) {
        apriltag_detection_t* det = calloc(1, sizeof(apriltag_detection_t));
        det->family = family;
        det->id = entry.id;
        det->hamming = entry.hamming;
        det->decision_margin = decision_margin;

        double theta = entry.rotation * M_PI / 2.0;
        double c = cos(theta), s = sin(theta);

        matd_t* R = matd_create(3, 3);
        MATD_EL(R, 0, 0) = c;
        MATD_EL(R, 0, 1) = -s;
        MATD_EL(R, 1, 0) = s;
        MATD_EL(R, 1, 1) = c;
        MATD_EL(R, 2, 2) = 1;

        det->H = matd_op("M*M", quad->H, R);
        matd_destroy(R);

        homography_project(det->H, 0, 0, &det->c[0], &det->c[1]);

        for (int i = 0; i < 4; i++) {
          int tcx = (i == 1 || i == 2) ? 1 : -1;
          int tcy = (i < 2) ? 1 : -1;
          double p[2];
          homography_project(det->H, tcx, tcy, &p[0], &p[1]);
          det->p[i][0] = p[0];
          det->p[i][1] = p[1];
        }

        pthread_mutex_lock(&td->mutex);
        zarray_add(task->detections, &det);
        pthread_mutex_unlock(&td->mutex);
      }

      quad_destroy(quad);
    }
  }
}

static int prefer_smaller(int pref, double q0, double q1) {
  if (pref) return pref;
  if (q0 < q1) return -1;
  if (q1 < q0) return 1;
  return 0;
}

static inline int detection_compare_function(const void* _a, const void* _b) {
  apriltag_detection_t* a = *(apriltag_detection_t**)_a;
  apriltag_detection_t* b = *(apriltag_detection_t**)_b;
  return a->id - b->id;
}

// Lazily creates td->wp like apriltag_detector_detect does. Returns false
// when creation fails, in which case callers fail gracefully.
static bool ensure_workerpool(apriltag_detector_t* td) {
  if (td->wp == NULL || td->nthreads != workerpool_get_nthreads(td->wp)) {
    workerpool_destroy(td->wp);
    td->wp = workerpool_create(td->nthreads);
    if (td->wp == NULL) {
      return false;
    }
  }
  return true;
}

static zarray_t* fit_quads_and_cleanup(apriltag_detector_t* td, int w, int h,
                                       zarray_t* clusters, image_u8_t* im);

// Destroys a clusters array (inner arrays and the outer array).
static void destroy_clusters(zarray_t* clusters) {
  if (clusters == NULL) return;
  for (int i = 0; i < zarray_size(clusters); i++) {
    zarray_t* cluster;
    zarray_get(clusters, i, &cluster);
    zarray_destroy(cluster);
  }
  zarray_destroy(clusters);
}

// Takes ownership of external_uf (destroys it on return); NULL builds the
// union-find via CPU connected_components, mirroring upstream ownership.
static zarray_t* quad_thresh_from_threshim_and_uf(apriltag_detector_t* td,
                                                  image_u8_t* im,
                                                  image_u8_t* threshim,
                                                  unionfind_t* external_uf) {
  assert(td != NULL);
  assert(im != NULL);
  assert(threshim != NULL);
  assert(im->width == threshim->width);
  assert(im->height == threshim->height);

  // threshold()/connected_components()/gradient_clusters()/fit_quads() all
  // dereference td->wp. apriltag_detector_create() leaves wp NULL (deferred
  // until apriltag_detector_detect), so init it lazily like detect does.
  if (!ensure_workerpool(td)) {
    if (external_uf != NULL) unionfind_destroy(external_uf);
    return zarray_create(sizeof(struct quad));
  }

  int w = im->width, h = im->height;
  int ts = threshim->stride;

  if (td->debug) {
    image_u8_write_pnm(threshim, "debug_threshold.pnm");
  }

  // Record threshold completion in the timeprofile
  timeprofile_stamp(td->tp, "threshold");

  ////////////////////////////////////////////////////////
  // Step 2. Find connected components (or reuse externally provided labels,
  // e.g. GPU CCL output converted to unionfind_t; NULL keeps the CPU path).
  unionfind_t* uf = external_uf;
  if (uf == NULL) {
    uf = connected_components(td, threshim, w, h, ts);
  }

  if (td->debug) {
    image_u8x3_t* d = image_u8x3_create(w, h);
    uint32_t* colors = (uint32_t*)calloc(w * h, sizeof(*colors));

    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        uint32_t v = unionfind_get_representative(uf, y * w + x);
        if ((int)unionfind_get_set_size(uf, v) < td->qtp.min_cluster_pixels)
          continue;

        uint32_t color = colors[v];
        uint8_t r = color >> 16, g = color >> 8, b = color;

        if (color == 0) {
          const int bias = 50;
          r = bias + (random() % (200 - bias));
          g = bias + (random() % (200 - bias));
          b = bias + (random() % (200 - bias));
          colors[v] = (r << 16) | (g << 8) | b;
        }

        d->buf[y * d->stride + 3 * x + 0] = r;
        d->buf[y * d->stride + 3 * x + 1] = g;
        d->buf[y * d->stride + 3 * x + 2] = b;
      }
    }
    free(colors);
    image_u8x3_write_pnm(d, "debug_segmentation.pnm");
    image_u8x3_destroy(d);
  }

  timeprofile_stamp(td->tp, "unionfind");

  ////////////////////////////////////////////////////////
  // Step 3. Find gradient clusters.
  zarray_t* clusters = gradient_clusters(td, threshim, w, h, ts, uf);

  if (td->debug) {
    image_u8x3_t* d = image_u8x3_create(w, h);
    for (int i = 0; i < zarray_size(clusters); i++) {
      zarray_t* cluster;
      zarray_get(clusters, i, &cluster);
      uint32_t r, g, b;
      const int bias = 50;
      r = bias + (random() % (200 - bias));
      g = bias + (random() % (200 - bias));
      b = bias + (random() % (200 - bias));

      for (int j = 0; j < zarray_size(cluster); j++) {
        struct pt* p;
        zarray_get_volatile(cluster, j, &p);
        int x = p->x / 2;
        int y = p->y / 2;
        d->buf[y * d->stride + 3 * x + 0] = r;
        d->buf[y * d->stride + 3 * x + 1] = g;
        d->buf[y * d->stride + 3 * x + 2] = b;
      }
    }
    image_u8x3_write_pnm(d, "debug_clusters.pnm");
    image_u8x3_destroy(d);
  }

  // Note: Caller retains ownership of threshim; threshim is NOT destroyed here.
  timeprofile_stamp(td->tp, "make clusters");

  zarray_t* quads = fit_quads_and_cleanup(td, w, h, clusters, im);
  unionfind_destroy(uf);
  return quads;
}

// Runs Step 4 (fit quads) on externally provided clusters and cleans up.
// Takes ownership of clusters (destroys inner arrays and the outer array).
static zarray_t* fit_quads_and_cleanup(apriltag_detector_t* td, int w, int h,
                                       zarray_t* clusters, image_u8_t* im) {
  assert(td != NULL);
  assert(clusters != NULL);
  assert(im != NULL);

  ////////////////////////////////////////////////////////
  // Step 4. Fit quads to clusters.
  zarray_t* quads = fit_quads(td, w, h, clusters, im);

  if (td->debug) {
    FILE* f = fopen("debug_lines.ps", "w");
    if (f) {
      fprintf(f, "%%!PS\n\n");
      image_u8_t* im2 = image_u8_copy(im);
      image_u8_darken(im2);
      image_u8_darken(im2);
      double scale = fmin(612.0 / im->width, 792.0 / im2->height);
      fprintf(f, "%.15f %.15f scale\n", scale, scale);
      fprintf(f, "0 %d translate\n", im2->height);
      fprintf(f, "1 -1 scale\n");
      postscript_image(f, im2);
      image_u8_destroy(im2);

      for (int i = 0; i < zarray_size(quads); i++) {
        struct quad* q;
        zarray_get_volatile(quads, i, &q);
        float rgb[3];
        int bias = 100;
        for (int j = 0; j < 3; j++) rgb[j] = bias + (random() % (255 - bias));

        fprintf(f, "%f %f %f setrgbcolor\n", rgb[0] / 255.0f, rgb[1] / 255.0f,
                rgb[2] / 255.0f);
        fprintf(f,
                "%.15f %.15f moveto %.15f %.15f lineto %.15f %.15f lineto "
                "%.15f %.15f lineto %.15f %.15f lineto stroke\n",
                q->p[0][0], q->p[0][1], q->p[1][0], q->p[1][1], q->p[2][0],
                q->p[2][1], q->p[3][0], q->p[3][1], q->p[0][0], q->p[0][1]);
      }
      fclose(f);
    }
  }

  timeprofile_stamp(td->tp, "fit quads to clusters");
  destroy_clusters(clusters);

  return quads;
}

zarray_t* apriltag_quad_thresh_from_threshim(apriltag_detector_t* td,
                                             image_u8_t* im,
                                             image_u8_t* threshim) {
  return quad_thresh_from_threshim_and_uf(td, im, threshim, NULL);
}

// Converts GPU connected-components output to unionfind_t. The GPU flatten
// kernel writes canonical roots (parent[root] == root, 0xFFFFFFFF for
// ignored pixels) and sizes counting members INCLUDING the root, while the
// CPU convention excludes the root; adjust accordingly. Returns NULL when
// either input is NULL so callers transparently keep the CPU path.
static unionfind_t* unionfind_from_gpu_labels(int w, int h,
                                              const uint32_t* gpu_parents,
                                              const uint32_t* gpu_sizes) {
  if (gpu_parents == NULL || gpu_sizes == NULL) {
    return NULL;
  }
  unionfind_t* uf = unionfind_create((uint32_t)(w * h));
  if (uf == NULL) {
    return NULL;
  }
  const size_t n = (size_t)w * (size_t)h;
  memcpy(uf->parent, gpu_parents, n * sizeof(uint32_t));
  for (size_t i = 0; i < n; i++) {
    uint32_t s = gpu_sizes[i];
    uf->size[i] = (uf->parent[i] == 0xffffffff || s == 0) ? 0 : s - 1;
  }
  return uf;
}

zarray_t* apriltag_quad_thresh_from_gpu_labels(apriltag_detector_t* td,
                                               image_u8_t* im,
                                               image_u8_t* threshim,
                                               const uint32_t* gpu_parents,
                                               const uint32_t* gpu_sizes) {
  unionfind_t* uf = NULL;
  if (im != NULL && threshim != NULL) {
    uf = unionfind_from_gpu_labels(im->width, im->height, gpu_parents,
                                   gpu_sizes);
  }
  // NULL uf keeps the CPU connected-components path.
  return quad_thresh_from_threshim_and_uf(td, im, threshim, uf);
}

zarray_t* apriltag_quad_thresh_from_gpu_clusters(apriltag_detector_t* td,
                                                 image_u8_t* im,
                                                 zarray_t* clusters) {
  if (td == NULL || im == NULL || clusters == NULL) {
    destroy_clusters(clusters);
    return NULL;
  }
  if (!ensure_workerpool(td)) {
    destroy_clusters(clusters);
    return zarray_create(sizeof(struct quad));
  }
  // Thresholding, connected components, and clustering all ran externally
  // (e.g. on GPU); record their completion so downstream timing stays coherent.
  timeprofile_stamp(td->tp, "threshold");
  timeprofile_stamp(td->tp, "unionfind");
  timeprofile_stamp(td->tp, "make clusters");
  return fit_quads_and_cleanup(td, im->width, im->height, clusters, im);
}

static zarray_t* detect_with_external_stages(apriltag_detector_t* td,
                                             image_u8_t* im_orig,
                                             image_u8_t* threshim,
                                             const uint32_t* gpu_parents,
                                             const uint32_t* gpu_sizes,
                                             zarray_t* external_clusters) {
  if (threshim == NULL) {
    destroy_clusters(external_clusters);
    return apriltag_detector_detect(td, im_orig);
  }

  if (zarray_size(td->tag_families) == 0) {
    debug_print("No tag families enabled\n");
    destroy_clusters(external_clusters);
    return zarray_create(sizeof(apriltag_detection_t*));
  }

  if (td->wp == NULL || td->nthreads != workerpool_get_nthreads(td->wp)) {
    workerpool_destroy(td->wp);
    td->wp = workerpool_create(td->nthreads);
    if (td->wp == NULL) {
      destroy_clusters(external_clusters);
      return zarray_create(sizeof(apriltag_detection_t*));
    }
  }

  timeprofile_clear(td->tp);
  timeprofile_stamp(td->tp, "init");

  ///////////////////////////////////////////////////////////
  // Step 1. Prepare image matching threshim dimensions
  image_u8_t* quad_im = im_orig;
  float decimate_factor = 1.0f;

  if (td->quad_decimate > 1.0f && (threshim->width != im_orig->width ||
                                   threshim->height != im_orig->height)) {
    decimate_factor = td->quad_decimate;
    quad_im = image_u8_decimate(im_orig, td->quad_decimate);
    timeprofile_stamp(td->tp, "decimate");
  }

  if (td->quad_sigma != 0) {
    float sigma = fabsf((float)td->quad_sigma);
    int ksz = 4 * sigma;
    if ((ksz & 1) == 0) ksz++;

    if (ksz > 1) {
      if (quad_im == im_orig) {
        quad_im = image_u8_copy(im_orig);
      }
      if (td->quad_sigma > 0) {
        image_u8_gaussian_blur_parallel(td->wp, quad_im, sigma, ksz);
      } else {
        image_u8_t* orig = image_u8_copy(quad_im);
        image_u8_gaussian_blur_parallel(td->wp, quad_im, sigma, ksz);
        for (int y = 0; y < orig->height; y++) {
          for (int x = 0; x < orig->width; x++) {
            int vorig = orig->buf[y * orig->stride + x];
            int vblur = quad_im->buf[y * quad_im->stride + x];
            int v = 2 * vorig - vblur;
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            quad_im->buf[y * quad_im->stride + x] = (uint8_t)v;
          }
        }
        image_u8_destroy(orig);
      }
    }
    timeprofile_stamp(td->tp, "blur/sharp");
  }

  if (td->debug) image_u8_write_pnm(quad_im, "debug_preprocess.pnm");

  zarray_t* quads;
  if (external_clusters != NULL && quad_im->width == threshim->width &&
      quad_im->height == threshim->height) {
    // Thresholding, connected components, and clustering all ran
    // externally (e.g. on GPU); record completion for timing coherence.
    // fit_quads_and_cleanup takes ownership of external_clusters.
    timeprofile_stamp(td->tp, "threshold");
    timeprofile_stamp(td->tp, "unionfind");
    timeprofile_stamp(td->tp, "make clusters");
    // Quad fitting runs on CPU: earlier GPU line-fit experiments
    // (terms, window-fit batch, fused integer prefixes) each held
    // parity but measured slower than the CPU sweep, so they were
    // removed. fit_quads_and_cleanup takes ownership of
    // external_clusters.
    quads = fit_quads_and_cleanup(td, quad_im->width, quad_im->height,
                                  external_clusters, quad_im);
  } else if (gpu_parents != NULL && gpu_sizes != NULL &&
             quad_im->width == threshim->width &&
             quad_im->height == threshim->height) {
    quads = apriltag_quad_thresh_from_gpu_labels(td, quad_im, threshim,
                                                 gpu_parents, gpu_sizes);
  } else {
    destroy_clusters(external_clusters);
    quads = apriltag_quad_thresh_from_threshim(td, quad_im, threshim);
  }

  // Quad fitting uses the decimated image; refinement and decoding below
  // use im_orig. Restore full-resolution coordinates before either step,
  // matching apriltag_detector_detect's quad_decimate contract.
  if (decimate_factor > 1.0f) {
    for (int i = 0; i < zarray_size(quads); i++) {
      struct quad* q;
      zarray_get_volatile(quads, i, &q);
      for (int j = 0; j < 4; j++) {
        q->p[j][0] *= decimate_factor;
        q->p[j][1] *= decimate_factor;
      }
    }
  }

  if (quad_im != im_orig) image_u8_destroy(quad_im);

  zarray_t* detections = zarray_create(sizeof(apriltag_detection_t*));
  td->nquads = zarray_size(quads);
  timeprofile_stamp(td->tp, "quads");

  if (td->debug) {
    image_u8_t* im_quads = image_u8_copy(im_orig);
    image_u8_darken(im_quads);
    image_u8_darken(im_quads);
    srandom(0);
    for (int i = 0; i < zarray_size(quads); i++) {
      struct quad* quad;
      zarray_get_volatile(quads, i, &quad);
      const int bias = 100;
      int color = bias + (random() % (255 - bias));
      image_u8_draw_line(im_quads, quad->p[0][0], quad->p[0][1], quad->p[1][0],
                         quad->p[1][1], color, 1);
      image_u8_draw_line(im_quads, quad->p[1][0], quad->p[1][1], quad->p[2][0],
                         quad->p[2][1], color, 1);
      image_u8_draw_line(im_quads, quad->p[2][0], quad->p[2][1], quad->p[3][0],
                         quad->p[3][1], color, 1);
      image_u8_draw_line(im_quads, quad->p[3][0], quad->p[3][1], quad->p[0][0],
                         quad->p[0][1], color, 1);
    }
    image_u8_write_pnm(im_quads, "debug_quads_raw.pnm");
    image_u8_destroy(im_quads);
  }

  ////////////////////////////////////////////////////////////////
  // Step 2. Decode tags from each quad.
  image_u8_t* im_samples = td->debug ? image_u8_copy(im_orig) : NULL;
  int chunksize = 1 + zarray_size(quads) /
                          (APRILTAG_TASKS_PER_THREAD_TARGET * td->nthreads);
  struct quad_decode_task* tasks = malloc(sizeof(struct quad_decode_task) *
                                          (zarray_size(quads) / chunksize + 1));
  int ntasks = 0;
  for (int i = 0; i < zarray_size(quads); i += chunksize) {
    tasks[ntasks].i0 = i;
    tasks[ntasks].i1 = imin(zarray_size(quads), i + chunksize);
    tasks[ntasks].quads = quads;
    tasks[ntasks].td = td;
    tasks[ntasks].im = im_orig;
    tasks[ntasks].detections = detections;
    tasks[ntasks].im_samples = im_samples;
    workerpool_add_task(td->wp, quad_decode_task, &tasks[ntasks]);
    ntasks++;
  }
  workerpool_run(td->wp);
  free(tasks);

  if (im_samples != NULL) {
    image_u8_write_pnm(im_samples, "debug_samples.pnm");
    image_u8_destroy(im_samples);
  }

  if (td->debug) {
    image_u8_t* im_quads = image_u8_copy(im_orig);
    image_u8_darken(im_quads);
    image_u8_darken(im_quads);
    srandom(0);
    for (int i = 0; i < zarray_size(quads); i++) {
      struct quad* quad;
      zarray_get_volatile(quads, i, &quad);
      const int bias = 100;
      int color = bias + (random() % (255 - bias));
      image_u8_draw_line(im_quads, quad->p[0][0], quad->p[0][1], quad->p[1][0],
                         quad->p[1][1], color, 1);
      image_u8_draw_line(im_quads, quad->p[1][0], quad->p[1][1], quad->p[2][0],
                         quad->p[2][1], color, 1);
      image_u8_draw_line(im_quads, quad->p[2][0], quad->p[2][1], quad->p[3][0],
                         quad->p[3][1], color, 1);
      image_u8_draw_line(im_quads, quad->p[3][0], quad->p[3][1], quad->p[0][0],
                         quad->p[0][1], color, 1);
    }
    image_u8_write_pnm(im_quads, "debug_quads_fixed.pnm");
    image_u8_destroy(im_quads);
  }

  timeprofile_stamp(td->tp, "decode+refinement");

  ////////////////////////////////////////////////////////////////
  // Step 3. Reconcile duplicate detections.
  zarray_t* poly0 = g2d_polygon_create_zeros(4);
  zarray_t* poly1 = g2d_polygon_create_zeros(4);
  for (int i0 = 0; i0 < zarray_size(detections); i0++) {
    apriltag_detection_t* det0;
    zarray_get(detections, i0, &det0);
    for (int k = 0; k < 4; k++) zarray_set(poly0, k, det0->p[k], NULL);

    for (int i1 = i0 + 1; i1 < zarray_size(detections); i1++) {
      apriltag_detection_t* det1;
      zarray_get(detections, i1, &det1);
      if (det0->id != det1->id || det0->family != det1->family) continue;

      for (int k = 0; k < 4; k++) zarray_set(poly1, k, det1->p[k], NULL);

      if (g2d_polygon_overlaps_polygon(poly0, poly1)) {
        int pref = 0;
        pref = prefer_smaller(pref, det0->hamming, det1->hamming);
        pref = prefer_smaller(pref, -det0->decision_margin,
                              -det1->decision_margin);
        for (int i = 0; i < 4; i++) {
          pref = prefer_smaller(pref, det0->p[i][0], det1->p[i][0]);
          pref = prefer_smaller(pref, det0->p[i][1], det1->p[i][1]);
        }

        if (pref < 0) {
          apriltag_detection_destroy(det1);
          zarray_remove_index(detections, i1, 1);
          i1--;
          goto retry1;
        } else {
          apriltag_detection_destroy(det0);
          zarray_remove_index(detections, i0, 1);
          i0--;
          goto retry0;
        }
      }
    retry1:;
    }
  retry0:;
  }
  zarray_destroy(poly0);
  zarray_destroy(poly1);

  timeprofile_stamp(td->tp, "reconcile");

  ////////////////////////////////////////////////////////////////
  // Produce final debug output
  if (td->debug) {
    image_u8_t* darker = image_u8_copy(im_orig);
    image_u8_darken(darker);
    image_u8_darken(darker);
    FILE* f = fopen("debug_output.ps", "w");
    if (f) {
      double scale = fmin(612.0 / darker->width, 792.0 / darker->height);
      fprintf(f, "%%!PS\n\n");
      fprintf(f, "%f %f scale\n", scale, scale);
      fprintf(f, "0 %d translate\n", darker->height);
      fprintf(f, "1 -1 scale\n");
      postscript_image(f, darker);
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        float rgb[3];
        int bias = 100;
        for (int j = 0; j < 3; j++) rgb[j] = bias + (random() % (255 - bias));
        fprintf(f, "%f %f %f setrgbcolor\n", rgb[0] / 255.0f, rgb[1] / 255.0f,
                rgb[2] / 255.0f);
        fprintf(f,
                "%f %f moveto %f %f lineto %f %f lineto %f %f lineto %f %f "
                "lineto stroke\n",
                det->p[0][0], det->p[0][1], det->p[1][0], det->p[1][1],
                det->p[2][0], det->p[2][1], det->p[3][0], det->p[3][1],
                det->p[0][0], det->p[0][1]);
      }
      fprintf(f, "showpage\n");
      fclose(f);
    }
    image_u8_destroy(darker);
  }

  if (td->debug) {
    image_u8_t* darker = image_u8_copy(im_orig);
    image_u8_darken(darker);
    image_u8_darken(darker);
    image_u8x3_t* out = image_u8x3_create(darker->width, darker->height);
    for (int y = 0; y < im_orig->height; y++) {
      for (int x = 0; x < im_orig->width; x++) {
        out->buf[y * out->stride + 3 * x + 0] =
            darker->buf[y * darker->stride + x];
        out->buf[y * out->stride + 3 * x + 1] =
            darker->buf[y * darker->stride + x];
        out->buf[y * out->stride + 3 * x + 2] =
            darker->buf[y * darker->stride + x];
      }
    }
    image_u8_destroy(darker);
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t* det;
      zarray_get(detections, i, &det);
      float rgb[3];
      int bias = 100;
      for (int j = 0; j < 3; j++) rgb[j] = bias + (random() % (255 - bias));
      for (int j = 0; j < 4; j++) {
        int k = (j + 1) & 3;
        image_u8x3_draw_line(
            out, det->p[j][0], det->p[j][1], det->p[k][0], det->p[k][1],
            (uint8_t[]){(uint8_t)rgb[0], (uint8_t)rgb[1], (uint8_t)rgb[2]});
      }
    }
    image_u8x3_write_pnm(out, "debug_output.pnm");
    image_u8x3_destroy(out);
  }

  if (td->debug) {
    FILE* f = fopen("debug_quads.ps", "w");
    if (f) {
      image_u8_t* darker = image_u8_copy(im_orig);
      image_u8_darken(darker);
      image_u8_darken(darker);
      double scale = fmin(612.0 / darker->width, 792.0 / darker->height);
      fprintf(f, "%%!PS\n\n");
      fprintf(f, "%f %f scale\n", scale, scale);
      fprintf(f, "0 %d translate\n", darker->height);
      fprintf(f, "1 -1 scale\n");
      postscript_image(f, darker);
      image_u8_destroy(darker);
      for (int i = 0; i < zarray_size(quads); i++) {
        struct quad* q;
        zarray_get_volatile(quads, i, &q);
        float rgb[3];
        int bias = 100;
        for (int j = 0; j < 3; j++) rgb[j] = bias + (random() % (255 - bias));
        fprintf(f, "%f %f %f setrgbcolor\n", rgb[0] / 255.0f, rgb[1] / 255.0f,
                rgb[2] / 255.0f);
        fprintf(f,
                "%f %f moveto %f %f lineto %f %f lineto %f %f lineto %f %f "
                "lineto stroke\n",
                q->p[0][0], q->p[0][1], q->p[1][0], q->p[1][1], q->p[2][0],
                q->p[2][1], q->p[3][0], q->p[3][1], q->p[0][0], q->p[0][1]);
      }
      fprintf(f, "showpage\n");
      fclose(f);
    }
  }

  timeprofile_stamp(td->tp, "debug output");

  for (int i = 0; i < zarray_size(quads); i++) {
    struct quad* quad;
    zarray_get_volatile(quads, i, &quad);
    matd_destroy(quad->H);
    matd_destroy(quad->Hinv);
  }
  zarray_destroy(quads);

  zarray_sort(detections, detection_compare_function);
  timeprofile_stamp(td->tp, "cleanup");

  return detections;
}

image_u8_t* apriltag_threshold_cpu(apriltag_detector_t* td, image_u8_t* im) {
  if (td->wp == NULL || td->nthreads != workerpool_get_nthreads(td->wp)) {
    workerpool_destroy(td->wp);
    td->wp = workerpool_create(td->nthreads);
    if (td->wp == NULL) {
      return NULL;
    }
  }
  return threshold(td, im);
}

zarray_t* apriltag_detector_detect_with_threshim(apriltag_detector_t* td,
                                                 image_u8_t* im_orig,
                                                 image_u8_t* threshim) {
  return detect_with_external_stages(td, im_orig, threshim, NULL, NULL, NULL);
}

zarray_t* apriltag_detector_detect_with_gpu_labels(apriltag_detector_t* td,
                                                   image_u8_t* im_orig,
                                                   image_u8_t* threshim,
                                                   const uint32_t* gpu_parents,
                                                   const uint32_t* gpu_sizes) {
  return detect_with_external_stages(td, im_orig, threshim, gpu_parents,
                                     gpu_sizes, NULL);
}

zarray_t* apriltag_detector_detect_with_gpu_clusters(apriltag_detector_t* td,
                                                     image_u8_t* im_orig,
                                                     image_u8_t* threshim,
                                                     zarray_t* gpu_clusters) {
  return detect_with_external_stages(td, im_orig, threshim, NULL, NULL,
                                     gpu_clusters);
}

// NOLINTEND
