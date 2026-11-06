#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#if defined(__ARM_NEON) || defined(__aarch64__)
#include <arm_neon.h>
#endif

#if defined(__APPLE__)
#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#endif

#include "talOS/vision/apriltag/cpu_apriltag_backend.h"
#include "talOS/vision/apriltag/test_image_loader.h"
#include "talOS/vision/metal/gpu_sparse_clusters.h"
#include "talOS/vision/metal/metal_shaders.h"
#include "talOS/vision/synthetic_source.h"

extern "C" {
#include "apriltag.h"
#include "common/image_u8.h"
#include "common/pjpeg.h"
#include "common/timeprofile.h"
#include "common/unionfind.h"
#include "common/zarray.h"
#include "tag36h11.h"
}

// Forward declarations from apriltag internal headers / apriltag_quad_thresh.c
extern "C" {
image_u8_t* threshold(apriltag_detector_t* td, image_u8_t* im);
unionfind_t* connected_components(apriltag_detector_t* td, image_u8_t* threshim,
                                  int w, int h, int ts);
zarray_t* gradient_clusters(apriltag_detector_t* td, image_u8_t* threshim,
                            int w, int h, int ts, unionfind_t* uf);
zarray_t* fit_quads(apriltag_detector_t* td, int w, int h, zarray_t* clusters,
                    image_u8_t* im);
}

namespace talos::vision::metal {
namespace {

using Clock = std::chrono::steady_clock;

#if defined(__APPLE__)
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

// Helper struct for cluster points matching AprilTag's struct pt
// Layout must stay identical: fit_quads writes p->slope into every point.
struct ClusterPoint {
  uint16_t x, y;
  int16_t gx, gy;
  float slope;
};

// Fast Candidate-based gradient clusters
// Evaluates ONLY the coordinates in candidate_coords rather than all w*h pixels
zarray_t* FastGradientClustersFromCandidates(image_u8_t* threshim, int w, int h,
                                             int ts, unionfind_t* uf,
                                             const uint32_t* candidate_coords,
                                             uint32_t candidate_count) {
  zarray_t* clusters = zarray_create(sizeof(zarray_t*));

  // Organize candidates into rows for cache-friendly access and duplicate
  // filtering
  std::vector<std::vector<uint32_t>> row_candidates(h);
  for (uint32_t i = 0; i < candidate_count; ++i) {
    uint32_t entry = candidate_coords[i];
    uint32_t x = entry & 0x3FFF;
    uint32_t y = (entry >> 14) & 0x3FFF;
    if (x < static_cast<uint32_t>(w) && y < static_cast<uint32_t>(h)) {
      row_candidates[y].push_back(entry);
    }
  }

  // Hash table for clusters
  struct Entry {
    uint64_t id;
    zarray_t* cluster;
    Entry* next;
  };

  const int nclustermap = std::max(256, static_cast<int>(0.2 * w * h));
  std::vector<Entry*> clustermap(nclustermap, nullptr);
  std::vector<std::unique_ptr<Entry[]>> mem_pools;
  int pool_loc = 0;
  constexpr int kPoolSize = 2048;
  mem_pools.push_back(std::make_unique<Entry[]>(kPoolSize));

  auto u64hash = [](uint64_t v) -> uint32_t {
    v = (v ^ (v >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    v = (v ^ (v >> 27)) * UINT64_C(0x94d049bb133111eb);
    v = v ^ (v >> 31);
    return static_cast<uint32_t>(v);
  };

  for (int y = 1; y < h - 1; ++y) {
    auto& row = row_candidates[y];
    if (row.empty()) continue;

    // Sort row candidates by x
    std::sort(row.begin(), row.end(), [](uint32_t a, uint32_t b) {
      return (a & 0x3FFF) < (b & 0x3FFF);
    });

    bool connected_last = false;
    int last_x = -2;

    for (uint32_t entry : row) {
      int x = static_cast<int>(entry & 0x3FFF);
      uint32_t flags = (entry >> 28) & 0xF;

      if (x < 1 || x >= w - 1) {
        connected_last = false;
        last_x = x;
        continue;
      }
      if (x != last_x + 1) {
        connected_last = false;
      }
      last_x = x;

      uint8_t v0 = threshim->buf[y * ts + x];
      if (v0 == 127) {
        connected_last = false;
        continue;
      }

      uint64_t rep0 = unionfind_get_representative(uf, y * w + x);
      if (unionfind_get_set_size(uf, rep0) < 25) {
        connected_last = false;
        continue;
      }

      bool connected = false;

      auto do_conn = [&](int dx, int dy) {
        uint8_t v1 = threshim->buf[(y + dy) * ts + x + dx];
        if (v0 + v1 == 255) {
          uint64_t rep1 =
              unionfind_get_representative(uf, (y + dy) * w + x + dx);
          if (unionfind_get_set_size(uf, rep1) > 24) {
            uint64_t clusterid =
                (rep0 < rep1) ? ((rep1 << 32) + rep0) : ((rep0 << 32) + rep1);

            uint32_t bucket = u64hash(clusterid) % nclustermap;
            Entry* e = clustermap[bucket];
            while (e && e->id != clusterid) {
              e = e->next;
            }

            if (!e) {
              if (pool_loc == kPoolSize) {
                pool_loc = 0;
                mem_pools.push_back(std::make_unique<Entry[]>(kPoolSize));
              }
              e = &mem_pools.back()[pool_loc++];
              e->id = clusterid;
              e->cluster = zarray_create(sizeof(ClusterPoint));
              e->next = clustermap[bucket];
              clustermap[bucket] = e;
            }

            ClusterPoint p{
                .x = static_cast<uint16_t>(2 * x + dx),
                .y = static_cast<uint16_t>(2 * y + dy),
                .gx = static_cast<int16_t>(dx * (static_cast<int>(v1) - v0)),
                .gy = static_cast<int16_t>(dy * (static_cast<int>(v1) - v0)),
                .slope = 0.0f,
            };
            zarray_add(e->cluster, &p);
            return true;
          }
        }
        return false;
      };

      // 4-connectivity
      if (flags & 1) do_conn(1, 0);
      if (flags & 2) do_conn(0, 1);

      // 8-connectivity
      if (!connected_last && (flags & 4)) {
        do_conn(-1, 1);
      }
      connected = false;
      if (flags & 8) {
        connected = do_conn(1, 1);
      }
      connected_last = connected;
    }
  }

  for (int i = 0; i < nclustermap; ++i) {
    for (Entry* e = clustermap[i]; e; e = e->next) {
      zarray_add(clusters, reinterpret_cast<const void*>(&e->cluster));
    }
  }

  return clusters;
}
#endif

// ARM NEON SIMD Accelerated Edge Scanner:
// Checks 16 horizontal pixels in parallel using 128-bit vector instructions
uint32_t NeonSimdScanCandidates(const uint8_t* thresh, int w, int h, int stride,
                                uint32_t* out_candidates,
                                uint32_t max_candidates) {
  uint32_t count = 0;
#if defined(__ARM_NEON) || defined(__aarch64__)
  const uint8x16_t k255 = vdupq_n_u8(255);
  const uint8x16_t k127 = vdupq_n_u8(127);

  for (int y = 1; y < h - 1; ++y) {
    const uint8_t* row_curr = &thresh[static_cast<std::ptrdiff_t>(y) * stride];
    const uint8_t* row_next =
        &thresh[(static_cast<std::ptrdiff_t>(y) + 1) * stride];

    int x = 1;
    for (; x + 16 < w - 1; x += 16) {
      uint8x16_t v0 = vld1q_u8(&row_curr[x]);
      uint8x16_t vr = vld1q_u8(&row_curr[x + 1]);
      uint8x16_t vd = vld1q_u8(&row_next[x]);
      uint8x16_t vdl = vld1q_u8(&row_next[x - 1]);
      uint8x16_t vdr = vld1q_u8(&row_next[x + 1]);

      uint8x16_t sum_r = vaddq_u8(v0, vr);
      uint8x16_t sum_d = vaddq_u8(v0, vd);
      uint8x16_t sum_dl = vaddq_u8(v0, vdl);
      uint8x16_t sum_dr = vaddq_u8(v0, vdr);

      uint8x16_t is_255_r = vceqq_u8(sum_r, k255);
      uint8x16_t is_255_d = vceqq_u8(sum_d, k255);
      uint8x16_t is_255_dl = vceqq_u8(sum_dl, k255);
      uint8x16_t is_255_dr = vceqq_u8(sum_dr, k255);

      uint8x16_t any_edge = vorrq_u8(vorrq_u8(is_255_r, is_255_d),
                                     vorrq_u8(is_255_dl, is_255_dr));
      uint8x16_t not_127 = vmvnq_u8(vceqq_u8(v0, k127));
      uint8x16_t valid = vandq_u8(any_edge, not_127);

      // Fast check: if no pixel in this 16-pixel chunk is an edge, SKIP ENTIRE
      // CHUNK!
      if (vmaxvq_u8(valid) == 0) {
        continue;
      }

      // Scalar fallback for the active lanes within this 16-pixel chunk
      uint8_t valid_bytes[16];
      vst1q_u8(valid_bytes, valid);
      uint8_t flags_r[16], flags_d[16], flags_dl[16], flags_dr[16];
      vst1q_u8(flags_r, is_255_r);
      vst1q_u8(flags_d, is_255_d);
      vst1q_u8(flags_dl, is_255_dl);
      vst1q_u8(flags_dr, is_255_dr);

      for (int lane = 0; lane < 16; ++lane) {
        if (valid_bytes[lane]) {
          uint32_t flags = (flags_r[lane] ? 1 : 0) | (flags_d[lane] ? 2 : 0) |
                           (flags_dl[lane] ? 4 : 0) | (flags_dr[lane] ? 8 : 0);
          if (count < max_candidates) {
            int px = x + lane;
            out_candidates[count++] =
                (flags << 28) | ((y & 0x3FFF) << 14) | (px & 0x3FFF);
          }
        }
      }
    }

    // Process leftover pixels at end of row
    for (; x < w - 1; ++x) {
      uint8_t v0 = row_curr[x];
      if (v0 == 127) continue;
      uint8_t vr = row_curr[x + 1];
      uint8_t vd = row_next[x];
      uint8_t vdl = row_next[x - 1];
      uint8_t vdr = row_next[x + 1];

      uint32_t flags = ((v0 + vr == 255) ? 1 : 0) | ((v0 + vd == 255) ? 2 : 0) |
                       ((v0 + vdl == 255) ? 4 : 0) |
                       ((v0 + vdr == 255) ? 8 : 0);
      if (flags != 0 && count < max_candidates) {
        out_candidates[count++] =
            (flags << 28) | ((y & 0x3FFF) << 14) | (x & 0x3FFF);
      }
    }
  }
#endif
  return count;
}

// Test Fixture
class MetalCclClusteringTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family_bits(td_, tf_, 2);
    td_->nthreads = 1;
    td_->wp = workerpool_create(td_->nthreads);
    // NB: apriltag_detector_create() already allocated td_->tp; do not
    // overwrite.
    td_->quad_decimate = 2.0f;
    td_->quad_sigma = 0.0f;

#if defined(__APPLE__)
    device_ = MTL::CreateSystemDefaultDevice();
    if (device_ != nullptr) {
      NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
      NS::String* source =
          NS::String::string(kMetalShadersSource, NS::UTF8StringEncoding);
      MTL::CompileOptions* compile_options =
          MTL::CompileOptions::alloc()->init();
      NS::Error* error = nullptr;

      library_ = device_->newLibrary(source, compile_options, &error);
      compile_options->release();

      if (library_ != nullptr) {
        queue_ = device_->newCommandQueue();

        auto load_pso = [&](const char* name) -> MTL::ComputePipelineState* {
          NS::String* ns_name =
              NS::String::string(name, NS::UTF8StringEncoding);
          MTL::Function* fn = library_->newFunction(ns_name);
          NS::Error* pso_err = nullptr;
          MTL::ComputePipelineState* pso =
              device_->newComputePipelineState(fn, &pso_err);
          fn->release();
          return pso;
        };

        pso_extract_candidates_ = load_pso("extract_edge_candidates");
        pso_ccl_init_ = load_pso("ccl_init");
        pso_ccl_union_ = load_pso("ccl_union_pass");
        pso_ccl_flatten_ = load_pso("ccl_flatten");
        pso_ccl_sizes_ = load_pso("ccl_compute_sizes");
      }
      pool->release();
    }
#endif
  }

  void TearDown() override {
#if defined(__APPLE__)
    if (pso_extract_candidates_) pso_extract_candidates_->release();
    if (pso_ccl_init_) pso_ccl_init_->release();
    if (pso_ccl_union_) pso_ccl_union_->release();
    if (pso_ccl_flatten_) pso_ccl_flatten_->release();
    if (pso_ccl_sizes_) pso_ccl_sizes_->release();
    if (queue_) queue_->release();
    if (library_) library_->release();
    if (device_) device_->release();
#endif
    if (td_) apriltag_detector_destroy(td_);
    if (tf_) tag36h11_destroy(tf_);
  }

  apriltag_family_t* tf_{nullptr};
  apriltag_detector_t* td_{nullptr};

#if defined(__APPLE__)
  MTL::Device* device_{nullptr};
  MTL::Library* library_{nullptr};
  MTL::CommandQueue* queue_{nullptr};
  MTL::ComputePipelineState* pso_extract_candidates_{nullptr};
  MTL::ComputePipelineState* pso_ccl_init_{nullptr};
  MTL::ComputePipelineState* pso_ccl_union_{nullptr};
  MTL::ComputePipelineState* pso_ccl_flatten_{nullptr};
  MTL::ComputePipelineState* pso_ccl_sizes_{nullptr};
#endif
};

// TEST 1: GPU Candidate Edge Extraction correctness and candidate reduction
// ratio
TEST_F(MetalCclClusteringTest,
       GpuCandidateExtractionReducesPixelsByOver95Percent) {
#if !defined(__APPLE__)
  GTEST_SKIP() << "Metal is only supported on Apple platforms.";
#else
  ASSERT_NE(device_, nullptr);
  ASSERT_NE(pso_extract_candidates_, nullptr);

  // Generate 1920x1080 frame with tags
  SyntheticCameraSource source(0, "hd_cam",
                               CameraMode{.width = 1920,
                                          .height = 1080,
                                          .fps = 30.0,
                                          .format = PixelFormat::kGray8});
  source.AddTag(SyntheticTagDescriptor{
      .tag_id = 0, .center_x = 600.0, .center_y = 450.0, .size_pixels = 220.0});
  source.AddTag(SyntheticTagDescriptor{.tag_id = 1,
                                       .center_x = 1320.0,
                                       .center_y = 630.0,
                                       .size_pixels = 200.0});

  Frame frame = source.GenerateFrame();
  ASSERT_FALSE(frame.empty());

  // Use decimate=2: 960x540 = 518,400 pixels
  const int w = 960;
  const int h = 540;
  image_u8_t* dec_im = image_u8_create(w, h);

  // Downsample 2x
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      dec_im->buf[y * dec_im->stride + x] =
          frame.data()[(y * 2) * 1920 + (x * 2)];
    }
  }

  image_u8_t* threshim = threshold(td_, dec_im);
  ASSERT_NE(threshim, nullptr);

  const uint32_t total_pixels = w * h;
  constexpr uint32_t kMaxCandidates = 65536;

  MTL::Buffer* thresh_buf = device_->newBuffer(
      threshim->buf, static_cast<NS::UInteger>(threshim->stride) * h,
      MTL::ResourceStorageModeShared);
  MTL::Buffer* cand_buf = device_->newBuffer(kMaxCandidates * sizeof(uint32_t),
                                             MTL::ResourceStorageModeShared);
  MTL::Buffer* count_buf =
      device_->newBuffer(sizeof(uint32_t), MTL::ResourceStorageModeShared);

  uint32_t zero = 0;
  std::memcpy(count_buf->contents(), &zero, sizeof(zero));

  uint32_t mw = w;
  uint32_t mh = h;
  uint32_t ms = threshim->stride;
  uint32_t max_c = kMaxCandidates;

  // Run GPU extraction kernel
  MTL::CommandBuffer* cmd = queue_->commandBuffer();
  MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();
  enc->setComputePipelineState(pso_extract_candidates_);
  enc->setBuffer(thresh_buf, 0, 0);
  enc->setBuffer(cand_buf, 0, 1);
  enc->setBuffer(count_buf, 0, 2);
  enc->setBytes(&mw, sizeof(mw), 3);
  enc->setBytes(&mh, sizeof(mh), 4);
  enc->setBytes(&ms, sizeof(ms), 5);
  enc->setBytes(&max_c, sizeof(max_c), 6);

  MTL::Size tg_size = MTL::Size::Make(16, 16, 1);
  MTL::Size tg_count = MTL::Size::Make((w + 15) / 16, (h + 15) / 16, 1);
  enc->dispatchThreadgroups(tg_count, tg_size);
  enc->endEncoding();

  auto t0 = Clock::now();
  cmd->commit();
  cmd->waitUntilCompleted();
  auto t1 = Clock::now();

  double gpu_time_us = (cmd->GPUEndTime() - cmd->GPUStartTime()) * 1e6;
  if (gpu_time_us <= 0.0) {
    gpu_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
  }

  uint32_t gpu_candidate_count =
      *static_cast<const uint32_t*>(count_buf->contents());

  std::cout
      << "\n================= GPU Candidate Edge Extraction ================="
      << '\n';
  std::cout << "  Input Grid Size:       " << w << " x " << h << " ("
            << total_pixels << " pixels)" << '\n';
  std::cout << "  Extracted Candidates:  " << gpu_candidate_count << " pixels"
            << '\n';
  std::cout << "  Candidate Fraction:    " << std::fixed << std::setprecision(2)
            << (100.0 * gpu_candidate_count / total_pixels) << " % of frame"
            << '\n';
  std::cout << "  Search Space Saved:    " << std::fixed << std::setprecision(2)
            << (100.0 *
                (1.0 - static_cast<double>(gpu_candidate_count) / total_pixels))
            << " % reduction!" << '\n';
  std::cout << "  GPU Kernel Time:       " << std::fixed << std::setprecision(1)
            << gpu_time_us << " us" << '\n';
  std::cout
      << "=================================================================\n"
      << '\n';

  EXPECT_GT(gpu_candidate_count, 1000u);
  EXPECT_LT(gpu_candidate_count, 20000u);
  // Verify that candidate filtering reduces the active pixels by over 95%!
  EXPECT_GT(1.0 - (static_cast<double>(gpu_candidate_count) / total_pixels),
            0.95);

  thresh_buf->release();
  cand_buf->release();
  count_buf->release();
  image_u8_destroy(threshim);
  image_u8_destroy(dec_im);
#endif
}

// TEST 2: Candidate-Based Gradient Clusters vs Full CPU Scan Speedup & Parity
TEST_F(MetalCclClusteringTest, CandidateClusteringGivesMassiveSpeedup) {
#if !defined(__APPLE__)
  GTEST_SKIP() << "Metal is only supported on Apple platforms.";
#else
  ASSERT_NE(device_, nullptr);

  SyntheticCameraSource source(0, "hd_cam",
                               CameraMode{.width = 1920,
                                          .height = 1080,
                                          .fps = 30.0,
                                          .format = PixelFormat::kGray8});
  source.AddTag(SyntheticTagDescriptor{
      .tag_id = 0, .center_x = 600.0, .center_y = 450.0, .size_pixels = 220.0});
  source.AddTag(SyntheticTagDescriptor{.tag_id = 1,
                                       .center_x = 1320.0,
                                       .center_y = 630.0,
                                       .size_pixels = 200.0});

  Frame frame = source.GenerateFrame();
  const int w = 960;
  const int h = 540;
  image_u8_t* dec_im = image_u8_create(w, h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      dec_im->buf[y * dec_im->stride + x] =
          frame.data()[(y * 2) * 1920 + (x * 2)];
    }
  }

  image_u8_t* threshim = threshold(td_, dec_im);
  unionfind_t* uf = connected_components(td_, threshim, w, h, threshim->stride);

  // 1. Extract candidates using GPU
  constexpr uint32_t kMaxCandidates = 65536;
  MTL::Buffer* thresh_buf = device_->newBuffer(
      threshim->buf, static_cast<NS::UInteger>(threshim->stride) * h,
      MTL::ResourceStorageModeShared);
  MTL::Buffer* cand_buf = device_->newBuffer(kMaxCandidates * sizeof(uint32_t),
                                             MTL::ResourceStorageModeShared);
  MTL::Buffer* count_buf =
      device_->newBuffer(sizeof(uint32_t), MTL::ResourceStorageModeShared);

  uint32_t zero = 0;
  std::memcpy(count_buf->contents(), &zero, sizeof(zero));

  uint32_t mw = w;
  uint32_t mh = h;
  uint32_t ms = threshim->stride;
  uint32_t max_c = kMaxCandidates;

  MTL::CommandBuffer* cmd = queue_->commandBuffer();
  MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();
  enc->setComputePipelineState(pso_extract_candidates_);
  enc->setBuffer(thresh_buf, 0, 0);
  enc->setBuffer(cand_buf, 0, 1);
  enc->setBuffer(count_buf, 0, 2);
  enc->setBytes(&mw, sizeof(mw), 3);
  enc->setBytes(&mh, sizeof(mh), 4);
  enc->setBytes(&ms, sizeof(ms), 5);
  enc->setBytes(&max_c, sizeof(max_c), 6);
  MTL::Size tg_size = MTL::Size::Make(16, 16, 1);
  MTL::Size tg_count = MTL::Size::Make((w + 15) / 16, (h + 15) / 16, 1);
  enc->dispatchThreadgroups(tg_count, tg_size);
  enc->endEncoding();
  cmd->commit();
  cmd->waitUntilCompleted();

  uint32_t candidate_count =
      *static_cast<const uint32_t*>(count_buf->contents());
  const uint32_t* candidate_coords =
      static_cast<const uint32_t*>(cand_buf->contents());

  // The GPU kernel keeps incrementing the atomic counter past max_candidates
  // even though stores are guarded, so clamp before indexing cand_buf.
  if (candidate_count > kMaxCandidates) candidate_count = kMaxCandidates;

  constexpr int kIterations = 20;

  // Benchmark Standard CPU gradient_clusters (Full 518,400 pixel raster scan)
  double cpu_scan_total_us = 0.0;
  zarray_t* standard_clusters = nullptr;
  for (int i = 0; i < kIterations; ++i) {
    if (standard_clusters) {
      for (int c = 0; c < zarray_size(standard_clusters); ++c) {
        zarray_t* cluster;
        zarray_get(standard_clusters, c, reinterpret_cast<void*>(&cluster));
        zarray_destroy(cluster);
      }
      zarray_destroy(standard_clusters);
    }
    auto t0 = Clock::now();
    standard_clusters =
        gradient_clusters(td_, threshim, w, h, threshim->stride, uf);
    auto t1 = Clock::now();
    cpu_scan_total_us +=
        std::chrono::duration<double, std::micro>(t1 - t0).count();
  }
  double cpu_scan_avg_us = cpu_scan_total_us / kIterations;

  // Benchmark Fast Candidate-based gradient clusters
  double cand_cluster_total_us = 0.0;
  zarray_t* fast_clusters = nullptr;
  for (int i = 0; i < kIterations; ++i) {
    if (fast_clusters) {
      for (int c = 0; c < zarray_size(fast_clusters); ++c) {
        zarray_t* cluster;
        zarray_get(fast_clusters, c, reinterpret_cast<void*>(&cluster));
        zarray_destroy(cluster);
      }
      zarray_destroy(fast_clusters);
    }
    auto t0 = Clock::now();
    fast_clusters =
        FastGradientClustersFromCandidates(threshim, w, h, threshim->stride, uf,
                                           candidate_coords, candidate_count);
    auto t1 = Clock::now();
    cand_cluster_total_us +=
        std::chrono::duration<double, std::micro>(t1 - t0).count();
  }
  double cand_cluster_avg_us = cand_cluster_total_us / kIterations;
  double clustering_speedup = cpu_scan_avg_us / cand_cluster_avg_us;

  // Benchmark Quad Fitting on both sets of clusters
  zarray_t* quads_standard = fit_quads(td_, w, h, standard_clusters, dec_im);
  zarray_t* quads_fast = fit_quads(td_, w, h, fast_clusters, dec_im);

  int quads_standard_count = zarray_size(quads_standard);
  int quads_fast_count = zarray_size(quads_fast);

  std::cout
      << "\n=========== Gradient Clustering Performance Benchmark ==========="
      << '\n';
  std::cout << "  Standard Full-Scan CPU:    " << std::fixed
            << std::setprecision(1) << cpu_scan_avg_us << " us" << '\n';
  std::cout << "  Fast Candidate Clustering: " << std::fixed
            << std::setprecision(1) << cand_cluster_avg_us << " us" << '\n';
  std::cout << "  Clustering Speedup:        " << std::fixed
            << std::setprecision(2) << clustering_speedup << "x faster!"
            << '\n';
  std::cout << "  Standard Clusters Extracted: "
            << zarray_size(standard_clusters) << '\n';
  std::cout << "  Fast Clusters Extracted:     " << zarray_size(fast_clusters)
            << '\n';
  std::cout << "  Quads Detected (Standard):   " << quads_standard_count
            << '\n';
  std::cout << "  Quads Detected (Fast):       " << quads_fast_count << '\n';
  std::cout
      << "=================================================================\n"
      << '\n';

  EXPECT_GT(quads_fast_count, 0);
  EXPECT_EQ(quads_fast_count, quads_standard_count);

  // Cleanup
  zarray_destroy(quads_standard);
  zarray_destroy(quads_fast);
  for (int c = 0; c < zarray_size(standard_clusters); ++c) {
    zarray_t* cluster;
    zarray_get(standard_clusters, c, reinterpret_cast<void*>(&cluster));
    zarray_destroy(cluster);
  }
  zarray_destroy(standard_clusters);
  for (int c = 0; c < zarray_size(fast_clusters); ++c) {
    zarray_t* cluster;
    zarray_get(fast_clusters, c, reinterpret_cast<void*>(&cluster));
    zarray_destroy(cluster);
  }
  zarray_destroy(fast_clusters);

  thresh_buf->release();
  cand_buf->release();
  count_buf->release();
  unionfind_destroy(uf);
  image_u8_destroy(threshim);
  image_u8_destroy(dec_im);
#endif
}

// TEST 2b: Label-native sparse clustering parity (Stage 2b).
// Builds clusters from GPU CCL labels + GPU edge candidates with zero dense
// CPU touch, and requires quad parity with the CPU gradient_clusters path.
TEST_F(MetalCclClusteringTest, SparseClustersFromGpuLabelsMatchCpu) {
#if !defined(__APPLE__)
  GTEST_SKIP() << "Metal is only supported on Apple platforms.";
#else
  ASSERT_NE(device_, nullptr);
  ASSERT_NE(pso_extract_candidates_, nullptr);

  SyntheticCameraSource source(0, "hd_cam",
                               CameraMode{.width = 1920,
                                          .height = 1080,
                                          .fps = 30.0,
                                          .format = PixelFormat::kGray8});
  source.AddTag(SyntheticTagDescriptor{
      .tag_id = 0, .center_x = 600.0, .center_y = 450.0, .size_pixels = 220.0});
  source.AddTag(SyntheticTagDescriptor{.tag_id = 1,
                                       .center_x = 1320.0,
                                       .center_y = 630.0,
                                       .size_pixels = 200.0});

  Frame frame = source.GenerateFrame();
  const int w = 960;
  const int h = 540;
  image_u8_t* dec_im = image_u8_create(w, h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      dec_im->buf[y * dec_im->stride + x] =
          frame.data()[(y * 2) * 1920 + (x * 2)];
    }
  }

  image_u8_t* threshim = threshold(td_, dec_im);
  ASSERT_NE(threshim, nullptr);

  // CPU baseline: full-scan gradient clusters.
  unionfind_t* uf = connected_components(td_, threshim, w, h, threshim->stride);
  ASSERT_NE(uf, nullptr);
  zarray_t* standard_clusters =
      gradient_clusters(td_, threshim, w, h, threshim->stride, uf);
  ASSERT_NE(standard_clusters, nullptr);

  // GPU labels + GPU edge candidates.
  const uint32_t total_pixels = static_cast<uint32_t>(w * h);
  constexpr uint32_t kMaxCandidates = 65536;
  MTL::Buffer* thresh_buf = device_->newBuffer(
      threshim->buf, static_cast<NS::UInteger>(threshim->stride) * h,
      MTL::ResourceStorageModeShared);
  MTL::Buffer* parent_buf = device_->newBuffer(total_pixels * sizeof(uint32_t),
                                               MTL::ResourceStorageModeShared);
  MTL::Buffer* sizes_buf = device_->newBuffer(total_pixels * sizeof(uint32_t),
                                              MTL::ResourceStorageModeShared);
  MTL::Buffer* cand_buf = device_->newBuffer(kMaxCandidates * sizeof(uint32_t),
                                             MTL::ResourceStorageModeShared);
  MTL::Buffer* count_buf =
      device_->newBuffer(sizeof(uint32_t), MTL::ResourceStorageModeShared);
  ASSERT_NE(thresh_buf, nullptr);
  ASSERT_NE(parent_buf, nullptr);
  ASSERT_NE(sizes_buf, nullptr);
  ASSERT_NE(cand_buf, nullptr);
  ASSERT_NE(count_buf, nullptr);

  uint32_t zero = 0;
  std::memcpy(count_buf->contents(), &zero, sizeof(zero));
  const uint32_t mw = w, mh = h, ms = threshim->stride, max_c = kMaxCandidates;

  MTL::CommandBuffer* cmd = queue_->commandBuffer();
  MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();
  const MTL::Size tg2 = MTL::Size::Make(16, 16, 1);
  const MTL::Size n2 = MTL::Size::Make((w + 15) / 16, (h + 15) / 16, 1);
  const MTL::Size tg1 = MTL::Size::Make(256, 1, 1);
  const MTL::Size n1 = MTL::Size::Make((total_pixels + 255) / 256, 1, 1);
  enc->setComputePipelineState(pso_ccl_init_);
  enc->setBuffer(thresh_buf, 0, 0);
  enc->setBuffer(parent_buf, 0, 1);
  enc->setBuffer(sizes_buf, 0, 2);
  enc->setBytes(&mw, sizeof(mw), 3);
  enc->setBytes(&mh, sizeof(mh), 4);
  enc->setBytes(&ms, sizeof(ms), 5);
  enc->dispatchThreadgroups(n2, tg2);
  enc->setComputePipelineState(pso_ccl_union_);
  enc->setBuffer(thresh_buf, 0, 0);
  enc->setBuffer(parent_buf, 0, 1);
  enc->setBytes(&mw, sizeof(mw), 2);
  enc->setBytes(&mh, sizeof(mh), 3);
  enc->setBytes(&ms, sizeof(ms), 4);
  enc->dispatchThreadgroups(n2, tg2);
  enc->setComputePipelineState(pso_ccl_flatten_);
  enc->setBuffer(parent_buf, 0, 0);
  enc->setBytes(&total_pixels, sizeof(total_pixels), 1);
  enc->dispatchThreadgroups(n1, tg1);
  enc->setComputePipelineState(pso_ccl_sizes_);
  enc->setBuffer(parent_buf, 0, 0);
  enc->setBuffer(sizes_buf, 0, 1);
  enc->setBytes(&total_pixels, sizeof(total_pixels), 2);
  enc->dispatchThreadgroups(n1, tg1);
  enc->setComputePipelineState(pso_extract_candidates_);
  enc->setBuffer(thresh_buf, 0, 0);
  enc->setBuffer(cand_buf, 0, 1);
  enc->setBuffer(count_buf, 0, 2);
  enc->setBytes(&mw, sizeof(mw), 3);
  enc->setBytes(&mh, sizeof(mh), 4);
  enc->setBytes(&ms, sizeof(ms), 5);
  enc->setBytes(&max_c, sizeof(max_c), 6);
  enc->dispatchThreadgroups(n2, tg2);
  enc->endEncoding();
  cmd->commit();
  cmd->waitUntilCompleted();

  const uint32_t* gpu_parents =
      static_cast<const uint32_t*>(parent_buf->contents());
  const uint32_t* gpu_sizes =
      static_cast<const uint32_t*>(sizes_buf->contents());
  uint32_t candidate_count =
      *static_cast<const uint32_t*>(count_buf->contents());
  if (candidate_count > kMaxCandidates) candidate_count = kMaxCandidates;
  const uint32_t* candidate_coords =
      static_cast<const uint32_t*>(cand_buf->contents());

  zarray_t* sparse_clusters = BuildSparseClustersFromGpuLabels(
      threshim, w, h, threshim->stride, gpu_parents, gpu_sizes,
      candidate_coords, candidate_count);
  ASSERT_NE(sparse_clusters, nullptr);

  std::cout << "\n=========== Sparse GPU-Label Clustering Parity ==========="
            << '\n';
  std::cout << "  Standard Clusters: " << zarray_size(standard_clusters)
            << '\n';
  std::cout << "  Sparse Clusters:   " << zarray_size(sparse_clusters) << '\n';
  std::cout << "============================================================\n"
            << '\n';

  EXPECT_EQ(zarray_size(sparse_clusters), zarray_size(standard_clusters));

  zarray_t* quads_standard = fit_quads(td_, w, h, standard_clusters, dec_im);
  zarray_t* quads_sparse = fit_quads(td_, w, h, sparse_clusters, dec_im);
  const int n_standard = zarray_size(quads_standard);
  const int n_sparse = zarray_size(quads_sparse);
  std::cout << "  Quads (Standard): " << n_standard << '\n';
  std::cout << "  Quads (Sparse):   " << n_sparse << '\n';
  EXPECT_GT(n_sparse, 0);
  ASSERT_EQ(n_sparse, n_standard);
  // Cluster hash iteration order differs from upstream, so quads may come in
  // a different order: match by nearest centroid, then compare corners.
  auto centroid = [](struct quad* q) {
    double cx = 0.0, cy = 0.0;
    for (int j = 0; j < 4; ++j) {
      cx += q->p[j][0];
      cy += q->p[j][1];
    }
    return std::make_pair(cx / 4.0, cy / 4.0);
  };
  for (int i = 0; i < n_sparse; ++i) {
    struct quad* qf;
    zarray_get_volatile(quads_sparse, i, reinterpret_cast<void*>(&qf));
    const auto fc = centroid(qf);
    int best = -1;
    double best_d2 = 1.0;  // 1px^2 match gate
    for (int k = 0; k < n_standard; ++k) {
      struct quad* qs;
      zarray_get_volatile(quads_standard, k, reinterpret_cast<void*>(&qs));
      const auto sc = centroid(qs);
      const double d2 = (fc.first - sc.first) * (fc.first - sc.first) +
                        (fc.second - sc.second) * (fc.second - sc.second);
      if (d2 < best_d2) {
        best_d2 = d2;
        best = k;
      }
    }
    ASSERT_GE(best, 0) << "Sparse quad " << i << " matches no standard quad";
    struct quad* qs;
    zarray_get_volatile(quads_standard, best, reinterpret_cast<void*>(&qs));
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(qs->p[j][0], qf->p[j][0], 0.1);
      EXPECT_NEAR(qs->p[j][1], qf->p[j][1], 0.1);
    }
  }

  zarray_destroy(quads_standard);
  zarray_destroy(quads_sparse);
  for (int c = 0; c < zarray_size(standard_clusters); ++c) {
    zarray_t* cluster;
    zarray_get(standard_clusters, c, reinterpret_cast<void*>(&cluster));
    zarray_destroy(cluster);
  }
  zarray_destroy(standard_clusters);
  for (int c = 0; c < zarray_size(sparse_clusters); ++c) {
    zarray_t* cluster;
    zarray_get(sparse_clusters, c, reinterpret_cast<void*>(&cluster));
    zarray_destroy(cluster);
  }
  zarray_destroy(sparse_clusters);

  thresh_buf->release();
  parent_buf->release();
  sizes_buf->release();
  cand_buf->release();
  count_buf->release();
  unionfind_destroy(uf);
  image_u8_destroy(threshim);
  image_u8_destroy(dec_im);
#endif
}

// TEST 2c: Sparse clustering parity on a photographic rotated-tag fixture.
// Regression guard: candidate-direction coverage must match the CPU full scan
// on real (non-synthetic) edge statistics.
TEST_F(MetalCclClusteringTest, SparseClustersPhotographicRotatedParity) {
#if !defined(__APPLE__)
  GTEST_SKIP() << "Metal is only supported on Apple platforms.";
#else
  const std::string path = ResolveTestdataPath("scene_rotated_tag.png");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Fixture not found: " << path;
  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);
  const int w = im->width;
  const int h = im->height;

  image_u8_t* threshim = threshold(td_, im);
  ASSERT_NE(threshim, nullptr);

  unionfind_t* uf = connected_components(td_, threshim, w, h, threshim->stride);
  zarray_t* standard_clusters =
      gradient_clusters(td_, threshim, w, h, threshim->stride, uf);

  const uint32_t total_pixels = static_cast<uint32_t>(w * h);
  constexpr uint32_t kMaxCandidates = 65536;
  MTL::Buffer* thresh_buf = device_->newBuffer(
      threshim->buf, static_cast<NS::UInteger>(threshim->stride) * h,
      MTL::ResourceStorageModeShared);
  MTL::Buffer* parent_buf = device_->newBuffer(total_pixels * sizeof(uint32_t),
                                               MTL::ResourceStorageModeShared);
  MTL::Buffer* sizes_buf = device_->newBuffer(total_pixels * sizeof(uint32_t),
                                              MTL::ResourceStorageModeShared);
  MTL::Buffer* cand_buf = device_->newBuffer(kMaxCandidates * sizeof(uint32_t),
                                             MTL::ResourceStorageModeShared);
  MTL::Buffer* count_buf =
      device_->newBuffer(sizeof(uint32_t), MTL::ResourceStorageModeShared);

  uint32_t zero = 0;
  std::memcpy(count_buf->contents(), &zero, sizeof(zero));
  const uint32_t mw = w, mh = h, ms = threshim->stride, max_c = kMaxCandidates;

  MTL::CommandBuffer* cmd = queue_->commandBuffer();
  MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();
  const MTL::Size tg2 = MTL::Size::Make(16, 16, 1);
  const MTL::Size n2 = MTL::Size::Make((w + 15) / 16, (h + 15) / 16, 1);
  const MTL::Size tg1 = MTL::Size::Make(256, 1, 1);
  const MTL::Size n1 = MTL::Size::Make((total_pixels + 255) / 256, 1, 1);
  enc->setComputePipelineState(pso_ccl_init_);
  enc->setBuffer(thresh_buf, 0, 0);
  enc->setBuffer(parent_buf, 0, 1);
  enc->setBuffer(sizes_buf, 0, 2);
  enc->setBytes(&mw, sizeof(mw), 3);
  enc->setBytes(&mh, sizeof(mh), 4);
  enc->setBytes(&ms, sizeof(ms), 5);
  enc->dispatchThreadgroups(n2, tg2);
  enc->setComputePipelineState(pso_ccl_union_);
  enc->setBuffer(thresh_buf, 0, 0);
  enc->setBuffer(parent_buf, 0, 1);
  enc->setBytes(&mw, sizeof(mw), 2);
  enc->setBytes(&mh, sizeof(mh), 3);
  enc->setBytes(&ms, sizeof(ms), 4);
  enc->dispatchThreadgroups(n2, tg2);
  enc->setComputePipelineState(pso_ccl_flatten_);
  enc->setBuffer(parent_buf, 0, 0);
  enc->setBytes(&total_pixels, sizeof(total_pixels), 1);
  enc->dispatchThreadgroups(n1, tg1);
  enc->setComputePipelineState(pso_ccl_sizes_);
  enc->setBuffer(parent_buf, 0, 0);
  enc->setBuffer(sizes_buf, 0, 1);
  enc->setBytes(&total_pixels, sizeof(total_pixels), 2);
  enc->dispatchThreadgroups(n1, tg1);
  enc->setComputePipelineState(pso_extract_candidates_);
  enc->setBuffer(thresh_buf, 0, 0);
  enc->setBuffer(cand_buf, 0, 1);
  enc->setBuffer(count_buf, 0, 2);
  enc->setBytes(&mw, sizeof(mw), 3);
  enc->setBytes(&mh, sizeof(mh), 4);
  enc->setBytes(&ms, sizeof(ms), 5);
  enc->setBytes(&max_c, sizeof(max_c), 6);
  enc->dispatchThreadgroups(n2, tg2);
  enc->endEncoding();
  cmd->commit();
  cmd->waitUntilCompleted();

  const uint32_t* gpu_parents =
      static_cast<const uint32_t*>(parent_buf->contents());
  const uint32_t* gpu_sizes =
      static_cast<const uint32_t*>(sizes_buf->contents());
  uint32_t candidate_count =
      *static_cast<const uint32_t*>(count_buf->contents());
  if (candidate_count > kMaxCandidates) candidate_count = kMaxCandidates;
  const uint32_t* candidate_coords =
      static_cast<const uint32_t*>(cand_buf->contents());

  zarray_t* sparse_clusters = BuildSparseClustersFromGpuLabels(
      threshim, w, h, threshim->stride, gpu_parents, gpu_sizes,
      candidate_coords, candidate_count);

  std::cout << "\n===== Sparse Photographic (rotated) Parity =====" << '\n';
  std::cout << "  Image:             " << w << " x " << h << '\n';
  std::cout << "  Candidates:        " << candidate_count << '\n';
  std::cout << "  Standard Clusters: " << zarray_size(standard_clusters)
            << '\n';
  std::cout << "  Sparse Clusters:   " << zarray_size(sparse_clusters) << '\n';
  for (int c = 0; c < zarray_size(standard_clusters); ++c) {
    zarray_t* cluster;
    zarray_get(standard_clusters, c, reinterpret_cast<void*>(&cluster));
    std::cout << "  std cluster[" << c << "] size " << zarray_size(cluster)
              << '\n';
  }
  for (int c = 0; c < zarray_size(sparse_clusters); ++c) {
    zarray_t* cluster;
    zarray_get(sparse_clusters, c, reinterpret_cast<void*>(&cluster));
    std::cout << "  gpu cluster[" << c << "] size " << zarray_size(cluster)
              << '\n';
  }
  std::cout << "================================================\n" << '\n';

  EXPECT_EQ(zarray_size(sparse_clusters), zarray_size(standard_clusters));

  zarray_t* quads_standard = fit_quads(td_, w, h, standard_clusters, im);
  zarray_t* quads_sparse = fit_quads(td_, w, h, sparse_clusters, im);
  EXPECT_EQ(zarray_size(quads_sparse), zarray_size(quads_standard));
  EXPECT_GT(zarray_size(quads_sparse), 0);

  zarray_destroy(quads_standard);
  zarray_destroy(quads_sparse);
  for (int c = 0; c < zarray_size(standard_clusters); ++c) {
    zarray_t* cluster;
    zarray_get(standard_clusters, c, reinterpret_cast<void*>(&cluster));
    zarray_destroy(cluster);
  }
  zarray_destroy(standard_clusters);
  for (int c = 0; c < zarray_size(sparse_clusters); ++c) {
    zarray_t* cluster;
    zarray_get(sparse_clusters, c, reinterpret_cast<void*>(&cluster));
    zarray_destroy(cluster);
  }
  zarray_destroy(sparse_clusters);

  thresh_buf->release();
  parent_buf->release();
  sizes_buf->release();
  cand_buf->release();
  count_buf->release();
  unionfind_destroy(uf);
  image_u8_destroy(threshim);
  image_u8_destroy(im);
#endif
}

// TEST 3: ARM NEON SIMD Accelerated Edge Scanning
TEST_F(MetalCclClusteringTest, NeonSimdScanAcceleration) {
#if !defined(__ARM_NEON) && !defined(__aarch64__)
  GTEST_SKIP() << "NEON scanning requires an ARM NEON-capable platform.";
#else
  SyntheticCameraSource source(0, "hd_cam",
                               CameraMode{.width = 1920,
                                          .height = 1080,
                                          .fps = 30.0,
                                          .format = PixelFormat::kGray8});
  source.AddTag(SyntheticTagDescriptor{
      .tag_id = 0, .center_x = 600.0, .center_y = 450.0, .size_pixels = 220.0});
  source.AddTag(SyntheticTagDescriptor{.tag_id = 1,
                                       .center_x = 1320.0,
                                       .center_y = 630.0,
                                       .size_pixels = 200.0});

  Frame frame = source.GenerateFrame();
  const int w = 960;
  const int h = 540;
  image_u8_t* dec_im = image_u8_create(w, h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      dec_im->buf[y * dec_im->stride + x] =
          frame.data()[(y * 2) * 1920 + (x * 2)];
    }
  }

  image_u8_t* threshim = threshold(td_, dec_im);
  ASSERT_NE(threshim, nullptr);

  std::vector<uint32_t> simd_candidates(65536);
  constexpr int kIterations = 50;

  double total_simd_us = 0.0;
  uint32_t candidate_count = 0;
  for (int i = 0; i < kIterations; ++i) {
    auto t0 = Clock::now();
    candidate_count =
        NeonSimdScanCandidates(threshim->buf, w, h, threshim->stride,
                               simd_candidates.data(), simd_candidates.size());
    auto t1 = Clock::now();
    total_simd_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
  }
  double avg_simd_us = total_simd_us / kIterations;

  std::cout << "\n================= NEON SIMD Candidate Edge Scanner "
               "================="
            << '\n';
  std::cout << "  Input Grid Size:    " << w << " x " << h << " (" << (w * h)
            << " pixels)" << '\n';
  std::cout << "  Extracted Count:    " << candidate_count << " candidates"
            << '\n';
  std::cout << "  NEON SIMD Scan:     " << std::fixed << std::setprecision(1)
            << avg_simd_us << " us (" << (avg_simd_us / 1000.0) << " ms)"
            << '\n';
  std::cout << "  Throughput:         " << std::fixed << std::setprecision(2)
            << ((w * h) / (avg_simd_us * 1e-6) / 1e6) << " MPixels/sec" << '\n';
  std::cout
      << "===================================================================\n"
      << '\n';

  EXPECT_GT(candidate_count, 1000u);

  image_u8_destroy(threshim);
  image_u8_destroy(dec_im);
#endif
}

// TEST 4: GPU Connected Components (CCL) Execution & Timing
TEST_F(MetalCclClusteringTest, GpuConnectedComponentsSpeedAndParity) {
#if !defined(__APPLE__)
  GTEST_SKIP() << "Metal is only supported on Apple platforms.";
#else
  ASSERT_NE(device_, nullptr);
  ASSERT_NE(pso_ccl_init_, nullptr);
  ASSERT_NE(pso_ccl_union_, nullptr);
  ASSERT_NE(pso_ccl_flatten_, nullptr);
  ASSERT_NE(pso_ccl_sizes_, nullptr);

  SyntheticCameraSource source(0, "hd_cam",
                               CameraMode{.width = 1920,
                                          .height = 1080,
                                          .fps = 30.0,
                                          .format = PixelFormat::kGray8});
  source.AddTag(SyntheticTagDescriptor{
      .tag_id = 0, .center_x = 600.0, .center_y = 450.0, .size_pixels = 220.0});
  source.AddTag(SyntheticTagDescriptor{.tag_id = 1,
                                       .center_x = 1320.0,
                                       .center_y = 630.0,
                                       .size_pixels = 200.0});

  Frame frame = source.GenerateFrame();
  const int w = 960;
  const int h = 540;
  image_u8_t* dec_im = image_u8_create(w, h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      dec_im->buf[y * dec_im->stride + x] =
          frame.data()[(y * 2) * 1920 + (x * 2)];
    }
  }

  image_u8_t* threshim = threshold(td_, dec_im);

  // 1. CPU Connected Components Baseline
  constexpr int kIterations = 15;
  double cpu_ccl_total_us = 0.0;
  unionfind_t* cpu_uf = nullptr;
  for (int i = 0; i < kIterations; ++i) {
    if (cpu_uf) unionfind_destroy(cpu_uf);
    auto t0 = Clock::now();
    cpu_uf = connected_components(td_, threshim, w, h, threshim->stride);
    auto t1 = Clock::now();
    cpu_ccl_total_us +=
        std::chrono::duration<double, std::micro>(t1 - t0).count();
  }
  double cpu_ccl_avg_us = cpu_ccl_total_us / kIterations;

  // 2. GPU Connected Components Pipeline
  const uint32_t total_pixels = w * h;
  MTL::Buffer* thresh_buf = device_->newBuffer(
      threshim->buf, static_cast<NS::UInteger>(threshim->stride) * h,
      MTL::ResourceStorageModeShared);
  MTL::Buffer* parent_buf = device_->newBuffer(total_pixels * sizeof(uint32_t),
                                               MTL::ResourceStorageModeShared);
  MTL::Buffer* sizes_buf = device_->newBuffer(total_pixels * sizeof(uint32_t),
                                              MTL::ResourceStorageModeShared);

  uint32_t mw = w;
  uint32_t mh = h;
  uint32_t ms = threshim->stride;
  uint32_t tot_px = total_pixels;
  MTL::Size tg_2d = MTL::Size::Make(16, 16, 1);
  MTL::Size count_2d = MTL::Size::Make((w + 15) / 16, (h + 15) / 16, 1);
  MTL::Size tg_1d = MTL::Size::Make(256, 1, 1);
  MTL::Size count_1d = MTL::Size::Make((total_pixels + 255) / 256, 1, 1);

  double gpu_ccl_total_us = 0.0;
  for (int i = 0; i < kIterations; ++i) {
    MTL::CommandBuffer* cmd = queue_->commandBuffer();
    MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();

    // Stage 1: Init
    enc->setComputePipelineState(pso_ccl_init_);
    enc->setBuffer(thresh_buf, 0, 0);
    enc->setBuffer(parent_buf, 0, 1);
    enc->setBuffer(sizes_buf, 0, 2);
    enc->setBytes(&mw, sizeof(mw), 3);
    enc->setBytes(&mh, sizeof(mh), 4);
    enc->setBytes(&ms, sizeof(ms), 5);
    enc->dispatchThreadgroups(count_2d, tg_2d);

    // Stage 2: Union pass
    enc->setComputePipelineState(pso_ccl_union_);
    enc->setBuffer(thresh_buf, 0, 0);
    enc->setBuffer(parent_buf, 0, 1);
    enc->setBytes(&mw, sizeof(mw), 2);
    enc->setBytes(&mh, sizeof(mh), 3);
    enc->setBytes(&ms, sizeof(ms), 4);
    enc->dispatchThreadgroups(count_2d, tg_2d);

    // Stage 3: Flatten
    enc->setComputePipelineState(pso_ccl_flatten_);
    enc->setBuffer(parent_buf, 0, 0);
    enc->setBytes(&tot_px, sizeof(tot_px), 1);
    enc->dispatchThreadgroups(count_1d, tg_1d);

    // Stage 4: Compute component sizes
    enc->setComputePipelineState(pso_ccl_sizes_);
    enc->setBuffer(parent_buf, 0, 0);
    enc->setBuffer(sizes_buf, 0, 1);
    enc->setBytes(&tot_px, sizeof(tot_px), 2);
    enc->dispatchThreadgroups(count_1d, tg_1d);

    enc->endEncoding();

    auto t0 = Clock::now();
    cmd->commit();
    cmd->waitUntilCompleted();
    auto t1 = Clock::now();

    double hw_us = (cmd->GPUEndTime() - cmd->GPUStartTime()) * 1e6;
    if (hw_us > 0.0) {
      gpu_ccl_total_us += hw_us;
    } else {
      gpu_ccl_total_us +=
          std::chrono::duration<double, std::micro>(t1 - t0).count();
    }
  }

  double gpu_ccl_avg_us = gpu_ccl_total_us / kIterations;
  double ccl_speedup = cpu_ccl_avg_us / gpu_ccl_avg_us;

  // Parity check: Compare component counts with size >= 25
  const uint32_t* gpu_parents =
      static_cast<const uint32_t*>(parent_buf->contents());
  const uint32_t* gpu_sizes =
      static_cast<const uint32_t*>(sizes_buf->contents());

  uint32_t cpu_valid_components = 0;
  uint32_t gpu_valid_components = 0;

  for (uint32_t i = 0; i < total_pixels; ++i) {
    if (cpu_uf->parent[i] == i && cpu_uf->size[i] + 1 >= 25) {
      cpu_valid_components++;
    }
    if (gpu_parents[i] == i && gpu_sizes[i] >= 25) {
      gpu_valid_components++;
    }
  }

  std::cout
      << "\n=========== GPU Connected Components (CCL) Benchmark ==========="
      << '\n';
  std::cout << "  Input Grid Size:       " << w << " x " << h << " ("
            << total_pixels << " pixels)" << '\n';
  std::cout << "  CPU connected_components: " << std::fixed
            << std::setprecision(1) << cpu_ccl_avg_us << " us" << '\n';
  std::cout << "  Metal GPU CCL:            " << std::fixed
            << std::setprecision(1) << gpu_ccl_avg_us << " us" << '\n';
  std::cout << "  CCL Speedup:              " << std::fixed
            << std::setprecision(2) << ccl_speedup << "x faster on GPU!"
            << '\n';
  std::cout << "  Valid Components (CPU):   " << cpu_valid_components << '\n';
  std::cout << "  Valid Components (GPU):   " << gpu_valid_components << '\n';
  std::cout
      << "=================================================================\n"
      << '\n';

  EXPECT_GT(gpu_valid_components, 0u);
  EXPECT_EQ(gpu_valid_components, cpu_valid_components);

  thresh_buf->release();
  parent_buf->release();
  sizes_buf->release();
  unionfind_destroy(cpu_uf);
  image_u8_destroy(threshim);
  image_u8_destroy(dec_im);
#endif
}

}  // namespace
}  // namespace talos::vision::metal
