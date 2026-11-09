#include "talOS/vision/metal/metal_apriltag_backend.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <format>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <utility>
#include <vector>

#include "talOS/geometry/se3.h"
#include "talOS/geometry/so3.h"
#include "talOS/vision/apriltag/cpu_apriltag_backend.h"
#include "talOS/vision/compute_backend.h"
#include "talOS/vision/localization.h"
#include "talOS/vision/metal/apriltag_metal_adapter.h"
#include "talOS/vision/metal/gpu_sparse_clusters.h"
#include "talOS/vision/metal/metal_shaders.h"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/image_u8.h"
#include "common/image_u8x3.h"
#include "common/pjpeg.h"
#include "common/timeprofile.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
}

#if defined(__APPLE__)
#define NS_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION
#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#endif

namespace talos::vision::metal {

namespace {

#if defined(__APPLE__)
uint32_t FormatToMetalCode(PixelFormat format) {
  switch (format) {
    case PixelFormat::kRgb8:
      return 0;
    case PixelFormat::kBgra8:
      return 5;
    case PixelFormat::kYuyv:
      return 2;
    case PixelFormat::kBgr8:
      return 4;
    case PixelFormat::kGray8:
    default:
      return 3;
  }
}
#endif

apriltag_family_t* CreateFamily(const std::string& name) {
  if (name == "tag16h5") return tag16h5_create();
  if (name == "tag25h9") return tag25h9_create();
  return tag36h11_create();
}

void DestroyFamily(const std::string& name, apriltag_family_t* tf) {
  if (!tf) return;
  if (name == "tag16h5") {
    tag16h5_destroy(tf);
  } else if (name == "tag25h9") {
    tag25h9_destroy(tf);
  } else {
    tag36h11_destroy(tf);
  }
}

// Stage 2 (GPU connected-components) is default-ON in Detect();
// TALOS_METAL_STAGE2=0 opts out (keeps the Stage 1 CPU path).
// Read per call so tests can toggle.
//
// NOTE: earlier Stage 3 (GPU line-fit terms), window-fit batch, and fused
// integer-prefix experiments were removed: each held parity but measured
// slower than the CPU sweep on sparse and dense scenes (dispatch/gather
// overhead exceeds the relocated fits; blur/pick/combos dominate and stay
// CPU). Quad fitting therefore stays on CPU by measurement, not by default.

}  // namespace

struct MetalAprilTagBackend::Impl {
  talos::vision::apriltag::DetectorOptions options;
  // Recursive: Detect() holds the lock while invoking RunGpuThreshold(),
  // which also locks to protect the shared cached buffers.
  std::recursive_mutex mutex;
  apriltag_family_t* tf{nullptr};
  apriltag_detector_t* td{nullptr};

  talos::vision::StageTimings last_timings;
  bool is_fallback{false};
  bool last_cpu_fallback{false};
  std::unique_ptr<talos::vision::CpuAprilTagBackend> compute_cpu_backend;
  std::string device_name{"CPU"};

  std::unique_ptr<talos::vision::apriltag::CpuAprilTagBackend> cpu_backend;

#if defined(__APPLE__)
  MTL::Device* device{nullptr};
  MTL::Library* library{nullptr};
  MTL::CommandQueue* queue{nullptr};
  MTL::ComputePipelineState* pso_rgb_to_gray{nullptr};
  MTL::ComputePipelineState* pso_decimate_2x{nullptr};
  MTL::ComputePipelineState* pso_rgb_to_gray_decimate_2x{nullptr};
  MTL::ComputePipelineState* pso_grad_thresh{nullptr};
  MTL::ComputePipelineState* pso_cand_mask{nullptr};
  MTL::ComputePipelineState* pso_tile_min_max_4x4{nullptr};
  MTL::ComputePipelineState* pso_tile_blur_and_threshold{nullptr};

  MTL::Buffer* cached_in_buf{nullptr};
  size_t cached_in_capacity{0};
  MTL::Buffer* cached_gray_buf{nullptr};
  MTL::Buffer* cached_grad_mag_buf{nullptr};
  MTL::Buffer* cached_grad_dir_buf{nullptr};
  MTL::Buffer* cached_thresh_buf{nullptr};
  size_t cached_thresh_capacity{0};
  MTL::Buffer* cached_im_min_buf{nullptr};
  size_t cached_im_min_capacity{0};
  MTL::Buffer* cached_im_max_buf{nullptr};
  size_t cached_im_max_capacity{0};
  MTL::Buffer* cached_cand_mask_buf{nullptr};
  size_t cached_gray_capacity{0};
  MTL::Buffer* cached_dec_buf{nullptr};
  size_t cached_dec_capacity{0};
  // Stage 2: GPU CCL path with CPU fallback preserved. All StorageModeShared,
  // grow-only, like cached_thresh_buf. Lifetimes are Impl members so contents
  // outlive Detect().
  MTL::Buffer* cached_parent_buf{nullptr};
  size_t cached_parent_capacity{0};
  MTL::Buffer* cached_size_buf{nullptr};
  size_t cached_size_capacity{0};
  MTL::Buffer* cached_cand_buf{nullptr};
  size_t cached_cand_capacity{0};
  MTL::Buffer* cached_cand_count_buf{nullptr};
  MTL::ComputePipelineState* pso_ccl_init{nullptr};
  MTL::ComputePipelineState* pso_ccl_union{nullptr};
  MTL::ComputePipelineState* pso_ccl_flatten{nullptr};
  MTL::ComputePipelineState* pso_ccl_sizes{nullptr};
  MTL::ComputePipelineState* pso_extract_cands{nullptr};
#endif

  explicit Impl(const talos::vision::apriltag::DetectorOptions& opts)
      : options{opts},
        cpu_backend{
            std::make_unique<talos::vision::apriltag::CpuAprilTagBackend>(
                opts)} {
    compute_cpu_backend =
        std::make_unique<talos::vision::CpuAprilTagBackend>(opts);
    InitDetector();
    InitMetal();
  }

  ~Impl() {
    DestroyDetector();
    DestroyMetal();
  }

  void InitDetector() {
    tf = CreateFamily(options.family_name);
    td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, options.max_hamming);
    td->nthreads = options.threads;
    td->quad_decimate = options.quad_decimate;
    td->quad_sigma = options.quad_sigma;
    td->refine_edges = options.refine_edges ? 1 : 0;
    td->decode_sharpening = options.decode_sharpening;
  }

  void DestroyDetector() {
    if (td) {
      apriltag_detector_destroy(td);
      td = nullptr;
    }
    if (tf) {
      DestroyFamily(options.family_name, tf);
      tf = nullptr;
    }
  }

  void DestroyMetal() {
#if defined(__APPLE__)
    if (cached_in_buf) {
      cached_in_buf->release();
      cached_in_buf = nullptr;
    }
    if (cached_gray_buf) {
      cached_gray_buf->release();
      cached_gray_buf = nullptr;
    }
    if (cached_grad_mag_buf) {
      cached_grad_mag_buf->release();
      cached_grad_mag_buf = nullptr;
    }
    if (cached_grad_dir_buf) {
      cached_grad_dir_buf->release();
      cached_grad_dir_buf = nullptr;
    }
    if (cached_thresh_buf) {
      cached_thresh_buf->release();
      cached_thresh_buf = nullptr;
    }
    if (cached_im_min_buf) {
      cached_im_min_buf->release();
      cached_im_min_buf = nullptr;
    }
    if (cached_im_max_buf) {
      cached_im_max_buf->release();
      cached_im_max_buf = nullptr;
    }
    if (cached_cand_mask_buf) {
      cached_cand_mask_buf->release();
      cached_cand_mask_buf = nullptr;
    }
    if (cached_dec_buf) {
      cached_dec_buf->release();
      cached_dec_buf = nullptr;
    }
    if (cached_parent_buf) {
      cached_parent_buf->release();
      cached_parent_buf = nullptr;
    }
    if (cached_size_buf) {
      cached_size_buf->release();
      cached_size_buf = nullptr;
    }
    if (cached_cand_buf) {
      cached_cand_buf->release();
      cached_cand_buf = nullptr;
    }
    if (cached_cand_count_buf) {
      cached_cand_count_buf->release();
      cached_cand_count_buf = nullptr;
    }
    if (pso_ccl_init) {
      pso_ccl_init->release();
      pso_ccl_init = nullptr;
    }
    if (pso_ccl_union) {
      pso_ccl_union->release();
      pso_ccl_union = nullptr;
    }
    if (pso_ccl_flatten) {
      pso_ccl_flatten->release();
      pso_ccl_flatten = nullptr;
    }
    if (pso_ccl_sizes) {
      pso_ccl_sizes->release();
      pso_ccl_sizes = nullptr;
    }
    if (pso_extract_cands) {
      pso_extract_cands->release();
      pso_extract_cands = nullptr;
    }
    if (pso_rgb_to_gray) {
      pso_rgb_to_gray->release();
      pso_rgb_to_gray = nullptr;
    }
    if (pso_decimate_2x) {
      pso_decimate_2x->release();
      pso_decimate_2x = nullptr;
    }
    if (pso_rgb_to_gray_decimate_2x) {
      pso_rgb_to_gray_decimate_2x->release();
      pso_rgb_to_gray_decimate_2x = nullptr;
    }
    if (pso_grad_thresh) {
      pso_grad_thresh->release();
      pso_grad_thresh = nullptr;
    }
    if (pso_cand_mask) {
      pso_cand_mask->release();
      pso_cand_mask = nullptr;
    }
    if (pso_tile_min_max_4x4) {
      pso_tile_min_max_4x4->release();
      pso_tile_min_max_4x4 = nullptr;
    }
    if (pso_tile_blur_and_threshold) {
      pso_tile_blur_and_threshold->release();
      pso_tile_blur_and_threshold = nullptr;
    }
    if (queue) {
      queue->release();
      queue = nullptr;
    }
    if (library) {
      library->release();
      library = nullptr;
    }
    if (device) {
      device->release();
      device = nullptr;
    }
#endif
  }

  void InitMetal() {
#if defined(__APPLE__)
    device = MTL::CreateSystemDefaultDevice();
    if (device != nullptr) {
      NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();

      NS::String* source =
          NS::String::string(kMetalShadersSource, NS::UTF8StringEncoding);
      MTL::CompileOptions* compile_options =
          MTL::CompileOptions::alloc()->init();
      NS::Error* error = nullptr;

      library = device->newLibrary(source, compile_options, &error);
      compile_options->release();

      if (library != nullptr) {
        NS::String* n_rgb =
            NS::String::string("rgb_to_gray", NS::UTF8StringEncoding);
        NS::String* n_dec =
            NS::String::string("decimate_2x", NS::UTF8StringEncoding);
        NS::String* n_fused = NS::String::string("rgb_to_gray_decimate_2x",
                                                 NS::UTF8StringEncoding);
        NS::String* n_gt = NS::String::string("gradient_and_threshold",
                                              NS::UTF8StringEncoding);
        NS::String* n_cm =
            NS::String::string("candidate_mask", NS::UTF8StringEncoding);
        NS::String* n_tmm =
            NS::String::string("tile_min_max_4x4", NS::UTF8StringEncoding);
        NS::String* n_tbt = NS::String::string("tile_blur_and_threshold",
                                               NS::UTF8StringEncoding);

        MTL::Function* fn_rgb = library->newFunction(n_rgb);
        MTL::Function* fn_dec = library->newFunction(n_dec);
        MTL::Function* fn_fused = library->newFunction(n_fused);
        MTL::Function* fn_gt = library->newFunction(n_gt);
        MTL::Function* fn_cm = library->newFunction(n_cm);
        MTL::Function* fn_tmm = library->newFunction(n_tmm);
        MTL::Function* fn_tbt = library->newFunction(n_tbt);

        NS::Error* pso_err = nullptr;
        pso_rgb_to_gray = device->newComputePipelineState(fn_rgb, &pso_err);
        pso_decimate_2x = device->newComputePipelineState(fn_dec, &pso_err);
        pso_rgb_to_gray_decimate_2x =
            device->newComputePipelineState(fn_fused, &pso_err);
        pso_grad_thresh = device->newComputePipelineState(fn_gt, &pso_err);
        pso_cand_mask = device->newComputePipelineState(fn_cm, &pso_err);
        pso_tile_min_max_4x4 =
            device->newComputePipelineState(fn_tmm, &pso_err);
        pso_tile_blur_and_threshold =
            device->newComputePipelineState(fn_tbt, &pso_err);

        fn_rgb->release();
        fn_dec->release();
        fn_fused->release();
        fn_gt->release();
        fn_cm->release();
        fn_tmm->release();
        fn_tbt->release();

        // Stage 2: GPU CCL + edge candidates. Tolerate nullptr (missing kernel
        // or PSO failure auto-disables Stage 2; Stage 1 remains the default).
        auto make_stage2_pso = [&](const char* kernel_name) {
          NS::String* n =
              NS::String::string(kernel_name, NS::UTF8StringEncoding);
          MTL::Function* fn = library->newFunction(n);
          if (fn == nullptr) {
            return static_cast<MTL::ComputePipelineState*>(nullptr);
          }
          NS::Error* e = nullptr;
          MTL::ComputePipelineState* pso =
              device->newComputePipelineState(fn, &e);
          fn->release();
          return pso;
        };
        pso_ccl_init = make_stage2_pso("ccl_init");
        pso_ccl_union = make_stage2_pso("ccl_union_pass");
        pso_ccl_flatten = make_stage2_pso("ccl_flatten");
        pso_ccl_sizes = make_stage2_pso("ccl_compute_sizes");
        pso_extract_cands = make_stage2_pso("extract_edge_candidates");

        if (pso_rgb_to_gray && pso_decimate_2x && pso_rgb_to_gray_decimate_2x &&
            pso_grad_thresh && pso_cand_mask && pso_tile_min_max_4x4 &&
            pso_tile_blur_and_threshold) {
          queue = device->newCommandQueue();
          device_name = device->name()->utf8String();
          is_fallback = false;
        } else {
          is_fallback = true;
          device_name = "CPU (Pipeline creation failed)";
        }
      } else {
        is_fallback = true;
        device_name = "CPU (Shader compilation failed)";
      }
      pool->release();
    } else {
      is_fallback = true;
      device_name = "CPU (No Metal Device)";
    }
#else
    is_fallback = true;
    device_name = "CPU (Non-Apple Platform)";
#endif
  }

  void SetOptions(const talos::vision::apriltag::DetectorOptions& opts) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    options = opts;
    cpu_backend->SetOptions(opts);
    compute_cpu_backend =
        std::make_unique<talos::vision::CpuAprilTagBackend>(opts);
    DestroyDetector();
    InitDetector();
  }

  std::vector<talos::vision::apriltag::Detection> CpuDetect(
      const uint8_t* data, int width, int height, int stride,
      PixelFormat format,
      const talos::vision::apriltag::CameraIntrinsics* p_intrinsics,
      double tag_size_meters) {
    const int row_stride = stride > 0
                               ? stride
                               : static_cast<int>(static_cast<size_t>(width) *
                                                  BytesPerPixel(format));
    Frame frame(0, 0, 0, event::steady_now(), width, height, format,
                row_stride);
    std::memcpy(frame.mutable_data(), data,
                static_cast<size_t>(row_stride) * height);
    const auto result = compute_cpu_backend->Execute(frame);
    last_timings = result.timings;
    last_cpu_fallback = true;
    std::vector<talos::vision::apriltag::Detection> detections;
    for (const auto& raw : result.detections) {
      talos::vision::apriltag::Detection d;
      d.id = raw.id;
      d.hamming = raw.hamming;
      d.decision_margin = raw.decision_margin;
      d.center = {raw.center_x, raw.center_y};
      for (size_t i = 0; i < 4; ++i) {
        d.corners[i] = {raw.corners[i * 2], raw.corners[i * 2 + 1]};
      }
      for (int i = 0; i < 9; ++i) {
        d.homography(i / 3, i % 3) = raw.homography[i];
      }
      if (p_intrinsics && tag_size_meters > 0) {
        d.pose = cpu_backend->EstimatePose(d, *p_intrinsics, tag_size_meters);
      }
      detections.push_back(d);
    }
    return detections;
  }

  std::vector<talos::vision::apriltag::Detection> Detect(
      const uint8_t* data, int width, int height, int stride,
      PixelFormat format,
      const talos::vision::apriltag::CameraIntrinsics* p_intrinsics,
      double tag_size_meters) {
    if (data == nullptr || width <= 0 || height <= 0) {
      return {};
    }

    std::lock_guard<std::recursive_mutex> detect_lock(mutex);
    last_cpu_fallback = false;
    if (is_fallback || options.quad_sigma != 0 ||
        (options.quad_decimate != 1 && options.quad_decimate != 2)) {
      return CpuDetect(data, width, height, stride, format, p_intrinsics,
                       tag_size_meters);
    }

#if defined(__APPLE__)
    std::lock_guard<std::recursive_mutex> lock(mutex);
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();

    // Fresh counters every frame: threshold/CCL accumulate with += below,
    // so stale values must not carry over.
    last_timings = talos::vision::StageTimings{};

    const size_t bpp = BytesPerPixel(format);
    const int actual_stride =
        (stride > 0) ? stride : width * static_cast<int>(bpp);
    const size_t in_bytes = static_cast<size_t>(actual_stride) * height;
    const size_t gray_bytes = static_cast<size_t>(width) * height;

    MTL::Buffer* in_buf = nullptr;
    bool in_buf_needs_release = false;
    if (reinterpret_cast<uintptr_t>(data) % 4096 == 0) {
      in_buf = device->newBuffer(data, in_bytes, MTL::ResourceStorageModeShared,
                                 nullptr);
      if (in_buf != nullptr) {
        in_buf_needs_release = true;
      }
    }
    const auto cpu_fallback = [&]() {
      if (in_buf_needs_release) in_buf->release();
      pool->release();
      return CpuDetect(data, width, height, stride, format, p_intrinsics,
                       tag_size_meters);
    };
    if (in_buf == nullptr) {
      if (!cached_in_buf || cached_in_capacity < in_bytes) {
        if (cached_in_buf) cached_in_buf->release();
        cached_in_buf =
            device->newBuffer(in_bytes, MTL::ResourceStorageModeShared);
        cached_in_capacity = in_bytes;
      }
      if (!cached_in_buf || !cached_in_buf->contents()) return cpu_fallback();
      std::memcpy(cached_in_buf->contents(), data, in_bytes);
      in_buf = cached_in_buf;
    }
    if (!in_buf->contents()) return cpu_fallback();

    const bool use_gpu_decimated = (options.quad_decimate >= 2.0f);
    MTL::Buffer* dec_buf = nullptr;
    MTL::Buffer* gray_buf = nullptr;

    if (use_gpu_decimated) {
      uint32_t dec_bytes = (width / 2) * (height / 2);
      if (!cached_dec_buf || cached_dec_capacity < dec_bytes) {
        if (cached_dec_buf) cached_dec_buf->release();
        cached_dec_buf =
            device->newBuffer(dec_bytes, MTL::ResourceStorageModeShared);
        cached_dec_capacity = dec_bytes;
      }
      dec_buf = cached_dec_buf;
    }
    if (!cached_gray_buf || cached_gray_capacity < gray_bytes) {
      if (cached_gray_buf) cached_gray_buf->release();
      cached_gray_buf =
          device->newBuffer(gray_bytes, MTL::ResourceStorageModeShared);
      cached_gray_capacity = gray_bytes;
    }
    gray_buf = cached_gray_buf;
    if (!gray_buf || !gray_buf->contents() ||
        (use_gpu_decimated && (!dec_buf || !dec_buf->contents()))) {
      return cpu_fallback();
    }

    MTL::CommandBuffer* cmd_buf = queue->commandBuffer();
    MTL::ComputeCommandEncoder* enc = cmd_buf->computeCommandEncoder();

    uint32_t w = width;
    uint32_t h = height;
    uint32_t in_s = actual_stride;
    uint32_t fmt = FormatToMetalCode(format);
    MTL::Size tg_size = MTL::Size::Make(16, 16, 1);

    if (format == PixelFormat::kGray8 && actual_stride == width) {
      std::memcpy(gray_buf->contents(), data, gray_bytes);
    } else {
      enc->setComputePipelineState(pso_rgb_to_gray);
      enc->setBuffer(in_buf, 0, 0);
      enc->setBuffer(gray_buf, 0, 1);
      enc->setBytes(&w, sizeof(w), 2);
      enc->setBytes(&h, sizeof(h), 3);
      enc->setBytes(&in_s, sizeof(in_s), 4);
      uint32_t out_s = width;
      enc->setBytes(&out_s, sizeof(out_s), 5);
      enc->setBytes(&fmt, sizeof(fmt), 6);
      enc->dispatchThreadgroups(
          MTL::Size::Make((width + 15) / 16, (height + 15) / 16, 1), tg_size);
      enc->memoryBarrier(MTL::BarrierScopeBuffers);
    }
    if (use_gpu_decimated) {
      enc->setComputePipelineState(pso_rgb_to_gray_decimate_2x);
      enc->setBuffer(in_buf, 0, 0);
      enc->setBuffer(dec_buf, 0, 1);
      enc->setBytes(&w, sizeof(w), 2);
      enc->setBytes(&h, sizeof(h), 3);
      enc->setBytes(&in_s, sizeof(in_s), 4);
      uint32_t out_s = width / 2;
      enc->setBytes(&out_s, sizeof(out_s), 5);
      enc->setBytes(&fmt, sizeof(fmt), 6);
      MTL::Size dec_tg_count =
          MTL::Size::Make((width / 2 + 15) / 16, (height / 2 + 15) / 16, 1);
      enc->dispatchThreadgroups(dec_tg_count, tg_size);
    }

    enc->memoryBarrier(MTL::BarrierScopeBuffers);

    const int detect_w = use_gpu_decimated ? (width / 2) : width;
    const int detect_h = use_gpu_decimated ? (height / 2) : height;

    image_u8_t im{
        .width = width,
        .height = height,
        .stride = width,
        .buf = static_cast<uint8_t*>(gray_buf->contents()),
    };

    // Stage 1: run the proven GPU threshold over the detect-resolution gray
    // image and feed it to the detector via the threshim adapter, bypassing
    // the CPU threshold (~1,650 us). Same min_white_black_diff contract as
    // the CPU threshold() the parity tests assert against. The thresh buffer
    // is an Impl member, so its contents outlive the detect call below.
    // NULL threshim falls back to the standard detect path inside the adapter.
    image_u8_t* threshim_ptr = nullptr;
    // image_u8_t has const width/height/stride, so it must be initialized in
    // place; only the non-const buf pointer is filled in after the GPU wait.
    image_u8_t threshim{
        .width = detect_w,
        .height = detect_h,
        .stride = detect_w,
        .buf = nullptr,
    };
    MTL::Buffer* detect_gray_buf = use_gpu_decimated ? dec_buf : gray_buf;
    if (pso_tile_min_max_4x4 != nullptr &&
        pso_tile_blur_and_threshold != nullptr && detect_gray_buf != nullptr &&
        detect_w >= 4 && detect_h >= 4) {
      const bool threshold_ok = RunGpuThreshold(
          cmd_buf, detect_gray_buf, nullptr, static_cast<uint32_t>(detect_w),
          static_cast<uint32_t>(detect_h), static_cast<uint32_t>(detect_w),
          static_cast<uint32_t>(td->qtp.min_white_black_diff), enc);
      enc->memoryBarrier(MTL::BarrierScopeBuffers);
      if (threshold_ok && cached_thresh_buf != nullptr &&
          cached_thresh_buf->contents() != nullptr) {
        threshim.buf = static_cast<uint8_t*>(cached_thresh_buf->contents());
        threshim_ptr = &threshim;
      }
    }

    // Stage 2: GPU CCL path with CPU fallback preserved (default ON).
    // Opt out via TALOS_METAL_STAGE2=0. Runs ccl_init -> ccl_union_pass ->
    // ccl_flatten -> ccl_compute_sizes -> extract_edge_candidates over the
    // detect-resolution thresh buffer, reading back O(candidates, ~5-10% px)
    // instead of O(pixels). Any missing PSO, allocation failure, or disabled
    // flag falls through to the Stage 1 threshim path below with identical
    // 0.1px parity. Buffers are grow-only Impl members (StorageModeShared) so
    // lifetimes outlive this call. GPU time is accumulated into timings.
    const uint32_t* stage2_parents = nullptr;
    const uint32_t* stage2_sizes = nullptr;
    const uint32_t* stage2_cands = nullptr;
    uint32_t stage2_cand_count = 0;
    if (threshim_ptr != nullptr) {
      // Stage 2 (GPU CCL + sparse clustering) is default-ON: parity-proven
      // against CPU on all fixtures. Opt out per-process with
      // TALOS_METAL_STAGE2=0 (keeps the Stage 1 CPU-CC path).
      const char* stage2_env = std::getenv("TALOS_METAL_STAGE2");
      const bool stage2_enabled =
          !(stage2_env != nullptr && stage2_env[0] == '0' &&
            stage2_env[1] == '\0');
      if (stage2_enabled && pso_ccl_init != nullptr &&
          pso_ccl_union != nullptr && pso_ccl_flatten != nullptr &&
          pso_ccl_sizes != nullptr && pso_extract_cands != nullptr &&
          detect_w > 0 && detect_h > 0) {
        const size_t npix =
            static_cast<size_t>(detect_w) * static_cast<size_t>(detect_h);
        if (npix > 0 && npix <= static_cast<size_t>(64) * 1024 * 1024) {
          const size_t parent_bytes = npix * sizeof(uint32_t);
          if (cached_parent_buf == nullptr ||
              cached_parent_capacity < parent_bytes) {
            if (cached_parent_buf) cached_parent_buf->release();
            cached_parent_buf =
                device->newBuffer(parent_bytes, MTL::ResourceStorageModeShared);
            cached_parent_capacity =
                (cached_parent_buf != nullptr) ? parent_bytes : 0;
          }
          if (cached_size_buf == nullptr ||
              cached_size_capacity < parent_bytes) {
            if (cached_size_buf) cached_size_buf->release();
            cached_size_buf =
                device->newBuffer(parent_bytes, MTL::ResourceStorageModeShared);
            cached_size_capacity =
                (cached_size_buf != nullptr) ? parent_bytes : 0;
          }
          // Candidate capacity at half the pixels: busy photographic frames
          // reach ~40% edge density, far above sparse assumptions. Slots are
          // packed uint32 (flags/y/x), one uint32 each.
          size_t max_cands = npix / 2;
          if (max_cands == 0) max_cands = 1;
          const size_t cand_bytes = max_cands * sizeof(uint32_t);
          if (cached_cand_buf == nullptr || cached_cand_capacity < max_cands) {
            if (cached_cand_buf) cached_cand_buf->release();
            cached_cand_buf =
                device->newBuffer(cand_bytes, MTL::ResourceStorageModeShared);
            cached_cand_capacity = (cached_cand_buf != nullptr) ? max_cands : 0;
          }
          if (cached_cand_count_buf == nullptr) {
            cached_cand_count_buf = device->newBuffer(
                sizeof(uint32_t), MTL::ResourceStorageModeShared);
          }
          if (cached_parent_buf != nullptr && cached_size_buf != nullptr &&
              cached_cand_buf != nullptr && cached_cand_count_buf != nullptr &&
              cached_thresh_buf != nullptr && cached_parent_buf->contents() &&
              cached_size_buf->contents() && cached_cand_buf->contents() &&
              cached_cand_count_buf->contents() &&
              cached_thresh_buf->contents()) {
            *static_cast<uint32_t*>(cached_cand_count_buf->contents()) = 0;
            MTL::ComputeCommandEncoder* cenc = enc;
            const uint32_t dw = static_cast<uint32_t>(detect_w);
            const uint32_t dh = static_cast<uint32_t>(detect_h);
            const uint32_t ds = static_cast<uint32_t>(detect_w);
            const uint32_t total = static_cast<uint32_t>(npix);
            const uint32_t maxc = static_cast<uint32_t>(max_cands);
            const MTL::Size tg2 = MTL::Size::Make(16, 16, 1);
            const MTL::Size n2 =
                MTL::Size::Make((dw + 15) / 16, (dh + 15) / 16, 1);
            cenc->setComputePipelineState(pso_ccl_init);
            cenc->setBuffer(cached_thresh_buf, 0, 0);
            cenc->setBuffer(cached_parent_buf, 0, 1);
            cenc->setBuffer(cached_size_buf, 0, 2);
            cenc->setBytes(&dw, sizeof(dw), 3);
            cenc->setBytes(&dh, sizeof(dh), 4);
            cenc->setBytes(&ds, sizeof(ds), 5);
            cenc->dispatchThreadgroups(n2, tg2);
            cenc->memoryBarrier(MTL::BarrierScopeBuffers);
            cenc->setComputePipelineState(pso_ccl_union);
            cenc->setBuffer(cached_thresh_buf, 0, 0);
            cenc->setBuffer(cached_parent_buf, 0, 1);
            cenc->setBytes(&dw, sizeof(dw), 2);
            cenc->setBytes(&dh, sizeof(dh), 3);
            cenc->setBytes(&ds, sizeof(ds), 4);
            cenc->dispatchThreadgroups(n2, tg2);
            cenc->memoryBarrier(MTL::BarrierScopeBuffers);
            const MTL::Size tg1 = MTL::Size::Make(256, 1, 1);
            const MTL::Size n1 = MTL::Size::Make((total + 255) / 256, 1, 1);
            cenc->setComputePipelineState(pso_ccl_flatten);
            cenc->setBuffer(cached_parent_buf, 0, 0);
            cenc->setBytes(&total, sizeof(total), 1);
            cenc->dispatchThreadgroups(n1, tg1);
            cenc->memoryBarrier(MTL::BarrierScopeBuffers);
            cenc->setComputePipelineState(pso_ccl_sizes);
            cenc->setBuffer(cached_parent_buf, 0, 0);
            cenc->setBuffer(cached_size_buf, 0, 1);
            cenc->setBytes(&total, sizeof(total), 2);
            cenc->dispatchThreadgroups(n1, tg1);
            cenc->memoryBarrier(MTL::BarrierScopeBuffers);
            cenc->setComputePipelineState(pso_extract_cands);
            cenc->setBuffer(cached_thresh_buf, 0, 0);
            cenc->setBuffer(cached_cand_buf, 0, 1);
            cenc->setBuffer(cached_cand_count_buf, 0, 2);
            cenc->setBytes(&dw, sizeof(dw), 3);
            cenc->setBytes(&dh, sizeof(dh), 4);
            cenc->setBytes(&ds, sizeof(ds), 5);
            cenc->setBytes(&maxc, sizeof(maxc), 6);
            cenc->dispatchThreadgroups(n2, tg2);
            stage2_parents =
                static_cast<const uint32_t*>(cached_parent_buf->contents());
            stage2_sizes =
                static_cast<const uint32_t*>(cached_size_buf->contents());
          }
        }
      }
    }

    enc->endEncoding();
    cmd_buf->commit();
    cmd_buf->waitUntilCompleted();
    if (cmd_buf->status() != MTL::CommandBufferStatusCompleted) {
      if (in_buf_needs_release) in_buf->release();
      pool->release();
      return CpuDetect(data, width, height, stride, format, p_intrinsics,
                       tag_size_meters);
    }
    last_timings.threshold_us = static_cast<uint64_t>(
        (cmd_buf->GPUEndTime() - cmd_buf->GPUStartTime()) * 1e6);
    if (stage2_parents != nullptr) {
      const uint32_t count =
          *static_cast<uint32_t*>(cached_cand_count_buf->contents());
      if (count <= cached_cand_capacity) {
        stage2_cands =
            static_cast<const uint32_t*>(cached_cand_buf->contents());
        stage2_cand_count = count;
      }
    }

    // Stage 2b: sparse clusters straight from GPU labels + candidates with
    // zero dense CPU touch (no O(n) unionfind conversion). Falls back to the
    // Stage 2a labels path, then Stage 1, preserving parity at each step.
    zarray_t* stage2b_clusters = nullptr;
    if (stage2_parents != nullptr && stage2_sizes != nullptr &&
        stage2_cands != nullptr && threshim_ptr != nullptr) {
      stage2b_clusters = BuildSparseClustersFromGpuLabels(
          threshim_ptr, detect_w, detect_h, detect_w, stage2_parents,
          stage2_sizes, stage2_cands, stage2_cand_count);
    }

    auto t_detect_start = std::chrono::steady_clock::now();
    zarray_t* raw_detections = nullptr;
    // Quad fitting stays on CPU by measurement: GPU line-fit terms, the
    // window-fit batch, and fused integer prefixes each held parity but
    // measured slower than the CPU sweep on sparse and dense scenes, so
    // those experiments were removed. See the namespace note above.
    if (stage2b_clusters != nullptr) {
      raw_detections = apriltag_detector_detect_with_gpu_clusters(
          td, &im, threshim_ptr, stage2b_clusters);
    } else if (stage2_parents != nullptr && stage2_sizes != nullptr) {
      raw_detections = apriltag_detector_detect_with_gpu_labels(
          td, &im, threshim_ptr, stage2_parents, stage2_sizes);
    } else {
      raw_detections =
          apriltag_detector_detect_with_threshim(td, &im, threshim_ptr);
    }
    auto t_detect_end = std::chrono::steady_clock::now();

    uint64_t detect_total_us =
        std::chrono::duration_cast<std::chrono::microseconds>(t_detect_end -
                                                              t_detect_start)
            .count();

    // Time profile extraction
    int64_t start_time = 0;
    int64_t quad_time = 0;
    int num_stamps = zarray_size(td->tp->stamps);
    for (int i = 0; i < num_stamps; ++i) {
      struct timeprofile_entry* entry;
      zarray_get_volatile(td->tp->stamps, i, reinterpret_cast<void*>(&entry));
      if (i == 0) {
        start_time = entry->utime;
      }
      if (std::strcmp(entry->name, "quads") == 0) {
        quad_time = entry->utime - start_time;
      }
    }
    if (quad_time > 0 && num_stamps > 0) {
      struct timeprofile_entry* last_entry;
      zarray_get_volatile(td->tp->stamps, num_stamps - 1,
                          reinterpret_cast<void*>(&last_entry));
      int64_t total_detect_time = last_entry->utime - start_time;
      last_timings.quad_us = static_cast<uint64_t>(quad_time);
      last_timings.decode_us =
          static_cast<uint64_t>(total_detect_time - quad_time);
    } else {
      last_timings.quad_us = detect_total_us;
      last_timings.decode_us = 0;
    }

    auto t_pose_start = std::chrono::steady_clock::now();
    std::vector<talos::vision::apriltag::Detection> results;
    if (raw_detections) {
      const int count = zarray_size(raw_detections);
      results.reserve(count);

      for (int i = 0; i < count; ++i) {
        apriltag_detection_t* raw_det = nullptr;
        zarray_get(raw_detections, i, reinterpret_cast<void*>(&raw_det));
        if (!raw_det) continue;

        talos::vision::apriltag::Detection det;
        det.id = raw_det->id;
        det.hamming = raw_det->hamming;
        det.decision_margin = raw_det->decision_margin;
        det.center =
            talos::vision::apriltag::Point2D{raw_det->c[0], raw_det->c[1]};

        for (int j = 0; j < 4; ++j) {
          det.corners[j] = talos::vision::apriltag::Point2D{raw_det->p[j][0],
                                                            raw_det->p[j][1]};
        }

        if (raw_det->H != nullptr) {
          for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
              det.homography(r, c) = MATD_EL(raw_det->H, r, c);
            }
          }
        }

        if (p_intrinsics != nullptr && tag_size_meters > 0.0) {
          det.pose =
              cpu_backend->EstimatePose(det, *p_intrinsics, tag_size_meters);
        }

        results.push_back(std::move(det));
      }
      apriltag_detections_destroy(raw_detections);
    }

    auto t_pose_end = std::chrono::steady_clock::now();
    last_timings.pose_us =
        std::chrono::duration_cast<std::chrono::microseconds>(t_pose_end -
                                                              t_pose_start)
            .count();
    last_timings.total_us = last_timings.grayscale_us +
                            last_timings.threshold_us + last_timings.quad_us +
                            last_timings.decode_us + last_timings.pose_us;

    if (in_buf_needs_release && in_buf) {
      in_buf->release();
    }

    pool->release();
    return results;
#else
    if (p_intrinsics != nullptr && tag_size_meters > 0.0) {
      return CpuDetect(data, width, height, stride, format, p_intrinsics,
                       tag_size_meters);
    }
    return CpuDetect(data, width, height, stride, format, p_intrinsics,
                     tag_size_meters);
#endif
  }

  bool RunGpuThreshold(MTL::CommandBuffer* cmd_buf, MTL::Buffer* gray_buf,
                       MTL::Buffer* thresh_buf, uint32_t width, uint32_t height,
                       uint32_t stride, uint32_t min_diff,
                       MTL::ComputeCommandEncoder* shared_encoder = nullptr) {
#if defined(__APPLE__)
    if (!cmd_buf || !gray_buf || width == 0 || height == 0) {
      return false;
    }
    if (!pso_tile_min_max_4x4 || !pso_tile_blur_and_threshold) {
      return false;
    }

    std::lock_guard<std::recursive_mutex> lock(mutex);

    const uint32_t actual_stride = (stride > 0) ? stride : width;
    const uint32_t tw = width / 4;
    const uint32_t th = height / 4;
    if (tw == 0 || th == 0) {
      return false;
    }

    const size_t min_max_bytes = static_cast<size_t>(tw) * th;
    if (!cached_im_min_buf || cached_im_min_capacity < min_max_bytes) {
      if (cached_im_min_buf) cached_im_min_buf->release();
      cached_im_min_buf =
          device->newBuffer(min_max_bytes, MTL::ResourceStorageModeShared);
      cached_im_min_capacity = min_max_bytes;
    }
    if (!cached_im_max_buf || cached_im_max_capacity < min_max_bytes) {
      if (cached_im_max_buf) cached_im_max_buf->release();
      cached_im_max_buf =
          device->newBuffer(min_max_bytes, MTL::ResourceStorageModeShared);
      cached_im_max_capacity = min_max_bytes;
    }

    const size_t thresh_bytes = static_cast<size_t>(actual_stride) * height;
    MTL::Buffer* out_thresh = thresh_buf;
    if (!out_thresh) {
      if (!cached_thresh_buf || cached_thresh_capacity < thresh_bytes) {
        if (cached_thresh_buf) cached_thresh_buf->release();
        cached_thresh_buf =
            device->newBuffer(thresh_bytes, MTL::ResourceStorageModeShared);
        cached_thresh_capacity = thresh_bytes;
      }
      out_thresh = cached_thresh_buf;
    }
    if (!gray_buf->contents() || !cached_im_min_buf ||
        !cached_im_min_buf->contents() || !cached_im_max_buf ||
        !cached_im_max_buf->contents() || !out_thresh ||
        !out_thresh->contents()) {
      return false;
    }

    MTL::ComputeCommandEncoder* enc = shared_encoder != nullptr
                                          ? shared_encoder
                                          : cmd_buf->computeCommandEncoder();

    // Pass 1: tile_min_max_4x4
    enc->setComputePipelineState(pso_tile_min_max_4x4);
    enc->setBuffer(gray_buf, 0, 0);
    enc->setBuffer(cached_im_min_buf, 0, 1);
    enc->setBuffer(cached_im_max_buf, 0, 2);
    enc->setBytes(&width, sizeof(width), 3);
    enc->setBytes(&height, sizeof(height), 4);
    enc->setBytes(&actual_stride, sizeof(actual_stride), 5);

    const MTL::Size tg_size = MTL::Size::Make(16, 16, 1);
    const MTL::Size tg_count_min_max =
        MTL::Size::Make((tw + 15) / 16, (th + 15) / 16, 1);
    enc->dispatchThreadgroups(tg_count_min_max, tg_size);

    enc->memoryBarrier(MTL::BarrierScopeBuffers);

    // Pass 2: tile_blur_and_threshold
    enc->setComputePipelineState(pso_tile_blur_and_threshold);
    enc->setBuffer(gray_buf, 0, 0);
    enc->setBuffer(cached_im_min_buf, 0, 1);
    enc->setBuffer(cached_im_max_buf, 0, 2);
    enc->setBuffer(out_thresh, 0, 3);
    enc->setBytes(&width, sizeof(width), 4);
    enc->setBytes(&height, sizeof(height), 5);
    enc->setBytes(&actual_stride, sizeof(actual_stride), 6);
    enc->setBytes(&min_diff, sizeof(min_diff), 7);

    const MTL::Size tg_count_thresh =
        MTL::Size::Make((width + 15) / 16, (height + 15) / 16, 1);
    enc->dispatchThreadgroups(tg_count_thresh, tg_size);

    if (shared_encoder == nullptr) enc->endEncoding();
    return true;
#else
    (void)shared_encoder;
    (void)cmd_buf;
    (void)gray_buf;
    (void)thresh_buf;
    (void)width;
    (void)height;
    (void)stride;
    (void)min_diff;
    return false;
#endif
  }
};

MetalAprilTagBackend::MetalAprilTagBackend(
    const talos::vision::apriltag::DetectorOptions& options)
    : impl_{std::make_unique<Impl>(options)} {}

MetalAprilTagBackend::~MetalAprilTagBackend() = default;
MetalAprilTagBackend::MetalAprilTagBackend(MetalAprilTagBackend&&) noexcept =
    default;
MetalAprilTagBackend& MetalAprilTagBackend::operator=(
    MetalAprilTagBackend&&) noexcept = default;

std::string MetalAprilTagBackend::device_info() const {
  return std::format(
      "Metal GPU AprilTag [{}] (threads={}, decimate={:.1f}, sigma={:.1f})",
      impl_->device_name, impl_->options.threads, impl_->options.quad_decimate,
      impl_->options.quad_sigma);
}

bool MetalAprilTagBackend::supports_gpu() const { return !impl_->is_fallback; }

bool MetalAprilTagBackend::is_fallback() const { return impl_->is_fallback; }

const talos::vision::StageTimings& MetalAprilTagBackend::last_timings() const {
  return impl_->last_timings;
}

void MetalAprilTagBackend::SetOptions(
    const talos::vision::apriltag::DetectorOptions& options) {
  impl_->SetOptions(options);
}

const talos::vision::apriltag::DetectorOptions& MetalAprilTagBackend::options()
    const {
  return impl_->options;
}

std::vector<talos::vision::apriltag::Detection> MetalAprilTagBackend::Detect(
    const uint8_t* data, int width, int height, int stride,
    PixelFormat format) {
  return impl_->Detect(data, width, height, stride, format, nullptr, 0.0);
}

std::vector<talos::vision::apriltag::Detection> MetalAprilTagBackend::Detect(
    const uint8_t* data, int width, int height, int stride, PixelFormat format,
    const talos::vision::apriltag::CameraIntrinsics& intrinsics,
    double tag_size_meters) {
  return impl_->Detect(data, width, height, stride, format, &intrinsics,
                       tag_size_meters);
}

std::vector<talos::vision::apriltag::Detection>
MetalAprilTagBackend::DetectImageFile(const std::string& filepath) {
  return DetectImageFile(filepath, talos::vision::apriltag::CameraIntrinsics{},
                         0.0);
}

std::vector<talos::vision::apriltag::Detection>
MetalAprilTagBackend::DetectImageFile(
    const std::string& filepath,
    const talos::vision::apriltag::CameraIntrinsics& intrinsics,
    double tag_size_meters) {
  image_u8_t* im = nullptr;

  if (filepath.ends_with(".jpg") || filepath.ends_with(".jpeg") ||
      filepath.ends_with(".JPG") || filepath.ends_with(".JPEG")) {
    pjpeg_t* pjpeg = pjpeg_create_from_file(filepath.c_str(), 0, nullptr);
    if (pjpeg != nullptr) {
      im = pjpeg_to_u8_baseline(pjpeg);
      pjpeg_destroy(pjpeg);
    }
  } else {
    // PNG and anything else: OpenCV grayscale decode (replaces the
    // upstream PNM loader; fixtures are PNG).
    const cv::Mat gray = cv::imread(filepath, cv::IMREAD_GRAYSCALE);
    if (!gray.empty() && gray.cols > 0 && gray.rows > 0) {
      im = image_u8_create(static_cast<uint32_t>(gray.cols),
                           static_cast<uint32_t>(gray.rows));
      for (int y = 0; y < gray.rows; ++y) {
        std::memcpy(im->buf + static_cast<size_t>(y) * im->stride,
                    gray.ptr<uint8_t>(y), static_cast<size_t>(gray.cols));
      }
    }
  }

  if (im == nullptr) {
    return {};
  }

  auto results = Detect(im->buf, im->width, im->height, im->stride,
                        PixelFormat::kGray8, intrinsics, tag_size_meters);
  image_u8_destroy(im);
  return results;
}

talos::vision::BackendExecutionResult MetalAprilTagBackend::Execute(
    const talos::vision::Frame& frame) {
  talos::vision::BackendExecutionResult result;
  if (frame.empty() || frame.width() == 0 || frame.height() == 0) {
    result.success = false;
    result.error_message = "Frame is empty or has invalid dimensions";
    return result;
  }

  // The fused GPU path implements exactly 1x/2x and no Gaussian filter.
  // Preserve requested semantics for every other setting, including color
  // input.
  if (impl_->is_fallback || impl_->options.quad_sigma != 0.0f ||
      (impl_->options.quad_decimate != 1.0f &&
       impl_->options.quad_decimate != 2.0f)) {
    result = impl_->compute_cpu_backend->Execute(frame);
    result.fallback_reason = impl_->is_fallback
                                 ? "Metal device unavailable"
                                 : "Metal supports decimation 1 or 2 with "
                                   "sigma 0; using CPU for requested settings";
    return result;
  }

  const auto execution_start = std::chrono::steady_clock::now();
  auto dets = Detect(frame.data(), static_cast<int>(frame.width()),
                     static_cast<int>(frame.height()),
                     static_cast<int>(frame.stride()), frame.format());

  result.timings = impl_->last_timings;
  result.effective_backend = impl_->last_cpu_fallback ? "cpu" : "metal";
  if (impl_->last_cpu_fallback) {
    result.fallback_reason = "GPU execution failed; used CPU";
  }
  result.timings.total_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - execution_start)
          .count();
  result.detections.reserve(dets.size());

  for (const auto& det : dets) {
    talos::vision::RawTagDetection raw;
    raw.id = det.id;
    raw.hamming = det.hamming;
    raw.decision_margin = det.decision_margin;
    raw.center_x = static_cast<float>(det.center.x);
    raw.center_y = static_cast<float>(det.center.y);
    for (size_t c = 0; c < 4; ++c) {
      raw.corners[c * 2] = static_cast<float>(det.corners[c].x);
      raw.corners[c * 2 + 1] = static_cast<float>(det.corners[c].y);
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        raw.homography[r * 3 + c] = static_cast<float>(det.homography(r, c));
      }
    }
    raw.valid = (det.hamming == 0);
    result.detections.push_back(raw);
  }

  result.success = true;
  return result;
}

void MetalAprilTagBackend::RunGpuThreshold(MTL::CommandBuffer* cmd_buf,
                                           MTL::Buffer* gray_buf,
                                           MTL::Buffer* thresh_buf,
                                           uint32_t width, uint32_t height,
                                           uint32_t stride, uint32_t min_diff) {
  impl_->RunGpuThreshold(cmd_buf, gray_buf, thresh_buf, width, height, stride,
                         min_diff);
}

MTL::Device* MetalAprilTagBackend::device() const {
#if defined(__APPLE__)
  return impl_->device;
#else
  return nullptr;
#endif
}

MTL::CommandQueue* MetalAprilTagBackend::queue() const {
#if defined(__APPLE__)
  return impl_->queue;
#else
  return nullptr;
#endif
}

MTL::Buffer* MetalAprilTagBackend::thresh_buffer() const {
#if defined(__APPLE__)
  return impl_->cached_thresh_buf;
#else
  return nullptr;
#endif
}

MTL::Buffer* MetalAprilTagBackend::min_buffer() const {
#if defined(__APPLE__)
  return impl_->cached_im_min_buf;
#else
  return nullptr;
#endif
}

MTL::Buffer* MetalAprilTagBackend::max_buffer() const {
#if defined(__APPLE__)
  return impl_->cached_im_max_buf;
#else
  return nullptr;
#endif
}

}  // namespace talos::vision::metal
