#include "talOS/vision/metal/metal_apriltag_backend.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <format>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "talOS/vision/apriltag/cpu_apriltag_backend.h"
#include "talOS/vision/compute_backend.h"
#include "talOS/vision/metal/apriltag_metal_adapter.h"
#include "talOS/vision/metal/gpu_sparse_clusters.h"
#include "talOS/vision/metal/metal_shaders.h"

extern "C" {
#include "apriltag.h"
#include "common/image_u8.h"
#include "common/timeprofile.h"
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

template <typename T>
void ReleaseAndNull(T*& ptr) {
  if (ptr != nullptr) {
    ptr->release();
    ptr = nullptr;
  }
}

// Grow-only StorageModeShared Metal buffer reused across frames.
struct CachedBuffer {
  MTL::Buffer* buffer{nullptr};
  size_t capacity{0};

  void Release() {
    ReleaseAndNull(buffer);
    capacity = 0;
  }

  MTL::Buffer* Ensure(MTL::Device* dev, size_t bytes) {
    if (buffer == nullptr || capacity < bytes) {
      Release();
      buffer = dev->newBuffer(bytes, MTL::ResourceStorageModeShared);
      capacity = (buffer != nullptr) ? bytes : 0;
    }
    return buffer;
  }
};

void Dispatch2D(MTL::ComputeCommandEncoder* enc, uint32_t width,
                uint32_t height) {
  const MTL::Size tg_size = MTL::Size::Make(16, 16, 1);
  const MTL::Size tg_count =
      MTL::Size::Make((static_cast<NS::UInteger>(width) + 15) / 16,
                      (static_cast<NS::UInteger>(height) + 15) / 16, 1);
  enc->dispatchThreadgroups(tg_count, tg_size);
}

void Dispatch1D(MTL::ComputeCommandEncoder* enc, uint32_t count) {
  const MTL::Size tg_size = MTL::Size::Make(256, 1, 1);
  const MTL::Size tg_count =
      MTL::Size::Make((static_cast<NS::UInteger>(count) + 255) / 256, 1, 1);
  enc->dispatchThreadgroups(tg_count, tg_size);
}

void EncodeColorConvert(MTL::ComputeCommandEncoder* enc,
                        MTL::ComputePipelineState* pso, MTL::Buffer* in_buf,
                        MTL::Buffer* out_buf, uint32_t width, uint32_t height,
                        uint32_t in_stride, uint32_t out_stride,
                        uint32_t format, uint32_t dispatch_w,
                        uint32_t dispatch_h) {
  enc->setComputePipelineState(pso);
  enc->setBuffer(in_buf, 0, 0);
  enc->setBuffer(out_buf, 0, 1);
  enc->setBytes(&width, sizeof(width), 2);
  enc->setBytes(&height, sizeof(height), 3);
  enc->setBytes(&in_stride, sizeof(in_stride), 4);
  enc->setBytes(&out_stride, sizeof(out_stride), 5);
  enc->setBytes(&format, sizeof(format), 6);
  Dispatch2D(enc, dispatch_w, dispatch_h);
}
#endif

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
  std::string active_family;

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
  MTL::ComputePipelineState* pso_rgb_to_gray_decimate_2x{nullptr};
  MTL::ComputePipelineState* pso_tile_min_max_4x4{nullptr};
  MTL::ComputePipelineState* pso_tile_blur_and_threshold{nullptr};

  CachedBuffer cached_in;
  CachedBuffer cached_gray;
  CachedBuffer cached_thresh;
  CachedBuffer cached_im_min;
  CachedBuffer cached_im_max;
  CachedBuffer cached_dec;
  // Stage 2: GPU CCL path with CPU fallback preserved. All StorageModeShared,
  // grow-only, like cached_thresh. Lifetimes are Impl members so contents
  // outlive Detect().
  CachedBuffer cached_parent;
  CachedBuffer cached_size;
  CachedBuffer cached_cand;
  CachedBuffer cached_cand_count;
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
    tf = talos::vision::apriltag::CreateTagFamily(options.family_name);
    if (tf == nullptr) {
      throw std::invalid_argument("Unsupported AprilTag family: " +
                                  options.family_name);
    }
    active_family = options.family_name;
    td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, options.max_hamming);
    td->nthreads = options.threads;
    td->quad_decimate = options.quad_decimate;
    td->quad_sigma = options.quad_sigma;
    td->refine_edges = options.refine_edges ? 1 : 0;
    td->decode_sharpening = options.decode_sharpening;
  }

  void DestroyDetector() {
    if (td != nullptr) {
      apriltag_detector_destroy(td);
      td = nullptr;
    }
    if (tf != nullptr) {
      talos::vision::apriltag::DestroyTagFamily(active_family, tf);
      tf = nullptr;
    }
    active_family.clear();
  }

  void DestroyMetal() {
#if defined(__APPLE__)
    cached_in.Release();
    cached_gray.Release();
    cached_thresh.Release();
    cached_im_min.Release();
    cached_im_max.Release();
    cached_dec.Release();
    cached_parent.Release();
    cached_size.Release();
    cached_cand.Release();
    cached_cand_count.Release();
    ReleaseAndNull(pso_ccl_init);
    ReleaseAndNull(pso_ccl_union);
    ReleaseAndNull(pso_ccl_flatten);
    ReleaseAndNull(pso_ccl_sizes);
    ReleaseAndNull(pso_extract_cands);
    ReleaseAndNull(pso_rgb_to_gray);
    ReleaseAndNull(pso_rgb_to_gray_decimate_2x);
    ReleaseAndNull(pso_tile_min_max_4x4);
    ReleaseAndNull(pso_tile_blur_and_threshold);
    ReleaseAndNull(queue);
    ReleaseAndNull(library);
    ReleaseAndNull(device);
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
        auto make_pso = [&](const char* kernel_name) {
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

        pso_rgb_to_gray = make_pso("rgb_to_gray");
        pso_rgb_to_gray_decimate_2x = make_pso("rgb_to_gray_decimate_2x");
        pso_tile_min_max_4x4 = make_pso("tile_min_max_4x4");
        pso_tile_blur_and_threshold = make_pso("tile_blur_and_threshold");

        // Stage 2: GPU CCL + edge candidates. Tolerate nullptr (missing kernel
        // or PSO failure auto-disables Stage 2; Stage 1 remains the default).
        pso_ccl_init = make_pso("ccl_init");
        pso_ccl_union = make_pso("ccl_union_pass");
        pso_ccl_flatten = make_pso("ccl_flatten");
        pso_ccl_sizes = make_pso("ccl_compute_sizes");
        pso_extract_cands = make_pso("extract_edge_candidates");

        if (pso_rgb_to_gray && pso_rgb_to_gray_decimate_2x &&
            pso_tile_min_max_4x4 && pso_tile_blur_and_threshold) {
          queue = device->newCommandQueue();
          if (queue != nullptr) {
            const NS::String* name = device->name();
            device_name = (name != nullptr && name->utf8String() != nullptr)
                              ? name->utf8String()
                              : "Metal";
            is_fallback = false;
          } else {
            is_fallback = true;
            device_name = "CPU (Command queue creation failed)";
          }
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
      in_buf = cached_in.Ensure(device, in_bytes);
      if (!in_buf || !in_buf->contents()) return cpu_fallback();
      std::memcpy(in_buf->contents(), data, in_bytes);
    }
    if (!in_buf->contents()) return cpu_fallback();

    const bool use_gpu_decimated = (options.quad_decimate >= 2.0f);
    // Match upstream image_u8_decimate(..., 2.0f): ceil dims 1 + (dim - 1) / 2.
    const int detect_w = use_gpu_decimated ? (1 + (width - 1) / 2) : width;
    const int detect_h = use_gpu_decimated ? (1 + (height - 1) / 2) : height;
    MTL::Buffer* dec_buf = nullptr;
    if (use_gpu_decimated) {
      const size_t dec_bytes =
          static_cast<size_t>(detect_w) * static_cast<size_t>(detect_h);
      dec_buf = cached_dec.Ensure(device, dec_bytes);
    }
    MTL::Buffer* gray_buf = cached_gray.Ensure(device, gray_bytes);
    if (!gray_buf || !gray_buf->contents() ||
        (use_gpu_decimated && (!dec_buf || !dec_buf->contents()))) {
      return cpu_fallback();
    }

    MTL::CommandBuffer* cmd_buf = queue ? queue->commandBuffer() : nullptr;
    if (cmd_buf == nullptr) {
      return cpu_fallback();
    }
    MTL::ComputeCommandEncoder* enc = cmd_buf->computeCommandEncoder();
    if (enc == nullptr) {
      return cpu_fallback();
    }

    const uint32_t w = static_cast<uint32_t>(width);
    const uint32_t h = static_cast<uint32_t>(height);
    const uint32_t in_s = static_cast<uint32_t>(actual_stride);
    const uint32_t fmt = FormatToMetalCode(format);

    if (format == PixelFormat::kGray8 && actual_stride == width) {
      std::memcpy(gray_buf->contents(), data, gray_bytes);
    } else {
      EncodeColorConvert(enc, pso_rgb_to_gray, in_buf, gray_buf, w, h, in_s, w,
                         fmt, w, h);
      enc->memoryBarrier(MTL::BarrierScopeBuffers);
    }
    if (use_gpu_decimated) {
      const uint32_t dw = static_cast<uint32_t>(detect_w);
      const uint32_t dh = static_cast<uint32_t>(detect_h);
      EncodeColorConvert(enc, pso_rgb_to_gray_decimate_2x, in_buf, dec_buf, w,
                         h, in_s, dw, fmt, dw, dh);
    }

    enc->memoryBarrier(MTL::BarrierScopeBuffers);

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
      if (threshold_ok && cached_thresh.buffer != nullptr &&
          cached_thresh.buffer->contents() != nullptr) {
        threshim.buf = static_cast<uint8_t*>(cached_thresh.buffer->contents());
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
    uint32_t stage2_max_cands = 0;
    if (threshim_ptr != nullptr) {
      // Stage 2 (GPU CCL + sparse clustering) is default-ON: parity-proven
      // against CPU on all fixtures. Opt out per-process with
      // TALOS_METAL_STAGE2=0 (keeps the Stage 1 CPU-CC path).
      const char* stage2_env = std::getenv("TALOS_METAL_STAGE2");
      const bool stage2_enabled =
          !(stage2_env != nullptr && stage2_env[0] == '0' &&
            stage2_env[1] == '\0');
      // 14-bit x/y packing in extract_edge_candidates bounds dims to 16384.
      if (stage2_enabled && pso_ccl_init != nullptr &&
          pso_ccl_union != nullptr && pso_ccl_flatten != nullptr &&
          pso_ccl_sizes != nullptr && pso_extract_cands != nullptr &&
          detect_w > 0 && detect_h > 0 && detect_w <= 16384 &&
          detect_h <= 16384) {
        const size_t npix =
            static_cast<size_t>(detect_w) * static_cast<size_t>(detect_h);
        if (npix > 0 && npix <= static_cast<size_t>(64) * 1024 * 1024) {
          const size_t parent_bytes = npix * sizeof(uint32_t);
          MTL::Buffer* parent_buf = cached_parent.Ensure(device, parent_bytes);
          MTL::Buffer* size_buf = cached_size.Ensure(device, parent_bytes);
          // Candidate capacity at half the pixels: busy photographic frames
          // reach ~40% edge density, far above sparse assumptions. Slots are
          // packed uint32 (flags/y/x), one uint32 each.
          size_t max_cands = npix / 2;
          if (max_cands == 0) max_cands = 1;
          const size_t cand_bytes = max_cands * sizeof(uint32_t);
          MTL::Buffer* cand_buf = cached_cand.Ensure(device, cand_bytes);
          MTL::Buffer* cand_count_buf =
              cached_cand_count.Ensure(device, sizeof(uint32_t));
          if (parent_buf != nullptr && size_buf != nullptr &&
              cand_buf != nullptr && cand_count_buf != nullptr &&
              cached_thresh.buffer != nullptr && parent_buf->contents() &&
              size_buf->contents() && cand_buf->contents() &&
              cand_count_buf->contents() && cached_thresh.buffer->contents()) {
            *static_cast<uint32_t*>(cand_count_buf->contents()) = 0;
            const uint32_t dw = static_cast<uint32_t>(detect_w);
            const uint32_t dh = static_cast<uint32_t>(detect_h);
            const uint32_t ds = static_cast<uint32_t>(detect_w);
            const uint32_t total = static_cast<uint32_t>(npix);
            const uint32_t maxc = static_cast<uint32_t>(max_cands);

            enc->setComputePipelineState(pso_ccl_init);
            enc->setBuffer(cached_thresh.buffer, 0, 0);
            enc->setBuffer(parent_buf, 0, 1);
            enc->setBuffer(size_buf, 0, 2);
            enc->setBytes(&dw, sizeof(dw), 3);
            enc->setBytes(&dh, sizeof(dh), 4);
            enc->setBytes(&ds, sizeof(ds), 5);
            Dispatch2D(enc, dw, dh);
            enc->memoryBarrier(MTL::BarrierScopeBuffers);

            enc->setComputePipelineState(pso_ccl_union);
            enc->setBuffer(cached_thresh.buffer, 0, 0);
            enc->setBuffer(parent_buf, 0, 1);
            enc->setBytes(&dw, sizeof(dw), 2);
            enc->setBytes(&dh, sizeof(dh), 3);
            enc->setBytes(&ds, sizeof(ds), 4);
            Dispatch2D(enc, dw, dh);
            enc->memoryBarrier(MTL::BarrierScopeBuffers);

            enc->setComputePipelineState(pso_ccl_flatten);
            enc->setBuffer(parent_buf, 0, 0);
            enc->setBytes(&total, sizeof(total), 1);
            Dispatch1D(enc, total);
            enc->memoryBarrier(MTL::BarrierScopeBuffers);

            enc->setComputePipelineState(pso_ccl_sizes);
            enc->setBuffer(parent_buf, 0, 0);
            enc->setBuffer(size_buf, 0, 1);
            enc->setBytes(&total, sizeof(total), 2);
            Dispatch1D(enc, total);
            enc->memoryBarrier(MTL::BarrierScopeBuffers);

            enc->setComputePipelineState(pso_extract_cands);
            enc->setBuffer(cached_thresh.buffer, 0, 0);
            enc->setBuffer(cand_buf, 0, 1);
            enc->setBuffer(cand_count_buf, 0, 2);
            enc->setBytes(&dw, sizeof(dw), 3);
            enc->setBytes(&dh, sizeof(dh), 4);
            enc->setBytes(&ds, sizeof(ds), 5);
            enc->setBytes(&maxc, sizeof(maxc), 6);
            Dispatch2D(enc, dw, dh);

            stage2_parents =
                static_cast<const uint32_t*>(parent_buf->contents());
            stage2_sizes = static_cast<const uint32_t*>(size_buf->contents());
            stage2_max_cands = maxc;
          }
        }
      }
    }

    enc->endEncoding();
    cmd_buf->commit();
    cmd_buf->waitUntilCompleted();
    if (cmd_buf->status() != MTL::CommandBufferStatusCompleted) {
      return cpu_fallback();
    }
    const double gpu_sec = cmd_buf->GPUEndTime() - cmd_buf->GPUStartTime();
    last_timings.threshold_us =
        (gpu_sec > 0.0) ? static_cast<uint64_t>(gpu_sec * 1e6) : 0;
    if (stage2_parents != nullptr) {
      const uint32_t count =
          *static_cast<uint32_t*>(cached_cand_count.buffer->contents());
      // Compare against this frame's maxc (not grow-only cached_cand.capacity)
      // so overflow falls back to Stage 2a instead of reading stale tail slots.
      if (count <= stage2_max_cands) {
        stage2_cands =
            static_cast<const uint32_t*>(cached_cand.buffer->contents());
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

    talos::vision::apriltag::EnsureBackgroundWorkerPool(td);
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
    std::vector<talos::vision::apriltag::Detection> results =
        talos::vision::apriltag::ExtractDetections(
            raw_detections, *cpu_backend, p_intrinsics, tag_size_meters);
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
    return CpuDetect(data, width, height, stride, format, p_intrinsics,
                     tag_size_meters);
#endif
  }

  bool RunGpuThreshold(MTL::CommandBuffer* cmd_buf, MTL::Buffer* gray_buf,
                       MTL::Buffer* thresh_buf, uint32_t width, uint32_t height,
                       uint32_t stride, uint32_t min_diff,
                       MTL::ComputeCommandEncoder* shared_encoder = nullptr) {
#if defined(__APPLE__)
    if (!device || !cmd_buf || !gray_buf || width == 0 || height == 0) {
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
    MTL::Buffer* im_min_buf = cached_im_min.Ensure(device, min_max_bytes);
    MTL::Buffer* im_max_buf = cached_im_max.Ensure(device, min_max_bytes);

    const size_t thresh_bytes = static_cast<size_t>(actual_stride) * height;
    MTL::Buffer* out_thresh = (thresh_buf != nullptr)
                                  ? thresh_buf
                                  : cached_thresh.Ensure(device, thresh_bytes);
    if (!gray_buf->contents() || !im_min_buf || !im_min_buf->contents() ||
        !im_max_buf || !im_max_buf->contents() || !out_thresh ||
        !out_thresh->contents()) {
      return false;
    }

    MTL::ComputeCommandEncoder* enc = shared_encoder != nullptr
                                          ? shared_encoder
                                          : cmd_buf->computeCommandEncoder();
    if (enc == nullptr) {
      return false;
    }

    // Pass 1: tile_min_max_4x4
    enc->setComputePipelineState(pso_tile_min_max_4x4);
    enc->setBuffer(gray_buf, 0, 0);
    enc->setBuffer(im_min_buf, 0, 1);
    enc->setBuffer(im_max_buf, 0, 2);
    enc->setBytes(&width, sizeof(width), 3);
    enc->setBytes(&height, sizeof(height), 4);
    enc->setBytes(&actual_stride, sizeof(actual_stride), 5);
    Dispatch2D(enc, tw, th);

    enc->memoryBarrier(MTL::BarrierScopeBuffers);

    // Pass 2: tile_blur_and_threshold
    enc->setComputePipelineState(pso_tile_blur_and_threshold);
    enc->setBuffer(gray_buf, 0, 0);
    enc->setBuffer(im_min_buf, 0, 1);
    enc->setBuffer(im_max_buf, 0, 2);
    enc->setBuffer(out_thresh, 0, 3);
    enc->setBytes(&width, sizeof(width), 4);
    enc->setBytes(&height, sizeof(height), 5);
    enc->setBytes(&actual_stride, sizeof(actual_stride), 6);
    enc->setBytes(&min_diff, sizeof(min_diff), 7);
    Dispatch2D(enc, width, height);

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
  image_u8_t* im = talos::vision::apriltag::LoadGrayscaleImageFile(filepath);
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
  if (!frame.holds_image()) {
    result.success = false;
    result.error_message =
        "Frame has invalid dimensions or stores fewer than stride * height "
        "bytes";
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
    result.detections.push_back(talos::vision::ToRawTagDetection(det));
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

}  // namespace talos::vision::metal
