#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include "talOS/vision/apriltag/cpu_apriltag_backend.h"
#include "talOS/vision/compute_backend.h"
#include "talOS/vision/frame.h"
#include "talOS/vision/metal/metal_apriltag_backend.h"
#include "talOS/vision/synthetic_source.h"

namespace talos::vision::metal {
namespace {

using Clock = std::chrono::steady_clock;

// Warm up both backends to eliminate initialization / first-time JIT overhead.
void Warmup(talos::vision::ComputeBackend& backend,
            const talos::vision::Frame& frame, int iterations = 3) {
  for (int i = 0; i < iterations; ++i) {
    auto res = backend.Execute(frame);
    ASSERT_TRUE(res.success);
  }
}

TEST(MetalAprilTagPerfTest, GpuAndCpuDetectionParityOnHdFrames) {
  // Generate a realistic 1920x1080 RGB8 camera frame with AprilTags
  SyntheticCameraSource source(0, "test_camera",
                               CameraMode{.width = 1920,
                                          .height = 1080,
                                          .fps = 30.0,
                                          .format = PixelFormat::kRgb8});
  source.AddTag(SyntheticTagDescriptor{
      .tag_id = 0,
      .center_x = 600.0,
      .center_y = 450.0,
      .size_pixels = 220.0,
  });
  source.AddTag(SyntheticTagDescriptor{
      .tag_id = 1,
      .center_x = 1320.0,
      .center_y = 630.0,
      .size_pixels = 200.0,
  });

  Frame frame = source.GenerateFrame();
  ASSERT_FALSE(frame.empty());
  ASSERT_EQ(frame.width(), 1920u);
  ASSERT_EQ(frame.height(), 1080u);
  ASSERT_EQ(frame.format(), PixelFormat::kRgb8);

  talos::vision::apriltag::DetectorOptions options;
  options.family_name = "tag36h11";
  options.threads = 2;
  options.quad_decimate = 2.0f;

  MetalAprilTagBackend metal_backend(options);
  talos::vision::CpuAprilTagBackend cpu_backend(
      options.threads, options.quad_decimate, options.quad_sigma,
      options.refine_edges);

  // Warm up both backends
  Warmup(metal_backend, frame, 3);
  Warmup(cpu_backend, frame, 3);

  constexpr int kIterations = 20;

  // Benchmark CPU Execution
  std::vector<double> cpu_times_ms;
  cpu_times_ms.reserve(kIterations);
  size_t cpu_detections = 0;

  for (int i = 0; i < kIterations; ++i) {
    const auto t0 = Clock::now();
    auto result = cpu_backend.Execute(frame);
    const auto t1 = Clock::now();
    ASSERT_TRUE(result.success);
    cpu_detections = result.detections.size();
    cpu_times_ms.push_back(
        std::chrono::duration<double, std::milli>(t1 - t0).count());
  }

  // Benchmark Metal GPU Execution
  std::vector<double> gpu_times_ms;
  gpu_times_ms.reserve(kIterations);
  size_t gpu_detections = 0;

  for (int i = 0; i < kIterations; ++i) {
    const auto t0 = Clock::now();
    auto result = metal_backend.Execute(frame);
    const auto t1 = Clock::now();
    ASSERT_TRUE(result.success);
    gpu_detections = result.detections.size();
    gpu_times_ms.push_back(
        std::chrono::duration<double, std::milli>(t1 - t0).count());
    if (i == 0) {
      std::cout << "\n--- Stage Timings Breakdown (MetalAprilTagBackend) ---"
                << '\n';
      std::cout << "  GPU Grayscale (separate dispatch timing unavailable): "
                << result.timings.grayscale_us << " us" << '\n';
      std::cout << "  GPU Grayscale + Threshold + CCL: "
                << result.timings.threshold_us << " us" << '\n';
      std::cout << "  Quad Extraction:          " << result.timings.quad_us
                << " us" << '\n';
      std::cout << "  Decode + Refinement:      " << result.timings.decode_us
                << " us" << '\n';
      std::cout << "  Pose Estimation:          " << result.timings.pose_us
                << " us" << '\n';
      std::cout << "  Total:                    " << result.timings.total_us
                << " us" << '\n';
    }
  }

  // Verify that detections match
  EXPECT_GT(cpu_detections, 0u);
  EXPECT_EQ(cpu_detections, gpu_detections);

  const double cpu_total_ms =
      std::accumulate(cpu_times_ms.begin(), cpu_times_ms.end(), 0.0);
  const double gpu_total_ms =
      std::accumulate(gpu_times_ms.begin(), gpu_times_ms.end(), 0.0);

  const double cpu_avg_ms = cpu_total_ms / kIterations;
  const double gpu_avg_ms = gpu_total_ms / kIterations;
  const double speedup = cpu_avg_ms / gpu_avg_ms;

  std::cout << "\n================ Performance Benchmark (1920x1080 RGB8) "
               "================\n"
            << "  Hardware Backend: " << metal_backend.device_info() << "\n"
            << "  Frames Processed: " << kIterations << "\n"
            << "  CPU Reference:    " << std::fixed << std::setprecision(2)
            << cpu_avg_ms << " ms/frame (" << (1000.0 / cpu_avg_ms) << " FPS)\n"
            << "  Metal GPU:        " << std::fixed << std::setprecision(2)
            << gpu_avg_ms << " ms/frame (" << (1000.0 / gpu_avg_ms) << " FPS)\n"
            << "  Speedup Factor:   " << std::fixed << std::setprecision(2)
            << speedup << "x faster on GPU\n"
            << "==============================================================="
               "========\n"
            << '\n';

  // Fallback still exercises detection parity, but cannot measure GPU speedup.
}

TEST(MetalAprilTagPerfTest, GpuPreprocessingBatchTiming) {
  // A single command buffer exposes batch timing, not conversion-only timing.
  SyntheticCameraSource source(0, "hd_camera",
                               CameraMode{.width = 1920,
                                          .height = 1080,
                                          .fps = 30.0,
                                          .format = PixelFormat::kRgb8});

  Frame frame = source.GenerateFrame();
  ASSERT_FALSE(frame.empty());

  talos::vision::apriltag::DetectorOptions options;
  MetalAprilTagBackend metal_backend(options);
  talos::vision::CpuAprilTagBackend cpu_backend;

  if (!metal_backend.supports_gpu() || metal_backend.is_fallback()) {
    GTEST_SKIP() << "Skipping performance test: Metal GPU is not available on "
                    "this platform.";
  }

  Warmup(metal_backend, frame, 2);
  Warmup(cpu_backend, frame, 2);

  constexpr int kIterations = 15;

  double total_cpu_gray_us = 0.0;
  for (int i = 0; i < kIterations; ++i) {
    auto res = cpu_backend.Execute(frame);
    ASSERT_TRUE(res.success);
    total_cpu_gray_us += static_cast<double>(res.timings.grayscale_us);
  }

  double total_gpu_gray_us = 0.0;
  for (int i = 0; i < kIterations; ++i) {
    auto res = metal_backend.Execute(frame);
    ASSERT_TRUE(res.success);
    total_gpu_gray_us += static_cast<double>(res.timings.grayscale_us +
                                             res.timings.threshold_us);
  }

  const double avg_cpu_gray_us = total_cpu_gray_us / kIterations;
  const double avg_gpu_gray_us = total_gpu_gray_us / kIterations;
  const double gray_speedup = avg_cpu_gray_us / avg_gpu_gray_us;

  std::cout
      << "\n=========== Preprocessing Benchmark (1920x1080 RGB8) ===========\n"
      << "  CPU Conversion:  " << std::fixed << std::setprecision(1)
      << avg_cpu_gray_us << " us (" << (avg_cpu_gray_us / 1000.0) << " ms)\n"
      << "  GPU Preprocess:  " << std::fixed << std::setprecision(1)
      << avg_gpu_gray_us << " us (" << (avg_gpu_gray_us / 1000.0) << " ms)\n"
      << "  Conversion/batch ratio: " << std::fixed << std::setprecision(2)
      << gray_speedup << "x\n"
      << "=================================================================\n"
      << '\n';

  // CPU conversion alone and the entire GPU preprocessing batch are no
  // longer equivalent timing scopes; do not assert a misleading speedup.
}

}  // namespace
}  // namespace talos::vision::metal
