#include <gtest/gtest.h>

#include <algorithm>

#include "talOS/vision/apriltag/cpu_apriltag_backend.h"
#include "talOS/vision/compute_backend.h"
#include "talOS/vision/frame.h"
#include "talOS/vision/metal/metal_apriltag_backend.h"
#include "talOS/vision/synthetic_source.h"

namespace talos::vision::metal {
namespace {

TEST(MetalAprilTagPerfTest, GpuAndCpuDetectionParityOnHdFrame) {
  // Exercise HD RGB preprocessing once; throughput belongs in a benchmark.
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

  talos::vision::apriltag::DetectorOptions options;
  options.family_name = "tag36h11";
  options.threads = 2;
  options.quad_decimate = 2.0f;

  MetalAprilTagBackend metal_backend(options);
  talos::vision::CpuAprilTagBackend cpu_backend(
      options.threads, options.quad_decimate, options.quad_sigma,
      options.refine_edges);

  const auto cpu = cpu_backend.Execute(frame);
  const auto metal = metal_backend.Execute(frame);
  ASSERT_TRUE(cpu.success) << cpu.error_message;
  ASSERT_TRUE(metal.success) << metal.error_message;
  ASSERT_EQ(cpu.detections.size(), 2u);
  ASSERT_EQ(metal.detections.size(), cpu.detections.size());
  for (const auto& expected : cpu.detections) {
    const auto actual = std::find_if(
        metal.detections.begin(), metal.detections.end(),
        [&](const auto& detection) { return detection.id == expected.id; });
    ASSERT_NE(actual, metal.detections.end());
    for (size_t coordinate = 0; coordinate < expected.corners.size();
         ++coordinate) {
      EXPECT_NEAR(actual->corners[coordinate], expected.corners[coordinate],
                  0.1);
    }
  }
}

}  // namespace
}  // namespace talos::vision::metal
