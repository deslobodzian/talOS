#include "talOS/vision/metal/apriltag_metal_adapter.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "talOS/vision/apriltag/test_image_loader.h"
#include "talOS/vision/metal/metal_shaders.h"

extern "C" {
#include "apriltag.h"
#include "common/image_u8.h"
#include "tag36h11.h"
}

#if defined(__APPLE__)
#define NS_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION
#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#endif

// Declared in apriltag_quad_thresh.c
extern "C" zarray_t* apriltag_quad_thresh(apriltag_detector_t* td,
                                          image_u8_t* im);

namespace talos::vision::metal {
namespace {

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

class AprilTagMetalAdapterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family_bits(td_, tf_, 2);
    td_->nthreads = 1;
    td_->quad_decimate = 1.0f;
    td_->quad_sigma = 0.0f;
    td_->refine_edges = true;
    td_->decode_sharpening = 0.25;
  }

  void TearDown() override {
    if (td_) {
      apriltag_detector_destroy(td_);
      td_ = nullptr;
    }
    if (tf_) {
      tag36h11_destroy(tf_);
      tf_ = nullptr;
    }
  }

  apriltag_family_t* tf_{nullptr};
  apriltag_detector_t* td_{nullptr};
};

TEST_F(AprilTagMetalAdapterTest, QuadsExactParityWithCpuThreshold) {
  const std::string path = ResolveTestdataPath("scene_single_tag0.png");
  ASSERT_TRUE(std::filesystem::exists(path)) << "Path: " << path;

  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);

  // 1. Upstream CPU thresholding and quads
  image_u8_t* threshim = apriltag_threshold_cpu(td_, im);
  ASSERT_NE(threshim, nullptr);

  // 2. Extract quads using adapter from externally provided threshim
  zarray_t* adapter_quads =
      apriltag_quad_thresh_from_threshim(td_, im, threshim);
  ASSERT_NE(adapter_quads, nullptr);

  // 3. Upstream apriltag_quad_thresh
  zarray_t* upstream_quads = apriltag_quad_thresh(td_, im);
  ASSERT_NE(upstream_quads, nullptr);

  // Both should extract identical number of quads
  const int n_adapter = zarray_size(adapter_quads);
  const int n_upstream = zarray_size(upstream_quads);
  EXPECT_EQ(n_adapter, n_upstream);
  EXPECT_GT(n_adapter, 0);

  for (int i = 0; i < n_adapter; ++i) {
    struct quad *qa, *qu;
    zarray_get_volatile(adapter_quads, i, reinterpret_cast<void*>(&qa));
    zarray_get_volatile(upstream_quads, i, reinterpret_cast<void*>(&qu));

    EXPECT_EQ(qa->reversed_border, qu->reversed_border);
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(qa->p[j][0], qu->p[j][0], 1e-4)
          << "Mismatch at quad " << i << " corner " << j << " x";
      EXPECT_NEAR(qa->p[j][1], qu->p[j][1], 1e-4)
          << "Mismatch at quad " << i << " corner " << j << " y";
    }
  }

  for (int i = 0; i < n_adapter; ++i) {
    struct quad* q;
    zarray_get_volatile(adapter_quads, i, reinterpret_cast<void*>(&q));
    matd_destroy(q->H);
    matd_destroy(q->Hinv);
  }
  zarray_destroy(adapter_quads);

  for (int i = 0; i < n_upstream; ++i) {
    struct quad* q;
    zarray_get_volatile(upstream_quads, i, reinterpret_cast<void*>(&q));
    matd_destroy(q->H);
    matd_destroy(q->Hinv);
  }
  zarray_destroy(upstream_quads);

  image_u8_destroy(threshim);
  image_u8_destroy(im);
}

TEST_F(AprilTagMetalAdapterTest, DetectWithThreshimExactParitySingleTag) {
  const std::string path = ResolveTestdataPath("scene_single_tag0.png");
  ASSERT_TRUE(std::filesystem::exists(path));

  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);

  image_u8_t* threshim = apriltag_threshold_cpu(td_, im);
  ASSERT_NE(threshim, nullptr);

  // Standard upstream detection
  zarray_t* standard_dets = apriltag_detector_detect(td_, im);
  ASSERT_NE(standard_dets, nullptr);

  // Adapter detection with threshim
  zarray_t* adapter_dets =
      apriltag_detector_detect_with_threshim(td_, im, threshim);
  ASSERT_NE(adapter_dets, nullptr);

  const int count_std = zarray_size(standard_dets);
  const int count_adp = zarray_size(adapter_dets);
  EXPECT_EQ(count_std, 1);
  EXPECT_EQ(count_adp, 1);

  apriltag_detection_t *det_std, *det_adp;
  zarray_get(standard_dets, 0, reinterpret_cast<void*>(&det_std));
  zarray_get(adapter_dets, 0, reinterpret_cast<void*>(&det_adp));

  EXPECT_EQ(det_std->id, 0);
  EXPECT_EQ(det_adp->id, 0);
  EXPECT_EQ(det_std->hamming, det_adp->hamming);
  EXPECT_NEAR(det_std->decision_margin, det_adp->decision_margin, 1e-4);
  EXPECT_NEAR(det_std->c[0], det_adp->c[0], 1e-4);
  EXPECT_NEAR(det_std->c[1], det_adp->c[1], 1e-4);

  for (int j = 0; j < 4; ++j) {
    EXPECT_NEAR(det_std->p[j][0], det_adp->p[j][0], 1e-4);
    EXPECT_NEAR(det_std->p[j][1], det_adp->p[j][1], 1e-4);
  }

  apriltag_detections_destroy(standard_dets);
  apriltag_detections_destroy(adapter_dets);
  image_u8_destroy(threshim);
  image_u8_destroy(im);
}

TEST_F(AprilTagMetalAdapterTest, MultiTagSceneParity) {
  const std::string path = ResolveTestdataPath("scene_multi_tag.png");
  ASSERT_TRUE(std::filesystem::exists(path));

  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);

  image_u8_t* threshim = apriltag_threshold_cpu(td_, im);
  ASSERT_NE(threshim, nullptr);

  zarray_t* standard_dets = apriltag_detector_detect(td_, im);
  zarray_t* adapter_dets =
      apriltag_detector_detect_with_threshim(td_, im, threshim);

  ASSERT_NE(standard_dets, nullptr);
  ASSERT_NE(adapter_dets, nullptr);

  EXPECT_EQ(zarray_size(standard_dets), zarray_size(adapter_dets));
  const int count = zarray_size(standard_dets);
  EXPECT_GT(count, 1);

  for (int i = 0; i < count; ++i) {
    apriltag_detection_t *det_std, *det_adp;
    zarray_get(standard_dets, i, reinterpret_cast<void*>(&det_std));
    zarray_get(adapter_dets, i, reinterpret_cast<void*>(&det_adp));

    EXPECT_EQ(det_std->id, det_adp->id);
    EXPECT_EQ(det_std->hamming, det_adp->hamming);
    EXPECT_NEAR(det_std->decision_margin, det_adp->decision_margin, 1e-3);
    EXPECT_NEAR(det_std->c[0], det_adp->c[0], 0.05);
    EXPECT_NEAR(det_std->c[1], det_adp->c[1], 0.05);
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(det_std->p[j][0], det_adp->p[j][0], 0.05);
      EXPECT_NEAR(det_std->p[j][1], det_adp->p[j][1], 0.05);
    }
  }

  apriltag_detections_destroy(standard_dets);
  apriltag_detections_destroy(adapter_dets);
  image_u8_destroy(threshim);
  image_u8_destroy(im);
}

TEST_F(AprilTagMetalAdapterTest, RotatedTagSceneParity) {
  const std::string path = ResolveTestdataPath("scene_rotated_tag.png");
  ASSERT_TRUE(std::filesystem::exists(path));

  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);

  image_u8_t* threshim = apriltag_threshold_cpu(td_, im);
  ASSERT_NE(threshim, nullptr);

  zarray_t* standard_dets = apriltag_detector_detect(td_, im);
  zarray_t* adapter_dets =
      apriltag_detector_detect_with_threshim(td_, im, threshim);

  ASSERT_NE(standard_dets, nullptr);
  ASSERT_NE(adapter_dets, nullptr);

  EXPECT_EQ(zarray_size(standard_dets), zarray_size(adapter_dets));
  EXPECT_EQ(zarray_size(adapter_dets), 1);

  apriltag_detection_t *det_std, *det_adp;
  zarray_get(standard_dets, 0, reinterpret_cast<void*>(&det_std));
  zarray_get(adapter_dets, 0, reinterpret_cast<void*>(&det_adp));

  EXPECT_EQ(det_std->id, det_adp->id);
  EXPECT_EQ(det_std->id, 4);
  EXPECT_EQ(det_adp->id, 4);
  EXPECT_EQ(det_std->hamming, det_adp->hamming);
  EXPECT_NEAR(det_std->decision_margin, det_adp->decision_margin, 1e-4);
  EXPECT_NEAR(det_std->c[0], det_adp->c[0], 1e-4);
  EXPECT_NEAR(det_std->c[1], det_adp->c[1], 1e-4);

  apriltag_detections_destroy(standard_dets);
  apriltag_detections_destroy(adapter_dets);
  image_u8_destroy(threshim);
  image_u8_destroy(im);
}

TEST_F(AprilTagMetalAdapterTest, EmptySceneParity) {
  const std::string path = ResolveTestdataPath("scene_empty.png");
  ASSERT_TRUE(std::filesystem::exists(path));

  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);

  image_u8_t* threshim = apriltag_threshold_cpu(td_, im);
  ASSERT_NE(threshim, nullptr);

  zarray_t* standard_dets = apriltag_detector_detect(td_, im);
  zarray_t* adapter_dets =
      apriltag_detector_detect_with_threshim(td_, im, threshim);

  EXPECT_EQ(zarray_size(standard_dets), 0);
  EXPECT_EQ(zarray_size(adapter_dets), 0);

  apriltag_detections_destroy(standard_dets);
  apriltag_detections_destroy(adapter_dets);
  image_u8_destroy(threshim);
  image_u8_destroy(im);
}

TEST_F(AprilTagMetalAdapterTest, NullThreshimFallbackPath) {
  const std::string path = ResolveTestdataPath("scene_single_tag0.png");
  ASSERT_TRUE(std::filesystem::exists(path));

  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);

  // Passing NULL threshim must fall back to standard detector
  zarray_t* standard_dets = apriltag_detector_detect(td_, im);
  zarray_t* fallback_dets =
      apriltag_detector_detect_with_threshim(td_, im, nullptr);

  ASSERT_NE(standard_dets, nullptr);
  ASSERT_NE(fallback_dets, nullptr);
  EXPECT_EQ(zarray_size(standard_dets), zarray_size(fallback_dets));
  EXPECT_EQ(zarray_size(fallback_dets), 1);

  apriltag_detections_destroy(standard_dets);
  apriltag_detections_destroy(fallback_dets);
  image_u8_destroy(im);
}

TEST_F(AprilTagMetalAdapterTest, DecimatedImageParity) {
  td_->quad_decimate = 2.0f;

  const std::string path = ResolveTestdataPath("scene_single_tag0.png");
  ASSERT_TRUE(std::filesystem::exists(path));

  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);

  // Decimate image manually for adapter threshim
  image_u8_t* dec_im = image_u8_decimate(im, 2.0f);
  image_u8_t* threshim = apriltag_threshold_cpu(td_, dec_im);

  zarray_t* standard_dets = apriltag_detector_detect(td_, im);
  zarray_t* adapter_dets =
      apriltag_detector_detect_with_threshim(td_, im, threshim);

  ASSERT_NE(standard_dets, nullptr);
  ASSERT_NE(adapter_dets, nullptr);
  EXPECT_EQ(zarray_size(standard_dets), 1);
  EXPECT_EQ(zarray_size(adapter_dets), 1);

  apriltag_detection_t *det_std, *det_adp;
  zarray_get(standard_dets, 0, reinterpret_cast<void*>(&det_std));
  zarray_get(adapter_dets, 0, reinterpret_cast<void*>(&det_adp));

  EXPECT_EQ(det_std->id, det_adp->id);
  EXPECT_EQ(det_std->hamming, det_adp->hamming);
  EXPECT_NEAR(det_std->c[0], det_adp->c[0], 0.1);
  EXPECT_NEAR(det_std->c[1], det_adp->c[1], 0.1);

  apriltag_detections_destroy(standard_dets);
  apriltag_detections_destroy(adapter_dets);
  image_u8_destroy(threshim);
  image_u8_destroy(dec_im);
  image_u8_destroy(im);
}

#if defined(__APPLE__)
TEST_F(AprilTagMetalAdapterTest, GpuGeneratedThreshimDetection) {
  MTL::Device* device = MTL::CreateSystemDefaultDevice();
  if (device == nullptr) {
    GTEST_SKIP() << "No Metal device available";
  }

  NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();

  NS::String* source =
      NS::String::string(kMetalShadersSource, NS::UTF8StringEncoding);
  MTL::CompileOptions* compile_options = MTL::CompileOptions::alloc()->init();
  NS::Error* error = nullptr;
  MTL::Library* library = device->newLibrary(source, compile_options, &error);
  compile_options->release();
  ASSERT_NE(library, nullptr);

  NS::String* n_gt =
      NS::String::string("gradient_and_threshold", NS::UTF8StringEncoding);
  MTL::Function* fn_gt = library->newFunction(n_gt);
  ASSERT_NE(fn_gt, nullptr);

  MTL::ComputePipelineState* pso_gt =
      device->newComputePipelineState(fn_gt, &error);
  fn_gt->release();
  library->release();
  ASSERT_NE(pso_gt, nullptr);

  MTL::CommandQueue* queue = device->newCommandQueue();
  ASSERT_NE(queue, nullptr);

  const std::string path = ResolveTestdataPath("scene_single_tag0.png");
  image_u8_t* im = talos::vision::apriltag::LoadGrayImageU8(path);
  ASSERT_NE(im, nullptr);

  const size_t num_pixels = static_cast<size_t>(im->stride) * im->height;
  MTL::Buffer* in_buf =
      device->newBuffer(im->buf, num_pixels, MTL::ResourceStorageModeShared);
  MTL::Buffer* grad_mag_buf =
      device->newBuffer(num_pixels, MTL::ResourceStorageModeShared);
  MTL::Buffer* grad_dir_buf =
      device->newBuffer(num_pixels, MTL::ResourceStorageModeShared);
  MTL::Buffer* thresh_buf =
      device->newBuffer(num_pixels, MTL::ResourceStorageModeShared);

  uint32_t w = im->width;
  uint32_t h = im->height;
  uint32_t stride = im->stride;
  uint32_t min_diff = td_->qtp.min_white_black_diff;

  MTL::CommandBuffer* cmd = queue->commandBuffer();
  MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();
  enc->setComputePipelineState(pso_gt);
  enc->setBuffer(in_buf, 0, 0);
  enc->setBuffer(grad_mag_buf, 0, 1);
  enc->setBuffer(grad_dir_buf, 0, 2);
  enc->setBuffer(thresh_buf, 0, 3);
  enc->setBytes(&w, sizeof(w), 4);
  enc->setBytes(&h, sizeof(h), 5);
  enc->setBytes(&stride, sizeof(stride), 6);
  enc->setBytes(&min_diff, sizeof(min_diff), 7);

  MTL::Size tg_size = MTL::Size::Make(16, 16, 1);
  MTL::Size tg_count = MTL::Size::Make((w + 15) / 16, (h + 15) / 16, 1);
  enc->dispatchThreadgroups(tg_count, tg_size);
  enc->endEncoding();

  cmd->commit();
  cmd->waitUntilCompleted();

  // Wrap GPU-generated threshold buffer as image_u8_t
  image_u8_t gpu_threshim{
      .width = im->width,
      .height = im->height,
      .stride = im->stride,
      .buf = static_cast<uint8_t*>(thresh_buf->contents()),
  };

  // Run detection using GPU threshold intermediate buffer
  zarray_t* dets =
      apriltag_detector_detect_with_threshim(td_, im, &gpu_threshim);
  ASSERT_NE(dets, nullptr);

  // Successfully detected the tag!
  EXPECT_EQ(zarray_size(dets), 1);
  if (zarray_size(dets) > 0) {
    apriltag_detection_t* det;
    zarray_get(dets, 0, reinterpret_cast<void*>(&det));
    EXPECT_EQ(det->id, 0);
    EXPECT_EQ(det->hamming, 0);
    EXPECT_GT(det->decision_margin, 20.0f);
  }

  apriltag_detections_destroy(dets);

  in_buf->release();
  grad_mag_buf->release();
  grad_dir_buf->release();
  thresh_buf->release();
  pso_gt->release();
  queue->release();
  device->release();
  pool->release();
  image_u8_destroy(im);
}
#endif

}  // namespace
}  // namespace talos::vision::metal
