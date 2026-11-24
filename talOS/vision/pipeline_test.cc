#include "talOS/vision/pipeline.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "talOS/process/process.h"
#include "talOS/vision/metal/metal_apriltag_backend.h"
#include "talOS/vision/synthetic_source.h"

namespace talos::vision {
namespace {

PipelineConfig Config() {
  PipelineConfig config;
  config.intrinsics.fx = 500;
  config.intrinsics.fy = 500;
  config.intrinsics.cx = 320;
  config.intrinsics.cy = 240;
  config.intrinsics.image_width = 640;
  config.intrinsics.image_height = 480;
  config.mounting = CameraMounting::Identity();
  config.layout.set_default_tag_size_m(0.20);
  config.layout.SetTagPose(
      0, talos::geometry::SE3<double>(talos::geometry::SO3<double>::Identity(),
                                      Eigen::Vector3d(3, 0, 0.5)));
  config.validation_options.max_ambiguity = 1;
  return config;
}

Frame TagFrame() {
  SyntheticCameraSource source(0, "synthetic",
                               {640, 480, 0, PixelFormat::kGray8});
  source.AddTag(
      {.tag_id = 0, .center_x = 320, .center_y = 240, .size_pixels = 160});
  return source.GenerateFrame();
}

TEST(PipelineTest, DetectionAndCameraRelativePose) {
  AprilTagPipeline pipeline(Config());
  auto result = pipeline.ProcessFrame(TagFrame());
  ASSERT_TRUE(result.success) << result.error_message;
  ASSERT_EQ(result.detections.size(), 1);
  const auto& det = result.detections.front();
  EXPECT_EQ(det.tag_id, 0);
  EXPECT_EQ(det.hamming, 0);
  EXPECT_NEAR(det.center_x, 320, 5);
  EXPECT_NEAR(det.center_y, 240, 5);
  EXPECT_GE(det.decision_margin, 30);
  EXPECT_TRUE(det.valid) << "margin=" << det.decision_margin;
  EXPECT_GE(det.secondary_reprojection_error_px, 0);
  EXPECT_TRUE(det.secondary_T_camera_tag.matrix().allFinite());
}

TEST(PipelineTest, BackendRejectsFramesLargerThanTheirStorage) {
  // A truncated buffer, or a stride shorter than one row, declares pixels the
  // frame does not store: detection fails instead of reading past them.
  const auto time = talos::event::MonotonicTime::from_nanos(1);
  constexpr std::size_t kHalfImageBytes = std::size_t{640} * 480 / 2;
  const Frame truncated(0, 1, 0, time, 640, 480, 640, PixelFormat::kGray8,
                        std::vector<std::uint8_t>(kHalfImageBytes));
  const Frame short_stride(0, 1, 0, time, 640, 480, 320, PixelFormat::kGray8,
                           std::vector<std::uint8_t>(kHalfImageBytes));
  CpuAprilTagBackend backend;
  EXPECT_FALSE(backend.Execute(truncated).success);
  EXPECT_FALSE(backend.Execute(short_stride).success);
}

// Keep the real decoded geometry, varying only the decode quality to exercise
// the pipeline's post-decode validity gate independently of decoder pruning.
class DecodeQualityBackend : public ComputeBackend {
 public:
  float margin = 30;
  int hamming = 1;
  std::string name() const override { return "cpu"; }
  std::string device_info() const override { return "test decode quality"; }
  bool supports_gpu() const override { return false; }
  BackendExecutionResult Execute(const Frame& frame) override {
    auto result = cpu_.Execute(frame);
    for (auto& detection : result.detections) {
      detection.decision_margin = margin;
      detection.hamming = hamming;
    }
    return result;
  }

 private:
  CpuAprilTagBackend cpu_;
};

TEST(PipelineTest, DecodeQualityLimitsInvalidateAndExcludeFromLocalization) {
  auto backend = std::make_shared<DecodeQualityBackend>();
  const auto config = Config();
  AprilTagPipeline pipeline(config, backend);
  const auto frame = TagFrame();
  auto result = pipeline.ProcessFrame(frame);
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.detections.size(), 1);
  ASSERT_TRUE(result.detections.front().valid);
  backend->margin = 29;
  for (int test = 0; test < 2; ++test) {
    if (test == 1) {
      backend->margin = 30;
      backend->hamming = 2;
    }
    result = pipeline.ProcessFrame(frame);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.detections.size(), 1);
    EXPECT_FALSE(result.detections.front().valid);
    const auto pose = SolveMultiTagRobotPose(
        result.detections, 0, config.mounting, config.layout, config.intrinsics,
        config.validation_options);
    EXPECT_FALSE(pose.validity.valid);
    EXPECT_EQ(pose.tag_count, 0);
  }
  auto relaxed = config;
  relaxed.detector_options.max_hamming = 2;
  AprilTagPipeline relaxed_pipeline(relaxed, backend);
  EXPECT_TRUE(relaxed_pipeline.ProcessFrame(frame).detections.front().valid);
}

class FixedCornersBackend : public ComputeBackend {
 public:
  RawTagDetection detection;
  std::string name() const override { return "cpu"; }
  std::string device_info() const override { return "pinned pose fixture"; }
  bool supports_gpu() const override { return false; }
  BackendExecutionResult Execute(const Frame&) override {
    BackendExecutionResult result;
    result.success = true;
    result.effective_backend = "cpu";
    result.detections.push_back(detection);
    return result;
  }
};

TEST(PipelineTest, DoesNotInventASecondPoseWhenAmbiguitySearchFails) {
  auto config = Config();
  config.intrinsics.fx = config.intrinsics.fy = 700;
  config.intrinsics.cx = 640;
  config.intrinsics.cy = 360;
  config.layout.set_default_tag_size_m(0.1651);
  auto backend = std::make_shared<FixedCornersBackend>();
  backend->detection.id = 0;
  backend->detection.decision_margin = 100;
  backend->detection.corners = {488.490112f, 242.771851f, 497.506744f,
                                250.814423f, 504.358826f, 243.886002f,
                                498.08548f,  236.256561f};
  AprilTagPipeline pipeline(config, backend);
  const auto result = pipeline.ProcessFrame({});
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.detections.size(), 1);
  EXPECT_DOUBLE_EQ(result.detections[0].ambiguity, 1);
  EXPECT_TRUE(std::isinf(result.detections[0].secondary_reprojection_error_px));
}

// Stands in for Metal: its construction blocks until the gate opens, like
// pipeline compilation waiting on Apple's compiler-cache lock (INC-7).
struct Gate {
  std::mutex mutex;
  std::condition_variable cv;
  bool open = false;
  void Open() {
    {
      std::lock_guard lock(mutex);
      open = true;
    }
    cv.notify_all();
  }
  void Wait() {
    std::unique_lock lock(mutex);
    cv.wait(lock, [this] { return open; });
  }
};

class SlowGpuBackend : public ComputeBackend {
 public:
  explicit SlowGpuBackend(const std::shared_ptr<Gate>& gate) { gate->Wait(); }
  std::string name() const override { return "metal"; }
  std::string device_info() const override { return "test gpu"; }
  bool supports_gpu() const override { return true; }
  BackendExecutionResult Execute(const Frame& frame) override {
    auto result = cpu_.Execute(frame);
    result.effective_backend = "metal";
    return result;
  }

 private:
  CpuAprilTagBackend cpu_;
};

TEST(PipelineTest, DeferredGpuInitNeverBlocksConstruction) {
  auto gate = std::make_shared<Gate>();
  auto config = Config();
  config.backend_name = "auto";
  config.deferred_gpu_init = true;
  config.gpu_backend_factory = [gate](const apriltag::DetectorOptions&) {
    return std::make_shared<SlowGpuBackend>(gate);
  };
  // Returns while the GPU backend is still blocked, and detects on the CPU.
  AprilTagPipeline pipeline(config);
  const auto frame = TagFrame();
  auto result = pipeline.ProcessFrame(frame);
  ASSERT_TRUE(result.success) << result.error_message;
  EXPECT_EQ(result.backend, "cpu");
  EXPECT_EQ(result.detections.size(), 1);
  // Once the GPU backend exists, the next frame runs on it.
  gate->Open();
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (pipeline.backend_name() != "metal" &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    result = pipeline.ProcessFrame(frame);
  }
  EXPECT_EQ(pipeline.ProcessFrame(frame).backend, "metal");

  // A required GPU waits only gpu_init_timeout, then construction fails.
  auto stuck = std::make_shared<Gate>();
  config.backend_name = "metal";
  config.gpu_init_timeout = std::chrono::milliseconds(20);
  config.gpu_backend_factory = [stuck](const apriltag::DetectorOptions&) {
    return std::make_shared<SlowGpuBackend>(stuck);
  };
  EXPECT_THROW(AprilTagPipeline{config}, std::invalid_argument);
  stuck->Open();
}

void ExpectInvalid(const PipelineConfig& config, const char* field) {
  SCOPED_TRACE(field);
  EXPECT_THROW(AprilTagPipeline{config}, std::invalid_argument);
}

TEST(PipelineTest, RejectsInvalidTypedSettings) {
  for (const float value :
       {0.0f, 9.0f, std::numeric_limits<float>::quiet_NaN()}) {
    auto config = Config();
    config.detector_options.quad_decimate = value;
    ExpectInvalid(config, "detector.decimate");
  }
  for (const int value : {0, 17}) {
    auto config = Config();
    config.detector_options.threads = value;
    ExpectInvalid(config, "detector.threads");
  }
  for (const float value :
       {-1.0f, 5.0f, std::numeric_limits<float>::infinity()}) {
    auto config = Config();
    config.detector_options.quad_sigma = value;
    ExpectInvalid(config, "detector.sigma");
  }
  for (const int value : {-1, 3}) {
    auto config = Config();
    config.detector_options.max_hamming = value;
    ExpectInvalid(config, "detector.hamming_threshold");
  }
  for (const double value :
       {-1.0, 1001.0, std::numeric_limits<double>::quiet_NaN()}) {
    auto config = Config();
    config.min_decision_margin = value;
    ExpectInvalid(config, "detector.min_decision_margin");
  }
  auto config = Config();
  config.backend_name = "invalid";
  ExpectInvalid(config, "backend");
  config = Config();
  config.gpu_init_timeout = std::chrono::milliseconds(-1);
  ExpectInvalid(config, "gpu_init_timeout");
}

TEST(PipelineTest, AutoFallbackHonorsSettingsAndForcedMetalRejectsThem) {
  auto config = Config();
  config.backend_name = "auto";
  config.detector_options.quad_sigma = 1;
  config.detector_options.quad_decimate = 1.5;
  const auto frame = TagFrame();
  metal::MetalAprilTagBackend backend(config.detector_options);
  const auto backend_result = backend.Execute(frame);
  ASSERT_TRUE(backend_result.success) << backend_result.error_message;
  EXPECT_EQ(backend_result.effective_backend, "cpu");
  AprilTagPipeline pipeline(config);
  const auto result = pipeline.ProcessFrame(frame);
  ASSERT_TRUE(result.success) << result.error_message;
  ASSERT_EQ(result.detections.size(), 1);
  EXPECT_EQ(result.detections.front().tag_id, 0);
  config.backend_name = "metal";
  ExpectInvalid(config, "detector.sigma");
  config.detector_options.quad_sigma = 0;
  ExpectInvalid(config, "detector.decimate");
}

// REQ-RT-10: deferred GPU compilation runs on a background thread configured
// via ConfigureBackgroundThread(), below every realtime node loop.
TEST(PipelineTest, DeferredGpuInitWorkerRunsAtBackgroundPriority) {
  auto promise = std::make_shared<std::promise<bool>>();
  std::future<bool> worker_is_background = promise->get_future();
  auto gate = std::make_shared<Gate>();
  gate->Open();

  auto config = Config();
  config.backend_name = "auto";
  config.deferred_gpu_init = true;
  config.gpu_backend_factory = [promise,
                                gate](const apriltag::DetectorOptions&) {
    promise->set_value(talos::process::ThreadIsBackground());
    return std::make_shared<SlowGpuBackend>(gate);
  };
  AprilTagPipeline pipeline(config);
  ASSERT_EQ(worker_is_background.wait_for(std::chrono::seconds(10)),
            std::future_status::ready);
  EXPECT_TRUE(worker_is_background.get());
}

TEST(PipelineTest, SyntheticSourceHandlesIntMinTagIdWithoutUb) {
  SyntheticCameraSource source(0, "synthetic",
                               {640, 480, 0, PixelFormat::kGray8});
  source.AddTag({.tag_id = std::numeric_limits<int>::min(),
                 .center_x = 320,
                 .center_y = 240,
                 .size_pixels = 160});
  const Frame frame = source.GenerateFrame();
  AprilTagPipeline pipeline(Config());
  const auto result = pipeline.ProcessFrame(frame);
  ASSERT_TRUE(result.success) << result.error_message;
  ASSERT_EQ(result.detections.size(), 1u);
  // |INT_MIN| % 10 == 2147483648 % 10 == 8.
  EXPECT_EQ(result.detections.front().tag_id, 8);
}

TEST(PipelineTest, NullGpuBackendFactoryFallsBackOrRejectsWithoutCrash) {
  const auto frame = TagFrame();
  const auto null_factory =
      [](const apriltag::DetectorOptions&) -> std::shared_ptr<ComputeBackend> {
    return nullptr;
  };

  // Synchronous auto falls back to CPU instead of leaving backend_ null.
  {
    auto config = Config();
    config.backend_name = "auto";
    config.deferred_gpu_init = false;
    config.gpu_backend_factory = null_factory;
    AprilTagPipeline pipeline(config);
    EXPECT_EQ(pipeline.backend_name(), "cpu");
    const auto result = pipeline.ProcessFrame(frame);
    ASSERT_TRUE(result.success) << result.error_message;
    EXPECT_EQ(result.backend, "cpu");
    ASSERT_EQ(result.detections.size(), 1u);
    EXPECT_EQ(result.detections.front().tag_id, 0);
  }

  // Synchronous metal refuses unavailable GPU without dereferencing null.
  {
    auto config = Config();
    config.backend_name = "metal";
    config.deferred_gpu_init = false;
    config.gpu_backend_factory = null_factory;
    EXPECT_THROW(AprilTagPipeline{config}, std::invalid_argument);
  }

  // Deferred metal waits for the background worker and rejects unavailable GPU.
  {
    auto config = Config();
    config.backend_name = "metal";
    config.deferred_gpu_init = true;
    config.gpu_backend_factory = null_factory;
    EXPECT_THROW(AprilTagPipeline{config}, std::invalid_argument);
  }

  // Deferred auto stays on CPU after the background worker sees a null factory.
  {
    auto called = std::make_shared<std::promise<void>>();
    std::future<void> called_future = called->get_future();
    auto config = Config();
    config.backend_name = "auto";
    config.deferred_gpu_init = true;
    config.gpu_backend_factory = [called](const apriltag::DetectorOptions&)
        -> std::shared_ptr<ComputeBackend> {
      called->set_value();
      return nullptr;
    };
    AprilTagPipeline pipeline(config);
    ASSERT_EQ(called_future.wait_for(std::chrono::seconds(10)),
              std::future_status::ready);
    const auto result = pipeline.ProcessFrame(frame);
    ASSERT_TRUE(result.success) << result.error_message;
    EXPECT_EQ(result.backend, "cpu");
    ASSERT_EQ(result.detections.size(), 1u);
    EXPECT_EQ(result.detections.front().tag_id, 0);
  }
}

TEST(PipelineTest, CpuBackendHandlesOddWidthYuyvInvalidFormatAndOversizedDims) {
  CpuAprilTagBackend backend;
  const auto t0 = talos::event::MonotonicTime::from_nanos(1);

  // Odd-width YUYV (63x48) must not throw cv::Exception.
  const Frame odd_small(0, 1, 1, t0, 63, 48, PixelFormat::kYuyv);
  ASSERT_TRUE(odd_small.holds_image());
  const auto small_res = backend.Execute(odd_small);
  EXPECT_TRUE(small_res.success) << small_res.error_message;

  // Odd-width YUYV carrying a real tag must extract luma accurately.
  const Frame gray_tag = TagFrame();
  const uint32_t odd_w = gray_tag.width() + 1;
  const uint32_t odd_h = gray_tag.height();
  Frame odd_yuyv(0, 1, 2, t0, odd_w, odd_h, PixelFormat::kYuyv);
  uint8_t* dst = odd_yuyv.mutable_data();
  for (uint32_t y = 0; y < odd_h; ++y) {
    const uint8_t* src_row =
        gray_tag.data() + static_cast<size_t>(y) * gray_tag.stride();
    uint8_t* dst_row = dst + static_cast<size_t>(y) * odd_yuyv.stride();
    for (uint32_t x = 0; x < odd_w; ++x) {
      dst_row[static_cast<size_t>(x) * 2] =
          (x < gray_tag.width()) ? src_row[x] : 255;
      dst_row[static_cast<size_t>(x) * 2 + 1] = 128;
    }
  }
  const auto yuyv_res = backend.Execute(odd_yuyv);
  ASSERT_TRUE(yuyv_res.success) << yuyv_res.error_message;
  ASSERT_EQ(yuyv_res.detections.size(), 1u);
  EXPECT_EQ(yuyv_res.detections.front().id, 0);

  // Unknown PixelFormat enum value fails cleanly instead of throwing.
  const auto bad_format = static_cast<PixelFormat>(99);
  const Frame unknown_fmt(0, 1, 3, t0, 16, 16, 16, bad_format,
                          std::vector<uint8_t>(size_t{16} * 16, 0));
  ASSERT_TRUE(unknown_fmt.holds_image());
  const auto fmt_res = backend.Execute(unknown_fmt);
  EXPECT_FALSE(fmt_res.success);

  // Stride / dimensions exceeding INT_MAX fail cleanly before OpenCV or
  // AprilTag casts.
  const uint8_t dummy_byte = 0;
  const uint32_t over_int_max =
      static_cast<uint32_t>(std::numeric_limits<int>::max()) + 1u;
  const Frame huge_stride(0, 1, 4, t0, 1, 1, over_int_max, PixelFormat::kGray8,
                          &dummy_byte, static_cast<size_t>(over_int_max));
  ASSERT_TRUE(huge_stride.holds_image());
  EXPECT_FALSE(backend.Execute(huge_stride).success);
}

TEST(PipelineTest, DeferredGpuInitCatchesNonStdExceptionAndCancelsOnDestroy) {
  const auto frame = TagFrame();
  // A non-std::exception from the background GPU factory must never terminate
  // the process: "metal" throws std::invalid_argument and "auto" stays on CPU.
  {
    auto config = Config();
    config.backend_name = "metal";
    config.deferred_gpu_init = true;
    config.gpu_backend_factory = [](const apriltag::DetectorOptions&)
        -> std::shared_ptr<ComputeBackend> { throw 42; };
    EXPECT_THROW(AprilTagPipeline{config}, std::invalid_argument);
  }
  {
    auto threw = std::make_shared<std::promise<void>>();
    std::future<void> threw_future = threw->get_future();
    auto config = Config();
    config.backend_name = "auto";
    config.deferred_gpu_init = true;
    config.gpu_backend_factory = [threw](const apriltag::DetectorOptions&)
        -> std::shared_ptr<ComputeBackend> {
      threw->set_value();
      throw 42;
    };
    AprilTagPipeline pipeline(config);
    ASSERT_EQ(threw_future.wait_for(std::chrono::seconds(10)),
              std::future_status::ready);
    const auto result = pipeline.ProcessFrame(frame);
    ASSERT_TRUE(result.success) << result.error_message;
    EXPECT_EQ(result.backend, "cpu");
  }
  // Destroying a pipeline while background GPU init is still blocked must not
  // block destruction or leak the created backend once the gate opens.
  auto gate = std::make_shared<Gate>();
  auto finished = std::make_shared<std::promise<void>>();
  std::future<void> finished_future = finished->get_future();
  {
    auto config = Config();
    config.backend_name = "auto";
    config.deferred_gpu_init = true;
    config.gpu_backend_factory = [gate,
                                  finished](const apriltag::DetectorOptions&) {
      auto gpu = std::make_shared<SlowGpuBackend>(gate);
      finished->set_value();
      return gpu;
    };
    AprilTagPipeline pipeline(config);
    EXPECT_EQ(pipeline.backend_name(), "cpu");
  }
  gate->Open();
  ASSERT_EQ(finished_future.wait_for(std::chrono::seconds(10)),
            std::future_status::ready);
}

TEST(PipelineTest, NaNDecisionMarginAndNegativeHammingGetExactRejectReasons) {
  auto backend = std::make_shared<DecodeQualityBackend>();
  const auto config = Config();
  AprilTagPipeline pipeline(config, backend);
  const auto frame = TagFrame();

  backend->hamming = 0;
  backend->margin = std::numeric_limits<float>::quiet_NaN();
  auto result = pipeline.ProcessFrame(frame);
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.detections.size(), 1u);
  EXPECT_FALSE(result.detections.front().valid);
  EXPECT_EQ(result.detections.front().reject_reason, "decision_margin");

  backend->hamming = -1;
  backend->margin = 100.0f;
  result = pipeline.ProcessFrame(frame);
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.detections.size(), 1u);
  EXPECT_FALSE(result.detections.front().valid);
  EXPECT_EQ(result.detections.front().reject_reason, "hamming");
}

}  // namespace
}  // namespace talos::vision
