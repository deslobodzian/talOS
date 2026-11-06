#include "talOS/vision/calibrate/calibration.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <filesystem>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace talos::vision::calibrate {
namespace {
Result Example() {
  return {(cv::Mat_<double>(3, 3) << 800, 0, 480, 0, 810, 360, 0, 0, 1),
          (cv::Mat_<double>(5, 1) << -0.12, 0.04, 0.001, -0.002, -0.01),
          {960, 720},
          0,
          20};
}
TEST(Calibration, TooFewViews) {
  Calibration calibration;
  EXPECT_THROW(calibration.Solve(), std::runtime_error);
}
TEST(Calibration, RefusesPoorCoverageAndDuplicateViews) {
  Calibration calibration;
  cv::Mat board;
  cv::resize(calibration.BoardImage(), board, {280, 200}, 0, 0, cv::INTER_AREA);
  for (int i = 0; i < 13; ++i) {
    cv::Mat image(720, 960, CV_8UC1, cv::Scalar(255));
    board.copyTo(image(cv::Rect(180 + 26 * i, 260, 280, 200)));
    ASSERT_TRUE(calibration.AddImage(image));
    EXPECT_FALSE(calibration.AddImage(image));
  }
  EXPECT_EQ(calibration.views(), 13);
  EXPECT_THROW(calibration.Solve(), std::runtime_error);
}
TEST(Calibration, TomlPreservesUntargetedBytes) {
  const auto r = Example();
  const std::string input =
      "# robot\n[robot]\ncommissioned = false # keep\n[camera] # lens\nfx = 1 "
      "# measured\nmount = 3\n[can]\nid = 42\n";
  const std::string expected =
      "# robot\n[robot]\ncommissioned = false # keep\n[camera] # lens\nfx = "
      "800 # measured\nmount = 3\ncx = 480\ncy = 360\nfy = 810\nheight = "
      "720\nk1 = -0.12\nk2 = 0.040000000000000001\nk3 = -0.01\np1 = 0.001\np2 "
      "= -0.002\nwidth = 960\n[can]\nid = 42\n";
  EXPECT_EQ(UpdateToml(input, r), expected);
  EXPECT_THROW(UpdateToml("[camera]\nfx=1\nfx=2\n", r), std::runtime_error);
  EXPECT_THROW(UpdateToml("[camera]\n[camera]\n", r), std::runtime_error);
  EXPECT_THROW(UpdateToml("[\"camera\"]\nfx=1\n", r), std::runtime_error);
}
TEST(Calibration, RecoversRenderedDistortedBoard) {
  const auto truth = Example();
  Calibration calibration;
  // Independent texture: construct the specified OpenCV board, not the
  // library's rendering helper. Ray-plane intersection maps distorted sensor
  // pixels back to this printed planar target, retaining lens distortion in the
  // images.
  cv::aruco::CharucoBoard board(
      {7, 5}, .04F, .03F,
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
  cv::Mat texture;
  board.generateImage({1400, 1000}, texture, 0, 1);
  std::vector<cv::Point2f> pixels;
  for (int y = 0; y < 720; ++y) {
    for (int x = 0; x < 960; ++x) pixels.emplace_back(x, y);
  }
  std::vector<cv::Point2f> rays;
  cv::undistortPoints(pixels, rays, truth.camera, truth.distortion);
  const char* output = std::getenv("TALOS_CALIBRATION_IMAGES");
  if (output) std::filesystem::create_directories(output);
  // Independent tilt/translation samples avoid the correlated grid poses that
  // leave principal point and tangential distortion poorly constrained.
  cv::RNG rng(458);
  for (int i = 0; i < 48; ++i) {
    const cv::Vec3d rotation{rng.uniform(-.65, .65), rng.uniform(-.65, .65),
                             rng.uniform(-.3, .3)};
    const std::array<double, 3> translation{
        rng.uniform(-.27, .03), rng.uniform(-.22, .04), rng.uniform(.4, .65)};
    cv::Matx33d R;
    cv::Rodrigues(rotation, R);
    const cv::Matx33d H(R(0, 0), R(0, 1), translation[0], R(1, 0), R(1, 1),
                        translation[1], R(2, 0), R(2, 1), translation[2]);
    const auto inverse = H.inv();
    cv::Mat mapx(720, 960, CV_32F), mapy(720, 960, CV_32F);
    for (int y = 0; y < 720; ++y) {
      for (int x = 0; x < 960; ++x) {
        const auto ray = rays[y * 960 + x];
        const auto p = inverse * cv::Vec3d(ray.x, ray.y, 1);
        mapx.at<float>(y, x) = static_cast<float>(p[0] / p[2] * 5000);
        mapy.at<float>(y, x) = static_cast<float>(p[1] / p[2] * 5000);
      }
    }
    cv::Mat image;
    cv::remap(texture, image, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
              cv::Scalar(255));
    calibration.AddImage(image);
    if (output) {
      ASSERT_TRUE(cv::imwrite(
          std::string(output) + "/view" + std::to_string(i) + ".png", image));
    }
  }
  const auto result = calibration.Solve();
  EXPECT_GE(result.views, 12);
  EXPECT_LT(result.rms, .5);
  EXPECT_NEAR(result.camera.at<double>(0, 0), 800, 5);
  EXPECT_NEAR(result.camera.at<double>(1, 1), 810, 5);
  EXPECT_NEAR(result.camera.at<double>(0, 2), 480, 4);
  EXPECT_NEAR(result.camera.at<double>(1, 2), 360, 4);
  EXPECT_NEAR(result.distortion.at<double>(0), -.12, .01);
  EXPECT_NEAR(result.distortion.at<double>(1), .04, .025);
  EXPECT_NEAR(result.distortion.at<double>(2), .001, .001);
  EXPECT_NEAR(result.distortion.at<double>(3), -.002, .001);
  EXPECT_NEAR(result.distortion.at<double>(4), -.01, .035);
}
}  // namespace
}  // namespace talos::vision::calibrate
