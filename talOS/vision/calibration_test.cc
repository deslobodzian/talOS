#include "talOS/vision/calibration.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace talos::vision {
namespace {

TEST(CalibrationTest, IntrinsicsValidityAndMatrices) {
  CameraIntrinsics intr;
  EXPECT_FALSE(intr.IsValid());

  intr.fx = 600.0;
  intr.fy = 600.0;
  intr.cx = 320.0;
  intr.cy = 240.0;
  intr.image_width = 640;
  intr.image_height = 480;
  EXPECT_TRUE(intr.IsValid());

  const Eigen::Matrix3d K = intr.K();
  const Eigen::Matrix3d K_inv = intr.KInverse();
  const Eigen::Matrix3d prod = K * K_inv;

  EXPECT_TRUE(prod.isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(CalibrationTest, PointProjection) {
  CameraIntrinsics intr;
  intr.fx = 500.0;
  intr.fy = 500.0;
  intr.cx = 320.0;
  intr.cy = 240.0;

  // Point on optical axis at 2.0m depth
  Eigen::Vector3d p_center(0.0, 0.0, 2.0);
  Eigen::Vector2d uv;
  EXPECT_TRUE(intr.ProjectPoint(p_center, &uv));
  EXPECT_NEAR(uv.x(), 320.0, 1e-6);
  EXPECT_NEAR(uv.y(), 240.0, 1e-6);

  // Point at (0.5, -0.3, 1.0)
  Eigen::Vector3d p1(0.5, -0.3, 1.0);
  EXPECT_TRUE(intr.ProjectPoint(p1, &uv));
  EXPECT_NEAR(uv.x(), 320.0 + 500.0 * 0.5, 1e-6);
  EXPECT_NEAR(uv.y(), 240.0 - 500.0 * 0.3, 1e-6);

  // Point behind camera
  Eigen::Vector3d p_behind(0.0, 0.0, -1.0);
  EXPECT_FALSE(intr.ProjectPoint(p_behind, &uv));
}

TEST(CalibrationTest, DistortionAndUndistortRoundTrip) {
  CameraIntrinsics intr;
  intr.fx = 600.0;
  intr.fy = 600.0;
  intr.cx = 320.0;
  intr.cy = 240.0;
  intr.k1 = -0.15;
  intr.k2 = 0.05;
  intr.p1 = 0.001;
  intr.p2 = -0.002;
  intr.k3 = 0.01;

  // Test several points across the field of view
  const std::vector<Eigen::Vector3d> test_points = {
      {0.1, 0.1, 1.5},
      {-0.2, 0.15, 2.0},
      {0.25, -0.2, 1.8},
      {-0.15, -0.1, 1.2},
  };

  for (const auto& p : test_points) {
    Eigen::Vector2d uv;
    ASSERT_TRUE(intr.ProjectPoint(p, &uv));

    // Normalized ideal coordinates
    const double expected_x = p.x() / p.z();
    const double expected_y = p.y() / p.z();

    // Undistort pixel coordinates
    const Eigen::Vector2d norm_xy = intr.UndistortNormalizedPoint(uv, 15);
    EXPECT_NEAR(norm_xy.x(), expected_x, 1e-4);
    EXPECT_NEAR(norm_xy.y(), expected_y, 1e-4);
    const auto ideal = intr.UndistortPoint(uv, 15);
    EXPECT_NEAR(ideal.x(), intr.fx * expected_x + intr.cx, 1e-4);
    EXPECT_NEAR(ideal.y(), intr.fy * expected_y + intr.cy, 1e-4);
  }
}

TEST(CalibrationTest, CameraMounting) {
  CameraMounting mount = CameraMounting::Identity();
  EXPECT_TRUE(mount.T_robot_camera.translation().isZero());

  // Mount camera 0.3m forward, 0.1m up, rotated 45 deg yaw
  const Eigen::Vector3d t(0.3, 0.0, 0.1);
  const auto r =
      talos::geometry::SO3<double>::Exp(Eigen::Vector3d(0.0, 0.0, 0.785398));
  talos::geometry::SE3<double> T_robot_cam(r, t);

  CameraMounting custom = CameraMounting::FromOpticalTransform(T_robot_cam);
  EXPECT_NEAR(custom.T_robot_camera.translation().x(), 0.3, 1e-6);
  EXPECT_NEAR(custom.T_robot_camera.translation().z(), 0.1, 1e-6);
  EXPECT_TRUE(mount.T_robot_camera.act(Eigen::Vector3d::UnitZ())
                  .isApprox(Eigen::Vector3d::UnitX()));
  const auto yawed =
      CameraMounting::FromExtrinsics(0, 0, 0, 0, 0, std::acos(-1.0) / 2);
  EXPECT_TRUE(yawed.T_robot_camera.act(Eigen::Vector3d::UnitZ())
                  .isApprox(Eigen::Vector3d::UnitY()));
}

TEST(CalibrationTest, FieldLayout) {
  FieldLayout layout(0.1651);
  EXPECT_EQ(layout.default_tag_size_m(), 0.1651);
  EXPECT_EQ(layout.size(), 0);

  // Add Tag 1 at (5.0, 2.0, 1.2)
  talos::geometry::SE3<double> T_field_tag1(
      talos::geometry::SO3<double>::Identity(), Eigen::Vector3d(5.0, 2.0, 1.2));
  layout.SetTagPose(1, T_field_tag1);

  EXPECT_TRUE(layout.HasTag(1));
  EXPECT_FALSE(layout.HasTag(2));
  EXPECT_EQ(layout.size(), 1);

  auto pose1 = layout.GetTagPose(1);
  if (!pose1.has_value()) {
    FAIL() << "Missing tag 1 pose";
  }
  const auto& tag_pose = *pose1;
  EXPECT_NEAR(tag_pose.translation().x(), 5.0, 1e-6);
  EXPECT_NEAR(tag_pose.translation().y(), 2.0, 1e-6);
  EXPECT_NEAR(tag_pose.translation().z(), 1.2, 1e-6);

  // Tag sizes
  EXPECT_NEAR(layout.GetTagSize(1), 0.1651, 1e-6);
  layout.SetTagSize(2, 0.20);
  EXPECT_NEAR(layout.GetTagSize(2), 0.20, 1e-6);
}

}  // namespace
}  // namespace talos::vision
