#include "talOS/vision/calibration.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
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

  EXPECT_TRUE(mount.T_robot_camera.act(Eigen::Vector3d::UnitZ())
                  .isApprox(Eigen::Vector3d::UnitX()));
  const auto yawed =
      CameraMounting::FromExtrinsics(0, 0, 0, 0, 0, std::acos(-1.0) / 2);
  EXPECT_TRUE(yawed.T_robot_camera.act(Eigen::Vector3d::UnitZ())
                  .isApprox(Eigen::Vector3d::UnitY()));
}

TEST(CalibrationTest, UndistortGuardsNonPositiveRadial) {
  CameraIntrinsics intr;
  intr.fx = 600.0;
  intr.fy = 600.0;
  intr.cx = 320.0;
  intr.cy = 240.0;
  intr.k1 = -0.5;

  // radial == 0.0 on first iteration (r2 = 2.0): must not divide by zero.
  const double x_zero = std::sqrt(2.0);
  const Eigen::Vector2d p_zero =
      intr.UndistortNormalizedPoint({intr.cx + intr.fx * x_zero, intr.cy});
  EXPECT_TRUE(std::isfinite(p_zero.x()));
  EXPECT_TRUE(std::isfinite(p_zero.y()));
  EXPECT_DOUBLE_EQ(p_zero.x(), x_zero);
  EXPECT_DOUBLE_EQ(p_zero.y(), 0.0);

  // radial < 0.0 on first iteration (r2 = 4.0): must not flip sign or diverge.
  const Eigen::Vector2d p_neg =
      intr.UndistortNormalizedPoint({intr.cx + intr.fx * 2.0, intr.cy});
  EXPECT_TRUE(std::isfinite(p_neg.x()));
  EXPECT_TRUE(std::isfinite(p_neg.y()));
  EXPECT_DOUBLE_EQ(p_neg.x(), 2.0);
  EXPECT_DOUBLE_EQ(p_neg.y(), 0.0);

  // Diverges into radial < 0.0 on second iteration (r2 = 1.96 -> x1 = 70.0).
  const Eigen::Vector2d p_div =
      intr.UndistortNormalizedPoint({intr.cx + intr.fx * 1.4, intr.cy});
  EXPECT_TRUE(std::isfinite(p_div.x()));
  EXPECT_TRUE(std::isfinite(p_div.y()));
  EXPECT_DOUBLE_EQ(p_div.x(), 1.4);
  EXPECT_DOUBLE_EQ(p_div.y(), 0.0);
}

TEST(CalibrationTest, IsValidRejectsNonFiniteIntrinsicsAndDistortion) {
  auto make_valid = [] {
    CameraIntrinsics intr;
    intr.fx = 600.0;
    intr.fy = 600.0;
    intr.cx = 320.0;
    intr.cy = 240.0;
    return intr;
  };
  ASSERT_TRUE(make_valid().IsValid());

  const double bad_values[] = {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
  };
  double CameraIntrinsics::*const fields[] = {
      &CameraIntrinsics::fx, &CameraIntrinsics::fy, &CameraIntrinsics::cx,
      &CameraIntrinsics::cy, &CameraIntrinsics::k1, &CameraIntrinsics::k2,
      &CameraIntrinsics::p1, &CameraIntrinsics::p2, &CameraIntrinsics::k3,
  };
  for (const double bad : bad_values) {
    for (auto field : fields) {
      CameraIntrinsics intr = make_valid();
      intr.*field = bad;
      EXPECT_FALSE(intr.IsValid());
    }
  }
}

TEST(CalibrationTest, ProjectPointRejectsNonFiniteInputsAndOverflow) {
  CameraIntrinsics intr;
  intr.fx = 500.0;
  intr.fy = 500.0;
  intr.cx = 320.0;
  intr.cy = 240.0;
  intr.k3 = 1.0;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  Eigen::Vector2d uv(123.0, 456.0);

  EXPECT_FALSE(intr.ProjectPoint({0.0, 0.0, nan}, &uv));
  EXPECT_FALSE(intr.ProjectPoint({nan, 0.0, 1.0}, &uv));
  EXPECT_FALSE(intr.ProjectPoint({0.0, inf, 1.0}, &uv));
  EXPECT_FALSE(intr.ProjectPoint({0.0, 0.0, inf}, &uv));
  EXPECT_FALSE(intr.ProjectPoint({1e60, 0.0, 1.0}, &uv));
  EXPECT_FALSE(intr.ProjectPoint({0.0, 0.0, 1.0}, nullptr));
  // Rejected projections must not clobber uv_out with non-finite values.
  EXPECT_DOUBLE_EQ(uv.x(), 123.0);
  EXPECT_DOUBLE_EQ(uv.y(), 456.0);
}

TEST(CalibrationTest, UndistortGuardsInfiniteRadialAndFinalIterationOverflow) {
  CameraIntrinsics intr;
  intr.fx = 600.0;
  intr.fy = 600.0;
  intr.cx = 320.0;
  intr.cy = 240.0;
  intr.k3 = 1.0;

  // r6 overflows to +inf so radial == +inf; must fall back to (x_dist, y_dist)
  // rather than collapsing to 0 or producing NaN.
  const double x_huge = 1e60;
  const Eigen::Vector2d uv_huge(intr.cx + intr.fx * x_huge, intr.cy);
  const Eigen::Vector2d p_inf_radial = intr.UndistortNormalizedPoint(uv_huge);
  EXPECT_TRUE(std::isfinite(p_inf_radial.x()));
  EXPECT_TRUE(std::isfinite(p_inf_radial.y()));
  EXPECT_DOUBLE_EQ(p_inf_radial.x(), (uv_huge.x() - intr.cx) / intr.fx);
  EXPECT_DOUBLE_EQ(p_inf_radial.y(), 0.0);

  // Overflow on the final iteration (radial == 1.0, dx == +inf) must not
  // return non-finite coordinates.
  CameraIntrinsics tangential;
  tangential.fx = 600.0;
  tangential.fy = 600.0;
  tangential.cx = 320.0;
  tangential.cy = 240.0;
  tangential.p2 = 1e300;
  const double x_big = 1e10;
  const Eigen::Vector2d uv_big(tangential.cx + tangential.fx * x_big,
                               tangential.cy);
  const Eigen::Vector2d p_overflow =
      tangential.UndistortNormalizedPoint(uv_big, /*max_iterations=*/1);
  EXPECT_TRUE(std::isfinite(p_overflow.x()));
  EXPECT_TRUE(std::isfinite(p_overflow.y()));
  EXPECT_DOUBLE_EQ(p_overflow.x(),
                   (uv_big.x() - tangential.cx) / tangential.fx);
  EXPECT_DOUBLE_EQ(p_overflow.y(), 0.0);
}

}  // namespace
}  // namespace talos::vision
