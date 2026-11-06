#include "talOS/vision/localization.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace talos::vision {
namespace {

TEST(LocalizationTest, TagModelCorners) {
  // Layout tag frame (+X out of the face, +Z up), detector index order:
  // bottom-left, bottom-right, top-right, top-left as seen facing the tag.
  const double size_m = 0.1651;
  const auto corners = GetTagModelCorners(size_m);
  const double half = size_m * 0.5;

  EXPECT_NEAR(corners[0].x(), 0.0, 1e-6);
  EXPECT_NEAR(corners[0].y(), -half, 1e-6);
  EXPECT_NEAR(corners[0].z(), -half, 1e-6);

  EXPECT_NEAR(corners[1].x(), 0.0, 1e-6);
  EXPECT_NEAR(corners[1].y(), half, 1e-6);
  EXPECT_NEAR(corners[1].z(), -half, 1e-6);

  EXPECT_NEAR(corners[2].x(), 0.0, 1e-6);
  EXPECT_NEAR(corners[2].y(), half, 1e-6);
  EXPECT_NEAR(corners[2].z(), half, 1e-6);

  EXPECT_NEAR(corners[3].x(), 0.0, 1e-6);
  EXPECT_NEAR(corners[3].y(), -half, 1e-6);
  EXPECT_NEAR(corners[3].z(), half, 1e-6);
}

TEST(LocalizationTest, ReprojectionErrorCalculation) {
  CameraIntrinsics intr;
  intr.fx = 600.0;
  intr.fy = 600.0;
  intr.cx = 320.0;
  intr.cy = 240.0;

  const double tag_size = 0.20;
  talos::geometry::SE3<double> T_cam_tag(
      talos::geometry::SO3<double>::Identity(), Eigen::Vector3d(0.0, 0.0, 2.0));

  const auto model_corners = GetTagModelCorners(tag_size);
  std::array<float, 8> perfect_corners;
  for (size_t i = 0; i < 4; ++i) {
    Eigen::Vector3d p = T_cam_tag.act(model_corners[i]);
    Eigen::Vector2d uv;
    ASSERT_TRUE(intr.ProjectPoint(p, &uv));
    perfect_corners[i * 2] = static_cast<float>(uv.x());
    perfect_corners[i * 2 + 1] = static_cast<float>(uv.y());
  }

  // Exact projection should have near zero error
  double err =
      ComputeReprojectionError(perfect_corners, T_cam_tag, intr, tag_size);
  EXPECT_NEAR(err, 0.0, 1e-5);

  // Perturb corners by 2 pixels
  std::array<float, 8> noisy_corners = perfect_corners;
  noisy_corners[0] += 2.0f;
  noisy_corners[2] -= 2.0f;
  noisy_corners[5] += 2.0f;
  noisy_corners[7] -= 2.0f;

  double noisy_err =
      ComputeReprojectionError(noisy_corners, T_cam_tag, intr, tag_size);
  EXPECT_GT(noisy_err, 1.0);
  EXPECT_LT(noisy_err, 3.0);
}

TEST(LocalizationTest, RobotFieldPoseAnalytical) {
  // Analytical test:
  // Robot at field origin (0, 0, 0).
  // Camera mounted at robot origin (identity).
  // Tag 1 is at field position (4.0, 0.0, 1.0).
  // Camera looks directly at Tag 1: camera observes tag at (0.0, 0.0, 4.0).
  // T_field_tag = (4.0, 0.0, 1.0)
  // T_camera_tag = (0.0, 0.0, 4.0) with camera Z pointing at tag
  // In our coordinate convention:
  // p_field = T_field_tag * T_camera_tag.inverse() * T_robot_camera.inverse() *
  // p_robot

  FieldLayout layout;
  layout.SetTagPose(
      1, talos::geometry::SE3<double>(talos::geometry::SO3<double>::Identity(),
                                      Eigen::Vector3d(4.0, 0.0, 1.0)));

  CameraMounting mount = CameraMounting::Identity();

  const auto tag_pose = layout.GetTagPose(1);
  if (!tag_pose.has_value()) {
    FAIL() << "Missing tag 1 pose";
  }
  const auto T_camera_tag = mount.T_robot_camera.inverse() * *tag_pose;

  RobotPoseEstimate res =
      ComputeRobotFieldPose(1, 0, T_camera_tag, mount, layout, 0.5, 0.1);

  ASSERT_TRUE(res.validity.valid);
  EXPECT_EQ(res.validity.reason, PoseInvalidReason::kValid);

  EXPECT_TRUE(res.T_field_robot.translation().isZero(1e-6));
  EXPECT_TRUE(res.T_field_robot.rotation().rotationMatrix().isApprox(
      Eigen::Matrix3d::Identity(), 1e-6));
}

TEST(LocalizationTest, RobotFieldPoseWithMountingOffset) {
  FieldLayout layout;
  layout.SetTagPose(
      5, talos::geometry::SE3<double>(talos::geometry::SO3<double>::Identity(),
                                      Eigen::Vector3d(10.0, 5.0, 0.0)));

  // Camera mounted 0.5m in front of robot center: T_robot_camera = (0.5, 0.0,
  // 0.0)
  CameraMounting mount = CameraMounting::FromOpticalTransform(
      talos::geometry::SE3<double>(talos::geometry::SO3<double>::Identity(),
                                   Eigen::Vector3d(0.5, 0.0, 0.0)));

  // Tag observed at (0.0, 0.0, 2.0) relative to camera
  talos::geometry::SE3<double> T_camera_tag(
      talos::geometry::SO3<double>::Identity(), Eigen::Vector3d(0.0, 0.0, 2.0));

  RobotPoseEstimate res =
      ComputeRobotFieldPose(5, 0, T_camera_tag, mount, layout, 0.2, 0.05);

  ASSERT_TRUE(res.validity.valid);
  // T_field_robot = T_field_tag * T_camera_tag^-1 * T_robot_cam^-1
  // translation = (10, 5, 0) + (0, 0, -2) + (-0.5, 0, 0) = (9.5, 5.0, -2.0)
  EXPECT_NEAR(res.T_field_robot.translation().x(), 9.5, 1e-6);
  EXPECT_NEAR(res.T_field_robot.translation().y(), 5.0, 1e-6);
  EXPECT_NEAR(res.T_field_robot.translation().z(), -2.0, 1e-6);
}

TEST(LocalizationTest, ValidityRejections) {
  FieldLayout layout;
  layout.SetTagPose(
      1, talos::geometry::SE3<double>(talos::geometry::SO3<double>::Identity(),
                                      Eigen::Vector3d(5.0, 0.0, 0.0)));
  CameraMounting mount = CameraMounting::Identity();

  talos::geometry::SE3<double> valid_pose(
      talos::geometry::SO3<double>::Identity(), Eigen::Vector3d(0.0, 0.0, 2.0));

  // 1. Tag not in layout
  auto res_unknown = ComputeRobotFieldPose(99, 0, valid_pose, mount, layout);
  EXPECT_FALSE(res_unknown.validity.valid);
  EXPECT_EQ(res_unknown.validity.reason, PoseInvalidReason::kTagNotInLayout);

  // 2. High reprojection error
  auto res_high_err =
      ComputeRobotFieldPose(1, 0, valid_pose, mount, layout, 5.0);
  EXPECT_FALSE(res_high_err.validity.valid);
  EXPECT_EQ(res_high_err.validity.reason,
            PoseInvalidReason::kHighReprojectionError);

  // 3. High ambiguity
  auto res_high_amb =
      ComputeRobotFieldPose(1, 0, valid_pose, mount, layout, 0.5, 0.8);
  EXPECT_FALSE(res_high_amb.validity.valid);
  EXPECT_EQ(res_high_amb.validity.reason, PoseInvalidReason::kHighAmbiguity);

  // 4. Tag behind camera
  talos::geometry::SE3<double> pose_behind(
      talos::geometry::SO3<double>::Identity(),
      Eigen::Vector3d(0.0, 0.0, -1.0));
  auto res_behind = ComputeRobotFieldPose(1, 0, pose_behind, mount, layout);
  EXPECT_FALSE(res_behind.validity.valid);
  EXPECT_EQ(res_behind.validity.reason, PoseInvalidReason::kBehindCamera);

  // 5. Tag too far (> 15m)
  talos::geometry::SE3<double> pose_far(
      talos::geometry::SO3<double>::Identity(),
      Eigen::Vector3d(0.0, 0.0, 20.0));
  auto res_far = ComputeRobotFieldPose(1, 0, pose_far, mount, layout);
  EXPECT_FALSE(res_far.validity.valid);
  EXPECT_EQ(res_far.validity.reason, PoseInvalidReason::kTooFar);
}

TEST(LocalizationTest, CovarianceScaling) {
  EXPECT_DOUBLE_EQ(ComputeObservationStdDevs(2, 1, 0).x_std_m, 0.32);
  EXPECT_DOUBLE_EQ(ComputeObservationStdDevs(2, 4, 0).x_std_m, 0.16);
  EXPECT_DOUBLE_EQ(ComputeObservationStdDevs(2, 4, 0.5).x_std_m, 0.32);
  EXPECT_DOUBLE_EQ(ComputeObservationStdDevs(0, 0, -1).x_std_m, 0.02);
}

TEST(LocalizationTest, SecondaryPoseResolvesFloorAndAmbiguity) {
  using Pose = talos::geometry::SE3<double>;
  const auto mount = CameraMounting::Identity();
  FieldLayout layout;
  layout.SetTagPose(1,
                    Pose(talos::geometry::SO3<double>::Identity(), {4, 0, 1}));
  const auto tag_pose = layout.GetTagPose(1);
  if (!tag_pose.has_value()) {
    FAIL() << "Missing tag 1 pose";
  }
  const auto good = mount.T_robot_camera.inverse() * *tag_pose;
  const Pose underground(talos::geometry::SO3<double>::Identity(), {0, 0, -1});
  const auto bad =
      mount.T_robot_camera.inverse() * underground.inverse() * *tag_pose;
  const auto result =
      ComputeRobotFieldPose(1, 0, bad, mount, layout, 0.1, 0.9, {}, &good, 0.2);
  ASSERT_TRUE(result.validity.valid);
  EXPECT_TRUE(result.T_field_robot.translation().isZero(1e-8));
  EXPECT_DOUBLE_EQ(result.reprojection_error_px, 0.2);
}

TEST(LocalizationTest, UpstreamCornerOrderRecoversPositiveDepth) {
  using Pose = talos::geometry::SE3<double>;
  CameraIntrinsics intr;
  intr.fx = 600;
  intr.fy = 610;
  intr.cx = 320;
  intr.cy = 240;
  const Pose solver(talos::geometry::SO3<double>::Exp({0.15, -0.25, 0.08}),
                    {0.2, -0.1, 3.0});
  // Specify upstream solver points independently of the layout helper so a
  // mirrored DLT correspondence cannot pass by sharing the same mistake.
  const std::array<Eigen::Vector3d, 4> points = {
      {{-0.2, 0.2, 0}, {0.2, 0.2, 0}, {0.2, -0.2, 0}, {-0.2, -0.2, 0}}};
  const auto expected = SolverTagFrameToLayoutFrame(solver);
  const auto layout_points = GetTagModelCorners(0.4);
  std::array<float, 8> observed{};
  for (size_t j = 0; j < 4; ++j) {
    EXPECT_TRUE(
        solver.act(points[j]).isApprox(expected.act(layout_points[j]), 1e-12));
    Eigen::Vector2d uv;
    ASSERT_TRUE(intr.ProjectPoint(solver.act(points[j]), &uv));
    observed[2 * j] = static_cast<float>(uv.x());
    observed[2 * j + 1] = static_cast<float>(uv.y());
  }
  Pose primary, secondary;
  double error1, error2, ambiguity;
  ASSERT_TRUE(EstimateTagPoseFromHomography({}, intr, 0.4, &primary, &secondary,
                                            &error1, &error2, &ambiguity,
                                            &observed));
  EXPECT_GT(primary.translation().z(), 0);
  EXPECT_LT(error1, 0.01);
  EXPECT_TRUE(primary.matrix().isApprox(expected.matrix(), 1e-4));
}

TEST(LocalizationTest, DistortedHomographyAndJointPnP) {
  using Pose = talos::geometry::SE3<double>;
  CameraIntrinsics intr;
  intr.fx = 600;
  intr.fy = 610;
  intr.cx = 320;
  intr.cy = 240;
  intr.k1 = -0.15;
  intr.k2 = 0.04;
  intr.p1 = 0.001;
  intr.p2 = -0.002;
  const auto mount = CameraMounting::Identity();
  FieldLayout layout(0.4);
  std::vector<TagDetection3D> observations;
  for (int id = 1; id <= 2; ++id) {
    const auto solver =
        Pose(talos::geometry::SO3<double>::Exp({0.1, 0.2, 0.05}),
             {id == 1 ? -0.5 : 0.7, 0.1, 3.0});
    const auto optical = SolverTagFrameToLayoutFrame(solver);
    layout.SetTagPose(id, mount.T_robot_camera * optical);
    TagDetection3D tag;
    tag.tag_id = id;
    tag.valid = true;
    tag.T_camera_tag = optical;
    tag.reprojection_error = 0.01;
    const auto corners = GetTagModelCorners(0.4);
    for (size_t j = 0; j < 4; ++j) {
      Eigen::Vector2d uv;
      ASSERT_TRUE(intr.ProjectPoint(optical.act(corners[j]), &uv));
      tag.corners[2 * j] = static_cast<float>(uv.x());
      tag.corners[2 * j + 1] = static_cast<float>(uv.y());
    }
    Pose primary, secondary;
    double ambiguity, err1, err2;
    ASSERT_TRUE(EstimateTagPoseFromHomography({}, intr, 0.4, &primary,
                                              &secondary, &err1, &err2,
                                              &ambiguity, &tag.corners));
    EXPECT_LT(err1, 0.01);
    EXPECT_LT((primary.translation() - optical.translation()).norm(), 0.01);
    EXPECT_TRUE(LayoutFrameToSolverTagFrame(optical).matrix().isApprox(
        solver.matrix(), 1e-10));
    Eigen::Matrix<double, 6, 1> offset;
    offset << 0.01, -0.01, 0.005, 0.03, -0.02, 0.01;
    const auto tag_pose = layout.GetTagPose(id);
    if (!tag_pose.has_value()) {
      FAIL() << "Missing tag pose for " << id;
    }
    tag.T_camera_tag = mount.T_robot_camera.inverse() *
                       Pose::Identity().boxplus(offset).inverse() * *tag_pose;
    observations.push_back(tag);
  }
  const auto result =
      SolveMultiTagRobotPose(observations, 7, mount, layout, intr);
  ASSERT_TRUE(result.validity.valid);
  EXPECT_EQ(result.tag_count, 2);
  EXPECT_EQ(result.camera_id, 7);
  EXPECT_LT(result.T_field_robot.translation().norm(), 0.002);
  EXPECT_LT(result.reprojection_error_px, 0.01);
  EXPECT_DOUBLE_EQ(result.ambiguity, 0);
  observations.resize(1);
  const auto single =
      SolveMultiTagRobotPose(observations, 7, mount, layout, intr);
  EXPECT_TRUE(single.validity.valid);
  EXPECT_EQ(single.tag_count, 1);
}

TEST(LocalizationTest, JointUncertaintyUsesMeanCameraRangeAndZeroAmbiguity) {
  using Pose = talos::geometry::SE3<double>;
  CameraIntrinsics intr;
  intr.fx = 600;
  intr.fy = 610;
  intr.cx = 320;
  intr.cy = 240;
  const auto mount = CameraMounting::FromExtrinsics(0.3, -0.1, 0.5, 0, 0, 0.2);
  const Pose robot(talos::geometry::SO3<double>::Exp({0, 0, 0.4}),
                   {1.99, 2.01, 0});
  FieldLayout layout(0.1651);
  std::vector<TagDetection3D> tags;
  double true_range_sum = 0;
  for (int id = 1; id <= 3; ++id) {
    const Eigen::Vector3d camera_translation(0.15 * (id - 2), -0.1, id + 1.0);
    const Pose camera_tag_solver(
        talos::geometry::SO3<double>::Exp({0.1, 0.2, 0}), camera_translation);
    const auto camera_tag = SolverTagFrameToLayoutFrame(camera_tag_solver);
    layout.SetTagPose(id, robot * mount.T_robot_camera * camera_tag);
    true_range_sum += camera_translation.norm();
    TagDetection3D tag;
    tag.tag_id = id;
    tag.valid = true;
    tag.T_camera_tag = camera_tag;
    // This seed ambiguity must not leak into joint measurement uncertainty.
    tag.ambiguity = 0.9;
    const auto corners = GetTagModelCorners(layout.GetTagSize(id));
    for (size_t j = 0; j < 4; ++j) {
      Eigen::Vector2d uv;
      ASSERT_TRUE(intr.ProjectPoint(camera_tag.act(corners[j]), &uv));
      tag.corners[2 * j] = static_cast<float>(uv.x());
      tag.corners[2 * j + 1] = static_cast<float>(uv.y());
    }
    tags.push_back(tag);
  }
  const auto result = SolveMultiTagRobotPose(tags, 3, mount, layout, intr);
  ASSERT_TRUE(result.validity.valid);
  EXPECT_EQ(result.tag_count, 3);
  EXPECT_TRUE(result.T_field_robot.matrix().isApprox(robot.matrix(), 1e-4));
  const double mean = true_range_sum / 3;
  EXPECT_NEAR(result.average_distance_m, mean, 0.01);
  EXPECT_DOUBLE_EQ(result.ambiguity, 0);
  const double scale = mean * mean / std::sqrt(3.0);
  EXPECT_NEAR(result.std_devs.x_std_m, 0.08 * scale, 1e-12);
  EXPECT_NEAR(result.std_devs.y_std_m, 0.08 * scale, 1e-12);
  EXPECT_NEAR(result.std_devs.yaw_std_rad, 0.06 * scale, 1e-12);
  // A distant field origin must not inflate a camera-relative range.
  const Pose shift(talos::geometry::SO3<double>::Identity(), {20, -30, 0});
  for (int id = 1; id <= 3; ++id) {
    const auto tag_pose = layout.GetTagPose(id);
    if (!tag_pose.has_value()) {
      FAIL() << "Missing tag pose for " << id;
    }
    layout.SetTagPose(id, shift * *tag_pose);
  }
  const auto shifted = SolveMultiTagRobotPose(tags, 3, mount, layout, intr);
  ASSERT_TRUE(shifted.validity.valid);
  EXPECT_NEAR(shifted.average_distance_m, mean, 1e-12);
  EXPECT_NEAR(shifted.std_devs.x_std_m, result.std_devs.x_std_m, 1e-12);
}

TEST(LocalizationTest, InvalidDetectionCannotEnterJointSolve) {
  using Pose = talos::geometry::SE3<double>;
  CameraIntrinsics intr;
  intr.fx = 600;
  intr.fy = 600;
  intr.cx = 320;
  intr.cy = 240;
  const auto mount = CameraMounting::Identity();
  FieldLayout layout;
  TagDetection3D good;
  good.tag_id = 1;
  good.valid = true;
  good.T_camera_tag = SolverTagFrameToLayoutFrame(
      Pose(talos::geometry::SO3<double>::Identity(), {0, 0, 3}));
  layout.SetTagPose(1, mount.T_robot_camera * good.T_camera_tag);
  TagDetection3D invalid = good;
  invalid.tag_id = 2;
  invalid.valid = false;
  invalid.corners.fill(10000);
  layout.SetTagPose(
      2, Pose(talos::geometry::SO3<double>::Identity(), {50, 50, 0}));
  const auto single = SolveMultiTagRobotPose({good}, 0, mount, layout, intr);
  const auto mixed =
      SolveMultiTagRobotPose({good, invalid}, 0, mount, layout, intr);
  ASSERT_TRUE(single.validity.valid);
  ASSERT_TRUE(mixed.validity.valid);
  EXPECT_EQ(mixed.tag_count, 1);
  EXPECT_TRUE(
      mixed.T_field_robot.matrix().isApprox(single.T_field_robot.matrix()));
  EXPECT_DOUBLE_EQ(mixed.average_distance_m, single.average_distance_m);
  EXPECT_DOUBLE_EQ(mixed.std_devs.x_std_m, single.std_devs.x_std_m);
}

TEST(LocalizationTest, CandidateOrderingKeepsLowerErrorPrimary) {
  using Pose = talos::geometry::SE3<double>;
  const Pose better(talos::geometry::SO3<double>::Identity(), {0, 0, 3});
  Pose primary(talos::geometry::SO3<double>::Identity(), {1, 0, 3});
  Pose secondary = better;
  double first_error = 2, second_error = 0.5;
  EXPECT_DOUBLE_EQ(detail::OrderPoseCandidates(primary, secondary, first_error,
                                               second_error),
                   0.25);
  EXPECT_TRUE(primary.matrix().isApprox(better.matrix()));
  EXPECT_DOUBLE_EQ(first_error, 0.5);
  // Small but distinct solver errors must not be classified as fully ambiguous.
  first_error = 1e-8;
  second_error = 1e-7;
  EXPECT_NEAR(detail::OrderPoseCandidates(primary, secondary, first_error,
                                          second_error),
              0.1, 1e-12);
  first_error = 0;
  second_error = 0;
  EXPECT_DOUBLE_EQ(detail::OrderPoseCandidates(primary, secondary, first_error,
                                               second_error),
                   1);
}

}  // namespace
}  // namespace talos::vision
