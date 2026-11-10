#include "talOS/vision/localization.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace talos::vision {
namespace {

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

struct JointFixture {
  using Pose = talos::geometry::SE3<double>;
  CameraIntrinsics intr;
  CameraMounting mount =
      CameraMounting::FromExtrinsics(0.2, -0.1, 0.5, 0, 0, 0.2);
  FieldLayout layout{0.4};
  std::vector<TagDetection3D> tags;
  Pose robot{talos::geometry::SO3<double>::Exp({0, 0, 0.4}), {2, 2, 0}};

  explicit JointFixture(double distance = 3, int count = 6,
                        const Pose& floor_offset = Pose::Identity()) {
    intr.fx = 600;
    intr.fy = 610;
    intr.cx = 320;
    intr.cy = 240;
    robot = robot * floor_offset;
    for (int id = 1; id <= count; ++id) {
      const int row = (id - 1) / 3;
      const Pose optical = SolverTagFrameToLayoutFrame(
          Pose(talos::geometry::SO3<double>::Exp({0.1, 0.2, 0.05}),
               {0.5 * ((id - 1) % 3 - 1), 0.3 * static_cast<double>(row) - 0.2,
                distance + 0.15 * (id % 2)}));
      layout.SetTagPose(id, robot * mount.T_robot_camera * optical);
      TagDetection3D tag;
      tag.tag_id = id;
      tag.valid = true;
      tag.T_camera_tag = optical;
      const auto corners = GetTagModelCorners(layout.GetTagSize(id));
      for (size_t j = 0; j < corners.size(); ++j) {
        Eigen::Vector2d uv;
        EXPECT_TRUE(intr.ProjectPoint(optical.act(corners[j]), &uv));
        tag.corners[2 * j] = static_cast<float>(uv.x());
        tag.corners[2 * j + 1] = static_cast<float>(uv.y());
      }
      tags.push_back(tag);
    }
  }

  RobotPoseEstimate Solve(const PoseSolverOptions& noise = {},
                          double max_error_px = 3.5) const {
    PoseValidationOptions validation;
    validation.max_reprojection_error_px = max_error_px;
    return SolveMultiTagRobotPose(tags, 1, mount, layout, intr, validation,
                                  noise);
  }
};

TEST(LocalizationTest, SolverCovarianceShrinksWithTagsAndGrowsWithDistance) {
  PoseSolverOptions noise;
  noise.min_xy_std_m = 0;
  noise.min_yaw_std_rad = 0;
  const auto single = JointFixture(3, 1).Solve(noise);
  const auto near = JointFixture(3, 6).Solve(noise);
  const auto far = JointFixture(7, 6).Solve(noise);
  ASSERT_TRUE(single.validity.valid);
  ASSERT_TRUE(near.validity.valid);
  ASSERT_TRUE(far.validity.valid);
  EXPECT_LT(near.std_devs.x_std_m, single.std_devs.x_std_m);
  EXPECT_LT(near.std_devs.y_std_m, single.std_devs.y_std_m);
  EXPECT_LT(near.std_devs.yaw_std_rad, single.std_devs.yaw_std_rad);
  EXPECT_GT(far.std_devs.x_std_m, near.std_devs.x_std_m);
  EXPECT_GT(far.std_devs.y_std_m, near.std_devs.y_std_m);
  EXPECT_GT(far.std_devs.yaw_std_rad, near.std_devs.yaw_std_rad);
  for (const int count : {1, 6}) {
    const auto floored = JointFixture(3, count).Solve();
    ASSERT_TRUE(floored.validity.valid);
    EXPECT_GE(floored.std_devs.x_std_m, 0.02);
    EXPECT_GE(floored.std_devs.y_std_m, 0.02);
    EXPECT_GE(floored.std_devs.yaw_std_rad,
              PoseSolverOptions{}.min_yaw_std_rad);
  }
}

TEST(LocalizationTest, FitResidualInflatesPublishedCovariance) {
  PoseSolverOptions noise;
  noise.min_xy_std_m = 0;
  noise.min_yaw_std_rad = 0;
  noise.mount_angle_std_rad = 0;
  noise.mount_translation_std_m = 0;
  JointFixture fixture(3, 6);
  const auto clean = fixture.Solve(noise);
  // Shear every tag by 1.2 px per coordinate: under the 2 px Huber transition
  // and the 3.5 px reject threshold, yet about 3.6 nominal sigma. Accepting
  // that fit must not also publish the clean fit's uncertainty.
  for (auto& tag : fixture.tags) {
    for (size_t i = 0; i < tag.corners.size(); ++i) {
      tag.corners[i] += (i / 2) % 2 == 0 ? 1.2f : -1.2f;
    }
  }
  const auto noisy = fixture.Solve(noise);
  ASSERT_TRUE(clean.validity.valid);
  ASSERT_TRUE(noisy.validity.valid) << noisy.validity.message;
  EXPECT_GT(noisy.std_devs.x_std_m, 2 * clean.std_devs.x_std_m);
  EXPECT_GT(noisy.std_devs.y_std_m, 2 * clean.std_devs.y_std_m);
  EXPECT_GT(noisy.std_devs.yaw_std_rad, 2 * clean.std_devs.yaw_std_rad);
}

TEST(LocalizationTest, HuberSuppressesTwentyPixelTagOutlier) {
  JointFixture fixture(3, 9);
  for (size_t i = 0; i < 4; ++i) {
    fixture.tags.back().corners[2 * i] += 20;
    fixture.tags.back().corners[2 * i + 1] -= 20;
  }
  const auto robust = fixture.Solve({}, 100);
  PoseSolverOptions least_squares;
  least_squares.huber_delta_px = 1e6;
  const auto plain = fixture.Solve(least_squares, 100);
  ASSERT_TRUE(robust.validity.valid);
  ASSERT_TRUE(plain.validity.valid);
  const double robust_error =
      (robust.T_field_robot.translation() - fixture.robot.translation()).norm();
  const double plain_error =
      (plain.T_field_robot.translation() - fixture.robot.translation()).norm();
  EXPECT_LT(robust_error, 0.04);
  EXPECT_GT(plain_error, 0.04);
  EXPECT_LT(robust_error, plain_error * 0.4);
  // Robust weighting must not disguise the reported RMS pixel error.
  EXPECT_GT(robust.reprojection_error_px, 5);
}

TEST(LocalizationTest, WhitenedFloorPriorUsesMetresRadiansAndPixelNoise) {
  using Pose = talos::geometry::SE3<double>;
  const JointFixture fixture(
      3, 3,
      Pose(talos::geometry::SO3<double>::Exp({0.12, -0.08, 0}), {0, 0, 0.2}));
  PoseSolverOptions weak;
  weak.floor_z_std_m = 100;
  weak.floor_tilt_std_rad = 100;
  weak.huber_delta_px = 1e6;
  const auto unconstrained = fixture.Solve(weak, 100);
  PoseSolverOptions strong = weak;
  // Dominate image geometry so every floor component is pulled toward zero,
  // rather than trading height against pitch within a moderately weak prior.
  strong.floor_z_std_m = 0.0001;
  strong.floor_tilt_std_rad = 0.0001;
  const auto constrained = fixture.Solve(strong, 100);
  strong.pixel_std_px = 10;
  const auto noisier_pixels = fixture.Solve(strong, 100);
  ASSERT_TRUE(unconstrained.validity.valid);
  ASSERT_TRUE(constrained.validity.valid);
  ASSERT_TRUE(noisier_pixels.validity.valid);
  const auto a = FloorResidual(unconstrained.T_field_robot).cwiseAbs().eval();
  const auto b = FloorResidual(constrained.T_field_robot).cwiseAbs().eval();
  const auto c = FloorResidual(noisier_pixels.T_field_robot).cwiseAbs().eval();
  for (int i = 0; i < 3; ++i) {
    EXPECT_LT(b(i), a(i));
    EXPECT_LT(c(i), b(i));
  }
}

TEST(LocalizationTest, CovarianceMapsRightIncrementIntoFieldCoordinates) {
  RobotPoseEstimate estimate;
  estimate.T_field_robot = {talos::geometry::SO3<double>::Exp({0.3, -0.4, 0.7}),
                            {4, -3, 0.2}};
  detail::PoseNormalMatrix normal = detail::PoseNormalMatrix::Zero();
  normal.diagonal() << 2, 3, 5, 7, 11, 13;
  PoseSolverOptions noise;
  noise.min_xy_std_m = 0;
  noise.min_yaw_std_rad = 0;
  // A perfectly calibrated mount (zero uncertainty) must reproduce the pure
  // corner covariance exactly.
  noise.mount_angle_std_rad = 0;
  noise.mount_translation_std_m = 0;
  ASSERT_TRUE(detail::SetCovarianceStdDevs(normal, noise, &estimate));
  Eigen::Matrix<double, 3, 6> numerical;
  auto coordinates = [](const auto& pose) {
    const auto r = pose.rotation().rotationMatrix();
    return Eigen::Vector3d(pose.translation().x(), pose.translation().y(),
                           std::atan2(r(1, 0), r(0, 0)));
  };
  for (int i = 0; i < 6; ++i) {
    Eigen::Matrix<double, 6, 1> step = Eigen::Matrix<double, 6, 1>::Zero();
    step(i) = 1e-6;
    numerical.col(i) = (coordinates(estimate.T_field_robot.boxplus(step)) -
                        coordinates(estimate.T_field_robot.boxplus(-step))) /
                       2e-6;
  }
  const Eigen::Matrix3d expected =
      numerical * normal.inverse() * numerical.transpose();
  EXPECT_NEAR(estimate.std_devs.x_std_m, std::sqrt(expected(0, 0)), 1e-8);
  EXPECT_NEAR(estimate.std_devs.y_std_m, std::sqrt(expected(1, 1)), 1e-8);
  EXPECT_NEAR(estimate.std_devs.yaw_std_rad, std::sqrt(expected(2, 2)), 1e-8);
}

TEST(LocalizationTest, MountUncertaintyInflatesCovarianceWithRange) {
  // An identity normal matrix maps to unit x/y/yaw variance; the mount terms
  // must add (range * angle_std)^2 + translation_std^2 to x/y and angle_std^2
  // to yaw before the floors.
  auto make_estimate = [] {
    RobotPoseEstimate estimate;
    estimate.T_field_robot = {talos::geometry::SO3<double>::Identity(),
                              Eigen::Vector3d::Zero()};
    estimate.average_distance_m = 4.0;
    return estimate;
  };
  const detail::PoseNormalMatrix normal = detail::PoseNormalMatrix::Identity();

  PoseSolverOptions zero;
  zero.min_xy_std_m = 0;
  zero.min_yaw_std_rad = 0;
  zero.mount_angle_std_rad = 0;
  zero.mount_translation_std_m = 0;
  RobotPoseEstimate base = make_estimate();
  ASSERT_TRUE(detail::SetCovarianceStdDevs(normal, zero, &base));
  // Zero mount uncertainty reproduces the pure corner covariance exactly.
  EXPECT_DOUBLE_EQ(base.std_devs.x_std_m, 1.0);
  EXPECT_DOUBLE_EQ(base.std_devs.y_std_m, 1.0);
  EXPECT_DOUBLE_EQ(base.std_devs.yaw_std_rad, 1.0);

  PoseSolverOptions noise = zero;
  noise.mount_angle_std_rad = 0.01;
  noise.mount_translation_std_m = 0.02;
  RobotPoseEstimate estimate = make_estimate();
  ASSERT_TRUE(detail::SetCovarianceStdDevs(normal, noise, &estimate));

  const double mount_xy_var = 0.02 * 0.02 + (4.0 * 0.01) * (4.0 * 0.01);
  const double mount_yaw_var = 0.01 * 0.01;
  EXPECT_NEAR(estimate.std_devs.x_std_m * estimate.std_devs.x_std_m -
                  base.std_devs.x_std_m * base.std_devs.x_std_m,
              mount_xy_var, 1e-12);
  EXPECT_NEAR(estimate.std_devs.y_std_m * estimate.std_devs.y_std_m -
                  base.std_devs.y_std_m * base.std_devs.y_std_m,
              mount_xy_var, 1e-12);
  EXPECT_NEAR(estimate.std_devs.yaw_std_rad * estimate.std_devs.yaw_std_rad -
                  base.std_devs.yaw_std_rad * base.std_devs.yaw_std_rad,
              mount_yaw_var, 1e-12);

  // The lever-arm term scales with the square of range.
  RobotPoseEstimate farther = make_estimate();
  farther.average_distance_m = 8.0;
  ASSERT_TRUE(detail::SetCovarianceStdDevs(normal, noise, &farther));
  const double farther_xy_var = 0.02 * 0.02 + (8.0 * 0.01) * (8.0 * 0.01);
  EXPECT_NEAR(farther.std_devs.x_std_m * farther.std_devs.x_std_m -
                  base.std_devs.x_std_m * base.std_devs.x_std_m,
              farther_xy_var, 1e-12);
  EXPECT_NEAR(farther.std_devs.yaw_std_rad, estimate.std_devs.yaw_std_rad,
              1e-12);
}

TEST(LocalizationTest, NonPositiveDefiniteCovarianceInvalidatesObservation) {
  RobotPoseEstimate estimate;
  auto normal = detail::PoseNormalMatrix::Identity().eval();
  normal(0, 0) = -1;
  EXPECT_FALSE(detail::SetCovarianceStdDevs(normal, {}, &estimate));
  EXPECT_FALSE(estimate.validity.valid);
  EXPECT_EQ(estimate.validity.reason, PoseInvalidReason::kInvalidCovariance);
  normal(0, 0) = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(detail::SetCovarianceStdDevs(normal, {}, &estimate));

  JointFixture fixture(3, 1);
  fixture.mount = CameraMounting::Identity();
  fixture.layout.SetTagSize(1, 0);
  fixture.tags[0].T_camera_tag = SolverTagFrameToLayoutFrame(
      JointFixture::Pose(talos::geometry::SO3<double>::Identity(), {0, 0, 3}));
  fixture.layout.SetTagPose(
      1, fixture.mount.T_robot_camera * fixture.tags[0].T_camera_tag);
  for (size_t i = 0; i < 4; ++i) {
    fixture.tags[0].corners[2 * i] = static_cast<float>(fixture.intr.cx);
    fixture.tags[0].corners[2 * i + 1] = static_cast<float>(fixture.intr.cy);
  }
  const auto singular = fixture.Solve();
  EXPECT_FALSE(singular.validity.valid);
  EXPECT_EQ(singular.validity.reason, PoseInvalidReason::kInvalidCovariance);
}

TEST(LocalizationTest, MissingSecondPoseDistinguishesMultipleMinimaFromUnique) {
  CameraIntrinsics intr;
  intr.fx = 700;
  intr.fy = 700;
  intr.cx = 640;
  intr.cy = 360;
  // Pinned upstream ambiguity-search cases, including the float conversion
  // and DLT recomputation used by the production pipeline.
  const std::array<std::array<float, 8>, 2> fixtures = {{
      {488.490112f, 242.771851f, 497.506744f, 250.814423f, 504.358826f,
       243.886002f, 498.08548f, 236.256561f},
      {941.858398f, 566.63446f, 945.342529f, 666.266479f, 1055.43237f,
       680.513306f, 1049.01001f, 574.729675f},
  }};
  for (size_t i = 0; i < fixtures.size(); ++i) {
    TagDetection3D tag;
    tag.tag_id = 1;
    tag.valid = true;
    tag.corners = fixtures[i];
    ASSERT_TRUE(EstimateTagPoseFromHomography(
        {}, intr, 0.1651, &tag.T_camera_tag, &tag.secondary_T_camera_tag,
        &tag.reprojection_error, &tag.secondary_reprojection_error_px,
        &tag.ambiguity, &tag.corners));
    EXPECT_TRUE(std::isinf(tag.secondary_reprojection_error_px));
    EXPECT_DOUBLE_EQ(tag.ambiguity, i == 0 ? 1.0 : 0.0);
    const auto mount = CameraMounting::Identity();
    FieldLayout layout;
    layout.SetTagPose(1, mount.T_robot_camera * tag.T_camera_tag);
    const auto result = SolveMultiTagRobotPose({tag}, 0, mount, layout, intr);
    if (i == 0) {
      EXPECT_FALSE(result.validity.valid);
      EXPECT_EQ(result.validity.reason, PoseInvalidReason::kHighAmbiguity);
      EXPECT_EQ(result.tag_count, 1);
      EXPECT_TRUE(IsSingleTagAmbiguityRejection(result));
      std::vector<TagDetection3D> detections{tag};
      AnnotateAmbiguityRejection(detections, result);
      EXPECT_EQ(detections[0].reject_reason, "ambiguity");
    } else {
      EXPECT_TRUE(result.validity.valid) << result.validity.message;
    }
  }
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
  PoseSolverOptions uncertain_floor;
  uncertain_floor.floor_z_std_m = 1;
  const auto unresolved = ComputeRobotFieldPose(
      1, 0, bad, mount, layout, 0.1, 0.9, {}, &good, 0.2, uncertain_floor);
  EXPECT_FALSE(unresolved.validity.valid);
  EXPECT_EQ(unresolved.validity.reason, PoseInvalidReason::kHighAmbiguity);
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
  EXPECT_GE(result.std_devs.x_std_m, 0.02);
  EXPECT_GE(result.std_devs.y_std_m, 0.02);
  EXPECT_GE(result.std_devs.yaw_std_rad, PoseSolverOptions{}.min_yaw_std_rad);
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
  EXPECT_NEAR(shifted.std_devs.x_std_m, result.std_devs.x_std_m, 1e-8);
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
  const auto corners = GetTagModelCorners(layout.GetTagSize(1));
  for (size_t j = 0; j < corners.size(); ++j) {
    Eigen::Vector2d uv;
    ASSERT_TRUE(intr.ProjectPoint(good.T_camera_tag.act(corners[j]), &uv));
    good.corners[2 * j] = static_cast<float>(uv.x());
    good.corners[2 * j + 1] = static_cast<float>(uv.y());
  }
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

TEST(LocalizationTest, RepeatedTagIdKeepsTheDetectionConsistentWithTheFrame) {
  // A printed copy of tag 1 elsewhere in view decodes as a second tag 1 with
  // its own consistent single-tag pose. Beside another tag, the copy must lose
  // even with the stronger decode; a lone id has nothing to contradict either
  // detection, so the stronger decode, here the real tag, is kept.
  struct Case {
    int tag_count;
    float decoy_margin;
  };
  for (const Case c : {Case{2, 120}, Case{1, 40}}) {
    SCOPED_TRACE(testing::Message() << c.tag_count << " tags, decoy margin "
                                    << c.decoy_margin << " vs 80");
    JointFixture fixture(3, c.tag_count);
    for (auto& tag : fixture.tags) tag.decision_margin = 80;
    auto decoy = fixture.tags.front();
    decoy.decision_margin = c.decoy_margin;
    decoy.T_camera_tag = SolverTagFrameToLayoutFrame(JointFixture::Pose(
        talos::geometry::SO3<double>::Exp({0.1, 0.2, 0.05}), {0.6, 0.1, 2.6}));
    const auto corners = GetTagModelCorners(fixture.layout.GetTagSize(1));
    for (size_t j = 0; j < corners.size(); ++j) {
      Eigen::Vector2d uv;
      ASSERT_TRUE(
          fixture.intr.ProjectPoint(decoy.T_camera_tag.act(corners[j]), &uv));
      decoy.corners[2 * j] = static_cast<float>(uv.x());
      decoy.corners[2 * j + 1] = static_cast<float>(uv.y());
    }
    fixture.tags.insert(fixture.tags.begin(), decoy);
    const auto result = fixture.Solve();
    ASSERT_TRUE(result.validity.valid) << result.validity.message;
    EXPECT_EQ(result.tag_count, c.tag_count);
    EXPECT_TRUE(
        result.T_field_robot.matrix().isApprox(fixture.robot.matrix(), 1e-4));
  }
}

void ExpectReplayedSingleTagRejected(const std::array<float, 8>& corners,
                                     const talos::geometry::SE3<double>& truth,
                                     double expected_range,
                                     double expected_ambiguity) {
  using Pose = talos::geometry::SE3<double>;
  CameraIntrinsics intr;
  intr.fx = intr.fy = 910.5;
  intr.cx = 640;
  intr.cy = 360;
  const auto mount = CameraMounting::FromExtrinsics(
      -0.25, 0.2, 0.45, 0, 0.2617993877991494, 2.356194490192345);
  const Pose optical_to_mount(
      CameraMounting::OpticalToRobotRotation().inverse(),
      Eigen::Vector3d::Zero());
  const auto delta = CameraMounting::FromExtrinsics(
      0.01, 0.01, -0.01, 0, 0.008726646259971648, -0.017453292519943295);
  const auto true_mount =
      mount.T_robot_camera * optical_to_mount * delta.T_robot_camera;
  FieldLayout layout;
  const Pose field_tag(talos::geometry::SO3<double>::Identity(),
                       {0.0080772, 4.1775126, 0.55245});
  layout.SetTagPose(32, field_tag);
  TagDetection3D tag;
  tag.tag_id = 32;
  tag.valid = true;
  tag.corners = corners;
  const auto actual_camera_tag =
      true_mount.inverse() * truth.inverse() * field_tag;
  const auto model = GetTagModelCorners(layout.GetTagSize(32));
  double corner_squared_error = 0;
  for (size_t i = 0; i < model.size(); ++i) {
    Eigen::Vector2d expected;
    ASSERT_TRUE(intr.ProjectPoint(actual_camera_tag.act(model[i]), &expected));
    corner_squared_error +=
        (expected - Eigen::Vector2d(tag.corners[2 * i], tag.corners[2 * i + 1]))
            .squaredNorm();
  }
  EXPECT_LT(std::sqrt(corner_squared_error / 8), 1.0);
  ASSERT_TRUE(EstimateTagPoseFromHomography(
      {}, intr, layout.GetTagSize(32), &tag.T_camera_tag,
      &tag.secondary_T_camera_tag, &tag.reprojection_error,
      &tag.secondary_reprojection_error_px, &tag.ambiguity, &tag.corners));
  EXPECT_NEAR(tag.T_camera_tag.translation().norm(), expected_range, 0.001);
  EXPECT_NEAR(tag.ambiguity, expected_ambiguity, 0.001);
  EXPECT_TRUE(std::isfinite(tag.secondary_reprojection_error_px));
  const auto primary =
      field_tag * tag.T_camera_tag.inverse() * mount.T_robot_camera.inverse();
  EXPECT_LT(FloorSquaredMahalanobis(primary, {}), 36);
  const auto seed = ComputeRobotFieldPose(
      tag.tag_id, 2, tag.T_camera_tag, mount, layout, tag.reprojection_error,
      tag.ambiguity, {}, &tag.secondary_T_camera_tag,
      tag.secondary_reprojection_error_px);
  EXPECT_TRUE(seed.T_camera_tag.matrix().isApprox(tag.T_camera_tag.matrix()));
  const auto estimate = SolveMultiTagRobotPose({tag}, 2, mount, layout, intr);
  EXPECT_FALSE(estimate.validity.valid);
  EXPECT_EQ(estimate.validity.reason, PoseInvalidReason::kHighAmbiguity);
}

TEST(LocalizationTest, UnresolvedFloorCannotChooseReplayedFlippedMinimum) {
  // Sample 3798897563162458: the old floor heuristic chose the flipped
  // secondary, despite not declaring its floor comparison decisive.
  ExpectReplayedSingleTagRejected(
      {15.7997456f, 94.9962006f, 69.897522f, 95.0006866f, 62.4674149f,
       46.517746f, 7.18865633f, 44.5019646f},
      {talos::geometry::SO3<double>::Exp({0, 0, 0.006735016726192925}),
       {4.094207414502776, 3.0702794816736043, 0}},
      3.93883, 0.413699);
}

TEST(LocalizationTest, FlippedPrimaryWithinPixelNoiseFailsAmbiguityGate) {
  // Sample 3799483805968666: the flipped primary fit was 0.0821 px versus
  // 0.2969 px for the correct secondary, both below the measured pixel noise.
  ExpectReplayedSingleTagRejected(
      {88.4926682f, 94.6360931f, 141.462875f, 95.9015961f, 134.494308f,
       45.4809265f, 80.6816025f, 43.5307922f},
      {talos::geometry::SO3<double>::Exp({0, 0, 0.014802429336405042}),
       {3.8460874563979917, 2.947888150751131, 0}},
      3.72735, 0.276650);
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

TEST(LocalizationTest, MultiTagSeedIgnoresFlippedPrimaryCandidate) {
  // Near the planar-ambiguity boundary a single tag's primary can be the wrong
  // minimum. The joint seed selection scores every candidate (primary and
  // secondary) by the joint residual over all tags' corners, so a flipped
  // primary must not steer the solve away from the true pose.
  JointFixture fixture(3, 2);
  const auto correct = fixture.tags[0].T_camera_tag;
  fixture.tags[0].T_camera_tag = SolverTagFrameToLayoutFrame(JointFixture::Pose(
      talos::geometry::SO3<double>::Identity(), {1.0, 1.0, 3.0}));
  fixture.tags[0].secondary_T_camera_tag = correct;
  fixture.tags[0].secondary_reprojection_error_px = 0.5;
  for (auto& tag : fixture.tags) tag.ambiguity = 0.65;
  const auto result = fixture.Solve();
  ASSERT_TRUE(result.validity.valid) << result.validity.message;
  EXPECT_EQ(result.tag_count, 2);
  EXPECT_TRUE(
      result.T_field_robot.matrix().isApprox(fixture.robot.matrix(), 1e-4));
}

TEST(LocalizationTest, AmbiguityAnnotationIsSingleTagOnly) {
  // The per-detection annotation no longer applies the ambiguity gate; only a
  // single-tag observation actually rejected for it carries the "ambiguity"
  // reason, and only on that lone detection.
  FieldLayout layout;
  layout.SetTagPose(
      1, talos::geometry::SE3<double>(talos::geometry::SO3<double>::Identity(),
                                      Eigen::Vector3d(4.0, 0.0, 1.0)));
  layout.SetTagPose(
      2, talos::geometry::SE3<double>(talos::geometry::SO3<double>::Identity(),
                                      Eigen::Vector3d(4.0, 1.0, 1.0)));
  TagDetection3D ambiguous, clear;
  ambiguous.tag_id = 1;
  ambiguous.valid = true;
  ambiguous.T_camera_tag = talos::geometry::SE3<double>(
      talos::geometry::SO3<double>::Identity(), Eigen::Vector3d(0.0, 0.0, 2.0));
  ambiguous.ambiguity = 0.8;
  clear = ambiguous;
  clear.tag_id = 2;
  clear.ambiguity = 0.1;

  std::vector<TagDetection3D> detections{ambiguous, clear};
  AnnotateDetectionReasons(detections, layout, 8.0, {});
  EXPECT_TRUE(detections[0].reject_reason.empty());
  EXPECT_TRUE(detections[1].reject_reason.empty());

  RobotPoseEstimate single;
  single.tag_count = 1;
  single.tag_id = 1;
  single.validity = {false, PoseInvalidReason::kHighAmbiguity,
                     "Ambiguity exceeds threshold"};
  AnnotateAmbiguityRejection(detections, single);
  EXPECT_EQ(detections[0].reject_reason, "ambiguity");
  EXPECT_TRUE(detections[1].reject_reason.empty());

  RobotPoseEstimate multi;
  multi.tag_count = 2;
  multi.tag_id = -1;
  multi.validity = {true, PoseInvalidReason::kValid, "Valid"};
  AnnotateAmbiguityRejection(detections, multi);
  EXPECT_EQ(detections[0].reject_reason, "ambiguity");
  EXPECT_TRUE(detections[1].reject_reason.empty());
}

}  // namespace
}  // namespace talos::vision
