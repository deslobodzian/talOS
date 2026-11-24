// Property and numerical-edge tests for talOS/geometry Lie groups:
// float/double principal-angle boundaries, huge finite angles and vectors,
// small-angle cancellation thresholds, non-finite and singular inputs, and
// SE(3) adjoint / float round-trip invariants.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <random>

#include "talOS/geometry/lie.h"

namespace talos::geometry {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr float kPiF = static_cast<float>(kPi);

std::mt19937_64 Rng() { return std::mt19937_64{0x50C0FFEEULL}; }

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> RandomAxis(std::mt19937_64& rng) {
  std::normal_distribution<Scalar> n;
  Eigen::Matrix<Scalar, 3, 1> a(n(rng), n(rng), n(rng));
  return a.normalized();
}

TEST(SO2Property, FloatAndDoublePrincipalBoundariesAndHugeAngles) {
  // float(kPi) > exact pi: wrapping must keep +pi in (-pi, pi] rather than
  // flipping it to -pi + 1 ulp via atan2(sin(pi_f), cos(pi_f)).
  EXPECT_FLOAT_EQ(SO2<float>(kPiF).angle(), kPiF);
  EXPECT_FLOAT_EQ(SO2<float>(-kPiF).angle(), kPiF);
  EXPECT_FLOAT_EQ(SO2<float>::Exp(kPiF).Log(), kPiF);
  EXPECT_FLOAT_EQ(SO2<float>::FromCosSin(-1.0f, 0.0f).angle(), kPiF);
  EXPECT_FLOAT_EQ(SO2<float>::FromCosSin(-1.0f, -0.0f).angle(), kPiF);

  EXPECT_DOUBLE_EQ(SO2<double>(kPi).angle(), kPi);
  EXPECT_DOUBLE_EQ(SO2<double>(-kPi).angle(), kPi);
  EXPECT_DOUBLE_EQ(SO2<double>::FromCosSin(-1.0, 0.0).angle(), kPi);
  EXPECT_DOUBLE_EQ(SO2<double>::FromCosSin(-1.0, -0.0).angle(), kPi);

  for (const double angle :
       {1e6, -1e6, 1e12, -1e12, 1e17, -1e17, 1e100, -1e100, 1e308, -1e308}) {
    SCOPED_TRACE(angle);
    const SO2<double> r(angle);
    ASSERT_TRUE(std::isfinite(r.angle()));
    EXPECT_GT(r.angle(), -kPi);
    EXPECT_LE(r.angle(), kPi);
    const Eigen::Matrix2d m = r.rotationMatrix();
    EXPECT_TRUE(
        (m.transpose() * m).isApprox(Eigen::Matrix2d::Identity(), 1e-12));
    EXPECT_NEAR(m.determinant(), 1.0, 1e-12);
  }

  for (const float angle : {1e6f, -1e6f, 1e15f, -1e15f, 1e30f, -1e30f}) {
    SCOPED_TRACE(angle);
    const SO2<float> r(angle);
    ASSERT_TRUE(std::isfinite(r.angle()));
    EXPECT_GT(r.angle(), -kPiF);
    EXPECT_LE(r.angle(), kPiF);
  }

  // Non-finite inputs propagate NaN rather than fabricating a zero angle.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {nan, inf, -inf}) {
    EXPECT_TRUE(std::isnan(SO2<double>(bad).angle()));
  }
}

TEST(SE2Property, SmallAngleNoCancellationAndLargeAngleRightJacobian) {
  // Just above SE2::Exp's TaylorThreshold (eps^(1/4) ~ 1.22e-4 in double,
  // 1.86e-2 in float): half-angle m avoids 1 - cos(w) cancellation.
  for (const double w : {1.25e-4, 2.0e-4, 1.0e-3, -1.25e-4}) {
    SCOPED_TRACE(w);
    const SE2<double> pose = SE2<double>::Exp(Eigen::Vector3d(w, 0.0, 1.0));
    const double expected_x =
        -(w / 2.0) * (1.0 - (w * w) / 12.0 + (w * w * w * w) / 360.0);
    EXPECT_NEAR(pose.translation().x(), expected_x, 1e-15);
  }

  // RightJacobianOfExp stays accurate across the Taylor/trig transition.
  for (const double w : {1.25e-4, 2.0e-4, 1.0e-3, 0.049, 0.051, -0.051}) {
    SCOPED_TRACE(w);
    const Eigen::Vector3d tau(w, 0.7, -0.4);
    const Eigen::Matrix3d jr = SE2<double>::RightJacobianOfExp(tau);
    ASSERT_TRUE(jr.allFinite());
    const double w2 = w * w;
    const double a =
        0.5 - w2 / 24.0 + (w2 * w2) / 720.0 - (w2 * w2 * w2) / 40320.0;
    const double b =
        1.0 / 6.0 - w2 / 120.0 + (w2 * w2) / 5040.0 - (w2 * w2 * w2) / 362880.0;
    EXPECT_NEAR(jr(1, 0), -a * tau(2) + (b * w) * tau(1), 1e-11);
    EXPECT_NEAR(jr(2, 0), a * tau(1) + (b * w) * tau(2), 1e-11);
    EXPECT_NEAR(jr(1, 1), std::sin(w) / w, 1e-12);
    EXPECT_NEAR(jr(1, 2), (1.0 - std::cos(w)) / w, 1e-11);
  }

  // Huge |w| past DBL_MAX^(1/3) (~5.6e102) and sqrt(DBL_MAX) (~1.34e154) must
  // stay finite with bottom-right block -> 0 (not Identity from w^3 overflow).
  for (const double w : {1e110, -1e110, 1e200, -1e200, 1e308, -1e308}) {
    SCOPED_TRACE(w);
    const Eigen::Matrix3d jr =
        SE2<double>::RightJacobianOfExp(Eigen::Vector3d(w, 1.0, -2.0));
    ASSERT_TRUE(jr.allFinite()) << jr;
    EXPECT_DOUBLE_EQ(jr(0, 0), 1.0);
    EXPECT_NEAR(jr(1, 1), 0.0, 1e-12);
    EXPECT_NEAR(jr(2, 2), 0.0, 1e-12);
    EXPECT_NEAR(jr(1, 2), 0.0, 1e-12);
    EXPECT_NEAR(jr(2, 1), 0.0, 1e-12);
  }
  for (const float w : {1e13f, -1e13f, 1e20f, -1e20f, 1e30f}) {
    SCOPED_TRACE(w);
    const Eigen::Matrix3f jr =
        SE2<float>::RightJacobianOfExp(Eigen::Vector3f(w, 1.0f, -2.0f));
    ASSERT_TRUE(jr.allFinite()) << jr;
    EXPECT_NEAR(jr(1, 1), 0.0f, 1e-6f);
    EXPECT_NEAR(jr(2, 2), 0.0f, 1e-6f);
  }
}

TEST(SO3Property, FromMatrixNormalizedSafeOnNonFiniteSingularAndExtremeScale) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  // Non-finite matrices must not read uninitialized JacobiSVD U/V buffers.
  for (const double bad : {nan, inf, -inf}) {
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    m(1, 2) = bad;
    const SO3<double> out = SO3<double>::FromMatrixNormalized(m);
    ASSERT_TRUE(out.rotationMatrix().allFinite());
    EXPECT_EQ(out.rotationMatrix(), Eigen::Matrix3d::Identity());
  }

  // Zero, rank-1, rank-2, and reflection matrices project to proper SO(3).
  Eigen::Matrix3d rank1;
  rank1 << 1, 2, 3, 2, 4, 6, 3, 6, 9;
  Eigen::Matrix3d rank2 = Eigen::Matrix3d::Identity();
  rank2(2, 2) = 0.0;
  for (const Eigen::Matrix3d& singular :
       {Eigen::Matrix3d(Eigen::Matrix3d::Zero()), rank1, rank2,
        Eigen::Matrix3d(-Eigen::Matrix3d::Identity())}) {
    const Eigen::Matrix3d r =
        SO3<double>::FromMatrixNormalized(singular).rotationMatrix();
    ASSERT_TRUE(r.allFinite());
    EXPECT_TRUE(
        (r.transpose() * r).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
  }

  // Extreme-scale proper rotations recover the exact rotation.
  const Eigen::Matrix3d ref =
      SO3<double>::Exp(Eigen::Vector3d(0.3, -0.5, 0.2)).rotationMatrix();
  for (const double scale : {1e200, 1e-200}) {
    const Eigen::Matrix3d r =
        SO3<double>::FromMatrixNormalized(scale * ref).rotationMatrix();
    ASSERT_TRUE(r.allFinite());
    EXPECT_TRUE(r.isApprox(ref, 1e-12));
  }
}

TEST(SO3Property, LogAccurateNearPiInFloatAndFiniteOnDegenerateMatrices) {
  auto rng = Rng();
  // Float Log near pi: exact symmetric-column branch avoids O(eps/(pi-theta))
  // skew blowup below pi - 1e-5.
  for (const float eps_pi : {1e-2f, 1e-3f, 1e-4f, 2e-5f, 1e-5f, 1e-6f}) {
    SCOPED_TRACE(eps_pi);
    for (int i = 0; i < 25; ++i) {
      const Eigen::Vector3f w = (kPiF - eps_pi) * RandomAxis<float>(rng);
      const Eigen::Vector3f back = SO3<float>::Exp(w).Log();
      ASSERT_TRUE(back.allFinite());
      EXPECT_TRUE(SO3<float>::Exp(back).rotationMatrix().isApprox(
          SO3<float>::Exp(w).rotationMatrix(), 1e-5f));
    }
  }

  // Degenerate matrices with all diagonal entries <= -1 must not divide 0/0.
  for (const Eigen::Matrix3d& bad :
       {Eigen::Matrix3d(-Eigen::Matrix3d::Identity()),
        Eigen::Matrix3d(-2.0 * Eigen::Matrix3d::Identity()),
        Eigen::Matrix3d(Eigen::Matrix3d::Zero())}) {
    const Eigen::Vector3d w = SO3<double>(bad).Log();
    EXPECT_TRUE(w.allFinite()) << w.transpose();
  }

  // Non-finite rotation matrix propagates NaN cleanly.
  Eigen::Matrix3d nan_mat = Eigen::Matrix3d::Identity();
  nan_mat(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(SO3<double>(nan_mat).Log().allFinite());
}

TEST(SO3AndSE3Property, ExpJacobianAndLeftVHandleSmallAndHugeVectors) {
  auto rng = Rng();
  // Large finite w where squaredNorm() overflows: Exp stays orthogonal.
  for (const Eigen::Vector3d& w :
       {Eigen::Vector3d(1e200, -1e200, 1e200), Eigen::Vector3d(1e308, 0.0, 0.0),
        Eigen::Vector3d(0.0, -1e250, 1e250)}) {
    const Eigen::Matrix3d r = SO3<double>::Exp(w).rotationMatrix();
    ASSERT_TRUE(r.allFinite()) << r;
    EXPECT_TRUE(
        (r.transpose() * r).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
  }
  {
    const Eigen::Matrix3f r =
        SO3<float>::Exp(Eigen::Vector3f(1e20f, -1e20f, 1e20f)).rotationMatrix();
    ASSERT_TRUE(r.allFinite()) << r;
    EXPECT_TRUE(
        (r.transpose() * r).isApprox(Eigen::Matrix3f::Identity(), 1e-5f));
    EXPECT_NEAR(r.determinant(), 1.0f, 1e-5f);
  }

  // Small-angle transition around sqrt(eps) and 0.05: no cancellation spike.
  for (const double theta : {1.25e-4, 2e-4, 1e-3, 0.049, 0.051}) {
    SCOPED_TRACE(theta);
    const Eigen::Vector3d w = theta * RandomAxis<double>(rng);
    const Eigen::Matrix3d jr = SO3<double>::RightJacobianOfExp(w);
    const Eigen::Matrix3d jr_inv = SO3<double>::RightJacobianInverse(w);
    const Eigen::Matrix3d lv = SE3<double>::LeftV(w);
    const Eigen::Matrix3d lv_inv = SE3<double>::LeftVInverse(w);
    ASSERT_TRUE(jr.allFinite() && jr_inv.allFinite());
    EXPECT_TRUE((jr * jr_inv).isApprox(Eigen::Matrix3d::Identity(), 1e-13));
    EXPECT_TRUE((lv * lv_inv).isApprox(Eigen::Matrix3d::Identity(), 1e-13));
    EXPECT_EQ(lv, SO3<double>::RightJacobianOfExp(-w));
    EXPECT_EQ(lv_inv, SO3<double>::RightJacobianInverse(-w));
  }

  // Large finite w past DBL_MAX^(1/3) and sqrt(DBL_MAX): RightJacobianOfExp
  // and LeftV converge to u * u^T (I + u_hat^2), not Identity or NaN.
  for (const Eigen::Vector3d& w :
       {Eigen::Vector3d(1e110, 0.0, 0.0), Eigen::Vector3d(1e200, -1e200, 1e200),
        Eigen::Vector3d(1.5e308, 1.5e308, 0.0)}) {
    const Eigen::Vector3d u = (w / w.cwiseAbs().maxCoeff()).normalized();
    const Eigen::Matrix3d expected = u * u.transpose();
    const Eigen::Matrix3d jr = SO3<double>::RightJacobianOfExp(w);
    const Eigen::Matrix3d lv = SE3<double>::LeftV(w);
    ASSERT_TRUE(jr.allFinite() && lv.allFinite());
    EXPECT_TRUE(jr.isApprox(expected, 1e-12));
    EXPECT_TRUE(lv.isApprox(expected, 1e-12));
  }
  for (const Eigen::Vector3f& w : {Eigen::Vector3f(1e13f, 0.0f, 0.0f),
                                   Eigen::Vector3f(1e20f, -1e20f, 1e20f)}) {
    const Eigen::Vector3f u = (w / w.cwiseAbs().maxCoeff()).normalized();
    const Eigen::Matrix3f expected = u * u.transpose();
    EXPECT_TRUE(SO3<float>::RightJacobianOfExp(w).isApprox(expected, 1e-5f));
    EXPECT_TRUE(SE3<float>::LeftV(w).isApprox(expected, 1e-5f));
  }

  // RightJacobianInverse and LeftVInverse stay finite on huge finite w whose
  // squaredNorm overflows, and stay bit-identical under w -> -w.
  for (const Eigen::Vector3d& w : {Eigen::Vector3d(1e200, 0.0, 0.0),
                                   Eigen::Vector3d(1e155, -1e155, 0.0)}) {
    const Eigen::Matrix3d jr_inv = SO3<double>::RightJacobianInverse(w);
    const Eigen::Matrix3d lv_inv = SE3<double>::LeftVInverse(-w);
    ASSERT_TRUE(jr_inv.allFinite());
    EXPECT_EQ(jr_inv, lv_inv);
  }
}

TEST(SE3Property, AdjointIdentityAndFloatRoundTrip) {
  auto rng = Rng();
  std::normal_distribution<double> nd;
  std::uniform_real_distribution<double> angle(-kPi + 0.1, kPi - 0.1);
  for (int i = 0; i < 200; ++i) {
    Eigen::Matrix<double, 6, 1> tau, delta;
    tau.head<3>() = angle(rng) * RandomAxis<double>(rng);
    tau.tail<3>() = Eigen::Vector3d(nd(rng), nd(rng), nd(rng));
    delta.head<3>() = 0.05 * RandomAxis<double>(rng);
    delta.tail<3>() = 0.05 * Eigen::Vector3d(nd(rng), nd(rng), nd(rng));
    const SE3<double> x = SE3<double>::Exp(tau);
    const SE3<double> lhs = x * SE3<double>::Exp(delta);
    const SE3<double> rhs = SE3<double>::Exp(x.adjoint() * delta) * x;
    EXPECT_TRUE(lhs.matrix().isApprox(rhs.matrix(), 1e-11));
  }

  // SE3<float> round-trip across generic, tiny, and near-pi rotations.
  std::normal_distribution<float> nf;
  for (const float a : {1e-5f, 0.049f, 0.051f, 1.2f, kPiF - 1e-4f, kPiF}) {
    SCOPED_TRACE(a);
    Eigen::Matrix<float, 6, 1> tau;
    tau.head<3>() = a * RandomAxis<float>(rng);
    tau.tail<3>() = Eigen::Vector3f(nf(rng), nf(rng), nf(rng));
    const SE3<float> pose = SE3<float>::Exp(tau);
    const Eigen::Matrix<float, 6, 1> log_pose = pose.Log();
    ASSERT_TRUE(log_pose.allFinite());
    EXPECT_TRUE(
        SE3<float>::Exp(log_pose).matrix().isApprox(pose.matrix(), 1e-4f));
  }
}

}  // namespace
}  // namespace talos::geometry
