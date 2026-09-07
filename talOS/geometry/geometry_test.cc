// Tests for talOS/geometry. Three independent proof lines per group:
// group axioms on random inputs, exp/log round-trips including the
// small-angle and near-pi regimes, and cross-checks against independent
// Eigen paths (AngleAxis, Rotation2D, 4x4 homogeneous composition).
// Analytic Jacobians are checked against central finite differences.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <random>

#include "talOS/geometry/lie.h"

namespace talos::geometry {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTight = 1e-12;  // closed-form round-trips
constexpr double kFD = 1e-6;      // finite-difference agreement

std::mt19937_64 Rng() { return std::mt19937_64{0xC10C}; }

Eigen::Vector3d RandomAxis(std::mt19937_64& rng) {
  std::normal_distribution<double> n;
  Eigen::Vector3d a(n(rng), n(rng), n(rng));
  return a.normalized();
}

double RandomAngle(std::mt19937_64& rng, double lo = -kPi + 0.1,
                   double hi = kPi - 0.1) {
  return std::uniform_real_distribution<double>(lo, hi)(rng);
}

SO3<double> RandomSO3(std::mt19937_64& rng) {
  return SO3<double>::Exp(RandomAngle(rng) * RandomAxis(rng));
}

SE3<double> RandomSE3(std::mt19937_64& rng) {
  std::normal_distribution<double> n;
  Eigen::Matrix<double, 6, 1> tau;
  tau.head<3>() = RandomAngle(rng) * RandomAxis(rng);
  tau.tail<3>() = Eigen::Vector3d(n(rng), n(rng), n(rng));
  return SE3<double>::Exp(tau);
}

TEST(SO2, Axioms) {
  auto rng = Rng();
  std::uniform_real_distribution<double> u(-10, 10);
  for (int i = 0; i < 1000; ++i) {
    const SO2<double> a(u(rng)), b(u(rng)), c(u(rng));
    EXPECT_NEAR(((a * b) * c).angle(), (a * (b * c)).angle(), 1e-9);
    EXPECT_NEAR((a * SO2<double>::Identity()).angle(), a.angle(), 1e-12);
    EXPECT_NEAR((a * a.inverse()).angle(), 0.0, 1e-9);
    EXPECT_NEAR((a.boxplus(0.3)).boxminus(a), 0.3, 1e-12);
  }
}

TEST(SO2, MatchesEigenRotation2D) {
  auto rng = Rng();
  std::uniform_real_distribution<double> u(-kPi, kPi);
  for (int i = 0; i < 200; ++i) {
    const double t = u(rng);
    EXPECT_TRUE(SO2<double>(t).rotationMatrix().isApprox(
        Eigen::Rotation2Dd(t).matrix(), 1e-12));
    EXPECT_NEAR(SO2<double>::Exp(t).Log(), t, 1e-12);
  }
}

TEST(SE2, RoundTripAndOracle) {
  auto rng = Rng();
  std::uniform_real_distribution<double> u(-3, 3);
  for (int i = 0; i < 500; ++i) {
    Eigen::Vector3d tau(u(rng), u(rng), u(rng));
    const SE2<double> x = SE2<double>::Exp(tau);
    EXPECT_TRUE(x.Log().isApprox(tau, kTight)) << tau.transpose();
    // Inverse and composition against 3x3 homogeneous math.
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    m.block<2, 2>(0, 0) = x.rotation().rotationMatrix();
    m.block<2, 1>(0, 2) = x.translation();
    const SE2<double> y =
        SE2<double>::Exp(Eigen::Vector3d(u(rng), u(rng), u(rng)));
    Eigen::Matrix3d n = Eigen::Matrix3d::Identity();
    n.block<2, 2>(0, 0) = y.rotation().rotationMatrix();
    n.block<2, 1>(0, 2) = y.translation();
    const SE2<double> xy = x * y;
    EXPECT_TRUE(xy.rotation().rotationMatrix().isApprox(
        (m * n).block<2, 2>(0, 0), 1e-12));
    EXPECT_TRUE(xy.translation().isApprox((m * n).block<2, 1>(0, 2), 1e-12));
    EXPECT_TRUE((x * x.inverse()).translation().norm() < 1e-12);
  }
  // Origin: exact, no branches taken badly.
  EXPECT_TRUE(
      SE2<double>::Exp(Eigen::Vector3d::Zero()).translation().isZero(1e-15));
}

TEST(SO3, Axioms) {
  auto rng = Rng();
  for (int i = 0; i < 500; ++i) {
    const SO3<double> a = RandomSO3(rng), b = RandomSO3(rng),
                      c = RandomSO3(rng);
    EXPECT_TRUE(((a * b) * c)
                    .rotationMatrix()
                    .isApprox((a * (b * c)).rotationMatrix(), 1e-9));
    EXPECT_TRUE((a * a.inverse())
                    .rotationMatrix()
                    .isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_TRUE(a.boxplus(Eigen::Vector3d(0.1, -0.2, 0.05))
                    .boxminus(a)
                    .isApprox(Eigen::Vector3d(0.1, -0.2, 0.05), 1e-9));
  }
}

TEST(SO3, ExpMatchesAngleAxis) {
  auto rng = Rng();
  for (int i = 0; i < 500; ++i) {
    const Eigen::Vector3d w = RandomAngle(rng) * RandomAxis(rng);
    const double theta = w.norm();
    Eigen::Matrix3d ref =
        Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
    EXPECT_TRUE(SO3<double>::Exp(w).rotationMatrix().isApprox(ref, 1e-12));
  }
  // Golden: 90 degrees about z.
  Eigen::Matrix3d rz90;
  rz90 << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  EXPECT_TRUE(SO3<double>::Exp(Eigen::Vector3d(0, 0, kPi / 2))
                  .rotationMatrix()
                  .isApprox(rz90, 1e-15));
}

TEST(SO3, LogRoundTripEverywhere) {
  auto rng = Rng();
  // Generic angles.
  for (int i = 0; i < 500; ++i) {
    const Eigen::Vector3d w = RandomAngle(rng) * RandomAxis(rng);
    EXPECT_TRUE(SO3<double>::Exp(w).Log().isApprox(w, 1e-9)) << w.transpose();
  }
  // Tiny angles exercise the Taylor branch (down to 1e-12).
  for (const double s : {1e-4, 1e-6, 1e-8, 1e-10, 1e-12}) {
    const Eigen::Vector3d w = s * RandomAxis(rng);
    EXPECT_TRUE(SO3<double>::Exp(w).Log().isApprox(w, 1e-9)) << s;
  }
  // Near pi exercises the largest-diagonal fallback.
  for (const double e : {1e-2, 1e-4, 1e-6}) {
    const Eigen::Vector3d w = (kPi - e) * RandomAxis(rng);
    const Eigen::Vector3d back = SO3<double>::Exp(w).Log();
    EXPECT_NEAR(back.norm(), kPi - e, 1e-6) << e;
    // Same rotation: Log(Exp(w)) reproduces R, the only exact claim at
    // the branch cut.
    EXPECT_TRUE(SO3<double>::Exp(back).rotationMatrix().isApprox(
        SO3<double>::Exp(w).rotationMatrix(), 1e-6));
  }
  EXPECT_TRUE(SO3<double>::Identity().Log().isZero(1e-15));
}

TEST(SO3, AdjointIdentity) {
  // X * Exp(d) == Exp(Ad_X d) * X.
  auto rng = Rng();
  for (int i = 0; i < 200; ++i) {
    const SO3<double> x = RandomSO3(rng);
    const Eigen::Vector3d d =
        0.01 * RandomAxis(rng) * RandomAngle(rng, 0.0, 1.0);
    EXPECT_TRUE(
        (x * SO3<double>::Exp(d))
            .rotationMatrix()
            .isApprox((SO3<double>::Exp(x.adjoint() * d) * x).rotationMatrix(),
                      1e-12));
  }
}

TEST(SO3, RightJacobianMatchesFiniteDifferences) {
  auto rng = Rng();
  const double h = 1e-8;
  for (int i = 0; i < 100; ++i) {
    const Eigen::Vector3d w = RandomAngle(rng) * RandomAxis(rng);
    const Eigen::Matrix3d jr = SO3<double>::RightJacobianOfExp(w);
    Eigen::Matrix3d fd;
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d dw = Eigen::Vector3d::Zero();
      dw(k) = h;
      fd.col(k) = (SO3<double>::Exp(w + dw).boxminus(SO3<double>::Exp(w))) / h;
    }
    EXPECT_TRUE(jr.isApprox(fd, kFD));
  }
}

TEST(SE3, Axioms) {
  auto rng = Rng();
  for (int i = 0; i < 500; ++i) {
    const SE3<double> a = RandomSE3(rng), b = RandomSE3(rng),
                      c = RandomSE3(rng);
    EXPECT_TRUE(((a * b) * c).matrix().isApprox((a * (b * c)).matrix(), 1e-9));
    EXPECT_TRUE(
        (a * a.inverse()).matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-9));
  }
}

TEST(SE3, ExpLogRoundTrip) {
  auto rng = Rng();
  std::normal_distribution<double> n;
  for (int i = 0; i < 500; ++i) {
    Eigen::Matrix<double, 6, 1> tau;
    tau.head<3>() = RandomAngle(rng) * RandomAxis(rng);
    tau.tail<3>() = Eigen::Vector3d(n(rng), n(rng), n(rng));
    EXPECT_TRUE(SE3<double>::Exp(tau).Log().isApprox(tau, 1e-9));
  }
  // Tiny and near-pi rotation parts.
  for (const double s : {1e-6, 1e-10}) {
    Eigen::Matrix<double, 6, 1> tau;
    tau.head<3>() = s * RandomAxis(rng);
    tau.tail<3>() = Eigen::Vector3d(0.1, -0.2, 0.3);
    EXPECT_TRUE(SE3<double>::Exp(tau).Log().isApprox(tau, 1e-9)) << s;
  }
  {
    Eigen::Matrix<double, 6, 1> tau;
    tau.head<3>() = (kPi - 1e-4) * RandomAxis(rng);
    tau.tail<3>() = Eigen::Vector3d(1, 2, 3);
    const SE3<double> x = SE3<double>::Exp(tau);
    EXPECT_TRUE(SE3<double>::Exp(x.Log()).matrix().isApprox(x.matrix(), 1e-6));
  }
  // boxplus/boxminus round-trip.
  for (int i = 0; i < 200; ++i) {
    const SE3<double> a = RandomSE3(rng);
    Eigen::Matrix<double, 6, 1> dx;
    dx << 0.01, -0.02, 0.03, 0.1, 0.2, -0.1;
    EXPECT_TRUE(a.boxplus(dx).boxminus(a).isApprox(dx, 1e-9));
  }
}

TEST(SE3, MatchesHomogeneousMatrices) {
  // Composition and action against plain 4x4 math (no shared formulas).
  auto rng = Rng();
  for (int i = 0; i < 300; ++i) {
    const SE3<double> a = RandomSE3(rng), b = RandomSE3(rng);
    EXPECT_TRUE((a * b).matrix().isApprox(a.matrix() * b.matrix(), 1e-9));
    const Eigen::Vector3d p(0.5, -1.5, 2.5);
    Eigen::Vector4d ph;
    ph << p, 1.0;
    EXPECT_TRUE(a.act(p).isApprox((a.matrix() * ph).head<3>(), 1e-9));
  }
}

TEST(SE3, RightJacobianMatchesFiniteDifferences) {
  // Series domain: optimizer-scale steps. Truncation is O(|tau|^4).
  const double h = 1e-8;
  for (int i = 0; i < 50; ++i) {
    Eigen::Matrix<double, 6, 1> tau =
        0.01 * Eigen::Matrix<double, 6, 1>::Random();
    const Eigen::Matrix<double, 6, 6> jr = SE3<double>::RightJacobianOfExp(tau);
    Eigen::Matrix<double, 6, 6> fd;
    for (int k = 0; k < 6; ++k) {
      Eigen::Matrix<double, 6, 1> dtau = Eigen::Matrix<double, 6, 1>::Zero();
      dtau(k) = h;
      fd.col(k) =
          (SE3<double>::Exp(tau + dtau).boxminus(SE3<double>::Exp(tau))) / h;
    }
    EXPECT_TRUE(jr.isApprox(fd, kFD));
  }
}

}  // namespace
}  // namespace talos::geometry
