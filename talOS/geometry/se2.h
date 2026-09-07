#pragma once

// SE(2): planar rigid motions. Stores angle + translation; composition is
// one add plus a 2x2 action. The V matrix below is exact (division-free at
// the origin), so no Taylor branches are needed anywhere here.

#include <Eigen/Dense>
#include <cmath>

#include "talOS/geometry/so2.h"

namespace talos::geometry {

template <typename Scalar = double>
class SE2 {
 public:
  using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;  // tangent (omega, vx, vy)
  using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

  SE2() : theta_(Scalar(0)), t_(Vector2::Zero()) {}
  SE2(Scalar theta, const Vector2& t) : theta_(theta), t_(t) {}

  static SE2 Identity() { return SE2(); }
  static SE2 FromAngleAndTranslation(Scalar theta, const Vector2& t) {
    return SE2(theta, t);
  }

  // Exponential map, exact. V = sinc(w) I + m(w) J, J = [0 -1; 1 0],
  // sinc = sin w / w, m = (1 - cos w) / w; both extend smoothly to w = 0
  // (sinc -> 1, m -> 0), so this is division-free at the origin.
  static SE2 Exp(const Vector3& tau) {
    const Scalar w = tau(0);
    const Scalar aw = std::abs(w);
    Scalar sinc, m;
    if (aw < TaylorThreshold()) {
      const Scalar w2 = w * w;
      sinc = Scalar(1) - w2 / Scalar(6);
      m = w / Scalar(2) * (Scalar(1) - w2 / Scalar(12));
    } else {
      sinc = std::sin(w) / w;
      m = (Scalar(1) - std::cos(w)) / w;
    }
    Matrix2 v;
    v << sinc, -m, m, sinc;
    return SE2(w, v * tau.template tail<2>());
  }

  // Logarithmic map, exact: omega = theta, v = V^-1 t with
  // V^-1 = (sinc I - m J) / (sinc^2 + m^2), regular at the origin.
  Vector3 Log() const {
    const Scalar w = theta_;
    const Scalar aw = std::abs(w);
    Scalar sinc, m;
    if (aw < TaylorThreshold()) {
      const Scalar w2 = w * w;
      sinc = Scalar(1) - w2 / Scalar(6);
      m = w / Scalar(2) * (Scalar(1) - w2 / Scalar(12));
    } else {
      sinc = std::sin(w) / w;
      m = (Scalar(1) - std::cos(w)) / w;
    }
    const Scalar denom = sinc * sinc + m * m;
    Matrix2 vinv;
    vinv << sinc, m, -m, sinc;
    vinv /= denom;
    Vector3 tau;
    tau(0) = w;
    tau.template tail<2>() = vinv * t_;
    return tau;
  }

  Scalar angle() const { return theta_; }
  const Vector2& translation() const { return t_; }
  SO2<Scalar> rotation() const { return SO2<Scalar>(theta_); }

  SE2 operator*(const SE2& other) const {
    return SE2(theta_ + other.theta_, t_ + rotation().act(other.t_));
  }
  SE2 inverse() const {
    const SO2<Scalar> rinv = rotation().inverse();
    return SE2(-theta_, rinv.act(-t_));
  }
  Vector2 act(const Vector2& v) const { return rotation().act(v) + t_; }

  // Right perturbation: this (+) dx = this * Exp(dx). Exact.
  SE2 boxplus(const Vector3& dx) const { return *this * Exp(dx); }
  // dx such that other.boxplus(dx) == *this.
  Vector3 boxminus(const SE2& other) const {
    return (other.inverse() * *this).Log();
  }

  // Adjoint: [1 0; [t]^ R, R] in (omega, v) ordering.
  Matrix3 adjoint() const {
    Matrix3 ad = Matrix3::Zero();
    ad(0, 0) = Scalar(1);
    ad.template block<2, 2>(1, 1) = rotation().rotationMatrix();
    ad(1, 0) = -t_(1);
    ad(2, 0) = t_(0);
    return ad;
  }

  // Right Jacobian of Exp via 2nd-order BCH on the 3x3 adjoint:
  // Jr = I - A/2 + A^2/6, error O(|tau|^3). Adequate for optimizer steps;
  // the group operations above stay exact.
  static Matrix3 RightJacobianOfExp(const Vector3& tau) {
    const Matrix3 a = Hat(tau);
    return Matrix3::Identity() - a / Scalar(2) + (a * a) / Scalar(6);
  }

  // Lie algebra hat: (omega, vx, vy) -> [[0,-w,vx],[w,0,vy],[0,0,0]].
  static Matrix3 Hat(const Vector3& tau) {
    Matrix3 h = Matrix3::Zero();
    h(0, 1) = -tau(0);
    h(0, 2) = tau(1);
    h(1, 0) = tau(0);
    h(1, 2) = tau(2);
    return h;
  }

 private:
  // Below this |w| the Taylor remainders sit under machine epsilon.
  static Scalar TaylorThreshold() {
    using std::sqrt;
    return sqrt(Eigen::NumTraits<Scalar>::epsilon());
  }

  Scalar theta_;
  Vector2 t_;
};

}  // namespace talos::geometry
