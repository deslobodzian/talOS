#pragma once

// SE(3): spatial rigid motions. Stores rotation + translation separately so
// composition costs a 3x3 product plus one action instead of a 4x4 product.
// Exp/Log use the closed-form V matrix (no 4x4 inverse anywhere).
//
// RightJacobianOfExp uses a 3rd-order BCH series on the 6x6 adjoint,
// error O(|tau|^4): direction-exact for optimizer steps, where |tau| << 1.
// For the exact rotation-only Jacobian see SO3::RightJacobianOfExp.

#include <Eigen/Dense>
#include <cmath>

#include "talOS/geometry/so3.h"

namespace talos::geometry {

template <typename Scalar = double>
class SE3 {
 public:
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vector6 = Eigen::Matrix<Scalar, 6, 1>;  // tangent (omega, v)
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
  using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;

  SE3() : r_(), t_(Vector3::Zero()) {}
  SE3(const SO3<Scalar>& r, const Vector3& t) : r_(r), t_(t) {}

  static SE3 Identity() { return SE3(); }

  // Exponential map, exact. t = V v with
  // V = I + a W + b W^2, a = (1-cos t)/t^2, b = (t-sin t)/t^3.
  static SE3 Exp(const Vector6& tau) {
    const Vector3 w = tau.template head<3>();
    const Vector3 v = tau.template tail<3>();
    const SO3<Scalar> r = SO3<Scalar>::Exp(w);
    return SE3(r, LeftV(w) * v);
  }

  // Logarithmic map, exact. v = V^-1 t with
  // V^-1 = I - W/2 + c W^2, c = (1 - (t/2) / tan(t/2)) / t^2.
  Vector6 Log() const {
    const Vector3 w = r_.Log();
    Vector6 tau;
    tau.template head<3>() = w;
    tau.template tail<3>() = LeftVInverse(w) * t_;
    return tau;
  }

  const SO3<Scalar>& rotation() const { return r_; }
  const Vector3& translation() const { return t_; }
  Matrix4 matrix() const {
    Matrix4 m = Matrix4::Identity();
    m.template block<3, 3>(0, 0) = r_.rotationMatrix();
    m.template block<3, 1>(0, 3) = t_;
    return m;
  }

  SE3 operator*(const SE3& other) const {
    return SE3(r_ * other.r_, t_ + r_.act(other.t_));
  }
  SE3 inverse() const {
    const SO3<Scalar> rinv = r_.inverse();
    return SE3(rinv, rinv.act(-t_));
  }
  Vector3 act(const Vector3& v) const { return r_.act(v) + t_; }

  // Right perturbation: this (+) dx = this * Exp(dx). Exact.
  SE3 boxplus(const Vector6& dx) const { return *this * Exp(dx); }
  // dx such that other.boxplus(dx) == *this.
  Vector6 boxminus(const SE3& other) const {
    return (other.inverse() * *this).Log();
  }

  // Adjoint: [R 0; [t]^ R, R] in (omega, v) ordering.
  Matrix6 adjoint() const {
    Matrix6 ad = Matrix6::Zero();
    const Matrix3 r = r_.rotationMatrix();
    ad.template block<3, 3>(0, 0) = r;
    ad.template block<3, 3>(3, 3) = r;
    ad.template block<3, 3>(3, 0) = SO3<Scalar>::Hat(t_) * r;
    return ad;
  }

  // Right Jacobian of Exp, 3rd-order BCH series on the adjoint:
  // Jr = I - A/2 + A^2/6 - A^3/24, A = ad(tau). Three 6x6 products,
  // error O(|tau|^4); use with optimizer-scale steps.
  static Matrix6 RightJacobianOfExp(const Vector6& tau) {
    const Matrix6 a = Hat(tau);
    const Matrix6 a2 = a * a;
    return Matrix6::Identity() - a / Scalar(2) + a2 / Scalar(6) -
           (a2 * a) / Scalar(24);
  }

  // Lie algebra adjoint hat: (w, v) -> [[w]^, 0; [v]^, [w]^] (6x6 adjoint).
  static Matrix6 Hat(const Vector6& tau) {
    const Matrix3 w_hat = SO3<Scalar>::Hat(tau.template head<3>());
    const Matrix3 v_hat = SO3<Scalar>::Hat(tau.template tail<3>());
    Matrix6 h = Matrix6::Zero();
    h.template block<3, 3>(0, 0) = w_hat;
    h.template block<3, 3>(3, 3) = w_hat;
    h.template block<3, 3>(3, 0) = v_hat;
    return h;
  }

  // The V matrix of Exp (also useful to callers integrating velocity).
  // Equals SO3::RightJacobianOfExp(-w).
  static Matrix3 LeftV(const Vector3& w) {
    return SO3<Scalar>::RightJacobianOfExp(-w);
  }

  // Inverse of V, closed form. Equals SO3::RightJacobianInverse(-w).
  static Matrix3 LeftVInverse(const Vector3& w) {
    return SO3<Scalar>::RightJacobianInverse(-w);
  }

 private:
  SO3<Scalar> r_;
  Vector3 t_;
};

}  // namespace talos::geometry
