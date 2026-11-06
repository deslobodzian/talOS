#pragma once

// SO(2): planar rotations. Angle storage keeps composition a single add;
// rotationMatrix() materializes the 2x2 for batch point transforms.

#include <Eigen/Dense>
#include <cmath>

namespace talos::geometry {

template <typename Scalar = double>
class SO2 {
 public:
  using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
  using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;

  SO2() : theta_(Scalar(0)) {}
  explicit SO2(Scalar theta) : theta_(Wrap(theta)) {}
  SO2(Scalar cos_theta, Scalar sin_theta)
      : SO2(std::atan2(sin_theta, cos_theta)) {}

  static SO2 Identity() { return SO2(Scalar(0)); }
  static SO2 FromAngle(Scalar theta) { return SO2(theta); }
  static SO2 FromCosSin(Scalar cos_theta, Scalar sin_theta) {
    return SO2(cos_theta, sin_theta);
  }

  // Exponential map: R = [cos w, -sin w; sin w, cos w]. Exact.
  static SO2 Exp(Scalar w) { return SO2(w); }
  // Logarithmic map. Exact up to angle wrapping.
  Scalar Log() const { return theta_; }

  Scalar angle() const { return theta_; }
  Scalar cos() const { return std::cos(theta_); }
  Scalar sin() const { return std::sin(theta_); }
  Matrix2 rotationMatrix() const {
    const Scalar c = std::cos(theta_);
    const Scalar s = std::sin(theta_);
    Matrix2 r;
    r << c, -s, s, c;
    return r;
  }

  SO2 operator*(const SO2& other) const { return SO2(theta_ + other.theta_); }
  SO2 inverse() const { return SO2(-theta_); }
  Vector2 act(const Vector2& v) const { return rotationMatrix() * v; }

  // Right perturbation: this (+) dx = this * Exp(dx). Exact.
  SO2 boxplus(Scalar dx) const { return *this * Exp(dx); }
  // dx such that other.boxplus(dx) == *this. Exact up to wrapping.
  Scalar boxminus(const SO2& other) const {
    return (other.inverse() * *this).Log();
  }

  // Right Jacobian of Exp is the 1x1 identity.
  static Scalar RightJacobianOfExp(Scalar /*w*/) { return Scalar(1); }

  // Adjoint of SO(2) is the identity.
  static Scalar Adjoint(const SO2& /*r*/) { return Scalar(1); }

  // Already principal-valued.
  SO2 wrapped() const { return *this; }

 private:
  static constexpr double kPi = 3.14159265358979323846;
  static Scalar Wrap(Scalar theta) {
    const Scalar wrapped = std::atan2(std::sin(theta), std::cos(theta));
    return wrapped <= -Scalar(kPi) ? Scalar(kPi) : wrapped;
  }
  Scalar theta_;
};

}  // namespace talos::geometry
