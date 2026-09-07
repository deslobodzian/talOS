#pragma once

// SO(3): spatial rotations stored as a 3x3 matrix (fastest compose/act;
// re-orthogonalize by projecting through Exp(Log()) if a matrix drifts).
// Exp uses Rodrigues; Log uses the atan2 form with a largest-diagonal
// fallback at the pi singularity, so every rotation round-trips.

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace talos::geometry {

template <typename Scalar = double>
class SO3 {
 public:
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

  SO3() : r_(Matrix3::Identity()) {}
  explicit SO3(const Matrix3& r) : r_(r) {}

  static SO3 Identity() { return SO3(); }

  // Exponential map (Rodrigues). Taylor below |w| ~ sqrt(eps), where the
  // dropped O(|w|^3) terms sit under machine epsilon.
  static SO3 Exp(const Vector3& w) {
    const Scalar theta_sq = w.squaredNorm();
    const Scalar theta = std::sqrt(theta_sq);
    const Matrix3 w_hat = Hat(w);
    if (theta < TaylorThreshold()) {
      return SO3(Matrix3::Identity() + w_hat + Scalar(0.5) * w_hat * w_hat);
    }
    const Scalar a = std::sin(theta) / theta;
    const Scalar b = (Scalar(1) - std::cos(theta)) / theta_sq;
    return SO3(Matrix3::Identity() + a * w_hat + b * w_hat * w_hat);
  }

  // Logarithmic map. Exact away from the pi branch cut; at |angle| ~ pi
  // reconstructs the axis by the largest-diagonal method.
  Vector3 Log() const {
    const Scalar trace = r_.trace();
    Scalar cos_theta = (trace - Scalar(1)) / Scalar(2);
    cos_theta = std::max(Scalar(-1), std::min(Scalar(1), cos_theta));
    const Scalar theta = std::acos(cos_theta);
    if (theta < TaylorThreshold()) {
      // theta / (2 sin theta) = 1/2 (1 + theta^2/6 + ...).
      const Scalar k = Scalar(0.5) * (Scalar(1) + theta * theta / Scalar(6));
      return k * Vee(r_ - r_.transpose());
    }
    if (theta < Scalar(kPi) - Scalar(1e-6)) {
      return (theta / (Scalar(2) * std::sin(theta))) * Vee(r_ - r_.transpose());
    }
    // Near pi: R = 2 n n^T - I, so n_i^2 = (R_ii + 1) / 2.
    const Scalar xx = (r_(0, 0) + Scalar(1)) / Scalar(2);
    const Scalar yy = (r_(1, 1) + Scalar(1)) / Scalar(2);
    const Scalar zz = (r_(2, 2) + Scalar(1)) / Scalar(2);
    Vector3 n;
    if (xx > yy && xx > zz) {
      const Scalar x = std::sqrt(std::max(xx, Scalar(0)));
      n << x, (r_(0, 1) + r_(1, 0)) / (Scalar(4) * x),
          (r_(0, 2) + r_(2, 0)) / (Scalar(4) * x);
    } else if (yy > zz) {
      const Scalar y = std::sqrt(std::max(yy, Scalar(0)));
      n << (r_(0, 1) + r_(1, 0)) / (Scalar(4) * y), y,
          (r_(1, 2) + r_(2, 1)) / (Scalar(4) * y);
    } else {
      const Scalar z = std::sqrt(std::max(zz, Scalar(0)));
      n << (r_(0, 2) + r_(2, 0)) / (Scalar(4) * z),
          (r_(1, 2) + r_(2, 1)) / (Scalar(4) * z), z;
    }
    // The diagonal reconstruction fixes |n| but not its overall sign
    // (n and -n give the same R at exactly pi). Align with the skew part,
    // whose sign is correct for theta < pi.
    if (Vee(r_ - r_.transpose()).dot(n) < Scalar(0)) n = -n;
    return theta * n;
  }

  const Matrix3& rotationMatrix() const { return r_; }

  SO3 operator*(const SO3& other) const { return SO3(r_ * other.r_); }
  SO3 inverse() const { return SO3(r_.transpose()); }
  Vector3 act(const Vector3& v) const { return r_ * v; }

  // Right perturbation: this (+) dx = this * Exp(dx). Exact.
  SO3 boxplus(const Vector3& dx) const { return *this * Exp(dx); }
  // dx such that other.boxplus(dx) == *this.
  Vector3 boxminus(const SO3& other) const {
    return (other.inverse() * *this).Log();
  }

  // Adjoint of SO(3) is the rotation matrix itself.
  Matrix3 adjoint() const { return r_; }

  // Exact right Jacobian of Exp:
  // Jr = I - a W + b W^2, a = (1-cos t)/t^2, b = (t-sin t)/t^3,
  // with Taylor a -> 1/2 - t^2/24, b -> 1/6 - t^2/120 at the origin.
  static Matrix3 RightJacobianOfExp(const Vector3& w) {
    const Scalar theta_sq = w.squaredNorm();
    const Scalar theta = std::sqrt(theta_sq);
    const Matrix3 w_hat = Hat(w);
    const Matrix3 w_hat_sq = w_hat * w_hat;
    Scalar a, b;
    if (theta < TaylorThreshold()) {
      a = Scalar(0.5) - theta_sq / Scalar(24);
      b = Scalar(1) / Scalar(6) - theta_sq / Scalar(120);
    } else {
      a = (Scalar(1) - std::cos(theta)) / theta_sq;
      b = (theta - std::sin(theta)) / (theta_sq * theta);
    }
    return Matrix3::Identity() - a * w_hat + b * w_hat_sq;
  }

  // Lie algebra hat / vee.
  static Matrix3 Hat(const Vector3& w) {
    Matrix3 h;
    h << Scalar(0), -w(2), w(1), w(2), Scalar(0), -w(0), -w(1), w(0), Scalar(0);
    return h;
  }
  static Vector3 Vee(const Matrix3& h) {
    Vector3 w;
    w << h(2, 1), h(0, 2), h(1, 0);
    return w;
  }

 private:
  static Scalar TaylorThreshold() {
    using std::sqrt;
    return sqrt(Eigen::NumTraits<Scalar>::epsilon());
  }

  static constexpr double kPi = 3.14159265358979323846;
  Matrix3 r_;
};

}  // namespace talos::geometry
