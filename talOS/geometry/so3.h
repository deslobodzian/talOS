#pragma once

// SO(3): spatial rotations stored as a 3x3 matrix. Normalize noisy input
// matrices with FromMatrixNormalized before treating them as rotations.
// Exp uses Rodrigues; Log reconstructs the axis near the pi singularity.

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>

namespace talos::geometry {

template <typename Scalar = double>
class SO3 {
 public:
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

  SO3() : r_(Matrix3::Identity()) {}
  explicit SO3(const Matrix3& r) : r_(r) {}

  static SO3 Identity() { return SO3(); }

  static SO3 FromMatrixNormalized(const Matrix3& r) {
    if (!r.allFinite()) return Identity();
    const Eigen::JacobiSVD<Matrix3> svd(
        r, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.info() != Eigen::Success) return Identity();
    const Matrix3& u = svd.matrixU();
    const Matrix3 vt = svd.matrixV().transpose();
    Matrix3 normalized = u * vt;
    if (normalized.determinant() < Scalar(0)) {
      Matrix3 u_flipped = u;
      u_flipped.col(2) = -u_flipped.col(2);
      normalized = u_flipped * vt;
    }
    if (!normalized.allFinite()) return Identity();
    return SO3(normalized);
  }

  // Inverse of the right Jacobian of Exp (equals SE3::LeftVInverse(-w)).
  // Uses a 4-term Taylor series for theta < 0.05 to avoid 1 - (t/2)cot(t/2)
  // cancellation, and factors out unit axis u = w/|w| elsewhere to prevent
  // theta^2 / w_hat^2 overflow on large finite vectors.
  static Matrix3 RightJacobianInverse(const Vector3& w) {
    const Scalar theta_sq = w.squaredNorm();
    if (theta_sq < Scalar(2.5e-3)) {
      const Matrix3 h = Hat(w);
      const Scalar c = Scalar(1) / Scalar(12) +
                       theta_sq * (Scalar(1) / Scalar(720) +
                                   theta_sq * (Scalar(1) / Scalar(30240) +
                                               theta_sq / Scalar(1209600)));
      return Matrix3::Identity() + h / Scalar(2) + c * (h * h);
    }
    Scalar theta;
    Vector3 unit_axis;
    if (!ScaleAxis(w, theta_sq, theta, unit_axis) || !std::isfinite(theta)) {
      return Matrix3::Constant(std::numeric_limits<Scalar>::quiet_NaN());
    }
    const Scalar half = theta / Scalar(2);
    const Scalar sin_half = std::sin(half);
    if (sin_half == Scalar(0)) return Matrix3::Zero();
    const Scalar cot_term = half * (std::cos(half) / sin_half);
    if (!std::isfinite(cot_term)) return Matrix3::Zero();
    const Matrix3 u_hat = Hat(unit_axis);
    const Scalar c_scaled = Scalar(1) - cot_term;
    const Matrix3 result =
        Matrix3::Identity() + Hat(w) / Scalar(2) + c_scaled * (u_hat * u_hat);
    return result.allFinite() ? result : Matrix3::Zero();
  }

  // Exponential map (Rodrigues). Taylor below |w|^2 ~ sqrt(eps), where the
  // dropped O(|w|^4) terms sit under machine epsilon; elsewhere the half-angle
  // identity avoids 1 - cos(theta) cancellation, and an overflow-safe fallback
  // factors out the unit axis when |w|^2 overflows Scalar.
  static SO3 Exp(const Vector3& w) {
    const Scalar theta_sq = w.squaredNorm();
    if (theta_sq < TaylorThreshold()) {
      const Matrix3 w_hat = Hat(w);
      const Scalar a = Scalar(1) - theta_sq / Scalar(6);
      const Scalar b = Scalar(0.5) - theta_sq / Scalar(24);
      return SO3(Matrix3::Identity() + a * w_hat + b * (w_hat * w_hat));
    }
    if (std::isfinite(theta_sq)) {
      const Scalar theta = std::sqrt(theta_sq);
      const Matrix3 w_hat = Hat(w);
      const Scalar a = std::sin(theta) / theta;
      const Scalar half_sin = std::sin(Scalar(0.5) * theta);
      const Scalar b = (Scalar(2) * half_sin * half_sin) / theta_sq;
      return SO3(Matrix3::Identity() + a * w_hat + b * (w_hat * w_hat));
    }
    Scalar theta;
    Vector3 unit_axis;
    if (!ScaleAxis(w, theta_sq, theta, unit_axis) || !std::isfinite(theta)) {
      return SO3(Matrix3::Constant(std::numeric_limits<Scalar>::quiet_NaN()));
    }
    const Matrix3 u_hat = Hat(unit_axis);
    const Scalar half_sin = std::sin(Scalar(0.5) * theta);
    return SO3(Matrix3::Identity() + std::sin(theta) * u_hat +
               (Scalar(2) * half_sin * half_sin) * (u_hat * u_hat));
  }

  // Logarithmic map. The angle comes from atan2(sin, cos), with sin taken from
  // the skew part, which stays accurate up to pi; acos of the trace loses
  // relative accuracy in sin(theta) as (pi - theta) shrinks. Away from pi the
  // axis comes from the skew part; within 0.1 rad of pi it comes from the
  // largest-diagonal column of the exact symmetric matrix
  // (R + R^T) / 2 - cos(theta) I = (1 - cos(theta)) n n^T, which has no
  // O((pi - theta)^2) truncation error and stays nonzero even on degenerate
  // matrices with cos(theta) < 0 (such as -I).
  Vector3 Log() const {
    const Vector3 skew = Vee(r_ - r_.transpose());  // 2 sin(theta) n
    const Scalar sin_theta = skew.norm() / Scalar(2);
    const Scalar cos_theta = (r_.trace() - Scalar(1)) / Scalar(2);
    const Scalar theta = std::atan2(sin_theta, cos_theta);
    if (theta < TaylorThreshold()) {
      // theta / (2 sin theta) = 1/2 (1 + theta^2/6 + ...).
      const Scalar k = Scalar(0.5) * (Scalar(1) + theta * theta / Scalar(6));
      return k * skew;
    }
    if (theta < Scalar(kPi) - Scalar(0.1)) {
      return (theta / (Scalar(2) * sin_theta)) * skew;
    }
    const Scalar d0 = r_(0, 0) - cos_theta;
    const Scalar d1 = r_(1, 1) - cos_theta;
    const Scalar d2 = r_(2, 2) - cos_theta;
    Vector3 n;
    if (d0 >= d1 && d0 >= d2) {
      n << d0, (r_(0, 1) + r_(1, 0)) / Scalar(2),
          (r_(0, 2) + r_(2, 0)) / Scalar(2);
    } else if (d1 >= d2) {
      n << (r_(0, 1) + r_(1, 0)) / Scalar(2), d1,
          (r_(1, 2) + r_(2, 1)) / Scalar(2);
    } else {
      n << (r_(0, 2) + r_(2, 0)) / Scalar(2), (r_(1, 2) + r_(2, 1)) / Scalar(2),
          d2;
    }
    // The symmetric part fixes n up to sign (n and -n give the same R at
    // exactly pi). Align with the skew part, whose sign is correct for
    // theta < pi.
    if (skew.dot(n) < Scalar(0)) n = -n;
    const Scalar n_norm = n.norm();
    if (!(n_norm > Scalar(0)) || !std::isfinite(n_norm)) {
      return Vector3::Constant(std::numeric_limits<Scalar>::quiet_NaN());
    }
    return (theta / n_norm) * n;
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
  // Jr = I - a W + b W^2, a = (1-cos t)/t^2, b = (t-sin t)/t^3.
  // Uses a 4-term Taylor series for theta < 0.05 to avoid cancellation, and
  // unit-axis form I - (2 sin^2(t/2)/t) U + (1 - sin t / t) U^2 elsewhere to
  // avoid t^3 and W^2 overflow on large finite vectors.
  static Matrix3 RightJacobianOfExp(const Vector3& w) {
    const Scalar theta_sq = w.squaredNorm();
    if (theta_sq < Scalar(2.5e-3)) {
      const Matrix3 w_hat = Hat(w);
      const Scalar a =
          Scalar(0.5) + theta_sq * (-Scalar(1) / Scalar(24) +
                                    theta_sq * (Scalar(1) / Scalar(720) -
                                                theta_sq / Scalar(40320)));
      const Scalar b = Scalar(1) / Scalar(6) +
                       theta_sq * (-Scalar(1) / Scalar(120) +
                                   theta_sq * (Scalar(1) / Scalar(5040) -
                                               theta_sq / Scalar(362880)));
      return Matrix3::Identity() - a * w_hat + b * (w_hat * w_hat);
    }
    Scalar theta;
    Vector3 unit_axis;
    if (!ScaleAxis(w, theta_sq, theta, unit_axis)) {
      return Matrix3::Constant(std::numeric_limits<Scalar>::quiet_NaN());
    }
    const Matrix3 u_hat = Hat(unit_axis);
    if (!std::isfinite(theta)) {
      // Finite w with ||w|| > Scalar::max: (1 - cos t)/t -> 0, sin t / t -> 0.
      return Matrix3::Identity() + u_hat * u_hat;
    }
    const Scalar inv_theta = Scalar(1) / theta;
    const Scalar half_sin = std::sin(Scalar(0.5) * theta);
    const Scalar a_scaled = (Scalar(2) * half_sin * half_sin) * inv_theta;
    const Scalar b_scaled = Scalar(1) - std::sin(theta) * inv_theta;
    return Matrix3::Identity() - a_scaled * u_hat + b_scaled * (u_hat * u_hat);
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
  static bool ScaleAxis(const Vector3& w, Scalar theta_sq, Scalar& theta,
                        Vector3& unit_axis) {
    if (std::isfinite(theta_sq)) {
      theta = std::sqrt(theta_sq);
      unit_axis = w / theta;
      return true;
    }
    if (!w.allFinite()) return false;
    const Scalar scale = w.cwiseAbs().maxCoeff();
    const Vector3 scaled = w / scale;
    const Scalar scaled_norm = scaled.norm();
    theta = scale * scaled_norm;
    unit_axis = scaled / scaled_norm;
    return true;
  }

  static Scalar TaylorThreshold() {
    using std::sqrt;
    return sqrt(Eigen::NumTraits<Scalar>::epsilon());
  }

  static constexpr double kPi = 3.14159265358979323846;
  Matrix3 r_;
};

}  // namespace talos::geometry
