# talOS geometry

Header-only Lie groups on Eigen: `SO2`/`SE2` for planar work, `SO3`/`SE3`
for spatial. Include `talOS/geometry/lie.h`.

## Conventions

* Right perturbation everywhere: `x.boxplus(dx) = x * Exp(dx)`, and
  `x.boxminus(other)` is the `dx` with `other.boxplus(dx) == x`.
* Tangents are `(omega, v)` ordered: 3-vectors for SE(2), 6-vectors
  for SE(3).
* `SO2` construction, composition and logarithms use angles in `(-pi, pi]`.
  `FromCosSin` accepts an unnormalized cosine/sine pair.
* `SE2::Discretize(vx, vy, omega, dt)` returns the constant twist whose `Exp`
  over `dt` reaches the finite-period pose `(omega * dt, {vx * dt, vy * dt})`.
  Its rate stays `omega` for any `dt` (it never wraps past pi); only the linear
  part goes through `V(omega * dt)^-1`, which grows without bound near the
  nonzero multiples of 2 pi where no twist with that rate translates.
  Nonpositive periods return the original velocities.
* `SO3::FromMatrixNormalized` projects noisy matrices onto proper rotations
  using fixed-size SVD, including a determinant correction for reflections.
  `RightJacobianInverse` is the closed-form inverse of the right Jacobian.
* `SO3::Log` uses the skew part away from pi and reconstructs the axis by the
  largest-diagonal method near pi.
* `SE3::RightJacobianOfExp` is a 3rd-order BCH series (error O(|tau|^4)):
  accurate for small optimizer steps, not a large-angle formula. The
  rotation-only `SO3::RightJacobianOfExp` is exact closed form.
* Scalar-templated (`double` default). Small-angle branches require scalar
  comparisons.

## Performance notes

* Geometry operations use fixed-size Eigen types.
* SE(3) stores rotation + translation separately, so composition costs a
  3x3 product plus one action instead of a 4x4 product.
* Exp/Log are closed form (Rodrigues, V matrix): no matrix exponential,
  no 4x4 inverse.
* Run `bazel run //talOS/geometry:geometry_bench -c opt` for ns/op.
