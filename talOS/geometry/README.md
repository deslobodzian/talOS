# talOS geometry

Header-only Lie groups on Eigen: `SO2`/`SE2` for planar work, `SO3`/`SE3`
for spatial. Include `talOS/geometry/lie.h`.

## Conventions

* Right perturbation everywhere: `x.boxplus(dx) = x * Exp(dx)`, and
  `other.boxminus(x)` is the `dx` with `other.boxplus(dx) == x`.
* Tangents are `(omega, v)` ordered: 3-vectors for SE(2), 6-vectors
  for SE(3).
* `SO3::Log` is exact away from the pi branch cut and reconstructs the
  axis by the largest-diagonal method at it; every rotation round-trips.
* `SE3::RightJacobianOfExp` is a 3rd-order BCH series (error O(|tau|^4)):
  exact in the optimizer-step regime, not a large-angle formula. The
  rotation-only `SO3::RightJacobianOfExp` is exact closed form.
* Scalar-templated (`double` default; `float` works). Autodiff scalar
  types are future work: the small-angle branches compare against a
  threshold, which ordering-free scalar types cannot do.

## Performance notes

* Fixed-size Eigen types only: zero heap allocation on any path.
* SE(3) stores rotation + translation separately, so composition costs a
  3x3 product plus one action instead of a 4x4 product.
* Exp/Log are closed form (Rodrigues, V matrix): no matrix exponential,
  no 4x4 inverse.
* Run `bazel run //talOS/geometry:geometry_bench -c opt` for ns/op.
  Reference (Apple M-series, -O3): so3_compose 6, so3_exp 27, so3_log 16,
  so3_jr 6, se3_compose 3, se3_exp 21, se3_log 39, se3_act 4, se3_jr 52
  (all ns/op). For scale: a 200 Hz loop budget is 5,000,000 ns, so a
  full odometry update (compose + act + log) costs ~0.0002% of a cycle.
