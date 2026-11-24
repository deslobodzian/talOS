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
  Nonpositive or non-finite periods return the original velocities.
* `SO3::FromMatrixNormalized` projects noisy matrices onto proper rotations
  using fixed-size SVD, including a determinant correction for reflections
  (non-finite inputs return `SO3::Identity()`).
  `RightJacobianInverse` is the closed-form inverse of the right Jacobian.
* `SO3::Log` uses the skew part away from pi and reconstructs the axis from the
  symmetric part near pi.
* `SE3::RightJacobianOfExp` is a 3rd-order BCH series (error O(|tau|^4)):
  accurate for small optimizer steps, not a large-angle formula. The
  rotation-only `SO3::RightJacobianOfExp` is exact closed form.
* Scalar-templated (`double` default). Small-angle branches require scalar
  comparisons.

## Swerve module setpoint

`talOS/geometry/swerve_module.h`: `OptimizeSwerveModule(speed,
asked_heading_rad, measured_rad)` returns the `SwerveModuleSetpoint` a swerve
module drives to. The steering change is the shortest rotation from the
measured angle (unwrapped, any magnitude) to the asked heading; past a
quarter turn it takes the opposite heading, negates the speed and sets
`reversed` (exactly a quarter turn keeps the asked heading). The speed is
then scaled by `max(0, cos(steer_delta_rad))`, so `steer_delta_rad` is in
`[-pi/2, pi/2]`. A non-finite input returns `{0, 0, false}`. It allocates
nothing. The 2026 drivetrain (`ToModuleStates`) and the bench
`swerve_module` node take their module setpoints from it.

## Performance notes

* Geometry operations use fixed-size Eigen types.
* SE(3) stores rotation + translation separately, so composition costs a
  3x3 product plus one action instead of a 4x4 product.
* Exp/Log are closed form (Rodrigues, V matrix): no matrix exponential,
  no 4x4 inverse.
* Run `bazel run //talOS/geometry:geometry_bench -c opt` for ns/op.
