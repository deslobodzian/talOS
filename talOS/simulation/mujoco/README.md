# MuJoCo

The [MuJoCo](https://mujoco.org) physics engine as a Bazel dependency
(`@mujoco//:mujoco`, 3.17.0 from source with a root-only patch
(`third_party/mujoco_mesh_transformed_init.patch`,
[INSTALL](../../../docs/INSTALL.md)), C engine `-O2` in every build by
`talos.bazelrc`; [why](../../../docs/METHODOLOGY.md#13-open-source-first)).
Only this package and its subpackages may depend on it, which
`tools/check_layers.py --source-only` enforces, so no node and not the RoboRIO
program links it.

`:determinism_test` pins what a simulator stepping MuJoCo relies on:

- two compiles of one model, stepped from one state, end in the same
  `mjSTATE_INTEGRATION` state to the last bit, sleeping off and on;
- a bit-exact restart must carry `mjSTATE_INTEGRATION` (`mj_getState`, then
  `mj_setState` into a fresh `mjData`): it includes the solver's warm start
  (`qacc_warmstart`), without which, under CG with 20 iterations, the
  restarted run diverges. The restart is pinned with sleeping off;
  `mjSTATE_INTEGRATION` does not carry which trees are asleep.

Bit identity holds per host and build: the same binary, thread and
floating-point environment. It is not claimed across hosts: arm64 compilers
contract `a*b+c` into a fused multiply-add, x86-64 ones do not without
`-mfma`, so a Mac and a Linux machine can produce different bits.

[`fields/rebuilt_2026`](fields/rebuilt_2026/README.md) is the 2026 field as
a generated MuJoCo model.
