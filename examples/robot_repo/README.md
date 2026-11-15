# robot_repo

A robot in its own Bazel repository. It depends on talOS the way a team's
robot repository does: talOS checked out at one commit (here, the checkout
two directories up; in a robot repository, a git submodule) and pinned in
`MODULE.bazel` with `bazel_dep(name = "talos")` and `local_path_override`.
Its BUILD files and configs name talOS only through `@talos//` labels.

It holds a C++ node (`roller/`) and a Python node (`gate/`), written as
`//tools:new_subsystem` writes them, each with its Sim test, plus talOS's
teleop and telemetry nodes, all listed in `robot.toml`. talOS's own
`bazel build //...` skips this directory (`.bazelignore`); it is a separate
workspace.

```sh
cd examples/robot_repo
bazel test //...                       # graph test, node tests, sim smoke test
bazel build //:robot_bundle            # the deployable bundle
bazel run //:sim -- --duration-s 10    # the robot against the simulator
bazel run @talos//tools:sync_manifest -- robot.toml   # after editing [[subsystems]]
```

`bazel run //:sim -- ARGS` runs `launcher --config robot.toml --sim
--start-sim-gateway --sim-gateway-arg --physics ARGS` from the target's
runfiles (`talos_sim`, `talOS/build/contract.bzl`); it starts no Studio bridge
unless ARGS include `--start-studio-bridge`. The simulator is the one
`[robot] sim_gateway` names, which `BUILD.bazel` also lists in the target's
`data`. The launcher gives it the bridge's ports (`[robot] controller_port`
and `local_port`, or ARGS' `--remote-port` and `--local-port`).

`sim_smoke_test` is `talos_sim_test(sim = ":sim")`: it runs `//:sim` with
those two flags on free ports until the bridge has accepted a State and
every member's loop has started, then 1 s more, stops it, and fails unless
every member the session records ran once and exited 0 and the bridge
received State from the simulator. A robot repository adds the same few lines; what the check
judges is in `talos_sim_test`'s docstring (`talOS/build/contract.bzl`).

## What a robot repository names by `@talos//`

`robot.toml`'s `[robot] hardware_node`, `[robot] sim_gateway` and `[studio]
bridge`, and every talOS node (`teleop`, `telemetry`) in its subsystem file.
Their defaults are labels of the main repository, which in a robot
repository name nothing.

## What it copies from talOS by hand

| Copied | From | Stale or missing |
|---|---|---|
| `.bazelrc`'s C++23 `--cxxopt` and `--host_cxxopt` lines (and macOS's `-fexperimental-library` and minimum OS) | talOS's `.bazelrc` | talOS stops compiling once it uses a newer language feature, or the config resolver the bundle and manifest actions run fails to build (exec configuration) |
| `--incompatible_default_to_explicit_init_py`, `--test_env=PYTHONDONTWRITEBYTECODE=1` | talOS's `.bazelrc` | parallel Python tests can corrupt shared runfiles |
| `build:linux --@@libjpeg_turbo+//:noasm=yes` | talOS's `.bazelrc` | a node linking OpenCV fails to load on x86-64 Linux |
| `.bazelversion` | talOS's | a different Bazel can resolve the module graph differently |
| `python.toolchain(python_version = "3.11", is_default = True)` | talOS's `MODULE.bazel` | the root module picks the interpreter, and talOS's 3.11 wheels run under another |

Nothing is copied for talOS's patched toml++: talOS fetches it as its own
patched archive. talOS's OpenCV patch is a `single_version_override`, which
applies only in talOS's own workspace, so a repository that depends on talOS
builds OpenCV unpatched; it differs only under `--config=tsan`
([docs/INSTALL.md](../../docs/INSTALL.md)).

Studio's bridge (`@talos//studio/bridge:studio_bridge`) builds from a
repository that depends on talOS; Studio's pages (`@talos//studio:web_dist`)
do not, since they need rules_js and pnpm, development dependencies of
talOS.
