# robot_repo

A robot in its own Bazel repository. It depends on talOS the way a team's
robot repository does: talOS checked out at one commit (here, the checkout
two directories up; in a robot repository, a git submodule) and pinned in
`MODULE.bazel` with `bazel_dep(name = "talos")` and `local_path_override`.
Its BUILD files and configs name talOS only through `@talos//` labels.

It holds a C++ node (`roller/`) and a Python node (`gate/`), written as
`@talos//tools:new_subsystem` writes them, each with its Sim test, plus talOS's
teleop and telemetry nodes, all listed in `robot.toml`. talOS's own
`bazel build //...` skips this directory (`.bazelignore`); it is a separate
workspace.

```sh
cd examples/robot_repo
bazel test //...                       # graph test, node tests, sim smoke, Studio and session_report tests
bazel build //:robot_bundle            # the deployable bundle
bazel run //:sim -- --duration-s 10    # the robot against the simulator
bazel run //:sim -- --start-studio-bridge   # with Studio (below)
bazel run @talos//talOS/launcher:session_report -- <session dir>   # a recorded session's report
bazel run @talos//tools:sync_manifest -- robot.toml   # after editing [[subsystems]]
```

## Add a subsystem

From the repository's root:

```sh
bazel run @talos//tools:new_subsystem -- intake --lang cpp   # or --lang py; --type position|estimator
bazel test //...
```

It writes the node package (messages, node, Sim test) beside the config's
other nodes (here `intake/`; in a robot whose nodes are
`//main_processor/<name>:node`, `main_processor/intake/`), its subsystem
file in `subsystems/`, the `[[subsystems]]` entry in `robot.toml`, and
regenerates `robot_manifest.bzl` and the manifest of every other config that
includes `robot.toml`. Its labels name talOS by `@talos//`. `--config PATH`
names another robot config (default `robot.toml`, else
`main_processor/configuration/robot.toml`); `--package-dir DIR` puts the
package elsewhere. Its `Next:` lists the placeholders to replace (the CAN
id) and simulation-only lines for a `robot_sim.toml`. This fixture's
`//:sim` runs `robot.toml`, not that file, and no `manifest_sync_test`
checks it, so here those lines do not make the mechanism move.

`bazel test //...` passes right after the scaffold. In a robot whose config
requires target coverage (`[robot] require_target_coverage = true`, set
directly or by a config that includes it, or `commissioned = true`), the
subsystem file gets an uncommented `[targets]` with every mode empty: the
new subsystem has no producer yet, the coverage check judges only modes
that list a lane, and with none the node stays neutral. Listing
`teleop = ["teleop"]` there goes with uncommenting its teleop binding. This
fixture does not require coverage, so its file keeps `[targets]` commented
and `[robot.targets]` applies.

## Simulation

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

## Studio

`bazel run //:sim -- --start-studio-bridge` also runs talOS's Studio bridge
as a session member: Studio at `http://localhost:5800` (`[studio]
http_port`, or `--studio-http-port PORT`) and the driver dashboard at
`/drive`. The pages come from the talOS checkout, which builds them as its
own workspace. In a robot repository, whose talOS is the submodule
`talos/`, a student types:

```sh
(cd talos && bazel build //studio:web_dist)   # once, and after moving the talOS pin
bazel run //:sim -- --start-studio-bridge     # Studio at http://localhost:5800, the driver dashboard at /drive
```

(here, `(cd ../.. && bazel build //studio:web_dist)`).

`MODULE.bazel`'s `studio_web` reads that build's folder (`path`, here
`../../bazel-bin/studio/dist`, in a robot repository
`talos/bazel-bin/studio/dist`), `BUILD.bazel`'s `//:studio_webroot` makes it
one directory in `//:sim`'s `data`, and `robot.toml`'s `[studio] webroot`
names it (`bazel-bin/studio_webroot`); `@talos//studio/bridge:webroot.bzl`
says how. Until the pages are built, `//:studio_webroot` holds only
`NOT_BUILT.txt` (Bazel leaves an empty directory out of runfiles), and `/`
and `/drive` answer 503 with the command above; Bazel fetches the folder again when they are built, so the
next `bazel run //:sim` serves them. This robot has no `[studio] dashboard`
layout, so `/drive` shows no widgets
([talOS/telemetry](../../talOS/telemetry/README.md#dashboard-layout)).

`studio_test` runs `//:sim --start-studio-bridge` on free ports until the
launcher reports the bridge up, requires `/drive` to answer with the page or
with that command naming `//:studio_webroot`'s folder (so a webroot the
config or `//:sim`'s `data` miswires fails it), stops the session, and requires the bridge to have run
once and exited 0. It is not a `talos_sim_test`, which waits for every
member's registry row; the bridge takes none.

`robot_bundle` carries no Studio: `talos_bundle` refuses a webroot without
`index.html`, and this fixture's checks run where the pages are not built.
A robot repository that builds them adds
`studio_bridge = "@talos//studio/bridge:studio_bridge"` and
`studio_webroot = ":studio_webroot"` to its `talos_bundle`.

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
