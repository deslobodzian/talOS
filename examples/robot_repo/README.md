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
workspace. `talos_checks/` checks talOS itself from a dependent repository
(`session_report` on a session talOS's fixture records), nothing of this
robot's.

It is also the template of a team's robot repository:
`bazel run //tools:new_robot -- <name> --standalone DIR`, from a talOS
checkout, copies it to DIR as `TEMPLATE` in `tools/new_robot.py` places each
file, without `roller/`, `gate/` and `talos_checks/`, naming the new robot
and pointing `@talos` at that checkout
([docs/FIRST_MECHANISM.md](../../docs/FIRST_MECHANISM.md) walks it). A file
git tracks here that `TEMPLATE` does not place makes `new_robot` refuse
(untracked files are ignored and never copied), which
`tools/check_robot_repo_scaffold.sh` (prepush and CI) catches, so changing
this fixture means saying there whether a robot repository gets the file.

```sh
cd examples/robot_repo
bazel test //...                       # graph test, node tests, sim smoke, Studio, bundle install, pin, trace and session_report tests
bazel build //:robot_bundle            # the deployable bundle (--config=release records the revision)
bazel run //:sim -- --duration-s 10    # the robot against the simulator
bazel run //:sim -- --start-studio-bridge   # with Studio (below)
bazel run @talos//talOS/launcher:session_report -- <session dir>   # a recorded session's report
bazel run @talos//tools:sync_manifest -- robot.toml   # after editing [[subsystems]]
bazel run @talos//controller_processor/rio:gradlew -- buildRio -Pcompanion=<ipv4>  # RoboRIO binary (deploy also needs -PteamNumber or .wpilib/wpilib_preferences.json)
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
package elsewhere. A repository without a simulation config gets one with
its first mechanism: `robot_sim.toml` beside the config (it includes the
config, which never reads it), `robot_sim_manifest.bzl`, and its
`sim_manifest_sync_test` and `sim_graph_test` in the config's `BUILD`, so
`bazel test //...` checks it. When the root `BUILD` has a `talos_sim(name =
"sim")` pointed at the robot config with its manifest's `CONFIG_FILES` and
`NODES` (as here), the scaffold also points `//:sim` at `robot_sim.toml`,
`SIM_CONFIG_FILES` and `SIM_NODES`. Its `Next:` lists the placeholders to
replace (the CAN id) and simulation-only lines to append to
`robot_sim.toml`, then the command that runs it: `bazel run //:sim` when
the root `talos_sim` names that file, else `@talos//talOS/launcher` on it
with `--sim --start-sim-gateway`.

`bazel test //...` passes right after the scaffold. In a robot whose config
requires target coverage (`[robot] require_target_coverage = true`, set
directly or by a config that includes it, or `commissioned = true`), the
subsystem file gets an uncommented `[targets]` with every mode empty: the
new subsystem has no producer yet, the coverage check judges only modes
that list a lane, and with none the node stays neutral. Listing
`teleop = ["teleop"]` there goes with uncommenting its teleop binding. This
fixture does not require coverage, so its file keeps `[targets]` commented
and `[robot.targets]` applies. `tools/check_robot_repo_scaffold.sh`
(prepush and CI) holds both cases, and the repository `new_robot` makes: it
scaffolds a C++ and a Python subsystem into a copy of this fixture, as it is
and with coverage required, and into `new_robot --standalone`'s copy of it,
and runs `bazel test //...` in each.

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
own workspace, so a student builds them there once, and again after moving
the talOS version:

```sh
(cd <talOS checkout> && bazel build //studio:web_dist)
bazel run //:sim -- --start-studio-bridge     # Studio at http://localhost:5800, the driver dashboard at /drive
```

The checkout is whichever one `MODULE.bazel`'s `local_path_override` names:
here `../..`; in a repository from `tools/new_robot.py`, the checkout it was
made from (beside the repository, by a relative path, or an absolute one);
with a git submodule, `talos`. A `git_override` leaves no checkout to build
in. `MODULE.bazel`'s `studio_web` reads that build's folder (`path`,
`<checkout>/bazel-bin/studio/dist`: here `../../bazel-bin/studio/dist`), `BUILD.bazel`'s `//:studio_webroot` makes it
one directory in `//:sim`'s `data`, and `robot.toml`'s `[studio] webroot`
names it (`bazel-bin/studio_webroot`); `@talos//studio/bridge:webroot.bzl`
says how. Until the pages are built, `//:studio_webroot` holds only
`NOT_BUILT.txt` (Bazel leaves an empty directory out of runfiles), naming
the talOS checkout `studio_web`'s `path` lies in, and `/` and `/drive`
answer 503 with the command above for that checkout; Bazel fetches the folder again when they are built, so the
next `bazel run //:sim` serves them. This robot has no `[studio] dashboard`
layout, so `/drive` shows no widgets
([talOS/telemetry](../../talOS/telemetry/README.md#dashboard-layout)).

`studio_test` runs `//:sim --start-studio-bridge` on free ports until the
launcher reports the bridge up. It requires `/drive` to answer with the page
when `//:studio_webroot` holds no `NOT_BUILT.txt`, else with that command
naming `//:studio_webroot`'s folder and a talOS checkout that exists (so a
webroot the config or `//:sim`'s `data` miswires fails it). It then stops
the session and requires the bridge to have run once and exited 0. It runs
outside Bazel's sandbox, which would stage each page as a link out of the
webroot, a file the bridge does not serve. It is not a `talos_sim_test`, which waits for every
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

## Deployment

`bazel build --config=release //:robot_bundle` builds what the robot's main
processor installs, stamped with this repository's git revision
(`.bazelrc`'s `--workspace_status_command`, talOS's
`tools/workspace_status.sh` run here), and `@talos//tools:deploy.sh install`
installs it ([docs/DEPLOYMENT.md](../../docs/DEPLOYMENT.md)).
`bundle_install_test` (e2e, about 2 s) does that into a scratch prefix: the
install exits 0 with no warm-up warning (its graph check ran the bundle's
own `run.sh --describe-only`), and the installed `run.sh --describe-only`
describes every node of `robot_manifest.bzl` and the hardware bridge. It
installs from a copy of `bazel-bin/robot_bundle` that it removes first,
runs without Bazel's runfiles variables, and fails on a link in the install
that leaves it or a file naming Bazel's output base or either checkout, so
the bundle runs where the build tree is not. A robot repository gets the
same test: talOS's own bundle tests install bundles built inside talOS,
never one built under `@talos`'s external name.

To build or deploy the RoboRIO gateway program from this repository, run
`bazel run @talos//controller_processor/rio:gradlew -- buildRio -Pcompanion=<ipv4>`
or `bazel run @talos//controller_processor/rio:gradlew -- deploy -Pcompanion=<ipv4> -PteamNumber=<team>`
(or commit `.wpilib/wpilib_preferences.json` with `"teamNumber"` in the robot
repository; talOS carries no default team number or companion address). It
stages the Gradle project and required `talOS/` sources into a cache directory
without modifying the talOS checkout and stamps `-PtalosRevision` and
`-PtalosDirty` from `@talos`.

## Requirements

Its tests tag the talOS requirements they guard
([docs/TESTING.md §3](../../docs/TESTING.md#3-requirements-and-traceability)).
`trace_test` runs talOS's `tools/trace.py --check` on this repository's
tracked files (every file, before `git init`) with talOS's catalogue, the
one in the checkout `@talos` is built from: a tag naming no row of it fails.
This fixture has no rows of its own, so it has no `docs/REQUIREMENTS.md`; a
robot repository that adds one, holding only its own rows in talOS's
numbering and its own package scope table, is checked against it from then
on, as talOS's is against its own, and an ID in both catalogues fails. The
test reads files Bazel does not track, so it runs every time (`external`).

## What it shares with talOS

A `.bazelrc` belongs to the workspace Bazel runs in, so none of talOS's
reaches a repository that depends on it. `.bazelrc` therefore imports the
flags talOS's code needs from talOS's [`talos.bazelrc`](../../talos.bazelrc)
(C++23 for the target and exec configurations, macOS's libc++ flags, the
Python runfiles flags, libjpeg-turbo's `noasm`, and the `asan`, `tsan`,
`release` and test-lane configs), from the checkout `MODULE.bazel`'s
`local_path_override` names, by the same path: `../..` here,
`%workspace%/talos/talos.bazelrc` with talOS as the git submodule `talos/`,
an absolute path where `new_robot` had to write one. The repository's own
flags go below the import. Bazel's `import` takes a path, never a label, so
a talOS that `git_override` fetches has no file to import: such a
repository commits a copy of that commit's `talos.bazelrc` and imports it.

Two more things belong to the workspace and are kept equal by hand:

| Kept equal | To | Stale |
|---|---|---|
| `.bazelversion` | talOS's | a different Bazel can resolve the module graph differently |
| `python.toolchain(python_version = "3.11", is_default = True)` | talOS's `MODULE.bazel` | the root module picks the interpreter, and talOS's 3.11 wheels run under another |

`talos_pin_test` fails when `.bazelrc` imports no `talos.bazelrc`, or
another checkout's than the one `@talos` is built from (for a fetched talOS,
a copy whose text differs), when either row above differs from that
talOS's, or when `--workspace_status_command` names no executable file. It
reads that talOS's files directly, which Bazel does not track, so it runs
every time (`external`); it runs no Bazel.

Nothing is copied for talOS's patched toml++: talOS fetches it as its own
patched archive. talOS's OpenCV patch is a `single_version_override`, which
applies only in talOS's own workspace, so a repository that depends on talOS
builds OpenCV unpatched; it differs only under `--config=tsan`
([docs/INSTALL.md](../../docs/INSTALL.md)).
