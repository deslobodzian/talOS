# Deployment

The main processor runs a **bundle**: one directory with every node binary, the
launcher, the config and the runtime libraries. `tools/deploy.sh` installs
bundles, starts one on boot, shows logs, rolls back and prunes old bundles.

## Build a bundle

A robot is built and deployed from its own repository, which pins a talOS
checkout beside it or as a git submodule
([robot repository](GLOSSARY.md); [examples/robot_repo](../examples/robot_repo/README.md#deployment)
is the template `new_robot --standalone` copies). From that repository's root:

```sh
bazel build --config=release //:robot_bundle
```

`//:robot_bundle` is the template's `talos_bundle`; a repository that keeps
its bundle elsewhere uses its own label, here and below. talOS's in-tree
example is the bench robot's:

```sh
bazel build --config=release //bench/main_processor:bench_bundle
```

`--config=release` stamps the build: the bundle records the git revision of
the workspace Bazel runs in and whether that tree had uncommitted changes
(`git_revision` and `git_dirty`), and when a separate robot repository builds
against a talOS git checkout, talOS's revision and dirty state as well
(`talos_git_revision` and `talos_git_dirty`).
The config (`build:release --stamp`) is in [`talos.bazelrc`](../talos.bazelrc),
which talOS's `.bazelrc` and every robot repository's import; the revision
comes from the `--workspace_status_command` each workspace's own `.bazelrc`
sets beside that import (talOS's `tools/workspace_status.sh`, which a robot
repository runs from the talOS checkout its `MODULE.bazel` names; the
template's `.bazelrc` sets it). Without it the bundle builds the
same but records no revision.

`talos_bundle` (`talOS/build/contract.bzl`) takes `nodes`, `config` (the
robot's field config, the bench's `robot_bench.toml`) and `config_files`; the
robot's BUILD passes the hardware node plus `NODES`, and `CONFIG_FILES`, from
that config's generated manifest (the bench's
[`robot_bench_manifest.bzl`](../bench/main_processor/configuration/robot_bench_manifest.bzl),
from `bazel run //tools:sync_manifest -- bench/main_processor/configuration/robot_bench.toml`;
in a robot repository `bazel run @talos//tools:sync_manifest -- <config>`).
The build fails unless `nodes` is exactly what the config launches: its
subsystems' nodes and its hardware node, `[robot] hardware_node` or by default
`//talOS/bridge:hardware_node`. A robot viewed with Studio also sets
`studio_bridge` (`//studio/bridge:studio_bridge`) and `studio_webroot`
(`//studio:web_dist`, the built site), both or neither; the build fails when
`studio_bridge` is not the config's `[studio] bridge` (when it sets one) or
the site has no `index.html` or `drive.html`. The bench robot and `tools/new_robot`'s robots
inside talOS set both, which add about 2.5 MB; a robot repository made by
`new_robot --standalone` sets neither, so its deployed robot has no driver
dashboard until someone builds the pages and adds
`studio_bridge = "@talos//studio/bridge:studio_bridge"` and
`studio_webroot = ":studio_webroot"` to its `talos_bundle`
([examples/robot_repo](../examples/robot_repo/README.md#studio)).
It writes the bundle directory `bazel-bin/<package>/<name>/`
(`bazel-bin/bench/main_processor/bench_bundle/`):

| Entry | What |
|---|---|
| `run.sh` | runs `./launcher --bundle <dir> --config config/<config> "$@"` (the robot config's path below `config/`, `config/robot_bench.toml` in the bench bundle) from `<dir>`, its own directory with symlinks resolved, adding `--output-dir "$TALOS_LOG_DIR"` when that is set and, in a bundle that carries Studio, `--start-studio-bridge` unless given `--no-studio` (which it does not pass on); a `--config`/`--output-dir` argument replaces the default |
| `launcher` | `//talOS/launcher` |
| `bin/` | node binaries; a Python node's runfiles beside it (its wheels, e.g. `robotpy-wpimath` and `gtsam`, and `libtalos_node.so`) |
| `python/` | with a Python node: the Python runtime (interpreter and standard library) every Python node runs, one directory per runtime repository however many nodes carry it |
| `lib/` | any shared libraries C++ nodes link, one copy per name; no C++ WPILib libraries are staged. `talos_cc_node` binaries carry the rpath `@loader_path/../lib` (macOS) or `$ORIGIN/../lib` (Linux) |
| `studio/` | with `studio_bridge` and `studio_webroot` only: the bridge binary (`studio/studio_bridge`) and the site it serves (`studio/webroot/`) |
| `config/` | the robot config and every file it reads (`include`, `path`, `defaults`), at the same paths relative to each other as in the source tree |
| `bundle_manifest.toml` | node names, targets, binaries, each binary's SHA-256 (`binary_sha256`), languages, config SHA-256; in a stamped build `git_revision` and `git_dirty` (and, when a separate robot repository is built against a talOS git checkout, `talos_git_revision` and `talos_git_dirty`); from a `testonly` `talos_bundle`, `testonly = true` ([install](#on-the-main-processor) refuses it); with Studio, a `[studio]` table naming its bridge (path, SHA-256, label) and webroot |

The bundle holds no symlinks. Bazel may restore a tree artifact from its
disk or remote cache with each symlink in it replaced by a copy of its
target, so a bundle with one would run differently when built than when
restored. A link Bazel declares in a node's runfiles (a py_binary venv's
`bin/python3`) is staged as a two-line script that execs the bundled
interpreter at its own path in `python/`, where it finds its standard
library; the process stays the one the launcher started. The runtime's
repository is left out of each node's runfiles and staged once under
`python/`, and the interpreter's other names there (`python`, `python3.11`,
which Bazel's runfiles hold as copies of it) are such scripts too, so the
bundle holds the interpreter once, however many Python nodes it carries. On
Linux, when the bundled interpreter is an ELF binary that does not list
`libpython*` in `DT_NEEDED` and no other ELF binary or shared library in the
bundle does either, unused `libpython3*.so*` shared libraries are omitted
from the runtime's `lib/` (macOS `.dylib` runtimes and dynamically linked
Linux runtimes keep them). The build fails when two nodes carry different
files under one runtime's name.

The launcher copies `config_sha256`, `git_revision` and `git_dirty` (plus
`talos_git_revision` and `talos_git_dirty` when present) into every
session manifest, and records each member's binary SHA-256 again when the
session starts ([session manifest](../talOS/launcher/README.md#session-manifest)).
A digest covers only the file under `bin/`: not a C++ node's shared libraries
in `lib/`, nor a Python node's runfiles beside its entry script or the
runtime in `python/`. The bundle's
revision covers those.

The config SHA-256 is
the identity of the staged config: every file under `config/`, the robot
config included, contributes its path relative to `config/` and its bytes.
Absolute and workspace locations never participate, so moving the whole
config tree or copying the bundle elsewhere keeps the identity; editing,
renaming or moving any one file relative to the others (the root config too)
changes it.

A robot's bundle config is a field overlay of its simulation config: the
bench's `robot_bench.toml` includes the base
[robot config](../bench/main_processor/configuration/robot.toml) and changes
`[robot] controller_address` to the RoboRIO's `10.56.87.2` (and restates
`commissioned = false`). A field overlay for a multi-homed companion can also
set `[robot] local_address` to the companion's Ethernet IPv4 address (matching
the RoboRIO build's `-Pcompanion=<ipv4>`) so `hardware_node` binds that
interface on every socket open and reopen rather than `0.0.0.0`; leaving it
unset on a non-loopback target emits a non-fatal warning during
`--describe-only` and `deploy.sh install`'s graph check
([controller addressing](../talOS/launcher/README.md#controller-addressing)).
Simulation and its tests keep using the base config
with loopback (`127.0.0.1`), so no source edits are needed when switching
between simulation and the field bundle. Such an overlay does not change CAN
ids or commissioning: hardware must still be checked and commissioned through
the [bring-up](BRINGUP.md) process.

A bundle that carries Studio serves it whenever `run.sh` runs, by hand or
from the boot service: `run.sh` passes the launcher `--start-studio-bridge`,
which runs the bundle's bridge for `[studio] bridge` on `[studio] http_port`
(5800; Studio at `/`, the [driver dashboard](../studio/README.md#driver-dashboard)
at `/drive`, for a browser on the Driver Station laptop) and serves the
bundle's webroot unless `[studio] webroot` is an absolute path
([Studio bridge](../talOS/launcher/README.md#studio-bridge)).
`./run.sh --no-studio` runs the session without it. From a bundle built
without Studio, `run.sh` does not ask for it, and `--start-studio-bridge`
starts nothing: the session runs without Studio; so it does, after one
warning, when the bundled bridge cannot run on the robot (it does not answer
`--describe`), and from a checkout whose bridge is not built (the warning
gives the `bazel build` command).

To bring up the bundle against a different controller or companion interface
address without editing the config, run `./run.sh --remote <ip>` (for example,
`./run.sh --remote 10.56.87.3 --local-address 10.56.87.5`). These override
`[robot] controller_address` and `[robot] local_address` for that launch; use
`--remote-port` and `--local-port` if the ports also differ.

An installed bundle runs simulated with `./run.sh --sim --remote 127.0.0.1`
(plus `--local-address 127.0.0.1` if the bundle's config sets `[robot]
local_address` to a physical interface):
`--sim` requires the loopback remote and an unset or loopback local address,
and the hardware bridge's `--describe` probe and its runs both get the
effective values
([controller addressing](../talOS/launcher/README.md#controller-addressing)).
A bundle intentionally carries no simulator (`--start-sim-gateway` reports the
`[robot] sim_gateway` binary missing from the bundle): deployed bundles carry
only robot code and its config, so running a bundle simulated uses an external
simulator standing in for the RoboRIO on that host—a `sim_gateway` run from a
checkout with `--config` naming the bundle's config (which uses that config's
`[robot] controller_port` and `local_port`), or the RoboRIO program under WPILib
simulation. A launch that changes the ports with `--remote-port` and
`--local-port` needs the same values given to a hand-run `sim_gateway` as
`--port` and `--remote-port`.

## On the main processor

```sh
./tools/deploy.sh install <bundle> [--prefix /opt/talos] [--no-warm-up]
./tools/deploy.sh enable-boot [--prefix /opt/talos] [--user talos] [--dry-run]
./tools/deploy.sh boot-check
./tools/deploy.sh run [--prefix /opt/talos] [--dry-run] [--] [run.sh arguments]
./tools/deploy.sh logs [--prefix /opt/talos] [LOG_ROOT]
./tools/deploy.sh fetch [--prefix /opt/talos] DEST
./tools/deploy.sh rollback [--prefix /opt/talos] [--no-warm-up]
./tools/deploy.sh prune [--prefix /opt/talos] [--keep 3]
```

From a talOS checkout, as shown; from a robot repository's root, the same
script in the talOS checkout its `MODULE.bazel` names (`../talos` beside
the repository, or `talos/` as a submodule):
`../talos/tools/deploy.sh install bazel-bin/robot_bundle`.
A relative `--prefix` is resolved against the working directory.

- **install** refuses a test-only bundle, one whose `bundle_manifest.toml`
  says `testonly = true` (a `talos_bundle` with `testonly = True`, which may
  carry test fixtures), before writing anything: it exits 1 with
  `deploy: error: <bundle> is a test-only bundle ...`. There is no override
  flag. The one exception is how talOS's own tests install their fixture
  bundles: install accepts a test-only bundle when `TEST_TARGET` is set and
  the prefix lies inside the directory `TEST_TMPDIR` names, as Bazel sets
  both for a test. It then refuses, also before writing anything, a bundle
  whose `#!/usr/bin/env` interpreter (a Python node's `python3`) the boot
  service's `PATH` cannot run ([host interpreters](#host-interpreters)).
  Otherwise it
  copies the bundle to `<prefix>/bundles/<stamp>/` (read-only),
  checks that the copy has an executable `run.sh`, runs
  `sync`, [warms the copy](#warm-up), [checks its graph](#graph-check),
  retains `previous`, and atomically replaces `current` using a temporary
  sibling symlink and rename (Linux and macOS). This selects the next bundle;
  restart the service to activate it. A bundle whose graph the launcher
  refuses, or whose install fails after the copy (a copy without an
  executable `run.sh`, a graph check that cannot create its temporary
  directory), is not installed: its copy is removed and `current` and
  `previous` stay as they were. A `talos_bundle` holds no symlinks, but a bundle Bazel's
  sandbox staged, or one copied with its links, is a tree of links into a
  build tree, which would dangle after a `bazel clean`: install copies a
  link whose target leaves the bundle (an absolute target, a relative one
  that climbs out, or one that resolves outside) as the file or directory
  it names, and says how many in one line, `deploy: copied <n> links that
  leave the bundle as the files they name`; a relative link that stays
  inside stays a link. Before anything is copied, install follows each
  leaving link as the copy would, through every directory it names and
  every link there, and refuses the install, naming each such link
  relative to the bundle (`deploy: error: <path> -> <target>: ...`), when
  one names nothing (dangling, or a loop of links), names neither a
  regular file nor a directory (a FIFO, socket or device), or names a
  directory holding the link itself, the bundle or the install (which the
  copy would follow until it failed or the disk filled); `current` and
  `previous` stay as they were. Stopping `deploy.sh` (Ctrl-C, SIGTERM,
  SIGHUP) at any point before it selects the copy (while it copies, makes
  the copy read-only and syncs it, warms it or checks its graph) stops
  that step's processes, selects nothing, removes the copy and exits 128
  plus the signal's number, so no stray `bundles/<stamp>` is left for
  `prune` to count among the newest.
- **enable-boot** starts `<prefix>/current/run.sh --serve-refusal` at
  boot as `--user` (default `talos`; the account must exist), so a booted
  robot serves the Studio its bundle carries ([above](#build-a-bundle)) and
  serves a phase-1 refusal reason on `/drive` if startup is refused
  ([refusal visibility](../talOS/launcher/README.md#refusal-visibility---serve-refusal)).
  Linux: writes and enables
  `/etc/systemd/system/talos.service` (`Type=notify`, `Restart=always`, a
  three-second supervision watchdog, a stop through the launcher alone
  ([below](#supervision-and-service-recovery)), `LimitRTPRIO=49` and
  `LimitMEMLOCK=infinity`, so the unprivileged service gets the
  [realtime scheduling](NODE_DEVELOPMENT_GUIDE.md#realtime-scheduling) its
  nodes request). macOS: refuses while FileVault is on (`fdesetup status`
  is anything but `FileVault is Off.`), since the daemon then cannot start
  until someone logs in; sets `pmset -a autorestart 1 sleep 0` (power back on
  after a power loss, never sleep while idle; as defense-in-depth independent of
  `pmset`, the launcher also holds a macOS IOKit `PreventSystemSleep` assertion
  for every running session,
  [sleep prevention](../talOS/launcher/README.md#sleep-prevention)); writes the
  LaunchDaemon `/Library/LaunchDaemons/org.talos.robot.plist` (`KeepAlive`,
  `ProcessType` `Interactive`, stderr to `/Library/Logs/talos/launchd.stderr.log`,
  [below](#boot-service-output)) and the rotation rule
  `/etc/newsyslog.d/org.talos.robot.conf`, bootstraps the daemon into the
  system domain, so it starts without a login, and ends with `boot-check`.
  `ProcessType` `Interactive` gives the launcher an app's resource limits,
  that is none; without the key launchd applies light limits that throttle
  the job's CPU and I/O (`man launchd.plist`), `Background` throttles harder,
  and `Adaptive` follows XPC activity the launcher has none of.
  Both use sudo. Before enabling, it checks bundle read/traversal access as
  the selected account, with the `python3` the boot service's `PATH`
  resolves ([host interpreters](#host-interpreters)), and creates `<prefix>/logs` owned by that account,
  mode 0755. Bundles stay read-only. An inaccessible bundle fails clearly;
  fix ancestor traversal/read permissions rather than granting bundle writes.
  `--dry-run` prints these provisioning/check commands and the unit, or the
  `pmset` command, plist and rotation rule (on macOS reporting, not refusing,
  FileVault).
- **boot-check** (macOS; elsewhere it only says so) checks, read-only and
  without sudo, what a robot that boots with nobody logged in needs: FileVault
  off and `pmset -g` showing `autorestart 1` and `sleep 0`; it exits 1 naming
  each one that does not hold. It also reports the automatic-update settings
  in `/Library/Preferences/com.apple.SoftwareUpdate`, warning when macOS
  updates install on their own (which restarts the Mini); `enable-boot`
  changes neither FileVault nor updates, which a person sets in System
  Settings. `tools/mac_qualify.sh` runs it as a step.
- **run** starts the selected bundle by hand, in the foreground, as the
  boot service starts it: `<prefix>/current/run.sh` with
  `TALOS_LOG_DIR=<prefix>/logs`, every argument after `run` passed to
  `run.sh` and so to the launcher (`deploy.sh run --remote 10.0.0.72
  --duration-s 60` for a RoboRIO on another network; after `--`,
  `deploy.sh`'s own `--prefix`, `--dry-run` and `--no-warm-up` are passed
  on too). On Linux it first takes the boot unit's limits, `ulimit -r 49` and
  `ulimit -l unlimited`, which needs no privilege once the account's hard limits allow
  them ([the one-time grant](INSTALL.md#realtime-grant-for-deploysh-run));
  without it, it starts nothing and exits 1 with one line naming the hard
  limits it found, the grant and, for one terminal only, `sudo prlimit --pid
  <its shell> --rtprio=49:49 --memlock=unlimited:unlimited`. macOS's
  realtime policy needs no limit, so there it just starts. It prints
  `deploy: launcher pid <pid>; its session directory is a new <session
  id>/ under <root> ...` and then execs `run.sh`, which execs the launcher
  in the same process, so that pid is the launcher's and Ctrl-C reaches it
  alone; the launcher prints the session directory as `Output Dir:`. An
  `--output-dir` argument replaces the root, as it does for `run.sh`
  (give an absolute path: `run.sh` changes into the bundle directory, so a
  relative one resolves inside it); a
  root the person cannot write (`enable-boot` gives `<prefix>/logs` to the
  service account) is refused, naming `--output-dir`. It refuses to start
  while the boot service is running the same `run.sh` (`systemctl show
  talos.service`, `launchctl print system/org.talos.robot`, read without
  sudo), naming the command that stops it; one it cannot ask is left to the
  launcher's [one session per namespace](../talOS/launcher/README.md#one-session-per-namespace).
  `--dry-run` prints the `ulimit` (Linux) and the `exec` it would run.
- **logs** lists session directories under `<prefix>/logs` (or `LOG_ROOT`),
  oldest first by the `start_wall_ns` their manifest records (the directory's
  modification time without one; session ids are not ordered in time), each
  with its start, `git_revision` (and `dirty`, plus `talos_git_revision` /
  `talos_git_dirty` when present), the launcher's match
  classification from its `session_class` file (`match`, `other`, `unknown`,
  or `unclassified` before retention has read it; a `--sim` session is never
  classified, so it stays `unclassified`) and its runs counted by
  `reason`. It then summarizes the newest manifest: session id, start, config
  path and hash, revision, and per node its exit code (and the signal that
  ended it, if one did), last run's reason, restarts, event log path and
  output log. It says when there are no sessions yet or the newest session
  directory has no manifest, and prints the last lines of
  `/Library/Logs/talos/launchd.stderr.log` when that file was written after
  the newest session started, as by a launch that failed before
  recording anything. No installed bundle is needed to inspect retained
  sessions.
- **fetch** copies every session folder from `<prefix>/logs` into `DEST`,
  preserving its manifest, every node's event logs (restarted attempts
  included) and output logs.
  Run it on the main processor, for example
  `./tools/deploy.sh fetch /media/usb/robot-sessions`, then transfer that
  directory to the analysis machine. Existing destination session names are
  rejected rather than merged; choose a fresh destination for each collection.
- **rollback** swaps `current` and `previous`: the whole previous bundle, with
  its own config. It refuses when `current` is missing or `previous` no longer
  holds a bundle. It [warms](#warm-up) `previous` before selecting it: one
  installed by a `deploy.sh` without the warm-up and never launched has never
  run. A bundle that has already run needs none, and its warm-up costs about
  10 ms a member. It then runs install's [graph check](#graph-check) on
  `previous` and, when the launcher refuses it (one installed by a
  `deploy.sh` without the check, say), exits 1 with `deploy: error: not
  rolled back: the launcher refuses <bundle>; current stays <bundle>`,
  changing neither link. Selection does not change already-running processes: a
  running launcher restarts nodes from the bundle it started in.
  With the robot disabled, activate install/rollback with
  `sudo systemctl restart talos` on Linux, or
  `sudo launchctl kickstart -k system/org.talos.robot` on macOS.
- **prune** deletes installed bundles except the newest `--keep N` (default 3)
  and the ones `current` and `previous` select, however old. Prune after
  activating the selection, so the running bundle is among those kept.

Run by hand: `./tools/deploy.sh run --duration-s 30` (above), which gives
the launcher the boot service's realtime limits on Linux, or
`/opt/talos/current/run.sh --duration-s 30`, which runs with the caller's
limits. Stop the boot service first (`sudo systemctl stop talos`; on macOS
`sudo launchctl bootout system/org.talos.robot`, and `enable-boot` again
afterwards): only one launcher may run a session in a shared-memory namespace,
so a second is refused with the running one's pid
([one session per namespace](../talOS/launcher/README.md#one-session-per-namespace)).

Boot services set `TALOS_LOG_DIR=<prefix>/logs`. The generated `run.sh` passes
that directory as the launcher's output **root**, unless an explicit
`--output-dir ROOT` is supplied. The launcher appends a unique session id on
every invocation, so recordings live in `<prefix>/logs/<session-id>/` and
restarts of the service never reuse a previous session folder. `deploy.sh run`
sets it the same way; for a manual
run of `run.sh` to use the same root, set `TALOS_LOG_DIR=/opt/talos/logs` or pass
`--output-dir /opt/talos/logs`; `logs LOG_ROOT` can inspect another output root.

Each node's stdout and stderr, every run of it, land in
`<prefix>/logs/<session-id>/<name>.log`, and the launcher's own lines in
`<prefix>/logs/<session-id>/talos.log` ([node output](../talOS/launcher/README.md#node-output)).

For event-log byte budgets, retention, capped recordings, manifest-write
recovery and what is synced to disk when, see [session segments and logs](../talOS/launcher/README.md#session-segments-and-logs).

### Warm-up

macOS charges the first exec, and first load, of every newly written
executable file about 100 ms, one file at a time host-wide; the charge is per
file and paid only by running or loading it, not by reading it. A fresh
install is thousands of new files (2,347 in the Linux bundle of the
two-mechanism robot [FIRST_MECHANISM.md](FIRST_MECHANISM.md) builds, 2,286
of them its Python runtime), and a first launch would pay the charge of
every file it runs or loads inside the launcher's per-probe `--describe`
deadline ([below](#supervision-and-service-recovery)), every probe at once;
a probe that missed its deadline on the Mac mini right after a build is in
[BENCH_RESULTS.md](BENCH_RESULTS.md#2026-11-13) (MAC-2).

So `install` and `rollback` run every member of the bundle they are about to
select once, one at a time, from its final path: each `[[nodes]]` binary of
`bundle_manifest.toml` and the `[studio]` bridge with
`--describe --config <config_path>`, as the launcher's probe runs them, and
the launcher with `--help`. On macOS one more member then loads every
`.so` and `.dylib` in the bundle once, in a process
of its own, ignoring each failure: `--describe` imports no backend, so a
node's first imports (a Python node's numpy, for one) would otherwise still
pay. dyld's first mapping of a file pays its charge
even when the load then fails on an unresolved symbol; reading the file or
`codesign -v` does not. The loaded libraries' static initializers run in that
process, nothing calls into them, and it exits without running a destructor.
Linux skips it. The members run from the bundle directory with `bin/`
first on `PATH`, Bazel's and Python's environment variables unset (as
`run.sh` does), and `PYTHONDONTWRITEBYTECODE=1`, so the read-only bundle
stays as installed even under sudo. A `--describe` run opens no topic,
socket or registry row, so the warm-up touches no hardware and no network
and can run beside a running robot. On the Mac mini it took about 11 s for
the 2026 robot's 14-member bundle, 58 native libraries (about 4.5 s without
them), and the first `--describe-only` after it 0.4 to 0.5 s idle
([BENCH_RESULTS.md](BENCH_RESULTS.md#2026-11-13): DEPLOY-10, MAC-2). A
member that exits nonzero, or is still running after 60 s (then its process group is killed),
is a `deploy: warning: warm-up: <name>: ...` line and the command still
succeeds: the [graph check](#graph-check) that follows judges the members. The warm-up's last line counts what finished, members that
exited 0 and native libraries whose load returned (failing or not), and
names each member that did not with how it ended:
`deploy: warmed 4 of 5 members and 40 of 58 native libraries in 6.2 s; not
warmed: vision (exited 1), native libraries (killed by SIGSEGV)`;
when everything finished it reads `deploy: warmed 5 members and 58 native
libraries in 9.8 s`. A member that detaches a process
into a new session (`setsid()`) leaves the group that kill reaches. Stopping
`deploy.sh` during the warm-up (Ctrl-C, SIGTERM, SIGHUP) kills the running
member's process group, selects nothing and exits 128 plus the signal's
number; `install` also removes its copy. Linux has no such
charge; the warm-up runs there too. `--no-warm-up` skips it. The
[graph check](#graph-check) that `install` and `rollback` run next still
runs every member and the launcher once, all at once inside the probes'
deadline, so the first launch pays only, on macOS, each native library's
first load.

The members run as the user that runs `deploy.sh`, root under `sudo`: the
bundle's code runs at install time with that privilege, so install only a
bundle you would launch.

### Graph check

After the warm-up, `install` runs the copy's own `run.sh --describe-only
--describe-as-launch` with a temporary `--output-dir`: the bundle's
launcher, on the config and with the Studio bridge `run.sh` gives it,
probes every member with `--describe` and lints the graph, the first phase
of every launch ([two phases](../talOS/launcher/README.md#two-phases)), and
exits, refusing too what the boot service's launch (`run.sh` without
`--sim`) refuses before its probes, such as a config that sets `[robot]
simulation_only = true`. Whatever that refuses (a naming or graph rule, a
critical member that does not answer, a parameter key no node reads, a
simulation config) the boot service's launch would
refuse every time it restarts, with nothing running and the driver
dashboard showing only a lost link. When the launcher exits nonzero, for
that or for a host failure such as a full `TMPDIR`, the bundle is not
installed: `deploy.sh` prints `deploy: the launcher's check of this bundle
failed (exit <status>); its output:` and the launcher's output, each line
prefixed `  | `, removes the copy, and exits 1 with

```
deploy: error: <bundle> not installed (the paths above were its removed copy, <prefix>/bundles/<stamp>); current stays <prefix>/bundles/<selected>
```

(`nothing is selected` on a first install). The bundle given still holds
every file the launcher names, at the same path below it. Warnings, from the
launcher or the warm-up, do not refuse; on acceptance the launcher's graph
warnings (its `  warning: ` lines, among them a non-critical member that did
not answer `--describe`, which the launch still starts, and a Studio bridge
that did not, which it leaves out) are printed, prefixed the same way. Stopping `deploy.sh` during the
check stops the launcher, selects nothing, removes the copy and exits 128
plus the signal's number.

The check takes nothing a running robot holds: `--describe-only` takes no
session lock, shared memory, registry row, controller link or realtime
priority, and a `--describe` run opens no device, so it needs neither the
RoboRIO nor a camera and can run beside a live session. It adds one
`--describe-only` to the install. `--no-warm-up` does not skip it;
on macOS a fresh bundle's probes then pay the first-run charge inside their
deadline, as its first launch would, so a probe that times out there is
answered by installing again without `--no-warm-up`. There is no override:
the launcher accepts `--allow-graph-errors` only with `--sim`, so a robot
could never launch such a bundle. `rollback` runs the same check on
`previous` after warming it, and a refusal leaves `current` and `previous`
as they were ([rollback](#on-the-main-processor)); stopping it during the
check selects nothing.

A bundle's launcher built before `--describe-as-launch` (a rollback to an
older bundle) refuses the flag while it parses its arguments, before it
reads the config, printing only `launcher error: unknown argument:
--describe-as-launch`. On exactly that output `deploy.sh` runs the check
again with `--describe-only` alone and warns

```
deploy: warning: this bundle's launcher predates --describe-as-launch: checked with --describe-only alone, so a config that sets [robot] simulation_only = true is not refused
```

Any refusal of the bundle prints other lines, so it is never retried.

The check is not the whole launch. It does not cover:

- The refusals of a launch that need what a running robot holds: another
  launcher holding the [session lock](../talOS/launcher/README.md#one-session-per-namespace)
  (at boot the previous session has exited), a live registry that cannot
  be prepared, and the ports the bridges bind once started
  ([two phases](../talOS/launcher/README.md#two-phases) lists them).
- The boot service's account and environment: the check runs as the person
  running `deploy.sh`, with their environment, while the service runs as
  its account with its own environment. What members find through the
  service's `PATH` is checked separately ([host
  interpreters](#host-interpreters)); anything else a member reads from
  the environment can differ at boot.

### Host interpreters

A bundle needs a host `python3` when it carries a Python node: each Python
node's entry script, `bin/<name>`, starts `#!/usr/bin/env python3`, and
that interpreter only starts the node, which then runs on the bundle's own
runtime in `python/` ([bundle layout](#build-a-bundle)). `run.sh` starts
`#!/usr/bin/env bash`. `env` resolves both on the `PATH` the boot service
starts `run.sh` with, the bundle's `bin/` first and then the service
manager's default, not the installer's: `/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin`
under systemd, `/usr/bin:/bin:/usr/sbin:/sbin` under launchd, where neither
Homebrew's nor a pyenv `python3` is. So `install`, before writing anything,
runs each program that `run.sh`, the launcher, a `bin/` entry or the Studio
bridge names after `#!/usr/bin/env`, once, as `<program> --version` with
only that `PATH`, and refuses the bundle when one does not run, absent or
failing (macOS's `/usr/bin/python3` without the Command Line Tools fails):

```
deploy: error: bin/<name> starts #!/usr/bin/env python3, which the boot service resolves on its PATH, <path>, where python3 does not run; nothing was written to <prefix>. Install python3 in a directory on that PATH (...)
```

The fix is a `python3` on that `PATH`: on macOS `xcode-select --install`
(the Command Line Tools provide the `/usr/bin/python3` launchd finds), on
Linux the distribution's `python3` package. The check runs as the
installer; the service account resolves the same `PATH`, and `enable-boot`
checks that account's access to the bundle.

### Boot service output

The launcher's own output is the record of a launch that fails before it
creates its session directory (a bad config, a held session lock); from that
directory's creation on, a refused graph included, its lines are also in
`<session>/talos.log`. Its errors go to stderr. On Linux both streams go to the journal (`journalctl -u talos`). The
macOS LaunchDaemon sends stderr to `/Library/Logs/talos/launchd.stderr.log`
and stdout (progress lines, already in the manifest, and the uncapped echo
of node output, already in each `<name>.log`) to `/dev/null`. Node output,
including a node's `realtime scheduling incomplete` line, is in its
`<session>/<name>.log`, not this file; the launcher's own warning for a
bridge or control loop without its realtime grant is here (and in
`<session>/talos.log`), and the outcome is
in the [session manifest](../talOS/launcher/README.md#session-manifest). launchd appends to it across
launches without marking where one starts. newsyslog, which macOS runs
hourly as root, renames it once it passes 1024 KiB and keeps four older
copies (`launchd.stderr.log.0` to `.3`), adding no line of its own, so a
crash loop, which launchd restarts at most every 10 s, cannot fill the disk;
a running launcher keeps appending to the copy it opened until it restarts.
newsyslog sets each new file's owner and mode by path, so the file lives in
a root-owned directory (`enable-boot` creates it `root:wheel` 0755 and the
file owned by the service account), never in the account's own log root,
where the account could put a link to a root file in its place.
`deploy.sh logs` shows the file's tail when it is newer than the newest
session.

## Supervision and service recovery

Boot probes each node before spawning the graph. `[subsystem] critical = true`
is the fail-closed default; failed probes refuse startup. Observation-only
nodes may explicitly set `critical = false`: a timeout, crash or import failure
then logs a warning without blocking the rest of the valid graph, and the
node is launched under restart supervision. Motors, digital outputs, PWM
outputs and described target-lane senders cannot be non-critical; the hardware
bridge is always critical. Naming and graph errors remain fatal, as does a
critical node's first run whose
[live endpoints](../talOS/launcher/README.md#live-endpoints) differ from its
probe (a non-critical node's is killed alone). The launcher accepts
`--allow-graph-errors` only with `--sim`, so a deployed launch cannot weaken
the graph checks.
`--describe-timeout-ms MS` adjusts the existing 15000 ms per-probe deadline
and can be passed through `run.sh`.

The Linux unit sets `TimeoutStartSec=1200` (20 minutes), independently of
the running watchdog. Description probes use the per-probe deadline above;
larger graphs or deadline overrides must fit the unit's startup budget.
Watchdog notifications begin only after READY, from the supervision loop,
never an independent heartbeat.

On Linux the launcher sends `READY=1` when the graph has started and enters
supervision, then `WATCHDOG=1` on each supervision iteration, which runs at
least every half of the unit's watchdog interval (1.5 s) and, while nodes
run, at least every quarter of their shortest heartbeat timeout (125 ms at
its 500 ms floor;
[supervision passes](../talOS/launcher/README.md#supervision-passes)).
With `WatchdogSec=3`, systemd treats three seconds without a
notification as a hung launcher and restarts it, leaving ample scheduling
headroom: the loop never waits for a disk sync. A stalled loop cannot be
masked by an independent watchdog thread. `Restart=always` starts a fresh
session after either a watchdog failure or a launcher exit, however often:
`StartLimitIntervalSec=0` lifts systemd's start-rate limit (five starts in ten
seconds by default, manual restarts included). `systemctl stop talos` still
stops the service deliberately. Notifications use `NOTIFY_SOCKET` directly,
including Linux abstract (`@...`) Unix datagram sockets, without libsystemd.

A stop goes through the launcher alone. `KillMode=mixed` sends systemd's
SIGTERM only to the launcher, which stops the session in two phases: first
SIGINT to `hardware_node` (SIGKILL if still alive 500 ms later), then SIGINT to
the remaining members (SIGKILL to any still alive 2 s later; `stop_timeout` in
the [session manifest](../talOS/launcher/README.md#session-manifest)), then the
final manifest. systemd SIGKILLs whatever remains once the launcher exits, or
at `TimeoutStopSec=90`: 90 s against the launcher's up to 2.5 s, so the
launcher's own escalation and its final write always come first. systemd's
default, `KillMode=control-group`, would SIGTERM every member as well, and
a member that does not hold a stop while starting (`hardware_node`, the
simulated gateway, the Studio bridge: none declares `holds_stop`; a
framework node holds it) would die of a signal the launcher did not send,
ending the session 255 and the unit failed. launchd
needs no such setting: stopping the job (`launchctl bootout`) sends SIGTERM
to the launcher's process alone, SIGKILL after `ExitTimeOut` (90 s, set
explicitly in the plist to match systemd's `TimeoutStopSec=90` rather than
relying on launchd's system-defined default; `man launchd.plist`),
and when the launcher exits kills only the launcher's own process group,
which no member is in.

Each node (including description probes and the Studio bridge) owns a process
group. Restart, stop and shutdown signal that whole group, not just its
leader. A per-node guardian watches a launcher-owned lifetime pipe and kills
the group when the launcher dies, even on SIGKILL; Linux additionally sets
`PR_SET_PDEATHSIG` on the node. macOS uses the same lifetime-pipe protection,
but has no `PR_SET_PDEATHSIG` or systemd watchdog: launchd's `KeepAlive`
restarts an exited launcher, not a hung one. Nodes and helpers must remain in
their assigned process group rather than daemonizing into another session.

Critical nodes never enter quarantine: crashes and stalls keep
restarting on the capped [restart backoff](../talOS/launcher/README.md#restarts),
even after exhausting the rolling budget, and only a critical node's first
crash or stall in the rolling window restarts it at once. Non-critical nodes
retain the circuit breaker: exhausting their budget quarantines them until the
oldest restart ages out, plus normal backoff.
Exactly one retry is pending, so clearing a transient fault can recover without a service
restart. The session manifest records `quarantined` and
`retry_at_monotonic_ns`; a stopping session cancels pending retries.
`--max-restarts 0` never restarts a node; `[robot] max_restarts` must be at
least 1. Registry version 6
(`/talos_registry.6`) uses host-monotonic heartbeat/liveness timestamps, so
wall-clock corrections cannot falsely evict healthy nodes; deploy all node
and viewer binaries together. Run one session's nodes in one pid namespace: a
node in another (a container sharing `/dev/shm` with `--ipc=host`) cannot be
restarted, because its dead topic claims stay until the session ends. Keep
them in one time namespace too: Linux shows process start times shifted by the
reader's time namespace, so a node in another would judge live claims dead.

## RoboRIO

`controller_processor/rio` is built and deployed with GradleRIO. talOS has no
built-in team number or companion address:

- Every RoboRIO target build (`buildRio` or `deploy`) requires
  `-Pcompanion=<ipv4>` on the command line — the main processor's IPv4 address
  (port 5803; refused from `gradle.properties` or the environment,
  [RoboRIO](../talOS/hardware/README.md#roborio)). Desktop `phoenixTest` and
  `simulateNative` use `127.0.0.1` and need neither `-Pcompanion` nor a team
  number.
- `deploy` also requires a team number: pass `-PteamNumber=<team>` on the
  command line, or provide `.wpilib/wpilib_preferences.json` with
  `"teamNumber"` in the robot repository.

From a robot repository, build and deploy through Bazel without touching the
pinned talOS checkout:

```sh
bazel run @talos//controller_processor/rio:gradlew -- buildRio -Pcompanion=<ipv4>
bazel run @talos//controller_processor/rio:gradlew -- deploy -Pcompanion=<ipv4>                 # uses .wpilib/wpilib_preferences.json
bazel run @talos//controller_processor/rio:gradlew -- deploy -PteamNumber=<team> -Pcompanion=<ipv4>
```

`@talos//controller_processor/rio:gradlew` stages the Gradle project and the
`talOS/` sources it compiles into a cache directory (`$XDG_CACHE_HOME/talos/rio`
or `~/.cache/talos/rio`, keyed by both the talOS root and the calling workspace
and preferences so concurrent robot repositories sharing one talOS checkout do
not collide, and serialized with an exclusive lock across staging and Gradle),
computes `-PtalosRevision` and `-PtalosDirty` from the pinned talOS git work
tree before copying (refusing caller-supplied `talosRevision` or `talosDirty`
overrides when a git checkout stamp exists), and forwards the robot
repository's `.wpilib/wpilib_preferences.json` (if present). From a talOS
checkout directly, run `(cd controller_processor/rio && ./gradlew deploy
-PteamNumber=<team> -Pcompanion=<ipv4>)` (or `bazel run
//controller_processor/rio:gradlew -- ...`).

Deployed from a copy with no talOS git history — including when a robot
repository fetches talOS via `git_override` or an archive (so Bazel's external
`@talos` repository contains no `.git` directory), or a copy placed inside an
unrelated git repository whose work-tree root does not match the talOS root —
`@talos//controller_processor/rio:gradlew` builds with revision `0`
(`"unknown"`) unless `-PtalosRevision=<first 16 hex digits of its commit>
-PtalosDirty=true|false` (whether the copy differs from that commit) is passed
on the command line; inside a talOS git checkout itself (including
`local_path_override` or a git submodule), both the runner and `build.gradle`
refuse those overrides ([build stamp](../talOS/hardware/README.md#roborio)).
Deploy it from the talOS checkout the main-processor bundle was built
against (the one a robot repository's `MODULE.bazel` names): each side
rejects frames of another protocol version (currently 12; see
[DATA_FLOW.md](DATA_FLOW.md)), and `hardware_node` logs both versions once
when that happens; the RoboRIO reports it in the Driver Station message log
(a `talos companion link` error naming both versions,
[RoboRIO](../talOS/hardware/README.md#roborio)), and the driver dashboard's
status line shows the RoboRIO link lost.
Robot bring-up checklist: [BRINGUP.md](BRINGUP.md); before each match:
[PRE_MATCH.md](PRE_MATCH.md).
