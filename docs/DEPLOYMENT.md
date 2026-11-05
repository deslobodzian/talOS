# Deployment

The main processor runs a **bundle**: one directory with every node binary, the
launcher, the config and the runtime libraries. `tools/deploy.sh` installs
bundles, starts one on boot, shows logs and rolls back.

## Build a bundle

```sh
bazel build //2026-robot/main_processor:robot_bundle
```

`talos_bundle` (`talOS/build/contract.bzl`) takes `nodes`, `config`
(`robot_field.toml`) and `config_files`; the robot's BUILD passes `NODES` and
`CONFIG_FILES` from the generated `configuration/robot_field_manifest.bzl`
(`bazel run //tools:sync_manifest -- <robot_field.toml>`). It writes
`bazel-bin/2026-robot/main_processor/robot_bundle/`:

| Entry | What |
|---|---|
| `run.sh` | runs `./launcher --bundle <dir> --config config/robot_field.toml "$@"` |
| `launcher` | `//talOS/launcher` |
| `bin/` | node binaries; a Python node's runfiles beside it (its wheels, e.g. `robotpy-wpimath` and `gtsam`, and `libtalos_node.so`) |
| `lib/` | the shared libraries C++ nodes link (`libwpimath`, `libwpiutil`), one copy per name; `talos_cc_node` binaries carry the rpath `@loader_path/../lib` (macOS) or `$ORIGIN/../lib` (Linux) |
| `config/` | `robot_field.toml` and every file it reads (`include`, `path`, `defaults`), at the same paths relative to each other as in the source tree |
| `bundle_manifest.toml` | node names, targets, binaries, languages, config SHA-256 |

The config SHA-256 (`config_sha256`, copied into every session manifest) is
the identity of the staged config: every file under `config/`, the robot
config included, contributes its path relative to `config/` and its bytes.
Absolute and workspace locations never participate, so moving the whole
config tree or copying the bundle elsewhere keeps the identity; editing,
renaming or moving any one file relative to the others (the root config too)
changes it.

The bundle uses `2026-robot/main_processor/configuration/robot_field.toml`,
which includes `robot.toml` and overrides only `[robot] controller_address`
to the RoboRIO's `10.56.87.2`. Simulation and its tests keep using
`robot.toml` with loopback (`127.0.0.1`); no source edits are needed when
switching between simulation and the field bundle. The overlay does not
change CAN ids or commissioning: hardware must still be checked and
commissioned through the normal bring-up process.

To bring up the bundle against a different controller address without editing
the config, run `./run.sh --remote <ip>` (for example,
`./run.sh --remote 10.56.87.3`). This overrides the effective address for that
launch; use `--remote-port` and `--local-port` if the ports also differ.

## On the main processor

```sh
./tools/deploy.sh install <bundle> [--prefix /opt/talos]
./tools/deploy.sh enable-boot [--prefix /opt/talos] [--user talos] [--dry-run]
./tools/deploy.sh logs [--prefix /opt/talos] [LOG_ROOT]
./tools/deploy.sh fetch [--prefix /opt/talos] DEST
./tools/deploy.sh rollback [--prefix /opt/talos]
```

- **install** copies the bundle to `<prefix>/bundles/<stamp>/` (read-only,
  symlinks kept), moves `current` to `previous`, and points `current` at the
  new one.
- **enable-boot** starts `<prefix>/current/run.sh` at boot as `--user`
  (default `talos`; the account must exist). Linux: writes and enables
  `/etc/systemd/system/talos.service` (restart on failure). macOS: writes the
  LaunchDaemon `/Library/LaunchDaemons/org.talos.robot.plist` (`KeepAlive`)
  and bootstraps it into the system domain, so it starts without a login.
  Both use sudo. `--dry-run` prints the unit or plist instead.
- **logs** lists session directories under `<prefix>/logs` (or `LOG_ROOT`) and
  summarizes the newest manifest by modification time: session id, config path
  and hash, each node's exit code and log path. No installed bundle is needed
  to inspect retained sessions.
- **fetch** copies every session folder from `<prefix>/logs` into `DEST`,
  preserving its manifest and all node logs, including restarted attempts.
  Run it on the main processor, for example
  `./tools/deploy.sh fetch /media/usb/robot-sessions`, then transfer that
  directory to the analysis machine. Existing destination session names are
  rejected rather than merged; choose a fresh destination for each collection.
- **rollback** swaps `current` and `previous`: the whole previous bundle, with
  its own config.

Run by hand: `/opt/talos/current/run.sh --duration-s 30`.

Boot services set `TALOS_LOG_DIR=<prefix>/logs`. The generated `run.sh` passes
that directory as the launcher's output **root**, unless an explicit
`--output-dir ROOT` is supplied. The launcher appends a unique session id on
every invocation, so recordings live in `<prefix>/logs/<session-id>/` and
restarts of the service never reuse a previous session folder. For a manual
run to use the same root, set `TALOS_LOG_DIR=/opt/talos/logs` or pass
`--output-dir /opt/talos/logs`; `logs LOG_ROOT` can inspect another output root.

## RoboRIO

`2026-robot/controller_processor/rio` is deployed with GradleRIO
(`./gradlew deploy` in that directory). Set `kCompanionAddress` in
`src/main/cpp/Robot.cpp` to the main processor's address first.
Robot bring-up checklist: [BRINGUP.md](BRINGUP.md).
