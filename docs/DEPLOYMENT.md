# Deployment

The main processor runs a **bundle**: one directory with every node binary, the
launcher, the config and the runtime libraries. `tools/deploy.sh` installs
bundles, starts one on boot, shows logs and rolls back.

## Build a bundle

```sh
bazel build //2026-robot/main_processor:robot_bundle
```

`talos_bundle` (`talOS/build/contract.bzl`) takes `nodes`, `config`
(`robot.toml`) and `subsystems` (the `subsystems/*.toml` files) and writes
`bazel-bin/2026-robot/main_processor/robot_bundle/`:

| Entry | What |
|---|---|
| `run.sh` | runs `./launcher --bundle <dir> --config config/robot.toml "$@"` |
| `launcher` | `//talOS/launcher` |
| `bin/` | node binaries and their runfiles |
| `lib/libtalos_node.so` | runtime for Python nodes |
| `config/` | `robot.toml` plus one `<name>.toml` per subsystem |
| `bundle_manifest.json` | node names, targets, languages, config SHA-256 |

Before deploying, set `[robot] controller_address` in the config to the RoboRIO
(the shipped value is loopback for simulation) and `commissioned = true` once
the hardware is checked; or pass `--remote <ip>` to `run.sh`.

## On the main processor

```sh
./tools/deploy.sh install <bundle> [--prefix /opt/talos]
./tools/deploy.sh enable-boot [--prefix /opt/talos] [--user talos] [--dry-run]
./tools/deploy.sh logs [--prefix /opt/talos] [dir]
./tools/deploy.sh rollback [--prefix /opt/talos]
```

- **install** copies the bundle to `<prefix>/bundles/<stamp>/` (read-only),
  moves `current` to `previous`, and points `current` at the new one.
- **enable-boot** writes `/etc/systemd/system/talos.service` running
  `<prefix>/current/run.sh` (restart on failure) and enables it. `--dry-run`, or
  macOS, prints the unit instead.
- **logs** prints the session manifest (session id, config path and hash, each
  node's exit code and log path) found under `current` (or `dir`), or lists the
  directory if there is none.
- **rollback** swaps `current` and `previous`: the whole previous bundle, with
  its own config.

Run by hand: `/opt/talos/current/run.sh --duration-s 30`.

## RoboRIO

`2026-robot/controller_processor/rio` is deployed with GradleRIO
(`./gradlew deploy` in that directory). Set `kCompanionAddress` in
`src/main/cpp/Robot.cpp` to the main processor's address first.
