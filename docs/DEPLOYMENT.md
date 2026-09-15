# talOS deployment guide

Ship a whole robot as one directory: binaries, configuration, and the
provenance that says which is which. Deployments roll forward and back as
whole bundles with their matching configuration and runtime, never by
mixing node versions across bundles.

All commands below are executed by `tools/delivery/walkthrough_test.py`
(scaffold/launcher flags against the real `--help` output, deploy/readiness
flags against the real CLIs), so a stale command fails the build, not the
pit crew.

## Example vs competition configuration

Two configuration kinds appear in this repo; they are not interchangeable:

- **Example config** (scaffolded `subsystem.toml`, `testdata/robot.toml` in a
  fresh `tools/node_scaffold` project, `test/fixtures/*`): nonzero placeholder
  device IDs for local `bazel test //...`. Never deploy these.
- **Competition config** (`2026-robot/main_processor/configuration/robot.toml`
  plus `subsystems/*.toml`): the physical CAN IDs, buses, inversions, gear
  ratios, and gains of the real robot. Only bundles assembled from this tree
  go on the companion.

Every `subsystem.toml` snippet in this guide is marked. Unmarked TOML is an
example.

## 1. Assemble the bundle (competition config)

```bash
bazel build //2026-robot/main_processor:robot_bundle
```

The `talos_bundle` rule stages the launcher, every node binary with its
runtime closure, the competition `robot.toml` plus each `subsystems/*.toml`,
and writes `bundle_manifest.json`: per-node target/binary/contract labels
and `config_sha256` over the robot config plus every subsystem file (names
mixed in, so a rename is an identity change).

Record the deployment (item 57) alongside the bundle:

```bash
python3 tools/delivery/compat_record.py --bundle bazel-bin/2026-robot/main_processor/robot_bundle \
    --out deploy_record.json --vendor vendor_expectations.json
python3 tools/delivery/compat_record.py --bundle bazel-bin/2026-robot/main_processor/robot_bundle --check
```

`--check` refuses a bundle whose configuration was hand-edited after
assembly (hash mismatch), that lost a node binary, or that runs an unknown
`bundle_version`. A missing contract label is a warning, and only valid for
schema-less nodes (e.g. the arbiter): provenance is traceability, not a
compatibility proof.

## 2. Install

```bash
./tools/delivery/deploy.sh install <bundle> --prefix /opt/talos
```

Verifies the manifest hash first, copies the bundle to
`/opt/talos/bundles/<timestamp>/`, and flips the `current` symlink,
keeping `previous` for rollback. `--dry-run` prints every step instead.

## 3. Networking

The companion talks to the RoboRIO gateway over UDP (5802 on the gateway,
5803 on the companion). The only network touch in the tooling is this probe:

```bash
./tools/delivery/deploy.sh check-network --controller 10.56.87.2 --port 5802
```

Everything else — install, verify, run — works with the cable unplugged.
The launch itself takes the endpoint from the competition config
(`[robot] controller_address`, overrides `--remote`, `--remote-port`,
`--local-port`):

```toml
# competition config: 2026-robot/main_processor/configuration/robot.toml
[robot]
controller_address = "10.56.87.2"
controller_port = 5802
local_port = 5803
```

## 4. Boot start

```bash
./tools/delivery/deploy.sh enable-boot --prefix /opt/talos
```

Installs a systemd unit that runs `/opt/talos/current/run.sh` on boot with
restart-on-failure. `--dry-run` prints the unit; on macOS (no systemd) the
`ExecStart` line goes into a launchd plist instead.

## 5. Run and inspect logs

```bash
/opt/talos/current/run.sh
/opt/talos/current/run.sh -- --duration-s 30
./tools/delivery/deploy.sh logs --prefix /opt/talos
```

`logs` prints the session manifest first (session id, config path and hash),
then each node's exit code, failure flag, and record count with its log
path — the same fields the readiness report consumes.

## 6. Roll back

```bash
./tools/delivery/deploy.sh rollback --prefix /opt/talos
```

Swaps `current` back to `previous`: the whole previous bundle with its
matching configuration and runtime. Diagnose from the kept logs, fix by
rebuilding the bundle, never by mixing node versions.

## 7. Offline startup (item 58)

```bash
./tools/delivery/deploy.sh verify-offline <bundle>
```

Fails when the launch path references the network (`curl`/`wget`) or the
build tree (`bazel-bin`), then re-runs the manifest `--check`. An installed
bundle starts from its own files: `run.sh` sets `LD_LIBRARY_PATH` and
`TALOS_NODE_LIB` from the bundle's `lib/` and needs no network, no Bazel,
no source checkout. For air-gapped event builds, vendor the module closure
first (`bazel vendor --vendor_dir=vendor`, then
`bazel build --vendor_dir=vendor //2026-robot/main_processor:robot_bundle`).

## 8. Pre-flight and readiness

```bash
bazel run //talOS/launcher:launcher -- --config 2026-robot/main_processor/configuration/robot.toml --describe-only
python3 tools/delivery/readiness.py --config 2026-robot/main_processor/configuration/robot.toml \
    --bundle /opt/talos/current --vendor vendor_expectations.json
```

`--describe-only` asks every binary what it is, lints the declared graph,
and refuses on errors unless `--allow-graph-errors` (competition-queue
override: printed errors, human decision). `readiness.py` reports the four
dimensions — processes, hardware, config, feedback — plus vendor/firmware,
each with specific inhibit reasons; exit 0 is READY, exit 1 names what
blocks. `docs/READINESS.md` is captured output of this tool; regenerate it,
never hand-edit it.
