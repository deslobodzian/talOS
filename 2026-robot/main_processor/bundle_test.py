"""Installed bundles describe their graph and keep their config identity.

Each bundle is installed the way the robot gets it (tools/deploy.sh install
into a fresh prefix, which runs every member once without a warning), then
--describe runs every node binary from there, so a
C++ node that cannot load a shared library it links, a
Python node that cannot import its wheels or start its interpreter (as
when Bazel restored the bundle from its cache with links copied) fails
here. The bench
rig's bundle, which link and stop qualification install, is checked too.
A bundle that carries Studio is described by the command the boot service
runs (deploy.sh enable-boot's systemd ExecStart and launchd
ProgramArguments), which must run its bridge from the install, found through
the manifest's [studio] table, beside the index page of the webroot that
table names; one that does not is asked for Studio with
--start-studio-bridge and must describe without a bridge rather than fail on
its absence. Install's warm-up is checked on a bundle of stubs that record
each run.

On macOS each newly written executable's first exec costs about 100 ms, and
seconds under parallel load (TODO.md MAC-2), and every installed stub is a
new file. So a stub runs under the default 60 s bound, except in the one
case that waits out a short bound: there every member has already run once
from its installed path, so only the stub that never exits can reach it."""

import json
import os
import plistlib
import select
import shlex
import signal
import subprocess
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles

BUNDLES = (
    "2026-robot/main_processor/robot_bundle",
    "2026-robot/main_processor/relocation_bundle",
    "bench/main_processor/bench_bundle",
)


def deploy_script():
    return runfiles.Create().Rlocation("_main/tools/deploy.sh")


def boot_command(prefix, system):
    """The argv `deploy.sh enable-boot --dry-run` writes into the boot
    service on `system` ("Linux": the systemd unit's ExecStart; "Darwin":
    the LaunchDaemon's ProgramArguments), with uname and FileVault faked."""
    with tempfile.TemporaryDirectory() as fake:
        for name, body in (("uname", f"echo {system}"),
                           ("fdesetup", "echo 'FileVault is Off.'")):
            with open(os.path.join(fake, name), "w", encoding="utf-8") as f:
                f.write(f"#!/bin/sh\n{body}\n")
            os.chmod(os.path.join(fake, name), 0o755)
        env = dict(os.environ, PATH=fake + os.pathsep + os.environ["PATH"])
        out = subprocess.run(
            ["bash", deploy_script(), "enable-boot", "--dry-run",
             "--prefix", prefix],
            check=True, capture_output=True, text=True, env=env,
            timeout=60).stdout
    if system == "Darwin":
        plist = plistlib.loads(
            out[out.index("<?xml"):out.index("</plist>") + 8].encode())
        return plist["ProgramArguments"]
    line = next(line for line in out.splitlines()
                if line.startswith("ExecStart="))
    return shlex.split(line.removeprefix("ExecStart="))


def describe(bundle, prefix, command, *args):
    """Installs `bundle` under `prefix` with deploy.sh and runs `command`
    (an argv starting with the installed run.sh) with --describe-only and
    `args`. Returns the process, the session's graph, the installed
    bundle's directory and the install's stderr."""
    install = subprocess.run(["bash", deploy_script(), "install", bundle,
                              "--prefix", prefix], check=True,
                             capture_output=True, text=True, timeout=120)
    where = os.path.join(prefix, "current")
    log_dir = os.path.join(prefix, "logs")
    env = dict(os.environ, TALOS_LOG_DIR=log_dir)
    proc = subprocess.run([*command, "--describe-only", *args], cwd=where,
                          env=env, capture_output=True, text=True, timeout=120)
    sessions = sorted(
        (os.path.join(log_dir, name) for name in os.listdir(log_dir)
         if os.path.isdir(os.path.join(log_dir, name))),
        key=os.path.getmtime, reverse=True)
    graph = os.path.join(sessions[0], "graph.json") if sessions else ""
    if not os.path.isfile(graph):
        return proc, None, where, install.stderr
    with open(graph, encoding="utf-8") as f:
        return proc, json.load(f), where, install.stderr


class BundleTest(unittest.TestCase):
    def test_installed_bundles_describe_with_a_stable_identity(self):
        r = runfiles.Create()
        for name in BUNDLES:
            # deploy.sh installs read-only, so cleanup cannot remove it all;
            # Bazel clears TEST_TMPDIR.
            with self.subTest(name), tempfile.TemporaryDirectory(
                    ignore_cleanup_errors=True) as tmp:
                src = r.Rlocation(f"_main/{name}")
                self.assertTrue(src and os.path.isdir(src), src)
                with open(os.path.join(src, "bundle_manifest.toml"), "rb") as f:
                    manifest = tomllib.load(f)
                studio = manifest.get("studio")
                prefix = os.path.join(tmp, "prefix")
                if studio:
                    # REQ-OPS-34: a booted robot serves Studio. Both boot
                    # services run the same command, with no argument that
                    # asks for or declines the bridge.
                    command = boot_command(prefix, "Linux")
                    self.assertEqual(command, boot_command(prefix, "Darwin"))
                    args = ()
                else:
                    # Asked for Studio, a bundle without it still describes.
                    command = [os.path.join(prefix, "current", "run.sh")]
                    args = ("--start-studio-bridge",)
                proc, graph, where, installed = describe(
                    src, prefix, command, *args)
                out = proc.stdout + "\n" + proc.stderr
                # Install ran every member the way the probe will, so each
                # one's first exec is paid before the first launch.
                self.assertNotIn("warning: warm-up", installed)
                self.assertEqual(proc.returncode, 0, out)
                self.assertIsNotNone(graph, out)
                # The bundle holds exactly the nodes its config launches, so
                # the graph lists each of them, and its Studio bridge.
                self.assertEqual(len(graph["nodes"]),
                                 len(manifest["nodes"]) + bool(studio), out)
                if studio:
                    self.assertIn("studio_bridge",
                                  [node.get("name") for node in graph["nodes"]],
                                  out)
                    self.assertTrue(os.path.isfile(os.path.join(
                        where, studio["webroot"], "index.html")), studio)
                for node in graph["nodes"]:
                    self.assertTrue(node.get("described"),
                                    f"{node.get('name')}: {node.get('describe_error')}\n{out}")
                # The copy keeps the identity the bundle was built with.
                self.assertTrue(manifest["config_sha256"])
                self.assertEqual(graph["config_sha256"],
                                 manifest["config_sha256"])


# Each stub records, under its installed path, "start" with its pid, what
# $STUB_PREFIX/current selected then ("-" for nothing) and its argv before
# acting, and "end" after, so the log orders every warm-up run.
STUB = """#!/usr/bin/env bash
me="$(cd "$(dirname "$0")" && pwd -P)/$(basename "$0")"
current="$(readlink "$STUB_PREFIX/current" || echo -)"
printf 'start %s %s %s %s\\n' "$me" "$$" "$current" "$*" >> "$STUB_LOG"
case "$(basename "$0")" in
  fails) printf 'end %s\\n' "$me" >> "$STUB_LOG"; exit 3 ;;
  hangs)
    # STUB_PREWARM: run once to pay a first exec, then exit. STUB_FIFO: say
    # it is running.
    [ -z "${STUB_PREWARM:-}" ] || exit 0
    [ -z "${STUB_FIFO:-}" ] || echo up > "$STUB_FIFO"
    exec tail -f /dev/null ;;
esac
printf 'end %s\\n' "$me" >> "$STUB_LOG"
"""

def stub_bundle(root, nodes):
    """A bundle laid out as talos_bundle writes one, every executable a
    stub: `nodes` (`fails` exits 3, `hangs` never exits, any other exits
    0), a Studio bridge, the launcher and run.sh."""
    for sub in ("bin", "config", "studio"):
        os.makedirs(os.path.join(root, sub))
    with open(os.path.join(root, "config", "robot.toml"), "w",
              encoding="utf-8") as f:
        f.write("[robot]\n")
    for rel in [f"bin/{n}" for n in nodes] + [
            "studio/studio_bridge", "launcher", "run.sh"]:
        with open(os.path.join(root, rel), "w", encoding="utf-8") as f:
            f.write(STUB)
        os.chmod(os.path.join(root, rel), 0o755)
    lines = ['bundle_version = 1', 'config_path = "config/robot.toml"',
             'config_sha256 = "0"', '', '[studio]',
             'bridge = "studio/studio_bridge"', 'bridge_sha256 = "0"',
             'targets = ["//studio/bridge:studio_bridge"]',
             'webroot = "studio/webroot"']
    for n in nodes:
        lines += ['', '[[nodes]]', f'name = "{n}"',
                  f'targets = ["//x:{n}"]', f'binary = "bin/{n}"',
                  'binary_sha256 = "0"', 'language = "cpp"']
    with open(os.path.join(root, "bundle_manifest.toml"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


class WarmUpTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.addCleanup(tmp.cleanup)
        self.tmp = tmp.name
        self.prefix = os.path.join(self.tmp, "prefix")
        self.log = os.path.join(self.tmp, "stub.log")
        # The default bound: no fresh stub is killed for its first exec.
        self.env = dict(os.environ, STUB_LOG=self.log, STUB_PREFIX=self.prefix)
        self.env.pop("TALOS_DEPLOY_WARM_UP_TIMEOUT_S", None)

    def deploy(self, *args):
        proc = subprocess.run(
            ["bash", deploy_script(), *args, "--prefix", self.prefix],
            capture_output=True, text=True, env=self.env, timeout=60)
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        return proc

    def install(self, name, nodes, *args):
        src = os.path.join(self.tmp, name)
        stub_bundle(src, nodes)
        return self.deploy("install", src, *args)

    def records(self):
        if not os.path.exists(self.log):
            return []
        with open(self.log, encoding="utf-8") as f:
            return [line.split(" ", 4) for line in f.read().splitlines()]

    def assert_warmed_in_turn(self, bundle, nodes, selected, proc):
        """Every member of the installed `bundle` ran exactly once, one at a
        time, while `current` still selected `selected`, and nothing else
        ran; a member that failed or hung is named in a warning."""
        records = self.records()
        out = proc.stdout + proc.stderr
        describe = "--describe --config config/robot.toml"
        # The installed files (the macOS charge is per file), never run.sh.
        started = sorted((r[1], r[4] if len(r) > 4 else "")
                         for r in records if r[0] == "start")
        want = sorted(
            [(os.path.join(bundle, "bin", n), describe) for n in nodes]
            + [(os.path.join(bundle, "studio", "studio_bridge"), describe),
               (os.path.join(bundle, "launcher"), "--help")])
        self.assertEqual(started, want, out)
        hung_pids = []
        for i, r in enumerate(records):
            if r[0] != "start":
                continue
            self.assertEqual(r[3], selected, records)
            if os.path.basename(r[1]) == "hangs":
                # Killed before the next starts, its pid gone by the return.
                hung_pids.append(int(r[2]))
                self.assertNotEqual(records[i + 1:i + 2], [["end", r[1]]])
                continue
            self.assertEqual(records[i + 1][:2], ["end", r[1]], records)
        self.assertEqual(len(hung_pids), "hangs" in nodes, records)
        for pid in hung_pids:
            with self.assertRaises(ProcessLookupError, msg="hung member alive"):
                os.kill(pid, 0)
        for name in ("fails", "hangs"):
            self.assertEqual(name in proc.stderr, name in nodes, out)

    # REQ-OPS-47
    def test_install_runs_each_member_once_in_turn_before_selecting(self):
        nodes = ("alpha", "fails", "omega")
        proc = self.install("bundle", nodes)
        bundle = os.path.realpath(os.path.join(self.prefix, "current"))
        self.assert_warmed_in_turn(bundle, nodes, "-", proc)

    # REQ-OPS-47
    def test_no_warm_up_runs_nothing(self):
        self.install("bundle", ("alpha",), "--no-warm-up")
        self.assertEqual(self.records(), [])

    # REQ-OPS-47
    def test_rollback_warms_the_previous_bundle_before_selecting_it(self):
        # `previous` as a deploy.sh that predates the warm-up leaves it.
        nodes = ("alpha", "fails")
        self.install("old", nodes, "--no-warm-up")
        old = os.readlink(os.path.join(self.prefix, "current"))
        self.install("new", nodes, "--no-warm-up")
        new = os.readlink(os.path.join(self.prefix, "current"))
        proc = self.deploy("rollback")
        self.assertEqual(os.readlink(os.path.join(self.prefix, "current")),
                         old)
        self.assert_warmed_in_turn(os.path.realpath(old), nodes, new, proc)


    # REQ-OPS-47
    def test_a_hung_member_is_killed_at_its_bound(self):
        # Every member of `previous` has run once from its installed path,
        # so within the short bound only `hangs` is slow.
        nodes = ("alpha", "hangs", "omega")
        self.install("old", nodes, "--no-warm-up")
        old = os.path.realpath(os.path.join(self.prefix, "current"))
        prewarm = dict(self.env, STUB_PREWARM="1",
                       STUB_LOG=os.path.join(self.tmp, "prewarm.log"))
        for rel in [f"bin/{n}" for n in nodes] + [
                "studio/studio_bridge", "launcher"]:
            subprocess.run([os.path.join(old, rel)], env=prewarm, check=True,
                           timeout=60)
        self.install("new", nodes, "--no-warm-up")
        new = os.readlink(os.path.join(self.prefix, "current"))
        self.env["TALOS_DEPLOY_WARM_UP_TIMEOUT_S"] = "3"
        proc = self.deploy("rollback")
        self.assert_warmed_in_turn(old, nodes, new, proc)

    # REQ-OPS-47
    def test_a_stopped_warm_up_kills_its_member_and_selects_nothing(self):
        src = os.path.join(self.tmp, "bundle")
        stub_bundle(src, ("alpha", "hangs", "omega"))
        fifo = os.path.join(self.tmp, "running")
        os.mkfifo(fifo)
        # Read and write: the stub's open never blocks, and no EOF wakes us.
        reader = os.open(fifo, os.O_RDWR | os.O_NONBLOCK)
        self.addCleanup(os.close, reader)
        deploy = subprocess.Popen(
            ["bash", deploy_script(), "install", src, "--prefix", self.prefix],
            env=dict(self.env, STUB_FIFO=fifo), stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True)
        self.addCleanup(deploy.kill)
        ready, _, _ = select.select([reader], [], [], 60)
        self.assertTrue(ready, "the hung member never started")
        deploy.send_signal(signal.SIGTERM)
        out, err = deploy.communicate(timeout=60)
        self.assertNotEqual(deploy.returncode, 0, out + err)
        self.assertFalse(os.path.lexists(os.path.join(self.prefix, "current")),
                         out + err)
        records = self.records()
        started = [os.path.basename(r[1]) for r in records if r[0] == "start"]
        self.assertEqual(started, ["studio_bridge", "alpha", "hangs"], records)
        with self.assertRaises(ProcessLookupError, msg="hung member alive"):
            os.kill(int(records[-1][2]), 0)

if __name__ == "__main__":
    unittest.main()
