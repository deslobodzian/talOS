"""Installed bundles describe their graph and keep their config identity.

Each bundle is installed the way the robot gets it (tools/deploy.sh install
into a fresh prefix), then --describe runs every node binary from there, so a
C++ node that cannot load a shared library it links, a
Python node that cannot import its wheels or start its interpreter, or an
install that breaks the bundle's relative symlinks fails here. The bench
rig's bundle, which link and stop qualification install, is checked too.
A bundle that carries Studio is described by the command the boot service
runs (deploy.sh enable-boot's systemd ExecStart and launchd
ProgramArguments), which must run its bridge from the install, found through
the manifest's [studio] table, beside the index page of the webroot that
table names; one that does not is asked for Studio with
--start-studio-bridge and must describe without a bridge rather than fail on
its absence."""

import json
import os
import plistlib
import shlex
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
    `args`. Returns the process, the session's graph and the installed
    bundle's directory."""
    subprocess.run(["bash", deploy_script(), "install", bundle,
                    "--prefix", prefix],
                   check=True, capture_output=True, text=True, timeout=120)
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
        return proc, None, where
    with open(graph, encoding="utf-8") as f:
        return proc, json.load(f), where


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
                proc, graph, where = describe(src, prefix, command, *args)
                out = proc.stdout + "\n" + proc.stderr
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


if __name__ == "__main__":
    unittest.main()
