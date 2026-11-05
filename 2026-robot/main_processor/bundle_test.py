"""Installed bundles describe their graph and keep their config identity.

Each bundle is installed the way the robot gets it (tools/deploy.sh install
into a fresh prefix), then --describe runs every node binary from there, so a
C++ node that cannot load a shared library it links (libwpimath from lib/), a
Python node that cannot import its wheels or start its interpreter, or an
install that breaks the bundle's relative symlinks fails here."""

import json
import os
import subprocess
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles

BUNDLES = ("robot_bundle", "relocation_bundle")
# The 2026 drivetrain links wpimath (//third_party/wpilib:wpimath).
SHARED_LIBS = {"robot_bundle": ("libwpimath", "libwpiutil")}


def describe(bundle, prefix):
    """Installs `bundle` under `prefix` with deploy.sh and runs the installed
    run.sh --describe-only."""
    r = runfiles.Create()
    deploy = r.Rlocation("_main/tools/deploy.sh")
    subprocess.run(["bash", deploy, "install", bundle, "--prefix", prefix],
                   check=True, capture_output=True, text=True, timeout=120)
    where = os.path.join(prefix, "current")
    log_dir = os.path.join(prefix, "logs")
    env = dict(os.environ, TALOS_LOG_DIR=log_dir)
    proc = subprocess.run(["./run.sh", "--describe-only"], cwd=where, env=env,
                          capture_output=True, text=True, timeout=120)
    sessions = sorted(
        (os.path.join(log_dir, name) for name in os.listdir(log_dir)
         if os.path.isdir(os.path.join(log_dir, name))),
        key=os.path.getmtime, reverse=True)
    graph = os.path.join(sessions[0], "graph.json") if sessions else ""
    if not os.path.isfile(graph):
        return proc, None
    with open(graph, encoding="utf-8") as f:
        return proc, json.load(f)


class BundleTest(unittest.TestCase):
    def test_installed_bundles_describe_with_a_stable_identity(self):
        r = runfiles.Create()
        for name in BUNDLES:
            # deploy.sh installs read-only, so cleanup cannot remove it all;
            # Bazel clears TEST_TMPDIR.
            with self.subTest(name), tempfile.TemporaryDirectory(
                    ignore_cleanup_errors=True) as tmp:
                src = r.Rlocation(f"_main/2026-robot/main_processor/{name}")
                self.assertTrue(src and os.path.isdir(src), src)
                with open(os.path.join(src, "bundle_manifest.toml"), "rb") as f:
                    manifest = tomllib.load(f)
                if name == "robot_bundle":
                    # Resolve the deployed overlay's robot scalar over its base,
                    # rather than accidentally checking the simulation config.
                    self.assertEqual(manifest["config_path"], "config/robot_field.toml")
                    config_path = os.path.join(src, manifest["config_path"])
                    config_dir = os.path.dirname(config_path)
                    with open(config_path, "rb") as f:
                        overlay = tomllib.load(f)
                    with open(os.path.join(config_dir, overlay["include"]), "rb") as f:
                        base = tomllib.load(f)
                    robot = base["robot"] | overlay.get("robot", {})
                    self.assertEqual(robot["controller_address"], "10.56.87.2")
                proc, graph = describe(src, os.path.join(tmp, "prefix"))
                out = proc.stdout + "\n" + proc.stderr
                self.assertEqual(proc.returncode, 0, out)
                self.assertNotIn("error:", proc.stdout.lower())
                self.assertNotIn("not described", proc.stdout)
                self.assertIsNotNone(graph, out)
                for node in graph["nodes"]:
                    self.assertTrue(node.get("described"),
                                    f"{node.get('name')}: {node.get('describe_error')}\n{out}")
                libs = os.listdir(os.path.join(src, "lib")) if os.path.isdir(
                    os.path.join(src, "lib")) else []
                for lib in SHARED_LIBS.get(name, ()):
                    self.assertTrue(
                        any(f.startswith(lib + ".") for f in libs), (lib, libs))
                # The copy keeps the identity the bundle was built with.
                self.assertTrue(manifest["config_sha256"])
                self.assertEqual(graph["config_sha256"],
                                 manifest["config_sha256"])


if __name__ == "__main__":
    unittest.main()
