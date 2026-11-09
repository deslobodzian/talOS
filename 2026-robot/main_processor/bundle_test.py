"""Installed bundles describe their graph and keep their config identity.

Each bundle is installed the way the robot gets it (tools/deploy.sh install
into a fresh prefix), then --describe runs every node binary from there, so a
C++ node that cannot load a shared library it links, a
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
                proc, graph = describe(src, os.path.join(tmp, "prefix"))
                out = proc.stdout + "\n" + proc.stderr
                self.assertEqual(proc.returncode, 0, out)
                self.assertIsNotNone(graph, out)
                for node in graph["nodes"]:
                    self.assertTrue(node.get("described"),
                                    f"{node.get('name')}: {node.get('describe_error')}\n{out}")
                # The copy keeps the identity the bundle was built with.
                self.assertTrue(manifest["config_sha256"])
                self.assertEqual(graph["config_sha256"],
                                 manifest["config_sha256"])


if __name__ == "__main__":
    unittest.main()
