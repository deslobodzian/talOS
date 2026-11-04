"""Relocated bundles describe their graph and keep their config identity."""

import json
import os
import shutil
import subprocess
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles

BUNDLES = ("robot_bundle", "relocation_bundle")


def describe(bundle, where):
    """Copies `bundle` to `where` and runs its run.sh --describe-only."""
    shutil.copytree(os.path.realpath(bundle), where, symlinks=True)
    log_dir = os.path.join(where, "logs")
    env = dict(os.environ, TALOS_LOG_DIR=log_dir)
    proc = subprocess.run(["./run.sh", "--describe-only"], cwd=where, env=env,
                          capture_output=True, text=True, timeout=120)
    graph = os.path.join(log_dir, "graph.json")
    if not os.path.isfile(graph):
        return proc, None
    with open(graph, encoding="utf-8") as f:
        return proc, json.load(f)


class BundleTest(unittest.TestCase):
    def test_relocated_bundles_describe_with_a_stable_identity(self):
        r = runfiles.Create()
        for name in BUNDLES:
            with self.subTest(name), tempfile.TemporaryDirectory() as tmp:
                src = r.Rlocation(f"_main/2026-robot/main_processor/{name}")
                self.assertTrue(src and os.path.isdir(src), src)
                with open(os.path.join(src, "bundle_manifest.toml"), "rb") as f:
                    manifest = tomllib.load(f)
                proc, graph = describe(src, os.path.join(tmp, "moved", name))
                out = proc.stdout + "\n" + proc.stderr
                self.assertEqual(proc.returncode, 0, out)
                self.assertNotIn("error:", proc.stdout.lower())
                self.assertNotIn("not described", proc.stdout)
                self.assertIsNotNone(graph, out)
                # The copy keeps the identity the bundle was built with.
                self.assertTrue(manifest["config_sha256"])
                self.assertEqual(graph["config_sha256"],
                                 manifest["config_sha256"])


if __name__ == "__main__":
    unittest.main()
