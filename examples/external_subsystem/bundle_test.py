"""A standalone module's node runs after its consumer bundle is relocated."""

import json
import os
import subprocess
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles


class BundleTest(unittest.TestCase):
    def test_external_node_in_relocated_bundle(self):
        source = runfiles.Create().Rlocation(
            "_main/examples/external_subsystem/robot_bundle")
        self.assertTrue(source and os.path.isdir(source), source)
        with open(os.path.join(source, "bundle_manifest.toml"), "rb") as stream:
            manifest = tomllib.load(stream)
        deploy = runfiles.Create().Rlocation("_main/tools/deploy.sh")
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
            prefix = os.path.join(tmp, "prefix")
            subprocess.run(
                ["bash", deploy, "install", source, "--prefix", prefix],
                check=True, capture_output=True, text=True, timeout=120)
            moved = os.path.join(prefix, "current")
            logs = os.path.join(tmp, "logs")
            proc = subprocess.run(
                ["./run.sh", "--describe-only"], cwd=moved,
                env=dict(os.environ, TALOS_LOG_DIR=logs),
                capture_output=True, text=True, timeout=120)
            output = proc.stdout + "\n" + proc.stderr
            self.assertEqual(proc.returncode, 0, output)
            sessions = sorted(
                (os.path.join(logs, name) for name in os.listdir(logs)
                 if os.path.isdir(os.path.join(logs, name))),
                key=os.path.getmtime, reverse=True)
            self.assertTrue(sessions, output)
            with open(os.path.join(sessions[0], "graph.json"), encoding="utf-8") as stream:
                graph = json.load(stream)
            self.assertEqual(graph["config_sha256"], manifest["config_sha256"])
            self.assertTrue(graph["nodes"], output)
            for node in graph["nodes"]:
                self.assertTrue(node.get("described"),
                                f"{node.get('name')}: {node.get('describe_error')}\n{output}")
            self.assertIn("wrist", [node["name"] for node in graph["nodes"]])


if __name__ == "__main__":
    unittest.main()
