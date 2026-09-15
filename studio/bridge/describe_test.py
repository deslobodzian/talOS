"""studio_bridge --describe answers without ports, shm, or a robot."""

import json
import os
import subprocess
from pathlib import Path
import unittest


def bridge_binary():
    workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get(
        "TEST_WORKSPACE", "_main")
    return workspace / "studio/bridge/studio_bridge"


def testdata(name):
    workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get(
        "TEST_WORKSPACE", "_main")
    return workspace / "studio/bridge/testdata" / name


class BridgeDescribe(unittest.TestCase):
    def test_describe_reports_session_topic(self):
        proc = subprocess.run(
            [str(bridge_binary()), "--describe", "--config",
             str(testdata("robot.toml")), "--sim"],
            capture_output=True, text=True, timeout=30)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        doc = json.loads(proc.stdout)
        self.assertEqual(doc["describe_version"], 1)
        self.assertEqual(doc["name"], "studio_bridge")
        self.assertEqual(doc["target"], "//studio/bridge:studio_bridge")
        kinds = [(s["kind"], s["name"]) for s in doc["sources"]]
        self.assertIn(("WATCHER", "/custom/studio_feed"), kinds)

    def test_describe_without_config_uses_default_topic(self):
        proc = subprocess.run(
            [str(bridge_binary()), "--describe"],
            capture_output=True, text=True, timeout=30)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        doc = json.loads(proc.stdout)
        names = [s["name"] for s in doc["sources"]]
        self.assertIn("/talos/telemetry", names)


if __name__ == "__main__":
    unittest.main()
