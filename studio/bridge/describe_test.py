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
        # The launcher refuses a topic end without schema and ring layout
        # (talOS/introspection/describe.h), which kept ./sim.sh with Studio
        # from starting; they must match the telemetry byte topic.
        source = doc["sources"][0]
        self.assertEqual(source["schema"], "talos.Bytes")
        self.assertTrue(source["schema_hash"].startswith("0x"))
        self.assertEqual(source["alignment"], 8)
        self.assertEqual(source["slots"], 64)

    # REQ-SAFE-35
    def test_describe_declares_no_writer(self):
        """The bridge declares only reader ends: no SENDER, no hardware request.

        The launcher's graph lint and the live endpoint comparison judge the
        bridge by this description, so a writer declared here is a writer the
        session would accept.
        """
        for args in ([], ["--config", str(testdata("robot.toml")), "--sim"]):
            with self.subTest(args=args):
                proc = subprocess.run(
                    [str(bridge_binary()), "--describe", *args],
                    capture_output=True, text=True, timeout=30)
                self.assertEqual(proc.returncode, 0, proc.stderr)
                sources = json.loads(proc.stdout)["sources"]
                self.assertTrue(sources, proc.stdout)
                for source in sources:
                    self.assertIn(source["kind"], ("WATCHER", "FETCHER", "TIMER"),
                                  source)
                    self.assertFalse(source.get("hardware_request", False), source)

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
