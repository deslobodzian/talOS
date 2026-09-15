"""Readiness report checks (item 60): processes, hardware, config, feedback
with specific inhibit reasons, over hermetic fixture configs (item 54: no
fixed ports, isolated temp dirs, owned-resource cleanup only)."""

import json
import tempfile
import unittest
from pathlib import Path

from compat_record import recompute_config_sha256
from readiness import assess

ROBOT_TOML = """[robot]
name = "fixture"
commissioned = {commissioned}

[[subsystems]]
name = "roller"
path = "roller.toml"
"""

ROLLER_TOML = """[subsystem]
name = "roller"
node = "//example/roller:node"

[motors.roller]
type = "TalonFX"
bus = "rio"
can_id = 5
{feedback}

[sensors.roller_encoder]
type = "CANcoder"
bus = "rio"
can_id = 6
"""


def make_tree(root, commissioned=True, feedback=""):
    root = Path(root)
    (root / "config").mkdir(parents=True)
    (root / "config" / "robot.toml").write_text(
        ROBOT_TOML.format(commissioned=str(commissioned).lower()))
    (root / "config" / "roller.toml").write_text(
        ROLLER_TOML.format(feedback=feedback))
    return root


def make_manifest(root, with_node=True, sha=None):
    nodes = ([{"name": "roller", "target": "//example/roller:node",
               "binary_path": "bin/roller", "language": "cpp",
               "contract": "@talos_x//:contract"}] if with_node else [])
    return {"bundle_version": 1, "config_path": "config/robot.toml",
            "config_sha256": sha or "", "nodes": nodes,
            "_bundle_dir": str(root)}


class ReadinessTest(unittest.TestCase):
    def ready_tree(self, tmp, **kwargs):
        root = make_tree(tmp, **kwargs)
        sha = recompute_config_sha256(str(root / "config"))
        return root, make_manifest(root, sha=sha)

    def verdict(self, sections):
        return all(ok for _, f in sections for _, ok, _ in f)

    def test_ready_bundle(self):
        with tempfile.TemporaryDirectory(prefix="talos_ready_ok_") as tmp:
            root, manifest = self.ready_tree(tmp)
            sections, inhibited = assess(
                str(root / "config" / "robot.toml"), manifest,
                vendor={"devices": {"TalonFX": {}}}, simulation=False)
            self.assertFalse(inhibited)
            self.assertTrue(self.verdict(sections))

    def test_uncommissioned_inhibits_hardware_on_real_robot(self):
        with tempfile.TemporaryDirectory(prefix="talos_ready_hw_") as tmp:
            root = make_tree(tmp, commissioned=False)
            sha = recompute_config_sha256(str(root / "config"))
            _, inhibited = assess(
                str(root / "config" / "robot.toml"),
                make_manifest(root, sha=sha),
                vendor={"devices": {}}, simulation=False)
            self.assertTrue(inhibited)
            # ... but sim sessions are explicitly allowed through.
            _, sim_inhibited = assess(
                str(root / "config" / "robot.toml"),
                make_manifest(root, sha=sha),
                vendor={"devices": {}}, simulation=True)
            self.assertFalse(sim_inhibited)

    def test_missing_node_inhibits_processes(self):
        with tempfile.TemporaryDirectory(prefix="talos_ready_proc_") as tmp:
            root, _ = self.ready_tree(tmp)
            sections, inhibited = assess(
                str(root / "config" / "robot.toml"),
                make_manifest(root, with_node=False,
                              sha=recompute_config_sha256(
                                  str(root / "config"))),
                vendor={"devices": {}}, simulation=False)
            self.assertTrue(inhibited)
            procs = dict((n, (ok, r)) for t, f in sections
                         for n, ok, r in f if t == "processes")
            self.assertIn("roller", procs)
            self.assertFalse(procs["roller"][0])
            self.assertIn("not staged", procs["roller"][1])

    def test_tampered_config_inhibits(self):
        with tempfile.TemporaryDirectory(prefix="talos_ready_cfg_") as tmp:
            root, manifest = self.ready_tree(tmp)
            with open(root / "config" / "roller.toml", "a") as f:
                f.write("# pit hand edit\n")
            _, inhibited = assess(
                str(root / "config" / "robot.toml"), manifest,
                vendor={"devices": {}}, simulation=False)
            self.assertTrue(inhibited)

    def test_dangling_feedback_sensor_inhibits_with_reason(self):
        with tempfile.TemporaryDirectory(prefix="talos_ready_fb_") as tmp:
            root = make_tree(
                tmp,
                feedback='feedback = "RemoteCANcoder"\n'
                         'feedback_sensor = "ghost_encoder"')
            sha = recompute_config_sha256(str(root / "config"))
            sections, inhibited = assess(
                str(root / "config" / "robot.toml"),
                make_manifest(root, sha=sha),
                vendor={"devices": {}}, simulation=False)
            self.assertTrue(inhibited)
            fb = dict((n, (ok, r)) for t, f in sections
                      for n, ok, r in f if t == "feedback")
            self.assertFalse(fb["roller.roller"][0])
            self.assertIn("ghost_encoder", fb["roller.roller"][1])

    def test_valid_remote_cancoder_is_ready(self):
        with tempfile.TemporaryDirectory(prefix="talos_ready_fb2_") as tmp:
            root = make_tree(
                tmp,
                feedback='feedback = "RemoteCANcoder"\n'
                         'feedback_sensor = "roller_encoder"')
            sha = recompute_config_sha256(str(root / "config"))
            sections, inhibited = assess(
                str(root / "config" / "robot.toml"),
                make_manifest(root, sha=sha),
                vendor={"devices": {}}, simulation=False)
            self.assertFalse(inhibited)
            fb = dict((n, (ok, r)) for t, f in sections
                      for n, ok, r in f if t == "feedback")
            self.assertTrue(fb["roller.roller"][0])

    def test_missing_vendor_expectations_inhibit(self):
        with tempfile.TemporaryDirectory(prefix="talos_ready_ven_") as tmp:
            root, manifest = self.ready_tree(tmp)
            _, inhibited = assess(
                str(root / "config" / "robot.toml"), manifest,
                vendor=None, simulation=False)
            self.assertTrue(inhibited)

    def test_report_render_names_every_inhibit(self):
        from readiness import render_report
        sections = [("hardware", [("roller", False, "uncommissioned: xyz")])]
        text = render_report(sections, True)
        self.assertIn("[INHIBIT] roller: uncommissioned: xyz", text)
        self.assertIn("verdict: INHIBITED", text)


if __name__ == "__main__":
    unittest.main()
