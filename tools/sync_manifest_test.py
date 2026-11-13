"""Unit tests for //tools:sync_manifest."""

import contextlib
import io
import os
import json
import tempfile
import tomllib
import unittest
from unittest import mock

from tools import sync_manifest


def put(root, path, text):
    full = os.path.join(root, path)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    with open(full, "w", encoding="utf-8") as f:
        f.write(text)


def check_teleop(test, teleop, name):
    """Exercise validation through the actual CLI with a TOML inline table."""
    def literal(value):
        if isinstance(value, dict):
            return "{ " + ", ".join(
                json.dumps(key) + " = " + literal(item)
                for key, item in value.items()) + " }"
        if isinstance(value, list):
            return "[" + ", ".join(literal(item) for item in value) + "]"
        return json.dumps(value)
    sync_manifest.effective(
        test.root, "teleop.toml",
        overrides={"teleop.toml": f"[subsystems.{name}]\nteleop = {literal(teleop)}\n"})


class SyncManifestTest(unittest.TestCase):
    def test_rejects_structural_tables_and_unknown_motor_keys(self):
        bad = [
            "robot = 1\n[subsystems.test]\n",
            "[subsystems]\ntest = 1\n",
            "[subsystems.test.motors.m]\ncan_id = 1\ncanid = 2\n",
            ('[subsystems.test.sensors.encoder]\n'
             'type = "CANcoder"\ncan_id = 2\noffest_rot = 0.25\n'),
        ]
        bad += [f"[subsystems.test]\n{section} = 1\n"
                for section in ("motors", "sensors", "digital_inputs",
                                "digital_outputs", "analog_inputs",
                                "encoders", "pwm_outputs")]
        for slot in ("slot0", "slot1", "slot2"):
            bad += [
                f"[subsystems.test.motors.m]\ncan_id = 1\n{slot} = 1\n",
                f"[subsystems.test.motors.m.{slot}]\nunknown = 1\n",
            ]
        for text in bad:
            with self.subTest(text=text):
                with self.assertRaises(sync_manifest.ManifestError):
                    sync_manifest.effective(
                        self.root, "bad.toml", overrides={"bad.toml": text})
        text = ("[subsystems.test.motors.m]\ncan_id = 1\nbrake = true\n"
                "[subsystems.test.motors.m.slot0]\np = 1.0\n"
                'gravity_type = "Arm"\n')
        sync_manifest.effective(
            self.root, "good.toml", overrides={"good.toml": text})


    def test_teleop_defaults_use_assignment_scalar_types(self):
        check_teleop(self,
            {"defaults": {"position_rot": 0.25, "mode": "HOLD",
                          "enabled": False, "count": 2}}, "arm")
        for defaults in (3, [], {"position_rot": []},
                         {"nested": {"position_rot": 0.25}}):
            with self.subTest(defaults=defaults):
                with self.assertRaises(sync_manifest.ManifestError):
                    check_teleop(self, {"defaults": defaults}, "arm")

    def test_pov_toggle_validation(self):
        for angle in range(0, 360, 45):
            check_teleop(self,
                {"bindings": [{"pov": angle, "pov_index": 3,
                               "on_toggle": {"velocity_rps": 60.0}}]}, "roller")
        for entry in (
                {"pov": True, "while_held": {"x": 1}},
                {"pov": -45, "while_held": {"x": 1}},
                {"pov": 90, "pov_index": 0.0, "while_held": {"x": 1}},
                {"pov": 90, "on_toggle": {}},
                {"button": 1, "while_held": {}, "on_toggle": {"x": 1}},
                {"axis": 0, "field": "x", "scale": 1, "pov_index": 0}):
            with self.subTest(entry=entry):
                with self.assertRaises(sync_manifest.ManifestError):
                    check_teleop(self, {"bindings": [entry]}, "roller")

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = self.tmp.name
        put(self.root, "bot/BUILD", "")
        put(self.root, "shared/BUILD", "")
        put(self.root, "shared/arm_defaults.toml",
            '[subsystem]\nname = "arm"\nnode = "@arm//:node"\n')
        put(self.root, "bot/robot.toml",
            '[robot]\nname = "bot"\n\n'
            '[[subsystems]]\nname = "intake"\npath = "subsystems/intake.toml"\n\n'
            '[[subsystems]]\nname = "arm"\npath = "arm.toml"\n'
            'defaults = "../shared/arm_defaults.toml"\n\n'
            '[[subsystems]]\nname = "wrist"\npath = "subsystems/wrist.toml"\n'
            'node = "//bot/py:wrist"\n')
        put(self.root, "bot/subsystems/intake.toml",
            '[subsystem]\nname = "intake"\nnode = "//bot/intake:node"\n')
        put(self.root, "bot/arm.toml", '[subsystem]\nname = "arm"\n')
        put(self.root, "bot/subsystems/wrist.toml",
            '[subsystem]\nname = "wrist"\nnode = "//bot/wrist:node"\n'
            'defaults = "wrist_defaults.toml"\n')
        put(self.root, "bot/subsystems/wrist_defaults.toml",
            '[subsystem]\nname = "wrist"\n')

    def tearDown(self):
        self.tmp.cleanup()

    def test_resolves_nodes_defaults_and_overrides(self):
        nodes, files = sync_manifest.resolve(self.root, "bot/robot.toml")
        self.assertEqual(nodes, ["//bot/intake:node", "//bot/py:wrist", "@arm//:node"])
        self.assertEqual(files, [
            "bot/arm.toml",
            "bot/subsystems/intake.toml",
            "bot/subsystems/wrist.toml",
            "bot/subsystems/wrist_defaults.toml",
            "shared/arm_defaults.toml",
        ])
        text = sync_manifest.generate(self.root, "bot/robot.toml")
        self.assertIn('    "//shared:arm_defaults.toml",\n', text)
        self.assertIn('    "//bot:subsystems/wrist.toml",\n', text)

    def test_cli_reports_effective_config_and_pending_file_provenance(self):
        result = sync_manifest._resolved(
            self.root, "pending/robot.toml", overrides={
                "pending/robot.toml":
                    '[[subsystems]]\nname = "wrist"\npath = "wrist.toml"\n'
                    'node = "@wrist//:node"\n',
                "pending/wrist.toml":
                    '[subsystem]\nname = "wrist"\nperiod_us = 1000\n',
            })
        self.assertEqual(result["nodes"], ["@wrist//:node"])
        self.assertEqual(result["files"], ["pending/wrist.toml"])
        self.assertEqual(result["subsystem_paths"], {"wrist": "pending/wrist.toml"})
        self.assertEqual(result["subsystems"]["wrist"]["period_us"], 1000)
        self.assertFalse(os.path.exists(os.path.join(self.root, "pending")))

    def test_include_merges_entries_by_name(self):
        put(self.root, "bot/robot_py.toml",
            'include = "robot.toml"\n\n'
            '[[subsystems]]\nname = "intake"\nnode = "//bot/py:intake"\n')
        nodes, files = sync_manifest.resolve(self.root, "bot/robot_py.toml")
        self.assertEqual(nodes, ["//bot/py:intake", "//bot/py:wrist", "@arm//:node"])
        self.assertIn("bot/robot.toml", files)
        self.assertEqual(sync_manifest.manifest_path("bot/robot_py.toml"),
                         "bot/robot_py_manifest.bzl")

    def test_single_file_tables(self):
        put(self.root, "bot/flat.toml",
            '[subsystems.a]\nnode = "//a:node"\n[subsystems.b]\nperiod_us = 1\n')
        self.assertEqual(sync_manifest.resolve(self.root, "bot/flat.toml"),
                         (["//a:node"], []))

    def test_critical_is_an_optional_boolean(self):
        for value in ("true", "false"):
            with self.subTest(value=value):
                subs, _ = sync_manifest.effective(
                    self.root, "critical.toml",
                    overrides={"critical.toml":
                               f"[subsystems.arm]\ncritical = {value}\n"})
                self.assertIs(subs["arm"]["critical"], value == "true")
        for value in ("0", "1", '"false"', "[]", "{}"):
            with self.subTest(value=value):
                with self.assertRaises(sync_manifest.ManifestError):
                    sync_manifest.effective(
                        self.root, "critical.toml",
                        overrides={"critical.toml":
                                   f"[subsystems.arm]\ncritical = {value}\n"})

    def test_target_coverage_key_accepts_only_booleans(self):
        for value in ("true", "false"):
            with self.subTest(value=value):
                sync_manifest.resolve(
                    self.root, "coverage.toml",
                    overrides={"coverage.toml":
                               f"[robot]\nrequire_target_coverage = {value}\n"
                               '[subsystems.arm]\nnode = "//bot/arm:node"\n'})
        for value in ("1", '"true"', "[]"):
            with self.subTest(value=value):
                with self.assertRaises(sync_manifest.ManifestError):
                    sync_manifest.resolve(
                        self.root, "coverage.toml",
                        overrides={"coverage.toml":
                                   f"[robot]\nrequire_target_coverage = {value}\n"
                                   "[subsystems.arm]\n"})

    def test_check_mode_fails_on_a_stale_roster_without_rewriting_it(self):
        sync_manifest.write(self.root, "bot/robot.toml")
        manifest = os.path.join(self.root, "bot/robot_manifest.bzl")
        with open(manifest, encoding="utf-8") as f:
            original = f.read()
        with mock.patch.dict(os.environ,
                             {"BUILD_WORKSPACE_DIRECTORY": self.root}):
            self.assertEqual(sync_manifest.main(["--check", "bot/robot.toml"]), 0)
            put(self.root, "bot/subsystems/intake.toml",
                '[subsystem]\nname = "intake"\nnode = "//bot/py:intake"\n')
            errors = io.StringIO()
            with contextlib.redirect_stderr(errors):
                self.assertEqual(
                    sync_manifest.main(["--check", "bot/robot.toml"]), 1)
        with open(manifest, encoding="utf-8") as f:
            self.assertEqual(f.read(), original)

    # REQ-CONTRACT-19, REQ-CONTRACT-31: a [trajectories] file is a config dependency, so the
    # bundle ships it and a manifest that omits it is stale.
    def test_trajectory_file_is_listed_and_its_omission_is_stale(self):
        sync_manifest.write(self.root, "bot/robot.toml")
        put(self.root, "bot/trajectories/taxi.traj", "{}\n")
        put(self.root, "bot/subsystems/intake.toml",
            '[subsystem]\nname = "intake"\nnode = "//bot/intake:node"\n'
            '[trajectories]\ntaxi = "../trajectories/taxi.traj"\n')
        with mock.patch.dict(os.environ,
                             {"BUILD_WORKSPACE_DIRECTORY": self.root}):
            errors = io.StringIO()
            with contextlib.redirect_stderr(errors):
                self.assertEqual(
                    sync_manifest.main(["--check", "bot/robot.toml"]), 1)
            self.assertIn("bot/trajectories/taxi.traj", errors.getvalue())
            self.assertIn(sync_manifest.COMMAND, errors.getvalue())
            sync_manifest.write(self.root, "bot/robot.toml")
            with open(os.path.join(self.root, "bot/robot_manifest.bzl"),
                      encoding="utf-8") as f:
                self.assertIn('    "//bot:trajectories/taxi.traj",\n', f.read())
            self.assertEqual(sync_manifest.main(["--check", "bot/robot.toml"]), 0)



    def test_refuses_paths_without_labels(self):
        put(self.root, "bot/abs.toml",
            '[[subsystems]]\nname = "x"\npath = "/etc/x.toml"\n')
        with self.assertRaises(sync_manifest.ManifestError):
            sync_manifest.resolve(self.root, "bot/abs.toml")


CASES = "talOS/configuration/testdata"


class SharedResolutionCasesTest(unittest.TestCase):
    """The cases //talOS/configuration:test runs through the C++ parser."""

    def test_cases_match_the_parser(self):
        names = sorted(os.listdir(CASES))
        self.assertTrue(names)
        for name in names:
            root = os.path.join(CASES, name)
            with open(os.path.join(root, "expect.toml"), "rb") as f:
                expect = tomllib.load(f)
            config = expect.get("config", "robot.toml")
            with self.subTest(name):
                if not expect["accept"]:
                    with self.assertRaises(sync_manifest.ManifestError) as caught:
                        sync_manifest.resolve(root, config)
                    if "error_contains" in expect:
                        self.assertIn(expect["error_contains"],
                                      str(caught.exception))
                    continue
                self.assertEqual(sync_manifest.resolve(root, config),
                                 (expect["nodes"], expect["files"]))


if __name__ == "__main__":
    unittest.main()
