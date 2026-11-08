"""Unit tests for //tools:sync_manifest."""

import contextlib
import io
import os
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


class SyncManifestTest(unittest.TestCase):
    def test_rejects_structural_tables_and_unknown_motor_keys(self):
        bad = [
            "robot = 1\n[subsystems.test]\n",
            "[subsystems]\ntest = 1\n",
            "[subsystems.test.motors.m]\ncan_id = 1\ncanid = 2\n",
        ]
        bad += [f"[subsystems.test]\n{section} = 1\n"
                for section in sync_manifest._DEVICE_SECTIONS]
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

    def test_unknown_device_key_error_names_config_device_and_key(self):
        text = ('[subsystems.test.sensors.encoder]\n'
                'type = "CANcoder"\ncan_id = 2\noffest_rot = 0.25\n')
        with self.assertRaises(sync_manifest.ManifestError) as caught:
            sync_manifest.effective(
                self.root, "bad.toml", overrides={"bad.toml": text})
        message = str(caught.exception)
        self.assertIn("bad.toml", message)
        self.assertIn("[subsystems.test].sensors.encoder", message)
        self.assertIn("unknown sensor key 'offest_rot'", message)

    def test_teleop_defaults_use_assignment_scalar_types(self):
        sync_manifest._check_teleop(
            {"defaults": {"position_rot": 0.25, "mode": "HOLD",
                          "enabled": False, "count": 2}}, "arm")
        for defaults in (3, [], {"position_rot": []},
                         {"nested": {"position_rot": 0.25}}):
            with self.subTest(defaults=defaults):
                with self.assertRaises(sync_manifest.ManifestError):
                    sync_manifest._check_teleop({"defaults": defaults}, "arm")

    def test_pov_toggle_validation(self):
        for angle in range(0, 360, 45):
            sync_manifest._check_teleop(
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
                    sync_manifest._check_teleop({"bindings": [entry]}, "roller")

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
        self.assertIn("bazel run //tools:sync_manifest -- bot/robot.toml", text)

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
                with self.assertRaisesRegex(sync_manifest.ManifestError,
                                            "require_target_coverage"):
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
        self.assertIn("is stale", errors.getvalue())
        self.assertIn("bazel run //tools:sync_manifest -- bot/robot.toml",
                      errors.getvalue())
        with open(manifest, encoding="utf-8") as f:
            self.assertEqual(f.read(), original)

    def test_check_reports_an_unregenerated_entry_with_the_fix(self):
        self.assertTrue(sync_manifest.write(self.root, "bot/robot.toml"))
        self.assertEqual(sync_manifest.check(self.root, "bot/robot.toml"), "")
        self.assertFalse(sync_manifest.write(self.root, "bot/robot.toml"))

        # A new entry naming a file the manifest does not list.
        put(self.root, "bot/subsystems/climber.toml",
            '[subsystem]\nname = "climber"\nnode = "//bot/climber:node"\n')
        with open(os.path.join(self.root, "bot/robot.toml"), "a",
                  encoding="utf-8") as f:
            f.write('\n[[subsystems]]\nname = "climber"\n'
                    'path = "subsystems/climber.toml"\n')
        problem = sync_manifest.check(self.root, "bot/robot.toml")
        self.assertIn("subsystems/climber.toml", problem)
        self.assertIn("bazel run //tools:sync_manifest -- bot/robot.toml", problem)

    def test_check_reports_a_changed_node_with_a_diff(self):
        sync_manifest.write(self.root, "bot/robot.toml")
        put(self.root, "bot/subsystems/intake.toml",
            '[subsystem]\nname = "intake"\nnode = "//bot/py:intake"\n')
        problem = sync_manifest.check(self.root, "bot/robot.toml")
        self.assertIn('+    "//bot/py:intake",', problem)
        self.assertIn('-    "//bot/intake:node",', problem)

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
