"""Unit tests for //tools:sync_manifest."""

import os
import tempfile
import unittest

from tools import sync_manifest


def put(root, path, text):
    full = os.path.join(root, path)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    with open(full, "w", encoding="utf-8") as f:
        f.write(text)


class SyncManifestTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
