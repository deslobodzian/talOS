"""Unit tests for //tools:new_subsystem."""

import os
import tempfile
import unittest

from tools import new_subsystem


class NewSubsystemTest(unittest.TestCase):
    def test_check_name_accepts_valid_names_and_rejects_invalid_ones(self):
        self.assertIsNone(new_subsystem.check_name("wrist"))
        self.assertIsNone(new_subsystem.check_name("climber_arm_2"))

        # Empty, uppercase, bad characters, underscores
        self.assertIsNotNone(new_subsystem.check_name(""))
        self.assertIsNotNone(new_subsystem.check_name("Wrist"))
        self.assertIsNotNone(new_subsystem.check_name("1wrist"))
        self.assertIsNotNone(new_subsystem.check_name("wrist-arm"))
        self.assertIsNotNone(new_subsystem.check_name("wrist__arm"))
        self.assertIsNotNone(new_subsystem.check_name("wrist_"))

        # Abbreviations, direction suffixes, reserved owners, keywords/modules
        self.assertIn("target", new_subsystem.check_name("tgt"))
        self.assertIn("drivetrain", new_subsystem.check_name("drive"))
        self.assertIn("direction", new_subsystem.check_name("wrist_out"))
        self.assertIn("reserved", new_subsystem.check_name("hw"))
        self.assertIn("keyword", new_subsystem.check_name("class"))
        self.assertIn("keyword", new_subsystem.check_name("sys"))
        self.assertIn("keyword", new_subsystem.check_name("python"))
        self.assertIn("collides", new_subsystem.check_name("node"))

        # Too long to leave room for /<name>/target/teleop (31 bytes max)
        self.assertIsNotNone(new_subsystem.check_name("a" * 17))
        self.assertIsNone(new_subsystem.check_name("a" * 16))

    def test_insert_label_maintains_sorted_peer_order(self):
        build_text = """\
load("//talOS/build:contract.bzl", "talos_bundle")

talos_bundle(
    name = "robot_bundle",
    config = "//my-robot/main_processor/configuration:robot.toml",
    nodes = [
        "//talOS/bridge:hardware_node",
        "//my-robot/main_processor/arbiter:node",
        "//my-robot/main_processor/shooter:node",
    ],
    subsystems = [
        "//my-robot/main_processor/configuration:subsystems/arbiter.toml",
        "//my-robot/main_processor/configuration:subsystems/shooter.toml",
    ],
)
"""
        updated = new_subsystem.insert_label(
            build_text,
            "nodes",
            "//my-robot/main_processor/climber:node",
            lambda x: x.startswith("//my-robot/main_processor/") and x.endswith(":node"),
            "BUILD",
        )
        self.assertIn(
            '        "//my-robot/main_processor/arbiter:node",\n'
            '        "//my-robot/main_processor/climber:node",\n'
            '        "//my-robot/main_processor/shooter:node",\n',
            updated,
        )
        with self.assertRaises(new_subsystem.Refused):
            new_subsystem.insert_label(
                updated,
                "nodes",
                "//my-robot/main_processor/climber:node",
                lambda x: True,
                "BUILD",
            )

    def test_plan_scaffolds_into_custom_robot_and_assigns_lowest_free_can_id(self):
        with tempfile.TemporaryDirectory() as root:
            proc = os.path.join(root, "custom-bot", "main_processor")
            cfg = os.path.join(proc, "configuration")
            subs = os.path.join(cfg, "subsystems")
            os.makedirs(subs)
            with open(os.path.join(cfg, "robot.toml"), "w", encoding="utf-8") as f:
                f.write(
                    '[robot]\nname = "custom"\n\n'
                    '[[subsystems]]\nname = "shooter"\npath = "subsystems/shooter.toml"\n'
                )
            with open(os.path.join(subs, "shooter.toml"), "w", encoding="utf-8") as f:
                f.write('[subsystem]\nname = "shooter"\n[motors.flywheel]\ncan_id = 1\n')
            with open(os.path.join(proc, "BUILD"), "w", encoding="utf-8") as f:
                f.write(
                    'talos_bundle(\n'
                    '    name = "robot_bundle",\n'
                    '    nodes = [\n'
                    '        "//custom-bot/main_processor/shooter:node",\n'
                    '    ],\n'
                    '    subsystems = [\n'
                    '        "//custom-bot/main_processor/configuration:subsystems/shooter.toml",\n'
                    '    ],\n'
                    ')\n'
                )
            with open(os.path.join(cfg, "BUILD"), "w", encoding="utf-8") as f:
                f.write(
                    'exports_files(glob(["subsystems/*.toml"]))\n\n'
                    'talos_graph_test(\n'
                    '    name = "graph_test",\n'
                    '    nodes = [\n'
                    '        "//custom-bot/main_processor/shooter:node",\n'
                    '    ],\n'
                    ')\n'
                )

            writes, can_id = new_subsystem.plan(root, "custom-bot", "wrist", "cpp")
            self.assertEqual(can_id, 2)
            by_rel = {
                os.path.relpath(path, root).replace(os.sep, "/"): content
                for path, _, content in writes
            }
            self.assertIn("custom-bot/main_processor/wrist/BUILD", by_rel)
            self.assertIn("teleop_target", by_rel["custom-bot/main_processor/wrist/BUILD"])
            self.assertIn("auto_target", by_rel["custom-bot/main_processor/wrist/BUILD"])
            self.assertIn(
                "can_id = 2",
                by_rel["custom-bot/main_processor/configuration/subsystems/wrist.toml"],
            )
            self.assertIn(
                "//custom-bot/main_processor/wrist:node",
                by_rel["custom-bot/main_processor/BUILD"],
            )
            self.assertIn(
                "//custom-bot/main_processor/wrist:node",
                by_rel["custom-bot/main_processor/configuration/BUILD"],
            )

    def test_plan_standalone_generates_self_contained_bzlmod_repo(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = os.path.join(tmp, "talos-wrist")
            writes = new_subsystem.plan_standalone(target, "wrist", "cpp")
            by_rel = {
                os.path.relpath(path, target).replace(os.sep, "/"): content
                for path, _, content in writes
            }
            self.assertIn("MODULE.bazel", by_rel)
            self.assertIn('module(name = "wrist"', by_rel["MODULE.bazel"])
            self.assertIn('bazel_dep(name = "talos"', by_rel["MODULE.bazel"])
            self.assertIn("defaults.toml", by_rel)
            self.assertIn('node = "@wrist//:node"', by_rel["defaults.toml"])
            self.assertNotIn("can_id =", by_rel["defaults.toml"])
            self.assertIn("@talos//talOS/build:contract.bzl", by_rel["BUILD"])
            self.assertIn('exports_files(["defaults.toml"])', by_rel["BUILD"])

    def test_position_archetype_generates_motion_magic_for_cpp_and_python(self):
        cpp_files = new_subsystem.package_files("wrist", "cpp", kind="position")
        self.assertIn("position_rot:double;", cpp_files["wrist_message.fbs"])
        self.assertIn("at_goal:bool;", cpp_files["wrist_message.fbs"])
        self.assertIn("set_motion_magic", cpp_files["wrist.h"])
        self.assertIn("Mode::kMotionMagic", cpp_files["node_test.cc"])

        py_files = new_subsystem.package_files("wrist", "py", kind="position")
        self.assertIn("set_motion_magic", py_files["wrist.py"])
        self.assertIn("talos.Mode.MOTION_MAGIC", py_files["node_test.py"])


if __name__ == "__main__":
    unittest.main()
