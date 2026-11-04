"""Unit tests for //tools:new_subsystem."""

import os
import tempfile
import unittest

from tools import new_subsystem

GOLDEN_FILEGROUP = """
# Golden copy of //tools:new_subsystem output; only this target is not generated.
filegroup(
    name = "golden",
    srcs = glob(["**"]),
    visibility = ["//tools:__pkg__"],
)
"""


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
                f.write('[subsystem]\nname = "shooter"\n'
                        'node = "//custom-bot/main_processor/shooter:node"\n'
                        '[motors.flywheel]\ncan_id = 1\n')
            with open(os.path.join(cfg, "BUILD"), "w", encoding="utf-8") as f:
                f.write('exports_files(glob(["subsystems/*.toml"]))\n')
            # A config including robot.toml goes stale with it; one that does
            # not must be left alone.
            for path, text in (
                    ("practice/BUILD", ""),
                    ("practice/robot.toml", 'include = "../configuration/robot.toml"\n'),
                    ("practice/robot_manifest.bzl", ""),
                    ("other/BUILD", ""),
                    ("other/robot.toml", '[subsystems.x]\nnode = "//x:node"\n'),
                    ("other/robot_manifest.bzl", "")):
                os.makedirs(os.path.dirname(os.path.join(proc, path)), exist_ok=True)
                with open(os.path.join(proc, path), "w", encoding="utf-8") as f:
                    f.write(text)

            writes, can_id = new_subsystem.plan(root, "custom-bot", "wrist", "cpp")
            self.assertEqual(can_id, 2)
            by_rel = {
                os.path.relpath(path, root).replace(os.sep, "/"): content
                for path, _, content in writes
            }
            self.assertIn("custom-bot/main_processor/wrist/BUILD", by_rel)
            build = by_rel["custom-bot/main_processor/wrist/BUILD"]
            # Only the state topic: producers own their target lanes.
            self.assertIn('"state": "/wrist/state"', build)
            self.assertNotIn("target", build.split("topics = {")[1].split("}")[0])
            subsystem = by_rel[
                "custom-bot/main_processor/configuration/subsystems/wrist.toml"]
            self.assertIn('# [targets]\n# teleop = ["teleop"]', subsystem)
            self.assertNotIn("arbitrat", subsystem)
            self.assertIn(
                "can_id = 2",
                by_rel["custom-bot/main_processor/configuration/subsystems/wrist.toml"],
            )
            self.assertIn(
                '[[subsystems]]\nname = "wrist"\npath = "subsystems/wrist.toml"\n',
                by_rel["custom-bot/main_processor/configuration/robot.toml"],
            )
            # No BUILD file is edited: the manifest carries the node and file.
            self.assertNotIn("custom-bot/main_processor/BUILD", by_rel)
            self.assertNotIn("custom-bot/main_processor/configuration/BUILD", by_rel)
            manifest = by_rel["custom-bot/main_processor/configuration/robot_manifest.bzl"]
            self.assertIn('    "//custom-bot/main_processor/wrist:node",\n', manifest)
            self.assertIn('    "//custom-bot/main_processor/shooter:node",\n', manifest)
            self.assertIn(
                '    "//custom-bot/main_processor/configuration:subsystems/wrist.toml",\n',
                manifest,
            )
            # The robot had no manifest yet, so it is written as a new file.
            self.assertIn((os.path.join(cfg, "robot_manifest.bzl"), None, manifest),
                          writes)
            practice = "custom-bot/main_processor/practice/robot_manifest.bzl"
            self.assertEqual(
                sorted(p for p in by_rel if p.endswith(".bzl")),
                ["custom-bot/main_processor/configuration/robot_manifest.bzl",
                 practice])
            self.assertIn('    "//custom-bot/main_processor/wrist:node",\n',
                          by_rel[practice])

    def test_plan_sees_inherited_subsystems_and_can_ids(self):
        # Ids and names reach robot.toml only through an include, a custom
        # path outside subsystems/ and a parent-directory defaults file.
        files = {
            "bot/main_processor/configuration/BUILD": "",
            "bot/main_processor/configuration/robot.toml":
                'include = "base.toml"\n',
            "bot/main_processor/configuration/base.toml":
                '[[subsystems]]\nname = "shooter"\npath = "mech/shooter.toml"\n'
                'defaults = "../../shared/shooter_defaults.toml"\n\n'
                '[[subsystems]]\nname = "feeder"\npath = "mech/feeder.toml"\n',
            "bot/main_processor/configuration/mech/shooter.toml":
                '[subsystem]\nname = "shooter"\n',
            "bot/main_processor/configuration/mech/feeder.toml":
                '[subsystem]\nname = "feeder"\nnode = "//f:node"\n'
                '[motors.feed]\ncan_id = 2\n',
            "bot/shared/BUILD": "",
            "bot/shared/shooter_defaults.toml":
                '[subsystem]\nname = "shooter"\nnode = "//s:node"\n'
                '[motors.flywheel]\ncan_id = 1\n',
        }
        with tempfile.TemporaryDirectory() as root:
            for path, text in files.items():
                os.makedirs(os.path.dirname(os.path.join(root, path)),
                            exist_ok=True)
                with open(os.path.join(root, path), "w", encoding="utf-8") as f:
                    f.write(text)
            _, can_id = new_subsystem.plan(root, "bot", "wrist", "py")
            self.assertEqual(can_id, 3)
            for inherited in ("shooter", "feeder"):
                with self.assertRaises(new_subsystem.Refused):
                    new_subsystem.plan(root, "bot", inherited, "py")

    def test_plan_standalone_builds_against_this_checkout(self):
        # A generated repo must build as written: it points @talos at the
        # checkout it came from and takes its C++23 flags and Bazel version.
        with tempfile.TemporaryDirectory() as tmp:
            talos = os.path.join(tmp, "src", "talOS")
            os.makedirs(talos)
            for config, text in ((".bazelrc", "build --cxxopt=-std=c++23\n"),
                                 (".bazelversion", "8.6.0\n")):
                with open(os.path.join(talos, config), "w",
                          encoding="utf-8") as f:
                    f.write(text)
            target = os.path.join(tmp, "repos", "wrist")
            writes = new_subsystem.plan_standalone(target, "wrist", "cpp",
                                                   talos)
            by_rel = {
                os.path.relpath(path, target).replace(os.sep, "/"): content
                for path, _, content in writes
            }
            self.assertIn(
                'local_path_override(module_name = "talos", '
                'path = "../../src/talOS")', by_rel["MODULE.bazel"])
            self.assertEqual(by_rel[".bazelrc"], "build --cxxopt=-std=c++23\n")
            self.assertEqual(by_rel[".bazelversion"], "8.6.0\n")
            # Loads stay first; every talOS label names the @talos module.
            build = by_rel["BUILD"]
            self.assertTrue(build.startswith("load("))
            self.assertNotIn('"//talOS', build)

    def test_generated_packages_match_the_built_golden_packages(self):
        # tools/scaffold_testdata holds the generator's output, built and
        # tested by `bazel test //...`: the templates compile and their
        # tests pass only while this equality holds. Each golden BUILD adds
        # one filegroup, which hands the files to this test.
        for name, lang in (("gripper", "cpp"), ("scaffold_py", "py")):
            golden = os.path.join("tools", "scaffold_testdata", name)
            on_disk = {}
            for directory, _, files in os.walk(golden):
                for f in files:
                    path = os.path.join(directory, f)
                    with open(path, encoding="utf-8") as text:
                        on_disk[os.path.relpath(path, golden).replace(
                            os.sep, "/")] = text.read()
            generated = new_subsystem.package_files(name, lang)
            generated["BUILD"] += GOLDEN_FILEGROUP
            with self.subTest(name):
                self.assertEqual(sorted(on_disk), sorted(generated))
                for path, content in generated.items():
                    self.assertEqual(
                        on_disk[path], content,
                        f"{golden}/{path} differs from the generator's "
                        f"output, new_subsystem.package_files({name!r}, "
                        f"{lang!r}) (plus the golden filegroup in BUILD)")

    def test_position_archetype_generates_motion_magic_for_cpp_and_python(self):
        cpp_files = new_subsystem.package_files("wrist", "cpp", kind="position")
        self.assertIn("position_rot:double;", cpp_files["wrist_message.fbs"])
        self.assertIn("at_goal:bool;", cpp_files["wrist_message.fbs"])
        self.assertIn("set_motion_magic", cpp_files["wrist.h"])
        self.assertIn("Mode::kMotionMagic", cpp_files["node_test.cc"])

        py_files = new_subsystem.package_files("wrist", "py", kind="position")
        self.assertIn("set_motion_magic", py_files["wrist.py"])
        self.assertIn("talos.Mode.MOTION_MAGIC", py_files["node_test.py"])

    def test_target_lanes_replace_the_arbiter(self):
        for kind, field in (("velocity", "velocity_rps"),
                            ("position", "position_rot")):
            with self.subTest(kind):
                cpp = new_subsystem.package_files("wrist", "cpp", kind=kind)
                fbs = cpp["wrist_message.fbs"]
                target = fbs.split("struct WristTarget {")[1].split("}")[0]
                # 8 bytes, wire-compatible with { double demand; }.
                self.assertEqual(target.strip(), f"{field}:double;")
                self.assertIn("talos::Target<Target> target_;", cpp["wrist.h"])
                self.assertIn("target_.get()", cpp["wrist.h"])
                self.assertIn("std::isfinite", cpp["wrist.h"])
                for case in ("DisabledRobot", "StaleLane", "ReleasedLane",
                             "NeverSent", "DriverStationLoss",
                             "NonFiniteDemand"):
                    self.assertIn(f"{case}GoesNeutral", cpp["node_test.cc"])
                self.assertIn("PublishTarget(sim", cpp["node_test.cc"])
                self.assertIn("ReleaseTarget<Target>", cpp["node_test.cc"])

                py = new_subsystem.package_files("wrist", "py", kind=kind)
                self.assertIn("talos.Target(node, WristTarget)", py["wrist.py"])
                self.assertIn("math.isfinite", py["wrist.py"])
                for case in ("disabled_robot", "stale_lane", "released_lane",
                             "never_sent", "driver_station_loss",
                             "non_finite_demand"):
                    self.assertIn(f"test_{case}_sends_neutral",
                                  py["node_test.py"])
                self.assertIn(f"{field}=demand", py["node_test.py"])
                for files in (cpp, py):
                    for content in files.values():
                        self.assertNotIn("arbiter", content)


if __name__ == "__main__":
    unittest.main()
