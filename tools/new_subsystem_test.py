"""Unit tests for //tools:new_subsystem."""

import os
import re
import tempfile
import tomllib
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

        # Abbreviations, reserved owners and language/module collisions.
        for name in ("tgt", "drive", "wrist_out", "hw", "class", "sys",
                     "python", "node"):
            with self.subTest(name=name):
                self.assertIsNotNone(new_subsystem.check_name(name))

        # Too long to leave room for /<name>/target/teleop (31 bytes max)
        self.assertIsNotNone(new_subsystem.check_name("a" * 17))
        self.assertIsNone(new_subsystem.check_name("a" * 16))

    def test_check_name_uses_the_rules_of_names_h(self):
        # check_name copies talOS/introspection/names.h's limits and lists;
        # a rule changed on one side only fails here.
        with open(os.path.join("talOS", "introspection", "names.h"),
                  encoding="utf-8") as f:
            header = f.read()

        def number(constant):
            return int(re.search(rf"\b{constant} = (\d+);", header).group(1))

        def strings(array):
            body = re.search(rf"\b{array}\[\] = \{{(.*?)\}};", header,
                             re.DOTALL).group(1)
            return re.findall(r'"([^"]*)"', body)

        self.assertEqual(number("kMaxSegmentBytes"),
                         new_subsystem.MAX_SEGMENT_BYTES)
        self.assertEqual(number("kMaxTopicBytes"),
                         new_subsystem.MAX_TOPIC_BYTES)
        self.assertEqual(set(strings("kReservedOwners")),
                         new_subsystem.RESERVED_OWNERS)
        pairs = strings("kAbbreviations")
        self.assertEqual(dict(zip(pairs[::2], pairs[1::2])),
                         new_subsystem.ABBREVIATIONS)
        self.assertEqual(set(strings("kDirectionSuffixes")),
                         set(new_subsystem.DIRECTION_SUFFIXES))

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

    def test_plan_confines_a_position_mechanism_until_its_travel_is_set(self):
        # The gateway and the devices layer bound a position demand only
        # with soft_limits, and validation refuses a zero range, so the
        # placeholder travel is a window narrower than the at-goal tolerance.
        with tempfile.TemporaryDirectory() as root:
            configuration = os.path.join(root, "bot", "main_processor",
                                         "configuration")
            os.makedirs(os.path.join(configuration, "subsystems"))
            for path, text in (
                    ("BUILD", ""),
                    ("robot.toml", '[robot]\nname = "bot"\n\n[[subsystems]]\n'
                     'name = "shooter"\npath = "subsystems/shooter.toml"\n'),
                    ("subsystems/shooter.toml",
                     '[subsystem]\nname = "shooter"\n')):
                with open(os.path.join(configuration, path), "w",
                          encoding="utf-8") as f:
                    f.write(text)
            writes, _ = new_subsystem.plan(root, "bot", "wrist", "cpp",
                                           kind="position")
            subsystem = tomllib.loads(next(
                new for path, _, new in writes
                if path == os.path.join(configuration, "subsystems",
                                        "wrist.toml")))
        motor = subsystem["motors"]["wrist_motor"]
        self.assertIs(motor["soft_limits"], True)
        travel = motor["forward_limit_rot"] - motor["reverse_limit_rot"]
        self.assertGreater(travel, 0)
        self.assertLess(travel, subsystem["subsystem"]["tolerance_rot"])


    def test_generated_packages_match_the_built_golden_packages(self):
        # tools/scaffold_testdata holds the generator's output, built and
        # tested by `bazel test //...`: the templates compile and their
        # tests pass only while this equality holds. Each golden BUILD adds
        # one filegroup, which hands the files to this test.
        for name, lang, kind in (("gripper", "cpp", "velocity"),
                                 ("scaffold_py", "py", "velocity"),
                                 ("position_cpp", "cpp", "position"),
                                 ("position_py", "py", "position"),
                                 ("localizer", "cpp", "estimator"),
                                 ("py_localizer", "py", "estimator")):
            golden = os.path.join("tools", "scaffold_testdata", name)
            on_disk = {}
            for directory, _, files in os.walk(golden):
                for f in files:
                    path = os.path.join(directory, f)
                    with open(path, encoding="utf-8") as text:
                        on_disk[os.path.relpath(path, golden).replace(
                            os.sep, "/")] = text.read()
            generated = new_subsystem.package_files(name, lang, kind=kind)
            generated["BUILD"] += GOLDEN_FILEGROUP
            with self.subTest(name):
                self.assertEqual(sorted(on_disk), sorted(generated))
                for path, content in generated.items():
                    self.assertEqual(
                        on_disk[path], content,
                        f"{golden}/{path} differs from the generator's "
                        f"output, new_subsystem.package_files({name!r}, "
                        f"{lang!r}, kind={kind!r}) (plus the golden "
                        f"filegroup in BUILD)")


    def test_plan_wires_an_estimator_without_a_can_id(self):
        with tempfile.TemporaryDirectory() as root:
            configuration = os.path.join(root, "bot", "main_processor",
                                         "configuration")
            os.makedirs(os.path.join(configuration, "subsystems"))
            with open(os.path.join(configuration, "robot.toml"), "w") as f:
                f.write('[robot]\nname = "bot"\nperiod_us = 5000\n'
                        'command_timeout_us = 100000\n\n[[subsystems]]\n'
                        'name = "drivetrain"\n'
                        'path = "subsystems/drivetrain.toml"\n')
            with open(os.path.join(configuration, "subsystems",
                                   "drivetrain.toml"), "w") as f:
                f.write('[subsystem]\nname = "drivetrain"\n\n'
                        '[motors.drive]\ntype = "TalonFX"\nbus = "rio"\n'
                        'can_id = 1\n')
            open(os.path.join(configuration, "BUILD"), "w").close()
            writes, can_id = new_subsystem.plan(root, "bot", "localizer", "py",
                                                kind="estimator")
            self.assertIsNone(can_id)
            by_rel = {os.path.relpath(path, root): new
                      for path, _, new in writes}
            subsystem = by_rel[
                "bot/main_processor/configuration/subsystems/localizer.toml"]
            self.assertIn("cameras = []", subsystem)
            self.assertNotIn("can_id", subsystem)
            self.assertIn('name = "localizer"', by_rel[
                "bot/main_processor/configuration/robot.toml"])


if __name__ == "__main__":
    unittest.main()
