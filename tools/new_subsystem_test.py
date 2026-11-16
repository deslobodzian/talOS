"""Unit tests for //tools:new_subsystem."""

import ast
import collections
import contextlib
import io
import os
import posixpath
import re
import tempfile
import textwrap
import tomllib
import unittest
from unittest import mock

from tools import new_subsystem

GOLDEN_FILEGROUP = """
# Golden copy of //tools:new_subsystem output; only this target is not generated.
filegroup(
    name = "golden",
    srcs = glob(["**"]),
    visibility = ["//tools:__pkg__"],
)
"""


def manifest_text(root, config):
    """The text of `config`'s manifest under `root`."""
    path = new_subsystem.sync_manifest.manifest_path(config)
    with open(os.path.join(root, path), encoding="utf-8") as f:
        return f.read()


# A robot config with one subsystem of its own, and that subsystem's file.
ROBOT_WITH_ROLLER = ('[robot]\nname = "robot_repo"\n\n[[subsystems]]\n'
                     'name = "roller"\npath = "subsystems/roller.toml"\n')
ROLLER = '[subsystem]\nname = "roller"\nnode = "//roller:node"\n'


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

    # REQ-DEV-01
    def test_subsystem_file_keys_are_the_keys_of_config_parser_h(self):
        # A position node's parameter table is [<name>], so check_kind
        # refuses a name the config parser reads or refuses in a subsystem
        # file. SUBSYSTEM_FILE_KEYS copies those lists; a key added on one
        # side only fails here, before a scaffold's table collides with it.
        with open(os.path.join("talOS", "configuration", "config_parser.h"),
                  encoding="utf-8") as f:
            header = f.read()

        def strings(pattern):
            body = re.search(pattern, header, re.DOTALL).group(1)
            return re.findall(r'"([^"]*)"', body)

        framework = strings(
            r"\bkFrameworkSubsystemKeys\[\] = \{(.*?)\};")
        pairs = strings(r"\bkSections\[\] = \{(.*?)\};")
        self.assertTrue(framework and pairs)
        self.assertEqual(new_subsystem.SUBSYSTEM_FILE_KEYS,
                         {"subsystem", *framework, *pairs[::2]})

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
                    # It requires target coverage, so the new subsystem's
                    # file lists no lane until its producer is added.
                    ("practice/robot.toml",
                     'include = "../configuration/robot.toml"\n'
                     '[robot]\nrequire_target_coverage = true\n'),
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
            self.assertEqual(tomllib.loads(subsystem)["targets"],
                             {"teleop": [], "autonomous": [], "test": []})
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

    # REQ-DEV-01
    def test_each_mechanism_gets_its_own_teleop_button(self):
        # Two mechanisms on one button both move on one press. Each scaffold
        # binds a button no other binding of the robot uses: neither one
        # live in the simulation config (the first mechanism's, appended as
        # its `Next:` says) nor one still commented in a subsystem file (the
        # second's, not appended yet). Its `Next:` runs the simulation with
        # that button held.
        with tempfile.TemporaryDirectory() as root:
            configuration = os.path.join(root, "bot", "main_processor",
                                         "configuration")
            os.makedirs(os.path.join(configuration, "subsystems"))
            sim = os.path.join(configuration, "robot_sim.toml")
            for path, text in (
                    ("BUILD", ""),
                    ("robot.toml",
                     '[robot]\nname = "bot"\n\n[[subsystems]]\n'
                     'name = "teleop"\npath = "subsystems/teleop.toml"\n'),
                    ("subsystems/teleop.toml",
                     '[subsystem]\nname = "teleop"\n'
                     'node = "//talOS/teleop:node"\n'),
                    ("robot_sim.toml", new_subsystem.fill(
                        new_subsystem.SIM_CONFIG_HEADER,
                        CONFIG="bot/main_processor/configuration/"
                        "robot_sim.toml")),
                    ("robot_sim_manifest.bzl", "")):
                with open(os.path.join(configuration, path), "w",
                          encoding="utf-8") as f:
                    f.write(text)

            def scaffold(name, *args):
                out = io.StringIO()
                with mock.patch.dict(os.environ,
                                     BUILD_WORKSPACE_DIRECTORY=root), \
                        contextlib.redirect_stdout(out):
                    status = new_subsystem.main(
                        [name, "--robot", "bot", *args])
                self.assertEqual(status, 0, out.getvalue())
                return out.getvalue()

            def button(steps):
                (bound,) = re.findall(r"^ +button = (\d+)$", steps, re.M)
                (mask,) = re.findall(r"\|3:axes=0;buttons=(\d+);", steps)
                self.assertEqual(int(mask), 1 << (int(bound) - 1), steps)
                return int(bound)

            first = scaffold("wrist", "--lang", "py")
            block = first.split("gains zero:\n")[1].split("  Then")[0]
            with open(sim, "a", encoding="utf-8") as f:
                f.write(textwrap.dedent(block))
            second = scaffold("arm", "--lang", "cpp", "--type", "position")
            third = scaffold("roller", "--lang", "py")
            buttons = [button(steps) for steps in (first, second, third)]
            self.assertEqual(len(set(buttons)), 3, buttons)

    # REQ-DEV-01
    def test_talos_names_its_robots_when_none_is_given(self):
        # Inside talOS no robot is the default: without --robot (or
        # --standalone) the scaffold is refused, naming the robot
        # directories at the workspace root, and nothing is written.
        with tempfile.TemporaryDirectory() as root:
            for robot in ("my_robot", "bench"):
                configuration = os.path.join(root, robot, "main_processor",
                                             "configuration")
                os.makedirs(configuration)
                with open(os.path.join(configuration, "robot.toml"), "w",
                          encoding="utf-8") as f:
                    f.write(f'[robot]\nname = "{robot}"\n')
            os.makedirs(os.path.join(root, "examples", "main_processor"))

            def tree():
                return sorted(os.path.relpath(os.path.join(d, f), root)
                              for d, _, files in os.walk(root) for f in files)

            before = tree()
            out = io.StringIO()
            with mock.patch.dict(os.environ, BUILD_WORKSPACE_DIRECTORY=root), \
                    contextlib.redirect_stdout(out), \
                    contextlib.redirect_stderr(out):
                status = new_subsystem.main(["wrist", "--lang", "py"])
            self.assertEqual(status, 1, out.getvalue())
            self.assertIn("--robot", out.getvalue())
            self.assertIn("bench, my_robot", out.getvalue())
            self.assertEqual(tree(), before)

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

    def test_plan_homes_a_position_mechanism_and_confines_its_travel(self):
        # Positions count from a home switch, never from the power-on frame:
        # the motor requires a zero, has no hardware soft limits (which act
        # in that frame and can stop homing), and its placeholder travel,
        # clamped by the node, is narrower than the at-goal tolerance. The
        # homing parameters pass PositionMechanism's checks (home at
        # min_rot, |homing_v| toward the switch and at most half max_voltage,
        # homing on the generated switch), in both languages, and the switch
        # gets the lowest free DIO channel, as the motor gets the lowest free
        # CAN id.
        for lang in ("cpp", "py"):
            with self.subTest(lang=lang), tempfile.TemporaryDirectory() as root:
                configuration = os.path.join(root, "bot", "main_processor",
                                             "configuration")
                os.makedirs(os.path.join(configuration, "subsystems"))
                for path, text in (
                        ("BUILD", ""),
                        ("robot.toml",
                         '[robot]\nname = "bot"\n\n[[subsystems]]\n'
                         'name = "shooter"\npath = "subsystems/shooter.toml"\n'),
                        ("subsystems/shooter.toml",
                         '[subsystem]\nname = "shooter"\n'
                         '[digital_inputs.beam]\ndio = 0\n'
                         '[encoders.angle]\ndio_a = 1\ndio_b = 3\n')):
                    with open(os.path.join(configuration, path), "w",
                              encoding="utf-8") as f:
                        f.write(text)
                writes, _ = new_subsystem.plan(root, "bot", "wrist", lang,
                                               kind="position")
                subsystem = tomllib.loads(next(
                    new for path, _, new in writes
                    if path == os.path.join(configuration, "subsystems",
                                            "wrist.toml")))
                motor = subsystem["motors"]["wrist"]
                params = subsystem["wrist"]
                self.assertEqual(params["homing"], "limit_switch")
                self.assertIn(params["limit_switch"],
                              subsystem["digital_inputs"])
                self.assertEqual(params["min_rot"], 0)
                travel = params["max_rot"] - params["min_rot"]
                self.assertLessEqual(abs(params["homing_v"]),
                                     motor["max_voltage"] / 2)
                self.assertEqual(motor["position_frame"], "rezero")
                self.assertIs(motor["soft_limits"], False)
                self.assertGreater(travel, 0)
                self.assertLess(travel, params["tolerance_rot"])
                self.assertLess(params["homing_v"], 0)
                for key in ("homing_timeout_us", "homing_stall_rps",
                            "homing_stall_us", "home_margin_rot"):
                    self.assertGreater(params[key], 0, key)
                self.assertEqual(
                    subsystem["digital_inputs"]["wrist_home"]["dio"], 2)

    def test_position_name_cannot_shadow_a_subsystem_file_key(self):
        # A position node's parameter table is [<name>] in its subsystem
        # file (C++ and Python alike), so a name the framework reads there
        # ([targets]), refuses ([motor]) or heads it with ([subsystem]) is
        # refused.
        for name in ("targets", "motor", "subsystem"):
            for lang in ("cpp", "py"):
                with self.subTest(name=name, lang=lang):
                    self.assertIsNotNone(
                        new_subsystem.check_kind(name, lang, "position"))
        self.assertIsNone(new_subsystem.check_kind("wrist", "py", "position"))
        self.assertIsNone(new_subsystem.check_kind("targets", "py", "velocity"))

    def test_test_robot_has_a_neighbor_even_for_a_subsystem_so_named(self):
        # The generated feedback case disconnects the node's motor while
        # another subsystem's stays valid; a scaffold named like that
        # neighbor must not merge into it (its test would lose the
        # neighbor, or the config would name one subsystem twice).
        for lang in ("cpp", "py"):
            for kind in ("velocity", "position"):
                with self.subTest(lang=lang, kind=kind):
                    files = new_subsystem.package_files("neighbor", lang,
                                                        kind=kind)
                    robot = tomllib.loads(files["testdata/robot.toml"])
                    subsystems = robot["subsystems"]
                    names = ([entry["name"] for entry in subsystems]
                             if lang == "cpp" else list(subsystems))
                    self.assertEqual(len(set(names)), 2, names)
                    self.assertIn("neighbor", names)

    def test_generated_packages_match_the_built_golden_packages(self):
        # tools/scaffold_testdata holds the generator's output, built and
        # tested by `bazel test //...`: the templates compile and their
        # tests pass only while this equality holds. Each golden BUILD adds
        # one filegroup, which hands the files to this test.
        for name, lang, kind, raw in (
                ("gripper", "cpp", "velocity", False),
                ("scaffold_py", "py", "velocity", False),
                ("position_cpp", "cpp", "position", False),
                ("position_py", "py", "position", False),
                ("position_raw_cpp", "cpp", "position", True),
                ("position_raw_py", "py", "position", True),
                ("localizer", "cpp", "estimator", False),
                ("py_localizer", "py", "estimator", False)):
            golden = os.path.join("tools", "scaffold_testdata", name)
            on_disk = {}
            for directory, _, files in os.walk(golden):
                for f in files:
                    path = os.path.join(directory, f)
                    with open(path, encoding="utf-8") as text:
                        on_disk[os.path.relpath(path, golden).replace(
                            os.sep, "/")] = text.read()
            generated = new_subsystem.package_files(name, lang, kind=kind,
                                                    raw=raw)
            generated["BUILD"] += GOLDEN_FILEGROUP
            with self.subTest(name):
                self.assertEqual(sorted(on_disk), sorted(generated))
                for path, content in generated.items():
                    self.assertEqual(
                        on_disk[path], content,
                        f"{golden}/{path} differs from the generator's "
                        f"output, new_subsystem.package_files({name!r}, "
                        f"{lang!r}, kind={kind!r}, raw={raw!r}) (plus the "
                        f"golden filegroup in BUILD)")

    # REQ-DEV-01
    def test_raw_position_node_is_the_students_own_code(self):
        # --raw writes the homing and limits into the node's own file: the
        # package depends on no mechanism library, and the subsystem file
        # (robot and standalone) has no [<name>] table, which no node would
        # read and the launcher would refuse. --raw without --type position
        # is refused before anything is read.
        for lang in ("cpp", "py"):
            with self.subTest(lang=lang), tempfile.TemporaryDirectory() as root:
                configuration = os.path.join(root, "bot", "main_processor",
                                             "configuration")
                os.makedirs(os.path.join(configuration, "subsystems"))
                for path, text in (
                        ("BUILD", ""),
                        ("robot.toml",
                         '[robot]\nname = "bot"\n\n[[subsystems]]\n'
                         'name = "shooter"\npath = "subsystems/shooter.toml"\n'),
                        ("subsystems/shooter.toml",
                         '[subsystem]\nname = "shooter"\n')):
                    with open(os.path.join(configuration, path), "w",
                              encoding="utf-8") as f:
                        f.write(text)
                writes, _ = new_subsystem.plan(root, "bot", "arm", lang,
                                               kind="position", raw=True)
                by_rel = {os.path.relpath(path, root).replace(os.sep, "/"): new
                          for path, _, new in writes}
                for path, content in by_rel.items():
                    if path.startswith("bot/main_processor/arm/"):
                        for library in ("talOS/mechanisms", "talos.mechanisms",
                                        "PositionMechanism"):
                            self.assertNotIn(library, content, path)
                subsystem = tomllib.loads(by_rel[
                    "bot/main_processor/configuration/subsystems/arm.toml"])
                self.assertNotIn("arm", subsystem)
                self.assertIn("arm_home", subsystem["digital_inputs"])
                self.assertEqual(subsystem["motors"]["arm"]["position_frame"],
                                 "rezero")
                defaults = tomllib.loads(new_subsystem.package_files(
                    "arm", lang, standalone=True, kind="position",
                    raw=True)["defaults.toml"])
                self.assertNotIn("arm", defaults)
        for kind in ("velocity", "estimator"):
            with self.subTest(kind=kind), \
                    contextlib.redirect_stderr(io.StringIO()) as err, \
                    self.assertRaises(SystemExit) as exit_:
                new_subsystem.main(["arm", "--lang", "cpp", "--type", kind,
                                    "--raw"])
            self.assertEqual(exit_.exception.code, 2)
            self.assertIn("--raw", err.getvalue())


    # REQ-DEV-01
    def test_scaffolds_into_a_robot_repository_that_depends_on_talos(self):
        # `bazel run @talos//tools:new_subsystem` from a robot repository
        # (talOS a dependency: sync_manifest.current_repository() is
        # `talos+`) writes the package where the config's own nodes are,
        # names talOS by @talos// and its own packages relative to its root,
        # and regenerates every manifest of a config that includes the edited
        # one, as manifest_sync_test checks it, without reading the talOS
        # submodule's configs. Two layouts: examples/robot_repo's (config and
        # packages at the root) and a talOS robot's moved into its own
        # repository (main_processor/configuration/robot.toml beside a
        # config including it, packages in main_processor/).
        def node(name, label):
            return (f'[subsystem]\nname = "{name}"\nnode = "{label}"\n'
                    'period_us = 20000\n')

        def robot(entries, coverage=False):
            return ('[robot]\nname = "robot_repo"\n'
                    + ("require_target_coverage = true\n" if coverage
                       else "")) + "".join(
                f'\n[[subsystems]]\nname = "{name}"\n'
                f'path = "subsystems/{name}.toml"\n' for name in entries)

        fixture = {
            "MODULE.bazel": 'module(name = "robot")\n',
            "BUILD.bazel": 'exports_files(glob(["subsystems/*.toml"]))\n',
            "robot.toml": robot(["teleop", "roller"]),
            "subsystems/teleop.toml":
                node("teleop", "@talos//talOS/teleop:node"),
            "subsystems/roller.toml": node("roller", "//roller:node")
                + '[motors.roller_motor]\ncan_id = 1\n',
            "roller/BUILD": "",
        }
        configuration = "main_processor/configuration"
        real = {
            "MODULE.bazel": 'module(name = "robot_2026")\n',
            "BUILD.bazel": "",
            f"{configuration}/BUILD":
                'exports_files(glob(["*.toml", "subsystems/*.toml"]))\n',
            f"{configuration}/robot.toml": robot(["teleop", "shooter"],
                                                 coverage=True),
            f"{configuration}/robot_field.toml":
                'include = "robot.toml"\n',
            f"{configuration}/subsystems/teleop.toml":
                node("teleop", "@talos//talOS/teleop:node"),
            f"{configuration}/subsystems/shooter.toml":
                node("shooter", "//main_processor/shooter:node"),
            "main_processor/shooter/BUILD": "",
        }
        # The talOS submodule: its configs are not the robot's, and this one
        # cannot even be read.
        submodule = {
            "talos/MODULE.bazel": 'module(name = "talos")\n',
            "talos/BUILD": "",
            "talos/robot.toml": 'include = "missing.toml"\n',
            "talos/robot_manifest.bzl": "",
        }
        talos_label = re.compile(r"(?<![@\w])//talOS/")
        for layout, files, packages, configs, scaffolds in (
                ("robot_repo", fixture, "", ["robot.toml"],
                 [("intake", "cpp"), ("arm", "py")]),
                ("2026", real, "main_processor",
                 [f"{configuration}/robot.toml",
                  f"{configuration}/robot_field.toml"],
                 [("climber", "py")])):
            with self.subTest(layout=layout), \
                    tempfile.TemporaryDirectory() as root, \
                    mock.patch.object(new_subsystem.sync_manifest,
                                      "current_repository",
                                      return_value="talos+"):
                for path, text in {**files, **submodule}.items():
                    os.makedirs(os.path.dirname(os.path.join(root, path)),
                                exist_ok=True)
                    with open(os.path.join(root, path), "w",
                              encoding="utf-8") as f:
                        f.write(text)
                for config in configs:
                    new_subsystem.sync_manifest.write(root, config)
                for name, lang in scaffolds:
                    out = io.StringIO()
                    with mock.patch.dict(os.environ,
                                         BUILD_WORKSPACE_DIRECTORY=root), \
                            contextlib.redirect_stdout(out), \
                            contextlib.redirect_stderr(out):
                        status = new_subsystem.main([name, "--lang", lang])
                    self.assertEqual(status, 0, out.getvalue())
                    package = posixpath.join(packages, name)
                    self.assertTrue(os.path.isfile(
                        os.path.join(root, package,
                                     {"cpp": "node_test.cc",
                                      "py": "node_test.py"}[lang])),
                        out.getvalue())
                    written = {}
                    for path in (posixpath.join(package, "BUILD"),
                                 posixpath.join(
                                     posixpath.dirname(configs[0]),
                                     "subsystems", f"{name}.toml")):
                        with open(os.path.join(root, path),
                                  encoding="utf-8") as f:
                            written[path] = f.read()
                    for path, text in written.items():
                        self.assertIsNone(talos_label.search(text), path)
                    self.assertIn(
                        'load("@talos//talOS/build:contract.bzl"',
                        written[posixpath.join(package, "BUILD")])
                    subsystem = tomllib.loads(written[posixpath.join(
                        posixpath.dirname(configs[0]), "subsystems",
                        f"{name}.toml")])
                    self.assertEqual(subsystem["subsystem"]["node"],
                                     f"//{package}:node")
                    # A robot that requires target coverage lints a mode
                    # only when its policy lists a lane: the new subsystem,
                    # with no producer yet, lists none in any mode (and
                    # stays neutral); otherwise [robot.targets] applies.
                    if layout == "2026":
                        self.assertEqual(subsystem.get("targets"),
                                         {"teleop": [], "autonomous": [],
                                          "test": []})
                    else:
                        self.assertNotIn("targets", subsystem)
                    with open(os.path.join(root, configs[0]), "rb") as f:
                        entries = tomllib.load(f)["subsystems"]
                    self.assertIn({"name": name,
                                   "path": f"subsystems/{name}.toml"},
                                  entries)
                    for config in configs:
                        # Current as manifest_sync_test sees it, and as
                        # sync_manifest itself writes it.
                        self.assertEqual(
                            new_subsystem.sync_manifest.check(root, config),
                            "", config)
                        manifest = manifest_text(root, config)
                        self.assertEqual(
                            manifest,
                            new_subsystem.sync_manifest.generate(root,
                                                                 config))
                        self.assertIn(f'    "//{package}:node",\n', manifest)
                # The first mechanism gave the repository its simulation
                # config; the second kept its manifest current.
                sim = posixpath.join(posixpath.dirname(configs[0]),
                                     "robot_sim.toml")
                self.assertEqual(
                    new_subsystem.sync_manifest.check(root, sim), "", sim)
                with open(os.path.join(root, "talos", "robot_manifest.bzl"),
                          encoding="utf-8") as f:
                    self.assertEqual(f.read(), "")

    # REQ-DEV-01
    def test_robot_repository_paths_stay_in_the_repository(self):
        # --config and --package-dir name places in the robot repository:
        # one that is absolute outside it, escapes it through `..`, or lies
        # in a nested repository (the talos/ submodule) is refused before
        # anything is written; such a path would write outside the robot,
        # or label its node `///tmp/...:node` or `//../x:node`.
        files = {
            "MODULE.bazel": 'module(name = "robot")\n',
            "BUILD.bazel": 'exports_files(glob(["subsystems/*.toml"]))\n',
            "robot.toml": ROBOT_WITH_ROLLER,
            "subsystems/roller.toml": ROLLER,
            "talos/MODULE.bazel": 'module(name = "talos")\n',
            "talos/bot/BUILD": "",
            "talos/bot/robot.toml": ROBOT_WITH_ROLLER,
            "talos/bot/subsystems/roller.toml": ROLLER,
        }

        def snapshot(directory):
            tree = {}
            for current, _, names in os.walk(directory):
                for f in names:
                    path = os.path.join(current, f)
                    with open(path, "rb") as text:
                        tree[path] = text.read()
            return tree

        for flag, value in (
                ("--config", "{outside}/robot.toml"),
                ("--config", "../outside/robot.toml"),
                ("--config", "talos/bot/robot.toml"),
                ("--package-dir", "{outside}"),
                ("--package-dir", "../outside"),
                ("--package-dir", "talos/bot")):
            with self.subTest(flag=flag, value=value), \
                    tempfile.TemporaryDirectory() as top, \
                    mock.patch.object(new_subsystem.sync_manifest,
                                      "current_repository",
                                      return_value="talos+"):
                root = os.path.join(top, "robot")
                outside = os.path.join(top, "outside")
                for path, text in {
                        **{f"robot/{p}": t for p, t in files.items()},
                        "outside/robot.toml": ROBOT_WITH_ROLLER,
                        "outside/subsystems/roller.toml": ROLLER,
                        "outside/BUILD": ""}.items():
                    os.makedirs(os.path.dirname(os.path.join(top, path)),
                                exist_ok=True)
                    with open(os.path.join(top, path), "w",
                              encoding="utf-8") as f:
                        f.write(text)
                before = snapshot(top)
                out = io.StringIO()
                with mock.patch.dict(os.environ,
                                     BUILD_WORKSPACE_DIRECTORY=root), \
                        contextlib.redirect_stdout(out), \
                        contextlib.redirect_stderr(out):
                    status = new_subsystem.main(
                        ["feeder", "--lang", "py", flag,
                         value.format(outside=outside)])
                self.assertNotEqual(status, 0, out.getvalue())
                self.assertIn(flag, out.getvalue())
                self.assertEqual(snapshot(top), before, out.getvalue())

    # REQ-DEV-01
    def test_standalone_from_a_robot_repository_builds_against_talos(self):
        # --standalone run from a robot repository writes a repository that
        # overrides @talos with the talOS checkout the tool runs from (the
        # robot's talos/ submodule) and whose .bazelrc imports that
        # checkout's talos.bazelrc by the same path, not the robot
        # repository's flags (no copied talOS-only lines);
        # when that talOS is Bazel's copy of a fetched module (under an
        # output base's external/), it refuses and writes nothing.
        def write(path, text):
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)

        for case in ("checkout", "fetched"):
            with self.subTest(case=case), \
                    tempfile.TemporaryDirectory() as top, \
                    mock.patch.object(new_subsystem.sync_manifest,
                                      "current_repository",
                                      return_value="talos+"):
                talos = (os.path.join(top, "robot", "talos")
                         if case == "checkout" else
                         os.path.join(top, "base", "external", "talos+"))
                os.makedirs(os.path.join(top, "base", "execroot"))
                for directory, name in ((talos, "talos"),
                                        (os.path.join(top, "robot"),
                                         "robot")):
                    write(os.path.join(directory, "MODULE.bazel"),
                          f'module(name = "{name}")\n')
                    write(os.path.join(directory, "talos.bazelrc"),
                          f"# {name}\n")
                    write(os.path.join(directory, ".bazelversion"), "8.4.2\n")
                script = os.path.join(talos, "tools", "new_subsystem.py")
                write(script, "")
                target = os.path.join(top, "arm")
                out = io.StringIO()
                with mock.patch.dict(os.environ,
                                     BUILD_WORKSPACE_DIRECTORY=os.path.join(
                                         top, "robot")), \
                        mock.patch.object(new_subsystem, "__file__", script), \
                        contextlib.redirect_stdout(out), \
                        contextlib.redirect_stderr(out):
                    status = new_subsystem.main(
                        ["arm", "--lang", "py", "--standalone", "../arm"])
                if case == "fetched":
                    self.assertNotEqual(status, 0, out.getvalue())
                    self.assertFalse(os.path.exists(target), out.getvalue())
                    continue
                self.assertEqual(status, 0, out.getvalue())
                with open(os.path.join(target, "MODULE.bazel"),
                          encoding="utf-8") as f:
                    (path,) = re.findall(
                        r'module_name = "talos",\n\s*path = "([^"]*)"',
                        f.read())
                self.assertEqual(
                    os.path.realpath(os.path.join(target, path)),
                    os.path.realpath(talos))
                # Bazel's reading of the import: %workspace% is the new
                # repository's root.
                with open(os.path.join(target, ".bazelrc"),
                          encoding="utf-8") as f:
                    (imported,) = re.findall(r"^import (\S+)$", f.read(),
                                             re.MULTILINE)
                imported = imported.replace("%workspace%", target)
                with open(imported, encoding="utf-8") as f:
                    self.assertEqual(f.read(), "# talos\n", imported)

    # REQ-DEV-01
    def test_robot_repository_simulation_steps_run_the_simulation_config(self):
        # A robot repository has no simulation config until its first
        # mechanism, whose scaffold writes one: it includes
        # the robot config by its own name, from its own directory, so it
        # resolves with the new subsystem in it; its manifest is current;
        # and the config's BUILD gains the manifest_sync_test and
        # talos_graph_test that make `bazel test //...` check it, with
        # their loads among the BUILD's own (buildifier's order, one load
        # per file). The block `Next:` prints is appended to it, and the
        # printed run runs that config: `bazel run //:sim` only when the
        # root talos_sim names it, else the launcher on it with --sim.
        files = {
            "MODULE.bazel": 'module(name = "robot")\n',
            "cfg/practice.toml": ROBOT_WITH_ROLLER,
            "cfg/subsystems/roller.toml": ROLLER,
            "cfg/BUILD": ('load("@talos//talOS/build:contract.bzl", '
                          '"talos_bundle")\n'
                          'load(":practice_manifest.bzl", "NODES")\n\n'
                          'exports_files(["practice.toml"])\n'),
            "roller/BUILD": "",
        }
        for sim in ("//cfg:practice.toml", "//cfg:robot_sim.toml"):
            with self.subTest(sim=sim), \
                    tempfile.TemporaryDirectory() as root, \
                    mock.patch.object(new_subsystem.sync_manifest,
                                      "current_repository",
                                      return_value="talos+"):
                build = (f'talos_sim(\n    name = "sim",\n'
                         f'    config = "{sim}",\n)\n')
                for path, text in {**files, "BUILD.bazel": build}.items():
                    os.makedirs(os.path.dirname(os.path.join(root, path)),
                                exist_ok=True)
                    with open(os.path.join(root, path), "w",
                              encoding="utf-8") as f:
                        f.write(text)
                out = io.StringIO()
                with mock.patch.dict(os.environ,
                                     BUILD_WORKSPACE_DIRECTORY=root), \
                        contextlib.redirect_stdout(out), \
                        contextlib.redirect_stderr(out):
                    status = new_subsystem.main(
                        ["feeder", "--lang", "py", "--config",
                         "cfg/practice.toml"])
                steps = out.getvalue()
                self.assertEqual(status, 0, steps)
                self.assertEqual(new_subsystem.sync_manifest.check(
                    root, "cfg/robot_sim.toml"), "", steps)
                with open(os.path.join(root, "cfg", "BUILD"),
                          encoding="utf-8") as f:
                    build = f.read()
                self.assertEqual(
                    re.findall(r"^load\((.*)\)$", build, re.M),
                    ['"@talos//talOS/build:contract.bzl", "talos_bundle", '
                     '"talos_graph_test"',
                     '"@talos//tools:sync_manifest.bzl", "manifest_sync_test"',
                     '":practice_manifest.bzl", "NODES"',
                     '":robot_sim_manifest.bzl", SIM_CONFIG_FILES = '
                     '"CONFIG_FILES", SIM_NODES = "NODES"'])
                for rule, name in (("manifest_sync_test",
                                    "sim_manifest_sync_test"),
                                   ("talos_graph_test", "sim_graph_test")):
                    self.assertRegex(
                        build, rf'\n{rule}\(\n    name = "{name}",\n'
                        r'    config = "robot_sim\.toml",\n')
                block = steps.split("gains zero:\n")[1].split("\n  Then")[0]
                with open(os.path.join(root, "cfg", "robot_sim.toml"), "a",
                          encoding="utf-8") as f:
                    f.write(textwrap.dedent(block))
                subsystems, _ = new_subsystem.sync_manifest.effective(
                    root, "cfg/robot_sim.toml")
                self.assertIn("feeder", subsystems)
                (run,) = [line.strip() for line in steps.splitlines()
                          if line.strip().startswith("bazel run")
                          and "--duration-s" in line]
                if sim.endswith("robot_sim.toml"):
                    self.assertTrue(run.startswith("bazel run //:sim --"),
                                    steps)
                else:
                    self.assertIn("--config $PWD/cfg/robot_sim.toml --sim",
                                  run)

    # REQ-DEV-01
    def test_simulation_test_loads_keep_the_build_parsing(self):
        # The loads the first mechanism adds to a robot repository's config
        # BUILD (sim_test_loads) never leave it unparsable, never bind a
        # name twice, keep its comments, and go below a header comment or
        # docstring. A load the merger does not rewrite (a comment inside,
        # single quotes) gains a separate load of the missing symbols; a
        # name bound to something else leaves the BUILD alone (None: the
        # scaffold writes nothing to it and `Next:` says nothing checks the
        # simulation config).
        contract = 'load("@talos//talOS/build:contract.bzl", "talos_graph_test")'
        sync = 'load("@talos//tools:sync_manifest.bzl", "manifest_sync_test")'
        manifest = ('load(":robot_sim_manifest.bzl", SIM_CONFIG_FILES = '
                    '"CONFIG_FILES", SIM_NODES = "NODES")')
        body = 'exports_files(["robot.toml"])\n'
        rows = (
            ("comment inside a multi-line load",
             'load(\n    "@talos//talOS/build:contract.bzl",\n'
             '    "talos_bundle",  # the robot bundle\n)\n\n' + body,
             'load(\n    "@talos//talOS/build:contract.bzl",\n'
             '    "talos_bundle",  # the robot bundle\n)\n'
             f"{contract}\n{sync}\n{manifest}\n\n" + body),
            ("comment after the load",
             f"{contract}  # the graph lint\n\n" + body,
             f"{contract}  # the graph lint\n{sync}\n{manifest}\n\n" + body),
            ("single quotes",
             "load('@talos//talOS/build:contract.bzl', 'talos_graph_test')\n"
             "\n" + body,
             "load('@talos//talOS/build:contract.bzl', 'talos_graph_test')\n"
             f"{sync}\n{manifest}\n\n" + body),
            ("no final newline", contract,
             f"{contract}\n{sync}\n{manifest}\n"),
            ("no loads below a header and docstring",
             '# The robot configuration.\n\n"""Its configs."""\n\n' + body,
             '# The robot configuration.\n\n"""Its configs."""\n\n'
             f"{contract}\n{sync}\n{manifest}\n\n" + body),
            ("plain load merged",
             'load("@talos//talOS/build:contract.bzl", "talos_bundle")\n\n'
             + body,
             'load("@talos//talOS/build:contract.bzl", "talos_bundle", '
             f'"talos_graph_test")\n{sync}\n{manifest}\n\n' + body),
            ("name bound elsewhere",
             'load("//rules:graph.bzl", talos_graph_test = "graph_test")\n\n'
             + body, None),
        )

        def comments(text):
            return [line.split("#", 1)[1] for line in text.splitlines()
                    if "#" in line]

        def bindings(text):
            bound = collections.Counter()
            for call in ast.walk(ast.parse(text)):
                if (isinstance(call, ast.Call)
                        and getattr(call.func, "id", None) == "load"):
                    bound.update(arg.value for arg in call.args[1:])
                    bound.update(keyword.arg for keyword in call.keywords)
            return bound

        for shape, before, after in rows:
            with self.subTest(shape=shape):
                build = new_subsystem.sim_test_loads(
                    before, "@talos", "robot_sim_manifest.bzl")
                self.assertEqual(build, after)
                if after is None:
                    continue
                bound = bindings(build)  # parses
                for name in ("talos_graph_test", "manifest_sync_test",
                             "SIM_CONFIG_FILES", "SIM_NODES"):
                    self.assertEqual(bound[name], 1, name)
                self.assertEqual(comments(build), comments(before))

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
