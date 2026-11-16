"""Tests for //tools:new_robot: the generated robot, with a mechanism or
none, passes the launcher's graph lint as generated, the edits its `Next:` output lists make its
mechanism drivable, and a name that cannot be a robot is refused.

With --standalone (RobotRepositoryTest), on a stand-in fixture: the
repository is the fixture as TEMPLATE places it, a directory in talOS or
already used is refused, and a fixture the template does not match refuses
the copy. tools/check_robot_repo_scaffold.sh builds and tests what it makes
of the real fixture, which a Bazel test cannot read (.bazelignore)."""

import json
import os
import re
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from python.runfiles import runfiles

from tools import new_robot
from tools import new_subsystem
from tools import sync_manifest

CONFIG = "my_robot/main_processor/configuration/robot.toml"
SIM_CONFIG = "my_robot/main_processor/configuration/robot_sim.toml"
SUBSYSTEMS = "my_robot/main_processor/configuration/subsystems"


def workspace(tmp):
    """A workspace in `tmp` that mirrors the runfiles tree, with a bazel-bin
    beside it, the layout the launcher resolves node labels in (the one
    talos_graph_test builds)."""
    r = runfiles.Create()
    launcher = r.Rlocation("_main/talOS/launcher/launcher")
    root = os.environ.get("RUNFILES_DIR")
    if not root or not os.path.isdir(os.path.join(root, "_main")):
        root = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(launcher))))
    main = os.path.join(root, "_main")
    work = os.path.join(tmp, "ws")
    bazel_bin = os.path.join(work, "bazel-bin")
    os.makedirs(bazel_bin)
    for entry in os.listdir(main):
        os.symlink(os.path.join(main, entry), os.path.join(work, entry))
        os.symlink(os.path.join(main, entry), os.path.join(bazel_bin, entry))
    env = dict(os.environ, **r.EnvVars())
    env["RUNFILES_DIR"] = root
    env.pop("PYTHONPATH", None)
    env.pop("BUILD_WORKSPACE_DIRECTORY", None)
    return work, launcher, env


def stand_in_binary(work, mechanism, lang):
    """Puts a node binary where `bazel build` of the generated package would.

    A C++ mechanism is new_subsystem's output, which
    tools/scaffold_testdata/<mechanism> holds built (new_subsystem_test keeps
    the two equal), so its binary stands in. A Python mechanism runs the
    generated file itself, the way talos_py_node's runner does, with the
    message modules talos_messages built from the golden package's identical
    schema: a test cannot run flatc and descriptor_gen on the new one.
    """
    package = os.path.join(work, "bazel-bin", "my_robot", "main_processor",
                           mechanism)
    os.makedirs(package)
    node = os.path.join(package, "node")
    golden = os.path.join(work, "tools", "scaffold_testdata", mechanism)
    if lang == "cpp":
        os.symlink(os.path.join(golden, "node"), node)
        return
    source = os.path.join(work, "my_robot", "main_processor", mechanism,
                          f"{mechanism}.py")
    runner = (
        "import sys, talos\n"
        f"node = talos.load({source!r})\n"
        f"sys.exit(talos.run(node.{new_subsystem.pascal(mechanism)}, "
        f"{mechanism!r}, '//my_robot/main_processor/{mechanism}:node'))\n")
    with open(os.path.join(package, "node_runner.py"), "w",
              encoding="utf-8") as f:
        f.write(runner)
    path = os.pathsep.join(
        [os.path.join(golden, "messages_py_gen")] + sys.path[1:])
    with open(node, "w", encoding="utf-8") as f:
        f.write("#!/bin/sh\n"
                f"PYTHONPATH={shlex.quote(path)} exec "
                f"{shlex.quote(sys.executable)} "
                f"{shlex.quote(os.path.join(package, 'node_runner.py'))} "
                '"$@"\n')
    os.chmod(node, 0o755)


def generate(work, mechanism, lang, kind="velocity"):
    writes, can_id = new_robot.plan(work, "my_robot", mechanism, lang, kind)
    for path, _, content in writes:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
    if mechanism is not None:
        stand_in_binary(work, mechanism, lang)
    return can_id


def append_simulation_block(work, mechanism, kind,
                            button=new_subsystem.TELEOP_BUTTONS[0]):
    """Appends `mechanism`'s simulation block, binding `button`, to the
    robot's simulation config, as its `Next:` says."""
    with open(os.path.join(work, SIM_CONFIG), "a", encoding="utf-8") as f:
        f.write(new_subsystem.simulation_block(mechanism, kind, button))


def describe(work, launcher, env, out, config=CONFIG):
    """The launcher's --describe-only on the generated robot: (exit status,
    output, graph.json or None)."""
    proc = subprocess.run(
        [launcher, "--config", config, "--describe-only", "--output-dir",
         out],
        cwd=work, env=env, capture_output=True, text=True, timeout=60)
    graph = None
    sessions = os.listdir(out) if os.path.isdir(out) else []
    if proc.returncode == 0 and len(sessions) == 1:
        with open(os.path.join(out, sessions[0], "graph.json"),
                  encoding="utf-8") as f:
            graph = json.load(f)
    return proc.returncode, proc.stdout + proc.stderr, graph


def edit(path, replacements):
    with open(path, encoding="utf-8") as f:
        text = f.read()
    for old, new in replacements:
        if text.count(old) != 1:
            raise AssertionError(f"{path} holds {text.count(old)} copies of "
                                 f"{old!r}, not one")
        text = text.replace(old, new)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


class NewRobotTest(unittest.TestCase):
    # REQ-DEV-23
    def test_generated_robot_passes_the_graph_lint(self):
        # C++ with the golden binary; Python with the generated node file.
        for mechanism, lang in (("gripper", "cpp"), ("scaffold_py", "py")):
            with self.subTest(lang=lang), tempfile.TemporaryDirectory() as tmp:
                work, launcher, env = workspace(tmp)
                self.assertEqual(generate(work, mechanism, lang), 1)
                # What its manifest_sync_test and sim_manifest_sync_test
                # check.
                self.assertEqual(sync_manifest.check(work, CONFIG), "")
                self.assertEqual(sync_manifest.check(work, SIM_CONFIG), "")

                status, output, graph = describe(work, launcher, env,
                                                 os.path.join(tmp, "out"))
                self.assertEqual(status, 0, output)
                self.assertNotIn("error:", output)
                self.assertEqual(
                    sorted(n["name"] for n in graph["nodes"]),
                    sorted([mechanism, "hardware_node", "telemetry",
                            "teleop"]))

    # REQ-CONTRACT-35
    def test_robot_with_no_mechanism_passes_the_graph_lint(self):
        # A robot with no device: the hardware bridge, teleop with nothing to
        # bind and telemetry. Its manifests are not written by new_subsystem,
        # so both must come out current here.
        with tempfile.TemporaryDirectory() as tmp:
            work, launcher, env = workspace(tmp)
            self.assertIsNone(generate(work, None, None))
            for config in (CONFIG, SIM_CONFIG):
                with self.subTest(config=config):
                    self.assertEqual(sync_manifest.check(work, config), "")
                    status, output, graph = describe(
                        work, launcher, env,
                        os.path.join(tmp, os.path.basename(config)), config)
                    self.assertEqual(status, 0, output)
                    self.assertNotIn("error:", output)
                    self.assertEqual(
                        sorted(n["name"] for n in graph["nodes"]),
                        ["hardware_node", "telemetry", "teleop"])
                    robot, _ = sync_manifest.effective(work, config)
                    for table in robot.values():
                        self.assertNotIn("motors", table)

    # REQ-CONTRACT-09
    def test_commissioning_needs_only_the_commissioned_flag(self):
        # A person commissions a generated robot by setting commissioned =
        # true at the robot (docs/BRINGUP.md); config load then requires
        # target coverage, so the generated config must not have turned it
        # off. With no mechanism every subsystem is covered, so the graph
        # passes. (A temporary copy, never a robot's own config: RULES.md 4.)
        with tempfile.TemporaryDirectory() as tmp:
            work, launcher, env = workspace(tmp)
            generate(work, None, None)
            edit(os.path.join(work, CONFIG),
                 [("commissioned = false", "commissioned = true")])
            status, output, graph = describe(work, launcher, env,
                                             os.path.join(tmp, "out"))
            self.assertEqual(status, 0, output)
            self.assertIsNotNone(graph)

    # REQ-SAFE-40
    def test_simulation_config_is_refused_without_sim(self):
        # robot_sim.toml holds simulation gains, bindings and travel (the
        # blocks new_subsystem's `Next:` appends): the launcher refuses to
        # start it without --sim, before anything is probed or spawned.
        # --duration-s bounds a launch that wrongly starts.
        with tempfile.TemporaryDirectory() as tmp:
            work, launcher, env = workspace(tmp)
            generate(work, None, None)
            proc = subprocess.run(
                [launcher, "--config", SIM_CONFIG, "--duration-s", "1",
                 "--output-dir", os.path.join(tmp, "out")],
                cwd=work, env=env, capture_output=True, text=True, timeout=60)
            output = proc.stdout + proc.stderr
            self.assertEqual(proc.returncode, 1, output)
            self.assertIn("sets [robot] simulation_only = true; run it with "
                          "--sim (or --describe-only); nothing was started",
                          output)

    # REQ-DEV-23
    def test_next_steps_block_makes_the_mechanism_drivable_in_simulation(
            self):
        # A scaffold leaves its teleop binding commented and its gains zero.
        # This robot turns target coverage on (2026-robot's policy) after the
        # scaffold, so its subsystem file lists the default lanes with no
        # writer and the launcher refuses it. The simulation block `Next:`
        # prints, appended to the simulation config, gives the mechanism a
        # producer in every mode the policy covers and a teleop target it can
        # reach there, and leaves robot.toml's mechanism unbound. Whether the
        # block's values move the motor is scaffold_sim_test's (e2e).
        for mechanism, kind in (("gripper", "velocity"),
                                ("position_cpp", "position")):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as tmp:
                work, launcher, env = workspace(tmp)
                generate(work, mechanism, "cpp", kind)
                edit(os.path.join(work, CONFIG),
                     [("commissioned = false\n",
                       "commissioned = false\n"
                       "require_target_coverage = true\n"),
                      ("autonomous = []", 'autonomous = ["auto"]')])
                status, output, _ = describe(work, launcher, env,
                                             os.path.join(tmp, "before"),
                                             SIM_CONFIG)
                self.assertNotEqual(status, 0, output)
                self.assertIn("target coverage required for mode 'teleop'",
                              output)

                append_simulation_block(work, mechanism, kind)
                status, output, _ = describe(work, launcher, env,
                                             os.path.join(tmp, "after"),
                                             SIM_CONFIG)
                self.assertEqual(status, 0, output)
                self.assertNotIn("error:", output)
                # ./sim.sh's manifest check still passes.
                self.assertEqual(sync_manifest.check(work, SIM_CONFIG), "")
                # The robot's own config gets neither the binding nor the
                # simulated gains.
                robot, _ = sync_manifest.effective(work, CONFIG)
                self.assertNotIn("teleop", robot[mechanism])
                for motor in robot[mechanism]["motors"].values():
                    self.assertEqual(set(motor["slot0"].values()), {0.0})

                # The bound target is one the node does not clamp away.
                subsystems, _ = sync_manifest.effective(work, SIM_CONFIG)
                table = subsystems[mechanism]
                (binding,) = table["teleop"]["bindings"]
                (motor,) = table["motors"].values()
                if kind == "position":
                    params = table[mechanism]
                    goal = binding["while_held"]["position_rot"]
                    self.assertGreater(goal - params["min_rot"],
                                       params["tolerance_rot"])
                    self.assertLessEqual(goal, params["max_rot"])
                else:
                    speed = binding["while_held"]["velocity_rps"]
                    self.assertGreater(speed, 0)
                    self.assertLessEqual(speed, motor["max_velocity_rps"])

    # REQ-DEV-23
    def test_refuses_a_bad_name_or_an_existing_directory(self):
        with tempfile.TemporaryDirectory() as root:
            for name in ("My_robot", "my-robot", "hw", "robot_", "sys"):
                with self.subTest(name=name):
                    with self.assertRaisesRegex(new_subsystem.Refused,
                                                "robot name"):
                        new_robot.plan(root, name, "wrist", "py")
            with self.assertRaisesRegex(new_subsystem.Refused,
                                        "node name 'tgt'"):
                new_robot.plan(root, "my_robot", "tgt", "py")
            os.makedirs(os.path.join(root, "taken"))
            with self.assertRaisesRegex(new_subsystem.Refused,
                                        "already exists"):
                new_robot.plan(root, "taken", "wrist", "py")
            with self.assertRaisesRegex(new_subsystem.Refused,
                                        "outside the workspace"):
                new_robot.plan(root, "my_robot", "wrist", "py", parent="..")
            self.assertEqual(os.listdir(root), ["taken"])


FIXTURE_MODULE = '''module(
    name = "robot",
    version = "0.1.0",
)

bazel_dep(name = "talos", version = "0.1.0")
local_path_override(
    module_name = "talos",
    path = "../..",
)

studio_web(
    name = "studio_web",
    path = "../../bazel-bin/studio/dist",
)
'''

FIXTURE_BAZELRC = '''# talOS's flags.
import %workspace%/../../talos.bazelrc

build:release --workspace_status_command=../../tools/workspace_status.sh
'''

FIXTURE_ROBOT = '''[robot]
name = "robot_repo"
hardware_node = "@talos//talOS/bridge:hardware_node"

[[subsystems]]
name = "teleop"
path = "subsystems/teleop.toml"

[[subsystems]]
name = "telemetry"
path = "subsystems/telemetry.toml"

[[subsystems]]
name = "roller"
path = "subsystems/roller.toml"

[[subsystems]]
name = "gate"
path = "subsystems/gate.toml"
'''

FIXTURE_MANIFEST = '''"""Generated from robot.toml by @talos//tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run @talos//tools:sync_manifest -- robot.toml
"""

# Every node label robot.toml launches.
NODES = [
    "//gate:node",
    "//roller:node",
    "@talos//talOS/telemetry:node",
    "@talos//talOS/teleop:node",
]

# Every file robot.toml reads besides itself.
CONFIG_FILES = [
    "//:subsystems/gate.toml",
    "//:subsystems/roller.toml",
    "//:subsystems/telemetry.toml",
    "//:subsystems/teleop.toml",
]
'''


def subsystem(name, node):
    return (f'[subsystem]\nname = "{name}"\nnode = "{node}"\n'
            "period_us = 20000\n")


def fake_talos(tmp, tracked=True):
    """A talOS checkout at tmp/talos whose examples/robot_repo has a file at
    every TEMPLATE entry, shaped as the real fixture's where new_robot
    substitutes, each tracked in a git work tree unless `tracked` is false
    (a tarball); returns (checkout, fixture)."""
    talos = os.path.join(tmp, "talos")
    fixture = os.path.join(talos, new_robot.FIXTURE)
    files = {
        ".bazelrc": FIXTURE_BAZELRC,
        "MODULE.bazel": FIXTURE_MODULE,
        "robot.toml": FIXTURE_ROBOT,
        "robot_manifest.bzl": FIXTURE_MANIFEST,
        "subsystems/teleop.toml": subsystem("teleop",
                                            "@talos//talOS/teleop:node"),
        "subsystems/telemetry.toml": subsystem(
            "telemetry", "@talos//talOS/telemetry:node"),
        "subsystems/roller.toml": subsystem("roller", "//roller:node"),
        "subsystems/gate.toml": subsystem("gate", "//gate:node"),
    }
    for key in new_robot.TEMPLATE:
        path = key + "BUILD.bazel" if key.endswith("/") else key
        files.setdefault(path, f"# {path}\n")
    for path, content in files.items():
        write(os.path.join(fixture, path), content)
    if tracked:
        git(talos, "init", "-q")
        git(talos, "add", "-A")
    return talos, fixture


def git(work, *args):
    subprocess.run(["git", "-C", work, *args], check=True,
                   capture_output=True)


def write(path, content):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


class RobotRepositoryTest(unittest.TestCase):
    """--standalone: a robot repository copied from talOS's fixture."""

    # REQ-DEV-23, REQ-DEV-30
    def test_repository_is_the_fixture_without_its_mechanisms(self):
        with tempfile.TemporaryDirectory() as tmp:
            talos, fixture = fake_talos(tmp)
            target = os.path.join(tmp, "robots", "my_robot")
            files = {rel_path(target, path): content for path, content in
                     new_robot.plan_repository(talos, target, "my_robot")}
            self.assertEqual(sorted(files), [
                ".bazelignore", ".bazelrc", ".bazelversion", ".gitignore",
                "BUILD.bazel",
                "MODULE.bazel", "README.md",
                "robot.toml", "robot_manifest.bzl", "studio_test.py",
                "subsystems/telemetry.toml", "subsystems/teleop.toml",
                "talos_pin_test.py"])
            for path, action in new_robot.TEMPLATE.items():
                if action == new_robot.COPY:
                    with open(os.path.join(fixture, path),
                              encoding="utf-8") as f:
                        self.assertEqual(files[path], f.read(), path)
            # @talos and Studio's pages are the checkout's, from the new
            # repository's directory.
            module = files["MODULE.bazel"]
            self.assertIn('name = "my_robot"', module)
            override, pages = re_paths(module)
            self.assertEqual(
                os.path.normpath(os.path.join(target, override)), talos)
            self.assertEqual(
                os.path.normpath(os.path.join(target, pages)),
                os.path.join(talos, "bazel-bin", "studio", "dist"))
            # .bazelrc takes talOS's flags and status script from the same
            # checkout (talos_pin_test checks the import in the repository).
            imported, script = re_bazelrc_paths(files[".bazelrc"])
            self.assertEqual(os.path.normpath(imported.replace(
                "%workspace%", target)), os.path.join(talos, "talos.bazelrc"))
            self.assertEqual(
                os.path.normpath(os.path.join(target, script)),
                os.path.join(talos, "tools", "workspace_status.sh"))
            self.assertIn('name = "my_robot"', files["robot.toml"])
            for gone in ("roller", "gate"):
                self.assertNotIn(gone, files["robot.toml"])
                self.assertNotIn(gone, files["robot_manifest.bzl"])
            # What the repository's manifest_sync_test checks.
            for path, content in files.items():
                write(os.path.join(target, path), content)
            nodes, read = sync_manifest.resolve(target, "robot.toml")
            self.assertEqual(nodes, ["@talos//talOS/telemetry:node",
                                     "@talos//talOS/teleop:node"])
            self.assertEqual(sorted(read), ["subsystems/telemetry.toml",
                                            "subsystems/teleop.toml"])

    # REQ-DEV-23
    def test_refuses_a_directory_in_talos_or_a_used_one(self):
        # The name's rules are check_robot_name's (NewRobotTest).
        with tempfile.TemporaryDirectory() as tmp:
            talos, _ = fake_talos(tmp)
            used = os.path.join(tmp, "used")
            write(os.path.join(used, "notes.txt"), "mine\n")
            for target in (talos, os.path.join(talos, "robots", "my_robot"),
                           used, os.path.join(used, "notes.txt")):
                with self.subTest(target=target):
                    with self.assertRaises(new_subsystem.Refused):
                        new_robot.plan_repository(talos, target, "my_robot")
            self.assertFalse(os.path.exists(os.path.join(talos, "robots")))
            self.assertEqual(os.listdir(used), ["notes.txt"])

    # REQ-DEV-23
    def test_a_fixture_the_template_does_not_match_refuses_the_copy(self):
        # The template is the fixture: a file it does not place, a place
        # the fixture lost, or a line a substitution no longer finds stops
        # the copy, naming the path, before anything is written.
        def add(fixture, path):
            write(os.path.join(fixture, path), "# new\n")

        def remove(fixture, path):
            os.remove(os.path.join(fixture, path))

        def edit_module(fixture, path):
            edit(os.path.join(fixture, path),
                 [('    path = "../..",\n', '    path = "../../..",\n')])

        cases = (
            (add, "intake/BUILD.bazel"),
            (add, "talos_test.py"),
            (add, "subsystems/intake.toml"),
            (remove, "studio_test.py"),
            (edit_module, "MODULE.bazel"),
        )
        for change, path in cases:
            with self.subTest(change=change.__name__, path=path), \
                    tempfile.TemporaryDirectory() as tmp:
                talos, fixture = fake_talos(tmp)
                change(fixture, path)
                git(talos, "add", "-A")
                target = os.path.join(tmp, "my_robot")
                with self.assertRaisesRegex(new_subsystem.Refused,
                                            re.escape(path)):
                    new_robot.plan_repository(talos, target, "my_robot")
                self.assertFalse(os.path.exists(target))

    # REQ-DEV-23
    def test_an_untracked_file_in_the_fixture_is_not_copied_or_refused(self):
        # A student's checkout gathers files nobody committed (.DS_Store
        # once Finder opens the folder); a tarball or runfiles copy has no
        # index to tell them apart. The copy takes TEMPLATE's paths only.
        for tracked in (True, False):
            with self.subTest(tracked=tracked), \
                    tempfile.TemporaryDirectory() as tmp:
                talos, fixture = fake_talos(tmp, tracked)
                write(os.path.join(fixture, ".DS_Store"), "finder\n")
                write(os.path.join(fixture, "intake", "BUILD.bazel"), "#\n")
                target = os.path.join(tmp, "my_robot")
                paths = [rel_path(target, path) for path, _ in
                         new_robot.plan_repository(talos, target, "my_robot")]
                self.assertNotIn(".DS_Store", paths)
                self.assertNotIn("intake/BUILD.bazel", paths)

    # REQ-DEV-23
    def test_talos_path_is_absolute_only_when_sharing_just_the_root(self):
        self.assertEqual(new_robot.talos_path("/home/a/talos", "/home/a/r"),
                         ("../talos", False))
        self.assertEqual(new_robot.talos_path("/home/a/talos", "/tmp/r"),
                         ("/home/a/talos", True))
        # An absolute checkout is imported as itself: %workspace% before it
        # would name a path under the repository.
        self.assertEqual(
            re_bazelrc_paths(new_robot.substitute(
                ".bazelrc", FIXTURE_BAZELRC, "my_robot", "/home/a/talos")),
            ("/home/a/talos/talos.bazelrc",
             "/home/a/talos/tools/workspace_status.sh"))

    # REQ-DEV-23
    def test_a_write_that_fails_leaves_nothing_at_the_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            talos, _ = fake_talos(tmp)
            target = os.path.join(tmp, "robots", "my_robot")
            writes = new_robot.plan_repository(talos, target, "my_robot")
            real_open, opened = open, []

            def failing_open(*args, **kwargs):
                opened.append(args[0])
                if len(opened) == 3:
                    raise OSError(28, "No space left on device")
                return real_open(*args, **kwargs)

            with mock.patch("builtins.open", failing_open):
                with self.assertRaises(new_subsystem.Refused):
                    new_robot.write_repository(target, writes)
            self.assertEqual(len(opened), 3)
            self.assertEqual(os.listdir(os.path.join(tmp, "robots")), [])
            # An unwritable parent: refused too, not a traceback.
            locked = os.path.join(tmp, "locked")
            os.makedirs(locked)
            os.chmod(locked, 0o500)
            try:
                with self.assertRaises(new_subsystem.Refused):
                    new_robot.write_repository(
                        os.path.join(locked, "my_robot"), writes)
            finally:
                os.chmod(locked, 0o700)
            self.assertEqual(os.listdir(locked), [])


def rel_path(base, path):
    return os.path.relpath(path, base).replace(os.sep, "/")


def re_bazelrc_paths(bazelrc):
    """The talos.bazelrc a .bazelrc imports and its status command."""
    (imported,) = re.findall(r"^import (\S+)$", bazelrc, re.MULTILINE)
    (script,) = re.findall(r"--workspace_status_command=(\S+)$", bazelrc,
                           re.MULTILINE)
    return imported, script


def re_paths(module):
    """The local_path_override and studio_web paths of a MODULE.bazel."""
    return re.findall(r'^    path = "([^"]*)",$', module, re.MULTILINE)


if __name__ == "__main__":
    unittest.main()
