"""Tests for //tools:new_robot: the generated robot passes the launcher's
graph lint as generated, the edits its `Next:` output lists make its
mechanism drivable, and a name that cannot be a robot is refused."""

import json
import os
import subprocess
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles

from tools import new_robot
from tools import new_subsystem
from tools import sync_manifest

CONFIG = "my_robot/main_processor/configuration/robot.toml"
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


def stand_in_binary(work, mechanism):
    """Puts the C++ mechanism's binary where `bazel build` of the generated
    package would: the package is new_subsystem's output, which
    tools/scaffold_testdata/<mechanism> holds built (new_subsystem_test keeps
    the two equal)."""
    package = os.path.join(work, "bazel-bin", "my_robot", "main_processor",
                           mechanism)
    os.makedirs(package)
    os.symlink(os.path.join(work, "tools", "scaffold_testdata", mechanism,
                            "node"),
               os.path.join(package, "node"))


def generate(work, mechanism, lang, kind="velocity"):
    writes, can_id = new_robot.plan(work, "my_robot", mechanism, lang, kind)
    for path, _, content in writes:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
    stand_in_binary(work, mechanism)
    return can_id


def describe(work, launcher, env, out):
    """The launcher's --describe-only on the generated robot: (exit status,
    output, graph.json or None)."""
    proc = subprocess.run(
        [launcher, "--config", CONFIG, "--describe-only", "--output-dir",
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
        for mechanism, lang in (("gripper", "cpp"),):
            with self.subTest(lang=lang), tempfile.TemporaryDirectory() as tmp:
                work, launcher, env = workspace(tmp)
                self.assertEqual(generate(work, mechanism, lang), 1)
                # What its manifest_sync_test checks.
                self.assertEqual(sync_manifest.check(work, CONFIG), "")

                status, output, graph = describe(work, launcher, env,
                                                 os.path.join(tmp, "out"))
                self.assertEqual(status, 0, output)
                self.assertNotIn("error:", output)
                self.assertEqual(
                    sorted(n["name"] for n in graph["nodes"]),
                    sorted([mechanism, "hardware_node", "telemetry",
                            "teleop"]))

    # REQ-DEV-23
    def test_next_steps_edits_make_the_mechanism_drivable(self):
        # A scaffold leaves its teleop binding commented and its gains zero,
        # so on a robot that requires target coverage (2026-robot's policy)
        # the launcher refuses it as generated. The edits `Next:` lists,
        # applied exactly, give the mechanism a producer in every mode the
        # policy covers and a teleop target it can reach.
        for mechanism, kind in (("gripper", "velocity"),
                                ("position_cpp", "position")):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as tmp:
                work, launcher, env = workspace(tmp)
                generate(work, mechanism, "cpp", kind)
                edit(os.path.join(work, CONFIG),
                     [("require_target_coverage = false",
                       "require_target_coverage = true"),
                      ("autonomous = []", 'autonomous = ["auto"]')])
                status, output, _ = describe(work, launcher, env,
                                             os.path.join(tmp, "before"))
                self.assertNotEqual(status, 0, output)
                self.assertIn("target coverage required for mode 'teleop'",
                              output)

                subsystem = os.path.join(work, SUBSYSTEMS, f"{mechanism}.toml")
                for _, _, _, replacements in new_subsystem.simulation_edits(
                        mechanism, kind):
                    edit(subsystem, replacements)
                status, output, _ = describe(work, launcher, env,
                                             os.path.join(tmp, "after"))
                self.assertEqual(status, 0, output)
                self.assertNotIn("error:", output)

                # The bound target is one the node does not clamp away.
                with open(subsystem, "rb") as f:
                    table = tomllib.load(f)
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


if __name__ == "__main__":
    unittest.main()
