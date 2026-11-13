"""Tests for //tools:new_robot: the generated robot, with a mechanism or
none, passes the launcher's graph lint as generated, the edits its `Next:` output lists make its
mechanism drivable, and a name that cannot be a robot is refused."""

import json
import os
import shlex
import subprocess
import sys
import tempfile
import unittest

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


def append_simulation_block(work, mechanism, kind):
    """Appends `mechanism`'s simulation block to the robot's simulation
    config, as its `Next:` says."""
    with open(os.path.join(work, SIM_CONFIG), "a", encoding="utf-8") as f:
        f.write(new_subsystem.simulation_block(mechanism, kind))


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
        # A scaffold leaves its teleop binding commented and its gains zero,
        # so on a robot that requires target coverage (2026-robot's policy)
        # the launcher refuses it as generated. The simulation block `Next:`
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
                     [("require_target_coverage = false",
                       "require_target_coverage = true"),
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


if __name__ == "__main__":
    unittest.main()
