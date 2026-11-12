"""Tests for //tools:new_robot: the generated robot passes the launcher's
graph lint as generated, and a name that cannot be a robot is refused."""

import json
import os
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles

from tools import new_robot
from tools import new_subsystem
from tools import sync_manifest

CONFIG = "my_robot/main_processor/configuration/robot.toml"


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


class NewRobotTest(unittest.TestCase):
    # REQ-DEV-23
    def test_generated_robot_passes_the_graph_lint(self):
        with tempfile.TemporaryDirectory() as tmp:
            work, launcher, env = workspace(tmp)
            writes, can_id = new_robot.plan(work, "my_robot", "gripper",
                                            "cpp")
            self.assertEqual(can_id, 1)
            for path, _, content in writes:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, "w", encoding="utf-8") as f:
                    f.write(content)
            # The gripper package is new_subsystem's output, which
            # tools/scaffold_testdata/gripper holds built (new_subsystem_test
            # keeps the two equal); stand its binary in for the one
            # `bazel build` would make.
            package = os.path.join(work, "bazel-bin", "my_robot",
                                   "main_processor", "gripper")
            os.makedirs(package)
            os.symlink(os.path.join(work, "tools", "scaffold_testdata",
                                    "gripper", "node"),
                       os.path.join(package, "node"))

            # What its manifest_sync_test checks.
            self.assertEqual(sync_manifest.check(work, CONFIG), "")

            out = os.path.join(tmp, "out")
            proc = subprocess.run(
                [launcher, "--config", CONFIG, "--describe-only",
                 "--output-dir", out],
                cwd=work, env=env, capture_output=True, text=True,
                timeout=60)
            output = proc.stdout + proc.stderr
            self.assertEqual(proc.returncode, 0, output)
            self.assertNotIn("error:", output)
            (session,) = os.listdir(out)
            with open(os.path.join(out, session, "graph.json"),
                      encoding="utf-8") as f:
                graph = json.load(f)
            self.assertEqual(
                sorted(n["name"] for n in graph["nodes"]),
                ["gripper", "hardware_node", "telemetry", "teleop"])

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
