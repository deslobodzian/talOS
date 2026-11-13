"""The 2026 graph with one more mechanism passes `launcher --describe-only`.

The 2026 testbed already has eight readers of /hw/state/driver_station, as
many as a topic's reader table once held: adding one more `Target` consumer,
the example elevator, made the launcher refuse the graph. The test copies
robot.toml and its subsystem files into a temporary directory, adds the
example elevator as a team would (its CAN id and DIO moved off the
drivetrain's and intake's), and expects the launcher to accept the graph with
more than eight Driver Station readers. The robot's own configs are not
touched.
"""

import json
import os
import re
import shutil
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles

CONFIG = "_main/2026-robot/main_processor/configuration/robot.toml"
ELEVATOR = "_main/examples/superstructure/config/subsystems/elevator.toml"
DRIVER_STATION = "/hw/state/driver_station"


def replace_once(text, pattern, replacement):
    """`text` with the one match of `pattern` replaced; fails on none or more."""
    result, count = re.subn(pattern, replacement, text, flags=re.MULTILINE)
    if count != 1:
        raise AssertionError(f"{pattern!r} matched {count} times")
    return result


class FanoutGraphTest(unittest.TestCase):
    # REQ-CONTRACT-36
    def test_a_ninth_driver_station_reader_is_accepted(self):
        r = runfiles.Create()
        launcher = r.Rlocation("_main/talOS/launcher/launcher")
        self.assertTrue(launcher and os.path.exists(launcher), f"missing launcher: {launcher}")
        runfiles_root = os.environ.get("RUNFILES_DIR")
        if not runfiles_root or not os.path.isdir(os.path.join(runfiles_root, "_main")):
            runfiles_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launcher))))
        main_dir = os.path.join(runfiles_root, "_main")

        with tempfile.TemporaryDirectory() as tmp:
            # The launcher finds a node label's binary under bazel-bin of its
            # working directory: point both at the runfiles tree.
            work_dir = os.path.join(tmp, "ws")
            out_dir = os.path.join(tmp, "out")
            bazel_bin = os.path.join(work_dir, "bazel-bin")
            os.makedirs(bazel_bin)
            os.makedirs(out_dir)
            for entry in os.listdir(main_dir):
                source = os.path.join(main_dir, entry)
                os.symlink(source, os.path.join(work_dir, entry))
                os.symlink(source, os.path.join(bazel_bin, entry))
            if not os.path.exists(os.path.join(bazel_bin, "external")):
                os.symlink(runfiles_root, os.path.join(bazel_bin, "external"))

            config_dir = os.path.join(tmp, "config")
            source_dir = os.path.dirname(r.Rlocation(CONFIG))
            os.makedirs(config_dir)
            # robot.toml and the data file it names ([studio] dashboard).
            # Contents only, not modes: CI makes tracked files read-only, and
            # robot.toml is rewritten below.
            for name in ("robot.toml", "dashboard.toml"):
                shutil.copyfile(os.path.join(source_dir, name),
                                os.path.join(config_dir, name))
            shutil.copytree(os.path.join(source_dir, "subsystems"),
                            os.path.join(config_dir, "subsystems"),
                            copy_function=shutil.copyfile)

            with open(r.Rlocation(ELEVATOR), encoding="utf-8") as handle:
                elevator = handle.read()
            # CAN id 2 is a drivetrain motor's and DIO 1 the intake's here.
            elevator = replace_once(elevator, r"^can_id = 2$", "can_id = 40")
            elevator = replace_once(elevator, r"^dio = 1$", "dio = 2")
            with open(os.path.join(config_dir, "subsystems", "elevator.toml"), "w",
                      encoding="utf-8") as handle:
                handle.write(elevator)

            robot_path = os.path.join(config_dir, "robot.toml")
            with open(robot_path, encoding="utf-8") as handle:
                robot = handle.read()
            # The elevator's lane belongs to the example's superstructure,
            # which is not in this graph; without it, coverage would refuse
            # the graph for a reason other than its readers.
            robot = replace_once(robot, r"^require_target_coverage = true$",
                                 "require_target_coverage = false")
            robot += ('\n[[subsystems]]\nname = "elevator"\n'
                      'path = "subsystems/elevator.toml"\n'
                      'node = "//examples/superstructure/cpp:elevator"\n')
            with open(robot_path, "w", encoding="utf-8") as handle:
                handle.write(robot)

            env = dict(os.environ, **r.EnvVars())
            env["RUNFILES_DIR"] = runfiles_root
            env.pop("PYTHONPATH", None)
            env.pop("BUILD_WORKSPACE_DIRECTORY", None)
            proc = subprocess.run(
                [launcher, "--config", robot_path, "--describe-only", "--output-dir", out_dir],
                cwd=work_dir, env=env, capture_output=True, text=True, timeout=120)
            self.assertEqual(proc.returncode, 0, proc.stdout + "\n" + proc.stderr)

            sessions = [os.path.join(out_dir, d) for d in os.listdir(out_dir)
                        if os.path.isdir(os.path.join(out_dir, d))]
            self.assertEqual(len(sessions), 1, sessions)
            with open(os.path.join(sessions[0], "graph.json"), encoding="utf-8") as handle:
                graph = json.load(handle)
            nodes = graph["nodes"]
            undescribed = [n["name"] for n in nodes if not n.get("described")]
            self.assertEqual(undescribed, [])
            self.assertIn("elevator", [n["name"] for n in nodes])
            readers = sorted(
                node["name"] for node in nodes for source in node["sources"]
                if source["name"] == DRIVER_STATION
                and source["kind"] in ("WATCHER", "FETCHER"))
            self.assertGreater(len(readers), 8, readers)
            self.assertIn("elevator", readers)


if __name__ == "__main__":
    unittest.main()
