"""The launcher accepts a config whose [robot] sim_gateway is the MuJoCo
gateway (REQ-DEV-37): its roster member sim_gateway passes the node label
rule, and the described graph carries the program's /sim/state/match."""

import json
import os
import shutil
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles

# Bounds a hang, not the run (docs/TESTING.md §6): --describe-only probes
# the bridge and the gateway and exits.
DESCRIBE_TIMEOUT_S = 50

_CONFIG = """[robot]
name = "mujoco_describe"
period_us = 5000
command_timeout_us = 100000
controller_address = "127.0.0.1"
sim_gateway = "//talOS/simulation/mujoco:sim_gateway"

# No node: the graph is the bridge and the simulator alone.
[subsystems]

[simulation.mujoco]
robot_model = "arm.xml"
"""


class LauncherDescribeTest(unittest.TestCase):

    # REQ-DEV-37: `launcher --sim --start-sim-gateway --describe-only` on a
    # config selecting the MuJoCo gateway exits 0 with the sim_gateway
    # member described, /sim/state/match among its endpoints.
    def test_launcher_accepts_the_mujoco_gateway(self):
        r = runfiles.Create()
        launcher = r.Rlocation(os.environ["LAUNCHER"])
        arm_xml = r.Rlocation(os.environ["ARM_XML"])
        # The launcher resolves //pkg:name to ./bazel-bin/pkg/name; the
        # runfiles tree has the same layout under the main repository.
        root = os.path.dirname(os.path.dirname(os.path.dirname(launcher)))
        with tempfile.TemporaryDirectory() as work:
            os.symlink(root, os.path.join(work, "bazel-bin"))
            shutil.copy(arm_xml, os.path.join(work, "arm.xml"))
            config = os.path.join(work, "robot.toml")
            with open(config, "w", encoding="utf-8") as f:
                f.write(_CONFIG)
            output = os.path.join(work, "sessions")
            proc = subprocess.run(
                [launcher, "--config", config, "--sim", "--start-sim-gateway",
                 "--describe-only", "--output-dir", output],
                cwd=work, capture_output=True, text=True,
                timeout=DESCRIBE_TIMEOUT_S, check=False)
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            (session,) = os.listdir(output)
            with open(os.path.join(output, session, "graph.json"),
                      encoding="utf-8") as f:
                nodes = json.load(f)["nodes"]
        (gateway,) = [n for n in nodes if n.get("name") == "sim_gateway"]
        self.assertTrue(gateway.get("described"), gateway)
        self.assertIn("/sim/state/match",
                      [source["name"] for source in gateway["sources"]],
                      gateway)


if __name__ == "__main__":
    unittest.main()
