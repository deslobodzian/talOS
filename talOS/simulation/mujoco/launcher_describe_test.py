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

# talOS/hardware/sim_rings.h kPiecesSlots.
PIECES_SLOTS = 8

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

    # REQ-DEV-37, REQ-DEV-40, REQ-DEV-43: `launcher --sim --start-sim-gateway
    # --describe-only` on a config selecting the MuJoCo gateway exits 0 with
    # the sim_gateway member described, /sim/state, /sim/state/match, and
    # /sim/telemetry/pieces (8 slots) among its endpoints, and no graph errors.
    def test_launcher_accepts_the_mujoco_gateway(self):
        r = runfiles.Create()
        launcher = r.Rlocation(os.environ["LAUNCHER"])
        arm_xml = r.Rlocation(os.environ["ARM_XML"])
        self.assertIsNotNone(launcher, "LAUNCHER runfile missing")
        self.assertIsNotNone(arm_xml, "ARM_XML runfile missing")
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
            env = os.environ.copy()
            env.pop("TALOS_SHM_NAMESPACE", None)
            env["TEST_TMPDIR"] = work
            proc = subprocess.run(
                [launcher, "--config", config, "--sim", "--start-sim-gateway",
                 "--describe-only", "--output-dir", output],
                cwd=work, env=env, capture_output=True, text=True,
                timeout=DESCRIBE_TIMEOUT_S, check=False)
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            (session,) = os.listdir(output)
            with open(os.path.join(output, session, "graph.json"),
                      encoding="utf-8") as f:
                graph = json.load(f)
        errors = [d for d in graph.get("diagnostics", [])
                  if d.get("severity") == "error"]
        self.assertEqual(errors, [], graph)
        nodes = graph["nodes"]
        (gateway,) = [n for n in nodes if n.get("name") == "sim_gateway"]
        self.assertTrue(gateway.get("described"), gateway)
        self.assertEqual(gateway.get("target"),
                         "//talOS/simulation/mujoco:sim_gateway", gateway)
        sources = {source["name"]: source for source in gateway["sources"]}
        self.assertIn("/sim/state", sources, gateway)
        self.assertIn("/sim/state/match", sources, gateway)
        self.assertIn("/sim/telemetry/pieces", sources, gateway)
        self.assertEqual(sources["/sim/telemetry/pieces"].get("slots"),
                         PIECES_SLOTS, gateway)


if __name__ == "__main__":
    unittest.main()
