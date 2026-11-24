"""The launcher's graph lint with Studio's bridge reading the game pieces
(REQ-DEV-43): `--describe-only` with `--start-studio-bridge` on a config
whose simulator is the MuJoCo gateway, which writes /sim/telemetry/pieces,
and on one whose simulator is sim_gateway, which does not, exits 0 with no
finding about the topic."""

import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

from python.runfiles import runfiles

# Bounds a hang, not the run (docs/TESTING.md §6): --describe-only probes
# the hardware bridge, telemetry, the simulator and Studio's bridge, then
# exits.
DESCRIBE_TIMEOUT_S = 50
PIECES = "/sim/telemetry/pieces"

_ROBOT = """<mujoco><worldbody>
  <body name="post" pos="0 0 0.5">
    <geom type="box" size="0.1 0.1 0.5" contype="4" conaffinity="21"/>
  </body></worldbody></mujoco>
"""

_CONFIG = """[robot]
name = "pieces_describe"
period_us = 5000
command_timeout_us = 100000
controller_address = "127.0.0.1"
{simulator}
# Telemetry, which Studio's bridge reads, and no other node.
[subsystems.telemetry]
node = "//talOS/telemetry:node"
{simulation}"""

_MUJOCO = dict(
    simulator='sim_gateway = "//talOS/simulation/mujoco:sim_gateway"\n',
    simulation='\n[simulation.mujoco]\nrobot_model = "robot.xml"\n'
               'field = "rebuilt_2026"\n')
_PLAIN = dict(simulator="", simulation="")


class PiecesDescribeTest(unittest.TestCase):

    def describe(self, config_values):
        r = runfiles.Create()
        launcher = r.Rlocation(os.environ["LAUNCHER"])
        # The launcher resolves //pkg:name to ./bazel-bin/pkg/name; the
        # runfiles tree has the same layout under the main repository.
        root = os.path.dirname(os.path.dirname(os.path.dirname(launcher)))
        with tempfile.TemporaryDirectory() as work:
            os.symlink(root, os.path.join(work, "bazel-bin"))
            Path(work, "robot.xml").write_text(_ROBOT, encoding="utf-8")
            config = os.path.join(work, "robot.toml")
            Path(config).write_text(_CONFIG.format(**config_values),
                                    encoding="utf-8")
            output = os.path.join(work, "sessions")
            proc = subprocess.run(
                [launcher, "--config", config, "--sim", "--start-sim-gateway",
                 "--start-studio-bridge", "--describe-only",
                 "--output-dir", output],
                cwd=work, capture_output=True, text=True,
                timeout=DESCRIBE_TIMEOUT_S, check=False)
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            (session,) = os.listdir(output)
            with open(os.path.join(output, session, "graph.json"),
                      encoding="utf-8") as f:
                nodes = {n["name"]: n for n in json.load(f)["nodes"]}
        return proc.stdout + proc.stderr, nodes

    @staticmethod
    def kinds(node):
        return [s["kind"] for s in node["sources"] if s["name"] == PIECES]

    @staticmethod
    def slots(node):
        return [s["slots"] for s in node["sources"] if s["name"] == PIECES]

    # REQ-DEV-43
    def test_the_mujoco_gateway_writes_what_studio_reads(self):
        out, nodes = self.describe(_MUJOCO)
        self.assertEqual(self.kinds(nodes["sim_gateway"]), ["SENDER"], out)
        self.assertEqual(self.kinds(nodes["studio_bridge"]), ["FETCHER"], out)
        # Both ends on the topic's ring (talOS/hardware/sim_rings.h), 8 slots.
        self.assertEqual(self.slots(nodes["sim_gateway"]), [8], out)
        self.assertEqual(self.slots(nodes["studio_bridge"]), [8], out)
        self.assertNotIn(PIECES, out)

    # REQ-DEV-43: a session without the MuJoCo gateway has no writer of the
    # topic; Studio's read is optional, so the lint names nothing.
    def test_a_session_without_pieces_lints_clean(self):
        out, nodes = self.describe(_PLAIN)
        for node in nodes.values():
            self.assertNotIn("SENDER", self.kinds(node), out)
        self.assertEqual(self.kinds(nodes["studio_bridge"]), ["FETCHER"], out)
        self.assertNotIn(PIECES, out)


if __name__ == "__main__":
    unittest.main()
