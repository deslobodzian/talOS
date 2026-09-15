"""sim_gateway must fail fast without --config instead of running dead.

Regression guard: an unconfigured Gateway stamps config_id 0, which every
mechanism's AcceptState gate rejects, so the graph runs green and motionless.
The binary therefore requires --config; this test pins that contract plus
the --describe envelope the launcher probes.
"""

import json
import os
import subprocess
from pathlib import Path
import unittest


def binary():
    workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get(
        "TEST_WORKSPACE", "_main")
    return workspace / "2026-robot/main_processor/simulation/sim_gateway"


class SimGatewayStartup(unittest.TestCase):
    def test_describe_answers_without_hardware(self):
        proc = subprocess.run(
            [str(binary()), "--describe"], capture_output=True, text=True,
            timeout=30)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        doc = json.loads(proc.stdout)
        self.assertEqual(doc["name"], "sim_gateway")
        self.assertEqual(doc["target"],
                         "//2026-robot/main_processor/simulation:sim_gateway")

    def test_missing_config_fails_fast(self):
        proc = subprocess.run(
            [str(binary()), "--duration-s", "1"], capture_output=True,
            text=True, timeout=30)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("--config", proc.stderr)


if __name__ == "__main__":
    unittest.main()
