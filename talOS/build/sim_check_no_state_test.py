"""e2e: talos_sim_test's check fails a session whose hardware bridge never
received a State, and names the controller link (REQ-DEV-30).

The session (:no_state_sim) runs talOS's telemetry node, which reads
/hw/status, beside the bridge and talOS's sim_gateway sending its State to
the wrong port (testdata/sim_check/misaddressed_gateway.py). Every member
starts and exits 0 on the session's stop, as in a passing session, and the
bridge's exit status cannot say whether a State came. The wait for the
session to be up is READY_S here, not the timeout's bound: no State ever
comes, so a longer wait only makes the test slower.
"""

import os
import unittest

from python.runfiles import runfiles

from talOS.build import sim_check

SETTLE_S = 1
READY_S = 3


def rlocation(path):
    return runfiles.Create().Rlocation(path, source_repo="")


class SimCheckNoStateTest(unittest.TestCase):
    # REQ-DEV-30
    def test_session_without_state_fails_on_the_controller_link(self):
        problems, diagnostics = sim_check.check(
            rlocation(os.environ["SIM"]), SETTLE_S,
            rlocation(os.environ["LOG_DUMP"]),
            rlocation("_main/talOS/launcher/session_report.py"),
            rlocation(os.environ["REGISTRY_UNLINK"]),
            READY_S)
        self.assertTrue(
            any(p.startswith("controller link FAIL") for p in problems),
            f"{problems}\n{diagnostics}")


if __name__ == "__main__":
    unittest.main()
