"""e2e: talos_sim_test's check fails a session whose hardware bridge never
received a State, and names the controller link (REQ-DEV-30).

The session (:no_state_sim) runs talOS's telemetry node, which reads
/hw/status, beside the bridge and talOS's sim_gateway sending its State to
the wrong port (testdata/sim_check/misaddressed_gateway.py). Every member
runs once and exits 0 on the session's stop, as in a passing session,
unless the bridge reaches its own --duration-s before the launcher stops
it: it then exits 1 for want of a State, which the check reports too. So
the check must name the controller link whichever ends the bridge; without
that verdict it passes such a session whenever the stop comes first.
"""

import os
import unittest

from python.runfiles import runfiles

from talOS.build import sim_check

DURATION_S = 2


def rlocation(path):
    return runfiles.Create().Rlocation(path, source_repo="")


class SimCheckNoStateTest(unittest.TestCase):
    # REQ-DEV-30
    def test_session_without_state_fails_on_the_controller_link(self):
        problems, diagnostics = sim_check.check(
            rlocation(os.environ["SIM"]), DURATION_S,
            rlocation(os.environ["LOG_DUMP"]),
            rlocation("_main/talOS/launcher/session_report.py"),
            rlocation(os.environ["REGISTRY_UNLINK"]),
            sim_check.hang_bound(DURATION_S))
        self.assertTrue(
            any(p.startswith("controller link FAIL") for p in problems),
            f"{problems}\n{diagnostics}")


if __name__ == "__main__":
    unittest.main()
