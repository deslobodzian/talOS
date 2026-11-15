"""e2e: talos_sim_test's check fails a session whose hardware bridge never
received a State, and names the controller link (REQ-DEV-30).

The session (:no_state_sim) runs talOS's telemetry node, which reads
/hw/status, beside the bridge and talOS's sim_gateway sending its State to
the wrong port (testdata/sim_check/misaddressed_gateway.py). Every member
starts and exits 0 on the session's stop, as in a passing session, and the
bridge's exit status cannot say whether a State came. The wait for the
session to be up is READY_S here, not the timeout's bound: no State ever
comes, so a longer wait only makes the test slower.

It also fails, rather than raises, on a launch that refuses its graph: a
stand-in that makes the session directory, writes no manifest and exits 1.
"""

import os
import sys
import tempfile
import unittest

from python.runfiles import runfiles

from talOS.build import sim_check

SETTLE_S = 1
READY_S = 3

# A launcher that refuses the graph: it makes the session directory, writes
# no manifest and exits 1 at once.
REFUSING_LAUNCH = """import os, sys
out = sys.argv[sys.argv.index("--output-dir") + 1]
os.makedirs(os.path.join(out, "refused"))
print("launcher: the declared graph has errors; refusing to start")
sys.exit(1)
"""


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

    # REQ-DEV-30
    def test_launch_that_leaves_no_manifest_fails_with_its_output(self):
        with tempfile.NamedTemporaryFile(
                "w", suffix=".py", dir=os.environ.get("TEST_TMPDIR"),
                delete=False) as f:
            f.write(f"#!{sys.executable}\n{REFUSING_LAUNCH}")
        os.chmod(f.name, 0o755)
        problems, diagnostics = sim_check.check(
            f.name, SETTLE_S, rlocation(os.environ["LOG_DUMP"]),
            rlocation("_main/talOS/launcher/session_report.py"),
            rlocation(os.environ["REGISTRY_UNLINK"]), READY_S)
        self.assertIn("the launch ended on its own (exit 1) before the check "
                      "stopped it", problems)
        self.assertTrue(any("has no manifest" in p for p in problems),
                        problems)
        self.assertIn("the declared graph has errors; refusing to start",
                      diagnostics)


if __name__ == "__main__":
    unittest.main()
