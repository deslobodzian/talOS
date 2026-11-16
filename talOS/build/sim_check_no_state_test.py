"""e2e: talos_sim_test's check fails a session whose hardware bridge never
received a State, and names the controller link (REQ-DEV-30).

The session (:no_state_sim) runs talOS's telemetry node, which reads
/hw/status, beside the bridge and talOS's sim_gateway sending its State to
the wrong port (testdata/sim_check/misaddressed_gateway.py). Every member
starts and exits 0 on the session's stop, as in a passing session, and the
bridge's exit status cannot say whether a State came. The session gets
the check's ordinary bound (sim_check.ready_bound) to start, so a slow
start-up (a sanitizer build on a loaded host) still reaches the condition
under test; only the wait for a State once every member's loop has started
is STATE_S, not that bound: no State ever comes, so a longer wait only makes
the test slower. A second case starts the same launch STATE_S + 1 s late,
so a State bound counted from the launch rather than from the session's
start would end the wait before the session started.

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
STATE_S = 3

# A launcher that refuses the graph: it makes the session directory, writes
# no manifest and exits 1 at once.
REFUSING_LAUNCH = """import os, sys
out = sys.argv[sys.argv.index("--output-dir") + 1]
os.makedirs(os.path.join(out, "refused"))
print("launcher: the declared graph has errors; refusing to start")
sys.exit(1)
"""

# The launch, started late: longer than STATE_S, as a sanitizer build's
# start-up can be.
SLOW_LAUNCH = """#!/bin/sh
sleep {delay_s}
exec {sim} "$@"
"""


def rlocation(path):
    return runfiles.Create().Rlocation(path, source_repo="")


def stand_in(content):
    """An executable script in TEST_TMPDIR holding `content`."""
    with tempfile.NamedTemporaryFile(
            "w", dir=os.environ.get("TEST_TMPDIR"), delete=False) as f:
        f.write(content)
    os.chmod(f.name, 0o755)
    return f.name


class SimCheckNoStateTest(unittest.TestCase):
    def assert_fails_on_the_controller_link(self, sim):
        problems, diagnostics = sim_check.check(
            sim, SETTLE_S, rlocation(os.environ["LOG_DUMP"]),
            rlocation("_main/talOS/launcher/session_report.py"),
            rlocation(os.environ["REGISTRY_UNLINK"]),
            sim_check.ready_bound(SETTLE_S), state_s=STATE_S)
        # The session started and the wait for its State is what ran out.
        self.assertTrue(
            any(p.startswith("no State accepted by the bridge within")
                for p in problems), f"{problems}\n{diagnostics}")
        self.assertTrue(
            any(p.startswith("controller link FAIL") for p in problems),
            f"{problems}\n{diagnostics}")

    # REQ-DEV-30
    def test_session_without_state_fails_on_the_controller_link(self):
        self.assert_fails_on_the_controller_link(
            rlocation(os.environ["SIM"]))

    # REQ-DEV-30
    def test_slow_start_still_reaches_the_wait_for_a_state(self):
        self.assert_fails_on_the_controller_link(stand_in(SLOW_LAUNCH.format(
            delay_s=STATE_S + 1, sim=rlocation(os.environ["SIM"]))))

    # REQ-DEV-30
    def test_launch_that_leaves_no_manifest_fails_with_its_output(self):
        problems, diagnostics = sim_check.check(
            stand_in(f"#!{sys.executable}\n{REFUSING_LAUNCH}"), SETTLE_S, rlocation(os.environ["LOG_DUMP"]),
            rlocation("_main/talOS/launcher/session_report.py"),
            rlocation(os.environ["REGISTRY_UNLINK"]),
            sim_check.ready_bound(SETTLE_S))
        self.assertIn("the launch ended on its own (exit 1) before the check "
                      "stopped it", problems)
        self.assertTrue(any("has no manifest" in p for p in problems),
                        problems)
        self.assertIn("the declared graph has errors; refusing to start",
                      diagnostics)


if __name__ == "__main__":
    unittest.main()
