"""e2e + unit: talos_sim_test's check with studio=True fails when Studio's
bridge is missing from the session manifest or fails its manifest checks
(REQ-DEV-30, REQ-OPS-30, REQ-OPS-34).

When [studio] bridge is missing from runfiles or does not answer --describe,
the launcher warns and DropUnansweredStudioBridge removes studio_bridge from
the roster and manifest while the rest of the session still runs and exits 0.
sim_check.check(..., studio=True) must not hang waiting for the absent bridge,
and judge(..., studio=True) must fail naming studio_bridge.
"""

import json
import os
import tempfile
import unittest
from unittest import mock

from python.runfiles import runfiles

from talOS.build import sim_check

SETTLE_S = 1


def rlocation(path):
    return runfiles.Create().Rlocation(path, source_repo="")


def _node(name, exit_code=0, exit_signal=0, restart_count=0,
          recorder_failed=False, http_port=None):
    entry = {
        "name": name,
        "exit_code": exit_code,
        "exit_signal": exit_signal,
        "restart_count": restart_count,
        "recorder_failed": recorder_failed,
        "runs_dropped": 0,
        "runs": [{
            "pid": 100,
            "start_wall_ns": 1,
            "end_wall_ns": 2,
            "exit_code": exit_code,
            "exit_signal": exit_signal,
            "reason": "stop" if (exit_code, exit_signal) == (0, 0) else "kill",
            "stopped_by_session": True,
        }],
    }
    if http_port is not None:
        entry["http_port"] = http_port
    return entry


def _judge_manifest(nodes, studio=True):
    with tempfile.TemporaryDirectory(
            dir=os.environ.get("TEST_TMPDIR")) as session:
        with open(os.path.join(session, "manifest.json"), "w",
                  encoding="utf-8") as f:
            json.dump({"stopped_before_start": False, "nodes": nodes}, f)
        with mock.patch.object(sim_check, "controller_link",
                               return_value=(True, "PASS: ok")):
            return sim_check.judge(session, "log_dump", None, studio=studio)


class SimCheckStudioTest(unittest.TestCase):
    # REQ-DEV-30, REQ-OPS-30, REQ-OPS-34
    def test_missing_studio_bridge_fails_naming_studio_bridge(self):
        problems, diagnostics = sim_check.check(
            rlocation(os.environ["MISSING_STUDIO_SIM"]), SETTLE_S,
            rlocation(os.environ["LOG_DUMP"]),
            rlocation("_main/talOS/launcher/session_report.py"),
            rlocation(os.environ["REGISTRY_UNLINK"]),
            sim_check.ready_bound(SETTLE_S), studio=True)
        self.assertTrue(
            any("studio_bridge" in p for p in problems),
            f"expected a studio_bridge problem, got {problems}\n{diagnostics}")
        # The session itself ran cleanly (did not hang waiting for the absent
        # bridge and did not fail the controller link).
        self.assertFalse(
            any("still waiting for" in p for p in problems),
            f"waited until timeout instead of failing after stop: "
            f"{problems}\n{diagnostics}")
        self.assertFalse(
            any(p.startswith("controller link") for p in problems),
            f"unexpected controller link failure: {problems}\n{diagnostics}")

    # REQ-DEV-30, REQ-OPS-30, REQ-OPS-34
    def test_judge_studio_accepts_healthy_bridge_and_rejects_faults(self):
        base = [_node("hardware_node"), _node("sim_gateway"),
                _node("telemetry")]

        healthy = base + [_node("studio_bridge", http_port=45123)]
        self.assertEqual(_judge_manifest(healthy, studio=True), [])

        # Missing studio_bridge entry fails when studio=True, passes when False.
        absent = _judge_manifest(base, studio=True)
        self.assertTrue(any("studio_bridge" in p for p in absent), absent)
        self.assertEqual(_judge_manifest(base, studio=False), [])

        # Missing or invalid http_port fails.
        for bad_port in (None, 0, -1, 65536, True, "5800"):
            with self.subTest(http_port=bad_port):
                problems = _judge_manifest(
                    base + [_node("studio_bridge", http_port=bad_port)],
                    studio=True)
                self.assertTrue(
                    any("studio_bridge" in p and "http_port" in p
                        for p in problems), problems)

        # Restart count nonzero fails.
        restarted = _judge_manifest(
            base + [_node("studio_bridge", http_port=45123, restart_count=1)],
            studio=True)
        self.assertTrue(
            any("studio_bridge" in p and "restart" in p for p in restarted),
            restarted)

        # recorder_failed True fails.
        failed = _judge_manifest(
            base + [_node("studio_bridge", http_port=45123,
                          recorder_failed=True)],
            studio=True)
        self.assertTrue(
            any("studio_bridge" in p and "recorder_failed" in p
                for p in failed), failed)

        # Unsettled SIGKILL (exit_signal == 9) fails.
        killed = _judge_manifest(
            base + [_node("studio_bridge", http_port=45123,
                          exit_code=-1, exit_signal=9)],
            studio=True)
        self.assertTrue(
            any("studio_bridge" in p and "signal 9" in p for p in killed),
            killed)


if __name__ == "__main__":
    unittest.main()
