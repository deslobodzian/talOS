"""Unit-style CLI validation tests for //talOS/simulation:sim_gateway.

Kept separate from sim_gateway_test.py; exercises flag-parsing fail-closed
cases without opening sockets or sleeping.
"""

import os
import subprocess
import unittest

from python.runfiles import runfiles


def binary():
    return runfiles.Create().Rlocation(os.environ["SIM_GATEWAY"])


class SimGatewayCliTest(unittest.TestCase):
    # REQ-DEV-05, REQ-DEV-31
    def test_rejects_negative_whitespace_session_id_and_non_strict_numbers(self):
        # Leading whitespace before '-1' previously bypassed value.starts_with('-')
        # and wrapped std::stoull to UINT64_MAX.
        for bad_args in (
            ("--describe", "--session-id", " -1"),
            ("--describe", "--session-id", "+1"),
            ("--describe", "--session-id", " 42"),
            ("--describe", "--port", " 5802"),
            ("--describe", "--port", "+5802"),
            ("--describe", "--port", "99999999999999999999"),
            ("--describe", "--physics", "--gyro-drift-radps", "1e400"),
            ("--describe", "--duration-s", "1e400"),
        ):
            with self.subTest(args=bad_args):
                proc = subprocess.run(
                    [str(binary()), *bad_args],
                    capture_output=True, text=True, timeout=30)
                self.assertNotEqual(proc.returncode, 0, proc.stdout + proc.stderr)

    # REQ-DEV-05, REQ-DEV-31
    def test_describe_as_flag_value_does_not_trigger_describe_mode(self):
        # Passing `--describe` as the value of `--config` (not as a flag) must
        # not short-circuit into describe mode and exit 0.
        proc = subprocess.run(
            [str(binary()), "--config", "--describe"],
            capture_output=True, text=True, timeout=30)
        self.assertNotEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertNotIn('"name": "sim_gateway"', proc.stdout)


if __name__ == "__main__":
    unittest.main()
