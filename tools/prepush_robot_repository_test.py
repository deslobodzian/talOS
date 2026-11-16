"""Tests for tools/prepush_robot_repository.sh, prepush's robot-repository step.

REQ-DEV-32: tools/prepush.sh runs the robot repository check when
TALOS_ROBOT_REPOSITORY names a directory, reports the step skipped when the
variable is unset, and fails the step when it names no directory: an empty
value too, which is what `TALOS_ROBOT_REPOSITORY="$MISSPELT" tools/prepush.sh`
passes, so a typo cannot skip the check it asked for.
"""

import os
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles


class RobotRepositoryDecisionTest(unittest.TestCase):
    def setUp(self):
        self.helper = runfiles.Create().Rlocation(
            "_main/tools/prepush_robot_repository.sh")
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = os.path.realpath(tmp.name)

    def decide(self, value=None, cwd=None):
        """(status, line) of the decision with the variable set to `value`,
        or unset when it is None."""
        env = {k: v for k, v in os.environ.items() if k != "TALOS_ROBOT_REPOSITORY"}
        if value is not None:
            env["TALOS_ROBOT_REPOSITORY"] = value
        result = subprocess.run(
            ["bash", "-c", '. "$1" && robot_repository_decision', "_", self.helper],
            env=env, cwd=cwd or self.tmp, capture_output=True, text=True, check=False)
        self.assertEqual(result.stderr, "")
        return result.returncode, result.stdout.rstrip("\n")

    # REQ-DEV-32
    def test_unset_skips_and_set_to_no_directory_fails(self):
        self.assertEqual(self.decide(), (1, "TALOS_ROBOT_REPOSITORY unset"))
        afile = os.path.join(self.tmp, "afile")
        with open(afile, "w", encoding="utf-8"):
            pass
        for value in ("", os.path.join(self.tmp, "missing"), afile):
            with self.subTest(value=value):
                status, line = self.decide(value)
                self.assertEqual(status, 2, line)
                self.assertIn(f"'{value}' is not a directory", line)

    # REQ-DEV-32
    def test_a_directory_runs_on_its_physical_path(self):
        robot = os.path.join(self.tmp, "robot")
        os.makedirs(robot)
        os.symlink(robot, os.path.join(self.tmp, "link"))
        # Relative to where prepush was started, and through a symlink.
        self.assertEqual(self.decide("link", cwd=self.tmp), (0, robot))


if __name__ == "__main__":
    unittest.main()
