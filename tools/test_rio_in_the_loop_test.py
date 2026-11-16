"""Tests for tools/test_rio_in_the_loop.py's subject config.

REQ-SAFE-16, REQ-SAFE-17, REQ-SAFE-18: the RoboRIO-in-the-loop run judges
the subject on the robot's own settings. subject_config narrows the bench
config to the subject's subsystem by text; it must keep every robot table,
and refuse a table it does not know rather than drop it unseen.
"""

import os
import tempfile
import unittest
from pathlib import Path

from tools import test_rio_in_the_loop as rio

CONFIG = """\
# A comment before the first table.
[robot]
name = "bench"
period_us = 5000

[robot.targets]
teleop = ["teleop"]

[studio]
webroot = "bazel-bin/studio/dist"

[[subsystems]]
name = "motor"
path = "subsystems/motor.toml"

[[subsystems]]
name = "{subject}"
path = "subsystems/{subject}.toml"
"""


class SubjectConfigTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.dir = Path(tmp.name)

    def narrow(self, text):
        config = self.dir / "robot.toml"
        config.write_text(text)
        return rio.subject_config(config, self.dir).read_text()

    def test_keeps_the_robot_tables_and_the_subjects_entry(self):
        out = self.narrow(CONFIG.format(subject=rio.SUBJECT))
        self.assertIn('[robot]\nname = "bench"\nperiod_us = 5000\n', out)
        self.assertIn('[robot.targets]\nteleop = ["teleop"]\n', out)
        subject = self.dir.resolve() / "subsystems" / f"{rio.SUBJECT}.toml"
        self.assertIn(f'[[subsystems]]\nname = "{rio.SUBJECT}"\n'
                      f'path = "{subject}"\n', out)
        self.assertNotIn('"motor"', out)
        self.assertNotIn("[studio]", out)

    def test_refuses_a_table_it_does_not_know(self):
        # A hardware table the subject's devices might need, which a
        # narrowed config would otherwise lose without a word.
        text = CONFIG.format(subject=rio.SUBJECT) + (
            "\n[can_buses.rio]\nbitrate_bps = 1000000\n")
        with self.assertRaisesRegex(RuntimeError, r"\[can_buses\.rio\]"):
            self.narrow(text)
        self.assertFalse(os.path.exists(self.dir / "subject.toml"))


if __name__ == "__main__":
    unittest.main()
