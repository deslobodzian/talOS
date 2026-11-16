"""Tests for tools/test_rio_in_the_loop.py's subject config and bring-up.

REQ-DEV-06: the RoboRIO-in-the-loop run (the real RoboRIO program against
the Bazel-built bridge and node) judges the subject on the robot's own
settings. subject_config narrows the bench config to the subject's
subsystem by text; it must keep every robot table, and refuse a table it
does not know rather than drop it unseen. Each scenario's RoboRIO program
must start unconfigured: one that restored a saved configuration enables on
it and refuses the bridge's push of another, so the run is refused.
"""

import os
import socket
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


class NothingRestoredTest(unittest.TestCase):
    def test_refuses_a_program_that_restored_a_configuration(self):
        saved = Path("talos_config.bin")
        line = ("banner\n[13967.318] hardware gateway config={} "
                "commissioned={} (waiting for config push)\n")
        rio.check_nothing_restored(line.format(0, 0), saved)
        with self.assertRaisesRegex(RuntimeError, "9664505023115029081"):
            rio.check_nothing_restored(
                line.format(9664505023115029081, 1), saved)


class PortsTest(unittest.TestCase):
    # REQ-DEV-06: a held port fails the run naming the port, not with a
    # bare OSError that names none.
    def test_a_held_port_is_named(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as holder:
            holder.bind(("0.0.0.0", 0))
            port = holder.getsockname()[1]
            with self.assertRaises(RuntimeError) as raised:
                rio.check_ports((port,))
            self.assertIn(str(port), str(raised.exception))


if __name__ == "__main__":
    unittest.main()
