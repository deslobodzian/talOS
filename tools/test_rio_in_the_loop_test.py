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

    def test_refuses_mismatched_toml_header_brackets(self):
        for bad_header in ("[[robot]\nname = \"bench\"\n",
                           "[subsystems]]\nname = \"swerve_module\"\n"):
            with self.subTest(bad_header=bad_header), \
                    self.assertRaises(RuntimeError):
                self.narrow(bad_header)


class RotationsAndSilenceJudgmentTest(unittest.TestCase):
    # REQ-DEV-06
    def test_rotations_of_handles_signed_scientific_and_rejects_non_finite(self):
        self.assertAlmostEqual(
            rio.rotations_of("sim_gateway: first_wheel_rotations=1e-05 x=0"),
            1e-5,
        )
        self.assertAlmostEqual(
            rio.rotations_of("sim_gateway: first_wheel_rotations=-2.500000"),
            -2.5,
        )
        for bad in ("first_wheel_rotations=nan", "first_wheel_rotations=inf"):
            with self.subTest(bad=bad), self.assertRaises(RuntimeError):
                rio.rotations_of(f"sim_gateway: {bad}")

    # REQ-DEV-06
    def test_judge_silence_rejects_invalid_digital_or_pwm_samples(self):
        def make_state(seq, sample_us, epoch, active, volts, rps,
                       digital_outputs=(), pwm_outputs=()):
            flags = rio.FLAG_CONFIGURED | rio.FLAG_ENABLED
            if active:
                flags |= rio.FLAG_COMMAND_ACTIVE
            return {
                "sequence": seq,
                "sample_us": sample_us,
                "epoch": epoch,
                "flags": flags,
                "last_command_sequence": 1,
                "motors": [(1, True, rps, volts)],
                "digital_outputs": list(digital_outputs),
                "pwm_outputs": list(pwm_outputs),
                "apply_failures": 0,
                "resets": 0,
            }

        commands = {1: (100_000, [(1, 1)])}
        base_states = [
            make_state(1, 100_000, 1, True, 3.0, 10.0),
            make_state(2, 195_000, 1, True, 3.0, 10.0),
            make_state(3, 200_000, 2, False, 0.0, 0.0),
        ]
        # Settled at 200_000 + 4 * 5_000 = 220_000 (status_hz = 200).
        for label, extra in (
            ("invalid_digital_output",
             make_state(4, 225_000, 2, False, 0.0, 0.0,
                        digital_outputs=[(2, 0, 0)])),
            ("invalid_pwm_output",
             make_state(4, 225_000, 2, False, 0.0, 0.0,
                        pwm_outputs=[(3, 0, 0.0)])),
        ):
            with self.subTest(label=label), self.assertRaises(RuntimeError):
                rio.judge_silence(base_states + [extra], commands,
                                  timeout_us=100_000, status_hz=200)


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
