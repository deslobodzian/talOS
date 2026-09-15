"""Teaching example tests: config, simulation, diagnostics per example.

Sim-only (no shared lib, no shared memory): each example's CONFIG_TOML
must parse through control_config, each run_sim must meet its goal when
tuned and visibly fail when untuned, and each diagnose must describe
what happened. Owner run:
  bazel test //2026-robot/main_processor/intake:examples_test
"""

import tempfile
import unittest
from pathlib import Path

from talos_nodes.intake import control_config
from talos_nodes.intake.examples import flywheel, position_mechanism
from talos_nodes.intake.examples import roller_intake


def _load(toml_text, motor):
    tmp = tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False)
    tmp.write(toml_text)
    tmp.close()
    try:
        return control_config.load_motor_config(tmp.name, motor)
    finally:
        Path(tmp.name).unlink()


class RollerIntakeTest(unittest.TestCase):
    def test_config_is_untuned_roller(self):
        config = _load(roller_intake.CONFIG_TOML, "roller")
        self.assertEqual(config.name, "roller")
        self.assertFalse(config.is_tuned)  # Gains come later; demands first.

    def test_intake_spins_up_tuned(self):
        result = roller_intake.run_sim(30.0, tuned=True)
        self.assertGreater(result["final_rps"], 25.0)
        self.assertEqual(result["captures"], 0)

    def test_beam_capture_holds_stop(self):
        result = roller_intake.run_sim(30.0, beam_at_s=0.5, tuned=True)
        self.assertEqual(result["captures"], 1)
        # After capture every demand holds stop, so the wheel spins down.
        self.assertLess(abs(result["demands"][-1]), 1e-9)
        self.assertLess(result["final_rps"], 5.0)

    def test_eject_clears_beam_blocked_piece(self):
        result = roller_intake.run_sim(30.0, beam_at_s=0.5,
                                       eject_at_s=1.0, tuned=True)
        self.assertEqual(result["captures"], 1)
        # Intentional ejection runs despite the earlier capture.
        self.assertLess(result["demands"][-1], 0.0)
        diagnosis = roller_intake.diagnose(result)
        self.assertTrue(any("eject ran" in line
                            for line in diagnosis["lines"]))

    def test_untuned_wheel_lags(self):
        result = roller_intake.run_sim(30.0, tuned=False)
        self.assertLess(result["final_rps"], 5.0)
        diagnosis = roller_intake.diagnose(result)
        self.assertTrue(any("untuned" in line
                            for line in diagnosis["lines"]))

    def test_debounce_survives_chatter(self):
        # A chattery beam still captures exactly once with 3-tick debounce.
        result = roller_intake.run_sim(30.0, beam_at_s=0.5, tuned=True,
                                       beam_stable_ticks=3)
        self.assertEqual(result["captures"], 1)


class FlywheelTest(unittest.TestCase):
    def test_config_is_tuned_flywheel(self):
        config = _load(flywheel.CONFIG_TOML, "flywheel")
        self.assertTrue(config.is_tuned)
        self.assertFalse(config.brake)  # Flywheels coast, never brake.

    def test_reaches_speed_before_timeout(self):
        result = flywheel.run_sim()
        self.assertTrue(result["done"])
        self.assertLessEqual(result["error_rps"], flywheel.TOLERANCE_RPS)
        diagnosis = flywheel.diagnose(result)
        self.assertTrue(diagnosis["done"])
        self.assertTrue(any("at speed" in line
                            for line in diagnosis["lines"]))

    def test_untuned_times_out(self):
        result = flywheel.run_sim(timeout_s=0.5, tuned=False)
        self.assertFalse(result["done"])
        diagnosis = flywheel.diagnose(result)
        self.assertFalse(diagnosis["done"])
        self.assertTrue(any("TIMED OUT" in line
                            for line in diagnosis["lines"]))


class PositionMechanismTest(unittest.TestCase):
    def test_config_gearing_is_25_to_1(self):
        config = _load(position_mechanism.CONFIG_TOML, "lift")
        self.assertEqual(config.sensor_to_mechanism_ratio, 25.0)
        self.assertTrue(config.brake)  # Lifts hold position at stop.
        self.assertTrue(config.is_tuned)

    def test_arrives_at_shelf(self):
        result = position_mechanism.run_sim(3.0)
        self.assertTrue(result["done"])
        self.assertLessEqual(result["error_rot"],
                             position_mechanism.TOLERANCE_ROT)
        self.assertTrue(result["cruise_limited"])  # Long move slews.

    def test_short_move_needs_no_cruise(self):
        result = position_mechanism.run_sim(0.1)
        self.assertTrue(result["done"])
        self.assertFalse(result["cruise_limited"])

    def test_untuned_never_arrives(self):
        result = position_mechanism.run_sim(3.0, timeout_s=1.0,
                                            tuned=False)
        self.assertFalse(result["done"])
        diagnosis = position_mechanism.diagnose(result)
        self.assertTrue(any("TIMED OUT" in line
                            for line in diagnosis["lines"]))

    def test_diagnose_reports_cruise(self):
        result = position_mechanism.run_sim(3.0)
        diagnosis = position_mechanism.diagnose(result)
        self.assertTrue(any("cruise" in line
                            for line in diagnosis["lines"]))


if __name__ == "__main__":
    unittest.main()
