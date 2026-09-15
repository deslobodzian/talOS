"""Position-mechanism example tests: config, gearing, wire, motion, diagnostics.

Sim-only (no shared lib, no shared memory). Owner run:
  bazel test //2026-robot/main_processor/example_position:position_example_test
"""

import struct
import tempfile
import unittest
from pathlib import Path

import position_example
from talos_nodes.intake import control_config, packet, units


def _load(toml_text, motor):
    tmp = tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False)
    tmp.write(toml_text)
    tmp.close()
    try:
        return control_config.load_motor_config(tmp.name, motor)
    finally:
        Path(tmp.name).unlink()


def _state():
    return {"config_id": 7, "boot_id": 3, "epoch": 1,
            "sample_time_us": 1000}


def _demand(payload):
    (demand,) = struct.unpack_from("<d", payload, 4 + 42)
    (mode,) = struct.unpack_from("<B", payload, 4 + 40)
    (slot,) = struct.unpack_from("<B", payload, 4 + 41)
    (mid,) = struct.unpack_from("<H", payload, 4 + 38)
    return (mid, mode, slot, demand)


class ConfigTest(unittest.TestCase):
    def test_geared_lift_holds_position(self):
        config = _load(position_example.CONFIG_TOML, "lift")
        self.assertEqual(config.name, "lift")
        self.assertEqual(config.sensor_to_mechanism_ratio, 25.0)
        self.assertTrue(config.brake)  # Lifts hold position at stop.
        self.assertTrue(config.is_tuned)


class GearingTest(unittest.TestCase):
    def test_frames_cross_at_ratio(self):
        self.assertAlmostEqual(
            units.mechanism_rotations_to_rotor(1.0, 25.0), 25.0)
        self.assertAlmostEqual(
            units.rotor_to_mechanism_rotations(25.0, 25.0), 1.0)

    def test_demand_law_clamps_to_cruise(self):
        self.assertAlmostEqual(position_example.demand_for(3.0, 0.0),
                               position_example.CRUISE_RPS)
        self.assertAlmostEqual(position_example.demand_for(0.0, 3.0),
                               -position_example.CRUISE_RPS)
        self.assertAlmostEqual(position_example.demand_for(0.1, 0.0),
                               0.2)

    def test_nonfinite_positions_raise(self):
        with self.assertRaises(ValueError):
            position_example.demand_for(float("nan"), 0.0)
        with self.assertRaises(ValueError):
            position_example.demand_for(1.0, float("inf"))


class WireTest(unittest.TestCase):
    IDS = {"lift": 11}

    def test_position_mode_encodes_with_slot(self):
        self.assertEqual(
            _demand(position_example.encode_position(_state(), self.IDS,
                                                     2.5)),
            (11, packet.MODE_POSITION, 0, 2.5))

    def test_bad_positions_raise(self):
        with self.assertRaises(ValueError):
            position_example.encode_position(_state(), self.IDS,
                                             float("nan"))
        with self.assertRaises(ValueError):
            position_example.encode_position(_state(), self.IDS, True)
        with self.assertRaises(KeyError):
            position_example.encode_position(_state(), self.IDS, 1.0,
                                             name="roller")


class SimTest(unittest.TestCase):
    def test_long_move_slews_and_arrives(self):
        result = position_example.run_sim(3.0)
        self.assertTrue(result["done"])
        self.assertLessEqual(result["error_rot"],
                             position_example.TOLERANCE_ROT)
        self.assertTrue(result["cruise_limited"])
        self.assertEqual(result["hold"], ("position", 3.0))

    def test_short_move_needs_no_cruise(self):
        result = position_example.run_sim(0.1)
        self.assertTrue(result["done"])
        self.assertFalse(result["cruise_limited"])
        self.assertEqual(result["hold"], ("position", 0.1))

    def test_untuned_never_arrives_and_stops(self):
        result = position_example.run_sim(3.0, timeout_s=1.0, tuned=False)
        self.assertFalse(result["done"])
        self.assertEqual(result["hold"], ("stop", 0.0))
        diagnosis = position_example.diagnose(result)
        self.assertTrue(any("TIMED OUT" in line
                            for line in diagnosis["lines"]))


class DiagnoseTest(unittest.TestCase):
    def test_arrival_reports_hold(self):
        diagnosis = position_example.diagnose(
            position_example.run_sim(3.0))
        self.assertTrue(diagnosis["done"])
        self.assertEqual(diagnosis["hold"], ("position", 3.0))
        self.assertTrue(any("cruise" in line
                            for line in diagnosis["lines"]))

    def test_timeout_names_gains_not_timeout(self):
        result = position_example.run_sim(3.0, timeout_s=1.0, tuned=False)
        diagnosis = position_example.diagnose(result)
        self.assertTrue(any("not the timeout" in line
                            for line in diagnosis["lines"]))


if __name__ == "__main__":
    unittest.main()
