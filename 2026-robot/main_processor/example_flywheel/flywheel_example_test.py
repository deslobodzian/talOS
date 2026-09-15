"""Flywheel example tests: config, demand law, wire, spin-up, diagnostics.

Sim-only (no shared lib, no shared memory). Owner run:
  bazel test //2026-robot/main_processor/example_flywheel:flywheel_example_test
"""

import struct
import tempfile
import unittest
from pathlib import Path

import flywheel_example
from talos_nodes.intake import control_config, packet


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
    def test_tuned_flywheel_coasts(self):
        config = _load(flywheel_example.CONFIG_TOML, "flywheel")
        self.assertEqual(config.name, "flywheel")
        self.assertTrue(config.is_tuned)
        self.assertFalse(config.brake)  # Flywheels coast, never brake.
        self.assertEqual(config.sensor_to_mechanism_ratio, 1.0)


class DemandTest(unittest.TestCase):
    def test_positive_spins_up_negative_holds_stop(self):
        self.assertEqual(flywheel_example.spin_demand(80.0), 80.0)
        self.assertEqual(flywheel_example.spin_demand(-80.0), 0.0)
        self.assertEqual(flywheel_example.spin_demand(0.0), 0.0)

    def test_nonfinite_rejected_at_boundary(self):
        with self.assertRaises(ValueError):
            flywheel_example.spin_demand(float("nan"))


class WireTest(unittest.TestCase):
    IDS = {"flywheel": 9}

    def test_velocity_mode_encodes_with_slot(self):
        self.assertEqual(
            _demand(flywheel_example.encode_velocity(_state(), self.IDS,
                                                     80.0)),
            (9, packet.MODE_VELOCITY, 0, 80.0))

    def test_over_limit_clamps_sign_kept(self):
        self.assertEqual(
            _demand(flywheel_example.encode_velocity(_state(), self.IDS,
                                                     150.0)),
            (9, packet.MODE_VELOCITY, 0, 100.0))
        self.assertEqual(
            _demand(flywheel_example.encode_velocity(_state(), self.IDS,
                                                     -150.0)),
            (9, packet.MODE_VELOCITY, 0, -100.0))

    def test_bad_demands_raise(self):
        with self.assertRaises(ValueError):
            flywheel_example.encode_velocity(_state(), self.IDS,
                                             float("nan"))
        with self.assertRaises(KeyError):
            flywheel_example.encode_velocity(_state(), self.IDS, 10.0,
                                             name="roller")


class SimTest(unittest.TestCase):
    def test_reaches_cruise_before_timeout(self):
        result = flywheel_example.run_sim()
        self.assertTrue(result["done"])
        self.assertLessEqual(result["error_rps"],
                             flywheel_example.TOLERANCE_RPS)
        self.assertEqual(result["hold_rps"], flywheel_example.CRUISE_RPS)

    def test_ramp_limits_first_step(self):
        result = flywheel_example.run_sim()
        # 160 rps/s * 5 ms = 0.8 rps per tick: cruise arrives via a ramp.
        self.assertLessEqual(result["velocities"][0], 1.0)
        spin_time = result["spin_up_s"]
        self.assertGreater(spin_time, 80.0 / 160.0)

    def test_untuned_times_out_and_holds_stop(self):
        result = flywheel_example.run_sim(timeout_s=0.5, tuned=False)
        self.assertFalse(result["done"])
        self.assertEqual(result["hold_rps"], 0.0)
        diagnosis = flywheel_example.diagnose(result)
        self.assertFalse(diagnosis["done"])
        self.assertTrue(any("TIMED OUT" in line
                            for line in diagnosis["lines"]))


class DiagnoseTest(unittest.TestCase):
    def test_at_speed_reports_hold(self):
        diagnosis = flywheel_example.diagnose(flywheel_example.run_sim())
        self.assertTrue(diagnosis["done"])
        self.assertTrue(any("at speed" in line
                            for line in diagnosis["lines"]))
        self.assertEqual(diagnosis["hold_rps"],
                         flywheel_example.CRUISE_RPS)

    def test_dead_wheel_names_tuning(self):
        result = flywheel_example.run_sim(timeout_s=0.5, tuned=False)
        diagnosis = flywheel_example.diagnose(result)
        self.assertTrue(any("untuned" in line
                            for line in diagnosis["lines"]))


if __name__ == "__main__":
    unittest.main()
