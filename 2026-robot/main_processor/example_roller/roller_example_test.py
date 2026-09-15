"""Voltage-roller example tests: config, volts map, wire, sim, diagnostics.

Sim-only (no shared lib, no shared memory). Owner run:
  bazel test //2026-robot/main_processor/example_roller:roller_example_test
"""

import struct
import tempfile
import unittest
from pathlib import Path

import roller_example
from talos_nodes.intake import control_config, motor_commands, packet


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
    def test_open_loop_is_untuned_by_design(self):
        config = _load(roller_example.CONFIG_TOML, "roller")
        self.assertEqual(config.name, "roller")
        self.assertFalse(config.is_tuned)  # No PID in open loop: correct.
        self.assertTrue(config.brake)
        self.assertEqual(config.sensor_to_mechanism_ratio, 1.0)
        self.assertEqual(config.max_voltage, 12.0)


class VoltsMapTest(unittest.TestCase):
    def test_linear_map(self):
        self.assertAlmostEqual(roller_example.volts_to_rps(6.0), 50.0)
        self.assertAlmostEqual(roller_example.volts_to_rps(-12.0), -100.0)
        self.assertAlmostEqual(roller_example.volts_to_rps(0.0), 0.0)

    def test_boundaries_reject(self):
        for bad in (12.1, -12.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                roller_example.volts_to_rps(bad)


class WireTest(unittest.TestCase):
    IDS = {"roller": 10}

    def test_voltage_mode_encodes(self):
        self.assertEqual(
            _demand(roller_example.encode_voltage(_state(), self.IDS, 6.0)),
            (10, packet.MODE_VOLTAGE, 0, 6.0))
        payload = roller_example.encode_voltage(_state(), self.IDS, -3.0)
        self.assertEqual(len(payload), packet.PACKET_SIZE)

    def test_header_words_echo_state(self):
        payload = roller_example.encode_voltage(_state(), self.IDS, 6.0)
        self.assertEqual(struct.unpack_from("<QQQQ", payload, 4),
                         (7, 3, 1, 1000))

    def test_bad_volts_and_names_raise(self):
        with self.assertRaises(ValueError):
            roller_example.encode_voltage(_state(), self.IDS, 13.0)
        with self.assertRaises(KeyError):
            roller_example.encode_voltage(_state(), self.IDS, 6.0,
                                          name="flywheel")
        # Same validation the student API enforces everywhere.
        with self.assertRaises(ValueError):
            motor_commands.voltage(_state(), {"roller": 10}, "roller",
                                   float("nan"))


class SimTest(unittest.TestCase):
    def test_half_voltage_reaches_half_speed(self):
        result = roller_example.run_sim(6.0)
        self.assertAlmostEqual(result["expected_rps"], 50.0)
        self.assertGreater(result["final_rps"], 45.0)
        self.assertLess(result["final_rps"], 55.0)

    def test_zero_volts_holds_rest(self):
        result = roller_example.run_sim(0.0)
        self.assertAlmostEqual(result["final_rps"], 0.0)

    def test_ramp_limits_first_step(self):
        result = roller_example.run_sim(6.0)
        # 150 rps/s * 20 ms = 3 rps per tick: no snap on the bus.
        self.assertLessEqual(result["demands"][0], 3.0 + 1e-9)
        self.assertGreater(result["demands"][-1], 45.0)

    def test_window_always_ends_at_stop(self):
        result = roller_example.run_sim(6.0)
        self.assertEqual(result["stop_rps"], 0.0)

    def test_over_voltage_fails_before_motion(self):
        with self.assertRaises(ValueError):
            roller_example.run_sim(13.0)


class DiagnoseTest(unittest.TestCase):
    def test_healthy_push_reports_peaks(self):
        diagnosis = roller_example.diagnose(roller_example.run_sim(6.0))
        self.assertGreater(diagnosis["measured_peak"], 45.0)
        self.assertFalse(any("does not follow" in line
                             for line in diagnosis["lines"]))

    def test_dead_wheel_names_wiring_not_gains(self):
        result = roller_example.run_sim(6.0)
        result = dict(result, velocities=[0.0] * len(result["velocities"]),
                      final_rps=0.0)
        diagnosis = roller_example.diagnose(result)
        self.assertTrue(any("does not follow" in line
                            for line in diagnosis["lines"]))
        self.assertTrue(any("not gains" in line
                            for line in diagnosis["lines"]))


if __name__ == "__main__":
    unittest.main()
