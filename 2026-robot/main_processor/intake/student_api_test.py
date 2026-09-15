"""Student API tests: policy, units, motor commands, config, behaviors.

Pure stdlib + packet codecs: needs no shared lib, no shared memory, so
these run everywhere. Owner run:
  bazel test //2026-robot/main_processor/intake:student_api_test
"""

import math
import struct
import tempfile
import unittest
from pathlib import Path

from talos_nodes.intake import behaviors, control_config, motor_commands
from talos_nodes.intake import packet, student_policy, units
from talos_nodes.intake.student_policy import Feedback, Target


def _state():
    return {"config_id": 7, "boot_id": 3, "epoch": 1,
            "sample_time_us": 1000}


def _demand(payload):
    (demand,) = struct.unpack_from("<d", payload, 4 + 42)
    (mode,) = struct.unpack_from("<B", payload, 4 + 40)
    (slot,) = struct.unpack_from("<B", payload, 4 + 41)
    (mid,) = struct.unpack_from("<H", payload, 4 + 38)
    return (mid, mode, slot, demand)


class PolicyTest(unittest.TestCase):
    def test_sign_selects_presets(self):
        self.assertEqual(student_policy.command_for(Target(21.5)),
                         student_policy.INTAKE_RPS)
        self.assertEqual(student_policy.command_for(Target(-5.0)),
                         student_policy.EJECT_RPS)
        self.assertEqual(student_policy.command_for(Target(0.0)),
                         student_policy.STOP_RPS)

    def test_presets_match_student_section_values(self):
        self.assertEqual((student_policy.INTAKE_RPS,
                         student_policy.EJECT_RPS,
                         student_policy.STOP_RPS), (30.0, -20.0, 0.0))

    def test_beam_blocks_intake_only(self):
        blocked = Feedback(beam_broken=True)
        # Capture prevents further intake ...
        self.assertEqual(student_policy.command_for(Target(21.5), blocked),
                         student_policy.STOP_RPS)
        # ... but intentional ejection with the beam blocked is allowed.
        self.assertEqual(student_policy.command_for(Target(-5.0), blocked),
                         student_policy.EJECT_RPS)
        self.assertEqual(student_policy.command_for(Target(0.0), blocked),
                         student_policy.STOP_RPS)

    def test_clear_beam_drives_both_ways(self):
        for feedback in (None, Feedback(), Feedback(beam_broken=False)):
            self.assertEqual(
                student_policy.command_for(Target(21.5), feedback),
                student_policy.INTAKE_RPS)
            self.assertEqual(
                student_policy.command_for(Target(-21.5), feedback),
                student_policy.EJECT_RPS)

    def test_overrides_still_win(self):
        self.assertEqual(student_policy.command_for(
            Target(5.0), Feedback(), intake_rps=11.0), 11.0)
        self.assertEqual(student_policy.command_for(
            Target(-5.0), Feedback(beam_broken=True), eject_rps=-3.0), -3.0)

    def test_nonfinite_passes_through(self):
        self.assertEqual(student_policy.command_for(Target(float("inf"))),
                         float("inf"))
        self.assertTrue(math.isnan(
            student_policy.command_for(Target(float("nan")))))
        # Even with the beam blocked: the demand gate coasts, not the policy.
        self.assertEqual(student_policy.command_for(
            Target(float("inf")), Feedback(beam_broken=True)),
            float("inf"))

    def test_typed_shapes_reject_legacy_unions(self):
        for bad_target in (21.5, (123, True, 21.5),
                           {"roller_velocity_rps": 21.5}, None):
            with self.assertRaises(TypeError):
                student_policy.command_for(bad_target)
        for bad_feedback in (True, {"beam_broken": True}, 1.0):
            with self.assertRaises(TypeError):
                student_policy.command_for(Target(1.0), bad_feedback)

    def test_feedback_defaults_to_clear_and_valid(self):
        self.assertEqual(Feedback(), Feedback(beam_broken=False,
                                              beam_valid=True))

    def test_target_coerces_int_and_rejects_non_numbers(self):
        self.assertEqual(student_policy.command_for(Target(5)),
                         student_policy.INTAKE_RPS)
        self.assertIsInstance(Target(5).velocity_rps, float)
        for bad in (True, False, "30.0", None, (1.0,), {"v": 1.0},
                    b"1.0"):
            with self.assertRaises(TypeError):
                Target(bad)

    def test_feedback_full_shape_defaults(self):
        feedback = Feedback()
        self.assertEqual(feedback.measured_velocity_rps, 0.0)
        self.assertTrue(feedback.velocity_valid)
        self.assertEqual(feedback.measured_position_rot, 0.0)
        self.assertFalse(feedback.position_valid)
        self.assertTrue(feedback.enabled)
        self.assertFalse(feedback.fault)
        self.assertTrue(feedback.drive_ok)
        # Plain 0/1 ints from C-style decoders coerce; anything else
        # raises (see rejects test below).
        self.assertTrue(Feedback(beam_broken=1).beam_broken)
        self.assertFalse(Feedback(enabled=0).enabled)

    def test_feedback_rejects_transport_and_wrong_types(self):
        # Transport exposure: dicts/payloads are never feedback.
        for bad in ({"beam_broken": True}, b"\x01", (True,),
                    {"config_id": 7}):
            with self.assertRaises(TypeError):
                student_policy.command_for(Target(1.0), bad)
        # Wrong-typed fields raise at construction.
        with self.assertRaises(TypeError):
            Feedback(beam_broken="yes")
        with self.assertRaises(TypeError):
            Feedback(beam_valid=1.5)
        with self.assertRaises(TypeError):
            Feedback(measured_velocity_rps=True)
        with self.assertRaises(TypeError):
            Feedback(measured_velocity_rps="10.0")
        with self.assertRaises(TypeError):
            Feedback(measured_position_rot=None)
        with self.assertRaises(TypeError):
            Feedback(enabled="true")
        with self.assertRaises(TypeError):
            Feedback(fault=2)
        # Non-finite readings are allowed: validity flags carry trust.
        feedback = Feedback(measured_velocity_rps=float("inf"),
                            velocity_valid=False)
        self.assertFalse(feedback.drive_ok)

    def test_richer_feedback_never_steers_demand(self):
        # Validity/enabled/fault gate in the node, not the mapping: only
        # the ask's sign and beam_broken change the demand.
        variants = [
            Feedback(),
            Feedback(measured_velocity_rps=99.0, velocity_valid=False),
            Feedback(measured_position_rot=3.0, position_valid=True),
            Feedback(enabled=False),
            Feedback(fault=True),
            Feedback(beam_valid=False),
        ]
        for feedback in variants:
            self.assertEqual(
                student_policy.command_for(Target(21.5), feedback),
                student_policy.INTAKE_RPS)
            self.assertEqual(
                student_policy.command_for(Target(-21.5), feedback),
                student_policy.EJECT_RPS)
        blocked_variants = [
            Feedback(beam_broken=True),
            Feedback(beam_broken=True, fault=True, enabled=False,
                     velocity_valid=False),
        ]
        for feedback in blocked_variants:
            self.assertEqual(
                student_policy.command_for(Target(21.5), feedback),
                student_policy.STOP_RPS)
            self.assertEqual(
                student_policy.command_for(Target(-21.5), feedback),
                student_policy.EJECT_RPS)

    def test_drive_ok_advisory_gate(self):
        self.assertTrue(Feedback().drive_ok)
        self.assertFalse(Feedback(enabled=False).drive_ok)
        self.assertFalse(Feedback(fault=True).drive_ok)
        self.assertFalse(Feedback(velocity_valid=False).drive_ok)
        # Beam state alone never closes the advisory gate.
        self.assertTrue(Feedback(beam_broken=True).drive_ok)


class UnitsTest(unittest.TestCase):
    def test_rotor_mechanism_roundtrip(self):
        self.assertAlmostEqual(
            units.rotor_rps_to_mechanism_rps(
                units.mechanism_rps_to_rotor_rps(30.0, 25.0), 25.0), 30.0)
        self.assertAlmostEqual(
            units.rotor_to_mechanism_rotations(
                units.mechanism_rotations_to_rotor(2.0, 25.0), 25.0), 2.0)

    def test_direct_drive_is_identity(self):
        self.assertEqual(units.mechanism_rps_to_rotor_rps(30.0, 1.0), 30.0)

    def test_radians(self):
        self.assertAlmostEqual(units.rps_to_rad_per_s(1.0), 2.0 * math.pi)
        self.assertAlmostEqual(units.rad_per_s_to_rps(2.0 * math.pi), 1.0)

    def test_boundaries_reject(self):
        for bad in (0.0, -1.0, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                units.check_ratio(bad)
        with self.assertRaises(ValueError):
            units.check_rps(float("nan"))
        with self.assertRaises(ValueError):
            units.mechanism_rps_to_rotor_rps(1.0, 0.0)
        with self.assertRaises(ValueError):
            units.check_volts(13.0)
        with self.assertRaises(ValueError):
            units.check_duty(1.5)
        self.assertEqual(units.check_volts(12.0), 12.0)
        self.assertEqual(units.check_duty(-1.0), -1.0)

    def test_clamp_keeps_sign(self):
        self.assertEqual(units.clamp_demand(150.0, 100.0), 100.0)
        self.assertEqual(units.clamp_demand(-150.0, 100.0), -100.0)
        self.assertEqual(units.clamp_demand(30.0, 100.0), 30.0)
        self.assertEqual(units.clamp_demand(0.0, 100.0), 0.0)

    def test_rotor_mechanism_radians(self):
        # Mechanism rotations <-> radians round-trip ...
        self.assertAlmostEqual(
            units.radians_to_mechanism_rotations(
                units.mechanism_rotations_to_radians(2.0)), 2.0)
        self.assertAlmostEqual(units.mechanism_rotations_to_radians(0.5),
                               math.pi)
        # ... and radians are validated at the boundary.
        for bad in (float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                units.mechanism_rotations_to_radians(bad)
            with self.assertRaises(ValueError):
                units.radians_to_mechanism_rotations(bad)
        with self.assertRaises(ValueError):
            units.check_rotations(float("nan"))

    def test_rotor_frame_radians(self):
        # Rotor-frame radian helpers use the same factor under explicit
        # names, so frames never mix silently.
        self.assertAlmostEqual(units.rotor_rps_to_rotor_rad_per_s(1.0),
                               2.0 * math.pi)
        self.assertAlmostEqual(units.rotor_rad_per_s_to_rotor_rps(
            2.0 * math.pi), 1.0)
        with self.assertRaises(ValueError):
            units.rotor_rps_to_rotor_rad_per_s(float("nan"))
        with self.assertRaises(ValueError):
            units.rotor_rad_per_s_to_rotor_rps(float("inf"))

    def test_composed_rad_per_s_crosses_frames(self):
        # Mechanism rad/s -> rotor rev/s and back, through the ratio.
        self.assertAlmostEqual(
            units.mechanism_rad_per_s_to_rotor_rps(2.0 * math.pi, 25.0),
            25.0)
        self.assertAlmostEqual(
            units.rotor_rps_to_mechanism_rad_per_s(25.0, 25.0),
            2.0 * math.pi)
        with self.assertRaises(ValueError):
            units.mechanism_rad_per_s_to_rotor_rps(1.0, 0.0)
        with self.assertRaises(ValueError):
            units.rotor_rps_to_mechanism_rad_per_s(float("nan"), 1.0)

    def test_amps_validated_against_limits(self):
        self.assertEqual(units.check_amps(40.0, 40.0), 40.0)
        self.assertEqual(units.check_amps(-80.0, 80.0,
                                          "stator_current_a"), -80.0)
        with self.assertRaises(ValueError):
            units.check_amps(40.1, 40.0)
        with self.assertRaises(ValueError):
            units.check_amps(float("nan"), 40.0)
        with self.assertRaises(ValueError):
            units.check_amps(True, 40.0)
        for bad_limit in (0.0, -5.0, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                units.check_amps(1.0, bad_limit)


class MotorCommandsTest(unittest.TestCase):
    IDS = {"roller": 2}

    def test_each_mode_encodes(self):
        state = _state()
        self.assertEqual(
            _demand(motor_commands.duty_cycle(state, self.IDS, "roller",
                                              0.5)),
            (2, packet.MODE_DUTY_CYCLE, 0, 0.5))
        self.assertEqual(
            _demand(motor_commands.voltage(state, self.IDS, "roller", 6.0)),
            (2, packet.MODE_VOLTAGE, 0, 6.0))
        self.assertEqual(
            _demand(motor_commands.velocity(state, self.IDS, "roller",
                                            30.0, slot=0)),
            (2, packet.MODE_VELOCITY, 0, 30.0))
        self.assertEqual(
            _demand(motor_commands.position(state, self.IDS, "roller",
                                            2.5, slot=0)),
            (2, packet.MODE_POSITION, 0, 2.5))
        self.assertEqual(
            _demand(motor_commands.stop(state, self.IDS, "roller")),
            (2, packet.MODE_NEUTRAL, 0, 0.0))

    def test_header_words_echo_state(self):
        payload = motor_commands.velocity(_state(), self.IDS, "roller",
                                          30.0)
        words = struct.unpack_from("<QQQQ", payload, 4)
        self.assertEqual(words, (7, 3, 1, 1000))

    def test_unknown_name_raises(self):
        with self.assertRaises(KeyError):
            motor_commands.velocity(_state(), self.IDS, "flywheel", 1.0)

    def test_ranges_validated(self):
        state = _state()
        with self.assertRaises(ValueError):
            motor_commands.duty_cycle(state, self.IDS, "roller", 2.0)
        with self.assertRaises(ValueError):
            motor_commands.voltage(state, self.IDS, "roller", 13.0)
        with self.assertRaises(ValueError):
            motor_commands.velocity(state, self.IDS, "roller",
                                    float("nan"))
        with self.assertRaises(ValueError):
            motor_commands.position(state, self.IDS, "roller",
                                    float("inf"))

    def test_velocity_clamps_to_config_limit(self):
        state = _state()
        # Over-limit demands clamp with sign kept; in-limit pass through.
        self.assertEqual(
            _demand(motor_commands.velocity(state, self.IDS, "roller",
                                            150.0,
                                            max_velocity_rps=100.0)),
            (2, packet.MODE_VELOCITY, 0, 100.0))
        self.assertEqual(
            _demand(motor_commands.velocity(state, self.IDS, "roller",
                                            -150.0,
                                            max_velocity_rps=100.0)),
            (2, packet.MODE_VELOCITY, 0, -100.0))
        self.assertEqual(
            _demand(motor_commands.velocity(state, self.IDS, "roller",
                                            30.0,
                                            max_velocity_rps=100.0)),
            (2, packet.MODE_VELOCITY, 0, 30.0))
        # No limit given: no clamp (backward compatible).
        self.assertEqual(
            _demand(motor_commands.velocity(state, self.IDS, "roller",
                                            150.0)),
            (2, packet.MODE_VELOCITY, 0, 150.0))
        with self.assertRaises(ValueError):
            motor_commands.velocity(state, self.IDS, "roller", 10.0,
                                    max_velocity_rps=-1.0)

    def test_feedforward_and_position_validated(self):
        state = _state()
        for bad_ff in (float("nan"), float("inf"), float("-inf")):
            with self.assertRaises(ValueError):
                motor_commands.velocity(state, self.IDS, "roller", 10.0,
                                        feedforward_v=bad_ff)
            with self.assertRaises(ValueError):
                motor_commands.position(state, self.IDS, "roller", 1.0,
                                        feedforward_v=bad_ff)
        with self.assertRaises(ValueError):
            motor_commands.position(state, self.IDS, "roller",
                                    float("nan"))
        with self.assertRaises(ValueError):
            motor_commands.position(state, self.IDS, "roller", True)


class ControlConfigTest(unittest.TestCase):
    TOML = """\
[subsystem]
name = "intake"
period_us = 20000

[motors.roller]
type = "TalonFX"
bus = "rio"
can_id = 10
inverted = false
brake = true
max_voltage = 12.0
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }
"""

    def _write(self, text):
        tmp = tempfile.NamedTemporaryFile("w", suffix=".toml",
                                          delete=False)
        tmp.write(text)
        tmp.close()
        self.addCleanup(Path(tmp.name).unlink)
        return tmp.name

    def test_loads_untuned_roller(self):
        config = control_config.load_motor_config(self._write(self.TOML),
                                                  "roller")
        self.assertEqual(config.name, "roller")
        self.assertEqual(config.sensor_to_mechanism_ratio, 1.0)
        self.assertEqual(config.max_velocity_rps, 100.0)
        self.assertFalse(config.is_tuned)
        self.assertIn("UNTUNED", config.describe())

    def test_tuned_config_detected(self):
        text = self.TOML.replace("v = 0.0", "v = 0.12")
        config = control_config.load_motor_config(self._write(text),
                                                  "roller", slot=0)
        self.assertTrue(config.is_tuned)
        self.assertIn("TUNED", config.describe())
        self.assertIn("slot 0", config.describe())

    def test_missing_motor_raises(self):
        with self.assertRaises(KeyError):
            control_config.load_motor_config(self._write(self.TOML),
                                             "flywheel")

    def test_invalid_scaling_and_gains_raise(self):
        cases = [
            ("sensor_to_mechanism_ratio = 1.0",
             "sensor_to_mechanism_ratio = 0.0"),
            ("sensor_to_mechanism_ratio = 1.0",
             "sensor_to_mechanism_ratio = -2.0"),
            ("max_velocity_rps = 100.0", "max_velocity_rps = -1.0"),
            ("max_voltage = 12.0", "max_voltage = 0.0"),
            ("v = 0.0", "v = inf"),
        ]
        for old, new in cases:
            with self.assertRaises(ValueError):
                control_config.load_motor_config(
                    self._write(self.TOML.replace(old, new)), "roller")

    def test_describe_names_slot_gearing_and_tune(self):
        config = control_config.load_motor_config(self._write(self.TOML),
                                                  "roller")
        text = config.describe()
        self.assertIn("slot 0", text)
        self.assertIn("sensor_to_mechanism_ratio=1.0", text)
        self.assertIn("UNTUNED", text)

    def test_fallback_parser_matches_tomllib(self):
        text = self.TOML
        table = control_config._fallback_parse_motor(text, "roller")
        self.assertEqual(table["can_id"], 10)
        self.assertEqual(table["brake"], True)
        self.assertEqual(table["slot0"]["v"], 0.0)
        with self.assertRaises(KeyError):
            control_config._fallback_parse_motor(text, "flywheel")


class BehaviorsTest(unittest.TestCase):
    def test_debounce_immediate_by_default(self):
        debounce = behaviors.Debounce()
        self.assertTrue(debounce.update(True))
        self.assertFalse(debounce.update(False))

    def test_debounce_filters_chatter(self):
        debounce = behaviors.Debounce(stable_ticks=3)
        self.assertFalse(debounce.update(True))
        self.assertFalse(debounce.update(False))  # Chatter resets.
        self.assertFalse(debounce.update(True))
        self.assertFalse(debounce.update(True))
        self.assertTrue(debounce.update(True))  # Three in a row flips.
        self.assertTrue(debounce.update(True))
        with self.assertRaises(ValueError):
            behaviors.Debounce(stable_ticks=0)

    def test_edge_detector_counts_once(self):
        edges = behaviors.EdgeDetector()
        for value in (False, True, True, True, False, False, True):
            rose, _ = edges.update(value)
        self.assertEqual(edges.rising, 2)
        self.assertEqual(edges.falling, 1)
        rose, fell = edges.update(True)
        self.assertEqual((rose, fell), (False, False))

    def test_elapsed_timer(self):
        timer = behaviors.ElapsedTimer(0.1)
        self.assertFalse(timer.update(0.05))
        self.assertTrue(timer.update(0.05))
        self.assertTrue(timer.expired)
        with self.assertRaises(ValueError):
            behaviors.ElapsedTimer(-1.0)
        with self.assertRaises(ValueError):
            timer.update(float("nan"))

    def test_run_until_or_timeout_meets_goal(self):
        calls = []
        steps, done = behaviors.run_until_or_timeout(
            lambda dt: calls.append(dt), lambda: len(calls) >= 3,
            timeout_s=1.0, dt_s=0.02)
        self.assertEqual((steps, done), (3, True))

    def test_run_until_or_timeout_times_out(self):
        steps, done = behaviors.run_until_or_timeout(
            lambda dt: None, lambda: False, timeout_s=0.1, dt_s=0.02)
        self.assertFalse(done)
        self.assertEqual(steps, 5)

    def test_run_until_already_done_runs_zero_steps(self):
        ran = []
        steps, done = behaviors.run_until_or_timeout(
            lambda dt: ran.append(dt), lambda: True, timeout_s=1.0,
            dt_s=0.02)
        self.assertEqual((steps, done, ran), (0, True, []))

    def test_slew_limiter_ramps_at_rate(self):
        limiter = behaviors.SlewLimiter(100.0)  # 100 rps per second.
        self.assertAlmostEqual(limiter.update(30.0, 0.1), 10.0)
        self.assertAlmostEqual(limiter.update(30.0, 0.1), 20.0)
        self.assertAlmostEqual(limiter.update(30.0, 0.1), 30.0)
        # Small remaining errors snap instead of oscillating.
        self.assertAlmostEqual(limiter.update(30.0, 0.1), 30.0)
        # Down-ramps are symmetric, and reset snaps the output.
        self.assertAlmostEqual(limiter.update(-30.0, 0.3), 0.0)
        self.assertEqual(limiter.reset(-30.0), -30.0)
        self.assertEqual(limiter.update(0.0, 0.0), -30.0)  # Zero dt holds.

    def test_slew_limiter_validates(self):
        for bad_rate in (0.0, -1.0, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                behaviors.SlewLimiter(bad_rate)
        limiter = behaviors.SlewLimiter(10.0)
        with self.assertRaises(ValueError):
            limiter.update(float("nan"), 0.02)
        with self.assertRaises(ValueError):
            limiter.update(1.0, -0.02)
        with self.assertRaises(ValueError):
            limiter.update(1.0, float("nan"))
        with self.assertRaises(ValueError):
            limiter.reset(float("inf"))
        with self.assertRaises(ValueError):
            behaviors.SlewLimiter(10.0, initial_rps=float("nan"))

    def test_cancellation_demand_maps_every_outcome(self):
        # Completion holds the goal; timeout/interruption/disable stop.
        self.assertEqual(behaviors.cancellation_demand(done=True,
                                                       goal_rps=80.0),
                         80.0)
        self.assertEqual(behaviors.cancellation_demand(done=False,
                                                       goal_rps=80.0),
                         0.0)
        self.assertEqual(behaviors.cancellation_demand(done=False,
                                                       goal_rps=80.0,
                                                       safe_rps=-20.0),
                         -20.0)
        # Interruption propagates out of the loop; the caller still
        # commands safe in cleanup.
        def _boom(dt_s):
            raise RuntimeError("plant fault")

        with self.assertRaises(RuntimeError):
            behaviors.run_until_or_timeout(_boom, lambda: False,
                                           timeout_s=1.0, dt_s=0.02)
        self.assertEqual(behaviors.cancellation_demand(done=False,
                                                       goal_rps=80.0),
                         0.0)
        with self.assertRaises(TypeError):
            behaviors.cancellation_demand(done=1, goal_rps=80.0)
        with self.assertRaises(TypeError):
            behaviors.cancellation_demand(done=True, goal_rps=True)
        with self.assertRaises(ValueError):
            behaviors.cancellation_demand(done=True,
                                          goal_rps=float("inf"))
        with self.assertRaises(ValueError):
            behaviors.cancellation_demand(done=False, goal_rps=80.0,
                                          safe_rps=float("nan"))


if __name__ == "__main__":
    unittest.main()
