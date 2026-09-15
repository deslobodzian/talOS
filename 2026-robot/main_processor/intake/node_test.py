"""Intake tests: policy math (no shm) + --describe JSON shape.

The policy binds its gating answers from the C++ ground truth through the
node_api ABI, so -- like --describe -- it needs the shared lib, still
without touching shared memory.

Owner run:
  bazel test //2026-robot/main_processor/intake:node_test
"""

import json
import math
import os
import shutil
import struct
import subprocess
import sys
import unittest

from pathlib import Path
import tempfile

from python.runfiles import runfiles
from talos_nodes.intake import packet, node, student_policy
from talos_nodes.intake import main as intake_main
from talos_runtime import node_api, runner

_HERE = Path(__file__).parent


def _needs_lib():
    """Skip unless libtalos_node.so is reachable (policy binds its gating)."""
    if node_api.find_lib() is None and not os.environ.get("TALOS_NODE_LIB"):
        return "libtalos_node.so not built; set TALOS_NODE_LIB"
    return None


def make_hw_state(config_id=7, boot_id=3, epoch=1, sample_time_us=1000,
                  flags=packet.FLAG_CONFIGURED | packet.FLAG_ENABLED,
                  roller_id=2, roller_valid=True, roller_vel=10.0,
                  beam_id=1, beam_valid=True, beam_broken=False):
    """Craft a minimal State wire image per packet.decode_hw_state_minimal."""
    header = struct.pack("<QQQQQIHHHHHHH", config_id, boot_id, epoch,
                         sample_time_us, 0, flags, 1, 0, 1, 0, 0, 0, 0)
    motor = struct.pack("<HB", roller_id, 1 if roller_valid else 0)
    motor += struct.pack("<dddd", 0.0, roller_vel, 0.0, 0.0)
    motor += struct.pack("<II", 0, 0)
    dio = struct.pack("<HBB", beam_id, 1 if beam_valid else 0,
                      1 if beam_broken else 0)
    wire = header + motor + dio
    envelope = struct.pack("<I", len(wire)) + wire
    return envelope + bytes(packet.PACKET_SIZE - len(envelope))


def decode_command_demand(request):
    """Read back the one-motor command at its documented offsets."""
    assert len(request) == packet.PACKET_SIZE
    (size,) = struct.unpack_from("<I", request, 0)
    assert size == 58  # 38 header + 20 one-motor slice.
    (count,) = struct.unpack_from("<H", request, 4 + 32)
    assert count == 1
    (mid,) = struct.unpack_from("<H", request, 4 + 38)
    (mode,) = struct.unpack_from("<B", request, 4 + 40)
    (slot,) = struct.unpack_from("<B", request, 4 + 41)
    (demand,) = struct.unpack_from("<d", request, 4 + 42)
    (ff,) = struct.unpack_from("<d", request, 4 + 50)
    return (mid, mode, slot, demand, ff)


class PacketTest(unittest.TestCase):
    def test_target_roundtrip_struct(self):
        blob = packet.encode_target(123456789, True, 21.5)
        self.assertEqual(len(blob), packet.TARGET_SIZE)
        self.assertEqual(packet.decode_target(blob), (123456789, True, 21.5))

    def test_target_bindings_agree(self):
        blob = packet.encode_target(999, False, -3.25)
        self.assertEqual(node.decode_target_bindings(blob), (999, False, -3.25))
        back = packet.decode_target(blob)
        self.assertEqual(back[0], 999)
        self.assertEqual(back[1], False)
        self.assertAlmostEqual(back[2], -3.25, places=5)

    def test_target_builder_crosscheck(self):
        import flatbuffers
        from talos.intake.IntakeTarget import CreateIntakeTarget
        builder = flatbuffers.Builder(32)
        CreateIntakeTarget(builder, 4242, True, 8.5)
        head = builder.head
        built = bytes(builder.Bytes[head:head + packet.TARGET_SIZE])
        self.assertEqual(built, packet.encode_target(4242, True, 8.5))

    def test_state_roundtrip(self):
        blob = packet.encode_state(10**9, True, 4.0, True, 21.5,
                                   packet.INHIBIT_DRIVE)
        self.assertEqual(len(blob), packet.STATE_SIZE)
        issued, enabled, vel, beam, requested, inhibit = packet.decode_state(
            blob)
        self.assertEqual((issued, enabled, beam), (10**9, True, True))
        self.assertAlmostEqual(vel, 4.0, places=5)
        self.assertAlmostEqual(requested, 21.5, places=5)
        self.assertEqual(inhibit, packet.INHIBIT_DRIVE)

    def test_state_roundtrip_defaults(self):
        # Older call sites pass four fields; requested/inhibit default to
        # stopped-and-driving.
        blob = packet.encode_state(10**9, True, 4.0, True)
        self.assertEqual(len(blob), packet.STATE_SIZE)
        self.assertEqual(packet.decode_state(blob)[4:], (0.0,
                                                         packet.INHIBIT_DRIVE))

    def test_state_bad_inhibit_rejected(self):
        with self.assertRaises(ValueError):
            packet.encode_state(0, True, 0.0, False, 0.0, 99)

    def test_state_builder_crosscheck(self):
        import flatbuffers
        from talos.intake.IntakeState import CreateIntakeState
        builder = flatbuffers.Builder(64)
        CreateIntakeState(builder, 777, False, 1.5, True, 2.5, 3)
        head = builder.head
        built = bytes(builder.Bytes[head:head + packet.STATE_SIZE])
        self.assertEqual(built, packet.encode_state(777, False, 1.5, True,
                                                   2.5, 3))

    def test_command_exact_offsets(self):
        req = packet.encode_command(7, 3, 1, 1000, 2, packet.MODE_VELOCITY,
                                    0, 21.5, 0.0)
        self.assertEqual(len(req), packet.PACKET_SIZE)
        self.assertEqual(decode_command_demand(req), (2, 3, 0, 21.5, 0.0))
        # Tail past the wire must be zero (byte-stable outputs).
        (size,) = struct.unpack_from("<I", req, 0)
        self.assertEqual(req[4 + size:], bytes(packet.PACKET_SIZE - 4 - size))

    def test_hw_state_minimal_decode(self):
        payload = make_hw_state(roller_vel=33.0, beam_broken=True)
        state = packet.decode_hw_state_minimal(payload, 2, 1)
        self.assertIsNotNone(state)
        self.assertEqual(state["config_id"], 7)
        self.assertEqual(state["sample_time_us"], 1000)
        self.assertTrue(state["roller_valid"])
        self.assertAlmostEqual(state["roller_velocity_rps"], 33.0)
        self.assertTrue(state["beam_broken"])

    def test_hw_state_malformed(self):
        self.assertIsNone(packet.decode_hw_state_minimal(b"\x00" * 10, 2, 1))
        self.assertIsNone(packet.decode_hw_state_minimal(b"\xff" * 100, 2, 1))

    def test_gate_predicates_answered_by_abi(self):
        # The decisions live in C++ (gate.h); python only binds them.
        reason = _needs_lib()
        if reason is not None:
            self.skipTest(reason)
        self.assertTrue(node_api.accept_state(7, None, {"config_id": 7,
                                                        "boot_id": 3,
                                                        "sample_time_us":
                                                        1000}))
        self.assertFalse(node_api.accept_state(99, None, {"config_id": 7,
                                                          "boot_id": 3,
                                                          "sample_time_us":
                                                          1000}))
        self.assertTrue(node_api.gateway_ready(
            packet.FLAG_CONFIGURED | packet.FLAG_ENABLED))
        self.assertFalse(node_api.gateway_ready(0))
        now = 5_000_000_000
        self.assertTrue(node_api.target_fresh_ns(now, now - 1_000_000, True,
                                                now, packet.COMMAND_TIMEOUT_US))
        self.assertFalse(node_api.target_fresh_ns(now, now - 1_000_000, False,
                                                 now,
                                                 packet.COMMAND_TIMEOUT_US))


class PolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Every policy fold asks the C++ predicates through the ABI: needs
        # the shared lib like --describe does, still no shared memory.
        reason = _needs_lib()
        if reason is not None:
            raise unittest.SkipTest(reason)

    def _live(self, **kw):
        return node.IntakePolicy(roller_id=2, beam_break_id=1, **kw)

    def test_no_state_ticks_nothing(self):
        policy = self._live()
        self.assertEqual(policy.tick(10**9), (None, None))

    def test_valid_target_drives_velocity(self):
        policy = self._live()
        now = 5_000_000_000
        self.assertTrue(policy.on_hw_state(make_hw_state(), now))
        self.assertTrue(policy.on_target(
            packet.encode_target(now - 1_000_000, True, 21.5), now))
        request, published = policy.tick(now)
        # A positive ask runs at the INTAKE_RPS preset, not passthrough.
        self.assertEqual(decode_command_demand(request)[:4],
                         (2, 3, 0, student_policy.INTAKE_RPS))
        issued, enabled, vel, beam, requested, inhibit = packet.decode_state(
            published)
        self.assertEqual(issued, 1000 * 1000)
        self.assertTrue(enabled)
        self.assertAlmostEqual(vel, 10.0)
        self.assertFalse(beam)
        # Studio view: requested echoes the live demand, measured the wheel.
        self.assertAlmostEqual(requested, student_policy.INTAKE_RPS,
                               places=5)
        self.assertEqual(inhibit, packet.INHIBIT_DRIVE)
        self.assertEqual(policy.debug_snapshot()["inhibit"],
                         packet.INHIBIT_DRIVE)

    def test_intake_vs_eject_follow_config(self):
        # INTAKE and EJECT asks drive at their configured presets, and a
        # zero ask holds stop -- all as velocity demands.
        now = 5_000_000_000
        for ask, want in ((25.0, student_policy.INTAKE_RPS),
                          (-25.0, student_policy.EJECT_RPS),
                          (0.0, student_policy.STOP_RPS)):
            policy = self._live()
            self.assertTrue(policy.on_hw_state(make_hw_state(), now))
            self.assertTrue(policy.on_target(
                packet.encode_target(now - 1_000, True, ask), now))
            request, published = policy.tick(now)
            mid, mode, slot, demand, _ = decode_command_demand(request)
            self.assertEqual((mid, mode, slot), (2, packet.MODE_VELOCITY,
                                                0))
            self.assertAlmostEqual(demand, want, places=5)
            self.assertAlmostEqual(packet.decode_state(published)[4], want,
                                   places=5)
            self.assertEqual(packet.decode_state(published)[5],
                             packet.INHIBIT_DRIVE)

    def test_beam_break_holds_stop_against_intake(self):
        # A broken beam means a captured piece: the wheel holds stop
        # instead of pulling at INTAKE_RPS.
        now = 5_000_000_000
        policy = self._live()
        self.assertTrue(
            policy.on_hw_state(make_hw_state(beam_broken=True), now))
        self.assertTrue(policy.on_target(
            packet.encode_target(now - 1_000, True, 25.0), now))
        request, published = policy.tick(now)
        mid, mode, slot, demand, _ = decode_command_demand(request)
        self.assertEqual(mode, packet.MODE_VELOCITY)
        self.assertAlmostEqual(demand, student_policy.STOP_RPS, places=5)
        self.assertAlmostEqual(packet.decode_state(published)[4], 0.0,
                               places=5)
        self.assertTrue(packet.decode_state(published)[3])

    def test_beam_break_allows_eject(self):
        # Intentional ejection with the beam blocked: the captured piece
        # gets out by running the wheel backwards.
        now = 5_000_000_000
        policy = self._live()
        self.assertTrue(
            policy.on_hw_state(make_hw_state(beam_broken=True), now))
        self.assertTrue(policy.on_target(
            packet.encode_target(now - 1_000, True, -25.0), now))
        request, published = policy.tick(now)
        mid, mode, slot, demand, _ = decode_command_demand(request)
        self.assertEqual(mode, packet.MODE_VELOCITY)
        self.assertAlmostEqual(demand, student_policy.EJECT_RPS, places=5)
        self.assertAlmostEqual(packet.decode_state(published)[4],
                               student_policy.EJECT_RPS, places=5)
        self.assertTrue(packet.decode_state(published)[3])

    def test_capture_count_and_debounce(self):
        # Each beam-break arrival counts once; chatter within one tick's
        # debounce window does not double-count.
        now = 5_000_000_000
        policy = self._live()
        self.assertTrue(policy.on_hw_state(make_hw_state(), now))
        self.assertTrue(policy.on_target(
            packet.encode_target(now - 1_000, True, 25.0), now))
        policy.tick(now)
        self.assertEqual(policy.capture_count, 0)
        policy.on_hw_state(
            make_hw_state(beam_broken=True, sample_time_us=2000), now + 1)
        policy.tick(now + 1)
        policy.on_hw_state(
            make_hw_state(beam_broken=True, sample_time_us=3000), now + 2)
        policy.tick(now + 2)
        self.assertEqual(policy.capture_count, 1)
        self.assertEqual(policy.debug_snapshot()["captures"], 1)

    def test_student_constants_take_effect(self):
        # Editing INTAKE_RPS/EJECT_RPS in student_policy.py -- the single
        # copy -- changes tick behavior.
        old_intake, old_eject = (student_policy.INTAKE_RPS,
                                 student_policy.EJECT_RPS)
        student_policy.INTAKE_RPS, student_policy.EJECT_RPS = 45.0, -7.5
        try:
            now = 5_000_000_000
            for ask, want in ((10.0, 45.0), (-10.0, -7.5)):
                policy = self._live()
                self.assertTrue(policy.on_hw_state(make_hw_state(), now))
                self.assertTrue(policy.on_target(
                    packet.encode_target(now - 1_000, True, ask), now))
                request, _ = policy.tick(now)
                self.assertAlmostEqual(decode_command_demand(request)[3],
                                       want, places=5)
        finally:
            (student_policy.INTAKE_RPS,
             student_policy.EJECT_RPS) = old_intake, old_eject

    def test_beam_break_reaches_state(self):
        policy = self._live()
        now = 5_000_000_000
        policy.on_hw_state(make_hw_state(beam_broken=True), now)
        policy.on_target(packet.encode_target(now - 1_000_000, True, 5.0),
                         now)
        _, published = policy.tick(now)
        self.assertTrue(packet.decode_state(published)[3])

    def test_stale_target_coasts_neutral(self):
        policy = self._live()
        now = 5_000_000_000
        policy.on_hw_state(make_hw_state(), now)
        old = now - (packet.COMMAND_TIMEOUT_US * 1000 + 1)
        policy.on_target(packet.encode_target(old, True, 21.5), now)
        request, published = policy.tick(now)
        mid, mode, slot, demand, _ = decode_command_demand(request)
        self.assertEqual((mid, mode, demand), (2, packet.MODE_NEUTRAL, 0.0))

    def test_disabled_target_coasts_neutral(self):
        policy = self._live()
        now = 5_000_000_000
        policy.on_hw_state(make_hw_state(), now)
        policy.on_target(packet.encode_target(now - 1_000, False, 21.5), now)
        request, _ = policy.tick(now)
        self.assertEqual(decode_command_demand(request)[1],
                         packet.MODE_NEUTRAL)

    def test_gateway_fault_coasts_neutral(self):
        policy = self._live()
        now = 5_000_000_000
        policy.on_hw_state(make_hw_state(flags=packet.FLAG_CONFIGURED |
                                         packet.FLAG_ENABLED |
                                         packet.FLAG_HARDWARE_FAULT), now)
        policy.on_target(packet.encode_target(now - 1_000, True, 21.5), now)
        request, _ = policy.tick(now)
        self.assertEqual(decode_command_demand(request)[1],
                         packet.MODE_NEUTRAL)

    def test_inhibit_reasons_name_the_failed_gate(self):
        now = 5_000_000_000

        def state_for(policy, **kw):
            issued = kw.pop("issued", now - 1_000)
            tenabled = kw.pop("tenabled", True)
            policy.on_hw_state(make_hw_state(**kw), now)
            policy.on_target(packet.encode_target(issued, tenabled, 21.5),
                             now)
            _, published = policy.tick(now)
            return packet.decode_state(published)

        fresh = {}
        policy = self._live()
        _, _, _, _, requested, inhibit = state_for(policy, **fresh)
        self.assertEqual(inhibit, packet.INHIBIT_DRIVE)
        self.assertAlmostEqual(requested, student_policy.INTAKE_RPS,
                               places=5)

        policy = self._live()
        _, _, _, _, requested, inhibit = state_for(
            policy, flags=packet.FLAG_CONFIGURED)
        self.assertEqual(inhibit, packet.INHIBIT_GATEWAY)
        # A live arbiter ask still shows as requested while inhibited.
        self.assertAlmostEqual(requested, student_policy.INTAKE_RPS,
                               places=5)

        policy = self._live()
        _, _, _, _, requested, inhibit = state_for(policy, tenabled=False)
        self.assertEqual(inhibit, packet.INHIBIT_TARGET_DISABLED)
        self.assertEqual(requested, 0.0)

        policy = self._live()
        old = now - (packet.COMMAND_TIMEOUT_US * 1000 + 1)
        _, _, _, _, requested, inhibit = state_for(policy, issued=old)
        self.assertEqual(inhibit, packet.INHIBIT_TARGET_STALE)
        self.assertEqual(requested, 0.0)

        policy = self._live()
        _, _, _, _, _, inhibit = state_for(policy, roller_valid=False)
        self.assertEqual(inhibit, packet.INHIBIT_FEEDBACK_INVALID)

    def test_nonfinite_demand_coasts_neutral(self):
        policy = self._live()
        now = 5_000_000_000
        policy.on_hw_state(make_hw_state(), now)
        policy.on_target(packet.encode_target(now - 1_000, True, float("inf")),
                         now)
        request, published = policy.tick(now)
        self.assertEqual(decode_command_demand(request)[1],
                         packet.MODE_NEUTRAL)
        self.assertEqual(packet.decode_state(published)[5],
                         packet.INHIBIT_DEMAND_NONFINITE)

    def test_student_command_maps_sign_to_presets(self):
        # The student mapping snaps the ask to its preset: positive asks
        # intake, negative asks eject, zero holds stop. Presets live in
        # student_policy (single copy); node.py defines no copies.
        self.assertEqual(node.student_command_for(21.5),
                         student_policy.INTAKE_RPS)
        self.assertEqual(node.student_command_for(-21.5),
                         student_policy.EJECT_RPS)
        self.assertEqual(node.student_command_for(0.0),
                         student_policy.STOP_RPS)
        self.assertEqual(
            node.student_command_for(student_policy.EJECT_RPS),
            student_policy.EJECT_RPS)
        self.assertFalse(hasattr(node, "INTAKE_RPS"))
        self.assertFalse(hasattr(node, "EJECT_RPS"))
        self.assertFalse(hasattr(node, "STOP_RPS"))

    def test_student_policy_command_for(self):
        # Direct unit tests for the student policy: demand mapping plus
        # directional beam-break handling (intake held, eject allowed).
        # Full typed-shape coverage lives in student_api_test.py (no
        # shared lib needed); this exercises the policy through the node.
        from talos_nodes.intake.student_policy import Feedback, Target
        self.assertEqual(student_policy.command_for(Target(21.5)),
                         student_policy.INTAKE_RPS)
        self.assertEqual(student_policy.command_for(
            Target(-5.0), Feedback(beam_broken=False)),
            student_policy.EJECT_RPS)
        self.assertEqual(student_policy.command_for(Target(0.0)),
                         student_policy.STOP_RPS)
        # A broken beam holds stop against intake but allows eject.
        self.assertEqual(student_policy.command_for(
            Target(21.5), Feedback(beam_broken=True)),
            student_policy.STOP_RPS)
        self.assertEqual(student_policy.command_for(
            Target(-5.0), Feedback(beam_broken=True)),
            student_policy.EJECT_RPS)
        # Non-finite asks pass through so the demand gate still coasts.
        self.assertEqual(student_policy.command_for(Target(float("inf"))),
                         float("inf"))
        self.assertTrue(math.isnan(
            student_policy.command_for(Target(float("nan")))))

    def test_tick_builds_typed_feedback(self):
        # The tick hands command_for decoded values only (never transport):
        # measured velocity/validity, beam pair, enabled, fault.
        seen = {}
        real = student_policy.command_for

        def _recording(target, feedback=None, **kw):
            seen["target"] = target
            seen["feedback"] = feedback
            return real(target, feedback, **kw)

        student_policy.command_for = _recording
        try:
            now = 5_000_000_000
            policy = self._live()
            self.assertTrue(policy.on_hw_state(
                make_hw_state(roller_vel=33.0, beam_broken=True), now))
            self.assertTrue(policy.on_target(
                packet.encode_target(now - 1_000, True, 21.5), now))
            request, _ = policy.tick(now)
            feedback = seen["feedback"]
            self.assertIsInstance(feedback,
                                  student_policy.Feedback)
            self.assertEqual(feedback.measured_velocity_rps, 33.0)
            self.assertTrue(feedback.velocity_valid)
            self.assertTrue(feedback.beam_broken)
            self.assertTrue(feedback.beam_valid)
            self.assertTrue(feedback.enabled)
            self.assertFalse(feedback.fault)
            self.assertTrue(feedback.drive_ok)
            # Beam-held capture still stops intake through the typed path.
            self.assertEqual(decode_command_demand(request)[3],
                             student_policy.STOP_RPS)
        finally:
            student_policy.command_for = real

    def test_tick_reports_fault_through_feedback(self):
        seen = {}
        real = student_policy.command_for

        def _recording(target, feedback=None, **kw):
            seen["feedback"] = feedback
            return real(target, feedback, **kw)

        student_policy.command_for = _recording
        try:
            now = 5_000_000_000
            policy = self._live()
            self.assertTrue(policy.on_hw_state(make_hw_state(
                flags=packet.FLAG_CONFIGURED | packet.FLAG_ENABLED |
                packet.FLAG_HARDWARE_FAULT), now))
            self.assertTrue(policy.on_target(
                packet.encode_target(now - 1_000, True, 21.5), now))
            request, published = policy.tick(now)
            # Fault latches in the typed feedback; the gateway gate still
            # owns the stop (neutral, never the last demand held open).
            self.assertTrue(seen["feedback"].fault)
            self.assertFalse(seen["feedback"].drive_ok)
            mid, mode, slot, demand, _ = decode_command_demand(request)
            self.assertEqual((mode, demand),
                             (packet.MODE_NEUTRAL, 0.0))
            self.assertEqual(packet.decode_state(published)[5],
                             packet.INHIBIT_GATEWAY)
        finally:
            student_policy.command_for = real

    def test_beam_invalid_sample_still_drives(self):
        # Beam validity rides along in Feedback but never gates the
        # wheel: only the roller sample's validity inhibits.
        now = 5_000_000_000
        policy = self._live()
        self.assertTrue(policy.on_hw_state(
            make_hw_state(beam_valid=False, beam_broken=False), now))
        self.assertTrue(policy.on_target(
            packet.encode_target(now - 1_000, True, 21.5), now))
        request, published = policy.tick(now)
        self.assertEqual(decode_command_demand(request)[1],
                         packet.MODE_VELOCITY)
        self.assertEqual(packet.decode_state(published)[5],
                         packet.INHIBIT_DRIVE)

    def test_nonfinite_sensor_marks_feedback_invalid(self):
        # An inf wheel reading decodes to valid=False (packet layer) and
        # flows through Feedback with the flag, not finiteness, as trust.
        now = 5_000_000_000
        policy = self._live()
        self.assertTrue(policy.on_hw_state(
            make_hw_state(roller_vel=float("inf")), now))
        self.assertFalse(policy._state["roller_valid"])
        self.assertTrue(policy.on_target(
            packet.encode_target(now - 1_000, True, 21.5), now))
        request, published = policy.tick(now)
        self.assertEqual(decode_command_demand(request)[1],
                         packet.MODE_NEUTRAL)
        self.assertEqual(packet.decode_state(published)[5],
                         packet.INHIBIT_FEEDBACK_INVALID)

    def test_cancellation_table_commands_neutral(self):
        # Every non-drive outcome -- timeout, disable, gateway fault,
        # invalid feedback, non-finite demand -- says stop (neutral at
        # 0.0) rather than holding the last demand.
        now = 5_000_000_000
        old = now - (packet.COMMAND_TIMEOUT_US * 1000 + 1)
        cases = [
            {"issued": old},  # Stale target: timeout.
            {"tenabled": False},  # Disabled target.
            {"flags": packet.FLAG_CONFIGURED},  # Gateway not ready.
            {"flags": (packet.FLAG_CONFIGURED | packet.FLAG_ENABLED |
                       packet.FLAG_HARDWARE_FAULT)},  # Latched fault.
            {"roller_valid": False},  # Invalid feedback.
        ]
        for case in cases:
            policy = self._live()
            issued = case.pop("issued", now - 1_000)
            tenabled = case.pop("tenabled", True)
            policy.on_hw_state(make_hw_state(**case), now)
            policy.on_target(packet.encode_target(issued, tenabled,
                                                  21.5), now)
            request, _ = policy.tick(now)
            mid, mode, slot, demand, _ = decode_command_demand(request)
            self.assertEqual((mode, demand),
                             (packet.MODE_NEUTRAL, 0.0))
        # Non-finite demand coasts too (INHIBIT_DEMAND_NONFINITE).
        policy = self._live()
        policy.on_hw_state(make_hw_state(), now)
        policy.on_target(packet.encode_target(now - 1_000, True,
                                              float("nan")), now)
        request, _ = policy.tick(now)
        self.assertEqual(decode_command_demand(request)[1],
                         packet.MODE_NEUTRAL)

    def test_measured_velocity_reaches_snapshot(self):
        policy = self._live()
        now = 5_000_000_000
        policy.on_hw_state(make_hw_state(roller_vel=33.0), now)
        policy.on_target(packet.encode_target(now - 1_000, True, 21.5),
                         now)
        policy.tick(now)
        snapshot = policy.debug_snapshot()
        self.assertAlmostEqual(snapshot["measured_rps"], 33.0)
        self.assertAlmostEqual(snapshot["requested_rps"],
                               student_policy.INTAKE_RPS)

    def test_stale_sample_dropped(self):
        policy = self._live()
        now = 5_000_000_000
        self.assertTrue(policy.on_hw_state(
            make_hw_state(sample_time_us=2000), now))
        self.assertFalse(policy.on_hw_state(
            make_hw_state(sample_time_us=1000), now + 1))
        self.assertEqual(policy._state["sample_time_us"], 2000)

    def test_config_gate(self):
        policy = self._live(config_id=99)
        now = 5_000_000_000
        self.assertFalse(policy.on_hw_state(make_hw_state(config_id=7), now))
        self.assertTrue(policy.on_hw_state(make_hw_state(config_id=99), now))


class ConfigTest(unittest.TestCase):
    def test_resolve_standalone_subsystem(self):
        path = str(_HERE / "testdata" / "subsystem.toml")
        roller_id, beam_id, period_us = node.resolve_ids(path)
        # Sorted (subsystem, device): intake_beam < roller -> 1, 2.
        self.assertEqual((roller_id, beam_id, period_us), (2, 1, 20000))

    def test_from_context(self):
        path = str(_HERE / "testdata" / "subsystem.toml")
        with node_api.load_config(path) as cfg:
            ctx = runner.NodeContext("intake", cfg, simulation=False, session_id=42)
            self.assertEqual(ctx.get_device_id("roller"), 2)
            self.assertEqual(ctx.get_device_id("intake_beam"), 1)
            intake = node.IntakeNode.from_context(ctx)
            try:
                self.assertEqual(intake.policy.roller_id, 2)
                self.assertEqual(intake.policy.beam_break_id, 1)
                self.assertEqual(intake.policy.period_us, 20000)
                self.assertEqual(intake.policy.config_id, cfg.config_id)
            finally:
                intake.close()

    def test_missing_device_raises(self):
        path = str(_HERE / "testdata" / "subsystem.toml")
        with self.assertRaises(RuntimeError):
            node.resolve_ids(path, roller="nonexistent_roller")
        with self.assertRaises(RuntimeError):
            node.resolve_ids(path, beam_break="nonexistent_beam")


class PackagingTest(unittest.TestCase):
    def test_packages_relocate_and_share_namespace(self):
        import flatbuffers
        import talos_runtime
        import talos.intake
        import talos_nodes.intake

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            # Copy only the declared packages, with no original runfiles tree.
            for module, destination in (
                    (talos_runtime, "talos_runtime"),
                    (talos_nodes.intake, "talos_nodes/intake"),
                    (talos.intake, "talos/intake"),
                    (flatbuffers, "flatbuffers")):
                shutil.copytree(Path(module.__file__).parent,
                                root / destination)
            # A separately installed node must share the namespace, not mask it.
            peer = root / "another_distribution" / "talos_nodes"
            peer.mkdir(parents=True)
            (peer / "peer.py").write_text("NAME = 'peer'\n")
            for name in ("node", "packet", "rtms", "node_api"):
                (root / (name + ".py")).write_text(
                    "raise AssertionError('bare import used')\n")
            env = dict(os.environ)
            for name in ("PYTHONPATH", "TALOS_NODE_LIB", "RUNFILES_DIR",
                         "RUNFILES_MANIFEST_FILE", "TEST_SRCDIR"):
                env.pop(name, None)
            code = """
import sys
from pathlib import Path
root = Path(sys.argv[1])
sys.path[:0] = [str(root), str(root / 'another_distribution')]
from talos_nodes import peer
from talos_nodes.intake import main
from talos_runtime import node_api
assert peer.NAME == 'peer'
assert 'flatbuffers' not in sys.modules
assert 'talos_nodes.intake.node' not in sys.modules
assert Path(node_api.find_lib()) == root / 'talos_runtime/libtalos_node.so'
assert main.run_describe() == 0
from talos_nodes.intake import node, packet
assert node.decode_target_bindings(packet.encode_target(123, True, 4.5)) == (123, True, 4.5)
assert node.resolve_ids(main._default_config()) == (2, 1, 20000)
"""
            proc = subprocess.run([sys.executable, "-I", "-c", code, str(root)],
                                  cwd=root, env=env, capture_output=True,
                                  text=True, timeout=120)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertEqual(json.loads(proc.stdout)["name"], "intake")


class DescribeTest(unittest.TestCase):
    def test_rows_match_node(self):
        self.assertEqual(intake_main.describe_source_rows(),
                         node.describe_sources())

    def test_rows_are_the_single_packet_table(self):
        # Rule 11: one owning table; main and node only reference it.
        expected = packet.describe_sources(node_api.TIMER, node_api.WATCHER,
                                           node_api.SENDER)
        self.assertEqual(intake_main.describe_source_rows(), expected)
        self.assertEqual(node.describe_sources(), expected)

    def test_main_stays_import_light(self):
        # --describe must run without shm/flatbuffers/ctypes clients: only
        # top-level imports count; deferred imports inside run_node are fine.
        import ast
        with open(intake_main.__file__, encoding="utf-8") as fh:
            tree = ast.parse(fh.read())
        top = set()
        for stmt in tree.body:
            if isinstance(stmt, ast.Import):
                top.update(a.asname or a.name.split(".")[0]
                           for a in stmt.names)
            elif isinstance(stmt, ast.ImportFrom):
                top.add((stmt.module or "").split(".")[0])
        for banned in ("rtms", "node", "flatbuffers", "ctypes"):
            self.assertNotIn(banned, top)

    def _run_main(self, *argv):
        resolver = runfiles.Create()
        binary = resolver.Rlocation(os.environ["INTAKE_BINARY"])
        env = dict(os.environ)
        env.update(resolver.EnvVars())
        env.pop("PYTHONPATH", None)
        # Execute the actual packaged launcher from outside the checkout.
        with tempfile.TemporaryDirectory() as cwd:
            return subprocess.run([binary, *argv], cwd=cwd,
                                  capture_output=True, text=True, env=env,
                                  timeout=120)

    def test_deterministic_simulation_and_replay(self):
        reason = _needs_lib()
        if reason is not None:
            self.skipTest(reason)

        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = os.path.join(tmpdir, "intake_sim.tlog")
            
            # 1. Record a deterministic simulation run
            with node_api.Loop.create_simulated(log_path=log_path, stream_name="intake") as sim_loop:
                sim_node = node.IntakeNode(
                    roller_id=2,
                    beam_break_id=1,
                    config_id=7,
                    loop=sim_loop,
                )
                # Inject initial inputs
                state_blob = make_hw_state(config_id=7, boot_id=3, sample_time_us=1000,
                                           roller_id=2, roller_vel=20.0, beam_id=1)
                sim_loop.sim_inject(packet.HW_STATE_TOPIC, state_blob)
                target_blob = packet.encode_target(1000000, True, 25.0)
                sim_loop.sim_inject(packet.TARGET_TOPIC, target_blob)

                # Step simulation
                sim_loop.run_for(60_000_000)  # 60 ms
                sim_dispatches = sim_loop.dispatch_count
                self.assertGreater(sim_dispatches, 0)

            # 2. Replay the recorded log with the exact same node configuration
            with node_api.Loop.create_replay(log_path) as replay_loop:
                replay_node = node.IntakeNode(
                    roller_id=2,
                    beam_break_id=1,
                    config_id=7,
                    loop=replay_loop,
                )
                replay_loop.run()
                self.assertFalse(replay_loop.diverged)
                self.assertTrue(replay_loop.reached_exit)
                # Replay executes the simulated dispatches plus the terminal EXIT dispatch.
                self.assertEqual(replay_loop.dispatch_count, sim_dispatches + 1)

            # 3. Verify divergence detection when replaying with altered node behavior
            with node_api.Loop.create_replay(log_path) as diverged_loop:
                # Different roller_id causes different command encoding, triggering divergence
                diverged_node = node.IntakeNode(
                    roller_id=99,
                    beam_break_id=1,
                    config_id=7,
                    loop=diverged_loop,
                )
                try:
                    diverged_loop.run()
                except node_api.TalosError:
                    pass
                self.assertTrue(diverged_loop.diverged)

    def test_replay_cli_end_to_end(self):
        reason = _needs_lib()
        if reason is not None:
            self.skipTest(reason)

        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = os.path.join(tmpdir, "cli_sim.tlog")
            proc_sim = self._run_main("--sim", "--duration-s", "1", "--log", log_path)
            self.assertEqual(proc_sim.returncode, 0, proc_sim.stderr)
            self.assertTrue(os.path.exists(log_path))

            proc_replay = self._run_main("--replay", log_path)
            self.assertEqual(proc_replay.returncode, 0, proc_replay.stderr)
            self.assertIn("replay: diverged=0 clean_exit=1", proc_replay.stdout)

    def test_describe_shape(self):
        if node_api.find_lib() is None and not os.environ.get(
                "TALOS_NODE_LIB"):
            self.skipTest("libtalos_node.so not built; set TALOS_NODE_LIB")
        proc = self._run_main("--describe")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        doc = json.loads(proc.stdout)
        # Fields the C++ DescribeReader requires (describe.h: Parse).
        for field in ("describe_version", "name", "target", "sources"):
            self.assertIn(field, doc)
        self.assertEqual(doc["describe_version"], 1)
        self.assertEqual(doc["name"], packet.NODE_NAME)
        self.assertEqual(doc["target"], packet.NODE_TARGET)
        kinds = {s["kind"] for s in doc["sources"]}
        self.assertTrue({"TIMER", "WATCHER", "SENDER"} <= kinds)
        by_name = {s["name"]: s for s in doc["sources"]}
        self.assertEqual(by_name[packet.TARGET_TOPIC]["message_bytes"],
                         packet.TARGET_SIZE)
        self.assertEqual(by_name[packet.STATE_TOPIC]["message_bytes"],
                         packet.STATE_SIZE)
        self.assertEqual(by_name[packet.HW_STATE_TOPIC]["message_bytes"],
                         packet.PACKET_SIZE)
        for source in doc["sources"]:
            for field in ("kind", "name", "message_bytes", "external",
                          "optional"):
                self.assertIn(field, source)

    def test_duration_execution(self):
        if node_api.find_lib() is None and not os.environ.get(
                "TALOS_NODE_LIB"):
            self.skipTest("libtalos_node.so not built; set TALOS_NODE_LIB")
        proc = self._run_main("--duration-s", "1", "--sim")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        lines = [line.strip() for line in proc.stdout.splitlines() if line.strip()]
        self.assertTrue(len(lines) > 0)
        summary = json.loads(lines[-1])
        self.assertIn("dispatches", summary)
        self.assertGreater(summary["dispatches"], 0)
        self.assertTrue(summary["sim"])


class RegistryTest(unittest.TestCase):
    def test_register_publish_heartbeat_close(self):
        if node_api.find_lib() is None and not os.environ.get(
                "TALOS_NODE_LIB"):
            self.skipTest("libtalos_node.so not built; set TALOS_NODE_LIB")
        rows = [(node_api.TIMER, "tick", 0, False, False)]
        try:
            reg = node_api.Node("intake_test", "//pkg:intake_test", 999, 0,
                                rows)
        except node_api.TalosError as exc:
            self.skipTest("registry unavailable: %s" % (exc,))
        try:
            reg.heartbeat(1)
            reg.heartbeat(2)
        finally:
            reg.close()
        reg.close()  # double close is safe.


if __name__ == "__main__":
    unittest.main()
