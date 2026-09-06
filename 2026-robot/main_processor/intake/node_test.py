"""Intake tests: policy math without shm + --describe JSON shape.

Owner run (needs the shared lib for --describe, shm never touched):
  bazel test //2026-robot/main_processor/intake:node_test
Direct (build the lib first, or TALOS_NODE_LIB must point at it):
  TALOS_NODE_LIB=/tmp/libtalos_node.so python3 node_test.py
"""

import json
import os
import struct
import subprocess
import sys
import unittest

_HERE = os.path.abspath(os.path.dirname(__file__))
for _extra in (_HERE,
               os.path.normpath(os.path.join(_HERE, "..", "..", "..",
                                              "talOS", "ipc", "python"))):
    if os.path.isdir(_extra) and _extra not in sys.path:
        sys.path.insert(0, _extra)

import packet
import node_api
import node
import main as intake_main


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
        blob = packet.encode_state(10**9, True, 4.0, True)
        self.assertEqual(len(blob), packet.STATE_SIZE)
        issued, enabled, vel, beam = packet.decode_state(blob)
        self.assertEqual((issued, enabled, beam), (10**9, True, True))
        self.assertAlmostEqual(vel, 4.0, places=5)

    def test_state_builder_crosscheck(self):
        import flatbuffers
        from talos.intake.IntakeState import CreateIntakeState
        builder = flatbuffers.Builder(32)
        CreateIntakeState(builder, 777, False, 1.5, True)
        head = builder.head
        built = bytes(builder.Bytes[head:head + packet.STATE_SIZE])
        self.assertEqual(built, packet.encode_state(777, False, 1.5, True))

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


class PolicyTest(unittest.TestCase):
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
        self.assertEqual(decode_command_demand(request)[:4], (2, 3, 0, 21.5))
        issued, enabled, vel, beam = packet.decode_state(published)
        self.assertEqual(issued, 1000 * 1000)
        self.assertTrue(enabled)
        self.assertAlmostEqual(vel, 10.0)
        self.assertFalse(beam)

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
        path = os.path.join(_HERE, "subsystem.toml")
        roller_id, beam_id, period_us = node.resolve_ids(path)
        # Sorted (subsystem, device): intake_beam < roller -> 1, 2.
        self.assertEqual((roller_id, beam_id, period_us), (2, 1, 20000))


class DescribeTest(unittest.TestCase):
    def test_rows_match_node(self):
        self.assertEqual(intake_main.describe_source_rows(),
                         node.describe_sources())

    def test_main_stays_import_light(self):
        # --describe must run without shm/flatbuffers/ctypes clients: only
        # top-level imports count; deferred imports inside run_node are fine.
        import ast
        with open(os.path.join(_HERE, "main.py"), encoding="utf-8") as fh:
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
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join(
            [p for p in sys.path if p]) + os.pathsep + _HERE
        proc = subprocess.run([sys.executable,
                               os.path.join(_HERE, "main.py")] + list(argv),
                              capture_output=True, text=True, env=env,
                              timeout=120)
        return proc

    def test_replay_unsupported(self):
        proc = self._run_main("--replay", "some/path.tlog")
        self.assertEqual(proc.returncode, 2)
        self.assertIn("replay unsupported", proc.stderr)

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
