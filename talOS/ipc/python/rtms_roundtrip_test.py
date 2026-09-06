"""Round-trip tests for the Python RTMS client (talOS/ipc/python).

Levels:
  wire     - payload bytes: struct pack == flatc-builder bytes, and the
             generated accessors decode them (no shared memory needed).
  layout   - frozen header offsets/sizes against C++-probed values (no shm).
  protocol - full publish/subscribe incl. lap recovery against a fake
             bytearray-backed segment (no shm; exercises RTMSQueue logic).
  peer     - live cross-language round trip against the C++ peer binary:
             python->C++ and C++->python asserting identical payload bytes.
             Needs shm + the peer binary; skipped otherwise (see message).

Owner run (needs shm, so not sandbox-safe):
  bazel test //talOS/ipc/python:rtms_roundtrip_test
Direct:
  RTMS_PY_PEER_BIN=/tmp/rtms_peer python3 talOS/ipc/python/rtms_roundtrip_test.py
"""

import os
import struct
import subprocess
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rtms
from rtms import RTMSQueue
import messages


def _peer_binary():
    override = os.environ.get("RTMS_PY_PEER_BIN")
    if override and os.path.isfile(override):
        return override
    srcdir = os.environ.get("TEST_SRCDIR")
    if srcdir:
        workspace = os.environ.get("TEST_WORKSPACE", "_main")
        candidate = os.path.join(srcdir, workspace, "talOS/ipc/python/peer")
        if os.path.isfile(candidate):
            return candidate
    return None


def _shm_available():
    probe = "/py_rtms_probe_%d" % os.getpid()
    try:
        q = RTMSQueue(probe, 8, 4, slots=8,
                      reclaim_mismatched_segment=True)
    except (OSError, RuntimeError):
        return False
    q.close()
    rtms._ShmSegment.unlink(rtms.shm_object_name(probe))
    return True


def _fake_queue(slots=8):
    q = RTMSQueue.__new__(RTMSQueue)
    q.topic = "/fake"
    q.shm_name = "/fake"
    q.message_size = 8
    q.message_alignment = 4
    q.slots = slots
    q.data_offset = rtms.align_up(rtms.HEADER_SIZE, 4)
    q.stride = 8
    q.total_bytes = q.data_offset + q.stride * slots

    class FakeSeg:
        def __init__(self, size):
            self.buf = bytearray(size)

        def close(self):
            pass

    q._seg = FakeSeg(q.total_bytes)
    q._init_header()
    return q


class WireFormatTest(unittest.TestCase):
    def test_builder_bytes_equal_struct_bytes(self):
        for msg_id, value in [(10, 200.0), (0, 0.0), (-3, -1.5), (2**31 - 1, 1e10)]:
            self.assertEqual(messages.encode(msg_id, value),
                             messages.encode_via_builder(msg_id, value))

    def test_decode_round_trip(self):
        for msg_id, value in [(10, 200.0), (23, 23.2), (0, 0.111111)]:
            payload = messages.encode(msg_id, value)
            self.assertEqual(len(payload), 8)
            got_id, got_value = messages.decode(payload)
            self.assertEqual(got_id, msg_id)
            self.assertAlmostEqual(got_value, struct.unpack("<f", struct.pack("<f", value))[0])

    def test_known_bytes(self):
        # id=10 LE @0, 200.0f LE @4.
        self.assertEqual(messages.encode(10, 200.0).hex(), "0a00000000004843")

    def test_message_size_matches_cpp(self):
        # sizeof(IPCMessage::TestMessage)==8, alignof==4 (peer.cc static_asserts).
        self.assertEqual(messages.MESSAGE_SIZE, 8)
        self.assertEqual(messages.MESSAGE_ALIGNMENT, 4)


class LayoutTest(unittest.TestCase):
    def test_frozen_offsets(self):
        # C++-probed: sizeof(RTMSHeader)==640, writer@64, readers@128 stride 64.
        self.assertEqual(rtms.HEADER_SIZE, 640)
        self.assertEqual(rtms.OFF_WRITER_SEQ, 64)
        self.assertEqual(rtms.OFF_READERS, 128)
        self.assertEqual(rtms.READER_STRIDE, 64)
        self.assertEqual(rtms.MAX_SLOTS, 1024)
        self.assertEqual(rtms.MAX_READERS, 8)

    def test_total_bytes_math(self):
        q = _fake_queue(slots=1024)
        self.assertEqual(q.data_offset, 640)
        self.assertEqual(q.stride, 8)
        self.assertEqual(q.total_bytes, 640 + 8 * 1024)

    def test_shm_name_mapping(self):
        self.assertEqual(rtms.shm_object_name("/hw/state"), "/hw.state")
        self.assertEqual(rtms.shm_object_name("/drivetrain/target/teleop"),
                         "/drivetrain.target.teleop")
        # Injective: '_' stays, '/' becomes '.'.
        self.assertNotEqual(rtms.shm_object_name("/hw/state/driver_station"),
                            rtms.shm_object_name("/hw/state_driver/station"))
        with self.assertRaises(ValueError):
            rtms.shm_object_name("/drivetrain/target/teleop_extra_long")


class ProtocolTest(unittest.TestCase):
    def test_sequence_no_drops(self):
        q = _fake_queue()
        rid = q.register_reader()
        self.assertIsNotNone(rid)
        for i in range(20):
            q.write(messages.encode(i, float(i)))
            status, payload, seq, dropped = q.read_next(rid)
            self.assertEqual(status, "ok")
            self.assertEqual((seq, dropped), (i, 0))
            self.assertEqual(payload, messages.encode(i, float(i)))
        self.assertEqual(q.read_next(rid)[0], "empty")

    def test_lap_recovers_to_oldest_live_slot(self):
        # Mirrors rtms test ReadNextLapsUsingDocumentedFormula.
        slots, writes = 8, 8 * 5 + 3
        q = _fake_queue(slots=slots)
        rid = q.register_reader()
        for i in range(writes):
            q.write(messages.encode(i, float(i)))
        status, payload, seq, dropped = q.read_next(rid)
        expected_first = writes - slots + 1
        self.assertEqual((status, seq, dropped), ("ok", expected_first, expected_first))
        self.assertEqual(payload, messages.encode(expected_first, float(expected_first)))
        for expected in range(expected_first + 1, writes):
            status, payload, seq, dropped = q.read_next(rid)
            self.assertEqual((status, seq, dropped), ("ok", expected, 0))
        self.assertEqual(q.read_next(rid)[0], "empty")

    def test_latest_skips_to_newest(self):
        q = _fake_queue()
        rid = q.register_reader()
        for i in range(5):
            q.write(messages.encode(i, float(i)))
        status, payload, seq, dropped = q.read_next(
            rid, rtms.OVERWRITE_OLDEST, rtms.LATEST)
        self.assertEqual((status, seq, dropped), ("ok", 4, 4))
        self.assertEqual(payload, messages.encode(4, 4.0))
        self.assertEqual(q.read_next(rid, rtms.OVERWRITE_OLDEST, rtms.LATEST)[0],
                         "empty")

    def test_empty_inactive_invalid(self):
        q = _fake_queue()
        rid = q.register_reader()
        self.assertEqual(q.read_next(rid)[0], "empty")
        self.assertEqual(q.read_next(rtms.MAX_READERS)[0], "invalid")
        q.release_reader(rid)
        self.assertEqual(q.read_next(rid)[0], "inactive")
        # A released slot is claimable again.
        self.assertEqual(q.register_reader(), rid)


class CppPeerRoundTripTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.peer = _peer_binary()
        if cls.peer is None:
            raise unittest.SkipTest(
                "no C++ peer binary: build it and set RTMS_PY_PEER_BIN, or run "
                "bazel test //talOS/ipc/python:rtms_roundtrip_test")
        if not _shm_available():
            raise unittest.SkipTest(
                "shared memory unavailable in this sandbox (shm_open EPERM); "
                "owner run: bazel test //talOS/ipc/python:rtms_roundtrip_test")

    def _fresh_topic(self, name):
        topic = "/%s_%d" % (name, os.getpid())
        rtms._ShmSegment.unlink(rtms.shm_object_name(topic))
        self.addCleanup(rtms._ShmSegment.unlink, rtms.shm_object_name(topic))
        return topic

    def test_python_publishes_cpp_receives_identical_bytes(self):
        topic = self._fresh_topic("py_pub")
        expected = messages.encode(10, 200.0)
        # Peer subscribes first so its cursor predates the publish.
        proc = subprocess.Popen(
            [self.peer, "--mode=sub", "--topic=" + topic],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            import time
            time.sleep(0.5)  # let the peer create the segment and register
            pub = rtms.Publisher(topic, messages.MESSAGE_SIZE,
                                 messages.MESSAGE_ALIGNMENT)
            try:
                pub.write(expected)
                out, _ = proc.communicate(timeout=15)
            finally:
                pub.close()
        finally:
            if proc.poll() is None:
                proc.kill()
        self.assertEqual(proc.returncode, 0, out)
        self.assertEqual(out.splitlines()[0], expected.hex())

    def test_cpp_publishes_python_receives_identical_bytes(self):
        topic = self._fresh_topic("cpp_pub")
        expected = messages.encode(7, 42.5)
        # Python subscribes first so its cursor predates the publish.
        sub = rtms.Subscriber(topic, messages.MESSAGE_SIZE,
                              messages.MESSAGE_ALIGNMENT)
        try:
            proc = subprocess.run(
                [self.peer, "--mode=pub", "--topic=" + topic,
                 "--id=7", "--value=42.5"],
                capture_output=True, text=True, timeout=15)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            status, payload, seq, dropped = sub.read_next()
        finally:
            sub.close()
        self.assertEqual(proc.stdout.splitlines()[0], expected.hex())
        self.assertEqual(status, "ok")
        self.assertEqual((seq, dropped), (0, 0))
        self.assertEqual(payload, expected)
        self.assertEqual(messages.decode(payload), (7, 42.5))


if __name__ == "__main__":
    unittest.main()
