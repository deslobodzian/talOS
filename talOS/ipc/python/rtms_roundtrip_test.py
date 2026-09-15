"""Round-trip tests for the Python RTMS client (talOS/ipc/python).

Levels:
  wire     - payload bytes: struct pack == flatc-builder bytes, and the
             generated accessors decode them (no shared memory needed).
  layout   - frozen geometry read from the library via talos_rtms_layout
             against the frozen contract values (needs the built .so,
             no shm).
  protocol - full publish/subscribe incl. lap recovery against a local
             counter/dict double (no shm; exercises the protocol formulas,
             not the transport -- see the wrapper rule in ARCHITECTURE.md).
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

from talos_runtime import rtms, node_api, runner
from talos_runtime.node_api import RTMSQueue
import messages


class FakeQueue:
    """Test-only protocol double (no shared memory, no transport).

    Lives here -- not in rtms.py -- per the wrapper rule: it exercises the
    publish/subscribe formulas (sequence, lap recovery, LATEST) without
    reimplementing anything fundamental. Mirrors the documented
    OVERWRITE_OLDEST semantics; DROP_NEWEST disables lap recovery.
    """

    def __init__(self, slots=8, message_size=8):
        self.slots = slots
        self.message_size = message_size
        self._wseq = 0
        self._store = {}
        self._readers = {}

    def register_reader(self):
        for i in range(rtms.MAX_READERS):
            if self._readers.get(i, (None, False))[1] is not True:
                self._readers[i] = [self._wseq, True]
                return i
        return None

    def release_reader(self, reader_id):
        if reader_id in self._readers:
            self._readers[reader_id][1] = False

    def write(self, payload):
        data = bytes(payload)
        if len(data) > self.message_size:
            raise ValueError(
                "Cannot write message larger than slot size: %d > %d"
                % (len(data), self.message_size))
        seq = self._wseq
        self._store[seq] = data + b"\x00" * (self.message_size - len(data))
        self._wseq = seq + 1
        return seq

    def read_next(self, reader_id,
                  overflow_policy=rtms.OVERWRITE_OLDEST,
                  read_mode=rtms.SEQUENCE):
        if reader_id >= rtms.MAX_READERS:
            return ("invalid", None, 0, 0)
        cursor, active = self._readers.get(reader_id, (0, False))
        if not active:
            return ("inactive", None, 0, 0)
        writer = self._wseq
        if cursor >= writer:
            return ("empty", None, 0, 0)
        seq = cursor
        if overflow_policy != rtms.DROP_NEWEST and writer - seq >= self.slots:
            seq = writer - self.slots + 1
        if read_mode == rtms.LATEST:
            seq = writer - 1
        payload = self._store.get(seq)
        if payload is None:  # pragma: no cover - corrupt emulation
            return ("torn", None, 0, 0)
        self._readers[reader_id][0] = seq + 1
        return ("ok", bytes(payload), seq, seq - cursor)


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
    return FakeQueue(slots=slots, message_size=8)


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
        data_offset = rtms.align_up(rtms.HEADER_SIZE, 4)
        stride, slots = 8, 1024
        self.assertEqual(data_offset, 640)
        self.assertEqual(data_offset + stride * slots, 640 + 8 * 1024)

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


class ConfigBindingTest(unittest.TestCase):
    def test_abi_version_is_6(self):
        self.assertEqual(node_api.TALOS_NODE_ABI_VERSION, 6)
        self.assertEqual(node_api.abi_version(), 6)

    def test_bad_path_raises_talos_error(self):
        with self.assertRaises(node_api.TalosError):
            node_api.load_config("/nonexistent/file.toml")

    def test_config_lifecycle_and_queries(self):
        import tempfile
        toml_content = b"""
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.drive.motors.front_left]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.drive.sensors.pigeon]
type = "Pigeon2"
bus = "rio"
can_id = 2

[subsystems.intake.digital_inputs.limit_switch]
dio = 3

[subsystems.intake.digital_outputs.solenoid]
dio = 4

[subsystems.intake.analog_inputs.pressure]
analog = 0

[subsystems.intake.encoders.quad]
type = "QuadratureEncoder"
dio_a = 5
dio_b = 6

[subsystems.intake.pwm_outputs.servo]
channel = 1
"""
        with tempfile.NamedTemporaryFile(suffix=".toml", delete=False) as f:
            f.write(toml_content)
            temp_path = f.name

        try:
            with node_api.load_config(temp_path, simulation=False) as cfg:
                self.assertEqual(cfg.period_us, 5000)
                self.assertEqual(cfg.command_timeout_us, 100000)
                self.assertGreater(cfg.config_id, 0)

                # Test get_device_id across device kinds
                self.assertGreater(cfg.get_device_id("drive", "front_left"), 0)
                self.assertGreater(cfg.get_device_id("drive", "pigeon"), 0)
                self.assertGreater(cfg.get_device_id("intake", "limit_switch"), 0)
                self.assertGreater(cfg.get_device_id("intake", "solenoid"), 0)
                self.assertGreater(cfg.get_device_id("intake", "pressure"), 0)
                self.assertGreater(cfg.get_device_id("intake", "quad"), 0)
                self.assertGreater(cfg.get_device_id("intake", "servo"), 0)

                # Missing device returns 0
                self.assertEqual(cfg.get_device_id("drive", "nonexistent"), 0)
                self.assertEqual(cfg.get_device_id("nonexistent", "front_left"), 0)

                live_id = cfg.config_id

            # Manual close and closed check
            cfg2 = node_api.load_config(temp_path)
            cfg2.close()
            with self.assertRaises(node_api.TalosError):
                cfg2.get_device_id("drive", "front_left")

            # Simulation flag changes config ID
            with node_api.load_config(temp_path, simulation=True) as sim_cfg:
                self.assertNotEqual(sim_cfg.config_id, live_id)
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)


class NativeLoopTest(unittest.TestCase):
    def test_simulated_loop_timer_and_sender(self):
        with node_api.Loop.create_simulated() as loop:
            sender_id = loop.register_sender("/test/sim_loop/cmd", 8, 4)
            self.assertNotEqual(sender_id, 0xFFFF)
            ticks = []

            def on_timer(ctx):
                ticks.append(ctx.dispatch_index)
                payload = struct.pack("<Q", 0x12345678)
                loop.send(sender_id, payload)

            timer_id = loop.register_timer("timer", 5_000_000, 5_000_000, on_timer)
            self.assertNotEqual(timer_id, 0xFFFF)
            loop.run_for(12_000_000)
            self.assertGreater(loop.dispatch_count, 0)
            self.assertEqual(len(ticks), 2)

    def test_simulated_loop_sim_inject(self):
        with node_api.Loop.create_simulated() as loop:
            received = []

            def on_msg(ctx, data):
                self.assertIsInstance(ctx, node_api.DispatchContext)
                self.assertEqual(ctx.kind, node_api.WATCHER)
                self.assertGreaterEqual(ctx.now_ns, ctx.event_time_ns)
                self.assertEqual(ctx.latency_ns, ctx.now_ns - ctx.event_time_ns)
                received.append(data)

            watcher_id = loop.register_watcher("/test/sim/sensor", 8, 4, on_msg)
            self.assertNotEqual(watcher_id, 0xFFFF)
            test_payload = b"\x01\x02\x03\x04\x05\x06\x07\x08"
            loop.sim_inject("/test/sim/sensor", test_payload)
            loop.run_for(10_000_000)
            self.assertEqual(len(received), 1)
            self.assertEqual(received[0], test_payload)

    def test_replay_loop_determinism(self):
        import tempfile
        with tempfile.NamedTemporaryFile(suffix=".tlog", delete=False) as f:
            tlog_path = f.name
        try:
            # 1. Record simulated loop run
            with node_api.Loop.create_simulated(log_path=tlog_path, stream_name="sim_stream") as sim_loop:
                sender_id = sim_loop.register_sender("/test/sim/cmd", 4, 4)
                self.assertNotEqual(sender_id, 0xFFFF)

                def on_sensor(ctx, data):
                    (val,) = struct.unpack("<I", data)
                    out = struct.pack("<I", val * 2)
                    sim_loop.send(sender_id, out)

                sim_loop.register_watcher("/test/sim/sensor", 4, 4, on_sensor)
                sim_loop.register_timer("sim_timer", 10_000_000, 10_000_000, lambda ctx: None)

                sim_loop.sim_inject("/test/sim/sensor", struct.pack("<I", 21))
                sim_loop.run_for(25_000_000)
                sim_loop.sim_inject("/test/sim/sensor", struct.pack("<I", 50))
                sim_loop.run_for(10_000_000)

            self.assertTrue(os.path.exists(tlog_path))
            self.assertGreater(os.path.getsize(tlog_path), 0)

            # 2. Replay loop execution
            with node_api.Loop.create_replay(tlog_path) as replay_loop:
                sender_id = replay_loop.register_sender("/test/sim/cmd", 4, 4)
                self.assertNotEqual(sender_id, 0xFFFF)
                replay_sensor_vals = []

                def replay_on_sensor(ctx, data):
                    (val,) = struct.unpack("<I", data)
                    replay_sensor_vals.append(val)
                    out = struct.pack("<I", val * 2)
                    replay_loop.send(sender_id, out)

                replay_loop.register_watcher("/test/sim/sensor", 4, 4, replay_on_sensor)
                replay_loop.register_timer("sim_timer", 10_000_000, 10_000_000, lambda ctx: None)

                replay_loop.run()
                self.assertFalse(replay_loop.diverged)
                self.assertTrue(replay_loop.reached_exit)
                self.assertEqual(replay_sensor_vals, [21, 50])
        finally:
            if os.path.exists(tlog_path):
                os.unlink(tlog_path)

    def test_exception_handling(self):
        with node_api.Loop.create_simulated() as loop:
            def failing_callback(ctx):
                raise RuntimeError("clean callback exception")

            loop.register_timer("fail_timer", 5_000_000, 5_000_000, failing_callback)
            with self.assertRaises(RuntimeError) as cm:
                loop.run_for(10_000_000)
            self.assertIn("clean callback exception", str(cm.exception))

    def test_arm_disarm_timer(self):
        with node_api.Loop.create_simulated() as loop:
            ticks = []
            timer_id = loop.register_timer("t", 0, 0, lambda ctx: ticks.append(1))
            loop.run_for(10_000_000)
            self.assertEqual(len(ticks), 0)
            loop.arm_timer(timer_id, 5_000_000, 5_000_000)
            loop.run_for(12_000_000)
            self.assertEqual(len(ticks), 2)
            loop.disarm_timer(timer_id)
            loop.run_for(20_000_000)
            self.assertEqual(len(ticks), 2)

    def test_scheduling_overhead_measurement(self):
        import time
        with node_api.Loop.create_simulated() as loop:
            dispatches = 0
            latencies = []

            def on_timer(ctx):
                nonlocal dispatches
                dispatches += 1
                latencies.append(ctx.latency_ns)

            loop.register_timer("perf_timer", 1_000_000, 1_000_000, on_timer)
            # Warmup
            loop.run_for(10_000_000)
            self.assertEqual(dispatches, 10)
            self.assertTrue(all(lat == 0 for lat in latencies))

            # Measure dispatch overhead
            t0 = time.perf_counter_ns()
            loop.run_for(100_000_000)  # 100 dispatches
            t1 = time.perf_counter_ns()
            elapsed_ns = t1 - t0
            avg_overhead_us = (elapsed_ns / 100) / 1000.0
            # Ensure C++ to Python ctypes dispatch overhead is reasonable (< 500 us per dispatch on mac)
            self.assertLess(avg_overhead_us, 500.0)


class SampleNativeNode:
    """Test node for runner native loop execution and replay."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.ticks = 0
        self.sender_id = None

    def register_sources(self, loop):
        self.sender_id = loop.register_sender("/test/runner/cmd", 4, 4)
        loop.register_timer("runner_timer", 5_000_000, 5_000_000, self.on_timer)

    def on_timer(self, ctx):
        self.ticks += 1
        self.ctx.loop.send(self.sender_id, struct.pack("<I", self.ticks))
        if self.ticks >= 3:
            self.ctx.loop.exit()


class RunnerNativeLoopTest(unittest.TestCase):
    def test_runner_native_loop_and_replay(self):
        import tempfile
        with tempfile.NamedTemporaryFile(suffix=".tlog", delete=False) as f:
            tlog_path = f.name
        try:
            # 1. Run under simulation with logging
            sim_rc = runner.run_node_main(
                node_name="test_runner_node",
                node_target="//test:target",
                entry_point="rtms_roundtrip_test:SampleNativeNode",
                argv=["--sim", "--log", tlog_path],
            )
            self.assertEqual(sim_rc, 0)
            self.assertTrue(os.path.exists(tlog_path))
            self.assertGreater(os.path.getsize(tlog_path), 0)

            # 2. Run under replay
            replay_rc = runner.run_node_main(
                node_name="test_runner_node",
                node_target="//test:target",
                entry_point="rtms_roundtrip_test:SampleNativeNode",
                argv=["--replay", tlog_path],
            )
            self.assertEqual(replay_rc, 0)
        finally:
            if os.path.exists(tlog_path):
                os.unlink(tlog_path)


if __name__ == "__main__":
    unittest.main()
