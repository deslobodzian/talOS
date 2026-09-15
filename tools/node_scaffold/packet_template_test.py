"""Scaffold packet template wire-compatibility tests.

Validates that ``templates.render_packet_py`` generates the canonical
hardware wire format (talOS/hardware/packet.h + messages.h/messages.cc),
not the old scaffold-local double-at-offset-0 stand-in.
"""

import importlib.util
import struct
import sys
import types
import unittest
from pathlib import Path

from templates import render_node_py, render_packet_py


def _load_generated(name="scaffold_check", hardware=True):
    """Render packet.py for a scratch node and exec it with a stub node_api."""
    src = render_packet_py(name, hardware)
    stub_runtime = types.ModuleType("talos_runtime")
    stub_api = types.ModuleType("talos_runtime.node_api")
    stub_api.TIMER = 0
    stub_api.WATCHER = 1
    stub_api.SENDER = 2
    stub_runtime.node_api = stub_api
    old_runtime = sys.modules.get("talos_runtime")
    old_api = sys.modules.get("talos_runtime.node_api")
    sys.modules["talos_runtime"] = stub_runtime
    sys.modules["talos_runtime.node_api"] = stub_api
    try:
        mod = types.ModuleType("generated_packet")
        exec(compile(src, "generated_packet.py", "exec"), mod.__dict__)
    finally:
        if old_runtime is not None:
            sys.modules["talos_runtime"] = old_runtime
        else:
            sys.modules.pop("talos_runtime", None)
        if old_api is not None:
            sys.modules["talos_runtime.node_api"] = old_api
        else:
            sys.modules.pop("talos_runtime.node_api", None)
    return mod


def _find_canonical_packet():
    """Locate the canonical intake packet.py without leaving the repo.

    Tries (in order): the bazel-provided dep import, then workspace-relative
    filesystem candidates. Returns the loaded module or None.
    """
    try:
        from talos_nodes.intake import packet as ref  # type: ignore

        return ref
    except Exception:
        pass
    here = Path(__file__).resolve()
    candidates = [
        here.parents[2] / "2026-robot" / "main_processor" / "intake"
        / "talos_nodes" / "intake" / "packet.py",
        Path.cwd() / "2026-robot" / "main_processor" / "intake"
        / "talos_nodes" / "intake" / "packet.py",
    ]
    for path in candidates:
        try:
            if path.is_file():
                spec = importlib.util.spec_from_file_location(
                    "canonical_intake_packet", str(path)
                )
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                return mod
        except Exception:
            continue
    return None


def _cpp_decode_command(buf, max_payload=1400):
    """Independent mirror of hardware::Decode(Command) (messages.cc:84-122).

    Returns the demand of motors[0]. Raises AssertionError when the C++
    decoder would return false.
    """
    assert len(buf) == 1404, len(buf)
    (size,) = struct.unpack_from("<I", buf, 0)
    assert 0 < size <= max_payload, size
    end = 4 + size
    (config_id, boot_id, epoch, observed_us, count, do_count, pwm_count) = (
        struct.unpack_from("<QQQQHHH", buf, 4)
    )
    assert count <= 16 and do_count <= 4 and pwm_count <= 4
    pos = 4 + 38
    demands = []
    for _ in range(count):
        assert pos + 20 <= end
        (mid, mode, slot) = struct.unpack_from("<HBB", buf, pos)
        (demand,) = struct.unpack_from("<d", buf, pos + 4)
        (ff,) = struct.unpack_from("<d", buf, pos + 12)
        assert mode <= 5, mode
        assert slot <= 2, slot
        assert demand == demand and abs(demand) != float("inf"), demand
        assert ff == ff and abs(ff) != float("inf"), ff
        demands.append((mid, mode, slot, demand, ff))
        pos += 20
    for _ in range(do_count):
        assert pos + 3 <= end
        (val,) = struct.unpack_from("<B", buf, pos + 2)
        assert val <= 1
        pos += 3
    for _ in range(pwm_count):
        assert pos + 10 <= end
        (pout,) = struct.unpack_from("<d", buf, pos + 2)
        assert pout == pout and abs(pout) != float("inf")
        pos += 10
    assert pos == end, (pos, end)
    return demands


class PacketTemplateTest(unittest.TestCase):
    def test_sizes_and_alignment(self):
        pkt = _load_generated()
        # Scaffold Target {ulong, double, bool} -> 24B/align 8; State
        # {ulong, double, double, bool, bool} -> 32B/align 8 (flatbuffers
        # struct rules, fields in declaration order).
        self.assertEqual(pkt.TARGET_SIZE, 24)
        self.assertEqual(pkt.TARGET_ALIGN, 8)
        self.assertEqual(pkt.STATE_SIZE, 32)
        self.assertEqual(pkt.STATE_ALIGN, 8)
        # hardware::Packet {uint32 size, uint8 data[1400]} -> 1404B/align 4
        # (packet.h:14-21; kMaxPayloadSize 1400 in protocol/frame.h:16).
        self.assertEqual(pkt.PACKET_SIZE, 1404)
        self.assertEqual(pkt.PACKET_ALIGN, 4)
        self.assertEqual(pkt.MAX_PAYLOAD, 1400)

    def test_target_state_round_trip(self):
        pkt = _load_generated()
        t = pkt.encode_target(123456789, 30.0, True)
        self.assertEqual(len(t), pkt.TARGET_SIZE)
        self.assertEqual(pkt.decode_target(t), (123456789, 30.0, True))
        s = pkt.encode_state(987654321, 29.5, 30.0, False, True)
        self.assertEqual(len(s), pkt.STATE_SIZE)
        self.assertEqual(
            pkt.decode_state(s), (987654321, 29.5, 30.0, False, True)
        )

    def test_30rps_request_accepted_by_cpp_decoder(self):
        pkt = _load_generated()
        req = pkt.encode_command(7, 9, 3, 1_000_000, 2, pkt.MODE_VELOCITY, 0, 30.0)
        self.assertEqual(len(req), pkt.PACKET_SIZE)
        (size,) = struct.unpack_from("<I", req, 0)
        # One-motor Command wire is 38 header + 20 motor = 58 bytes.
        self.assertEqual(size, 58)
        self.assertEqual(req[4 + size :], bytes(pkt.PACKET_SIZE - 4 - size))
        # The real C++ decoder (messages.cc Decode) must accept this.
        demands = _cpp_decode_command(req)
        self.assertEqual(len(demands), 1)
        mid, mode, slot, demand, ff = demands[0]
        self.assertEqual(mid, 2)
        self.assertEqual(mode, pkt.MODE_VELOCITY)
        self.assertEqual(slot, 0)
        self.assertAlmostEqual(demand, 30.0)
        # The template's own decoder agrees with the independent mirror.
        cmd = pkt.decode_command(req)
        self.assertEqual(cmd["motors"][0]["demand"], 30.0)
        self.assertEqual(cmd["motors"][0]["mode"], pkt.MODE_VELOCITY)
        # The old placeholder (double at offset 0, size prefix 0) fails.
        bad = struct.pack("<d", 30.0) + bytes(pkt.PACKET_SIZE - 8)
        with self.assertRaises(AssertionError):
            _cpp_decode_command(bad)

    def test_hw_state_round_trip(self):
        pkt = _load_generated()
        state = pkt.encode_hw_state(7, 9, 3, 2_000_000, 2, 12.5)
        self.assertEqual(len(state), pkt.PACKET_SIZE)
        info = pkt.decode_hw_state_minimal(state, 2)
        self.assertIsNotNone(info)
        self.assertTrue(info["motor_valid"])
        self.assertAlmostEqual(info["motor_velocity_rps"], 12.5)
        self.assertEqual(info["config_id"], 7)
        # Wrong motor id: header decodes but this motor is absent.
        missing = pkt.decode_hw_state_minimal(state, 99)
        self.assertIsNotNone(missing)
        self.assertFalse(missing["motor_valid"])

    def test_byte_compatible_with_canonical_intake(self):
        pkt = _load_generated()
        ref = _find_canonical_packet()
        if ref is None:
            self.skipTest("canonical intake packet.py not available")
        # Packet envelope geometry must match the canonical module.
        self.assertEqual(pkt.PACKET_SIZE, ref.PACKET_SIZE)
        self.assertEqual(pkt.PACKET_ALIGN, ref.PACKET_ALIGN)
        self.assertEqual(pkt.MAX_PAYLOAD, ref.MAX_PAYLOAD)
        # Same Command inputs must produce byte-identical envelopes.
        args = dict(
            config_id=11, boot_id=13, epoch=17, observed_time_us=19,
            motor_id=5, mode=3, slot=0, demand=30.0, feedforward_v=0.0,
        )
        self.assertEqual(pkt.encode_command(**args), ref.encode_command(**args))


class _FakeCtx:
    """Minimal dispatch context: the generated node only reads now_ns."""

    def __init__(self, now_ns):
        self.now_ns = now_ns


class _FakeLoop:
    """Minimal stand-in for node_api.Loop driving a generated node."""

    def __init__(self):
        self.now = 5_000_000_000
        self._timer = None
        self._period_ns = 0
        self._watchers = {}
        self._pending = []
        self.sent = []
        self._next_id = 1
        self.dispatch_count = 0

    def register_timer(self, name, period_ns=0, offset_ns=0, callback=None):
        del name, offset_ns
        timer_id = self._next_id
        self._next_id += 1
        self._timer = callback
        self._period_ns = period_ns
        return timer_id

    def register_watcher(self, topic, size, align, callback):
        del size, align
        watcher_id = self._next_id
        self._next_id += 1
        self._watchers[topic] = callback
        return watcher_id

    def register_sender(self, topic, size, align):
        del topic, size, align
        sender_id = self._next_id
        self._next_id += 1
        return sender_id

    def send(self, sender_id, payload):
        self.sent.append((sender_id, bytes(payload)))

    def now_ns(self):
        return self.now

    def sim_inject(self, topic, payload):
        self._pending.append((topic, bytes(payload)))

    def run_for(self, duration_ns):
        for topic, payload in self._pending:
            callback = self._watchers.get(topic)
            if callback is not None:
                callback(_FakeCtx(self.now), payload)
                self.dispatch_count += 1
        self._pending = []
        steps = 1
        if self._period_ns:
            steps = max(1, duration_ns // self._period_ns)
        for _ in range(steps):
            self.now += self._period_ns or duration_ns
            if self._timer is not None:
                self._timer(_FakeCtx(self.now), b"")
                self.dispatch_count += 1


class _FakeConfig:
    """Minimal config handle: one motor for the scratch subsystem."""

    config_id = 0xABCDEF
    period_us = 5000
    command_timeout_us = 100000

    def get_device_id(self, subsystem, device_name):
        if (subsystem == "scaffold_check"
                and device_name == "scaffold_check_motor"):
            return 9
        return 0

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def _stub_node_api():
    """Gate predicates mirroring talOS/hardware/gate.h for the stub loop."""
    stub = types.ModuleType("talos_runtime.node_api")
    stub.TIMER = 0
    stub.WATCHER = 1
    stub.SENDER = 2

    def accept_state(config_id, prev, nxt):
        if config_id != 0 and nxt["config_id"] != config_id:
            return False
        if (prev is not None and nxt["boot_id"] == prev["boot_id"]
                and nxt["sample_time_us"] <= prev["sample_time_us"]):
            return False
        return True

    def gateway_ready(flags):
        return (flags & 3) == 3 and not (flags & 8)

    def target_fresh_ns(now_ns, issued_ns, enabled, state_rx_ns, timeout_us):
        timeout_ns = timeout_us * 1000
        return (bool(enabled) and issued_ns <= now_ns
                and (now_ns - issued_ns) < timeout_ns
                and (now_ns - state_rx_ns) < timeout_ns)

    def load_config(config_path, simulation=False):
        del config_path, simulation
        return _FakeConfig()

    stub.accept_state = accept_state
    stub.gateway_ready = gateway_ready
    stub.target_fresh_ns = target_fresh_ns
    stub.load_config = load_config
    return stub


def _stub_runner():
    stub = types.ModuleType("talos_runtime.runner")

    class NodeContext:
        def __init__(self, node_name="", config=None, simulation=False,
                     session_id=0, loop=None):
            self.node_name = node_name
            self.config = config
            self.simulation = simulation
            self.session_id = session_id
            self.loop = loop

        @property
        def config_id(self):
            return self.config.config_id if self.config else 0

        @property
        def period_us(self):
            return self.config.period_us if self.config else 5000

        @property
        def command_timeout_us(self):
            return (self.config.command_timeout_us if self.config
                    else 100000)

        def get_device_id(self, device_name, subsystem=None):
            if not self.config:
                raise RuntimeError("No configuration loaded")
            sub = subsystem or self.node_name
            dev_id = self.config.get_device_id(sub, device_name)
            if dev_id == 0:
                raise RuntimeError(
                    "device %r not found for subsystem %r"
                    % (device_name, sub))
            return dev_id

    stub.NodeContext = NodeContext
    return stub


def _load_generated_node(name="scaffold_check"):
    """Render packet.py + node.py for a scratch node and exec with stubs."""
    pkt_src = render_packet_py(name, True)
    node_src = render_node_py(name, True)
    stub_runtime = types.ModuleType("talos_runtime")
    stub_api = _stub_node_api()
    stub_runner = _stub_runner()
    stub_runtime.node_api = stub_api
    stub_runtime.runner = stub_runner
    saved = {}
    for key in ("talos_runtime", "talos_runtime.node_api",
                "talos_runtime.runner", "packet", "node"):
        saved[key] = sys.modules.get(key)
    sys.modules["talos_runtime"] = stub_runtime
    sys.modules["talos_runtime.node_api"] = stub_api
    sys.modules["talos_runtime.runner"] = stub_runner
    try:
        pkt_mod = types.ModuleType("packet")
        exec(compile(pkt_src, "generated_packet.py", "exec"),
             pkt_mod.__dict__)
        sys.modules["packet"] = pkt_mod
        node_mod = types.ModuleType("node")
        exec(compile(node_src, "generated_node.py", "exec"),
             node_mod.__dict__)
        sys.modules["node"] = node_mod
    finally:
        for key, value in saved.items():
            if value is not None:
                sys.modules[key] = value
            else:
                sys.modules.pop(key, None)
    return (pkt_mod, node_mod, stub_runner)


class NodeTemplateTest(unittest.TestCase):
    """Generated hardware node: config IDs plus state/target gates."""

    def _make(self, simulation=False, use_config=True):
        pkt, mod, runner = _load_generated_node()
        loop = _FakeLoop()
        cfg = _FakeConfig() if use_config else None
        ctx = runner.NodeContext(
            node_name="scaffold_check",
            config=cfg,
            simulation=simulation,
            session_id=1,
            loop=loop,
        )
        node_cls = getattr(mod, "ScaffoldCheckNode")
        return (pkt, mod, loop, node_cls.from_context(ctx))

    def test_from_context_resolves_device_id(self):
        pkt, mod, loop, node = self._make()
        self.assertEqual(mod.MOTOR_NAME, "scaffold_check_motor")
        self.assertNotEqual(node.motor_id, 0)
        self.assertEqual(node.motor_id, 9)
        self.assertEqual(node.config_id, _FakeConfig.config_id)
        self.assertEqual(node.period_us, 5000)
        self.assertEqual(node.command_timeout_us, 100000)
        # File-based resolution agrees, and a missing name raises.
        self.assertEqual(mod.resolve_ids("any.toml"), (9, 5000))
        with self.assertRaises(RuntimeError):
            mod.resolve_ids("any.toml", motor="nonexistent_motor")

    def test_missing_device_raises(self):
        pkt, mod, runner = _load_generated_node()
        loop = _FakeLoop()
        ctx = runner.NodeContext(
            node_name="other_subsystem",
            config=_FakeConfig(),
            simulation=False,
            session_id=1,
            loop=None,
        )
        with self.assertRaises(RuntimeError):
            getattr(mod, "ScaffoldCheckNode").from_context(ctx)

    def test_fresh_target_drives_with_gateway_timestamp(self):
        pkt, mod, loop, node = self._make()
        loop.sim_inject(
            pkt.HW_STATE_TOPIC,
            pkt.encode_hw_state(_FakeConfig.config_id, 7, 3, 2_000_000, 9,
                                10.0),
        )
        loop.sim_inject(
            pkt.TARGET_TOPIC,
            pkt.encode_target(
                issued_ns=loop.now_ns(), target_vel=42.0, enabled=True),
        )
        loop.run_for(30_000_000)

        self.assertGreater(node.ticks, 0)
        self.assertTrue(node.have_state)
        self.assertAlmostEqual(node.measured_vel, 10.0)
        requests = [p for sid, p in loop.sent
                    if sid == node.request_sender_id]
        self.assertGreater(len(requests), 0)
        cmd = pkt.decode_command(requests[-1])
        self.assertEqual(cmd["motors"][0]["mode"], pkt.MODE_VELOCITY)
        self.assertEqual(cmd["motors"][0]["id"], 9)
        self.assertAlmostEqual(cmd["motors"][0]["demand"], 42.0)
        # Observed timestamp echoes the gateway sample, not the local clock.
        self.assertEqual(cmd["observed_time_us"], 2_000_000)
        self.assertEqual(cmd["config_id"], _FakeConfig.config_id)
        ts, vel, target_vel, _, enabled = node.latest_state
        self.assertEqual(ts, 2_000_000 * 1000)
        self.assertAlmostEqual(vel, 10.0)
        self.assertAlmostEqual(target_vel, 42.0)
        self.assertTrue(enabled)

    def test_stale_target_coasts_neutral(self):
        pkt, mod, loop, node = self._make()
        loop.sim_inject(
            pkt.HW_STATE_TOPIC,
            pkt.encode_hw_state(_FakeConfig.config_id, 7, 3, 2_000_000, 9,
                                10.0),
        )
        old = loop.now_ns() - (node.command_timeout_us * 1000 + 1)
        loop.sim_inject(
            pkt.TARGET_TOPIC,
            pkt.encode_target(issued_ns=old, target_vel=42.0, enabled=True),
        )
        loop.run_for(30_000_000)

        self.assertTrue(node.have_state)
        cmd = pkt.decode_command(node.last_request_bytes)
        self.assertEqual(cmd["motors"][0]["mode"], pkt.MODE_NEUTRAL)
        self.assertEqual(cmd["motors"][0]["demand"], 0.0)

    def test_disabled_target_coasts_neutral(self):
        pkt, mod, loop, node = self._make()
        loop.sim_inject(
            pkt.HW_STATE_TOPIC,
            pkt.encode_hw_state(_FakeConfig.config_id, 7, 3, 2_000_000, 9,
                                10.0),
        )
        loop.sim_inject(
            pkt.TARGET_TOPIC,
            pkt.encode_target(
                issued_ns=loop.now_ns(), target_vel=42.0, enabled=False),
        )
        loop.run_for(30_000_000)

        self.assertTrue(node.have_state)
        cmd = pkt.decode_command(node.last_request_bytes)
        self.assertEqual(cmd["motors"][0]["mode"], pkt.MODE_NEUTRAL)

    def test_gateway_fault_coasts_neutral(self):
        pkt, mod, loop, node = self._make()
        loop.sim_inject(
            pkt.HW_STATE_TOPIC,
            pkt.encode_hw_state(_FakeConfig.config_id, 7, 3, 2_000_000, 9,
                                10.0, flags=3 | 8),
        )
        loop.sim_inject(
            pkt.TARGET_TOPIC,
            pkt.encode_target(
                issued_ns=loop.now_ns(), target_vel=42.0, enabled=True),
        )
        loop.run_for(30_000_000)

        self.assertTrue(node.have_state)
        cmd = pkt.decode_command(node.last_request_bytes)
        self.assertEqual(cmd["motors"][0]["mode"], pkt.MODE_NEUTRAL)

    def test_wrong_config_and_stale_sample_dropped(self):
        pkt, mod, loop, node = self._make()
        loop.sim_inject(
            pkt.HW_STATE_TOPIC,
            pkt.encode_hw_state(_FakeConfig.config_id + 1, 7, 3, 2_000_000,
                                9, 10.0),
        )
        loop.run_for(10_000_000)
        self.assertFalse(node.have_state)
        loop.sim_inject(
            pkt.HW_STATE_TOPIC,
            pkt.encode_hw_state(_FakeConfig.config_id, 7, 3, 2_000_000, 9,
                                10.0),
        )
        loop.run_for(10_000_000)
        self.assertTrue(node.have_state)
        self.assertAlmostEqual(node.measured_vel, 10.0)
        loop.sim_inject(
            pkt.HW_STATE_TOPIC,
            pkt.encode_hw_state(_FakeConfig.config_id, 7, 3, 1_000_000, 9,
                                20.0),
        )
        loop.run_for(10_000_000)
        self.assertAlmostEqual(node.measured_vel, 10.0)
        self.assertEqual(node.sample_time_us, 2_000_000)


if __name__ == "__main__":
    unittest.main()
