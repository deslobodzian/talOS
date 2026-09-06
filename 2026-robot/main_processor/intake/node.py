"""Intake node: linear roller policy with RTMS transport.

Policy mirrors 2026-robot/main_processor/shooter/node.h Tick (lines 85-159):
a fresh IntakeTarget.roller_velocity_rps becomes a one-motor velocity
Command on /hw/request/intake, otherwise the roller coasts (neutral); the
beam_break DIO word plus roller feedback become /intake/state. "Linear"
means the demand passes straight through: no curves, no feedforward.

Transport uses ONLY the wrapper contract: rtms.Publisher / rtms.Subscriber
from talOS/ipc/python (never reimplement the shm layout) plus
node_api.describe_json for the --describe envelope (kind ints 1..4).
Target payloads are decoded through the flatc --python bindings generated
from the same .fbs as C++, exactly like talOS/ipc/python/messages.py.
"""

import math
import os
import sys

try:
    import rtms
except ImportError:  # direct `python3 node.py` run: point at the wrapper pkg.
    _HERE = os.path.abspath(os.path.dirname(__file__))
    for _cand in (
            os.path.join(_HERE, "..", "..", "..", "talOS", "ipc", "python"),
            os.path.join(_HERE, "talOS", "ipc", "python")):
        if os.path.isfile(os.path.join(_cand, "rtms.py")):
            sys.path.insert(0, os.path.normpath(_cand))
            break
    import rtms

import packet
import node_api

try:
    from talos.intake.IntakeTarget import IntakeTarget
except ImportError:  # same fallback for direct runs.
    _HERE = os.path.abspath(os.path.dirname(__file__))
    if _HERE not in sys.path:
        sys.path.insert(0, _HERE)
    from talos.intake.IntakeTarget import IntakeTarget

try:
    import tomllib
except ImportError:  # python < 3.11 direct runs.
    import tomli as tomllib


def decode_target_bindings(payload):
    """Decode an IntakeTarget via the generated accessors (messages.py style)."""
    if len(payload) != packet.TARGET_SIZE:
        raise ValueError("IntakeTarget payload must be %d bytes, got %d"
                         % (packet.TARGET_SIZE, len(payload)))
    msg = IntakeTarget()
    msg.Init(bytes(payload), 0)
    return (msg.IssuedNs(), bool(msg.Enabled()),
            float(msg.RollerVelocityRps()))


def describe_sources():
    """Source rows for node_api.describe_json: (kind, topic, bytes[, flags]).

    Mirrors the ShooterNode constructor (shooter/node.h:51-56): one timer,
    watchers for /hw/state + target, senders for the hw request + state.
    message_bytes are sizeof(T) as registered via handles.h:133-150.
    """
    return [
        (node_api.TIMER, "intake", 0, False, False),
        (node_api.WATCHER, packet.HW_STATE_TOPIC, packet.PACKET_SIZE,
         False, False),
        (node_api.WATCHER, packet.TARGET_TOPIC, packet.TARGET_SIZE,
         False, False),
        (node_api.SENDER, packet.REQUEST_TOPIC, packet.PACKET_SIZE,
         False, False),
        (node_api.SENDER, packet.STATE_TOPIC, packet.STATE_SIZE,
         False, False),
    ]


class IntakePolicy:
    """Pure intake policy: no shared memory, no I/O. Fully unit-testable."""

    def __init__(self, roller_id, beam_break_id,
                 period_us=packet.PERIOD_US,
                 command_timeout_us=packet.COMMAND_TIMEOUT_US, config_id=0):
        self.roller_id = roller_id
        self.beam_break_id = beam_break_id
        self.period_us = period_us
        self.command_timeout_us = command_timeout_us
        self.config_id = config_id  # 0 = accept any (standalone/sim).
        self._state = None
        self._state_rx_ns = 0
        self._target = (0, False, 0.0)  # (issued_ns, enabled, velocity_rps).

    def on_hw_state(self, payload, now_ns):
        """Fold one /hw/state payload. Returns True when accepted."""
        state = packet.decode_hw_state_minimal(payload, self.roller_id,
                                               self.beam_break_id)
        if state is None:  # Decode failure -> drop (node.h:70).
            return False
        if self.config_id != 0 and state["config_id"] != self.config_id:
            return False  # node.h:71.
        prev = self._state
        if (prev is not None and state["boot_id"] == prev["boot_id"] and
                state["sample_time_us"] <= prev["sample_time_us"]):
            return False  # stale/duplicate sample (node.h:72-75).
        self._state = state
        self._state_rx_ns = now_ns
        return True

    def on_target(self, payload, now_ns):
        """Fold one /intake/target payload. Returns True when accepted."""
        del now_ns
        try:
            self._target = decode_target_bindings(payload)  # node.h:81-83.
        except (ValueError, RuntimeError):
            return False
        return True

    def tick(self, now_ns):
        """Returns (request_bytes_or_None, state_bytes_or_None)."""
        if self._state is None:
            return (None, None)  # node.h:86.
        state = self._state
        flags = state["flags"]
        gateway_ok = (
            (flags & (packet.FLAG_CONFIGURED | packet.FLAG_ENABLED)) ==
            (packet.FLAG_CONFIGURED | packet.FLAG_ENABLED) and
            not (flags & packet.FLAG_HARDWARE_FAULT))  # node.h:91-94.
        issued_ns, enabled, target_vel = self._target
        timeout_ns = self.command_timeout_us * 1000
        target_fresh = (enabled and issued_ns <= now_ns and
                        now_ns - issued_ns < timeout_ns and
                        now_ns - self._state_rx_ns < timeout_ns)  # node.h:96-98.
        valid = (gateway_ok and target_fresh and state["roller_valid"] and
                 math.isfinite(target_vel))  # node.h:120-121, minus beam.
        if valid:  # node.h:132-135.
            mode, demand = packet.MODE_VELOCITY, target_vel
        else:  # node.h:136-140.
            mode, demand = packet.MODE_NEUTRAL, 0.0
        request = packet.encode_command(
            state["config_id"], state["boot_id"], state["epoch"],
            state["sample_time_us"], self.roller_id, mode, 0, demand,
            0.0)  # node.h:124-146; feedforward 0: linear policy.
        beam_broken = state["beam_broken"]  # node.h:112-118.
        enabled_out = (flags & (packet.FLAG_CONFIGURED |
                                packet.FLAG_ENABLED)) == (
            packet.FLAG_CONFIGURED | packet.FLAG_ENABLED)  # node.h:153-154.
        published = packet.encode_state(
            state["sample_time_us"] * 1000,  # node.h:155.
            enabled_out, state["roller_velocity_rps"], beam_broken)
        return (request, published)


# Device-kind tables read by the C++ parser (config_parser.h:224-564); every
# device in every listed subsystem shares one numbering counter.
_DEVICE_TABLES = ("motors", "sensors", "digital_inputs", "digital_outputs",
                  "analog_inputs", "encoders", "pwm_outputs", "pwms")


def _devices_of(sub_toml):
    names = []
    for table in _DEVICE_TABLES:
        entries = sub_toml.get(table, {})
        if isinstance(entries, dict):
            names.extend(entries.keys())
    return names


def resolve_ids(config_path, subsystem="intake", roller="roller",
                beam_break="intake_beam"):
    """Resolve global logical IDs by mirroring the C++ parser.

    config_parser.h:629-633 sorts every device by (subsystem, device) and
    numbers from 1 in one counter. Accepts either the robot manifest
    ([[subsystems]] entries with name/path, pathed relative to the config)
    or a standalone subsystem.toml ([subsystem] name + device tables), in
    which case numbering covers that file alone. Returns
    (roller_id, beam_break_id, period_us); raises on missing devices.
    """
    with open(config_path, "rb") as handle:
        config = tomllib.load(handle)
    base = os.path.dirname(os.path.abspath(config_path))
    all_devices = []  # (subsystem, device).
    period_us = packet.PERIOD_US
    manifest = config.get("subsystems")
    if isinstance(manifest, list):  # robot.toml shape.
        for entry in manifest:
            sub_path = os.path.join(base, entry["path"])
            with open(sub_path, "rb") as handle:
                sub = tomllib.load(handle)
            for dev in _devices_of(sub):
                all_devices.append((entry["name"], dev))
            if entry["name"] == subsystem:
                period_us = sub.get("subsystem", {}).get("period_us",
                                                         period_us)
    else:  # standalone subsystem.toml shape.
        subsystem = config.get("subsystem", {}).get("name", subsystem)
        period_us = config.get("subsystem", {}).get("period_us", period_us)
        for dev in _devices_of(config):
            all_devices.append((subsystem, dev))
    all_devices.sort()  # config_parser.h:630-633.
    ids = {pair: i + 1 for i, pair in enumerate(all_devices)}
    try:
        roller_id = ids[(subsystem, roller)]
        beam_id = ids[(subsystem, beam_break)]
    except KeyError as exc:
        raise RuntimeError("device %r not found for subsystem %r in %s" %
                           (exc.args[0][1], subsystem, config_path))
    return (roller_id, beam_id, period_us)


class IntakeNode:
    """RTMS wiring around IntakePolicy. Only this class touches shm."""

    def __init__(self, roller_id, beam_break_id,
                 period_us=packet.PERIOD_US,
                 command_timeout_us=packet.COMMAND_TIMEOUT_US, config_id=0):
        self.policy = IntakePolicy(roller_id, beam_break_id, period_us,
                                   command_timeout_us, config_id)
        # Publisher owns its layout (reclaims stale segments); Subscriber
        # never does -- the rtms.py contract (RTMSQueue docstring).
        self.hw_sub = rtms.Subscriber(packet.HW_STATE_TOPIC,
                                      packet.PACKET_SIZE, packet.PACKET_ALIGN)
        self.target_sub = rtms.Subscriber(packet.TARGET_TOPIC,
                                          packet.TARGET_SIZE,
                                          packet.TARGET_ALIGN)
        self.request_pub = rtms.Publisher(packet.REQUEST_TOPIC,
                                          packet.PACKET_SIZE,
                                          packet.PACKET_ALIGN)
        self.state_pub = rtms.Publisher(packet.STATE_TOPIC, packet.STATE_SIZE,
                                        packet.STATE_ALIGN)

    def poll_once(self, now_ns):
        """Drain inputs, tick, publish. Returns (request_seq, state_seq)."""
        while True:  # latest-wins drain, like a WATCHER loop dispatch.
            status, payload, _, _ = self.hw_sub.read_next()
            if status != "ok":
                break
            self.policy.on_hw_state(payload, now_ns)
        while True:
            status, payload, _, _ = self.target_sub.read_next()
            if status != "ok":
                break
            self.policy.on_target(payload, now_ns)
        request, published = self.policy.tick(now_ns)
        if request is not None:
            req_seq = self.request_pub.write(request)
        else:
            req_seq = None
        if published is not None:
            st_seq = self.state_pub.write(published)
        else:
            st_seq = None
        return (req_seq, st_seq)

    def close(self):
        for handle in (self.hw_sub, self.target_sub, self.request_pub,
                       self.state_pub):
            handle.close()
