"""Intake node: the teaching example. Students edit the top, not the bottom.

Read this file in two halves:

* STUDENT SECTION (below): motor names, wheel speeds, and the demand
  mapping -- intake / eject / stop behavior. This is the lesson; nearly
  every student change lands here.
* POLICY + WIRING (below that): the linear-roller policy and its RTMS
  transport. It mirrors 2026-robot/main_processor/shooter/node.h Tick and
  binds every gating answer to the C++ ground truth in
  talOS/hardware/gate.h through the node_api ABI predicates -- never
  re-spelled here (ARCHITECTURE.md rules 10/11). Packet encoding, boot
  IDs, epochs, and transport registration live in the shared helper
  ``talos_runtime.hardware_node`` -- the same helper every future Python
  hardware node will use -- so this file never touches them directly.

What "linear" means: a fresh ``IntakeTarget.roller_velocity_rps`` becomes
a one-motor velocity Command on /hw/request/intake, otherwise the roller
coasts (neutral); the beam_break DIO word plus roller feedback become
/intake/state, which also carries the Studio view fields (requested
speed, measured speed, inhibit reason -- see intake_message.fbs).

Gains live in ``subsystem.toml`` (slot0), not here: all zeros means the
loop is untuned and the wheel will lag its demand -- see the config
comments for what to raise first.
"""

import math

from talos_runtime import hardware_node, node_api
from talos_nodes.intake import behaviors, motor_commands, packet
from talos_nodes.intake import student_policy
from talos_nodes.intake.student_policy import Feedback, Target
from talos.intake.IntakeTarget import IntakeTarget


# ---------------------------------------------------------------------------
# STUDENT SECTION -- edit this block. Everything below it is plumbing.
# ---------------------------------------------------------------------------
# Device names, resolved to global logical IDs from subsystem.toml. Rename
# the motor here (and in the config) and the rest of the file follows.
MOTOR_NAME = "roller"
BEAM_NAME = "intake_beam"

# Which onboard PID slot the velocity demand uses (slot0 in subsystem.toml).
SLOT = 0

# Wheel-speed presets live in student_policy.py (INTAKE_RPS / EJECT_RPS /
# STOP_RPS) -- the single copy. Edit them there; this file reads them
# through command_for's defaults and defines no copies. Positive pulls a
# game piece in, negative spits it back out, zero holds still. A broken
# beam holds stop against further intake, but eject still runs: that is
# how a captured piece gets back out.


def student_command_for(target_vel_rps):
    """Map the arbiter's demand (mechanism rev/s) to the wheel's demand. Start here.

    Beam-blind convenience over ``student_policy.command_for``: a
    positive ask runs at the intake preset, a negative ask at the eject
    preset, and zero holds at stop, so editing the presets in
    ``student_policy.py`` changes what the wheel does. The live tick
    calls ``student_policy`` directly with the beam-break state; this
    wrapper passes clear-beam feedback and stays so earlier call sites
    keep working.

    Whatever this returns must be finite: a NaN/inf demand is never
    commanded (INHIBIT_DEMAND_NONFINITE) -- the wheel coasts instead.
    """
    return student_policy.command_for(Target(float(target_vel_rps)))


# ---------------------------------------------------------------------------
# Plumbing -- students read, rarely edit.
# ---------------------------------------------------------------------------

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

    One line over the single table in packet.py (rule 11): the same rows
    ``main.py`` reports, so the two can never drift apart.
    """
    return packet.describe_sources(node_api.TIMER, node_api.WATCHER,
                                   node_api.SENDER)


class IntakePolicy:
    """Intake policy: no shared memory, no I/O, fully unit-testable.

    Gating answers come from the C++ ground truth through the node_api ABI
    (needs libtalos_node.so, like --describe), never from Python copies.
    Header words (boot IDs, epochs) and the accept gate live in the shared
    ``hardware_node`` helper; the student's demand mapping above is the
    only behavior knob.
    """

    def __init__(self, roller_id, beam_break_id,
                 period_us=packet.PERIOD_US,
                 command_timeout_us=packet.COMMAND_TIMEOUT_US, config_id=0,
                 beam_stable_ticks=1):
        self.roller_id = roller_id
        self.beam_break_id = beam_break_id
        self.period_us = period_us
        self.command_timeout_us = command_timeout_us
        self.config_id = config_id  # 0 = accept any (standalone/sim).
        self._latch = hardware_node.HwStateLatch(self._decode_hw_state)
        self._target = (0, False, 0.0)  # (issued_ns, enabled, velocity_rps).
        # Beam signal chain (behaviors.py): debounce the chattery switch,
        # then count rising edges as captured pieces.
        self._beam = behaviors.Debounce(initial=False,
                                        stable_ticks=beam_stable_ticks)
        self._capture_edge = behaviors.EdgeDetector()
        self.capture_count = 0
        self._ids = {MOTOR_NAME: roller_id}
        # Studio-view snapshot, refreshed every tick that has state.
        self.last_requested_rps = 0.0
        self.last_measured_rps = 0.0
        self.last_inhibit = None  # None until the first tick with state.

    def _decode_hw_state(self, payload):
        return packet.decode_hw_state_minimal(payload, self.roller_id,
                                              self.beam_break_id)

    @property
    def _state(self):
        return self._latch.state

    @property
    def _state_rx_ns(self):
        return self._latch.rx_ns

    def on_hw_state(self, payload, now_ns):
        """Fold one /hw/state payload. Returns True when accepted."""
        # Latch answers config match + staleness via node_api.accept_state
        # (gate.h through the ABI); decode failure drops before the gate.
        return self._latch.update(payload, self.config_id, now_ns)

    def on_target(self, payload, now_ns):
        """Fold one /intake/target payload. Returns True when accepted."""
        del now_ns
        try:
            self._target = decode_target_bindings(payload)  # node.h:81-83.
        except (ValueError, RuntimeError):
            return False
        return True

    def tick(self, now_ns):
        """Tick once. Returns (request_bytes_or_None, state_bytes_or_None).

        Cancellation table -- every timed/conditional outcome states its
        output (the request command; the state row always publishes):

        * drive (gateway ready, target enabled + fresh, roller sample
          valid, finite demand) -> closed-loop velocity at the demand;
        * completion (beam blocked a captured piece) -> velocity at
          stop (intake held; eject still drives -- see student_policy);
        * timeout (stale target) / disable (target or gateway not
          enabled) / invalid feedback / non-finite demand / latched
          fault -> neutral stop ("say stop rather than go silent",
          node.h:136-140), never the last demand held open.
        """
        if self._state is None:
            return (None, None)  # node.h:86.
        state = self._state
        flags = state["flags"]
        gateway_ok = node_api.gateway_ready(flags)  # node.h:91-94, via ABI.
        issued_ns, enabled, target_vel = self._target
        target_fresh = node_api.target_fresh_ns(
            now_ns, issued_ns, enabled, self._state_rx_ns,
            self.command_timeout_us)  # node.h:96-98, via ABI.
        # The student's demand mapping is the only behavior knob: the
        # ask's sign selects the intake / eject / stop presets, and a
        # broken (debounced) beam holds stop against further intake while
        # eject still runs (student_policy). Gating below is unchanged --
        # NaN/inf still coasts, and the node.h check order still names the
        # first failed gate.
        beam_stable = self._beam.update(state["beam_broken"])
        rose, _ = self._capture_edge.update(beam_stable)
        if rose:
            self.capture_count += 1
        beam_broken = state["beam_broken"]  # node.h:112-118 (raw word).
        enabled_out = (flags & (packet.FLAG_CONFIGURED |
                                packet.FLAG_ENABLED)) == (
            packet.FLAG_CONFIGURED | packet.FLAG_ENABLED)  # node.h:153-154.
        # Typed feedback for the student mapping: decoded values only,
        # never the transport dict. Only beam_broken steers the demand;
        # validity/enabled/fault gate below in node.h check order.
        demand = student_policy.command_for(
            Target(target_vel),
            Feedback(beam_broken=beam_stable,
                     beam_valid=bool(state["beam_valid"]),
                     measured_velocity_rps=state["roller_velocity_rps"],
                     velocity_valid=bool(state["roller_valid"]),
                     enabled=bool(enabled_out),
                     fault=bool(flags & packet.FLAG_HARDWARE_FAULT)))
        # Inhibit reasons, first failure wins in node.h check order. Fresh
        # implies enabled (gate.h), so `valid` below equals the original
        # conjunction exactly; the reason only names which term failed.
        if not gateway_ok:
            inhibit = packet.INHIBIT_GATEWAY
        elif not enabled:
            inhibit = packet.INHIBIT_TARGET_DISABLED
        elif not target_fresh:
            inhibit = packet.INHIBIT_TARGET_STALE
        elif not state["roller_valid"]:
            inhibit = packet.INHIBIT_FEEDBACK_INVALID
        elif not math.isfinite(demand):
            inhibit = packet.INHIBIT_DEMAND_NONFINITE
        else:
            inhibit = packet.INHIBIT_DRIVE
        valid = inhibit == packet.INHIBIT_DRIVE  # node.h:120-121, minus beam.
        if valid:  # node.h:132-135: closed-loop velocity on the Talon PID.
            request = motor_commands.velocity(state, self._ids, MOTOR_NAME,
                                              demand, slot=SLOT)
        else:  # node.h:136-140: say stop rather than go silent.
            request = motor_commands.stop(state, self._ids, MOTOR_NAME)
        # Requested echoes the live arbiter ask (0 when there is none or it
        # is non-finite); measured is the wheel's own feedback. Studio shows
        # both plus inhibit, so "asked 30, spinning 0, stale target" reads
        # off one row.
        requested = (demand if (enabled and target_fresh
                                and math.isfinite(demand)) else 0.0)
        # Request bytes above come from motor_commands (node.h:124-146;
        # feedforward 0: linear policy); the header words ride along from
        # the accepted state via the shared helper's convention.
        # (beam_broken / enabled_out were decoded above for the Feedback.)
        published = packet.encode_state(
            state["sample_time_us"] * 1000,  # node.h:155.
            enabled_out, state["roller_velocity_rps"], beam_broken,
            requested, inhibit)
        self.last_requested_rps = requested
        self.last_measured_rps = state["roller_velocity_rps"]
        self.last_inhibit = inhibit
        return (request, published)

    def debug_snapshot(self):
        """Student introspection: what the last tick saw, without shm."""
        return {
            "requested_rps": self.last_requested_rps,
            "measured_rps": self.last_measured_rps,
            "inhibit": self.last_inhibit,
            "beam_broken": (self._state["beam_broken"]
                            if self._state is not None else None),
            "captures": self.capture_count,
        }


def resolve_ids(config_path, subsystem="intake", roller="roller",
                beam_break="intake_beam"):
    """Resolve global logical IDs using the canonical C++ config parser via node_api.

    Returns (roller_id, beam_break_id, period_us); raises RuntimeError on missing devices.
    """
    (ids, period_us) = hardware_node.resolve_device_ids(
        config_path, subsystem, roller, beam_break)
    if not period_us:
        period_us = packet.PERIOD_US
    return (ids[0], ids[1], period_us)


class IntakeNode(hardware_node.HardwareNode):
    """RTMS wiring around IntakePolicy. Only this class touches shm.

    Transport registration, the standalone poll path, and teardown all come
    from the shared helper; this subclass only names this node's sources
    and routes them into the policy.
    """

    HW_WATCHER = 0
    TARGET_WATCHER = 1
    REQUEST_SENDER = 0
    STATE_SENDER = 1

    @classmethod
    def from_context(cls, ctx):
        if ctx.config is not None:
            roller_id = ctx.get_device_id(MOTOR_NAME)
            beam_id = ctx.get_device_id(BEAM_NAME)
            period_us = ctx.period_us
            timeout_us = ctx.command_timeout_us
            config_id = ctx.config_id
        else:
            roller_id = 2
            beam_id = 1
            period_us = packet.PERIOD_US
            timeout_us = packet.COMMAND_TIMEOUT_US
            config_id = 0
        return cls(roller_id=roller_id,
                   beam_break_id=beam_id,
                   period_us=period_us,
                   command_timeout_us=timeout_us,
                   config_id=config_id,
                   loop=getattr(ctx, "loop", None))

    def __init__(self, roller_id, beam_break_id,
                 period_us=packet.PERIOD_US,
                 command_timeout_us=packet.COMMAND_TIMEOUT_US, config_id=0,
                 loop=None):
        self.policy = IntakePolicy(roller_id, beam_break_id, period_us,
                                   command_timeout_us, config_id)
        super().__init__(
            timer_name="intake",
            period_us=period_us,
            watchers=[(packet.HW_STATE_TOPIC, packet.PACKET_SIZE,
                       packet.PACKET_ALIGN),
                      (packet.TARGET_TOPIC, packet.TARGET_SIZE,
                       packet.TARGET_ALIGN)],
            senders=[(packet.REQUEST_TOPIC, packet.PACKET_SIZE,
                      packet.PACKET_ALIGN),
                     (packet.STATE_TOPIC, packet.STATE_SIZE,
                      packet.STATE_ALIGN)],
            loop=loop)

    # Historic attribute spellings (pre-helper layout); kept so readers of
    # this node keep their bearings.
    @property
    def hw_sub_id(self):
        return self.watcher_ids[self.HW_WATCHER]

    @property
    def target_sub_id(self):
        return self.watcher_ids[self.TARGET_WATCHER]

    @property
    def request_sender_id(self):
        return self.sender_ids[self.REQUEST_SENDER]

    @property
    def state_sender_id(self):
        return self.sender_ids[self.STATE_SENDER]

    @property
    def hw_sub(self):
        return self._subs[self.HW_WATCHER]

    @property
    def target_sub(self):
        return self._subs[self.TARGET_WATCHER]

    @property
    def request_pub(self):
        return self._pubs[self.REQUEST_SENDER]

    @property
    def state_pub(self):
        return self._pubs[self.STATE_SENDER]

    def register_sources(self, loop):
        """Register sources in the exact order defined by describe_sources."""
        super().register_sources(loop)

    def on_watcher(self, index, payload, now_ns):
        if index == self.HW_WATCHER:
            self.policy.on_hw_state(payload, now_ns)
        elif index == self.TARGET_WATCHER:
            self.policy.on_target(payload, now_ns)

    def on_timer(self, now_ns):
        request, published = self.policy.tick(now_ns)
        return [request, published]

    def poll_once(self, now_ns):
        """Drain inputs, tick, publish. Returns (request_seq, state_seq)."""
        seqs = super().poll_once(now_ns)
        return (seqs[self.REQUEST_SENDER], seqs[self.STATE_SENDER])
