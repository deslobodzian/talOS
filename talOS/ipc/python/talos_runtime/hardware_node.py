"""Shared plumbing for Python hardware nodes (teaching nodes read this last).

A hardware node turns one /hw/state feed plus one target topic into one
hardware::Command stream plus one state topic. Every Python hardware node
repeats the same four chores, and none of them is the lesson:

* resolving logical device IDs from the TOML config (with a clear error
  when a name is missing),
* folding /hw/state through the C++ accept gate (config match + staleness
  answered by the ``node_api`` ABI over ``talOS/hardware/gate.h`` -- never
  re-spelled here, ARCHITECTURE.md rules 10/11),
* stamping the ``hardware::Command`` header words (config/boot/epoch/
  observed time) that the bridge uses to match commands to state,
* registering the timer/watcher/sender sources on the native loop (or the
  matching ``rtms`` handles in standalone ``poll_once`` mode) in the exact
  order the ``--describe`` table reports.

This module owns those chores. A node file (see
``2026-robot/main_processor/intake/talos_nodes/intake/node.py`` for the
worked example) keeps only its student surface: motor names, behavior
gains/speeds, and the demand mapping. Importing this module never touches
shared memory or the ``.so``; like ``rtms`` it binds on first use.
"""

from talos_runtime import node_api, rtms


def resolve_device_ids(config_path, subsystem, *names):
    """Resolve global logical IDs via the canonical C++ config parser.

    Returns ``(ids, period_us)`` where ``ids`` parallels ``names``.
    Raises ``RuntimeError`` naming the first missing device.
    """
    with node_api.load_config(config_path) as cfg:
        ids = []
        for name in names:
            device_id = cfg.get_device_id(subsystem, name)
            if device_id == 0:
                raise RuntimeError(
                    "device %r not found for subsystem %r in %s"
                    % (name, subsystem, config_path))
            ids.append(device_id)
        period_us = cfg.period_us if cfg.period_us else 0
        return (ids, period_us)


class HwStateLatch:
    """Latest accepted /hw/state plus the header words commands echo back.

    ``decode_fn`` maps raw payload bytes to the minimal state dict (header
    words, validity bits, feedback); ``None`` means malformed. Acceptance
    itself is answered by ``node_api.accept_state`` -- the C++ ground truth
    for the config match and the boot/staleness check.
    """

    def __init__(self, decode_fn):
        self._decode = decode_fn
        self.state = None
        self.rx_ns = 0

    def update(self, payload, config_id, now_ns):
        """Fold one /hw/state payload. Returns True when accepted."""
        state = self._decode(payload)
        if state is None:  # Decode failure -> drop (gate.h never sees it).
            return False
        if not node_api.accept_state(config_id, self.state, state):
            return False
        self.state = state
        self.rx_ns = now_ns
        return True


def build_command(encode_fn, state, motor_id, mode, slot, demand,
                  feedforward_v=0.0):
    """Encode a one-motor command reusing the accepted state's header words.

    ``encode_fn`` is the node's explicit codec (rule 11 keeps codecs spelled
    out in the node's ``packet.py``); the latch supplies ``config_id``,
    ``boot_id``, ``epoch``, and ``sample_time_us`` so behavior files never
    touch them.
    """
    return encode_fn(state["config_id"], state["boot_id"], state["epoch"],
                     state["sample_time_us"], motor_id, mode, slot, demand,
                     feedforward_v)


class HardwareNode:
    """RTMS wiring around a policy. Only this class touches shared memory.

    Subclasses declare the ordered source table (timer first, then watchers,
    then senders -- the same rows ``--describe`` reports) and implement two
    hooks: :meth:`on_watcher` folds input ``payload`` bytes, and
    :meth:`on_timer` returns one payload (or None) per sender. The base
    registers those sources on a native loop, or -- with ``loop=None`` --
    opens the matching ``rtms`` handles for the standalone ``poll_once``
    path. Publisher owns its layout (reclaims stale segments); Subscriber
    never does -- the ``rtms.Publisher``/``rtms.Subscriber`` contract.
    """

    def __init__(self, timer_name, period_us, watchers, senders, loop=None):
        # watchers: [(topic, size, align)]; senders: [(topic, size, align)].
        self._timer_name = timer_name
        self._period_us = period_us
        self._watcher_specs = list(watchers)
        self._sender_specs = list(senders)
        self.loop = None
        self.uses_native_loop = loop is not None
        self._registered_loop = None
        self._sources_registered = False
        self.timer_id = None
        self.watcher_ids = [None] * len(self._watcher_specs)
        self.sender_ids = [None] * len(self._sender_specs)
        self._subs = [None] * len(self._watcher_specs)
        self._pubs = [None] * len(self._sender_specs)
        if loop is not None:
            self.register_sources(loop)
        else:
            for i, (topic, size, align) in enumerate(self._watcher_specs):
                self._subs[i] = rtms.Subscriber(topic, size, align)
            for i, (topic, size, align) in enumerate(self._sender_specs):
                self._pubs[i] = rtms.Publisher(topic, size, align)

    def register_sources(self, loop):
        """Register sources in --describe order. Idempotent per loop."""
        if self._registered_loop is loop:
            return
        self._registered_loop = loop
        self._sources_registered = True
        self.loop = loop
        self.uses_native_loop = True
        self.timer_id = loop.register_timer(
            self._timer_name,
            period_ns=self._period_us * 1000,
            offset_ns=0,
            callback=self._on_timer_native,
        )
        for i, (topic, size, align) in enumerate(self._watcher_specs):
            self.watcher_ids[i] = loop.register_watcher(
                topic, size, align, self._make_watcher_cb(i))
        for i, (topic, size, align) in enumerate(self._sender_specs):
            self.sender_ids[i] = loop.register_sender(topic, size, align)

    def _make_watcher_cb(self, index):
        def _cb(ctx, payload):
            self.on_watcher(index, payload, ctx.now_ns)
        return _cb

    def _on_timer_native(self, ctx, payload):
        del payload
        for sender_id, out in zip(self.sender_ids, self.on_timer(ctx.now_ns)):
            if out is not None and sender_id is not None:
                self.loop.send(sender_id, out)

    # -- Subclass hooks -------------------------------------------------
    def on_watcher(self, index, payload, now_ns):
        """Fold one watcher payload. ``index`` parallels the watcher specs."""
        raise NotImplementedError

    def on_timer(self, now_ns):
        """Return one payload-or-None per sender, in sender-spec order."""
        raise NotImplementedError

    # -- Standalone (loop-free) path ------------------------------------
    def poll_once(self, now_ns):
        """Drain inputs latest-wins, tick, publish. Returns [seq-or-None]."""
        if self._subs[0] is None:
            return [None] * len(self._sender_specs)
        for i, sub in enumerate(self._subs):
            while True:  # Latest wins, like a WATCHER loop dispatch.
                status, payload, _, _ = sub.read_next()
                if status != "ok":
                    break
                self.on_watcher(i, payload, now_ns)
        seqs = []
        for pub, out in zip(self._pubs, self.on_timer(now_ns)):
            seqs.append(pub.write(out) if out is not None else None)
        return seqs

    def close(self):
        for handle in list(self._subs) + list(self._pubs):
            if handle is not None:
                handle.close()
