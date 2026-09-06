"""Intake node entry point. Mirrors the talOS/process/node_main.h flag set.

--describe is import-light on purpose (stdlib + json + sizes only): it must
run from the launcher probe without touching shared memory, hardware, or
the network, exactly like the C++ describe branch (node_main.h: describe
builds the node on a simulated loop and prints its manifest). In particular
--describe never imports node.py (which pulls in rtms + flatbuffers); the
source table below mirrors node.describe_sources() and node_test.py asserts
the two agree.
"""

import argparse
import json
import os
import signal
import sys
import time

import packet
import node_api


def _default_config():
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(here, "subsystem.toml")


def parse_args(argv):
    parser = argparse.ArgumentParser(prog="node")
    parser.add_argument("--describe", action="store_true")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--session-id", default="0")
    parser.add_argument("--log", default="")
    parser.add_argument("--config", default=_default_config())
    parser.add_argument("--replay", default="")
    parser.add_argument("--duration-s", type=int, default=0)
    args = parser.parse_args(argv)
    if args.duration_s < 0:
        parser.error("duration must be nonnegative")
    return args


def describe_source_rows():
    """Import-light source table. Must match node.describe_sources()."""
    return [
        (node_api.TIMER, "intake", 0),
        (node_api.WATCHER, packet.HW_STATE_TOPIC, packet.PACKET_SIZE),
        (node_api.WATCHER, packet.TARGET_TOPIC, packet.TARGET_SIZE),
        (node_api.SENDER, packet.REQUEST_TOPIC, packet.PACKET_SIZE),
        (node_api.SENDER, packet.STATE_TOPIC, packet.STATE_SIZE),
    ]


def run_describe():
    # Only stdlib + sizes + libtalos_node.so on this path: no rtms, no
    # flatbuffers, no shm. The JSON itself is built by the C++ ground truth
    # (node_api.describe_json), never hand-formatted here.
    print(node_api.describe_json(packet.NODE_NAME, packet.NODE_TARGET,
                                 describe_source_rows()))
    return 0


def run_node(args):
    import node as intake_node  # heavy imports (rtms/flatbuffers) live here.

    roller_id, beam_id, period_us = intake_node.resolve_ids(args.config)
    period_s = period_us / 1e6
    stop = {"hit": False}

    def _on_signal(signum, _frame):
        del signum
        stop["hit"] = True

    signal.signal(signal.SIGINT, _on_signal)
    try:
        signal.signal(signal.SIGTERM, _on_signal)
    except (OSError, ValueError):
        pass

    node = intake_node.IntakeNode(roller_id, beam_id, period_us)
    log_handle = None
    if args.log:
        log_handle = open(args.log, "a", encoding="utf-8")
        log_handle.write(json.dumps({"event": "start",
                                     "session_id": args.session_id,
                                     "roller_id": roller_id,
                                     "beam_break_id": beam_id,
                                     "period_us": period_us}) + "\n")
        log_handle.flush()
    deadline_ns = 0
    if args.duration_s > 0:
        deadline_ns = time.monotonic_ns() + args.duration_s * 1_000_000_000
    dispatches = 0
    try:
        while not stop["hit"]:
            now_ns = time.monotonic_ns()
            if deadline_ns and now_ns >= deadline_ns:
                break
            tick_start = now_ns
            node.poll_once(now_ns)
            dispatches += 1
            elapsed_s = (time.monotonic_ns() - tick_start) / 1e9
            time.sleep(max(0.0, period_s - elapsed_s))  # 20ms poll loop.
    finally:
        node.close()
        if log_handle is not None:
            log_handle.write(json.dumps({"event": "stop",
                                         "session_id": args.session_id,
                                         "dispatches": dispatches}) + "\n")
            log_handle.close()
    summary = {"dispatches": dispatches, "sim": bool(args.sim),
               "session_id": args.session_id}
    print(json.dumps(summary))
    return 0


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.replay:
        print("replay unsupported", file=sys.stderr)
        return 2
    if args.describe:
        return run_describe()
    return run_node(args)


if __name__ == "__main__":
    sys.exit(main())
