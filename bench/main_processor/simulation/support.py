"""Helpers for the bench's full-graph simulation tests (soak_test)."""

import os
from pathlib import Path
import re
import socket
import struct
import sys
import time
import tomllib


def workspace():
    return Path(os.environ["TEST_SRCDIR"]) / os.environ.get(
        "TEST_WORKSPACE", "_main")


def udp_ports():
    # Hold both reservations until distinct ephemeral ports have been selected.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        return gateway.getsockname()[1], bridge.getsockname()[1]


def tcp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


NEUTRAL = ("neutral", 0, 0.0, 0.0)


def hardware(row):
    """log_dump's decoded hardware packet (talOS/launcher/hardware_record.h)."""
    decoded = row.get("hardware")
    if decoded is None or "error" in decoded:
        raise ValueError(f"undecoded hardware packet: {decoded}")
    return decoded


def motor_requests(row):
    """Each requested motor's (mode, slot, demand, feedforward_v), by id."""
    return {m["id"]: (m["mode"], m["slot"], m["demand"], m["feedforward_v"])
            for m in hardware(row)["motors"]}


def ds_snapshot(row):
    """The Driver Station packet's flags (bit 2: enabled) and joystick 0's
    axis 1; a packet without the scripted joystick is an error."""
    data = hardware(row)
    stick = next((j for j in data["joysticks"] if j["port"] == 0), None)
    if stick is None:
        raise ValueError("scripted joystick disconnected")
    return data["flags"], stick["axes"][1]


def log_baseline_ns(logs):
    """Find log_dump's exact origin without assuming a Python/C++ clock epoch.

    `logs` is a session directory (every *.tlog in it) or the log files
    log_dump is given; None when they hold no record. Walk v7 record headers
    only (events/log/format.h); payload decoding remains log_dump's job. On
    macOS Node time is MONOTONIC_RAW, not steady_clock.
    """
    if isinstance(logs, Path) and logs.is_dir():
        logs = logs.glob("*.tlog")
    baseline = None
    for path in logs:
        with path.open("rb") as stream:
            header = stream.read(120)
            magic, version, header_bytes = struct.unpack_from("<8sII", header)
            if magic != b"TALOSLOG" or version != 7:
                raise ValueError(f"unsupported log format: {path}")
            manifests, = struct.unpack_from("<I", header, 40)
            schema_bytes, = struct.unpack_from("<Q", header, 112)
            stream.seek(header_bytes + 112 * manifests + schema_bytes)
            while record := stream.read(64):
                if len(record) != 64:
                    raise ValueError(f"truncated log header: {path}")
                timestamp, = struct.unpack_from("<q", record, 16)
                baseline = timestamp if baseline is None else min(baseline, timestamp)
                stored, = struct.unpack_from("<I", record, 60)
                stream.seek(stored, 1)
    return baseline


def run_logs(session, node):
    """[(k, [files])]: each run of a manifest node with the event logs that
    run wrote, first run (k = 0) first. Restart k writes
    `<name>.restart<k>.tlog` (the manifest's `log_paths`), and a log over its
    byte share keeps older segments beside it as `<stem>.segment<j>.tlog`
    (talOS/launcher/README.md)."""
    runs = []
    for logged in node["log_paths"]:
        path = session / Path(logged).name
        match = re.fullmatch(rf"{re.escape(node['name'])}(?:\.restart(\d+))?\.tlog",
                             path.name)
        if match is None:
            raise ValueError(f"unexpected run log name: {path}")
        segments = sorted(session.glob(f"{path.name[:-len('.tlog')]}.segment*.tlog"))
        runs.append((int(match.group(1) or 0), [path, *segments]))
    return sorted(runs, key=lambda run: run[0])


def launcher_env(**extra):
    """The launcher environment for a full-graph sim test.

    On macOS the loop threads request realtime scheduling (the production
    policy): at timeshare priority a saturated runner delayed 5 ms loops by
    10-50 ms and broke the request-gap bounds. Linux passes without it, and
    SCHED_FIFO there has no per-thread fail-safe.
    """
    env = dict(os.environ, **extra)
    if sys.platform == "darwin":
        env["TALOS_REALTIME"] = "1"
    return env


def assert_realtime(test, launcher_output):
    """Fail a macOS run whose nodes silently fell back to timeshare."""
    if sys.platform != "darwin":
        return
    test.assertNotIn("talos: realtime scheduling incomplete", launcher_output)


def node_clock_ns():
    clock = time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin" else time.CLOCK_MONOTONIC
    return time.clock_gettime_ns(clock)


def load_config(config):
    """(robot table, {subsystem name: its file}) of a config with one include.

    A config's [robot] keys override its include's, and its [[subsystems]]
    entries replace the included entry of the same name (talOS/configuration).
    """
    document = tomllib.loads(config.read_text())
    robot = {}
    files = {}
    layers = [(config, document)]
    if "include" in document:
        included = config.parent / document["include"]
        layers.insert(0, (included, tomllib.loads(included.read_text())))
    for path, layer in layers:
        robot.update(layer.get("robot", {}))
        for entry in layer.get("subsystems", []):
            if "path" in entry:
                files[entry["name"]] = path.parent / entry["path"]
    return robot, files


def device_layout(config):
    """Mirror the config's documented sorted logical-ID and motor-bit order."""
    robot = tomllib.loads(config.read_text())
    devices = []
    for subsystem in robot["subsystems"]:
        document = tomllib.loads((config.parent / subsystem["path"]).read_text())
        for kind in ("motors", "sensors", "digital_inputs", "digital_outputs",
                     "analog_inputs", "encoders", "pwm_outputs"):
            devices.extend((subsystem["name"], name, kind)
                           for name in document.get(kind, {}))
    return [(i, *device) for i, device in enumerate(sorted(devices), 1)]
