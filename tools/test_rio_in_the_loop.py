"""Run the loopback-only, multi-process RoboRIO-in-the-loop test.

Its subject is the bench's swerve module (bench/main_processor/swerve_module:
a C++ node driving two Talon FXs from its target lane, fed by that package's
send_target) on bench's loopback config
(bench/main_processor/configuration/robot.toml), and the node's log is
replayed. Two gateways can stand at the RoboRIO end of the UDP link, and
both are tested the same way because both run the same
`talos::hardware::Gateway`:

    python3 tools/test_rio_in_the_loop.py            # //talOS/simulation:sim_gateway
    python3 tools/test_rio_in_the_loop.py --wpilib   # controller_processor/rio/, under WPILib simulation

The second one is the real RoboRIO program. It is slower to start, because it
goes through Gradle, but it exercises the actual `Robot` loop and the actual
Driver Station enable path, so it also runs two scenarios that the
stand-alone gateway cannot: with the Driver Station never enabled, the
module must not move at all; and with the Driver Station still enabled and
the wheel driven, the main processor falls silent at once, and the RoboRIO
program alone must neutralize every output on its first tick at or past the
command's expiry (REQ-SAFE-16, REQ-SAFE-17, REQ-SAFE-18), judged from its
own States on its own clock.

Logs go to a new directory under the system temporary directory, or under
--output-dir when given (CI keeps that directory when the run fails).

First build: bazel build //bench/main_processor/swerve_module:all \
                         //talOS/simulation:sim_gateway \
                         //talOS/bridge:hardware_node
             (for --wpilib, Gradle builds the robot program itself)
Run from any directory.
"""

import argparse
import json
import os
import re
import selectors
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

# The gateway outlives the bridge and the bridge outlives the node, so the
# driving scenario's final State comes after commands stop arriving. That
# State does not single out the gateway's own stop: the target is released at
# 2 s and the node and bridge exit first, and under --wpilib the Driver
# Station also disables at 9 s. The silent-main-processor scenario checks the
# gateway's own stop.
GATEWAY_SECONDS = 10
BRIDGE_SECONDS = 8
NODE_SECONDS = 6

# Under WPILib the robot's run timer and the bridge both start when the
# robot's gateway socket opens, after Gradle and the JVM are up, so a cold
# Gradle start costs no drive time. The node waits for the robot to enable
# (the bridge's config push, well under a second), then runs NODE_SECONDS
# inside BRIDGE_SECONDS, so the stand-alone timings fit.

# A host that sleeps mid-run freezes every process while steady clocks keep
# counting (a headless Mac's DarkWake maintenance sleep lasts 6 to 8 s), which
# reads as a RoboRIO loop stall. More than this much sleep fails the run as a
# host fault.
HOST_SLEEP_LIMIT_SECONDS = 0.05

# The simulated RoboRIO loop ticks every 5 ms and a loaded desktop delays a
# tick by tens of milliseconds; an interval past this is a stall.
LOOP_INTERVAL_LIMIT_US = 250_000

# Gradle, a JVM and WPILib startup. Generous, because a cold Gradle daemon on a
# loaded machine is slow and starting the bridge early would fail the run for
# the wrong reason.
WPILIB_STARTUP_TIMEOUT = 180

SUMMARY = re.compile(r"^sim_(?:gateway|robot):.*$", re.MULTILINE)

# RioLog's periodic timing report and LoopWatchdog's stall report (through
# HAL_SendError, "Error at talos loop watchdog: [<s>.<ms>] RoboRIO loop
# stalled ...").
LOOP_TIMING = re.compile(
    r"\[rio\] loop timing: (\d+) overruns .*?, (\d+) late ticks .*"
    r"max interval (\d+) us")
LOOP_STALLED = "RoboRIO loop stalled past an expired command"

# The silent-main-processor scenario. The robot runs this long from its
# gateway socket opening: bring-up and driving take about 2 s, the judged
# silence a quarter second.
SILENT_GATEWAY_SECONDS = 6
# How long the bridge and node would run; they are stopped first.
SILENT_MAIN_SECONDS = 30
# send_target has no stop handler, so it ends on its own: its target holds
# this long, through the cut (about half a second after it starts), and its
# exit closes its queues.
SILENT_TARGET_SECONDS = 3
# Bounds a stop handler that hangs; past it the process is SIGKILLed and the
# run fails, since a killed holder leaves its shared memory behind.
STOP_SECONDS = 10
# The subject: the bench swerve module's package, node name and config.
SUBJECT = "swerve_module"
SUBJECT_PACKAGE = "bench/main_processor/swerve_module"
SUBJECT_CONFIG = "bench/main_processor/configuration/robot.toml"
# send_target's drive velocity (the drive motor's mechanism unit, rotor rps
# while the bench module's reduction is 1) and wheel heading. The Phoenix
# simulation turns a rotor at 100 rps per 12 V, so the drive settles near
# 10 rps and 1.2 V (the config's provisional kV, 0.12 V/rps, is that plant's);
# neutral reads exactly 0 V. Past both thresholds below on its way up, the
# motor is plainly driven and turning. 10 rps is also the node's
# max_drive_velocity_rps, which caps the target.
DRIVE_RPS = 10.0
STEER_ROT = 0.0
DRIVEN_VOLTS = 1.0
DRIVEN_RPS = 1.0
# Bounds only a hang: every wait below ends on a State or a process exit.
SILENT_WAIT_SECONDS = 30

# What the scenario decodes of the RoboRIO link, as talOS/hardware/messages.cc
# and talOS/protocol/frame.h encode it at protocol version 8. A frame of
# another version fails the run rather than being misread: a layout change
# updates this decoder (RULES.md 9).
PROTOCOL_VERSION = 8
FRAME_MAGIC = 0x4F4952534F4C4154
FRAME_HEADER = struct.Struct("<QHHHHQQII")
STATE_FRAME, COMMAND_FRAME, DRIVER_STATION_FRAME = 20, 21, 24
# config, boot, epoch, sample time, last command sequence, clock probe, tick
# sequence, build revision; build program and dirty; flags; seven counts.
STATE_BASE = struct.Struct("<8QBBI7H")
# id, valid, position, velocity, voltage, stator current, two ages, faults,
# temperature, supply current.
MOTOR_SAMPLE = struct.Struct("<HBdddhIIIhh")
SENSOR_SAMPLE_SIZE = 27
DIGITAL_INPUT_SAMPLE_SIZE = 4
DIGITAL_OUTPUT_SAMPLE = struct.Struct("<HBB")
ANALOG_INPUT_SAMPLE_SIZE = 15
ENCODER_SAMPLE_SIZE = 23
PWM_SAMPLE = struct.Struct("<HBd")
HEALTH_SAMPLE_SIZE = 31
# rejected commands, neutralized motors, apply failures, device resets.
COUNTERS = struct.Struct("<4I")
# config, boot, epoch, observed time; motor, digital and PWM counts.
COMMAND_BASE = struct.Struct("<4Q3H")
MOTOR_REQUEST = struct.Struct("<HBBdd")
DIGITAL_OUTPUT_REQUEST_SIZE = 3
PWM_REQUEST_SIZE = 10
NEUTRAL_MODE = 0
FLAG_CONFIGURED, FLAG_ENABLED = 1, 2
FLAG_COMMAND_ACTIVE, FLAG_HARDWARE_FAULT = 4, 8


def host_sleep_seconds():
    """Seconds this host has spent asleep since boot, or None if unknown.

    The difference between a clock that counts sleep and one that does not.
    """
    if sys.platform == "darwin":
        return (time.clock_gettime(time.CLOCK_MONOTONIC) -
                time.clock_gettime(time.CLOCK_UPTIME_RAW))
    if hasattr(time, "CLOCK_BOOTTIME"):
        return (time.clock_gettime(time.CLOCK_BOOTTIME) -
                time.clock_gettime(time.CLOCK_MONOTONIC))
    return None


class Scenario:
    """One full run of gateway + bridge + node, cleaned up on the way out."""

    def __init__(self, root, output, config, wpilib=False):
        self.root = root
        self.wpilib = wpilib
        self.slept_at_start = host_sleep_seconds()
        self.binaries = root / "bazel-bin" / SUBJECT_PACKAGE
        # sim_gateway and the bridge are talOS's (talOS/simulation,
        # talOS/bridge), not the subject package's, so each is spelled out
        # by its own package: a stale binary of that name left in the subject
        # package would otherwise be picked up silently.
        self.sim_gateway = (
            root / "bazel-bin/talOS/simulation/sim_gateway")
        self.bridge = root / "bazel-bin/talOS/bridge/hardware_node"
        self.config = config
        self.output = output
        self.children = []
        self.names = {}
        self.stopped = set()
        self.files = []

    def start(self, name, argv, cwd=None, env=None):
        log = (self.output / (name + ".txt")).open("w")
        self.files.append(log)
        # Its own process group, so a signal reaches the whole child. Gradle
        # is the exception: the simulated robot runs under the Gradle daemon,
        # in the daemon's group, not the wrapper's. It stops on its own at
        # TALOS_SIM_RUN_MS; when the wrapper's group is signalled first, the
        # wrapper exits and the daemon cancels the build, which stops the
        # robot and frees port 5802.
        process = subprocess.Popen(
            argv, stdout=log, stderr=subprocess.STDOUT,
            cwd=cwd or self.root, env=env, start_new_session=True,
        )
        self.children.append(process)
        self.names[process.pid] = name
        return process

    def start_sim_gateway(self, seconds):
        # --config is required: without it sim_gateway exits with a usage
        # error before any simulation traffic exists.
        self.start("gateway", [str(self.sim_gateway),
                               "--config", str(self.config),
                               "--duration-s", str(seconds)])

    def start_wpilib_gateway(self, seconds, enabled=True, disable_after_ms=None,
                             enable_when_configured=False):
        environment = dict(os.environ)
        # The simulated robot has no Driver Station of its own, and it must stop
        # on its own rather than be killed, so that its summary line is written.
        if enable_when_configured:
            # The gateway refuses hardware configuration while enabled, so an
            # enabled-at-startup robot wedges: the bridge's config push is
            # refused forever and no command is ever accepted. Start disabled
            # and let the robot enable once the push lands instead.
            environment["TALOS_SIM_DS"] = "auto"
        else:
            environment["TALOS_SIM_DS"] = "1" if enabled else "0"
        environment["TALOS_SIM_RUN_MS"] = str(seconds * 1000)
        if disable_after_ms is not None:
            environment["TALOS_SIM_DS_DISABLE_AFTER_MS"] = str(disable_after_ms)

        gateway = self.start(
            "gateway",
            # -Pheadless keeps the simulation GUI out of an automated run; the
            # TALOS_SIM_DS variables above stand in for it. The task is the
            # plain simulateNative the rest of the repo uses (sim.sh and the
            # hardware README), run from the Gradle project that owns the
            # RoboRIO program.
            ["./gradlew", "--offline", "--quiet", "-Pheadless",
             "simulateNative"],
            cwd=self.root / "controller_processor/rio",
            env=environment,
        )
        # The robot prints this once its gateway socket is open, which is the
        # only moment it is safe to start the bridge.
        self.wait_for("hardware gateway config=", gateway)
        return gateway

    def wait_for(self, needle, process, others=()):
        path = self.output / "gateway.txt"
        deadline = time.monotonic() + WPILIB_STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            if path.exists() and needle in path.read_text():
                return
            if process.poll() is not None:
                raise RuntimeError(f"Gateway exited early; see {self.output}")
            ended = self.exited(others)
            if ended:
                raise RuntimeError(f"waiting for '{needle}': {ended}; see "
                                   f"{self.output}")
            time.sleep(0.25)
        raise RuntimeError(f"Gateway never started; see {self.output}")

    def drive(self, log_path, bridge_seconds, node_seconds):
        self.start_bridge(bridge_seconds)
        self.start_node(log_path, node_seconds)

    def start_bridge(self, bridge_seconds, link=()):
        # --config is required: without it the bridge exits with a usage error
        # before any simulation traffic exists. --sim, as on the node, treats
        # the config as commissioned; one without the other would give them
        # different config ids, and the node would drop every State.
        return self.start("bridge", [str(self.bridge), "--sim",
                                     "--config", str(self.config),
                                     "--duration-s", str(bridge_seconds),
                                     *link])

    def start_node(self, log_path, node_seconds, target_seconds=2):
        # The node requires --config like the bridge does.
        node = self.start("node", [str(self.binaries / "node"), "--sim",
                                   "--config", str(self.config),
                                   "--duration-s", str(node_seconds),
                                   "--log", str(log_path)])
        time.sleep(0.5)
        sender = self.start("send_target",
                            [str(self.binaries / "send_target"),
                             str(DRIVE_RPS), str(STEER_ROT),
                             str(target_seconds)])
        return node, sender

    def stop(self, processes):
        """SIGTERM each process's group and wait for its stop handler.

        Returns the names of those that did not exit 0 within STOP_SECONDS
        (SIGKILLed then). A clean exit closes their shared-memory queues, and
        the last closer unlinks each; a killed holder would leave it.
        """
        for process in processes:
            _terminate_group(process, signal.SIGTERM)
        failed = []
        deadline = time.monotonic() + STOP_SECONDS
        for process in processes:
            try:
                code = process.wait(
                    timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                _terminate_group(process, signal.SIGKILL)
                code = process.wait()
            self.stopped.add(process.pid)
            if code != 0:
                failed.append(f"{self.names[process.pid]} ({code})")
        return failed

    def exited(self, processes):
        """The name and exit code of the first of these that has exited."""
        for process in processes:
            if process.poll() is not None:
                name = self.names[process.pid]
                return f"{name} exited ({process.returncode})"
        return None

    def wait(self):
        failed = False
        for child in self.children:
            code = child.wait(timeout=WPILIB_STARTUP_TIMEOUT)
            if code != 0 and child.pid not in self.stopped:
                failed = True
        # A host sleep explains a failed process or a loop stall, so it is
        # reported first.
        self.check_host_sleep()
        if self.wpilib:
            self.check_loop_timing()
        if failed:
            raise RuntimeError(f"Process failed; see {self.output}")

    def check_host_sleep(self):
        if self.slept_at_start is None:
            return
        slept = host_sleep_seconds() - self.slept_at_start
        if slept > HOST_SLEEP_LIMIT_SECONDS:
            raise RuntimeError(
                f"host slept {slept:.2f} s during the run; not a robot result "
                f"(see {self.output})")

    def check_loop_timing(self):
        text = (self.output / "gateway.txt").read_text()
        # The timing report names the cause, so it is checked before the
        # watchdog's stall report.
        for overruns, _late, interval in LOOP_TIMING.findall(text):
            if int(interval) <= LOOP_INTERVAL_LIMIT_US:
                continue
            cause = ("overruns: blocking work in the tick" if int(overruns)
                     else "late ticks only: the loop was not scheduled, "
                     "not slow work in the tick")
            raise RuntimeError(
                f"The RoboRIO loop went {int(interval) / 1000:.0f} ms between "
                f"ticks ({cause}); see {self.output / 'gateway.txt'}")
        if LOOP_STALLED in text:
            raise RuntimeError(
                f"The RoboRIO loop watchdog reported a stalled loop; see "
                f"{self.output / 'gateway.txt'}")

    def summary(self):
        text = (self.output / "gateway.txt").read_text()
        # The last match: sim_gateway prints a startup line ("backend=...") in
        # the same "sim_gateway:" shape before its exit summary, so the first
        # match is not the summary. A gateway that dies mid-run exits nonzero
        # and is already rejected by wait() before this runs.
        found = SUMMARY.findall(text)
        if not found:
            raise RuntimeError(f"The gateway printed no summary; see "
                               f"{self.output}")
        return found[-1].strip()

    def replay(self, log_path):
        return subprocess.run(
            [str(self.binaries / "node"), "--sim",
             "--config", str(self.config), "--replay", str(log_path)],
            capture_output=True, text=True, timeout=15, check=True,
        ).stdout.strip()

    def close(self):
        for child in self.children:
            if child.poll() is None:
                _terminate_group(child, signal.SIGTERM)
        for child in self.children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                _terminate_group(child, signal.SIGKILL)
                child.wait()
        for file in self.files:
            file.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
        return False


def _terminate_group(process, sig):
    try:
        os.killpg(os.getpgid(process.pid), sig)
    except (ProcessLookupError, PermissionError):
        pass


def rotations_of(summary):
    found = re.search(r"first_wheel_rotations=([0-9.]+)", summary)
    if not found:
        raise RuntimeError(f"Unreadable gateway summary: {summary}")
    return float(found.group(1))


def check_ports():
    """Refuse to run if another gateway or bridge holds the demo's ports."""
    for port in (5802, 5803):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("0.0.0.0", port))


def robot_settings(config, *keys):
    """Values from robot.toml's [robot] table, which sets them directly."""
    # Imported here: tomllib is Python 3.11+, and only --wpilib needs it, so
    # an older system python3 still runs the sim_gateway mode.
    import tomllib
    robot = tomllib.loads(config.read_text()).get("robot", {})
    missing = [key for key in keys if key not in robot]
    if missing:
        raise RuntimeError(f"{config} sets no [robot] {', '.join(missing)}")
    return [robot[key] for key in keys]


TOML_HEADER = re.compile(r"^\s*(\[\[?)\s*([\w.-]+)\s*\]\]?\s*(?:#.*)?$")
TOML_NAME = re.compile(r'^\s*name\s*=\s*"([^"]*)"\s*(?:#.*)?$')
TOML_PATH = re.compile(r'^(\s*(?:path|defaults)\s*=\s*)"([^"]*)"(\s*(?:#.*)?)$')


def subject_config(config, output):
    """`config` narrowed to the subject's own subsystem, written in output.

    The gateways' exit summary (the driving and disabled scenarios' verdict)
    reports the first motor by logical id, and logical ids follow (subsystem,
    device name) order (talOS/configuration/config_parser.h), so in bench's
    robot.toml that is the bench motor's Kraken, which the subject never
    drives. This config is that file's [robot] tables, verbatim, and the
    subject's [[subsystems]] entry with its paths made absolute, so its
    subsystem file is read in place: the same robot settings, devices, CAN
    ids and buses, with the subject's drive motor first, and nothing copied
    by hand. [studio] is left out (no Studio runs here). It is cut line by
    line, not through tomllib, which the system python3 the sim_gateway mode
    supports (3.8) lacks, and it fails closed: a top-level key, a table it
    does not know (a future hardware table must not be dropped unseen), a
    header it cannot read or a missing entry fails the run, and anything
    else the node and bridge cannot load fails it there.
    """
    blocks = []  # (bracket, table name, lines)
    for line in config.read_text().splitlines():
        header = TOML_HEADER.match(line)
        if header:
            blocks.append((header.group(1), header.group(2), [line]))
        elif line.lstrip().startswith("["):
            raise RuntimeError(f"{config}: cannot read the header "
                               f"{line.strip()!r}")
        elif blocks:
            blocks[-1][2].append(line)
        elif line.strip() and not line.lstrip().startswith("#"):
            raise RuntimeError(f"{config}: top-level key {line.strip()!r} "
                               f"is not copied")
    kept = []
    entries = 0
    for bracket, name, lines in blocks:
        if name == "robot" or name.startswith("robot."):
            kept += lines
        elif bracket == "[" and name == "studio":
            continue
        elif bracket != "[[" or name != "subsystems":
            raise RuntimeError(f"{config}: {lines[0].strip()} is not "
                               f"[robot], [robot.*], [studio] or "
                               f"[[subsystems]], so the subject config "
                               f"would drop it")
        else:
            names = [m.group(1) for m in map(TOML_NAME.match, lines) if m]
            if names != [SUBJECT]:
                continue
            entries += 1
            for line in lines:
                path = TOML_PATH.match(line)
                if path:
                    absolute = (config.parent / path.group(2)).resolve()
                    line = f'{path.group(1)}"{absolute}"{path.group(3)}'
                kept.append(line)
    if entries != 1:
        raise RuntimeError(f"{config} has {entries} [[subsystems]] entries "
                           f"named '{SUBJECT}'; expected one")
    narrowed = output / "subject.toml"
    narrowed.write_text(f"# {config}, narrowed to [[subsystems]] {SUBJECT} "
                        f"by tools/test_rio_in_the_loop.py\n" +
                        "\n".join(kept) + "\n")
    return narrowed


def decode_frame(data):
    """(type, sequence, payload) of one talOS frame, or None if not one."""
    if len(data) < FRAME_HEADER.size:
        return None
    (magic, version, header_size, frame_type, _flags, sequence, _time_us,
     payload_size, _reserved) = FRAME_HEADER.unpack_from(data)
    if magic != FRAME_MAGIC:
        return None
    if version != PROTOCOL_VERSION or header_size != FRAME_HEADER.size:
        raise RuntimeError(
            f"frame of protocol version {version} (header {header_size} "
            f"bytes); this script decodes version {PROTOCOL_VERSION}: "
            f"update its decoder with the layout change")
    payload = data[header_size:]
    if len(payload) != payload_size:
        raise RuntimeError(f"frame payload of {len(payload)} bytes, header "
                           f"says {payload_size}")
    return frame_type, sequence, payload


def decode_state(payload):
    """The fields of a State this scenario judges, as a dict."""
    (_config, _boot, epoch, sample_us, last_command_sequence, _probe,
     sequence, _revision, _program, _dirty, flags, motor_count, sensor_count,
     digital_input_count, digital_output_count, analog_input_count,
     encoder_count, pwm_count) = STATE_BASE.unpack_from(payload)
    expected = (STATE_BASE.size + motor_count * MOTOR_SAMPLE.size +
                sensor_count * SENSOR_SAMPLE_SIZE +
                digital_input_count * DIGITAL_INPUT_SAMPLE_SIZE +
                digital_output_count * DIGITAL_OUTPUT_SAMPLE.size +
                analog_input_count * ANALOG_INPUT_SAMPLE_SIZE +
                encoder_count * ENCODER_SAMPLE_SIZE +
                pwm_count * PWM_SAMPLE.size + HEALTH_SAMPLE_SIZE +
                COUNTERS.size)
    if len(payload) != expected:
        raise RuntimeError(f"State of {len(payload)} bytes, its counts give "
                           f"{expected}: update this script's decoder")
    offset = STATE_BASE.size
    motors = []
    for _ in range(motor_count):
        (motor_id, valid, _position, velocity_rps, voltage, *_rest) = (
            MOTOR_SAMPLE.unpack_from(payload, offset))
        motors.append((motor_id, bool(valid), velocity_rps, voltage))
        offset += MOTOR_SAMPLE.size
    offset += (sensor_count * SENSOR_SAMPLE_SIZE +
               digital_input_count * DIGITAL_INPUT_SAMPLE_SIZE)
    digital_outputs = []
    for _ in range(digital_output_count):
        digital_outputs.append(DIGITAL_OUTPUT_SAMPLE.unpack_from(payload,
                                                                 offset))
        offset += DIGITAL_OUTPUT_SAMPLE.size
    offset += (analog_input_count * ANALOG_INPUT_SAMPLE_SIZE +
               encoder_count * ENCODER_SAMPLE_SIZE)
    pwm_outputs = []
    for _ in range(pwm_count):
        pwm_outputs.append(PWM_SAMPLE.unpack_from(payload, offset))
        offset += PWM_SAMPLE.size
    _rejected, _neutralized, apply_failures, resets = COUNTERS.unpack_from(
        payload, len(payload) - COUNTERS.size)
    return {
        "sequence": sequence, "sample_us": sample_us, "epoch": epoch,
        "flags": flags, "last_command_sequence": last_command_sequence,
        "motors": motors, "digital_outputs": digital_outputs,
        "pwm_outputs": pwm_outputs, "apply_failures": apply_failures,
        "resets": resets,
    }


def decode_command(payload):
    """(observed_time_us, [(motor id, mode)]) of a Command."""
    (_config, _boot, _epoch, observed_us, motor_count, digital_count,
     pwm_count) = COMMAND_BASE.unpack_from(payload)
    expected = (COMMAND_BASE.size + motor_count * MOTOR_REQUEST.size +
                digital_count * DIGITAL_OUTPUT_REQUEST_SIZE +
                pwm_count * PWM_REQUEST_SIZE)
    if len(payload) != expected:
        raise RuntimeError(f"Command of {len(payload)} bytes, its counts give "
                           f"{expected}: update this script's decoder")
    motors = []
    for i in range(motor_count):
        offset = COMMAND_BASE.size + i * MOTOR_REQUEST.size
        motor_id, mode, _slot, _demand, _feedforward = (
            MOTOR_REQUEST.unpack_from(payload, offset))
        motors.append((motor_id, mode))
    return observed_us, motors


class Relay:
    """The main processor's end of the RoboRIO link, as the RoboRIO sees it.

    It holds 127.0.0.1:5803, where the RoboRIO program sends, and relays
    every datagram both ways between it and a bridge started on other
    ports, recording each State it receives and each command it forwards.
    The cut (`wait(..., cut=True)`) is the main processor dying: from then
    on nothing crosses the relay in either direction, and it only listens. Its
    sends keep leaving without an error, as they would towards a powered-off
    host, which answers nothing.
    """

    def __init__(self):
        self.robot_side = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.bridge_side = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.robot_side.bind(("127.0.0.1", 5803))
            self.bridge_side.bind(("127.0.0.1", 0))
            # The bridge takes no port 0, so one is reserved and released for
            # it; another process could take it first, and the bridge's
            # failed bind then fails the run.
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(("127.0.0.1", 0))
                self.bridge_port = probe.getsockname()[1]
        except OSError:
            self.robot_side.close()
            self.bridge_side.close()
            raise
        self.relay_port = self.bridge_side.getsockname()[1]
        self.condition = threading.Condition()
        self.forwarding = True
        self.forwarded_after_cut = 0
        self.states = []
        self.commands = {}
        self.driver_station_after_cut = 0
        self.error = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        selector = selectors.DefaultSelector()
        selector.register(self.robot_side, selectors.EVENT_READ)
        selector.register(self.bridge_side, selectors.EVENT_READ)
        try:
            while not self.stopping.is_set():
                for key, _ in selector.select(timeout=0.1):
                    try:
                        data = key.fileobj.recv(65536)
                    except ConnectionRefusedError:
                        continue  # an earlier send found no bridge bound
                    if key.fileobj is self.robot_side:
                        self._from_robot(data)
                    else:
                        self._from_bridge(data)
        except Exception as error:  # reported to the waiting thread
            with self.condition:
                self.error = error
                self.condition.notify_all()
        finally:
            selector.close()

    def _from_robot(self, data):
        frame = decode_frame(data)
        with self.condition:
            # Under the lock, as on the other path: nothing is forwarded
            # once the cut is made.
            if self.forwarding:
                self._send(self.bridge_side, data, self.bridge_port)
            if frame is None:
                return
            frame_type, _sequence, payload = frame
            if frame_type == STATE_FRAME:
                self.states.append(decode_state(payload))
                self.condition.notify_all()
            elif frame_type == DRIVER_STATION_FRAME and not self.forwarding:
                self.driver_station_after_cut += 1

    def _from_bridge(self, data):
        with self.condition:
            if not self.forwarding:
                return
            frame = decode_frame(data)
            if frame is not None and frame[0] == COMMAND_FRAME:
                self.commands[frame[1]] = decode_command(frame[2])
            self._send(self.robot_side, data, 5802)

    def _send(self, sock, data, port):
        # Called with the lock held. Both callers return before this once the
        # link is cut, so the count stays 0 unless a later edit forwards
        # without checking `forwarding`: the harness checking itself.
        if not self.forwarding:
            self.forwarded_after_cut += 1
        try:
            sock.sendto(data, ("127.0.0.1", port))
        except OSError:
            pass  # nobody bound there yet: the datagram is lost, as on a link

    def wait(self, predicate, what, exited, cut=False):
        """Wait for predicate() over the recorded traffic, under the lock.

        exited() names a process that ended first, or returns None. With cut,
        the cut is made under the same lock that saw predicate() hold, so no
        datagram passes between the two.
        """
        deadline = time.monotonic() + SILENT_WAIT_SECONDS
        with self.condition:
            while not predicate():
                if self.error is not None:
                    raise RuntimeError(f"relay failed: {self.error!r}")
                ended = exited()
                if ended:
                    raise RuntimeError(f"{what}: {ended} first")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(f"{what}: not within "
                                       f"{SILENT_WAIT_SECONDS} s")
                self.condition.wait(min(remaining, 0.5))
            if cut:
                self.forwarding = False

    def dump(self, output):
        """Write what the relay recorded, one JSON object a line, for CI's
        artifacts: every State (relay_states.jsonl) and every forwarded
        command by frame sequence (relay_commands.jsonl)."""
        with self.condition:
            states = list(self.states)
            commands = sorted(self.commands.items())
        with (output / "relay_states.jsonl").open("w") as out:
            for state in states:
                out.write(json.dumps(state) + "\n")
        with (output / "relay_commands.jsonl").open("w") as out:
            for sequence, (observed_us, motors) in commands:
                out.write(json.dumps({"sequence": sequence,
                                      "observed_us": observed_us,
                                      "motors": motors}) + "\n")

    def close(self):
        self.stopping.set()
        self.thread.join()
        self.robot_side.close()
        self.bridge_side.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
        return False


def run_driving_scenario(root, wpilib, output_root):
    """The robot is enabled, is told to drive, and then loses its commands."""
    output = Path(tempfile.mkdtemp(prefix="talos-drive-", dir=output_root))
    log_path = output / "drive.tlog"
    config = subject_config(root / SUBJECT_CONFIG, output)

    with Scenario(root, output, config, wpilib) as scenario:
        if wpilib:
            # Explicit bring-up order: the robot starts disabled, the bridge's
            # config push lands while it is disabled, and only then does the
            # robot enable. Driving starts after the enable print so the
            # two-second drive window falls inside enabled time, and the robot
            # disables itself near the end so the summary below also covers
            # the Driver Station disable path.
            gateway = scenario.start_wpilib_gateway(
                GATEWAY_SECONDS, enable_when_configured=True,
                disable_after_ms=(GATEWAY_SECONDS - 1) * 1000)
            scenario.start_bridge(BRIDGE_SECONDS)
            scenario.wait_for("simulated Driver Station: enabling", gateway)
            scenario.start_node(log_path, NODE_SECONDS)
        else:
            scenario.start_sim_gateway(GATEWAY_SECONDS)
            scenario.drive(log_path, BRIDGE_SECONDS, NODE_SECONDS)

        scenario.wait()

        print(scenario.replay(log_path))
        summary = scenario.summary()

        if rotations_of(summary) <= 1:
            raise RuntimeError("The simulated swerve module did not move")
        if "first_wheel_rps=0.000000 command_active=0" not in summary:
            raise RuntimeError(
                "The gateway did not neutralize after communication stopped")

        print(summary)
        print((output / "bridge.txt").read_text().strip())
        print((output / "node.txt").read_text().strip())

    return output


def run_disabled_scenario(root, output_root):
    """The same drive request, with the Driver Station never enabled.

    Runs against the WPILib RoboRIO program, so it checks that program's own
    Driver Station gating; sim_gateway's disabled schedules
    (`--mode-schedule`) are covered by //talOS/simulation:sim_gateway_test.
    """
    output = Path(tempfile.mkdtemp(prefix="talos-disabled-", dir=output_root))
    config = subject_config(root / SUBJECT_CONFIG, output)

    with Scenario(root, output, config, wpilib=True) as scenario:
        scenario.start_wpilib_gateway(6, enabled=False)
        scenario.drive(output / "drive.tlog", 4, 3)
        scenario.wait()

        summary = scenario.summary()
        if rotations_of(summary) != 0:
            raise RuntimeError(
                "A disabled robot moved: the Driver Station gate is not real")
        if "command_active=0" not in summary:
            raise RuntimeError("A disabled robot accepted a command")

        print(summary)

    return output


def turning(state):
    """Some valid motor sample is driven and turning."""
    return any(valid and abs(volts) >= DRIVEN_VOLTS and abs(rps) >= DRIVEN_RPS
               for _, valid, rps, volts in state["motors"])


def driven(relay):
    """The newest State shows a turning motor under a driving command."""
    if not relay.states:
        return False
    state = relay.states[-1]
    wanted = FLAG_CONFIGURED | FLAG_ENABLED | FLAG_COMMAND_ACTIVE
    command = relay.commands.get(state["last_command_sequence"])
    return ((state["flags"] & wanted) == wanted and command is not None and
            any(mode != NEUTRAL_MODE for _, mode in command[1]) and
            turning(state))


def judge_silence(states, commands, timeout_us, status_hz):
    """Judge the RoboRIO program's States after the main processor died.

    ARCHITECTURE.md "The tick": the RoboRIO neutralizes every output within
    command_timeout_us of the State the last accepted command was built from,
    plus its loop's lateness. The gateway (Gateway::Tick) does it on its first
    tick at or past command_expiry_us(), the older of that observation and the
    command's acceptance plus command_timeout_us. So the State of the tick
    before must still be in force and earlier than the expiry, and the State
    of that tick neutral, at or past it: the lateness is that one tick's
    interval, measured, not assumed. Returns the measurements.
    """
    by_sequence = {s["sequence"]: s for s in states}
    last = states[-1]["last_command_sequence"]
    if last not in commands:
        raise RuntimeError(f"the RoboRIO's last accepted command, sequence "
                           f"{last}, never passed the relay")
    observed_us, requests = commands[last]
    if all(mode == NEUTRAL_MODE for _, mode in requests):
        raise RuntimeError("the last accepted command drove nothing: the "
                           "precondition did not hold at the cut")
    accepted = min((s for s in states if s["last_command_sequence"] == last),
                   key=lambda s: s["sequence"])
    expiry_us = min(accepted["sample_us"], observed_us) + timeout_us
    stop = min((s for s in states if s["sequence"] > accepted["sequence"] and
                not s["flags"] & FLAG_COMMAND_ACTIVE),
               key=lambda s: s["sequence"], default=None)
    newest = max(s["sample_us"] for s in states)
    if stop is None:
        raise RuntimeError(
            f"the RoboRIO program still drove {newest - expiry_us} us past "
            f"the command's expiry (observed State {observed_us} us + "
            f"command_timeout_us {timeout_us})")
    before = by_sequence.get(stop["sequence"] - 1)
    if before is None:
        raise RuntimeError(f"State {stop['sequence'] - 1} never reached the "
                           f"relay; the stop cannot be placed")
    if before["sample_us"] >= expiry_us:
        raise RuntimeError(
            f"the RoboRIO program drove at {before['sample_us']} us, "
            f"{before['sample_us'] - expiry_us} us past the expiry "
            f"{expiry_us} us (observed {observed_us} + {timeout_us}), and "
            f"stopped only at {stop['sample_us']} us")
    if stop["sample_us"] < expiry_us:
        raise RuntimeError(
            f"the command stopped at {stop['sample_us']} us, "
            f"{expiry_us - stop['sample_us']} us before its expiry: another "
            f"stop than the command timeout")
    # A steer motor holding its angle in position mode is a non-neutral
    # command alone; the wheel must still be turning up to the stop.
    if not turning(before):
        raise RuntimeError(
            f"State {before['sequence']}, the last before the stop, has no "
            f"valid motor at >= {DRIVEN_VOLTS} V and >= {DRIVEN_RPS} rps: "
            f"{before['motors']}")
    # The timeout's own mark: still enabled and healthy, a new epoch, and no
    # apply failure or device reset, the gateway's other stops.
    wanted = FLAG_CONFIGURED | FLAG_ENABLED
    if (stop["flags"] & (wanted | FLAG_HARDWARE_FAULT)) != wanted:
        raise RuntimeError(f"stop State flags {stop['flags']:#x}: not an "
                           f"enabled, healthy gateway timing out")
    if (stop["epoch"] != before["epoch"] + 1 or
            stop["apply_failures"] != before["apply_failures"] or
            stop["resets"] != before["resets"]):
        raise RuntimeError(f"stop State {stop} after {before}: not the "
                           f"command timeout")
    # Every output stays neutral while the Driver Station stays enabled. A
    # motor's voltage is its Phoenix MotorVoltage status signal, so the
    # NeutralOut sent in the stop tick reaches it a status period or more
    # later. The 0 V check therefore starts four status periods
    # (4 / status_hz) after the stop State; that window only delays where the
    # check starts. No document bounds this lag: every State from there to
    # the end of the observation (about 100 ms more) must read 0 V on a valid
    # sample, so a longer lag fails the run. The measured lag is reported.
    settled_us = stop["sample_us"] + 4 * 1_000_000 / status_hz
    settled = 0
    zero_after_us = None
    for s in states:
        if s["sequence"] < stop["sequence"]:
            continue
        if (s["flags"] & (wanted | FLAG_COMMAND_ACTIVE | FLAG_HARDWARE_FAULT)
                != wanted or s["last_command_sequence"] != last):
            raise RuntimeError(f"after the stop, State {s['sequence']} has "
                               f"flags {s['flags']:#x} and command "
                               f"{s['last_command_sequence']}")
        if zero_after_us is None and all(
                valid and volts == 0.0 for _, valid, _, volts in s["motors"]):
            zero_after_us = s["sample_us"] - stop["sample_us"]
        if s["sample_us"] < settled_us:
            continue
        settled += 1
        live = [(i, volts) for i, valid, _, volts in s["motors"]
                if not valid or volts != 0.0]
        # The bench config has no digital or PWM output, so these two lines
        # check nothing there; they hold any config that has them.
        live += [(i, value) for i, _, value in s["digital_outputs"] if value]
        live += [(i, output) for i, _, output in s["pwm_outputs"]
                 if output != 0.0]
        if live:
            raise RuntimeError(
                f"State {s['sequence']}, {s['sample_us'] - stop['sample_us']}"
                f" us after the stop, has outputs not neutral (id, value; an "
                f"invalid motor counts): {live}")
    if settled == 0:
        raise RuntimeError("no State came after the motor voltages settled")
    return {
        "stop_after_observed_us": stop["sample_us"] - observed_us,
        "stop_after_expiry_us": stop["sample_us"] - expiry_us,
        "stop_tick_interval_us": stop["sample_us"] - before["sample_us"],
        "zero_volts_after_stop_us": zero_after_us,
        "settled_states": settled,
    }


# REQ-SAFE-16, REQ-SAFE-17, REQ-SAFE-18: composition evidence that the
# RoboRIO program's own loop (Robot.cpp, its Endpoint, Gateway and Phoenix
# backend, on its real clock) stops a driven robot when the main processor
# dies, the Driver Station still enabled. gateway_test.cc pins the expiry tick
# and the neutral refresh on a fake clock; nothing below this runs Robot.cpp.
def run_silent_main_processor_scenario(root, output_root):
    """The main processor dies while the robot drives, enabled.

    The relay cuts the link while a wheel turns: from then on nothing the
    main processor sends reaches the RoboRIO program, and the Driver Station
    stays enabled, so only the RoboRIO program can stop the wheel. Its
    States are read at the relay, not through the bridge.

    The cut is the death. The bridge's socket is connected to the relay
    alone, the RoboRIO program's to 127.0.0.1:5803 alone (which the relay
    holds), and the node and send_target use no socket, so after the cut no
    release, neutral request or disable from any of them can reach the
    RoboRIO: to it, a cut followed by clean stops is the same silence as a
    SIGKILL, or a coprocessor losing power. The bridge and node are then
    stopped with SIGTERM and send_target ends on its own, because a killed
    process never closes its shared-memory queues and leaves their
    production-name segments in /dev/shm for every later run.

    It runs bench's robot.toml whole, not narrowed: it judges every output
    from the States, so the bench's other devices, which nothing drives,
    must read neutral too.
    """
    output = Path(tempfile.mkdtemp(prefix="talos-silent-", dir=output_root))
    config = root / SUBJECT_CONFIG
    timeout_us, status_hz = robot_settings(config, "command_timeout_us",
                                           "status_hz")

    with Relay() as relay, Scenario(root, output, config,
                                    wpilib=True) as scenario:
        gateway = scenario.start_wpilib_gateway(SILENT_GATEWAY_SECONDS,
                                                enable_when_configured=True)
        bridge = scenario.start_bridge(
            SILENT_MAIN_SECONDS,
            ["--remote", "127.0.0.1", "--remote-port", str(relay.relay_port),
             "--local-port", str(relay.bridge_port)])
        scenario.wait_for("simulated Driver Station: enabling", gateway,
                          [bridge])
        node, sender = scenario.start_node(
            output / "drive.tlog", SILENT_MAIN_SECONDS,
            target_seconds=SILENT_TARGET_SECONDS)
        relay.wait(lambda: driven(relay),
                   "no State showed a driven, turning motor",
                   lambda: scenario.exited([gateway, bridge, node, sender]),
                   cut=True)
        with relay.condition:
            cut_sample_us = relay.states[-1]["sample_us"]
            driven_volts = max(abs(volts) for *_, volts in
                               relay.states[-1]["motors"])

        failure = None
        try:
            unclean = scenario.stop([bridge, node])
            # Every command the RoboRIO had was built from a State the relay
            # passed before the cut, so the expiry is at most one timeout
            # past cut_sample_us; one more covers the stop and the settled
            # outputs.
            relay.wait(lambda: relay.states[-1]["sample_us"] >=
                       cut_sample_us + 2 * timeout_us,
                       "States through the expiry",
                       lambda: scenario.exited([gateway]))
            with relay.condition:
                if relay.forwarded_after_cut:
                    raise RuntimeError(f"the relay forwarded "
                                       f"{relay.forwarded_after_cut} "
                                       f"datagrams after the cut")
                if relay.driver_station_after_cut == 0:
                    raise RuntimeError("the Driver Station sent nothing after "
                                       "the main processor died")
                measured = judge_silence(list(relay.states),
                                         dict(relay.commands), timeout_us,
                                         status_hz)
            if unclean:
                raise RuntimeError(
                    f"not stopped cleanly within {STOP_SECONDS} s by SIGTERM "
                    f"(exit code; SIGKILLed past the bound): "
                    f"{', '.join(unclean)}")
        except RuntimeError as error:
            failure = error
        finally:
            relay.dump(output)
        # The robot's exit, a host sleep or a late tick explains a judgement
        # failure, so they are checked first and the judgement follows them.
        try:
            scenario.wait()
        except RuntimeError as error:
            if failure is None:
                raise
            raise RuntimeError(f"{error}; then: {failure}") from failure
        if failure is not None:
            raise RuntimeError(f"{failure} (see {output})") from failure

    print(f"silent main processor: driven at {driven_volts:.2f} V when it "
          f"died; neutral {measured['stop_after_observed_us']}"
          f" us after the last command's observed State (command_timeout_us "
          f"{timeout_us}), {measured['stop_after_expiry_us']} us past its "
          f"expiry, on a tick {measured['stop_tick_interval_us']} us after "
          f"the last driven one; motors at 0 V "
          f"{measured['zero_volts_after_stop_us']} us after the stop, "
          f"{measured['settled_states']} settled States all neutral")
    return output


def _exit_on_sigterm(signum, _frame):
    # Raised in the main thread, so every Scenario and Relay `with` block
    # stops its processes and closes its sockets on the way out.
    raise SystemExit(128 + signum)


def main():
    signal.signal(signal.SIGTERM, _exit_on_sigterm)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--wpilib",
        action="store_true",
        help="use the RoboRIO program under WPILib simulation as the gateway",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="create each scenario's log directory under this directory",
    )
    args = parser.parse_args()
    if args.output_dir is not None:
        args.output_dir.mkdir(parents=True, exist_ok=True)

    root = Path(__file__).resolve().parents[1]
    binaries = root / "bazel-bin" / SUBJECT_PACKAGE
    sim_gateway = (
        root / "bazel-bin/talOS/simulation/sim_gateway")

    needed = [binaries / "node", binaries / "send_target",
              root / "bazel-bin/talOS/bridge/hardware_node",
              root / SUBJECT_CONFIG]
    if not args.wpilib:
        needed.append(sim_gateway)
    for path in needed:
        if not path.is_file():
            raise RuntimeError(
                f"Build //{SUBJECT_PACKAGE}:all, "
                "//talOS/simulation:sim_gateway, and "
                "//talOS/bridge:hardware_node first")

    if args.wpilib:
        gradlew = root / "controller_processor/rio/gradlew"
        if not gradlew.is_file():
            raise RuntimeError(f"No Gradle wrapper at {gradlew}")

    # Holds the Mac awake for the run, including DarkWake maintenance sleep,
    # which a plain idle-sleep assertion (as Bazel takes) does not prevent;
    # -w ends it with this process, however that ends.
    caffeinate = None
    if sys.platform == "darwin":
        caffeinate = subprocess.Popen(
            ["caffeinate", "-s", "-i", "-w", str(os.getpid())])
    try:
        check_ports()
        output = run_driving_scenario(root, args.wpilib, args.output_dir)

        if args.wpilib:
            check_ports()
            run_disabled_scenario(root, args.output_dir)
            check_ports()
            run_silent_main_processor_scenario(root, args.output_dir)
            print(f"PASS (WPILib simulation): separate processes, motion, "
                  f"watchdog stop, replay, no motion while disabled, and the "
                  f"RoboRIO's own stop when the main processor dies. "
                  f"Logs: {output}")
        else:
            print(f"PASS (sim_gateway): separate processes, motion, watchdog "
                  f"stop, replay. Logs: {output}")
    finally:
        if caffeinate is not None:
            caffeinate.terminate()
            caffeinate.wait()


if __name__ == "__main__":
    main()
