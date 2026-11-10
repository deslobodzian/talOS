"""Run the loopback-only, multi-process swerve smoke test and replay its log.

Two gateways can stand at the RoboRIO end of the UDP link, and both are tested
the same way because both run the same `talos::hardware::Gateway`:

    python3 tools/test_drivetrain.py            # //2026-robot/main_processor/simulation:sim_gateway
    python3 tools/test_drivetrain.py --wpilib   # 2026-robot/controller_processor/rio/, under WPILib simulation

The second one is the real RoboRIO program. It is slower to start, because it
goes through Gradle, but it exercises the actual `Robot` loop and the actual
Driver Station enable path, so it also runs a second scenario that the
stand-alone gateway cannot: with the Driver Station never enabled, the
drivetrain must not move at all.

Logs go to a new directory under the system temporary directory, or under
--output-dir when given (CI keeps that directory when the run fails).

First build: bazel build //2026-robot/main_processor/drivetrain:all \
                         //2026-robot/main_processor/simulation:sim_gateway \
                         //talOS/bridge:hardware_node
             (for --wpilib, Gradle builds the robot program itself)
Run from any directory.
"""

import argparse
import os
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# The gateway outlives the bridge and the bridge outlives the node, so the last
# thing the test observes is the gateway neutralizing its motors once commands
# stop arriving.
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
# HAL_SendError, "Error at talos loop watchdog: ...").
LOOP_TIMING = re.compile(
    r"\[rio\] loop timing: (\d+) overruns .*?, (\d+) late ticks .*"
    r"max interval (\d+) us")
LOOP_STALLED = "talos loop watchdog: RoboRIO loop stalled"


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

    def __init__(self, root, output, wpilib=False):
        self.root = root
        self.wpilib = wpilib
        self.slept_at_start = host_sleep_seconds()
        self.binaries = root / "bazel-bin/2026-robot/main_processor/drivetrain"
        # sim_gateway moved out of the drivetrain package into simulation, so
        # it is spelled out separately: pointing it at the drivetrain package
        # silently picks up a stale binary left over from before the split
        # instead of failing.
        self.sim_gateway = (
            root / "bazel-bin/2026-robot/main_processor/simulation/sim_gateway")
        # The bridge is framework code and lives under talOS, not with the
        # robot's drivetrain. Pointing this at the drivetrain package silently
        # picks up a stale binary left over from before the split instead of
        # failing, so it is spelled out separately.
        self.bridge = root / "bazel-bin/talOS/bridge/hardware_node"
        self.config = (
            root / "2026-robot/main_processor/configuration/robot.toml")
        self.output = output
        self.children = []
        self.files = []

    def start(self, name, argv, cwd=None, env=None):
        log = (self.output / (name + ".txt")).open("w")
        self.files.append(log)
        # Its own process group: Gradle forks the simulated robot, and killing
        # Gradle alone would leave that robot holding port 5802.
        process = subprocess.Popen(
            argv, stdout=log, stderr=subprocess.STDOUT,
            cwd=cwd or self.root, env=env, start_new_session=True,
        )
        self.children.append(process)
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
            # drivetrain/hardware/protocol READMEs), run from the Gradle
            # project that owns the RoboRIO program.
            ["./gradlew", "--offline", "--quiet", "-Pheadless",
             "simulateNative"],
            cwd=self.root / "2026-robot/controller_processor/rio",
            env=environment,
        )
        # The robot prints this once its gateway socket is open, which is the
        # only moment it is safe to start the bridge.
        self.wait_for("hardware gateway config=", gateway)
        return gateway

    def wait_for(self, needle, process):
        path = self.output / "gateway.txt"
        deadline = time.monotonic() + WPILIB_STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            if path.exists() and needle in path.read_text():
                return
            if process.poll() is not None:
                raise RuntimeError(f"Gateway exited early; see {self.output}")
            time.sleep(0.25)
        raise RuntimeError(f"Gateway never started; see {self.output}")

    def drive(self, log_path, bridge_seconds, node_seconds):
        self.start_bridge(bridge_seconds)
        self.start_node(log_path, node_seconds)

    def start_bridge(self, bridge_seconds):
        # --config is required: without it the bridge exits with a usage error
        # before any simulation traffic exists. --sim, as on the node, treats
        # the config as commissioned; one without the other would give them
        # different config ids, and the node would drop every State.
        self.start("bridge", [str(self.bridge), "--sim",
                              "--config", str(self.config),
                              "--duration-s", str(bridge_seconds)])

    def start_node(self, log_path, node_seconds):
        # The node requires --config like the bridge does.
        self.start("node", [str(self.binaries / "node"), "--sim",
                            "--config", str(self.config),
                            "--duration-s", str(node_seconds),
                            "--log", str(log_path)])
        time.sleep(0.5)
        self.start("send_target",
                   [str(self.binaries / "send_target"), "1", "0", "0", "2"])

    def wait(self):
        failed = False
        for child in self.children:
            if child.wait(timeout=WPILIB_STARTUP_TIMEOUT) != 0:
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


def run_driving_scenario(root, wpilib, output_root):
    """The robot is enabled, is told to drive, and then loses its commands."""
    output = Path(tempfile.mkdtemp(prefix="talos-drive-", dir=output_root))
    log_path = output / "drive.tlog"

    with Scenario(root, output, wpilib) as scenario:
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
            raise RuntimeError("The simulated drivetrain did not move")
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
    (`--mode-schedule`) are covered by simulation:startup_test.
    """
    output = Path(tempfile.mkdtemp(prefix="talos-disabled-", dir=output_root))

    with Scenario(root, output, wpilib=True) as scenario:
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


def main():
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
    binaries = root / "bazel-bin/2026-robot/main_processor/drivetrain"
    sim_gateway = (
        root / "bazel-bin/2026-robot/main_processor/simulation/sim_gateway")

    needed = [binaries / "node", binaries / "send_target",
              root / "bazel-bin/talOS/bridge/hardware_node",
              root / "2026-robot/main_processor/configuration/robot.toml"]
    if not args.wpilib:
        needed.append(sim_gateway)
    for path in needed:
        if not path.is_file():
            raise RuntimeError(
                "Build //2026-robot/main_processor/drivetrain:all, "
                "//2026-robot/main_processor/simulation:sim_gateway, and "
                "//talOS/bridge:hardware_node first")

    if args.wpilib:
        gradlew = root / "2026-robot/controller_processor/rio/gradlew"
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
            print(f"PASS (WPILib simulation): separate processes, motion, "
                  f"watchdog stop, replay, and no motion while disabled. "
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
