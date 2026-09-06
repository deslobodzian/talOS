"""Run the loopback-only, multi-process swerve smoke test and replay its log.

Two gateways can stand at the RoboRIO end of the UDP link, and both are tested
the same way because both run the same `talos::hardware::Gateway`:

    python3 tools/test_drivetrain.py            # //2026-robot/main_processor/drivetrain:sim_gateway
    python3 tools/test_drivetrain.py --wpilib   # robot/, under WPILib simulation

The second one is the real RoboRIO program. It is slower to start, because it
goes through Gradle, but it exercises the actual `Robot` loop and the actual
Driver Station enable path, so it also runs a second scenario that the
stand-alone gateway cannot: with the Driver Station never enabled, the
drivetrain must not move at all.

First build: bazel build //2026-robot/main_processor/drivetrain:all
             (for --wpilib, Gradle builds the robot program itself)
Run from any directory.
"""

import argparse
import os
import re
import signal
import socket
import subprocess
import tempfile
import time
from pathlib import Path

# The gateway outlives the bridge and the bridge outlives the node, so the last
# thing the test observes is the gateway neutralizing its motors once commands
# stop arriving.
GATEWAY_SECONDS = 10
BRIDGE_SECONDS = 8
NODE_SECONDS = 6

# Gradle, a JVM and WPILib startup. Generous, because a cold Gradle daemon on a
# loaded machine is slow and starting the bridge early would fail the run for
# the wrong reason.
WPILIB_STARTUP_TIMEOUT = 180

SUMMARY = re.compile(r"^sim_(?:gateway|robot):.*$", re.MULTILINE)


class Scenario:
    """One full run of gateway + bridge + node, cleaned up on the way out."""

    def __init__(self, root, output):
        self.root = root
        self.binaries = root / "bazel-bin/2026-robot/main_processor/drivetrain"
        # The bridge is framework code and lives under talOS, not with the
        # robot's drivetrain. Pointing this at the drivetrain package silently
        # picks up a stale binary left over from before the split instead of
        # failing, so it is spelled out separately.
        self.bridge = root / "bazel-bin/talOS/bridge/hardware_node"
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
        self.start("gateway", [str(self.binaries / "sim_gateway"),
                               "--duration-s", str(seconds)])

    def start_wpilib_gateway(self, seconds, enabled=True, disable_after_ms=None):
        environment = dict(os.environ)
        # The simulated robot has no Driver Station of its own, and it must stop
        # on its own rather than be killed, so that its summary line is written.
        environment["TALOS_SIM_DS"] = "1" if enabled else "0"
        environment["TALOS_SIM_RUN_MS"] = str(seconds * 1000)
        if disable_after_ms is not None:
            environment["TALOS_SIM_DS_DISABLE_AFTER_MS"] = str(disable_after_ms)

        gateway = self.start(
            "gateway",
            # -Pheadless keeps the simulation GUI out of an automated run; the
            # TALOS_SIM_DS variables above stand in for it.
            ["./gradlew", "--offline", "--quiet", "-Pheadless",
             "simulateNativeRelease"],
            cwd=self.root / "robot", env=environment,
        )
        # The robot prints this once its gateway socket is open, which is the
        # only moment it is safe to start the bridge.
        self.wait_for("hardware gateway config=", gateway)

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
        self.start("bridge", [str(self.bridge),
                              "--duration-s", str(bridge_seconds)])
        self.start("node", [str(self.binaries / "node"), "--sim",
                            "--duration-s", str(node_seconds),
                            "--log", str(log_path)])
        time.sleep(0.5)
        self.start("send_target",
                   [str(self.binaries / "send_target"), "1", "0", "0", "2"])

    def wait(self):
        for child in self.children:
            if child.wait(timeout=WPILIB_STARTUP_TIMEOUT) != 0:
                raise RuntimeError(f"Process failed; see {self.output}")

    def summary(self):
        text = (self.output / "gateway.txt").read_text()
        found = SUMMARY.search(text)
        if not found:
            raise RuntimeError(f"The gateway printed no summary; see "
                               f"{self.output}")
        return found[0].strip()

    def replay(self, log_path):
        return subprocess.run(
            [str(self.binaries / "node"), "--sim", "--replay", str(log_path)],
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


def run_driving_scenario(root, wpilib):
    """The robot is enabled, is told to drive, and then loses its commands."""
    output = Path(tempfile.mkdtemp(prefix="talos-drive-"))
    log_path = output / "drive.tlog"

    with Scenario(root, output) as scenario:
        if wpilib:
            scenario.start_wpilib_gateway(GATEWAY_SECONDS)
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


def run_disabled_scenario(root):
    """The same drive request, with the Driver Station never enabled.

    Only the WPILib path can check this: sim_gateway hardcodes enable, so it
    cannot tell a working safety gate from a missing one.
    """
    output = Path(tempfile.mkdtemp(prefix="talos-disabled-"))

    with Scenario(root, output) as scenario:
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
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    binaries = root / "bazel-bin/2026-robot/main_processor/drivetrain"

    needed = [binaries / "node", binaries / "send_target",
              root / "bazel-bin/talOS/bridge/hardware_node"]
    if not args.wpilib:
        needed.append(binaries / "sim_gateway")
    for path in needed:
        if not path.is_file():
            raise RuntimeError(
                "Build //2026-robot/main_processor/drivetrain:all and "
                "//talOS/bridge:hardware_node first")

    check_ports()
    output = run_driving_scenario(root, args.wpilib)

    if args.wpilib:
        check_ports()
        run_disabled_scenario(root)
        print(f"PASS (WPILib simulation): separate processes, motion, watchdog "
              f"stop, replay, and no motion while disabled. Logs: {output}")
    else:
        print(f"PASS (sim_gateway): separate processes, motion, watchdog stop, "
              f"replay. Logs: {output}")


if __name__ == "__main__":
    main()
