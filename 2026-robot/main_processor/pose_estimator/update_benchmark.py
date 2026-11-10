"""Wall-clock duration of PoseEstimator.update() under a synthetic match load.

    bazel run //2026-robot/main_processor/pose_estimator:update_benchmark -- \
        [--backend kalman|factor_graph|both] [--seconds N]

Runs the real PoseEstimator on talos.testing.Sim with the production
config's pose_estimator settings (only `backend` replaced), the drivetrain
publishing at its configured period and every configured camera at ~60 Hz,
and times each update() with time.perf_counter_ns. Exits non-zero when the
workload was not the intended one: an observation or odometry step rejected,
or the estimate not valid and localized at the end. Timing is reported, never
judged. See README.md "Update benchmark".
"""

import argparse
import bisect
import importlib.metadata
import math
import os
import platform
import random
import re
import shutil
import subprocess
import sys
import tempfile
import time
import tomllib

import talos
from drivetrain import topics as drivetrain_topics
from drivetrain.DrivetrainState import DrivetrainState
from geometry.PoseEstimate import PoseEstimate
from geometry.PoseObservation import PoseObservation
from pose_estimator import topics
from python.runfiles import runfiles
from talos.testing import Sim, SimHardware

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = "_main/2026-robot/main_processor/configuration/robot.toml"
BACKENDS = ("kalman", "factor_graph")

STEP_NS = 1_000_000  # simulation step: camera arrival resolution
CAMERA_HZ = 60.0
CAMERA_JITTER_S = 0.0005
# Capture to arrival: exposure, readout, detection and transport. A frame is
# delayed further with DELAY_PROBABILITY (a slow detection, a USB hiccup) and
# may then arrive after newer frames: delayed observations attach to past keys.
LATENCY_S = (0.020, 0.035)
DELAY_PROBABILITY = 0.15
DELAY_S = (0.020, 0.100)
# Per-frame measurement noise, reported as the observation's std devs. The
# yaw bound stays far inside the production yaw innovation gate.
STD_XY_M = (0.02, 0.08)
STD_YAW_RAD = (0.01, 0.03)
ODOMETRY_SLIP_STD = 0.02  # per wheel and drivetrain sample
GYRO_NOISE_RAD = 0.0005
SEED = 5687


class Path:
    """Closed-form chassis truth from the configured initial pose: a
    Lissajous sweep inside the field (up to ~3.7 m/s wheel speed) while the
    heading swings +-1.5 rad (up to 1.35 rad/s)."""

    def __init__(self, x0, y0, yaw0, field_length_m, field_width_m):
        self.x0, self.y0, self.yaw0 = x0, y0, yaw0
        self.ax = max(0.0, min(3.0, (field_length_m - x0 - 1.0) / 2))
        self.ay = max(0.0, min(1.5, (field_width_m - y0 - 1.0) / 2))
        self.wx, self.wy = 0.8, 1.3
        self.yaw_amplitude, self.wyaw = 1.5, 0.9

    def pose(self, t):
        return (self.x0 + self.ax * (1 - math.cos(self.wx * t)),
                self.y0 + self.ay * (1 - math.cos(self.wy * t)),
                self.yaw0 + self.yaw_amplitude * math.sin(self.wyaw * t))

    def robot_velocity(self, t):
        """(vx, vy, omega) in the robot frame."""
        fx = self.ax * self.wx * math.sin(self.wx * t)
        fy = self.ay * self.wy * math.sin(self.wy * t)
        yaw = self.pose(t)[2]
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * fx + s * fy, -s * fx + c * fy,
                self.yaw_amplitude * self.wyaw * math.cos(self.wyaw * t))


class Drivetrain:
    """Publishes what the drivetrain reports: wheel distances integrated
    from the path with per-wheel slip noise, and a noisy gyro."""

    def __init__(self, sim, path, modules, rng):
        self.sim, self.path, self.modules, self.rng = sim, path, modules, rng
        self.distances = [0.0] * len(modules)
        self.angles = [0.0] * len(modules)
        self.last_t = None

    def publish(self, t):
        if self.last_t is not None:
            dt = t - self.last_t
            vx, vy, omega = self.path.robot_velocity(self.last_t + dt / 2)
            for i, (mx, my) in enumerate(self.modules):
                wx, wy = vx - omega * my, vy + omega * mx
                speed = math.hypot(wx, wy)
                if speed > 0.0:
                    self.angles[i] = math.atan2(wy, wx)
                self.distances[i] += speed * dt * (
                    1.0 + self.rng.gauss(0.0, ODOMETRY_SLIP_STD))
        self.last_t = t
        vx, vy, omega = self.path.robot_velocity(t)
        fields = dict(
            sample_time_ns=self.sim_ns(), enabled=False, modules_valid=True,
            yaw_valid=True, vx_mps=vx, vy_mps=vy, omega_radps=omega,
            yaw_rad=self.path.pose(t)[2] + self.rng.gauss(0.0, GYRO_NOISE_RAD))
        for i in range(len(self.modules)):
            fields[f"module{i}_distance_m"] = self.distances[i]
            fields[f"module{i}_angle_rad"] = self.angles[i]
        self.sim.publish(DrivetrainState, drivetrain_topics.STATE, **fields)

    def sim_ns(self):
        return round(self.sim.now * 1e9)


class Camera:
    """One camera: frames captured at ~60 Hz, published on arrival."""

    def __init__(self, sim, name, path, rng, first_capture_s, t0):
        self.sim, self.topic, self.path, self.rng = sim, f"/{name}/state", path, rng
        self.t0 = t0
        self.next_capture_s = first_capture_s + rng.uniform(0.0, 1.0 / CAMERA_HZ)
        self.pending = []  # (arrival_s, capture_s, fields), sorted

    def poll(self, now_s):
        while self.next_capture_s <= now_s:
            capture_s = self.next_capture_s
            latency = self.rng.uniform(*LATENCY_S)
            if self.rng.random() < DELAY_PROBABILITY:
                latency += self.rng.uniform(*DELAY_S)
            bisect.insort(self.pending, (capture_s + latency, capture_s,
                                         self.frame(capture_s)))
            self.next_capture_s += 1.0 / CAMERA_HZ + self.rng.gauss(
                0.0, CAMERA_JITTER_S)
        while self.pending and self.pending[0][0] <= now_s:
            self.sim.publish(PoseObservation, self.topic,
                             **self.pending.pop(0)[2])

    def frame(self, capture_s):
        x, y, yaw = self.path.pose(capture_s - self.t0)
        std_xy = self.rng.uniform(*STD_XY_M)
        std_yaw = self.rng.uniform(*STD_YAW_RAD)
        return dict(
            sample_time_ns=round(capture_s * 1e9),
            pose_x_m=x + self.rng.gauss(0.0, std_xy),
            pose_y_m=y + self.rng.gauss(0.0, std_xy),
            pose_yaw_rad=yaw + self.rng.gauss(0.0, std_yaw),
            x_std_m=std_xy, y_std_m=std_xy, yaw_std_rad=std_yaw,
            tag_count=self.rng.randint(1, 3), valid=True)


def timed(estimator_class):
    class TimedPoseEstimator(estimator_class):
        """Records (sim time, update() ns, observations accepted) per tick."""

        def __init__(self, node):
            super().__init__(node)
            self.ticks = []

        def update(self):
            accepted = self.accepted
            start = time.perf_counter_ns()
            super().update()
            elapsed = time.perf_counter_ns() - start
            self.ticks.append(
                (self.node.now, elapsed, self.accepted - accepted))

    return TimedPoseEstimator


def production_config(directory, backend):
    """Copies the production robot config into `directory` with the
    pose_estimator backend replaced; returns (robot.toml, pose_estimator
    settings, drivetrain period_us)."""
    source = runfiles.Create().Rlocation(CONFIG)
    shutil.copytree(os.path.dirname(source), directory, dirs_exist_ok=True)
    subsystems = os.path.join(directory, "subsystems")
    pose_path = os.path.join(subsystems, "pose_estimator.toml")
    with open(pose_path, encoding="utf-8") as f:
        text, count = re.subn(r'^backend = "\w+"$', f'backend = "{backend}"',
                              f.read(), flags=re.MULTILINE)
    if count != 1:
        raise SystemExit(f"update_benchmark: expected one backend line in "
                         f"{pose_path}, found {count}")
    os.chmod(pose_path, 0o644)
    with open(pose_path, "w", encoding="utf-8") as f:
        f.write(text)
    settings = tomllib.loads(text)["subsystem"]
    with open(os.path.join(subsystems, "drivetrain.toml"), "rb") as f:
        drivetrain_period_us = tomllib.load(f)["subsystem"]["period_us"]
    return os.path.join(directory, "robot.toml"), settings, drivetrain_period_us


def run(backend, seconds, estimator_class):
    """Runs one backend; returns (steady ticks, warm-up ticks, workload)."""
    with tempfile.TemporaryDirectory(prefix="pose_benchmark_") as directory:
        config, settings, drivetrain_period_us = production_config(
            directory, backend)
        return _run(Sim(config), settings, drivetrain_period_us, seconds,
                    backend, estimator_class)


def _run(sim, settings, drivetrain_period_us, seconds, backend,
         estimator_class):
    node = sim.add(timed(estimator_class), "pose_estimator")
    hw = SimHardware(sim)
    hw.enable(False)
    # Disabled-first Driver Station: the configured seed is trusted, so every
    # observation is fused against a localized estimate.
    hw.set_driver_station(enabled=False, teleop=True, attached=True)
    rng = random.Random(SEED)
    path = Path(settings.get("initial_x_m", 0.0), settings.get("initial_y_m", 0.0),
                settings.get("initial_yaw_rad", 0.0), settings["field_length_m"],
                settings["field_width_m"])
    modules = list(zip(settings["module_x_m"], settings["module_y_m"]))
    drivetrain = Drivetrain(sim, path, modules, rng)
    period_s = settings["period_us"] / 1e6
    lag_s = settings["max_observation_age_us"] / 1e6

    sim.step(STEP_NS / 1e9)  # sample time 0 means "unknown"
    t0 = sim.now
    # Frames start once the estimator has integrated its first sample: an
    # earlier one would be rejected as predating odometry.
    cameras = [Camera(sim, name, path, rng, t0 + 2 * period_s, t0)
               for name in settings["cameras"]]
    drive_steps = drivetrain_period_us * 1000 // STEP_NS
    for step in range(round(seconds * 1e9 / STEP_NS)):
        now = sim.now
        if step % drive_steps == 0:
            drivetrain.publish(now - t0)
        for camera in cameras:
            camera.poll(now)
        sim.step(STEP_NS / 1e9)

    estimate = sim.latest(PoseEstimate, topics.STATE)
    problems = []
    if node.rejected or node.odometry_rejected_count:
        problems.append(f"{node.rejected} observation(s) and "
                        f"{node.odometry_rejected_count} odometry step(s) "
                        f"rejected")
    if estimate is None or not (estimate.valid and estimate.localized):
        problems.append("the estimate is not valid and localized")
    if not node.accepted:
        problems.append("no observation was accepted")
    if problems:
        raise SystemExit(f"update_benchmark: {backend}: the workload is not "
                         f"the intended one: {'; '.join(problems)}")
    workload = dict(cameras=len(cameras), period_s=period_s, lag_s=lag_s,
                    drivetrain_hz=1e6 / drivetrain_period_us,
                    error_m=math.dist(
                        (estimate.pose_x_m, estimate.pose_y_m),
                        path.pose(estimate.sample_time_ns / 1e9 - t0)[:2]))
    # The lag window fills during the first lag_s: those ticks are warm-up.
    start = t0 + lag_s
    return ([t for t in node.ticks if t[0] >= start],
            [t for t in node.ticks if t[0] < start], workload)


def percentile(sorted_values, q):
    """Nearest rank."""
    return sorted_values[max(0, math.ceil(q * len(sorted_values)) - 1)]


def cpu_model():
    system = platform.system()
    try:
        if system == "Darwin":
            return subprocess.run(
                ["sysctl", "-n", "machdep.cpu.brand_string"], check=True,
                capture_output=True, text=True).stdout.strip()
        if system == "Linux":
            with open("/proc/cpuinfo", encoding="utf-8") as f:
                for line in f:
                    if line.startswith("model name"):
                        return line.split(":", 1)[1].strip()
    except (OSError, subprocess.CalledProcessError):
        pass
    return platform.processor() or "unknown"


def versions():
    found = []
    for package in ("gtsam", "robotpy-wpimath", "numpy"):
        try:
            found.append(f"{package} {importlib.metadata.version(package)}")
        except importlib.metadata.PackageNotFoundError:
            found.append(f"{package} unknown")
    return ", ".join(found)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--backend", choices=BACKENDS + ("both",),
                        default="both")
    parser.add_argument("--seconds", type=float, default=900.0,
                        help="simulated seconds per backend (default 900)")
    args = parser.parse_args(argv)
    if not args.seconds > 0:
        parser.error("--seconds must be positive")
    estimator = talos.load(os.path.join(HERE, "pose_estimator.py"))

    print(f"host: {platform.platform()} | {cpu_model()} | "
          f"{platform.python_implementation()} {platform.python_version()} | "
          f"{os.cpu_count()} CPUs")
    print(f"packages: {versions()}")
    rows = []
    for backend in BACKENDS if args.backend == "both" else (args.backend,):
        ticks, warmup, workload = run(backend, args.seconds,
                                      estimator.PoseEstimator)
        rows.append((backend, ticks, warmup, workload))
    workload = rows[0][3]
    print(f"workload: {args.seconds:g} s simulated per backend; drivetrain "
          f"{workload['drivetrain_hz']:g} Hz; {workload['cameras']} cameras "
          f"~{CAMERA_HZ:g} Hz, capture-to-arrival {LATENCY_S[0] * 1e3:g}-"
          f"{LATENCY_S[1] * 1e3:g} ms, {DELAY_PROBABILITY:.0%} of frames "
          f"+{DELAY_S[0] * 1e3:g}-{DELAY_S[1] * 1e3:g} ms; period "
          f"{workload['period_s'] * 1e3:g} ms; lag {workload['lag_s']:g} s")
    budget_ns = workload["period_s"] * 1e9
    print(f"{'backend':<13}{'ticks':>7}{'p50 ms':>9}{'p99 ms':>9}"
          f"{'p99.9 ms':>10}{'max ms':>9}{'>period':>9}{'obs/tick':>10}"
          f"{'4-obs':>8}{'warm-up max ms':>16}{'error m':>9}")
    for backend, ticks, warmup, workload in rows:
        durations = sorted(t[1] for t in ticks)
        over = sum(d > budget_ns for d in durations)
        observations = [t[2] for t in ticks]
        print(f"{backend:<13}{len(durations):>7}"
              f"{percentile(durations, 0.5) / 1e6:>9.3f}"
              f"{percentile(durations, 0.99) / 1e6:>9.3f}"
              f"{percentile(durations, 0.999) / 1e6:>10.3f}"
              f"{durations[-1] / 1e6:>9.3f}"
              f"{over / len(durations):>9.3%}"
              f"{sum(observations) / len(observations):>10.2f}"
              f"{sum(o >= 4 for o in observations) / len(observations):>8.1%}"
              f"{max((t[1] for t in warmup), default=0) / 1e6:>16.3f}"
              f"{workload['error_m']:>9.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
