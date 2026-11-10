"""Run a physical robot session and score fused/replayed poses via log_dump."""

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import tempfile
import time
import tomllib
import uuid

from python.runfiles import runfiles


CONFIG = "2026-robot/main_processor/configuration/robot_physics_sim.toml"
ESTIMATE = "/pose_estimator/state"
TRUTH = "/sim/state"
DRIVETRAIN = "/drivetrain/state"
CAMERAS = ("front_left_camera", "front_right_camera",
           "rear_left_camera", "rear_right_camera")
# Deterministic, camera-specific true-minus-believed NWU mounting errors.
# Keep the moderate preset's magnitudes and sign pattern in this one source.
MOUNT_ERROR_PRESET = {
    name: {
        "sim_mount_error_x_m": 0.01 * signs[0],
        "sim_mount_error_y_m": 0.01 * signs[1],
        "sim_mount_error_z_m": 0.01 * signs[2],
        "sim_mount_error_pitch_rad": math.radians(0.5) * signs[3],
        "sim_mount_error_yaw_rad": math.radians(1.0) * signs[4],
    }
    for name, signs in zip(CAMERAS, (
        (+1, -1, +1, +1, +1),
        (-1, +1, +1, -1, +1),
        (+1, +1, -1, +1, -1),
        (-1, -1, -1, -1, -1),
    ))
}
# At the routine's 2 m/s limit, 40 ms is 8 cm: above the 2 cm xy floor.
TIMESTAMP_LAG_US = 40_000
# Report row order; each row is scored once per backend that produced it.
RUNS = ("odometry-only", "fused", "fused-live", "fused-lag-uncorrected",
        "fused-lag-corrected")
# Every pose_estimator `backend`; each replays the same recorded sessions.
BACKENDS = ("kalman", "factor_graph")


@dataclass(frozen=True)
class CameraMetrics:
    samples: int
    normalized_rms: tuple[float, float, float]
    normalized_abs_p95: tuple[float, float, float]
    yaw_rms_deg: float
    consistent_yaw_floor_rad: float | None


@dataclass(frozen=True)
class CameraTiming:
    frame_period_us: int
    dropped: int
    captured: int
    processed: int


@dataclass(frozen=True)
class Metrics:
    samples: int
    position_rms_m: float
    position_max_m: float
    heading_rms_deg: float
    heading_max_deg: float
    final_position_m: float
    final_heading_deg: float
    accepted: int
    rejected: int


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def decode_struct(text):
    # log_dump's reflection decoder prints numeric/bool structs with bare keys.
    # These three schemas contain no strings or enums; json retains uint64 ints.
    return json.loads(re.sub(r"\b([A-Za-z_]\w*)\s*:", r'"\1":', text))


def pose(message):
    p = message["pose"]
    return p["x_m"], p["y_m"], p["yaw_rad"]


def interpolate_truth(times, poses, timestamp):
    """Linear x/y and shortest-arc yaw interpolation; never extrapolate."""
    i = bisect_left(times, timestamp)
    if i < len(times) and times[i] == timestamp:
        return poses[i]
    if i == 0 or i == len(times):
        return None
    fraction = (timestamp - times[i - 1]) / (times[i] - times[i - 1])
    a, b = poses[i - 1], poses[i]
    return (a[0] + fraction * (b[0] - a[0]),
            a[1] + fraction * (b[1] - a[1]),
            wrap(a[2] + fraction * wrap(b[2] - a[2])))


def score(records):
    """Score valid samples while the drivetrain is freshly enabled (auto here)."""
    truth = {}
    modes = {}
    estimates = {}
    for row in records:
        topic = row["topic"]
        if topic == TRUTH and row["kind"] == "MESSAGE":
            message = decode_struct(row["decoded"])
            truth[int(message["sample_time_ns"])] = pose(message)
        elif topic == DRIVETRAIN and row["kind"] == "SEND":
            modes[row["event_ms"]] = decode_struct(row["decoded"])["enabled"]
        elif topic == ESTIMATE and row["kind"] == "SEND":
            message = decode_struct(row["decoded"])
            estimates[int(message["sample_time_ns"])] = (row["event_ms"], message)
    if not truth or not modes or not estimates:
        raise ValueError("missing truth, drivetrain, or estimator records")
    truth_times = sorted(truth)
    truth_poses = [truth[t] for t in truth_times]
    mode_times = sorted(modes)
    positions, headings = [], []
    accepted = rejected = 0
    for timestamp, (event_ms, message) in sorted(estimates.items()):
        mode_i = bisect_right(mode_times, event_ms) - 1
        if mode_i < 0 or event_ms - mode_times[mode_i] >= 100:
            continue
        if not modes[mode_times[mode_i]] or not message["valid"]:
            continue
        actual = interpolate_truth(truth_times, truth_poses, timestamp)
        if actual is None:
            continue
        estimate = pose(message)
        values = estimate + actual
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite pose in valid estimator/truth sample")
        positions.append(math.hypot(estimate[0] - actual[0],
                                    estimate[1] - actual[1]))
        headings.append(abs(math.degrees(wrap(estimate[2] - actual[2]))))
        accepted = int(message["observation_accepted_count"])
        rejected = int(message["observation_rejected_count"])
    if not positions:
        raise ValueError("no valid, truth-bracketed estimates during autonomous")
    return Metrics(
        len(positions), math.sqrt(sum(e * e for e in positions) / len(positions)),
        max(positions), math.sqrt(sum(e * e for e in headings) / len(headings)),
        max(headings), positions[-1], headings[-1], accepted, rejected)


def percentile95(values):
    """Linearly interpolated 95th percentile of absolute normalized errors."""
    ordered = sorted(abs(value) for value in values)
    index = 0.95 * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    return ordered[lower] + (index - lower) * (ordered[upper] - ordered[lower])


def score_cameras(records):
    """Score each valid camera sample once, against sample-time physical truth."""
    truth, modes = {}, {}
    observations = {name: {} for name in CAMERAS}
    camera_topics = {f"/{name}/state": name for name in CAMERAS}
    for row in records:
        topic = row["topic"]
        if topic == TRUTH and row["kind"] == "MESSAGE":
            message = decode_struct(row["decoded"])
            truth[int(message["sample_time_ns"])] = pose(message)
        elif topic == DRIVETRAIN and row["kind"] == "SEND":
            modes[row["event_ms"]] = decode_struct(row["decoded"])["enabled"]
        elif topic in camera_topics and row["kind"] == "SEND":
            message = decode_struct(row["decoded"])
            observations[camera_topics[topic]][int(message["sample_time_ns"])] = (
                row["event_ms"], message)
    if not truth or not modes:
        raise ValueError("missing truth or drivetrain records for camera scoring")
    truth_times = sorted(truth)
    truth_poses = [truth[t] for t in truth_times]
    mode_times = sorted(modes)
    results = {}
    for name, samples in observations.items():
        normalized = [[], [], []]
        yaw_errors, yaw_stds = [], []
        for timestamp, (event_ms, message) in sorted(samples.items()):
            mode_i = bisect_right(mode_times, event_ms) - 1
            if (mode_i < 0 or event_ms - mode_times[mode_i] >= 100 or
                    not modes[mode_times[mode_i]] or not message["valid"]):
                continue
            actual = interpolate_truth(truth_times, truth_poses, timestamp)
            if actual is None:
                continue
            observed = pose(message)
            stds = (message["x_std_m"], message["y_std_m"],
                    message["yaw_std_rad"])
            if (not all(math.isfinite(v) for v in observed + actual + stds) or
                    any(std <= 0 for std in stds)):
                raise ValueError(f"invalid pose/std dev in valid {name} sample")
            errors = (observed[0] - actual[0], observed[1] - actual[1],
                      wrap(observed[2] - actual[2]))
            for axis in range(3):
                normalized[axis].append(errors[axis] / stds[axis])
            yaw_errors.append(errors[2])
            yaw_stds.append(stds[2])
        if not yaw_errors:
            continue
        count = len(yaw_errors)
        rms = tuple(math.sqrt(sum(e * e for e in axis) / count)
                    for axis in normalized)
        yaw_rms = math.sqrt(sum(e * e for e in yaw_errors) / count)
        floor = None
        if rms[2] > 1:
            # The smallest additional floor making these observed yaw errors
            # consistent, retaining every covariance std already above it.
            low, high = 0.0, max(abs(e) for e in yaw_errors)
            for _ in range(60):
                middle = (low + high) / 2
                mean_square = sum(
                    (error / max(std, middle)) ** 2
                    for error, std in zip(yaw_errors, yaw_stds)) / count
                if mean_square > 1:
                    low = middle
                else:
                    high = middle
            floor = high
        results[name] = CameraMetrics(
            count, rms, tuple(percentile95(axis) for axis in normalized),
            math.degrees(yaw_rms), floor)
    return results


def camera_markdown(results):
    lines = [
        "| Camera | Samples | x normalized RMS | x abs p95 | y normalized RMS | y abs p95 | yaw normalized RMS | yaw abs p95 | Raw yaw RMS (deg) | Consistent yaw floor (rad) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name in CAMERAS:
        if name not in results:
            lines.append(f"| {name} | 0 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a |")
            continue
        metric = results[name]
        axes = " | ".join(
            f"{metric.normalized_rms[i]:.3f} | {metric.normalized_abs_p95[i]:.3f}"
            for i in range(3))
        floor = ("n/a" if metric.consistent_yaw_floor_rad is None else
                 f"{metric.consistent_yaw_floor_rad:.4f}")
        lines.append(f"| {name} | {metric.samples} | {axes} | "
                     f"{metric.yaw_rms_deg:.3f} | {floor} |")
    return "\n".join(lines)


def camera_timing(records):
    """Final per-camera cadence/loss counters, including disabled startup."""
    topics = {f"/{name}/status": name for name in CAMERAS}
    latest = {}
    for record in records:
        name = topics.get(record["topic"])
        if name is None or record["kind"] != "SEND":
            continue
        if name not in latest or record["event_ms"] >= latest[name][0]:
            latest[name] = (record["event_ms"], decode_struct(record["decoded"]))
    if len(latest) != len(CAMERAS):
        raise ValueError("missing camera status records")
    return {
        name: CameraTiming(
            int(message["frame_period_us"]),
            int(message["frames_dropped_count"]),
            int(message["frames_captured_count"]),
            int(message["frames_processed_count"]))
        for name, (_, message) in latest.items()}


def camera_health_issues(timings):
    """Require >=40 of the requested 60 fps and >=95% processing per camera.

    The deadline counters cover the whole session, unlike estimator accepts,
    which also depend on visibility and innovation gating. This prerequisite
    is independent of whether fusion happened to beat odometry.
    """
    issues = []
    for camera in CAMERAS:
        timing = timings[camera]
        scheduled = timing.captured + timing.dropped
        if (not scheduled or timing.captured * 3 < scheduled * 2
                or not 0 < timing.frame_period_us <= 25_000
                or not timing.captured
                or timing.processed * 20 < timing.captured * 19):
            issues.append(
                f"{camera}: period={timing.frame_period_us} us, "
                f"captured={timing.captured}/{scheduled} deadlines, "
                f"processed={timing.processed}/{timing.captured}")
    return issues


def timing_markdown(results):
    lines = [
        "| Run | Camera | frame_period_us | Dropped | Captured | Processed |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for name in ("fused-live", "fused-lag-uncorrected", "fused-lag-corrected"):
        for camera in CAMERAS:
            timing = results[name][camera]
            lines.append(
                f"| {name} | {camera} | {timing.frame_period_us} | "
                f"{timing.dropped} | {timing.captured} | {timing.processed} |")
    return "\n".join(lines)


def rows(results):
    """(run, backend) rows in RUNS order, with a run's backends adjacent."""
    return [(run, backend) for run in RUNS for backend in BACKENDS
            if (run, backend) in results["metrics"]]


def markdown(results):
    lines = [
        "| Run | Backend | Samples | Position RMS (m) | Position max (m) | Heading RMS (deg) | Heading max (deg) | Final position (m) | Final heading (deg) | Accepted | Rejected |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for run, backend in rows(results):
        m = results["metrics"][(run, backend)]
        lines.append(
            f"| {run} | {backend} | {m.samples} | {m.position_rms_m:.4f} | "
            f"{m.position_max_m:.4f} | {m.heading_rms_deg:.3f} | "
            f"{m.heading_max_deg:.3f} | {m.final_position_m:.4f} | "
            f"{m.final_heading_deg:.3f} | {m.accepted} | {m.rejected} |")
    if "cameras" in results:
        lines.extend(["", camera_markdown(results["cameras"])])
    if "timing" in results:
        lines.extend(["", timing_markdown(results["timing"])])
        issues = camera_health_issues(results["timing"]["fused-live"])
        health = ("Fusion improvement NOT EVALUATED: INSUFFICIENT camera "
                  "throughput: " + "; ".join(issues)) if issues else (
                      "Camera throughput sufficient; fusion improvement required.")
        lines.extend(["", health])
    if "runtime_s" in results:
        lines.extend(["", f"Total runtime: {results['runtime_s']:.1f} s"])
    return "\n".join(lines)


def summary_markdown(scenarios):
    """Summarize independent sessions; never average away an individual report."""
    if not scenarios:
        raise ValueError("at least one scenario is required for a summary")
    lines = [
        "| Run | Backend | Position RMS mean (m) | min (m) | max (m) | Heading RMS mean (deg) | min (deg) | max (deg) |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows(scenarios[0]):
        cells = list(row)
        for field, precision in (("position_rms_m", 4),
                                 ("heading_rms_deg", 3)):
            values = [getattr(result["metrics"][row], field)
                      for result in scenarios]
            cells.extend(f"{value:.{precision}f}" for value in
                         (sum(values) / len(values), min(values), max(values)))
        lines.append("| " + " | ".join(cells) + " |")
    elapsed = sum(result.get("runtime_s", 0) for result in scenarios)
    lines.extend(["", f"Scenarios: {len(scenarios)}; total runtime: {elapsed:.1f} s"])
    return "\n".join(lines)


def udp_ports():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        return gateway.getsockname()[1], bridge.getsockname()[1]


def run_launcher(binary, args, cwd, env, diagnostic, timeout):
    with diagnostic.open("w+", encoding="utf-8") as log:
        with subprocess.Popen(
                [str(binary)] + args, cwd=cwd, env=env, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True) as proc:
            try:
                proc.wait(timeout=timeout)
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
            log.seek(0)
            if proc.returncode:
                raise RuntimeError(f"launcher failed ({proc.returncode}):\n{log.read()}")


def session_directory(root):
    manifests = list(root.glob("*/manifest.json"))
    if len(manifests) != 1:
        raise RuntimeError(f"expected one session under {root}, got {len(manifests)}")
    manifest = json.loads(manifests[0].read_text(encoding="utf-8"))
    for node in manifest["nodes"]:
        if node["exit_code"] != 0 or node["recorder_failed"] or node["restart_count"]:
            raise RuntimeError(f"unclean session node: {node}")
    return manifests[0].parent


def dump_records(binary, cameras, drivetrain, estimator, env):
    topics = [TRUTH, DRIVETRAIN, ESTIMATE]
    topics.extend(f"/{name}/state" for name in CAMERAS)
    topics.extend(f"/{name}/status" for name in CAMERAS)
    proc = subprocess.run(
        [str(binary), "--format", "jsonl", "--topic", ",".join(topics),
         *(str(camera) for camera in cameras), str(drivetrain), str(estimator)],
        env=env, capture_output=True, text=True, timeout=60, check=True)
    return [json.loads(line) for line in proc.stdout.splitlines()]


def overlay_config(physics_config, subsystem_overrides):
    """Flatten the physics include, preserving its delta and overlaying nodes."""
    overrides = tomllib.loads(physics_config.read_text(encoding="utf-8"))
    base = physics_config.parent / overrides.pop("include")
    for name, values in subsystem_overrides.items():
        target = overrides.setdefault("subsystems", {}).setdefault(name, {})
        for key, value in values.items():
            if isinstance(value, dict):
                target.setdefault(key, {}).update(value)
            else:
                target[key] = value

    def tables(values, path=()):
        lines = []
        if path:
            lines.append("[" + ".".join(json.dumps(key) for key in path) + "]")
        for key, value in values.items():
            if not isinstance(value, dict):
                lines.append(f"{json.dumps(key)} = {json.dumps(value)}")
        for key, value in values.items():
            if isinstance(value, dict):
                lines.extend(tables(value, path + (key,)))
        return lines

    return f"include = {json.dumps(str(base))}\n\n" + "\n".join(tables(overrides)) + "\n"


def configured_backend(physics_config):
    """The backend a live session runs: the physics overlay's, else the one in
    the pose_estimator subsystem file that the included robot config names."""
    overrides = tomllib.loads(physics_config.read_text(encoding="utf-8"))
    backend = overrides.get("subsystems", {}).get(
        "pose_estimator", {}).get("backend")
    if backend is not None:
        return backend
    base = physics_config.parent / overrides["include"]
    for entry in tomllib.loads(base.read_text(encoding="utf-8")).get(
            "subsystems", []):
        if entry.get("name") == "pose_estimator":
            subsystem = base.parent / entry["path"]
            return tomllib.loads(subsystem.read_text(
                encoding="utf-8"))["subsystem"]["backend"]
    raise ValueError(f"no pose_estimator subsystem in {base}")


def replay_config(physics_config, backend):
    """Odometry-only replay: the fused replays' estimator, cameras and all,
    with each camera topic remapped to one no session records, so no
    observation arrives, and the pair differ only in observations, not in
    the estimator's subscriptions. No mounting-error overlay; nothing reads
    a camera's mount."""
    return overlay_config(physics_config, {"pose_estimator": {
        "backend": backend,
        "remap": {f"/{camera}/state": f"/{camera}/state/none"
                  for camera in CAMERAS}}})


def scenario_config(physics_config, min_yaw_std_rad=None,
                    timestamp_lag_us=0, latency_offset_us=0, backend=None):
    """Live camera overlays; `backend` also selects the estimator (replays)."""
    overrides = {
        name: {"camera": dict(error, sim_timestamp_lag_us=timestamp_lag_us,
                               latency_offset_us=latency_offset_us)}
        for name, error in MOUNT_ERROR_PRESET.items()}
    if min_yaw_std_rad is not None:
        for values in overrides.values():
            values["min_yaw_std_rad"] = min_yaw_std_rad
    if backend is not None:
        overrides["pose_estimator"] = {"backend": backend}
    return overlay_config(physics_config, overrides)


def run_scenario(duration_s, output_dir, min_yaw_std_rad=None,
                 lag_duration_s=None, backends=BACKENDS):
    """Score each backend's input replays of three live sessions, plus the
    configured backend's live estimate of the baseline session."""
    started = time.monotonic()
    if lag_duration_s is None:
        lag_duration_s = duration_s
    if not math.isfinite(lag_duration_s) or lag_duration_s <= 0:
        raise ValueError("lag duration must be finite and positive")
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("duration must be finite and positive")
    if min_yaw_std_rad is not None and (
            not math.isfinite(min_yaw_std_rad) or min_yaw_std_rad < 0):
        raise ValueError("yaw std floor must be finite and nonnegative")
    if not backends or any(backend not in BACKENDS for backend in backends):
        raise ValueError(f"backends must be a nonempty subset of {BACKENDS}")
    files = runfiles.Create()
    config = Path(files.Rlocation("_main/" + CONFIG))
    launcher = Path(files.Rlocation("_main/talOS/launcher/launcher"))
    root = launcher.parents[2]
    dumper = Path(files.Rlocation("_main/talOS/launcher/log_dump"))
    output_dir = Path(output_dir).absolute()
    output_dir.mkdir(parents=True, exist_ok=True)
    # A unique child avoids mistaking older retained sessions for this run.
    destination = Path(tempfile.mkdtemp(prefix="accuracy-", dir=output_dir))
    env = dict(os.environ, **files.EnvVars(),
               TALOS_SHM_NAMESPACE=f"accuracy-{uuid.uuid4().hex}")
    # Preserve the runfiles binary layout instead of using the source workspace.
    env.pop("BUILD_WORKSPACE_DIRECTORY", None)
    live_backend = configured_backend(config)
    results = {"metrics": {}, "timing": {}}
    with tempfile.TemporaryDirectory(prefix="accuracy-launcher-") as temporary:
        cwd = Path(temporary)
        (cwd / "bazel-bin").symlink_to(root, target_is_directory=True)
        for name, lag_us, offset_us, seconds in (
                ("fused", 0, 0, duration_s),
                ("fused-lag-uncorrected", TIMESTAMP_LAG_US, 0, lag_duration_s),
                ("fused-lag-corrected", TIMESTAMP_LAG_US, TIMESTAMP_LAG_US,
                 lag_duration_s)):
            live_config = destination / f"{name}.toml"
            live_config.write_text(
                scenario_config(config, min_yaw_std_rad, lag_us, offset_us),
                encoding="utf-8")
            env["TALOS_SHM_NAMESPACE"] = f"accuracy-{uuid.uuid4().hex}"
            gateway_port, bridge_port = udp_ports()
            gateway_args = [
                "--physics", "--wheel-radius-error", "-0.02",
                "--wheel-slip-gain", "0.05", "--gyro-drift-radps", "0.002",
                "--mode-schedule", f"disabled:1,auto:{seconds}",
                "--port", str(gateway_port), "--remote-port", str(bridge_port),
            ]
            args = ["--config", str(live_config), "--sim", "--start-sim-gateway",
                    "--remote", "127.0.0.1", "--remote-port", str(gateway_port),
                    "--local-port", str(bridge_port),
                    "--output-dir", str(destination / name),
                    "--duration-s", str(seconds + 1)]
            for arg in gateway_args:
                args.extend(["--sim-gateway-arg", arg])
            run_launcher(launcher, args, cwd, env, destination / f"{name}.txt",
                         seconds + 90)
            session = session_directory(destination / name)
            cameras = [session / f"{camera}.tlog" for camera in CAMERAS]
            drivetrain = session / "drivetrain.tlog"
            records = dump_records(dumper, cameras, drivetrain,
                                   session / "pose_estimator.tlog", env)
            live_name = "fused-live" if name == "fused" else name
            results["timing"][live_name] = camera_timing(records)
            replays = [(name, backend, scenario_config(
                config, min_yaw_std_rad, lag_us, offset_us, backend))
                for backend in backends]
            if name == "fused":
                results["metrics"][("fused-live", live_backend)] = score(records)
                results["cameras"] = score_cameras(records)
                replays.extend(
                    ("odometry-only", backend, replay_config(config, backend))
                    for backend in backends)
            # Every backend and odometry-only run on the same virtual-clock
            # schedule and inputs. Comparing live fusion with offline odometry
            # also compares host scheduling: live polling can skip different
            # drivetrain samples and deliver cameras at different
            # odometry-buffer positions.
            for run, backend, text in replays:
                label = f"{run}-{backend}-replay"
                replay_config_path = destination / f"{label}.toml"
                replay_config_path.write_text(text, encoding="utf-8")
                run_launcher(
                    launcher,
                    ["--config", str(replay_config_path),
                     "--replay-inputs", str(session),
                     "--only", "pose_estimator", "--session-id", session.name,
                     "--output-dir", str(destination / label)],
                    cwd, env, destination / f"{label}.txt", 90)
                # Input replay writes a node log, not a session manifest.
                replayed = dump_records(
                    dumper, cameras, drivetrain,
                    destination / label / session.name / "pose_estimator.tlog",
                    env)
                results["metrics"][(run, backend)] = score(replayed)
    results["runtime_s"] = time.monotonic() - started
    (destination / "report.md").write_text(markdown(results) + "\n", encoding="utf-8")
    return results, destination
