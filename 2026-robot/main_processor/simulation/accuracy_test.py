"""Loose end-to-end fusion gate and focused scoring regression tests."""

import json
import math
from pathlib import Path
import tempfile
import tomllib
import unittest

from accuracy import (CAMERAS, MOUNT_ERROR_PRESET, RUNS, TIMESTAMP_LAG_US, Metrics,
                      camera_markdown, camera_timing, interpolate_truth, markdown,
                      replay_config, run_scenario, scenario_config, score,
                      score_cameras, summary_markdown, timing_markdown)


def row(topic, message, event_ms=0, kind="MESSAGE"):
    return {"topic": topic, "kind": kind, "event_ms": event_ms,
            "decoded": json.dumps(message)}


def drivetrain(enabled, event_ms):
    return row("/drivetrain/state", {"enabled": enabled}, event_ms, "SEND")


def estimate(timestamp, x, yaw, valid=True):
    return {"sample_time_ns": timestamp,
            "pose": {"x_m": x, "y_m": 0, "yaw_rad": yaw}, "valid": valid,
            "observation_accepted_count": 7, "observation_rejected_count": 2}


class ScoringTest(unittest.TestCase):
    def test_interpolation_wraps_heading_and_never_extrapolates(self):
        times = [10, 30]
        poses = [(0, 0, math.radians(179)), (2, 4, math.radians(-179))]
        x, y, yaw = interpolate_truth(times, poses, 20)
        self.assertEqual((x, y), (1, 2))
        self.assertAlmostEqual(abs(yaw), math.pi)
        self.assertEqual(interpolate_truth(times, poses, 10), poses[0])
        self.assertIsNone(interpolate_truth(times, poses, 9))
        self.assertIsNone(interpolate_truth(times, poses, 31))

    def test_filters_disabled_stale_invalid_and_unbracketed_samples(self):
        records = [
            row("/sim/state", estimate(10, 0, math.radians(179))),
            row("/sim/state", estimate(30, 2, math.radians(179))),
            drivetrain(False, 0), drivetrain(True, 10),
            row("/pose_estimator/state", estimate(11, 99, 0), 5, "SEND"),
            row("/pose_estimator/state", estimate(12, 99, 0, False), 12, "SEND"),
            row("/pose_estimator/state", estimate(20, 1.3, math.radians(-179)),
                20, "SEND"),
            # A second publication of this odometry sample replaces the first.
            row("/pose_estimator/state", estimate(20, 1.5, math.radians(-179)),
                21, "SEND"),
            row("/pose_estimator/state", estimate(21, 99, 0), 110, "SEND"),
            row("/pose_estimator/state", estimate(31, 99, 0), 25, "SEND"),
        ]
        result = score(records)
        self.assertEqual(result.samples, 1)
        self.assertAlmostEqual(result.position_rms_m, 0.5)
        self.assertAlmostEqual(result.position_max_m, 0.5)
        self.assertAlmostEqual(result.heading_rms_deg, 2)
        self.assertAlmostEqual(result.heading_max_deg, 2)
        self.assertAlmostEqual(result.final_position_m, 0.5)
        self.assertAlmostEqual(result.final_heading_deg, 2)
        self.assertEqual((result.accepted, result.rejected), (7, 2))

    def test_camera_normalized_errors_use_sample_time_and_absolute_p95(self):
        def observation(timestamp, x, y, yaw, valid=True, std=0.5):
            return {"sample_time_ns": timestamp,
                    "pose": {"x_m": x, "y_m": y, "yaw_rad": yaw},
                    "x_std_m": std, "y_std_m": 0.25, "yaw_std_rad": 0.1,
                    "valid": valid}

        topic = "/front_left_camera/state"
        records = [
            row("/sim/state", estimate(10, 0, math.radians(179))),
            row("/sim/state", estimate(30, 2, math.radians(-179))),
            drivetrain(False, 0), drivetrain(True, 10),
            row(topic, observation(11, 99, 0, 0), 5, "SEND"),
            row(topic, observation(12, 99, 0, 0, False), 12, "SEND"),
            row(topic, observation(20, 99, 0, 0), 20, "SEND"),
            # Duplicate timestamp takes the last SEND, not the first.
            row(topic, observation(20, 0.5, 0.5, math.pi + 0.2), 21, "SEND"),
            row(topic, observation(30, 3, -0.25, math.radians(-179) - 0.4),
                30, "SEND"),
            row(topic, observation(21, 99, 0, 0), 110, "SEND"),
            row(topic, observation(31, 99, 0, 0), 25, "SEND"),
        ]
        result = score_cameras(records)["front_left_camera"]
        self.assertEqual(result.samples, 2)
        for actual, expected in zip(result.normalized_rms,
                                    (math.sqrt(2.5), math.sqrt(2.5),
                                     math.sqrt(10))):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(result.normalized_abs_p95, (1.95, 1.95, 3.9)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(result.yaw_rms_deg, math.degrees(math.sqrt(0.1)))
        self.assertAlmostEqual(result.consistent_yaw_floor_rad, math.sqrt(0.1))
        table = camera_markdown({"front_left_camera": result})
        self.assertIn("front_left_camera | 2", table)
        self.assertIn("rear_left_camera | 0 | n/a", table)

        records.append(row(topic, observation(25, 1.5, 0, math.pi, std=0),
                           25, "SEND"))
        with self.assertRaisesRegex(ValueError, "front_left_camera"):
            score_cameras(records)

    def test_camera_timing_uses_latest_send_and_reports_real_drop_counts(self):
        def status(period, dropped, captured):
            return {"frame_period_us": period, "frames_dropped_count": dropped,
                    "frames_captured_count": captured,
                    "frames_processed_count": captured}

        records = [
            row(f"/{camera}/status", status(16667, 2, 61), 100, "SEND")
            for camera in CAMERAS]
        records.append(row("/front_left_camera/status",
                           status(20000, 0, 50), 90, "SEND"))
        records.append(row("/front_left_camera/status",
                           status(99999, 99, 1), 110))
        timing = camera_timing(records)
        for camera in CAMERAS:
            self.assertEqual(timing[camera].frame_period_us, 16667)
            self.assertEqual(timing[camera].dropped, 2)
            self.assertEqual(timing[camera].captured, 61)
            self.assertEqual(timing[camera].processed, 61)
        report = timing_markdown({
            run: timing for run in ("fused", "fused-lag-uncorrected",
                                   "fused-lag-corrected")})
        self.assertIn("| fused | front_left_camera | 16667 | 2 | 61 | 61 |",
                      report)
        with self.assertRaisesRegex(ValueError, "missing camera status"):
            camera_timing([])

    def test_repeat_summary_reports_mean_min_max_for_each_row(self):
        scenarios = []
        for position in (0.01, 0.03):
            result = {
                name: Metrics(1, position, position, position * 10,
                              position * 10, position, position * 10, 1, 0)
                for name in RUNS}
            result["runtime_s"] = 2
            scenarios.append(result)
        report = summary_markdown(scenarios)
        for name in RUNS:
            self.assertIn(
                f"| {name} | 0.0200 | 0.0100 | 0.0300 | 0.200 | 0.100 | 0.300 |",
                report)
        self.assertIn("Scenarios: 2; total runtime: 4.0 s", report)
        with self.assertRaisesRegex(ValueError, "at least one scenario"):
            summary_markdown([])

    def test_missing_data_is_a_tool_failure(self):
        with self.assertRaisesRegex(ValueError, "missing"):
            score([])

    def test_replay_config_preserves_physics_delta_without_nested_include(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "physics.toml"
            config.write_text(
                'include = "robot.toml"\n'
                '[subsystems.drivetrain]\nintegrate_heading = false\n'
                '[subsystems.driver]\nheading_hold_radps = 2.0\n',
                encoding="utf-8")
            replay = tomllib.loads(replay_config(config))
            self.assertEqual(replay["include"], str(Path(directory) / "robot.toml"))
            self.assertFalse(replay["subsystems"]["drivetrain"]["integrate_heading"])
            self.assertEqual(replay["subsystems"]["driver"]["heading_hold_radps"], 2)
            self.assertEqual(replay["subsystems"]["pose_estimator"]["cameras"], [])
            live = tomllib.loads(scenario_config(config))
            self.assertEqual(live["include"], replay["include"])
            self.assertFalse(live["subsystems"]["drivetrain"]["integrate_heading"])
            self.assertEqual(live["subsystems"]["driver"]["heading_hold_radps"], 2)
            for name in CAMERAS:
                self.assertNotIn(name, replay["subsystems"])
                camera = live["subsystems"][name]["camera"]
                self.assertEqual(camera, dict(
                    MOUNT_ERROR_PRESET[name], sim_timestamp_lag_us=0,
                    latency_offset_us=0))
                for key, value in MOUNT_ERROR_PRESET[name].items():
                    magnitude = (math.radians(1.0) if key.endswith("yaw_rad")
                                 else math.radians(0.5) if key.endswith("pitch_rad")
                                 else 0.01)
                    self.assertAlmostEqual(abs(value), magnitude)
            patterns = {tuple(error.values())
                        for error in MOUNT_ERROR_PRESET.values()}
            self.assertEqual(len(patterns), len(CAMERAS))
            swept = tomllib.loads(scenario_config(config, 0.035))
            for name in CAMERAS:
                self.assertEqual(
                    swept["subsystems"][name]["min_yaw_std_rad"], 0.035)
                self.assertEqual(swept["subsystems"][name]["camera"],
                                 live["subsystems"][name]["camera"])
            lagged = tomllib.loads(scenario_config(
                config, timestamp_lag_us=TIMESTAMP_LAG_US))
            corrected = tomllib.loads(scenario_config(
                config, timestamp_lag_us=TIMESTAMP_LAG_US,
                latency_offset_us=TIMESTAMP_LAG_US))
            for name in CAMERAS:
                for session, offset in ((lagged, 0),
                                        (corrected, TIMESTAMP_LAG_US)):
                    self.assertEqual(session["subsystems"][name]["camera"],
                                     dict(MOUNT_ERROR_PRESET[name],
                                          sim_timestamp_lag_us=TIMESTAMP_LAG_US,
                                          latency_offset_us=offset))


class AccuracyScenarioTest(unittest.TestCase):
    def test_fusion_improves_same_input_odometry(self):
        with tempfile.TemporaryDirectory(prefix="accuracy-test-") as directory:
            results, _ = run_scenario(8, directory, lag_duration_s=4)
            table = markdown(results)
            print("\n" + table, flush=True)
            fused, odometry = results["fused"], results["odometry-only"]
            self.assertLess(fused.position_rms_m, odometry.position_rms_m, table)
            self.assertLess(fused.position_rms_m, 0.15, table)
            self.assertLess(fused.heading_rms_deg, 3, table)
            self.assertGreater(fused.accepted, 0, table)
            uncorrected = results["fused-lag-uncorrected"]
            corrected = results["fused-lag-corrected"]
            self.assertGreater(uncorrected.accepted, 0, table)
            self.assertGreater(corrected.accepted, 0, table)
            print("Latency correction position RMS improvement: "
                  f"{uncorrected.position_rms_m - corrected.position_rms_m:.4f} m "
                  f"({uncorrected.position_rms_m / corrected.position_rms_m:.2f}x)",
                  flush=True)


if __name__ == "__main__":
    unittest.main()
