"""Paired virtual-clock fusion gate and live-stack accuracy/scoring regressions."""

from contextlib import redirect_stdout
import io
import json
import math
import os
import tempfile
import unittest
from unittest.mock import patch

from accuracy import (BACKENDS, CAMERAS, RUNS, CameraTiming, Metrics,
                      camera_health_issues, camera_timing, interpolate_truth,
                      markdown, run_scenario, score, score_cameras,
                      summary_markdown)


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

        records.append(row(topic, observation(25, 1.5, 0, math.pi, std=0),
                           25, "SEND"))
        with self.assertRaises(ValueError):
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
        with self.assertRaises(ValueError):
            camera_timing([])

    def test_camera_health_is_per_camera_and_independent_of_fusion_error(self):
        healthy = {camera: CameraTiming(25_000, 100, 200, 190)
                   for camera in CAMERAS}
        self.assertEqual(camera_health_issues(healthy), [])
        for timing in (
                CameraTiming(25_001, 100, 200, 190),
                CameraTiming(25_000, 101, 200, 190),
                CameraTiming(25_000, 100, 200, 189),
                CameraTiming(0, 0, 0, 0),
                # macOS follow-up: only ~28% of scheduled captures survived.
                CameraTiming(58_710, 353, 141, 141)):
            with self.subTest(timing=timing):
                timings = dict(healthy, rear_right_camera=timing)
                issues = camera_health_issues(timings)
                self.assertEqual(len(issues), 1)

    def test_health_condition_reports_inversion_and_capable_host_fails(self):
        # A worse fused RMS from any backend must fail on healthy inputs, but
        # is not evidence of a fusion regression when the capture
        # prerequisite is unmet. Only factor_graph's fused replay is worse.
        def metrics(run, backend):
            rms = (0.10 if run == "odometry-only" else
                   0.11 if (run, backend) == ("fused", "factor_graph") else 0.09)
            return Metrics(10, rms, 0.12, 1, 2, 0.1, 1, 1, 0)
        results = {"metrics": {(run, backend): metrics(run, backend)
                               for run in RUNS for backend in BACKENDS}}
        timings = {camera: CameraTiming(16_667, 0, 500, 500)
                   for camera in CAMERAS}
        results["timing"] = {
            name: timings for name in
            ("fused-live", "fused-lag-uncorrected", "fused-lag-corrected")}
        scenario = AccuracyScenarioTest(
            "test_fusion_accuracy_and_healthy_same_input_improvement")
        with patch(f"{__name__}.run_scenario", return_value=(results, None)), \
                patch.dict(os.environ, TALOS_ACCURACY_REQUIRE_CAMERA_HEALTH="0"), \
                redirect_stdout(io.StringIO()):
            with self.assertRaises(AssertionError):
                scenario.test_fusion_accuracy_and_healthy_same_input_improvement()
            timings["rear_right_camera"] = CameraTiming(58_710, 353, 141, 141)
            scenario.test_fusion_accuracy_and_healthy_same_input_improvement()
            with patch.dict(os.environ, TALOS_ACCURACY_REQUIRE_CAMERA_HEALTH="1"):
                with self.assertRaises(AssertionError):
                    scenario.test_fusion_accuracy_and_healthy_same_input_improvement()

    def test_missing_data_is_a_tool_failure(self):
        with self.assertRaises(ValueError):
            score([])
        with self.assertRaises(ValueError):
            summary_markdown([])


class AccuracyScenarioTest(unittest.TestCase):
    def test_fusion_accuracy_and_healthy_same_input_improvement(self):
        with tempfile.TemporaryDirectory(prefix="accuracy-test-") as directory:
            results, _ = run_scenario(8, directory, lag_duration_s=4)
            table = markdown(results)
            print("\n" + table, flush=True)
            metrics = results["metrics"]
            issues = camera_health_issues(results["timing"]["fused-live"])
            if os.environ.get("TALOS_ACCURACY_REQUIRE_CAMERA_HEALTH") == "1":
                self.assertFalse(issues, table)
            # Identical rows would mean the backend never reached the replay.
            self.assertNotEqual(metrics[("fused", "kalman")],
                                metrics[("fused", "factor_graph")], table)
            for backend in BACKENDS:
                with self.subTest(backend=backend):
                    fused = metrics[("fused", backend)]
                    odometry = metrics[("odometry-only", backend)]
                    # These rows share virtual timer firings, recorded input
                    # payloads and delivery (the odometry-only replay keeps
                    # the camera subscriptions, remapped to unrecorded
                    # topics). The live source is separately subject to host
                    # scheduling.
                    self.assertEqual(fused.samples, odometry.samples, table)
                    improvement = odometry.position_rms_m - fused.position_rms_m
                    print(f"{backend} paired fusion position RMS improvement: "
                          f"{improvement:.4f} m; "
                          + ("REQUIRED (camera throughput sufficient)"
                             if not issues else
                             "NOT EVALUATED (INSUFFICIENT camera throughput)"),
                          flush=True)
                    if not issues:
                        self.assertLess(fused.position_rms_m,
                                        odometry.position_rms_m, table)
                    live = metrics.get(("fused-live", backend))
                    for result in (fused,) if live is None else (fused, live):
                        self.assertLess(result.position_rms_m, 0.15, table)
                        self.assertLess(result.heading_rms_deg, 3, table)
                        self.assertGreater(result.accepted, 0, table)
                    uncorrected = metrics[("fused-lag-uncorrected", backend)]
                    corrected = metrics[("fused-lag-corrected", backend)]
                    self.assertGreater(uncorrected.accepted, 0, table)
                    self.assertGreater(corrected.accepted, 0, table)
                    print(f"{backend} latency correction position RMS "
                          "improvement: "
                          f"{uncorrected.position_rms_m - corrected.position_rms_m:.4f} m "
                          f"({uncorrected.position_rms_m / corrected.position_rms_m:.2f}x)",
                          flush=True)


if __name__ == "__main__":
    unittest.main()
