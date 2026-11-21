"""Unit tests for tools/retry_unacquired.py (CI-11)."""

import json
import unittest

from tools import retry_unacquired

UNACQUIRED_ANN = [
    {
        "annotation_level": "failure",
        "message": "The job was not acquired by Runner of type hosted even after multiple attempts",
    }
]
CONCURRENCY_ANN = [
    {
        "annotation_level": "failure",
        "message": "Canceling since a higher priority waiting request exists",
    }
]


def _job(
    job_id,
    name,
    conclusion="success",
    runner_id=101,
    runner_name="GitHub Actions 1",
    steps=None,
):
    if steps is None:
        steps = [{"name": "Checkout", "conclusion": conclusion}]
    return {
        "id": job_id,
        "name": name,
        "conclusion": conclusion,
        "runner_id": runner_id,
        "runner_name": runner_name,
        "steps": steps,
    }


def _unacquired(job_id, name="macOS (build, test)", conclusion="cancelled"):
    return {
        "id": job_id,
        "name": name,
        "conclusion": conclusion,
        "runner_id": 0,
        "runner_name": "",
        "steps": [],
    }


class RetryUnacquiredTest(unittest.TestCase):
    def test_retries_ci_unacquired_macos_with_failed_gate(self):
        jobs = [
            _job(1, "Linux (build, test, clang-tidy)", conclusion="success"),
            _unacquired(2, "macOS (build, test)"),
            _job(3, "CI gate", conclusion="failure"),
        ]
        retry, reason = retry_unacquired.should_retry_unacquired(
            jobs, {2: UNACQUIRED_ANN}, attempt=1
        )
        self.assertTrue(retry, reason)
        self.assertIn("macOS (build, test)", reason)

    def test_retries_nightly_two_unacquired_jobs_on_attempt_two(self):
        jobs = [
            _job(1, "Bazel (asan, macOS)", conclusion="success"),
            _unacquired(2, "Bazel (tsan, macOS)"),
            _unacquired(3, "macos-soak / Soak (macos-latest, seed 1)"),
        ]
        retry, reason = retry_unacquired.should_retry_unacquired(
            jobs, {2: UNACQUIRED_ANN, 3: UNACQUIRED_ANN}, attempt=2, max_attempts=3
        )
        self.assertTrue(retry, reason)
        self.assertIn("2 unacquired hosted job(s)", reason)

    def test_stops_at_max_attempts(self):
        jobs = [_unacquired(2)]
        retry, reason = retry_unacquired.should_retry_unacquired(
            jobs, {2: UNACQUIRED_ANN}, attempt=3, max_attempts=3
        )
        self.assertFalse(retry)
        self.assertIn("max_attempts=3", reason)

    def test_refuses_concurrency_cancelled_run(self):
        jobs = [
            _job(1, "Linux (build, test, clang-tidy)", conclusion="cancelled"),
            _unacquired(2, "CI gate"),
        ]
        retry, _ = retry_unacquired.should_retry_unacquired(
            jobs, {2: CONCURRENCY_ANN}, attempt=1
        )
        self.assertFalse(retry)

    def test_refuses_cancelled_zero_step_job_without_unacquired_annotation(self):
        jobs = [_unacquired(2, "macOS (build, test)")]
        retry, _ = retry_unacquired.should_retry_unacquired(
            jobs, {2: CONCURRENCY_ANN}, attempt=1
        )
        self.assertFalse(retry)

        retry_empty, _ = retry_unacquired.should_retry_unacquired(jobs, {}, attempt=1)
        self.assertFalse(retry_empty)

    def test_refuses_real_step_failure_or_acquired_runner(self):
        step_fail = [
            _job(2, "macOS (build, test)", conclusion="failure", runner_id=0, runner_name="",
                 steps=[{"name": "Checkout", "conclusion": "failure"}]),
        ]
        retry, _ = retry_unacquired.should_retry_unacquired(
            step_fail, {2: UNACQUIRED_ANN}, attempt=1
        )
        self.assertFalse(retry)

        runner_fail = [
            _job(2, "macOS (build, test)", conclusion="cancelled", runner_id=55,
                 runner_name="GitHub Actions 55", steps=[]),
        ]
        retry2, _ = retry_unacquired.should_retry_unacquired(
            runner_fail, {2: UNACQUIRED_ANN}, attempt=1
        )
        self.assertFalse(retry2)

    def test_refuses_mixed_unacquired_and_real_failure(self):
        jobs = [
            _job(1, "Linux (build, test, clang-tidy)", conclusion="failure"),
            _unacquired(2, "macOS (build, test)"),
            _job(3, "CI gate", conclusion="failure"),
        ]
        retry, reason = retry_unacquired.should_retry_unacquired(
            jobs, {2: UNACQUIRED_ANN}, attempt=1
        )
        self.assertFalse(retry)
        self.assertIn("Linux", reason)

    def test_refuses_empty_jobs_or_all_passing(self):
        self.assertFalse(retry_unacquired.should_retry_unacquired([], {}, attempt=1)[0])
        self.assertFalse(
            retry_unacquired.should_retry_unacquired([_job(1, "Linux")], {}, attempt=1)[0]
        )

    def test_evaluate_run_fetches_annotations_only_for_candidates(self):
        calls = []

        def fake_api(endpoint):
            calls.append(endpoint)
            if "/jobs" in endpoint:
                return (
                    json.dumps({"jobs": [_job(1, "Linux")]})
                    + "\n"
                    + json.dumps({"jobs": [_unacquired(2, "macOS"), _job(3, "CI gate", "failure")]})
                )
            if "/check-runs/2/annotations" in endpoint:
                return json.dumps(UNACQUIRED_ANN)
            raise AssertionError(f"unexpected endpoint: {endpoint}")

        retry, reason = retry_unacquired.evaluate_run(
            "owner/repo", run_id=99, attempt=1, api_get=fake_api
        )
        self.assertTrue(retry, reason)
        self.assertEqual(len(calls), 2)


if __name__ == "__main__":
    unittest.main()
