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


def _run_meta(
    run_id=99,
    workflow_id=7,
    attempt=1,
    event="push",
    head_branch="feature/dry-and-hardening",
    status="completed",
    conclusion="failure",
    pull_requests=None,
    head_repo="owner/repo",
):
    return {
        "id": run_id,
        "workflow_id": workflow_id,
        "run_attempt": attempt,
        "event": event,
        "head_branch": head_branch,
        "status": status,
        "conclusion": conclusion,
        "pull_requests": pull_requests if pull_requests is not None else [],
        "head_repository": {"full_name": head_repo},
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

    def test_refuses_when_newer_run_exists_for_branch(self):
        meta = _run_meta(run_id=99, attempt=1)
        fresh, reason = retry_unacquired.check_run_freshness(
            99, 1, meta, [_run_meta(run_id=100, attempt=1)]
        )
        self.assertFalse(fresh)
        self.assertIn("newer run 100", reason)

    def test_refuses_when_run_attempt_already_advanced(self):
        meta = _run_meta(run_id=99, attempt=3)
        fresh, reason = retry_unacquired.check_run_freshness(
            99, 1, meta, [meta]
        )
        self.assertFalse(fresh)
        self.assertIn("attempt 3, not 1", reason)

    def test_refuses_newer_pull_request_run_for_same_pr_only(self):
        pr42_old = _run_meta(
            run_id=99, event="pull_request", head_branch="topic", pull_requests=[{"number": 42}]
        )
        pr43_other = _run_meta(
            run_id=101, event="pull_request", head_branch="other", pull_requests=[{"number": 43}]
        )
        pr42_new = _run_meta(
            run_id=102, event="pull_request", head_branch="topic", pull_requests=[{"number": 42}]
        )
        # Another PR's newer run does not block PR #42.
        ok, _ = retry_unacquired.check_run_freshness(99, 1, pr42_old, [pr43_other, pr42_old])
        self.assertTrue(ok)
        # A newer run on PR #42 itself refuses.
        stale, reason = retry_unacquired.check_run_freshness(
            99, 1, pr42_old, [pr42_new, pr43_other, pr42_old]
        )
        self.assertFalse(stale)
        self.assertIn("newer run 102", reason)

    def test_push_run_not_blocked_by_sibling_pr_or_fork_run_on_same_branch(self):
        push_100 = _run_meta(
            run_id=100, event="push", head_branch="feature/x", head_repo="owner/repo"
        )
        pr_101 = _run_meta(
            run_id=101, event="pull_request", head_branch="feature/x", head_repo="owner/repo"
        )
        fork_102 = _run_meta(
            run_id=102, event="push", head_branch="feature/x", head_repo="fork/repo"
        )
        push_103 = _run_meta(
            run_id=103, event="push", head_branch="feature/x", head_repo="owner/repo"
        )
        ok, reason = retry_unacquired.check_run_freshness(
            100, 1, push_100, [fork_102, pr_101, push_100], repo="owner/repo"
        )
        self.assertTrue(ok, reason)
        stale, reason2 = retry_unacquired.check_run_freshness(
            100, 1, push_100, [push_103, pr_101, push_100], repo="owner/repo"
        )
        self.assertFalse(stale)
        self.assertIn("newer run 103", reason2)

    def test_evaluate_run_checks_freshness_then_fetches_candidate_annotations(self):
        calls = []

        def fake_api(endpoint, *, paginate=True):
            calls.append((endpoint, paginate))
            if endpoint == "/repos/owner/repo/actions/runs/99":
                return json.dumps(_run_meta(run_id=99, workflow_id=7, attempt=1))
            if endpoint == (
                "/repos/owner/repo/actions/workflows/7/runs"
                "?branch=feature%2Fdry-and-hardening&event=push&per_page=10"
            ):
                return json.dumps({"workflow_runs": [_run_meta(run_id=99, workflow_id=7)]})
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
        self.assertEqual(len(calls), 4)
        self.assertFalse(calls[0][1])
        self.assertFalse(calls[1][1])


if __name__ == "__main__":
    unittest.main()
