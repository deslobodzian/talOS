"""Fail-closed retry decider for unacquired hosted GitHub Actions jobs (CI-11).

    python3 tools/retry_unacquired.py --repo OWNER/REPO --run-id ID --attempt N [--apply]

Hosted macOS runners occasionally abort before acquiring a VM with check-run
annotation "The job was not acquired by Runner of type hosted even after
multiple attempts" (leaving the job `cancelled` with `runner_id == 0`, empty
`runner_name`, and `steps == []`; on `Continuous Integration`, `CI gate` then
fails solely because that required job did not pass).

This script inspects the run metadata, the completed attempt's jobs, and
check-run annotations via `gh api` and reruns failed/cancelled jobs
(`gh run rerun <id> --failed`) ONLY when ALL of these hold:

1. `1 <= attempt < max_attempts` (default 3, capping auto-retries at 2);
2. the run is still completed at `run_attempt == attempt` (so re-running an
   older retry workflow event cannot bypass the attempt cap);
3. the run is still the newest run for its workflow, event, and branch/PR (so
   a rerun cannot re-enter the concurrency group and cancel or displace a newer
   commit's run; a ~1 s TOCTOU window remains between the freshness query and
   `gh run rerun`);
4. at least one job failed to acquire a runner (zero steps, no runner, and a
   check-run annotation matching "not acquired by runner"); and
5. every other non-success/non-skipped job in the attempt is either also
   unacquired or the `CI gate` aggregator job.

Real build/test failures, runner crashes after acquisition, stale runs, and
user or concurrency cancellations fail closed and are never retried.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import urllib.parse
from collections.abc import Callable
from typing import Any

UNACQUIRED_PHRASE = "not acquired by runner"
AGGREGATOR_JOB_NAMES = frozenset({"CI gate"})


def has_unacquired_annotation(annotations: list[dict[str, Any]]) -> bool:
    """True iff any check-run annotation reports an unacquired hosted runner."""
    for ann in annotations:
        msg = str(ann.get("message") or "").lower()
        if UNACQUIRED_PHRASE in msg:
            return True
    return False


def is_unacquired_candidate(job: dict[str, Any]) -> bool:
    """True iff `job` ended unsuccessfully with no runner and zero steps."""
    if job.get("conclusion") not in ("cancelled", "failure"):
        return False
    if job.get("runner_id"):
        return False
    if job.get("runner_name"):
        return False
    steps = job.get("steps")
    if steps is None or not isinstance(steps, list) or len(steps) != 0:
        return False
    return True


def _pr_identity(run: dict[str, Any]) -> tuple[Any, ...]:
    """Stable identity for a pull_request run's concurrency group."""
    prs = run.get("pull_requests")
    if isinstance(prs, list) and prs:
        numbers = tuple(
            sorted(pr["number"] for pr in prs if isinstance(pr, dict) and isinstance(pr.get("number"), int))
        )
        if numbers:
            return ("pr", numbers)
    head_repo = run.get("head_repository")
    repo_name = head_repo.get("full_name") if isinstance(head_repo, dict) else None
    head_branch = run.get("head_branch")
    if repo_name and head_branch:
        return ("head", repo_name, head_branch)
    return ()


def check_run_freshness(
    run_id: int,
    attempt: int,
    run_meta: dict[str, Any],
    recent_workflow_runs: list[dict[str, Any]],
    repo: str | None = None,
) -> tuple[bool, str]:
    """Verify `run_id` is still at `attempt` and is the newest run for its ref."""
    current_attempt = run_meta.get("run_attempt")
    if current_attempt != attempt:
        return (
            False,
            f"refusing to retry: run {run_id} is now at attempt {current_attempt}, not {attempt}",
        )
    if run_meta.get("status") != "completed":
        return (
            False,
            f"refusing to retry: run {run_id} status is {run_meta.get('status')!r}, not 'completed'",
        )
    if run_meta.get("conclusion") not in ("failure", "cancelled"):
        return (
            False,
            f"refusing to retry: run {run_id} conclusion is {run_meta.get('conclusion')!r}",
        )
    if not recent_workflow_runs:
        return False, "refusing to retry: workflow runs query returned no runs"

    event = run_meta.get("event")
    if event == "pull_request":
        want_id = _pr_identity(run_meta)
        if not want_id:
            return False, "refusing to retry: pull_request run lacks PR identity"
        matching = [
            r
            for r in recent_workflow_runs
            if r.get("event") == "pull_request" and _pr_identity(r) == want_id
        ]
        if not matching:
            return False, f"refusing to retry: run {run_id} not found among recent PR runs"
        newest = matching[0]
    else:
        same_event = [r for r in recent_workflow_runs if r.get("event") == event]
        if event == "push" and repo:
            want_repo = repo.lower()
            same_event = [
                r
                for r in same_event
                if isinstance(r.get("head_repository"), dict)
                and str(r["head_repository"].get("full_name") or "").lower() == want_repo
            ]
        if not same_event:
            return (
                False,
                f"refusing to retry: no recent {event!r} runs found for this workflow and ref",
            )
        newest = same_event[0]

    newest_id = newest.get("id")
    if newest_id != run_id:
        return (
            False,
            f"refusing to retry: newer run {newest_id} exists for this workflow and ref "
            f"(this run is {run_id})",
        )
    return True, "ok"


def should_retry_unacquired(
    jobs: list[dict[str, Any]],
    annotations_by_job_id: dict[int, list[dict[str, Any]]],
    attempt: int,
    max_attempts: int = 3,
) -> tuple[bool, str]:
    """Decide whether a completed workflow attempt should be auto-rerun.

    Fails closed on any real failure, concurrency/manual cancellation, missing
    annotation proof, missing job list, or exhausted attempt budget.
    """
    if attempt < 1:
        return False, f"invalid attempt number {attempt}"
    if attempt >= max_attempts:
        return False, f"attempt {attempt} reached max_attempts={max_attempts}; not retrying"
    if not jobs:
        return False, "no jobs reported for this attempt; refusing to retry"

    problem_jobs = [
        j for j in jobs if j.get("conclusion") not in ("success", "skipped", "neutral")
    ]
    if not problem_jobs:
        return False, "no failed or cancelled jobs in this attempt"

    unacquired: list[dict[str, Any]] = []
    other_problems: list[dict[str, Any]] = []
    for job in problem_jobs:
        job_id = job.get("id")
        anns = annotations_by_job_id.get(job_id, []) if isinstance(job_id, int) else []
        if is_unacquired_candidate(job) and has_unacquired_annotation(anns):
            unacquired.append(job)
        else:
            other_problems.append(job)

    if not unacquired:
        names = ", ".join(str(j.get("name", "<unnamed>")) for j in problem_jobs)
        return False, f"no unacquired-runner jobs found (non-passing: {names})"

    # Once at least one real job is proven unacquired, allow only the pure
    # aggregator job (`CI gate`, which runs `if: !cancelled()` and fails
    # because a required dependency was unacquired) among other non-passing jobs.
    non_gate_problems = [
        j
        for j in other_problems
        if not (j.get("name") in AGGREGATOR_JOB_NAMES and j.get("conclusion") == "failure")
    ]
    if non_gate_problems:
        names = ", ".join(str(j.get("name", "<unnamed>")) for j in non_gate_problems)
        return (
            False,
            f"refusing to retry: non-unacquired failure/cancellation present: {names}",
        )

    names = ", ".join(str(j.get("name", "<unnamed>")) for j in unacquired)
    return (
        True,
        f"attempt {attempt}/{max_attempts - 1} retryable: {len(unacquired)} unacquired "
        f"hosted job(s): {names}",
    )


def parse_paginated_json(raw: str) -> list[Any]:
    """Parse concatenated JSON values from `gh api --paginate`."""
    decoder = json.JSONDecoder()
    idx = 0
    length = len(raw)
    values: list[Any] = []
    while idx < length:
        while idx < length and raw[idx].isspace():
            idx += 1
        if idx >= length:
            break
        obj, end = decoder.raw_decode(raw, idx)
        idx = end
        values.append(obj)
    return values


def parse_paginated_jobs(raw: str) -> list[dict[str, Any]]:
    jobs: list[dict[str, Any]] = []
    for obj in parse_paginated_json(raw):
        if not isinstance(obj, dict) or not isinstance(obj.get("jobs"), list):
            raise ValueError("unexpected GitHub jobs API payload shape")
        jobs.extend(obj["jobs"])
    return jobs


def parse_paginated_annotations(raw: str) -> list[dict[str, Any]]:
    annotations: list[dict[str, Any]] = []
    for obj in parse_paginated_json(raw):
        if not isinstance(obj, list):
            raise ValueError("unexpected GitHub annotations API payload shape")
        annotations.extend(obj)
    return annotations


def gh_api(endpoint: str, *, paginate: bool = True) -> str:
    cmd = ["gh", "api"]
    if paginate:
        cmd.append("--paginate")
    cmd.append(endpoint)
    res = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return res.stdout


def evaluate_run(
    repo: str,
    run_id: int,
    attempt: int,
    max_attempts: int = 3,
    *,
    check_freshness: bool = True,
    api_get: Callable[..., str] = gh_api,
) -> tuple[bool, str]:
    if attempt < 1:
        return False, f"invalid attempt number {attempt}"
    if attempt >= max_attempts:
        return False, f"attempt {attempt} reached max_attempts={max_attempts}; not retrying"

    if check_freshness:
        run_meta = json.loads(api_get(f"/repos/{repo}/actions/runs/{run_id}", paginate=False))
        if not isinstance(run_meta, dict):
            raise ValueError("unexpected GitHub run API payload shape")
        workflow_id = run_meta.get("workflow_id")
        if not isinstance(workflow_id, int):
            return False, f"refusing to retry: run {run_id} missing workflow_id"
        event = run_meta.get("event")
        if not isinstance(event, str) or not event:
            return False, f"refusing to retry: run {run_id} missing event"
        if event == "pull_request":
            recent_endpoint = (
                f"/repos/{repo}/actions/workflows/{workflow_id}/runs"
                f"?event=pull_request&per_page=30"
            )
        else:
            branch = run_meta.get("head_branch")
            if not isinstance(branch, str) or not branch:
                return False, f"refusing to retry: run {run_id} missing head_branch"
            encoded_branch = urllib.parse.quote(branch, safe="")
            encoded_event = urllib.parse.quote(event, safe="")
            recent_endpoint = (
                f"/repos/{repo}/actions/workflows/{workflow_id}/runs"
                f"?branch={encoded_branch}&event={encoded_event}&per_page=10"
            )
        recent_payload = json.loads(api_get(recent_endpoint, paginate=False))
        recent_runs = (
            recent_payload.get("workflow_runs")
            if isinstance(recent_payload, dict)
            else None
        )
        if not isinstance(recent_runs, list):
            raise ValueError("unexpected GitHub workflow runs API payload shape")
        fresh, reason = check_run_freshness(
            run_id, attempt, run_meta, recent_runs, repo=repo
        )
        if not fresh:
            return False, reason

    jobs_raw = api_get(
        f"/repos/{repo}/actions/runs/{run_id}/attempts/{attempt}/jobs?per_page=100",
        paginate=True,
    )
    jobs = parse_paginated_jobs(jobs_raw)
    annotations_by_job_id: dict[int, list[dict[str, Any]]] = {}
    for job in jobs:
        job_id = job.get("id")
        if is_unacquired_candidate(job) and isinstance(job_id, int):
            ann_raw = api_get(
                f"/repos/{repo}/check-runs/{job_id}/annotations",
                paginate=True,
            )
            annotations_by_job_id[job_id] = parse_paginated_annotations(ann_raw)
    return should_retry_unacquired(jobs, annotations_by_job_id, attempt, max_attempts)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", required=True, help="owner/repo")
    parser.add_argument("--run-id", type=int, required=True, help="workflow run ID")
    parser.add_argument("--attempt", type=int, required=True, help="completed attempt number")
    parser.add_argument("--max-attempts", type=int, default=3, help="maximum total attempts")
    parser.add_argument(
        "--apply",
        action="store_true",
        help="execute `gh run rerun <run-id> --failed` when retryable",
    )
    parser.add_argument(
        "--ignore-freshness",
        action="store_true",
        help="skip latest-run and current-attempt checks (for dry-running historical runs; "
        "refused with --apply)",
    )
    args = parser.parse_args(argv)
    if args.apply and args.ignore_freshness:
        parser.error("--ignore-freshness cannot be combined with --apply")

    retry, reason = evaluate_run(
        args.repo,
        args.run_id,
        args.attempt,
        args.max_attempts,
        check_freshness=not args.ignore_freshness,
    )
    print(f"retry-unacquired: {reason}")
    if retry and args.apply:
        subprocess.run(
            ["gh", "run", "rerun", str(args.run_id), "--repo", args.repo, "--failed"],
            check=True,
        )
        print(f"retry-unacquired: requested failed-job rerun of run {args.run_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
