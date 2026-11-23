"""Classifies a `bazel query --keep_going` exit 3 for the tools that query.

`tools/check_layers.py` and `tools/slow_tests.py` run `bazel query
--keep_going`, which exits 3 when some package cannot load; some external
packages cannot load on every host (opencv's Android branch;
docs/DEVELOPMENT.md §5). Each accepts an exit 3 only when
`exit_3_is_external_only` says every error came from an external `@`
repository.

The scripts import it as `tools.bazel_query` under Bazel and as
`bazel_query` when run as `python3 tools/<script>.py`.
"""

import os
import re

_ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]|\r")
_BAZEL_ERROR_LINE_RE = re.compile(r"^ERROR:\s*(.+)$")
_BAZEL_TRANSITIVE_SUMMARY_RE = re.compile(
    r"^Evaluation of query .* failed: errors were encountered while computing "
    r"transitive closure$")
_BAZEL_WORKSPACE_REF_RE = re.compile(r"referenced by '(?:@@)?//")
_BAZEL_EXTERNAL_PATH_RE = re.compile(
    r"^(?:[A-Za-z]:[/\\]|/)[^\s:]*[/\\]external[/\\][^/\\:]+[/\\]")
_BAZEL_EXTERNAL_PKG_RE = re.compile(
    r"^(?:error loading package|no such package|Skipping) '@@?[^'/@]+//")


def exit_3_is_external_only(stderr):
    """True only when every ERROR line in `stderr` from a `bazel query` exit 3
    belongs to an external `@` repository (plus Bazel's transitive-closure
    summary line). Any workspace package error, workspace reference, unknown
    error, or empty stderr fails closed (returns False)."""
    cwd = os.getcwd()
    workspace_prefixes = {
        os.path.realpath(cwd) + os.sep,
        os.path.abspath(cwd) + os.sep,
    }
    saw_external = False
    saw_any_error = False
    for raw_line in stderr.splitlines():
        line = _ANSI_ESCAPE_RE.sub("", raw_line).strip()
        m = _BAZEL_ERROR_LINE_RE.match(line)
        if not m:
            continue
        saw_any_error = True
        body = m.group(1)
        if _BAZEL_TRANSITIVE_SUMMARY_RE.match(body):
            continue
        if _BAZEL_WORKSPACE_REF_RE.search(body):
            return False
        if any(body.startswith(p) for p in workspace_prefixes):
            return False
        if (_BAZEL_EXTERNAL_PATH_RE.match(body) or
                _BAZEL_EXTERNAL_PKG_RE.match(body)):
            saw_external = True
            continue
        return False
    return saw_any_error and saw_external
