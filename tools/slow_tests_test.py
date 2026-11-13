"""Unit tests for tools/slow_tests.py on fixture test results."""

import contextlib
import io
import json
import os
import tempfile
import unittest

from tools import slow_tests

# googletest's own file: the binary's time, to the millisecond.
GTEST_XML = """<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="2" failures="0" time="12.164" name="AllTests">
  <testsuite name="ReplayInputsTest" tests="2" time="12.164">
    <testcase name="A" status="run" time="10.1" classname="ReplayInputsTest" />
    <testcase name="B" status="run" time="2.064" classname="ReplayInputsTest" />
  </testsuite>
</testsuites>
"""

# What Bazel writes for a test that writes none: whole seconds of the action,
# and the test's log, here Python unittest's.
GENERATED_XML = """<?xml version="1.0" encoding="UTF-8"?>
<testsuites>
  <testsuite name="pkg/sim/startup_test" tests="1" failures="0" errors="0">
    <testcase name="pkg/sim/startup_test" status="run" duration="9" time="9"></testcase>
    <system-out><![CDATA[.....
----------------------------------------------------------------------
Ran 5 tests in 9.800s

OK]]></system-out>
  </testsuite>
</testsuites>
"""

FAST_XML = GENERATED_XML.replace('"9"', '"1"').replace("9.800s", "1.2s")

# What Bazel writes for a shell test: whole seconds, no unittest summary.
SHELL_XML = """<?xml version="1.0" encoding="UTF-8"?>
<testsuites>
  <testsuite name="pkg/sh_test" tests="1" failures="0" errors="0">
    <testcase name="pkg/sh_test" status="run" duration="5" time="5"></testcase>
    <system-out><![CDATA[PASS]]></system-out>
  </testsuite>
</testsuites>
"""


def varint(value):
    out = bytearray()
    while True:
        out.append(value & 0x7F | (0x80 if value > 0x7F else 0))
        value >>= 7
        if not value:
            return bytes(out)


def cache_status(run_duration_millis):
    """A test.cache_status (Bazel's TestResultData) with the fields Bazel
    writes around run_duration_millis (12): test_passed (2), passed_log (10),
    test_process_times (11), start_time_millis_epoch (15)."""
    log = b"/execroot/_main/bazel-out/k8-fastbuild/testlogs/pkg/test.log"
    return (b"\x10\x01" + b"\x52" + varint(len(log)) + log +
            b"\x58" + varint(run_duration_millis) +
            b"\x60" + varint(run_duration_millis) +
            b"\x78" + varint(1794541817011))


def write_tree(root, files):
    for path, content in files:
        full = os.path.join(root, path)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, "wb" if isinstance(content, bytes) else "w") as handle:
            handle.write(content)


class SlowTestsTest(unittest.TestCase):
    def test_testlogs_report_each_action_over_the_threshold_slowest_first(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("TEST_TMPDIR")) as root:
            for path, text in (
                    ("talOS/launcher/replay_inputs_test/test.xml", GTEST_XML),
                    ("pkg/sim/startup_test/shard_2_of_3/test.xml", GENERATED_XML),
                    ("pkg/sim/fast_test/test.xml", FAST_XML),
                    ("pkg/sim/broken_test/test.xml", "<testsuites")):
                os.makedirs(os.path.dirname(os.path.join(root, path)), exist_ok=True)
                with open(os.path.join(root, path), "w", encoding="utf-8") as handle:
                    handle.write(text)
            results = slow_tests.from_testlogs(root)
            out = io.StringIO()
            slow = slow_tests.report(results, 3.0, out)
        self.assertEqual(slow, [
            ("//talOS/launcher:replay_inputs_test", 12.164),
            ("//pkg/sim:startup_test (shard_2_of_3)", 9.8)])
        self.assertIn("2 of 3 test actions over 3 s", out.getvalue())

    def test_bep_reads_milliseconds_names_the_run_and_takes_lanes_from_tags(self):
        lines = [
            json.dumps({"id": {"targetCompleted": {"label": "//a:slow_test"}},
                        "completed": {"success": True,
                                      "tag": ["integration", "exclusive"]}}),
            json.dumps({"id": {"testResult": {"label": "//a:slow_test", "run": 2,
                                              "shard": 1, "attempt": 1}},
                        "testResult": {"testAttemptDurationMillis": "4250"}}),
            json.dumps({"id": {"targetCompleted": {"label": "//a:new_test"}},
                        "completed": {"success": True, "tag": ["no_tsan"]}}),
            json.dumps({"id": {"testResult": {"label": "//a:new_test", "run": 1,
                                              "shard": 1, "attempt": 1}},
                        "testResult": {"testAttemptDuration": "3.5s"}}),
            json.dumps({"id": {"progress": {}}, "progress": {}}),
            ""]
        results, lanes = slow_tests.from_bep(lines)
        self.assertEqual(results, [("//a:slow_test (run 2)", 4.25),
                                   ("//a:new_test", 3.5)])
        self.assertEqual(lanes, {"//a:slow_test": "integration",
                                 "//a:new_test": "fast"})

    def test_check_holds_untagged_tests_to_five_seconds_and_tagged_to_their_lane(self):
        results = [("//a:unit_test", 5.5), ("//a:ok_test", 4.9),
                   ("//a:sim_test (shard_1_of_2)", 29.0), ("//a:sim_test", 31.0),
                   ("//a:render_test", 100.0)]
        lanes = slow_tests.read_lanes(io.StringIO(
            "//a:sim_test integration\n//a:render_test heavy\n"))
        self.assertEqual(slow_tests.over_budget(results, lanes), [
            ("//a:sim_test", 31.0, "integration"),
            ("//a:unit_test", 5.5, "fast")])

    def test_check_fails_only_on_the_enforced_lanes(self):
        # CI enforces the lanes a hosted runner can hold; an overrun in
        # another lane is printed and does not fail the check.
        with tempfile.TemporaryDirectory(dir=os.environ.get("TEST_TMPDIR")) as root:
            write_tree(root, (
                ("lanes.txt", "//pkg:sim_test integration\n"),
                ("testlogs/pkg/sh_test/test.cache_status", cache_status(5900)),
                ("testlogs/pkg/sim_test/test.cache_status", cache_status(1000))))
            args = ["--check", "--lanes", os.path.join(root, "lanes.txt"),
                    "--testlogs", os.path.join(root, "testlogs")]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(slow_tests.main(args), 1)
                self.assertEqual(slow_tests.main(
                    args + ["--enforce", "integration"]), 0)

    def test_check_times_an_action_to_the_millisecond(self):
        # A shell test that ran 5.9 s: Bazel's own test.xml says 5, under the
        # fast lane's 5 s, so whole seconds hid the overrun. The action's
        # test.cache_status holds its run time in milliseconds.
        with tempfile.TemporaryDirectory(dir=os.environ.get("TEST_TMPDIR")) as root:
            write_tree(root, (("pkg/sh_test/test.xml", SHELL_XML),
                              ("pkg/sh_test/test.cache_status", cache_status(5900))))
            results = slow_tests.from_testlogs(root)
        self.assertEqual(results, [("//pkg:sh_test", 5.9)])
        self.assertEqual(slow_tests.over_budget(results, {}),
                         [("//pkg:sh_test", 5.9, "fast")])

    def test_check_fails_when_it_finds_no_test_results(self):
        # A later bazel invocation can point bazel-testlogs at an empty tree
        # (another configuration's): "0 of 0 over budget" is not a pass.
        with tempfile.TemporaryDirectory(dir=os.environ.get("TEST_TMPDIR")) as root:
            lanes = os.path.join(root, "lanes.txt")
            write_tree(root, (("lanes.txt", ""), ("testlogs/pkg/BUILD", "")))
            with self.assertRaises(SystemExit) as raised, \
                    contextlib.redirect_stderr(io.StringIO()):
                slow_tests.main(["--check", "--lanes", lanes,
                                 "--testlogs", os.path.join(root, "testlogs")])
        self.assertNotEqual(raised.exception.code, 0)


if __name__ == "__main__":
    unittest.main()
