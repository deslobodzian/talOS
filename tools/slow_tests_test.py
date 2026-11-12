"""Unit tests for tools/slow_tests.py on fixture test results."""

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

    def test_bep_reads_milliseconds_and_names_the_run(self):
        lines = [
            json.dumps({"id": {"testResult": {"label": "//a:slow_test", "run": 2,
                                              "shard": 1, "attempt": 1}},
                        "testResult": {"testAttemptDurationMillis": "4250"}}),
            json.dumps({"id": {"testResult": {"label": "//a:new_test", "run": 1,
                                              "shard": 1, "attempt": 1}},
                        "testResult": {"testAttemptDuration": "3.5s"}}),
            json.dumps({"id": {"progress": {}}, "progress": {}}),
            ""]
        self.assertEqual(slow_tests.from_bep(lines), [
            ("//a:slow_test (run 2)", 4.25), ("//a:new_test", 3.5)])


if __name__ == "__main__":
    unittest.main()
