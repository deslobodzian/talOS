"""Unit tests for tools/lint_test_time.py on in-memory files."""

import io
import unittest

from tools import lint_test_time

# Built by concatenation so this file holds no hit of its own.
SLEEP = "std::this_thread::sleep" + "_for(1ms);\n"
PY_SLEEP = "time.sleep" + "(0.1)\n"
CLOCK = "auto t = std::chrono::steady_clock::" + "now();\n"
ALIAS_CLOCK = ("using Clock = std::chrono::steady_clock; auto t = Clock::" +
               "now();\n")
PY_CLOCK = "t = time." + "time()\n"
FLAKY = "    flaky = " + "True,\n"
ATTEMPTS = "bazel test --flaky" + "_test_attempts=3 //...\n"

NODE_BUILD = 'talos_cc_node(\n    name = "node",\n)\n'


def lint(files, allowlist=""):
    out = io.StringIO()
    failures = lint_test_time.lint(sorted(files), files.get, allowlist, out)
    return failures, out.getvalue()


class LintTestTimeTest(unittest.TestCase):
    # REQ-RT-17
    def test_new_wall_clock_or_retry_fails_naming_the_file(self):
        for path, text, extra in (
                ("talOS/rtms/rtms_test.cc", SLEEP, {}),
                ("talOS/rtms/alias_test.cc", ALIAS_CLOCK, {}),
                ("controller_processor/rio/src/test/cpp/"
                 "Loop_test.cpp", CLOCK, {}),
                ("talOS/python/test_node.py", PY_CLOCK, {}),
                ("studio/bridge/thing_test.py", PY_SLEEP, {}),
                ("bench/main_processor/motor/motor.cc", CLOCK,
                 {"bench/main_processor/motor/BUILD": NODE_BUILD}),
                ("examples/arm/arm.py", PY_SLEEP,
                 {"examples/arm/BUILD.bazel":
                  "talos_py_node(name = 'node')\n"}),
                ("talOS/node/BUILD", FLAKY, {}),
                (".github/workflows/ci.yaml", ATTEMPTS, {}),
                (".bazelrc", "test " + ATTEMPTS.split(" ", 2)[2], {})):
            with self.subTest(path):
                failures, out = lint({path: text, **extra})
                self.assertEqual(failures, 1, out)
                self.assertIn(f"{path}:1:", out)

    def test_real_clock_outside_tests_and_nodes_passes(self):
        # Framework code, a package without a node, and comments may name
        # the clock; only tests and node code are held to node.now().
        failures, out = lint({
            "talOS/launcher/launcher.cc": SLEEP + CLOCK,
            "examples/lib/helper.cc": CLOCK,
            "examples/lib/BUILD": "cc_library(name = 'helper')\n",
            "talOS/rtms/rtms_test.cc": "// " + SLEEP + "# " + PY_CLOCK,
        })
        self.assertEqual(failures, 0, out)

    def test_allowlist_needs_a_reason_and_only_shrinks(self):
        files = {"talOS/launcher/launcher_test.cc": SLEEP}
        for allowlist, expected in (
                ("talOS/launcher/launcher_test.cc sleep integration: spawns",
                 0),
                ("talOS/launcher/launcher_test.cc sleep", 2),
                ("talOS/launcher/launcher_test.cc sleep spawns\n"
                 "talOS/launcher/launcher_test.cc clock gone", 1)):
            with self.subTest(allowlist):
                failures, out = lint(files, allowlist)
                self.assertEqual(failures, expected, out)

    # REQ-RT-17: a file allowed two waits may not gain a third, and once one
    # is gone the count comes down with it.
    def test_an_allowlist_count_bounds_the_hits_and_only_shrinks(self):
        path = "talOS/launcher/launcher_test.cc"
        files = {path: SLEEP + "int x;\n" + SLEEP}
        for count, expected, said in (("2", 0, "no new"),
                                      ("1", 1, f"{path}:1, 3: 2 sleep hits, "
                                               "1 allowed"),
                                      ("3", 1, "lower it to 2")):
            with self.subTest(count):
                failures, out = lint(files, f"{path} sleep {count} spawns")
                self.assertEqual(failures, expected, out)
                self.assertIn(said, out)


if __name__ == "__main__":
    unittest.main()
