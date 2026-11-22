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
TS_SLEEP = "await new Promise(r => set" + "Timeout(r, 150));\n"
TS_CLOCK = "const t = performance." + "now();\n"
FLAKY = "    flaky = " + "True,\n"
ATTEMPTS = "bazel test --flaky" + "_test_attempts=3 //...\n"

NODE_BUILD = 'talos_cc_node(\n    name = "node",\n)\n'
PY_NODE_BUILD = 'talos_py_node(\n    name = "node",\n)\n'

# Node-only wall-clock and randomness snippets (built by concatenation so this
# file holds no hit of its own).
CLOCK_GETTIME = "clock_" + "gettime(CLOCK_MONOTONIC, &ts);\n"
GETTIMEOFDAY = "get" + "timeofday(&tv, nullptr);\n"
POLLER_NOW = "auto t = talos::event::Poller::" + "now();\n"
POLLER_ALIAS = ("using P = talos::event::Poller; auto t = P::" + "now();\n")
STD_TIME = "auto t = std::" + "time(nullptr);\n"
PY_MONOTONIC_NS = "t = time." + "monotonic_ns()\n"
PY_TIME_NS = "t = time." + "time_ns()\n"
PY_PERF_COUNTER = "t = time." + "perf_counter()\n"
PY_PERF_COUNTER_NS = "t = time." + "perf_counter_ns()\n"
PY_PROCESS_TIME = "t = time." + "process_time()\n"
PY_DATETIME_NOW = "t = datetime." + "now()\n"
PY_DATETIME_UTCNOW = "t = datetime." + "utcnow()\n"
PY_FROM_TIME = "from " + "time import monotonic_ns\n"
PY_IMPORT_TIME = "import " + "time as wall_clock\n"

STD_RAND = "int x = std::" + "rand();\n"
BARE_RAND = "int x = " + "rand();\n"
STD_MT19937 = "std::" + "mt19937 rng{42};\n"
STD_MT19937_64 = "std::" + "mt19937_64 rng{42};\n"
STD_DEFAULT_RNG = "std::" + "default_random_engine rng{42};\n"
STD_MINSTD_RAND = "std::" + "minstd_rand rng{42};\n"
PY_RANDOM_CALL = "x = random." + "random()\n"
PY_NP_RANDOM = "x = np.random." + "rand()\n"
PY_IMPORT_RANDOM = "import " + "random\n"
PY_FROM_RANDOM = "from " + "random import randint\n"
PY_URANDOM = "b = os." + "urandom(8)\n"
PY_SECRETS = "b = secrets." + "token_bytes(8)\n"


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
                # A framework node's own source is node code.
                ("talOS/teleop/node.cc", SLEEP, {}),
                ("talOS/telemetry/node.cc", CLOCK, {}),
                ("studio/tests/agent.test.ts", TS_SLEEP, {}),
                ("studio/tests/view.test.tsx", TS_CLOCK, {}),
                ("talOS/node/BUILD", FLAKY, {}),
                (".github/workflows/ci.yaml", ATTEMPTS, {}),
                (".bazelrc", "test " + ATTEMPTS.split(" ", 2)[2], {})):
            with self.subTest(path):
                failures, out = lint({path: text, **extra})
                self.assertEqual(failures, 1, out)
                self.assertIn(f"{path}:1:", out)

    # REQ-RT-17: exact-basename test files (`test.cc`, `test.cpp`, `test.py`)
    # are test sources too; a non-test suffix like `contest.cc` is not.
    def test_exact_test_basename_sources_are_scanned(self):
        for path, text in (
                ("talOS/rtms/test.cc", CLOCK),
                ("talOS/rtms/test.cc", SLEEP),
                ("controller_processor/rio/src/test/cpp/test.cpp", CLOCK),
                ("talOS/python/test.py", PY_SLEEP),
                ("test.cc", CLOCK)):
            with self.subTest(path=path, text=text.strip()):
                failures, out = lint({path: text})
                self.assertEqual(failures, 1, out)
                self.assertIn(f"{path}:1:", out)
        failures, out = lint({"talOS/rtms/contest.cc": CLOCK + SLEEP})
        self.assertEqual(failures, 0, out)

    # REQ-RT-17: a wait moved out of a test into the header or module it
    # includes is still the test's wait.
    def test_a_test_helper_is_held_to_the_same_rule(self):
        for path, text in (
                ("talOS/protocol/udp_test_helpers.h", SLEEP),
                ("talOS/bridge/node_test_support.h", CLOCK),
                ("talOS/events/log/gap_testing.h", ALIAS_CLOCK),
                ("bench/main_processor/deploy_test_support.py", PY_SLEEP),
                ("talOS/node/testing.h", CLOCK),
                ("talOS/launcher/testdata/supervision_events.h", CLOCK),
                ("talOS/python/talos/testing.py", PY_CLOCK)):
            with self.subTest(path):
                failures, out = lint({path: text})
                self.assertEqual(failures, 1, out)
                self.assertIn(f"{path}:1:", out)

    def test_real_clock_outside_tests_and_nodes_passes(self):
        # Framework code, a package without a node, and comments may name
        # the clock; only tests and node code are held to node.now().
        failures, out = lint({
            "talOS/launcher/launcher.cc": SLEEP + CLOCK,
            "examples/lib/helper.cc": CLOCK,
            "examples/lib/BUILD": "cc_library(name = 'helper')\n",
            "talOS/rtms/rtms_test.cc": (
                "// " + SLEEP +
                "/* " + CLOCK.rstrip("\n") + " */\n" +
                "/*\n" + SLEEP + " * " + CLOCK + " */\n"
            ),
            "talOS/python/test_comment.py": "# " + PY_CLOCK + "x = 1  # " + PY_SLEEP,
            "studio/src/worker.ts": TS_SLEEP + TS_CLOCK,
            "studio/tests/view.test.tsx": (
                "// " + TS_SLEEP +
                "/*\n" + TS_CLOCK + "*/\n"
            ),
        })
        self.assertEqual(failures, 0, out)

    # REQ-RT-17: inline block comments and leading `*` dereferences must not
    # hide real-clock or sleep calls.
    def test_block_comment_prefix_and_pointer_deref_do_not_hide_hits(self):
        ptr_clock = "*ptr = std::chrono::steady_clock::" + "now();\n"
        cases = (
            ("inline_block_comment_before_sleep",
             "talOS/rtms/rtms_test.cc", "/* ok */ " + SLEEP, 1),
            ("pointer_deref_clock",
             "talOS/rtms/rtms_test.cc", ptr_clock, 1),
            ("multiline_block_comment_trailing_code",
             "talOS/rtms/rtms_test.cc", "/* line 1\n * line 2 */ " + SLEEP, 2),
            ("code_before_block_comment",
             "talOS/rtms/rtms_test.cc", SLEEP.rstrip("\n") + " /* ok\n*/\n", 1),
            ("ts_inline_block_comment",
             "studio/tests/agent.test.ts", "/* ok */ " + TS_SLEEP, 1),
            ("py_code_before_hash_comment",
             "talOS/python/test_node.py", PY_SLEEP.rstrip("\n") + "  # ok\n", 1),
            ("cpp_define_not_a_comment",
             "talOS/rtms/rtms_test.cc", "#define WAIT() " + SLEEP, 1),
        )
        for name, path, text, expected_line in cases:
            with self.subTest(name):
                failures, out = lint({path: text})
                self.assertEqual(failures, 1, out)
                self.assertIn(f"{path}:{expected_line}:", out)

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


    # REQ-RT-17: node code may not read any wall clock (POSIX, Poller, Python
    # high-res/datetime clocks) or make random decisions.
    def test_node_code_catches_wall_clock_and_random_apis(self):
        cc_node = ("bench/main_processor/motor/motor.cc",
                   {"bench/main_processor/motor/BUILD": NODE_BUILD})
        py_node = ("examples/arm/arm.py",
                   {"examples/arm/BUILD.bazel": PY_NODE_BUILD})
        framework_node = ("talOS/teleop/node.cc", {})
        cases = (
            ("clock_gettime", *cc_node, CLOCK_GETTIME, "clock"),
            ("gettimeofday", *cc_node, GETTIMEOFDAY, "clock"),
            ("poller_now", *cc_node, POLLER_NOW, "clock"),
            ("poller_alias", *cc_node, POLLER_ALIAS, "clock"),
            ("std_time", *cc_node, STD_TIME, "clock"),
            ("framework_clock_gettime", *framework_node, CLOCK_GETTIME,
             "clock"),
            ("py_monotonic_ns", *py_node, PY_MONOTONIC_NS, "clock"),
            ("py_time_ns", *py_node, PY_TIME_NS, "clock"),
            ("py_perf_counter", *py_node, PY_PERF_COUNTER, "clock"),
            ("py_perf_counter_ns", *py_node, PY_PERF_COUNTER_NS, "clock"),
            ("py_process_time", *py_node, PY_PROCESS_TIME, "clock"),
            ("py_datetime_now", *py_node, PY_DATETIME_NOW, "clock"),
            ("py_datetime_utcnow", *py_node, PY_DATETIME_UTCNOW, "clock"),
            ("py_from_time", *py_node, PY_FROM_TIME, "clock"),
            ("py_import_time", *py_node, PY_IMPORT_TIME, "clock"),
            ("std_rand", *cc_node, STD_RAND, "random"),
            ("bare_rand", *cc_node, BARE_RAND, "random"),
            ("std_mt19937", *cc_node, STD_MT19937, "random"),
            ("std_mt19937_64", *cc_node, STD_MT19937_64, "random"),
            ("std_default_random_engine", *cc_node, STD_DEFAULT_RNG, "random"),
            ("std_minstd_rand", *cc_node, STD_MINSTD_RAND, "random"),
            ("framework_mt19937", *framework_node, STD_MT19937, "random"),
            ("py_random_call", *py_node, PY_RANDOM_CALL, "random"),
            ("py_np_random", *py_node, PY_NP_RANDOM, "random"),
            ("py_import_random", *py_node, PY_IMPORT_RANDOM, "random"),
            ("py_from_random", *py_node, PY_FROM_RANDOM, "random"),
            ("py_os_urandom", *py_node, PY_URANDOM, "random"),
            ("py_secrets", *py_node, PY_SECRETS, "random"),
        )
        for name, path, extra, text, kind in cases:
            with self.subTest(name):
                failures, out = lint({path: text, **extra})
                self.assertEqual(failures, 1, out)
                self.assertIn(f"{path}:1: {kind}", out)

    # REQ-RT-17: a clock alias in node code must not drop the node-only wall
    # clock patterns on the same file.
    def test_clock_alias_preserves_node_clock_patterns(self):
        path = "bench/main_processor/motor/motor.cc"
        text = ("using Clock = std::chrono::steady_clock;\n" +
                CLOCK_GETTIME)
        failures, out = lint({
            path: text,
            "bench/main_processor/motor/BUILD": NODE_BUILD,
        })
        self.assertEqual(failures, 1, out)
        self.assertIn(f"{path}:2: clock", out)

    # REQ-RT-17: node-only wall-clock and randomness patterns apply to node
    # code, not test sources (even inside a node package), comments, or
    # unrelated C++ member names.
    def test_node_only_patterns_exclude_tests_comments_and_non_node_code(self):
        failures, out = lint({
            "bench/main_processor/motor/BUILD": NODE_BUILD,
            # A test inside a node package is a test source, not node code:
            # seeded RNG and Poller::now() belong to tests, not node code.
            "bench/main_processor/motor/node_test.cc": (
                STD_MT19937 + STD_DEFAULT_RNG + POLLER_NOW + CLOCK_GETTIME
            ),
            "talOS/python/messages_test.py": (
                PY_IMPORT_RANDOM + PY_RANDOM_CALL + PY_TIME_NS
            ),
            # Non-node library code is unscanned.
            "examples/lib/BUILD": "cc_library(name = 'helper')\n",
            "examples/lib/helper.cc": CLOCK_GETTIME + STD_MT19937 + STD_RAND,
            # Comments and unrelated C++ member accesses do not hit.
            "bench/main_processor/motor/motor.cc": (
                "// " + CLOCK_GETTIME +
                "/* " + STD_MT19937.rstrip("\n") + " */\n" +
                "int a = helper.rand();\n" +
                "int b = ptr->rand();\n" +
                "int c = random.value;\n"
            ),
            "examples/arm/BUILD.bazel": PY_NODE_BUILD,
            "examples/arm/arm.py": (
                "# " + PY_DATETIME_NOW +
                "# " + PY_RANDOM_CALL +
                "from mypkg import time\n" +
                "from mypkg import random\n"
            ),
        })
        self.assertEqual(failures, 0, out)

    # REQ-RT-17: random hits follow the same allowlist format and shrink-only
    # count enforcement as sleep and clock.
    def test_random_allowlist_entry_is_supported(self):
        path = "bench/main_processor/motor/motor.cc"
        files = {
            path: STD_MT19937,
            "bench/main_processor/motor/BUILD": NODE_BUILD,
        }
        failures, out = lint(files, f"{path} random 1 documented exception")
        self.assertEqual(failures, 0, out)


if __name__ == "__main__":
    unittest.main()
