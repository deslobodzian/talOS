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

# Node-only blocking I/O and lock snippets (RULES.md 3; METHODOLOGY.md §10).
STD_COUT = "std::" + "cout << 1;\n"
STD_CERR = "std::" + "cerr << 1;\n"
STD_CLOG = "std::" + "clog << 1;\n"
STD_CIN = "std::" + "cin >> x;\n"
BARE_PRINTF = "prin" + 'tf("%d\\n", x);\n'
STD_PRINTF = "std::prin" + 'tf("%d\\n", x);\n'
GLOBAL_PRINTF = "::prin" + 'tf("%d\\n", x);\n'
STD_FPRINTF = "std::fprin" + 'tf(stderr, "%s\\n", msg);\n'
BARE_PUTS = "pu" + 'ts("hi");\n'
BARE_FOPEN = "FILE* f = fo" + 'pen("a.txt", "r");\n'
STD_IFSTREAM = "std::if" + 'stream in("a.txt");\n'
STD_OFSTREAM = "std::of" + 'stream out("a.txt");\n'
STD_FSTREAM = "std::f" + 'stream io("a.txt");\n'
POSIX_SOCKET = "int fd = sock" + "et(AF_INET, SOCK_DGRAM, 0);\n"
POSIX_GLOBAL_SOCKET = "int fd = ::sock" + "et(AF_INET, SOCK_DGRAM, 0);\n"
POSIX_BIND = "bi" + "nd(fd, addr, len);\n"
POSIX_LISTEN = "list" + "en(fd, 5);\n"
POSIX_ACCEPT = "acc" + "ept(fd, nullptr, nullptr);\n"
POSIX_CONNECT = "conn" + "ect(fd, addr, len);\n"
POSIX_SENDTO = "send" + "to(fd, buf, n, 0, addr, len);\n"
POSIX_RECVFROM = "recv" + "from(fd, buf, n, 0, addr, &len);\n"

PY_PRINT = "pri" + 'nt("hi")\n'
PY_OPEN = "f = op" + 'en("a.txt")\n'
PY_FSTRING_OPEN = 's = f"{op' + "en('a.txt')}\"\n"
PY_IMPORT_SOCKET = "import " + "socket\n"
PY_IMPORT_URLLIB = "import " + "urllib.request\n"
PY_IMPORT_REQUESTS = "import " + "requests\n"
PY_IMPORT_SUBPROCESS = "import " + "subprocess\n"
PY_IMPORT_LOGGING = "import " + "logging\n"
PY_IMPORT_MULTI_LOGGING = "import os, " + "logging as log\n"
PY_FROM_SOCKET = "from " + "socket import AF_INET\n"
PY_FROM_URLLIB = "from " + "urllib.request import urlopen\n"
PY_FROM_REQUESTS = "from " + "requests import get\n"
PY_FROM_SUBPROCESS = "from " + "subprocess import run\n"
PY_FROM_LOGGING = "from " + "logging import getLogger\n"

STD_MUTEX = "std::" + "mutex mu_;\n"
STD_SHARED_MUTEX = "std::" + "shared_mutex mu_;\n"
STD_RECURSIVE_MUTEX = "std::" + "recursive_mutex mu_;\n"
STD_LOCK_GUARD = "std::" + "lock_guard<int> lk(mu_);\n"
STD_UNIQUE_LOCK = "std::" + "unique_lock<int> lk(mu_);\n"
STD_SCOPED_LOCK = "std::" + "scoped_lock lk(mu_);\n"
STD_CONDVAR = "std::" + "condition_variable cv_;\n"
PTHREAD_LOCK = "pthread_" + "mutex_lock(&mu_);\n"

PY_THREADING_LOCK = "mu = threading." + "Lock()\n"
PY_THREADING_RLOCK = "mu = threading." + "RLock()\n"
PY_THREADING_SEMAPHORE = "sem = threading." + "Semaphore(1)\n"
PY_THREADING_CONDITION = "cv = threading." + "Condition()\n"
PY_FROM_THREADING_LOCK = "from threading import " + "Lock\n"
PY_FROM_THREADING_MULTI = "from threading import Event, " + "RLock\n"


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

    # REQ-RT-06, REQ-RT-17: node code may not do blocking I/O, logging, print,
    # or take locks (RULES.md 3; METHODOLOGY.md §10).
    def test_node_code_catches_io_and_lock_apis(self):
        cc_node = ("bench/main_processor/motor/motor.cc",
                   {"bench/main_processor/motor/BUILD": NODE_BUILD})
        py_node = ("examples/arm/arm.py",
                   {"examples/arm/BUILD.bazel": PY_NODE_BUILD})
        framework_node = ("talOS/teleop/node.cc", {})
        cases = (
            ("std_cout", *cc_node, STD_COUT, "io"),
            ("std_cerr", *cc_node, STD_CERR, "io"),
            ("std_clog", *cc_node, STD_CLOG, "io"),
            ("std_cin", *cc_node, STD_CIN, "io"),
            ("bare_printf", *cc_node, BARE_PRINTF, "io"),
            ("std_printf", *cc_node, STD_PRINTF, "io"),
            ("global_printf", *cc_node, GLOBAL_PRINTF, "io"),
            ("std_fprintf", *cc_node, STD_FPRINTF, "io"),
            ("bare_puts", *cc_node, BARE_PUTS, "io"),
            ("bare_fopen", *cc_node, BARE_FOPEN, "io"),
            ("std_ifstream", *cc_node, STD_IFSTREAM, "io"),
            ("std_ofstream", *cc_node, STD_OFSTREAM, "io"),
            ("std_fstream", *cc_node, STD_FSTREAM, "io"),
            ("posix_socket", *cc_node, POSIX_SOCKET, "io"),
            ("posix_global_socket", *cc_node, POSIX_GLOBAL_SOCKET, "io"),
            ("posix_bind", *cc_node, POSIX_BIND, "io"),
            ("posix_listen", *cc_node, POSIX_LISTEN, "io"),
            ("posix_accept", *cc_node, POSIX_ACCEPT, "io"),
            ("posix_connect", *cc_node, POSIX_CONNECT, "io"),
            ("posix_sendto", *cc_node, POSIX_SENDTO, "io"),
            ("posix_recvfrom", *cc_node, POSIX_RECVFROM, "io"),
            ("framework_std_cerr", *framework_node, STD_CERR, "io"),
            ("py_print", *py_node, PY_PRINT, "io"),
            ("py_open", *py_node, PY_OPEN, "io"),
            ("py_fstring_open", *py_node, PY_FSTRING_OPEN, "io"),
            ("py_import_socket", *py_node, PY_IMPORT_SOCKET, "io"),
            ("py_import_urllib", *py_node, PY_IMPORT_URLLIB, "io"),
            ("py_import_requests", *py_node, PY_IMPORT_REQUESTS, "io"),
            ("py_import_subprocess", *py_node, PY_IMPORT_SUBPROCESS, "io"),
            ("py_import_logging", *py_node, PY_IMPORT_LOGGING, "io"),
            ("py_import_multi_logging", *py_node, PY_IMPORT_MULTI_LOGGING,
             "io"),
            ("py_from_socket", *py_node, PY_FROM_SOCKET, "io"),
            ("py_from_urllib", *py_node, PY_FROM_URLLIB, "io"),
            ("py_from_requests", *py_node, PY_FROM_REQUESTS, "io"),
            ("py_from_subprocess", *py_node, PY_FROM_SUBPROCESS, "io"),
            ("py_from_logging", *py_node, PY_FROM_LOGGING, "io"),
            ("std_mutex", *cc_node, STD_MUTEX, "lock"),
            ("std_shared_mutex", *cc_node, STD_SHARED_MUTEX, "lock"),
            ("std_recursive_mutex", *cc_node, STD_RECURSIVE_MUTEX, "lock"),
            ("std_lock_guard", *cc_node, STD_LOCK_GUARD, "lock"),
            ("std_unique_lock", *cc_node, STD_UNIQUE_LOCK, "lock"),
            ("std_scoped_lock", *cc_node, STD_SCOPED_LOCK, "lock"),
            ("std_condvar", *cc_node, STD_CONDVAR, "lock"),
            ("pthread_mutex_lock", *cc_node, PTHREAD_LOCK, "lock"),
            ("framework_std_mutex", *framework_node, STD_MUTEX, "lock"),
            ("inline_comment_before_mutex", *cc_node, "/* ok */ " + STD_MUTEX,
             "lock"),
            ("py_threading_lock", *py_node, PY_THREADING_LOCK, "lock"),
            ("py_threading_rlock", *py_node, PY_THREADING_RLOCK, "lock"),
            ("py_threading_semaphore", *py_node, PY_THREADING_SEMAPHORE,
             "lock"),
            ("py_threading_condition", *py_node, PY_THREADING_CONDITION,
             "lock"),
            ("py_from_threading_lock", *py_node, PY_FROM_THREADING_LOCK,
             "lock"),
            ("py_from_threading_multi", *py_node, PY_FROM_THREADING_MULTI,
             "lock"),
        )
        for name, path, extra, text, kind in cases:
            with self.subTest(name):
                failures, out = lint({path: text, **extra})
                self.assertEqual(failures, 1, out)
                self.assertIn(f"{path}:1: {kind}", out)

    # REQ-RT-06, REQ-RT-17: io and lock patterns exclude test sources,
    # non-node libraries, comments, non-executing strings, member calls,
    # qualified non-POSIX calls, and unrelated identifiers.
    def test_node_io_and_lock_exclude_tests_comments_strings_and_members(self):
        failures, out = lint({
            "bench/main_processor/motor/BUILD": NODE_BUILD,
            # Test sources in a node package are not held to node io/lock rules.
            "bench/main_processor/motor/node_test.cc": (
                STD_COUT + STD_MUTEX + POSIX_SOCKET + PTHREAD_LOCK
            ),
            "examples/arm/BUILD.bazel": PY_NODE_BUILD,
            "examples/arm/arm_test.py": (
                PY_PRINT + PY_OPEN + PY_IMPORT_LOGGING + PY_THREADING_LOCK
            ),
            # Non-node library code is unscanned.
            "examples/lib/BUILD": "cc_library(name = 'helper')\n",
            "examples/lib/helper.cc": STD_COUT + STD_MUTEX + BARE_PRINTF,
            # C++ comments, strings, raw strings, headers, member calls, and
            # non-POSIX qualified calls do not hit.
            "bench/main_processor/motor/motor.cc": (
                "#include <fstream>\n"
                "#include <mutex>\n"
                "#include <sys/socket.h>\n"
                "// " + STD_COUT +
                "// " + STD_MUTEX +
                "/* " + POSIX_CONNECT.rstrip("\n") + " */\n"
                "/*\n * " + STD_LOCK_GUARD + " */\n"
                'const char* a = "' + STD_COUT.rstrip("\n") + '";\n'
                'const char* b = "' + STD_MUTEX.rstrip("\n") + '";\n'
                'const char* c = R"(std::ifstream x; connect(fd, a, l);)";\n'
                "obj.connect(1);\n"
                "ptr->bind(1);\n"
                "obj.listen(1);\n"
                "ptr->accept(1);\n"
                "obj.printf(\"ok\");\n"
                "ptr->fopen(\"x\", \"r\");\n"
                "auto fn = std::bind(&Foo::bar, this);\n"
                "my_ns::connect(1);\n"
                "std::ostringstream ss;\n"
                "std::snprintf(buf, sizeof(buf), \"%d\", 1);\n"
                "int mutex = 0;\n"
            ),
            # Python comments, plain/triple/raw strings, method calls, method
            # definitions, and unrelated imports do not hit.
            "examples/arm/arm.py": (
                "# " + PY_PRINT +
                "# " + PY_OPEN +
                "# " + PY_THREADING_LOCK +
                'msg = "' + PY_PRINT.rstrip("\n") + '"\n'
                "path = '" + PY_OPEN.rstrip("\n") + "'\n"
                'doc = """\n' + PY_IMPORT_SOCKET + PY_THREADING_LOCK + '"""\n'
                'raw = r"open(x) print(y)"\n'
                "class Gripper:\n"
                "    def open(self):\n"
                "        pass\n"
                "obj.open('x')\n"
                "self.print('x')\n"
                "reopen('x')\n"
                "from mypkg import socket\n"
                "from mypkg import logging\n"
                "from mypkg import Lock\n"
                "import my_logging\n"
                "x = obj.Lock()\n"
            ),
        })
        self.assertEqual(failures, 0, out)

    # REQ-RT-06, REQ-RT-17: io and lock allowlist entries enforce exact
    # shrink-only counts just like sleep, clock, and random.
    def test_io_and_lock_allowlist_counts_only_shrink(self):
        path = "bench/main_processor/motor/motor.cc"
        extra = {"bench/main_processor/motor/BUILD": NODE_BUILD}
        for kind, snippet in (("io", STD_CERR), ("lock", STD_MUTEX)):
            files = {path: snippet + "int x;\n" + snippet, **extra}
            for count, expected, said in (
                    ("2", 0, "no new"),
                    ("1", 1, f"{path}:1, 3: 2 {kind} hits, 1 allowed"),
                    ("3", 1, "lower it to 2")):
                with self.subTest(kind=kind, count=count):
                    failures, out = lint(
                        files, f"{path} {kind} {count} construction-only")
                    self.assertEqual(failures, expected, out)
                    self.assertIn(said, out)
            with self.subTest(kind=kind, case="stale"):
                failures, out = lint(
                    {path: "int x;\n", **extra},
                    f"{path} {kind} 1 construction-only")
                self.assertEqual(failures, 1, out)
                self.assertIn(f"stale entry `{path} {kind}`", out)

    # REQ-RT-17: files in subdirectories of a node package belong to that node
    # package up to any nested BUILD / BUILD.bazel boundary.
    def test_node_package_subdirectories_scanned_up_to_nested_build_boundary(
            self):
        for path, text, kind, extra in (
                ("bench/arm/detail/helper.cc", CLOCK_GETTIME, "clock",
                 {"bench/arm/BUILD": NODE_BUILD}),
                ("bench/arm/sub/deep/helper.h", STD_MUTEX, "lock",
                 {"bench/arm/BUILD": NODE_BUILD}),
                ("examples/arm/internal/util.py", PY_IMPORT_RANDOM, "random",
                 {"examples/arm/BUILD.bazel": PY_NODE_BUILD}),
                ("talOS/teleop/detail/helper.cc", STD_CERR, "io", {})):
            with self.subTest(caught=path):
                failures, out = lint({path: text, **extra})
                self.assertEqual(failures, 1, out)
                self.assertIn(f"{path}:1: {kind}", out)

        # A nested BUILD or BUILD.bazel starts a separate non-node package.
        failures, out = lint({
            "bench/arm/BUILD": NODE_BUILD,
            "bench/arm/subpkg/BUILD": 'cc_library(name = "sub")\n',
            "bench/arm/subpkg/tool.cc": CLOCK_GETTIME + STD_MUTEX + STD_COUT,
            "bench/arm/subpkg/inner/tool.cc": STD_RAND + SLEEP,
            "talOS/teleop/subpkg/BUILD.bazel": 'cc_library(name = "sub")\n',
            "talOS/teleop/subpkg/helper.cc": STD_CERR + STD_MUTEX,
        })
        self.assertEqual(failures, 0, out)

    # REQ-RT-17: multiline invocations split across lines (or comments) are
    # still caught for clock, sleep, random, io, and lock.
    def test_multiline_invocations_are_caught(self):
        cc_node = ("bench/main_processor/motor/motor.cc",
                   {"bench/main_processor/motor/BUILD": NODE_BUILD})
        py_node = ("examples/arm/arm.py",
                   {"examples/arm/BUILD.bazel": PY_NODE_BUILD})
        cases = (
            ("cc_multiline_clock", *cc_node,
             "auto t = std::chrono::steady_clock::\n    " + "now();\n",
             "clock", 1),
            ("cc_multiline_sleep", *cc_node,
             "std::this_thread::sleep" + "_for\n    (1ms);\n",
             "sleep", 1),
            ("cc_multiline_random", *cc_node,
             "int x = std::" + "rand\n    ();\n",
             "random", 1),
            ("cc_multiline_io_stream", *cc_node,
             "std::\n    " + "cout << 1;\n",
             "io", 1),
            ("cc_multiline_io_printf", *cc_node,
             "prin" + 'tf\n    ("%d", x);\n',
             "io", 1),
            ("cc_multiline_lock", *cc_node,
             "std::\n    " + "mutex mu_;\n",
             "lock", 1),
            ("py_multiline_clock", *py_node,
             "t = (\n    time\n    ." + "monotonic()\n)\n",
             "clock", 2),
            ("py_multiline_sleep", *py_node,
             "time.sleep \\\n    " + "(0.1)\n",
             "sleep", 1),
            ("py_multiline_from_os_urandom", *py_node,
             "from os import (\n    path,\n    " + "urandom,\n)\n",
             "random", 1),
            ("py_multiline_io_print", *py_node,
             "pri" + 'nt \\\n    ("hi")\n',
             "io", 1),
            ("py_multiline_from_threading_lock", *py_node,
             "from threading import (\n    Event,\n    " + "Lock,\n)\n",
             "lock", 1),
            ("py_multiline_threading_lock_attr", *py_node,
             "mu = (\n    threading\n    ." + "Lock()\n)\n",
             "lock", 2),
        )
        for name, path, extra, text, kind, expected_line in cases:
            with self.subTest(name=name):
                failures, out = lint({path: text, **extra})
                self.assertEqual(failures, 1, out)
                self.assertIn(f"{path}:{expected_line}: {kind}", out)

    # REQ-RT-17: C++ line splices, prefixed raw strings, aliased node loads,
    # numeric flaky=1, and duplicate allowlist lines must not slip past.
    def test_lexer_bypasses_aliased_node_load_and_duplicate_allowlist_fail(self):
        # 1. C++ phase-2 backslash-newline splice inside an identifier.
        spliced_sleep = "std::this_thread::sleep_\\\n" + "for(1ms);\n"
        failures, out = lint({"talOS/rtms/rtms_test.cc": spliced_sleep})
        self.assertEqual(failures, 1, out)
        self.assertIn("talOS/rtms/rtms_test.cc:1: sleep", out)

        # 2. Prefixed C++ raw strings (`u8R`, `LR`, `uR`, `UR`) containing
        #    quotes and `/*` must not open a block comment that swallows EOF.
        for prefix in ("LR", "u8R", "uR", "UR"):
            with self.subTest(prefix=prefix):
                src = (
                    f'const auto* s = {prefix}"(quote " /* not a comment)";\n'
                    + SLEEP
                )
                failures, out = lint({"talOS/rtms/rtms_test.cc": src})
                self.assertEqual(failures, 1, out)
                self.assertIn("talOS/rtms/rtms_test.cc:2: sleep", out)

        # 3. Aliased `load()` of `talos_cc_node` still marks the node package.
        aliased_build = (
            'load("//talOS/build:contract.bzl", my_node = "talos_cc_node")\n'
            'my_node(name = "node", srcs = ["motor.cc"])\n'
        )
        failures, out = lint({
            "bench/main_processor/motor/BUILD": aliased_build,
            "bench/main_processor/motor/motor.cc": CLOCK_GETTIME,
        })
        self.assertEqual(failures, 1, out)
        self.assertIn("bench/main_processor/motor/motor.cc:1: clock", out)

        # 4. Bazel accepts integer `flaky = 1` as truthy on test rules.
        failures, out = lint({"talOS/node/BUILD": "    fla" + "ky = 1,\n"})
        self.assertEqual(failures, 1, out)
        self.assertIn("talOS/node/BUILD:1: retry", out)

        # 5. Duplicate allowlist entries for the same (path, kind) fail closed.
        path = "talOS/launcher/launcher_test.cc"
        dup_allowlist = (
            f"{path} sleep 1 first reason\n"
            f"{path} sleep 2 widened reason\n"
        )
        failures, out = lint({path: SLEEP + SLEEP}, dup_allowlist)
        self.assertGreaterEqual(failures, 1, out)
        self.assertIn("duplicate", out.lower())


if __name__ == "__main__":
    unittest.main()
