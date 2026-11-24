"""Unit tests for tools/check_testing_rules.py (docs/TESTING.md §6.7, §8.2)."""

import unittest

try:
    from tools import check_testing_rules as checker
except ModuleNotFoundError:
    import check_testing_rules as checker


VALID_SIM_CHECK = """
import os

def preserve_failure_artifacts(log, sessions):
    return os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR")

def check():
    return preserve_failure_artifacts("a", "b")
"""

VALID_PERF_CC = """
#include <gtest/gtest.h>
#include "talOS/process/host_qualification.h"

TEST(SamplePerf, FastEnough) {
  talos::process::StallMeter meter;
  const auto stall = meter.Stop();
  if (stall > 1000) {
    ::testing::Test::RecordProperty("host_unqualified", "true");
    FAIL() << "[HOST_UNQUALIFIED] stalled";
  }
}
"""


def run_check(file_map):
    files = list(file_map.keys())
    return checker.check(files, lambda p: file_map[p])


class PerfHostQualificationTest(unittest.TestCase):
    # REQ-DEV-25, REQ-RT-07
    def test_compliant_cpp_perf_target_passes(self):
        problems, stats = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "bench/BUILD": """
cc_test(
    name = "sample_perf_test",
    srcs = ["sample_perf_test.cc"],
    tags = [
        "exclusive",
        "perf",
    ],
    deps = [
        "//talOS/process:host_qualification",
        "@googletest//:gtest_main",
    ],
)
""",
            "bench/sample_perf_test.cc": VALID_PERF_CC,
        })
        self.assertEqual(problems, [])
        self.assertEqual(stats["perf"], 1)

    # REQ-DEV-25, REQ-RT-07
    def test_perf_target_inside_talos_process_accepts_relative_dep(self):
        problems, stats = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "talOS/process/BUILD": """
cc_test(
    name = "stall_perf_test",
    srcs = ["stall_perf_test.cc"],
    tags = ["exclusive", "perf"],
    deps = [":host_qualification"],
)
""",
            "talOS/process/stall_perf_test.cc": VALID_PERF_CC,
        })
        self.assertEqual(problems, [])
        self.assertEqual(stats["perf"], 1)

    # REQ-DEV-25, REQ-RT-07
    def test_perf_target_missing_host_qualification_dep_fails(self):
        problems, _ = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
cc_test(
    name = "missing_dep_perf_test",
    srcs = ["missing_dep_perf_test.cc"],
    tags = [
        "exclusive",
        "perf",
    ],
    deps = ["@googletest//:gtest_main"],
)
""",
            "pkg/missing_dep_perf_test.cc": VALID_PERF_CC,
        })
        self.assertEqual(len(problems), 1, problems)
        self.assertIn("//talOS/process:host_qualification", problems[0])
        self.assertIn("//pkg:missing_dep_perf_test", problems[0])

    # REQ-DEV-25, REQ-RT-07
    def test_perf_target_select_dep_only_in_one_branch_fails_closed(self):
        problems, _ = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
cc_test(
    name = "partial_select_perf_test",
    srcs = ["partial_select_perf_test.cc"],
    tags = ["perf"],
    deps = select({
        "@platforms//os:macos": ["//talOS/process:host_qualification"],
        "//conditions:default": [],
    }),
)
""",
            "pkg/partial_select_perf_test.cc": VALID_PERF_CC,
        })
        self.assertEqual(len(problems), 1, problems)
        self.assertIn("//talOS/process:host_qualification", problems[0])

    # REQ-DEV-25, REQ-RT-07
    def test_perf_target_select_dep_in_every_branch_or_concat_passes(self):
        problems, stats = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
cc_test(
    name = "all_branch_select_perf_test",
    srcs = ["a_perf_test.cc"],
    tags = ["perf"],
    deps = select({
        "@platforms//os:macos": ["//talOS/process:host_qualification", ":mac"],
        "//conditions:default": ["//talOS/process:host_qualification"],
    }),
)
cc_test(
    name = "concat_select_perf_test",
    srcs = ["a_perf_test.cc"],
    tags = ["perf"],
    deps = ["//talOS/process:host_qualification"] + select({
        "@platforms//os:macos": [":mac"],
        "//conditions:default": [],
    }),
)
""",
            "pkg/a_perf_test.cc": VALID_PERF_CC,
        })
        self.assertEqual(problems, [])
        self.assertEqual(stats["perf"], 2)

    # REQ-DEV-25, REQ-RT-07
    def test_perf_tag_hidden_in_select_or_variable_still_checked(self):
        problems, stats = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
EXTRA = ["per" + "f"]
cc_test(
    name = "select_tag_perf_test",
    srcs = ["bad_perf_test.cc"],
    tags = select({
        "@platforms//os:linux": EXTRA,
        "//conditions:default": [],
    }),
    deps = [],
)
""",
            "pkg/bad_perf_test.cc": "int main() { return 0; }\n",
        })
        self.assertEqual(stats["perf"], 1)
        self.assertTrue(any("host_qualification" in p for p in problems), problems)
        self.assertTrue(any("StallMeter" in p for p in problems), problems)
        self.assertTrue(any("[HOST_UNQUALIFIED]" in p for p in problems), problems)

    # REQ-DEV-25, REQ-RT-07
    def test_perf_comments_and_strings_cannot_spoof_stall_meter_or_marker(self):
        problems, _ = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
cc_test(
    name = "spoof_perf_test",
    srcs = ["spoof_perf_test.cc"],
    tags = ["perf"],
    deps = ["//talOS/process:host_qualification"],
)
""",
            "pkg/spoof_perf_test.cc": """
// StallMeter [HOST_UNQUALIFIED] "host_unqualified"
/* StallMeter [HOST_UNQUALIFIED] "host_unqualified" */
const char* s = "StallMeter";
int main() { return 0; }
""",
        })
        self.assertTrue(any("StallMeter" in p for p in problems), problems)
        self.assertTrue(any("[HOST_UNQUALIFIED]" in p for p in problems), problems)
        self.assertTrue(any('"host_unqualified"' in p for p in problems), problems)

    # REQ-DEV-25, REQ-RT-07
    def test_non_perf_targets_not_flagged(self):
        problems, stats = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
TAGS = {"agent_test": ["integration"]}
cc_test(
    name = "unit_test",
    srcs = ["unit_test.cc"],
    deps = ["@googletest//:gtest_main"],
)
cc_test(
    name = "heavy_test",
    srcs = ["heavy_test.cc"],
    tags = ["heavy"],
    deps = ["@googletest//:gtest_main"],
)
js_test(
    name = "agent_test",
    tags = TAGS.get("agent_test", []),
)
""",
            "pkg/unit_test.cc": "int main() {}\n",
            "pkg/heavy_test.cc": "int main() {}\n",
        })
        self.assertEqual(problems, [])
        self.assertEqual(stats, {"perf": 0, "e2e": 0})


class E2eArtifactPreservationTest(unittest.TestCase):
    # REQ-DEV-25, REQ-DEV-30
    def test_direct_undeclared_outputs_and_sim_check_delegation_pass(self):
        problems, stats = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
load("//talOS/build:contract.bzl", my_sim = "talos_sim_test")

py_test(
    name = "direct_e2e_test",
    srcs = ["direct_e2e_test.py"],
    tags = ["e2e", "exclusive"],
)

py_test(
    name = "delegated_e2e_test",
    srcs = ["delegated_e2e_test.py", "//talOS/build:sim_check.py"],
    tags = ["e2e"],
)

my_sim(
    name = "smoke_test",
    sim = ":sim",
)
""",
            "pkg/direct_e2e_test.py": (
                'import os\n'
                'OUT = os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR")\n'
            ),
            "pkg/delegated_e2e_test.py": (
                'from talOS.build import sim_check\n'
                'sim_check.preserve_failure_artifacts("a", "b")\n'
            ),
        })
        self.assertEqual(problems, [])
        self.assertEqual(stats["e2e"], 3)

    # REQ-DEV-25, REQ-DEV-30
    def test_e2e_importing_sim_check_without_calling_preserve_fails(self):
        # Guards against the false-pass where sim_check.py is in srcs only for
        # udp_ports()/Launch()/wait_ready() without preserving artifacts.
        problems, _ = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
py_test(
    name = "leaky_e2e_test",
    srcs = ["leaky_e2e_test.py", "//talOS/build:sim_check.py"],
    tags = ["e2e"],
)
""",
            "pkg/leaky_e2e_test.py": (
                'from talOS.build import sim_check\n'
                'sim_check.udp_ports()\n'
            ),
        })
        self.assertEqual(len(problems), 1, problems)
        self.assertIn("//pkg:leaky_e2e_test", problems[0])
        self.assertIn("TEST_UNDECLARED_OUTPUTS_DIR", problems[0])

    # REQ-DEV-25, REQ-DEV-30
    def test_e2e_docstring_or_comment_cannot_spoof_undeclared_outputs(self):
        problems, _ = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
py_test(
    name = "docstring_spoof_e2e_test",
    srcs = ["spoof.py"],
    tags = ["e2e"],
)
""",
            "pkg/spoof.py": (
                '"""TODO: preserve under TEST_UNDECLARED_OUTPUTS_DIR."""\n'
                '# TEST_UNDECLARED_OUTPUTS_DIR\n'
                'def test_something():\n'
                '    """TEST_UNDECLARED_OUTPUTS_DIR"""\n'
                '    pass\n'
            ),
        })
        self.assertEqual(len(problems), 1, problems)
        self.assertIn("//pkg:docstring_spoof_e2e_test", problems[0])

    # REQ-DEV-25, REQ-DEV-30
    def test_removing_undeclared_outputs_from_sim_check_fails_sim_tests(self):
        problems, _ = run_check({
            "talOS/build/sim_check.py": (
                'def preserve_failure_artifacts(a, b):\n    return None\n'
            ),
            "pkg/BUILD": """
load("//talOS/build:contract.bzl", "talos_sim_test")
talos_sim_test(name = "smoke_test", sim = ":sim")
py_test(
    name = "delegated_e2e_test",
    srcs = ["delegated_e2e_test.py", "//talOS/build:sim_check.py"],
    tags = ["e2e"],
)
""",
            "pkg/delegated_e2e_test.py": (
                'from talOS.build import sim_check\n'
                'sim_check.preserve_failure_artifacts("a", "b")\n'
            ),
        })
        self.assertEqual(len(problems), 2, problems)
        self.assertTrue(all("sim_check.py" in p for p in problems), problems)

    # REQ-DEV-25, REQ-RT-07
    def test_perf_tag_added_via_aug_assign_caught(self):
        problems, stats = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
LANE_TAGS = ["exclusive"]
LANE_TAGS += ["perf"]
cc_test(
    name = "aug_perf_test",
    srcs = ["bad_perf_test.cc"],
    tags = LANE_TAGS,
    deps = [],
)
""",
            "pkg/bad_perf_test.cc": "int main() { return 0; }\n",
        })
        self.assertEqual(stats["perf"], 1)
        self.assertTrue(any("host_qualification" in p for p in problems), problems)
        self.assertTrue(any("StallMeter" in p for p in problems), problems)
        self.assertTrue(any("[HOST_UNQUALIFIED]" in p for p in problems), problems)

    # REQ-DEV-25, REQ-RT-07
    def test_perf_tag_cleared_after_call_still_checked_in_statement_order(self):
        problems, stats = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
LANE_TAGS = ["perf"]
cc_test(
    name = "rebound_after_perf_test",
    srcs = ["bad_perf_test.cc"],
    tags = LANE_TAGS,
    deps = [],
)
LANE_TAGS = []
""",
            "pkg/bad_perf_test.cc": "int main() { return 0; }\n",
        })
        self.assertEqual(stats["perf"], 1)
        self.assertTrue(any("host_qualification" in p for p in problems), problems)
        self.assertTrue(any("StallMeter" in p for p in problems), problems)
        self.assertTrue(any("[HOST_UNQUALIFIED]" in p for p in problems), problems)

    # REQ-DEV-25, REQ-RT-07
    def test_aug_assign_with_non_literal_rhs_fails_closed(self):
        problems, _ = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
LANE_TAGS = ["exclusive"]
LANE_TAGS += compute_tags()
cc_test(
    name = "dyn_aug_test",
    srcs = ["a.cc"],
    tags = LANE_TAGS,
    deps = [],
)
""",
            "pkg/a.cc": "int main() { return 0; }\n",
        })
        self.assertEqual(len(problems), 1, problems)
        self.assertIn("statically resolvable", problems[0])

    # REQ-DEV-25, REQ-RT-07
    def test_cpp14_digit_separators_do_not_corrupt_strip_cpp(self):
        # Digit separator before StallMeter must not start a fake char literal.
        code_with_sep = "if (dt > 1'000) { StallMeter m; }\n"
        stripped_code = checker.strip_cpp(code_with_sep, strip_strings=True)
        self.assertIn("StallMeter", stripped_code)

        # Digit separator before // comment must still strip the comment.
        commented = 'return 1\'000; // [HOST_UNQUALIFIED] "host_unqualified"\n'
        no_comments = checker.strip_cpp(commented, strip_strings=False)
        self.assertNotIn("[HOST_UNQUALIFIED]", no_comments)
        self.assertNotIn('"host_unqualified"', no_comments)

        # End-to-end: valid perf test with 1'000 before StallMeter passes, while
        # a test hiding [HOST_UNQUALIFIED] in a // comment after 1'000 fails.
        ok_problems, _ = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
cc_test(
    name = "digit_sep_ok_perf_test",
    srcs = ["ok.cc"],
    tags = ["perf"],
    deps = ["//talOS/process:host_qualification"],
)
""",
            "pkg/ok.cc": (
                "void f(int dt) {\n"
                "  if (dt > 1'000) { StallMeter m; }\n"
                '  RecordProperty("host_unqualified", "true");\n'
                '  FAIL() << "[HOST_UNQUALIFIED]";\n'
                "}\n"
            ),
        })
        self.assertEqual(ok_problems, [])

        bad_problems, _ = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
cc_test(
    name = "digit_sep_spoof_perf_test",
    srcs = ["spoof.cc"],
    tags = ["perf"],
    deps = ["//talOS/process:host_qualification"],
)
""",
            "pkg/spoof.cc": (
                "int f() {\n"
                "  StallMeter m;\n"
                '  return 1\'000; // [HOST_UNQUALIFIED] "host_unqualified"\n'
                "}\n"
            ),
        })
        self.assertEqual(len(bad_problems), 1, bad_problems)
        self.assertIn("[HOST_UNQUALIFIED]", bad_problems[0])
        self.assertIn('"host_unqualified"', bad_problems[0])

    # REQ-DEV-25
    def test_fail_closed_on_build_syntax_error_and_dynamic_tags(self):
        problems, _ = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "bad/BUILD": "cc_test(name = 'x',\n",
            "dyn/BUILD": """
py_test(
    name = "opaque_tags_test",
    srcs = ["opaque.py"],
    tags = compute_tags(),
)
""",
            "dyn/opaque.py": "pass\n",
        })
        self.assertEqual(len(problems), 2, problems)
        self.assertTrue(any("cannot parse BUILD file" in p for p in problems))
        self.assertTrue(any("statically resolvable" in p for p in problems))

    # REQ-DEV-25, REQ-RT-07, REQ-DEV-30
    def test_aliased_test_rules_and_sim_test_tags_fail_closed(self):
        problems, stats = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
load("@rules_python//python:defs.bzl", py_suite = "py_test")
load("//custom:defs.bzl", custom_runner = "my_custom_test")
load("//talOS/build:contract.bzl", "talos_sim_test")

_cc = cc_test

py_suite(
    name = "aliased_py_dyn",
    srcs = ["a.py"],
    tags = compute_tags(),
)

custom_runner(
    name = "aliased_custom_dyn",
    srcs = ["a.py"],
    tags = compute_tags(),
)

_cc(
    name = "var_alias_cc_dyn",
    srcs = ["a.cc"],
    tags = compute_tags(),
)

talos_sim_test(
    name = "sim_with_perf",
    sim = ":sim",
    tags = ["perf"],
)

talos_sim_test(
    name = "sim_with_dyn_tags",
    sim = ":sim",
    tags = compute_tags(),
)

talos_sim_test(
    name = "sim_with_explicit_e2e",
    sim = ":sim",
    tags = ["e2e"],
)
""",
            "pkg/a.py": "pass\n",
            "pkg/a.cc": "int main() { return 0; }\n",
        })
        self.assertEqual(stats["perf"], 1)
        self.assertEqual(stats["e2e"], 3)
        self.assertEqual(len(problems), 5, problems)
        self.assertTrue(
            any("//pkg:aliased_py_dyn" in p and "statically resolvable" in p
                for p in problems),
            problems,
        )
        self.assertTrue(
            any("//pkg:aliased_custom_dyn" in p and "statically resolvable" in p
                for p in problems),
            problems,
        )
        self.assertTrue(
            any("//pkg:var_alias_cc_dyn" in p and "statically resolvable" in p
                for p in problems),
            problems,
        )
        self.assertTrue(
            any("//pkg:sim_with_perf" in p and "[HOST_UNQUALIFIED]" in p
                for p in problems),
            problems,
        )
        self.assertTrue(
            any("//pkg:sim_with_dyn_tags" in p and "statically resolvable" in p
                for p in problems),
            problems,
        )

    # REQ-DEV-25
    def test_non_expr_mutating_calls_and_aliased_containers_fail_closed(self):
        # `_ = TAGS.append("perf")` mutates TAGS inside an Assign statement.
        problems, stats = run_check({
            "talOS/build/sim_check.py": VALID_SIM_CHECK,
            "pkg/BUILD": """
TAGS = []
_ = TAGS.append("perf")
cc_test(
    name = "assign_append_perf_test",
    srcs = ["bad.cc"],
    tags = TAGS,
    deps = [],
)
""",
            "pkg/bad.cc": "int main() { return 0; }\n",
        })
        self.assertEqual(stats["perf"], 1)
        self.assertTrue(any("host_qualification" in p for p in problems), problems)
        self.assertTrue(any("StallMeter" in p for p in problems), problems)
        self.assertTrue(any("[HOST_UNQUALIFIED]" in p for p in problems), problems)

        # Mutating an alias (via method call, subscript assign, or `+=`) or
        # mutating the original while reading through an alias must invalidate
        # the aliased binding and fail closed.
        alias_cases = (
            (
                "alias_append_e2e",
                """
TAGS = []
A = TAGS
A.append("e2e")
py_test(
    name = "alias_append_e2e_test",
    srcs = ["bad.py"],
    tags = TAGS,
)
""",
                "pkg/bad.py",
                "pass\n",
            ),
            (
                "alias_subscript_perf",
                """
TAGS = ["fast"]
A = TAGS
A[0] = "perf"
cc_test(
    name = "alias_sub_perf_test",
    srcs = ["bad.cc"],
    tags = TAGS,
    deps = [],
)
""",
                "pkg/bad.cc",
                "int main() { return 0; }\n",
            ),
            (
                "original_append_read_via_alias",
                """
TAGS = []
A = TAGS
TAGS.append("perf")
cc_test(
    name = "orig_append_alias_read_test",
    srcs = ["bad.cc"],
    tags = A,
    deps = [],
)
""",
                "pkg/bad.cc",
                "int main() { return 0; }\n",
            ),
            (
                "alias_aug_assign_perf",
                """
TAGS = ["fast"]
A = TAGS
A += ["perf"]
cc_test(
    name = "alias_aug_perf_test",
    srcs = ["bad.cc"],
    tags = TAGS,
    deps = [],
)
""",
                "pkg/bad.cc",
                "int main() { return 0; }\n",
            ),
            (
                "contained_item_append_perf",
                """
TAGS = ["fast"]
BOX = [TAGS]
ITEM = BOX[0]
ITEM.append("perf")
cc_test(
    name = "contained_item_append_perf_test",
    srcs = ["bad.cc"],
    tags = TAGS,
    deps = [],
)
""",
                "pkg/bad.cc",
                "int main() { return 0; }\n",
            ),
            (
                "binop_container_subscript_append_perf",
                """
TAGS = ["fast"]
BOX = [TAGS] + []
BOX[0].append("perf")
cc_test(
    name = "binop_container_perf_test",
    srcs = ["bad.cc"],
    tags = TAGS,
    deps = [],
)
""",
                "pkg/bad.cc",
                "int main() { return 0; }\n",
            ),
            (
                "post_insert_dict_subscript_append_perf",
                """
TAGS = ["fast"]
BOX = {}
BOX["k"] = TAGS
BOX["k"].append("perf")
cc_test(
    name = "post_insert_dict_perf_test",
    srcs = ["bad.cc"],
    tags = TAGS,
    deps = [],
)
""",
                "pkg/bad.cc",
                "int main() { return 0; }\n",
            ),
            (
                "post_insert_list_append_then_mutate_perf",
                """
TAGS = ["fast"]
BOX = []
BOX.append(TAGS)
BOX[0].append("perf")
cc_test(
    name = "post_insert_list_perf_test",
    srcs = ["bad.cc"],
    tags = TAGS,
    deps = [],
)
""",
                "pkg/bad.cc",
                "int main() { return 0; }\n",
            ),
            (
                "ifexp_bare_name_alias_append_perf",
                """
TAGS = ["fast"]
ALIAS = TAGS if True else ["fast"]
ALIAS.append("perf")
cc_test(
    name = "ifexp_alias_perf_test",
    srcs = ["bad.cc"],
    tags = TAGS,
    deps = [],
)
""",
                "pkg/bad.cc",
                "int main() { return 0; }\n",
            ),
            (
                "listcomp_and_dict_call_append_perf",
                """
TAGS = ["fast"]
BOX = dict(k = TAGS)
BOX["k"].append("perf")
cc_test(
    name = "dict_call_perf_test",
    srcs = ["bad.cc"],
    tags = TAGS,
    deps = [],
)
""",
                "pkg/bad.cc",
                "int main() { return 0; }\n",
            ),
        )
        for case, build, src_path, src_text in alias_cases:
            with self.subTest(case=case):
                problems, _ = run_check({
                    "talOS/build/sim_check.py": VALID_SIM_CHECK,
                    "pkg/BUILD": build,
                    src_path: src_text,
                })
                self.assertGreaterEqual(len(problems), 1, problems)
                self.assertTrue(
                    any("statically resolvable" in p for p in problems),
                    problems,
                )


if __name__ == "__main__":
    unittest.main()
