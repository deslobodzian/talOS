"""Unit tests for tools/trace.py on an in-memory fixture tree."""

import datetime
import io
import os
import tempfile
import unittest
from unittest import mock

from tools import trace

TODAY = datetime.date(2026, 11, 11)

HEADER = ("| ID | Requirement | P | Source | Observable | Cheapest tier |\n"
          "|---|---|---|---|---|---|\n")


def row(req_id, priority, source="ARCH", tier="component"):
    return f"| {req_id} | Does a thing. | {priority} | {source} | x | {tier} |\n"


def catalogue(*rows, scope=("`talOS/node`",)):
    entries = "".join(f"| {entry} | REQ-SAFE-01 |\n" for entry in scope)
    return ("# Requirements\n\n## Catalogue\n\n### SAFE\n\n" + HEADER +
            "".join(rows) + "\n## Package scope\n\n| Package | Serves |\n"
            "|---|---|\n" + entries)


def tree(**overrides):
    """A clean tree: every requirement that needs a test has one or cites a
    verification, every package is in scope."""
    files = {
        "docs/REQUIREMENTS.md": catalogue(
            row("REQ-SAFE-01", "P0"),
            row("REQ-SAFE-02", "P1", source='BR "Link timing"'),
            row("REQ-CONTRACT-01", "P2", tier="contract (gap: TODO ARCH-2)"),
            row("REQ-RT-01", "P0", source="BENCH.md loop stall check"),
            row("REQ-OPS-01", "P1")),
        "TODO.md": "## Open\n\n- [ ] **ARCH-2 — Layers are unchecked.**\n",
        "docs/BENCH.md": "# Bench\n## Loop stall check\n",
        "docs/BRINGUP.md": "# Bring-up\n## Link timing\n",
        "tools/trace_waivers.txt": "",
        "talOS/node/BUILD": (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = ["node_test.cc"],\n)\n'
        ),
        "talOS/node/node_test.cc": "// REQ-SAFE-01\nTEST(Node, Neutral) {}\n",
    }
    files.update(overrides)
    return {path: text for path, text in files.items() if text is not None}


def check(files):
    out = io.StringIO()
    failures = trace.check(sorted(files), lambda p: files.get(p, ""), TODAY,
                           out)
    return failures, out.getvalue()


class TraceTest(unittest.TestCase):

    def test_clean_tree_passes(self):
        failures, out = check(tree())
        self.assertEqual(failures, 0, out)

    # The fixture's P0 requirement stands for any: a safety requirement
    # losing its only test is what the check exists to catch.
    def test_untested_p0_requirement_fails_naming_it(self):
        failures, out = check(tree(**{
            "talOS/node/node_test.cc": "TEST(Node, Neutral) {}\n"}))
        self.assertEqual(failures, 1, out)
        self.assertIn("REQ-SAFE-01", out)

    def test_closed_todo_item_no_longer_verifies(self):
        failures, out = check(tree(**{"TODO.md": "## Open\n"}))
        self.assertEqual(failures, 1, out)
        self.assertIn("REQ-CONTRACT-01", out)

    def test_unknown_tag_fails_naming_file_and_line(self):
        failures, out = check(tree(**{
            "talOS/node/other_test.py":
                "import x\n\n    # REQ-SAFE-01, REQ-SAFE-99: why\n"}))
        self.assertEqual(failures, 1, out)
        self.assertIn("talOS/node/other_test.py:3:", out)
        self.assertIn("REQ-SAFE-99", out)

    def test_a_waiver_holds_a_gap_and_fails_once_the_gap_closes(self):
        untested = {"talOS/node/node_test.cc": "TEST(Node, Neutral) {}\n"}
        waiver = "REQ-SAFE-01 2026-11-01 ARCH-2 tag it\n"
        for name, files, expected in (
                ("waived gap", {**untested,
                                "tools/trace_waivers.txt": waiver}, 0),
                ("stale waiver", {"tools/trace_waivers.txt": waiver}, 1),
                ("closed TODO item", {**untested, "TODO.md": "",
                                      "tools/trace_waivers.txt": waiver}, 2),
                ("future date", {**untested, "tools/trace_waivers.txt":
                                 "REQ-SAFE-01 2026-12-01 ARCH-2\n"}, 1)):
            with self.subTest(name):
                failures, out = check(tree(**files))
                self.assertEqual(failures, expected, out)
                if expected:
                    self.assertIn("tools/trace_waivers.txt:1:", out)

    def test_quarantine_needs_a_current_dated_todo_item_and_hides_its_tags(self):
        build = ('cc_test(\n    name = "node_test",\n'
                 '    srcs = ["node_test.cc"],\n'
                 '    tags = ["manual", "quarantine"],\n)\n')
        todo = ("- [ ] **ARCH-2 — Layers are unchecked.**\n"
                "- [ ] **FLAKE-1 — `//talOS/node:node_test` fails.** "
                "Run 2026-11-10; fix or delete by {}.\n")
        for name, expiry, expected, needle in (
                ("in date", "2026-11-20", 1, "REQ-SAFE-01"),
                ("expired", "2026-11-10", 2, "expired"),
                ("too far out", "2026-12-01", 2, "more than 14 days"),
                ("no TODO item", None, 2, "without an open, dated")):
            with self.subTest(name):
                failures, out = check(tree(**{
                    "talOS/node/BUILD": build,
                    "TODO.md": todo.format(expiry) if expiry else
                               todo.splitlines()[0]}))
                self.assertEqual(failures, expected, out)
                self.assertIn(needle, out)
                # The quarantined test's tag does not count.
                self.assertIn("REQ-SAFE-01 (P0, SAFE) has no tagged test", out)

    def test_an_escaped_pipe_stays_inside_its_cell(self):
        # REQ-OPS-37's form: Markdown's `\|` is a pipe inside a cell, so the
        # priority after it is still the third cell, here an untested P0.
        escaped = ("| REQ-OPS-02 | A field named `alert_<error\\|warning>` "
                   "alerts. | P0 | ARCH | x | unit |\n")
        files = tree(**{"docs/REQUIREMENTS.md": catalogue(
            row("REQ-SAFE-01", "P0"), escaped)})
        failures, out = check(files)
        self.assertEqual(failures, 1, out)
        self.assertIn("REQ-OPS-02 (P0, OPS) has no tagged test", out)
        report = io.StringIO()
        trace.report(sorted(files), lambda p: files.get(p, ""), TODAY, report)
        self.assertIn("REQ-OPS-02 P0 [unit]", report.getvalue())

    # REQ-DEV-30: a robot repository's tests tag talOS's rows, which its own
    # catalogue does not hold, and its check passes on exactly its own rows.
    def test_a_framework_catalogue_validates_tags_and_requires_no_tests(self):
        framework = "talos/docs/REQUIREMENTS.md"
        rows = (row("REQ-RT-06", "P0"), row("REQ-SAFE-09", "P0"))
        talos = catalogue(*rows, scope=("`talOS/nowhere`",))
        tagged = "// REQ-SAFE-01\n// REQ-RT-06\nTEST(Node, Neutral) {}\n"
        for name, files, expected, needles in (
                # The untested P0 there, REQ-SAFE-09, is no gap here, and
                # talOS's scope entry naming no directory here is not
                # checked.
                ("framework tag", {"talOS/node/node_test.cc": tagged}, 0, ()),
                ("duplicate ID", {framework: catalogue(
                    *rows, row("REQ-SAFE-01", "P1"))},
                 1, ("REQ-SAFE-01", "docs/REQUIREMENTS.md:", framework)),
                ("tag in neither", {"talOS/node/other_test.cc":
                                    "// REQ-RT-99\n"},
                 1, ("other_test.cc:1:", "REQ-RT-99",
                     f"docs/REQUIREMENTS.md or {framework}")),
                # No rows of its own: no scope table, only tags checked.
                ("no local catalogue", {"docs/REQUIREMENTS.md": None,
                                        "talOS/node/node_test.cc": tagged},
                 1, ("REQ-SAFE-01", f"not in {framework}"))):
            with self.subTest(name):
                files = tree(**{framework: talos, **files})
                out = io.StringIO()
                failures = trace.check(sorted(files),
                                       lambda p, f=files: f.get(p, ""), TODAY,
                                       out, frameworks=[framework])
                self.assertEqual(failures, expected, out.getvalue())
                for needle in needles:
                    self.assertIn(needle, out.getvalue())

    # REQ-DEV-30: the repository new_robot writes passes `bazel test //...`
    # before `git init`; its trace then reads every file but Bazel's output
    # links and the talOS checkout .bazelignore keeps out.
    def test_outside_git_every_file_but_output_links_and_ignored_is_read(self):
        with tempfile.TemporaryDirectory() as root:
            for path in ("gate/node_test.py", "bazel-out/x_test.py",
                         "talos/tools/trace_test.py", ".git/x_test.py"):
                os.makedirs(os.path.dirname(os.path.join(root, path)),
                            exist_ok=True)
                open(os.path.join(root, path), "w").close()
            with open(os.path.join(root, ".bazelignore"), "w") as f:
                f.write("# a submodule of talOS\ntalos\n")
            cwd = os.getcwd()
            os.chdir(root)
            try:
                with mock.patch.dict(os.environ, {"GIT_DIR": "/nonexistent"}):
                    files = trace.tracked_files()
            finally:
                os.chdir(cwd)
        self.assertEqual(files, [".bazelignore", "gate/node_test.py"])

    def test_package_missing_from_scope_table_fails_naming_it(self):
        failures, out = check(tree(**{
            "talOS/vision/BUILD": "",
            "talOS/vision/testdata/BUILD": ""}))
        self.assertEqual(failures, 1, out)
        self.assertIn("package talOS/vision is missing", out)

    def test_cheapest_tier_is_enforced_against_bazel_lanes(self):
        # Clean tree reports cheapest tier enforced (no stale LANE-1 note).
        failures, out = check(tree())
        self.assertEqual(failures, 0, out)
        self.assertIn("cheapest tier enforced", out)
        self.assertNotIn("LANE-1", out)

        # A unit requirement tagged only in an integration test fails, naming
        # the requirement ID, required cheapest tier, and found lane.
        integ_build = (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = ["node_test.cc"],\n'
            '    tags = ["integration"],\n)\n'
        )
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0", tier="unit")),
            "talOS/node/BUILD": integ_build,
        }))
        self.assertEqual(failures, 1, out)
        self.assertIn("REQ-SAFE-01", out)
        self.assertIn("`unit`", out)
        self.assertIn("integration", out)

        # A heavy test satisfies component/contract, but NOT unit.
        heavy_build = (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = ["node_test.cc"],\n'
            '    tags = ["heavy"],\n)\n'
        )
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0", tier="component")),
            "talOS/node/BUILD": heavy_build,
        }))
        self.assertEqual(failures, 0, out)
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0", tier="unit")),
            "talOS/node/BUILD": heavy_build,
        }))
        self.assertEqual(failures, 1, out)
        self.assertIn("heavy", out)

        # Higher-tier composition tags remain allowed when a test at or below
        # the cheapest tier is also present.
        both_build = (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = ["node_test.cc"],\n)\n'
            'cc_test(\n    name = "e2e_test",\n'
            '    srcs = ["e2e_test.cc"],\n'
            '    tags = ["e2e"],\n)\n'
        )
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0", tier="unit")),
            "talOS/node/BUILD": both_build,
            "talOS/node/e2e_test.cc": "// REQ-SAFE-01\nTEST(Node, E2e) {}\n",
        }))
        self.assertEqual(failures, 0, out)

        # An integration requirement is satisfied by either fast or integration,
        # but not e2e or manual/soak.
        for lane_tag, expected in (
                (None, 0),
                ("integration", 0),
                ("e2e", 1),
                ("manual", 1),
                ("perf", 1),
        ):
            with self.subTest(lane_tag=lane_tag):
                tags_line = (f'    tags = ["{lane_tag}"],\n'
                             if lane_tag else "")
                build = (
                    'cc_test(\n    name = "node_test",\n'
                    '    srcs = ["node_test.cc"],\n' + tags_line + ')\n'
                )
                failures, out = check(tree(**{
                    "docs/REQUIREMENTS.md": catalogue(
                        row("REQ-SAFE-01", "P0", tier="integration")),
                    "talOS/node/BUILD": build,
                }))
                self.assertEqual(failures, expected, out)

    def test_unwired_test_source_fails_closed_for_tier(self):
        # A test file not wired into any BUILD test rule has unknown lane and
        # cannot satisfy a runnable cheapest tier.
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0", tier="unit")),
            "talOS/node/BUILD": 'cc_test(\n    name = "other",\n)\n',
        }))
        self.assertEqual(failures, 1, out)
        self.assertIn("REQ-SAFE-01", out)
        self.assertIn("unknown", out)

    def test_gradlerio_studio_vitest_and_bench_tiers(self):
        # GradleRIO phoenixTest and Studio Vitest test files count as fast
        # (unit/component), and bench-only tiers skip Bazel tier enforcement.
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0", tier="unit"),
                row("REQ-SAFE-02", "P0", tier="unit"),
                row("REQ-SAFE-03", "P0",
                    tier="bench (RoboRIO build and console)"),
                scope=("`talOS/node`",)),
            "talOS/node/BUILD": (
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = ["e2e"],\n)\n'
            ),
            "talOS/node/node_test.cc": "// REQ-SAFE-03\nTEST(Bench, X) {}\n",
            "controller_processor/rio/src/test/cpp/Foo_test.cpp":
                "// REQ-SAFE-01\nTEST(Rio, Fast) {}\n",
            "studio/tests/view.test.tsx":
                "// REQ-SAFE-02\ntest('fast', () => {});\n",
        }))
        self.assertEqual(failures, 0, out)

    def test_tier_gap_waiver_holds_then_turns_stale(self):
        waiver = "REQ-SAFE-01 2026-11-01 ARCH-2 move to unit\n"
        integ_build = (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = ["node_test.cc"],\n'
            '    tags = ["integration"],\n)\n'
        )
        # Waived tier gap passes.
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0", tier="unit")),
            "talOS/node/BUILD": integ_build,
            "tools/trace_waivers.txt": waiver,
        }))
        self.assertEqual(failures, 0, out)
        # Once moved to fast lane, the waiver is stale and fails.
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0", tier="unit")),
            "tools/trace_waivers.txt": waiver,
        }))
        self.assertEqual(failures, 1, out)
        self.assertIn("stale waiver", out)


if __name__ == "__main__":
    unittest.main()
