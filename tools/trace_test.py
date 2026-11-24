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
            row("REQ-SAFE-02", "P1", tier='bringup (BR "Link timing")'),
            row("REQ-CONTRACT-01", "P2", tier="contract (gap: TODO ARCH-2)"),
            row("REQ-RT-01", "P0", tier="bench (BENCH.md loop stall check)"),
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

    def test_ast_build_parsing_fails_closed_on_evasions_and_non_literals(self):
        req = catalogue(row("REQ-SAFE-01", "P0", tier="unit"))
        cases = (
            (
                "single-quoted integration tag",
                "cc_test(\n    name = 'node_test',\n"
                "    srcs = ['node_test.cc'],\n"
                "    tags = ['integration'],\n)\n",
                ("REQ-SAFE-01", "integration"),
            ),
            (
                "variable tags expression",
                'T = ["integration"]\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = T,\n)\n',
                ("REQ-SAFE-01", "unknown"),
            ),
            (
                "select() tags expression",
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = select({"//conditions:default": ["integration"]}),\n)\n',
                ("REQ-SAFE-01", "unknown"),
            ),
            (
                "concatenated tags expression",
                'BASE = ["small"]\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = BASE + ["integration"],\n)\n',
                ("REQ-SAFE-01", "unknown"),
            ),
            (
                "kwargs splat without explicit tags",
                'OPTS = {"tags": ["integration"]}\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    **OPTS\n)\n',
                ("REQ-SAFE-01", "unknown"),
            ),
            (
                "closing bracket inside comment in tags",
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = [\n'
                '        # note: ]\n'
                '        "integration",\n'
                '    ],\n)\n',
                ("REQ-SAFE-01", "integration"),
            ),
            (
                "commented-out empty tags before real integration tags",
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    # tags = []\n'
                '    tags = ["integration"],\n)\n',
                ("REQ-SAFE-01", "integration"),
            ),
            (
                "commented-out srcs leaves source unwired",
                'cc_test(\n    name = "node_test",\n'
                '    # srcs = ["node_test.cc"],\n)\n',
                ("REQ-SAFE-01", "unknown"),
            ),
            (
                "non-literal srcs expression fails closed",
                'SRCS = ["node_test.cc"]\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = SRCS,\n)\n',
                ("REQ-SAFE-01", "unknown"),
            ),
            (
                "BUILD syntax error fails closed",
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc",\n',
                ("cannot parse BUILD file", "REQ-SAFE-01", "unknown"),
            ),
        )
        for name, build, needles in cases:
            with self.subTest(name):
                failures, out = check(tree(**{
                    "docs/REQUIREMENTS.md": req,
                    "talOS/node/BUILD": build,
                }))
                self.assertGreaterEqual(failures, 1, out)
                for needle in needles:
                    self.assertIn(needle, out)

        # Single-quoted / comment-bracketed / concatenated quarantine tags are
        # caught by AST parsing; commented-out quarantine tags are ignored.
        for name, build in (
            (
                "single-quoted quarantine",
                "cc_test(\n    name = 'node_test',\n"
                "    srcs = ['node_test.cc'],\n"
                "    tags = ['quarantine'],\n)\n",
            ),
            (
                "comment bracket in quarantine tags",
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = [\n        # ]\n        "quarantine",\n    ],\n)\n',
            ),
            (
                "concatenated quarantine tags",
                'BASE = ["manual"]\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = BASE + ["quarantine"],\n)\n',
            ),
        ):
            with self.subTest(quarantine=name):
                failures, out = check(tree(**{"talOS/node/BUILD": build}))
                self.assertEqual(failures, 2, out)
                self.assertIn("tagged quarantine without an open, dated", out)
                self.assertIn("REQ-SAFE-01 (P0, SAFE) has no tagged test", out)

        # A commented-out quarantine line does not quarantine the target.
        failures, out = check(tree(**{
            "talOS/node/BUILD": (
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    # tags = ["quarantine"],\n)\n'
            ),
        }))
        self.assertEqual(failures, 0, out)

    def test_unknown_or_misspelled_cheapest_tier_fails_closed(self):
        for name, tier, needle in (
            ("misspelled unit", "unt", "`unt`"),
            ("misspelled integration", "integraton", "`integraton`"),
            ("mixed valid and misspelled", "unit + integraton",
             "`integraton`"),
            ("non-runnable plus typo", "bench + unt", "`unt`"),
            ("empty tier cell", "", "no recognized Cheapest tier"),
            ("parenthetical only", "(see docs)",
             "no recognized Cheapest tier"),
        ):
            with self.subTest(name):
                failures, out = check(tree(**{
                    "docs/REQUIREMENTS.md": catalogue(
                        row("REQ-SAFE-01", "P0", tier=tier)),
                }))
                self.assertGreaterEqual(failures, 1, out)
                self.assertIn("REQ-SAFE-01", out)
                self.assertIn(needle, out)

        # Nested parenthetical links (e.g. `bench ([BENCH.md](BENCH.md))`) are
        # stripped cleanly and pass.
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0",
                    tier="unit + integration (phoenixTest); "
                         "bench ([BENCH.md](BENCH.md))")),
        }))
        self.assertEqual(failures, 0, out)

    def test_bringup_citation_does_not_skip_cheapest_tier_for_tagged_tests(
            self):
        # REQ-DEV-33's form: a P0 requirement citing BR "..." with runnable
        # Cheapest tier `unit` and only an `integration` tagged test MUST fail.
        integ_build = (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = ["node_test.cc"],\n'
            '    tags = ["integration"],\n)\n'
        )
        req = catalogue(
            row("REQ-SAFE-01", "P0", source='BR "Link timing"', tier="unit"))
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": req,
            "talOS/node/BUILD": integ_build,
        }))
        self.assertEqual(failures, 1, out)
        self.assertIn("REQ-SAFE-01", out)
        self.assertIn("`unit`", out)
        self.assertIn("integration", out)

        # Adding a fast-lane test satisfies it while keeping the integration
        # test.
        both_build = (
            integ_build +
            'cc_test(\n    name = "unit_test",\n'
            '    srcs = ["unit_test.cc"],\n)\n'
        )
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": req,
            "talOS/node/BUILD": both_build,
            "talOS/node/unit_test.cc": "// REQ-SAFE-01\nTEST(N, Fast) {}\n",
        }))
        self.assertEqual(failures, 0, out)

        # An open TODO.md citation still waives the tier gap while tracked,
        # and fails once the TODO item closes.
        todo_req = catalogue(
            row("REQ-SAFE-01", "P0", source="ARCH",
                tier="unit (gap: TODO ARCH-2)"))
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": todo_req,
            "talOS/node/BUILD": integ_build,
        }))
        self.assertEqual(failures, 0, out)
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": todo_req,
            "talOS/node/BUILD": integ_build,
            "TODO.md": "## Open\n",
        }))
        self.assertEqual(failures, 1, out)
        self.assertIn("REQ-SAFE-01", out)

    def test_leading_colon_src_resolves_for_lanes_and_quarantine(self):
        # Package-local `:node_test.cc` resolves for fast-lane tier checks.
        colon_build = (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = [":node_test.cc"],\n)\n'
        )
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0", tier="unit")),
            "talOS/node/BUILD": colon_build,
        }))
        self.assertEqual(failures, 0, out)

        # Package-local `:node_test.cc` on a quarantined target still hides its
        # tags.
        q_build = (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = [":node_test.cc"],\n'
            '    tags = ["manual", "quarantine"],\n)\n'
        )
        todo = (
            "- [ ] **ARCH-2 — Layers are unchecked.**\n"
            "- [ ] **FLAKE-1 — `//talOS/node:node_test` fails.** "
            "Fix by 2026-11-20.\n"
        )
        failures, out = check(tree(**{
            "talOS/node/BUILD": q_build,
            "TODO.md": todo,
        }))
        self.assertEqual(failures, 1, out)
        self.assertIn("REQ-SAFE-01 (P0, SAFE) has no tagged test", out)

    def test_quarantined_target_with_non_literal_srcs_fails_closed(self):
        q_build = (
            'SRCS = ["node_test.cc"]\n'
            'cc_test(\n    name = "node_test",\n'
            '    srcs = SRCS,\n'
            '    tags = ["manual", "quarantine"],\n)\n'
        )
        todo = (
            "- [ ] **ARCH-2 — Layers are unchecked.**\n"
            "- [ ] **FLAKE-1 — `//talOS/node:node_test` fails.** "
            "Fix by 2026-11-20.\n"
        )
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0", tier="bench")),
            "talOS/node/BUILD": q_build,
            "TODO.md": todo,
        }))
        self.assertGreaterEqual(failures, 1, out)
        self.assertIn("non-literal", out)

    def test_quarantine_label_requires_token_boundaries_in_todo(self):
        q_build = (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = ["node_test.cc"],\n'
            '    tags = ["manual", "quarantine"],\n)\n'
        )
        # Substring matches (`node_test_extra`, `node_test2`, `node_test-slow`,
        # `foo//talOS/node:node_test`) must NOT satisfy the quarantine TODO.
        for bad_ref in (
            "`//talOS/node:node_test_extra`",
            "`//talOS/node:node_test2`",
            "`//talOS/node:node_test-slow`",
            "`foo//talOS/node:node_test`",
            "`//talOS/node:node_test.cc`",
        ):
            with self.subTest(bad_ref=bad_ref):
                todo = (
                    "- [ ] **ARCH-2 — Layers are unchecked.**\n"
                    f"- [ ] **FLAKE-1 — {bad_ref} fails.** Fix by 2026-11-20.\n"
                )
                failures, out = check(tree(**{
                    "talOS/node/BUILD": q_build,
                    "TODO.md": todo,
                }))
                self.assertEqual(failures, 2, out)
                self.assertIn("without an open, dated", out)

        # Punctuation delimiters around the exact label still match.
        for good_ref in (
            "//talOS/node:node_test.",
            "//talOS/node:node_test: fails",
            "(//talOS/node:node_test)",
        ):
            with self.subTest(good_ref=good_ref):
                todo = (
                    "- [ ] **ARCH-2 — Layers are unchecked.**\n"
                    f"- [ ] **FLAKE-1 — {good_ref} Fix by 2026-11-20.**\n"
                )
                failures, out = check(tree(**{
                    "talOS/node/BUILD": q_build,
                    "TODO.md": todo,
                }))
                self.assertEqual(failures, 1, out)
                self.assertNotIn("without an open, dated", out)

    def test_duplicate_requirement_id_row_fails_closed(self):
        # A duplicate REQ-ID in the same catalogue must fail rather than
        # silently overwriting the first row.
        dup_catalogue = catalogue(
            row("REQ-SAFE-01", "P0", tier="unit"),
            row("REQ-SAFE-01", "P2", tier="bench"),
        )
        failures, out = check(tree(**{"docs/REQUIREMENTS.md": dup_catalogue}))
        self.assertGreaterEqual(failures, 1, out)
        self.assertIn("duplicate requirement REQ-SAFE-01", out)

    def test_quarantine_variable_tags_kwargs_abs_srcs_and_bad_dates_fail_closed(
            self):
        # Quarantine tags hidden behind top-level variables, string
        # concatenation, or **kwargs splats must still be caught.
        for name, build in (
            (
                "variable_tag_list",
                'Q = ["manual", "quarantine"]\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = Q,\n)\n',
            ),
            (
                "variable_tag_element",
                'Q = "quarantine"\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = [Q],\n)\n',
            ),
            (
                "concatenated_tag_string",
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = ["quaran" + "tine"],\n)\n',
            ),
            (
                "kwargs_dict_quarantine",
                'OPTS = {"tags": ["manual", "quarantine"]}\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    **OPTS\n)\n',
            ),
        ):
            with self.subTest(case=name):
                failures, out = check(tree(**{"talOS/node/BUILD": build}))
                self.assertGreaterEqual(failures, 2, out)
                self.assertIn("tagged quarantine without an open, dated", out)
                self.assertIn("REQ-SAFE-01 (P0, SAFE) has no tagged test", out)

        # Repo-absolute `//pkg:file` in quarantined `srcs` must still exclude
        # that source's tags from requirement coverage.
        todo = (
            "- [ ] **ARCH-2 — Layers are unchecked.**\n"
            "- [ ] **FLAKE-1 — `//talOS/node:node_test` fails.** "
            "Fix by 2026-11-20.\n"
        )
        abs_src_build = (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = ["//talOS/node:node_test.cc"],\n'
            '    tags = ["manual", "quarantine"],\n)\n'
        )
        failures, out = check(tree(**{
            "talOS/node/BUILD": abs_src_build,
            "TODO.md": todo,
        }))
        self.assertEqual(failures, 1, out)
        self.assertIn("REQ-SAFE-01 (P0, SAFE) has no tagged test", out)

        # An invalid calendar date in a quarantine TODO entry must fail cleanly
        # rather than crashing with ValueError.
        bad_date_todo = (
            "- [ ] **ARCH-2 — Layers are unchecked.**\n"
            "- [ ] **FLAKE-1 — `//talOS/node:node_test` fails.** "
            "Fix by 2026-02-30.\n"
        )
        q_build = (
            'cc_test(\n    name = "node_test",\n'
            '    srcs = ["node_test.cc"],\n'
            '    tags = ["manual", "quarantine"],\n)\n'
        )
        failures, out = check(tree(**{
            "talOS/node/BUILD": q_build,
            "TODO.md": bad_date_todo,
        }))
        self.assertGreaterEqual(failures, 2, out)
        self.assertIn("invalid", out.lower())

        # A fenced code block containing `## Foo` inside Catalogue must not
        # stop parsing subsequent requirements.
        fenced_cat = (
            "# Requirements\n\n## Catalogue\n\n"
            "```markdown\n## Not a section\n```\n\n"
            + HEADER
            + row("REQ-SAFE-01", "P0")
            + row("REQ-SAFE-98", "P0")
            + "\n## Package scope\n\n| Package | Serves |\n|---|---|\n"
            "| `talOS/node` | REQ-SAFE-01 |\n"
        )
        failures, out = check(tree(**{"docs/REQUIREMENTS.md": fenced_cat}))
        self.assertEqual(failures, 1, out)
        self.assertIn("REQ-SAFE-98", out)

    # REQ-DEV-30: BR "..." and BENCH.md references in the Source/Doc column
    # (such as REQ-SAFE-44 and REQ-DEV-33) or alongside a runnable Cheapest
    # tier must never silently exempt a requirement from needing tagged tests.
    def test_doc_column_and_runnable_tier_citations_do_not_exempt_tests(self):
        cases = (
            ("doc_column_bench_with_component_tier",
             row("REQ-SAFE-44", "P0",
                 source='code Foo.cpp; BENCH.md "Loop stall check"',
                 tier="component"),
             "REQ-SAFE-44"),
            ("doc_column_bringup_with_unit_tier",
             row("REQ-DEV-33", "P0",
                 source='RULES 4; BR "Link timing"; TEST §9',
                 tier="unit"),
             "REQ-DEV-33"),
            ("doc_column_bringup_even_with_non_runnable_tier",
             row("REQ-SAFE-45", "P0",
                 source='BR "Link timing"',
                 tier="bringup"),
             "REQ-SAFE-45"),
            ("doc_column_bench_even_with_non_runnable_tier",
             row("REQ-SAFE-46", "P0",
                 source="BENCH.md loop stall check",
                 tier="bench"),
             "REQ-SAFE-46"),
            ("runnable_plus_bench_citation_in_tier",
             row("REQ-RT-24", "P0",
                 tier="unit + bench (BENCH.md loop stall check)"),
             "REQ-RT-24"),
        )
        for name, extra_row, req_id in cases:
            with self.subTest(name=name):
                failures, out = check(tree(**{
                    "docs/REQUIREMENTS.md": catalogue(
                        row("REQ-SAFE-01", "P0"), extra_row),
                }))
                self.assertEqual(failures, 1, out)
                self.assertIn(f"{req_id} (P0,", out)
                self.assertIn("has no tagged test", out)

        # A purely non-runnable Cheapest tier citing a real BRINGUP.md or
        # BENCH.md heading in the Cheapest tier column itself passes.
        failures, out = check(tree(**{
            "docs/REQUIREMENTS.md": catalogue(
                row("REQ-SAFE-01", "P0"),
                row("REQ-SAFE-50", "P0", tier='bringup (BR "Link timing")'),
                row("REQ-SAFE-51", "P0",
                    tier="bench (BENCH.md loop stall check)")),
        }))
        self.assertEqual(failures, 0, out)

    # REQ-DEV-25
    def test_quarantine_tracks_aug_assign_mutating_calls_aliases_and_order(
            self):
        # Statement-order evaluation catches quarantine tags introduced via
        # `+=`, `.append()` (both Expr and non-Expr), post-call reassignment,
        # and fails closed on aliased or subscript-mutated bindings.
        for name, build in (
            (
                "aug_assign_quarantine",
                'TAGS = []\n'
                'TAGS += ["quarantine"]\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
            (
                "append_quarantine",
                'TAGS = []\n'
                'TAGS.append("quarantine")\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
            (
                "assign_append_quarantine",
                'TAGS = []\n'
                '_ = TAGS.append("quarantine")\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
            (
                "reassigned_after_call",
                'TAGS = ["quarantine"]\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n'
                'TAGS = []\n',
            ),
            (
                "aliased_append_fails_closed",
                'TAGS = ["fast"]\n'
                'A = TAGS\n'
                'A.append("quarantine")\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
            (
                "subscript_mutation_fails_closed",
                'TAGS = ["fast"]\n'
                'TAGS[0] = "quarantine"\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
            (
                "contained_list_subscript_append_fails_closed",
                'TAGS = ["fast"]\n'
                'BOX = [TAGS]\n'
                'BOX[0].append("quarantine")\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
            (
                "binop_contained_list_item_append_fails_closed",
                'TAGS = ["fast"]\n'
                'BOX = [TAGS] + []\n'
                'ITEM = BOX[0]\n'
                'ITEM.append("quarantine")\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
            (
                "post_insert_dict_subscript_fails_closed",
                'TAGS = ["fast"]\n'
                'BOX = {}\n'
                'BOX["k"] = TAGS\n'
                'BOX["k"].append("quarantine")\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
            (
                "post_insert_list_append_fails_closed",
                'TAGS = ["fast"]\n'
                'BOX = []\n'
                'BOX.append(TAGS)\n'
                'BOX[0].append("quarantine")\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
            (
                "ifexp_bare_name_alias_fails_closed",
                'TAGS = ["fast"]\n'
                'ALIAS = TAGS if True else ["fast"]\n'
                'ALIAS.append("quarantine")\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
            (
                "dict_call_container_fails_closed",
                'TAGS = ["fast"]\n'
                'BOX = dict(k = TAGS)\n'
                'BOX["k"].append("quarantine")\n'
                'cc_test(\n    name = "node_test",\n'
                '    srcs = ["node_test.cc"],\n'
                '    tags = TAGS,\n)\n',
            ),
        ):
            with self.subTest(case=name):
                failures, out = check(tree(**{"talOS/node/BUILD": build}))
                self.assertGreaterEqual(failures, 2, out)
                self.assertIn("tagged quarantine without an open, dated", out)
                self.assertIn("REQ-SAFE-01 (P0, SAFE) has no tagged test", out)


if __name__ == "__main__":
    unittest.main()
