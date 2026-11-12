"""Unit tests for tools/trace.py on an in-memory fixture tree."""

import datetime
import io
import unittest

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
        "talOS/node/BUILD": 'cc_test(\n    name = "node_test",\n)\n',
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

    def test_package_missing_from_scope_table_fails_naming_it(self):
        failures, out = check(tree(**{
            "talOS/vision/BUILD": "",
            "talOS/vision/testdata/BUILD": ""}))
        self.assertEqual(failures, 1, out)
        self.assertIn("package talOS/vision is missing", out)


if __name__ == "__main__":
    unittest.main()
