"""Unit tests for tools/check_deps_listed.py on in-memory files.

REQ-DEV-34 (METHODOLOGY.md §13 rule 5): a bazel_dep
the §13 "Bazel module" table does not list, and a row naming a module
MODULE.bazel no longer has, each fail.
"""

import unittest

from tools import check_deps_listed

MODULE = """module(name = "talos")
# a comment naming bazel_dep(name = "commented") is not a dependency
bazel_dep(name = "rules_cc", version = "0.2.18")
bazel_dep(
    name = "opencv",
    version = "4.12.0",
)
bazel_dep(version = "1.0", name = "late_name")
bazel_dep(name = 'single_quoted', version = "1")
bazel_dep(
    repo_name = "a#b",  # a comment
    name = "with_hash",
)
"""
ALL = ("rules_cc", "opencv", "late_name", "single_quoted", "with_hash")


def methodology(*rows):
    table = "".join(f"| `{row}` | why |\n" for row in rows)
    return ("## 12. Before\n\n| Bazel module | For |\n|---|---|\n"
            "| `elsewhere` | not §13 |\n\n"
            "## 13. Open source first\n\n| Need | Use |\n|---|---|\n"
            "| Config | `tomlplusplus` |\n\n"
            f"| Bazel module | For |\n|---|---|\n{table}\n## 14. After\n")


class CheckDepsListedTest(unittest.TestCase):
    # REQ-DEV-34, METHODOLOGY.md §13 rule 5
    def test_table_equal_to_bazel_deps_passes(self):
        self.assertEqual(
            check_deps_listed.check(MODULE, methodology(*ALL)), [])

    # REQ-DEV-34, METHODOLOGY.md §13 rule 5
    def test_unlisted_and_stale_modules_fail(self):
        self.assertEqual(
            check_deps_listed.check(
                MODULE, methodology("rules_cc", *ALL[2:], "eigen")),
            ["MODULE.bazel: bazel_dep opencv is not listed in "
             "docs/METHODOLOGY.md §13",
             "docs/METHODOLOGY.md §13: eigen is not a bazel_dep of "
             "MODULE.bazel"])

    # REQ-DEV-34, METHODOLOGY.md §13 rule 5
    def test_starlark_bypasses_and_fail_closed_constructs_are_caught(self):
        # Nested parens, inline comments between `name =` and the string,
        # positional names, parenthesized/concatenated/triple-quoted strings
        # must all be discovered rather than slipping past the check.
        evasive_module = (
            'module(name = "talos")\n'
            'bazel_dep(\n'
            '    version = ("1.0"),\n'
            '    name = "after_parens",\n'
            ')\n'
            'bazel_dep(\n'
            '    name = # inline comment\n'
            '        "after_comment",\n'
            '    version = "1.0",\n'
            ')\n'
            'bazel_dep("positional_dep", "1.0")\n'
            'bazel_dep(name = ("paren_" + "concat"), version = "1.0")\n'
            'bazel_dep(name = """triple_quoted""", version = "1.0")\n'
        )
        expected = (
            "after_comment",
            "after_parens",
            "paren_concat",
            "positional_dep",
            "triple_quoted",
        )
        self.assertEqual(
            check_deps_listed.check(evasive_module, methodology(*expected)), [])
        problems = check_deps_listed.check(evasive_module, methodology())
        for dep in expected:
            self.assertIn(
                f"MODULE.bazel: bazel_dep {dep} is not listed in "
                "docs/METHODOLOGY.md §13",
                problems,
            )

        # include(), dynamic bazel_dep names, and syntax errors fail closed.
        for label, bad_module in (
            ("include", 'module(name = "talos")\ninclude("//deps.MODULE.bazel")\n'),
            ("dynamic_name", 'bazel_dep(name = get_dep(), version = "1.0")\n'),
            ("syntax_error", 'bazel_dep(name = "broken",\n'),
        ):
            with self.subTest(label=label):
                self.assertTrue(
                    check_deps_listed.check(bad_module, methodology()),
                    f"{label} silently passed",
                )

        # Fenced code examples inside §13 must not end or overwrite the table.
        fenced_doc = (
            "## 13. Open source first\n\n"
            "```markdown\n"
            "## 14. Fake heading inside code fence\n"
            "| Bazel module | For |\n|---|---|\n| `fake` | example |\n"
            "```\n\n"
            "| Bazel module | For |\n|---|---|\n| `rules_cc` | build |\n"
        )
        self.assertEqual(
            check_deps_listed.check(
                'bazel_dep(name = "rules_cc", version = "1.0")\n', fenced_doc),
            [],
        )


if __name__ == "__main__":
    unittest.main()
