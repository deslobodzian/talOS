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


if __name__ == "__main__":
    unittest.main()
