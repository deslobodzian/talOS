"""Checks that docs/METHODOLOGY.md §13 lists exactly MODULE.bazel's bazel_deps.

    python3 tools/check_deps_listed.py   # from the workspace root

METHODOLOGY.md §13 rule 5: a dependency is added in one change, with the
reason, to that section. Its "Bazel module" table has one row per
`bazel_dep` of the root MODULE.bazel, the module name in backticks in the
first cell. This prints `MODULE.bazel: bazel_dep <name> is not listed in
docs/METHODOLOGY.md §13` for a module the table lacks and `docs/METHODOLOGY.md
§13: <name> is not a bazel_dep of MODULE.bazel` for a row naming one
MODULE.bazel no longer has, and exits 1 when there is either. CI's lint job
and tools/prepush.sh run it through tools/source_lints.sh.

It reads the root MODULE.bazel only, not a segment it `include()`s.
"""

import re
import sys

MODULE = "MODULE.bazel"
METHODOLOGY = "docs/METHODOLOGY.md"
# Starlark tokens that matter here: a string (either quote), a comment, a
# bazel_dep call's opening and any closing parenthesis.
TOKEN = re.compile(r"\"(?:[^\"\\\n]|\\.)*\"|'(?:[^'\\\n]|\\.)*'|#[^\n]*|"
                   r"\bbazel_dep\s*\(|\)")
NAME = re.compile(r"\bname\s*=\s*$")
SECTION = re.compile(r"^## 13\.")
TABLE_HEADER = re.compile(r"^\|\s*Bazel module\s*\|")
ROW_NAME = re.compile(r"^\|\s*`([^`]+)`\s*\|")


def bazel_deps(module_text):
    """The `name` argument of every bazel_dep call outside a comment, in any
    argument position and either quote."""
    names, in_call, at = set(), False, 0
    for token in TOKEN.finditer(module_text):
        text = token.group(0)
        if text.startswith("#"):
            continue
        if text.startswith("bazel_dep"):
            in_call, at = True, token.end()
        elif text == ")":
            in_call = False
        elif in_call and NAME.search(module_text, at, token.start()):
            names.add(text[1:-1])
        if in_call and not text.startswith("bazel_dep"):
            at = token.end()
    return names


def listed(methodology_text):
    """The module names the §13 "Bazel module" table lists, or None."""
    in_section, in_table, names = False, False, None
    for line in methodology_text.splitlines():
        if line.startswith("## "):
            in_section = bool(SECTION.match(line))
            in_table = False
            continue
        if not in_section:
            continue
        if TABLE_HEADER.match(line):
            in_table, names = True, set()
            continue
        if in_table:
            if not line.startswith("|"):
                in_table = False
                continue
            match = ROW_NAME.match(line)
            if match:
                names.add(match.group(1))
    return names


def check(module_text, methodology_text):
    """Each problem, as a line to print."""
    names = listed(methodology_text)
    if names is None:
        return [f"{METHODOLOGY} §13: no \"Bazel module\" table"]
    deps = bazel_deps(module_text)
    return ([f"{MODULE}: bazel_dep {d} is not listed in {METHODOLOGY} §13"
             for d in sorted(deps - names)] +
            [f"{METHODOLOGY} §13: {n} is not a bazel_dep of {MODULE}"
             for n in sorted(names - deps)])


def main():
    with open(MODULE, encoding="utf-8") as f:
        module_text = f.read()
    with open(METHODOLOGY, encoding="utf-8") as f:
        methodology_text = f.read()
    problems = check(module_text, methodology_text)
    for problem in problems:
        print(problem)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
