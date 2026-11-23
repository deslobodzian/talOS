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

import ast
import re
import sys

MODULE = "MODULE.bazel"
METHODOLOGY = "docs/METHODOLOGY.md"
SECTION = re.compile(r"^## 13\.")
TABLE_HEADER = re.compile(r"^\|\s*Bazel module\s*\|")
ROW_NAME = re.compile(r"^\|\s*`([^`]+)`\s*\|")
OPEN_FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})(.*)$")
CLOSE_FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})[ \t]*$")


def _eval_str(node, bindings, seen=frozenset()):
    """Evaluates a static Starlark string expression, or None."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.Name) and node.id in bindings and node.id not in seen:
        return _eval_str(bindings[node.id], bindings, seen | {node.id})
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = _eval_str(node.left, bindings, seen)
        right = _eval_str(node.right, bindings, seen)
        if left is not None and right is not None:
            return left + right
    return None


def _parse_module_deps(module_text):
    """Returns ({dep_name, ...}, [error_lines]) from a MODULE.bazel AST.

    Fails closed on syntax errors, `include(...)` calls (segments outside the
    root MODULE.bazel are not scanned), or `bazel_dep(...)` calls whose name
    cannot be resolved statically.
    """
    try:
        tree = ast.parse(module_text, filename=MODULE)
    except SyntaxError as exc:
        loc = f":{exc.lineno}" if exc.lineno else ""
        return set(), [f"{MODULE}{loc}: cannot parse MODULE.bazel: {exc.msg}"]

    bindings = {}
    for stmt in tree.body:
        if isinstance(stmt, ast.Assign):
            for target in stmt.targets:
                if isinstance(target, ast.Name):
                    bindings[target.id] = stmt.value

    names, errors = set(), []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)):
            continue
        lineno = getattr(node, "lineno", 1)
        if node.func.id == "include":
            errors.append(
                f"{MODULE}:{lineno}: include() is not supported; declare "
                f"every bazel_dep in the root {MODULE}"
            )
            continue
        if node.func.id != "bazel_dep":
            continue
        name_node = next(
            (kw.value for kw in node.keywords if kw.arg == "name"),
            node.args[0] if node.args else None,
        )
        dep_name = _eval_str(name_node, bindings) if name_node is not None else None
        if not dep_name:
            errors.append(
                f"{MODULE}:{lineno}: bazel_dep() name must be a static string "
                f"literal"
            )
        else:
            names.add(dep_name)
    return names, errors


def bazel_deps(module_text):
    """The `name` argument of every bazel_dep call outside a comment, in any
    argument position and either quote."""
    names, errors = _parse_module_deps(module_text)
    if errors:
        raise ValueError("; ".join(errors))
    return names


def listed(methodology_text):
    """The module names the §13 "Bazel module" table lists, or None."""
    in_section, in_table, names = False, False, None
    fence_char, fence_len = None, 0
    for line in methodology_text.splitlines():
        if fence_char is not None:
            close = CLOSE_FENCE.match(line)
            if close and close.group(1)[0] == fence_char and len(close.group(1)) >= fence_len:
                fence_char, fence_len = None, 0
            continue
        open_m = OPEN_FENCE.match(line)
        if open_m:
            marker, info = open_m.group(1), open_m.group(2)
            if not (marker[0] == "`" and "`" in info):
                fence_char, fence_len = marker[0], len(marker)
                continue
        if line.startswith("## "):
            in_section = bool(SECTION.match(line))
            in_table = False
            continue
        if not in_section:
            continue
        if TABLE_HEADER.match(line):
            in_table = True
            if names is None:
                names = set()
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
    deps, parse_errors = _parse_module_deps(module_text)
    if parse_errors:
        return parse_errors
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
