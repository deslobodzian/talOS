"""Ties tests to requirements: docs/TESTING.md §3, docs/DESIGN.md §6.

    python3 tools/trace.py --check    # CI lint job, from the workspace root
    python3 tools/trace.py --report   # requirement -> tagged tests

Standard library only; no Bazel. Over the tracked files:

Tags. A test source is a `*_test.{cc,cpp,py,ts,tsx}`, `test_*.py`, `test.cc`
or `*.test.{ts,tsx}` file. A tag line is a line of a test source whose text,
after an optional `//` or `#`, starts with a `REQ-<AREA>-<NN>` ID (a comment
on the line above a test, or a line of a Python docstring); every ID on that
line is a tag. An ID mentioned mid-sentence is prose, not a tag. This is the
reading the existing tags follow (`// REQ-SAFE-09`, `# REQ-A, REQ-B: why`).

`--check` fails on:

- a tag naming an ID docs/REQUIREMENTS.md's catalogue lacks (file:line),
  and every framework catalogue's too (below);
- a SAFE or CONTRACT requirement, or any P0, with no tagged test and no
  verification cited: an open TODO.md item's ID (`- [ ] **ID`) in its row,
  or (when its `Cheapest tier` names no runnable tier) a quoted BRINGUP.md
  heading after `BR` or a BENCH.md heading after `BENCH.md` in its
  `Cheapest tier` cell. Tags in the sources of a quarantined target do not
  count (TESTING.md §8.3);
- a SAFE, CONTRACT or P0 requirement whose `Cheapest tier` names a
  Bazel-runnable tier (`unit`, `component`, `contract`, `fuzz`/`property`,
  `integration`, `e2e`) and that has tagged tests (and no open TODO.md
  citation tracking the gap) but no tagged test at or below that tier; or
  whose `Cheapest tier` contains an unrecognized or missing tier token.
  Bazel test targets are parsed with `ast.parse` from `BUILD` / `BUILD.bazel`
  and mapped to lanes from their literal `tags` (`fast` untagged, `heavy`,
  `integration`, `e2e`, `perf`, or `soak` when `manual`; non-literal `tags`
  or `srcs`, unwired files, and BUILD syntax errors fail closed as `unknown`).
  GradleRIO `controller_processor/rio/src/test/cpp/*` and Studio node:test
  (`tsx --test`) `studio/tests/*.test.ts*` map to `fast`. Higher-tier composition tags remain
  allowed as additional evidence;
- a target tagged `quarantine` in a BUILD file without an open TODO.md item
  naming its label (`//pkg:name`), or whose latest date (YYYY-MM-DD) is past
  or more than 14 days out;
- a Bazel package (a directory with a BUILD file, outside `third_party` and
  `*testdata*` directories) that no entry of REQUIREMENTS.md's "Package
  scope" table names or contains, and an entry naming no tracked directory.
  An entry is a backticked path from the repository root;
  `a/{b,c}` expands.

Framework catalogues. A robot repository keeps its own rows in its
docs/REQUIREMENTS.md and tags its tests with talOS's rows too. Each
`--framework-catalogue PATH` (repeatable) names another catalogue whose IDs
are valid tags here; its rows need no tagged test here, and its scope table,
sources and waivers are not checked. An ID in two catalogues fails, naming
both (IDs are one numbering). `--talos-catalogue` names the
docs/REQUIREMENTS.md of the talOS checkout this file is in. With a framework
catalogue and no docs/REQUIREMENTS.md here, the repository has no rows and
no scope table: only tags are checked, until it adds one.

`--root-of FILE` checks the repository holding FILE, links followed: a
Bazel test's files are links into the source tree, so a robot repository's
`py_test` runs `@talos//tools:trace.py` with `--root-of
$(rootpath MODULE.bazel) --talos-catalogue` (examples/robot_repo/BUILD.bazel).

Waivers. tools/trace_waivers.txt lists known gaps, one
`<REQ-ID | package path> <YYYY-MM-DD> <TODO-ID> [note]` per line: the date the
gap was written down and the open TODO.md item that will close it. A waived
gap passes. A waiver fails when its subject is no longer a gap (the
requirement has a tagged test or a cited verification, the package is in the
table), names an unknown requirement or a closed TODO item, or is dated in
the future, so the list only shrinks.
"""

import argparse
import ast
import datetime
import os
import posixpath
import re
import subprocess
import sys

REQUIREMENTS = "docs/REQUIREMENTS.md"
TODO = "TODO.md"
BENCH = "docs/BENCH.md"
BRINGUP = "docs/BRINGUP.md"
WAIVERS = "tools/trace_waivers.txt"

REQ_ID = re.compile(r"\bREQ-[A-Z]+-\d+\b")
TAG_LINE = re.compile(r"^\s*(?:(?://+|#)\s*)?REQ-[A-Z]+-\d+\b")
TEST_SOURCE = re.compile(r"(^|/)(\w+_test\.(cc|cpp|py|ts|tsx)|test_\w*\.py|"
                         r"test\.cc|[\w.-]+\.test\.(ts|tsx))$")
TODO_ITEM = re.compile(r"^- \[ \] \*\*([A-Z][A-Z0-9]*-\d+)\b")
TODO_ID = re.compile(r"(?<![\w-])([A-Z][A-Z0-9]*-\d+)\b")
DATE = re.compile(r"\b(\d{4}-\d{2}-\d{2})\b")
QUARANTINE_DAYS = 14

# Bazel lane -> numeric rank (docs/TESTING.md §9). Lower rank is cheaper.
# `fast` covers unit, component, contract and bounded fuzz; `heavy` covers
# CPU-bound deterministic component/contract tests; `soak`/`manual` and
# `unknown` (a test source wired to no test rule) never satisfy a runnable
# cheapest tier.
LANE_RANK = {
    "fast": 0,
    "heavy": 1,
    "integration": 2,
    "e2e": 3,
    "perf": 4,
    "soak": 5,
}
UNKNOWN_LANE_RANK = 99

# Cheapest-tier token (from docs/REQUIREMENTS.md's "Cheapest tier" column)
# -> (canonical tier name, maximum allowed lane rank, max lane display name).
RUNNABLE_TIERS = {
    "unit": ("unit", 0, "fast"),
    "component": ("component", 1, "heavy"),
    "contract": ("contract", 1, "heavy"),
    "fuzz": ("fuzz/property", 1, "heavy"),
    "property": ("fuzz/property", 1, "heavy"),
    "integration": ("integration", 2, "integration"),
    "e2e": ("e2e", 3, "e2e"),
}

# Explicit non-runnable tier tokens allowed in docs/REQUIREMENTS.md's
# "Cheapest tier" column (outside parenthetical notes). Any other word fails
# closed so typos such as `unt` or `integraton` cannot silently skip tier
# enforcement.
NON_RUNNABLE_TIERS = frozenset({
    "bench",
    "bringup",
    "build",
    "hardware",
    "manual",
    "perf",
    "review",
    "robot",
    "soak",
})


class Requirement:

    def __init__(self, req_id, priority, cells):
        self.id = req_id
        self.priority = priority
        self.area = req_id.split("-")[1]
        self.tier = cells[-1]
        self.row = " | ".join(cells)

    def must_verify(self):
        return self.area in ("SAFE", "CONTRACT") or self.priority == "P0"


# A cell boundary: a pipe Markdown does not escape (`\|` is a pipe in a cell).
CELL_BOUNDARY = re.compile(r"(?<!\\)\|")


def split_row(line):
    """A table row's cells; the outer pipes are optional, as in Markdown."""
    cells = [cell.strip() for cell in CELL_BOUNDARY.split(line.strip())]
    if len(cells) > 1 and not cells[0]:
        cells = cells[1:]
    if len(cells) > 1 and not cells[-1]:
        cells = cells[:-1]
    return cells


OPEN_FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})(.*)$")
CLOSE_FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})[ \t]*$")


def parse_requirements(text):
    """({id: Requirement}, [package scope entries], [(line_no, req_id)])."""
    catalogue, scope, duplicates, section = {}, [], [], None
    fence_char, fence_len = None, 0
    for number, line in enumerate(text.splitlines(), 1):
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
            section = line[3:].strip()
            continue
        if not line.startswith("|"):
            continue
        cells = split_row(line)
        if section == "Catalogue" and len(cells) >= 3 and REQ_ID.fullmatch(cells[0]):
            req_id = cells[0]
            if req_id in catalogue:
                duplicates.append((number, req_id))
            else:
                catalogue[req_id] = Requirement(req_id, cells[2], cells)
        elif (section == "Package scope" and len(cells) >= 2 and
              cells[0] not in ("Package", "---")):
            # A parenthesis qualifies the entry before it; it names no
            # package of its own.
            package_cell = re.sub(r"\([^)]*\)", "", cells[0])
            for token in re.findall(r"`([^`]+)`", package_cell):
                scope.extend(expand_braces(token))
    return catalogue, scope, duplicates


def expand_braces(token):
    match = re.search(r"\{([^}]*)\}", token)
    if not match:
        return [token.rstrip("/")]
    head, tail = token[:match.start()], token[match.end():]
    return [path for part in match.group(1).split(",")
            for path in expand_braces(head + part.strip() + tail)]


def open_todo_items(text):
    """{ID: the item's text} for every open TODO.md item."""
    items, current = {}, None
    for line in text.splitlines():
        match = TODO_ITEM.match(line)
        if match:
            current = match.group(1)
            items[current] = line
        elif line.startswith(("- ", "#")):
            current = None
        elif current:
            items[current] += "\n" + line
    return items


def headings(text):
    return [line.lstrip("#").strip().lower() for line in text.splitlines()
            if line.startswith("#")]


def cited_verification(req, todo_items, bench_headings, bringup_headings):
    """What the row cites that verifies it (TODO item, bench/bringup section),
    or None.

    An open TODO.md item may appear anywhere in the row. A BRINGUP.md
    (`BR "..."`) or BENCH.md heading counts only when cited in `req.tier` and
    `req.tier` names no runnable tier, never from the Source/Doc column."""
    for todo_id in TODO_ID.findall(req.row):
        if todo_id in todo_items:
            return f"TODO.md {todo_id}"
    if parse_cheapest_tier(req.tier)[0] is None:
        for name in re.findall(r'\bBR "([^"]+)"', req.tier):
            if name.lower() in bringup_headings:
                return f'BRINGUP.md "{name}"'
        for after in re.findall(r"\bBENCH\.md\b([^;|]*)", req.tier):
            for heading in bench_headings:
                if heading and heading in after.lower():
                    return f'BENCH.md "{heading}"'
    return None


def find_tags(files, read):
    """[(path, line number, id)] for every tag in a test source."""
    tags = []
    for path in files:
        if not TEST_SOURCE.search(path):
            continue
        for number, line in enumerate(read(path).splitlines(), 1):
            if TAG_LINE.match(line):
                tags.extend((path, number, i) for i in REQ_ID.findall(line))
    return tags


def _parse_build_file(path, text):
    """(ast.Module | None, error_str | None) for a BUILD / BUILD.bazel file."""
    try:
        return ast.parse(text, filename=path), None
    except SyntaxError as exc:
        loc = f" (line {exc.lineno})" if exc.lineno else ""
        return None, f"{path}: cannot parse BUILD file: {exc.msg}{loc}"


def _build_calls(tree):
    """Yields every named function call in a BUILD AST in source order."""
    calls = [node for node in ast.walk(tree)
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)]
    calls.sort(key=lambda n: (getattr(n, "lineno", 0),
                              getattr(n, "col_offset", 0)))
    return calls


def _literal_string(node):
    """Returns str if `node` is a string constant, else None."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _literal_string_list(node):
    """Returns [str, ...] if `node` is a list/tuple of string constants,
    else None (non-literal expression)."""
    if not isinstance(node, (ast.List, ast.Tuple)):
        return None
    values = []
    for elt in node.elts:
        val = _literal_string(elt)
        if val is None:
            return None
        values.append(val)
    return values


class _MutatedBinding:
    """Sentinel for a BUILD binding mutated in a way that cannot be statically
    proven free of `quarantine`."""


_MUTATED = _MutatedBinding()


class _BoundExpr:
    """AST expression captured with the bindings active at definition time."""

    __slots__ = ("node", "env")

    def __init__(self, node, env):
        self.node = node
        self.env = env


class _AugBinding:
    """Augmented binding (`NAME += rhs`, `NAME |= rhs`, `NAME.append(rhs)`)."""

    __slots__ = ("left", "right", "is_add")

    def __init__(self, left, right, is_add=True):
        self.left = left
        self.right = right
        self.is_add = is_add


_MUTATING_METHODS = frozenset({
    "append",
    "extend",
    "insert",
    "pop",
    "remove",
    "clear",
    "update",
    "setdefault",
})


def _subscript_root_name(node):
    cur = node
    while isinstance(cur, ast.Subscript):
        cur = cur.value
    return cur.id if isinstance(cur, ast.Name) else None


def _is_static_value_binding(val, seen=frozenset()):
    if isinstance(val, _MutatedBinding):
        return False
    if isinstance(val, _BoundExpr):
        return _is_static_value(val.node, val.env, seen)
    if isinstance(val, _AugBinding):
        return (_is_static_value_binding(val.left, seen) and
                _is_static_value_binding(val.right, seen))
    if isinstance(val, ast.AST):
        return _is_static_value(val, {}, seen)
    return False


def _is_static_value(node, bindings, seen=frozenset()):
    """True when `node` evaluates purely from static literals/bindings."""
    if isinstance(node, ast.Constant):
        return True
    if isinstance(node, ast.Name):
        if node.id in bindings and node.id not in seen:
            return _is_static_value_binding(bindings[node.id], seen | {node.id})
        return False
    if isinstance(node, (ast.List, ast.Tuple, ast.Set)):
        return all(_is_static_value(e, bindings, seen) for e in node.elts)
    if isinstance(node, ast.Dict):
        return all(
            (k is not None and _is_static_value(k, bindings, seen) and
             _is_static_value(v, bindings, seen))
            for k, v in zip(node.keys, node.values)
        )
    if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.BitOr)):
        return (_is_static_value(node.left, bindings, seen) and
                _is_static_value(node.right, bindings, seen))
    if isinstance(node, ast.IfExp):
        return (_is_static_value(node.body, bindings, seen) and
                _is_static_value(node.orelse, bindings, seen))
    return False


def _eval_str_binding(val, seen=frozenset()):
    if isinstance(val, _MutatedBinding):
        return None
    if isinstance(val, _BoundExpr):
        return _eval_str(val.node, val.env, seen)
    if isinstance(val, _AugBinding):
        if not val.is_add:
            return None
        left = _eval_str_binding(val.left, seen)
        right = _eval_str_binding(val.right, seen)
        if left is not None and right is not None:
            return left + right
        return None
    if isinstance(val, ast.AST):
        return _eval_str(val, {}, seen)
    return None


def _eval_str(node, bindings, seen=frozenset()):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.Name) and node.id in bindings and node.id not in seen:
        return _eval_str_binding(bindings[node.id], seen | {node.id})
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = _eval_str(node.left, bindings, seen)
        right = _eval_str(node.right, bindings, seen)
        if left is not None and right is not None:
            return left + right
    return None


def _contains_string_constant_binding(val, target, seen=frozenset()):
    if isinstance(val, _MutatedBinding):
        return True
    if isinstance(val, _BoundExpr):
        return _contains_string_constant(val.node, target, val.env, seen)
    if isinstance(val, _AugBinding):
        return (_contains_string_constant_binding(val.left, target, seen) or
                _contains_string_constant_binding(val.right, target, seen))
    if isinstance(val, ast.AST):
        return _contains_string_constant(val, target, {}, seen)
    return False


def _contains_string_constant(node, target, bindings=None, seen=frozenset()):
    """Whether any static string value reachable from `node` (following
    statement-order bindings and `+` string concatenation) equals `target`,
    or fails closed when `node` references a mutated binding."""
    bindings = bindings or {}
    val = _eval_str(node, bindings, seen)
    if val is not None:
        return val == target
    if isinstance(node, ast.Name) and node.id in bindings and node.id not in seen:
        return _contains_string_constant_binding(
            bindings[node.id], target, seen | {node.id})
    return any(
        _contains_string_constant(child, target, bindings, seen)
        for child in ast.iter_child_nodes(node)
    )


def _expr_ref_names(node):
    """Variable names referenced in an expression (skipping bare Call.func
    names like `dict(...)`)."""
    if node is None:
        return set()
    skip = set()
    for n in ast.walk(node):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name):
            skip.add(id(n.func))
    return {
        n.id for n in ast.walk(node)
        if isinstance(n, ast.Name) and id(n) not in skip
    }


def _container_ref_names(node):
    """Variable names embedded inside expressions that may hold or alias
    mutable containers (literals, BinOp, IfExp, comprehensions, Call)."""
    if isinstance(node, (ast.List, ast.Tuple, ast.Set, ast.Dict,
                         ast.ListComp, ast.SetComp, ast.DictComp,
                         ast.IfExp, ast.Call)):
        return _expr_ref_names(node)
    if isinstance(node, ast.BinOp) and isinstance(
        node.op, (ast.Add, ast.BitOr, ast.Mult)
    ):
        return (
            _container_ref_names(node.left) | _container_ref_names(node.right)
            | (_expr_ref_names(node.left)
               if isinstance(node.left, (ast.Name, ast.Subscript)) else set())
            | (_expr_ref_names(node.right)
               if isinstance(node.right, (ast.Name, ast.Subscript)) else set())
        )
    return set()


class _QuarantineEvalState:
    """Statement-order binding tracker for `quarantined_targets()`."""

    def __init__(self, package, build_path):
        self.package = package
        self.build_path = build_path
        self.bindings = {}
        self.alias_group = {}
        self.contained_names = {}
        self.next_alias_id = 1
        self.targets = []

    def clone_scope(self):
        copy = _QuarantineEvalState(self.package, self.build_path)
        copy.bindings = dict(self.bindings)
        copy.alias_group = dict(self.alias_group)
        copy.contained_names = {
            k: set(v) for k, v in self.contained_names.items()
        }
        copy.next_alias_id = self.next_alias_id
        copy.targets = self.targets
        return copy

    def alloc_alias_id(self):
        gid = self.next_alias_id
        self.next_alias_id += 1
        return gid

    def alias_members(self, name):
        gid = self.alias_group.get(name)
        if gid is None:
            return {name}
        return {k for k, v in self.alias_group.items() if v == gid}

    def mark_mutated(self, name, include_self=True, include_contained=True):
        visited = set()
        queue = list(self.alias_members(name))
        if include_contained:
            for member in list(queue):
                queue.extend(self.contained_names.get(member, ()))
        while queue:
            cur = queue.pop()
            if cur in visited:
                continue
            visited.add(cur)
            for member in self.alias_members(cur):
                if member not in visited:
                    queue.append(member)
            if include_contained:
                for inner in self.contained_names.get(cur, ()):
                    if inner not in visited:
                        queue.append(inner)
        affected = set()
        for member in visited:
            if not include_self and member == name:
                continue
            self.bindings[member] = _MUTATED
            affected.add(member)
        return affected


def _iter_calls_postorder(node):
    for child in ast.iter_child_nodes(node):
        yield from _iter_calls_postorder(child)
    if isinstance(node, ast.Call):
        yield node


def _apply_quarantine_mutating_call(call, state):
    func = call.func
    if not isinstance(func, ast.Attribute):
        return set()
    method = func.attr
    if method not in _MUTATING_METHODS:
        return set()
    inserted = set()
    for arg in call.args:
        inserted |= _expr_ref_names(arg)
    for kw in call.keywords:
        inserted |= _expr_ref_names(kw.value)
    if isinstance(func.value, ast.Name):
        target = func.value.id
        if target not in state.bindings and target not in state.alias_group:
            return set()
        if (method in ("append", "extend") and len(call.args) == 1
                and not call.keywords and target in state.bindings
                and _is_static_value(call.args[0], state.bindings)):
            snap_env = dict(state.bindings)
            state.bindings[target] = _AugBinding(
                state.bindings[target], _BoundExpr(call.args[0], snap_env),
                is_add=True,
            )
            modified = {target}
            modified |= state.mark_mutated(target, include_self=False)
            if inserted:
                state.contained_names.setdefault(target, set()).update(inserted)
            return modified
        modified = state.mark_mutated(target, include_self=True)
        if inserted:
            state.contained_names.setdefault(target, set()).update(inserted)
        return modified
    if isinstance(func.value, ast.Subscript):
        root = _subscript_root_name(func.value)
        if root and (root in state.bindings or root in state.alias_group):
            modified = state.mark_mutated(root, include_self=True)
            if inserted:
                state.contained_names.setdefault(root, set()).update(inserted)
            return modified
    return set()


def _check_quarantine_call(call, state):
    if not isinstance(call.func, ast.Name):
        return
    kws = {kw.arg: kw.value for kw in call.keywords if kw.arg is not None}
    tags_node = kws.get("tags")
    is_quarantined = False
    if tags_node is not None:
        parsed_tags = _literal_string_list(tags_node)
        is_quarantined = (
            "quarantine" in parsed_tags if parsed_tags is not None
            else _contains_string_constant(tags_node, "quarantine",
                                           state.bindings))
    if not is_quarantined:
        is_quarantined = any(
            _contains_string_constant(kw.value, "quarantine", state.bindings)
            for kw in call.keywords if kw.arg is None
        )
    if not is_quarantined:
        return
    name = _literal_string(kws.get("name"))
    if name is None:
        name = f"<dynamic:line{getattr(call, 'lineno', 0)}>"
    sources = []
    non_literal_srcs = False
    if "srcs" in kws:
        parsed_srcs = _literal_string_list(kws["srcs"])
        if parsed_srcs is None:
            non_literal_srcs = True
        else:
            for src in parsed_srcs:
                norm = _normalize_pkg_src(state.package, src)
                if norm is not None:
                    sources.append(norm)
    if "main" in kws:
        main_src = _literal_string(kws["main"])
        if main_src is None:
            non_literal_srcs = True
        else:
            norm = _normalize_pkg_src(state.package, main_src)
            if norm is not None and norm not in sources:
                sources.append(norm)
    if ("srcs" not in kws and "main" not in kws and
            any(kw.arg is None for kw in call.keywords)):
        non_literal_srcs = True
    state.targets.append((f"//{state.package}:{name}", state.build_path,
                          sources, non_literal_srcs))


def _walk_quarantine_expr(expr, state):
    modified = set()
    for call in _iter_calls_postorder(expr):
        _check_quarantine_call(call, state)
        modified |= _apply_quarantine_mutating_call(call, state)
    return modified


def _walk_quarantine_stmts(stmts, state):
    modified = set()
    for stmt in stmts:
        if isinstance(stmt, ast.Expr):
            modified |= _walk_quarantine_expr(stmt.value, state)
        elif isinstance(stmt, ast.Assign):
            for target in stmt.targets:
                modified |= _walk_quarantine_expr(target, state)
            modified |= _walk_quarantine_expr(stmt.value, state)
            snap_env = dict(state.bindings)
            alias_source = None
            if isinstance(stmt.value, ast.Name):
                alias_source = stmt.value.id
            elif isinstance(stmt.value, ast.Subscript):
                alias_source = _subscript_root_name(stmt.value)
            contained = _container_ref_names(stmt.value)
            shared_gid = None
            if alias_source is not None:
                shared_gid = state.alias_group.get(alias_source)
                if shared_gid is None:
                    shared_gid = state.alloc_alias_id()
                    state.alias_group[alias_source] = shared_gid
            for target in stmt.targets:
                if isinstance(target, ast.Name):
                    tid = target.id
                    modified.add(tid)
                    state.bindings[tid] = _BoundExpr(stmt.value, snap_env)
                    state.alias_group[tid] = (
                        shared_gid if shared_gid is not None
                        else state.alloc_alias_id()
                    )
                    if contained:
                        state.contained_names[tid] = set(contained)
                    else:
                        state.contained_names.pop(tid, None)
                elif isinstance(target, ast.Subscript):
                    root = _subscript_root_name(target)
                    if root:
                        modified |= state.mark_mutated(root, include_self=True)
                        inserted = _expr_ref_names(stmt.value)
                        if inserted:
                            state.contained_names.setdefault(
                                root, set()).update(inserted)
                else:
                    for node in ast.walk(target):
                        if isinstance(node, ast.Name):
                            modified.add(node.id)
                            state.bindings[node.id] = _MUTATED
                            state.alias_group[node.id] = state.alloc_alias_id()
                            state.contained_names.pop(node.id, None)
        elif isinstance(stmt, ast.AugAssign):
            modified |= _walk_quarantine_expr(stmt.target, state)
            modified |= _walk_quarantine_expr(stmt.value, state)
            if isinstance(stmt.target, ast.Name):
                tid = stmt.target.id
                modified.add(tid)
                if (isinstance(stmt.op, (ast.Add, ast.BitOr))
                        and _is_static_value(stmt.value, state.bindings)):
                    snap_env = dict(state.bindings)
                    prev = state.bindings.get(tid, _MUTATED)
                    state.bindings[tid] = _AugBinding(
                        prev, _BoundExpr(stmt.value, snap_env),
                        is_add=isinstance(stmt.op, ast.Add),
                    )
                    modified |= state.mark_mutated(tid, include_self=False)
                    inserted = _expr_ref_names(stmt.value)
                    if inserted:
                        state.contained_names.setdefault(tid, set()).update(
                            inserted)
                else:
                    modified |= state.mark_mutated(tid, include_self=True)
            elif isinstance(stmt.target, ast.Subscript):
                root = _subscript_root_name(stmt.target)
                if root:
                    modified |= state.mark_mutated(root, include_self=True)
                    inserted = _expr_ref_names(stmt.value)
                    if inserted:
                        state.contained_names.setdefault(root, set()).update(
                            inserted)
        elif isinstance(stmt, ast.AnnAssign):
            modified |= _walk_quarantine_expr(stmt.target, state)
            if stmt.value is not None:
                modified |= _walk_quarantine_expr(stmt.value, state)
                snap_env = dict(state.bindings)
                if isinstance(stmt.target, ast.Name):
                    tid = stmt.target.id
                    modified.add(tid)
                    state.bindings[tid] = _BoundExpr(stmt.value, snap_env)
                    state.alias_group[tid] = state.alloc_alias_id()
                elif isinstance(stmt.target, ast.Subscript):
                    root = _subscript_root_name(stmt.target)
                    if root:
                        modified |= state.mark_mutated(root, include_self=True)
        elif isinstance(stmt, ast.For):
            modified |= _walk_quarantine_expr(stmt.iter, state)
            loop_state = state.clone_scope()
            for node in ast.walk(stmt.target):
                if isinstance(node, ast.Name):
                    loop_state.bindings[node.id] = _MUTATED
                    modified.add(node.id)
            inner_mod = _walk_quarantine_stmts(stmt.body, loop_state)
            inner_mod |= _walk_quarantine_stmts(stmt.orelse, loop_state)
            for name in inner_mod:
                modified |= state.mark_mutated(name, include_self=True)
        elif isinstance(stmt, ast.If):
            modified |= _walk_quarantine_expr(stmt.test, state)
            then_state = state.clone_scope()
            else_state = state.clone_scope()
            inner_mod = _walk_quarantine_stmts(stmt.body, then_state)
            inner_mod |= _walk_quarantine_stmts(stmt.orelse, else_state)
            for name in inner_mod:
                modified |= state.mark_mutated(name, include_self=True)
        elif isinstance(stmt, ast.FunctionDef):
            fn_state = state.clone_scope()
            for arg in (stmt.args.posonlyargs + stmt.args.args +
                        stmt.args.kwonlyargs):
                fn_state.bindings[arg.arg] = _MUTATED
            if stmt.args.vararg:
                fn_state.bindings[stmt.args.vararg.arg] = _MUTATED
            if stmt.args.kwarg:
                fn_state.bindings[stmt.args.kwarg.arg] = _MUTATED
            _walk_quarantine_stmts(stmt.body, fn_state)
        else:
            modified |= _walk_quarantine_expr(stmt, state)
    return modified


def _normalize_pkg_src(package, src):
    """Workspace-relative repo path for a file label in `srcs`/`main`, or None
    for an external label (`@...`) or path escaping the workspace."""
    if not src or src.startswith("@"):
        return None
    if src.startswith("//"):
        body = src[2:]
        if ":" in body:
            target_pkg, name = body.split(":", 1)
            candidate = posixpath.join(target_pkg, name) if target_pkg else name
        else:
            candidate = body
    else:
        rel = src[1:] if src.startswith(":") else src
        if not rel:
            return None
        candidate = posixpath.join(package, rel) if package else rel
    norm = posixpath.normpath(candidate)
    if norm in ("", ".") or norm == ".." or norm.startswith("../"):
        return None
    return norm


def _todo_mentions_label(text, label):
    """True when `text` names `label` as a whole Bazel target label token."""
    pattern = re.compile(
        r"(?<![\w./:@+-])" + re.escape(label) +
        r"(?![\w/@+-]|[.:](?=[\w/@+-]))"
    )
    return pattern.search(text) is not None


def quarantined_targets(files, read):
    """[(label, BUILD path, [source paths], has_non_literal_srcs)] of targets
    tagged quarantine."""
    targets = []
    for path in files:
        if os.path.basename(path) not in ("BUILD", "BUILD.bazel"):
            continue
        tree, _ = _parse_build_file(path, read(path))
        if tree is None:
            continue
        state = _QuarantineEvalState(os.path.dirname(path), path)
        _walk_quarantine_stmts(tree.body, state)
        targets.extend(state.targets)
    return targets


def rule_lane(rule_tags):
    """The Bazel lane of a test rule from its tags (docs/TESTING.md §9)."""
    if "manual" in rule_tags:
        return "soak"
    # Fail closed: if multiple lane tags appear on one rule, take the
    # costliest (highest rank) lane.
    for lane in ("perf", "e2e", "integration", "heavy"):
        if lane in rule_tags:
            return lane
    return "fast"


def test_source_lanes(files, read):
    """{source_path: {lane, ...}} for every non-quarantined test source."""
    lanes = {}
    for path in files:
        if os.path.basename(path) in ("BUILD", "BUILD.bazel"):
            tree, _ = _parse_build_file(path, read(path))
            if tree is None:
                continue
            package = os.path.dirname(path)
            for call in _build_calls(tree):
                if not call.func.id.endswith("_test"):
                    continue
                kws = {kw.arg: kw.value for kw in call.keywords
                       if kw.arg is not None}
                has_splat = any(kw.arg is None for kw in call.keywords)
                if "tags" in kws:
                    parsed_tags = _literal_string_list(kws["tags"])
                    if parsed_tags is None:
                        # Fail closed on non-literal tags (variable, select(),
                        # list concatenation, etc.).
                        lane = "unknown"
                    else:
                        rule_tags = set(parsed_tags)
                        if "quarantine" in rule_tags:
                            continue
                        lane = rule_lane(rule_tags)
                elif has_splat:
                    # `**kwargs` may pass lane/quarantine tags dynamically.
                    lane = "unknown"
                else:
                    lane = "fast"

                rel_srcs = []
                if "srcs" in kws:
                    parsed_srcs = _literal_string_list(kws["srcs"])
                    if parsed_srcs is not None:
                        rel_srcs.extend(parsed_srcs)
                if "main" in kws:
                    main_src = _literal_string(kws["main"])
                    if main_src is not None:
                        rel_srcs.append(main_src)
                for src in rel_srcs:
                    joined = _normalize_pkg_src(package, src)
                    if joined is not None:
                        lanes.setdefault(joined, set()).add(lane)
        elif path.startswith("controller_processor/rio/src/test/cpp/"):
            lanes.setdefault(path, set()).add("fast")
        elif (path.startswith("studio/tests/") and
              re.search(r"\.test\.tsx?$", path)):
            lanes.setdefault(path, set()).add("fast")
    return lanes


def parse_cheapest_tier(tier_text):
    """(cheapest_runnable, unknown_tokens, has_known_tier) for a
    docs/REQUIREMENTS.md "Cheapest tier" cell.

    Parenthetical qualifiers (including nested parentheses) are stripped; every
    remaining word must appear in RUNNABLE_TIERS or NON_RUNNABLE_TIERS.
    """
    stripped = tier_text
    while True:
        reduced = re.sub(r"\([^()]*\)", "", stripped)
        if reduced == stripped:
            break
        stripped = reduced
    tokens = re.findall(r"[a-z0-9]+", stripped.lower())
    runnable = [RUNNABLE_TIERS[token] for token in tokens
                if token in RUNNABLE_TIERS]
    known_non_runnable = [token for token in tokens
                          if token in NON_RUNNABLE_TIERS]
    unknown = [token for token in tokens
               if token not in RUNNABLE_TIERS
               and token not in NON_RUNNABLE_TIERS]
    cheapest = min(runnable, key=lambda item: item[1]) if runnable else None
    return cheapest, unknown, bool(runnable or known_non_runnable)


def cheapest_runnable_tier(tier_text):
    """(tier_name, max_lane_rank, max_lane_name) for the cheapest
    Bazel-runnable tier named in `tier_text`, or None if it names no
    runnable tier."""
    return parse_cheapest_tier(tier_text)[0]


def bazel_packages(files):
    packages = set()
    for path in files:
        if os.path.basename(path) not in ("BUILD", "BUILD.bazel"):
            continue
        package = os.path.dirname(path)
        parts = package.split("/")
        if package and parts[0] != "third_party" and not any(
                "testdata" in part for part in parts):
            packages.add(package)
    return packages


def tracked_directories(files):
    dirs = set()
    for path in files:
        parts = path.split("/")[:-1]
        for i in range(1, len(parts) + 1):
            dirs.add("/".join(parts[:i]))
    return dirs


def in_scope(package, scope):
    return any(package == entry or package.startswith(entry + "/")
               for entry in scope)


def parse_waivers(text, today):
    """({subject: (date, todo id, line number)}, [errors])."""
    waivers, errors = {}, []
    for number, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split(None, 3)
        where = f"{WAIVERS}:{number}"
        if len(fields) < 3:
            errors.append(f"{where}: want `<REQ-ID|package> <YYYY-MM-DD> "
                          f"<TODO-ID> [note]`: {line}")
            continue
        subject, date, todo_id = fields[:3]
        try:
            when = datetime.date.fromisoformat(date)
        except ValueError:
            errors.append(f"{where}: `{date}` is not a YYYY-MM-DD date")
            continue
        if when > today:
            errors.append(f"{where}: {subject} is dated in the future "
                          f"({date})")
        if subject in waivers:
            errors.append(f"{where}: {subject} is waived twice")
        waivers[subject] = (date, todo_id, number)
    return waivers, errors


def framework_ids(catalogue, frameworks, read):
    """({id: the framework catalogue holding it}, [failures]): a framework
    catalogue with no rows, duplicate IDs, or an ID in two catalogues fail."""
    owners, failures = {}, []
    for path in frameworks:
        ids, _, duplicates = parse_requirements(read(path))
        for number, req_id in duplicates:
            failures.append(f"{path}:{number}: duplicate requirement {req_id}")
        if not ids:
            failures.append(f"{path}: framework catalogue holds no "
                            f"requirement")
        for req_id in ids:
            other = REQUIREMENTS if req_id in catalogue else owners.get(req_id)
            if other:
                failures.append(f"{other}: {req_id} is also in {path}; IDs "
                                f"are one numbering across catalogues")
            owners.setdefault(req_id, path)
    return owners, failures


def trace(files, read, today, frameworks=()):
    """(failures, {id: [(path, line)]}, {id: citation}, catalogue)."""
    failures = []
    catalogue, scope, duplicates = parse_requirements(read(REQUIREMENTS))
    for number, req_id in duplicates:
        failures.append(
            f"{REQUIREMENTS}:{number}: duplicate requirement {req_id}")
    # A repository with no catalogue of its own has no scope table either.
    check_scope = REQUIREMENTS in files or not frameworks
    framework, errors = framework_ids(catalogue, frameworks, read)
    failures.extend(errors)
    catalogues = " or ".join(([REQUIREMENTS] if check_scope else []) +
                             list(frameworks))
    todo_text = read(TODO)
    todo_items = open_todo_items(todo_text)
    bench_headings = headings(read(BENCH))
    bringup_headings = headings(read(BRINGUP))
    waivers, errors = parse_waivers(read(WAIVERS), today)
    failures.extend(errors)

    for path in files:
        if os.path.basename(path) in ("BUILD", "BUILD.bazel"):
            _, err = _parse_build_file(path, read(path))
            if err:
                failures.append(err)

    quarantined = quarantined_targets(files, read)
    quarantined_sources = {s for _, _, sources, _ in quarantined for s in sources}
    source_lanes = test_source_lanes(files, read)
    tests = {}
    for path, number, req_id in find_tags(files, read):
        if req_id not in catalogue and req_id not in framework:
            failures.append(f"{path}:{number}: tag {req_id} is not in "
                            f"{catalogues}")
        elif req_id in catalogue and path not in quarantined_sources:
            tests.setdefault(req_id, []).append((path, number))

    for label, build, _, non_literal_srcs in quarantined:
        if non_literal_srcs:
            failures.append(
                f"{build}: {label} is tagged quarantine with non-literal "
                f"srcs/main; list literal source paths so its tags can be "
                f"excluded (docs/TESTING.md §8.3)")
        entries = [text for text in todo_items.values()
                   if _todo_mentions_label(text, label)]
        raw_dates = sorted(d for text in entries for d in DATE.findall(text))
        if not raw_dates:
            failures.append(f"{build}: {label} is tagged quarantine without "
                            f"an open, dated {TODO} item naming it "
                            f"(docs/TESTING.md §8.3)")
            continue
        parsed_dates = []
        invalid_dates = []
        for raw in raw_dates:
            try:
                parsed_dates.append((datetime.date.fromisoformat(raw), raw))
            except ValueError:
                invalid_dates.append(raw)
        if invalid_dates:
            failures.append(
                f"{build}: {label}'s quarantine {TODO} item has invalid "
                f"date(s): {', '.join(invalid_dates)}")
            continue
        expiry, expiry_str = max(parsed_dates)
        if expiry < today:
            failures.append(f"{build}: {label}'s quarantine expired on "
                            f"{expiry_str}: fix or delete it "
                            f"(docs/TESTING.md §8.3)")
        elif (expiry - today).days > QUARANTINE_DAYS:
            failures.append(f"{build}: {label}'s quarantine date {expiry_str} "
                            f"is more than {QUARANTINE_DAYS} days out")

    citations = {}
    for req in catalogue.values():
        citation = cited_verification(req, todo_items, bench_headings,
                                      bringup_headings)
        if citation:
            citations[req.id] = citation
        has_todo_citation = bool(citation and citation.startswith("TODO.md "))
        missing_test = req.must_verify() and req.id not in tests and not citation
        tier_gap_msg = None
        if req.must_verify():
            required, unknown_tiers, has_known_tier = parse_cheapest_tier(
                req.tier)
            if unknown_tiers:
                unknown_text = ", ".join(f"`{t}`" for t in unknown_tiers)
                failures.append(
                    f"{REQUIREMENTS}: {req.id} ({req.priority}, {req.area}) "
                    f"has unrecognized Cheapest tier token(s): {unknown_text} "
                    f"in `{req.tier}`")
            elif not has_known_tier:
                failures.append(
                    f"{REQUIREMENTS}: {req.id} ({req.priority}, {req.area}) "
                    f"has no recognized Cheapest tier token in `{req.tier}`")
            elif (required is not None and req.id in tests and
                  not has_todo_citation):
                tier_name, max_rank, max_lane = required
                found_lanes = set()
                for path, _ in tests[req.id]:
                    found_lanes.update(source_lanes.get(path, {"unknown"}))
                min_rank = min(LANE_RANK.get(lane, UNKNOWN_LANE_RANK)
                               for lane in found_lanes)
                if min_rank > max_rank:
                    lanes_text = ", ".join(sorted(found_lanes))
                    files_text = ", ".join(sorted({p for p, _ in tests[req.id]}))
                    tier_gap_msg = (
                        f"{REQUIREMENTS}: {req.id} ({req.priority}, {req.area}) "
                        f"requires cheapest tier `{tier_name}` (lane <= `{max_lane}`), "
                        f"but only has tagged tests in: {lanes_text} ({files_text}); "
                        f"add a test at or below `{tier_name}` "
                        f"(docs/TESTING.md §2, §3)")
        gap = missing_test or (tier_gap_msg is not None)
        waiver = waivers.pop(req.id, None)
        if gap and not waiver:
            if missing_test:
                failures.append(
                    f"{REQUIREMENTS}: {req.id} ({req.priority}, {req.area}) has "
                    f"no tagged test and cites no open TODO.md item or bench "
                    f"section; tag the test that proves it (docs/TESTING.md §3)")
            else:
                failures.append(tier_gap_msg)
        elif waiver and not gap:
            why = ("needs no test by this check" if not req.must_verify() else
                   "now has a tagged test at or below its cheapest tier "
                   "or a cited verification")
            failures.append(f"{WAIVERS}:{waiver[2]}: stale waiver: {req.id} "
                            f"{why}; delete the line")

    packages = bazel_packages(files) if check_scope else set()
    for package in sorted(packages):
        waiver = waivers.pop(package, None)
        if not in_scope(package, scope) and not waiver:
            failures.append(f"{REQUIREMENTS}: package {package} is missing "
                            f"from the package scope table "
                            f"(docs/DESIGN.md §6)")
        elif waiver and in_scope(package, scope):
            failures.append(f"{WAIVERS}:{waiver[2]}: stale waiver: package "
                            f"{package} is in the scope table; delete the "
                            f"line")
    directories = tracked_directories(files)
    for entry in sorted(set(scope)):
        if entry not in directories:
            failures.append(f"{REQUIREMENTS}: package scope entry `{entry}` "
                            f"names no tracked directory")

    for subject, (_, _, number) in sorted(waivers.items()):
        failures.append(f"{WAIVERS}:{number}: {subject} is neither a "
                        f"catalogue requirement nor a Bazel package")
    for subject, (_, todo_id, number) in sorted(
            parse_waivers(read(WAIVERS), today)[0].items()):
        if todo_id not in todo_items:
            failures.append(f"{WAIVERS}:{number}: {subject}'s {todo_id} is "
                            f"not an open {TODO} item")
    return failures, tests, citations, catalogue


def check(files, read, today, out=sys.stdout, frameworks=()):
    """Prints each failure; returns how many there were."""
    failures, tests, _, catalogue = trace(files, read, today, frameworks)
    for failure in failures:
        print(failure, file=out)
    if not failures:
        waived = len(parse_waivers(read(WAIVERS), today)[0])
        print(f"trace: {len(catalogue)} requirements, {len(tests)} with "
              f"tagged tests, {waived} waived; cheapest tier enforced" +
              "".join(f"; tags of {path} accepted" for path in frameworks),
              file=out)
    return len(failures)


def report(files, read, today, out=sys.stdout, frameworks=()):
    failures, tests, citations, catalogue = trace(files, read, today,
                                                  frameworks)
    for req in catalogue.values():
        print(f"{req.id} {req.priority} [{req.tier}]", file=out)
        for path, number in tests.get(req.id, []):
            print(f"    {path}:{number}", file=out)
        if req.id in citations:
            print(f"    cites {citations[req.id]}", file=out)
        if req.id not in tests and req.id not in citations:
            print("    (none)", file=out)
    for failure in failures:
        print(f"FAIL {failure}", file=out)


def tracked_files():
    """git's tracked files; outside a git work tree (a repository new_robot
    has just written), every file but Bazel's output links, hidden
    directories and what .bazelignore names."""
    try:
        inside = subprocess.run(["git", "rev-parse", "--is-inside-work-tree"],
                                capture_output=True).returncode == 0
    except OSError:
        inside = False
    if inside:
        listing = subprocess.run(["git", "ls-files", "-z"],
                                 capture_output=True, check=True).stdout
        return [path for path in listing.decode().split("\0") if path]
    ignored = {line.strip().rstrip("/") for line in
               read_file(".bazelignore").splitlines()
               if line.strip() and not line.startswith("#")}
    files = []
    for top, dirs, names in os.walk("."):
        top = os.path.relpath(top)
        dirs[:] = sorted(
            d for d in dirs if not d.startswith((".", "bazel-")) and
            os.path.normpath(os.path.join(top, d)) not in ignored)
        files.extend(os.path.normpath(os.path.join(top, name))
                     for name in sorted(names))
    return files


def read_file(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except (FileNotFoundError, IsADirectoryError):
        return ""


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--report", action="store_true")
    parser.add_argument("--framework-catalogue", action="append", default=[],
                        metavar="PATH")
    parser.add_argument("--talos-catalogue", action="store_true")
    parser.add_argument("--root-of", metavar="FILE")
    args = parser.parse_args()
    frameworks = [os.path.abspath(p) for p in args.framework_catalogue]
    if args.talos_catalogue:
        talos = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        frameworks.append(os.path.join(talos, REQUIREMENTS))
    if args.root_of:
        os.chdir(os.path.dirname(os.path.realpath(args.root_of)))
    files, today = tracked_files(), datetime.date.today()
    if args.report:
        report(files, read_file, today, frameworks=frameworks)
        return 0
    return 1 if check(files, read_file, today, frameworks=frameworks) else 0


if __name__ == "__main__":
    sys.exit(main())
