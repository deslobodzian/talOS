"""Static lint for docs/TESTING.md §6.7 (perf host qualification) and §8.2
(e2e/soak failure artifact preservation).

    python3 tools/check_testing_rules.py   # from the workspace root

Standard library only; no Bazel. Over the tracked BUILD / BUILD.bazel files
(outside third_party/):

1. Host qualification for `perf`-tagged targets (docs/TESTING.md §6.7):
   - Every `perf`-tagged C++ test target (`cc_test` or a rule whose `srcs`
     include C++ sources) must:
       (a) declare `//talOS/process:host_qualification` (or
           `:host_qualification` inside `talOS/process`) unconditionally in
           `deps` (in every `select(...)` branch when `select` is used);
       (b) reference `StallMeter` in its C++ source code (outside comments and
           string literals);
       (c) record both `"host_unqualified"` and `[HOST_UNQUALIFIED]` in its
           source (outside comments).
   - Any non-C++ `perf`-tagged target must record `[HOST_UNQUALIFIED]` in its
     source (outside comments and Python docstrings).

2. Failure artifact preservation for `e2e`- and `soak`-tagged targets and
   `talos_sim_test` rules (docs/TESTING.md §8.2):
   - `talOS/build/sim_check.py` itself must reference
     `TEST_UNDECLARED_OUTPUTS_DIR` outside comments and docstrings.
   - Every `talos_sim_test` rule satisfies §8.2 through `sim_check.py`.
   - Every other target carrying tag `"e2e"` or `"soak"` must, in its own
     test source(s) (excluding `talOS/build/sim_check.py`), either:
       (a) reference `TEST_UNDECLARED_OUTPUTS_DIR` outside comments and Python
           docstrings, or
       (b) include `talOS/build/sim_check.py` in `srcs` and call
           `sim_check.preserve_failure_artifacts(...)` or
           `sim_check.check(...)` (or `preserve_failure_artifacts(...)`
           imported from `sim_check`).

Fail-closed guarantees:
- BUILD/BUILD.bazel syntax errors, unreadable/non-UTF-8 files, unresolvable
  `tags` or `**kwargs` on test rules, and unresolvable `srcs`/`main` on
  `perf`/`e2e`/`soak` targets are reported as errors.
- `select(...)` in `tags` is unioned (so a tag in any branch triggers the
  rule); `select(...)` in `deps` is intersected (so a required dep must appear
  in every branch).

Prints each finding; exits 1 on any finding, 0 when every target complies.
"""

import ast
import os
import posixpath
import re
import subprocess
import sys

SIM_CHECK_PATH = "talOS/build/sim_check.py"
HOST_QUALIFICATION_DEP = "//talOS/process:host_qualification"
CPP_SUFFIXES = (".cc", ".cpp", ".cxx", ".mm")
SHELL_SUFFIXES = (".sh", ".bash")
CC_TEST_RULES = frozenset({"cc_test", "talos_cc_test"})
SIM_TEST_RULES = frozenset({"talos_sim_test"})
KNOWN_TEST_RULES = frozenset({
    "cc_test",
    "py_test",
    "sh_test",
    "js_test",
    "talos_cc_test",
    "talos_py_test",
    "talos_sim_test",
    "talos_graph_test",
    "manifest_sync_test",
})
STALL_METER_RE = re.compile(r"\bStallMeter\b")
HOST_UNQUALIFIED_MARKER = "[HOST_UNQUALIFIED]"
HOST_UNQUALIFIED_PROP = '"host_unqualified"'
UNDECLARED_OUTPUTS_ENV = "TEST_UNDECLARED_OUTPUTS_DIR"


class _Unresolvable:
    """Sentinel for a binding whose static value cannot be proven."""


_UNRESOLVABLE = _Unresolvable()


class _BoundExpr:
    """AST expression captured with the bindings active at definition time."""

    __slots__ = ("node", "env")

    def __init__(self, node, env):
        self.node = node
        self.env = env


class _AugBinding:
    """Augmented binding (`NAME += rhs` or `NAME |= rhs`)."""

    __slots__ = ("left", "right", "is_add")

    def __init__(self, left, right, is_add=True):
        self.left = left
        self.right = right
        self.is_add = is_add


def _is_build_file(path):
    return (not path.startswith("third_party/") and
            os.path.basename(path) in ("BUILD", "BUILD.bazel"))


def _eval_str_binding(val):
    if isinstance(val, _Unresolvable):
        return None
    if isinstance(val, _BoundExpr):
        return _eval_str(val.node, val.env)
    if isinstance(val, _AugBinding):
        if not val.is_add:
            return None
        left = _eval_str_binding(val.left)
        right = _eval_str_binding(val.right)
        if left is not None and right is not None:
            return left + right
        return None
    return None


def _eval_str(node, bindings):
    """Evaluates a static Starlark string expression, or returns None."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.Name) and node.id in bindings:
        return _eval_str_binding(bindings[node.id])
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = _eval_str(node.left, bindings)
        right = _eval_str(node.right, bindings)
        if left is not None and right is not None:
            return left + right
    return None


def _resolve_dict_binding(val):
    if isinstance(val, _BoundExpr):
        if isinstance(val.node, ast.Dict):
            return val.node, val.env
        if isinstance(val.node, ast.Name) and val.node.id in val.env:
            return _resolve_dict_binding(val.env[val.node.id])
    return None, None


def _resolve_dict(node, bindings):
    if isinstance(node, ast.Dict):
        return node, bindings
    if isinstance(node, ast.Name) and node.id in bindings:
        return _resolve_dict_binding(bindings[node.id])
    return None, None


def _call_rule_name(call):
    """Returns the rule/macro identifier for an ast.Call, or None."""
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return None


def _is_test_rule(rule_name, test_rule_names):
    if not rule_name:
        return False
    return rule_name in test_rule_names or rule_name.endswith("_test")


def _possible_strings_binding(val):
    if isinstance(val, _Unresolvable):
        return set(), False
    if isinstance(val, _BoundExpr):
        return _possible_strings(val.node, val.env)
    if isinstance(val, _AugBinding):
        l_vals, l_ok = _possible_strings_binding(val.left)
        r_vals, r_ok = _possible_strings_binding(val.right)
        return l_vals | r_vals, l_ok and r_ok
    return set(), False


def _possible_strings(node, bindings):
    """Returns (set_of_strings, resolved_bool) for all string values a
    Starlark expression can produce (for `tags`).

    Unions branches of `select({...})`, dict `.get(...)` values, list
    concatenations, and conditionals so a tag appearing in any branch triggers
    the lint. Returns `(found_so_far, False)` when an opaque expression cannot
    be statically proven free of lane tags.
    """
    s = _eval_str(node, bindings)
    if s is not None:
        return {s}, True

    if isinstance(node, ast.Name):
        if node.id in bindings:
            return _possible_strings_binding(bindings[node.id])
        return set(), False

    if isinstance(node, (ast.List, ast.Tuple, ast.Set)):
        out = set()
        ok = True
        for elt in node.elts:
            vals, elt_ok = _possible_strings(elt, bindings)
            out |= vals
            ok = ok and elt_ok
        return out, ok

    if isinstance(node, ast.Dict):
        out = set()
        ok = True
        for val in node.values:
            vals, val_ok = _possible_strings(val, bindings)
            out |= vals
            ok = ok and val_ok
        return out, ok

    if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.BitOr)):
        l_vals, l_ok = _possible_strings(node.left, bindings)
        r_vals, r_ok = _possible_strings(node.right, bindings)
        return l_vals | r_vals, l_ok and r_ok

    if isinstance(node, ast.IfExp):
        b_vals, b_ok = _possible_strings(node.body, bindings)
        e_vals, e_ok = _possible_strings(node.orelse, bindings)
        return b_vals | e_vals, b_ok and e_ok

    if isinstance(node, ast.ListComp):
        comp_bindings = dict(bindings)
        for gen in node.generators:
            for target_node in ast.walk(gen.target):
                if isinstance(target_node, ast.Name):
                    comp_bindings[target_node.id] = _UNRESOLVABLE
        return _possible_strings(node.elt, comp_bindings)

    if isinstance(node, ast.Call):
        # select({...}) -> union of all branch values.
        if (isinstance(node.func, ast.Name) and node.func.id == "select" and
                len(node.args) == 1 and not node.keywords):
            dict_node, dict_env = _resolve_dict(node.args[0], bindings)
            if dict_node is not None:
                return _possible_strings(dict_node, dict_env)
            return set(), False
        # DICT.get(key[, default]) -> union of all dict values and default.
        if (isinstance(node.func, ast.Attribute) and node.func.attr == "get"
                and not node.keywords and 1 <= len(node.args) <= 2):
            dict_node, dict_env = _resolve_dict(node.func.value, bindings)
            if dict_node is not None:
                vals, ok = _possible_strings(dict_node, dict_env)
                if len(node.args) == 2:
                    d_vals, d_ok = _possible_strings(node.args[1], bindings)
                    vals |= d_vals
                    ok = ok and d_ok
                return vals, ok
        return set(), False

    return set(), False


def _guaranteed_strings_binding(val):
    if isinstance(val, _Unresolvable):
        return set()
    if isinstance(val, _BoundExpr):
        return _guaranteed_strings(val.node, val.env)
    if isinstance(val, _AugBinding):
        if not val.is_add:
            return set()
        return (_guaranteed_strings_binding(val.left) |
                _guaranteed_strings_binding(val.right))
    return set()


def _guaranteed_strings(node, bindings):
    """Returns the set of strings guaranteed to be present in a Starlark list
    expression (for `deps`).

    Intersects branches of `select({...})` so a required dep placed in only
    one branch fails closed. Unknown/dynamic expressions yield an empty set.
    """
    if isinstance(node, (ast.List, ast.Tuple)):
        out = set()
        for elt in node.elts:
            val = _eval_str(elt, bindings)
            if val is not None:
                out.add(val)
        return out

    if isinstance(node, ast.Name):
        if node.id in bindings:
            return _guaranteed_strings_binding(bindings[node.id])
        return set()

    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        return (_guaranteed_strings(node.left, bindings) |
                _guaranteed_strings(node.right, bindings))

    if isinstance(node, ast.Call):
        if (isinstance(node.func, ast.Name) and node.func.id == "select" and
                len(node.args) == 1 and not node.keywords):
            dict_node, dict_env = _resolve_dict(node.args[0], bindings)
            if dict_node is None or not dict_node.values:
                return set()
            if any(k is None for k in dict_node.keys):
                return set()
            branch_sets = [
                _guaranteed_strings(val, dict_env)
                for val in dict_node.values
            ]
            return set.intersection(*branch_sets) if branch_sets else set()

    return set()


def _resolve_string_list_binding(val):
    if isinstance(val, _Unresolvable):
        return None
    if isinstance(val, _BoundExpr):
        return _resolve_string_list(val.node, val.env)
    if isinstance(val, _AugBinding):
        if not val.is_add:
            return None
        left = _resolve_string_list_binding(val.left)
        right = _resolve_string_list_binding(val.right)
        if left is not None and right is not None:
            return left + right
        return None
    return None


def _resolve_string_list(node, bindings):
    """Resolves a static list of strings (supporting `Name` and `+`), or
    returns None if any element or container cannot be resolved statically."""
    if isinstance(node, (ast.List, ast.Tuple)):
        out = []
        for elt in node.elts:
            val = _eval_str(elt, bindings)
            if val is None:
                return None
            out.append(val)
        return out
    if isinstance(node, ast.Name):
        if node.id in bindings:
            return _resolve_string_list_binding(bindings[node.id])
        return None
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = _resolve_string_list(node.left, bindings)
        right = _resolve_string_list(node.right, bindings)
        if left is not None and right is not None:
            return left + right
        return None
    return None


def _normalize_dep_label(package, label):
    """Normalizes a Bazel dep label to canonical `//pkg:target` form."""
    if label.startswith("@talos//"):
        label = label[len("@talos"):]
    if label.startswith("@"):
        return label
    if label.startswith("//"):
        body = label[2:]
        if ":" in body:
            pkg, target = body.split(":", 1)
            return f"//{pkg}:{target}"
        target = posixpath.basename(body)
        return f"//{body}:{target}"
    if label.startswith(":"):
        return f"//{package}:{label[1:]}"
    return f"//{package}:{label}"


def _normalize_src_path(package, src):
    """Workspace-relative path for a file label in `srcs`/`main`, or None."""
    if not src:
        return None
    if src.startswith("@talos//"):
        src = src[len("@talos"):]
    if src.startswith("@"):
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


def _is_ident_char(ch):
    return (ch == "_" or ("a" <= ch <= "z") or ("A" <= ch <= "Z") or
            ("0" <= ch <= "9"))


def strip_cpp(text, strip_strings=False):
    """Strips C/C++ comments (and optionally string/char literals) while
    preserving newlines, handling raw string literals R\"delim(...)delim\" and
    C++14 numeric digit separators (1'000)."""
    out = []
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        # Line comment //
        if ch == "/" and i + 1 < n and text[i + 1] == "/":
            i += 2
            while i < n and text[i] != "\n":
                i += 1
            out.append(" ")
            continue
        # Block comment /* ... */
        if ch == "/" and i + 1 < n and text[i + 1] == "*":
            i += 2
            while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                if text[i] == "\n":
                    out.append("\n")
                i += 1
            i = min(n, i + 2)
            out.append(" ")
            continue
        # Identifier or raw string literal [u8|u|U|L]R"delim(...)delim"
        if ch == "_" or ("a" <= ch <= "z") or ("A" <= ch <= "Z"):
            j = i + 1
            while j < n and _is_ident_char(text[j]):
                j += 1
            ident = text[i:j]
            if (ident in ("R", "u8R", "uR", "UR", "LR") and j < n and
                    text[j] == '"'):
                k = j + 1
                while k < n and text[k] not in ("(", "\\", "\n", " ", "\t"):
                    k += 1
                if k < n and text[k] == "(":
                    delim = text[j + 1:k]
                    end_seq = ")" + delim + '"'
                    end = text.find(end_seq, k + 1)
                    end_idx = n if end == -1 else end + len(end_seq)
                    chunk = text[i:end_idx]
                    if strip_strings:
                        out.append("\n" * chunk.count("\n") + " ")
                    else:
                        out.append(chunk)
                    i = end_idx
                    continue
            out.append(ident)
            i = j
            continue
        # pp-number (consumes C++14 digit separators such as 1'000'000).
        nxt = text[i + 1] if i + 1 < n else ""
        if ("0" <= ch <= "9") or (ch == "." and "0" <= nxt <= "9"):
            start = i
            if ch == ".":
                i += 1
            i += 1
            while i < n:
                c = text[i]
                c_next = text[i + 1] if i + 1 < n else ""
                if c == "'" and c_next and _is_ident_char(c_next):
                    i += 2
                    continue
                if c in ("e", "E", "p", "P") and c_next in ("+", "-"):
                    i += 2
                    continue
                if _is_ident_char(c) or c == ".":
                    i += 1
                    continue
                break
            out.append(text[start:i])
            continue
        # Normal string or char literal
        if ch in ('"', "'"):
            quote = ch
            j = i + 1
            while j < n:
                if text[j] == "\\" and j + 1 < n:
                    j += 2
                    continue
                if text[j] == quote:
                    j += 1
                    break
                if text[j] == "\n":
                    break
                j += 1
            chunk = text[i:j]
            if strip_strings:
                out.append("\n" * chunk.count("\n") + " ")
            else:
                out.append(chunk)
            i = j
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def _docstring_nodes(tree):
    """Returns the set of ast.Constant nodes that serve as Python docstrings."""
    docs = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                             ast.AsyncFunctionDef)):
            if (node.body and isinstance(node.body[0], ast.Expr) and
                    isinstance(node.body[0].value, ast.Constant) and
                    isinstance(node.body[0].value.value, str)):
                docs.add(node.body[0].value)
    return docs


def _py_non_docstring_strings(tree):
    """Yields every non-docstring string constant value in a Python AST."""
    docs = _docstring_nodes(tree)
    for node in ast.walk(tree):
        if (isinstance(node, ast.Constant) and isinstance(node.value, str) and
                node not in docs):
            yield node.value


def _py_calls_sim_check_preservation(tree):
    """True when a Python module calls `sim_check.preserve_failure_artifacts`,
    `sim_check.check`, or an imported `preserve_failure_artifacts`."""
    sim_check_aliases = {"sim_check"}
    direct_helpers = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            mod = node.module or ""
            if mod == "talOS.build" or mod.endswith(".build"):
                for alias in node.names:
                    if alias.name == "sim_check":
                        sim_check_aliases.add(alias.asname or alias.name)
            elif mod.endswith("sim_check"):
                for alias in node.names:
                    if alias.name in ("preserve_failure_artifacts", "check"):
                        direct_helpers.add(alias.asname or alias.name)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name == "sim_check" or alias.name.endswith(".sim_check"):
                    sim_check_aliases.add(alias.asname or alias.name)

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if isinstance(func, ast.Attribute):
            if func.attr == "preserve_failure_artifacts":
                if (isinstance(func.value, ast.Name) and
                        func.value.id in sim_check_aliases):
                    return True
            elif func.attr == "check":
                if (isinstance(func.value, ast.Name) and
                        func.value.id in sim_check_aliases):
                    return True
        elif isinstance(func, ast.Name) and func.id in direct_helpers:
            return True
    return False


def _sim_check_preserves_artifacts(files_set, read):
    """Verifies that talOS/build/sim_check.py itself preserves artifacts under
    TEST_UNDECLARED_OUTPUTS_DIR."""
    if SIM_CHECK_PATH not in files_set:
        return False, f"{SIM_CHECK_PATH}: missing from workspace"
    try:
        text = read(SIM_CHECK_PATH)
    except (OSError, UnicodeDecodeError) as exc:
        return False, f"{SIM_CHECK_PATH}: cannot read file: {exc}"
    try:
        tree = ast.parse(text, filename=SIM_CHECK_PATH)
    except SyntaxError as exc:
        return False, f"{SIM_CHECK_PATH}: cannot parse Python: {exc.msg}"
    has_env = any(
        UNDECLARED_OUTPUTS_ENV in s for s in _py_non_docstring_strings(tree)
    )
    fn_names = {
        node.name for node in tree.body if isinstance(node, ast.FunctionDef)
    }
    if not has_env or "preserve_failure_artifacts" not in fn_names:
        return (
            False,
            f"{SIM_CHECK_PATH}: must preserve failure artifacts under "
            f"{UNDECLARED_OUTPUTS_ENV} in preserve_failure_artifacts()",
        )
    return True, None


def _source_mentions_marker(path, text, marker, allow_sim_check=False):
    """Checks one source file for `marker` (outside comments & Python
    docstrings), and optionally for a sim_check preservation call.

    Returns (matched_bool, error_or_None).
    """
    if path.endswith(".py"):
        try:
            tree = ast.parse(text, filename=path)
        except SyntaxError as exc:
            return False, f"{path}:{exc.lineno or 1}: cannot parse Python: {exc.msg}"
        if any(marker in s for s in _py_non_docstring_strings(tree)):
            return True, None
        if allow_sim_check and _py_calls_sim_check_preservation(tree):
            return True, None
        return False, None

    if path.endswith(CPP_SUFFIXES + (".h", ".hpp")):
        no_comments = strip_cpp(text, strip_strings=False)
        return marker in no_comments, None

    if path.endswith(SHELL_SUFFIXES):
        for line in text.splitlines():
            code = line.split("#", 1)[0]
            if marker in code:
                return True, None
        return False, None

    # Generic fallback (e.g., JS/TS): strip C-style comments.
    no_comments = strip_cpp(text, strip_strings=False)
    return marker in no_comments, None


def _check_perf_target(label, build_path, lineno, is_cpp, kws, package,
                       bindings, files_set, read, default_srcs=None):
    """Checks a `perf`-tagged target against docs/TESTING.md §6.7."""
    problems = []

    if default_srcs and "srcs" not in kws and "main" not in kws:
        srcs, src_err = list(default_srcs), None
    else:
        srcs, src_err = _extract_target_sources(
            label, build_path, lineno, kws, package, bindings)
    if src_err:
        problems.append(src_err)
        return problems

    if not is_cpp:
        is_cpp = any(s.endswith(CPP_SUFFIXES) for s in srcs)

    if is_cpp:
        deps_node = kws.get("deps")
        guaranteed = (
            _guaranteed_strings(deps_node, bindings)
            if deps_node is not None else set()
        )
        normalized_deps = {
            _normalize_dep_label(package, d) for d in guaranteed
        }
        if HOST_QUALIFICATION_DEP not in normalized_deps:
            problems.append(
                f"{build_path}:{lineno}: {label} (perf) must unconditionally "
                f"depend on {HOST_QUALIFICATION_DEP} (docs/TESTING.md §6.7)"
            )

        has_stall_meter = False
        has_marker = False
        has_prop = False
        for src in srcs:
            if src not in files_set:
                problems.append(
                    f"{build_path}:{lineno}: {label} (perf) source {src} "
                    f"not found in workspace"
                )
                continue
            try:
                text = read(src)
            except (OSError, UnicodeDecodeError) as exc:
                problems.append(f"{src}: cannot read source: {exc}")
                continue
            code_only = strip_cpp(text, strip_strings=True)
            no_comments = strip_cpp(text, strip_strings=False)
            if STALL_METER_RE.search(code_only):
                has_stall_meter = True
            if HOST_UNQUALIFIED_MARKER in no_comments:
                has_marker = True
            if HOST_UNQUALIFIED_PROP in no_comments:
                has_prop = True
        if not has_stall_meter:
            problems.append(
                f"{build_path}:{lineno}: {label} (perf) must use StallMeter "
                f"in its source (docs/TESTING.md §6.7)"
            )
        if not has_marker or not has_prop:
            missing = []
            if not has_marker:
                missing.append(HOST_UNQUALIFIED_MARKER)
            if not has_prop:
                missing.append(HOST_UNQUALIFIED_PROP)
            problems.append(
                f"{build_path}:{lineno}: {label} (perf) must record "
                f"{' and '.join(missing)} on host stall (docs/TESTING.md §6.7)"
            )
        return problems

    # Non-C++ perf target: must record [HOST_UNQUALIFIED].
    has_marker = False
    for src in srcs:
        if src not in files_set:
            problems.append(
                f"{build_path}:{lineno}: {label} (perf) source {src} "
                f"not found in workspace"
            )
            continue
        try:
            text = read(src)
        except (OSError, UnicodeDecodeError) as exc:
            problems.append(f"{src}: cannot read source: {exc}")
            continue
        matched, err = _source_mentions_marker(
            src, text, HOST_UNQUALIFIED_MARKER)
        if err:
            problems.append(err)
        elif matched:
            has_marker = True
    if not has_marker:
        problems.append(
            f"{build_path}:{lineno}: {label} (perf) must record "
            f"{HOST_UNQUALIFIED_MARKER} on host stall (docs/TESTING.md §6.7)"
        )
    return problems


def _extract_target_sources(label, build_path, lineno, kws, package, bindings):
    """Returns ([workspace_relative_src, ...], error_or_None)."""
    srcs = []
    if "srcs" in kws:
        raw = _resolve_string_list(kws["srcs"], bindings)
        if raw is None:
            return [], (
                f"{build_path}:{lineno}: {label}: srcs must be statically "
                f"resolvable"
            )
        for item in raw:
            norm = _normalize_src_path(package, item)
            if norm is not None and norm not in srcs:
                srcs.append(norm)
    if "main" in kws:
        main_val = _eval_str(kws["main"], bindings)
        if main_val is None:
            return [], (
                f"{build_path}:{lineno}: {label}: main must be a static string"
            )
        norm = _normalize_src_path(package, main_val)
        if norm is not None and norm not in srcs:
            srcs.append(norm)
    if not srcs:
        return [], (
            f"{build_path}:{lineno}: {label}: declares no resolvable source "
            f"files"
        )
    return srcs, None


def _check_e2e_target(label, build_path, lineno, kws, package, bindings,
                      files_set, read, sim_check_ok, sim_check_err):
    """Checks an `e2e`- or `soak`-tagged target against docs/TESTING.md §8.2."""
    problems = []
    srcs, src_err = _extract_target_sources(
        label, build_path, lineno, kws, package, bindings)
    if src_err:
        problems.append(src_err)
        return problems

    uses_sim_check_src = SIM_CHECK_PATH in srcs
    own_srcs = [s for s in srcs if s != SIM_CHECK_PATH]
    if not own_srcs:
        # Target directly runs sim_check.py as its sole source.
        if not sim_check_ok:
            problems.append(
                f"{build_path}:{lineno}: {label} (e2e): {sim_check_err}"
            )
        return problems

    preserved = False
    delegated_to_sim_check = False
    for src in own_srcs:
        if src not in files_set:
            problems.append(
                f"{build_path}:{lineno}: {label} (e2e) source {src} "
                f"not found in workspace"
            )
            continue
        try:
            text = read(src)
        except (OSError, UnicodeDecodeError) as exc:
            problems.append(f"{src}: cannot read source: {exc}")
            continue
        direct, err = _source_mentions_marker(
            src, text, UNDECLARED_OUTPUTS_ENV, allow_sim_check=False)
        if err:
            problems.append(err)
            continue
        if direct:
            preserved = True
            continue
        if uses_sim_check_src:
            via_sim, err = _source_mentions_marker(
                src, text, UNDECLARED_OUTPUTS_ENV, allow_sim_check=True)
            if err:
                problems.append(err)
            elif via_sim:
                delegated_to_sim_check = True

    if delegated_to_sim_check and not sim_check_ok:
        problems.append(
            f"{build_path}:{lineno}: {label} (e2e) delegates to "
            f"{SIM_CHECK_PATH}, but {sim_check_err}"
        )
        return problems

    if not preserved and not delegated_to_sim_check:
        problems.append(
            f"{build_path}:{lineno}: {label} (e2e) must preserve failure "
            f"artifacts under {UNDECLARED_OUTPUTS_ENV} (directly or via "
            f"sim_check.preserve_failure_artifacts / sim_check.check; "
            f"docs/TESTING.md §8.2)"
        )
    return problems


class _BuildState:
    """Statement-order evaluation state for one BUILD file."""

    def __init__(self, path, package, files_set, read, sim_check_ok,
                 sim_check_err):
        self.path = path
        self.package = package
        self.files_set = files_set
        self.read = read
        self.sim_check_ok = sim_check_ok
        self.sim_check_err = sim_check_err
        self.bindings = {}
        self.cc_names = set(CC_TEST_RULES)
        self.sim_names = set(SIM_TEST_RULES)
        self.test_rule_names = set(KNOWN_TEST_RULES)
        self.problems = []
        self.perf_count = 0
        self.e2e_count = 0

    def clone_scope(self):
        copy = _BuildState(
            self.path, self.package, self.files_set, self.read,
            self.sim_check_ok, self.sim_check_err,
        )
        copy.bindings = dict(self.bindings)
        copy.cc_names = set(self.cc_names)
        copy.sim_names = set(self.sim_names)
        copy.test_rule_names = set(self.test_rule_names)
        return copy

    def merge_results(self, other):
        self.problems.extend(other.problems)
        self.perf_count += other.perf_count
        self.e2e_count += other.e2e_count

    def discard_rule_aliases(self, name):
        self.cc_names.discard(name)
        self.sim_names.discard(name)
        self.test_rule_names.discard(name)


def _subscript_root_name(node):
    cur = node
    while isinstance(cur, ast.Subscript):
        cur = cur.value
    return cur.id if isinstance(cur, ast.Name) else None


def _apply_load(call, state):
    """Registers test-rule aliases introduced by a Starlark `load(...)` call."""
    for arg in call.args[1:]:
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
            sym = arg.value
            if sym in CC_TEST_RULES:
                state.cc_names.add(sym)
                state.test_rule_names.add(sym)
            elif sym in SIM_TEST_RULES:
                state.sim_names.add(sym)
                state.test_rule_names.add(sym)
            elif sym in KNOWN_TEST_RULES or sym.endswith("_test"):
                state.test_rule_names.add(sym)
    for kw in call.keywords:
        if not (kw.arg and isinstance(kw.value, ast.Constant) and
                isinstance(kw.value.value, str)):
            continue
        orig = kw.value.value
        local = kw.arg
        state.discard_rule_aliases(local)
        state.bindings.pop(local, None)
        if orig in CC_TEST_RULES:
            state.cc_names.add(local)
            state.test_rule_names.add(local)
        elif orig in SIM_TEST_RULES:
            state.sim_names.add(local)
            state.test_rule_names.add(local)
        elif orig in KNOWN_TEST_RULES or orig.endswith("_test"):
            state.test_rule_names.add(local)


def _check_call(call, state):
    if isinstance(call.func, ast.Name) and call.func.id == "load":
        _apply_load(call, state)
        return

    rule_name = _call_rule_name(call)
    lineno = getattr(call, "lineno", 1)
    kws = {kw.arg: kw.value for kw in call.keywords if kw.arg is not None}
    star_kwargs = [kw.value for kw in call.keywords if kw.arg is None]

    name_val = (
        _eval_str(kws["name"], state.bindings)
        if "name" in kws else None
    )
    target_name = name_val or f"<dynamic:line{lineno}>"
    label = (
        f"//{state.package}:{target_name}"
        if state.package else f"//:{target_name}"
    )

    is_sim_test = rule_name in state.sim_names
    if is_sim_test:
        # talos_sim_test implicitly tags its underlying py_test with "e2e"
        # and runs talOS/build/sim_check.py. Still fall through so dynamic
        # tags/**kwargs or explicit "perf" tags are checked.
        state.e2e_count += 1
        if not state.sim_check_ok:
            state.problems.append(
                f"{state.path}:{lineno}: {label} (talos_sim_test): "
                f"{state.sim_check_err}"
            )

    possible_tags = set()
    tags_resolved = True
    if "tags" in kws:
        t_vals, t_ok = _possible_strings(kws["tags"], state.bindings)
        possible_tags |= t_vals
        tags_resolved = tags_resolved and t_ok
    for kw_node in star_kwargs:
        k_vals, k_ok = _possible_strings(kw_node, state.bindings)
        possible_tags |= k_vals
        tags_resolved = tags_resolved and k_ok

    if not tags_resolved and _is_test_rule(rule_name, state.test_rule_names):
        state.problems.append(
            f"{state.path}:{lineno}: {label}: tags/kwargs on test rule "
            f"{rule_name} must be statically resolvable"
        )

    if "perf" in possible_tags:
        state.perf_count += 1
        default_srcs = [SIM_CHECK_PATH] if is_sim_test else None
        state.problems.extend(_check_perf_target(
            label, state.path, lineno, rule_name in state.cc_names, kws,
            state.package, state.bindings, state.files_set, state.read,
            default_srcs=default_srcs,
        ))

    if not is_sim_test and ("e2e" in possible_tags or "soak" in possible_tags):
        state.e2e_count += 1
        state.problems.extend(_check_e2e_target(
            label, state.path, lineno, kws, state.package, state.bindings,
            state.files_set, state.read, state.sim_check_ok,
            state.sim_check_err,
        ))


def _walk_expr_calls(expr, state):
    calls = [node for node in ast.walk(expr) if isinstance(node, ast.Call)]
    calls.sort(key=lambda n: (getattr(n, "lineno", 0),
                              getattr(n, "col_offset", 0)))
    for call in calls:
        _check_call(call, state)


def _apply_assign(targets, value, state):
    snap_env = dict(state.bindings)
    prev_cc = set(state.cc_names)
    prev_sim = set(state.sim_names)
    prev_test = set(state.test_rule_names)

    alias_of = None
    if isinstance(value, ast.Name):
        alias_of = value.id
    elif isinstance(value, ast.Attribute):
        alias_of = value.attr

    modified = set()
    for target in targets:
        if isinstance(target, ast.Name):
            tid = target.id
            modified.add(tid)
            state.bindings[tid] = _BoundExpr(value, snap_env)
            state.discard_rule_aliases(tid)
            if alias_of:
                if alias_of in prev_cc:
                    state.cc_names.add(tid)
                    state.test_rule_names.add(tid)
                if alias_of in prev_sim:
                    state.sim_names.add(tid)
                    state.test_rule_names.add(tid)
                if alias_of in prev_test or alias_of.endswith("_test"):
                    state.test_rule_names.add(tid)
        elif isinstance(target, ast.Subscript):
            root = _subscript_root_name(target)
            if root:
                modified.add(root)
                state.bindings[root] = _UNRESOLVABLE
                state.discard_rule_aliases(root)
        else:
            for node in ast.walk(target):
                if isinstance(node, ast.Name):
                    modified.add(node.id)
                    state.bindings[node.id] = _UNRESOLVABLE
                    state.discard_rule_aliases(node.id)
    return modified


def _apply_aug_assign(target, op, value, state):
    snap_env = dict(state.bindings)
    modified = set()
    if isinstance(target, ast.Name):
        tid = target.id
        modified.add(tid)
        state.discard_rule_aliases(tid)
        if isinstance(op, (ast.Add, ast.BitOr)):
            prev = state.bindings.get(tid, _UNRESOLVABLE)
            state.bindings[tid] = _AugBinding(
                prev, _BoundExpr(value, snap_env),
                is_add=isinstance(op, ast.Add),
            )
        else:
            state.bindings[tid] = _UNRESOLVABLE
    elif isinstance(target, ast.Subscript):
        root = _subscript_root_name(target)
        if root:
            modified.add(root)
            state.bindings[root] = _UNRESOLVABLE
            state.discard_rule_aliases(root)
    return modified


def _apply_mutating_call(expr, state):
    """Tracks in-place list/dict mutations like `TAGS.append('perf')`."""
    if not isinstance(expr, ast.Call):
        return set()
    func = expr.func
    if not (isinstance(func, ast.Attribute) and
            isinstance(func.value, ast.Name)):
        return set()
    target = func.value.id
    if target not in state.bindings:
        return set()
    method = func.attr
    if method in ("append", "extend") and len(expr.args) == 1 and not expr.keywords:
        snap_env = dict(state.bindings)
        state.bindings[target] = _AugBinding(
            state.bindings[target], _BoundExpr(expr.args[0], snap_env),
            is_add=True,
        )
        return {target}
    if method in ("insert", "pop", "remove", "clear", "update", "setdefault"):
        state.bindings[target] = _UNRESOLVABLE
        state.discard_rule_aliases(target)
        return {target}
    return set()


def _walk_statements(stmts, state):
    """Evaluates BUILD statements in source order, returning names modified."""
    modified = set()
    for stmt in stmts:
        if isinstance(stmt, ast.Expr):
            _walk_expr_calls(stmt.value, state)
            modified |= _apply_mutating_call(stmt.value, state)
        elif isinstance(stmt, ast.Assign):
            _walk_expr_calls(stmt.value, state)
            modified |= _apply_assign(stmt.targets, stmt.value, state)
        elif isinstance(stmt, ast.AugAssign):
            _walk_expr_calls(stmt.value, state)
            modified |= _apply_aug_assign(
                stmt.target, stmt.op, stmt.value, state)
        elif isinstance(stmt, ast.AnnAssign):
            if stmt.value is not None:
                _walk_expr_calls(stmt.value, state)
                modified |= _apply_assign([stmt.target], stmt.value, state)
        elif isinstance(stmt, ast.For):
            _walk_expr_calls(stmt.iter, state)
            loop_state = state.clone_scope()
            for node in ast.walk(stmt.target):
                if isinstance(node, ast.Name):
                    loop_state.bindings[node.id] = _UNRESOLVABLE
                    loop_state.discard_rule_aliases(node.id)
                    modified.add(node.id)
            inner_mod = _walk_statements(stmt.body, loop_state)
            inner_mod |= _walk_statements(stmt.orelse, loop_state)
            state.merge_results(loop_state)
            for name in inner_mod:
                state.bindings[name] = _UNRESOLVABLE
                state.discard_rule_aliases(name)
            modified |= inner_mod
        elif isinstance(stmt, ast.If):
            _walk_expr_calls(stmt.test, state)
            then_state = state.clone_scope()
            else_state = state.clone_scope()
            inner_mod = _walk_statements(stmt.body, then_state)
            inner_mod |= _walk_statements(stmt.orelse, else_state)
            state.merge_results(then_state)
            state.merge_results(else_state)
            for name in inner_mod:
                state.bindings[name] = _UNRESOLVABLE
                state.discard_rule_aliases(name)
            modified |= inner_mod
        elif isinstance(stmt, ast.FunctionDef):
            fn_state = state.clone_scope()
            for arg in (stmt.args.posonlyargs + stmt.args.args +
                        stmt.args.kwonlyargs):
                fn_state.bindings[arg.arg] = _UNRESOLVABLE
                fn_state.discard_rule_aliases(arg.arg)
            if stmt.args.vararg:
                fn_state.bindings[stmt.args.vararg.arg] = _UNRESOLVABLE
            if stmt.args.kwarg:
                fn_state.bindings[stmt.args.kwarg.arg] = _UNRESOLVABLE
            _walk_statements(stmt.body, fn_state)
            state.merge_results(fn_state)
        else:
            _walk_expr_calls(stmt, state)
    return modified


def check(files, read):
    """Returns (problems, stats) across the workspace's tracked files."""
    files_set = set(files)
    problems = []
    perf_count = 0
    e2e_count = 0

    sim_check_ok, sim_check_err = _sim_check_preserves_artifacts(
        files_set, read)

    for path in sorted(files):
        if not _is_build_file(path):
            continue
        try:
            text = read(path)
        except (OSError, UnicodeDecodeError) as exc:
            problems.append(f"{path}: cannot read BUILD file: {exc}")
            continue
        try:
            tree = ast.parse(text, filename=path)
        except SyntaxError as exc:
            loc = f":{exc.lineno}" if exc.lineno else ""
            problems.append(f"{path}{loc}: cannot parse BUILD file: {exc.msg}")
            continue

        package = os.path.dirname(path)
        state = _BuildState(
            path, package, files_set, read, sim_check_ok, sim_check_err)
        _walk_statements(tree.body, state)
        problems.extend(state.problems)
        perf_count += state.perf_count
        e2e_count += state.e2e_count

    return problems, {"perf": perf_count, "e2e": e2e_count}


def tracked_files():
    out = subprocess.check_output(["git", "ls-files", "-z"])
    return [p.decode("utf-8") for p in out.split(b"\0") if p]


def main():
    files = tracked_files()

    def read(path):
        with open(path, encoding="utf-8") as f:
            return f.read()

    problems, stats = check(files, read)
    for problem in problems:
        print(problem)
    if problems:
        return 1
    print(
        f"testing-rules: {stats['perf']} perf target(s) host-qualified, "
        f"{stats['e2e']} e2e/soak target(s) preserve failure artifacts"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
