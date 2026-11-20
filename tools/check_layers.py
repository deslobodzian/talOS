"""Checks the Bazel dependency graph and C++ node sources against
ARCHITECTURE.md's layers and RULES.md 2.

    python3 tools/check_layers.py                # from the workspace root
    python3 tools/check_layers.py --source-only  # C++ bypass scan only (no Bazel)

The Bazel checks are `bazel query` expressions that must come back empty: a
`somepath` from a lower set of targets to a higher one, or the direct
dependencies of a set that must reach the rest of talOS through one gateway.
The table below is ARCHITECTURE.md "Layers" and "Allowed dependencies"
(REQ-CONTRACT-15, REQ-CONTRACT-16) and RULES.md 2 (REQ-SAFE-27):

- the stack: rtms (with memory and ipc) < events < node < devices; a layer
  never depends on a layer above it, nor on the talOS packages built on the
  stack (the bridge, launcher, simulator, framework nodes, node_api,
  python), and talOS/node drives no hardware (it never depends on
  talOS/hardware);
- nothing under talOS/ depends on a robot, an example, the bench, a
  controller processor's program or studio/;
- talOS/geometry never depends on talOS/vision;
- talOS/python reaches the rest of talOS only through talOS/node_api's C ABI,
  generated messages and schemas, and C++ nodes exported for talos.cpp_node;
- a robot, example or bench node package depends directly on no rule of
  another node package but its talos_messages targets and the C++ nodes it
  exports for talos.cpp_node; this holds for every target of the package,
  its tests included, so a node's own targets never link another node's
  class;
- outside node packages (under the same roots), only a testonly target
  links a node package's rule other than those and its node binaries: a
  test of several nodes together may link their libraries, and nothing that
  ships can;
- non-testonly C++ nodes (`talos_cc_node`, `talos_cc_shared_node`) and
  `cc_library` targets outside `talOS/` never depend directly on
  `//talOS/rtms/...`, `//talOS/ipc/...`, `//talOS/memory/...`,
  `//talOS/hardware:hardware`, or internal `//talOS/node:*` targets
  (anything in `//talOS/node:*` other than `//talOS/node:node`).

The C++ source scan (also run standalone via `--source-only` in
`tools/source_lints.sh`) walks tracked C++ sources and headers in node
packages outside `talOS/` and refuses from the start (RULES.md 2):

- `#include` (or `#include_next`, quote or angle-bracket, after comment
  stripping, line-continuation splicing and path normalization) of
  `talOS/node/access.h`, `talOS/node/internal.h`, `talOS/devices/access.h`,
  `talOS/rtms/...`, `talOS/ipc/...`, `talOS/memory/...`,
  `talOS/hardware/messages.h`, `talOS/hardware/packet.h`, or
  `talOS/hardware/request...`, as well as computed `#include MACRO`
  directives that cannot be verified statically;
- non-comment, non-string references to `NodeAccess`, `add_hardware_request`,
  `HardwareAccess`, or `note_live_target`.

ALLOWLIST holds today's known Bazel exceptions, each with its reason and the
TODO.md item that removes it. NON_NODE_CLI_FILES holds non-node CLI binaries
that sit beside a node package (never exempting `NodeAccess`, `access.h` or
`add_hardware_request`). Both may only shrink: a stale entry fails the check,
so it is deleted with the fix. Prints each violation; exits 1 on a violation
or a stale entry.
"""

import argparse
import dataclasses
import os
import posixpath
import re
import subprocess
import sys

# ARCHITECTURE.md "Layers", bottom up. Each entry is (layer, packages).
STACK = (
    ("rtms", ("talOS/memory", "talOS/ipc", "talOS/rtms")),
    ("events", ("talOS/events",)),
    ("node", ("talOS/node",)),
    ("devices", ("talOS/devices",)),
)

# talOS packages built on the stack: processes and framework nodes, the C ABI
# and the Python API over it. No stack layer depends on them.
ON_THE_STACK = (
    "talOS/bridge",
    "talOS/launcher",
    "talOS/mechanisms",
    "talOS/node_api",
    "talOS/python",
    "talOS/simulation",
    "talOS/telemetry",
    "talOS/teleop",
)

# "talOS/node drives no hardware": the layers up to node never depend on the
# hardware config and wire package.
HARDWARE = "talOS/hardware"

# Robot code and what sits beside talOS; talOS never depends on them.
OUTSIDE_TALOS = ("bench", "controller_processor", "examples", "studio")

# Where robot, example and bench node packages live.
NODE_ROOTS = ("bench", "examples")
NODE_GENERATORS = ("talos_cc_node", "talos_py_node", "talos_cc_shared_node")

# Where non-testonly C++ nodes and node libraries outside talOS/ live for the
# RULES.md 2 direct-dependency check.
CC_NODE_DEP_ROOTS = ("bench", "examples", "tools/scaffold_testdata")
CC_NODE_GENERATORS = ("talos_cc_node", "talos_cc_shared_node")
FORBIDDEN_NODE_DEPS = (
    "talOS/rtms/...",
    "talOS/ipc/...",
    "talOS/memory/...",
    "talOS/hardware:hardware",
    "talOS/node:*",
)
FORBIDDEN_NODE_DEPS_EXCLUDE = ("talOS/node:node",)

# What talOS/python may depend on directly outside itself.
PYTHON_GATEWAYS = ("talOS/node_api", "talOS/schema", "talOS/build",
                   "talOS/msgs")

# Generated targets another package may depend on: messages, and C++ nodes
# exported for talos.cpp_node (ARCHITECTURE.md "Allowed dependencies").
SHARED_GENERATORS = ("talos_messages", "talos_cc_shared_node")

# (from, to) -> reason. Remove an entry with the change that removes its
# dependency; the check fails while a stale entry remains.
ALLOWLIST = {
    ("talOS/node", "talOS/hardware"):
        "talOS/node reads hardware::Config timeouts and actuator ownership "
        "through talOS/configuration, and names.h's /hw topic names through "
        "talOS/introspection (TODO ARCH-2)",
    ("talOS/devices", "talOS/bridge"):
        "talOS/devices:testing (SimHardware) runs the bridge's merge rules, "
        "//talOS/bridge:merge (TODO ARCH-2)",
}

# C++ source bypass scan (RULES.md 2, REQ-SAFE-27).
EXCLUDED_SOURCE_PREFIXES = (
    "talOS/",
    "third_party/",
    "controller_processor/",
    "studio/",
)
CPP_SUFFIXES = (
    ".cc", ".cpp", ".cxx", ".c",
    ".h", ".hh", ".hpp", ".hxx", ".ipp", ".inl", ".inc",
)
NODE_MACRO_RE = re.compile(
    r"\b(?:talos_cc_node|talos_py_node|talos_cc_shared_node)\b")
INCLUDE_DIRECTIVE_RE = re.compile(
    r"^[ \t]*(?:#|%:)[ \t]*(include(?:_next)?)\b(.*)$")
HEADER_NAME_RE = re.compile(r'^(?:"([^"]*)"|<([^>]*)>)')

FORBIDDEN_INCLUDE_EXACT = frozenset({
    "talOS/node/access.h",
    "talOS/node/internal.h",
    "talOS/devices/access.h",
    "talOS/hardware/messages.h",
    "talOS/hardware/packet.h",
})
FORBIDDEN_INCLUDE_PREFIXES = (
    "talOS/rtms/",
    "talOS/ipc/",
    "talOS/memory/",
    "talOS/hardware/request",
)
NEVER_EXEMPT_INCLUDES = frozenset({
    "talOS/node/access.h",
    "talOS/node/internal.h",
})
FORBIDDEN_SYMBOLS = (
    ("NodeAccess", re.compile(r"\bNodeAccess\b")),
    ("add_hardware_request", re.compile(r"\badd_hardware_request\b")),
    ("HardwareAccess", re.compile(r"\bHardwareAccess\b")),
    ("note_live_target", re.compile(r"\bnote_live_target\b")),
)

# Standalone non-node CLI binaries sitting beside a node package (not linked
# by any talos_cc_node or node library). Only transport/wire includes are
# exempted; NodeAccess, access.h, add_hardware_request, computed includes and
# syntax errors are never exempted. Checked for staleness so it only shrinks.
NON_NODE_CLI_FILES = {
    "bench/main_processor/swerve_module/send_target.cc":
        "operator CLI beside the node (tools/test_rio_in_the_loop.py); "
        "not node code",
}


@dataclasses.dataclass(frozen=True)
class Set:
    """A set of targets: the include patterns minus the exclude patterns.

    A pattern is `pkg/...` (the package and every package below it), `pkg:*`
    (one package) or `pkg:target` (one target). `rules_only` keeps rule
    targets only (no source or generated files); `only_kinds` and
    `only_generators` narrow to rules matching any named rule kind or
    generator macro; `except_generators` drops the targets a named macro
    generated; `except_testonly` drops testonly targets (every test rule).
    """
    include: tuple
    exclude: tuple = ()
    rules_only: bool = False
    except_generators: tuple = ()
    except_testonly: bool = False
    only_kinds: tuple = ()
    only_generators: tuple = ()


@dataclasses.dataclass(frozen=True)
class Check:
    """`frm` must not reach any of `to` (kind "path": through any number of
    dependencies; kind "direct": in one). `source` narrows `frm`'s set."""
    name: str
    frm: str
    to: tuple
    kind: str = "path"
    exclude: tuple = ()
    rules_only: bool = False
    except_generators: tuple = ()
    source: Set = None

    def target_set(self, to):
        return Set(tuple(to), self.exclude, self.rules_only,
                   self.except_generators)

    def search(self, backend, to):
        """The path or direct dependencies from `frm` to `to`, or []."""
        source = self.source or Set((self.frm,))
        target = self.target_set(to)
        if self.kind == "path":
            return backend.somepath(source, target)
        return backend.direct(source, target)


@dataclasses.dataclass(frozen=True)
class SourceViolation:
    path: str
    line: int
    kind: str
    detail: str
    header: str = ""


def recursive(package):
    return package + "/..."


def checks(node_packages):
    """Every check, given the robot, example and bench node packages."""
    result = []
    for i, (layer, packages) in enumerate(STACK):
        above = [p for _, higher in STACK[i + 1:] for p in higher]
        above += ON_THE_STACK
        if layer != "devices":
            above.append(HARDWARE)
        for package in packages:
            result.append(Check(f"{layer} layer below {', '.join(above)}",
                                recursive(package),
                                tuple(recursive(p) for p in above)))
    result.append(Check("talOS/ below robots, examples, bench and studio",
                        "talOS/...",
                        tuple(recursive(p) for p in OUTSIDE_TALOS)))
    result.append(Check("talOS/geometry below talOS/vision",
                        "talOS/geometry/...", ("talOS/vision/...",)))
    result.append(Check(
        "talOS/python reaches talOS only through talOS/node_api",
        "talOS/python/...", ("talOS/...",), kind="direct",
        exclude=tuple(recursive(p) for p in ("talOS/python",) +
                      PYTHON_GATEWAYS),
        except_generators=("talos_cc_shared_node",)))
    for package in node_packages:
        others = tuple(f"{p}:*" for p in node_packages if p != package)
        if others:
            result.append(Check(
                f"node package {package} shares only messages", f"{package}:*",
                others, kind="direct", rules_only=True,
                except_generators=SHARED_GENERATORS))
    every_node = tuple(f"{p}:*" for p in node_packages)
    for root in NODE_ROOTS:
        if not every_node:
            break
        # Node binaries are executables a bundle carries, not a class.
        result.append(Check(
            f"only testonly targets outside node packages under {root} link "
            f"a node", recursive(root), every_node, kind="direct",
            rules_only=True,
            except_generators=SHARED_GENERATORS + NODE_GENERATORS,
            source=Set((recursive(root),), every_node, rules_only=True,
                       except_testonly=True)))
    result.append(Check(
        "C++ nodes and libraries outside talOS/ never depend directly on "
        "raw transport, hardware wire or internal node targets (RULES.md 2)",
        "outside-talOS-cc-nodes",
        FORBIDDEN_NODE_DEPS,
        kind="direct",
        exclude=FORBIDDEN_NODE_DEPS_EXCLUDE,
        source=Set(
            tuple(recursive(r) for r in CC_NODE_DEP_ROOTS),
            only_kinds=("cc_library",),
            only_generators=CC_NODE_GENERATORS,
            except_testonly=True,
        ),
    ))
    return result


def strip_pattern(pattern):
    """The package a single-package or recursive pattern names."""
    return pattern.removesuffix("/...").removesuffix(":*")


def run(backend, out=sys.stdout):
    """Runs every Bazel check against `backend`; returns the failure count."""
    failures = 0
    used = set()
    all_checks = checks(backend.node_packages())
    for check in all_checks:
        frm = strip_pattern(check.frm)
        allowed = [t for t in check.to if (frm, strip_pattern(t)) in ALLOWLIST]
        found = check.search(backend,
                             [t for t in check.to if t not in allowed])
        if found:
            failures += 1
            what = "path" if check.kind == "path" else "direct dependencies"
            print(f"VIOLATION {check.name} ({check.frm}); {what}:", file=out)
            for label in found:
                print(f"  {label}", file=out)
        for t in allowed:
            entry = (frm, strip_pattern(t))
            used.add(entry)
            if not check.search(backend, [t]):
                failures += 1
                print(f"STALE allowlist entry {entry[0]} -> {entry[1]}: the "
                      f"dependency is gone; delete the entry", file=out)
    for frm, to in sorted(set(ALLOWLIST) - used):
        failures += 1
        print(f"STALE allowlist entry {frm} -> {to}: no check covers it",
              file=out)
    if failures:
        print(f"{failures} layer check(s) failed (docs/DESIGN.md §3)",
              file=out)
    else:
        print(f"{len(all_checks)} layer checks passed, "
              f"{len(ALLOWLIST)} allowlisted dependencies remain", file=out)
    return failures


def render(target_set):
    """The bazel query expression for a Set."""
    def union(patterns):
        return " + ".join(f"//{p}" for p in patterns)
    expr = union(target_set.include)
    if target_set.exclude:
        expr = f"({expr}) except ({union(target_set.exclude)})"
    if target_set.only_kinds or target_set.only_generators:
        parts = []
        if target_set.only_kinds:
            kinds = "|".join(f"^{k} rule$" for k in target_set.only_kinds)
            parts.append(f'kind("{kinds}", {expr})')
        if target_set.only_generators:
            generators = "|".join(target_set.only_generators)
            parts.append(
                f'attr(generator_function, "^({generators})$", {expr})')
        expr = " + ".join(parts)
    elif target_set.rules_only:
        expr = f"kind(rule, {expr})"
    if target_set.except_generators:
        generators = "|".join(target_set.except_generators)
        expr = (f"({expr}) except "
                f"attr(generator_function, \"^({generators})$\", {expr})")
    if target_set.except_testonly:
        expr = f"({expr}) except attr(testonly, 1, {expr})"
    return expr


class Bazel:
    """Answers the checks with `bazel query` in the current workspace."""

    def query(self, expr, output="label"):
        # --keep_going: some external packages cannot load on every host
        # (opencv's Android branch); the query then exits 3 with every
        # other edge evaluated (docs/DEVELOPMENT.md §5).
        proc = subprocess.run(
            ["bazel", "query", "--keep_going", "--noimplicit_deps",
             "--noshow_progress", f"--output={output}", expr],
            capture_output=True, text=True, check=False)
        if proc.returncode not in (0, 3):
            sys.stderr.write(proc.stderr)
            raise RuntimeError(f"bazel query failed ({proc.returncode}): "
                               f"{expr}")
        return [line for line in proc.stdout.splitlines() if line]

    def node_packages(self):
        roots = " + ".join(f"//{r}/..." for r in NODE_ROOTS)
        generators = "|".join(NODE_GENERATORS)
        return sorted(set(self.query(
            f"attr(generator_function, \"^({generators})$\", {roots})",
            output="package")))

    def somepath(self, frm, to):
        return self.query(f"somepath({render(frm)}, {render(to)})")

    def direct(self, frm, to):
        source = render(frm)
        return self.query(f"(deps({source}, 1) except ({source})) "
                          f"intersect ({render(to)})")


def _strip_starlark_comments(text):
    """Removes Starlark `#` comments outside single- and double-quoted strings."""
    out = []
    for line in text.splitlines():
        in_quote = None
        i = 0
        n = len(line)
        while i < n:
            ch = line[i]
            if in_quote:
                if ch == "\\" and i + 1 < n:
                    i += 2
                    continue
                if ch == in_quote:
                    in_quote = None
            elif ch in ('"', "'"):
                in_quote = ch
            elif ch == "#":
                line = line[:i]
                break
            i += 1
        out.append(line)
    return "\n".join(out)


def _is_ident_char(ch):
    return ch.isalnum() or ch == "_"


def _has_char_or_raw_prefix(src, idx):
    """True when `src[idx]` is not preceded by an identifier, or is preceded
    only by a C++ string/char encoding prefix (`u8`, `u`, `U`, `L`)."""
    if idx == 0 or not _is_ident_char(src[idx - 1]):
        return True
    if (idx >= 2 and src[idx - 2:idx] == "u8" and
            (idx == 2 or not _is_ident_char(src[idx - 3]))):
        return True
    if (src[idx - 1] in ("u", "U", "L") and
            (idx == 1 or not _is_ident_char(src[idx - 2]))):
        return True
    return False


def _strip_cpp_comments(text):
    """Splices C++ line continuations and strips comments, preserving line numbers.

    Returns `(with_strings, no_strings, line_map, errors)`:
    - `with_strings`: comment-stripped source with normal string literals kept
      (used to parse `#include "..."` directives);
    - `no_strings`: same length, with string/char literal bodies blanked to
      spaces (used to find forbidden identifier references);
    - `line_map`: 1-based original line number for every character;
    - `errors`: list of `(line, kind, detail)` for fail-closed lex errors
      (unterminated block comment or raw string).
    """
    # Phase 2: splice backslash-newline continuations (including optional
    # trailing horizontal whitespace, which GCC/Clang accept).
    spliced = []
    spliced_lines = []
    line = 1
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        if ch == "\\":
            j = i + 1
            while j < n and text[j] in (" ", "\t"):
                j += 1
            if j < n and text[j] == "\r":
                j += 1
            if j < n and text[j] == "\n":
                i = j + 1
                line += 1
                continue
        spliced.append(ch)
        spliced_lines.append(line)
        if ch == "\n":
            line += 1
        i += 1

    src = "".join(spliced)
    n = len(src)
    with_strings = []
    no_strings = []
    line_map = []
    errors = []

    def emit(ch_ws, ch_ns, lineno):
        with_strings.append(ch_ws)
        no_strings.append(ch_ns)
        line_map.append(lineno)

    i = 0
    while i < n:
        ch = src[i]
        nxt = src[i + 1] if i + 1 < n else ""
        # Line comment: replace with space, leave '\n' intact.
        if ch == "/" and nxt == "/":
            emit(" ", " ", spliced_lines[i])
            i += 2
            while i < n and src[i] != "\n":
                i += 1
            continue
        # Block comment: C++ translation phase 3 replaces the entire comment
        # (including newlines inside it) with a single space.
        if ch == "/" and nxt == "*":
            end = src.find("*/", i + 2)
            if end == -1:
                errors.append((spliced_lines[i], "unterminated-comment",
                               "unterminated block comment"))
                break
            emit(" ", " ", spliced_lines[i])
            i = end + 2
            continue
        # Raw string literal: R"delim(...)delim" (with optional u8/u/U/L).
        if ch == "R" and nxt == '"' and _has_char_or_raw_prefix(src, i):
            paren = src.find("(", i + 2)
            delim = src[i + 2:paren] if paren != -1 else ""
            if (paren == -1 or len(delim) > 16 or
                    any(c in (" ", "\t", "\r", "\n", "\\", ")")
                        for c in delim)):
                errors.append((spliced_lines[i], "unterminated-string",
                               "malformed raw string literal"))
                break
            closing = ")" + delim + '"'
            end = src.find(closing, paren + 1)
            if end == -1:
                errors.append((spliced_lines[i], "unterminated-string",
                               "unterminated raw string literal"))
                break
            stop = end + len(closing)
            for k in range(i, stop):
                c = "\n" if src[k] == "\n" else " "
                emit(c, c, spliced_lines[k])
            i = stop
            continue
        # Normal string literal: preserve in `with_strings` (for #include),
        # blank interior in `no_strings`.
        if ch == '"':
            emit('"', '"', spliced_lines[i])
            i += 1
            while i < n:
                c = src[i]
                if c == "\\":
                    emit(c, " ", spliced_lines[i])
                    i += 1
                    if i < n and src[i] != "\n":
                        emit(src[i], " ", spliced_lines[i])
                        i += 1
                    continue
                if c == '"':
                    emit('"', '"', spliced_lines[i])
                    i += 1
                    break
                if c == "\n":
                    emit("\n", "\n", spliced_lines[i])
                    i += 1
                    break
                emit(c, " ", spliced_lines[i])
                i += 1
            continue
        # Character literal (not a C++14 numeric digit separator like 1'000).
        if ch == "'" and _has_char_or_raw_prefix(src, i):
            emit("'", "'", spliced_lines[i])
            i += 1
            while i < n:
                c = src[i]
                if c == "\\":
                    emit(c, " ", spliced_lines[i])
                    i += 1
                    if i < n and src[i] != "\n":
                        emit(src[i], " ", spliced_lines[i])
                        i += 1
                    continue
                if c == "'":
                    emit("'", "'", spliced_lines[i])
                    i += 1
                    break
                if c == "\n":
                    emit("\n", "\n", spliced_lines[i])
                    i += 1
                    break
                emit(c, " ", spliced_lines[i])
                i += 1
            continue
        emit(ch, ch, spliced_lines[i])
        i += 1

    return "".join(with_strings), "".join(no_strings), line_map, errors


def _normalize_include(header):
    """Canonicalizes an included header path for bypass matching."""
    norm = posixpath.normpath(header.replace("\\", "/"))
    while norm.startswith("../"):
        norm = norm[3:]
    if norm.startswith("./"):
        norm = norm[2:]
    marker = "/talOS/"
    if marker in norm:
        norm = norm[norm.index(marker) + 1:]
    return norm


def _is_forbidden_include(norm):
    if norm in FORBIDDEN_INCLUDE_EXACT:
        return True
    return norm.startswith(FORBIDDEN_INCLUDE_PREFIXES)


def scan_cpp_text(path, text):
    """Returns a list of SourceViolation found in one C++ source/header."""
    violations = []
    with_strings, no_strings, line_map, errors = _strip_cpp_comments(text)
    for lineno, kind, detail in errors:
        violations.append(SourceViolation(path, lineno, kind, detail))

    # Check every logical line for #include / #include_next directives.
    offset = 0
    for segment in with_strings.splitlines(keepends=True):
        line_text = segment.rstrip("\r\n")
        m = INCLUDE_DIRECTIVE_RE.match(line_text)
        if m:
            lineno = line_map[offset + m.start()] if line_map else 1
            directive = m.group(1)
            rest = m.group(2).strip()
            hm = HEADER_NAME_RE.match(rest)
            if not hm:
                violations.append(SourceViolation(
                    path, lineno, "computed-include",
                    f"#{directive} {rest or '<empty>'} (computed #include "
                    f"cannot be verified statically)"))
            else:
                raw_header = hm.group(1) if hm.group(1) is not None else hm.group(2)
                norm = _normalize_include(raw_header)
                if _is_forbidden_include(norm):
                    violations.append(SourceViolation(
                        path, lineno, "forbidden-include",
                        f"#{directive} \"{raw_header}\" (normalized: {norm})",
                        header=norm))
        offset += len(segment)

    # Check non-comment, non-string code for forbidden internal symbols.
    for symbol, pattern in FORBIDDEN_SYMBOLS:
        for m in pattern.finditer(no_strings):
            lineno = line_map[m.start()] if line_map else 1
            violations.append(SourceViolation(
                path, lineno, "forbidden-symbol",
                f"references forbidden framework internal `{symbol}`"))

    violations.sort(key=lambda v: (v.line, v.kind, v.detail))
    return violations


def _owning_package(path, build_dirs):
    """Nearest ancestor directory (or `""` for root) holding a BUILD file."""
    directory = os.path.dirname(path)
    while True:
        if directory in build_dirs:
            return directory
        if not directory:
            return None
        directory = os.path.dirname(directory)


def find_node_source_files(files, read):
    """Discovers C++ sources/headers in node packages outside `talOS/`.

    Returns `(cpp_files, read_errors)` where `read_errors` is a list of
    `(path, detail)` for any BUILD file that could not be read (fail closed).
    """
    build_dirs = set()
    node_pkgs = set()
    read_errors = []
    for path in files:
        if os.path.basename(path) not in ("BUILD", "BUILD.bazel"):
            continue
        pkg = os.path.dirname(path)
        build_dirs.add(pkg)
        if path.startswith(EXCLUDED_SOURCE_PREFIXES):
            continue
        try:
            content = read(path)
        except OSError as exc:
            read_errors.append((path, f"cannot read BUILD file: {exc}"))
            continue
        if NODE_MACRO_RE.search(_strip_starlark_comments(content)):
            node_pkgs.add(pkg)

    cpp_files = []
    for path in sorted(files):
        if path.startswith(EXCLUDED_SOURCE_PREFIXES):
            continue
        if os.path.splitext(path)[1].lower() not in CPP_SUFFIXES:
            continue
        pkg = _owning_package(path, build_dirs)
        if pkg is not None and pkg in node_pkgs:
            cpp_files.append(path)
    return cpp_files, read_errors


def _is_exempt_cli_violation(violation, exempt_cli_files):
    if violation.path not in exempt_cli_files:
        return False
    if violation.kind != "forbidden-include":
        return False
    return violation.header not in NEVER_EXEMPT_INCLUDES


def run_source_checks(files, read, exempt_cli_files=None, out=sys.stdout):
    """Scans C++ sources in node packages outside `talOS/` for RULES.md 2
    bypasses. Returns the number of failures."""
    if exempt_cli_files is None:
        exempt_cli_files = NON_NODE_CLI_FILES
    cpp_files, read_errors = find_node_source_files(files, read)
    failures = 0
    for path, detail in read_errors:
        failures += 1
        print(f"VIOLATION {path}: {detail} (fail closed)", file=out)

    used_exemptions = set()
    for path in cpp_files:
        try:
            text = read(path)
        except OSError as exc:
            failures += 1
            print(f"VIOLATION {path}: cannot read source file: {exc} "
                  f"(fail closed)", file=out)
            continue
        for violation in scan_cpp_text(path, text):
            if _is_exempt_cli_violation(violation, exempt_cli_files):
                used_exemptions.add(path)
                continue
            failures += 1
            print(f"VIOLATION {violation.path}:{violation.line}: "
                  f"{violation.detail} (RULES.md 2 / REQ-SAFE-27)", file=out)

    for path in sorted(set(exempt_cli_files) - used_exemptions):
        failures += 1
        print(f"STALE non-node CLI exemption {path}: no exempted hit left; "
              f"delete the entry", file=out)

    if failures:
        print(f"{failures} RULES.md 2 C++ bypass check(s) failed", file=out)
    else:
        print(f"C++ devices-layer bypass check passed "
              f"({len(cpp_files)} node source file(s), "
              f"{len(exempt_cli_files)} non-node CLI exemption(s))", file=out)
    return failures


def tracked_files():
    """Returns tracked files from git; fails closed on error."""
    proc = subprocess.run(
        ["git", "ls-files", "-z"],
        capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(
            f"git ls-files failed ({proc.returncode}): {proc.stderr.strip()}")
    return [path for path in proc.stdout.split("\0") if path]


def read_source_file(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Checks Bazel layer dependencies and C++ RULES.md 2 bypasses.")
    parser.add_argument(
        "--source-only", action="store_true",
        help="Run only the C++ node-source bypass scan (no Bazel needed).")
    args = parser.parse_args(argv)

    failures = 0
    if not args.source_only:
        failures += run(Bazel())
    failures += run_source_checks(tracked_files(), read_source_file)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
