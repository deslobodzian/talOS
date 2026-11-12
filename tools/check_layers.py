"""Checks the Bazel dependency graph against ARCHITECTURE.md's layers.

    python3 tools/check_layers.py        # from the workspace root

Each check is a `bazel query` that must come back empty: a `somepath` from a
lower set of targets to a higher one, or the direct dependencies of a set
that must reach the rest of talOS through one gateway. The table below is
ARCHITECTURE.md "Layers" and "Allowed dependencies"
(REQ-CONTRACT-15, REQ-CONTRACT-16):

- the stack: rtms (with memory and ipc) < events < node < devices; a layer
  never depends on a layer above it, nor on the talOS packages built on the
  stack (the bridge, launcher, framework nodes, node_api, python), and
  talOS/node drives no hardware (it never depends on talOS/hardware);
- nothing under talOS/ depends on a robot, an example, the bench or studio/;
- talOS/geometry never depends on talOS/vision;
- talOS/python reaches the rest of talOS only through talOS/node_api's C ABI,
  generated messages and schemas, and C++ nodes exported for talos.cpp_node;
- a robot, example or bench node package depends directly on no rule of
  another node package but its talos_messages targets and the C++ nodes it
  exports for talos.cpp_node.

ALLOWLIST holds today's known exceptions, each with its reason and the
TODO.md item that removes it. It may only shrink: an entry whose dependency
no longer exists fails the check, so it is deleted with the fix. Prints each
violation with one dependency path; exits 1 on a violation or a stale entry.
"""

import dataclasses
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
    "talOS/telemetry",
    "talOS/teleop",
)

# "talOS/node drives no hardware": the layers up to node never depend on the
# hardware config and wire package.
HARDWARE = "talOS/hardware"

# Robot code and what sits beside talOS; talOS never depends on them.
OUTSIDE_TALOS = ("2026-robot", "bench", "examples", "studio")

# Where robot, example and bench node packages live.
NODE_ROOTS = ("2026-robot", "bench", "examples")
NODE_GENERATORS = ("talos_cc_node", "talos_py_node", "talos_cc_shared_node")

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
    ("2026-robot/main_processor/driver", "2026-robot/main_processor/drivetrain"):
        "driver's node_test composes the teleop chain with "
        "drivetrain::Drivetrain under Sim (TODO ARCH-2)",
}


@dataclasses.dataclass(frozen=True)
class Set:
    """A set of targets: the include patterns minus the exclude patterns.

    A pattern is `pkg/...` (the package and every package below it) or
    `pkg:*` (one package). `rules_only` keeps rule targets only (no source
    or generated files); `except_generators` drops the targets a named macro
    generated.
    """
    include: tuple
    exclude: tuple = ()
    rules_only: bool = False
    except_generators: tuple = ()


@dataclasses.dataclass(frozen=True)
class Check:
    """`frm` must not reach any of `to` (kind "path": through any number of
    dependencies; kind "direct": in one)."""
    name: str
    frm: str
    to: tuple
    kind: str = "path"
    exclude: tuple = ()
    rules_only: bool = False
    except_generators: tuple = ()

    def target_set(self, to):
        return Set(tuple(to), self.exclude, self.rules_only,
                   self.except_generators)

    def search(self, backend, to):
        """The path or direct dependencies from `frm` to `to`, or []."""
        source, target = Set((self.frm,)), self.target_set(to)
        if self.kind == "path":
            return backend.somepath(source, target)
        return backend.direct(source, target)


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
    return result


def strip_pattern(pattern):
    """The package a single-package or recursive pattern names."""
    return pattern.removesuffix("/...").removesuffix(":*")


def run(backend, out=sys.stdout):
    """Runs every check against `backend`; returns the number of failures."""
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
    if target_set.rules_only:
        expr = f"kind(rule, {expr})"
    if target_set.except_generators:
        generators = "|".join(target_set.except_generators)
        expr = (f"({expr}) except "
                f"attr(generator_function, \"^({generators})$\", {expr})")
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


def main():
    return 1 if run(Bazel()) else 0


if __name__ == "__main__":
    sys.exit(main())
