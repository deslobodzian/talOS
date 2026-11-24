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
  python); layers below node never depend on talOS/hardware, and talOS/node
  depends only on //talOS/hardware:config (timeouts and actuator ownership
  via talOS/configuration), never on //talOS/hardware:hardware or any other
  target in talOS/hardware/...;
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
  `//talOS/hardware:hardware`, `//talOS/devices:{access,access_hdr}`,
  `//talOS/events:{events,events_os,input_replay}`,
  `//talOS/telemetry:{telemetry,channels}`, `//talOS/teleop:teleop`,
  `//talOS/node_api:node_api`, `//talOS/bridge:node_lib`,
  `//talOS/introspection:{registry,reporter}`, `//talOS/launcher:launcher_lib`,
  or internal `//talOS/node:*` targets (anything in `//talOS/node:*` other than
  `//talOS/node:node`);
- structural Bazel visibility enforcement (RULES.md 2 / ARCH-2): privileged
  internal targets (`//talOS/node:{access,access_hdr,loop_port}`,
  `//talOS/devices:{access,access_hdr}`,
  `//talOS/rtms:{rtms,wake,process_liveness}`,
  `//talOS/memory:{shared_memory,residency}`,
  `//talOS/events:{events,events_os,input_replay}`,
  `//talOS/telemetry:{telemetry,channels}`, `//talOS/teleop:teleop`,
  `//talOS/node_api:node_api`, `//talOS/bridge:node_lib`,
  `//talOS/introspection:{registry,reporter}`, `//talOS/launcher:launcher_lib`,
  `//talOS/hardware:hardware`, `//talOS/ipc:ipc`) are never
  `//visibility:public`, and are not visible to robot/example/bench node
  packages (except `//talOS/hardware:hardware` and `//talOS/ipc:ipc` for
  `//bench/main_processor/swerve_module` which hosts the standalone
  `:send_target` CLI).

The C++ source scan (also run standalone via `--source-only` in
`tools/source_lints.sh`) walks every tracked file under `bench/`, `examples/`,
`tools/scaffold_testdata/`, and any other robot package outside `talOS/`,
`third_party/`, `controller_processor/`, `studio/` and `tools/` (skipping only
an explicit list of non-C++ basenames/suffixes, unless referenced via BUILD
`hdrs`/`textual_hdrs`, and failing closed on unreadable or non-UTF-8 files),
plus every `BUILD`, `BUILD.bazel`, and `.bzl` file under the same roots, and
refuses from the start (RULES.md 2):

- `#include` (or `#include_next`, quote or angle-bracket, after comment
  stripping, line-continuation splicing and path normalization) of
  `talOS/node/access.h`, `talOS/node/internal.h`, `talOS/devices/access.h`,
  `talOS/rtms/...`, `talOS/ipc/...`, `talOS/memory/...`,
  `talOS/hardware/messages.h`, `talOS/hardware/packet.h`,
  `talOS/hardware/request...`, `talOS/events/{event_loop_base,input_replay,realtime_event_loop,replay_event_loop,simulated_event_loop,wake_channel_wait}.h`,
  `talOS/events/os/...`, `talOS/telemetry/{channels,node}.h`,
  `talOS/teleop/node.h`, `talOS/node_api/...`, `talOS/bridge/node.h`,
  `talOS/introspection/{registry,reporter}.h`, non-C++ filenames
  (`NON_CPP_BASENAMES`, `NON_CPP_SUFFIXES`, and `.txt`, case-insensitive) that
  could hide headers from source discovery, and computed `#include MACRO`
  directives that cannot be verified statically;
- non-comment, non-string references to `NodeAccess`, `add_hardware_request`,
  `add_target_lane_publisher`, `add_internal_driver_station_subscription`,
  `mark_driver_station_reader`, `HardwareAccess`, or `note_live_target`,
  preprocessor token-pasting
  operators `##` and `%:%:` (which could otherwise assemble them), and C++
  trigraphs (`??[=/'()!<>-]` in code and comments, plus `??/` in string/char
  literals where it can escape quotes or splice lines under trigraph-enabling
  flags);
- Starlark string constants (and constant concatenations) in `BUILD`,
  `BUILD.bazel`, and `.bzl` files that carry those forbidden symbols, token
  pasting, C++ trigraphs, forbidden header paths, or `-include`/`-imacros`/
  `-trigraphs`/`-ftrigraphs`/`-fms-extensions`/`-fms-compatibility`/`-I`/
  `-iquote`/`-isystem`/`-cxx-isystem`/`-idirafter`/`--include-directory`/
  `-iwithprefix`/`-iwithsysroot` compiler flags (joined, separate, or via
  `-Wp,`/`-Xclang`).

It also refuses, in every tracked `BUILD`, `BUILD.bazel` and `.bzl` file
outside `talOS/simulation/mujoco/` (talOS/ and tools/ included), a string
constant (or constant concatenation, format or join) that is a `@mujoco`
label, a `load` from `//talOS/simulation/mujoco`, and a `repo_name` on
MODULE.bazel's mujoco `bazel_dep`: only that package may depend on MuJoCo
(docs/METHODOLOGY.md §13).

Structural enforcement (ARCH-2 / RULES.md 2): Bazel visibility (checked above
for every restricted target) prevents robot node code from depending on
privileged internal or transport targets, and `implementation_deps` on
`//talOS/node:node` and `//talOS/devices:devices` excludes their headers from
downstream robot-node compile sandboxes. The static source scan provides fast
defense-in-depth in `tools/source_lints.sh` without requiring Bazel.

ALLOWLIST holds any Bazel exceptions (empty since ARCH-2). NON_NODE_CLI_FILES
maps non-node CLI binaries
that sit beside a node package to the exact transport/wire headers they may
include (never exempting `NodeAccess`, `HardwareAccess`, `node/access.h`,
`node/internal.h`, `devices/access.h`, `add_hardware_request`,
`note_live_target`, token pasting, trigraphs, non-C++ includes or computed
includes). Both may only shrink
entry by entry: a stale entry fails the check, so it is deleted with the fix.
Prints each violation; exits 1 on a violation or a stale entry.
"""

import argparse
import ast
import dataclasses
import os
import posixpath
import re
import subprocess
import sys

try:  # under Bazel the workspace root is on sys.path
    from tools import bazel_query
except ModuleNotFoundError:  # python3 tools/<script>.py: tools/ is sys.path[0]
    import bazel_query

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

# "talOS/node drives no hardware": layers below node never depend on
# talOS/hardware; talOS/node reads only //talOS/hardware:config (timeouts and
# actuator ownership for loop tier) through talOS/configuration
# (ARCHITECTURE.md "Allowed dependencies").
HARDWARE = "talOS/hardware"
HARDWARE_CONFIG = "talOS/hardware:config"

# Robot code and what sits beside talOS; talOS never depends on them.
OUTSIDE_TALOS = ("bench", "controller_processor", "examples", "studio")

# Where robot, example and bench node packages live.
NODE_ROOTS = ("bench", "examples")
NODE_GENERATORS = ("talos_cc_node", "talos_py_node", "talos_cc_shared_node")

# Where non-testonly C++ nodes and node libraries outside talOS/ live for the
# RULES.md 2 direct-dependency check.
CC_NODE_DEP_ROOTS = ("bench", "examples", "tools/scaffold_testdata")
CC_NODE_GENERATORS = ("talos_cc_node", "talos_cc_shared_node")

# Privileged framework libraries outside the stack's transport layers: no
# node depends on one directly, and none is visible to a node package. Both
# FORBIDDEN_NODE_DEPS and RESTRICTED_INTERNAL_TARGETS list them.
_PRIVILEGED_FRAMEWORK_LIBRARIES = (
    "talOS/events:events",
    "talOS/events:events_os",
    "talOS/events:input_replay",
    "talOS/telemetry:telemetry",
    "talOS/telemetry:channels",
    "talOS/teleop:teleop",
    "talOS/node_api:node_api",
    "talOS/bridge:node_lib",
    "talOS/introspection:registry",
    "talOS/introspection:reporter",
    "talOS/launcher:launcher_lib",
    "talOS/simulation:schedules",
    "talOS/simulation:gateway_main",
    "talOS/simulation/mujoco:gateway_extension",
    "talOS/simulation/mujoco:plant",
)
FORBIDDEN_NODE_DEPS = (
    "talOS/rtms/...",
    "talOS/ipc/...",
    "talOS/memory/...",
    "talOS/hardware:hardware",
    "talOS/node:*",
    "talOS/devices:access",
    "talOS/devices:access_hdr",
    *_PRIVILEGED_FRAMEWORK_LIBRARIES,
)
FORBIDDEN_NODE_DEPS_EXCLUDE = ("talOS/node:node",)

# Targets that must never be //visibility:public and must not be visible to
# robot/example/bench node packages (structural RULES.md 2 enforcement).
RESTRICTED_INTERNAL_TARGETS = (
    "talOS/node:access",
    "talOS/node:access_hdr",
    "talOS/node:loop_port",
    "talOS/devices:access",
    "talOS/devices:access_hdr",
    "talOS/rtms:rtms",
    "talOS/rtms:wake",
    "talOS/rtms:process_liveness",
    "talOS/memory:shared_memory",
    "talOS/memory:residency",
    *_PRIVILEGED_FRAMEWORK_LIBRARIES,
    "talOS/vision/capture:avfoundation",
)
# Hardware wire and raw IPC targets: never //visibility:public, and not visible
# to any node package except WIRE_CLI_PACKAGES (which host standalone dev CLIs).
RESTRICTED_WIRE_TARGETS = (
    "talOS/hardware:hardware",
    "talOS/ipc:ipc",
)
WIRE_CLI_PACKAGES = frozenset({
    "bench/main_processor/swerve_module",
})

# What talOS/python may depend on directly outside itself.
PYTHON_GATEWAYS = ("talOS/node_api", "talOS/schema", "talOS/build",
                   "talOS/msgs")

# REQ-SAFE-35: studio/review is an offline log viewer and must never depend
# directly on talOS/rtms, talOS/ipc, talOS/protocol or talOS/bridge, nor
# transitively on RTMS segment/wake, IPC, UDP protocol, hardware UDP endpoint,
# shared_memory, bridge runtime or node/devices. Pure wire codecs
# (talOS/protocol:frame, talOS/hardware:codecs) stay allowed.
REVIEW_FORBIDDEN_DIRECT_DEPS = (
    "talOS/rtms/...",
    "talOS/ipc/...",
    "talOS/protocol/...",
    "talOS/bridge/...",
)
REVIEW_FORBIDDEN_TRANSITIVE_DEPS = (
    "talOS/rtms:rtms",
    "talOS/rtms:wake",
    "talOS/ipc/...",
    "talOS/protocol:protocol",
    "talOS/hardware:hardware",
    "talOS/memory:shared_memory",
    "talOS/bridge:node_lib",
    "talOS/bridge:hardware_node",
    "talOS/node/...",
    "talOS/devices/...",
)

# Generated targets another package may depend on: messages, and C++ nodes
# exported for talos.cpp_node (ARCHITECTURE.md "Allowed dependencies").
SHARED_GENERATORS = ("talos_messages", "talos_cc_shared_node")

# (from, to) -> reason. Must remain empty; any stale entry fails the check.
ALLOWLIST = {}

# C++ source bypass scan (RULES.md 2, REQ-SAFE-27).
# Every tracked C++ file under SCANNED_SOURCE_ROOTS is scanned (regardless of
# what macros its BUILD uses), as is any file outside EXCLUDED_SOURCE_PREFIXES
# (e.g. a standalone robot repository at the workspace root or a `<robot>/` dir).
SCANNED_SOURCE_ROOTS = (
    "bench/",
    "examples/",
    "tools/scaffold_testdata/",
)
EXCLUDED_SOURCE_PREFIXES = (
    "talOS/",
    "third_party/",
    "controller_processor/",
    "studio/",
    "tools/",
)
STARLARK_BASENAMES = frozenset({"BUILD", "BUILD.bazel"})
STARLARK_SUFFIXES = frozenset({".bzl"})

# Explicit non-C++ files skipped from the C++ source scan (fail closed: any
# suffix or extensionless filename not listed here — including .txt, .mm,
# .tlh, .tli, and arbitrary extensionless headers — IS scanned as C++).
# Files referenced via BUILD `hdrs` or `textual_hdrs` are scanned as C++
# even if their suffix appears here.
NON_CPP_BASENAMES = frozenset({
    "BUILD",
    "BUILD.bazel",
    "MODULE.bazel",
    "REPO.bazel",
    "WORKSPACE",
    "WORKSPACE.bazel",
    "README",
    "LICENSE",
    "COPYING",
    "AUTHORS",
    "NOTICE",
    ".gitignore",
    ".gitattributes",
    ".bazelrc",
    ".bazelignore",
    ".bazelversion",
    ".clang-format",
    ".clang-tidy",
    ".editorconfig",
})
NON_CPP_SUFFIXES = frozenset({
    ".md",
    ".py",
    ".pyi",
    ".toml",
    ".fbs",
    ".json",
    ".yaml",
    ".yml",
    ".sh",
    ".bzl",
    ".bazel",
    ".bazelrc",
    ".lock",
    ".png",
    ".svg",
    ".jpg",
    ".jpeg",
    ".gif",
    ".webp",
    ".ico",
    ".pdf",
    ".tlog",
})
NON_CPP_BASENAMES_LOWER = frozenset(b.lower() for b in NON_CPP_BASENAMES)
# Refused as `#include` targets even though `.txt` files under scanned roots
# are still scanned as C++ (prevents smuggling via cross-package/generated
# non-C++ files that discovery skips or does not reach).
FORBIDDEN_INCLUDE_SUFFIXES = NON_CPP_SUFFIXES | frozenset({".txt"})

INCLUDE_DIRECTIVE_RE = re.compile(
    r"^[ \t\v\f]*(?:#|%:)[ \t\v\f]*(include(?:_next)?|import)\b(.*)$")
HEADER_NAME_RE = re.compile(r'^(?:"([^"]*)"|<([^>]*)>)')
TOKEN_PASTE_RE = re.compile(r"##|%:%:")
TRIGRAPH_RE = re.compile(r"\?\?[=/'()!<>-]")
UCN_RE = re.compile(r"\\(?:u[0-9a-fA-F]{4}|U[0-9a-fA-F]{8})")
FORBIDDEN_COPT_FLAG_RE = re.compile(
    r"^(?:--?(?:include|imacros|f?trigraphs|fms-(?:extensions|compatibility)|"
    r"iquote|(?:cxx-)?isystem|idirafter|iwithprefix|iwithsysroot)|-I)"
)
FORBIDDEN_COPT_ADJACENT_FLAGS = frozenset({
    "-include", "--include",
    "-imacros", "--imacros",
    "-I",
    "-iquote", "--iquote",
    "-isystem", "--isystem",
    "-cxx-isystem", "--cxx-isystem",
    "-idirafter", "--idirafter",
    "--include-directory", "--include-directory-after",
    "-iwithprefix", "-iwithprefixbefore", "-iwithsysroot",
})
STARLARK_COPT_JOINED_RE = re.compile(r"^--?(?:include|imacros)=?(.+)$")
STARLARK_PATH_TOKEN_RE = re.compile(r"[^\s\"'<>(),;=]+")

FORBIDDEN_INCLUDE_EXACT = frozenset({
    "talOS/node/access.h",
    "talOS/node/internal.h",
    "talOS/devices/access.h",
    "talOS/hardware/config_wire.h",
    "talOS/hardware/endpoint.h",
    "talOS/hardware/gateway.h",
    "talOS/hardware/messages.h",
    "talOS/hardware/packet.h",
    "talOS/hardware/sim_backend.h",
    "talOS/hardware/sim_physics.h",
    "talOS/events/event_loop_base.h",
    "talOS/events/handles.h",
    "talOS/events/input_replay.h",
    "talOS/events/realtime_event_loop.h",
    "talOS/events/replay_event_loop.h",
    "talOS/events/simulated_event_loop.h",
    "talOS/events/wake_channel_wait.h",
    "talOS/simulation/schedules.h",
    "talOS/simulation/gateway_main.h",
    "talOS/telemetry/channels.h",
    "talOS/telemetry/node.h",
    "talOS/teleop/node.h",
    "talOS/bridge/node.h",
    "talOS/introspection/registry.h",
    "talOS/introspection/reporter.h",
})
FORBIDDEN_INCLUDE_PREFIXES = (
    "talOS/rtms/",
    "talOS/ipc/",
    "talOS/memory/",
    "talOS/hardware/request",
    "talOS/events/os/",
    "talOS/node_api/",
)
NEVER_EXEMPT_INCLUDES = frozenset({
    "talOS/node/access.h",
    "talOS/node/internal.h",
    "talOS/devices/access.h",
})
FORBIDDEN_INCLUDE_EXACT_LOWER = frozenset(
    h.lower() for h in FORBIDDEN_INCLUDE_EXACT)
FORBIDDEN_INCLUDE_PREFIXES_LOWER = tuple(
    p.lower() for p in FORBIDDEN_INCLUDE_PREFIXES)
NEVER_EXEMPT_INCLUDES_LOWER = frozenset(
    h.lower() for h in NEVER_EXEMPT_INCLUDES)
FORBIDDEN_SYMBOLS = (
    ("NodeAccess", re.compile(r"\bNodeAccess\b")),
    ("add_hardware_request", re.compile(r"\badd_hardware_request\b")),
    ("add_target_lane_publisher", re.compile(r"\badd_target_lane_publisher\b")),
    ("add_internal_driver_station_subscription",
     re.compile(r"\badd_internal_driver_station_subscription\b")),
    ("mark_driver_station_reader",
     re.compile(r"\bmark_driver_station_reader\b")),
    ("HardwareAccess", re.compile(r"\bHardwareAccess\b")),
    ("note_live_target", re.compile(r"\bnote_live_target\b")),
)

# Standalone non-node CLI binaries sitting beside a node package (not linked
# by any talos_cc_node or node library). Each entry names the exact normalized
# transport/wire headers that CLI may include; every header is checked
# individually for staleness so the allowlist only shrinks header by header.
# NodeAccess, HardwareAccess, node/access.h, node/internal.h, devices/access.h,
# add_hardware_request, note_live_target, token pasting, trigraphs, non-C++
# includes, computed includes and syntax errors are never exempted.
NON_NODE_CLI_FILES = {
    "bench/main_processor/swerve_module/send_target.cc": frozenset({
        "talOS/hardware/packet.h",
        "talOS/ipc/publisher.h",
        "talOS/ipc/subscriber.h",
    }),
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
        exclude = ()
        if layer == "node":
            above.append(HARDWARE)
            exclude = (HARDWARE_CONFIG,)
        elif layer != "devices":
            above.append(HARDWARE)
        for package in packages:
            result.append(Check(f"{layer} layer below {', '.join(above)}",
                                recursive(package),
                                tuple(recursive(p) for p in above),
                                exclude=exclude))
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
    # REQ-SAFE-35: studio/review is an offline log viewer.
    result.append(Check(
        "studio/review has no direct dependency on talOS/rtms, talOS/ipc, "
        "talOS/protocol or talOS/bridge (REQ-SAFE-35)",
        "studio/review/...",
        REVIEW_FORBIDDEN_DIRECT_DEPS,
        kind="direct",
        source=Set(("studio/review/...",), rules_only=True,
                   except_testonly=True),
    ))
    result.append(Check(
        "studio/review has no transitive dependency on RTMS, IPC, UDP "
        "protocol, hardware UDP endpoint, shared_memory, bridge runtime or "
        "node/devices (REQ-SAFE-35)",
        "studio/review/...",
        REVIEW_FORBIDDEN_TRANSITIVE_DEPS,
        kind="path",
        source=Set(("studio/review/...",), rules_only=True,
                   except_testonly=True),
    ))
    return result


def strip_pattern(pattern):
    """The package a single-package or recursive pattern names."""
    return pattern.removesuffix("/...").removesuffix(":*")


def run(backend, out=sys.stdout):
    """Runs every Bazel check against `backend`; returns the failure count."""
    failures = 0
    used = set()
    node_pkgs = backend.node_packages()
    all_checks = checks(node_pkgs)
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
    # Structural Bazel visibility checks (RULES.md 2 / ARCH-2): none of the
    # restricted targets may be //visibility:public, and node packages may not
    # see targets restricted to framework/CLI packages.
    all_restricted = RESTRICTED_INTERNAL_TARGETS + RESTRICTED_WIRE_TARGETS
    public = backend.public_targets(all_restricted)
    if public:
        failures += 1
        print("VIOLATION restricted internal/transport targets must not be "
              "//visibility:public (RULES.md 2):", file=out)
        for label in public:
            print(f"  {label}", file=out)
    for pkg, target in sorted(backend.node_sample_targets().items()):
        forbidden = list(RESTRICTED_INTERNAL_TARGETS)
        if pkg not in WIRE_CLI_PACKAGES:
            forbidden.extend(RESTRICTED_WIRE_TARGETS)
        visible = backend.visible_to(target, forbidden)
        if visible:
            failures += 1
            print(f"VIOLATION restricted targets visible to node package "
                  f"//{pkg} ({target}) (RULES.md 2):", file=out)
            for label in visible:
                print(f"  {label}", file=out)
    visibility_checks = 1 + len(node_pkgs)
    if failures:
        print(f"{failures} layer check(s) failed (docs/DESIGN.md §3)",
              file=out)
    else:
        print(f"{len(all_checks) + visibility_checks} layer checks passed, "
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
        # other edge evaluated (docs/DEVELOPMENT.md §5). Fail closed if any
        # load/eval error occurs outside allowed external `@` repositories.
        proc = subprocess.run(
            ["bazel", "query", "--keep_going", "--noimplicit_deps",
             "--noshow_progress", f"--output={output}", expr],
            capture_output=True, text=True, check=False)
        if proc.returncode not in (0, 3):
            sys.stderr.write(proc.stderr)
            raise RuntimeError(f"bazel query failed ({proc.returncode}): "
                               f"{expr}")
        if proc.returncode == 3 and not bazel_query.exit_3_is_external_only(
                proc.stderr):
            sys.stderr.write(proc.stderr)
            raise RuntimeError(
                f"bazel query exited 3 with non-external package error: "
                f"{expr}")
        return [line for line in proc.stdout.splitlines() if line]

    def node_sample_targets(self):
        """One generated node rule label per node package under NODE_ROOTS."""
        roots = " + ".join(f"//{r}/..." for r in NODE_ROOTS)
        generators = "|".join(NODE_GENERATORS)
        labels = sorted(self.query(
            f"attr(generator_function, \"^({generators})$\", {roots})",
            output="label"))
        samples = {}
        for label in labels:
            pkg = label.removeprefix("//").split(":", 1)[0]
            samples.setdefault(pkg, label)
        return samples

    def node_packages(self):
        return sorted(self.node_sample_targets().keys())

    def public_targets(self, targets):
        expr = " + ".join(f"//{t}" for t in targets)
        return sorted(self.query(
            f'attr(visibility, "//visibility:public", {expr})'))

    def visible_to(self, target_label, targets):
        expr = " + ".join(f"//{t}" for t in targets)
        return sorted(self.query(f"visible({target_label}, {expr})"))

    def somepath(self, frm, to):
        return self.query(f"somepath({render(frm)}, {render(to)})")

    def direct(self, frm, to):
        source = render(frm)
        return self.query(f"(deps({source}, 1) except ({source})) "
                          f"intersect ({render(to)})")


def _is_ascii_digit(ch):
    return "0" <= ch <= "9"


def _is_ident_char(ch):
    return ch.isalnum() or ch == "_"


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
            while j < n and text[j] in (" ", "\t", "\v", "\f"):
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

    def quoted_literal(start, quote, kind):
        """Emits the `quote`-delimited literal at `start`, delimiters kept and
        interior blanked in `no_strings`, and returns the index after it. It
        ends at its closing quote or an unescaped newline. Refuses `??/`
        inside it: phase-1 `??/` -> `\\` can escape the closing quote or
        splice lines under trigraph-enabling flags and desync the lexer."""
        emit(quote, quote, spliced_lines[start])
        i = start + 1
        while i < n:
            c = src[i]
            if c == "\\":
                emit(c, " ", spliced_lines[i])
                i += 1
                if i < n and src[i] != "\n":
                    emit(src[i], " ", spliced_lines[i])
                    i += 1
                continue
            if c == quote:
                emit(quote, quote, spliced_lines[i])
                i += 1
                break
            if c == "\n":
                emit("\n", "\n", spliced_lines[i])
                i += 1
                break
            emit(c, " ", spliced_lines[i])
            i += 1
        pos = src.find("??/", start, i)
        while pos != -1:
            errors.append((
                spliced_lines[pos],
                "trigraph",
                f"uses C++ trigraph `??/` in {kind} "
                "(can escape quotes or splice lines under -trigraphs)",
            ))
            pos = src.find("??/", pos + 3, i)
        return i

    i = 0
    while i < n:
        ch = src[i]
        nxt = src[i + 1] if i + 1 < n else ""
        # Line comment: replace with space, leave '\n' intact.
        if ch == "/" and nxt == "/":
            emit(" ", " ", spliced_lines[i])
            comment_start = i + 2
            i += 2
            while i < n and src[i] != "\n":
                i += 1
            for tm in TRIGRAPH_RE.finditer(src[comment_start:i]):
                errors.append((
                    spliced_lines[comment_start + tm.start()],
                    "trigraph",
                    f"uses C++ trigraph `{tm.group(0)}` in comment "
                    f"(can splice lines or close comments under -trigraphs)",
                ))
            continue
        # Block comment: C++ translation phase 3 replaces the entire comment
        # (including newlines inside it) with a single space.
        if ch == "/" and nxt == "*":
            end = src.find("*/", i + 2)
            if end == -1:
                errors.append((spliced_lines[i], "unterminated-comment",
                               "unterminated block comment"))
                break
            for tm in TRIGRAPH_RE.finditer(src[i + 2:end]):
                errors.append((
                    spliced_lines[i + 2 + tm.start()],
                    "trigraph",
                    f"uses C++ trigraph `{tm.group(0)}` in comment "
                    f"(can splice lines or close comments under -trigraphs)",
                ))
            emit(" ", " ", spliced_lines[i])
            i = end + 2
            continue
        # Identifier, or raw string literal: [u8|u|U|L]R"delim(...)delim".
        if ch == "_" or ch.isalpha():
            j = i + 1
            while j < n and _is_ident_char(src[j]):
                j += 1
            ident = src[i:j]
            if (ident in ("R", "u8R", "uR", "UR", "LR") and
                    j < n and src[j] == '"'):
                paren = src.find("(", j + 1)
                delim = src[j + 1:paren] if paren != -1 else ""
                if (paren == -1 or len(delim) > 16 or
                        any(c in (" ", "\t", "\v", "\f", "\r", "\n", "\\", ")")
                            for c in delim)):
                    errors.append((spliced_lines[i], "unterminated-string",
                                   "malformed raw string literal"))
                    break
                for tm in TRIGRAPH_RE.finditer(delim):
                    errors.append((
                        spliced_lines[j + 1 + tm.start()],
                        "trigraph",
                        f"uses C++ trigraph `{tm.group(0)}` in raw string "
                        f"delimiter",
                    ))
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
            for k in range(i, j):
                emit(src[k], src[k], spliced_lines[k])
            i = j
            continue
        # pp-number (C++14+ [lex.ppnumber]): starts with an ASCII digit or
        # '.' followed by an ASCII digit. Only here is '\'' a digit separator,
        # so tokens like `case'"':` or `return'"';` still lex `'"'` as a char
        # literal rather than opening a fake string.
        if _is_ascii_digit(ch) or (ch == "." and _is_ascii_digit(nxt)):
            if ch == ".":
                emit(ch, ch, spliced_lines[i])
                i += 1
            emit(src[i], src[i], spliced_lines[i])
            i += 1
            while i < n:
                c = src[i]
                c_next = src[i + 1] if i + 1 < n else ""
                if c == "'" and c_next and _is_ident_char(c_next):
                    emit(c, c, spliced_lines[i])
                    emit(c_next, c_next, spliced_lines[i + 1])
                    i += 2
                    continue
                if c in ("e", "E", "p", "P") and c_next in ("+", "-"):
                    emit(c, c, spliced_lines[i])
                    emit(c_next, c_next, spliced_lines[i + 1])
                    i += 2
                    continue
                if _is_ident_char(c) or c == ".":
                    emit(c, c, spliced_lines[i])
                    i += 1
                    continue
                break
            continue
        # Normal string literal: preserve in `with_strings` (for #include),
        # blank interior in `no_strings`. Character literal likewise
        # (pp-number digit separators were already consumed).
        if ch == '"':
            i = quoted_literal(i, '"', "string literal")
            continue
        if ch == "'":
            i = quoted_literal(i, "'", "character literal")
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
    marker = "/talos/"
    lower = norm.lower()
    if marker in lower:
        norm = norm[lower.rindex(marker) + 1:]
    return norm


def _is_forbidden_include(norm):
    lower = norm.lower()
    if lower in FORBIDDEN_INCLUDE_EXACT_LOWER:
        return True
    return lower.startswith(FORBIDDEN_INCLUDE_PREFIXES_LOWER)


def _is_non_cpp_include(norm):
    """True when an `#include` target has a non-C++ basename or suffix."""
    base = posixpath.basename(norm)
    if base.lower() in NON_CPP_BASENAMES_LOWER:
        return True
    return posixpath.splitext(base)[1].lower() in FORBIDDEN_INCLUDE_SUFFIXES


def scan_cpp_text(path, text):
    """Returns a list of SourceViolation found in one C++ source/header."""
    violations = []
    with_strings, no_strings, line_map, errors = _strip_cpp_comments(text)
    for lineno, kind, detail in errors:
        violations.append(SourceViolation(path, lineno, kind, detail))

    # Check every logical line for #include / #include_next directives.
    # Split only on '\n' (not str.splitlines(), which splits on \v and \f —
    # horizontal whitespace in GCC/Clang preprocessor directives).
    offset = 0
    total_len = len(with_strings)
    while offset < total_len:
        nl = with_strings.find("\n", offset)
        end = total_len if nl == -1 else nl + 1
        line_text = with_strings[offset:end].rstrip("\r\n")
        m = INCLUDE_DIRECTIVE_RE.match(line_text)
        if m:
            lineno = line_map[offset + m.start()] if line_map else 1
            directive = m.group(1)
            rest = m.group(2).strip(" \t\v\f")
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
                elif _is_non_cpp_include(norm):
                    violations.append(SourceViolation(
                        path, lineno, "non-cpp-include",
                        f"#{directive} \"{raw_header}\" (non-C++ filename "
                        f"cannot be included; bypasses source discovery)"))
        offset = end

    # Check non-comment, non-string code for forbidden internal symbols.
    for symbol, pattern in FORBIDDEN_SYMBOLS:
        for m in pattern.finditer(no_strings):
            lineno = line_map[m.start()] if line_map else 1
            violations.append(SourceViolation(
                path, lineno, "forbidden-symbol",
                f"references forbidden framework internal `{symbol}`"))

    # Refuse preprocessor token pasting (## and %:%:) outside comments and
    # strings so macros cannot assemble NodeAccess/HardwareAccess identifiers.
    for m in TOKEN_PASTE_RE.finditer(no_strings):
        lineno = line_map[m.start()] if line_map else 1
        violations.append(SourceViolation(
            path, lineno, "token-paste",
            f"uses preprocessor token pasting `{m.group(0)}` (can assemble "
            f"forbidden framework internals)"))

    # Refuse C++ trigraphs in code (comments and `??/` in literals were
    # checked in `_strip_cpp_comments`).
    for m in TRIGRAPH_RE.finditer(no_strings):
        lineno = line_map[m.start()] if line_map else 1
        violations.append(SourceViolation(
            path, lineno, "trigraph",
            f"uses C++ trigraph `{m.group(0)}` (can assemble directives, "
            f"token pasting, or line splices under -trigraphs)"))

    # Refuse C++ Universal Character Names (\uXXXX / \UXXXXXXXX) outside
    # comments and string/char literals, where they can spell forbidden
    # identifiers such as `Node\u0041ccess`.
    for m in UCN_RE.finditer(no_strings):
        lineno = line_map[m.start()] if line_map else 1
        violations.append(SourceViolation(
            path, lineno, "ucn-identifier",
            f"uses C++ universal character name `{m.group(0)}` in code "
            f"(can spell forbidden framework internals)"))

    violations.sort(key=lambda v: (v.line, v.kind, v.detail))
    return violations


def _is_in_scanned_tree(path):
    """True when `path` is under bench/, examples/, tools/scaffold_testdata/,
    or another robot tree outside talOS/ and repo tooling."""
    if path.startswith(SCANNED_SOURCE_ROOTS):
        return True
    if path.startswith(EXCLUDED_SOURCE_PREFIXES):
        return False
    return True


def _glob_to_regex(pattern):
    """Compiles a Bazel glob pattern (`*`, `**`, `?`) to a regex."""
    parts = ["^"]
    i = 0
    n = len(pattern)
    while i < n:
        if pattern[i:i + 3] == "**/":
            parts.append("(?:.*/)?")
            i += 3
        elif pattern[i:i + 2] == "**":
            parts.append(".*")
            i += 2
        elif pattern[i] == "*":
            parts.append("[^/]*")
            i += 1
        elif pattern[i] == "?":
            parts.append("[^/]")
            i += 1
        else:
            parts.append(re.escape(pattern[i]))
            i += 1
    parts.append("$")
    return re.compile("".join(parts))


def _resolve_build_label(pkg, raw):
    """Resolves a Bazel file label or package-relative path to a repo path."""
    if not raw or raw.startswith("@"):
        return None
    if raw.startswith("//"):
        body = raw[2:]
        if ":" in body:
            target_pkg, name = body.split(":", 1)
            candidate = f"{target_pkg}/{name}" if target_pkg else name
        else:
            candidate = body
    else:
        rel = raw[1:] if raw.startswith(":") else raw
        candidate = posixpath.join(pkg, rel) if pkg else rel
    norm = posixpath.normpath(candidate)
    if norm in ("", ".") or norm.startswith("../"):
        return None
    return norm


def _is_starlark_file(path):
    base = os.path.basename(path)
    if base in STARLARK_BASENAMES:
        return True
    return os.path.splitext(base)[1].lower() in STARLARK_SUFFIXES


def _is_non_cpp_file(path):
    base = os.path.basename(path)
    if base in NON_CPP_BASENAMES:
        return True
    return os.path.splitext(base)[1].lower() in NON_CPP_SUFFIXES


def _collect_starlark_bindings(tree):
    bindings = {}
    for stmt in tree.body:
        if isinstance(stmt, ast.Assign):
            for target in stmt.targets:
                if isinstance(target, ast.Name):
                    bindings[target.id] = stmt.value
    return bindings


def _eval_starlark_str(node, bindings, seen=frozenset()):
    """Evaluates a Starlark string constant, constant `+` concat, `%` format,
    `.format(...)`, or `.join(...)`."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.Name) and node.id in bindings:
        if node.id not in seen:
            return _eval_starlark_str(bindings[node.id], bindings,
                                      seen | {node.id})
        return None
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = _eval_starlark_str(node.left, bindings, seen)
        right = _eval_starlark_str(node.right, bindings, seen)
        if left is not None and right is not None:
            return left + right
        return None
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Mod):
        fmt = _eval_starlark_str(node.left, bindings, seen)
        if fmt is None:
            return None
        rhs = node.right
        if isinstance(rhs, ast.Name) and rhs.id in bindings and rhs.id not in seen:
            rhs = bindings[rhs.id]
            seen = seen | {node.right.id}
        if isinstance(rhs, ast.Tuple):
            items = [_eval_starlark_str(e, bindings, seen) for e in rhs.elts]
            if any(v is None for v in items):
                return None
            arg = tuple(items)
        elif isinstance(rhs, ast.Dict):
            arg = {}
            for k, v in zip(rhs.keys, rhs.values):
                if k is None:
                    return None
                ks = _eval_starlark_str(k, bindings, seen)
                vs = _eval_starlark_str(v, bindings, seen)
                if ks is None or vs is None:
                    return None
                arg[ks] = vs
        else:
            arg = _eval_starlark_str(rhs, bindings, seen)
            if arg is None:
                return None
        try:
            return fmt % arg
        except (TypeError, ValueError, KeyError):
            return None
    if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and
            node.func.attr == "format"):
        fmt = _eval_starlark_str(node.func.value, bindings, seen)
        if fmt is None:
            return None
        pos = []
        for a in node.args:
            val = _eval_starlark_str(a, bindings, seen)
            if val is None:
                return None
            pos.append(val)
        kw = {}
        for k in node.keywords:
            if k.arg is None:
                return None
            val = _eval_starlark_str(k.value, bindings, seen)
            if val is None:
                return None
            kw[k.arg] = val
        try:
            return fmt.format(*pos, **kw)
        except (ValueError, KeyError, IndexError):
            return None
    if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and
            node.func.attr == "join" and len(node.args) == 1 and
            not node.keywords and
            isinstance(node.args[0], (ast.List, ast.Tuple))):
        sep = _eval_starlark_str(node.func.value, bindings, seen)
        if sep is None:
            return None
        parts = []
        for elt in node.args[0].elts:
            val = _eval_starlark_str(elt, bindings, seen)
            if val is None:
                return None
            parts.append(val)
        return sep.join(parts)
    return None


def _extract_starlark_header_candidates(value):
    """Yields header-path candidates embedded in a Starlark string."""
    s_slash = value.replace("\\", "/")
    for tok in STARLARK_PATH_TOKEN_RE.findall(s_slash):
        yield tok
        m = STARLARK_COPT_JOINED_RE.match(tok)
        if m:
            yield m.group(1)
        if ":" in tok:
            pkg, name = tok.rsplit(":", 1)
            if "." in name or "/" in name:
                yield f"{pkg}/{name}"


def scan_starlark_ast(path, tree):
    """Refuses BUILD / `.bzl` string constants carrying bypass symbols,
    token pasting, C++ trigraphs, forbidden header paths, or forbidden
    `-include`/`-imacros`/`-trigraphs`/`-ftrigraphs`/include-path compiler
    flags."""
    bindings = _collect_starlark_bindings(tree)
    violations = []
    seen_keys = set()
    reported_flag_nodes = set()

    def add(lineno, kind, detail, header=""):
        key = (lineno, kind, detail)
        if key not in seen_keys:
            seen_keys.add(key)
            violations.append(SourceViolation(path, lineno, kind, detail,
                                              header=header))

    # First detect adjacent flag+arg list/tuple pairs like `["-include", "foo"]`.
    for node in ast.walk(tree):
        if isinstance(node, (ast.List, ast.Tuple)):
            elts = node.elts
            for idx in range(len(elts) - 1):
                flag = _eval_starlark_str(elts[idx], bindings)
                if flag in FORBIDDEN_COPT_ADJACENT_FLAGS:
                    arg = _eval_starlark_str(elts[idx + 1], bindings)
                    if arg is not None:
                        lineno = getattr(elts[idx], "lineno", 1)
                        add(lineno, "starlark-forbidden-copt",
                            f"Starlark list uses forbidden compiler flag "
                            f"`{flag}` with `{arg}`")
                        reported_flag_nodes.add(id(elts[idx]))

    # Inspect every string constant (and constant string concatenation/join).
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Constant, ast.BinOp, ast.Call)):
            continue
        val = _eval_starlark_str(node, bindings)
        if val is None:
            continue
        lineno = getattr(node, "lineno", 1)

        for symbol, pattern in FORBIDDEN_SYMBOLS:
            if pattern.search(val):
                add(lineno, "starlark-forbidden-symbol",
                    f"Starlark string references forbidden framework internal "
                    f"`{symbol}`")

        for m in TOKEN_PASTE_RE.finditer(val):
            add(lineno, "starlark-token-paste",
                f"Starlark string uses preprocessor token pasting "
                f"`{m.group(0)}` (can assemble forbidden framework internals)")

        for m in TRIGRAPH_RE.finditer(val):
            add(lineno, "starlark-trigraph",
                f"Starlark string uses C++ trigraph `{m.group(0)}` (can "
                f"assemble token pasting or directives in compiler defines)")

        if id(node) not in reported_flag_nodes:
            for tok in re.findall(r"[^\s,]+", val):
                if FORBIDDEN_COPT_FLAG_RE.match(tok):
                    add(lineno, "starlark-forbidden-copt",
                        f"Starlark string uses forbidden compiler flag `{tok}`")

        for cand in _extract_starlark_header_candidates(val):
            norm = _normalize_include(cand)
            if _is_forbidden_include(norm):
                add(lineno, "starlark-forbidden-include",
                    f"Starlark string references forbidden header `{norm}`",
                    header=norm)

    violations.sort(key=lambda v: (v.line, v.kind, v.detail))
    return violations


def _extract_build_referenced_files(build_path, tree, tracked_set, pkg_files):
    """Extracts tracked files referenced in BUILD `hdrs` or `textual_hdrs`
    (expanding same-package `filegroup`s), so even a file whose suffix is on
    `NON_CPP_SUFFIXES` is scanned as C++ if explicitly included as a header."""
    pkg = os.path.dirname(build_path)
    bindings = _collect_starlark_bindings(tree)

    def extract_strings(node, seen=frozenset()):
        val = _eval_starlark_str(node, bindings, seen)
        if val is not None:
            yield val
            return
        if isinstance(node, ast.Name) and node.id in bindings:
            if node.id not in seen:
                yield from extract_strings(bindings[node.id], seen | {node.id})
            return
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and
                node.func.id == "glob"):
            inc_patterns = []
            exc_patterns = []
            if node.args:
                inc_patterns.extend(extract_strings(node.args[0], seen))
            for kw in node.keywords:
                if kw.arg == "include":
                    inc_patterns.extend(extract_strings(kw.value, seen))
                elif kw.arg == "exclude":
                    exc_patterns.extend(extract_strings(kw.value, seen))
            inc_res = [_glob_to_regex(p) for p in inc_patterns]
            exc_res = [_glob_to_regex(p) for p in exc_patterns]
            for rel in pkg_files:
                if (any(r.match(rel) for r in inc_res) and
                        not any(r.match(rel) for r in exc_res)):
                    yield rel
            return
        for child in ast.iter_child_nodes(node):
            yield from extract_strings(child, seen)

    # Index same-package filegroups so `textual_hdrs = [":fg"]` expands `fg`.
    filegroups = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and
                node.func.id == "filegroup"):
            fg_name = None
            fg_srcs = []
            for kw in node.keywords:
                if kw.arg == "name":
                    fg_name = _eval_starlark_str(kw.value, bindings)
                elif kw.arg == "srcs":
                    fg_srcs.extend(extract_strings(kw.value))
            if fg_name:
                filegroups[fg_name] = fg_srcs

    def resolve_with_filegroups(raw, seen_fg=frozenset()):
        candidate = _resolve_build_label(pkg, raw)
        if not candidate:
            return
        if candidate in tracked_set:
            yield candidate
            return
        # Check same-package filegroup target name (`:fg`, `fg`, `//pkg:fg`).
        fg_key = None
        if raw.startswith(":"):
            fg_key = raw[1:]
        elif raw.startswith("//"):
            body = raw[2:]
            if ":" in body:
                target_pkg, target_name = body.split(":", 1)
                if target_pkg == pkg:
                    fg_key = target_name
        elif "/" not in raw:
            fg_key = raw
        if fg_key and fg_key in filegroups and fg_key not in seen_fg:
            for src in filegroups[fg_key]:
                yield from resolve_with_filegroups(src, seen_fg | {fg_key})

    found = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        for kw in node.keywords:
            if kw.arg not in ("hdrs", "textual_hdrs"):
                continue
            for raw in extract_strings(kw.value):
                for candidate in resolve_with_filegroups(raw):
                    if not _is_in_scanned_tree(candidate):
                        continue
                    if _is_starlark_file(candidate):
                        continue
                    found.add(candidate)
    return found


# The MuJoCo dependency (docs/METHODOLOGY.md §13): only packages under
# MUJOCO_PACKAGE may name `@mujoco`, so no node and not the RoboRIO program
# links it. Every tracked BUILD, BUILD.bazel and `.bzl` file elsewhere is
# scanned, talOS/ and tools/ included.
MUJOCO_PACKAGE = "talOS/simulation/mujoco/"
MUJOCO_LABEL_RE = re.compile(r"^@{1,2}mujoco\+?(?://|:|$)")
MUJOCO_LOAD_RE = re.compile(r"^(?:@{1,2}talos\+?)?//talOS/simulation/mujoco(?:[/:]|$)")


def _statement_context(stmt):
    """Names the top-level statement a finding sits in: a rule or macro call
    with its `name`, a `def`, or an assignment."""
    if isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call):
        call = stmt.value
        kind = call.func.id if isinstance(call.func, ast.Name) else "call"
        for kw in call.keywords:
            if kw.arg == "name" and isinstance(kw.value, ast.Constant):
                return f'{kind}(name = "{kw.value.value}")'
        return kind
    if isinstance(stmt, ast.FunctionDef):
        return f"def {stmt.name}"
    if isinstance(stmt, ast.Assign) and isinstance(stmt.targets[0], ast.Name):
        return f"{stmt.targets[0].id} = ..."
    return "top level"


def scan_mujoco_labels(path, tree):
    """Refuses a `@mujoco` label, or a load from the MuJoCo package, in a
    Starlark file outside MUJOCO_PACKAGE. Returns `(line, detail)` pairs."""
    bindings = _collect_starlark_bindings(tree)
    found = {}
    for stmt in tree.body:
        context = _statement_context(stmt)
        if (isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call) and
                isinstance(stmt.value.func, ast.Name) and
                stmt.value.func.id == "load" and stmt.value.args):
            label = _eval_starlark_str(stmt.value.args[0], bindings)
            if label is None or MUJOCO_LOAD_RE.match(label):
                found.setdefault(stmt.lineno, (
                    f"load from the MuJoCo package ({label}) outside "
                    f"{MUJOCO_PACKAGE}"))
            continue
        for node in ast.walk(stmt):
            if not isinstance(node, (ast.Constant, ast.BinOp, ast.Call)):
                continue
            val = _eval_starlark_str(node, bindings)
            if val is not None and MUJOCO_LABEL_RE.match(val.strip()):
                found.setdefault(getattr(node, "lineno", stmt.lineno), (
                    f"{context} names `{val.strip()}`; only {MUJOCO_PACKAGE} "
                    f"may depend on MuJoCo"))
    return sorted(found.items())


def run_mujoco_checks(files, read, out=sys.stdout):
    """Runs scan_mujoco_labels over every tracked Starlark file outside
    MUJOCO_PACKAGE. Returns the number of failures; fails closed on a file
    it cannot read or parse."""
    failures = 0
    scanned = 0
    for path in sorted(files):
        if path == "MODULE.bazel":
            # A repo_name would give the dependency a label the scan misses.
            for node in ast.walk(ast.parse(read(path), filename=path)):
                if (isinstance(node, ast.Call) and
                        getattr(node.func, "id", "") == "bazel_dep"):
                    kw = {k.arg: k.value for k in node.keywords}
                    name = kw.get("name")
                    repo = kw.get("repo_name")
                    if (isinstance(name, ast.Constant) and
                            name.value == "mujoco" and repo is not None and
                            getattr(repo, "value", None) != "mujoco"):
                        failures += 1
                        print(f"VIOLATION {path}:{node.lineno}: mujoco's "
                              f"bazel_dep sets a repo_name; the @mujoco "
                              f"check would not see its labels", file=out)
            continue
        if path.startswith(MUJOCO_PACKAGE) or not _is_starlark_file(path):
            continue
        scanned += 1
        try:
            tree = ast.parse(read(path), filename=path)
        except (OSError, UnicodeDecodeError, SyntaxError) as exc:
            failures += 1
            print(f"VIOLATION {path}: cannot scan for @mujoco: {exc} "
                  f"(fail closed)", file=out)
            continue
        for line, detail in scan_mujoco_labels(path, tree):
            failures += 1
            print(f"VIOLATION {path}:{line}: {detail}", file=out)
    if not failures:
        print(f"MuJoCo dependency check passed ({scanned} Starlark file(s) "
              f"outside {MUJOCO_PACKAGE})", file=out)
    return failures


def discover_scanned_files(files, read):
    """Discovers every tracked source file and Starlark (`BUILD`, `BUILD.bazel`,
    `.bzl`) file under `bench/`, `examples/`, `tools/scaffold_testdata/`, and
    any other robot tree outside `talOS/`.

    Returns `(cpp_files, starlark_trees, read_errors)` where:
    - `cpp_files`: sorted list of files to scan as C++ (every tracked file
      not on `NON_CPP_BASENAMES`/`NON_CPP_SUFFIXES`, plus any file referenced
      via BUILD `hdrs`/`textual_hdrs`);
    - `starlark_trees`: sorted list of `(path, ast_tree)` pairs for every
      `BUILD`, `BUILD.bazel`, and `.bzl` file under the scanned roots;
    - `read_errors`: list of `(path, detail)` for any Starlark file that could
      not be read, decoded as UTF-8, or parsed (fail closed).
    """
    tracked_set = set(files)
    by_pkg = {}
    for path in files:
        if not _is_in_scanned_tree(path):
            continue
        # Record every file under each ancestor directory so package globs see
        # subdirectory files (e.g. `glob(["inc/*"])`).
        parts = path.split("/")
        for depth in range(len(parts)):
            pkg = "/".join(parts[:depth])
            rel = "/".join(parts[depth:])
            by_pkg.setdefault(pkg, []).append(rel)

    cpp_files = set()
    starlark_trees = []
    read_errors = []
    for path in sorted(files):
        if not _is_in_scanned_tree(path):
            continue
        if _is_starlark_file(path):
            try:
                content = read(path)
            except (OSError, UnicodeDecodeError) as exc:
                read_errors.append((path, f"cannot read Starlark file: {exc}"))
                continue
            try:
                tree = ast.parse(content, filename=path)
            except SyntaxError as exc:
                read_errors.append((path, f"cannot parse Starlark file: {exc}"))
                continue
            starlark_trees.append((path, tree))
            if os.path.basename(path) in STARLARK_BASENAMES:
                pkg = os.path.dirname(path)
                cpp_files.update(_extract_build_referenced_files(
                    path, tree, tracked_set, by_pkg.get(pkg, ())))
            continue
        if not _is_non_cpp_file(path):
            cpp_files.add(path)
    return sorted(cpp_files), starlark_trees, read_errors


def _is_exempt_cli_violation(violation, exempt_cli_files):
    allowed_headers = exempt_cli_files.get(violation.path)
    if not allowed_headers:
        return False
    if violation.kind != "forbidden-include":
        return False
    if violation.header.lower() in NEVER_EXEMPT_INCLUDES_LOWER:
        return False
    return violation.header in allowed_headers


def run_source_checks(files, read, exempt_cli_files=None, out=sys.stdout):
    """Scans C++ sources and BUILD/`.bzl` files outside `talOS/` for RULES.md 2
    bypasses. Returns the number of failures."""
    if exempt_cli_files is None:
        exempt_cli_files = NON_NODE_CLI_FILES
    cpp_files, starlark_trees, read_errors = discover_scanned_files(files, read)
    failures = 0
    for path, detail in read_errors:
        failures += 1
        print(f"VIOLATION {path}: {detail} (fail closed)", file=out)

    for path, tree in starlark_trees:
        for violation in scan_starlark_ast(path, tree):
            failures += 1
            print(f"VIOLATION {violation.path}:{violation.line}: "
                  f"{violation.detail} (RULES.md 2 / REQ-SAFE-27)", file=out)

    for path, allowed_headers in sorted(exempt_cli_files.items()):
        if not allowed_headers:
            failures += 1
            print(f"STALE non-node CLI exemption {path}: empty header list; "
                  f"delete the entry", file=out)
            continue
        for header in sorted(allowed_headers):
            if header.lower() in NEVER_EXEMPT_INCLUDES_LOWER:
                failures += 1
                print(f"VIOLATION {path}: exemption cannot allow {header} "
                      f"(NEVER_EXEMPT_INCLUDES)", file=out)

    used_exemptions = set()
    for path in cpp_files:
        try:
            text = read(path)
        except (OSError, UnicodeDecodeError) as exc:
            failures += 1
            print(f"VIOLATION {path}: cannot read source file: {exc} "
                  f"(fail closed)", file=out)
            continue
        for violation in scan_cpp_text(path, text):
            if _is_exempt_cli_violation(violation, exempt_cli_files):
                used_exemptions.add((path, violation.header))
                continue
            failures += 1
            print(f"VIOLATION {violation.path}:{violation.line}: "
                  f"{violation.detail} (RULES.md 2 / REQ-SAFE-27)", file=out)

    for path, allowed_headers in sorted(exempt_cli_files.items()):
        for header in sorted(allowed_headers):
            if header.lower() in NEVER_EXEMPT_INCLUDES_LOWER:
                continue
            if (path, header) not in used_exemptions:
                failures += 1
                print(f"STALE non-node CLI exemption {path} ({header}): "
                      f"no exempted hit left; delete the entry", file=out)

    exempt_header_count = sum(len(h) for h in exempt_cli_files.values())
    if failures:
        print(f"{failures} RULES.md 2 C++ bypass check(s) failed", file=out)
    else:
        print(f"C++ devices-layer bypass check passed "
              f"({len(cpp_files)} source file(s), "
              f"{len(starlark_trees)} Starlark file(s), "
              f"{exempt_header_count} allowlisted CLI header(s))", file=out)
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
    with open(path, encoding="utf-8") as f:
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
    files = tracked_files()
    failures += run_source_checks(files, read_source_file)
    failures += run_mujoco_checks(files, read_source_file)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
