"""Unit tests for tools/check_layers.py on fixture graphs and C++ sources (no bazel)."""

import collections
import io
import unittest

from tools import check_layers

# label -> (generator macro or None, [deps]). Every label is a rule. With
# ARCH-2 resolved, ALLOWLIST is empty and this graph passes with no exceptions.
GRAPH = {
    "//talOS/memory:residency": (None, []),
    "//talOS/memory:shared_memory": (None, ["//talOS/memory:residency"]),
    "//talOS/rtms:process_liveness": (None, []),
    "//talOS/rtms:rtms": (None, ["//talOS/memory:shared_memory",
                                 "//talOS/rtms:process_liveness"]),
    "//talOS/rtms:wake": (None, []),
    "//talOS/ipc:concepts": (None, []),
    "//talOS/ipc:ipc": (None, ["//talOS/ipc:concepts", "//talOS/rtms:rtms"]),
    "//talOS/events:events_os": (None, ["//talOS/rtms:wake"]),
    "//talOS/events:events": (None, ["//talOS/events:events_os",
                                     "//talOS/rtms:rtms"]),
    "//talOS/events:input_replay": (None, ["//talOS/events:events"]),
    "//talOS/hardware:config": (None, []),
    "//talOS/hardware:merge": (None, []),
    "//talOS/protocol:frame": (None, []),
    "//talOS/protocol:protocol": (None, ["//talOS/protocol:frame"]),
    "//talOS/hardware:codecs": (None, ["//talOS/hardware:config",
                                       "//talOS/protocol:frame"]),
    "//talOS/hardware:hardware": (None, ["//talOS/hardware:codecs",
                                         "//talOS/hardware:config",
                                         "//talOS/protocol:protocol"]),
    "//talOS/bridge:messages": (None, []),
    "//talOS/launcher:hardware_record": (None, ["//talOS/bridge:messages",
                                                "//talOS/hardware:codecs",
                                                "//talOS/protocol:frame"]),
    "//talOS/launcher:session_retention": (
        None, ["//talOS/launcher:hardware_record",
               "//talOS/hardware:codecs",
               "//talOS/protocol:frame"]),
    "//studio/bridge:json_format": (None, []),
    "//studio/bridge:system": (None, ["//studio/bridge:json_format",
                                      "//talOS/introspection:registry"]),
    "//studio/review:review": (None, ["//studio/bridge:json_format",
                                      "//talOS/hardware:codecs",
                                      "//talOS/launcher:hardware_record",
                                      "//talOS/launcher:session_retention",
                                      "//talOS/telemetry:channels"]),
    "//studio/review:studio_review": (None, ["//studio/review:review"]),
    "//talOS/introspection:names": (None, []),
    "//talOS/introspection:registry": (None, ["//talOS/memory:shared_memory",
                                              "//talOS/rtms:process_liveness"]),
    "//talOS/introspection:reporter": (None, ["//talOS/introspection:registry",
                                              "//talOS/memory:residency"]),
    "//talOS/configuration:configuration": (None, ["//talOS/hardware:config",
                                                   "//talOS/introspection:names"]),
    "//talOS/node:loop_port": (None, []),
    "//talOS/node:access_hdr": (None, []),
    "//talOS/node:access": (None, ["//talOS/node:node"]),
    "//talOS/node:node": (None, ["//talOS/events:events",
                                 "//talOS/events:input_replay",
                                 "//talOS/ipc:concepts",
                                 "//talOS/configuration:configuration",
                                 "//talOS/introspection:reporter"]),
    "//talOS/bridge:node_lib": (None, ["//talOS/hardware:merge",
                                       "//talOS/hardware:hardware"]),
    "//talOS/bridge:hardware_node": (None, ["//talOS/bridge:node_lib"]),
    "//talOS/launcher:launcher_lib": (None, ["//talOS/events:events_os",
                                             "//talOS/introspection:registry"]),
    "//talOS/devices:access_hdr": (None, ["//talOS/hardware:hardware"]),
    "//talOS/devices:devices": (None, ["//talOS/node:node",
                                       "//talOS/node:access",
                                       "//talOS/hardware:hardware"]),
    "//talOS/devices:access": (None, ["//talOS/devices:devices",
                                      "//talOS/hardware:hardware"]),
    "//talOS/devices:testing": (None, ["//talOS/devices:devices",
                                       "//talOS/devices:access",
                                       "//talOS/hardware:merge"]),
    "//talOS/telemetry:channels": (None, ["//talOS/hardware:codecs"]),
    "//talOS/telemetry:telemetry": (None, ["//talOS/telemetry:channels",
                                           "//talOS/node:access"]),
    "//talOS/telemetry:node": ("talos_cc_node", ["//talOS/telemetry:telemetry"]),
    "//talOS/teleop:teleop": (None, ["//talOS/devices:devices",
                                     "//talOS/node:access"]),
    "//talOS/teleop:node": ("talos_cc_node", ["//talOS/teleop:teleop"]),
    "//talOS/node_api:node_api": (None, ["//talOS/devices:devices",
                                         "//talOS/node:access"]),
    "//talOS/node_api:libtalos_node.so": (None, ["//talOS/node_api:node_api"]),
    "//talOS/teleop:libteleop.so": ("talos_cc_shared_node",
                                    ["//talOS/teleop:teleop"]),
    "//talOS/python:talos": (None, ["//talOS/node_api:libtalos_node.so",
                                    "//talOS/teleop:libteleop.so"]),
    "//talOS/geometry:geometry": (None, []),
    "//talOS/vision:vision": (None, ["//talOS/geometry:geometry"]),
    "//bench/main_processor/swerve_module:messages": ("talos_messages", []),
    "//bench/main_processor/swerve_module:swerve_module": (
        None, ["//bench/main_processor/swerve_module:messages",
               "//talOS/devices:devices", "//talOS/node:node"]),
    "//bench/main_processor/swerve_module:node": (
        "talos_cc_node",
        ["//bench/main_processor/swerve_module:swerve_module",
         "//talOS/node:node"]),
    "//bench/main_processor/swerve_module:send_target": (
        None, ["//bench/main_processor/swerve_module:messages",
               "//talOS/ipc:ipc", "//talOS/hardware:hardware"]),
    "//bench/main_processor/motor:motor": (
        None, ["//bench/main_processor/swerve_module:messages",
               "//talOS/devices:devices", "//talOS/hardware:config"]),
    "//bench/main_processor/motor:node": (
        "talos_cc_node", ["//bench/main_processor/motor:motor",
                          "//talOS/node:node"]),
    "//bench/main_processor/motor:node_test": (
        None, ["//bench/main_processor/motor:motor"]),
    "//bench/main_processor/gyro:gyro": (
        None, ["//bench/main_processor/swerve_module:messages",
               "//talOS/devices:devices"]),
    "//bench/main_processor/gyro:node": (
        "talos_py_node", ["//bench/main_processor/gyro:gyro"]),
    "//bench/main_processor/simulation:closed_loop_test": (
        None, ["//bench/main_processor/gyro:gyro",
               "//bench/main_processor/swerve_module:swerve_module"]),
    "//bench/main_processor/configuration:config_golden_test": (
        None, ["//bench/main_processor/swerve_module:swerve_module"]),
    "//bench/main_processor:bench_bundle": (
        None, ["//bench/main_processor/swerve_module:node"]),
    "//examples/superstructure/cpp:mechanisms": (
        None, ["//talOS/devices:devices", "//talOS/node:node"]),
    "//examples/superstructure/cpp:libmechanisms.so": (
        "talos_cc_shared_node", ["//examples/superstructure/cpp:mechanisms",
                                 "//talOS/node:node"]),
    "//examples/superstructure/py:elevator": (
        "talos_py_node", ["//examples/superstructure/cpp:libmechanisms.so",
                          "//talOS/python:talos"]),
    "//tools/scaffold_testdata/gripper:gripper": (
        None, ["//talOS/devices:devices", "//talOS/node:node"]),
    "//tools/scaffold_testdata/gripper:node": (
        "talos_cc_node", ["//tools/scaffold_testdata/gripper:gripper",
                          "//talOS/node:node"]),
}

# Rule kinds that are NOT cc_library (defaults to cc_library when omitted).
KINDS = {
    "//bench/main_processor/swerve_module:send_target": "cc_binary",
    "//studio/review:studio_review": "cc_binary",
    "//bench/main_processor/swerve_module:node": "talos_node",
    "//bench/main_processor/motor:node": "talos_node",
    "//bench/main_processor/motor:node_test": "cc_test",
    "//bench/main_processor/gyro:node": "talos_node",
    "//bench/main_processor/simulation:closed_loop_test": "cc_test",
    "//bench/main_processor/configuration:config_golden_test": "cc_test",
    "//bench/main_processor:bench_bundle": "talos_bundle",
    "//examples/superstructure/cpp:libmechanisms.so": "cc_shared_library",
    "//examples/superstructure/py:elevator": "talos_node",
    "//tools/scaffold_testdata/gripper:node": "talos_node",
}

# What `attr(testonly, 1, ...)` returns: every test rule and testonly helper.
TESTONLY = frozenset({
    "//bench/main_processor/motor:node_test",
    "//bench/main_processor/simulation:closed_loop_test",
    "//bench/main_processor/configuration:config_golden_test",
})


def package(label):
    return label[2:].split(":")[0]


def matches(label, pattern):
    if pattern.endswith(":*"):
        return package(label) == pattern[:-2]
    if ":" in pattern:
        return label == "//" + pattern
    root = pattern.removesuffix("/...")
    return package(label) == root or package(label).startswith(root + "/")


# Default restricted visibility table matching the real BUILD files (ARCH-2).
VISIBILITY = {
    "//talOS/node:access": (
        "//talOS/devices:__pkg__", "//talOS/launcher:__pkg__",
        "//talOS/node:__pkg__", "//talOS/node_api:__pkg__",
        "//talOS/telemetry:__pkg__", "//talOS/teleop:__pkg__",
    ),
    "//talOS/node:access_hdr": ("//visibility:private",),
    "//talOS/node:loop_port": ("//visibility:private",),
    "//talOS/devices:access": (
        "//talOS/devices:__pkg__", "//talOS/mechanisms:__pkg__",
        "//talOS/node_api:__pkg__",
    ),
    "//talOS/devices:access_hdr": ("//visibility:private",),
    "//talOS/rtms:rtms": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/rtms:wake": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/rtms:process_liveness": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/memory:shared_memory": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/memory:residency": ("//talOS:__subpackages__",),
    "//talOS/events:events": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/events:events_os": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/events:input_replay": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/telemetry:telemetry": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/telemetry:channels": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/teleop:teleop": ("//talOS:__subpackages__", "//tools:__pkg__"),
    "//talOS/node_api:node_api": ("//talOS:__subpackages__",),
    "//talOS/bridge:node_lib": ("//talOS:__subpackages__", "//tools:__pkg__"),
    "//talOS/introspection:registry": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/introspection:reporter": (
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/launcher:launcher_lib": (
        "//bench/main_processor:__pkg__", "//studio:__subpackages__",
        "//talOS:__subpackages__", "//tools:__pkg__",
    ),
    "//talOS/hardware:hardware": (
        "//bench/main_processor/swerve_module:__pkg__",
        "//controller_processor/rio:__pkg__",
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
    "//talOS/ipc:ipc": (
        "//bench/main_processor/swerve_module:__pkg__",
        "//studio:__subpackages__", "//talOS:__subpackages__",
        "//tools:__pkg__",
    ),
}


def visibility_allows(vis_list, target_label, viewer_label):
    viewer_pkg = package(viewer_label)
    target_pkg = package(target_label)
    if viewer_pkg == target_pkg:
        return True
    for entry in vis_list:
        if entry == "//visibility:public":
            return True
        if entry == "//visibility:private":
            continue
        if entry.endswith(":__pkg__"):
            if viewer_pkg == entry[2:-len(":__pkg__")]:
                return True
        elif entry.endswith(":__subpackages__"):
            root = entry[2:-len(":__subpackages__")]
            if viewer_pkg == root or viewer_pkg.startswith(root + "/"):
                return True
    return False


class FakeBazel:
    """Evaluates the checks' sets over GRAPH, as `bazel query` would."""

    def __init__(self, graph, kinds=None, testonly=None, visibility=None):
        self.graph = graph
        self.kinds = kinds if kinds is not None else KINDS
        self.testonly = testonly if testonly is not None else TESTONLY
        self.visibility = visibility if visibility is not None else VISIBILITY

    def members(self, target_set):
        result = set()
        for label, (generator, _) in self.graph.items():
            if not any(matches(label, p) for p in target_set.include):
                continue
            if any(matches(label, p) for p in target_set.exclude):
                continue
            if target_set.only_kinds or target_set.only_generators:
                kind = self.kinds.get(label, "cc_library")
                if (kind not in target_set.only_kinds and
                        generator not in target_set.only_generators):
                    continue
            if generator in target_set.except_generators:
                continue
            if target_set.except_testonly and label in self.testonly:
                continue
            result.add(label)
        return result

    def node_sample_targets(self):
        samples = {}
        for label in sorted(self.graph):
            generator, _ = self.graph[label]
            pkg = package(label)
            if (generator in check_layers.NODE_GENERATORS and
                    pkg.split("/")[0] in check_layers.NODE_ROOTS):
                samples.setdefault(pkg, label)
        return samples

    def node_packages(self):
        return sorted(self.node_sample_targets().keys())

    def public_targets(self, targets):
        out = []
        for t in targets:
            label = "//" + t
            if "//visibility:public" in self.visibility.get(label, ()):
                out.append(label)
        return sorted(out)

    def visible_to(self, target_label, targets):
        out = []
        for t in targets:
            label = "//" + t
            if visibility_allows(self.visibility.get(label, ()),
                                 label, target_label):
                out.append(label)
        return sorted(out)

    def somepath(self, frm, to):
        targets = self.members(to)
        parent = {label: None for label in sorted(self.members(frm))}
        queue = collections.deque(parent)
        while queue:
            label = queue.popleft()
            if label in targets:
                path = []
                while label is not None:
                    path.append(label)
                    label = parent[label]
                return path[::-1]
            for dep in self.graph[label][1]:
                if dep not in parent:
                    parent[dep] = label
                    queue.append(dep)
        return []

    def direct(self, frm, to):
        sources = self.members(frm)
        deps = {d for s in sources for d in self.graph[s][1]} - sources
        return sorted(deps & self.members(to))


def run(graph, kinds=None, testonly=None, visibility=None):
    out = io.StringIO()
    return (check_layers.run(
        FakeBazel(graph, kinds, testonly, visibility), out), out.getvalue())


def with_edge(frm, to, generator=None):
    graph = {label: (gen, list(deps)) for label, (gen, deps) in GRAPH.items()}
    graph.setdefault(to, (generator, []))
    graph.setdefault(frm, (None, []))[1].append(to)
    return graph


def run_sources(file_map, exempt_cli_files=None):
    out = io.StringIO()
    failures = check_layers.run_source_checks(
        sorted(file_map.keys()),
        lambda p: file_map[p],
        exempt_cli_files={} if exempt_cli_files is None else exempt_cli_files,
        out=out,
    )
    return failures, out.getvalue()


class CheckLayersTest(unittest.TestCase):
    # REQ-CONTRACT-15, REQ-CONTRACT-16
    def test_each_forbidden_dependency_fails_with_its_path(self):
        for frm, to in (
                ("//talOS/rtms:rtms", "//talOS/events:events"),
                ("//talOS/events:events", "//talOS/devices:devices"),
                ("//talOS/node:node", "//talOS/devices:devices"),
                ("//talOS/node:node", "//talOS/bridge:hardware_node"),
                ("//talOS/node:node", "//talOS/hardware:hardware"),
                ("//talOS/devices:devices", "//talOS/bridge:hardware_node"),
                ("//talOS/events:events", "//talOS/hardware:config"),
                ("//talOS/devices:devices", "//talOS/teleop:libteleop.so"),
                ("//talOS/launcher:launcher",
                 "//bench/main_processor/motor:motor"),
                ("//talOS/telemetry:node", "//studio:bridge"),
                ("//talOS/geometry:geometry", "//talOS/vision:vision"),
                ("//talOS/python:talos", "//talOS/node:node"),
                ("//bench/main_processor/swerve_module:swerve_module",
                 "//bench/main_processor/motor:motor"),
                # A node's production library, and a node's own test, never
                # link another node's class.
                ("//bench/main_processor/gyro:gyro",
                 "//bench/main_processor/swerve_module:swerve_module"),
                ("//bench/main_processor/motor:node_test",
                 "//bench/main_processor/swerve_module:swerve_module"),
                # Outside node packages only a testonly target links one.
                ("//bench/main_processor/simulation:harness",
                 "//bench/main_processor/swerve_module:swerve_module"),
                ("//examples/superstructure/py:elevator",
                 "//examples/superstructure/cpp:mechanisms")):
            with self.subTest(frm=frm, to=to):
                failures, out = run(with_edge(frm, to))
                # A lower layer's edge can also fail the layers that reach it.
                self.assertGreaterEqual(failures, 1, out)
                # The path (or, for a direct check, the dependency) names
                # the forbidden target.
                self.assertIn(f"  {to}\n", out)

    # REQ-SAFE-27, REQ-CONTRACT-16
    def test_cc_node_direct_deps_on_bypass_targets_fail(self):
        for frm, to in (
                ("//bench/main_processor/swerve_module:swerve_module",
                 "//talOS/rtms:rtms"),
                ("//bench/main_processor/swerve_module:node",
                 "//talOS/ipc:ipc"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/memory:shared_memory"),
                ("//bench/main_processor/motor:node",
                 "//talOS/node:access"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/devices:access"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/node:loop_port"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/events:events"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/events:events_os"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/events:input_replay"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/telemetry:telemetry"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/telemetry:channels"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/teleop:teleop"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/node_api:node_api"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/bridge:node_lib"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/introspection:registry"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/introspection:reporter"),
                ("//bench/main_processor/motor:motor",
                 "//talOS/launcher:launcher_lib"),
                ("//examples/superstructure/cpp:mechanisms",
                 "//talOS/hardware:hardware"),
                ("//examples/superstructure/cpp:libmechanisms.so",
                 "//talOS/rtms:rtms"),
                ("//tools/scaffold_testdata/gripper:gripper",
                 "//talOS/ipc:ipc"),
                # A helper cc_library outside a node package cannot smuggle
                # a raw transport dep into a robot node either.
                ("//bench/main_processor/helper:helper",
                 "//talOS/rtms:rtms")):
            with self.subTest(frm=frm, to=to):
                failures, out = run(with_edge(frm, to))
                self.assertGreaterEqual(failures, 1, out)
                self.assertIn(f"  {to}\n", out)

    # REQ-SAFE-27, REQ-CONTRACT-16
    def test_testonly_target_outside_node_may_dep_on_transport(self):
        graph = with_edge("//bench/main_processor/simulation:closed_loop_test",
                          "//talOS/rtms:rtms")
        failures, out = run(graph)
        self.assertEqual(failures, 0, out)

    # REQ-CONTRACT-16
    def test_allowed_dependencies_pass(self):
        # Today's graph, with its allowlisted dependencies, testonly targets
        # outside node packages that link nodes' libraries, standalone CLI
        # send_target, and a bundle of node binaries.
        failures, out = run(GRAPH)
        self.assertEqual(failures, 0, out)
        # Plus a dependency through a gateway, on messages, on hardware:config,
        # or on a node exported for talos.cpp_node.
        for frm, to, generator in (
                ("//talOS/python:talos", "//talOS/msgs:control_py", None),
                ("//bench/main_processor/swerve_module:swerve_module",
                 "//examples/superstructure/cpp:libmechanisms.so",
                 "talos_cc_shared_node"),
                ("//examples/superstructure/py:elevator",
                 "//bench/main_processor/swerve_module:messages",
                 "talos_messages"),
                ("//examples/superstructure/cpp:mechanisms",
                 "//talOS/hardware:config", None)):
            with self.subTest(frm=frm, to=to):
                failures, out = run(with_edge(frm, to, generator))
                self.assertEqual(failures, 0, out)

    # REQ-CONTRACT-15, REQ-CONTRACT-16, REQ-SAFE-27
    def test_allowlist_is_empty_and_visibility_restrictions_hold(self):
        self.assertEqual(check_layers.ALLOWLIST, {})
        # Making any restricted target //visibility:public fails.
        for target in (check_layers.RESTRICTED_INTERNAL_TARGETS +
                       check_layers.RESTRICTED_WIRE_TARGETS):
            label = "//" + target
            with self.subTest(public=label):
                vis = dict(VISIBILITY)
                vis[label] = ("//visibility:public",)
                failures, out = run(GRAPH, visibility=vis)
                self.assertGreaterEqual(failures, 1, out)
                self.assertIn(f"  {label}\n", out)
        # Granting a robot node package visibility to a restricted target fails,
        # including internal targets for swerve_module.
        for label, pkg in (
                ("//talOS/node:access", "//bench/main_processor/motor:__pkg__"),
                ("//talOS/devices:access", "//examples/superstructure/cpp:__pkg__"),
                ("//talOS/rtms:rtms", "//bench/main_processor/swerve_module:__pkg__"),
                ("//talOS/rtms:process_liveness", "//bench/main_processor/motor:__pkg__"),
                ("//talOS/memory:residency", "//bench/main_processor/motor:__pkg__"),
                ("//talOS/events:events", "//bench/main_processor/motor:__pkg__"),
                ("//talOS/events:events_os", "//bench/main_processor/swerve_module:__pkg__"),
                ("//talOS/telemetry:telemetry", "//bench/main_processor/motor:__pkg__"),
                ("//talOS/telemetry:channels", "//bench/main_processor/motor:__pkg__"),
                ("//talOS/teleop:teleop", "//bench/main_processor/motor:__pkg__"),
                ("//talOS/node_api:node_api", "//examples/superstructure/cpp:__pkg__"),
                ("//talOS/hardware:hardware", "//bench/main_processor/motor:__pkg__"),
                ("//talOS/ipc:ipc", "//examples/superstructure/cpp:__pkg__")):
            with self.subTest(label=label, pkg=pkg):
                vis = dict(VISIBILITY)
                vis[label] = tuple(vis[label]) + (pkg,)
                failures, out = run(GRAPH, visibility=vis)
                self.assertGreaterEqual(failures, 1, out)
                self.assertIn(f"  {label}\n", out)

    # REQ-SAFE-35
    def test_studio_review_cannot_depend_on_rtms_ipc_protocol_or_bridge(self):
        # Direct deps from studio/review on rtms, ipc, protocol or bridge fail.
        for frm, to in (
                ("//studio/review:review", "//talOS/rtms:rtms"),
                ("//studio/review:review", "//talOS/rtms:process_liveness"),
                ("//studio/review:review", "//talOS/ipc:ipc"),
                ("//studio/review:review", "//talOS/protocol:protocol"),
                ("//studio/review:review", "//talOS/bridge:messages"),
                ("//studio/review:studio_review", "//talOS/rtms:wake"),
                ("//studio/review:studio_review", "//talOS/bridge:node_lib"),
                # Transitive paths from studio/review into RTMS/IPC/UDP/
                # hardware gateway/shared_memory/bridge runtime or
                # node/devices also fail.
                ("//studio/bridge:json_format", "//talOS/rtms:rtms"),
                ("//studio/bridge:json_format", "//talOS/rtms:wake"),
                ("//studio/bridge:json_format", "//talOS/ipc:ipc"),
                ("//studio/bridge:json_format", "//talOS/protocol:protocol"),
                ("//studio/bridge:json_format", "//talOS/hardware:hardware"),
                ("//studio/bridge:json_format", "//talOS/memory:shared_memory"),
                ("//studio/bridge:json_format", "//talOS/bridge:node_lib"),
                ("//studio/bridge:json_format", "//talOS/node:node"),
                ("//studio/bridge:json_format", "//talOS/devices:devices"),
                # Linking live :system or :hardware back into review fails via
                # transitive shared_memory / hardware:hardware / protocol:protocol.
                ("//studio/review:review", "//studio/bridge:system"),
                ("//studio/review:review", "//talOS/hardware:hardware"),
                ("//talOS/launcher:session_retention",
                 "//talOS/memory:shared_memory"),
        ):
            with self.subTest(frm=frm, to=to):
                failures, out = run(with_edge(frm, to))
                self.assertGreaterEqual(failures, 1, out)
                if to not in ("//studio/bridge:system",):
                    self.assertIn(f"  {to}\n", out)


class CheckNodeSourcesTest(unittest.TestCase):
    NODE_BUILD = (
        'load("//talOS/build:contract.bzl", "talos_cc_node")\n'
        'talos_cc_node(name = "node", srcs = ["arm.cc"], node_name = "arm")\n'
    )

    # REQ-SAFE-27
    def test_legitimate_node_code_and_comments_pass(self):
        files = {
            "bench/arm/BUILD": self.NODE_BUILD,
            "bench/arm/arm.h": (
                '#pragma once\n'
                '// Never include "talOS/node/access.h" or call\n'
                '// detail::NodeAccess::add_hardware_request here.\n'
                '/*\n'
                ' * #include <talOS/rtms/rtms.h>\n'
                ' * #include "talOS/ipc/publisher.h"\n'
                ' * #include "talOS/hardware/messages.h"\n'
                ' * detail::HardwareAccess::note_live_target\n'
                ' */\n'
                '#include "talOS/devices/devices.h"\n'
                '#include "talOS/hardware/config.h"\n'
                '#include "talOS/node/node.h"\n'
                'inline constexpr int kPeriodUs = 5\'000;\n'
                'inline constexpr unsigned kMask = 0xdead\'beef;\n'
                'inline constexpr double kScale = .5\'0e+1\'2;\n'
                'inline constexpr const char* kDoc = '
                '"detail::NodeAccess::add_hardware_request";\n'
            ),
            "bench/arm/arm.cc": (
                '#include "arm/arm.h"\n'
                '// Token paste operators in comments (## and %:%:) are ok.\n'
                '/* Also ## and %:%: in block comments. */\n'
                'const char* kStr = "## and %:%: in strings are ok";\n'
                'const char* kPunct = "what??! http://example.com";\n'
                'const char* kRaw = R"(\n'
                '#include "talOS/node/access.h"\n'
                'detail::NodeAccess::add_hardware_request\n'
                '#define CAT(a, b) a##b\n'
                '#define CAT2(a, b) a%:%:b\n'
                'what??!\n'
                ')";\n'
                'char quote(int k) {\n'
                '  switch (k) {\n'
                '    case\'"\': return\'"\';\n'
                '    default: return \'x\';\n'
                '  }\n'
                '}\n'
            ),
            # Explicit non-C++ files under scanned roots are not scanned as C++.
            "bench/arm/README": (
                "Notes mentioning NodeAccess and #include \"talOS/node/access.h\"\n"
            ),
            "bench/arm/notes.md": (
                "Docs mentioning NodeAccess and ##.\n"
            ),
            "bench/arm/helper.py": (
                "# Python mentioning NodeAccess and talOS/node/access.h\n"
            ),
            "bench/arm/config.toml": (
                '# TOML comment mentioning NodeAccess\nname = "arm"\n'
            ),
            "bench/arm/arm_message.fbs": (
                "// FlatBuffers mentioning NodeAccess\n"
            ),
            # Legal Starlark target deps (e.g. testonly/CLI targets) pass.
            "bench/arm/sim_test_BUILD_extra.bzl": (
                'DEPS = ["//talOS/ipc", "//talOS/rtms:rtms", "//talOS/node:testing"]\n'
            ),
            # Repo tooling outside tools/scaffold_testdata/ is not scanned.
            "tools/BUILD": (
                'cc_binary(name = "data_flow_sizes", srcs = ["data_flow_sizes.cc"])\n'
            ),
            "tools/data_flow_sizes.cc": (
                '#include "talOS/rtms/rtms.h"\n'
                '#include "talOS/hardware/messages.h"\n'
            ),
            # Framework code under talOS/ is not scanned by the node bypass check.
            "talOS/devices/BUILD": 'talos_cc_node(name = "x")\n',
            "talOS/devices/devices.cc": (
                '#include "talOS/node/access.h"\n'
                'void f() { talos::detail::NodeAccess::add_hardware_request(); }\n'
            ),
        }
        failures, out = run_sources(files)
        self.assertEqual(failures, 0, out)

    # REQ-SAFE-27
    def test_every_forbidden_include_and_symbol_is_caught(self):
        cases = [
            ("node_access_quotes", '#include "talOS/node/access.h"\n'),
            ("node_access_angles", "#include <talOS/node/access.h>\n"),
            ("node_internal", '#include "talOS/node/internal.h"\n'),
            ("devices_access", '#include "talOS/devices/access.h"\n'),
            ("rtms_angles", "#include <talOS/rtms/rtms.h>\n"),
            ("ipc_quotes", '#include "talOS/ipc/publisher.h"\n'),
            ("memory_quotes", '#include "talOS/memory/shared_memory_ptr.h"\n'),
            ("hw_messages", '#include "talOS/hardware/messages.h"\n'),
            ("hw_packet", '#include "talOS/hardware/packet.h"\n'),
            ("hw_request", '#include "talOS/hardware/request.h"\n'),
            ("events_realtime", '#include "talOS/events/realtime_event_loop.h"\n'),
            ("events_os_poller", '#include "talOS/events/os/poller.h"\n'),
            ("events_input_replay", '#include "talOS/events/input_replay.h"\n'),
            ("telemetry_node", '#include "talOS/telemetry/node.h"\n'),
            ("telemetry_channels", '#include "talOS/telemetry/channels.h"\n'),
            ("teleop_node", '#include "talOS/teleop/node.h"\n'),
            ("node_api_header", '#include "talOS/node_api/node_api.h"\n'),
            ("bridge_node", '#include "talOS/bridge/node.h"\n'),
            ("introspection_registry", '#include "talOS/introspection/registry.h"\n'),
            ("introspection_reporter", '#include "talOS/introspection/reporter.h"\n'),
            ("whitespace_include", '  #   include   "talOS/node/access.h"\n'),
            ("comment_before_include",
             '/* ok */ #include "talOS/ipc/publisher.h"\n'),
            ("comment_inside_include",
             '# /* c */ include /* c */ "talOS/node/access.h"\n'),
            ("multiline_comment_inside_include",
             '# /* line1\nline2 */ include "talOS/rtms/rtms.h"\n'),
            ("backslash_continuation",
             '#inc\\\nlude "talOS/node/access.h"\n'),
            ("path_traversal",
             '#include "talOS/devices/../node/access.h"\n'),
            ("relative_traversal",
             '#include "../../talOS/ipc/publisher.h"\n'),
            ("external_repo_prefix",
             '#include "external/talos+/talOS/rtms/rtms.h"\n'),
            ("include_next",
             '#include_next "talOS/node/access.h"\n'),
            ("digraph_include",
             '%:include "talOS/node/access.h"\n'),
            ("computed_include_macro",
             '#define H "talOS/node/access.h"\n#include H\n'),
            ("detail_node_access_call",
             "void f(talos::Node& n) {\n"
             "  talos::detail::NodeAccess::add_hardware_request(n, 0, 8, {});\n"
             "}\n"),
            ("node_access_via_using",
             "using namespace talos::detail;\n"
             "void f(talos::Node& n) { NodeAccess::publisher_send(n, 0, 0, 0); }\n"),
            ("add_hardware_request_alone",
             "auto fn = &add_hardware_request;\n"),
            ("hardware_access_note_live_target",
             "void f(talos::HardwareLink& l) {\n"
             "  talos::detail::HardwareAccess::note_live_target(l);\n"
             "}\n"),
            ("after_string_with_slash_star",
             'const char* s = "/*"; detail::NodeAccess::add_hardware_request();\n'),
            ("after_char_literal_quote",
             "char c = '\"'; detail::NodeAccess::add_hardware_request();\n"),
            ("case_char_literal_quote_no_space",
             "void f(char c, talos::Node& n) {\n"
             "  switch (c) {\n"
             "    case'\"': talos::detail::NodeAccess::add_hardware_request(n, 0, 0, {});\n"
             "  }\n"
             "}\n"),
            ("case_char_literal_quote_spliced_lines",
             "void f(char c, talos::Node& n) {\n"
             "  switch (c) {\n"
             "    case'\"':\\\n"
             "    talos::detail::NodeAccess::add_hardware_request(n, 0, 0, {});\n"
             "  }\n"
             "}\n"),
            ("return_char_literal_quote_no_space",
             "char f() { return'\"'; } "
             "void g() { talos::detail::NodeAccess::add_hardware_request(); }\n"),
            ("ident_ending_in_digit_before_char_quote",
             "int a1 = 0; char c = a1 ? '\"' : 'x';\n"
             "void g() { case1'\"': talos::detail::HardwareAccess::note_live_target(); }\n"),
            ("after_digit_separator",
             "int n = 1'000; /* ok */ detail::NodeAccess::add_hardware_request();\n"),
            ("unterminated_block_comment_fails_closed",
             "/* never closed\n"),
            ("unterminated_raw_string_fails_closed",
             'const char* s = R"tag(never closed\n'),
        ]
        for name, body in cases:
            with self.subTest(name=name):
                files = {
                    "bench/arm/BUILD": self.NODE_BUILD,
                    "bench/arm/arm.cc": body,
                }
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn("bench/arm/arm.cc:", out)

    # REQ-SAFE-27
    def test_token_pasting_is_refused_outside_comments_and_strings(self):
        cases = [
            ("hash_hash",
             "#define CAT(a, b) a##b\n"
             "void f() { talos::detail::CAT(Node, Access)::CAT(add_hardware, _request)(); }\n"),
            ("hash_hash_spaced",
             "#define CAT(a, b) a ## b\n"),
            ("digraph_paste",
             "#define CAT(a, b) a%:%:b\n"),
            ("digraph_paste_spaced",
             "#define CAT(a, b) a %:%: b\n"),
            ("spliced_hash_hash",
             "#define CAT(a, b) a#\\\n#b\n"),
            ("spliced_digraph_paste",
             "#define CAT(a, b) a%:\\\n%:b\n"),
        ]
        for name, body in cases:
            with self.subTest(name=name):
                files = {
                    "bench/arm/BUILD": self.NODE_BUILD,
                    "bench/arm/arm.cc": body,
                }
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn("bench/arm/arm.cc:", out)

    # REQ-SAFE-27
    def test_helper_packages_subpackages_and_wrapper_macros_are_scanned(self):
        cases = [
            ("helper_cc_library_no_node_macro", {
                "bench/main_processor/BUILD": (
                    'cc_library(name = "helper", hdrs = ["helper.h"])\n'
                ),
                "bench/main_processor/helper.h": (
                    'void bypass() { talos::detail::NodeAccess::add_hardware_request(); }\n'
                ),
            }, "bench/main_processor/helper.h:1:"),
            ("helper_cc_library_forbidden_include", {
                "bench/main_processor/BUILD": (
                    'cc_library(name = "helper", hdrs = ["helper.h"])\n'
                ),
                "bench/main_processor/helper.h": '#include "talOS/rtms/rtms.h"\n',
            }, "bench/main_processor/helper.h:1:"),
            ("subpackage_with_own_build", {
                "bench/arm/BUILD": self.NODE_BUILD,
                "bench/arm/subpkg/BUILD": (
                    'cc_library(name = "sub", hdrs = ["sub.h"])\n'
                ),
                "bench/arm/subpkg/sub.h": '#include "talOS/node/access.h"\n',
            }, "bench/arm/subpkg/sub.h:1:"),
            ("custom_wrapper_macro_package", {
                "examples/custom/BUILD": (
                    'load("//my:macros.bzl", "robot_node")\n'
                    'robot_node(name = "node", srcs = ["custom.cc"])\n'
                ),
                "examples/custom/custom.cc": (
                    'void f() { talos::detail::HardwareAccess::note_live_target(); }\n'
                ),
            }, "examples/custom/custom.cc:1:"),
            ("scaffold_testdata_helper_package", {
                "tools/scaffold_testdata/helper/BUILD": (
                    'cc_library(name = "h", hdrs = ["h.h"])\n'
                ),
                "tools/scaffold_testdata/helper/h.h": '#include "talOS/ipc/publisher.h"\n',
            }, "tools/scaffold_testdata/helper/h.h:1:"),
            ("root_package_standalone_robot", {
                "BUILD": self.NODE_BUILD,
                "wrist.h": '#include <talOS/rtms/rtms.h>\n',
            }, "wrist.h:1:"),
            ("subdirectory_without_build", {
                "bench/arm/BUILD": (
                    'load("//talOS/build:contract.bzl", my_node = "talos_cc_node")\n'
                    'my_node(name = "node", srcs = ["arm.cc"], node_name = "arm")\n'
                ),
                "bench/arm/detail/helper.inl": '#include "talOS/node/access.h"\n',
            }, "bench/arm/detail/helper.inl:1:"),
        ]
        for name, files, expected in cases:
            with self.subTest(name=name):
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn(expected, out)

    # REQ-SAFE-27
    def test_cpp_suffixes_and_extensionless_build_headers_are_scanned(self):
        for suffix in (
                ".tcc", ".h++", ".c++", ".cppm", ".ixx",
                ".hpp", ".hh", ".hxx", ".inl", ".inc", ".ipp",
                ".cc", ".cpp", ".cxx", ".h", ".c", ".TCC"):
            with self.subTest(suffix=suffix):
                path = f"bench/arm/bypass{suffix}"
                files = {
                    "bench/arm/BUILD": (
                        'cc_library(name = "arm", srcs = ["bypass' + suffix + '"])\n'
                    ),
                    path: '#include "talOS/node/access.h"\n',
                }
                failures, out = run_sources(files)
                self.assertEqual(failures, 1, f"{suffix}: {out}")
                self.assertIn(f"{path}:1:", out)

        extless_cases = [
            ("textual_hdrs", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", textual_hdrs = ["bypass_hdr"])\n'
                ),
                "bench/arm/bypass_hdr": '#include "talOS/node/access.h"\n',
            }, "bench/arm/bypass_hdr:1:"),
            ("hdrs_subdir", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", hdrs = ["detail/bypass_hdr"])\n'
                ),
                "bench/arm/detail/bypass_hdr": (
                    'void f() { talos::detail::NodeAccess::add_hardware_request(); }\n'
                ),
            }, "bench/arm/detail/bypass_hdr:1:"),
            ("srcs_extensionless", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = [":raw_hdr"])\n'
                ),
                "bench/arm/raw_hdr": '#include "talOS/rtms/rtms.h"\n',
            }, "bench/arm/raw_hdr:1:"),
            ("textual_hdrs_glob", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", textual_hdrs = glob(["inc/*"]))\n'
                ),
                "bench/arm/inc/secret": '#include "talOS/devices/access.h"\n',
            }, "bench/arm/inc/secret:1:"),
        ]
        for name, files, expected in extless_cases:
            with self.subTest(name=name):
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn(expected, out)

    # REQ-SAFE-27
    def test_non_node_cli_exemption_scope_and_staleness(self):
        exempt = {
            "bench/swerve/send_target.cc": frozenset({
                "talOS/ipc/publisher.h",
                "talOS/hardware/packet.h",
            }),
        }
        # Only the exact allowlisted transport/wire includes pass.
        files = {
            "bench/swerve/BUILD": self.NODE_BUILD,
            "bench/swerve/send_target.cc": (
                '#include "talOS/ipc/publisher.h"\n'
                '#include "talOS/hardware/packet.h"\n'
            ),
        }
        failures, out = run_sources(files, exempt_cli_files=exempt)
        self.assertEqual(failures, 0, out)

        # An unallowlisted transport include in the same CLI is refused.
        extra_transport = dict(files)
        extra_transport["bench/swerve/send_target.cc"] += (
            '#include "talOS/rtms/rtms.h"\n'
        )
        failures, out = run_sources(extra_transport, exempt_cli_files=exempt)
        self.assertEqual(failures, 1, out)
        self.assertIn("talOS/rtms/rtms.h", out)

        # talOS/devices/access.h, talOS/node/access.h, NodeAccess, and token
        # pasting are refused even inside send_target.cc (and even if someone
        # tries to list access.h in the exemption map).
        for label, bad_line, custom_exempt in (
                ("devices_access", '#include "talOS/devices/access.h"\n', exempt),
                ("devices_access_even_if_listed",
                 '#include "talOS/devices/access.h"\n',
                 {"bench/swerve/send_target.cc": frozenset({
                     "talOS/ipc/publisher.h",
                     "talOS/hardware/packet.h",
                     "talOS/devices/access.h",
                 })}),
                ("node_access_header", '#include "talOS/node/access.h"\n', exempt),
                ("node_access_symbol",
                 'void f() { detail::NodeAccess::add_hardware_request(); }\n',
                 exempt),
                ("token_paste", '#define CAT(a, b) a##b\n', exempt),
                ("trigraph_in_cli", '#define CAT(a, b) a??=??=b\n', exempt),
                ("non_cpp_include_in_cli", '#include "secret.md"\n', exempt)):
            with self.subTest(case=label):
                bad_files = dict(files)
                bad_files["bench/swerve/send_target.cc"] += bad_line
                failures, out = run_sources(bad_files, exempt_cli_files=custom_exempt)
                self.assertGreaterEqual(failures, 1, f"{label}: {out}")

        # Dropping one exempted header makes that header stale and fails.
        shrunk = dict(files)
        shrunk["bench/swerve/send_target.cc"] = '#include "talOS/ipc/publisher.h"\n'
        failures, out = run_sources(shrunk, exempt_cli_files=exempt)
        self.assertEqual(failures, 1, out)
        self.assertIn("STALE non-node CLI exemption", out)
        self.assertIn("talOS/hardware/packet.h", out)

    # REQ-SAFE-27
    def test_build_and_bzl_macro_and_copt_bypasses_are_refused(self):
        cases = [
            ("build_local_defines_token_paste", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'local_defines = ["CAT(a,b)=a##b"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_local_defines_digraph_paste", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'local_defines = ["CAT(a,b)=a%:%:b"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_defines_forbidden_symbol", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'defines = ["SEND=talos::detail::NodeAccess::add_hardware_request"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_defines_hardware_access", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'defines = ["MARK=talos::detail::HardwareAccess::note_live_target"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_defines_concat_symbol", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'defines = ["X=" + "Node" + "Access"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_copts_include_adjacent", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'copts = ["-include", "talOS/ipc/publisher.h"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_copts_include_local_header", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'copts = ["-include", "local_bypass.h"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_copts_include_equals", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'copts = ["-include=talOS/node/access.h"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_copts_include_joined", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'copts = ["-includetalOS/devices/access.h"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_copts_imacros", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'copts = ["-imacros", "macros.h"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_copts_trigraphs", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'copts = ["-trigraphs"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("build_defines_forbidden_header_path", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", srcs = ["arm.cc"], '
                    'defines = ["H=\\"talOS/devices/../node/access.h\\""])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/BUILD:1:"),
            ("bzl_wrapper_macro_defines_and_copts", {
                "bench/arm/macros.bzl": (
                    'def robot_lib(name, srcs):\n'
                    '    native.cc_library(\n'
                    '        name = name,\n'
                    '        srcs = srcs,\n'
                    '        local_defines = ["CAT(a,b)=a##b"],\n'
                    '    )\n'
                ),
                "bench/arm/BUILD": (
                    'load(":macros.bzl", "robot_lib")\n'
                    'robot_lib(name = "arm", srcs = ["arm.cc"])\n'
                ),
                "bench/arm/arm.cc": "int ok = 1;\n",
            }, "bench/arm/macros.bzl:5:"),
            ("bzl_wrapper_macro_copts_include", {
                "examples/custom/macros.bzl": (
                    'COPTS = ["-include", "talOS/rtms/rtms.h"]\n'
                ),
                "examples/custom/BUILD": 'cc_library(name = "c", srcs = ["c.cc"])\n',
                "examples/custom/c.cc": "int ok = 1;\n",
            }, "examples/custom/macros.bzl:1:"),
            ("bzl_syntax_error_fails_closed", {
                "bench/arm/broken.bzl": "def broken(:\n",
            }, "bench/arm/broken.bzl:"),
        ]
        for name, files, expected in cases:
            with self.subTest(name=name):
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn(expected, out)

    # REQ-SAFE-27
    def test_indirect_and_non_standard_suffix_files_are_scanned(self):
        cases = [
            ("filegroup_txt_textual_hdrs_include", {
                "bench/arm/BUILD": (
                    'filegroup(name = "fg", srcs = ["wrap.txt"])\n'
                    'cc_library(name = "arm", srcs = ["a.cc"], textual_hdrs = [":fg"])\n'
                ),
                "bench/arm/a.cc": "int ok = 1;\n",
                "bench/arm/wrap.txt": '#include "talOS/node/access.h"\n',
            }, "bench/arm/wrap.txt:1:"),
            ("filegroup_txt_symbol", {
                "bench/arm/BUILD": (
                    'filegroup(name = "fg", srcs = ["wrap.txt"])\n'
                    'cc_library(name = "arm", srcs = ["a.cc"], textual_hdrs = [":fg"])\n'
                ),
                "bench/arm/a.cc": "int ok = 1;\n",
                "bench/arm/wrap.txt": (
                    "void f() { talos::detail::NodeAccess::add_hardware_request(); }\n"
                ),
            }, "bench/arm/wrap.txt:1:"),
            ("wrapper_macro_txt_private_headers", {
                "bench/arm/BUILD": (
                    'load(":m.bzl", "robot_lib")\n'
                    'robot_lib(name = "arm", private_headers = ["wrap.txt"])\n'
                ),
                "bench/arm/m.bzl": "def robot_lib(**kwargs): pass\n",
                "bench/arm/wrap.txt": '#include "talOS/ipc/publisher.h"\n',
            }, "bench/arm/wrap.txt:1:"),
            ("objc_mm_suffix", {
                "bench/arm/BUILD": 'cc_library(name = "arm", srcs = ["bypass.mm"])\n',
                "bench/arm/bypass.mm": (
                    "void f() { talos::detail::NodeAccess::add_hardware_request(); }\n"
                ),
            }, "bench/arm/bypass.mm:1:"),
            ("tlh_suffix", {
                "bench/arm/BUILD": 'cc_library(name = "arm", srcs = ["bypass.tlh"])\n',
                "bench/arm/bypass.tlh": '#include "talOS/devices/access.h"\n',
            }, "bench/arm/bypass.tlh:1:"),
            ("tli_suffix", {
                "bench/arm/BUILD": 'cc_library(name = "arm", srcs = ["bypass.tli"])\n',
                "bench/arm/bypass.tli": '#include "talOS/rtms/rtms.h"\n',
            }, "bench/arm/bypass.tli:1:"),
            ("unreferenced_extensionless_file", {
                "bench/arm/BUILD": self.NODE_BUILD,
                "bench/arm/arm.cc": "int ok = 1;\n",
                "bench/arm/unlisted_hdr": '#include "talOS/node/access.h"\n',
            }, "bench/arm/unlisted_hdr:1:"),
            ("non_cpp_suffix_referenced_in_textual_hdrs", {
                "bench/arm/BUILD": (
                    'cc_library(name = "arm", textual_hdrs = ["sneaky.md"])\n'
                ),
                "bench/arm/sneaky.md": '#include "talOS/node/access.h"\n',
            }, "bench/arm/sneaky.md:1:"),
            ("filegroup_non_cpp_suffix_in_textual_hdrs", {
                "bench/arm/BUILD": (
                    'filegroup(name = "fg", srcs = ["sneaky.md"])\n'
                    'cc_library(name = "arm", textual_hdrs = [":fg"])\n'
                ),
                "bench/arm/sneaky.md": (
                    "void f() { talos::detail::NodeAccess::add_hardware_request(); }\n"
                ),
            }, "bench/arm/sneaky.md:1:"),
        ]
        for name, files, expected in cases:
            with self.subTest(name=name):
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn(expected, out)

    # REQ-SAFE-27
    def test_unreadable_and_non_utf8_files_fail_closed(self):
        def broken_read(path):
            if path.endswith(".cc"):
                raise PermissionError("denied")
            return self.NODE_BUILD

        out = io.StringIO()
        failures = check_layers.run_source_checks(
            ["bench/arm/BUILD", "bench/arm/arm.cc"],
            broken_read,
            exempt_cli_files={},
            out=out,
        )
        self.assertEqual(failures, 1, out.getvalue())
        self.assertIn("fail closed", out.getvalue())

        def non_utf8_read(path):
            if path.endswith(".txt"):
                raise UnicodeDecodeError("utf-8", b"\xff\xfe", 0, 1, "invalid")
            return self.NODE_BUILD

        out = io.StringIO()
        failures = check_layers.run_source_checks(
            ["bench/arm/BUILD", "bench/arm/blob.txt"],
            non_utf8_read,
            exempt_cli_files={},
            out=out,
        )
        self.assertEqual(failures, 1, out.getvalue())
        self.assertIn("fail closed", out.getvalue())

    # REQ-SAFE-27, REQ-CONTRACT-16
    def test_trigraphs_are_refused_in_cpp_and_starlark(self):
        cpp_cases = [
            ("code_token_paste_trigraph",
             "#define CAT(a, b) a??=??=b\n"),
            ("code_directive_trigraph",
             '??=include "talOS/node/access.h"\n'),
            ("block_comment_splice_hiding_call",
             "/* *??/\n"
             "/ void f() { talos::detail::NodeAccess::add_hardware_request(); } /* */\n"),
            ("line_comment_splice_hiding_call",
             "// ??/\n"
             "/*\n"
             "void f() { talos::detail::NodeAccess::add_hardware_request(); }\n"
             "// */\n"),
            ("string_literal_backslash_trigraph_desync",
             'const char* s = "??/"; /* "; '
             'talos::detail::NodeAccess::add_hardware_request(); /* */\n'),
            ("char_literal_backslash_trigraph_desync",
             "char c = '??/'; /* '; "
             "talos::detail::NodeAccess::add_hardware_request(); /* */\n"),
        ]
        for name, body in cpp_cases:
            with self.subTest(name=name):
                files = {
                    "bench/arm/BUILD": self.NODE_BUILD,
                    "bench/arm/arm.cc": body,
                }
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn("bench/arm/arm.cc:", out)

        starlark_cases = [
            ("copts_ftrigraphs",
             'cc_library(name = "arm", srcs = ["arm.cc"], copts = ["-ftrigraphs"])\n'),
            ("copts_double_dash_trigraphs",
             'cc_library(name = "arm", srcs = ["arm.cc"], copts = ["--trigraphs"])\n'),
            ("copts_xclang_ftrigraphs",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-Xclang", "-ftrigraphs"])\n'),
            ("local_defines_trigraph_paste",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'local_defines = ["CAT(a,b)=a??=??=b"])\n'),
        ]
        for name, build in starlark_cases:
            with self.subTest(name=name):
                files = {
                    "bench/arm/BUILD": build,
                    "bench/arm/arm.cc": "int ok = 1;\n",
                }
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn("bench/arm/BUILD:1:", out)

    # REQ-SAFE-27, REQ-CONTRACT-16
    def test_include_path_copts_are_refused_in_starlark(self):
        cases = [
            ("copts_I_joined",
             'cc_library(name = "arm", srcs = ["arm.cc"], copts = ["-ItalOS"])\n'),
            ("copts_I_equals",
             'cc_library(name = "arm", srcs = ["arm.cc"], copts = ["-I=talOS"])\n'),
            ("copts_I_adjacent",
             'cc_library(name = "arm", srcs = ["arm.cc"], copts = ["-I", "talOS"])\n'),
            ("copts_iquote_adjacent",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-iquote", "talOS/node"])\n'),
            ("copts_iquote_joined",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-iquotetalOS/node"])\n'),
            ("copts_isystem_adjacent",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-isystem", "talOS/node"])\n'),
            ("copts_isystem_equals",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-isystem=talOS/node"])\n'),
            ("copts_idirafter_adjacent",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-idirafter", "talOS/node"])\n'),
            ("copts_idirafter_equals",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-idirafter=talOS/node"])\n'),
            ("copts_include_directory_equals",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["--include-directory=talOS"])\n'),
            ("copts_include_directory_adjacent",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["--include-directory", "talOS"])\n'),
            ("copts_include_directory_after",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["--include-directory-after=talOS"])\n'),
            ("copts_wp_I_joined",
             'cc_library(name = "arm", srcs = ["arm.cc"], copts = ["-Wp,-ItalOS"])\n'),
            ("copts_wp_I_comma",
             'cc_library(name = "arm", srcs = ["arm.cc"], copts = ["-Wp,-I,talOS"])\n'),
            ("copts_wp_iquote",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-Wp,-iquote,talOS/node"])\n'),
            ("cxxopts_I_adjacent",
             'cc_library(name = "arm", srcs = ["arm.cc"], cxxopts = ["-I", "talOS"])\n'),
            ("conlyopts_isystem",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'conlyopts = ["-isystem", "talOS"])\n'),
            ("copts_cxx_isystem_adjacent",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-cxx-isystem", "talOS"])\n'),
            ("copts_cxx_isystem_equals",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-cxx-isystem=talOS"])\n'),
            ("copts_fms_extensions",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-fms-extensions"])\n'),
            ("copts_fms_compatibility",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-fms-compatibility"])\n'),
            ("copts_xclang_fms_compatibility",
             'cc_library(name = "arm", srcs = ["arm.cc"], '
             'copts = ["-Xclang", "-fms-compatibility"])\n'),
        ]
        for name, build in cases:
            with self.subTest(name=name):
                files = {
                    "bench/arm/BUILD": build,
                    "bench/arm/arm.cc": "int ok = 1;\n",
                }
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn("bench/arm/BUILD:1:", out)

        # Even in a .bzl macro where the directory is a parameter, `-I` is refused.
        bzl_files = {
            "bench/arm/macros.bzl": (
                'def robot_lib(name, srcs, inc):\n'
                '    native.cc_library(name = name, srcs = srcs, copts = ["-I", inc])\n'
            ),
            "bench/arm/BUILD": (
                'load(":macros.bzl", "robot_lib")\n'
                'robot_lib(name = "arm", srcs = ["arm.cc"], inc = "talOS")\n'
            ),
            "bench/arm/arm.cc": "int ok = 1;\n",
        }
        failures, out = run_sources(bzl_files)
        self.assertGreaterEqual(failures, 1, out)
        self.assertIn("bench/arm/macros.bzl:2:", out)

    # REQ-SAFE-27, REQ-CONTRACT-16
    def test_non_cpp_suffix_and_basename_includes_are_refused(self):
        cases = [
            ("md_quotes", '#include "secret.md"\n'),
            ("json_angles", "#include <pkg/bypass.json>\n"),
            ("txt_quotes", '#include "wrap.txt"\n'),
            ("uppercase_md", '#include "UPPER.MD"\n'),
            ("uppercase_txt", '#include "WRAP.TXT"\n'),
            ("readme_basename", '#include "other/README"\n'),
            ("include_next_toml", '#include_next "bypass.toml"\n'),
            ("digraph_json", '%:include "bypass.json"\n'),
            ("import_forbidden_publisher", '#import "talOS/ipc/publisher.h"\n'),
            ("import_forbidden_access", '#import <talOS/node/access.h>\n'),
            ("digraph_import_access", '%:import "talOS/node/access.h"\n'),
            ("import_non_cpp_md", '#import "secret.md"\n'),
        ]
        for name, body in cases:
            with self.subTest(name=name):
                files = {
                    "bench/arm/BUILD": self.NODE_BUILD,
                    "bench/arm/arm.cc": body,
                }
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn("bench/arm/arm.cc:1:", out)

    # REQ-SAFE-27, REQ-CONTRACT-16
    def test_case_and_nested_talos_segment_include_bypasses_are_refused(self):
        cases = [
            ("nested_talos_exact",
             '#include "a/talOS/b/talOS/node/access.h"\n'),
            ("nested_talos_mixed_case_prefix",
             '#include "a/talos/b/TALOS/rtms/rtms.h"\n'),
            ("lowercase_exact",
             '#include "talos/node/access.h"\n'),
            ("uppercase_exact",
             '#include <TALOS/NODE/INTERNAL.H>\n'),
            ("mixed_case_prefix",
             '#include "Talos/IPC/Publisher.h"\n'),
            ("mixed_case_devices_access",
             '#include "talOS/Devices/Access.h"\n'),
        ]
        for name, body in cases:
            with self.subTest(name=name):
                files = {
                    "bench/arm/BUILD": self.NODE_BUILD,
                    "bench/arm/arm.cc": body,
                }
                failures, out = run_sources(files)
                self.assertGreaterEqual(failures, 1, f"{name}: {out}")
                self.assertIn("bench/arm/arm.cc:1:", out)

        # Starlark defines carrying nested/lowercased forbidden paths fail too.
        starlark_files = {
            "bench/arm/BUILD": (
                'cc_library(name = "arm", srcs = ["arm.cc"], '
                'defines = ["H=\\"a/talOS/b/talos/devices/access.h\\""])\n'
            ),
            "bench/arm/arm.cc": "int ok = 1;\n",
        }
        failures, out = run_sources(starlark_files)
        self.assertGreaterEqual(failures, 1, out)
        self.assertIn("bench/arm/BUILD:1:", out)


if __name__ == "__main__":
    unittest.main()
