"""Unit tests for tools/check_layers.py on a fixture graph (no bazel)."""

import collections
import io
import unittest

from tools import check_layers

# label -> (generator macro or None, [deps]). Every label is a rule. Today's
# allowlisted dependencies are present, so the graph passes as it stands.
GRAPH = {
    "//talOS/memory:shared_memory": (None, []),
    "//talOS/rtms:rtms": (None, ["//talOS/memory:shared_memory"]),
    "//talOS/events:events": (None, ["//talOS/rtms:rtms"]),
    "//talOS/hardware:config": (None, []),
    "//talOS/configuration:configuration": (None, ["//talOS/hardware:config"]),
    "//talOS/node:node": (None, ["//talOS/events:events",
                                 "//talOS/configuration:configuration"]),
    "//talOS/bridge:merge": (None, ["//talOS/hardware:config"]),
    "//talOS/devices:devices": (None, ["//talOS/node:node"]),
    "//talOS/devices:testing": (None, ["//talOS/devices:devices",
                                       "//talOS/bridge:merge"]),
    "//talOS/node_api:libtalos_node.so": (None, ["//talOS/devices:devices"]),
    "//talOS/teleop:libteleop.so": ("talos_cc_shared_node",
                                    ["//talOS/devices:devices"]),
    "//talOS/python:talos": (None, ["//talOS/node_api:libtalos_node.so",
                                    "//talOS/teleop:libteleop.so"]),
    "//talOS/geometry:geometry": (None, []),
    "//talOS/vision:vision": (None, ["//talOS/geometry:geometry"]),
    "//2026-robot/main_processor/drivetrain:messages": ("talos_messages", []),
    "//2026-robot/main_processor/drivetrain:drivetrain": (
        None, ["//2026-robot/main_processor/drivetrain:messages",
               "//talOS/devices:devices"]),
    "//2026-robot/main_processor/drivetrain:node": (
        "talos_cc_node", ["//2026-robot/main_processor/drivetrain:drivetrain"]),
    "//2026-robot/main_processor/driver:driver": (
        None, ["//2026-robot/main_processor/drivetrain:messages",
               "//talOS/devices:devices"]),
    "//2026-robot/main_processor/driver:node": (
        "talos_cc_node", ["//2026-robot/main_processor/driver:driver"]),
    "//2026-robot/main_processor/driver:node_test": (
        None, ["//2026-robot/main_processor/driver:driver",
               "//2026-robot/main_processor/drivetrain:drivetrain"]),
    "//examples/superstructure/cpp:mechanisms": (
        None, ["//talOS/devices:devices"]),
    "//examples/superstructure/cpp:libmechanisms.so": (
        "talos_cc_shared_node", ["//examples/superstructure/cpp:mechanisms"]),
    "//examples/superstructure/py:elevator": (
        "talos_py_node", ["//examples/superstructure/cpp:libmechanisms.so",
                          "//talOS/python:talos"]),
}


def package(label):
    return label[2:].split(":")[0]


def matches(label, pattern):
    if pattern.endswith(":*"):
        return package(label) == pattern[:-2]
    root = pattern.removesuffix("/...")
    return package(label) == root or package(label).startswith(root + "/")


class FakeBazel:
    """Evaluates the checks' sets over GRAPH, as `bazel query` would."""

    def __init__(self, graph):
        self.graph = graph

    def members(self, target_set):
        return {label for label, (generator, _) in self.graph.items()
                if any(matches(label, p) for p in target_set.include)
                and not any(matches(label, p) for p in target_set.exclude)
                and generator not in target_set.except_generators}

    def node_packages(self):
        return sorted({package(label)
                       for label, (generator, _) in self.graph.items()
                       if generator in check_layers.NODE_GENERATORS
                       and package(label).split("/")[0]
                       in check_layers.NODE_ROOTS})

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


def run(graph):
    out = io.StringIO()
    return check_layers.run(FakeBazel(graph), out), out.getvalue()


def with_edge(frm, to, generator=None):
    graph = {label: (gen, list(deps)) for label, (gen, deps) in GRAPH.items()}
    graph.setdefault(to, (generator, []))
    graph.setdefault(frm, (None, []))[1].append(to)
    return graph


class CheckLayersTest(unittest.TestCase):
    # REQ-CONTRACT-15, REQ-CONTRACT-16
    def test_each_forbidden_dependency_fails_with_its_path(self):
        for frm, to in (
                ("//talOS/rtms:rtms", "//talOS/events:events"),
                ("//talOS/events:events", "//talOS/devices:devices"),
                ("//talOS/node:node", "//talOS/devices:devices"),
                ("//talOS/node:node", "//talOS/bridge:merge"),
                ("//talOS/events:events", "//talOS/hardware:config"),
                ("//talOS/devices:devices", "//talOS/teleop:libteleop.so"),
                ("//talOS/launcher:launcher",
                 "//2026-robot/main_processor/driver:driver"),
                ("//talOS/telemetry:node", "//studio:bridge"),
                ("//talOS/geometry:geometry", "//talOS/vision:vision"),
                ("//talOS/python:talos", "//talOS/node:node"),
                ("//2026-robot/main_processor/drivetrain:drivetrain",
                 "//2026-robot/main_processor/driver:driver"),
                ("//examples/superstructure/py:elevator",
                 "//examples/superstructure/cpp:mechanisms")):
            with self.subTest(frm=frm, to=to):
                failures, out = run(with_edge(frm, to))
                # A lower layer's edge can also fail the layers that reach it.
                self.assertGreaterEqual(failures, 1, out)
                # The path (or, for a direct check, the dependency) names
                # the forbidden target.
                self.assertIn(f"  {to}\n", out)

    # REQ-CONTRACT-16
    def test_allowed_dependencies_pass(self):
        # Today's graph with its allowlisted dependencies, plus a dependency
        # through a gateway, on messages, or on a node exported for
        # talos.cpp_node.
        for frm, to, generator in (
                ("//talOS/python:talos", "//talOS/msgs:control_py", None),
                ("//2026-robot/main_processor/drivetrain:drivetrain",
                 "//examples/superstructure/cpp:libmechanisms.so",
                 "talos_cc_shared_node"),
                ("//examples/superstructure/py:elevator",
                 "//2026-robot/main_processor/drivetrain:messages",
                 "talos_messages")):
            with self.subTest(frm=frm, to=to):
                failures, out = run(with_edge(frm, to, generator))
                self.assertEqual(failures, 0, out)

    def test_allowlist_only_shrinks(self):
        # An allowlisted dependency that is gone fails until its entry is
        # deleted, so the list cannot hold stale exceptions.
        graph = dict(GRAPH)
        graph["//talOS/devices:testing"] = (None, ["//talOS/devices:devices"])
        failures, out = run(graph)
        self.assertEqual(failures, 1, out)
        self.assertIn("STALE allowlist entry talOS/devices -> talOS/bridge",
                      out)


if __name__ == "__main__":
    unittest.main()
