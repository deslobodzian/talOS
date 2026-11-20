"""Unit tests for tools/check_layers.py on fixture graphs and C++ sources (no bazel)."""

import collections
import io
import unittest

from tools import check_layers

# label -> (generator macro or None, [deps]). Every label is a rule. Today's
# allowlisted dependencies are present, so the graph passes as it stands.
GRAPH = {
    "//talOS/memory:shared_memory": (None, []),
    "//talOS/rtms:rtms": (None, ["//talOS/memory:shared_memory"]),
    "//talOS/ipc:ipc": (None, ["//talOS/rtms:rtms"]),
    "//talOS/events:events": (None, ["//talOS/rtms:rtms"]),
    "//talOS/hardware:config": (None, []),
    "//talOS/hardware:hardware": (None, ["//talOS/hardware:config"]),
    "//talOS/configuration:configuration": (None, ["//talOS/hardware:config"]),
    "//talOS/node:loop_port": (None, []),
    "//talOS/node:access": (None, []),
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


class FakeBazel:
    """Evaluates the checks' sets over GRAPH, as `bazel query` would."""

    def __init__(self, graph, kinds=None, testonly=None):
        self.graph = graph
        self.kinds = kinds if kinds is not None else KINDS
        self.testonly = testonly if testonly is not None else TESTONLY

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


def run(graph, kinds=None, testonly=None):
    out = io.StringIO()
    return check_layers.run(FakeBazel(graph, kinds, testonly), out), out.getvalue()


def with_edge(frm, to, generator=None):
    graph = {label: (gen, list(deps)) for label, (gen, deps) in GRAPH.items()}
    graph.setdefault(to, (generator, []))
    graph.setdefault(frm, (None, []))[1].append(to)
    return graph


def run_sources(file_map, exempt_cli_files=()):
    out = io.StringIO()
    failures = check_layers.run_source_checks(
        sorted(file_map.keys()),
        lambda p: file_map[p],
        exempt_cli_files=exempt_cli_files,
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
                ("//talOS/node:node", "//talOS/bridge:merge"),
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
                 "//talOS/node:loop_port"),
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

    def test_allowlist_only_shrinks(self):
        # An allowlisted dependency that is gone fails until its entry is
        # deleted, so the list cannot hold stale exceptions.
        graph = dict(GRAPH)
        graph["//talOS/devices:testing"] = (None, ["//talOS/devices:devices"])
        failures, out = run(graph)
        self.assertEqual(failures, 1, out)
        self.assertIn("STALE allowlist entry talOS/devices -> talOS/bridge",
                      out)


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
                'inline constexpr const char* kDoc = '
                '"detail::NodeAccess::add_hardware_request";\n'
            ),
            "bench/arm/arm.cc": (
                '#include "arm/arm.h"\n'
                'const char* kRaw = R"(\n'
                '#include "talOS/node/access.h"\n'
                'detail::NodeAccess::add_hardware_request\n'
                ')";\n'
            ),
            # Non-node integration harness outside talOS/ is not scanned.
            "bench/integration/BUILD": (
                'cc_test(name = "it", srcs = ["it_test.cc"])\n'
            ),
            "bench/integration/it_test.cc": (
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
    def test_subdirectory_and_aliased_load_and_root_package_are_scanned(self):
        # Subdirectory without its own BUILD belongs to the parent node package.
        files = {
            "bench/arm/BUILD": (
                'load("//talOS/build:contract.bzl", my_node = "talos_cc_node")\n'
                'my_node(name = "node", srcs = ["arm.cc"], node_name = "arm")\n'
            ),
            "bench/arm/detail/helper.inl": '#include "talOS/node/access.h"\n',
        }
        failures, out = run_sources(files)
        self.assertEqual(failures, 1, out)
        self.assertIn("bench/arm/detail/helper.inl:1:", out)

        # Standalone node package at the workspace root (empty package dir).
        root_files = {
            "BUILD": self.NODE_BUILD,
            "wrist.h": '#include <talOS/rtms/rtms.h>\n',
        }
        failures, out = run_sources(root_files)
        self.assertEqual(failures, 1, out)
        self.assertIn("wrist.h:1:", out)

    # REQ-SAFE-27
    def test_non_node_cli_exemption_scope_and_staleness(self):
        exempt = {"bench/swerve/send_target.cc": "operator CLI"}
        # Transport includes in the exempted CLI pass.
        files = {
            "bench/swerve/BUILD": self.NODE_BUILD,
            "bench/swerve/send_target.cc": (
                '#include "talOS/ipc/publisher.h"\n'
                '#include "talOS/hardware/packet.h"\n'
            ),
        }
        failures, out = run_sources(files, exempt_cli_files=exempt)
        self.assertEqual(failures, 0, out)

        # Even in an exempted CLI, NodeAccess / node/access.h is refused.
        files["bench/swerve/send_target.cc"] += (
            '#include "talOS/node/access.h"\n'
            'void f() { detail::NodeAccess::add_hardware_request(); }\n'
        )
        failures, out = run_sources(files, exempt_cli_files=exempt)
        self.assertGreaterEqual(failures, 2, out)
        self.assertIn("talOS/node/access.h", out)
        self.assertIn("NodeAccess", out)

        # Removing the exempted includes makes the exemption stale and fails.
        files["bench/swerve/send_target.cc"] = '#include <cstdio>\n'
        failures, out = run_sources(files, exempt_cli_files=exempt)
        self.assertEqual(failures, 1, out)
        self.assertIn("STALE non-node CLI exemption", out)

    # REQ-SAFE-27
    def test_unreadable_file_fails_closed(self):
        def broken_read(path):
            if path.endswith(".cc"):
                raise PermissionError("denied")
            return self.NODE_BUILD

        out = io.StringIO()
        failures = check_layers.run_source_checks(
            ["bench/arm/BUILD", "bench/arm/arm.cc"],
            broken_read,
            exempt_cli_files=(),
            out=out,
        )
        self.assertEqual(failures, 1, out.getvalue())
        self.assertIn("fail closed", out.getvalue())


if __name__ == "__main__":
    unittest.main()
