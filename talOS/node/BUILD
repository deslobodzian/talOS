load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# The loop a Node runs on, type-erased. Framework-internal.
cc_library(
    name = "loop_port",
    hdrs = ["internal.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//talOS/events:events_core",
        "//talOS/introspection:describe",
        "//talOS/schema:identity",
    ],
)

# Safe, non-privileged thunks behind node.h's templates.
cc_library(
    name = "node_detail",
    hdrs = ["node_detail.h"],
    visibility = ["//visibility:private"],
    deps = ["//talOS/schema:identity"],
)

# Private header target for node.cc's implementation_deps so talOS/node/access.h
# never enters downstream robot-node compile sandboxes (ARCH-2 / RULES.md 2).
cc_library(
    name = "access_hdr",
    hdrs = ["access.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":node_detail",
        "//talOS/schema:identity",
    ],
)

# The node authoring API: talos::Node, Subscription, Publisher, StateMachine,
# talos::Run and TALOS_NODE. Generic: no hardware (see //talOS/devices).
# Internal framework deps stay in implementation_deps so their headers are
# excluded from downstream robot-node compile sandboxes (ARCH-2 / RULES.md 2).
cc_library(
    name = "node",
    srcs = ["node.cc"],
    hdrs = ["node.h"],
    implementation_deps = [
        ":access_hdr",
        ":loop_port",
        "//talOS/configuration",
        "//talOS/events",
        "//talOS/events:input_replay",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/introspection:reporter",
        "//talOS/process",
        "//talOS/rtms",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":node_detail",
        "//talOS/events:events_core",
        "//talOS/ipc:concepts",
        "//talOS/schema:identity",
    ],
)

# Privileged Node framework hooks (NodeAccess: add_hardware_request,
# mark_untargeted_outputs, create/start, declared_graph, add_observer).
# Restricted to framework layers only so robot node code cannot link it.
cc_library(
    name = "access",
    hdrs = ["access.h"],
    visibility = [
        "//talOS/devices:__pkg__",
        "//talOS/launcher:__pkg__",
        "//talOS/node:__pkg__",
        "//talOS/node_api:__pkg__",
        "//talOS/telemetry:__pkg__",
        "//talOS/teleop:__pkg__",
    ],
    deps = [
        ":node",
        "//talOS/schema:identity",
    ],
)

# talos::testing::Sim: several nodes on one virtual clock.
cc_library(
    name = "testing",
    testonly = True,
    srcs = ["testing.cc"],
    hdrs = [
        "sim_access.h",
        "testing.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":access",
        ":loop_port",
        ":node",
        "//talOS/configuration",
        "//talOS/events",
        "//talOS/events:events_core",
    ],
)

# talos::testing::AllocationProbe: counts one thread's heap allocations, for
# the RULES.md 3 check that a node's update() allocates nothing. On glibc it
# replaces malloc, so it links into the test binary itself.
cc_library(
    name = "allocation_probe",
    testonly = True,
    srcs = ["allocation_probe.cc"],
    hdrs = ["allocation_probe.h"],
    linkstatic = True,
    visibility = ["//visibility:public"],
    alwayslink = True,
)

cc_test(
    name = "allocation_probe_test",
    size = "small",
    srcs = ["allocation_probe_test.cc"],
    deps = [
        ":allocation_probe",
        "@googletest//:gtest_main",
    ],
)

# RULES.md 3 for the event log's gap path (talOS/events may not depend on
# the probe's layer).
cc_test(
    name = "log_writer_allocation_test",
    size = "small",
    srcs = ["log_writer_allocation_test.cc"],
    deps = [
        ":allocation_probe",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "@googletest//:gtest_main",
    ],
)

talos_messages(
    name = "test_messages",
    srcs = ["node_test_message.fbs"],
    include_prefix = "node_test",
    visibility = ["//visibility:private"],
)

# A small real node binary, for the --describe and log/replay tests.
talos_cc_node(
    name = "test_node",
    srcs = ["testdata/test_node.cc"],
    node_name = "test_node",
    visibility = ["//visibility:private"],
    deps = [":test_messages"],
)

# A node that reads the declared graph, for the --graph tests.
talos_cc_node(
    name = "graph_node",
    srcs = ["testdata/graph_node.cc"],
    node_name = "graph_node",
    visibility = ["//visibility:private"],
    deps = [
        ":access",
        "//talOS/introspection:describe",
    ],
)

# A node that prints Node::describing() from its constructor.
talos_cc_node(
    name = "describing_node",
    srcs = ["testdata/describing_node.cc"],
    node_name = "describing_node",
    visibility = ["//visibility:private"],
)

# A node whose constructor blocks until stdin closes and whose loop has
# nothing to run, for the stop tests: during construction, and while idle.
talos_cc_node(
    name = "blocked_node",
    srcs = ["testdata/blocked_node.cc"],
    node_name = "blocked_node",
    visibility = ["//visibility:private"],
    deps = ["//talOS/process"],
)

# Integration tier (fork/exec, signals, a node run on the real clock, the live
# registry): node binaries run as the launcher runs them, out of node_test's
# component lane.
cc_test(
    name = "node_binary_test",
    size = "small",
    srcs = ["node_binary_test.cc"],
    data = [
        "testdata/robot.toml",
        ":blocked_node",
        ":describing_node",
        ":graph_node",
        ":test_node",
    ],
    env = {
        "TALOS_TEST_NODE": "$(rlocationpath :test_node)",
        "TALOS_BLOCKED_NODE": "$(rlocationpath :blocked_node)",
        "TALOS_DESCRIBING_NODE": "$(rlocationpath :describing_node)",
        "TALOS_GRAPH_NODE": "$(rlocationpath :graph_node)",
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
    },
    tags = ["integration"],
    deps = [
        ":access",
        ":node",
        ":test_messages",
        ":testing",
        "//talOS/events",
        "//talOS/events:events_log",
        "//talOS/events:gap_testing",
        "//talOS/events:input_replay",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/introspection:registry",
        "//talOS/introspection:reporter",
        "//talOS/process",
        "//talOS/rtms",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = ["testdata/robot.toml"],
    env = {"TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)"},
    deps = [
        ":access",
        ":loop_port",
        ":node",
        ":test_messages",
        ":testing",
        "//talOS/configuration",
        "//talOS/events",
        "//talOS/introspection:describe",
        "//talOS/memory:shared_memory",
        "//talOS/process",
        "//talOS/rtms",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
