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

# The node authoring API: talos::Node, Subscription, Publisher, StateMachine,
# talos::Run and TALOS_NODE. Generic: no hardware (see //talOS/devices).
cc_library(
    name = "node",
    srcs = ["node.cc"],
    hdrs = [
        "access.h",
        "node.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":loop_port",
        "//talOS/configuration",
        "//talOS/events",
        "//talOS/events:events_core",
        "//talOS/events:input_replay",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/introspection:reporter",
        "//talOS/ipc",
        "//talOS/process",
        "//talOS/rtms",
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
        ":loop_port",
        ":node",
        "//talOS/configuration",
        "//talOS/events",
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
    alwayslink = True,
    linkstatic = True,
    visibility = ["//visibility:public"],
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
    deps = ["//talOS/introspection:describe"],
)

# A node that prints Node::describing() from its constructor.
talos_cc_node(
    name = "describing_node",
    srcs = ["testdata/describing_node.cc"],
    node_name = "describing_node",
    visibility = ["//visibility:private"],
)

# A node whose constructor blocks until stdin closes, for the stop-during-
# construction test.
talos_cc_node(
    name = "blocked_node",
    srcs = ["testdata/blocked_node.cc"],
    node_name = "blocked_node",
    visibility = ["//visibility:private"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
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
    deps = [
        ":loop_port",
        ":node",
        ":test_messages",
        ":testing",
        "//talOS/configuration",
        "//talOS/events",
        "//talOS/events:events_log",
        "//talOS/introspection:describe",
        "//talOS/memory:shared_memory",
        "//talOS/process",
        "//talOS/rtms",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
