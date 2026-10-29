load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# The loop a Node runs on, type-erased. Framework-internal.
cc_library(
    name = "loop_port",
    hdrs = ["internal.h"],
    visibility = ["//visibility:private"],
    deps = ["//talOS/events:events_core"],
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
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/introspection:reporter",
        "//talOS/ipc",
        "//talOS/process",
        "//talOS/rtms",
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

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/robot.toml",
        ":test_node",
    ],
    env = {
        "TALOS_TEST_NODE": "$(rlocationpath :test_node)",
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
    },
    deps = [
        ":loop_port",
        ":node",
        ":test_messages",
        ":testing",
        "//talOS/configuration",
        "//talOS/events",
        "//talOS/memory:shared_memory",
        "//talOS/rtms",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
