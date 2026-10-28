load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# The loop a Node runs on, type-erased. Framework-internal.
cc_library(
    name = "loop_port",
    hdrs = ["internal.h"],
    deps = ["//talOS/events:events_core"],
    visibility = ["//visibility:private"],
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
    deps = [
        ":loop_port",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
        "//talOS/events:events_core",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/ipc:ipc",
        "//talOS/process:node_main",
    ],
    visibility = ["//visibility:public"],
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
    deps = [
        ":loop_port",
        ":node",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
    ],
    visibility = ["//visibility:public"],
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
    node_name = "test_node",
    srcs = ["testdata/test_node.cc"],
    deps = [":test_messages"],
    visibility = ["//visibility:private"],
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
        ":node",
        ":test_messages",
        ":testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
