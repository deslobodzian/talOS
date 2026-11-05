load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@talos//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

exports_files(["defaults.toml"])

# WristTarget / WristState (namespace wrist) and the wrist's topics.
# Producers write their own lanes, /wrist/target/<source>, through
# talos::TargetLane / talos.TargetLane; the wrist reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["wrist_message.fbs"],
    include_prefix = "wrist",
    topics = {
        "state": "/wrist/state",
    },
)

cc_library(
    name = "wrist",
    hdrs = ["wrist.h"],
    include_prefix = "wrist",
    strip_include_prefix = ".",
    visibility = ["//visibility:private"],
    deps = [
        ":messages",
        "@talos//talOS/devices",
        "@talos//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["wrist.cc"],
    node_name = "wrist",
    deps = [":wrist"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/robot.toml",
        "testdata/wrist.toml",
        ":node",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
        "TALOS_TEST_NODE": "$(rlocationpath :node)",
    },
    # Run from the consuming workspace too (//examples/external_subsystem).
    visibility = ["//visibility:public"],
    deps = [
        ":wrist",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
        "@talos//talOS/devices:testing",
        "@talos//talOS/node:testing",
    ],
)
