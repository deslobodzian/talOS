load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@talos//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# RollerTarget / RollerState (namespace roller) and the roller's topics.
# Producers write their own lanes, /roller/target/<source>, through
# talos::TargetLane / talos.TargetLane; the roller reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["roller_message.fbs"],
    include_prefix = "roller",
    topics = {
        "state": "/roller/state",
    },
)

cc_library(
    name = "roller",
    hdrs = ["roller.h"],
    include_prefix = "roller",
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
    srcs = ["roller.cc"],
    node_name = "roller",
    deps = [":roller"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/neighbor.toml",
        "testdata/robot.toml",
        "testdata/roller.toml",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
    },
    deps = [
        ":roller",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
        "@talos//talOS/devices:testing",
        "@talos//talOS/node:allocation_probe",
        "@talos//talOS/node:testing",
    ],
)
