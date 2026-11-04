load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# GripperTarget / GripperState (namespace gripper) and the gripper's topics.
# Producers write their own lanes, /gripper/target/<source>, through
# talos::TargetLane / talos.TargetLane; the gripper reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["gripper_message.fbs"],
    include_prefix = "gripper",
    topics = {
        "state": "/gripper/state",
    },
)

cc_library(
    name = "gripper",
    hdrs = ["gripper.h"],
    include_prefix = "gripper",
    strip_include_prefix = ".",
    visibility = ["//visibility:private"],
    deps = [
        ":messages",
        "//talOS/devices",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["gripper.cc"],
    node_name = "gripper",
    deps = [":gripper"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/gripper.toml",
        "testdata/robot.toml",
        ":node",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
        "TALOS_TEST_NODE": "$(rlocationpath :node)",
    },
    deps = [
        ":gripper",
        "//talOS/devices:testing",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# Golden copy of //tools:new_subsystem output; only this target is not generated.
filegroup(
    name = "golden",
    srcs = glob(["**"]),
    visibility = ["//tools:__pkg__"],
)
