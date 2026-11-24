load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# FlywheelTarget / FlywheelState (namespace flywheel) and the flywheel's topics.
# Producers write their own lanes, /flywheel/target/<source>, through
# talos::TargetLane / talos.TargetLane; the flywheel reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["flywheel_message.fbs"],
    include_prefix = "flywheel",
    topics = {
        "state": "/flywheel/state",
    },
)

cc_library(
    name = "flywheel",
    hdrs = ["flywheel.h"],
    include_prefix = "flywheel",
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
    srcs = ["flywheel.cc"],
    node_name = "flywheel",
    deps = [":flywheel"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/flywheel.toml",
        "testdata/neighbor.toml",
        "testdata/robot.toml",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
    },
    deps = [
        ":flywheel",
        "//talOS/devices:testing",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
