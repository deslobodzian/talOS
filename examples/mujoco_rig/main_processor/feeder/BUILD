load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# FeederTarget / FeederState (namespace feeder) and the feeder's topics.
# Producers write their own lanes, /feeder/target/<source>, through
# talos::TargetLane / talos.TargetLane; the feeder reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["feeder_message.fbs"],
    include_prefix = "feeder",
    topics = {
        "state": "/feeder/state",
    },
)

cc_library(
    name = "feeder",
    hdrs = ["feeder.h"],
    include_prefix = "feeder",
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
    srcs = ["feeder.cc"],
    node_name = "feeder",
    deps = [":feeder"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/feeder.toml",
        "testdata/neighbor.toml",
        "testdata/robot.toml",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
    },
    deps = [
        ":feeder",
        "//talOS/devices:testing",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
