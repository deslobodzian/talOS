load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# The localizer's topic. Its state is the shared geometry.PoseEstimate and its
# camera inputs are geometry.PoseObservation (//talOS/msgs:geometry), so the
# package declares no schema of its own. Consumers find the pose through
# config (telemetry's pose_topic), not through this package.
talos_messages(
    name = "messages",
    srcs = [],
    include_prefix = "localizer",
    topics = {
        "state": "/localizer/state",
    },
    deps = ["//talOS/msgs:geometry"],
)

cc_library(
    name = "localizer",
    hdrs = ["localizer.h"],
    include_prefix = "localizer",
    strip_include_prefix = ".",
    visibility = ["//visibility:private"],
    deps = [
        ":messages",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["localizer.cc"],
    node_name = "localizer",
    deps = [":localizer"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/robot.toml",
        ":node",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
        "TALOS_TEST_NODE": "$(rlocationpath :node)",
    },
    deps = [
        ":localizer",
        "//talOS/devices:testing",
        "//talOS/node:allocation_probe",
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
