load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# PositionRawCppTarget / PositionRawCppState (namespace position_raw_cpp) and the position_raw_cpp's topics.
# Producers write their own lanes, /position_raw_cpp/target/<source>, through
# talos::TargetLane / talos.TargetLane; the position_raw_cpp reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["position_raw_cpp_message.fbs"],
    include_prefix = "position_raw_cpp",
    topics = {
        "state": "/position_raw_cpp/state",
    },
)

cc_library(
    name = "position_raw_cpp",
    hdrs = ["position_raw_cpp.h"],
    include_prefix = "position_raw_cpp",
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
    srcs = ["position_raw_cpp.cc"],
    node_name = "position_raw_cpp",
    deps = [":position_raw_cpp"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/neighbor.toml",
        "testdata/position_raw_cpp.toml",
        "testdata/robot.toml",
        ":node",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
        "TALOS_TEST_NODE": "$(rlocationpath :node)",
    },
    deps = [
        ":position_raw_cpp",
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
