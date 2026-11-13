load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# PositionCppTarget / PositionCppState (namespace position_cpp) and the position_cpp's topics.
# Producers write their own lanes, /position_cpp/target/<source>, through
# talos::TargetLane / talos.TargetLane; the position_cpp reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["position_cpp_message.fbs"],
    include_prefix = "position_cpp",
    topics = {
        "state": "/position_cpp/state",
    },
)

cc_library(
    name = "position_cpp",
    hdrs = ["position_cpp.h"],
    include_prefix = "position_cpp",
    strip_include_prefix = ".",
    visibility = ["//visibility:private"],
    deps = [
        ":messages",
        "//talOS/devices",
        "//talOS/mechanisms",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["position_cpp.cc"],
    node_name = "position_cpp",
    deps = [":position_cpp"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/neighbor.toml",
        "testdata/position_cpp.toml",
        "testdata/robot.toml",
        ":node",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
        "TALOS_TEST_NODE": "$(rlocationpath :node)",
    },
    deps = [
        ":position_cpp",
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
