load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# ShooterTarget / ShooterState (namespace shooter) and the shooter's topics.
# The arbiter, operator interface and telemetry consume them.
talos_messages(
    name = "messages",
    srcs = ["shooter_message.fbs"],
    hdrs = ["topics.h"],
)

cc_library(
    name = "shooter",
    hdrs = ["shooter.h"],
    strip_include_prefix = ".",
    include_prefix = "shooter",
    deps = [
        ":messages",
        "//talOS/devices",
        "//talOS/node",
    ],
    visibility = ["//visibility:private"],
)

talos_cc_node(
    name = "node",
    node_name = "shooter",
    srcs = ["shooter.cc"],
    deps = [":shooter"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/robot.toml",
        "testdata/shooter.toml",
        ":node",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
        "TALOS_TEST_NODE": "$(rlocationpath :node)",
    },
    deps = [
        ":shooter",
        "//talOS/devices:testing",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
