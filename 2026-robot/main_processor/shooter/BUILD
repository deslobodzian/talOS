load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# ShooterTarget / ShooterState (namespace shooter) and the shooter's topics.
# The arbiter, operator interface and telemetry consume them. /shooter/target
# has one writer, the arbiter; teleop and autonomous publish on their own
# lanes and the arbiter picks one.
talos_messages(
    name = "messages",
    srcs = ["shooter_message.fbs"],
    topics = {
        "state": "/shooter/state",
        "target": "/shooter/target",
        "teleop_target": "/shooter/target/teleop",
        "auto_target": "/shooter/target/auto",
    },
)

cc_library(
    name = "shooter",
    hdrs = ["shooter.h"],
    include_prefix = "shooter",
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
    srcs = ["shooter.cc"],
    node_name = "shooter",
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
