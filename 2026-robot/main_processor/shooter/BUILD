load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# ShooterTarget / ShooterState (namespace shooter) and the shooter's topics.
# The operator interface and telemetry consume them. The shooter reads its
# target through talos::Target: teleop and autonomous publish on their own
# lanes, /shooter/target/<source>, and the [robot.targets] policy picks one.
talos_messages(
    name = "messages",
    srcs = ["shooter_message.fbs"],
    topics = {
        "state": "/shooter/state",
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
        "//talOS/mechanisms",
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
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
