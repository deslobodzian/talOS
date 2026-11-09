load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

talos_messages(
    name = "messages",
    srcs = ["motor_message.fbs"],
    topics = {"state": "/motor/state"},
)

cc_library(
    name = "motor",
    hdrs = ["motor.h"],
    include_prefix = "motor",
    strip_include_prefix = ".",
    deps = [
        ":messages",
        "//talOS/bridge:messages",
        "//talOS/devices",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["motor.cc"],
    node_name = "motor",
    deps = [":motor"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = ["//bench/main_processor/configuration:robot_config"],
    deps = [
        ":motor",
        "//talOS/devices:testing",
        "//talOS/node:testing",
        "//talOS/teleop",
        "@googletest//:gtest_main",
    ],
)
