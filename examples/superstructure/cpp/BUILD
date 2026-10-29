load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

# The three node classes, header-only so the test can host them all in one
# simulated robot. Each <name>.cc is just TALOS_NODE(<Class>).
cc_library(
    name = "mechanisms",
    hdrs = [
        "elevator.h",
        "intake.h",
        "superstructure.h",
    ],
    deps = [
        "//examples/superstructure/messages",
        "//talOS/devices",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "intake",
    srcs = ["intake.cc"],
    node_name = "intake",
    deps = [":mechanisms"],
)

talos_cc_node(
    name = "elevator",
    srcs = ["elevator.cc"],
    node_name = "elevator",
    deps = [":mechanisms"],
)

talos_cc_node(
    name = "superstructure",
    srcs = ["superstructure.cc"],
    node_name = "superstructure",
    deps = [":mechanisms"],
)

cc_test(
    name = "superstructure_test",
    size = "small",
    srcs = ["superstructure_test.cc"],
    data = [
        "//examples/superstructure/config:robot.toml",
        "//examples/superstructure/config:subsystems",
    ],
    env = {"TALOS_TEST_CONFIG": "$(rlocationpath //examples/superstructure/config:robot.toml)"},
    deps = [
        ":mechanisms",
        "//talOS/devices:testing",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
