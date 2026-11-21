load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

talos_messages(
    name = "messages",
    srcs = ["module_message.fbs"],
    topics = {"state": "/swerve_module/state"},
)

cc_library(
    name = "swerve_module",
    hdrs = ["swerve_module.h"],
    include_prefix = "swerve_module",
    strip_include_prefix = ".",
    deps = [
        ":messages",
        "//talOS/devices",
        "//talOS/geometry",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["swerve_module.cc"],
    node_name = "swerve_module",
    deps = [":swerve_module"],
)

# Dev tool: publish a module target lane by hand (sim/bench only: it bypasses
# the operator interface). tools/test_rio_in_the_loop.py drives the module
# with it.
cc_binary(
    name = "send_target",
    srcs = ["send_target.cc"],
    deps = [
        ":messages",
        "//talOS/devices",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/introspection:names",
        "//talOS/ipc",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "//bench/main_processor/configuration:robot_config",
    ],
    deps = [
        ":swerve_module",
        # The motor's lane type only: the teleop case stands in a reader for
        # the bench motor's bound lane.
        "//bench/main_processor/motor:messages",
        "//talOS/devices:testing",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "//talOS/teleop:testing",
        "@googletest//:gtest_main",
    ],
)
