load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "config",
    srcs = ["config.cc"],
    hdrs = ["config.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hardware",
    srcs = [
        "gateway.cc",
        "messages.cc",
    ],
    hdrs = [
        "config.h",
        "config_error.h",
        "config_wire.h",
        "endpoint.h",
        "gateway.h",
        "messages.h",
        "packet.h",
        "sim_backend.h",
        "sim_physics.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":config",
        "//talOS/protocol",
    ],
)

cc_test(
    name = "gateway_test",
    size = "small",
    srcs = ["gateway_test.cc"],
    deps = [
        ":hardware",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "gateway_scenario_test",
    size = "small",
    srcs = ["gateway_scenario_test.cc"],
    deps = [
        ":hardware",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "sim_physics_test",
    size = "small",
    srcs = ["sim_physics_test.cc"],
    deps = [
        ":hardware",
        "@googletest//:gtest_main",
    ],
)

# Shared gating predicates for hardware-owning subsystem nodes: pure
# functions over State, so the test needs no loop and no transport.
cc_library(
    name = "gate",
    hdrs = ["gate.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":hardware",
        "//talOS/events:events_core",
    ],
)

cc_test(
    name = "gate_test",
    size = "small",
    srcs = ["gate_test.cc"],
    deps = [
        ":gate",
        "@googletest//:gtest_main",
    ],
)
