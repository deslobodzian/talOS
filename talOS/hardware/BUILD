load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_messages")

# Simulation ground truth (reserved owner `sim`): the simulated robot's true
# field pose and field-frame velocity as geometry.OdometrySample, written only
# by a physics-backed simulator (sim_gateway --physics), never on a robot.
talos_messages(
    name = "sim_messages",
    srcs = [],
    include_prefix = "sim",
    topics = {
        "state": "/sim/state",
    },
    visibility = ["//visibility:public"],
    deps = ["//talOS/msgs:geometry"],
)

cc_library(
    name = "config",
    srcs = ["config.cc"],
    hdrs = ["config.h"],
    visibility = ["//visibility:public"],
    deps = ["//talOS/driver_station"],
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
        "//talOS/schema:identity",
    ],
)

cc_test(
    name = "gateway_test",
    size = "small",
    srcs = ["gateway_test.cc"],
    deps = [
        ":hardware",
        "//talOS/protocol:udp_test_helpers",
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
    deps = [":hardware"],
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

cc_test(
    name = "codec_fuzz_test",
    size = "small",
    srcs = ["codec_fuzz_test.cc"],
    deps = [
        ":hardware",
        "//talOS/protocol:fuzz_corpus",
        "@googletest//:gtest_main",
    ],
)
