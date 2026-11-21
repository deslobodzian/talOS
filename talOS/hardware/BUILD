load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_messages")

# BridgeStatus (namespace bridge) and its topic (/hw/status): lives in
# //talOS/hardware/status so its generated topics.h does not collide with
# :sim_messages below.
alias(
    name = "bridge_messages",
    actual = "//talOS/hardware/status:messages",
    visibility = ["//visibility:public"],
)

alias(
    name = "bridge_messages_py",
    actual = "//talOS/hardware/status:messages_py",
    visibility = ["//visibility:public"],
)

# The gateway boots a stream has been in, current and recently retired: the
# State ordering gate (via clock_offset) and every Driver Station consumer.
cc_library(
    name = "boot_history",
    hdrs = ["boot_history.h"],
    visibility = ["//talOS:__subpackages__"],
)

# The gateway-to-node clock mapping the bridge publishes on /hw/status, also
# run by talos::testing::SimHardware when it plays the bridge.
cc_library(
    name = "clock_offset",
    srcs = ["clock_offset.cc"],
    hdrs = ["clock_offset.h"],
    visibility = ["//talOS:__subpackages__"],
    deps = [":boot_history"],
)

# The /hw/status CAN bus table, assembled from the one bus slot each State
# reports, also run by talos::testing::SimHardware when it plays the bridge.
cc_library(
    name = "can_buses",
    hdrs = ["can_buses.h"],
    visibility = ["//talOS:__subpackages__"],
    deps = [
        ":bridge_messages",
        ":hardware",
    ],
)

# The request merge rules (owner timeout, admission, merged observation),
# also run by talos::testing::SimHardware when it plays the bridge.
cc_library(
    name = "merge",
    srcs = ["merge.cc"],
    hdrs = ["merge.h"],
    visibility = ["//talOS:__subpackages__"],
)

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

# Pure wire codecs, packets, gating predicates and state machine (no UDP
# Endpoint or sockets), so log readers and studio/review decode recorded
# hardware packets without linking transport code (REQ-SAFE-35).
cc_library(
    name = "codecs",
    srcs = [
        "gateway.cc",
        "messages.cc",
    ],
    hdrs = [
        "config.h",
        "config_error.h",
        "config_wire.h",
        # Invalid samples and total feedback loss, shared with
        # testing::SimHardware.
        "feedback.h",
        # Shared gating predicates, for the gateway and for hardware-owning
        # nodes: pure functions over State, so gate_test needs no loop and no
        # transport.
        "gate.h",
        "gateway.h",
        "messages.h",
        "packet.h",
    ],
    visibility = [
        "//studio:__subpackages__",
        "//talOS:__subpackages__",
        "//tools:__pkg__",
    ],
    deps = [
        ":config",
        "//talOS/protocol:frame",
        "//talOS/schema:identity",
    ],
)

cc_library(
    name = "hardware",
    hdrs = [
        "endpoint.h",
        "sim_backend.h",
        "sim_physics.h",
    ],
    visibility = [
        "//bench/main_processor/swerve_module:__pkg__",
        "//controller_processor/rio:__pkg__",
        "//studio:__subpackages__",
        "//talOS:__subpackages__",
        "//tools:__pkg__",
    ],
    deps = [
        ":codecs",
        ":config",
        "//talOS/protocol",
        "//talOS/schema:identity",
    ],
)

# The swerve configuration, recording backend and command builder the
# gateway tests share.
cc_library(
    name = "gateway_test_support",
    testonly = True,
    hdrs = ["gateway_test_support.h"],
    deps = [":hardware"],
)

cc_test(
    name = "gateway_test",
    size = "small",
    srcs = ["gateway_test.cc"],
    deps = [
        ":gateway_test_support",
        ":hardware",
        "//talOS/protocol",
        "@googletest//:gtest_main",
    ],
)

# The Endpoint over a loopback UDP socket: its waits poll the socket on the
# real clock, so it runs in the integration lane, apart from gateway_test.
cc_test(
    name = "gateway_endpoint_test",
    size = "small",
    srcs = ["gateway_endpoint_test.cc"],
    tags = ["integration"],
    deps = [
        ":gateway_test_support",
        ":hardware",
        "//talOS/protocol",
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

# The State and Command bytes of the current hardware protocol: an encoding
# change fails it until kProtocolVersion is bumped (REQ-OPS-51).
cc_test(
    name = "wire_golden_test",
    size = "small",
    srcs = ["wire_golden_test.cc"],
    deps = [
        ":hardware",
        "//talOS/protocol",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "gate_test",
    size = "small",
    srcs = ["gate_test.cc"],
    deps = [
        ":hardware",
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

cc_test(
    name = "clock_offset_test",
    size = "small",
    srcs = ["clock_offset_test.cc"],
    deps = [
        ":clock_offset",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "can_buses_test",
    size = "small",
    srcs = ["can_buses_test.cc"],
    deps = [
        ":bridge_messages",
        ":can_buses",
        ":hardware",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "merge_test",
    size = "small",
    srcs = ["merge_test.cc"],
    deps = [
        ":merge",
        "@googletest//:gtest_main",
    ],
)
