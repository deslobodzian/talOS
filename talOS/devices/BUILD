load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_messages")

# Robot hardware for talos::Node: Motor, CANcoder, Gyro, digital/analog I/O,
# encoders and PWM through the bridge, attached as a Node extension, the
# Driver Station the bridge forwards, and target lanes (Target, TargetLane)
# resolved by that Driver Station's mode.
# Hardware wire/packet/access deps stay in implementation_deps so their
# headers are excluded from downstream robot-node compile sandboxes
# (ARCH-2 / RULES.md 2).
cc_library(
    name = "devices",
    srcs = [
        "access.h",
        "devices.cc",
        "target.cc",
    ],
    hdrs = [
        "devices.h",
        "target.h",
        "target_detail.h",
    ],
    implementation_deps = [
        "//talOS/configuration",
        "//talOS/events:events_core",
        "//talOS/hardware",
        "//talOS/hardware:boot_history",
        "//talOS/hardware:bridge_messages",
        "//talOS/introspection:names",
        "//talOS/node:access",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/driver_station",
        "//talOS/node",
        "//talOS/schema:identity",
    ],
)

# Privileged hardware-link plumbing (HardwareAccess, including note_live_target
# and device_id, and DriverStationAccess). Restricted to framework layers only
# so robot node code cannot include or link it.
cc_library(
    name = "access",
    hdrs = ["access.h"],
    visibility = [
        "//talOS/devices:__pkg__",
        "//talOS/mechanisms:__pkg__",
        "//talOS/node_api:__pkg__",
    ],
    deps = [
        ":devices",
        "//talOS/events:events_core",
        "//talOS/hardware",
        "//talOS/schema:identity",
    ],
)

# talos::testing::SimHardware: plays the bridge for a testing::Sim.
cc_library(
    name = "testing",
    testonly = True,
    srcs = ["testing.cc"],
    hdrs = ["testing.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":access",
        ":devices",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/hardware:bridge_messages",
        "//talOS/hardware:can_buses",
        "//talOS/hardware:clock_offset",
        "//talOS/hardware:merge",
        "//talOS/introspection:names",
        "//talOS/node:testing",
        "//talOS/protocol",
    ],
)

cc_test(
    name = "testing_test",
    size = "small",
    srcs = ["testing_test.cc"],
    deps = [
        ":access",
        ":devices",
        ":target_test_message",
        ":testing",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/hardware:bridge_messages",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "devices_test",
    size = "small",
    srcs = ["devices_test.cc"],
    deps = [
        ":devices",
        ":testing",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/hardware:bridge_messages",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
    ],
)

talos_messages(
    name = "target_test_message",
    srcs = ["target_test_message.fbs"],
    include_prefix = "target_test",
    visibility = ["//visibility:private"],
)

cc_test(
    name = "target_test",
    size = "small",
    srcs = ["target_test.cc"],
    deps = [
        ":devices",
        ":target_test_message",
        ":testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/node:access",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
    ],
)
