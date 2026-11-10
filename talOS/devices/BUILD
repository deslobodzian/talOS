load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_messages")

# Robot hardware for talos::Node: Motor, CANcoder, Gyro, digital/analog I/O,
# encoders and PWM through the bridge, attached as a Node extension, the
# Driver Station the bridge forwards, and target lanes (Target, TargetLane)
# resolved by that Driver Station's mode.
cc_library(
    name = "devices",
    srcs = [
        "devices.cc",
        "target.cc",
    ],
    hdrs = [
        "access.h",
        "devices.h",
        "target.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/bridge:boot_history",
        "//talOS/bridge:messages",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/events:events_core",
        "//talOS/hardware",
        "//talOS/hardware:gate",
        "//talOS/introspection:names",
        "//talOS/node",
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
        ":devices",
        "//talOS/bridge:clock_offset",
        "//talOS/bridge:merge",
        "//talOS/bridge:messages",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/node:testing",
    ],
)

cc_test(
    name = "testing_test",
    size = "small",
    srcs = ["testing_test.cc"],
    deps = [
        ":devices",
        ":testing",
        "//talOS/configuration",
        "//talOS/hardware",
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
        "//talOS/bridge:messages",
        "//talOS/driver_station",
        "//talOS/hardware",
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
        "//talOS/node:testing",
        "@googletest//:gtest_main",
    ],
)
