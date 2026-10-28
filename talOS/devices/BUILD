load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

# Robot hardware for talos::Node: Motor, CANcoder, Gyro, digital/analog I/O,
# encoders and PWM through the bridge, attached as a Node extension.
cc_library(
    name = "devices",
    srcs = ["devices.cc"],
    hdrs = [
        "access.h",
        "devices.h",
    ],
    deps = [
        "//talOS/configuration:configuration",
        "//talOS/hardware:gate",
        "//talOS/hardware:hardware",
        "//talOS/node",
    ],
    visibility = ["//visibility:public"],
)

# talos::testing::SimHardware: plays the bridge for a testing::Sim.
cc_library(
    name = "testing",
    testonly = True,
    srcs = ["testing.cc"],
    hdrs = ["testing.h"],
    deps = [
        ":devices",
        "//talOS/configuration:configuration",
        "//talOS/hardware:hardware",
        "//talOS/node:testing",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "devices_test",
    size = "small",
    srcs = ["devices_test.cc"],
    deps = [
        ":devices",
        ":testing",
        "//talOS/hardware:hardware",
        "@googletest//:gtest_main",
    ],
)
