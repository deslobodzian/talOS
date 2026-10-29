load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

# Robot hardware for talos::Node: Motor, CANcoder, Gyro, digital/analog I/O,
# encoders and PWM through the bridge, attached as a Node extension, and the
# Driver Station the bridge forwards.
cc_library(
    name = "devices",
    srcs = ["devices.cc"],
    hdrs = [
        "access.h",
        "devices.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/hardware:gate",
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
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/node:testing",
    ],
)

cc_test(
    name = "devices_test",
    size = "small",
    srcs = ["devices_test.cc"],
    deps = [
        ":devices",
        ":testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "@googletest//:gtest_main",
    ],
)
