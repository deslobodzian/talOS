load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

# The Driver Station wire codec (the bridge's packet payload).
cc_library(
    name = "driver_station",
    srcs = ["driver_station.cc"],
    hdrs = ["driver_station.h"],
)

cc_test(
    name = "driver_station_test",
    size = "small",
    srcs = ["driver_station_test.cc"],
    deps = [
        ":driver_station",
        "@googletest//:gtest_main",
    ],
)

cc_binary(
    name = "send_joystick",
    srcs = ["send_joystick.cc"],
    deps = [
        ":driver_station",
        "//talOS/events:events_os",
        "//talOS/hardware",
        "//talOS/ipc",
        "//talOS/process",
    ],
)
