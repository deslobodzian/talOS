load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "driver_station_message",
    srcs = ["driver_station_message.fbs"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "driver_station",
    hdrs = ["node.h"],
    deps = [
        ":driver_station_message",
        "//talOS/driver_station:driver_station",
        "//2026-robot/main_processor/drivetrain:drivetrain",
        "//talOS/events:events",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [
        ":driver_station",
        "//talOS/configuration:configuration",
        "//talOS/process:process",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "send_joystick",
    srcs = ["send_joystick.cc"],
    deps = [
        ":driver_station",
        "//talOS/ipc:ipc",
        "//talOS/process:process",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = ["//2026-robot/main_processor/configuration:robot.toml"],
    deps = [
        "//talOS/configuration:configuration",
        ":driver_station",
        "@googletest//:gtest_main",
    ],
)
