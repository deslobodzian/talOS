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
        "//common/driver_station:driver_station",
        "//talOS/drivetrain:drivetrain",
        "//talOS/events:events",
        "//talOS/shooter:shooter",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [
        ":driver_station",
        "//talOS/configuration:configuration",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":driver_station",
        "@googletest//:gtest_main",
    ],
)
