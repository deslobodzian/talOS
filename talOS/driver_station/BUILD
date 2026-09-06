load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

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
