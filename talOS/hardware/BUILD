load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "config",
    srcs = ["config.cc"],
    hdrs = ["config.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hardware",
    srcs = ["messages.cc", "gateway.cc"],
    hdrs = ["config.h", "config_wire.h", "messages.h", "gateway.h", "endpoint.h", "sim_backend.h", "packet.h"],
    deps = [
        ":config",
        "//talOS/protocol:protocol",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "gateway_test",
    size = "small",
    srcs = ["gateway_test.cc"],
    deps = [":hardware", "@googletest//:gtest_main"],
)
