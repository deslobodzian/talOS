load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

flatbuffer_cc_library(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
)

cc_library(
    name = "rtms",
    srcs = ["rtms.cc"],
    hdrs = ["rtms.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/memory:shared_memory",
        "//talOS/schema:identity",
    ],
)

# The schema variants are //talOS/schema's same-size type change fixtures.
_VARIANTS = ["float_level", "int_level"]

cc_test(
    name = "rtms_test",
    size = "small",
    srcs = ["test.cc"],
    data = ["//talOS/schema:variants/%s.bfbs" % v for v in _VARIANTS],
    env = {
        "TALOS_VARIANT_" + v.upper(): "$(rlocationpath //talOS/schema:variants/%s.bfbs)" % v
        for v in _VARIANTS
    },
    deps = [
        ":rtms",
        ":test_message_fbs",
        "//talOS/schema:schema_hash",
        "@flatbuffers",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
