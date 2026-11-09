load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "configuration",
    hdrs = ["config_parser.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/driver_station",
        "//talOS/hardware:config",
        "//talOS/introspection:names",
        "@tomlplusplus//:toml++",
    ],
)

cc_binary(
    name = "resolve",
    srcs = ["resolve.cc"],
    visibility = ["//visibility:public"],
    deps = [":configuration"],
)

# Config trees with the expected parse decision, nodes and read files. The
# parser (:test) and //tools:sync_manifest_test both run every case.
filegroup(
    name = "resolution_cases",
    srcs = glob(["testdata/**"]),
    visibility = ["//tools:__pkg__"],
)

cc_test(
    name = "test",
    size = "small",
    srcs = ["config_test.cc"],
    data = [":resolution_cases"],
    deps = [
        ":configuration",
        "//talOS/hardware:config",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "config_fuzz_test",
    size = "small",
    srcs = ["config_fuzz_test.cc"],
    deps = [
        ":configuration",
        "//talOS/protocol:fuzz_corpus",
        "@googletest//:gtest_main",
    ],
)
