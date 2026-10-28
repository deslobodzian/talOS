load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "telemetry",
    hdrs = ["node.h"],
    strip_include_prefix = ".",
    include_prefix = "telemetry",
    deps = [
        "//2026-robot/main_processor/drivetrain:messages",
        "//2026-robot/main_processor/intake:messages",
        "//2026-robot/main_processor/odometry:messages",
        "//2026-robot/main_processor/shooter:messages",
        "//studio/schema",
        "//studio/schema:wire",
        "//talOS/driver_station:messages",
        "//talOS/driver_station:driver_station",
        "//talOS/events:events",
        "//talOS/rtms:rtms",
        "@flatbuffers//:flatbuffers",
    ],
    visibility = ["//visibility:public"],
)

# The one node on talos::process::RunNode rather than talos_cc_node: its
# Studio feed bypasses the loop, so main.cc declares it as an external source
# in --describe and the live Reporter (see node.h).
cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [
        ":telemetry",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/introspection:reporter",
        "//talOS/process:node_main",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":telemetry",
        "//talOS/memory:shared_memory",
        "@googletest//:gtest_main",
    ],
)
