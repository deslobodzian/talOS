load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

package(default_visibility = ["//visibility:public"])

exports_files(["telemetry.fbs"])

# The Studio feed contract: the telemetry frame FlatBuffers schema plus the
# slot-geometry wire header. Its only dependency is flatbuffers, so robot
# nodes produce telemetry and Studio consumes it without either depending on
# the other.
flatbuffer_cc_library(
    name = "schema",
    srcs = ["telemetry.fbs"],
)

# The slot geometry is a contract between the bridge and whatever produces
# telemetry, so producers outside the bridge package need it too.
cc_library(
    name = "wire",
    hdrs = ["wire.h"],
)

# Page encoding: the producer, the Studio bridge and the mock build every
# TLM3 page through it.
cc_library(
    name = "frames",
    hdrs = ["frames.h"],
    deps = [
        ":schema",
        ":wire",
        "@flatbuffers//:runtime_cc",
    ],
)

cc_test(
    name = "frames_test",
    size = "small",
    srcs = ["tests/frames_test.cc"],
    deps = [
        ":frames",
        ":schema",
        ":wire",
        "@flatbuffers//:runtime_cc",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "wire_test",
    srcs = ["tests/wire_test.cc"],
    deps = [":wire"],
)

cc_library(
    name = "telemetry",
    hdrs = ["node.h"],
    deps = [
        ":frames",
        ":schema",
        ":wire",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/node",
        "//talOS/schema:identity",
        "//talOS/schema:unit_suffix",
        "@flatbuffers//:runtime_cc",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["node.cc"],
    node_name = "telemetry",
    deps = [":telemetry"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":frames",
        ":telemetry",
        ":wire",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/msgs:geometry",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "//talOS/rtms",
        "//talOS/schema:identity",
        "//talOS/schema:test_messages",
        "@googletest//:gtest_main",
    ],
)
