load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

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

cc_test(
    name = "wire_test",
    srcs = ["tests/wire_test.cc"],
    deps = [":wire"],
)
