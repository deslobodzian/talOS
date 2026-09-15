load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

exports_files(["telemetry.fbs"])

# Standalone Studio schema contract: the telemetry frame FlatBuffers schema
# plus the slot-geometry wire header. Its only dependency is flatbuffers, so
# robot nodes can consume the Studio feed contract without pulling the Studio
# frontend (Node/pnpm), bridge (uWS), or desktop (Rust) toolchains, which stay
# dev-only in the root talos module.
flatbuffer_cc_library(
    name = "telemetry_cc",
    srcs = ["telemetry.fbs"],
)

# Re-exported under the canonical studio/schema/ prefix, so consumers keep
# the same include paths as the in-tree //studio/schema package.
cc_library(
    name = "schema",
    hdrs = [":telemetry_cc_srcs"],
    strip_include_prefix = ".",
    include_prefix = "studio/schema",
    deps = [":telemetry_cc"],
)

# The slot geometry is a contract between the bridge and whatever produces
# telemetry, so producers outside the bridge package need it too.
cc_library(
    name = "wire",
    hdrs = ["wire.h"],
    strip_include_prefix = ".",
    include_prefix = "studio/schema",
)

cc_test(
    name = "wire_test",
    srcs = ["tests/wire_test.cc"],
    deps = [":wire"],
)
