load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

flatbuffer_cc_library(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
)

_TRANSPORT_VISIBILITY = [
    "//studio:__subpackages__",
    "//talOS:__subpackages__",
    "//tools:__pkg__",
]

# Whether a process recorded in shared memory (pid, start time and pid
# namespace) still runs: RTMS claims and the live registry's rows share this
# one check.
cc_library(
    name = "process_liveness",
    srcs = ["process_liveness.cc"],
    hdrs = ["process_liveness.h"],
    visibility = _TRANSPORT_VISIBILITY,
)

# The cross-process wake channel a publisher uses to wake a reader asleep on
# its topic (README.md "Wake channel").
cc_library(
    name = "wake",
    srcs = ["wake.cc"],
    hdrs = ["wake.h"],
    visibility = _TRANSPORT_VISIBILITY,
)

cc_library(
    name = "rtms",
    srcs = ["rtms.cc"],
    hdrs = ["rtms.h"],
    visibility = _TRANSPORT_VISIBILITY,
    deps = [
        ":process_liveness",
        ":wake",
        "//talOS/memory:shared_memory",
        "//talOS/schema:identity",
    ],
)

cc_test(
    name = "process_liveness_test",
    size = "small",
    srcs = ["process_liveness_test.cc"],
    deps = [
        ":process_liveness",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "wake_test",
    size = "small",
    srcs = ["wake_test.cc"],
    deps = [
        ":wake",
        "@googletest//:gtest_main",
    ],
)

# The schema variants are //talOS/schema's same-size type change fixtures.
_VARIANTS = [
    "float_level",
    "int_level",
]

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
        ":process_liveness",
        ":rtms",
        ":test_message_fbs",
        ":wake",
        "//talOS/schema:schema_hash",
        "@flatbuffers",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

cc_library(
    name = "ring_test_payload",
    testonly = True,
    hdrs = ["ring_test_payload.h"],
)

# Writer and reader threads on one mapping, so the thread sanitizer sees the
# ring's slot accesses as accesses to the same bytes (docs/TESTING.md §2).
cc_test(
    name = "ring_handoff_test",
    size = "small",
    srcs = ["ring_handoff_test.cc"],
    deps = [
        ":ring_test_payload",
        ":rtms",
        "//talOS/schema:identity",
        "@googletest//:gtest_main",
    ],
)

# Deterministic reader-registration boundary cases (<1 ms; fast lane).
cc_test(
    name = "ring_registration_test",
    size = "small",
    srcs = ["ring_registration_test.cc"],
    deps = [
        ":ring_test_payload",
        ":rtms",
        "//talOS/schema:identity",
        "@googletest//:gtest_main",
    ],
)

# Over 5 s on 4-vCPU Linux CI (5.1 s): runs three 4-thread busy-spin
# contention loops totaling 304k atomic-word slot writes; heavy lane.
cc_test(
    name = "ring_lap_test",
    size = "medium",
    srcs = ["ring_lap_test.cc"],
    tags = ["heavy"],
    deps = [
        ":ring_test_payload",
        ":rtms",
        "//talOS/schema:identity",
        "@googletest//:gtest_main",
    ],
)

# What a publication costs its writer with 0, 1, 8 and 32 reader processes
# asleep on the topic (README.md "Publish cost"). Run by hand:
# bazel run -c opt //talOS/rtms:wake_fanout_bench
cc_binary(
    name = "wake_fanout_bench",
    srcs = ["wake_fanout_bench.cc"],
    deps = [
        ":rtms",
        ":wake",
        "//talOS/schema:identity",
    ],
)
