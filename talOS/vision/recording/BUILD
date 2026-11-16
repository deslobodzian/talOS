load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

# The on-disk sidecar contract: structs plus a tiny JSON serializer/parser.
cc_library(
    name = "recording",
    srcs = ["recording.cc"],
    hdrs = ["recording.h"],
    deps = [
        "//talOS/vision:calibration",
    ],
)

# The asynchronous frame recorder the camera node feeds.
cc_library(
    name = "recorder",
    srcs = ["recorder.cc"],
    hdrs = ["recorder.h"],
    deps = [
        ":recording",
        "//talOS/process",
        "//talOS/vision:frame",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

# The replay analysis behind frame_replay, shared with tests.
cc_library(
    name = "replay",
    srcs = ["replay.cc"],
    hdrs = ["replay.h"],
    deps = [
        ":recording",
        "//talOS/vision:localization",
        "//talOS/vision:pipeline",
        "//talOS/vision/field_sim",
        "@opencv//:core",
        "@opencv//:imgcodecs",
    ],
)

# Offline replay: re-runs detection + localization over recorded frames.
cc_binary(
    name = "frame_replay",
    srcs = ["replay_main.cc"],
    deps = [
        ":replay",
        "//talOS/vision/field_sim",
    ],
)

cc_test(
    name = "recorder_test",
    size = "small",
    srcs = ["recorder_test.cc"],
    deps = [
        ":recorder",
        ":recording",
        "//talOS/vision:frame",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

# Replay re-runs recorded frames and reproduces the recorded outcomes. Uses
# the field-sim renderer to produce a real tag image without any robot code.
cc_test(
    name = "replay_test",
    size = "medium",
    srcs = ["replay_test.cc"],
    deps = [
        ":recorder",
        ":recording",
        ":replay",
        "//talOS/vision:localization",
        "//talOS/vision:pipeline",
        "//talOS/vision/field_sim",
        "//talOS/vision/field_sim:field_sim_patterns",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
